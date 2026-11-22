// TEACHING MODULE: FindPathAStar searches NavigationGraph using an admissible scaled Euclidean heuristic derived from cheapest edge cost per distance. Unlike arbitrary straight-line distance, this remains safe with cheap edges/teleports and falls back to Dijkstra when scale is zero. Returns node IDs and cost, not a NavMesh corridor or a moved character.
module;

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

export module Kairo.Foundation.Spatial.AStar;

import Kairo.Foundation.Spatial.NavigationGraph;

export namespace kairo::foundation::spatial
{
    struct AStarResult final
    {
        bool Found = false;
        float Cost = std::numeric_limits<float>::infinity();
        std::vector<NavigationNodeID> Path;
        std::uint32_t ExpandedNodes = 0;
    };

    [[nodiscard]]
    /// TEACHING API: AStarResult FindPathAStar( — Priority queue explores graph nodes by current path cost plus admissible scaled-distance heuristic; returned graph path is not character motion.
    // Search a *directed weighted NavigationGraph*, not NavMesh geometry or
    // rigid-body contacts. AStarResult holds a node path, summed traversal cost
    // and expansions; the caller must separately steer the actor through a
    // dynamic world.
    AStarResult FindPathAStar(
        const NavigationGraph& graph,
        NavigationNodeID start,
        NavigationNodeID goal)
    {
        struct OpenNode final
        {
            NavigationNodeID ID = InvalidNavigationNodeID;
            float Priority = 0.0f;
            float Cost = 0.0f;

            [[nodiscard]]
            bool operator<(
                const OpenNode& other) const noexcept
            {
                return Priority > other.Priority;
            }
        };

        AStarResult result;
        if (!graph.Contains(start) || !graph.Contains(goal))
        {
            return result;
        }

        // Raw Euclidean distance is not necessarily a lower cost bound:
        // graph edges may cost less than their geometric lengths (teleports,
        // roads, different movement units). Establish the smallest per-unit
        // edge cost so h(u)=scale*distance(u, goal) remains admissible AND
        // consistent by the triangle inequality. Zero-cost long edges yield
        // Dijkstra's algorithm automatically; empty/degenerate graphs do too.
        // Compute a lower bound on cost per Euclidean distance across graph edges.
        // h(node)=scale*distance(node,goal) is then admissible when edge costs are
        // nonnegative and path distances obey triangle inequality. Cheap teleports
        // can force scale toward zero.
        float heuristicScale = std::numeric_limits<float>::infinity();
        for (const GraphNode& node : graph.Nodes())
        {
            for (const GraphEdge& edge : graph.EdgesFrom(node.ID))
            {
                const float distance = HeuristicEuclidean(graph, node.ID, edge.To);
                if (!std::isfinite(distance))
                {
                    heuristicScale = 0.0f;
                    break;
                }
                if (distance > 0.0f)
                    heuristicScale = std::min(heuristicScale, edge.Cost / distance);
            }
            if (heuristicScale == 0.0f) break;
        }
        if (!std::isfinite(heuristicScale)) heuristicScale = 0.0f;

        // Coordinates may be finite while their Euclidean distance
        // overflows float. Multiplying an infinite distance by a Dijkstra
        // fallback scale of zero gives NaN, violating priority_queue's
        // ordering assumptions. Fall back to a true zero heuristic instead.
        // When the scaled Euclidean heuristic is nonfinite or zero, return a safe
        // zero fallback (Dijkstra). Multiplying infinity by zero could produce NaN
        // and violate priority_queue ordering.
        const auto heuristic = [&](NavigationNodeID id) -> float
        {
            if (!(heuristicScale > 0.0f)) return 0.0f;
            const float distance = HeuristicEuclidean(graph, id, goal);
            if (!std::isfinite(distance)) return 0.0f;
            const float h = heuristicScale * distance;
            return std::isfinite(h) ? h : 0.0f;
        };

        // Use a min-priority f=g+h heap via reversed comparison, a gScore map for
        // best known path costs and cameFrom for reconstruction. Stale duplicate heap
        // entries are skipped because std::priority_queue has no decrease-key.
        std::priority_queue<OpenNode> open;
        std::unordered_map<NavigationNodeID, float> gScore;
        std::unordered_map<NavigationNodeID, NavigationNodeID> cameFrom;

        gScore[start] = 0.0f;
        open.push({ start, heuristic(start), 0.0f });

        while (!open.empty())
        {
            const OpenNode next = open.top();
            open.pop();
            // Decrease-key is represented by a new entry in std::priority_queue.
            // Ignore its older, more expensive copies rather than re-expanding.
            // This is lazy decrease-key: an improved route pushes another heap entry
            // instead of mutating the old one. Discard outdated higher-cost entries
            // before expanding neighbors.
            if (next.Cost != gScore.at(next.ID)) continue;
            const NavigationNodeID current = next.ID;
            ++result.ExpandedNodes;

            // Once the consistent/admissible heuristic yields goal from the frontier,
            // reconstruct predecessor IDs in reverse to start and then reverse the
            // sequence. No character position is changed by path search.
            if (current == goal)
            {
                result.Found = true;
                result.Cost = gScore[current];

                NavigationNodeID step = goal;
                while (step != start)
                {
                    result.Path.push_back(step);
                    step = cameFrom[step];
                }

                result.Path.push_back(start);
                std::reverse(result.Path.begin(), result.Path.end());
                return result;
            }

            for (const GraphEdge& edge : graph.EdgesFrom(current))
            {
                // Relax a directed edge only when its finite accumulated g cost beats the
                // destination's previous cost. Overflowed float path sums are ignored rather
                // than published as a valid route.
                const float tentative =
                    gScore[current] + edge.Cost;
                // Finite edges may overflow when accumulated. Such a
                // candidate cannot represent a valid finite path cost.
                if (!std::isfinite(tentative)) continue;

                if (!gScore.contains(edge.To) ||
                    tentative < gScore[edge.To])
                {
                    cameFrom[edge.To] = current;
                    gScore[edge.To] = tentative;
                    open.push(
                        {
                            edge.To,
                            // A sum of separately finite heuristic and
                            // path values can still overflow. A finite path
                            // cost is a valid conservative queue priority.
                            std::isfinite(tentative + heuristic(edge.To))
                                ? tentative + heuristic(edge.To)
                                : tentative,
                            tentative
                        });
                }
            }
        }

        return result;
    }
}
