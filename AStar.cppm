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
        const auto heuristic = [&](NavigationNodeID id) -> float
        {
            if (!(heuristicScale > 0.0f)) return 0.0f;
            const float distance = HeuristicEuclidean(graph, id, goal);
            if (!std::isfinite(distance)) return 0.0f;
            const float h = heuristicScale * distance;
            return std::isfinite(h) ? h : 0.0f;
        };

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
            if (next.Cost != gScore.at(next.ID)) continue;
            const NavigationNodeID current = next.ID;
            ++result.ExpandedNodes;

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
                            tentative + heuristic(edge.To),
                            tentative
                        });
                }
            }
        }

        return result;
    }
}
