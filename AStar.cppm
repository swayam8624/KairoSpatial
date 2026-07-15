module;

#include <algorithm>
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

        std::priority_queue<OpenNode> open;
        std::unordered_map<NavigationNodeID, float> gScore;
        std::unordered_map<NavigationNodeID, NavigationNodeID> cameFrom;

        gScore[start] = 0.0f;
        open.push({ start, HeuristicEuclidean(graph, start, goal) });

        while (!open.empty())
        {
            const NavigationNodeID current = open.top().ID;
            open.pop();
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

                if (!gScore.contains(edge.To) ||
                    tentative < gScore[edge.To])
                {
                    cameFrom[edge.To] = current;
                    gScore[edge.To] = tentative;
                    open.push(
                        {
                            edge.To,
                            tentative + HeuristicEuclidean(graph, edge.To, goal)
                        });
                }
            }
        }

        return result;
    }
}
