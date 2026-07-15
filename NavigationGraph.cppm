module;

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <unordered_map>
#include <vector>

export module Kairo.Foundation.Spatial.NavigationGraph;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    using NavigationNodeID = std::uint32_t;
    inline constexpr NavigationNodeID InvalidNavigationNodeID =
        std::numeric_limits<NavigationNodeID>::max();

    struct GraphNode final
    {
        NavigationNodeID ID = InvalidNavigationNodeID;
        Vec3f Position = Vec3f::Zero();
    };

    struct GraphEdge final
    {
        NavigationNodeID From = InvalidNavigationNodeID;
        NavigationNodeID To = InvalidNavigationNodeID;
        float Cost = 1.0f;
    };

    class NavigationGraph final
    {
    public:
        void AddNode(
            NavigationNodeID id,
            const Vec3f& position)
        {
            if (m_NodeLookup.contains(id))
            {
                throw std::invalid_argument("NavigationGraph::AddNode failed: duplicate node id.");
            }

            m_NodeLookup[id] = m_Nodes.size();
            m_Nodes.push_back({ id, position });
            m_Adjacency[id] = {};
        }

        void AddEdge(
            NavigationNodeID from,
            NavigationNodeID to,
            float cost,
            bool bidirectional = true)
        {
            RequireNode(from);
            RequireNode(to);

            if (cost < 0.0f)
            {
                throw std::invalid_argument("NavigationGraph::AddEdge failed: cost must be non-negative.");
            }

            m_Adjacency[from].push_back({ from, to, cost });

            if (bidirectional)
            {
                m_Adjacency[to].push_back({ to, from, cost });
            }
        }

        bool RemoveNode(
            NavigationNodeID id)
        {
            auto found = m_NodeLookup.find(id);
            if (found == m_NodeLookup.end())
            {
                return false;
            }

            m_Nodes.erase(m_Nodes.begin() + static_cast<std::ptrdiff_t>(found->second));
            RebuildLookup();
            m_Adjacency.erase(id);

            for (auto& [nodeID, edges] : m_Adjacency)
            {
                edges.erase(
                    std::remove_if(
                        edges.begin(),
                        edges.end(),
                        [id](const GraphEdge& edge)
                        {
                            return edge.To == id;
                        }),
                    edges.end());
            }

            return true;
        }

        [[nodiscard]]
        const GraphNode& Node(
            NavigationNodeID id) const
        {
            RequireNode(id);
            return m_Nodes[m_NodeLookup.at(id)];
        }

        [[nodiscard]]
        const std::vector<GraphEdge>& EdgesFrom(
            NavigationNodeID id) const
        {
            RequireNode(id);
            return m_Adjacency.at(id);
        }

        [[nodiscard]]
        bool Contains(
            NavigationNodeID id) const noexcept
        {
            return m_NodeLookup.contains(id);
        }

        [[nodiscard]]
        const std::vector<GraphNode>& Nodes() const noexcept
        {
            return m_Nodes;
        }

    private:
        std::vector<GraphNode> m_Nodes;
        std::unordered_map<NavigationNodeID, std::size_t> m_NodeLookup;
        std::unordered_map<NavigationNodeID, std::vector<GraphEdge>> m_Adjacency;

        void RequireNode(
            NavigationNodeID id) const
        {
            if (!m_NodeLookup.contains(id))
            {
                throw std::out_of_range("NavigationGraph node id does not exist.");
            }
        }

        void RebuildLookup()
        {
            m_NodeLookup.clear();

            for (std::size_t i = 0; i < m_Nodes.size(); ++i)
            {
                m_NodeLookup[m_Nodes[i].ID] = i;
            }
        }
    };

    [[nodiscard]]
    inline float HeuristicEuclidean(
        const NavigationGraph& graph,
        NavigationNodeID from,
        NavigationNodeID to)
    {
        return Distance(graph.Node(from).Position, graph.Node(to).Position);
    }
}
