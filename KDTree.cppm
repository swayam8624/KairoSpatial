module;

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <queue>
#include <vector>

export module Kairo.Foundation.Spatial.KDTree;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    struct KDPoint final
    {
        SpatialID ID = SpatialInvalidID;
        Vec3f Position = Vec3f::Zero();
    };

    struct KDNearestResult final
    {
        SpatialID ID = SpatialInvalidID;
        Vec3f Position = Vec3f::Zero();
        float DistanceSquared = std::numeric_limits<float>::infinity();
    };

    class KDTree final
    {
    public:
        KDTree() = default;

        explicit KDTree(
            const std::vector<KDPoint>& points)
        {
            Build(points);
        }

        void Build(
            const std::vector<KDPoint>& points)
        {
            m_Points = points;
            m_Nodes.clear();

            std::vector<SpatialIndex> indices;
            indices.reserve(points.size());

            for (std::size_t i = 0; i < points.size(); ++i)
            {
                indices.push_back(static_cast<SpatialIndex>(i));
            }

            m_Root = BuildRecursive(indices, 0, indices.size(), 0);
        }

        [[nodiscard]]
        std::optional<KDNearestResult> NearestPoint(
            const Vec3f& query) const
        {
            if (m_Root == SpatialInvalidIndex)
            {
                return std::nullopt;
            }

            KDNearestResult best;
            SearchNearest(m_Root, query, best);
            return best;
        }

        [[nodiscard]]
        std::vector<KDNearestResult> RadiusSearch(
            const Vec3f& query,
            float radius) const
        {
            if (radius < 0.0f)
            {
                return {};
            }

            std::vector<KDNearestResult> result;
            SearchRadius(m_Root, query, radius * radius, result);

            std::stable_sort(
                result.begin(),
                result.end(),
                [](const KDNearestResult& lhs, const KDNearestResult& rhs)
                {
                    return lhs.DistanceSquared < rhs.DistanceSquared;
                });

            return result;
        }

        [[nodiscard]]
        std::vector<KDNearestResult> KNearest(
            const Vec3f& query,
            std::size_t k) const
        {
            if (k == 0 || m_Root == SpatialInvalidIndex)
            {
                return {};
            }

            std::priority_queue<HeapEntry> heap;
            SearchKNearest(m_Root, query, k, heap);

            std::vector<KDNearestResult> result;
            result.reserve(heap.size());

            while (!heap.empty())
            {
                result.push_back(heap.top().Result);
                heap.pop();
            }

            std::reverse(result.begin(), result.end());
            return result;
        }

        [[nodiscard]]
        const std::vector<KDPoint>& Points() const noexcept
        {
            return m_Points;
        }

    private:
        struct Node final
        {
            SpatialIndex PointIndex = SpatialInvalidIndex;
            SpatialIndex Left = SpatialInvalidIndex;
            SpatialIndex Right = SpatialInvalidIndex;
            std::uint32_t Axis = 0;
        };

        struct HeapEntry final
        {
            KDNearestResult Result;

            [[nodiscard]]
            bool operator<(
                const HeapEntry& other) const noexcept
            {
                return Result.DistanceSquared < other.Result.DistanceSquared;
            }
        };

        std::vector<KDPoint> m_Points;
        std::vector<Node> m_Nodes;
        SpatialIndex m_Root = SpatialInvalidIndex;

        [[nodiscard]]
        SpatialIndex BuildRecursive(
            std::vector<SpatialIndex>& indices,
            std::size_t begin,
            std::size_t end,
            std::uint32_t depth)
        {
            if (begin >= end)
            {
                return SpatialInvalidIndex;
            }

            const std::uint32_t axis = depth % 3;
            const std::size_t mid = begin + ((end - begin) / 2);

            std::nth_element(
                indices.begin() + static_cast<std::ptrdiff_t>(begin),
                indices.begin() + static_cast<std::ptrdiff_t>(mid),
                indices.begin() + static_cast<std::ptrdiff_t>(end),
                [&](SpatialIndex lhs, SpatialIndex rhs)
                {
                    return AxisValue(m_Points[lhs].Position, axis) <
                        AxisValue(m_Points[rhs].Position, axis);
                });

            const SpatialIndex nodeIndex =
                static_cast<SpatialIndex>(m_Nodes.size());

            m_Nodes.push_back(Node{ indices[mid], SpatialInvalidIndex, SpatialInvalidIndex, axis });
            m_Nodes[nodeIndex].Left = BuildRecursive(indices, begin, mid, depth + 1);
            m_Nodes[nodeIndex].Right = BuildRecursive(indices, mid + 1, end, depth + 1);
            return nodeIndex;
        }

        void SearchNearest(
            SpatialIndex nodeIndex,
            const Vec3f& query,
            KDNearestResult& best) const
        {
            if (nodeIndex == SpatialInvalidIndex)
            {
                return;
            }

            const Node& node = m_Nodes[nodeIndex];
            const KDPoint& point = m_Points[node.PointIndex];
            const float distanceSquared =
                DistanceSquared(query, point.Position);

            if (distanceSquared < best.DistanceSquared)
            {
                best = { point.ID, point.Position, distanceSquared };
            }

            const float delta =
                AxisValue(query, node.Axis) -
                AxisValue(point.Position, node.Axis);

            const SpatialIndex nearChild =
                delta <= 0.0f ? node.Left : node.Right;

            const SpatialIndex farChild =
                delta <= 0.0f ? node.Right : node.Left;

            SearchNearest(nearChild, query, best);

            if (delta * delta <= best.DistanceSquared)
            {
                SearchNearest(farChild, query, best);
            }
        }

        void SearchRadius(
            SpatialIndex nodeIndex,
            const Vec3f& query,
            float radiusSquared,
            std::vector<KDNearestResult>& result) const
        {
            if (nodeIndex == SpatialInvalidIndex)
            {
                return;
            }

            const Node& node = m_Nodes[nodeIndex];
            const KDPoint& point = m_Points[node.PointIndex];
            const float distanceSquared =
                DistanceSquared(query, point.Position);

            if (distanceSquared <= radiusSquared)
            {
                result.push_back({ point.ID, point.Position, distanceSquared });
            }

            const float delta =
                AxisValue(query, node.Axis) -
                AxisValue(point.Position, node.Axis);

            if (delta <= 0.0f)
            {
                SearchRadius(node.Left, query, radiusSquared, result);
                if (delta * delta <= radiusSquared)
                {
                    SearchRadius(node.Right, query, radiusSquared, result);
                }
            }
            else
            {
                SearchRadius(node.Right, query, radiusSquared, result);
                if (delta * delta <= radiusSquared)
                {
                    SearchRadius(node.Left, query, radiusSquared, result);
                }
            }
        }

        void SearchKNearest(
            SpatialIndex nodeIndex,
            const Vec3f& query,
            std::size_t k,
            std::priority_queue<HeapEntry>& heap) const
        {
            if (nodeIndex == SpatialInvalidIndex)
            {
                return;
            }

            const Node& node = m_Nodes[nodeIndex];
            const KDPoint& point = m_Points[node.PointIndex];
            const float distanceSquared =
                DistanceSquared(query, point.Position);

            if (heap.size() < k)
            {
                heap.push({ KDNearestResult{ point.ID, point.Position, distanceSquared } });
            }
            else if (distanceSquared < heap.top().Result.DistanceSquared)
            {
                heap.pop();
                heap.push({ KDNearestResult{ point.ID, point.Position, distanceSquared } });
            }

            const float delta =
                AxisValue(query, node.Axis) -
                AxisValue(point.Position, node.Axis);

            const SpatialIndex nearChild =
                delta <= 0.0f ? node.Left : node.Right;

            const SpatialIndex farChild =
                delta <= 0.0f ? node.Right : node.Left;

            SearchKNearest(nearChild, query, k, heap);

            const float worstDistanceSquared =
                heap.size() < k
                    ? std::numeric_limits<float>::infinity()
                    : heap.top().Result.DistanceSquared;

            if (delta * delta <= worstDistanceSquared)
            {
                SearchKNearest(farChild, query, k, heap);
            }
        }
    };
}
