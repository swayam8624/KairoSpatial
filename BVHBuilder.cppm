module;

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

export module Kairo.Foundation.Spatial.BVHBuilder;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Spatial.Types;
import Kairo.Foundation.Spatial.BVH;

export namespace kairo::foundation::spatial
{
    namespace bvh_builder_detail
    {
        struct Split final
        {
            std::size_t Mid = 0;
            std::uint32_t Axis = 0;
            float Cost = std::numeric_limits<float>::infinity();
            bool Valid = false;
        };

        [[nodiscard]]
        BVHPrimitive ToBVHPrimitive(
            const SpatialPrimitive& primitive,
            SpatialIndex index)
        {
            return
            {
                primitive.Bounds,
                primitive.Bounds.Center(),
                primitive.ID,
                index,
                primitive.LayerMask
            };
        }

        [[nodiscard]]
        SpatialAABB PrimitiveBounds(
            const std::vector<BVHPrimitive>& primitives,
            const std::vector<SpatialIndex>& order,
            std::size_t begin,
            std::size_t end)
        {
            SpatialAABB result = SpatialAABB::Empty();

            for (std::size_t i = begin; i < end; ++i)
            {
                result.ExpandToInclude(primitives[order[i]].Bounds);
            }

            return result;
        }

        [[nodiscard]]
        SpatialAABB CentroidBounds(
            const std::vector<BVHPrimitive>& primitives,
            const std::vector<SpatialIndex>& order,
            std::size_t begin,
            std::size_t end)
        {
            SpatialAABB result = SpatialAABB::Empty();

            for (std::size_t i = begin; i < end; ++i)
            {
                result.ExpandToInclude(primitives[order[i]].Centroid);
            }

            return result;
        }

        void SortRangeByAxis(
            const std::vector<BVHPrimitive>& primitives,
            std::vector<SpatialIndex>& order,
            std::size_t begin,
            std::size_t end,
            std::uint32_t axis)
        {
            std::stable_sort(
                order.begin() + static_cast<std::ptrdiff_t>(begin),
                order.begin() + static_cast<std::ptrdiff_t>(end),
                [&](SpatialIndex lhs, SpatialIndex rhs)
                {
                    return AxisValue(primitives[lhs].Centroid, axis) <
                        AxisValue(primitives[rhs].Centroid, axis);
                });
        }

        [[nodiscard]]
        std::uint32_t ExpandBits(
            std::uint32_t value) noexcept
        {
            value = (value * 0x00010001u) & 0xFF0000FFu;
            value = (value * 0x00000101u) & 0x0F00F00Fu;
            value = (value * 0x00000011u) & 0xC30C30C3u;
            value = (value * 0x00000005u) & 0x49249249u;
            return value;
        }

        [[nodiscard]]
        std::uint32_t Morton3D(
            const Vec3f& normalized) noexcept
        {
            const std::uint32_t x =
                ExpandBits(static_cast<std::uint32_t>(std::clamp(normalized.x, 0.0f, 0.999999f) * 1024.0f));

            const std::uint32_t y =
                ExpandBits(static_cast<std::uint32_t>(std::clamp(normalized.y, 0.0f, 0.999999f) * 1024.0f));

            const std::uint32_t z =
                ExpandBits(static_cast<std::uint32_t>(std::clamp(normalized.z, 0.0f, 0.999999f) * 1024.0f));

            return x * 4u + y * 2u + z;
        }

        void SortByMorton(
            const std::vector<BVHPrimitive>& primitives,
            std::vector<SpatialIndex>& order)
        {
            if (order.empty())
            {
                return;
            }

            SpatialAABB centroidBounds = SpatialAABB::Empty();
            for (SpatialIndex index : order)
            {
                centroidBounds.ExpandToInclude(primitives[index].Centroid);
            }

            const Vec3f size = centroidBounds.Size();
            const Vec3f safeSize
            {
                size.x == 0.0f ? 1.0f : size.x,
                size.y == 0.0f ? 1.0f : size.y,
                size.z == 0.0f ? 1.0f : size.z
            };

            std::stable_sort(
                order.begin(),
                order.end(),
                [&](SpatialIndex lhs, SpatialIndex rhs)
                {
                    const Vec3f lhsNormalized =
                    {
                        (primitives[lhs].Centroid.x - centroidBounds.Min.x) / safeSize.x,
                        (primitives[lhs].Centroid.y - centroidBounds.Min.y) / safeSize.y,
                        (primitives[lhs].Centroid.z - centroidBounds.Min.z) / safeSize.z
                    };

                    const Vec3f rhsNormalized =
                    {
                        (primitives[rhs].Centroid.x - centroidBounds.Min.x) / safeSize.x,
                        (primitives[rhs].Centroid.y - centroidBounds.Min.y) / safeSize.y,
                        (primitives[rhs].Centroid.z - centroidBounds.Min.z) / safeSize.z
                    };

                    return Morton3D(lhsNormalized) < Morton3D(rhsNormalized);
                });
        }

        [[nodiscard]]
        Split MedianSplit(
            const std::vector<BVHPrimitive>& primitives,
            std::vector<SpatialIndex>& order,
            std::size_t begin,
            std::size_t end)
        {
            const SpatialAABB centroidBounds =
                CentroidBounds(primitives, order, begin, end);

            const std::uint32_t axis =
                LargestAxis(centroidBounds.Size());

            SortRangeByAxis(primitives, order, begin, end, axis);
            return Split{ begin + ((end - begin) / 2), axis, 0.0f, end - begin > 1 };
        }

        [[nodiscard]]
        Split SAHSplit(
            const std::vector<BVHPrimitive>& primitives,
            std::vector<SpatialIndex>& order,
            std::size_t begin,
            std::size_t end,
            const BVHBuildSettings& settings)
        {
            const std::size_t count = end - begin;
            if (count <= 2)
            {
                return MedianSplit(primitives, order, begin, end);
            }

            Split best;
            const SpatialAABB fullBounds =
                PrimitiveBounds(primitives, order, begin, end);

            const float leafCost =
                static_cast<float>(count) * fullBounds.SurfaceArea();

            for (std::uint32_t axis = 0; axis < 3; ++axis)
            {
                SortRangeByAxis(primitives, order, begin, end, axis);

                std::vector<SpatialAABB> left(count);
                std::vector<SpatialAABB> right(count);

                SpatialAABB running = SpatialAABB::Empty();
                for (std::size_t i = 0; i < count; ++i)
                {
                    running.ExpandToInclude(primitives[order[begin + i]].Bounds);
                    left[i] = running;
                }

                running = SpatialAABB::Empty();
                for (std::size_t i = count; i > 0; --i)
                {
                    running.ExpandToInclude(primitives[order[begin + i - 1]].Bounds);
                    right[i - 1] = running;
                }

                for (std::size_t leftCount = 1; leftCount < count; ++leftCount)
                {
                    const std::size_t rightCount = count - leftCount;
                    const float cost =
                        left[leftCount - 1].SurfaceArea() * static_cast<float>(leftCount) +
                        right[leftCount].SurfaceArea() * static_cast<float>(rightCount);

                    if (cost < best.Cost)
                    {
                        best =
                        {
                            begin + leftCount,
                            axis,
                            cost,
                            true
                        };
                    }
                }
            }

            if (!best.Valid || best.Cost >= leafCost)
            {
                return MedianSplit(primitives, order, begin, end);
            }

            SortRangeByAxis(primitives, order, begin, end, best.Axis);
            return best;
        }

        SpatialIndex BuildRecursive(
            BVH& bvh,
            std::size_t begin,
            std::size_t end,
            std::uint32_t depth,
            SpatialIndex parent)
        {
            const SpatialIndex nodeIndex =
                static_cast<SpatialIndex>(bvh.Nodes.size());

            bvh.Nodes.push_back(BVHNode{});
            BVHNode& node = bvh.Nodes.back();
            node.Parent = parent;
            node.Bounds = PrimitiveBounds(bvh.Primitives, bvh.PrimitiveOrder, begin, end);

            bvh.Stats.MaxDepth =
                std::max(bvh.Stats.MaxDepth, depth);

            const std::size_t count = end - begin;
            if (count <= bvh.Settings.MaxLeafSize ||
                depth >= bvh.Settings.MaxDepth)
            {
                node.FirstPrimitive = static_cast<SpatialIndex>(begin);
                node.PrimitiveCount = static_cast<std::uint32_t>(count);
                ++bvh.Stats.LeafCount;
                return nodeIndex;
            }

            Split split =
                bvh.Settings.UseLBVH
                    ? Split{ begin + ((end - begin) / 2), 0, 0.0f, end - begin > 1 }
                    : bvh.Settings.UseSAH
                    ? SAHSplit(bvh.Primitives, bvh.PrimitiveOrder, begin, end, bvh.Settings)
                    : MedianSplit(bvh.Primitives, bvh.PrimitiveOrder, begin, end);

            if (!split.Valid || split.Mid <= begin || split.Mid >= end)
            {
                node.FirstPrimitive = static_cast<SpatialIndex>(begin);
                node.PrimitiveCount = static_cast<std::uint32_t>(count);
                ++bvh.Stats.LeafCount;
                return nodeIndex;
            }

            const SpatialIndex left =
                BuildRecursive(bvh, begin, split.Mid, depth + 1, nodeIndex);

            const SpatialIndex right =
                BuildRecursive(bvh, split.Mid, end, depth + 1, nodeIndex);

            BVHNode& refreshed = bvh.Nodes[nodeIndex];
            refreshed.Left = left;
            refreshed.Right = right;
            refreshed.PrimitiveCount = 0;
            return nodeIndex;
        }
    }

    [[nodiscard]]
    BVH BuildBVH(
        const std::vector<SpatialPrimitive>& primitives,
        BVHBuildSettings settings = {})
    {
        if (settings.MaxLeafSize == 0)
        {
            throw std::invalid_argument("BuildBVH failed: MaxLeafSize must be non-zero.");
        }

        BVH bvh;
        bvh.Settings = settings;
        bvh.Primitives.reserve(primitives.size());
        bvh.PrimitiveOrder.reserve(primitives.size());

        for (std::size_t i = 0; i < primitives.size(); ++i)
        {
            if (!primitives[i].Bounds.IsValid())
            {
                throw std::invalid_argument("BuildBVH failed: primitive has invalid bounds.");
            }

            bvh.Primitives.push_back(
                bvh_builder_detail::ToBVHPrimitive(
                    primitives[i],
                    static_cast<SpatialIndex>(i)));

            bvh.PrimitiveOrder.push_back(
                static_cast<SpatialIndex>(i));
        }

        const auto start =
            std::chrono::steady_clock::now();

        if (!primitives.empty())
        {
            if (settings.UseLBVH)
            {
                bvh_builder_detail::SortByMorton(
                    bvh.Primitives,
                    bvh.PrimitiveOrder);
            }

            bvh_builder_detail::BuildRecursive(
                bvh,
                0,
                primitives.size(),
                0,
                SpatialInvalidIndex);
        }

        const auto finish =
            std::chrono::steady_clock::now();

        bvh.Stats.NodeCount = static_cast<std::uint32_t>(bvh.Nodes.size());
        bvh.Stats.PrimitiveCount = static_cast<std::uint32_t>(bvh.Primitives.size());
        bvh.Stats.BuildMilliseconds =
            std::chrono::duration<double, std::milli>(finish - start).count();

        return bvh;
    }
}
