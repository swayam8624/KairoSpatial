module;

#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <vector>

export module Kairo.Foundation.Spatial.BVH;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    struct BVHPrimitive final
    {
        SpatialAABB Bounds = SpatialAABB::Empty();
        Vec3f Centroid = Vec3f::Zero();
        SpatialID UserID = SpatialInvalidID;
        SpatialIndex UserIndex = SpatialInvalidIndex;
        SpatialLayerMask LayerMask = 1u;
    };

    struct BVHNode final
    {
        SpatialAABB Bounds = SpatialAABB::Empty();
        SpatialIndex Left = SpatialInvalidIndex;
        SpatialIndex Right = SpatialInvalidIndex;
        SpatialIndex FirstPrimitive = 0;
        std::uint32_t PrimitiveCount = 0;
        std::uint32_t Parent = SpatialInvalidIndex;

        [[nodiscard]]
        constexpr bool IsLeaf() const noexcept
        {
            return PrimitiveCount > 0;
        }
    };

    struct BVHBuildSettings final
    {
        std::uint32_t MaxLeafSize = 4;
        std::uint32_t MaxDepth = 64;
        bool UseSAH = false;
        std::uint32_t SAHBins = 12;
        bool UseLBVH = false;
    };

    struct BVHBuildStats final
    {
        std::uint32_t NodeCount = 0;
        std::uint32_t LeafCount = 0;
        std::uint32_t MaxDepth = 0;
        std::uint32_t PrimitiveCount = 0;
        double BuildMilliseconds = 0.0;
    };

    class BVH final
    {
    public:
        std::vector<BVHNode> Nodes;
        std::vector<BVHPrimitive> Primitives;
        std::vector<SpatialIndex> PrimitiveOrder;
        BVHBuildSettings Settings;
        BVHBuildStats Stats;

        [[nodiscard]]
        bool Empty() const noexcept
        {
            return Nodes.empty();
        }

        [[nodiscard]]
        const BVHNode& Root() const
        {
            if (Nodes.empty())
            {
                throw std::runtime_error("BVH::Root failed: tree is empty.");
            }

            return Nodes[0];
        }

        [[nodiscard]]
        const BVHPrimitive& PrimitiveAtOrderedOffset(
            std::size_t orderedOffset) const
        {
            if (orderedOffset >= PrimitiveOrder.size())
            {
                throw std::out_of_range("BVH primitive offset is out of range.");
            }

            return Primitives[PrimitiveOrder[orderedOffset]];
        }
    };
}
