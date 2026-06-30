module;

#include <cstdint>
#include <cstddef>
#include <stdexcept>
#include <string>
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
        SpatialIndex Parent = SpatialInvalidIndex;

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
        bool UseMortonOrdering = false;
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
            return
                Nodes.empty() &&
                Primitives.empty() &&
                PrimitiveOrder.empty();
        }

        [[nodiscard]]
        bool IsValid() const noexcept
        {
            if (Nodes.empty())
            {
                return Primitives.empty() && PrimitiveOrder.empty();
            }

            if (Primitives.empty() ||
                PrimitiveOrder.size() != Primitives.size() ||
                !Nodes[0].Bounds.IsValid())
            {
                return false;
            }

            for (SpatialIndex index : PrimitiveOrder)
            {
                if (index >= Primitives.size())
                {
                    return false;
                }
            }

            for (std::size_t i = 0; i < Nodes.size(); ++i)
            {
                const BVHNode& node = Nodes[i];
                if (!node.Bounds.IsValid())
                {
                    return false;
                }

                if (node.Parent != SpatialInvalidIndex &&
                    node.Parent >= Nodes.size())
                {
                    return false;
                }

                if (node.IsLeaf())
                {
                    if (node.Left != SpatialInvalidIndex ||
                        node.Right != SpatialInvalidIndex)
                    {
                        return false;
                    }

                    const std::size_t end =
                        static_cast<std::size_t>(node.FirstPrimitive) +
                        static_cast<std::size_t>(node.PrimitiveCount);

                    if (end > PrimitiveOrder.size())
                    {
                        return false;
                    }
                }
                else
                {
                    if (node.Left >= Nodes.size() ||
                        node.Right >= Nodes.size() ||
                        node.Left == node.Right ||
                        Nodes[node.Left].Parent != i ||
                        Nodes[node.Right].Parent != i)
                    {
                        return false;
                    }
                }
            }

            return true;
        }

        [[nodiscard]]
        SpatialValidationResult Validate() const
        {
            return IsValid()
                ? SpatialValidationResult{ true, {} }
                : SpatialValidationResult{ false, "BVH node, primitive, or parent/child ranges are inconsistent." };
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
