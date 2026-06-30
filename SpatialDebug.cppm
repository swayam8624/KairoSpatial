module;

#include <array>
#include <vector>

export module Kairo.Foundation.Spatial.Debug;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Spatial.Types;
import Kairo.Foundation.Spatial.BVH;
import Kairo.Foundation.Spatial.DynamicAABBTree;
import Kairo.Foundation.Spatial.Octree;

export namespace kairo::foundation::spatial
{
    struct DebugLine final
    {
        Vec3f A = Vec3f::Zero();
        Vec3f B = Vec3f::Zero();
        Vec3f Color = Vec3f{ 1.0f, 1.0f, 1.0f };
    };

    struct DebugBox final
    {
        SpatialAABB Bounds = SpatialAABB::Empty();
        Vec3f Color = Vec3f{ 1.0f, 1.0f, 1.0f };
        SpatialID ID = SpatialInvalidID;
        std::uint32_t Depth = 0;
    };

    struct DebugSphere final
    {
        Vec3f Center = Vec3f::Zero();
        float Radius = 1.0f;
        Vec3f Color = Vec3f{ 1.0f, 1.0f, 1.0f };
        SpatialID ID = SpatialInvalidID;
    };

    struct SpatialDebugDrawList final
    {
        std::vector<DebugLine> Lines;
        std::vector<DebugBox> Boxes;
        std::vector<DebugSphere> Spheres;

        void Clear()
        {
            Lines.clear();
            Boxes.clear();
            Spheres.clear();
        }
    };

    [[nodiscard]]
    std::vector<DebugBox> ExtractBVHBoxes(
        const BVH& bvh)
    {
        std::vector<DebugBox> boxes;
        boxes.reserve(bvh.Nodes.size());

        for (std::size_t i = 0; i < bvh.Nodes.size(); ++i)
        {
            const BVHNode& node = bvh.Nodes[i];
            boxes.push_back(
                DebugBox
                {
                    node.Bounds,
                    node.IsLeaf()
                        ? Vec3f{ 0.2f, 0.9f, 0.3f }
                        : Vec3f{ 0.2f, 0.5f, 1.0f },
                    static_cast<SpatialID>(i),
                    0
                });
        }

        return boxes;
    }

    [[nodiscard]]
    std::vector<DebugBox> ExtractDynamicTreeBounds(
        const DynamicAABBTree& tree)
    {
        std::vector<DebugBox> boxes;

        for (std::size_t i = 0; i < tree.Nodes().size(); ++i)
        {
            const DynamicAABBTreeNode& node = tree.Nodes()[i];
            if (!node.Allocated)
            {
                continue;
            }

            boxes.push_back(
                DebugBox
                {
                    node.FatBounds,
                    node.IsLeaf()
                        ? Vec3f{ 1.0f, 0.8f, 0.2f }
                        : Vec3f{ 0.8f, 0.3f, 1.0f },
                    node.UserID,
                    node.Height
                });
        }

        return boxes;
    }

    [[nodiscard]]
    std::vector<DebugBox> ExtractOctreeCells(
        const Octree& octree)
    {
        std::vector<DebugBox> boxes;
        const std::vector<SpatialAABB> cells =
            octree.ExtractCells();

        boxes.reserve(cells.size());
        for (std::size_t i = 0; i < cells.size(); ++i)
        {
            boxes.push_back(
                DebugBox
                {
                    cells[i],
                    Vec3f{ 0.1f, 0.8f, 0.9f },
                    static_cast<SpatialID>(i),
                    0
                });
        }

        return boxes;
    }

    [[nodiscard]]
    std::vector<DebugLine> ExtractRayTraversalPath(
        const SpatialRay& ray,
        float distance,
        const Vec3f& color = Vec3f{ 1.0f, 0.1f, 0.1f })
    {
        return
        {
            DebugLine
            {
                ray.Origin,
                ray.GetPoint(distance),
                color
            }
        };
    }
}
