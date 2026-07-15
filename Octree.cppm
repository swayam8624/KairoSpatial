module;

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <vector>

export module Kairo.Foundation.Spatial.Octree;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Geometry.Frustum;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    struct OctreeSettings final
    {
        std::uint32_t MaxDepth = 8;
        std::uint32_t MaxItemsPerNode = 8;
        float LooseFactor = 1.5f;
    };

    struct OctreeItem final
    {
        SpatialID ID = SpatialInvalidID;
        SpatialAABB Bounds = SpatialAABB::Empty();
    };

    class Octree final
    {
    public:
        Octree(
            const SpatialAABB& worldBounds,
            OctreeSettings settings = {})
            : m_WorldBounds(worldBounds)
            , m_Settings(settings)
            , m_Root(std::make_unique<Node>(worldBounds, 0))
        {
            if (!worldBounds.IsValid())
            {
                throw std::invalid_argument("Octree failed: world bounds must be valid.");
            }
            if (settings.MaxItemsPerNode == 0 || settings.LooseFactor < 1.0f)
            {
                throw std::invalid_argument("Octree failed: invalid settings.");
            }
        }

        void Insert(
            SpatialID id,
            const SpatialAABB& bounds)
        {
            if (!bounds.IsValid())
            {
                throw std::invalid_argument("Octree::Insert failed: invalid bounds.");
            }

            Remove(id);
            Insert(*m_Root, OctreeItem{ id, bounds });
        }

        bool Remove(
            SpatialID id)
        {
            return Remove(*m_Root, id);
        }

        [[nodiscard]]
        std::vector<SpatialID> QueryAABB(
            const SpatialAABB& query) const
        {
            std::vector<SpatialID> result;
            QueryAABB(*m_Root, query, result);
            return result;
        }

        [[nodiscard]]
        std::vector<SpatialID> QuerySphere(
            const Vec3f& center,
            float radius) const
        {
            std::vector<SpatialID> result;
            QuerySphere(*m_Root, center, radius, result);
            return result;
        }

        [[nodiscard]]
        std::vector<SpatialID> QueryFrustum(
            const Frustumf& frustum) const
        {
            std::vector<SpatialID> result;
            QueryFrustum(*m_Root, frustum, result);
            return result;
        }

        [[nodiscard]]
        SpatialRaycastResult Raycast(
            const SpatialRay& ray,
            float maxDistance = std::numeric_limits<float>::infinity()) const
        {
            SpatialRaycastResult result;
            Raycast(*m_Root, ray, maxDistance, result);
            return result;
        }

        [[nodiscard]]
        std::vector<SpatialAABB> ExtractCells() const
        {
            std::vector<SpatialAABB> cells;
            ExtractCells(*m_Root, cells);
            return cells;
        }

        void Clear()
        {
            m_Root = std::make_unique<Node>(m_WorldBounds, 0);
        }

    private:
        struct Node final
        {
            SpatialAABB Bounds;
            SpatialAABB LooseBounds;
            std::uint32_t Depth = 0;
            std::vector<OctreeItem> Items;
            std::array<std::unique_ptr<Node>, 8> Children;

            Node(
                const SpatialAABB& bounds,
                std::uint32_t depth)
                : Bounds(bounds)
                , LooseBounds(bounds)
                , Depth(depth)
            {
            }

            [[nodiscard]]
            bool HasChildren() const noexcept
            {
                return Children[0] != nullptr;
            }
        };

        SpatialAABB m_WorldBounds;
        OctreeSettings m_Settings;
        std::unique_ptr<Node> m_Root;

        [[nodiscard]]
        SpatialAABB MakeLoose(
            const SpatialAABB& bounds) const noexcept
        {
            SpatialAABB loose = bounds;
            const Vec3f amount =
                bounds.Extent() * (m_Settings.LooseFactor - 1.0f);
            loose.Expand(amount);
            return loose;
        }

        void Subdivide(
            Node& node)
        {
            const Vec3f center = node.Bounds.Center();
            for (std::size_t i = 0; i < 8; ++i)
            {
                const Vec3f childMin
                {
                    (i & 1) ? center.x : node.Bounds.Min.x,
                    (i & 2) ? center.y : node.Bounds.Min.y,
                    (i & 4) ? center.z : node.Bounds.Min.z
                };

                const Vec3f childMax
                {
                    (i & 1) ? node.Bounds.Max.x : center.x,
                    (i & 2) ? node.Bounds.Max.y : center.y,
                    (i & 4) ? node.Bounds.Max.z : center.z
                };

                SpatialAABB bounds =
                    SpatialAABB::FromMinMax(childMin, childMax);

                node.Children[i] =
                    std::make_unique<Node>(bounds, node.Depth + 1);
                node.Children[i]->LooseBounds = MakeLoose(bounds);
            }
        }

        [[nodiscard]]
        Node* ChildContaining(
            Node& node,
            const SpatialAABB& bounds)
        {
            if (!node.HasChildren())
            {
                return nullptr;
            }

            for (std::unique_ptr<Node>& child : node.Children)
            {
                if (child->LooseBounds.ContainsAABB(bounds))
                {
                    return child.get();
                }
            }

            return nullptr;
        }

        void Insert(
            Node& node,
            const OctreeItem& item)
        {
            if (node.Depth < m_Settings.MaxDepth)
            {
                if (!node.HasChildren() &&
                    node.Items.size() >= m_Settings.MaxItemsPerNode)
                {
                    Subdivide(node);

                    std::vector<OctreeItem> oldItems =
                        std::move(node.Items);

                    for (const OctreeItem& oldItem : oldItems)
                    {
                        Insert(node, oldItem);
                    }
                }

                if (Node* child = ChildContaining(node, item.Bounds))
                {
                    Insert(*child, item);
                    return;
                }
            }

            node.Items.push_back(item);
        }

        bool Remove(
            Node& node,
            SpatialID id)
        {
            auto found =
                std::find_if(
                    node.Items.begin(),
                    node.Items.end(),
                    [id](const OctreeItem& item)
                    {
                        return item.ID == id;
                    });

            if (found != node.Items.end())
            {
                node.Items.erase(found);
                return true;
            }

            if (node.HasChildren())
            {
                for (std::unique_ptr<Node>& child : node.Children)
                {
                    if (Remove(*child, id))
                    {
                        CollapseIfEmpty(node);
                        return true;
                    }
                }
            }

            return false;
        }

        void CollapseIfEmpty(
            Node& node)
        {
            if (!node.HasChildren())
            {
                return;
            }

            for (const std::unique_ptr<Node>& child : node.Children)
            {
                if (!child->Items.empty() || child->HasChildren())
                {
                    return;
                }
            }

            for (std::unique_ptr<Node>& child : node.Children)
            {
                child.reset();
            }
        }

        void QueryAABB(
            const Node& node,
            const SpatialAABB& query,
            std::vector<SpatialID>& result) const
        {
            if (!Intersects(node.LooseBounds, query))
            {
                return;
            }

            for (const OctreeItem& item : node.Items)
            {
                if (Intersects(item.Bounds, query))
                {
                    result.push_back(item.ID);
                }
            }

            if (node.HasChildren())
            {
                for (const std::unique_ptr<Node>& child : node.Children)
                {
                    QueryAABB(*child, query, result);
                }
            }
        }

        void QuerySphere(
            const Node& node,
            const Vec3f& center,
            float radius,
            std::vector<SpatialID>& result) const
        {
            if (!OverlapsSphere(node.LooseBounds, center, radius))
            {
                return;
            }

            for (const OctreeItem& item : node.Items)
            {
                if (OverlapsSphere(item.Bounds, center, radius))
                {
                    result.push_back(item.ID);
                }
            }

            if (node.HasChildren())
            {
                for (const std::unique_ptr<Node>& child : node.Children)
                {
                    QuerySphere(*child, center, radius, result);
                }
            }
        }

        void QueryFrustum(
            const Node& node,
            const Frustumf& frustum,
            std::vector<SpatialID>& result) const
        {
            if (!frustum.IntersectsAABB(node.LooseBounds))
            {
                return;
            }

            for (const OctreeItem& item : node.Items)
            {
                if (frustum.IntersectsAABB(item.Bounds))
                {
                    result.push_back(item.ID);
                }
            }

            if (node.HasChildren())
            {
                for (const std::unique_ptr<Node>& child : node.Children)
                {
                    QueryFrustum(*child, frustum, result);
                }
            }
        }

        void Raycast(
            const Node& node,
            const SpatialRay& ray,
            float maxDistance,
            SpatialRaycastResult& result) const
        {
            float nodeDistance = 0.0f;
            if (!RayIntersectsAABB(ray, node.LooseBounds, maxDistance, nodeDistance))
            {
                return;
            }

            ++result.VisitedNodes;
            for (const OctreeItem& item : node.Items)
            {
                float distance = 0.0f;
                ++result.TestedPrimitives;
                if (RayIntersectsAABB(ray, item.Bounds, maxDistance, distance) &&
                    distance < result.Closest.Distance)
                {
                    result.Hit = true;
                    result.Closest.ID = item.ID;
                    result.Closest.Distance = distance;
                    result.Closest.Position = ray.GetPoint(distance);
                }
            }

            if (node.HasChildren())
            {
                for (const std::unique_ptr<Node>& child : node.Children)
                {
                    Raycast(*child, ray, maxDistance, result);
                }
            }
        }

        void ExtractCells(
            const Node& node,
            std::vector<SpatialAABB>& cells) const
        {
            cells.push_back(node.LooseBounds);
            if (node.HasChildren())
            {
                for (const std::unique_ptr<Node>& child : node.Children)
                {
                    ExtractCells(*child, cells);
                }
            }
        }
    };
}
