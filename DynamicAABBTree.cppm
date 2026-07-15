module;

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <stdexcept>
#include <vector>

export module Kairo.Foundation.Spatial.DynamicAABBTree;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    struct DynamicAABBTreeNode final
    {
        SpatialAABB TightBounds = SpatialAABB::Empty();
        SpatialAABB FatBounds = SpatialAABB::Empty();
        SpatialIndex Parent = SpatialInvalidIndex;
        SpatialIndex Left = SpatialInvalidIndex;
        SpatialIndex Right = SpatialInvalidIndex;
        SpatialIndex NextFree = SpatialInvalidIndex;
        std::uint32_t Height = 0;
        SpatialID UserID = SpatialInvalidID;
        SpatialLayerMask LayerMask = 1u;
        bool Allocated = false;

        [[nodiscard]]
        constexpr bool IsLeaf() const noexcept
        {
            return Left == SpatialInvalidIndex;
        }
    };

    struct DynamicAABBTreeStats final
    {
        std::uint32_t Height = 0;
        std::uint32_t MaxBalance = 0;
        float AreaRatio = 0.0f;
        std::uint32_t AllocatedNodeCount = 0;
        std::uint32_t LeafCount = 0;
    };

    class DynamicAABBTree final
    {
    public:
        explicit DynamicAABBTree(
            float fattenAmount = 0.1f)
            : m_FattenAmount(fattenAmount)
        {
        }

        [[nodiscard]]
        SpatialIndex Insert(
            SpatialID id,
            const SpatialAABB& bounds,
            SpatialLayerMask layerMask = 1u)
        {
            if (!bounds.IsValid())
            {
                throw std::invalid_argument("DynamicAABBTree::Insert failed: invalid bounds.");
            }

            const SpatialIndex leaf =
                AllocateNode();

            DynamicAABBTreeNode& node = m_Nodes[leaf];
            node.TightBounds = bounds;
            node.FatBounds = Expanded(bounds, m_FattenAmount);
            node.UserID = id;
            node.LayerMask = layerMask;
            node.Height = 0;
            node.Allocated = true;

            InsertLeaf(leaf);
            return leaf;
        }

        void Remove(
            SpatialIndex proxy)
        {
            RequireAllocated(proxy);
            RemoveLeaf(proxy);
            FreeNode(proxy);
        }

        [[nodiscard]]
        bool Update(
            SpatialIndex proxy,
            const SpatialAABB& newBounds)
        {
            RequireAllocated(proxy);
            if (!newBounds.IsValid())
            {
                throw std::invalid_argument("DynamicAABBTree::Update failed: invalid bounds.");
            }

            if (m_Nodes[proxy].FatBounds.ContainsAABB(newBounds))
            {
                m_Nodes[proxy].TightBounds = newBounds;
                return false;
            }

            RemoveLeaf(proxy);
            m_Nodes[proxy].TightBounds = newBounds;
            m_Nodes[proxy].FatBounds = Expanded(newBounds, m_FattenAmount);
            InsertLeaf(proxy);
            return true;
        }

        [[nodiscard]]
        SpatialOverlapResult QueryAABB(
            const SpatialAABB& query,
            SpatialLayerMask layerMask = ~SpatialLayerMask(0)) const
        {
            SpatialOverlapResult result;
            if (m_Root == SpatialInvalidIndex || !query.IsValid())
            {
                return result;
            }

            std::vector<SpatialIndex> stack{ m_Root };
            while (!stack.empty())
            {
                const SpatialIndex index = stack.back();
                stack.pop_back();
                const DynamicAABBTreeNode& node = m_Nodes[index];
                ++result.VisitedNodes;

                if (!Intersects(node.FatBounds, query))
                {
                    continue;
                }

                if (node.IsLeaf())
                {
                    if ((node.LayerMask & layerMask) != 0 &&
                        Intersects(node.TightBounds, query))
                    {
                        result.IDs.push_back(node.UserID);
                        result.PrimitiveIndices.push_back(index);
                    }
                }
                else
                {
                    stack.push_back(node.Left);
                    stack.push_back(node.Right);
                }
            }

            return result;
        }

        /// Input: a sphere expressed as a world-space center and positive radius.
        /// Output: leaf identifiers whose tight AABBs overlap that sphere.
        /// Task: provide a conservative, allocation-owned candidate set for
        /// higher-level systems such as physics overlap queries. Callers still
        /// perform their shape-specific narrowphase test after this traversal.
        [[nodiscard]]
        SpatialOverlapResult QuerySphere(
            const Vec3f& center,
            float radius,
            SpatialLayerMask layerMask = ~SpatialLayerMask(0)) const
        {
            SpatialOverlapResult result;
            if (m_Root == SpatialInvalidIndex ||
                !std::isfinite(radius) || radius <= 0.0f)
            {
                return result;
            }

            std::vector<SpatialIndex> stack{ m_Root };
            while (!stack.empty())
            {
                const SpatialIndex index = stack.back();
                stack.pop_back();
                const DynamicAABBTreeNode& node = m_Nodes[index];
                ++result.VisitedNodes;

                if (!OverlapsSphere(node.FatBounds, center, radius))
                {
                    continue;
                }

                if (node.IsLeaf())
                {
                    if ((node.LayerMask & layerMask) != 0u &&
                        OverlapsSphere(node.TightBounds, center, radius))
                    {
                        ++result.TestedPrimitives;
                        result.IDs.push_back(node.UserID);
                        result.PrimitiveIndices.push_back(index);
                    }
                }
                else
                {
                    stack.push_back(node.Left);
                    stack.push_back(node.Right);
                }
            }

            return result;
        }

        /// Input: normalized world-space ray, maximum distance, and layer mask.
        /// Output: every finite-tree leaf whose tight AABB intersects the ray.
        /// Task: expose broadphase candidates without applying nearest-hit
        /// pruning, because a physics world must test exact shapes and support
        /// multi-hit rays. Results are sorted by identifier for deterministic
        /// callers independent of tree rotation history.
        [[nodiscard]]
        SpatialOverlapResult QueryRay(
            const SpatialRay& ray,
            float maxDistance = std::numeric_limits<float>::infinity(),
            SpatialLayerMask layerMask = ~SpatialLayerMask(0)) const
        {
            SpatialOverlapResult result;
            if (m_Root == SpatialInvalidIndex ||
                std::isnan(maxDistance) || maxDistance <= 0.0f)
            {
                return result;
            }

            std::vector<SpatialIndex> stack{ m_Root };
            while (!stack.empty())
            {
                const SpatialIndex index = stack.back();
                stack.pop_back();
                const DynamicAABBTreeNode& node = m_Nodes[index];
                ++result.VisitedNodes;

                float fatDistance = 0.0f;
                if (!RayIntersectsAABB(ray, node.FatBounds, maxDistance, fatDistance))
                {
                    continue;
                }

                if (node.IsLeaf())
                {
                    float tightDistance = 0.0f;
                    if ((node.LayerMask & layerMask) != 0u &&
                        RayIntersectsAABB(ray, node.TightBounds, maxDistance, tightDistance))
                    {
                        ++result.TestedPrimitives;
                        result.IDs.push_back(node.UserID);
                        result.PrimitiveIndices.push_back(index);
                    }
                }
                else
                {
                    stack.push_back(node.Left);
                    stack.push_back(node.Right);
                }
            }

            std::vector<std::size_t> order(result.IDs.size());
            for (std::size_t i = 0; i < order.size(); ++i)
            {
                order[i] = i;
            }

            std::sort(order.begin(), order.end(), [&](std::size_t lhs, std::size_t rhs)
            {
                return result.IDs[lhs] < result.IDs[rhs];
            });

            SpatialOverlapResult sorted;
            sorted.VisitedNodes = result.VisitedNodes;
            sorted.TestedPrimitives = result.TestedPrimitives;
            sorted.IDs.reserve(order.size());
            sorted.PrimitiveIndices.reserve(order.size());
            for (const std::size_t index : order)
            {
                sorted.IDs.push_back(result.IDs[index]);
                sorted.PrimitiveIndices.push_back(result.PrimitiveIndices[index]);
            }

            return sorted;
        }

        [[nodiscard]]
        std::vector<SpatialPair> ComputePairs(
            SpatialLayerMask layerMask = ~SpatialLayerMask(0)) const
        {
            std::vector<SpatialPair> pairs;

            for (std::size_t i = 0; i < m_Nodes.size(); ++i)
            {
                if (!m_Nodes[i].Allocated ||
                    !m_Nodes[i].IsLeaf() ||
                    (m_Nodes[i].LayerMask & layerMask) == 0)
                {
                    continue;
                }

                const SpatialOverlapResult overlaps =
                    QueryAABB(m_Nodes[i].TightBounds, layerMask);

                for (SpatialID id : overlaps.IDs)
                {
                    if (id != m_Nodes[i].UserID)
                    {
                        pairs.push_back(
                            OrderedPair(m_Nodes[i].UserID, id));
                    }
                }
            }

            std::sort(
                pairs.begin(),
                pairs.end(),
                [](const SpatialPair& lhs, const SpatialPair& rhs)
                {
                    return lhs.A == rhs.A ? lhs.B < rhs.B : lhs.A < rhs.A;
                });

            pairs.erase(
                std::unique(pairs.begin(), pairs.end()),
                pairs.end());

            return pairs;
        }

        [[nodiscard]]
        SpatialRaycastResult RaycastBroadphase(
            const SpatialRay& ray,
            float maxDistance = std::numeric_limits<float>::infinity(),
            SpatialLayerMask layerMask = ~SpatialLayerMask(0)) const
        {
            SpatialRaycastResult result;
            if (m_Root == SpatialInvalidIndex)
            {
                return result;
            }

            std::vector<SpatialIndex> stack{ m_Root };
            while (!stack.empty())
            {
                const SpatialIndex index = stack.back();
                stack.pop_back();

                const DynamicAABBTreeNode& node = m_Nodes[index];
                ++result.VisitedNodes;

                float distance = 0.0f;
                if (!RayIntersectsAABB(ray, node.FatBounds, maxDistance, distance))
                {
                    continue;
                }

                if (node.IsLeaf())
                {
                    if ((node.LayerMask & layerMask) == 0)
                    {
                        continue;
                    }

                    float tightDistance = 0.0f;
                    if (!RayIntersectsAABB(ray, node.TightBounds, maxDistance, tightDistance))
                    {
                        continue;
                    }

                    ++result.TestedPrimitives;
                    if (tightDistance < result.Closest.Distance)
                    {
                        result.Hit = true;
                        result.Closest =
                        {
                            node.UserID,
                            index,
                            tightDistance,
                            ray.GetPoint(tightDistance),
                            Vec3f::Zero()
                        };
                    }
                }
                else
                {
                    stack.push_back(node.Left);
                    stack.push_back(node.Right);
                }
            }

            return result;
        }

        [[nodiscard]]
        SpatialRaycastResult Raycast(
            const SpatialRay& ray,
            float maxDistance = std::numeric_limits<float>::infinity(),
            SpatialLayerMask layerMask = ~SpatialLayerMask(0)) const
        {
            return RaycastBroadphase(
                ray,
                maxDistance,
                layerMask);
        }

        template<typename PrimitiveHitCallback>
        [[nodiscard]]
        SpatialRaycastResult Raycast(
            const SpatialRay& ray,
            PrimitiveHitCallback&& primitiveHit,
            float maxDistance = std::numeric_limits<float>::infinity(),
            SpatialLayerMask layerMask = ~SpatialLayerMask(0)) const
        {
            SpatialRaycastResult result;
            if (m_Root == SpatialInvalidIndex)
            {
                return result;
            }

            std::vector<SpatialIndex> stack{ m_Root };
            float closestDistance = maxDistance;
            while (!stack.empty())
            {
                const SpatialIndex index = stack.back();
                stack.pop_back();

                const DynamicAABBTreeNode& node = m_Nodes[index];
                ++result.VisitedNodes;

                float distance = 0.0f;
                if (!RayIntersectsAABB(ray, node.FatBounds, closestDistance, distance))
                {
                    continue;
                }

                if (node.IsLeaf())
                {
                    if ((node.LayerMask & layerMask) == 0)
                    {
                        continue;
                    }

                    ++result.TestedPrimitives;
                    std::optional<SpatialRayHit> hit =
                        primitiveHit(index, node.UserID, ray);

                    if (hit && hit->Distance >= 0.0f && hit->Distance <= closestDistance)
                    {
                        hit->ID = node.UserID;
                        hit->PrimitiveIndex = index;
                        result.Hits.push_back(*hit);
                        result.Hit = true;
                        result.Closest = *hit;
                        closestDistance = hit->Distance;
                    }
                }
                else
                {
                    stack.push_back(node.Left);
                    stack.push_back(node.Right);
                }
            }

            return result;
        }

        void RebuildBottomUp()
        {
            std::vector<SpatialPrimitive> leaves;
            leaves.reserve(m_Nodes.size());

            for (const DynamicAABBTreeNode& node : m_Nodes)
            {
                if (node.Allocated && node.IsLeaf())
                {
                    leaves.push_back(
                        SpatialPrimitive
                        {
                            node.UserID,
                            node.TightBounds,
                            node.LayerMask
                        });
                }
            }

            Clear();
            for (const SpatialPrimitive& primitive : leaves)
            {
                const SpatialIndex proxy =
                    Insert(primitive.ID, primitive.Bounds, primitive.LayerMask);

                (void)proxy;
            }
        }

        [[nodiscard]]
        SpatialValidationResult Validate() const
        {
            if (m_Root == SpatialInvalidIndex)
            {
                return { true, {} };
            }

            if (m_Root >= m_Nodes.size() || !m_Nodes[m_Root].Allocated)
            {
                return { false, "DynamicAABBTree root is invalid." };
            }

            for (std::size_t i = 0; i < m_Nodes.size(); ++i)
            {
                const DynamicAABBTreeNode& node = m_Nodes[i];
                if (!node.Allocated)
                {
                    continue;
                }

                if (!node.FatBounds.IsValid())
                {
                    return { false, "DynamicAABBTree node has invalid bounds." };
                }

                if (node.IsLeaf() &&
                    (!node.TightBounds.IsValid() ||
                     !node.FatBounds.ContainsAABB(node.TightBounds)))
                {
                    return { false, "DynamicAABBTree leaf fat bounds do not contain tight bounds." };
                }

                if (!node.IsLeaf())
                {
                    if (node.Left >= m_Nodes.size() || node.Right >= m_Nodes.size())
                    {
                        return { false, "DynamicAABBTree internal node has invalid child index." };
                    }
                }
            }

            return { true, {} };
        }

        [[nodiscard]]
        const std::vector<DynamicAABBTreeNode>& Nodes() const noexcept
        {
            return m_Nodes;
        }

        [[nodiscard]]
        SpatialIndex RootIndex() const noexcept
        {
            return m_Root;
        }

        [[nodiscard]]
        DynamicAABBTreeStats Stats() const noexcept
        {
            DynamicAABBTreeStats stats;
            if (m_Root != SpatialInvalidIndex)
            {
                stats.Height = m_Nodes[m_Root].Height;
                const float rootArea = m_Nodes[m_Root].FatBounds.SurfaceArea();
                float totalArea = 0.0f;

                for (const DynamicAABBTreeNode& node : m_Nodes)
                {
                    if (!node.Allocated)
                    {
                        continue;
                    }

                    ++stats.AllocatedNodeCount;
                    totalArea += node.FatBounds.SurfaceArea();

                    if (node.IsLeaf())
                    {
                        ++stats.LeafCount;
                    }
                    else
                    {
                        const std::uint32_t leftHeight = m_Nodes[node.Left].Height;
                        const std::uint32_t rightHeight = m_Nodes[node.Right].Height;
                        const std::uint32_t balance =
                            leftHeight > rightHeight
                                ? leftHeight - rightHeight
                                : rightHeight - leftHeight;

                        stats.MaxBalance =
                            std::max(stats.MaxBalance, balance);
                    }
                }

                stats.AreaRatio =
                    rootArea > 0.0f
                        ? totalArea / rootArea
                        : 0.0f;
            }

            return stats;
        }

        void Clear()
        {
            m_Nodes.clear();
            m_Root = SpatialInvalidIndex;
        }

    private:
        std::vector<DynamicAABBTreeNode> m_Nodes;
        SpatialIndex m_Root = SpatialInvalidIndex;
        float m_FattenAmount = 0.1f;

        [[nodiscard]]
        SpatialIndex AllocateNode()
        {
            const SpatialIndex index =
                static_cast<SpatialIndex>(m_Nodes.size());

            m_Nodes.push_back(DynamicAABBTreeNode{});
            m_Nodes.back().Allocated = true;
            return index;
        }

        void FreeNode(
            SpatialIndex index)
        {
            m_Nodes[index] = DynamicAABBTreeNode{};
        }

        void RequireAllocated(
            SpatialIndex index) const
        {
            if (index >= m_Nodes.size() || !m_Nodes[index].Allocated)
            {
                throw std::out_of_range("DynamicAABBTree proxy is invalid.");
            }
        }

        void InsertLeaf(
            SpatialIndex leaf)
        {
            if (m_Root == SpatialInvalidIndex)
            {
                m_Root = leaf;
                m_Nodes[leaf].Parent = SpatialInvalidIndex;
                return;
            }

            SpatialIndex sibling = m_Root;
            while (!m_Nodes[sibling].IsLeaf())
            {
                const SpatialIndex left = m_Nodes[sibling].Left;
                const SpatialIndex right = m_Nodes[sibling].Right;

                const float leftCost =
                    Union(m_Nodes[left].FatBounds, m_Nodes[leaf].FatBounds).SurfaceArea() -
                    m_Nodes[left].FatBounds.SurfaceArea();

                const float rightCost =
                    Union(m_Nodes[right].FatBounds, m_Nodes[leaf].FatBounds).SurfaceArea() -
                    m_Nodes[right].FatBounds.SurfaceArea();

                sibling = leftCost < rightCost ? left : right;
            }

            const SpatialIndex oldParent = m_Nodes[sibling].Parent;
            const SpatialIndex parent = AllocateNode();
            m_Nodes[parent].Parent = oldParent;
            m_Nodes[parent].FatBounds = Union(m_Nodes[leaf].FatBounds, m_Nodes[sibling].FatBounds);
            m_Nodes[parent].Height = m_Nodes[sibling].Height + 1;
            m_Nodes[parent].Left = sibling;
            m_Nodes[parent].Right = leaf;
            m_Nodes[parent].UserID = SpatialInvalidID;

            m_Nodes[sibling].Parent = parent;
            m_Nodes[leaf].Parent = parent;

            if (oldParent == SpatialInvalidIndex)
            {
                m_Root = parent;
            }
            else if (m_Nodes[oldParent].Left == sibling)
            {
                m_Nodes[oldParent].Left = parent;
            }
            else
            {
                m_Nodes[oldParent].Right = parent;
            }

            RefitAncestors(parent);
        }

        void RemoveLeaf(
            SpatialIndex leaf)
        {
            if (leaf == m_Root)
            {
                m_Root = SpatialInvalidIndex;
                return;
            }

            const SpatialIndex parent = m_Nodes[leaf].Parent;
            const SpatialIndex grandParent = m_Nodes[parent].Parent;
            const SpatialIndex sibling =
                m_Nodes[parent].Left == leaf
                    ? m_Nodes[parent].Right
                    : m_Nodes[parent].Left;

            if (grandParent == SpatialInvalidIndex)
            {
                m_Root = sibling;
                m_Nodes[sibling].Parent = SpatialInvalidIndex;
            }
            else
            {
                if (m_Nodes[grandParent].Left == parent)
                {
                    m_Nodes[grandParent].Left = sibling;
                }
                else
                {
                    m_Nodes[grandParent].Right = sibling;
                }

                m_Nodes[sibling].Parent = grandParent;
                RefitAncestors(grandParent);
            }

            FreeNode(parent);
            m_Nodes[leaf].Parent = SpatialInvalidIndex;
        }

        void RefitAncestors(
            SpatialIndex index)
        {
            while (index != SpatialInvalidIndex)
            {
                DynamicAABBTreeNode& node = m_Nodes[index];
                if (!node.IsLeaf())
                {
                    const DynamicAABBTreeNode& left = m_Nodes[node.Left];
                    const DynamicAABBTreeNode& right = m_Nodes[node.Right];
                    node.FatBounds = Union(left.FatBounds, right.FatBounds);
                    node.Height = 1 + std::max(left.Height, right.Height);
                }

                index = node.Parent;
            }
        }
    };
}
