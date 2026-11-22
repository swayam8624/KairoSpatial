// TEACHING MAP: KairoPhysicsEngine's BroadphaseWorld translates ColliderID
// into SpatialID and keeps a SpatialIndex proxy for each *finite* collider.
// This DynamicAABBTree owns index nodes, not rigid bodies or exact geometry.
// KairoPhysicsEngine::Narrowphase still tests candidate shape pairs.
//
// Each leaf stores TightBounds (current spatial shape bounds) and FatBounds
// (expanded bounds allowing limited motion before re-insertion). An internal
// node encloses its children. Query methods prune on fat internal boxes and
// test tight leaf boxes; the distinction limits false positive candidates.
// Insert returns a proxy handle; Update may refit or remove/reinsert a leaf.
//
// This tree's ComputePairs performs a QueryAABB for each allocated leaf,
// canonicalizes pair IDs, then sorts/deduplicates. It is not an exact-pair
// oracle or a claim of optimal complexity for dense/large workloads.
// RebuildBottomUp clears/reinserts leaves and can invalidate old proxy
// handles stored in other modules: callers must rebuild their handle map.
//
// See PhysicsEngine/Broadphase.cppm, SpatialTests.cpp and ScaleStressTests.cpp.
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
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    // Internal nodes contain child indices; leaves contain externally
    // supplied UserID values. SpatialIndex is an allocation handle and
    // must not be confused with SpatialID/PhysicsEngine's ColliderID.
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
        /// TEACHING API: SpatialIndex Insert( — Allocate fat-AABB leaf proxy with stable user ID; refit ancestors after insertion. PhysicsEngine stores proxy separately from collider.
        // Own a new *leaf proxy*, not a physics body or mesh. TightBounds
        // is exact for the caller's AABB; FatBounds includes movement
        // padding, reducing tree restructuring on nearby updates.
        // The returned SpatialIndex is invalid after removal/rebuild.
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

        // Deletes an index leaf and returns its node allocation to the
        // free list. The owning application must separately delete or
        // deactivate its real collider/primitive record.
        // Detach the leaf and free its tree slot only. The caller
        // still owns the associated primitive/ColliderID and is
        // responsible for updating that authoritative record.
        void Remove(
            SpatialIndex proxy)
        {
            RequireAllocated(proxy);
            RemoveLeaf(proxy);
            FreeNode(proxy);
        }

        [[nodiscard]]
        /// TEACHING API: bool Update( — Avoid tree restructuring when new tight box remains inside fat bounds; otherwise remove/reinsert to recover valid traversal.
        // Fast path: if newBounds is inside prior fat bounds, update
        // the tight leaf only and return false (no reinsertion).
        // Slow path: detach/reinsert the same leaf with new padding
        // and return true. Broadphase still owns the proxy mapping.
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
        // Traverse fat hierarchy bounds for pruning, then check tight
        // bounds and masks on leaves. Returns candidate IDs, not actual
        // geometry intersections; physics narrowphase checks those.
        // Iterative stack traversal prunes *fat internal* bounds;
        // a leaf only qualifies if its tight AABB and layer mask
        // intersect. IDs are conservative spatial candidates, not
        // manifold or precise mesh triangle intersections.
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
        // Use sphere-vs-AABB rejection on the hierarchy and tight
        // leaves, rather than a per-shape collision response.
        // radius <= 0 or nonfinite radius yields no candidates.
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
        // Collect *all* finite ray-vs-tight-AABB candidates and return
        // ID-sorted results for deterministic callers. This function
        // is not an exact raycast against spheres, capsules or meshes.
        // PhysicsWorld::RaycastCollider resolves actual hit distances.
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
        /// TEACHING API: std::vector<SpatialPair> ComputePairs( — Find canonically ordered candidate pairs using tree overlap, not exact collider-to-collider contact.
        // Simpler correctness-first O(n * query-cost) sweep: query
        // each live leaf's tight bounds against the same tree, then
        // canonicalize/deduplicate. This may be costly for dense scenes.
        // Correctness-first broadphase pairs: for every allocated
        // leaf, query its tight bounds through the same tree. Pair
        // duplicates and self-pairs are then normalized/removed.
        // Work is roughly O(n * query cost) before sort/dedup and can
        // be expensive for dense overlapping scenes.
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
        /// TEACHING API: SpatialRaycastResult RaycastBroadphase( — Traverse ray-vs-box intersections only; caller must still intersect real geometry to identify a surface hit.
        // Track the closest *AABB* entry, not the true surface.
        // Returning an AABB point/zero normal is deliberately less
        // informative than a primitive-specific callback raycast.
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

        // Callback overload: delegate the true intersection of a
        // candidate primitive to user code and tighten the current
        // maximum hit distance for subsequent subtree pruning.
        // This preserves spatial indexing separate from geometry math.
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

        /// TEACHING API: void RebuildBottomUp( — Reconstruct tree from live leaf data to improve hierarchy quality; invalidates old internal node topology.
        // WARNING: full Clear/Insert creates fresh node handles. Even if
        // UserID is preserved, an external cache of SpatialIndex proxies
        // must be rebuilt after this operation.
        // Save live (UserID,TightBounds,Mask) records, Clear the tree
        // and reinsert. This completely reallocates proxy indices:
        // any external SpatialIndex cached by PhysicsEngine becomes
        // stale and must be rederived from returned IDs before use.
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
        /// TEACHING API: SpatialValidationResult Validate( — Audit tree parent links, leaf bounds and hierarchy invariants for debugging after insertions/removals.
        // Audit the hierarchy rather than assuming insertion always
        // left correct links: check root, parent/child connectivity,
        // heights, bounding enclosure and leaf allocation invariants.
        // Useful after randomized insert/update/remove stress sequences.
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
        // Observe actual allocated nodes, leaf count, height, balance
        // and area ratio. Poor ratios indicate pruning overhead;
        // a measured benchmark is still needed to claim performance.
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

        // Destroy all current node allocations, resetting root;
        // every SpatialIndex becomes invalid. User-owned primitive
        // records still exist outside this acceleration structure.
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
        // Current implementation appends rather than reuses freed
        // indices. Frequent reinsertion therefore grows m_Nodes
        // despite some old slots becoming unallocated. Measure or
        // redesign node freelist reuse in a separate functional pass.
        SpatialIndex AllocateNode()
        {
            const SpatialIndex index =
                static_cast<SpatialIndex>(m_Nodes.size());

            m_Nodes.push_back(DynamicAABBTreeNode{});
            m_Nodes.back().Allocated = true;
            return index;
        }

        // Reset a slot to unallocated, but do not return it to a
        // reusable freelist in this implementation. Caller handles
        // pointing to this index must never be used again.
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

        // Descend using a greedy incremental surface-area enlargement
        // cost to choose a sibling, create an internal parent and refit
        // ancestors. Greedy choice is a heuristic, not a global optimum.
        // Greedy surface-area-enlargement descent chooses a sibling
        // then creates a parent enclosing two children. Refit parent
        // bounds/heights toward root; this is heuristic tree quality,
        // not guaranteed globally optimal balancing.
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

        // Splice the remaining sibling into the removed parent's
        // position, then refit ancestors. This changes parent/index
        // topology but does not remove the authoritative primitive.
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

        // Recompute parent fat bounds and subtree heights from children
        // up to the root after structural updates.
        // Walk parents until root, recomputing each internal node's
        // fat union and max-child height. Stale parent bounds could
        // cause false negatives during query pruning.
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
