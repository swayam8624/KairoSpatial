// TEACHING MODULE: BVHTraversal consumes the BVH built by BVHBuilder.
// It visits overlapping bounding nodes, then passes leaf UserIndex values
// to the caller's primitive-intersection callback. Neither traversal nor
// the BVH alone knows how to intersect the actual triangle or mesh object.
//
// Raycast tracks the closest accepted distance and prunes later boxes.
// Its Hits array records accepted progressively nearer hits encountered
// during traversal; do not assume it lists *all* intersections. Use
// RaycastAny for an early boolean hit/occlusion result. QueryAABB/QuerySphere
// instead return candidate primitive IDs after bounding-volume filtering.
//
// BVH nodes/index ranges are owned by BVH. A leaf refers into an ordered
// primitive array; UserID (external identity) differs from UserIndex
// (source primitive index). The physics mesh narrowphase consumes the
// candidate-list API; a renderer/ray tracer supplies exact hit callbacks.
module;
#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

export module Kairo.Foundation.Spatial.BVHTraversal;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Geometry.Frustum;
import Kairo.Foundation.Spatial.Types;
import Kairo.Foundation.Spatial.BVH;

export namespace kairo::foundation::spatial
{
    // Statistics are observations of traversal workload. They do not
    // certify exact geometry correctness or cross-machine performance.
    struct BVHTraversalStats final
    {
        std::uint32_t VisitedNodes = 0;
        std::uint32_t TestedPrimitives = 0;
    };

    template<typename PrimitiveHitCallback>
    [[nodiscard]]
    /// TEACHING API: SpatialRaycastResult Raycast( — AABB tests prune nodes while callback performs exact primitive hit testing; tighten closest ray distance when a nearer hit is found.
    // The callback must compute the hit for a source primitive index
    // and return an optional distance-bearing hit. The traversal fills
    // external UserID and primitive index into accepted hit records.
    // Traverse BVH boxes with an explicit stack but delegate true ray/primitive
    // intersection to caller-owned callback. Tighten closestDistance when a
    // nearer hit appears; SpatialRaycastResult::Hits tracks progressively
    // accepted hits, not every intersecting surface.
    SpatialRaycastResult Raycast(
        const BVH& bvh,
        const SpatialRay& ray,
        PrimitiveHitCallback&& primitiveHit,
        float maxDistance = std::numeric_limits<float>::infinity(),
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        SpatialRaycastResult result;

        if (bvh.Empty() || !bvh.IsValid())
        {
            return result;
        }

        std::vector<SpatialIndex> stack;
        stack.reserve(64);
        stack.push_back(0);

        // Monotonically lower this bound as closer hits are discovered.
        // Earlier accepted records may remain in Hits despite no longer
        // being closest, so use Closest for the final nearest hit.
        // Maintain the best exact hit distance found so far. Any node with its
        // nearest box entry beyond this value cannot contain a better answer and can
        // be pruned; box tests alone never create a SurfaceHit.
        float closestDistance = maxDistance;

        while (!stack.empty())
        {
            const SpatialIndex nodeIndex = stack.back();
            stack.pop_back();

            const BVHNode& node = bvh.Nodes[nodeIndex];
            ++result.VisitedNodes;

            float boundsDistance = 0.0f;
            if (!RayIntersectsAABB(ray, node.Bounds, closestDistance, boundsDistance))
            {
                continue;
            }

            if (node.IsLeaf())
            {
                for (std::uint32_t i = 0; i < node.PrimitiveCount; ++i)
                {
                    const BVHPrimitive& primitive =
                        bvh.PrimitiveAtOrderedOffset(node.FirstPrimitive + i);

                    if ((primitive.LayerMask & layerMask) == 0)
                    {
                        continue;
                    }

                    ++result.TestedPrimitives;
                    // Call the source primitive's exact geometry intersection only after BVH and
                    // layer-mask pruning. Populate UserID/UserIndex from BVH leaf metadata so
                    // render or physics consumers can map the result back.
                    std::optional<SpatialRayHit> hit =
                        primitiveHit(primitive.UserIndex, ray);

                    if (hit && hit->Distance >= 0.0f && hit->Distance <= closestDistance)
                    {
                        hit->ID = primitive.UserID;
                        hit->PrimitiveIndex = primitive.UserIndex;
                        result.Hits.push_back(*hit);
                        result.Hit = true;
                        result.Closest = *hit;
                        closestDistance = hit->Distance;
                    }
                }
            }
            else
            {
                float leftDistance = 0.0f;
                float rightDistance = 0.0f;
                const bool hitLeft =
                    RayIntersectsAABB(ray, bvh.Nodes[node.Left].Bounds, closestDistance, leftDistance);
                const bool hitRight =
                    RayIntersectsAABB(ray, bvh.Nodes[node.Right].Bounds, closestDistance, rightDistance);

                // Push the farther child first because this stack is LIFO; visit the nearer
                // child immediately. Finding a closer exact hit earlier improves subsequent
                // max-distance AABB pruning without changing closest-hit correctness.
                if (hitLeft && hitRight)
                {
                    if (leftDistance < rightDistance)
                    {
                        stack.push_back(node.Right);
                        stack.push_back(node.Left);
                    }
                    else
                    {
                        stack.push_back(node.Left);
                        stack.push_back(node.Right);
                    }
                }
                else if (hitLeft)
                {
                    stack.push_back(node.Left);
                }
                else if (hitRight)
                {
                    stack.push_back(node.Right);
                }
            }
        }

        return result;
    }

    template<typename PrimitiveHitCallback>
    [[nodiscard]]
    /// TEACHING API: bool RaycastAny( — Short-circuit on first accepted callback hit; cheaper than finding closest when caller needs only occlusion.
    // Occlusion/any-hit mode: return as soon as the caller callback
    // accepts a primitive. Unlike Raycast, no nearest-hit distance is
    // maintained and no materialized hit records are required.
    // Any-hit traversal needs only one callback-accepted primitive, not nearest
    // distance or UV/material details. Scene::Occluded supplies a finite
    // shadow-ray limit and a true primitive hit callback.
    bool RaycastAny(
        const BVH& bvh,
        const SpatialRay& ray,
        PrimitiveHitCallback&& primitiveHit,
        float maxDistance = std::numeric_limits<float>::infinity(),
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        if (bvh.Empty() || !bvh.IsValid())
        {
            return false;
        }

        std::vector<SpatialIndex> stack;
        stack.reserve(64);
        stack.push_back(0);

        while (!stack.empty())
        {
            const SpatialIndex nodeIndex = stack.back();
            stack.pop_back();

            const BVHNode& node = bvh.Nodes[nodeIndex];
            float boundsDistance = 0.0f;
            if (!RayIntersectsAABB(ray, node.Bounds, maxDistance, boundsDistance))
            {
                continue;
            }

            if (node.IsLeaf())
            {
                for (std::uint32_t i = 0; i < node.PrimitiveCount; ++i)
                {
                    const BVHPrimitive& primitive =
                        bvh.PrimitiveAtOrderedOffset(node.FirstPrimitive + i);

                    if ((primitive.LayerMask & layerMask) == 0)
                    {
                        continue;
                    }

                    if (primitiveHit(primitive.UserIndex, ray))
                    {
                        return true;
                    }
                }
            }
            else
            {
                stack.push_back(node.Left);
                stack.push_back(node.Right);
            }
        }

        return false;
    }

    [[nodiscard]]
    /// TEACHING API: SpatialOverlapResult QueryAABB( — Traverse bounding-box candidates and filter by layer mask; outputs are potential overlaps for downstream exact tests.
    // Box overlap candidates are returned as source UserIndex plus
    // external UserID. Downstream geometry still decides physical
    // contact for non-box shapes.
    // BVH overlap query returns bounded primitive candidates by layer mask, not
    // verified triangle or sphere collision. The downstream narrowphase or
    // caller's shape predicate must decide actual contact.
    SpatialOverlapResult QueryAABB(
        const BVH& bvh,
        const SpatialAABB& query,
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        SpatialOverlapResult result;
        if (bvh.Empty() || !bvh.IsValid() || !query.IsValid())
        {
            return result;
        }

        std::vector<SpatialIndex> stack{ 0 };
        while (!stack.empty())
        {
            const SpatialIndex nodeIndex = stack.back();
            stack.pop_back();

            const BVHNode& node = bvh.Nodes[nodeIndex];
            ++result.VisitedNodes;

            if (!Intersects(node.Bounds, query))
            {
                continue;
            }

            if (node.IsLeaf())
            {
                for (std::uint32_t i = 0; i < node.PrimitiveCount; ++i)
                {
                    const BVHPrimitive& primitive =
                        bvh.PrimitiveAtOrderedOffset(node.FirstPrimitive + i);

                    if ((primitive.LayerMask & layerMask) != 0 &&
                        Intersects(primitive.Bounds, query))
                    {
                        result.IDs.push_back(primitive.UserID);
                        result.PrimitiveIndices.push_back(primitive.UserIndex);
                    }

                    ++result.TestedPrimitives;
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
    SpatialOverlapResult QuerySphere(
        const BVH& bvh,
        const Vec3f& center,
        float radius,
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        SpatialOverlapResult result;
        if (bvh.Empty() || !bvh.IsValid() || radius < 0.0f)
        {
            return result;
        }

        std::vector<SpatialIndex> stack{ 0 };
        while (!stack.empty())
        {
            const SpatialIndex nodeIndex = stack.back();
            stack.pop_back();

            const BVHNode& node = bvh.Nodes[nodeIndex];
            ++result.VisitedNodes;

            if (!OverlapsSphere(node.Bounds, center, radius))
            {
                continue;
            }

            if (node.IsLeaf())
            {
                for (std::uint32_t i = 0; i < node.PrimitiveCount; ++i)
                {
                    const BVHPrimitive& primitive =
                        bvh.PrimitiveAtOrderedOffset(node.FirstPrimitive + i);

                    if ((primitive.LayerMask & layerMask) != 0 &&
                        OverlapsSphere(primitive.Bounds, center, radius))
                    {
                        result.IDs.push_back(primitive.UserID);
                        result.PrimitiveIndices.push_back(primitive.UserIndex);
                    }

                    ++result.TestedPrimitives;
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
    SpatialOverlapResult QueryPoint(
        const BVH& bvh,
        const Vec3f& point,
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        return QueryAABB(
            bvh,
            SpatialAABB::FromPoint(point),
            layerMask);
    }

    [[nodiscard]]
    /// TEACHING API: SpatialOverlapResult QueryFrustum( — Cull entire nodes with frustum plane tests, then report surviving primitive bounds as possible visible objects.
    SpatialOverlapResult QueryFrustum(
        const BVH& bvh,
        const Frustumf& frustum,
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        SpatialOverlapResult result;
        if (bvh.Empty() || !bvh.IsValid())
        {
            return result;
        }

        std::vector<SpatialIndex> stack{ 0 };
        while (!stack.empty())
        {
            const SpatialIndex nodeIndex = stack.back();
            stack.pop_back();

            const BVHNode& node = bvh.Nodes[nodeIndex];
            ++result.VisitedNodes;

            if (!frustum.IntersectsAABB(node.Bounds))
            {
                continue;
            }

            if (node.IsLeaf())
            {
                for (std::uint32_t i = 0; i < node.PrimitiveCount; ++i)
                {
                    const BVHPrimitive& primitive =
                        bvh.PrimitiveAtOrderedOffset(node.FirstPrimitive + i);

                    if ((primitive.LayerMask & layerMask) != 0 &&
                        frustum.IntersectsAABB(primitive.Bounds))
                    {
                        result.IDs.push_back(primitive.UserID);
                        result.PrimitiveIndices.push_back(primitive.UserIndex);
                    }

                    ++result.TestedPrimitives;
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
}
