module;

#include <algorithm>
#include <cstddef>
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
    struct BVHTraversalStats final
    {
        std::uint32_t VisitedNodes = 0;
        std::uint32_t TestedPrimitives = 0;
    };

    template<typename PrimitiveHitCallback>
    [[nodiscard]]
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
