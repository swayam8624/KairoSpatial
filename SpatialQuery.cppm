module;

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <optional>
#include <vector>

export module Kairo.Foundation.Spatial.Query;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Spatial.Types;

export namespace kairo::foundation::spatial
{
    [[nodiscard]]
    std::vector<SpatialID> CollectOverlaps(
        const std::vector<SpatialPrimitive>& primitives,
        const SpatialAABB& query,
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        std::vector<SpatialID> ids;

        for (const SpatialPrimitive& primitive : primitives)
        {
            if ((primitive.LayerMask & layerMask) != 0 &&
                Intersects(primitive.Bounds, query))
            {
                ids.push_back(primitive.ID);
            }
        }

        return ids;
    }

    [[nodiscard]]
    std::vector<SpatialID> CollectSphereOverlaps(
        const std::vector<SpatialPrimitive>& primitives,
        const Vec3f& center,
        float radius,
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        std::vector<SpatialID> ids;

        for (const SpatialPrimitive& primitive : primitives)
        {
            if ((primitive.LayerMask & layerMask) != 0 &&
                OverlapsSphere(primitive.Bounds, center, radius))
            {
                ids.push_back(primitive.ID);
            }
        }

        return ids;
    }

    [[nodiscard]]
    std::vector<SpatialRayHit> SortByDistance(
        std::vector<SpatialRayHit> hits)
    {
        std::stable_sort(
            hits.begin(),
            hits.end(),
            [](
                const SpatialRayHit& lhs,
                const SpatialRayHit& rhs)
            {
                return lhs.Distance < rhs.Distance;
            });

        return hits;
    }

    [[nodiscard]]
    std::vector<SpatialPrimitive> FilterByLayer(
        const std::vector<SpatialPrimitive>& primitives,
        SpatialLayerMask layerMask)
    {
        std::vector<SpatialPrimitive> filtered;

        for (const SpatialPrimitive& primitive : primitives)
        {
            if ((primitive.LayerMask & layerMask) != 0)
            {
                filtered.push_back(primitive);
            }
        }

        return filtered;
    }

    template<typename Predicate>
    [[nodiscard]]
    std::optional<SpatialPrimitive> FindAny(
        const std::vector<SpatialPrimitive>& primitives,
        Predicate&& predicate)
    {
        for (const SpatialPrimitive& primitive : primitives)
        {
            if (predicate(primitive))
            {
                return primitive;
            }
        }

        return std::nullopt;
    }

    [[nodiscard]]
    std::optional<SpatialPrimitive> FindNearest(
        const std::vector<SpatialPrimitive>& primitives,
        const Vec3f& point,
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        std::optional<SpatialPrimitive> nearest;
        float nearestDistanceSquared =
            std::numeric_limits<float>::infinity();

        for (const SpatialPrimitive& primitive : primitives)
        {
            if ((primitive.LayerMask & layerMask) == 0)
            {
                continue;
            }

            const float distanceSquared =
                primitive.Bounds.DistanceSquaredToPoint(point);

            if (distanceSquared < nearestDistanceSquared)
            {
                nearest = primitive;
                nearestDistanceSquared = distanceSquared;
            }
        }

        return nearest;
    }

    template<typename HitCallback>
    [[nodiscard]]
    std::vector<SpatialRayHit> CollectRayHits(
        const std::vector<SpatialPrimitive>& primitives,
        const SpatialRay& ray,
        HitCallback&& hitCallback,
        float maxDistance = std::numeric_limits<float>::infinity(),
        SpatialLayerMask layerMask = ~SpatialLayerMask(0))
    {
        std::vector<SpatialRayHit> hits;

        for (std::size_t i = 0; i < primitives.size(); ++i)
        {
            const SpatialPrimitive& primitive = primitives[i];

            if ((primitive.LayerMask & layerMask) == 0)
            {
                continue;
            }

            float boundsDistance = 0.0f;
            if (!RayIntersectsAABB(ray, primitive.Bounds, maxDistance, boundsDistance))
            {
                continue;
            }

            std::optional<SpatialRayHit> hit =
                hitCallback(static_cast<SpatialIndex>(i), ray);

            if (hit && hit->Distance <= maxDistance)
            {
                hit->ID = primitive.ID;
                hit->PrimitiveIndex = static_cast<SpatialIndex>(i);
                hits.push_back(*hit);
            }
        }

        return SortByDistance(std::move(hits));
    }
}
