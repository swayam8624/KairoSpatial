module;

#include <algorithm>
#include <array>
#include <cstdint>
#include <cstddef>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

export module Kairo.Foundation.Spatial.Types;

import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Geometry.Ray;
import Kairo.Foundation.Geometry.Sphere;
import Kairo.Foundation.Geometry.Frustum;

export namespace kairo::foundation::spatial
{
    using namespace kairo::foundation::math;
    using namespace kairo::foundation::geometry;

    using SpatialID = std::uint32_t;
    using SpatialIndex = std::uint32_t;
    using SpatialLayerMask = std::uint32_t;

    inline constexpr SpatialIndex SpatialInvalidIndex =
        std::numeric_limits<SpatialIndex>::max();

    inline constexpr SpatialID SpatialInvalidID =
        std::numeric_limits<SpatialID>::max();

    using SpatialAABB = AABBf;
    using SpatialRay = Rayf;

    /// Input: primitive identity, bounds, and query layer information.
    /// Output: normalized value consumed by all spatial acceleration structures.
    /// Task: keep BVH, trees, grids, and broadphase systems on one data contract.
    struct SpatialPrimitive final
    {
        SpatialID ID = SpatialInvalidID;
        SpatialAABB Bounds = SpatialAABB::Empty();
        SpatialLayerMask LayerMask = 1u;
    };

    /// Input: primitive storage index plus derived bounds data.
    /// Output: build-time primitive reference with cached centroid.
    /// Task: avoid repeatedly recomputing centroids during builders and splits.
    struct SpatialReference final
    {
        SpatialIndex PrimitiveIndex = SpatialInvalidIndex;
        SpatialID ID = SpatialInvalidID;
        SpatialAABB Bounds = SpatialAABB::Empty();
        Vec3f Centroid = Vec3f::Zero();
        SpatialLayerMask LayerMask = 1u;
    };

    struct SpatialPair final
    {
        SpatialID A = SpatialInvalidID;
        SpatialID B = SpatialInvalidID;

        [[nodiscard]]
        friend constexpr bool operator==(
            const SpatialPair& lhs,
            const SpatialPair& rhs) noexcept = default;
    };

    struct SpatialRayHit final
    {
        SpatialID ID = SpatialInvalidID;
        SpatialIndex PrimitiveIndex = SpatialInvalidIndex;
        float Distance = std::numeric_limits<float>::infinity();
        Vec3f Position = Vec3f::Zero();
        Vec3f Normal = Vec3f::Zero();
    };

    struct SpatialRaycastResult final
    {
        bool Hit = false;
        SpatialRayHit Closest;
        std::vector<SpatialRayHit> Hits;
        std::uint32_t VisitedNodes = 0;
        std::uint32_t TestedPrimitives = 0;
    };

    struct SpatialOverlapResult final
    {
        std::vector<SpatialID> IDs;
        std::vector<SpatialIndex> PrimitiveIndices;
        std::uint32_t VisitedNodes = 0;
        std::uint32_t TestedPrimitives = 0;
    };

    enum class SpatialBuildMethod : std::uint8_t
    {
        Median,
        SAH
    };

    struct SpatialBuildSettings final
    {
        std::uint32_t MaxLeafSize = 4;
        std::uint32_t MaxDepth = 64;
        SpatialBuildMethod Method = SpatialBuildMethod::Median;
        bool StableSort = true;
    };

    struct SpatialBuildStats final
    {
        std::uint32_t NodeCount = 0;
        std::uint32_t LeafCount = 0;
        std::uint32_t MaxDepth = 0;
        std::uint32_t PrimitiveCount = 0;
        double BuildMilliseconds = 0.0;
    };

    struct SpatialUpdateStats final
    {
        std::uint32_t Inserted = 0;
        std::uint32_t Removed = 0;
        std::uint32_t Updated = 0;
        std::uint32_t Reinserted = 0;
    };

    struct SpatialValidationResult final
    {
        bool Valid = true;
        std::string Message;
    };

    struct GridCoord2 final
    {
        int x = 0;
        int y = 0;

        [[nodiscard]]
        friend constexpr bool operator==(
            const GridCoord2& lhs,
            const GridCoord2& rhs) noexcept = default;
    };

    struct GridCoord3 final
    {
        int x = 0;
        int y = 0;
        int z = 0;

        [[nodiscard]]
        friend constexpr bool operator==(
            const GridCoord3& lhs,
            const GridCoord3& rhs) noexcept = default;
    };

    [[nodiscard]]
    inline constexpr bool IsValidIndex(
        SpatialIndex index) noexcept
    {
        return index != SpatialInvalidIndex;
    }

    [[nodiscard]]
    inline constexpr SpatialPair OrderedPair(
        SpatialID lhs,
        SpatialID rhs) noexcept
    {
        return lhs < rhs
            ? SpatialPair{ lhs, rhs }
            : SpatialPair{ rhs, lhs };
    }

    [[nodiscard]]
    inline SpatialAABB Union(
        const SpatialAABB& lhs,
        const SpatialAABB& rhs) noexcept
    {
        SpatialAABB result = lhs;
        result.ExpandToInclude(rhs);
        return result;
    }

    [[nodiscard]]
    inline SpatialAABB BoundsOf(
        const std::vector<SpatialReference>& references,
        std::size_t begin,
        std::size_t end)
    {
        if (begin > end || end > references.size())
        {
            throw std::out_of_range("BoundsOf failed: invalid reference range.");
        }

        SpatialAABB bounds = SpatialAABB::Empty();

        for (std::size_t i = begin; i < end; ++i)
        {
            bounds.ExpandToInclude(references[i].Bounds);
        }

        return bounds;
    }

    [[nodiscard]]
    inline SpatialAABB CentroidBoundsOf(
        const std::vector<SpatialReference>& references,
        std::size_t begin,
        std::size_t end)
    {
        if (begin > end || end > references.size())
        {
            throw std::out_of_range("CentroidBoundsOf failed: invalid reference range.");
        }

        SpatialAABB bounds = SpatialAABB::Empty();

        for (std::size_t i = begin; i < end; ++i)
        {
            bounds.ExpandToInclude(references[i].Centroid);
        }

        return bounds;
    }

    [[nodiscard]]
    inline constexpr std::uint32_t LargestAxis(
        const Vec3f& size) noexcept
    {
        if (size.x >= size.y && size.x >= size.z)
        {
            return 0;
        }

        return size.y >= size.z ? 1u : 2u;
    }

    [[nodiscard]]
    inline constexpr float AxisValue(
        const Vec3f& value,
        std::uint32_t axis) noexcept
    {
        return axis == 0
            ? value.x
            : (axis == 1 ? value.y : value.z);
    }

    [[nodiscard]]
    inline bool OverlapsSphere(
        const SpatialAABB& bounds,
        const Vec3f& center,
        float radius) noexcept
    {
        return bounds.DistanceSquaredToPoint(center) <= radius * radius;
    }

    [[nodiscard]]
    inline bool RayIntersectsAABB(
        const SpatialRay& ray,
        const SpatialAABB& bounds,
        float maxDistance,
        float& distance) noexcept
    {
        if (!bounds.IsValid())
        {
            return false;
        }

        float tMin = 0.0f;
        float tMax = maxDistance;

        auto testAxis =
            [&](
                float origin,
                float direction,
                float minValue,
                float maxValue) noexcept -> bool
            {
                constexpr float epsilon = 1.0e-8f;

                if (std::abs(direction) <= epsilon)
                {
                    return origin >= minValue && origin <= maxValue;
                }

                const float inverse = 1.0f / direction;
                float nearT = (minValue - origin) * inverse;
                float farT = (maxValue - origin) * inverse;

                if (nearT > farT)
                {
                    std::swap(nearT, farT);
                }

                tMin = std::max(tMin, nearT);
                tMax = std::min(tMax, farT);
                return tMin <= tMax;
            };

        if (!testAxis(ray.Origin.x, ray.Direction.x, bounds.Min.x, bounds.Max.x) ||
            !testAxis(ray.Origin.y, ray.Direction.y, bounds.Min.y, bounds.Max.y) ||
            !testAxis(ray.Origin.z, ray.Direction.z, bounds.Min.z, bounds.Max.z))
        {
            return false;
        }

        distance = tMin;
        return tMax >= 0.0f;
    }

    [[nodiscard]]
    inline std::vector<SpatialReference> MakeSpatialReferences(
        const std::vector<SpatialPrimitive>& primitives)
    {
        std::vector<SpatialReference> references;
        references.reserve(primitives.size());

        for (std::size_t i = 0; i < primitives.size(); ++i)
        {
            const SpatialPrimitive& primitive = primitives[i];
            if (!primitive.Bounds.IsValid())
            {
                throw std::invalid_argument("MakeSpatialReferences failed: primitive has invalid bounds.");
            }

            references.push_back(
                SpatialReference
                {
                    static_cast<SpatialIndex>(i),
                    primitive.ID,
                    primitive.Bounds,
                    primitive.Bounds.Center(),
                    primitive.LayerMask
                });
        }

        return references;
    }
}
