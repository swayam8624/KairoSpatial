#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <vector>

import Kairo.Foundation.Spatial;
import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;

using namespace kairo::foundation::spatial;
using kairo::foundation::math::Vec3f;

namespace
{
    SpatialAABB StressBox(float x, float z)
    {
        return SpatialAABB::FromMinMax(
            Vec3f{ x, -0.5f, z },
            Vec3f{ x + 0.75f, 0.5f, z + 0.75f });
    }

    std::vector<SpatialID> SortedIds(std::vector<SpatialID> ids)
    {
        std::sort(ids.begin(), ids.end());
        return ids;
    }

    std::vector<SpatialID> BruteQuery(
        const std::vector<SpatialPrimitive>& primitives,
        const SpatialAABB& query)
    {
        std::vector<SpatialID> ids;
        for (const auto& primitive : primitives)
            if (Intersects(primitive.Bounds, query)) ids.push_back(primitive.ID);
        return SortedIds(std::move(ids));
    }
}

TEST_CASE("large deterministic BVH queries match brute force",
    "[Spatial][Scale][BVH]")
{
    constexpr std::size_t side = 64u;
    std::vector<SpatialPrimitive> primitives;
    primitives.reserve(side * side);
    SpatialID id = 1u;
    for (std::size_t z = 0u; z < side; ++z)
        for (std::size_t x = 0u; x < side; ++x)
        {
            const float px = static_cast<float>(x) * 1.25f;
            const float pz = static_cast<float>(z) * 1.25f;
            primitives.push_back({ id++, StressBox(px, pz), 0x1u });
        }

    BVHBuildSettings settings;
    settings.MaxLeafSize = 8u;
    settings.UseMortonOrdering = true;
    const BVH bvh = BuildBVH(primitives, settings);

    REQUIRE(bvh.IsValid());
    REQUIRE(bvh.Stats.PrimitiveCount == primitives.size());

    for (std::size_t sample = 0u; sample < 128u; ++sample)
    {
        const float x = static_cast<float>((sample * 17u) % side) * 1.25f;
        const float z = static_cast<float>((sample * 29u) % side) * 1.25f;
        const SpatialAABB query = SpatialAABB::FromMinMax(
            Vec3f{ x - 2.0f, -1.0f, z - 2.0f },
            Vec3f{ x + 4.0f, 1.0f, z + 4.0f });
        CHECK(SortedIds(QueryAABB(bvh, query).IDs) ==
            BruteQuery(primitives, query));
    }
}

TEST_CASE("dynamic tree remains valid after thousands of inserts and updates",
    "[Spatial][Scale][DynamicTree]")
{
    constexpr std::size_t count = 4096u;
    DynamicAABBTree tree(0.05f);
    std::vector<SpatialIndex> proxies;
    proxies.reserve(count);
    std::vector<SpatialPrimitive> primitives;
    primitives.reserve(count);

    for (std::size_t index = 0u; index < count; ++index)
    {
        const float x = static_cast<float>(index % 128u);
        const float z = static_cast<float>(index / 128u);
        const SpatialAABB bounds = StressBox(x, z);
        const SpatialID id = static_cast<SpatialID>(index + 1u);
        primitives.push_back({ id, bounds, 0x1u });
        proxies.push_back(tree.Insert(id, bounds, 0x1u));
    }

    for (std::size_t index = 0u; index < count; index += 13u)
    {
        const float x = static_cast<float>(index % 128u) + 0.4f;
        const float z = static_cast<float>(index / 128u) + 0.2f;
        primitives[index].Bounds = StressBox(x, z);
        (void)tree.Update(proxies[index], primitives[index].Bounds);
    }

    REQUIRE(tree.Validate().Valid);
    const auto query = SpatialAABB::FromMinMax(
        Vec3f{ 10.0f, -1.0f, 5.0f }, Vec3f{ 40.0f, 1.0f, 20.0f });
    CHECK(SortedIds(tree.QueryAABB(query).IDs) ==
        BruteQuery(primitives, query));
}
