#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <set>
#include <vector>

import Kairo.Foundation.Spatial;
import Kairo.Foundation.Math.Vector;
import Kairo.Foundation.Geometry.AABB;
import Kairo.Foundation.Geometry.Frustum;

using namespace kairo::foundation::spatial;

namespace
{
    SpatialAABB Box(
        float minX,
        float minY,
        float minZ,
        float maxX,
        float maxY,
        float maxZ)
    {
        return SpatialAABB::FromMinMax(
            Vec3f{ minX, minY, minZ },
            Vec3f{ maxX, maxY, maxZ });
    }

    std::vector<SpatialPrimitive> SamplePrimitives()
    {
        return
        {
            { 1, Box(-3.0f, -0.5f, -0.5f, -2.0f, 0.5f, 0.5f), 1u },
            { 2, Box(-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f), 1u },
            { 3, Box(2.0f, -0.5f, -0.5f, 3.0f, 0.5f, 0.5f), 1u },
            { 4, Box(0.0f, 2.0f, 0.0f, 1.0f, 3.0f, 1.0f), 1u },
            { 5, Box(4.0f, 4.0f, 4.0f, 5.0f, 5.0f, 5.0f), 1u }
        };
    }

    std::vector<SpatialID> Sorted(
        std::vector<SpatialID> values)
    {
        std::sort(values.begin(), values.end());
        return values;
    }

    std::vector<SpatialID> BruteAABB(
        const std::vector<SpatialPrimitive>& primitives,
        const SpatialAABB& query)
    {
        std::vector<SpatialID> result;

        for (const SpatialPrimitive& primitive : primitives)
        {
            if (Intersects(primitive.Bounds, query))
            {
                result.push_back(primitive.ID);
            }
        }

        return Sorted(result);
    }

    std::vector<SpatialID> BruteSphere(
        const std::vector<SpatialPrimitive>& primitives,
        const Vec3f& center,
        float radius)
    {
        std::vector<SpatialID> result;

        for (const SpatialPrimitive& primitive : primitives)
        {
            if (OverlapsSphere(primitive.Bounds, center, radius))
            {
                result.push_back(primitive.ID);
            }
        }

        return Sorted(result);
    }
}

TEST_CASE("BVH builds correct root bounds", "[Spatial][BVH]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    BVHBuildSettings settings;
    settings.MaxLeafSize = 2;

    const BVH bvh =
        BuildBVH(primitives, settings);

    SpatialAABB brute = SpatialAABB::Empty();
    for (const SpatialPrimitive& primitive : primitives)
    {
        brute.ExpandToInclude(primitive.Bounds);
    }

    REQUIRE(!bvh.Empty());
    REQUIRE(bvh.IsValid());
    REQUIRE(bvh.Validate().Valid);
    REQUIRE(bvh.Root().Bounds == brute);
    REQUIRE(bvh.Stats.PrimitiveCount == primitives.size());
}

TEST_CASE("BVH handles empty and single primitive cases", "[Spatial][BVH]")
{
    const BVH empty =
        BuildBVH({});

    REQUIRE(empty.Empty());
    REQUIRE(empty.IsValid());
    REQUIRE(QueryAABB(empty, Box(-1, -1, -1, 1, 1, 1)).IDs.empty());

    const BVH single =
        BuildBVH({ { 42, Box(0, 0, 0, 1, 1, 1), 2u } });

    REQUIRE(!single.Empty());
    REQUIRE(single.IsValid());
    REQUIRE(single.Root().IsLeaf());
    REQUIRE(QueryAABB(single, Box(-1, -1, -1, 2, 2, 2), 2u).IDs == std::vector<SpatialID>{ 42 });
    REQUIRE(QueryAABB(single, Box(-1, -1, -1, 2, 2, 2), 4u).IDs.empty());
}

TEST_CASE("BVH leaf primitive count <= MaxLeafSize", "[Spatial][BVH]")
{
    BVHBuildSettings settings;
    settings.MaxLeafSize = 2;

    const BVH bvh =
        BuildBVH(SamplePrimitives(), settings);

    for (const BVHNode& node : bvh.Nodes)
    {
        if (node.IsLeaf())
        {
            REQUIRE(node.PrimitiveCount <= settings.MaxLeafSize);
        }
    }
}

TEST_CASE("BVH supports Morton-ordered build path", "[Spatial][BVH]")
{
    BVHBuildSettings settings;
    settings.MaxLeafSize = 2;
    settings.UseMortonOrdering = true;

    const BVH bvh =
        BuildBVH(SamplePrimitives(), settings);

    REQUIRE(!bvh.Empty());
    REQUIRE(bvh.Stats.PrimitiveCount == SamplePrimitives().size());
    REQUIRE(bvh.Stats.LeafCount > 0);
}

TEST_CASE("BVH raycast returns nearest primitive", "[Spatial][BVH]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    const BVH bvh = BuildBVH(primitives, { 2, 64, true, false });

    const SpatialRay ray =
        SpatialRay::FromOriginDirection(
            Vec3f{ -10.0f, 0.0f, 0.0f },
            Vec3f::UnitX());

    auto hitBox =
        [&](SpatialIndex primitiveIndex, const SpatialRay& queryRay) -> std::optional<SpatialRayHit>
        {
            float distance = 0.0f;
            if (!RayIntersectsAABB(queryRay, primitives[primitiveIndex].Bounds, 100.0f, distance))
            {
                return std::nullopt;
            }

            return SpatialRayHit
            {
                primitives[primitiveIndex].ID,
                primitiveIndex,
                distance,
                queryRay.GetPoint(distance),
                Vec3f::Zero()
            };
        };

    const SpatialRaycastResult result =
        Raycast(bvh, ray, hitBox, 100.0f);

    REQUIRE(result.Hit);
    REQUIRE(!result.Hits.empty());
    REQUIRE(result.Closest.ID == 1);
    REQUIRE(std::abs(result.Closest.Distance - 7.0f) < 1.0e-5f);
}

TEST_CASE("BVH raycast-any early exits", "[Spatial][BVH]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    const BVH bvh = BuildBVH(primitives);
    std::uint32_t callbackCount = 0;

    const bool hit =
        RaycastAny(
            bvh,
            SpatialRay::FromOriginDirection(Vec3f{ -10.0f, 0.0f, 0.0f }, Vec3f::UnitX()),
            [&](SpatialIndex primitiveIndex, const SpatialRay& ray)
            {
                ++callbackCount;
                float distance = 0.0f;
                return RayIntersectsAABB(ray, primitives[primitiveIndex].Bounds, 100.0f, distance);
            },
            100.0f);

    REQUIRE(hit);
    REQUIRE(callbackCount < primitives.size());
}

TEST_CASE("BVH AABB query returns correct overlaps", "[Spatial][BVH]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    const SpatialAABB query = Box(-1.0f, -1.0f, -1.0f, 2.25f, 1.0f, 1.0f);
    const BVH bvh = BuildBVH(primitives);

    REQUIRE(Sorted(QueryAABB(bvh, query).IDs) == BruteAABB(primitives, query));
}

TEST_CASE("BVH sphere query returns correct overlaps", "[Spatial][BVH]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    const Vec3f center{ 0.0f, 0.0f, 0.0f };
    const float radius = 2.1f;
    const BVH bvh = BuildBVH(primitives);

    REQUIRE(Sorted(QuerySphere(bvh, center, radius).IDs) == BruteSphere(primitives, center, radius));
}

TEST_CASE("BVH frustum query rejects outside boxes", "[Spatial][BVH]")
{
    const std::vector<SpatialPrimitive> primitives =
    {
        { 1, Box(-0.25f, -0.25f, 0.25f, 0.25f, 0.25f, 0.5f), 1u },
        { 2, Box(50.0f, 50.0f, 50.0f, 51.0f, 51.0f, 51.0f), 1u }
    };

    const BVH bvh = BuildBVH(primitives);
    const Frustumf frustum;

    REQUIRE(Sorted(QueryFrustum(bvh, frustum).IDs) == std::vector<SpatialID>{ 1 });
}

TEST_CASE("DynamicAABBTree insert/remove/update works", "[Spatial][DynamicAABBTree]")
{
    DynamicAABBTree tree(0.01f);
    const SpatialIndex proxy = tree.Insert(10, Box(0, 0, 0, 1, 1, 1));

    REQUIRE(Sorted(tree.QueryAABB(Box(-1, -1, -1, 2, 2, 2)).IDs) == std::vector<SpatialID>{ 10 });
    REQUIRE(tree.Update(proxy, Box(5, 5, 5, 6, 6, 6)));
    REQUIRE(tree.QueryAABB(Box(-1, -1, -1, 2, 2, 2)).IDs.empty());
    REQUIRE(Sorted(tree.QueryAABB(Box(4, 4, 4, 7, 7, 7)).IDs) == std::vector<SpatialID>{ 10 });

    const SpatialRaycastResult raycast =
        tree.Raycast(
            SpatialRay::FromOriginDirection(
                Vec3f{ 0.0f, 5.5f, 5.5f },
                Vec3f::UnitX()),
            100.0f);

    REQUIRE(raycast.Hit);
    REQUIRE(raycast.Closest.ID == 10);

    REQUIRE_FALSE(tree.Update(proxy, Box(5.001f, 5.001f, 5.001f, 5.999f, 5.999f, 5.999f)));
    REQUIRE(Sorted(tree.QueryAABB(Box(4, 4, 4, 7, 7, 7)).IDs) == std::vector<SpatialID>{ 10 });

    tree.Remove(proxy);
    REQUIRE(tree.QueryAABB(Box(4, 4, 4, 7, 7, 7)).IDs.empty());
    REQUIRE(tree.Validate().Valid);
}

TEST_CASE("DynamicAABBTree supports layer masks and rebuild without double fattening", "[Spatial][DynamicAABBTree]")
{
    DynamicAABBTree tree(0.5f);
    const SpatialIndex a = tree.Insert(1, Box(0, 0, 0, 1, 1, 1), 0x1u);
    const SpatialIndex b = tree.Insert(2, Box(0.25f, 0, 0, 1.25f, 1, 1), 0x2u);

    REQUIRE(Sorted(tree.QueryAABB(Box(-1, -1, -1, 2, 2, 2), 0x1u).IDs) == std::vector<SpatialID>{ 1 });
    REQUIRE(Sorted(tree.QueryAABB(Box(-1, -1, -1, 2, 2, 2), 0x2u).IDs) == std::vector<SpatialID>{ 2 });
    REQUIRE(tree.ComputePairs(0x1u).empty());
    REQUIRE(tree.ComputePairs().size() == 1);

    const float before =
        tree.Nodes()[a].FatBounds.Width();

    tree.RebuildBottomUp();
    tree.RebuildBottomUp();

    const SpatialOverlapResult after =
        tree.QueryAABB(Box(-1, -1, -1, 2, 2, 2));

    REQUIRE(Sorted(after.IDs) == std::vector<SpatialID>{ 1, 2 });
    REQUIRE(tree.Validate().Valid);
    REQUIRE(tree.Stats().LeafCount == 2);

    const auto& nodes = tree.Nodes();
    for (const DynamicAABBTreeNode& node : nodes)
    {
        if (node.Allocated && node.IsLeaf() && node.UserID == 1)
        {
            REQUIRE(std::abs(node.FatBounds.Width() - before) < 1.0e-5f);
        }
    }

    (void)b;
}

TEST_CASE("DynamicAABBTree supplies deterministic sphere and ray candidates", "[Spatial][DynamicAABBTree]")
{
    DynamicAABBTree tree(0.0f);
    [[maybe_unused]] const SpatialIndex left =
        tree.Insert(30u, Box(-3.0f, -0.5f, -0.5f, -2.0f, 0.5f, 0.5f), 0x1u);
    [[maybe_unused]] const SpatialIndex center =
        tree.Insert(10u, Box(-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f), 0x1u);
    [[maybe_unused]] const SpatialIndex right =
        tree.Insert(20u, Box(2.0f, -0.5f, -0.5f, 3.0f, 0.5f, 0.5f), 0x2u);

    const SpatialOverlapResult sphere =
        tree.QuerySphere(Vec3f::Zero(), 1.0f);
    REQUIRE(sphere.IDs == std::vector<SpatialID>{ 10u });
    REQUIRE(sphere.TestedPrimitives == 1u);

    const SpatialOverlapResult ray =
        tree.QueryRay(
            SpatialRay::FromOriginDirection(Vec3f{ -4.0f, 0.0f, 0.0f }, Vec3f::UnitX()),
            8.0f);
    REQUIRE(ray.IDs == std::vector<SpatialID>{ 10u, 20u, 30u });
    REQUIRE(ray.TestedPrimitives == 3u);

    const SpatialOverlapResult filtered =
        tree.QueryRay(
            SpatialRay::FromOriginDirection(Vec3f{ -4.0f, 0.0f, 0.0f }, Vec3f::UnitX()),
            8.0f,
            0x2u);
    REQUIRE(filtered.IDs == std::vector<SpatialID>{ 20u });
}

TEST_CASE("DynamicAABBTree returns same pairs as brute force", "[Spatial][DynamicAABBTree]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    DynamicAABBTree tree(0.0f);

    for (const SpatialPrimitive& primitive : primitives)
    {
        const SpatialIndex proxy =
            tree.Insert(primitive.ID, primitive.Bounds);

        (void)proxy;
    }

    std::vector<SpatialPair> brute;
    for (std::size_t i = 0; i < primitives.size(); ++i)
    {
        for (std::size_t j = i + 1; j < primitives.size(); ++j)
        {
            if (Intersects(primitives[i].Bounds, primitives[j].Bounds))
            {
                brute.push_back(OrderedPair(primitives[i].ID, primitives[j].ID));
            }
        }
    }

    REQUIRE(tree.ComputePairs() == brute);
}

TEST_CASE("SweepAndPrune returns same pairs as brute force", "[Spatial][SweepAndPrune]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    SweepAndPrune sap;

    for (const SpatialPrimitive& primitive : primitives)
    {
        sap.Insert(primitive.ID, primitive.Bounds);
    }

    REQUIRE(sap.ComputePairs().empty());

    sap.UpdateProxy(3, Box(0.25f, -0.25f, -0.25f, 0.75f, 0.25f, 0.25f));
    REQUIRE(sap.ComputePairs() == std::vector<SpatialPair>{ OrderedPair(2, 3) });
}

TEST_CASE("SpatialHash radius query matches brute force", "[Spatial][SpatialHash]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    SpatialHash hash(1.0f);

    for (const SpatialPrimitive& primitive : primitives)
    {
        hash.Insert(primitive.ID, primitive.Bounds);
    }

    REQUIRE(Sorted(hash.QueryRadius(Vec3f{ 0, 0, 0 }, 2.1f)) ==
        BruteSphere(primitives, Vec3f{ 0, 0, 0 }, 2.1f));
}

TEST_CASE("SpatialHash validates bounds and gives deterministic negative-coordinate queries", "[Spatial][SpatialHash]")
{
    SpatialHash hash(1.0f);
    hash.Insert(3, Box(-2, -2, -2, -1, -1, -1));
    hash.Insert(1, Box(-0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f));
    hash.Insert(2, Box(0.25f, 0.25f, 0.25f, 2.25f, 2.25f, 2.25f));

    REQUIRE(hash.QueryAABB(Box(-3, -3, -3, 3, 3, 3)) == std::vector<SpatialID>{ 1, 2, 3 });
    REQUIRE(hash.QueryRadius(Vec3f::Zero(), -1.0f).empty());
    REQUIRE_FALSE(hash.Remove(999));
    REQUIRE_THROWS_AS(hash.Insert(9, SpatialAABB::Empty()), std::invalid_argument);
    REQUIRE_THROWS_AS(hash.QueryAABB(SpatialAABB::Empty()), std::invalid_argument);
}

TEST_CASE("Octree AABB query matches brute force", "[Spatial][Octree]")
{
    const std::vector<SpatialPrimitive> primitives = SamplePrimitives();
    Octree octree(Box(-10, -10, -10, 10, 10, 10), { 4, 2, 1.5f });

    for (const SpatialPrimitive& primitive : primitives)
    {
        octree.Insert(primitive.ID, primitive.Bounds);
    }

    const SpatialAABB query = Box(-1, -1, -1, 2.5f, 1, 1);
    REQUIRE(Sorted(octree.QueryAABB(query)) == BruteAABB(primitives, query));
}

TEST_CASE("Quadtree 2D query matches brute force", "[Spatial][Quadtree]")
{
    Quadtree quadtree({ Vec2f{ -10.0f, -10.0f }, Vec2f{ 10.0f, 10.0f } });
    quadtree.Insert(1, { Vec2f{ -1.0f, -1.0f }, Vec2f{ 1.0f, 1.0f } });
    quadtree.Insert(2, { Vec2f{ 5.0f, 5.0f }, Vec2f{ 6.0f, 6.0f } });
    quadtree.Insert(3, { Vec2f{ 0.5f, 0.5f }, Vec2f{ 2.0f, 2.0f } });

    REQUIRE(Sorted(quadtree.QueryAABB({ Vec2f{ 0.0f, 0.0f }, Vec2f{ 1.5f, 1.5f } })) ==
        std::vector<SpatialID>{ 1, 3 });
}

TEST_CASE("KDTree nearest matches brute force", "[Spatial][KDTree]")
{
    std::vector<KDPoint> points =
    {
        { 1, Vec3f{ -2.0f, 0.0f, 0.0f } },
        { 2, Vec3f{ 3.0f, 0.0f, 0.0f } },
        { 3, Vec3f{ 0.2f, 0.1f, 0.0f } }
    };

    KDTree tree(points);
    const auto nearest = tree.NearestPoint(Vec3f{ 0.0f, 0.0f, 0.0f });

    REQUIRE(nearest.has_value());
    REQUIRE(nearest->ID == 3);
    REQUIRE(tree.RadiusSearch(Vec3f{ 0.0f, 0.0f, 0.0f }, 0.5f).size() == 1);
    REQUIRE(tree.RadiusSearch(Vec3f::Zero(), -1.0f).empty());
    REQUIRE(tree.KNearest(Vec3f::Zero(), 0).empty());
    REQUIRE(tree.KNearest(Vec3f::Zero(), 99).size() == points.size());
    REQUIRE(tree.KNearest(Vec3f::Zero(), 2)[0].ID == 3);
}

TEST_CASE("KDTree handles empty tree and duplicate positions", "[Spatial][KDTree]")
{
    KDTree empty;
    REQUIRE_FALSE(empty.NearestPoint(Vec3f::Zero()).has_value());
    REQUIRE(empty.KNearest(Vec3f::Zero(), 3).empty());

    KDTree duplicates(
        {
            { 1, Vec3f::Zero() },
            { 2, Vec3f::Zero() },
            { 3, Vec3f{ 10, 0, 0 } }
        });

    REQUIRE(duplicates.RadiusSearch(Vec3f::Zero(), 0.0f).size() == 2);
    REQUIRE(duplicates.KNearest(Vec3f::Zero(), 2).size() == 2);
}

TEST_CASE("AStar finds shortest path on grid graph", "[Spatial][AStar]")
{
    NavigationGraph graph;
    graph.AddNode(0, Vec3f{ 0, 0, 0 });
    graph.AddNode(1, Vec3f{ 1, 0, 0 });
    graph.AddNode(2, Vec3f{ 2, 0, 0 });
    graph.AddNode(3, Vec3f{ 1, 1, 0 });

    graph.AddEdge(0, 1, 1.0f);
    graph.AddEdge(1, 2, 1.0f);
    graph.AddEdge(0, 3, 4.0f);
    graph.AddEdge(3, 2, 4.0f);

    const AStarResult result = FindPathAStar(graph, 0, 2);

    REQUIRE(result.Found);
    REQUIRE(result.Cost == 2.0f);
    REQUIRE(result.Path == std::vector<NavigationNodeID>{ 0, 1, 2 });
}

TEST_CASE("Debug extraction returns expected number of boxes", "[Spatial][Debug]")
{
    const BVH bvh = BuildBVH(SamplePrimitives(), { 2, 64, false, false });
    const std::vector<DebugBox> boxes = ExtractBVHBoxes(bvh);

    REQUIRE(boxes.size() == bvh.Nodes.size());

    DynamicAABBTree tree(0.0f);
    const SpatialIndex firstProxy =
        tree.Insert(1, Box(0, 0, 0, 1, 1, 1));

    const SpatialIndex secondProxy =
        tree.Insert(2, Box(2, 2, 2, 3, 3, 3));

    (void)firstProxy;
    (void)secondProxy;

    REQUIRE(ExtractDynamicTreeBounds(tree).size() == 3);
}
