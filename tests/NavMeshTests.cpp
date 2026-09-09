#include <catch2/catch_test_macros.hpp>

#include <stdexcept>
#include <vector>

import Kairo.Foundation.Spatial;
import Kairo.Foundation.Math.Vector;

using namespace kairo::foundation::math;
using namespace kairo::foundation::spatial;

namespace
{
    [[nodiscard]] std::vector<Vec3f> Square(
        float minX, float minZ, float maxX, float maxZ,
        float y = 0.0f)
    {
        return {
            { minX, y, minZ },
            { minX, y, maxZ },
            { maxX, y, maxZ },
            { maxX, y, minZ }
        };
    }

    [[nodiscard]] NavMesh LinearMesh()
    {
        NavMesh mesh;
        mesh.AddPolygon(10u, Square(0.0f, 0.0f, 2.0f, 2.0f));
        mesh.AddPolygon(20u, Square(2.0f, 0.0f, 4.0f, 2.0f));
        mesh.AddPolygon(30u, Square(4.0f, 0.0f, 6.0f, 2.0f));
        mesh.BuildAdjacency();
        return mesh;
    }
}

TEST_CASE("NavMesh builds deterministic portals across welded shared edges")
{
    const NavMesh mesh = LinearMesh();
    const auto& first = mesh.Neighbors(10u);
    REQUIRE(first.size() == 1u);
    CHECK(first.front().From == 10u);
    CHECK(first.front().To == 20u);
    CHECK(first.front().Left.x == 2.0f);
    CHECK(first.front().Right.x == 2.0f);

    const auto& middle = mesh.Neighbors(20u);
    REQUIRE(middle.size() == 2u);
    CHECK(middle[0].To == 10u);
    CHECK(middle[1].To == 30u);
}

TEST_CASE("NavMesh containment and nearest queries resolve authored polygons")
{
    const NavMesh mesh = LinearMesh();

    const auto containing = mesh.FindContainingPolygon({ 3.0f, 0.2f, 1.0f }, 0.25f);
    REQUIRE(containing.has_value());
    CHECK(*containing == 20u);

    const auto tooHigh = mesh.FindContainingPolygon({ 3.0f, 2.0f, 1.0f }, 0.25f);
    CHECK_FALSE(tooHigh.has_value());

    const auto nearest = mesh.FindNearestPolygon({ 6.4f, 0.0f, 1.0f }, 0.5f);
    REQUIRE(nearest.has_value());
    CHECK(*nearest == 30u);

    CHECK_FALSE(mesh.FindNearestPolygon({ 20.0f, 0.0f, 20.0f }, 1.0f).has_value());
}

TEST_CASE("NavMesh A star returns polygon corridor and funnel collapses straight portals")
{
    const NavMesh mesh = LinearMesh();
    const Vec3f start{ 0.5f, 0.0f, 1.0f };
    const Vec3f end{ 5.5f, 0.0f, 1.0f };
    const NavMeshPath path = mesh.FindPath(start, end);

    REQUIRE(path.Reached);
    REQUIRE(path.Corridor.size() == 3u);
    CHECK(path.Corridor[0] == 10u);
    CHECK(path.Corridor[1] == 20u);
    CHECK(path.Corridor[2] == 30u);
    REQUIRE(path.Waypoints.size() == 2u);
    CHECK(path.Waypoints.front() == start);
    CHECK(path.Waypoints.back() == end);
    CHECK(path.Cost > 0.0f);
}

TEST_CASE("NavMesh same-polygon path bypasses graph traversal")
{
    const NavMesh mesh = LinearMesh();
    const Vec3f start{ 0.25f, 0.0f, 0.25f };
    const Vec3f end{ 1.75f, 0.0f, 1.75f };
    const auto path = mesh.FindPath(start, end);

    REQUIRE(path.Reached);
    REQUIRE(path.Corridor.size() == 1u);
    CHECK(path.Corridor.front() == 10u);
    REQUIRE(path.Waypoints.size() == 2u);
    CHECK(path.Waypoints.front() == start);
    CHECK(path.Waypoints.back() == end);
}

TEST_CASE("NavMesh reports disconnected destinations without fabricating a route")
{
    NavMesh mesh;
    mesh.AddPolygon(1u, Square(0.0f, 0.0f, 2.0f, 2.0f));
    mesh.AddPolygon(2u, Square(10.0f, 0.0f, 12.0f, 2.0f));
    mesh.BuildAdjacency();

    const auto path = mesh.FindPath(
        { 0.5f, 0.0f, 0.5f },
        { 10.5f, 0.0f, 0.5f });
    CHECK_FALSE(path.Reached);
    CHECK(path.Corridor.empty());
    CHECK(path.Waypoints.empty());
}

TEST_CASE("NavMesh area costs influence A star route selection")
{
    NavMesh mesh;
    // Two equal-length two-hop routes connect left to right. The upper route
    // has an expensive middle polygon and must lose to the lower route.
    mesh.AddPolygon(1u, Square(0.0f, 0.0f, 2.0f, 4.0f));
    mesh.AddPolygon(2u, Square(2.0f, 2.0f, 4.0f, 4.0f), 25.0f);
    mesh.AddPolygon(3u, Square(2.0f, 0.0f, 4.0f, 2.0f), 1.0f);
    mesh.AddPolygon(4u, Square(4.0f, 0.0f, 6.0f, 4.0f));
    mesh.BuildAdjacency();

    const auto path = mesh.FindPath(
        { 1.0f, 0.0f, 2.0f },
        { 5.0f, 0.0f, 2.0f });
    REQUIRE(path.Reached);
    REQUIRE(path.Corridor.size() == 3u);
    CHECK(path.Corridor[0] == 1u);
    CHECK(path.Corridor[1] == 3u);
    CHECK(path.Corridor[2] == 4u);
}

TEST_CASE("NavMesh rejects degenerate and concave authored polygons")
{
    NavMesh mesh;
    CHECK_THROWS_AS(
        mesh.AddPolygon(1u, {
            { 0.0f, 0.0f, 0.0f },
            { 1.0f, 0.0f, 0.0f },
            { 2.0f, 0.0f, 0.0f }
        }),
        std::invalid_argument);

    CHECK_THROWS_AS(
        mesh.AddPolygon(2u, {
            { 0.0f, 0.0f, 0.0f },
            { 0.0f, 0.0f, 2.0f },
            { 1.0f, 0.0f, 1.0f },
            { 2.0f, 0.0f, 2.0f },
            { 2.0f, 0.0f, 0.0f }
        }),
        std::invalid_argument);
}

TEST_CASE("NavMesh requires adjacency build before graph queries")
{
    NavMesh mesh;
    mesh.AddPolygon(1u, Square(0.0f, 0.0f, 2.0f, 2.0f));
    CHECK_THROWS_AS(mesh.Neighbors(1u), std::logic_error);
    CHECK_THROWS_AS(
        mesh.FindPath({ 0.5f, 0.0f, 0.5f }, { 1.5f, 0.0f, 1.5f }),
        std::logic_error);
}
