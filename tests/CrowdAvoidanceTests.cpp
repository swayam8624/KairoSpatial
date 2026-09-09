#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

import Kairo.Foundation.Spatial;
import Kairo.Foundation.Math.Vector;

using namespace kairo::foundation::math;
using namespace kairo::foundation::spatial;

namespace
{
    [[nodiscard]] float DotXZ(const Vec3f& a, const Vec3f& b) noexcept
    {
        return a.x * b.x + a.z * b.z;
    }

    [[nodiscard]] float LengthSquaredXZ(const Vec3f& value) noexcept
    {
        return DotXZ(value, value);
    }

    [[nodiscard]] bool CollisionWithin(
        const Vec3f& positionA,
        const Vec3f& velocityA,
        float radiusA,
        const Vec3f& positionB,
        const Vec3f& velocityB,
        float radiusB,
        float horizon) noexcept
    {
        const Vec3f relativePosition = positionB - positionA;
        const Vec3f relativeVelocity = velocityB - velocityA;
        const float speedSquared = LengthSquaredXZ(relativeVelocity);
        float t = 0.0f;
        if (speedSquared > 1.0e-8f)
        {
            t = std::clamp(
                -DotXZ(relativePosition, relativeVelocity) / speedSquared,
                0.0f,
                horizon);
        }
        const Vec3f separation = relativePosition + relativeVelocity * t;
        const float combinedRadius = radiusA + radiusB;
        return LengthSquaredXZ(separation) <
            combinedRadius * combinedRadius - 1.0e-4f;
    }

    [[nodiscard]] const CrowdVelocityCommand& ByID(
        const std::vector<CrowdVelocityCommand>& commands,
        CrowdAgentID id)
    {
        const auto found = std::find_if(commands.begin(), commands.end(),
            [&](const CrowdVelocityCommand& command)
            {
                return command.ID == id;
            });
        if (found == commands.end())
            throw std::runtime_error("Crowd command id missing from test result.");
        return *found;
    }
}

TEST_CASE("Crowd avoidance preserves preferred motion when no constraints exist")
{
    const std::vector<CrowdAgent> agents{
        {
            .ID = 1u,
            .Position = { 0.0f, 0.0f, 0.0f },
            .Velocity = Vec3f::Zero(),
            .PreferredVelocity = { 3.0f, 0.0f, 4.0f },
            .Radius = 0.35f,
            .MaxSpeed = 2.0f,
            .NeighborDistance = 4.0f,
            .TimeHorizon = 2.5f
        }
    };

    const auto commands = CrowdAvoidanceSolver::Solve(agents);
    REQUIRE(commands.size() == 1u);
    CHECK(std::abs(commands[0].Velocity.x - 1.2f) < 1.0e-5f);
    CHECK(std::abs(commands[0].Velocity.z - 1.6f) < 1.0e-5f);
    CHECK(commands[0].NeighborCount == 0u);
    CHECK(commands[0].ConstraintCount == 0u);
}

TEST_CASE("Crowd avoidance resolves head-on traffic inside the prediction horizon")
{
    const CrowdAgent left{
        .ID = 10u,
        .Position = { -1.25f, 0.0f, 0.0f },
        .Velocity = { 1.0f, 0.0f, 0.0f },
        .PreferredVelocity = { 1.0f, 0.0f, 0.0f },
        .Radius = 0.4f,
        .MaxSpeed = 1.5f,
        .NeighborDistance = 5.0f,
        .TimeHorizon = 2.5f
    };
    const CrowdAgent right{
        .ID = 20u,
        .Position = { 1.25f, 0.0f, 0.0f },
        .Velocity = { -1.0f, 0.0f, 0.0f },
        .PreferredVelocity = { -1.0f, 0.0f, 0.0f },
        .Radius = 0.4f,
        .MaxSpeed = 1.5f,
        .NeighborDistance = 5.0f,
        .TimeHorizon = 2.5f
    };

    const auto commands = CrowdAvoidanceSolver::Solve({ left, right });
    REQUIRE(commands.size() == 2u);
    const auto& leftCommand = ByID(commands, left.ID);
    const auto& rightCommand = ByID(commands, right.ID);

    CHECK(leftCommand.NeighborCount == 1u);
    CHECK(rightCommand.NeighborCount == 1u);
    CHECK_FALSE(CollisionWithin(
        left.Position, leftCommand.Velocity, left.Radius,
        right.Position, rightCommand.Velocity, right.Radius,
        2.5f));
    CHECK(LengthSquaredXZ(leftCommand.Velocity) <=
        left.MaxSpeed * left.MaxSpeed + 1.0e-5f);
    CHECK(LengthSquaredXZ(rightCommand.Velocity) <=
        right.MaxSpeed * right.MaxSpeed + 1.0e-5f);
}

TEST_CASE("Crowd avoidance pushes overlapping agents apart deterministically")
{
    CrowdAgent a{
        .ID = 1u,
        .Position = { -0.1f, 0.0f, 0.0f },
        .Velocity = Vec3f::Zero(),
        .PreferredVelocity = Vec3f::Zero(),
        .Radius = 0.4f,
        .MaxSpeed = 3.0f,
        .NeighborDistance = 2.0f,
        .TimeHorizon = 2.0f
    };
    CrowdAgent b = a;
    b.ID = 2u;
    b.Position = { 0.1f, 0.0f, 0.0f };

    const auto commands = CrowdAvoidanceSolver::Solve(
        { a, b }, {}, { .TimeStep = 0.1f });
    const Vec3f relativePosition = b.Position - a.Position;
    const Vec3f relativeVelocity =
        ByID(commands, b.ID).Velocity - ByID(commands, a.ID).Velocity;

    CHECK(DotXZ(relativePosition, relativeVelocity) > 0.0f);
}

TEST_CASE("Crowd avoidance treats circular runtime blockers as non-reciprocal constraints")
{
    const CrowdAgent agent{
        .ID = 7u,
        .Position = { -1.5f, 0.0f, 0.0f },
        .Velocity = { 1.0f, 0.0f, 0.0f },
        .PreferredVelocity = { 1.0f, 0.0f, 0.0f },
        .Radius = 0.35f,
        .MaxSpeed = 1.5f,
        .NeighborDistance = 5.0f,
        .TimeHorizon = 3.0f
    };
    const CrowdObstacle obstacle{
        .ID = 100u,
        .Position = Vec3f::Zero(),
        .Radius = 0.5f
    };

    const auto commands = CrowdAvoidanceSolver::Solve({ agent }, { obstacle });
    REQUIRE(commands.size() == 1u);
    CHECK(commands[0].ConstraintCount == 1u);
    CHECK_FALSE(CollisionWithin(
        agent.Position, commands[0].Velocity, agent.Radius,
        obstacle.Position, Vec3f::Zero(), obstacle.Radius,
        agent.TimeHorizon));
}

TEST_CASE("Crowd avoidance results are independent of input agent order")
{
    const CrowdAgent a{
        .ID = 11u,
        .Position = { -1.0f, 0.0f, 0.0f },
        .Velocity = { 0.8f, 0.0f, 0.0f },
        .PreferredVelocity = { 1.0f, 0.0f, 0.2f },
        .Radius = 0.3f,
        .MaxSpeed = 1.4f,
        .NeighborDistance = 4.0f,
        .TimeHorizon = 2.0f
    };
    const CrowdAgent b{
        .ID = 22u,
        .Position = { 1.0f, 0.0f, 0.0f },
        .Velocity = { -0.8f, 0.0f, 0.0f },
        .PreferredVelocity = { -1.0f, 0.0f, -0.1f },
        .Radius = 0.3f,
        .MaxSpeed = 1.4f,
        .NeighborDistance = 4.0f,
        .TimeHorizon = 2.0f
    };
    const CrowdAgent c{
        .ID = 33u,
        .Position = { 0.0f, 0.0f, 1.1f },
        .Velocity = { 0.0f, 0.0f, -0.6f },
        .PreferredVelocity = { 0.0f, 0.0f, -1.0f },
        .Radius = 0.3f,
        .MaxSpeed = 1.4f,
        .NeighborDistance = 4.0f,
        .TimeHorizon = 2.0f
    };

    const auto forward = CrowdAvoidanceSolver::Solve({ a, b, c });
    const auto reversed = CrowdAvoidanceSolver::Solve({ c, b, a });

    for (const CrowdAgentID id : { a.ID, b.ID, c.ID })
    {
        const auto& lhs = ByID(forward, id);
        const auto& rhs = ByID(reversed, id);
        CHECK(std::abs(lhs.Velocity.x - rhs.Velocity.x) < 1.0e-6f);
        CHECK(std::abs(lhs.Velocity.z - rhs.Velocity.z) < 1.0e-6f);
        CHECK(lhs.NeighborCount == rhs.NeighborCount);
        CHECK(lhs.ConstraintCount == rhs.ConstraintCount);
    }
}

TEST_CASE("Crowd avoidance ignores agents beyond the authored neighbor radius")
{
    const CrowdAgent a{
        .ID = 1u,
        .Position = Vec3f::Zero(),
        .Velocity = Vec3f::Zero(),
        .PreferredVelocity = { 1.0f, 0.0f, 0.0f },
        .Radius = 0.3f,
        .MaxSpeed = 1.0f,
        .NeighborDistance = 2.0f,
        .TimeHorizon = 2.0f
    };
    CrowdAgent b = a;
    b.ID = 2u;
    b.Position = { 20.0f, 0.0f, 0.0f };
    b.PreferredVelocity = { -1.0f, 0.0f, 0.0f };

    const auto commands = CrowdAvoidanceSolver::Solve({ a, b });
    CHECK(ByID(commands, a.ID).NeighborCount == 0u);
    CHECK(std::abs(ByID(commands, a.ID).Velocity.x - 1.0f) < 1.0e-6f);
}

TEST_CASE("Crowd avoidance validates stable ids and finite runtime state")
{
    CrowdAgent valid{
        .ID = 1u,
        .Position = Vec3f::Zero(),
        .Velocity = Vec3f::Zero(),
        .PreferredVelocity = Vec3f::Zero()
    };
    CrowdAgent duplicate = valid;
    CHECK_THROWS_AS(
        CrowdAvoidanceSolver::Solve({ valid, duplicate }),
        std::invalid_argument);

    CrowdAgent invalid = valid;
    invalid.ID = 2u;
    invalid.Position.x = std::numeric_limits<float>::quiet_NaN();
    CHECK_THROWS_AS(
        CrowdAvoidanceSolver::Solve({ invalid }),
        std::invalid_argument);

    CHECK_THROWS_AS(
        CrowdAvoidanceSolver::Solve(
            { valid }, {}, { .TimeStep = 0.0f }),
        std::invalid_argument);
}
