module;

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

export module Kairo.Foundation.Spatial.CrowdAvoidance;

import Kairo.Foundation.Math.Vector;

export namespace kairo::foundation::spatial
{
    using kairo::foundation::math::Vec3f;

    using CrowdAgentID = std::uint32_t;
    using CrowdObstacleID = std::uint32_t;

    inline constexpr CrowdAgentID InvalidCrowdAgentID =
        std::numeric_limits<CrowdAgentID>::max();
    inline constexpr CrowdObstacleID InvalidCrowdObstacleID =
        std::numeric_limits<CrowdObstacleID>::max();

    /// Runtime navigation state consumed by the local-avoidance solver.
    ///
    /// The solver operates only in the XZ navigation plane. Vertical motion,
    /// gravity, slopes and step handling remain owned by character/physics
    /// controllers. PreferredVelocity is therefore an intent velocity rather
    /// than an authoritative transform update.
    struct CrowdAgent final
    {
        CrowdAgentID ID = InvalidCrowdAgentID;
        Vec3f Position = Vec3f::Zero();
        Vec3f Velocity = Vec3f::Zero();
        Vec3f PreferredVelocity = Vec3f::Zero();
        float Radius = 0.35f;
        float MaxSpeed = 2.0f;
        float NeighborDistance = 6.0f;
        float TimeHorizon = 2.5f;
    };

    /// Circular static obstacle used by the lightweight runtime avoidance
    /// layer. NavMesh topology remains the authoritative traversability model;
    /// these discs are for dynamic authoring/gameplay blockers that should not
    /// require rebuilding navigation polygons every frame.
    struct CrowdObstacle final
    {
        CrowdObstacleID ID = InvalidCrowdObstacleID;
        Vec3f Position = Vec3f::Zero();
        float Radius = 0.5f;
    };

    struct CrowdAvoidanceSettings final
    {
        /// Simulation interval used only for already-overlapping recovery.
        /// Future collision constraints use each agent's TimeHorizon.
        float TimeStep = 1.0f / 60.0f;
    };

    struct CrowdVelocityCommand final
    {
        CrowdAgentID ID = InvalidCrowdAgentID;
        Vec3f Velocity = Vec3f::Zero();
        std::size_t NeighborCount = 0u;
        std::size_t ConstraintCount = 0u;
    };

    /// Deterministic ORCA-style reciprocal local-avoidance solver.
    ///
    /// Every solve is computed from one immutable input snapshot, so results do
    /// not depend on the caller's agent-array order. Agent/agent constraints
    /// split collision-avoidance responsibility equally; static circular
    /// obstacles assign the full correction to the moving agent. The resulting
    /// velocity is the closest feasible velocity to PreferredVelocity within
    /// MaxSpeed, found by the same incremental half-plane linear programming
    /// formulation used by reciprocal velocity-obstacle solvers.
    class CrowdAvoidanceSolver final
    {
    public:
        [[nodiscard]] static std::vector<CrowdVelocityCommand> Solve(
            const std::vector<CrowdAgent>& agents,
            const std::vector<CrowdObstacle>& obstacles = {},
            CrowdAvoidanceSettings settings = {})
        {
            ValidateSettings(settings);
            ValidateAgents(agents);
            ValidateObstacles(obstacles);

            std::vector<std::size_t> orderedAgents(agents.size());
            for (std::size_t i = 0u; i < agents.size(); ++i)
                orderedAgents[i] = i;
            std::sort(orderedAgents.begin(), orderedAgents.end(),
                [&](std::size_t lhs, std::size_t rhs)
                {
                    return agents[lhs].ID < agents[rhs].ID;
                });

            std::vector<std::size_t> orderedObstacles(obstacles.size());
            for (std::size_t i = 0u; i < obstacles.size(); ++i)
                orderedObstacles[i] = i;
            std::sort(orderedObstacles.begin(), orderedObstacles.end(),
                [&](std::size_t lhs, std::size_t rhs)
                {
                    return obstacles[lhs].ID < obstacles[rhs].ID;
                });

            std::vector<CrowdVelocityCommand> commands(agents.size());
            for (std::size_t inputIndex = 0u; inputIndex < agents.size(); ++inputIndex)
            {
                const CrowdAgent& agent = agents[inputIndex];
                std::vector<Line> constraints;
                constraints.reserve(agents.size() + obstacles.size());
                std::size_t neighborCount = 0u;

                const Vec2 selfPosition = XZ(agent.Position);
                const Vec2 selfVelocity = XZ(agent.Velocity);

                for (const std::size_t otherIndex : orderedAgents)
                {
                    if (otherIndex == inputIndex) continue;
                    const CrowdAgent& other = agents[otherIndex];
                    const Vec2 relativePosition = XZ(other.Position) - selfPosition;
                    const float combinedRadius = agent.Radius + other.Radius;
                    const float searchRadius = std::max(
                        agent.NeighborDistance, combinedRadius);
                    if (LengthSquared(relativePosition) > searchRadius * searchRadius)
                        continue;

                    ++neighborCount;
                    const Vec2 fallbackNormal = agent.ID < other.ID
                        ? Vec2{ 1.0f, 0.0f }
                        : Vec2{ -1.0f, 0.0f };
                    constraints.push_back(BuildConstraint(
                        selfPosition,
                        selfVelocity,
                        agent.Radius,
                        XZ(other.Position),
                        XZ(other.Velocity),
                        other.Radius,
                        agent.TimeHorizon,
                        settings.TimeStep,
                        0.5f,
                        fallbackNormal));
                }

                for (const std::size_t obstacleIndex : orderedObstacles)
                {
                    const CrowdObstacle& obstacle = obstacles[obstacleIndex];
                    const Vec2 relativePosition = XZ(obstacle.Position) - selfPosition;
                    const float combinedRadius = agent.Radius + obstacle.Radius;
                    const float searchRadius = std::max(
                        agent.NeighborDistance, combinedRadius);
                    if (LengthSquared(relativePosition) > searchRadius * searchRadius)
                        continue;

                    Vec2 fallbackNormal = NormalizeOr(
                        selfPosition - XZ(obstacle.Position),
                        { 1.0f, 0.0f });
                    constraints.push_back(BuildConstraint(
                        selfPosition,
                        selfVelocity,
                        agent.Radius,
                        XZ(obstacle.Position),
                        Vec2{},
                        obstacle.Radius,
                        agent.TimeHorizon,
                        settings.TimeStep,
                        1.0f,
                        fallbackNormal));
                }

                Vec2 result{};
                const Vec2 preferred = ClampLength(
                    XZ(agent.PreferredVelocity), agent.MaxSpeed);
                const std::size_t failedLine = LinearProgram2(
                    constraints,
                    agent.MaxSpeed,
                    preferred,
                    false,
                    result);
                if (failedLine < constraints.size())
                    LinearProgram3(
                        constraints, failedLine, agent.MaxSpeed, result);

                commands[inputIndex] = {
                    .ID = agent.ID,
                    .Velocity = { result.x, 0.0f, result.z },
                    .NeighborCount = neighborCount,
                    .ConstraintCount = constraints.size()
                };
            }
            return commands;
        }

    private:
        static constexpr float Epsilon = 1.0e-6f;

        struct Vec2 final
        {
            float x = 0.0f;
            float z = 0.0f;
        };

        struct Line final
        {
            Vec2 Point{};
            Vec2 Direction{};
        };

        [[nodiscard]] static Vec2 XZ(const Vec3f& value) noexcept
        {
            return { value.x, value.z };
        }

        [[nodiscard]] static Vec2 Add(Vec2 a, Vec2 b) noexcept
        {
            return { a.x + b.x, a.z + b.z };
        }

        [[nodiscard]] static Vec2 Subtract(Vec2 a, Vec2 b) noexcept
        {
            return { a.x - b.x, a.z - b.z };
        }

        [[nodiscard]] static Vec2 Multiply(Vec2 value, float scalar) noexcept
        {
            return { value.x * scalar, value.z * scalar };
        }

        friend Vec2 operator+(Vec2 a, Vec2 b) noexcept { return Add(a, b); }
        friend Vec2 operator-(Vec2 a, Vec2 b) noexcept { return Subtract(a, b); }
        friend Vec2 operator-(Vec2 value) noexcept { return { -value.x, -value.z }; }
        friend Vec2 operator*(Vec2 value, float scalar) noexcept { return Multiply(value, scalar); }
        friend Vec2 operator*(float scalar, Vec2 value) noexcept { return Multiply(value, scalar); }
        friend Vec2 operator/(Vec2 value, float scalar) noexcept
        {
            return { value.x / scalar, value.z / scalar };
        }

        [[nodiscard]] static float Dot(Vec2 a, Vec2 b) noexcept
        {
            return a.x * b.x + a.z * b.z;
        }

        [[nodiscard]] static float Det(Vec2 a, Vec2 b) noexcept
        {
            return a.x * b.z - a.z * b.x;
        }

        [[nodiscard]] static float LengthSquared(Vec2 value) noexcept
        {
            return Dot(value, value);
        }

        [[nodiscard]] static float Length(Vec2 value) noexcept
        {
            return std::sqrt(LengthSquared(value));
        }

        [[nodiscard]] static Vec2 NormalizeOr(Vec2 value, Vec2 fallback) noexcept
        {
            const float length = Length(value);
            if (length <= Epsilon) return fallback;
            return value / length;
        }

        [[nodiscard]] static Vec2 ClampLength(Vec2 value, float maximum) noexcept
        {
            if (maximum <= 0.0f) return {};
            const float lengthSquared = LengthSquared(value);
            if (lengthSquared <= maximum * maximum) return value;
            return value * (maximum / std::sqrt(lengthSquared));
        }

        [[nodiscard]] static Line BuildConstraint(
            Vec2 selfPosition,
            Vec2 selfVelocity,
            float selfRadius,
            Vec2 otherPosition,
            Vec2 otherVelocity,
            float otherRadius,
            float timeHorizon,
            float timeStep,
            float responsibility,
            Vec2 fallbackNormal)
        {
            const Vec2 relativePosition = otherPosition - selfPosition;
            const Vec2 relativeVelocity = selfVelocity - otherVelocity;
            const float distanceSquared = LengthSquared(relativePosition);
            const float combinedRadius = selfRadius + otherRadius;
            const float combinedRadiusSquared = combinedRadius * combinedRadius;

            Vec2 direction{};
            Vec2 correction{};

            if (distanceSquared > combinedRadiusSquared)
            {
                const float inverseTimeHorizon = 1.0f / timeHorizon;
                const Vec2 w = relativeVelocity -
                    inverseTimeHorizon * relativePosition;
                const float wLengthSquared = LengthSquared(w);
                const float dotProduct = Dot(w, relativePosition);

                if (dotProduct < 0.0f &&
                    dotProduct * dotProduct >
                        combinedRadiusSquared * wLengthSquared)
                {
                    const float wLength = std::sqrt(wLengthSquared);
                    const Vec2 unitW = NormalizeOr(w, fallbackNormal);
                    direction = { unitW.z, -unitW.x };
                    correction = (combinedRadius * inverseTimeHorizon -
                        wLength) * unitW;
                }
                else
                {
                    const float leg = std::sqrt(std::max(
                        0.0f, distanceSquared - combinedRadiusSquared));
                    if (Det(relativePosition, w) > 0.0f)
                    {
                        direction = {
                            (relativePosition.x * leg -
                                relativePosition.z * combinedRadius) /
                                distanceSquared,
                            (relativePosition.x * combinedRadius +
                                relativePosition.z * leg) /
                                distanceSquared
                        };
                    }
                    else
                    {
                        direction = -Vec2{
                            (relativePosition.x * leg +
                                relativePosition.z * combinedRadius) /
                                distanceSquared,
                            (-relativePosition.x * combinedRadius +
                                relativePosition.z * leg) /
                                distanceSquared
                        };
                    }
                    direction = NormalizeOr(direction,
                        { fallbackNormal.z, -fallbackNormal.x });
                    const float projectedVelocity = Dot(
                        relativeVelocity, direction);
                    correction = projectedVelocity * direction -
                        relativeVelocity;
                }
            }
            else
            {
                const float inverseTimeStep = 1.0f / timeStep;
                const Vec2 w = relativeVelocity -
                    inverseTimeStep * relativePosition;
                const float wLength = Length(w);
                const Vec2 unitW = NormalizeOr(w, fallbackNormal);
                direction = { unitW.z, -unitW.x };
                correction = (combinedRadius * inverseTimeStep -
                    wLength) * unitW;
            }

            return {
                .Point = selfVelocity + responsibility * correction,
                .Direction = NormalizeOr(direction, { 0.0f, 1.0f })
            };
        }

        [[nodiscard]] static bool LinearProgram1(
            const std::vector<Line>& lines,
            std::size_t lineIndex,
            float radius,
            Vec2 preferredVelocity,
            bool directionOptimal,
            Vec2& result)
        {
            const Line& line = lines[lineIndex];
            const float dotProduct = Dot(line.Point, line.Direction);
            const float discriminant = dotProduct * dotProduct +
                radius * radius - LengthSquared(line.Point);
            if (discriminant < 0.0f) return false;

            const float sqrtDiscriminant = std::sqrt(discriminant);
            float tLeft = -dotProduct - sqrtDiscriminant;
            float tRight = -dotProduct + sqrtDiscriminant;

            for (std::size_t i = 0u; i < lineIndex; ++i)
            {
                const float denominator = Det(
                    line.Direction, lines[i].Direction);
                const float numerator = Det(
                    lines[i].Direction, line.Point - lines[i].Point);

                if (std::abs(denominator) <= Epsilon)
                {
                    if (numerator < 0.0f) return false;
                    continue;
                }

                const float t = numerator / denominator;
                if (denominator >= 0.0f)
                    tRight = std::min(tRight, t);
                else
                    tLeft = std::max(tLeft, t);

                if (tLeft > tRight) return false;
            }

            if (directionOptimal)
            {
                result = line.Point +
                    (Dot(preferredVelocity, line.Direction) > 0.0f
                        ? tRight : tLeft) * line.Direction;
            }
            else
            {
                const float t = std::clamp(
                    Dot(line.Direction, preferredVelocity - line.Point),
                    tLeft, tRight);
                result = line.Point + t * line.Direction;
            }
            return true;
        }

        [[nodiscard]] static std::size_t LinearProgram2(
            const std::vector<Line>& lines,
            float radius,
            Vec2 preferredVelocity,
            bool directionOptimal,
            Vec2& result)
        {
            if (directionOptimal)
            {
                result = preferredVelocity * radius;
            }
            else if (LengthSquared(preferredVelocity) > radius * radius)
            {
                result = NormalizeOr(preferredVelocity, {}) * radius;
            }
            else
            {
                result = preferredVelocity;
            }

            for (std::size_t i = 0u; i < lines.size(); ++i)
            {
                if (Det(lines[i].Direction,
                    lines[i].Point - result) > 0.0f)
                {
                    const Vec2 previous = result;
                    if (!LinearProgram1(
                        lines, i, radius, preferredVelocity,
                        directionOptimal, result))
                    {
                        result = previous;
                        return i;
                    }
                }
            }
            return lines.size();
        }

        static void LinearProgram3(
            const std::vector<Line>& lines,
            std::size_t beginLine,
            float radius,
            Vec2& result)
        {
            float distance = 0.0f;
            for (std::size_t i = beginLine; i < lines.size(); ++i)
            {
                const float violation = Det(
                    lines[i].Direction, lines[i].Point - result);
                if (violation <= distance) continue;

                std::vector<Line> projectedLines;
                projectedLines.reserve(i);
                for (std::size_t j = 0u; j < i; ++j)
                {
                    Line projected{};
                    const float determinant = Det(
                        lines[i].Direction, lines[j].Direction);
                    if (std::abs(determinant) <= Epsilon)
                    {
                        if (Dot(lines[i].Direction,
                            lines[j].Direction) > 0.0f)
                            continue;
                        projected.Point = 0.5f *
                            (lines[i].Point + lines[j].Point);
                    }
                    else
                    {
                        projected.Point = lines[i].Point +
                            (Det(lines[j].Direction,
                                lines[i].Point - lines[j].Point) /
                                determinant) * lines[i].Direction;
                    }
                    projected.Direction = NormalizeOr(
                        lines[j].Direction - lines[i].Direction,
                        { -lines[i].Direction.z,
                            lines[i].Direction.x });
                    projectedLines.push_back(projected);
                }

                const Vec2 previous = result;
                const Vec2 directionOptimal{
                    -lines[i].Direction.z,
                    lines[i].Direction.x
                };
                if (LinearProgram2(
                    projectedLines,
                    radius,
                    directionOptimal,
                    true,
                    result) < projectedLines.size())
                {
                    // Floating-point degeneracy can make the projected LP look
                    // infeasible even though the previous solution was valid for
                    // all earlier constraints. Preserve the earlier solution.
                    result = previous;
                }
                distance = Det(
                    lines[i].Direction, lines[i].Point - result);
            }
        }

        static void RequireFinite(const Vec3f& value, const char* label)
        {
            if (!std::isfinite(value.x) || !std::isfinite(value.y) ||
                !std::isfinite(value.z))
                throw std::invalid_argument(std::string(label) +
                    " must be finite.");
        }

        static void ValidateSettings(CrowdAvoidanceSettings settings)
        {
            if (!std::isfinite(settings.TimeStep) || settings.TimeStep <= 0.0f)
                throw std::invalid_argument(
                    "Crowd avoidance time step must be finite and positive.");
        }

        static void ValidateAgents(const std::vector<CrowdAgent>& agents)
        {
            std::unordered_set<CrowdAgentID> ids;
            ids.reserve(agents.size());
            for (const CrowdAgent& agent : agents)
            {
                if (agent.ID == InvalidCrowdAgentID)
                    throw std::invalid_argument(
                        "Crowd agent id is invalid.");
                if (!ids.insert(agent.ID).second)
                    throw std::invalid_argument(
                        "Crowd agent ids must be unique.");
                RequireFinite(agent.Position, "Crowd agent position");
                RequireFinite(agent.Velocity, "Crowd agent velocity");
                RequireFinite(agent.PreferredVelocity,
                    "Crowd agent preferred velocity");
                if (!std::isfinite(agent.Radius) || agent.Radius <= 0.0f)
                    throw std::invalid_argument(
                        "Crowd agent radius must be finite and positive.");
                if (!std::isfinite(agent.MaxSpeed) || agent.MaxSpeed < 0.0f)
                    throw std::invalid_argument(
                        "Crowd agent max speed must be finite and non-negative.");
                if (!std::isfinite(agent.NeighborDistance) ||
                    agent.NeighborDistance < 0.0f)
                    throw std::invalid_argument(
                        "Crowd agent neighbor distance must be finite and non-negative.");
                if (!std::isfinite(agent.TimeHorizon) ||
                    agent.TimeHorizon <= 0.0f)
                    throw std::invalid_argument(
                        "Crowd agent time horizon must be finite and positive.");
            }
        }

        static void ValidateObstacles(
            const std::vector<CrowdObstacle>& obstacles)
        {
            std::unordered_set<CrowdObstacleID> ids;
            ids.reserve(obstacles.size());
            for (const CrowdObstacle& obstacle : obstacles)
            {
                if (obstacle.ID == InvalidCrowdObstacleID)
                    throw std::invalid_argument(
                        "Crowd obstacle id is invalid.");
                if (!ids.insert(obstacle.ID).second)
                    throw std::invalid_argument(
                        "Crowd obstacle ids must be unique.");
                RequireFinite(obstacle.Position,
                    "Crowd obstacle position");
                if (!std::isfinite(obstacle.Radius) ||
                    obstacle.Radius <= 0.0f)
                    throw std::invalid_argument(
                        "Crowd obstacle radius must be finite and positive.");
            }
        }
    };
}
