module;

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
#include <queue>
#include <stdexcept>
#include <unordered_map>
#include <utility>
#include <vector>

export module Kairo.Foundation.Spatial.NavMesh;

import Kairo.Foundation.Math.Vector;

export namespace kairo::foundation::spatial
{
    using namespace kairo::foundation::math;

    using NavPolygonID = std::uint32_t;
    inline constexpr NavPolygonID InvalidNavPolygonID =
        std::numeric_limits<NavPolygonID>::max();

    struct NavMeshPolygon final
    {
        NavPolygonID ID = InvalidNavPolygonID;
        std::vector<Vec3f> Vertices;
        float AreaCost = 1.0f;
    };

    /// Directed shared-edge portal. Left/right are oriented while travelling
    /// from `From` to `To`, which lets funnel/string-pulling stay deterministic.
    struct NavMeshPortal final
    {
        NavPolygonID From = InvalidNavPolygonID;
        NavPolygonID To = InvalidNavPolygonID;
        Vec3f Left = Vec3f::Zero();
        Vec3f Right = Vec3f::Zero();
    };

    struct NavMeshPath final
    {
        bool Reached = false;
        float Cost = 0.0f;
        NavPolygonID StartPolygon = InvalidNavPolygonID;
        NavPolygonID EndPolygon = InvalidNavPolygonID;
        std::vector<NavPolygonID> Corridor;
        std::vector<Vec3f> Waypoints;
    };

    class NavMesh final
    {
    public:
        void AddPolygon(
            NavPolygonID id,
            std::vector<Vec3f> vertices,
            float areaCost = 1.0f)
        {
            if (id == InvalidNavPolygonID)
                throw std::invalid_argument("NavMesh polygon id is invalid.");
            if (m_IndexById.contains(id))
                throw std::invalid_argument("NavMesh polygon id is duplicated.");
            ValidatePolygon(vertices, areaCost);

            m_IndexById[id] = m_Polygons.size();
            m_Polygons.push_back({ id, std::move(vertices), areaCost });
            m_Adjacency.emplace_back();
            m_Built = false;
        }

        bool RemovePolygon(NavPolygonID id)
        {
            const auto found = m_IndexById.find(id);
            if (found == m_IndexById.end()) return false;
            m_Polygons.erase(m_Polygons.begin() +
                static_cast<std::ptrdiff_t>(found->second));
            RebuildLookup();
            m_Adjacency.assign(m_Polygons.size(), {});
            m_Built = false;
            return true;
        }

        void Clear() noexcept
        {
            m_Polygons.clear();
            m_IndexById.clear();
            m_Adjacency.clear();
            m_Built = false;
        }

        /// Derives polygon connectivity from welded shared edges. This is an
        /// authoring/runtime-load operation, not a per-frame query. Portals are
        /// generated in both directions and sorted by destination id so A* ties
        /// resolve identically on every platform.
        void BuildAdjacency(float weldEpsilon = 1.0e-3f)
        {
            if (!std::isfinite(weldEpsilon) || weldEpsilon <= 0.0f)
                throw std::invalid_argument(
                    "NavMesh weld epsilon must be finite and positive.");
            m_Adjacency.assign(m_Polygons.size(), {});

            for (std::size_t a = 0u; a < m_Polygons.size(); ++a)
            {
                for (std::size_t b = a + 1u; b < m_Polygons.size(); ++b)
                {
                    const auto shared = SharedEdge(
                        m_Polygons[a], m_Polygons[b], weldEpsilon);
                    if (!shared.has_value()) continue;

                    m_Adjacency[a].push_back(OrientPortal(
                        m_Polygons[a], m_Polygons[b],
                        shared->first, shared->second));
                    m_Adjacency[b].push_back(OrientPortal(
                        m_Polygons[b], m_Polygons[a],
                        shared->first, shared->second));
                }
            }

            for (auto& neighbors : m_Adjacency)
            {
                std::sort(neighbors.begin(), neighbors.end(),
                    [](const NavMeshPortal& lhs, const NavMeshPortal& rhs)
                    {
                        return lhs.To < rhs.To;
                    });
            }
            m_Built = true;
        }

        [[nodiscard]] bool Contains(NavPolygonID id) const noexcept
        {
            return m_IndexById.contains(id);
        }

        [[nodiscard]] const NavMeshPolygon& Polygon(NavPolygonID id) const
        {
            return m_Polygons.at(RequireIndex(id));
        }

        [[nodiscard]] const std::vector<NavMeshPolygon>& Polygons() const noexcept
        {
            return m_Polygons;
        }

        [[nodiscard]] const std::vector<NavMeshPortal>& Neighbors(
            NavPolygonID id) const
        {
            RequireBuilt();
            return m_Adjacency.at(RequireIndex(id));
        }

        [[nodiscard]] std::optional<NavPolygonID> FindContainingPolygon(
            const Vec3f& point,
            float verticalTolerance = 1.0f) const
        {
            RequireFinite(point, "NavMesh containment point");
            if (!std::isfinite(verticalTolerance) || verticalTolerance < 0.0f)
                throw std::invalid_argument(
                    "NavMesh vertical tolerance must be finite and non-negative.");

            std::optional<NavPolygonID> best;
            float bestVerticalDistance = std::numeric_limits<float>::infinity();
            for (const auto& polygon : m_Polygons)
            {
                if (!PointInConvexXZ(polygon.Vertices, point, 1.0e-5f))
                    continue;
                const float height = ProjectedHeight(polygon.Vertices, point.x, point.z);
                const float verticalDistance = std::abs(point.y - height);
                if (verticalDistance > verticalTolerance) continue;
                if (!best.has_value() || verticalDistance < bestVerticalDistance ||
                    (std::abs(verticalDistance - bestVerticalDistance) <= 1.0e-6f &&
                        polygon.ID < *best))
                {
                    best = polygon.ID;
                    bestVerticalDistance = verticalDistance;
                }
            }
            return best;
        }

        [[nodiscard]] std::optional<NavPolygonID> FindNearestPolygon(
            const Vec3f& point,
            float maxDistance = std::numeric_limits<float>::infinity()) const
        {
            RequireFinite(point, "NavMesh nearest point");
            if (std::isnan(maxDistance) || maxDistance < 0.0f)
                throw std::invalid_argument(
                    "NavMesh maximum nearest distance must be non-negative.");

            std::optional<NavPolygonID> best;
            float bestDistanceSquared = maxDistance ==
                std::numeric_limits<float>::infinity()
                ? std::numeric_limits<float>::infinity()
                : maxDistance * maxDistance;
            for (const auto& polygon : m_Polygons)
            {
                const Vec3f closest = ClosestPoint(polygon.Vertices, point);
                const float distanceSquared = (closest - point).LengthSquared();
                if (distanceSquared < bestDistanceSquared ||
                    (std::abs(distanceSquared - bestDistanceSquared) <= 1.0e-6f &&
                        (!best.has_value() || polygon.ID < *best)))
                {
                    best = polygon.ID;
                    bestDistanceSquared = distanceSquared;
                }
            }
            return best;
        }

        /// Finds a polygon corridor with A* and converts it to a minimal corner
        /// chain with the classic funnel algorithm. Start/end may be slightly
        /// off-mesh and are snapped to the nearest polygon within snapDistance.
        [[nodiscard]] NavMeshPath FindPath(
            const Vec3f& start,
            const Vec3f& end,
            float snapDistance = 1.0f,
            float verticalTolerance = 1.0f) const
        {
            RequireBuilt();
            RequireFinite(start, "NavMesh path start");
            RequireFinite(end, "NavMesh path end");
            if (!std::isfinite(snapDistance) || snapDistance < 0.0f)
                throw std::invalid_argument(
                    "NavMesh path snap distance must be finite and non-negative.");

            NavMeshPath result;
            const auto startPolygon = ResolvePolygon(
                start, snapDistance, verticalTolerance);
            const auto endPolygon = ResolvePolygon(
                end, snapDistance, verticalTolerance);
            if (!startPolygon.has_value() || !endPolygon.has_value())
                return result;

            result.StartPolygon = *startPolygon;
            result.EndPolygon = *endPolygon;
            const std::size_t startIndex = RequireIndex(*startPolygon);
            const std::size_t goalIndex = RequireIndex(*endPolygon);
            if (startIndex == goalIndex)
            {
                result.Reached = true;
                result.Corridor = { *startPolygon };
                result.Waypoints = { start, end };
                return result;
            }

            const std::size_t count = m_Polygons.size();
            const float infinity = std::numeric_limits<float>::infinity();
            std::vector<float> gScore(count, infinity);
            std::vector<std::size_t> parent(count, count);
            std::vector<bool> closed(count, false);

            struct OpenNode final
            {
                float F = 0.0f;
                float G = 0.0f;
                std::size_t Index = 0u;
                NavPolygonID ID = InvalidNavPolygonID;
            };
            struct Greater final
            {
                bool operator()(const OpenNode& lhs, const OpenNode& rhs) const noexcept
                {
                    if (lhs.F != rhs.F) return lhs.F > rhs.F;
                    if (lhs.G != rhs.G) return lhs.G > rhs.G;
                    return lhs.ID > rhs.ID;
                }
            };

            std::priority_queue<OpenNode, std::vector<OpenNode>, Greater> open;
            gScore[startIndex] = 0.0f;
            open.push({ Heuristic(startIndex, goalIndex), 0.0f,
                startIndex, m_Polygons[startIndex].ID });

            while (!open.empty())
            {
                const OpenNode current = open.top();
                open.pop();
                if (closed[current.Index]) continue;
                if (current.G > gScore[current.Index] + 1.0e-6f) continue;
                closed[current.Index] = true;
                if (current.Index == goalIndex) break;

                for (const auto& portal : m_Adjacency[current.Index])
                {
                    const std::size_t next = RequireIndex(portal.To);
                    if (closed[next]) continue;
                    const float tentative = gScore[current.Index] +
                        EdgeCost(current.Index, next);
                    if (tentative + 1.0e-6f < gScore[next])
                    {
                        gScore[next] = tentative;
                        parent[next] = current.Index;
                        open.push({ tentative + Heuristic(next, goalIndex), tentative,
                            next, m_Polygons[next].ID });
                    }
                }
            }

            if (!closed[goalIndex]) return result;

            std::vector<std::size_t> reversed;
            for (std::size_t cursor = goalIndex;; cursor = parent[cursor])
            {
                reversed.push_back(cursor);
                if (cursor == startIndex) break;
                if (parent[cursor] == count)
                    throw std::logic_error(
                        "NavMesh A* parent chain is incomplete.");
            }
            std::reverse(reversed.begin(), reversed.end());
            result.Corridor.reserve(reversed.size());
            for (const std::size_t index : reversed)
                result.Corridor.push_back(m_Polygons[index].ID);

            result.Waypoints = Funnel(start, end, result.Corridor);
            result.Cost = gScore[goalIndex];
            result.Reached = true;
            return result;
        }

    private:
        std::vector<NavMeshPolygon> m_Polygons;
        std::unordered_map<NavPolygonID, std::size_t> m_IndexById;
        std::vector<std::vector<NavMeshPortal>> m_Adjacency;
        bool m_Built = false;

        static void RequireFinite(const Vec3f& point, const char* label)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z))
                throw std::invalid_argument(std::string(label) + " must be finite.");
        }

        static void ValidatePolygon(
            const std::vector<Vec3f>& vertices,
            float areaCost)
        {
            if (vertices.size() < 3u || vertices.size() > 64u)
                throw std::invalid_argument(
                    "NavMesh polygons require between three and sixty-four vertices.");
            if (!std::isfinite(areaCost) || areaCost <= 0.0f)
                throw std::invalid_argument(
                    "NavMesh polygon area cost must be finite and positive.");
            for (const auto& vertex : vertices)
                RequireFinite(vertex, "NavMesh polygon vertex");

            const float signedArea = SignedAreaXZ(vertices);
            if (std::abs(signedArea) <= 1.0e-6f)
                throw std::invalid_argument(
                    "NavMesh polygon projected XZ area is degenerate.");
            const float winding = signedArea > 0.0f ? 1.0f : -1.0f;
            for (std::size_t i = 0u; i < vertices.size(); ++i)
            {
                const Vec3f& a = vertices[i];
                const Vec3f& b = vertices[(i + 1u) % vertices.size()];
                const Vec3f& c = vertices[(i + 2u) % vertices.size()];
                const float turn = CrossXZ(b - a, c - b);
                if (std::abs(turn) > 1.0e-6f && turn * winding < 0.0f)
                    throw std::invalid_argument(
                        "NavMesh polygons must be convex in XZ projection.");
            }

            const Vec3f normal = PolygonNormal(vertices);
            const Vec3f origin = vertices.front();
            for (const auto& vertex : vertices)
            {
                const float planeDistance = std::abs(Dot(vertex - origin, normal));
                if (planeDistance > 1.0e-3f)
                    throw std::invalid_argument(
                        "NavMesh polygon vertices must be coplanar.");
            }
        }

        [[nodiscard]] static float Dot(const Vec3f& a, const Vec3f& b) noexcept
        {
            return a.x * b.x + a.y * b.y + a.z * b.z;
        }

        [[nodiscard]] static Vec3f Cross(const Vec3f& a, const Vec3f& b) noexcept
        {
            return {
                a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x
            };
        }

        [[nodiscard]] static float CrossXZ(const Vec3f& a, const Vec3f& b) noexcept
        {
            return a.x * b.z - a.z * b.x;
        }

        [[nodiscard]] static float TriArea2XZ(
            const Vec3f& a,
            const Vec3f& b,
            const Vec3f& c) noexcept
        {
            return (b.x - a.x) * (c.z - a.z) -
                (b.z - a.z) * (c.x - a.x);
        }

        [[nodiscard]] static float SignedAreaXZ(
            const std::vector<Vec3f>& vertices) noexcept
        {
            float twiceArea = 0.0f;
            for (std::size_t i = 0u; i < vertices.size(); ++i)
            {
                const auto& a = vertices[i];
                const auto& b = vertices[(i + 1u) % vertices.size()];
                twiceArea += a.x * b.z - b.x * a.z;
            }
            return twiceArea * 0.5f;
        }

        [[nodiscard]] static Vec3f PolygonNormal(
            const std::vector<Vec3f>& vertices)
        {
            Vec3f normal = Vec3f::Zero();
            for (std::size_t i = 0u; i < vertices.size(); ++i)
            {
                const Vec3f& a = vertices[i];
                const Vec3f& b = vertices[(i + 1u) % vertices.size()];
                normal.x += (a.y - b.y) * (a.z + b.z);
                normal.y += (a.z - b.z) * (a.x + b.x);
                normal.z += (a.x - b.x) * (a.y + b.y);
            }
            const float length = normal.Length();
            if (!std::isfinite(length) || length <= 1.0e-8f ||
                std::abs(normal.y) <= 1.0e-8f)
                throw std::invalid_argument(
                    "NavMesh polygon does not define a walkable XZ-projected plane.");
            return normal / length;
        }

        [[nodiscard]] static Vec3f Centroid(const NavMeshPolygon& polygon) noexcept
        {
            Vec3f result = Vec3f::Zero();
            for (const auto& vertex : polygon.Vertices) result += vertex;
            return result / static_cast<float>(polygon.Vertices.size());
        }

        [[nodiscard]] static bool Near(
            const Vec3f& a,
            const Vec3f& b,
            float epsilon) noexcept
        {
            return (a - b).LengthSquared() <= epsilon * epsilon;
        }

        [[nodiscard]] static std::optional<std::pair<Vec3f, Vec3f>> SharedEdge(
            const NavMeshPolygon& a,
            const NavMeshPolygon& b,
            float epsilon)
        {
            for (std::size_t ai = 0u; ai < a.Vertices.size(); ++ai)
            {
                const Vec3f& a0 = a.Vertices[ai];
                const Vec3f& a1 = a.Vertices[(ai + 1u) % a.Vertices.size()];
                for (std::size_t bi = 0u; bi < b.Vertices.size(); ++bi)
                {
                    const Vec3f& b0 = b.Vertices[bi];
                    const Vec3f& b1 = b.Vertices[(bi + 1u) % b.Vertices.size()];
                    if (Near(a0, b1, epsilon) && Near(a1, b0, epsilon))
                        return std::pair<Vec3f, Vec3f>{
                            (a0 + b1) * 0.5f,
                            (a1 + b0) * 0.5f };
                    if (Near(a0, b0, epsilon) && Near(a1, b1, epsilon))
                        return std::pair<Vec3f, Vec3f>{
                            (a0 + b0) * 0.5f,
                            (a1 + b1) * 0.5f };
                }
            }
            return std::nullopt;
        }

        [[nodiscard]] static NavMeshPortal OrientPortal(
            const NavMeshPolygon& from,
            const NavMeshPolygon& to,
            const Vec3f& a,
            const Vec3f& b) noexcept
        {
            const Vec3f fromCenter = Centroid(from);
            const Vec3f direction = Centroid(to) - fromCenter;
            const float sideA = CrossXZ(direction, a - fromCenter);
            const float sideB = CrossXZ(direction, b - fromCenter);
            if (sideA > sideB)
                return { from.ID, to.ID, a, b };
            if (sideB > sideA)
                return { from.ID, to.ID, b, a };
            const bool aFirst = a.x < b.x ||
                (a.x == b.x && (a.z < b.z || (a.z == b.z && a.y <= b.y)));
            return aFirst
                ? NavMeshPortal{ from.ID, to.ID, a, b }
                : NavMeshPortal{ from.ID, to.ID, b, a };
        }

        [[nodiscard]] static bool PointInConvexXZ(
            const std::vector<Vec3f>& vertices,
            const Vec3f& point,
            float epsilon) noexcept
        {
            float sign = 0.0f;
            for (std::size_t i = 0u; i < vertices.size(); ++i)
            {
                const float side = TriArea2XZ(
                    vertices[i], vertices[(i + 1u) % vertices.size()], point);
                if (std::abs(side) <= epsilon) continue;
                if (sign == 0.0f) sign = side > 0.0f ? 1.0f : -1.0f;
                else if (side * sign < -epsilon) return false;
            }
            return true;
        }

        [[nodiscard]] static float ProjectedHeight(
            const std::vector<Vec3f>& vertices,
            float x,
            float z)
        {
            const Vec3f normal = PolygonNormal(vertices);
            const Vec3f& origin = vertices.front();
            return origin.y -
                (normal.x * (x - origin.x) + normal.z * (z - origin.z)) /
                    normal.y;
        }

        [[nodiscard]] static Vec3f ClosestPoint(
            const std::vector<Vec3f>& vertices,
            const Vec3f& point)
        {
            if (PointInConvexXZ(vertices, point, 1.0e-5f))
                return { point.x, ProjectedHeight(vertices, point.x, point.z), point.z };

            Vec3f closest = vertices.front();
            float bestDistanceSquared = (closest - point).LengthSquared();
            for (std::size_t i = 0u; i < vertices.size(); ++i)
            {
                const Vec3f& a = vertices[i];
                const Vec3f& b = vertices[(i + 1u) % vertices.size()];
                const float dx = b.x - a.x;
                const float dz = b.z - a.z;
                const float denominator = dx * dx + dz * dz;
                const float t = denominator <= 1.0e-12f
                    ? 0.0f
                    : std::clamp(
                        ((point.x - a.x) * dx + (point.z - a.z) * dz) /
                            denominator,
                        0.0f, 1.0f);
                const Vec3f candidate = a + (b - a) * t;
                const float distanceSquared = (candidate - point).LengthSquared();
                if (distanceSquared < bestDistanceSquared)
                {
                    closest = candidate;
                    bestDistanceSquared = distanceSquared;
                }
            }
            return closest;
        }

        [[nodiscard]] std::optional<NavPolygonID> ResolvePolygon(
            const Vec3f& point,
            float snapDistance,
            float verticalTolerance) const
        {
            const auto containing = FindContainingPolygon(point, verticalTolerance);
            return containing.has_value()
                ? containing
                : FindNearestPolygon(point, snapDistance);
        }

        [[nodiscard]] std::size_t RequireIndex(NavPolygonID id) const
        {
            const auto found = m_IndexById.find(id);
            if (found == m_IndexById.end())
                throw std::out_of_range("NavMesh polygon id does not exist.");
            return found->second;
        }

        void RequireBuilt() const
        {
            if (!m_Built)
                throw std::logic_error(
                    "NavMesh adjacency must be built before path queries.");
        }

        void RebuildLookup()
        {
            m_IndexById.clear();
            for (std::size_t i = 0u; i < m_Polygons.size(); ++i)
                m_IndexById[m_Polygons[i].ID] = i;
        }

        [[nodiscard]] float Heuristic(
            std::size_t from,
            std::size_t to) const noexcept
        {
            return Distance(Centroid(m_Polygons[from]), Centroid(m_Polygons[to]));
        }

        [[nodiscard]] float EdgeCost(
            std::size_t from,
            std::size_t to) const noexcept
        {
            const float distance = Distance(
                Centroid(m_Polygons[from]), Centroid(m_Polygons[to]));
            const float costMultiplier =
                (m_Polygons[from].AreaCost + m_Polygons[to].AreaCost) * 0.5f;
            return distance * costMultiplier;
        }

        [[nodiscard]] const NavMeshPortal& PortalBetween(
            NavPolygonID from,
            NavPolygonID to) const
        {
            const auto& neighbors = m_Adjacency.at(RequireIndex(from));
            const auto found = std::find_if(neighbors.begin(), neighbors.end(),
                [to](const NavMeshPortal& portal) { return portal.To == to; });
            if (found == neighbors.end())
                throw std::logic_error(
                    "NavMesh corridor contains polygons without a shared portal.");
            return *found;
        }

        [[nodiscard]] static bool SameXZ(
            const Vec3f& a,
            const Vec3f& b,
            float epsilon = 1.0e-5f) noexcept
        {
            return std::abs(a.x - b.x) <= epsilon &&
                std::abs(a.z - b.z) <= epsilon;
        }

        [[nodiscard]] std::vector<Vec3f> Funnel(
            const Vec3f& start,
            const Vec3f& end,
            const std::vector<NavPolygonID>& corridor) const
        {
            struct PortalPair final { Vec3f Left; Vec3f Right; };
            std::vector<PortalPair> portals;
            portals.reserve(corridor.size() + 1u);
            portals.push_back({ start, start });
            for (std::size_t i = 0u; i + 1u < corridor.size(); ++i)
            {
                const auto& portal = PortalBetween(corridor[i], corridor[i + 1u]);
                portals.push_back({ portal.Left, portal.Right });
            }
            portals.push_back({ end, end });

            std::vector<Vec3f> points;
            points.push_back(start);
            Vec3f apex = start;
            Vec3f left = start;
            Vec3f right = start;
            std::size_t apexIndex = 0u;
            std::size_t leftIndex = 0u;
            std::size_t rightIndex = 0u;

            for (std::size_t i = 1u; i < portals.size(); ++i)
            {
                const Vec3f newLeft = portals[i].Left;
                const Vec3f newRight = portals[i].Right;

                if (TriArea2XZ(apex, right, newRight) <= 0.0f)
                {
                    if (SameXZ(apex, right) ||
                        TriArea2XZ(apex, left, newRight) > 0.0f)
                    {
                        right = newRight;
                        rightIndex = i;
                    }
                    else
                    {
                        points.push_back(left);
                        apex = left;
                        apexIndex = leftIndex;
                        left = apex;
                        right = apex;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;
                        i = apexIndex;
                        continue;
                    }
                }

                if (TriArea2XZ(apex, left, newLeft) >= 0.0f)
                {
                    if (SameXZ(apex, left) ||
                        TriArea2XZ(apex, right, newLeft) < 0.0f)
                    {
                        left = newLeft;
                        leftIndex = i;
                    }
                    else
                    {
                        points.push_back(right);
                        apex = right;
                        apexIndex = rightIndex;
                        left = apex;
                        right = apex;
                        leftIndex = apexIndex;
                        rightIndex = apexIndex;
                        i = apexIndex;
                        continue;
                    }
                }
            }

            if (points.empty() || !SameXZ(points.back(), end))
                points.push_back(end);
            else
                points.back() = end;
            return points;
        }
    };
}
