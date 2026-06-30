#include <algorithm>
#include <array>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

import Kairo.Foundation.Spatial;
import Kairo.Foundation.Geometry;
import Kairo.Foundation.Math;

using namespace kairo::foundation::spatial;
using namespace kairo::foundation::geometry;
using namespace kairo::foundation::math;

namespace
{
    struct Canvas final
    {
        float Width = 560.0f;
        float Height = 360.0f;
        float Scale = 52.0f;
        float OriginX = Width * 0.5f;
        float OriginY = Height * 0.5f;
        std::ostringstream Body;

        [[nodiscard]]
        std::string Point(
            const Vec3f& point) const
        {
            std::ostringstream out;
            out << std::fixed << std::setprecision(2)
                << (OriginX + point.x * Scale)
                << ","
                << (OriginY - point.y * Scale);
            return out.str();
        }

        void Line(
            const Vec3f& a,
            const Vec3f& b,
            std::string_view color,
            float stroke = 2.0f,
            std::string_view dash = "",
            std::string_view attributes = "")
        {
            Body << "<line x1=\"" << OriginX + a.x * Scale
                 << "\" y1=\"" << OriginY - a.y * Scale
                 << "\" x2=\"" << OriginX + b.x * Scale
                 << "\" y2=\"" << OriginY - b.y * Scale
                 << "\" stroke=\"" << color
                 << "\" stroke-width=\"" << stroke
                 << "\" stroke-linecap=\"round\"";

            if (!dash.empty())
            {
                Body << " stroke-dasharray=\"" << dash << "\"";
            }

            if (!attributes.empty())
            {
                Body << " " << attributes;
            }

            Body << " />\n";
        }

        void RectWorld(
            const SpatialAABB& box,
            std::string_view stroke,
            std::string_view fill,
            float strokeWidth = 2.0f,
            std::string_view dash = "",
            std::string_view attributes = "")
        {
            const float x = OriginX + box.Min.x * Scale;
            const float y = OriginY - box.Max.y * Scale;
            const float w = (box.Max.x - box.Min.x) * Scale;
            const float h = (box.Max.y - box.Min.y) * Scale;

            Body << "<rect x=\"" << x
                 << "\" y=\"" << y
                 << "\" width=\"" << w
                 << "\" height=\"" << h
                 << "\" fill=\"" << fill
                 << "\" stroke=\"" << stroke
                 << "\" stroke-width=\"" << strokeWidth << "\"";

            if (!dash.empty())
            {
                Body << " stroke-dasharray=\"" << dash << "\"";
            }

            if (!attributes.empty())
            {
                Body << " " << attributes;
            }

            Body << " />\n";
        }

        void CircleWorld(
            const Vec3f& center,
            float radius,
            std::string_view stroke,
            std::string_view fill,
            float strokeWidth = 2.0f,
            std::string_view attributes = "")
        {
            Body << "<circle cx=\"" << OriginX + center.x * Scale
                 << "\" cy=\"" << OriginY - center.y * Scale
                 << "\" r=\"" << radius * Scale
                 << "\" fill=\"" << fill
                 << "\" stroke=\"" << stroke
                 << "\" stroke-width=\"" << strokeWidth << "\"";

            if (!attributes.empty())
            {
                Body << " " << attributes;
            }

            Body << " />\n";
        }

        void PointMark(
            const Vec3f& point,
            std::string_view color,
            float radius = 4.0f,
            std::string_view attributes = "")
        {
            Body << "<circle cx=\"" << OriginX + point.x * Scale
                 << "\" cy=\"" << OriginY - point.y * Scale
                 << "\" r=\"" << radius
                 << "\" fill=\"" << color << "\"";

            if (!attributes.empty())
            {
                Body << " " << attributes;
            }

            Body << " />\n";
        }

        void Label(
            const Vec3f& point,
            std::string_view text,
            std::string_view color = "#e2e8f0",
            std::string_view attributes = "")
        {
            Body << "<text x=\"" << OriginX + point.x * Scale + 6.0f
                 << "\" y=\"" << OriginY - point.y * Scale - 6.0f
                 << "\" fill=\"" << color
                 << "\" font-size=\"12\" font-family=\"ui-monospace, SFMono-Regular, Menlo, monospace\"";

            if (!attributes.empty())
            {
                Body << " " << attributes;
            }

            Body << ">"
                 << text << "</text>\n";
        }

        [[nodiscard]]
        std::string Svg() const
        {
            std::ostringstream out;
            out << "<svg viewBox=\"0 0 " << Width << " " << Height << "\" data-default-viewbox=\"0 0 " << Width << " " << Height << "\" role=\"img\">\n"
                << "<rect width=\"100%\" height=\"100%\" rx=\"8\" fill=\"#09111f\" />\n"
                << "<g opacity=\"0.55\">\n";

            for (int x = -5; x <= 5; ++x)
            {
                out << "<line x1=\"" << OriginX + x * Scale
                    << "\" y1=\"0\" x2=\"" << OriginX + x * Scale
                    << "\" y2=\"" << Height << "\" stroke=\"#172033\" />\n";
            }

            for (int y = -3; y <= 3; ++y)
            {
                out << "<line x1=\"0\" y1=\"" << OriginY + y * Scale
                    << "\" x2=\"" << Width << "\" y2=\"" << OriginY + y * Scale
                    << "\" stroke=\"#172033\" />\n";
            }

            out << "</g>\n"
                << Body.str()
                << "</svg>\n";

            return out.str();
        }
    };

    [[nodiscard]]
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

    [[nodiscard]]
    std::vector<SpatialPrimitive> ScenePrimitives()
    {
        return
        {
            { 101, Box(-4.2f, -1.1f, -0.1f, -3.1f, 0.15f, 0.1f), 0x1u },
            { 102, Box(-1.9f, -0.7f, -0.1f, -0.8f, 0.65f, 0.1f), 0x1u },
            { 103, Box(0.15f, -0.55f, -0.1f, 1.15f, 0.45f, 0.1f), 0x2u },
            { 104, Box(2.0f, 0.55f, -0.1f, 3.1f, 1.55f, 0.1f), 0x2u },
            { 105, Box(3.2f, -1.65f, -0.1f, 4.35f, -0.45f, 0.1f), 0x4u },
            { 106, Box(-0.2f, 1.7f, -0.1f, 0.95f, 2.65f, 0.1f), 0x4u }
        };
    }

    [[nodiscard]]
    std::string JoinIDs(
        std::vector<SpatialID> ids)
    {
        std::sort(ids.begin(), ids.end());

        std::ostringstream out;
        for (std::size_t i = 0; i < ids.size(); ++i)
        {
            if (i > 0)
            {
                out << ", ";
            }

            out << ids[i];
        }

        return out.str();
    }

    [[nodiscard]]
    std::string FormatVec(
        const Vec3f& value)
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(2)
            << "(" << value.x << ", " << value.y << ", " << value.z << ")";
        return out.str();
    }

    [[nodiscard]]
    std::string Card(
        std::string_view id,
        std::string_view category,
        std::string_view title,
        std::string_view description,
        const Canvas& canvas,
        std::string_view metrics)
    {
        std::ostringstream out;
        out << "<section class=\"card is-active\" id=\"" << id << "\" data-panel=\"" << id
            << "\" data-category=\"" << category << "\">\n"
            << "<div class=\"copy\">\n"
            << "<h2>" << title << "</h2>\n"
            << "<p>" << description << "</p>\n"
            << "</div>\n"
            << "<div class=\"card-tools\" aria-label=\"" << title << " controls\">\n"
            << "<button type=\"button\" class=\"tool-button\" data-action=\"focus\">Focus</button>\n"
            << "<button type=\"button\" class=\"tool-button\" data-action=\"reset-view\">Reset view</button>\n"
            << "</div>\n"
            << canvas.Svg()
            << "<pre>" << metrics << "</pre>\n"
            << "</section>\n";

        return out.str();
    }

    void DrawPrimitiveSet(
        Canvas& canvas,
        const std::vector<SpatialPrimitive>& primitives)
    {
        for (const SpatialPrimitive& primitive : primitives)
        {
            const std::string_view fill =
                primitive.LayerMask == 0x1u
                    ? "rgba(56,189,248,0.18)"
                    : (primitive.LayerMask == 0x2u
                        ? "rgba(34,197,94,0.18)"
                        : "rgba(249,115,22,0.18)");

            const std::string_view stroke =
                primitive.LayerMask == 0x1u
                    ? "#38bdf8"
                    : (primitive.LayerMask == 0x2u
                        ? "#22c55e"
                        : "#f97316");

            std::ostringstream attributes;
            attributes << "class=\"spatial-primitive layer-" << primitive.LayerMask
                       << "\" data-kind=\"primitive\" data-id=\"" << primitive.ID
                       << "\" data-layer=\"" << primitive.LayerMask
                       << "\" tabindex=\"0\"";

            canvas.RectWorld(primitive.Bounds, stroke, fill, 2.0f, "", attributes.str());
            canvas.Label(
                primitive.Bounds.Center(),
                std::to_string(primitive.ID),
                "#e2e8f0",
                "class=\"primitive-label\" data-label-for=\"" + std::to_string(primitive.ID) + "\"");
        }
    }

    [[nodiscard]]
    std::string BuildBVHCard()
    {
        const std::vector<SpatialPrimitive> primitives =
            ScenePrimitives();

        BVHBuildSettings settings;
        settings.MaxLeafSize = 2;
        settings.UseSAH = true;

        const BVH bvh =
            BuildBVH(primitives, settings);

        const SpatialAABB query =
            Box(-2.25f, -1.0f, -0.2f, 1.3f, 0.85f, 0.2f);

        const SpatialOverlapResult overlap =
            QueryAABB(bvh, query);

        Canvas canvas;
        const std::vector<DebugBox> boxes =
            ExtractBVHBoxes(bvh);

        for (const DebugBox& debugBox : boxes)
        {
            canvas.RectWorld(
                debugBox.Bounds,
                debugBox.Color.x > 0.5f ? "#22c55e" : "#475569",
                "none",
                debugBox.Color.x > 0.5f ? 1.5f : 1.0f,
                debugBox.Color.x > 0.5f ? "" : "6 5",
                debugBox.Color.x > 0.5f ? "class=\"bvh-leaf\" data-kind=\"leaf\"" : "class=\"bvh-node\" data-kind=\"node\"");
        }

        DrawPrimitiveSet(canvas, primitives);
        canvas.RectWorld(query, "#eab308", "rgba(234,179,8,0.12)", 3.0f, "8 5", "class=\"query-shape\" data-kind=\"query\"");

        std::ostringstream metrics;
        metrics << "nodes = " << bvh.Nodes.size()
                << "\nleaves = " << bvh.Stats.LeafCount
                << "\nvalid = " << (bvh.IsValid() ? "true" : "false")
                << "\nquery ids = [" << JoinIDs(overlap.IDs) << "]"
                << "\nvisited nodes = " << overlap.VisitedNodes
                << "\ntested primitives = " << overlap.TestedPrimitives;

        return Card(
            "bvh-panel",
            "static",
            "BVH hierarchy and AABB query",
            "Blue/green/orange boxes are geometry primitives. Gray dashed boxes are BVH hierarchy bounds. Yellow shows the query AABB.",
            canvas,
            metrics.str());
    }

    [[nodiscard]]
    std::string BuildRayCard()
    {
        const std::vector<SpatialPrimitive> primitives =
            ScenePrimitives();

        const BVH bvh =
            BuildBVH(primitives, { 2, 64, false, false });

        const SpatialRay ray =
            SpatialRay::FromOriginDirection(
                Vec3f{ -5.1f, -0.25f, 0.0f },
                Vec3f{ 1.0f, 0.05f, 0.0f }.Normalized());

        const SpatialRaycastResult raycast =
            Raycast(
                bvh,
                ray,
                [&](SpatialIndex primitiveIndex, const SpatialRay& queryRay) -> std::optional<SpatialRayHit>
                {
                    float distance = 0.0f;
                    if (!RayIntersectsAABB(queryRay, primitives[primitiveIndex].Bounds, 20.0f, distance))
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
                },
                20.0f);

        Canvas canvas;
        DrawPrimitiveSet(canvas, primitives);
        canvas.Line(ray.Origin, ray.GetPoint(9.5f), "#eab308", 3.0f, "", "class=\"ray-line\" data-kind=\"ray\"");
        canvas.PointMark(ray.Origin, "#eab308", 5.0f, "class=\"ray-origin\" data-kind=\"ray-origin\"");

        if (raycast.Hit)
        {
            canvas.PointMark(raycast.Closest.Position, "#ef4444", 6.0f, "class=\"hit-point\" data-kind=\"ray-hit\"");
            canvas.Label(raycast.Closest.Position, "nearest", "#e2e8f0", "class=\"hit-label\"");
        }

        std::ostringstream metrics;
        metrics << "hit = " << (raycast.Hit ? "true" : "false")
                << "\nclosest id = " << raycast.Closest.ID
                << "\ndistance = " << std::fixed << std::setprecision(3) << raycast.Closest.Distance
                << "\nhits stored = " << raycast.Hits.size()
                << "\nvisited nodes = " << raycast.VisitedNodes
                << "\ntested primitives = " << raycast.TestedPrimitives;

        return Card(
            "ray-panel",
            "static",
            "BVH ray traversal",
            "The ray is tested against BVH bounds first, then primitive AABBs through the callback path used by future triangle ray tracing.",
            canvas,
            metrics.str());
    }

    [[nodiscard]]
    std::string BuildDynamicTreeCard()
    {
        DynamicAABBTree tree(0.25f);
        const std::vector<SpatialPrimitive> primitives =
            ScenePrimitives();

        for (const SpatialPrimitive& primitive : primitives)
        {
            const SpatialIndex proxy =
                tree.Insert(
                    primitive.ID,
                    primitive.Bounds,
                    primitive.LayerMask);

            (void)proxy;
        }

        const SpatialAABB layerQuery =
            Box(-2.5f, -1.2f, -0.3f, 1.4f, 0.9f, 0.3f);

        const SpatialOverlapResult all =
            tree.QueryAABB(layerQuery);

        const SpatialOverlapResult layerOne =
            tree.QueryAABB(layerQuery, 0x1u);

        const DynamicAABBTreeStats stats =
            tree.Stats();

        Canvas canvas;
        const std::vector<DebugBox> boxes =
            ExtractDynamicTreeBounds(tree);

        for (const DebugBox& box : boxes)
        {
            if (box.ID == SpatialInvalidID)
            {
                canvas.RectWorld(box.Bounds, "#64748b", "none", 1.0f, "4 4", "class=\"dynamic-fat-bound\" data-kind=\"fat-bound\"");
            }
        }

        DrawPrimitiveSet(canvas, primitives);
        canvas.RectWorld(layerQuery, "#eab308", "rgba(234,179,8,0.12)", 3.0f, "8 5", "class=\"query-shape\" data-kind=\"query\"");

        std::ostringstream metrics;
        metrics << "all query ids = [" << JoinIDs(all.IDs) << "]"
                << "\nlayer 0x1 ids = [" << JoinIDs(layerOne.IDs) << "]"
                << "\npair count = " << tree.ComputePairs().size()
                << "\nheight = " << stats.Height
                << "\nmax balance = " << stats.MaxBalance
                << "\narea ratio = " << std::fixed << std::setprecision(3) << stats.AreaRatio
                << "\nvalid = " << (tree.Validate().Valid ? "true" : "false");

        return Card(
            "dynamic-panel",
            "dynamic",
            "Dynamic AABB tree broadphase",
            "Primitive tight bounds are drawn with colors by layer. Dashed gray bounds are internal fat bounds used for moving-object broadphase.",
            canvas,
            metrics.str());
    }

    [[nodiscard]]
    std::string BuildSpatialHashCard()
    {
        const std::vector<SpatialPrimitive> primitives =
            ScenePrimitives();

        SpatialHash hash(1.0f);
        for (const SpatialPrimitive& primitive : primitives)
        {
            hash.Insert(primitive.ID, primitive.Bounds);
        }

        const Vec3f center{ 0.0f, 0.0f, 0.0f };
        constexpr float radius = 2.15f;
        const std::vector<SpatialID> radiusIDs =
            hash.QueryRadius(center, radius);

        Canvas canvas;
        canvas.Scale = 48.0f;

        for (int y = -3; y <= 3; ++y)
        {
            for (int x = -5; x <= 5; ++x)
            {
                canvas.RectWorld(
                    Box(static_cast<float>(x), static_cast<float>(y), -0.1f, static_cast<float>(x + 1), static_cast<float>(y + 1), 0.1f),
                    "#1e293b",
                    "none",
                    1.0f,
                    "",
                    "class=\"hash-cell\" data-kind=\"hash-cell\"");
            }
        }

        DrawPrimitiveSet(canvas, primitives);
        canvas.CircleWorld(center, radius, "#eab308", "rgba(234,179,8,0.10)", 3.0f, "class=\"query-shape\" data-kind=\"radius-query\"");

        std::ostringstream metrics;
        metrics << "cell size = 1.0"
                << "\nradius center = " << FormatVec(center)
                << "\nradius ids = [" << JoinIDs(radiusIDs) << "]"
                << "\nnegative cell lookup id count = " << hash.QueryCell({ -4, -1, 0 }).size();

        return Card(
            "hash-panel",
            "grid",
            "Spatial hash grid",
            "Uniform grid cells are shown behind the geometry. The yellow circle is a near-neighbor radius query.",
            canvas,
            metrics.str());
    }

    [[nodiscard]]
    std::string BuildKDTreeCard()
    {
        const std::vector<KDPoint> points =
        {
            { 301, Vec3f{ -3.0f, -0.8f, 0.0f } },
            { 302, Vec3f{ -1.3f, 1.2f, 0.0f } },
            { 303, Vec3f{ 0.35f, 0.25f, 0.0f } },
            { 304, Vec3f{ 1.4f, -1.4f, 0.0f } },
            { 305, Vec3f{ 3.0f, 1.0f, 0.0f } },
            { 306, Vec3f{ 3.6f, -0.9f, 0.0f } }
        };

        KDTree tree(points);
        const Vec3f query{ 0.0f, 0.0f, 0.0f };
        const auto nearest =
            tree.NearestPoint(query);

        const std::vector<KDNearestResult> kNearest =
            tree.KNearest(query, 3);

        const std::vector<KDNearestResult> radius =
            tree.RadiusSearch(query, 2.0f);

        Canvas canvas;
        canvas.CircleWorld(query, 2.0f, "#eab308", "rgba(234,179,8,0.10)", 3.0f, "class=\"query-shape\" data-kind=\"radius-query\"");
        canvas.PointMark(query, "#eab308", 6.0f, "class=\"query-point\" data-kind=\"query-point\"");
        canvas.Label(query, "query", "#e2e8f0", "class=\"query-label\"");

        for (const KDPoint& point : points)
        {
            canvas.PointMark(point.Position, "#38bdf8", 5.0f, "class=\"kd-point\" data-kind=\"kd-point\" data-id=\"" + std::to_string(point.ID) + "\" tabindex=\"0\"");
            canvas.Label(point.Position, std::to_string(point.ID), "#e2e8f0", "class=\"kd-label\"");
        }

        if (nearest)
        {
            canvas.Line(query, nearest->Position, "#ef4444", 3.0f, "", "class=\"nearest-line\" data-kind=\"nearest\"");
            canvas.PointMark(nearest->Position, "#ef4444", 7.0f, "class=\"nearest-point\" data-kind=\"nearest\"");
        }

        std::vector<SpatialID> nearestIDs;
        for (const KDNearestResult& result : kNearest)
        {
            nearestIDs.push_back(result.ID);
        }

        std::vector<SpatialID> radiusIDs;
        for (const KDNearestResult& result : radius)
        {
            radiusIDs.push_back(result.ID);
        }

        std::ostringstream metrics;
        metrics << "nearest = " << (nearest ? std::to_string(nearest->ID) : "none")
                << "\nk nearest = [" << JoinIDs(nearestIDs) << "]"
                << "\nradius ids = [" << JoinIDs(radiusIDs) << "]"
                << "\npoint count = " << points.size();

        return Card(
            "kd-panel",
            "points",
            "KDTree nearest-neighbor queries",
            "Blue points are KD-tree samples. The red segment marks the nearest point from the yellow query point.",
            canvas,
            metrics.str());
    }

    [[nodiscard]]
    std::string BuildNavigationCard()
    {
        NavigationGraph graph;
        graph.AddNode(1, Vec3f{ -3.0f, -1.4f, 0.0f });
        graph.AddNode(2, Vec3f{ -1.4f, -0.2f, 0.0f });
        graph.AddNode(3, Vec3f{ 0.1f, -0.4f, 0.0f });
        graph.AddNode(4, Vec3f{ 1.8f, 0.4f, 0.0f });
        graph.AddNode(5, Vec3f{ 3.2f, 1.45f, 0.0f });
        graph.AddNode(6, Vec3f{ 0.0f, 1.7f, 0.0f });

        graph.AddEdge(1, 2, 1.0f);
        graph.AddEdge(2, 3, 1.0f);
        graph.AddEdge(3, 4, 1.0f);
        graph.AddEdge(4, 5, 1.0f);
        graph.AddEdge(2, 6, 2.2f);
        graph.AddEdge(6, 5, 2.2f);

        const AStarResult path =
            FindPathAStar(graph, 1, 5);

        Canvas canvas;
        for (const GraphNode& node : graph.Nodes())
        {
            for (const GraphEdge& edge : graph.EdgesFrom(node.ID))
            {
                if (edge.From < edge.To)
                {
                    canvas.Line(graph.Node(edge.From).Position, graph.Node(edge.To).Position, "#475569", 2.0f, "", "class=\"graph-edge\" data-kind=\"graph-edge\"");
                }
            }
        }

        for (std::size_t i = 1; i < path.Path.size(); ++i)
        {
            canvas.Line(
                graph.Node(path.Path[i - 1]).Position,
                graph.Node(path.Path[i]).Position,
                "#22c55e",
                5.0f,
                "",
                "class=\"path-edge\" data-kind=\"path-edge\"");
        }

        for (const GraphNode& node : graph.Nodes())
        {
            canvas.PointMark(node.Position, "#38bdf8", 6.0f, "class=\"graph-node\" data-kind=\"graph-node\" data-id=\"" + std::to_string(node.ID) + "\" tabindex=\"0\"");
            canvas.Label(node.Position, std::to_string(node.ID), "#e2e8f0", "class=\"graph-label\"");
        }

        std::ostringstream pathText;
        for (std::size_t i = 0; i < path.Path.size(); ++i)
        {
            if (i > 0)
            {
                pathText << " -> ";
            }

            pathText << path.Path[i];
        }

        std::ostringstream metrics;
        metrics << "found = " << (path.Found ? "true" : "false")
                << "\ncost = " << path.Cost
                << "\nexpanded nodes = " << path.ExpandedNodes
                << "\npath = " << pathText.str();

        return Card(
            "nav-panel",
            "graph",
            "Navigation graph and A*",
            "Gray edges are the navigation graph. Green highlights the shortest path found by A* using Euclidean heuristic.",
            canvas,
            metrics.str());
    }

    [[nodiscard]]
    std::string BuildReport()
    {
        std::ostringstream cards;
        cards << BuildBVHCard()
              << BuildRayCard()
              << BuildDynamicTreeCard()
              << BuildSpatialHashCard()
              << BuildKDTreeCard()
              << BuildNavigationCard();

        return cards.str();
    }
}

int main()
{
    const std::filesystem::path outputPath =
        std::filesystem::path(__FILE__).parent_path() / "spatial_visualizer.html";

    std::ofstream out(outputPath);
    if (!out)
    {
        std::cerr << "Failed to open " << outputPath << " for writing.\n";
        return 1;
    }

    out << R"(<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>KairoSpatial Visualizer</title>
    <style>
        :root {
            color-scheme: dark;
            font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
            background: #070b13;
            color: #dbeafe;
        }

        * {
            box-sizing: border-box;
        }

        body {
            margin: 0;
            min-height: 100vh;
            background:
                linear-gradient(180deg, rgba(15,23,42,0.88), rgba(7,11,19,1)),
                #070b13;
        }

        main {
            width: min(1240px, calc(100vw - 40px));
            margin: 0 auto;
            padding: 28px 0 40px;
        }

        header {
            display: grid;
            grid-template-columns: minmax(0, 1.4fr) minmax(260px, 0.8fr);
            gap: 24px;
            align-items: end;
            margin-bottom: 24px;
            border-bottom: 1px solid #1e293b;
            padding-bottom: 22px;
        }

        h1 {
            margin: 0 0 8px;
            font-size: clamp(2rem, 5vw, 4.4rem);
            line-height: 0.95;
            letter-spacing: 0;
            color: #f8fafc;
        }

        header p {
            margin: 0;
            max-width: 760px;
            color: #93a4b8;
            font-size: 1rem;
            line-height: 1.6;
        }

        .stack {
            border: 1px solid #1e293b;
            background: #0b1220;
            border-radius: 8px;
            padding: 14px 16px;
            font-family: ui-monospace, SFMono-Regular, Menlo, monospace;
            color: #bae6fd;
            line-height: 1.45;
            white-space: pre;
        }

        .toolbar {
            display: flex;
            flex-wrap: wrap;
            gap: 10px;
            align-items: center;
            margin: 0 0 18px;
            padding: 12px;
            border: 1px solid #1e293b;
            border-radius: 8px;
            background: #0b1220;
        }

        .toolbar button,
        .tool-button {
            min-height: 34px;
            border: 1px solid #334155;
            border-radius: 8px;
            background: #111827;
            color: #dbeafe;
            font: inherit;
            font-size: 0.88rem;
            cursor: pointer;
            padding: 0 12px;
        }

        .toolbar button[aria-pressed="true"],
        .tool-button:hover,
        .tool-button:focus-visible {
            border-color: #38bdf8;
            background: #082f49;
            color: #f8fafc;
        }

        .card-tools {
            display: flex;
            gap: 8px;
            flex-wrap: wrap;
        }

        .grid {
            display: grid;
            grid-template-columns: repeat(2, minmax(0, 1fr));
            gap: 18px;
        }

        .card {
            display: grid;
            grid-template-rows: auto auto auto;
            gap: 14px;
            border: 1px solid #1e293b;
            background: #0b1220;
            border-radius: 8px;
            padding: 16px;
            min-width: 0;
        }

        .card[hidden] {
            display: none;
        }

        .card h2 {
            margin: 0 0 6px;
            font-size: 1.1rem;
            color: #f8fafc;
            letter-spacing: 0;
        }

        .card p {
            margin: 0;
            color: #94a3b8;
            line-height: 1.45;
            font-size: 0.92rem;
        }

        svg {
            width: 100%;
            height: auto;
            display: block;
            border: 1px solid #111827;
            border-radius: 8px;
            overflow: hidden;
        }

        pre {
            margin: 0;
            padding: 12px;
            overflow: auto;
            border-radius: 8px;
            background: #020617;
            color: #c4b5fd;
            font-size: 0.84rem;
            line-height: 1.45;
        }

        @media (max-width: 900px) {
            header,
            .grid {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <main>
        <header>
            <div>
                <h1>KairoSpatial Visualizer</h1>
                <p>Generated from the real C++ modules. Every panel uses KairoMath vectors, KairoGeometry primitives, and KairoSpatial acceleration/query structures.</p>
            </div>
            <div class="stack">KairoSpatial
  -> KairoGeometry
      -> KairoMath</div>
        </header>
        <nav class="toolbar" aria-label="Visualizer panels">
            <button type="button" data-filter="all" aria-pressed="true">All</button>
            <button type="button" data-filter="static" aria-pressed="false">Static acceleration</button>
            <button type="button" data-filter="dynamic" aria-pressed="false">Dynamic tree</button>
            <button type="button" data-filter="grid" aria-pressed="false">Grid</button>
            <button type="button" data-filter="points" aria-pressed="false">KD points</button>
            <button type="button" data-filter="graph" aria-pressed="false">Navigation</button>
        </nav>
        <section class="grid">
)";

    out << BuildReport();

    out << R"(        </section>
    </main>
</body>
</html>
)";

    std::cout << "Wrote " << outputPath << "\n";
    return 0;
}
