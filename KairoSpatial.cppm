// TEACHING MAP — KairoSpatial.cppm
// Umbrella C++23 exports for BVH, dynamic broadphase, spatial trees, query types and
// navigation. PhysicsEngine and RayTracer import these acceleration APIs; they must still
// independently validate their outputs.
// See READING_ORDER.md for the query path and downstream KairoEngine consumers.

export module Kairo.Foundation.Spatial;

export import Kairo.Foundation.Spatial.Types;
export import Kairo.Foundation.Spatial.BVH;
export import Kairo.Foundation.Spatial.BVHBuilder;
export import Kairo.Foundation.Spatial.BVHTraversal;
export import Kairo.Foundation.Spatial.Query;
export import Kairo.Foundation.Spatial.DynamicAABBTree;
export import Kairo.Foundation.Spatial.SweepAndPrune;
export import Kairo.Foundation.Spatial.SpatialHash;
export import Kairo.Foundation.Spatial.Octree;
export import Kairo.Foundation.Spatial.Quadtree;
export import Kairo.Foundation.Spatial.KDTree;
export import Kairo.Foundation.Spatial.NavigationGraph;
export import Kairo.Foundation.Spatial.AStar;
export import Kairo.Foundation.Spatial.NavMesh;
export import Kairo.Foundation.Spatial.CrowdAvoidance;
export import Kairo.Foundation.Spatial.Debug;
