// TEACHING ENTRY POINT: The spatial library answers which primitives
// warrant expensive tests. It never owns PhysicsWorld's bodies/colliders and
// never converts a bounding-box candidate into a physical collision.
//
// Data flows: Geometry/Math supply primitive bounds and coordinates.
// BVH + Builder + Traversal index mostly static scene primitives; the
// caller supplies actual ray/triangle hit callbacks. DynamicAABBTree,
// SweepAndPrune and SpatialHash are alternatives for dynamic overlap work;
// PhysicsEngine::BroadphaseWorld currently uses DynamicAABBTree. Octree,
// Quadtree and KDTree cover hierarchical spatial/range queries.
// NavigationGraph/AStar/NavMesh/CrowdAvoidance form separate pathfinding and
// local-steering layers rather than rigid-body constraints.
//
// C++23 `export import` re-exports the narrower named interfaces; it is
// not a build or execution order. Import a single module when a consumer
// needs only one data structure. Read docs/SOURCE_READING_ORDER.md and the
// PhysicsEngine collision walkthrough to follow actual cross-repo calls.
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
