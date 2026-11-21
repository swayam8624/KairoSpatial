# KairoSpatial: linked source reading order

Study spatial query types first, then static BVH construction and traversal, dynamic AABB tree and sweep-and-prune broadphase, spatial hashing/trees, and finally graph navigation. KairoPhysicsEngine asks this library for broadphase candidates; KairoRayTracer asks it for acceleration and ray queries. The return value of a broadphase query is not a physical collision.

## API interactions to understand

Follow the relationship between AABB bounds, ray-hit queries, BVH node layout and traversal, temporal coherence of dynamic broadphase, and pathfinding costs. Benchmark tree construction separately from query time; test false negatives against a brute-force oracle.

## Read these files in sequence (23)

1. [`KairoSpatial.cppm`](KairoSpatial.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
2. [`SpatialTypes.cppm`](SpatialTypes.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
3. [`BVH.cppm`](BVH.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
4. [`BVHBuilder.cppm`](BVHBuilder.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
5. [`BVHTraversal.cppm`](BVHTraversal.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
6. [`DynamicAABBTree.cppm`](DynamicAABBTree.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
7. [`SweepAndPrune.cppm`](SweepAndPrune.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
8. [`SpatialHash.cppm`](SpatialHash.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
9. [`Octree.cppm`](Octree.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
10. [`Quadtree.cppm`](Quadtree.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
11. [`KDTree.cppm`](KDTree.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
12. [`NavigationGraph.cppm`](NavigationGraph.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
13. [`AStar.cppm`](AStar.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
14. [`CrowdAvoidance.cppm`](CrowdAvoidance.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
15. [`NavMesh.cppm`](NavMesh.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
16. [`SpatialDebug.cppm`](SpatialDebug.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
17. [`SpatialQuery.cppm`](SpatialQuery.cppm) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
18. [`tests/CrowdAvoidanceTests.cpp`](tests/CrowdAvoidanceTests.cpp) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
19. [`tests/NavMeshTests.cpp`](tests/NavMeshTests.cpp) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
20. [`tests/ScaleStressTests.cpp`](tests/ScaleStressTests.cpp) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
21. [`tests/SpatialTests.cpp`](tests/SpatialTests.cpp) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
22. [`visual/server.py`](visual/server.py) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.
23. [`visual/SpatialVisualizer.cpp`](visual/SpatialVisualizer.cpp) — inspect its imports, data ownership, validation and exported functions; follow its use from previous modules into downstream modules and tests.

## Verification and reuse

Read [README.md](README.md) and [CMakeLists.txt](CMakeLists.txt) for build/module boundaries where present. Test invalid inputs, lifetime errors, domain boundaries and the exact source revision being used by [KairoGameEngine](https://github.com/swayam8624/KairoGameEngine). A source file list or module compile is not full integration acceptance. Original software is shared under [MIT](LICENSE) for learning, reuse and continued development; external dependencies and assets retain their own terms. Per-function annotations are a subsequent documentation pass.
