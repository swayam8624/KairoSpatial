# Teaching source map — KairoSpatial

Acceleration structures and spatial query API between geometric primitives and higher-level engines.

Follow source links sequentially. For each API/function, identify its input contract, owning data structure, called modules, failure modes, outputs, test and downstream consumers.

**Worked exercise:** [Dynamic AABB tree proxy identity, fat-box update, ray query, memory growth and rebuild invalidation](DYNAMIC_AABB_TREE_INSERT_QUERY_REBUILD_LAB.md). It explains why candidate sets must be checked against an independent AABB overlap oracle.

**Second worked lab:** [BVH exact-hit callbacks, near-first pruning, any-hit occlusion and A* admissibility/heap reconstruction](BVH_RAYCAST_AND_ASTAR_SEARCH_LAB.md). It compares these searches with independent brute-force and Dijkstra oracles.

## Read in this order

Spatial indices answer *where to look first*, not necessarily *what collides*.
Follow this sequence to understand the API and the implementation layers.
See [KairoGeometry's guide](https://github.com/swayam8624/KairoGeometry/blob/main/docs/SOURCE_READING_ORDER.md)
for primitive intersection rules and the
[KairoPhysicsEngine collision guide](https://github.com/swayam8624/KairoPhysicsEngine/blob/main/docs/SOURCE_READING_ORDER.md)
for the simulation consumer.

### Part A — Common contracts and static scene indexing

1. [`CMakeLists.txt`](../CMakeLists.txt): C++23 modules, dependent Kairo repositories and test targets.
2. [`KairoSpatial.cppm`](../KairoSpatial.cppm): umbrella exports versus narrower imports.
3. [`SpatialTypes.cppm`](../SpatialTypes.cppm): `SpatialID`, `SpatialIndex`, AABBs, layers and query-result conventions.
4. [`BVH.cppm`](../BVH.cppm): node topology, leaves and immutable-ish primitive ranges.
5. [`BVHBuilder.cppm`](../BVHBuilder.cppm): partitioning and tree construction heuristics.
6. [`BVHTraversal.cppm`](../BVHTraversal.cppm): box pruning and caller-owned primitive hit callbacks.
7. [`SpatialQuery.cppm`](../SpatialQuery.cppm): public query operations and result construction.

**Exercise:** build a BVH from a few boxes, compare candidate IDs with
brute-force AABB checks, and explain why an accepted leaf still needs a
shape intersection callback for a true ray hit.

### Part B — Dynamic broadphase trade-offs

8. [`DynamicAABBTree.cppm`](../DynamicAABBTree.cppm): tight vs fat AABBs, proxy handles and dynamic updates.
9. [`SweepAndPrune.cppm`](../SweepAndPrune.cppm): sorted axis intervals and candidate checks.
10. [`SpatialHash.cppm`](../SpatialHash.cppm): grid cell mapping and multi-cell candidate work.
11. [`Octree.cppm`](../Octree.cppm): hierarchical 3D subdivision.
12. [`Quadtree.cppm`](../Quadtree.cppm): hierarchical 2D subdivision.
13. [`KDTree.cppm`](../KDTree.cppm): spatial split axis and range/nearest search.

**Exercise:** move a collider inside its fat AABB and then beyond it,
checking `DynamicAABBTree::Update` return values. Record tree height,
candidate pairs, and false positives. Do not assume a faster broadphase
also improves narrowphase accuracy.

**Handle warning:** `RebuildBottomUp` reinserts leaves with new allocation
handles. Rebuild any external `SpatialIndex` maps after calling it; the
externally supplied `SpatialID` does not serve as a tree node index.

### Part C — Graph and crowd-navigation algorithms

14. [`NavigationGraph.cppm`](../NavigationGraph.cppm): vertices, edges and graph topology.
15. [`AStar.cppm`](../AStar.cppm): heuristic search and path-cost assumptions.
16. [`NavMesh.cppm`](../NavMesh.cppm): walkable regions and navigation topology.
17. [`CrowdAvoidance.cppm`](../CrowdAvoidance.cppm): local steering constraints.
18. [`SpatialDebug.cppm`](../SpatialDebug.cppm): inspectable index/graph diagnostics.

**Exercise:** compare a graph shortest path with local avoidance.
Neither is the same as collision response in a rigid-body physics solver.

### Part D — Tests, stress and visualization

19. [`tests/SpatialTests.cpp`](../tests/SpatialTests.cpp): reference comparisons for trees, grid and queries.
20. [`tests/ScaleStressTests.cpp`](../tests/ScaleStressTests.cpp): scale-sensitive behavior.
21. [`tests/NavMeshTests.cpp`](../tests/NavMeshTests.cpp): graph/navmesh constraints.
22. [`tests/CrowdAvoidanceTests.cpp`](../tests/CrowdAvoidanceTests.cpp): local crowd behavior.
23. [`visual/SpatialVisualizer.cpp`](../visual/SpatialVisualizer.cpp): C++ visualization entry.
24. [`visual/server.py`](../visual/server.py): optional local server bridge.

**Record:** tree node count/height, candidate count, validated shape hits,
time per scenario, configuration, and host. A benchmark result is only
meaningful with the actual workload and method attached.

## How this repository interacts with others

PhysicsEngine broadphase uses spatial candidates; RayTracer uses BVH traversal; Geometry owns exact intersection mathematics. Inspect false positives, bounds updates and algorithmic complexity separately.

## Verification and study rules

- Read the public C++23 module/entrypoint first, then its implementation and tests.
- Explain invariants, ownership, error states, concurrency or numerical assumptions in comments, not trivial syntax.
- A passing CI job covers only tests actually run on that commit/host; source presence is not runtime certification.
- Check [STATUS.md](../STATUS.md) and [STATUS.yaml](../STATUS.yaml) for current explicit verification limitations.
- Original work may be modified and extended under the repository license; third-party content retains upstream terms.


## Complete source-file inventory

After the curated teaching path, follow this deterministic inventory of first-party source, scripts, shaders, and tests. Grouping is a suggested learning order, not an indication that every function is already annotated. Vendored dependencies, ROMs, binaries, data, media, and generated evidence are excluded.

### Entry points and build (1)

1. [`CMakeLists.txt`](../CMakeLists.txt)

### Interfaces and modules (17)

1. [`AStar.cppm`](../AStar.cppm)
2. [`BVH.cppm`](../BVH.cppm)
3. [`BVHBuilder.cppm`](../BVHBuilder.cppm)
4. [`BVHTraversal.cppm`](../BVHTraversal.cppm)
5. [`CrowdAvoidance.cppm`](../CrowdAvoidance.cppm)
6. [`DynamicAABBTree.cppm`](../DynamicAABBTree.cppm)
7. [`KairoSpatial.cppm`](../KairoSpatial.cppm)
8. [`KDTree.cppm`](../KDTree.cppm)
9. [`NavigationGraph.cppm`](../NavigationGraph.cppm)
10. [`NavMesh.cppm`](../NavMesh.cppm)
11. [`Octree.cppm`](../Octree.cppm)
12. [`Quadtree.cppm`](../Quadtree.cppm)
13. [`SpatialDebug.cppm`](../SpatialDebug.cppm)
14. [`SpatialHash.cppm`](../SpatialHash.cppm)
15. [`SpatialQuery.cppm`](../SpatialQuery.cppm)
16. [`SpatialTypes.cppm`](../SpatialTypes.cppm)
17. [`SweepAndPrune.cppm`](../SweepAndPrune.cppm)

### Implementation and other source (2)

1. [`visual/server.py`](../visual/server.py)
2. [`visual/SpatialVisualizer.cpp`](../visual/SpatialVisualizer.cpp)

### Verification and tests (4)

1. [`tests/CrowdAvoidanceTests.cpp`](../tests/CrowdAvoidanceTests.cpp)
2. [`tests/NavMeshTests.cpp`](../tests/NavMeshTests.cpp)
3. [`tests/ScaleStressTests.cpp`](../tests/ScaleStressTests.cpp)
4. [`tests/SpatialTests.cpp`](../tests/SpatialTests.cpp)

Snapshot totals: 24 first-party source/build/test files, 33 tracked files overall. Re-audit when files are renamed or added.
