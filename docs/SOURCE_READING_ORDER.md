# Teaching source map — KairoSpatial

Acceleration structures and spatial query API between geometric primitives and higher-level engines.

Follow source links sequentially. For each API/function, identify its input contract, owning data structure, called modules, failure modes, outputs, test and downstream consumers.

## Read in this order

1. [`CMakeLists.txt`](../CMakeLists.txt)
2. [`KairoSpatial.cppm`](../KairoSpatial.cppm)
3. [`SpatialTypes.cppm`](../SpatialTypes.cppm)
4. [`BVH.cppm`](../BVH.cppm)
5. [`BVHBuilder.cppm`](../BVHBuilder.cppm)
6. [`BVHTraversal.cppm`](../BVHTraversal.cppm)
7. [`DynamicAABBTree.cppm`](../DynamicAABBTree.cppm)
8. [`SweepAndPrune.cppm`](../SweepAndPrune.cppm)
9. [`SpatialHash.cppm`](../SpatialHash.cppm)
10. [`Octree.cppm`](../Octree.cppm)
11. [`KDTree.cppm`](../KDTree.cppm)
12. [`SpatialQuery.cppm`](../SpatialQuery.cppm)
13. [`NavMesh.cppm`](../NavMesh.cppm)
14. [`AStar.cppm`](../AStar.cppm)
15. [`tests`](../tests)
16. [`visual`](../visual)

## How this repository interacts with others

PhysicsEngine broadphase uses spatial candidates; RayTracer uses BVH traversal; Geometry owns exact intersection mathematics. Inspect false positives, bounds updates and algorithmic complexity separately.

## Verification and study rules

- Read the public C++23 module/entrypoint first, then its implementation and tests.
- Explain invariants, ownership, error states, concurrency or numerical assumptions in comments, not trivial syntax.
- A passing CI job covers only tests actually run on that commit/host; source presence is not runtime certification.
- Check [STATUS.md](../STATUS.md) and [STATUS.yaml](../STATUS.yaml) for current explicit verification limitations.
- Original work may be modified and extended under the repository license; third-party content retains upstream terms.
