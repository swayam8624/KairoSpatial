# Spatial laboratory: two searches, one for surfaces and one for graph routes

Read [BVHTraversal.cppm](../BVHTraversal.cppm), [BVHBuilder.cppm](../BVHBuilder.cppm), [AStar.cppm](../AStar.cppm), [NavigationGraph.cppm](../NavigationGraph.cppm) and the existing [dynamic AABB tree lab](DYNAMIC_AABB_TREE_INSERT_QUERY_REBUILD_LAB.md). BVH ray queries and A* route search share the idea of avoiding irrelevant work, but their **correctness conditions are different**.

## 1. BVH culls boxes, it never creates a triangle hit

A bounding-volume hierarchy stores unions of spatial primitive AABBs in internal nodes. During Raycast, the traversal starts at the root and tests the ray against each node's bounding box. A failed node-box test safely prunes all primitives contained in that node, assuming the BVH was built from current correct bounds.

When reaching a leaf, the traversal checks layer masks and calls a **caller-owned exact intersection callback** for each candidate UserIndex. KairoRayTracer supplies a sphere/triangle intersection there; PhysicsEngine supplies its own geometry policy. KairoSpatial itself does not know the material or underlying triangle vertices from an AABB alone.

**Counterexample:** a triangle fits in box [0,1]×[0,1]×{0}, but a downward ray at (0.9,0.9) crosses the box and misses the right triangle whose vertices are (0,0), (1,0), (0,1). AABB candidate success does not permit a fake SurfaceHit.

## 2. Closest-hit pruning

Raycast carries closestDistance, initially the supplied maxDistance. Each accepted callback hit with valid nonnegative distance at or below that limit updates the closest hit and tightens the bound for subsequent node checks.

When two child boxes are hit, the traversal places the farther child on its LIFO stack first, so the nearer child is visited immediately. Finding a close actual surface hit early can prune more distant subtrees. This is an ordering optimization, not a semantic change to the nearest accepted primitive.

The result's Hits list retains records accepted during traversal, potentially including hits superseded by a later closer hit. Use Closest for the nearest answer; do not mistake Hits for every surface intersection along the ray.

## 3. Any-hit and overlap are distinct queries

RaycastAny returns as soon as a leaf callback confirms a valid hit. A shadow ray needs only existence of a blocker before its finite light distance, so a nearest-hit search with full material data is unnecessary. Scene::Occluded in KairoRayTracer uses this path.

QueryAABB returns candidates whose BVH boxes overlap the query AABB under a layer mask. It does not evaluate true triangle- or sphere-specific contact. The physics narrow phase must receive the candidates and test geometry before applying impulses.

**Exercise:** create two triangles with overlapping AABBs but one missing the ray. Compare BVH Raycast Closest, RaycastAny, and QueryAABB. Then confirm the exact primitive callback prevents a phantom shadow.

## 4. Graph A* is a cost problem, not geometry collision

NavigationGraph stores directed edges with nonnegative travel costs and node positions. AStar::FindPathAStar returns a sequence of NavigationNodeIDs, a total path cost and ExpandedNodes. It does **not** move a character or update collision bodies.

A* evaluates f(n)=g(n)+h(n), where g is best-known start-to-n cost and h is a lower-bound estimate to goal. Raw Euclidean distance might **overestimate** path cost if edges are unusually cheap relative to coordinate distance (teleport or fast transit). An overestimate can cause A* to choose a suboptimal route.

The current implementation computes the minimum edge cost per unit of nonzero Euclidean edge distance and uses:

\[
h(n)=s\,\|position(n)-position(goal)\|,\quad
s=\min_{edges(u,v),\,distance>0}\frac{cost(u,v)}{distance(u,v)}.
\]

Because straight-line distances obey triangle inequality and edge cost is at least s times Euclidean edge length, this heuristic is consistent under the graph's nonnegative, finite cost contract. A zero-cost long-distance edge yields s=0, reducing A* to Dijkstra.

**Worked example:** node A at x=0, B at x=10 and goal C at x=20. Edges A→B=2, B→C=2, A→C=10. Minimum ratio is 2/10=0.2. Heuristic at A is 0.2×20=4, no more than true cheapest cost 4. A heuristic using unscaled Euclidean 20 would exceed actual cost and fail admissibility.

## 5. Why duplicate heap entries are fine

std::priority_queue does not offer decrease-key. If a cheaper route to node V is found, AStar updates gScore[V] and pushes a new entry. The old higher-cost entry remains but is skipped when popped because its saved Cost does not equal current gScore.

cameFrom stores the best known predecessor for each improved node. When goal is popped under a consistent heuristic, reconstruction follows cameFrom backward from goal to start, then reverses the path. Missing nodes or unreachable goals produce a not-found result rather than a fabricated straight line.

The code rejects overflowing accumulated path costs and falls back to zero heuristic on nonfinite coordinate-distance calculations, protecting queue order from NaN.

**Exercise:** create A→B cost 5, A→C cost 1, C→B cost 1, B→D cost 1. Initially B may enter with g=5, later it is improved to g=2 through C. Identify which stale heap entry must be ignored, and verify final path A,C,B,D cost 3.

## 6. Navigation, crowd avoidance and physics must not be conflated

A* finds a route through graph connectivity. A NavMesh may additionally represent walkable polygon regions/corridors. CrowdAvoidance adjusts local moving-agent velocities near neighbors. PhysicsEngine resolves actual body contacts with collision geometry and solver constraints.

A route can be graph-optimal yet blocked by a newly spawned obstacle. The consumer must update path planning or steering as the world changes. BVH acceleration and A* are both performance-oriented searches, but only the first deals directly with bounding geometric volumes.

## 7. Verification

Read [SpatialTests.cpp](../tests/SpatialTests.cpp), [NavMeshTests.cpp](../tests/NavMeshTests.cpp) and [CrowdAvoidanceTests.cpp](../tests/CrowdAvoidanceTests.cpp). For BVH compare exact callback hit sets, nearest distance and any-hit existence against brute force across rays, layers and degenerate boxes. For A* compare returned path cost against Dijkstra on directed graphs with zero-cost edges, detours, unreachable goals and nonuniform geometry/cost ratios.

Record node/primitive count, actual source SHA, compiler, query distribution, work counters, correctness oracle and elapsed time separately. A smaller node expansion count is not equivalent to a faster runtime if heap overhead dominates; query speed alone is not a physics correctness claim.

*Source-derived teaching lab. No C++ test execution or benchmark is claimed.*
