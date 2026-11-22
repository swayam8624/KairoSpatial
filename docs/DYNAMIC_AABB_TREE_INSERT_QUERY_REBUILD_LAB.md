# Dynamic AABB tree laboratory: proxies, fat bounds and cache invalidation

Open [DynamicAABBTree.cppm](../DynamicAABBTree.cppm) and [KairoPhysicsEngine/Broadphase.cppm](https://github.com/swayam8624/KairoPhysicsEngine/blob/main/Broadphase.cppm). This lesson explains the dynamic tree as a data structure, an algorithm and a *non-owning acceleration cache*. It does not store physics bodies, mesh triangles or exact collision contacts.

## 1. Never confuse the three identities

A **SpatialID** is the caller-supplied identity of a primitive. A **SpatialIndex** is a temporary index into the dynamic tree's internal node array. A **PhysicsEngine ColliderID** identifies a collider record in PhysicsWorld.

The tree's Insert returns a SpatialIndex handle for the new leaf, while each leaf stores the SpatialID it represents. PhysicsEngine's BroadphaseWorld keeps the mapping ColliderID → SpatialIndex so it can update or remove the tree proxy when the authoritative collider changes.

The identity is why a successful tree query returns both UserID values and internal primitive indices. Persisting a SpatialIndex instead of a stable ColliderID is incorrect: a rebuild can allocate the same primitive at a different node slot.

## 2. The geometry of tight and fat AABBs

A tight axis-aligned bounding box has componentwise minimum and maximum vectors, enclosing a primitive at its **current** transform. The tree also stores an expanded, or fat, AABB by padding the tight bounds with a configured margin. That padding trades additional false-positive traversal for fewer tree structural edits when bodies move slightly.

**Update(proxy,newBounds)** follows two cases:

- If the new tight box fits inside the stored fat box, replace only TightBounds and return false. Parent fat boxes remain valid; no tree restructuring is needed.
- Otherwise, remove the leaf from its parent, recompute fat padding around newBounds, insert the leaf into the hierarchy and return true.

The second case preserves the allocated leaf proxy slot, but may allocate/free internal parent nodes. Removing or rebuilding the entire tree has different handle lifetime rules.

**Exercise:** choose a 1D illustrative tight interval [0,1] padded by 0.1 to fat interval [−0.1,1.1]. Move tight interval to [0.05,1.05], which stays contained and needs no reinsert. Move it to [0.5,1.5], which leaves the old fat interval and requires reinsertion. Repeat in three dimensions componentwise.

## 3. Inserting a leaf: greedy surface-area cost

**Insert** allocates a leaf with the supplied UserID, layer mask, tight box and expanded fat box, then invokes **InsertLeaf**. For a nonempty tree, InsertLeaf descends until it reaches a leaf sibling. At each internal node it considers the increase in union surface area if the new box joined the left versus right subtree.

It chooses the cheaper local branch, creates a new internal parent combining the selected sibling and new leaf, repairs the parent links and runs **RefitAncestors** to recompute fat-box unions and heights up to root.

This is a greedy local heuristic; it does not guarantee a globally minimum-area BVH or balanced tree. The cost is sensitive to insertion history, object motion and distribution of AABB sizes.

**Exercise:** draw three widely separated AABBs A, B and C. Insert A and B, then C, inspect which sibling the cost heuristic selects and identify the new root's enclosing fat bounds. Reverse insertion order and compare tree height and area ratio.

## 4. Queries prune internal fat boxes but test tight leaves

**QueryAABB**, **QuerySphere** and **QueryRay** use an explicit stack. They reject an entire subtree if its fat box cannot intersect the query. Only when reaching a leaf do they test the tighter current bounds and layer mask.

This distinction matters: because internal fat boxes conservatively enclose descendants, pruning is safe even after many small leaf updates. Because leaf queries recheck TightBounds, they avoid returning every false positive introduced by the expansion margin.

The result remains a **candidate**, not a confirmed contact. For example, a ray intersecting a sphere's AABB can miss the actual curved sphere. PhysicsWorld's geometry-specific RaycastCollider solves the true shape intersection after the broadphase reduction.

The current QueryRay sorts IDs for deterministic output; the order does not imply distance ordering. **RaycastBroadphase** reports closest AABB entry only, while the callback-based **Raycast** overload can invoke caller-supplied exact primitive intersection and tighten the maximum ray distance.

**Exercise:** create a sphere with world AABB [−1,1] on x/y/z. Choose a ray through one far AABB corner that misses the sphere. Show why QueryRay can include it, but the exact sphere ray test should not.

## 5. Pair generation is a different workload

**ComputePairs** iterates live leaf nodes and calls QueryAABB against the same tree for each tight box. It excludes self-ID pairs, canonicalizes pair order and sorts/deduplicates the final list. This is a correctness-oriented design whose work scales approximately with **number of leaves × query cost**, plus pair sorting.

Dense scenes with large overlapping boxes can still produce many candidate pairs. The pair count is an output-size lower bound, but tree traversal and redundant queries add overhead. A more advanced temporal broadphase might track moved leaves and incremental pairs instead of requerying every leaf, but that requires a separate algorithm and correctness tests.

BroadphaseWorld also pairs **finite** colliders against **infinite** planes outside this dynamic tree. Because two unbounded planes have no finite overlap box, their pair handling is not an ordinary dynamic-tree leaf query.

**Exercise:** compare a sparse set of N distant boxes and N boxes all occupying the same region. Record visited nodes, tested primitives, duplicate candidate counts and final pairs. Do not compare only elapsed time without confirming identical candidate sets to an exact AABB oracle.

## 6. RebuildBottomUp is a handle-breaking operation

**RebuildBottomUp** gathers live leaf UserIDs, TightBounds and layer masks, clears the current hierarchy and reinserts the saved primitive records. The stable IDs survive. **The old SpatialIndex proxy handles do not.**

Any external ColliderID → SpatialIndex cache must be rebuilt or refreshed after a tree rebuild, even if some new node slots happen to receive the same numerical index. Calling Update with a stale index may fail or silently refer to a different primitive.

This is a strict ownership boundary: the dynamic tree owns node indices, while PhysicsWorld owns collider identities. Read [the PhysicsEngine broadphase adapter](https://github.com/swayam8624/KairoPhysicsEngine/blob/main/Broadphase.cppm) to see who needs invalidation when a tree is reconstructed.

**Exercise:** insert two primitives, save proxy handles, rebuild, then find the primitives by stable UserID rather than assuming saved proxy indices still identify them. Add an integration test proving cache refresh across rebuild.

## 7. Why this allocator deserves a stress test

**AllocateNode** currently appends a new node to its vector. **FreeNode** resets a node slot to an unallocated state, but there is no implemented reuse of freed indices in AllocateNode. Removing/reinserting leaves can therefore leave holes and increase total node-array length even when the number of *live* leaves stays modest.

That is a source-observed scalability risk, not measured evidence of a leak or a guaranteed performance regression. To change it safely, a free-list allocator would need to preserve parent/child invariants, invalidate stale handles robustly and prove that an old proxy cannot become valid for the wrong object. A generation counter would be worth studying.

**Exercise:** repeatedly move one collider outside its fat bounds, then inspect live leaf count, total node-array size, allocated node count and query timing. Compare against a candidate freelist implementation under the same deterministic motion trace.

## 8. Verify before optimizing

**Validate** checks the hierarchy's structural invariants. **Stats** reports height, maximum balance, area ratio, allocated nodes and leaf count. Use these in randomized insertion, movement and deletion tests before trusting any performance claims.

Read [SpatialTests.cpp](../tests/SpatialTests.cpp) and [ScaleStressTests.cpp](../tests/ScaleStressTests.cpp), along with [SOURCE_READING_ORDER.md](SOURCE_READING_ORDER.md). For each test save source SHA, random seed, object distribution, number of updates per frame, proportion of large movers, layer-filter distribution, exact candidate-pair oracle and runtime metrics.

A fast tree that silently omits an overlapping AABB is wrong; a slower tree with an identical candidate set can still be a valid optimization baseline.

*Status: this tutorial is derived from source. No new benchmark, stress run or production allocator change is claimed.*
