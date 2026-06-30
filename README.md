# KairoSpatial

`KairoSpatial` is the engine spatial acceleration foundation. It keeps static
scene queries, dynamic broadphase, world partitioning, nearest-neighbor lookup,
navigation graph search, and debug extraction behind one consistent spatial
query language.

## Module Surface

```cpp
import Kairo.Foundation.Spatial;
```

Narrow imports are also available:

```cpp
import Kairo.Foundation.Spatial.Types;
import Kairo.Foundation.Spatial.BVH;
import Kairo.Foundation.Spatial.BVHBuilder;
import Kairo.Foundation.Spatial.BVHTraversal;
import Kairo.Foundation.Spatial.Query;
import Kairo.Foundation.Spatial.DynamicAABBTree;
import Kairo.Foundation.Spatial.SweepAndPrune;
import Kairo.Foundation.Spatial.SpatialHash;
import Kairo.Foundation.Spatial.Octree;
import Kairo.Foundation.Spatial.Quadtree;
import Kairo.Foundation.Spatial.KDTree;
import Kairo.Foundation.Spatial.NavigationGraph;
import Kairo.Foundation.Spatial.AStar;
import Kairo.Foundation.Spatial.Debug;
```

## Included Systems

```text
Static acceleration:
  - Median BVH
  - SAH BVH
  - Morton-ordered BVH build path
  - Raycast, RaycastAny, AABB, sphere, frustum, point queries

Dynamic broadphase:
  - Dynamic AABB tree with fat bounds, insert/remove/update, pair generation
  - Sweep and prune with update and pair generation
  - Spatial hash with cell, AABB, and radius queries

Partitioning and lookup:
  - Loose octree with subdivision, collapse, AABB/sphere/frustum/ray queries
  - Quadtree for 2D AABB queries
  - KDTree with nearest, k-nearest, and radius search

Navigation:
  - NavigationGraph node/edge storage
  - A* shortest path with Euclidean heuristic

Debug:
  - BVH boxes
  - Dynamic tree bounds
  - Octree cells
  - Ray traversal line data
```

## Shared Query Types

All systems use the same shared records from `SpatialTypes.cppm`:

```text
SpatialID
SpatialIndex
SpatialPrimitive
SpatialReference
SpatialAABB
SpatialRayHit
SpatialRaycastResult
SpatialOverlapResult
SpatialBuildSettings
SpatialBuildStats
SpatialUpdateStats
```

`SpatialAABB` and `SpatialRay` intentionally reuse KairoGeometry's `AABBf` and
`Rayf`, so Spatial does not duplicate primitive math or create a second geometry
contract.

## Foundation Dependency Chain

`KairoSpatial` is the top layer in the current foundation stack:

```text
KairoSpatial
  -> KairoGeometry
      -> KairoMath
```

`KairoSpatial` links `KairoGeometry` through CMake and imports geometry modules
for `AABBf`, `Rayf`, `Sphere`, and `Frustumf`. `KairoGeometry` links
`KairoMath` for vectors, matrices, transforms, and scalar helpers. The
dependency is one-way: Math does not call Geometry, and Geometry does not call
Spatial, so the foundation layers remain acyclic.

## Validation

Build with the same module-capable LLVM toolchain used by the other Kairo
foundation packages:

```bash
cmake -S . -B build -G Ninja -DCMAKE_CXX_COMPILER=/opt/homebrew/opt/llvm/bin/clang++
cmake --build build
ctest --test-dir build --output-on-failure
```

Spatial tests compare accelerated queries against brute-force checks wherever a
direct brute-force baseline is meaningful.

## Visualizer

The visualizer is a generated interactive HTML/SVG debugger backed by the real
C++ modules:

```bash
cmake --build build --target KairoSpatialVisual
./build/KairoSpatialVisual
python3 visual/server.py
```

Open:

```text
http://127.0.0.1:8093/spatial_visualizer.html
```

The report renders:

```text
BVH hierarchy and AABB queries
BVH ray traversal
Dynamic AABB tree broadphase stats and layer filtering
Spatial hash grid/radius queries
KDTree nearest-neighbor queries
NavigationGraph + A* paths
```

Interactive controls include:

```text
Panel filters for static, dynamic, grid, point, and graph views
Layer-mask toggles for primitives
Debug-overlay toggles for BVH nodes, query shapes, and grid cells
Entity inspection on hover, tap, and keyboard focus
Per-panel focus, zoom, reset, traversal animation, and SVG export
Live visible-entity summary counters
```
