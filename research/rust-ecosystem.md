# Rust CAD Ecosystem Reference

Analysis of existing Rust geometry kernels and relevant crates for informing
OpenSolid's architecture decisions.

---

## 1. Truck (ricosjp/truck) — 1,460 stars

The most complete Rust B-rep kernel. MIT/Apache dual-licensed.

### Crate Structure

```
truck/
├── truck-base         — Foundation: cgmath vectors, tolerance, bounding boxes, Newton's method
├── truck-geotrait     — Abstract traits: ParametricCurve, ParametricSurface, BoundedCurve, etc.
├── truck-geometry     — Concrete: BSplineCurve, BSplineSurface, NurbsCurve, NurbsSurface, KnotVec
├── truck-topology     — Core graph: Vertex, Edge, Wire, Face, Shell, Solid
├── truck-modeling     — High-level API: sweep, revolve, extrude, builders
├── truck-shapeops     — Boolean operations + fillet
├── truck-stepio       — STEP file I/O (via ruststep)
├── truck-meshalgo     — Meshing/tessellation algorithms
├── truck-polymesh     — Polygon mesh data structures
├── truck-assembly     — Assembly/DAG structures
└── truck-derivers     — Proc macros for deriving geometry traits
```

### Topology Architecture

**Identity via `Arc<Mutex<T>>`**:
- Each topology entity wraps its geometry in `Arc<Mutex<P/C/S>>`
- Identity is pointer-based (`Arc::as_ptr`), not value-based
- Cloning an entity gives a shared reference to the same underlying data
- Equality is identity (pointer comparison) — this is how topology sharing works

**Structs:**
```rust
Vertex<P>     = Arc<Mutex<P>>                          // Just a wrapped point
Edge<P,C>     = (Vertex, Vertex, bool, Arc<Mutex<C>>) // Two vertices + sense + curve
Wire<P,C>     = VecDeque<Edge>                         // Ordered edges
Face<P,C,S>   = (Vec<Wire>, bool, Arc<Mutex<S>>)      // Boundary wires + sense + surface
Shell<P,C,S>  = Vec<Face>                              // Collection of faces
Solid<P,C,S>  = Vec<Shell>                             // Collection of shells
```

**Thread safety**: `parking_lot::Mutex` for interior mutability, `Arc` (or `rclite::Arc`) for shared ownership. Parallel iteration via `rayon`.

### Boolean Operations (truck-shapeops)

Algorithm:
1. Triangulate both solids
2. Compute surface-surface intersection curves (polyline approximation → B-spline fitting)
3. Divide faces along intersection curves
4. Classify sub-faces as "inside" or "outside" (ray casting against triangulated mesh)
5. Select faces for AND (inside both) or OR (outside both)
6. Reconstruct shells from selected faces

**Limitations**: Only transversal intersections. Tangent/coplanar faces not handled. No BSP optimization.

### STEP I/O

- Input: Uses `ruststep` to parse STEP text → builds HashMap of entities → converts to truck topology
- Output: Implements `Display` trait on types to format as STEP text
- Roundtrip tested against OpenCASCADE-generated STEP files

### Testing

- Property-based testing with `proptest` (derivative consistency of B-splines)
- Golden file roundtrip: Read STEP → Write STEP → Read again → verify mesh closure
- `assert_near!` / `assert_near2!` macros for tolerance-based comparison
- ~110 test files across all crates

---

## 2. Fornjot (hannobraun/fornjot) — 2,527 stars

The most architecturally ambitious Rust kernel. 0BSD license.

### Architecture (Current / "Old")

```
fornjot/
├── fj-core      — Main crate: topology, geometry, validation, operations, storage
├── fj-math      — Math primitives (Point, Vector, Scalar)
├── fj-export    — Export to STL, 3MF
├── fj-interop   — Interoperability types
├── fj-viewer    — Visualization (wgpu)
└── fj-window    — Window management
```

### Topology Objects

- `Vertex` — **Empty struct!** Identity comes solely from `Handle<Vertex>` (pointer-based ID)
- `HalfEdge` — References a `Handle<Curve>` + `Handle<Vertex>` (start only; end = start of next)
- `Cycle` — `ObjectSet<HalfEdge>` (ordered set forming closed loop)
- `Region` — Exterior `Handle<Cycle>` + interior cycles (holes)
- `Face` — `Handle<Surface>` + `Handle<Region>`
- `Shell` — `ObjectSet<Face>`
- `Solid` — `ObjectSet<Shell>`
- `Curve`, `Surface` — Also empty marker types; geometry stored externally

### Storage / Handle System

- **Append-only `Store<T>`** with `parking_lot::RwLock` protection
- `Handle<T>` = Arc to store + index + raw pointer. Derefs to `&T`
- Identity is pointer-based (`Handle::id()` uses `ObjectId::from_ptr`)
- Objects are **immutable once stored** — no mutation, no `Mutex`

### Geometry-Topology Separation

- Geometry stored in a separate `Geometry` struct, keyed by `Handle<Curve>` / `Handle<Vertex>`
- Curves defined as `Path` (Line or Circle) in a surface's 2D coordinate system
- Each curve can have multiple "local" representations on different surfaces

### The "new/" Experimental Module

Simpler architecture:
- `Store<T>` is just a `Vec<T>`
- `Handle<T>` is just an index + PhantomData (**Copy!**)
- Topology has embedded geometry (points stored in edges/faces)
- Orientation is explicit enum: `Nominal` / `AntiNominal`
- Much simpler than the old architecture

### Why Booleans Are Hard for Fornjot

1. Geometry separation makes intersection queries difficult
2. Only Lines and Circles supported (no NURBS) — limiting intersection curve representation
3. Strict validation: any intermediate invalid state is flagged
4. "Half-edge must have a sibling" constraint requires perfect surface stitching

### Dependencies

`nalgebra`, `parry2d-f64`, `parry3d-f64`, `robust`, `spade`, `geo`, `parking_lot`, `itertools`

---

## 3. Key Rust Crates for Computational Geometry

| Crate | Version | Purpose | Notes |
|-------|---------|---------|-------|
| **nalgebra** | 0.34 | Linear algebra | Static/dynamic matrices, transforms, decompositions. Modern, maintained. |
| **cgmath** | 0.18 | Linear algebra | Simpler API. Used by truck. Upstream deprecated. |
| **geo** | 0.33 | 2D geometry | Polygons, boolean ops (i_overlay), area, intersections. Uses `robust`. |
| **parry3d** | 0.26 | 3D collision | Shapes, distance queries, raycasting. nalgebra-based. |
| **spade** | 2.15 | Delaunay triangulation | CDT, Voronoi. Uses `robust`. Essential for tessellation. |
| **robust** | 1.2 | Exact predicates | `orient2d`, `orient3d`, `incircle`, `insphere`. Adaptive precision. |
| **curvo** | 0.1.88 | NURBS modeling | Full curves/surfaces. Interpolation, loft, sweep, intersection. nalgebra. |
| **slotmap** | — | Generational indices | Arena allocator. `SlotMap<Key, Value>`. Copy keys. |
| **argmin** | — | Optimization | Newton/BFGS. Used by curvo for curve fitting. |

### NURBS-Specific Crates

| Crate | Stars | Notes |
|-------|-------|-------|
| **curvo** | 202 | Most complete standalone NURBS library. nalgebra-based. Intersection + boolean (2D). |
| **truck-geometry** | (part of truck) | cgmath-based. BSpline + NURBS curves/surfaces. |
| **stroke** | 51 | Zero-allocation, const-generic B-spline evaluation. |
| **bspline** | 42 | Simple generic B-spline curves. |

---

## 4. Architecture Decision Analysis

### Topology Graph Strategies

| Approach | Used By | Pros | Cons |
|----------|---------|------|------|
| `Arc<Mutex<T>>` | Truck | Shared ownership, thread-safe mutation, identity via pointer | Lock contention, deadlock risk, overhead |
| Append-only Store + Handle | Fornjot (old) | Immutable objects, safe deref, no locks for reads | Can't mutate, growing memory |
| Vec + index (generational) | Fornjot (new), slotmap | Copy handles, cache-friendly, simple | Dangling handles possible |

**Recommendation for OpenSolid**: Generational indices (slotmap pattern). Reasons:
- Copy semantics for handles (no Rc/Arc overhead)
- Cache-friendly (contiguous memory)
- Simple implementation
- Generation counter catches use-after-free
- Natural snapshot/undo support (copy arena state)

### ECS vs. Traditional Hierarchy

None of the Rust CAD kernels use ECS (like bevy_ecs). All use traditional B-rep hierarchies.
Fornjot's external geometry store (keyed by handles) is the closest to a "component" pattern.

**Recommendation**: Traditional hierarchy with generational indices. ECS adds complexity
without clear benefit for CAD workloads (which are structured, not ad-hoc).

### Error Handling Patterns

| Project | Pattern |
|---------|---------|
| Truck | `try_new()` → Result, `new()` → panics, `new_unchecked()` → skips validation |
| Fornjot | Validation errors collected (not early-returned), deferred until `Layers` drops |

**Recommendation**: Result types everywhere. Rich error enums. No panics in library code.

### Math Library

| Choice | Pros | Cons |
|--------|------|------|
| nalgebra | Modern, maintained, SIMD, widely used | Complex generics, long compile times |
| cgmath | Simple, clear API | Deprecated upstream |
| Custom | Full control, minimal dependencies | Reinventing the wheel |

**Recommendation**: nalgebra internally, wrapped in our own types for API stability.

---

## 5. Testing Patterns Observed

### Property-Based Testing (truck)

```rust
#[property_test]
fn test_derivative_consistency(
    #[strategy = 0f64..=1.0] t: f64,
    #[strategy = 2usize..=6] degree: usize,
) {
    // Verify: d/dt f^(n)(t) ≈ f^(n+1)(t) via finite differences
}
```

### Golden File / Roundtrip Testing (truck STEP I/O)

- Reference STEP files from OpenCASCADE + ABC dataset
- Roundtrip: Read STEP → Write STEP → Read again → triangulate → verify mesh is closed
- Shell condition checks: `ShellCondition::Closed` assertion

### Topology Consistency Validation (fornjot)

- Every half-edge must have a sibling (shell closure)
- Coincident half-edges must reference same curve (identity, not equality)
- Face winding consistency
- Half-edge connection (start of next == end of current)
- No multiple references to same object in a cycle

### Volume/Mesh Verification

- Signed tetrahedron method for volume computation after boolean ops
- Bounding box verification
- Mesh closure checks (every edge shared by exactly 2 triangles)

### Key Macros/Utilities

```rust
assert_near!(a, b, tolerance)      // Floating-point comparison
assert_near2!(a, b, tolerance)     // 2D point comparison
curve.is_geometric_consistent()    // Endpoint matches vertex
```

---

## 6. Performance Characteristics

### Truck Boolean Performance

- Simple cases (two boxes): ~10-50ms
- Complex cases (many face pairs): seconds
- Bottleneck: surface-surface intersection (marching + fitting)
- Triangulation used for classification is fast but approximate

### Fornjot

- No boolean implementation to benchmark
- Focus on correctness over performance
- Validation dominates runtime in complex operations

### Relevant Benchmarks from curvo

- NURBS curve evaluation (degree 3, 10 CPs): ~0.5μs per point
- Surface-surface intersection: ~50-200ms depending on complexity
- Curve fitting (100 points): ~5ms

---

## 7. Lessons Learned from Existing Projects

### From Truck
- `Arc<Mutex>` for topology creates unnecessary complexity
- Boolean operations work but are fragile (transversal only)
- STEP I/O is viable in Rust (ruststep provides the foundation)
- Property-based testing catches edge cases in B-spline algorithms

### From Fornjot
- Pure topology/geometry separation is architecturally elegant but makes intersections hard
- Append-only stores prevent mutation (problematic for modeling operations)
- Strict validation during construction catches bugs early but slows development
- 0BSD license maximizes adoption potential
- 5 years without booleans demonstrates the difficulty of the problem

### From curvo
- nalgebra works well as the linear algebra backend
- NURBS evaluation is straightforward to implement correctly
- Intersection algorithms are the hard part (not evaluation)
- `robust` crate is essential for predicate-based decisions

### General
- No Rust project has achieved production-grade booleans yet
- STEP I/O is achievable but requires handling many edge cases
- Testing against OpenCASCADE output is the pragmatic validation approach
- The B-rep kernel problem is fundamentally about handling degeneracies, not implementing algorithms
