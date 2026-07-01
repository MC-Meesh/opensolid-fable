# 00 — System Architecture Overview

## 1. High-Level Architecture

OpenSolid is a boundary representation (B-rep) geometry kernel. Like Parasolid, it maintains
two parallel data structures for every solid model:

- **Topology**: The connectivity graph (what's connected to what)
- **Geometry**: The mathematical definitions (where things are in space)

These are linked but independent. A face (topology) references a surface (geometry).
An edge (topology) references a curve (geometry). This separation is the key architectural
insight from Parasolid — it allows topology to change without invalidating geometry and
vice versa.

```
┌─────────────────────────────────────────────────────────┐
│                     OpenSolid Kernel                      │
├─────────────────────────────────────────────────────────┤
│                                                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐  │
│  │   Topology   │  │   Geometry   │  │  Attributes  │  │
│  │              │  │              │  │              │  │
│  │  Body        │  │  Curves      │  │  Tags        │  │
│  │  Shell       │  │  Surfaces    │  │  Colors      │  │
│  │  Face        │  │  Points      │  │  Names       │  │
│  │  Loop        │  │  Transforms  │  │  User data   │  │
│  │  Edge        │  │  Intervals   │  │              │  │
│  │  Vertex      │  │              │  │              │  │
│  │  Fin         │  │              │  │              │  │
│  └──────┬───────┘  └──────┬───────┘  └──────────────┘  │
│         │                  │                             │
│         └────────┬─────────┘                             │
│                  │                                        │
│  ┌───────────────▼───────────────────────────────────┐  │
│  │              Operations Layer                      │  │
│  │                                                    │  │
│  │  Booleans │ Blending │ Offset │ Sweep │ Direct    │  │
│  │  Loft │ Pattern │ Shell │ Check │ Heal │ Mass     │  │
│  └───────────────┬───────────────────────────────────┘  │
│                  │                                        │
│  ┌───────────────▼───────────────────────────────────┐  │
│  │              I/O Layer                             │  │
│  │                                                    │  │
│  │  STEP Reader │ STEP Writer │ STL │ OBJ │ Native   │  │
│  └───────────────────────────────────────────────────┘  │
│                                                          │
│  ┌───────────────────────────────────────────────────┐  │
│  │              Session Layer                         │  │
│  │                                                    │  │
│  │  Memory │ Undo/Redo │ Journal │ Partitions         │  │
│  └───────────────────────────────────────────────────┘  │
│                                                          │
├─────────────────────────────────────────────────────────┤
│                    Public API                            │
│  Rust API │ C FFI │ Python bindings │ WASM             │
└─────────────────────────────────────────────────────────┘
```

## 2. Crate Structure

```
opensolid/
├── crates/
│   ├── opensolid-core/          # Entity IDs, arena allocator, common traits
│   ├── opensolid-math/          # Vectors, points, transforms, intervals, tolerances
│   ├── opensolid-curves/        # All curve types + evaluation + intersection
│   ├── opensolid-surfaces/      # All surface types + evaluation + intersection
│   ├── opensolid-topology/      # Body/shell/face/loop/edge/vertex graph
│   ├── opensolid-booleans/      # Boolean operations (unite, subtract, intersect)
│   ├── opensolid-blend/         # Fillets, chamfers, variable-radius blends
│   ├── opensolid-offset/        # Face/body offset, shelling
│   ├── opensolid-sweep/         # Extrude, revolve, path sweep, loft
│   ├── opensolid-direct/        # Direct modeling (move/offset/replace face)
│   ├── opensolid-check/         # Validation, consistency checking, healing
│   ├── opensolid-mass/          # Volume, area, center of mass, inertia
│   ├── opensolid-spatial/       # BVH, spatial indexing, acceleration structures
│   ├── opensolid-tessellate/    # Faceting + mesh export (STL/OBJ, first-class output)
│   ├── opensolid-step/          # STEP AP203/AP214 reader + writer (heal-first import)
│   ├── opensolid-native-io/     # Native binary format (fast save/load)
│   ├── opensolid-session/       # Undo/redo, journaling, memory management
│   └── opensolid-api/           # High-level builder API + unified error reporting
├── opensolid/                   # Top-level facade crate (re-exports everything)
├── bindings/
│   ├── opensolid-c/             # C FFI header generation (cbindgen)
│   ├── opensolid-python/        # PyO3 bindings (ships same day as Rust API)
│   └── opensolid-wasm/          # WASM compilation target
├── tests/
│   ├── golden/                  # Reference STEP files for round-trip testing
│   ├── property/                # Property-based tests (quickcheck/proptest)
│   └── integration/             # End-to-end operation sequences
└── benches/                     # Performance benchmarks
```

## 3. Core Design Decisions

### 3.1 Entity Storage: Arena Allocation with Generational Indices

All topological and geometric entities live in typed arenas. References between entities
use generational indices (inspired by ECS patterns). This gives us:

- O(1) entity access
- No reference counting overhead
- Safe invalidation (generational check catches use-after-free)
- Cache-friendly iteration
- Natural undo/redo (snapshot the arena state)

```rust
/// A typed handle to an entity in the kernel's storage.
/// The generation prevents use-after-free when entities are deleted.
pub struct EntityId<T> {
    index: u32,
    generation: u32,
    _phantom: PhantomData<T>,
}

/// Typed arena for a specific entity kind.
pub struct Arena<T> {
    entries: Vec<ArenaEntry<T>>,
    free_list: Vec<u32>,
    len: u32,
}
```

### 3.2 Topology-Geometry Separation

Following Parasolid, topology and geometry are distinct layers connected by references:

```rust
// Topology references geometry, not the other way around
pub struct Face {
    pub id: EntityId<Face>,
    pub surface: EntityId<Surface>,     // The underlying surface
    pub outer_loop: EntityId<Loop>,     // Outer boundary
    pub inner_loops: Vec<EntityId<Loop>>, // Holes
    pub sense: bool,                    // Same or opposite to surface normal
    pub shell: EntityId<Shell>,         // Parent shell
}

pub struct Edge {
    pub id: EntityId<Edge>,
    pub curve: EntityId<Curve>,         // The underlying curve
    pub start_vertex: EntityId<Vertex>,
    pub end_vertex: EntityId<Vertex>,
    pub t_range: Interval,              // Parameter range on curve
    pub tolerance: f64,                 // Max deviation from exact geometry
}
```

### 3.3 Tolerant Modeling

Like Parasolid, OpenSolid supports tolerant modeling. Every edge carries a tolerance
value — the maximum distance between the edge's curve and the actual intersection of
its adjacent faces. This is essential for:

- Importing geometry from other systems (tolerance mismatch is inevitable)
- Boolean results where intersection curves are approximated
- Healing operations that fix near-miss topology

A "precise" body has all edge tolerances at the system resolution (~1e-10).
A "tolerant" body has some edges with elevated tolerances (up to ~0.01mm typically).

### 3.4 Immutable Snapshots for AI/Undo

Every modeling operation produces a new state snapshot. The previous state remains
accessible. This enables:

- **Undo/redo**: Just switch which snapshot is current
- **Branching**: Try operation A, save state, try operation B, compare results
- **AI exploration**: Agent can explore multiple design paths without commitment

Implementation: Copy-on-write (CoW) arena snapshots. Only modified entities are copied.

### 3.5 Error Model

Operations return rich, structured errors designed for programmatic consumption:

```rust
pub enum BooleanError {
    NoIntersection {
        body_a: EntityId<Body>,
        body_b: EntityId<Body>,
    },
    CoincidentFaces {
        faces: Vec<(EntityId<Face>, EntityId<Face>)>,
        suggestion: &'static str,
    },
    DegenerateResult {
        reason: &'static str,
        zero_volume_faces: Vec<EntityId<Face>>,
    },
    ToleranceExceeded {
        max_tolerance: f64,
        required_tolerance: f64,
        problematic_edges: Vec<EntityId<Edge>>,
    },
    SelfIntersection {
        body: EntityId<Body>,
        intersecting_faces: Vec<(EntityId<Face>, EntityId<Face>)>,
    },
}
```

## 4. Parasolid Functional Mapping

The following maps Parasolid's module structure to OpenSolid crates:

| Parasolid Module | Parasolid Prefix | OpenSolid Crate |
|-----------------|------------------|-----------------|
| Kernel Session | PK_SESSION | opensolid-session |
| Body | PK_BODY | opensolid-topology |
| Assembly | PK_ASSEMBLY | opensolid-topology |
| Geometry (curves) | PK_CURVE | opensolid-curves |
| Geometry (surfaces) | PK_SURF | opensolid-surfaces |
| Topology | PK_FACE, PK_EDGE, etc. | opensolid-topology |
| Boolean | PK_BODY_boolean | opensolid-booleans |
| Blend | PK_BODY_fix_blends | opensolid-blend |
| Offset | PK_FACE_offset, PK_BODY_offset | opensolid-offset |
| Sweep | PK_BODY_sweep | opensolid-sweep |
| Loft | PK_BODY_loft | opensolid-sweep |
| Section | PK_BODY_section | opensolid-booleans |
| Imprint | PK_FACE_imprint | opensolid-booleans |
| Check | PK_BODY_check | opensolid-check |
| Mass Properties | PK_TOPOL_eval_mass | opensolid-mass |
| Tessellation | PK_TOPOL_facet | opensolid-tessellate |
| Transmit (X_T) | PK_PART_transmit | opensolid-native-io |
| STEP | (via Parasolid XT Extensions) | opensolid-step |
| Rendering | PK_TOPOL_render | opensolid-tessellate |
| Local Ops | PK_BODY_local_ops | opensolid-direct |
| Attributes | PK_ATTRIB | opensolid-core |

## 5. Thread Safety Model

The kernel is designed for concurrent use:

- **Immutable operations** (queries, mass properties, tessellation): Freely parallel via shared references
- **Mutating operations** (booleans, blends): Require exclusive access to affected bodies
- **Session state**: Thread-local or explicitly passed (no global mutable state)

Rust's ownership system enforces this at compile time. No runtime locking for the common
case (read-heavy workloads).

## 6. Performance Targets

Published benchmarks with concrete targets. Every PR touching a performance-critical
path must include criterion benchmarks. Regressions > 10% block merge.

### Initial Targets (must pass to merge)

These are required for correctness-focused initial implementations:

| Operation | Initial Target | Measurement Method |
|-----------|---------------|--------------------|
| NURBS surface evaluation (single point) | < 5μs | criterion bench, degree 3-5, 10-50 CPs |
| NURBS curve evaluation (single point) | < 2μs | criterion bench |
| Boolean (two 1000-face bodies) | < 2s | wall-clock, release build |
| Boolean (two boxes, trivial) | < 50ms | wall-clock, release build |
| Fillet (single edge, constant radius) | < 500ms | wall-clock |
| STEP import (10,000 entities) | < 5s | wall-clock, includes healing |
| STEP export (10,000 entities) | < 3s | wall-clock |
| Tessellation (1000-face body, moderate) | < 1s | wall-clock |
| Body validation | < 200ms per 1000 faces | criterion bench |
| Spatial query (point containment, 10K faces) | < 20ms | BVH-accelerated |

### Optimized Targets (aspirational, matches/beats OCC)

Requires dedicated optimization passes after correctness is proven:

| Operation | Optimized Target | Measurement Method |
|-----------|------------------|--------------------|
| NURBS surface evaluation (single point) | < 1μs | criterion bench, SIMD-optimized |
| NURBS curve evaluation (single point) | < 0.5μs | criterion bench |
| Boolean (two 1000-face bodies) | < 500ms | wall-clock, release build |
| Boolean (two boxes, trivial) | < 10ms | wall-clock, release build |
| Fillet (single edge, constant radius) | < 100ms | wall-clock |
| Fillet (20 edges, variable radius, NURBS faces) | < 2s | wall-clock |
| STEP import (10,000 entities) | < 2s | wall-clock, includes healing |
| STEP export (10,000 entities) | < 1s | wall-clock |
| Tessellation (1000-face body, moderate) | < 200ms | wall-clock |
| Body validation | < 50ms per 1000 faces | criterion bench |
| Spatial query (point containment, 10K faces) | < 5ms | BVH-accelerated |
| Closest point on body (10K faces) | < 10ms | BVH-accelerated |

### Workflow Targets (composite operations)

Single-operation benchmarks miss inter-operation overhead (BVH reconstruction,
validation, snapshot creation). These workflow benchmarks catch that:

| Workflow | Target |
|----------|--------|
| Import 100-face STEP + 3 booleans + export STEP | < 10s |
| Create block + 5 subtracts + fillet 10 edges + tessellate | < 5s |
| Import 10,000-entity STEP + tessellate all bodies | < 15s |
| 50 sequential operations (mixed) + undo all | < 30s, peak RSS < 1GB |

Benchmarks live in `benches/` and run in CI on every PR.

## 7. Spatial Indexing (BVH)

All geometric queries (closest point, interference, containment, ray cast) are
accelerated by a bounding volume hierarchy. This is not optional — it is required
for any body with > 100 faces to hit performance targets.

```rust
/// Bounding Volume Hierarchy for spatial acceleration.
/// Built lazily on first spatial query, invalidated on topology change.
pub struct Bvh {
    nodes: Vec<BvhNode>,
    body: EntityId<Body>,
    generation: u32,  // Invalidated when body changes
}

pub enum BvhNode {
    Leaf { face: EntityId<Face>, bbox: BoundingBox3 },
    Internal { bbox: BoundingBox3, left: u32, right: u32 },
}

impl Kernel {
    /// Get or build BVH for a body. Cached until body is modified.
    pub fn body_bvh(&self, body: EntityId<Body>) -> &Bvh;

    /// Spatial queries (all BVH-accelerated):
    pub fn closest_point_on_body(&self, point: &Point3, body: EntityId<Body>)
        -> Result<ClosestPointResult, QueryError>;
    pub fn point_in_body(&self, point: &Point3, body: EntityId<Body>)
        -> Result<Containment, QueryError>;
    pub fn ray_cast(&self, ray: &Ray3, body: EntityId<Body>)
        -> Result<Vec<RayHit>, QueryError>;
    pub fn interference_check(&self, bodies: &[EntityId<Body>])
        -> Result<Vec<Interference>, QueryError>;
}
```

## 8. Dependency Philosophy

Minimize external dependencies. The kernel should be self-contained for correctness
and auditability. Allowed dependencies:

- `nalgebra` — Linear algebra (vectors, matrices, transforms)
- `robust` — Exact geometric predicates (orient2d, incircle)
- `rayon` — Data parallelism for batch operations
- `thiserror` — Error type derivation
- `serde` — Serialization (optional feature)

NOT allowed as dependencies (must implement ourselves):
- NURBS evaluation (core competency)
- Boolean algorithms (core competency)
- STEP parsing (too specialized, existing crates are incomplete)
- Tessellation (tightly coupled to our geometry)
- BVH construction (simple enough, avoids version churn)

## 9. Persistent Entity Naming

Every topological entity receives a stable name that survives modeling operations.
This is critical for parametric rebuilds, scripting, and AI agents that reference
specific faces/edges across operations.

### 9.1 Design: Geometric Hashing (Not Index-Based)

Persistent naming through booleans is fundamentally approximate — when a face splits
into N children, the ordering of children depends on intersection curve topology,
which can vary with floating-point non-determinism. Using `child_index` (ordering-dependent)
produces different names on different platforms or optimization levels.

Instead, we use **geometric hashing**: each child face is identified by the centroid
and bounding box of its resulting region. This is deterministic regardless of
processing order.

```rust
/// Persistent identifier for a topological entity.
/// Survives boolean ops, blends, and other transformations.
pub struct PersistentId {
    /// Origin: which operation created this entity.
    pub origin: OperationId,
    /// Lineage: for entities created by splitting/merging, tracks parents.
    pub parents: SmallVec<[PersistentId; 2]>,
    /// Geometric hash of the resulting face region (centroid + bbox extents).
    /// Deterministic across platforms and optimization levels.
    pub region_hash: u64,
}

/// Named entity lookup — find entities by their persistent name.
impl Kernel {
    /// Get the current EntityId for a persistent name. Exact match.
    /// Returns None if the entity no longer exists (consumed by a later op).
    pub fn resolve_name(&self, name: &PersistentId) -> Option<AnyEntityId>;

    /// Approximate name resolution: find the best-matching entity when exact
    /// match fails (e.g., after a tolerance-dependent re-split).
    /// Returns candidates ranked by geometric similarity to the original.
    pub fn resolve_name_approximate(
        &self, name: &PersistentId, tolerance: f64
    ) -> Vec<(AnyEntityId, f64)>;  // (entity, match_score 0.0-1.0)

    /// Get all persistent names that map to a given entity.
    pub fn entity_names(&self, entity: AnyEntityId) -> Vec<&PersistentId>;

    /// Get the faces that were created when face X was split by operation Y.
    pub fn trace_lineage(&self, original: EntityId<Face>, op: OperationId)
        -> Vec<EntityId<Face>>;
}
```

### 9.2 Geometric Hash Computation

```rust
impl PersistentId {
    /// Compute region_hash from a face's geometry.
    /// Uses centroid + quantized bounding box extents to produce a stable hash.
    pub fn compute_region_hash(
        centroid: &Point3,
        bbox: &BoundingBox3,
        quantization: f64,  // typically tolerance * 100
    ) -> u64 {
        // Quantize to avoid floating-point jitter
        let cx = (centroid.x / quantization).round() as i64;
        let cy = (centroid.y / quantization).round() as i64;
        let cz = (centroid.z / quantization).round() as i64;
        let dx = ((bbox.max.x - bbox.min.x) / quantization).round() as i64;
        let dy = ((bbox.max.y - bbox.min.y) / quantization).round() as i64;
        let dz = ((bbox.max.z - bbox.min.z) / quantization).round() as i64;
        // FNV-1a or similar hash of the quantized values
        hash_i64_tuple(cx, cy, cz, dx, dy, dz)
    }
}
```

### 9.3 Naming Rules

- Primitive creation: each face/edge gets a name derived from its geometric role
  (e.g., `block.top`, `cylinder.lateral`, `sphere.face`)
- Boolean ops: result faces inherit parent names with operation lineage + region_hash
- Blends: fillet faces get names like `fillet(edge_name, radius)` + region_hash
- Splits: child faces get parent name + region_hash (deterministic, not order-dependent)
