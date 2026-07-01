# 04 — Boolean Operations

Union, subtract, intersect — the most complex operations in the kernel.

## 1. Overview

Boolean operations combine two solid bodies into a result body. They are the primary
way users create complex shapes from simple primitives.

| Operation | Result | Parasolid API |
|-----------|--------|---------------|
| Unite (Union) | Material of A OR B | PK_BODY_boolean_2 |
| Subtract (Difference) | Material of A AND NOT B | PK_BODY_boolean_2 |
| Intersect | Material of A AND B | PK_BODY_boolean_2 |

### 1.1 Why Booleans Are Hard

The core difficulty is **topology reconstruction**. After intersecting geometry (SSI),
you must:

1. Classify every face region as "keep" or "discard"
2. Trim faces along intersection curves
3. Rebuild topology (new edges, vertices, face splits)
4. Handle degeneracies (tangent intersections, coincident faces, zero-thickness results)
5. Maintain tolerance consistency

Each step has hundreds of special cases. Parasolid has accumulated 35+ years of fixes.

## 2. Algorithm Pipeline

```
┌─────────────────────────────────────────────────────────────────┐
│                     Boolean Pipeline                              │
│                                                                   │
│  1. CLASH DETECTION                                              │
│     └── BVH overlap test → candidate face pairs                  │
│                                                                   │
│  2. FACE-FACE INTERSECTION                                       │
│     └── SSI for each candidate pair → intersection curves        │
│                                                                   │
│  3. CURVE MERGING & TOPOLOGY                                     │
│     └── Connect intersection curves → intersection graph         │
│                                                                   │
│  4. FACE CLASSIFICATION                                          │
│     └── For each face region: inside/outside/on the other body   │
│                                                                   │
│  5. FACE SPLITTING                                               │
│     └── Split faces along intersection curves → new faces        │
│                                                                   │
│  6. TOPOLOGY RECONSTRUCTION                                      │
│     └── Build result body from kept face regions                 │
│                                                                   │
│  7. EDGE MERGING & CLEANUP                                       │
│     └── Merge coincident edges, remove sliver faces              │
│                                                                   │
│  8. VALIDATION                                                   │
│     └── Check result body invariants                             │
└─────────────────────────────────────────────────────────────────┘
```

## 3. Phase 1: Clash Detection

Quickly determine which faces from body A might intersect faces from body B.

```rust
pub struct ClashDetector {
    bvh_a: BVHTree<EntityId<Face>>,
    bvh_b: BVHTree<EntityId<Face>>,
}

impl ClashDetector {
    /// Build BVH trees for both bodies.
    pub fn new(
        body_a: EntityId<Body>,
        body_b: EntityId<Body>,
        store: &TopologyStore,
        geo: &GeometryStore,
    ) -> Self;

    /// Find all potentially intersecting face pairs.
    /// Returns pairs where bounding boxes overlap (expanded by tolerance).
    pub fn find_candidates(&self, tolerance: f64) -> Vec<(EntityId<Face>, EntityId<Face>)>;

    /// Quick reject: are the bodies completely disjoint?
    pub fn bodies_disjoint(&self) -> bool;

    /// Quick accept: is one body completely inside the other?
    pub fn containment_test(&self, store: &TopologyStore, geo: &GeometryStore)
        -> Option<Containment>;
}
```

## 4. Phase 2: Face-Face Intersection

For each candidate pair, compute the intersection curves.

```rust
pub struct FaceIntersector;

impl FaceIntersector {
    /// Intersect two faces, producing trimmed intersection curves.
    ///
    /// This is the SSI algorithm from 02-geometry.md, but bounded to the
    /// face domains (trim curves must be respected).
    pub fn intersect_faces(
        face_a: EntityId<Face>,
        face_b: EntityId<Face>,
        store: &TopologyStore,
        geo: &GeometryStore,
        tolerance: f64,
    ) -> FaceIntersectionResult;
}

pub struct FaceIntersectionResult {
    /// Intersection curves between the two faces.
    pub curves: Vec<IntersectionCurveSegment>,
    /// Coincident regions (faces overlap in area).
    pub coincident_patches: Vec<CoincidentPatch>,
    /// Classification of the intersection type.
    pub intersection_type: IntersectionType,
}

pub struct IntersectionCurveSegment {
    pub curve_3d: BSplineCurve,
    pub pcurve_on_a: Curve2D,          // In face A's parameter space
    pub pcurve_on_b: Curve2D,          // In face B's parameter space
    pub start_on_boundary: Option<BoundaryLocation>,
    pub end_on_boundary: Option<BoundaryLocation>,
    pub tolerance: f64,
}

pub enum BoundaryLocation {
    EdgeOfA(EntityId<Edge>, f64),       // On an edge of face A at parameter t
    EdgeOfB(EntityId<Edge>, f64),       // On an edge of face B at parameter t
    VertexOfA(EntityId<Vertex>),
    VertexOfB(EntityId<Vertex>),
}

pub enum IntersectionType {
    Transverse,                         // Normal crossing
    Tangent,                            // Surfaces touch along curve
    Coincident,                         // Surfaces overlap in area
    None,                               // No intersection (BVH false positive)
}
```

## 5. Phase 3: Intersection Graph

Connect all intersection curve segments into a graph structure:

```rust
pub struct IntersectionGraph {
    /// All intersection curve segments.
    pub segments: Vec<IntersectionCurveSegment>,
    /// Nodes where segments meet (at face boundaries or at each other).
    pub nodes: Vec<IntersectionNode>,
    /// Adjacency: which segments connect at each node.
    pub adjacency: Vec<Vec<(usize, bool)>>,  // (segment_index, reversed)
}

pub struct IntersectionNode {
    pub point: Point3,
    pub vertex: Option<EntityId<Vertex>>,  // Existing vertex if on boundary
    pub on_edge_a: Option<(EntityId<Edge>, f64)>,
    pub on_edge_b: Option<(EntityId<Edge>, f64)>,
}

impl IntersectionGraph {
    /// Build the graph from all face-face intersection results.
    pub fn build(
        results: &[(EntityId<Face>, EntityId<Face>, FaceIntersectionResult)],
        tolerance: f64,
    ) -> Self;

    /// Trace closed loops through the intersection graph.
    /// Each loop represents a trim boundary on a face.
    pub fn trace_loops(&self) -> Vec<IntersectionLoop>;
}
```

## 6. Phase 4: Face Classification

Determine which regions of each face are "inside" or "outside" the other body.

```rust
pub enum FaceClassification {
    /// Entire face is outside the other body (keep for union, discard for intersect)
    Outside,
    /// Entire face is inside the other body (discard for union, keep for intersect)
    Inside,
    /// Face is split by intersection curves (must be subdivided)
    Split(Vec<FaceRegion>),
    /// Face is coincident with a face of the other body
    Coincident { other_face: EntityId<Face>, same_sense: bool },
}

pub struct FaceRegion {
    pub classification: RegionClass,
    pub boundary: Vec<RegionBoundarySegment>,
}

pub enum RegionClass {
    Inside,
    Outside,
    On,  // Coincident
}

pub struct FaceClassifier;

impl FaceClassifier {
    /// Classify all faces of body A relative to body B.
    ///
    /// Algorithm:
    /// 1. For faces not intersected: sample a point, ray-cast against body B
    /// 2. For split faces: classify each region separately
    /// 3. For coincident faces: compare face normals to determine same/opposite sense
    pub fn classify(
        body_a: EntityId<Body>,
        body_b: EntityId<Body>,
        intersection_graph: &IntersectionGraph,
        store: &TopologyStore,
        geo: &GeometryStore,
        tolerance: f64,
    ) -> FaceClassificationMap;
}

/// Map from face ID to its classification.
pub type FaceClassificationMap = HashMap<EntityId<Face>, FaceClassification>;
```

### 6.1 Ray Casting for Point Classification

```rust
/// Determine if a point is inside a solid body using ray casting.
///
/// Algorithm:
/// 1. Cast a ray from the point in an arbitrary direction
/// 2. Count intersections with the body's faces
/// 3. Odd count = inside, even count = outside
/// 4. Handle edge cases: ray hits edge/vertex, ray is tangent to face
///
/// Uses multiple rays if the first hits a degenerate case.
pub fn ray_classify_point(
    point: &Point3,
    body: EntityId<Body>,
    store: &TopologyStore,
    geo: &GeometryStore,
    tolerance: f64,
) -> Containment;
```

## 7. Phase 5: Face Splitting

Split faces along intersection curves to create new faces.

```rust
pub struct FaceSplitter;

impl FaceSplitter {
    /// Split a face along intersection curves.
    ///
    /// Algorithm:
    /// 1. Insert new vertices at intersection curve endpoints
    /// 2. Insert new edges along intersection curves
    /// 3. Split the face's loop structure into sub-regions
    /// 4. Create new faces for each sub-region
    ///
    /// The original face is consumed and replaced by the split faces.
    pub fn split_face(
        face: EntityId<Face>,
        curves: &[IntersectionCurveSegment],
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        tolerance: f64,
    ) -> Vec<EntityId<Face>>;
}
```

## 8. Phase 6: Topology Reconstruction

Build the result body from classified face regions.

```rust
pub struct BooleanReconstructor;

impl BooleanReconstructor {
    /// Reconstruct the result body for a boolean operation.
    ///
    /// Based on operation type, select which classified regions to keep:
    ///   Union:     A.outside + B.outside + coincident(same_sense)
    ///   Subtract:  A.outside + B.inside(reversed) + coincident(opposite_sense, reversed)
    ///   Intersect: A.inside + B.inside + coincident(same_sense)
    pub fn reconstruct(
        op: BooleanOp,
        body_a: EntityId<Body>,
        body_b: EntityId<Body>,
        classification_a: &FaceClassificationMap,
        classification_b: &FaceClassificationMap,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        tolerance: f64,
    ) -> Result<EntityId<Body>, BooleanError>;
}

pub enum BooleanOp {
    Unite,
    Subtract,
    Intersect,
}
```

## 9. Phase 7: Cleanup

```rust
pub struct BooleanCleanup;

impl BooleanCleanup {
    /// Merge edges that are now coincident (same curve, shared vertices).
    pub fn merge_coincident_edges(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        tolerance: f64,
    );

    /// Remove sliver faces (faces with zero or near-zero area).
    pub fn remove_sliver_faces(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        min_area: f64,
    );

    /// Merge coplanar adjacent faces (optimization).
    pub fn merge_coplanar_faces(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        angle_tolerance: f64,
    );

    /// Simplify topology: remove unnecessary edges/vertices.
    pub fn simplify_topology(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
    );
}
```

## 10. Special Cases and Degeneracies

These are the cases that take 90% of the implementation effort:

### 10.1 Coincident Face Detection and Handling

When faces of A and B lie on the same surface (or within tolerance of the same surface),
this requires a dedicated algorithm. This is historically the #1 source of boolean
crashes in open-source kernels.

**Detection Algorithm:**

```
1. SURFACE COMPATIBILITY CHECK
   For each candidate face pair (from BVH overlap):
   a) Check if surfaces are the same type (plane-plane, cylinder-cylinder, etc.)
   b) For analytic surfaces: compare parameters (origin, normal, radius, etc.)
      within tolerance. If they match → potentially coincident.
   c) For NURBS surfaces: sample both at a grid of points. If ALL samples on
      surface A project onto surface B within tolerance → potentially coincident.
   d) If surfaces are different types or don't match → skip (handle as normal SSI).

2. OVERLAP REGION DETERMINATION
   For confirmed same-surface face pairs:
   a) Map both faces' trim boundaries into their shared surface's parameter space.
   b) Compute the 2D boolean intersection of the trim regions.
      (This is a 2D polygon boolean — much simpler than 3D.)
   c) If the 2D intersection area > 0 → faces are coincident in that region.
   d) Classify the overlap:
      - FULL: one face entirely contains the other
      - PARTIAL: faces overlap in some region but not all
      - EDGE_TOUCHING: faces share a boundary but don't overlap in area

3. ORIENTATION DETERMINATION
   For coincident regions:
   a) Compare face normals at the overlap centroid.
   b) Same direction → same_sense (co-oriented)
   c) Opposite direction → opposite_sense (anti-oriented)

4. BOOLEAN RULES FOR COINCIDENT FACES
   | Operation | Same-sense | Opposite-sense |
   |-----------|-----------|----------------|
   | Union | Keep one copy | Faces cancel (material on both sides) |
   | Subtract | Remove (A-B at same face = nothing) | Keep A's copy |
   | Intersect | Keep one copy | Faces cancel (void on both sides) |
```

**v1.0 scope:** Return `BooleanError::CoincidentFacesNotSupported` with the face pair
identified. This is honest — coincident handling is the hardest degenerate case and
attempting it incorrectly is worse than refusing.

**v2.0 scope:** Full implementation per the algorithm above.

**Partial overlap (the hardest sub-case):** When NURBS faces partially overlap on
a shared surface, the overlap boundary must be computed in parameter space. This
requires a 2D curve-curve intersection of the projected trim boundaries. The result
is a set of new trim curves that split both faces at the overlap boundary.

### 10.2 Tangent Intersections

Surfaces touch along a curve but don't cross. The intersection curve is the
boundary between "on" and "outside" regions rather than "inside" and "outside".

### 10.3 Edge-on-Face

An edge of body A lies exactly on a face of body B. Must split the face at that edge
without creating degenerate topology.

### 10.4 Vertex-on-Edge / Vertex-on-Face

A vertex of one body touches an edge or face of the other. Creates a single
intersection point rather than a curve.

### 10.5 Zero-Thickness Results

A subtract operation may produce a body with zero-thickness regions (two faces
back-to-back). These must be detected and either removed or flagged.

### 10.6 Disconnected Results

A boolean may produce multiple disconnected shells. The result is either:
- A single body with multiple shells (Parasolid approach)
- Multiple separate bodies (alternative design choice)

## 11. Tolerant Boolean

When input bodies have elevated tolerances (from previous operations or import),
the boolean must propagate and potentially increase tolerances:

```rust
pub struct TolerantBooleanConfig {
    /// Maximum tolerance to accept on input bodies.
    pub max_input_tolerance: f64,
    /// Tolerance for intersection curve computation.
    pub intersection_tolerance: f64,
    /// Tolerance for face classification (ray casting distance threshold).
    pub classification_tolerance: f64,
    /// If true, attempt to heal result to reduce tolerances.
    pub heal_result: bool,
}
```

## 12. Public API

```rust
pub struct BooleanOptions {
    pub operation: BooleanOp,
    pub tolerance: f64,
    pub keep_input_bodies: bool,       // Don't consume input bodies
    pub check_result: bool,            // Validate result body
    pub merge_coincident: bool,        // Merge coincident edges in result
    pub simplify: bool,                // Remove unnecessary topology
}

impl Kernel {
    /// Perform a boolean operation between two bodies.
    pub fn boolean(
        &mut self,
        body_a: EntityId<Body>,
        body_b: EntityId<Body>,
        options: &BooleanOptions,
    ) -> Result<EntityId<Body>, BooleanError>;

    /// Unite multiple bodies (optimized for N-body union).
    pub fn unite_bodies(
        &mut self,
        bodies: &[EntityId<Body>],
        options: &BooleanOptions,
    ) -> Result<EntityId<Body>, BooleanError>;

    /// Section a body with a plane or sheet body.
    /// Returns the body split into two halves, or one half + the section curves.
    pub fn section(
        &mut self,
        body: EntityId<Body>,
        tool: SectionTool,
        options: &SectionOptions,
    ) -> Result<SectionResult, BooleanError>;

    /// Imprint curves/edges onto a face without splitting the body.
    /// Adds new edges to the face's topology.
    pub fn imprint(
        &mut self,
        face: EntityId<Face>,
        curves: &[EntityId<Curve>],
        tolerance: f64,
    ) -> Result<Vec<EntityId<Edge>>, BooleanError>;
}

pub enum SectionTool {
    Plane(PlaneSurface),
    SheetBody(EntityId<Body>),
}

pub struct SectionResult {
    pub bodies: Vec<EntityId<Body>>,   // Split pieces
    pub section_faces: Vec<EntityId<Face>>, // New faces at the cut
}
```

## 13. Scope: v1.0 vs v2.0

**v1.0 booleans handle transversal intersections only.** Degenerate configurations
return structured errors rather than producing wrong results. This is the honest
approach — truck claims to handle all cases and produces corrupt bodies 5% of the time.
We prefer correct refusal over silent corruption.

| Configuration | v1.0 | v2.0 |
|---------------|------|------|
| Transversal face-face intersection | Handled | Handled |
| Tangent intersections (surfaces touch) | `NotYetSupported` error | Handled |
| Coincident/coplanar faces | `NotYetSupported` error | Handled |
| Edge-on-face configurations | `NotYetSupported` error | Handled |
| Vertex-on-edge / vertex-on-face | `NotYetSupported` error | Handled |
| Identity operations (A∪A, A-A, A∩A) | Handled (fast path) | Handled |
| Disjoint bodies (no intersection) | Handled (fast path) | Handled |
| Zero-thickness results | Detected + error | Handled (remove) |

The `NotYetSupported` error includes:
- Which face pair triggered the degenerate case
- What type of degeneracy was detected
- A suggestion ("try translating body B by 0.001mm to avoid tangent contact")

This lets AI agents work around limitations while we harden the algorithm.

## 14. Error Types

```rust
pub enum BooleanError {
    /// Bodies don't intersect (for subtract/intersect this means no change or empty result).
    NoIntersection,
    /// Boolean would produce a zero-volume body.
    EmptyResult,
    /// Degenerate configuration not yet handled (v1.0 limitation).
    /// Includes the face pair and a workaround suggestion.
    NotYetSupported {
        face_a: EntityId<Face>,
        face_b: EntityId<Face>,
        degeneracy: DegeneracyType,
        suggestion: &'static str,
    },
    /// SSI failed on a face pair.
    IntersectionFailed {
        face_a: EntityId<Face>,
        face_b: EntityId<Face>,
        reason: String,
    },
    /// Face classification is ambiguous (near-tangent case).
    AmbiguousClassification {
        face: EntityId<Face>,
        point: Point3,
    },
    /// Result body fails validation.
    InvalidResult {
        errors: Vec<TopologyError>,
    },
    /// Input body is invalid.
    InvalidInput {
        body: EntityId<Body>,
        errors: Vec<TopologyError>,
    },
    /// Tolerance exceeded — result would require tolerance > max allowed.
    ToleranceExceeded {
        max_tolerance: f64,
        required: f64,
        location: Point3,
    },
    /// Self-intersecting result.
    SelfIntersection {
        faces: Vec<(EntityId<Face>, EntityId<Face>)>,
    },
}
```
