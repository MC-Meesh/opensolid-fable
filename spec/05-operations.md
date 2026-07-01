# 05 — Modeling Operations

Blending, offsetting, sweeping, lofting, direct modeling, and other operations.

## 1. Blending (Fillets and Chamfers)

### 1.1 Constant-Radius Fillet

The most common blend operation. Creates a smooth rolling-ball surface between two
adjacent faces along an edge.

```rust
pub struct FilletOptions {
    /// Radius of the rolling ball.
    pub radius: f64,
    /// How to handle corners where 3+ edges meet.
    pub corner_type: CornerType,
    /// Propagation: automatically fillet tangent-continuous edge chains.
    pub propagate: bool,
    /// Overflow handling: what to do when fillet consumes an entire face.
    pub overflow: OverflowBehavior,
}

pub enum CornerType {
    /// Smooth setback corner (default, most robust).
    Setback,
    /// Sharp mitered corner.
    Mitered,
    /// Rolling ball corner (spherical patch).
    RollingBall,
}

pub enum OverflowBehavior {
    /// Fail if fillet would consume a face.
    Fail,
    /// Remove consumed face and blend into adjacent faces.
    Remove,
    /// Trim fillet to available space.
    Trim,
}

impl Kernel {
    /// Apply constant-radius fillets to edges.
    pub fn fillet_edges(
        &mut self,
        edges: &[EntityId<Edge>],
        options: &FilletOptions,
    ) -> Result<(), BlendError>;
}
```

#### Fillet Algorithm

```
1. EDGE SELECTION & VALIDATION
   - Verify all edges are on the same body
   - Verify edges have exactly 2 adjacent faces (manifold)
   - Check radius is feasible (< minimum face width along edge)

2. SUPPORT SURFACE COMPUTATION
   - For each edge: compute the fillet surface (rolling ball locus)
   - For analytic faces: closed-form fillet surfaces
     - Plane + Plane: cylinder (constant radius between planes)
     - Plane + Cylinder: canal surface or torus section
     - Cylinder + Cylinder: depends on relative positions
   - For NURBS faces: offset-intersection method
     - Offset face A inward by radius → surface A'
     - Offset face B inward by radius → surface B'
     - Intersect A' and B' → fillet spine curve
     - Sweep a circle of given radius along spine → fillet surface

3. TRIM CURVE COMPUTATION
   - Intersect fillet surface with original faces
   - These intersections define where the fillet meets the original faces
   - Produces trim curves (pcurves) on both original faces and fillet surface

4. FACE SPLITTING
   - Split original faces along trim curves
   - Discard the regions consumed by the fillet
   - Insert new fillet face(s)

5. CORNER TREATMENT
   - Where 3+ fillet faces meet at a vertex:
     - Compute corner patch geometry (setback, sphere, etc.)
     - Trim adjacent fillet faces
     - Insert corner face

6. TOPOLOGY RECONSTRUCTION
   - Build new edges at fillet/face boundaries
   - Stitch everything together
   - Validate G1 continuity at fillet boundaries
```

### 1.2 Variable-Radius Fillet

Radius varies along the edge, defined by control values at specified positions.

```rust
pub struct VariableFilletOptions {
    /// (parameter_on_edge, radius) control points.
    pub radii: Vec<(f64, f64)>,
    /// Interpolation between control points.
    pub interpolation: RadiusInterpolation,
    pub corner_type: CornerType,
}

pub enum RadiusInterpolation {
    Linear,
    Cubic,    // Smooth C2 interpolation
    Custom(BSplineCurve),  // Arbitrary radius function
}
```

### 1.3 Chamfer

A flat cut between two faces (replaces the edge with a planar face).

```rust
pub struct ChamferOptions {
    pub chamfer_type: ChamferType,
}

pub enum ChamferType {
    /// Equal distance from edge on both faces.
    Symmetric(f64),
    /// Different distances on each face.
    Asymmetric { distance_a: f64, distance_b: f64 },
    /// Distance + angle from one face.
    DistanceAngle { distance: f64, angle: f64 },
}

impl Kernel {
    pub fn chamfer_edges(
        &mut self,
        edges: &[EntityId<Edge>],
        options: &ChamferOptions,
    ) -> Result<(), BlendError>;
}
```

### 1.4 Production Geometry Handling

Fillets in production parts are NOT simple box-cylinder intersections. The kernel
must handle these common real-world scenarios:

```
MUST WORK (production fillet scenarios):
- Fillet along edge where one face is NURBS (not just planes/cylinders)
- Variable-radius fillet on a curved edge
- Fillet chain that crosses tangent-discontinuous edge junctions
- Fillet that consumes an adjacent narrow face (overflow)
- Multi-edge fillet with different radii meeting at a vertex
- Fillet on imported geometry with elevated tolerances (1e-4 to 1e-3)
- Fillet on edges produced by a prior boolean operation
- Fillet rollback: if 1 of N edges fails, report which one and still produce
  the N-1 successes (via partial_results in BlendError)
```

### 1.5 Blend Errors

```rust
pub enum BlendError {
    /// Radius too large for available geometry.
    RadiusTooLarge {
        edge: EntityId<Edge>,
        max_feasible: f64,
        requested: f64,
    },
    /// Fillet surfaces self-intersect.
    SelfIntersection {
        edges: Vec<EntityId<Edge>>,
    },
    /// Cannot compute fillet surface (degenerate geometry).
    ComputationFailed {
        edge: EntityId<Edge>,
        reason: String,
    },
    /// Edge is not suitable for blending (free edge, non-manifold).
    InvalidEdge {
        edge: EntityId<Edge>,
        reason: String,
    },
    /// Corner treatment failed.
    CornerFailed {
        vertex: EntityId<Vertex>,
        reason: String,
    },
    /// Partial failure: some edges succeeded, some didn't.
    /// The body is returned with successful fillets applied.
    PartialFailure {
        succeeded: Vec<EntityId<Edge>>,
        failed: Vec<(EntityId<Edge>, Box<BlendError>)>,
        body_with_partial_result: EntityId<Body>,
    },
}
```

## 2. Offset and Shell

### 2.1 Face Offset

Move selected faces along their normal direction.

```rust
pub struct FaceOffsetOptions {
    pub distance: f64,                 // Positive = outward, negative = inward
    pub extend_adjacent: bool,         // Extend adjacent faces to close gaps
}

impl Kernel {
    /// Offset faces along their normals.
    pub fn offset_faces(
        &mut self,
        faces: &[EntityId<Face>],
        options: &FaceOffsetOptions,
    ) -> Result<(), OffsetError>;
}
```

### 2.2 Body Offset (Shelling)

Hollow out a solid body by offsetting all faces inward, optionally removing some faces.

```rust
pub struct ShellOptions {
    /// Offset distance (positive = inward, creating wall thickness).
    pub thickness: f64,
    /// Faces to remove (creating openings in the shell).
    pub open_faces: Vec<EntityId<Face>>,
    /// Per-face thickness overrides.
    pub face_overrides: Vec<(EntityId<Face>, f64)>,
}

impl Kernel {
    /// Create a thin-walled shell from a solid body.
    pub fn shell(
        &mut self,
        body: EntityId<Body>,
        options: &ShellOptions,
    ) -> Result<(), OffsetError>;
}
```

#### Shell Algorithm

```
1. OFFSET SURFACES
   - For each face (except open faces): compute offset surface
   - Analytic surfaces: trivial (adjust radius/position)
   - NURBS: offset surface entity (evaluated on-demand)

2. SELF-INTERSECTION DETECTION
   - Offset surfaces may self-intersect (concave regions)
   - Detect and resolve: trim or replace with intersection result

3. INTERSECTION RECOMPUTATION
   - Adjacent offset surfaces may no longer meet cleanly
   - Recompute all edge curves as intersections of offset surfaces

4. GAP FILLING
   - At convex edges: offset surfaces diverge → insert fillet
   - At concave edges: offset surfaces overlap → trim

5. OPEN FACE TREATMENT
   - Where faces were removed: insert side walls connecting outer
     and inner shells

6. ASSEMBLY
   - Inner shell (all offset faces) + outer shell (original, or offset outward)
   - Connect at open faces with side walls
```

### 2.3 Offset Errors

```rust
pub enum OffsetError {
    /// Offset distance too large (body would collapse).
    DistanceTooLarge {
        face: EntityId<Face>,
        max_feasible: f64,
    },
    /// Self-intersection in offset result.
    SelfIntersection {
        faces: Vec<EntityId<Face>>,
    },
    /// Cannot offset surface (degenerate, e.g., offset of a cone apex).
    ComputationFailed {
        face: EntityId<Face>,
        reason: String,
    },
}
```

## 3. Sweep Operations

### 3.1 Linear Extrusion

Sweep a profile along a direction vector.

```rust
pub struct ExtrudeOptions {
    pub direction: Vector3,
    pub distance: f64,
    /// Draft angle (taper).
    pub draft_angle: Option<f64>,
    /// Cap the ends (true = solid, false = sheet).
    pub cap: bool,
}

impl Kernel {
    /// Extrude a face/wire profile along a direction.
    pub fn extrude(
        &mut self,
        profile: SweepProfile,
        options: &ExtrudeOptions,
    ) -> Result<EntityId<Body>, SweepError>;
}

pub enum SweepProfile {
    /// A planar face (2D profile) — extrudes to solid.
    Face(EntityId<Face>),
    /// A wire body — extrudes to sheet.
    Wire(EntityId<Body>),
    /// A set of curves forming a closed profile.
    Curves(Vec<EntityId<Curve>>),
}
```

### 3.2 Revolution

Sweep a profile around an axis.

```rust
pub struct RevolveOptions {
    pub axis_origin: Point3,
    pub axis_direction: UnitVector3,
    pub angle: f64,                    // Radians, 2π for full revolution
    pub cap: bool,
}

impl Kernel {
    pub fn revolve(
        &mut self,
        profile: SweepProfile,
        options: &RevolveOptions,
    ) -> Result<EntityId<Body>, SweepError>;
}
```

### 3.3 Path Sweep

Sweep a profile along an arbitrary path curve.

```rust
pub struct PathSweepOptions {
    pub path: EntityId<Curve>,
    /// How the profile orients as it moves along the path.
    pub orientation: SweepOrientation,
    /// Scale factor at start and end (for tapered sweeps).
    pub scale_start: f64,
    pub scale_end: f64,
    /// Twist angle along the path (radians).
    pub twist: f64,
    pub cap: bool,
}

pub enum SweepOrientation {
    /// Profile stays perpendicular to path (Frenet frame).
    FrenetFrame,
    /// Profile normal stays parallel to a fixed direction.
    FixedDirection(UnitVector3),
    /// Profile follows a guide curve for orientation.
    GuideCurve(EntityId<Curve>),
    /// Minimize rotation (parallel transport / Bishop frame).
    MinimumRotation,
}

impl Kernel {
    pub fn path_sweep(
        &mut self,
        profile: SweepProfile,
        options: &PathSweepOptions,
    ) -> Result<EntityId<Body>, SweepError>;
}
```

### 3.4 Loft

Create a surface/solid passing through multiple cross-section profiles.

```rust
pub struct LoftOptions {
    /// Ordered cross-section profiles.
    pub sections: Vec<SweepProfile>,
    /// Guide curves constraining the loft surface.
    pub guides: Vec<EntityId<Curve>>,
    /// Continuity at start/end.
    pub start_condition: LoftEndCondition,
    pub end_condition: LoftEndCondition,
    /// Close the loft (connect last section to first).
    pub closed: bool,
    pub cap: bool,
}

pub enum LoftEndCondition {
    /// Free end (natural B-spline).
    Free,
    /// Tangent to a direction at the end.
    Tangent(Vector3),
    /// Curvature-continuous with adjacent geometry.
    Smooth,
    /// Match an existing surface at the boundary.
    MatchSurface(EntityId<Surface>),
}

impl Kernel {
    pub fn loft(
        &mut self,
        options: &LoftOptions,
    ) -> Result<EntityId<Body>, SweepError>;
}
```

## 4. Direct Modeling

Operations that modify existing body geometry without a feature tree.

### 4.1 Move Face

Translate selected faces while maintaining tangent continuity with adjacent faces.

```rust
impl Kernel {
    pub fn move_face(
        &mut self,
        faces: &[EntityId<Face>],
        transform: &Transform3,
        options: &DirectModelOptions,
    ) -> Result<(), DirectModelError>;
}

pub struct DirectModelOptions {
    /// How to treat adjacent faces.
    pub adjacent_behavior: AdjacentBehavior,
}

pub enum AdjacentBehavior {
    /// Extend/trim adjacent faces to maintain connectivity.
    ExtendTrim,
    /// Insert blend faces between moved and unmoved.
    InsertBlend { radius: f64 },
    /// Allow gaps (convert to sheet body).
    AllowGaps,
}
```

### 4.2 Replace Face

Replace a face's underlying surface with a new one.

```rust
impl Kernel {
    pub fn replace_face_surface(
        &mut self,
        face: EntityId<Face>,
        new_surface: EntityId<Surface>,
        options: &DirectModelOptions,
    ) -> Result<(), DirectModelError>;
}
```

### 4.3 Delete Face

Remove faces from a body and heal the gap.

```rust
impl Kernel {
    pub fn delete_faces(
        &mut self,
        faces: &[EntityId<Face>],
        heal: bool,
    ) -> Result<(), DirectModelError>;
}
```

## 5. Pattern Operations

### 5.1 Linear Pattern

```rust
pub struct LinearPatternOptions {
    pub direction: Vector3,
    pub count: u32,
    pub spacing: f64,
}

impl Kernel {
    /// Create a linear pattern of a body or feature.
    pub fn linear_pattern(
        &mut self,
        body: EntityId<Body>,
        options: &LinearPatternOptions,
    ) -> Result<EntityId<Body>, PatternError>;
}
```

### 5.2 Circular Pattern

```rust
pub struct CircularPatternOptions {
    pub axis_origin: Point3,
    pub axis_direction: UnitVector3,
    pub count: u32,
    pub total_angle: f64,              // 2π for full circle
}

impl Kernel {
    pub fn circular_pattern(
        &mut self,
        body: EntityId<Body>,
        options: &CircularPatternOptions,
    ) -> Result<EntityId<Body>, PatternError>;
}
```

## 6. Mass Properties

```rust
pub struct MassProperties {
    pub volume: f64,
    pub surface_area: f64,
    pub center_of_mass: Point3,
    pub moments_of_inertia: Matrix3x3,  // About center of mass
    pub principal_moments: [f64; 3],
    pub principal_axes: [UnitVector3; 3],
}

pub struct FaceProperties {
    pub area: f64,
    pub centroid: Point3,
    pub perimeter: f64,
}

impl Kernel {
    /// Compute mass properties of a solid body.
    /// Uses surface integral (divergence theorem) for exact volume.
    pub fn mass_properties(
        &self,
        body: EntityId<Body>,
        density: f64,
    ) -> Result<MassProperties, MassError>;

    /// Compute area properties of a face.
    pub fn face_properties(
        &self,
        face: EntityId<Face>,
    ) -> Result<FaceProperties, MassError>;

    /// Compute edge length.
    pub fn edge_length(&self, edge: EntityId<Edge>) -> f64;
}
```

## 7. Sweep Errors

```rust
pub enum SweepError {
    /// Profile is not planar (for extrude/revolve).
    NonPlanarProfile,
    /// Profile is not closed (for solid result).
    OpenProfile,
    /// Self-intersection in sweep result.
    SelfIntersection { parameter: f64 },
    /// Path has zero curvature radius < profile extent (profile collides with itself).
    PathTooTight { parameter: f64, min_radius: f64 },
    /// Loft sections are incompatible (different edge counts).
    IncompatibleSections { section_a: usize, section_b: usize },
    /// Guide curve doesn't span all sections.
    GuideIncomplete { guide_index: usize },
}
```
