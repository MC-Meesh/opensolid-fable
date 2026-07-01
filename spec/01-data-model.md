# 01 — Data Model

The complete type hierarchy for topology, geometry, and attributes.

## 1. Topology Hierarchy

Parasolid's topology forms a strict containment hierarchy. OpenSolid replicates this:

```
Body
 └── Region (for general bodies; solid bodies have exactly 1 material region)
      └── Shell
           └── Face
                └── Loop
                     └── Fin (half-edge)
                          └── Edge
                               └── Vertex
```

### 1.1 Body

The top-level container. A body has a type that constrains its topology:

```rust
pub enum BodyType {
    /// Closed volume (manifold, oriented, closed shells)
    Solid,
    /// Open surfaces (non-closed shells, sheet bodies)
    Sheet,
    /// Curves only (no faces)
    Wire,
    /// Mixed dimensionality (Parasolid's "general body")
    General,
    /// Minimum body — single vertex, no edges or faces
    Minimum,
}

pub struct Body {
    pub id: EntityId<Body>,
    pub body_type: BodyType,
    pub shells: Vec<EntityId<Shell>>,
    pub regions: Vec<EntityId<Region>>,
    pub attributes: AttributeSet,
    pub transform: Option<Transform3>,
}
```

### 1.2 Region

Regions represent volumes bounded by shells. A solid body with no internal voids has
two regions: one material region (inside) and one void region (outside). Shells with
internal voids create additional regions.

```rust
pub struct Region {
    pub id: EntityId<Region>,
    pub body: EntityId<Body>,
    pub shells: Vec<EntityId<Shell>>,  // Bounding shells
    pub is_material: bool,             // true = solid material, false = void
}
```

### 1.3 Shell

A connected set of faces forming a boundary. Solid bodies have closed shells;
sheet bodies have open shells.

```rust
pub struct Shell {
    pub id: EntityId<Shell>,
    pub body: EntityId<Body>,
    pub region: Option<EntityId<Region>>,
    pub faces: Vec<EntityId<Face>>,
    pub is_closed: bool,               // true = watertight (encloses volume)
    pub orientation: ShellOrientation,  // Outward-pointing normals convention
}

pub enum ShellOrientation {
    /// Face normals point outward from material
    Outward,
    /// Face normals point inward (inner void shell)
    Inward,
}
```

### 1.4 Face

A bounded region on a surface. Each face references exactly one underlying surface
and has one outer loop plus zero or more inner loops (holes).

```rust
pub struct Face {
    pub id: EntityId<Face>,
    pub shell: EntityId<Shell>,
    pub surface: EntityId<Surface>,
    pub sense: FaceSense,              // Does face normal agree with surface normal?
    pub outer_loop: EntityId<Loop>,
    pub inner_loops: Vec<EntityId<Loop>>,
    pub attributes: AttributeSet,
}

pub enum FaceSense {
    /// Face normal = surface normal
    Positive,
    /// Face normal = -surface normal
    Negative,
}
```

### 1.5 Loop

An ordered sequence of fins (half-edges) forming a closed boundary on a face.
Outer loops run counter-clockwise when viewed from the face normal direction.
Inner loops (holes) run clockwise.

```rust
pub struct Loop {
    pub id: EntityId<Loop>,
    pub face: EntityId<Face>,
    pub fins: Vec<EntityId<Fin>>,
    pub loop_type: LoopType,
}

pub enum LoopType {
    /// Standard loop bounding a face region
    Outer,
    /// Inner loop (hole in the face)
    Inner,
    /// Degenerate loop (single vertex, e.g., cone apex)
    Vertex,
    /// Loop at a singularity (e.g., sphere pole)
    Singular,
}
```

### 1.6 Fin (Half-Edge)

The fin is Parasolid's half-edge. Each edge has exactly two fins — one for each
adjacent face. The fin records the direction of traversal and links to adjacency.

```rust
pub struct Fin {
    pub id: EntityId<Fin>,
    pub edge: EntityId<Edge>,
    pub loop_ref: EntityId<Loop>,
    pub sense: FinSense,               // Same or opposite to edge direction
    pub next: EntityId<Fin>,           // Next fin in loop
    pub prev: EntityId<Fin>,           // Previous fin in loop
    pub mate: EntityId<Fin>,           // Opposite fin on other face (same edge)
    pub pcurve: Option<EntityId<Curve>>, // 2D curve in this face's parameter space (SP-curve for tolerant edges)
}

pub enum FinSense {
    /// Fin traverses edge in its natural direction
    Forward,
    /// Fin traverses edge in reverse
    Reversed,
}
```

### 1.7 Edge

A bounded curve segment between two vertices. Edges are shared between exactly
two faces (manifold) or may be free (wire bodies, sheet boundaries).

```rust
pub struct Edge {
    pub id: EntityId<Edge>,
    pub curve: EntityId<Curve>,
    pub start_vertex: EntityId<Vertex>,
    pub end_vertex: EntityId<Vertex>,
    pub t_start: f64,                  // Parameter at start vertex
    pub t_end: f64,                    // Parameter at end vertex
    pub tolerance: f64,                // Tolerant modeling: max gap to adjacent faces
    pub fins: [EntityId<Fin>; 2],      // The two half-edges (or 1 for free edges)
}

impl Edge {
    pub fn is_tolerant(&self) -> bool {
        self.tolerance > SYSTEM_RESOLUTION * 10.0
    }
}
```

### 1.8 Vertex

A point in 3D space. Vertices are shared between edges. Like edges, they carry tolerance.

```rust
pub struct Vertex {
    pub id: EntityId<Vertex>,
    pub point: Point3,
    pub tolerance: f64,                // Max distance to actual intersection point
    pub edges: Vec<EntityId<Edge>>,    // All edges meeting at this vertex
}
```

## 2. Geometry Types

### 2.1 Curves

All curves are parameterized: a function from parameter t ∈ [t_min, t_max] → Point3.

```rust
pub enum Curve {
    /// Infinite straight line
    Line(LineCurve),
    /// Full or partial circle
    Circle(CircleCurve),
    /// Full or partial ellipse
    Ellipse(EllipseCurve),
    /// Hyperbola (STEP: HYPERBOLA)
    Hyperbola(HyperbolaCurve),
    /// Parabola (STEP: PARABOLA)
    Parabola(ParabolaCurve),
    /// Non-uniform rational B-spline
    BSpline(BSplineCurve),
    /// Intersection of two surfaces (implicit curve)
    Intersection(IntersectionCurve),
    /// Curve on a surface (2D parameterization)
    Pcurve(Pcurve),
    /// Curve offset from another curve
    Offset(OffsetCurve),
    /// Portion of another curve (reparameterized)
    Trimmed(TrimmedCurve),
    /// Composite of multiple curve segments
    Composite(CompositeCurve),
    /// Helix (constant radius + constant pitch)
    Helix(HelixCurve),
}
```

#### Line
```rust
pub struct LineCurve {
    pub origin: Point3,
    pub direction: UnitVector3,
}
// Evaluation: P(t) = origin + t * direction
```

#### Circle
```rust
pub struct CircleCurve {
    pub center: Point3,
    pub normal: UnitVector3,
    pub ref_direction: UnitVector3,  // Defines t=0 point
    pub radius: f64,
}
// Evaluation: P(t) = center + radius * (cos(t) * ref_dir + sin(t) * (normal × ref_dir))
```

#### Ellipse
```rust
pub struct EllipseCurve {
    pub center: Point3,
    pub normal: UnitVector3,
    pub major_axis: UnitVector3,
    pub major_radius: f64,
    pub minor_radius: f64,
}
```

#### B-Spline Curve (NURBS)
```rust
pub struct BSplineCurve {
    pub degree: u32,
    pub control_points: Vec<Point3>,
    pub weights: Option<Vec<f64>>,     // None = non-rational (polynomial B-spline)
    pub knots: Vec<f64>,
    pub knot_multiplicities: Vec<u32>,
    pub is_periodic: bool,
}
```

#### Intersection Curve
```rust
/// A curve defined implicitly as the intersection of two surfaces.
/// Stores a B-spline approximation for fast evaluation, plus references
/// to the original surfaces for exact refinement.
pub struct IntersectionCurve {
    pub surface_a: EntityId<Surface>,
    pub surface_b: EntityId<Surface>,
    pub approximation: BSplineCurve,   // Approximate curve for evaluation
    pub tolerance: f64,                // Max deviation of approximation from true intersection
    pub sense: bool,                   // Direction relative to cross product of normals
}
```

#### Pcurve (Parameter-Space Curve)
```rust
/// A 2D curve in the parameter space of a surface.
/// Used for trim curves on faces.
pub struct Pcurve {
    pub surface: EntityId<Surface>,
    pub curve_2d: Curve2D,             // 2D B-spline in (u,v) space
}
```

### 2.2 Surfaces

All surfaces are parameterized: a function from (u, v) → Point3.

```rust
pub enum Surface {
    /// Infinite flat plane
    Plane(PlaneSurface),
    /// Cylinder (possibly infinite)
    Cylinder(CylinderSurface),
    /// Cone (half-angle)
    Cone(ConeSurface),
    /// Sphere
    Sphere(SphereSurface),
    /// Torus
    Torus(TorusSurface),
    /// Non-uniform rational B-spline surface
    BSpline(BSplineSurface),
    /// Surface offset from another surface at constant distance
    Offset(OffsetSurface),
    /// Surface of revolution
    Revolution(RevolutionSurface),
    /// Surface swept along a path
    Swept(SweptSurface),
    /// Ruled surface between two curves
    Ruled(RuledSurface),
}
```

#### Plane
```rust
pub struct PlaneSurface {
    pub origin: Point3,
    pub normal: UnitVector3,
    pub u_axis: UnitVector3,
    pub v_axis: UnitVector3,           // = normal × u_axis
}
// Evaluation: P(u,v) = origin + u * u_axis + v * v_axis
```

#### Cylinder
```rust
pub struct CylinderSurface {
    pub origin: Point3,                // Point on axis
    pub axis: UnitVector3,
    pub ref_direction: UnitVector3,    // Defines u=0
    pub radius: f64,
}
// Evaluation: P(u,v) = origin + v*axis + radius*(cos(u)*ref_dir + sin(u)*(axis×ref_dir))
```

#### Cone
```rust
pub struct ConeSurface {
    pub origin: Point3,                // Apex of cone
    pub axis: UnitVector3,
    pub ref_direction: UnitVector3,
    pub half_angle: f64,               // Radians (0, π/2)
}
```

#### Sphere
```rust
pub struct SphereSurface {
    pub center: Point3,
    pub axis: UnitVector3,             // North pole direction
    pub ref_direction: UnitVector3,    // Defines u=0 meridian
    pub radius: f64,
}
// Evaluation: P(u,v) = center + radius*(cos(v)*cos(u)*ref + cos(v)*sin(u)*(axis×ref) + sin(v)*axis)
```

#### Torus
```rust
pub struct TorusSurface {
    pub center: Point3,
    pub axis: UnitVector3,
    pub ref_direction: UnitVector3,
    pub major_radius: f64,            // Distance from center to tube center
    pub minor_radius: f64,            // Tube radius
}
```

#### B-Spline Surface (NURBS)
```rust
pub struct BSplineSurface {
    pub degree_u: u32,
    pub degree_v: u32,
    pub control_points: Vec<Vec<Point3>>,  // [u_count][v_count]
    pub weights: Option<Vec<Vec<f64>>>,    // None = non-rational
    pub knots_u: Vec<f64>,
    pub knot_multiplicities_u: Vec<u32>,
    pub knots_v: Vec<f64>,
    pub knot_multiplicities_v: Vec<u32>,
    pub is_periodic_u: bool,
    pub is_periodic_v: bool,
}
```

#### Offset Surface
```rust
pub struct OffsetSurface {
    pub base_surface: EntityId<Surface>,
    pub offset_distance: f64,          // Positive = outward from normal
}
// Evaluation: P(u,v) = base.P(u,v) + offset * base.normal(u,v)
// NOTE: Offset surfaces can self-intersect — must detect and handle
```

### 2.3 Points and Vectors

```rust
/// A point in 3D Euclidean space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A vector in 3D space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Vector3 {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

/// A unit vector (magnitude = 1). Construction is fallible.
#[derive(Clone, Copy, Debug)]
pub struct UnitVector3(Vector3);

/// A 2D point (used for parameter-space curves).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Point2 {
    pub x: f64,
    pub y: f64,
}
```

### 2.4 Transforms

```rust
/// Affine transformation (rotation + translation + optional scale).
pub struct Transform3 {
    pub matrix: Matrix3x3,             // Rotation + scale
    pub translation: Vector3,
}

/// 4x4 homogeneous transformation matrix for general transforms.
pub struct Matrix4x4 {
    pub data: [[f64; 4]; 4],
}
```

## 3. Attributes

Attributes are key-value metadata attached to any topological entity.

```rust
pub struct AttributeSet {
    entries: Vec<Attribute>,
}

pub enum Attribute {
    /// Unique integer tag (Parasolid's PK_ATTRIB_tag equivalent)
    Tag(i64),
    /// Named string attribute
    Named { key: String, value: String },
    /// Color (RGB, 0-1 range)
    Color { r: f64, g: f64, b: f64 },
    /// Display name
    Name(String),
    /// User-defined binary blob
    UserData { type_id: u32, data: Vec<u8> },
    /// Layer/level assignment
    Layer(u32),
    /// Material properties
    Material(MaterialProps),
}

pub struct MaterialProps {
    pub density: Option<f64>,          // kg/m³
    pub ambient: Option<[f64; 3]>,
    pub diffuse: Option<[f64; 3]>,
    pub specular: Option<[f64; 3]>,
    pub shininess: Option<f64>,
}
```

## 4. Intervals and Domains

```rust
/// A closed interval [min, max] on the real line.
#[derive(Clone, Copy, Debug)]
pub struct Interval {
    pub min: f64,
    pub max: f64,
}

/// A 2D parameter domain (bounding box in UV space).
#[derive(Clone, Copy, Debug)]
pub struct Domain2 {
    pub u: Interval,
    pub v: Interval,
}

/// Parameterization metadata for a curve/surface.
pub struct Parameterization {
    pub domain: Interval,              // Or Domain2 for surfaces
    pub is_periodic: bool,
    pub period: Option<f64>,
}
```

## 5. Topology Validation Invariants

A valid body must satisfy these invariants (enforced by `opensolid-check`):

1. **Manifold**: Every edge in a solid body is shared by exactly 2 faces
2. **Closed shells**: Solid body shells are watertight (no free edges)
3. **Consistent orientation**: All face normals in a shell point outward
4. **Vertex sharing**: Edges sharing a vertex must have consistent geometry
5. **Loop closure**: Every loop is a closed sequence of fins
6. **Fin pairing**: Every fin has exactly one mate (for manifold bodies)
7. **Geometry consistency**: Edge curve lies on both adjacent face surfaces
   (within tolerance)
8. **No self-intersection**: No face intersects itself or another face in the
   same shell (except at shared edges/vertices)
9. **Parameter consistency**: Edge t_start/t_end correspond to actual vertex
   positions on the curve
10. **Tolerance bounds**: No vertex/edge tolerance exceeds configured maximum
