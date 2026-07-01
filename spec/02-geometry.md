# 02 — Geometry: Curves and Surfaces

Algorithms for evaluating, interrogating, and intersecting geometric entities.

## Dispatch Model

The `CurveEval` and `SurfaceEval` traits define the interface contract — each concrete
curve/surface type implements them. However, **the traits are never used as `dyn` trait
objects**. Public API dispatches through the `Curve`/`Surface` enums via match:

```rust
impl Curve {
    pub fn point_at(&self, t: f64) -> Point3 {
        match self {
            Curve::Line(c) => c.point_at(t),
            Curve::Circle(c) => c.point_at(t),
            Curve::BSpline(c) => c.point_at(t),
            // ... exhaustive
        }
    }
}
```

This gives static dispatch (inlinable, SIMD-friendly) while the trait ensures every
type has the same method set. The trait exists for implementation organization, not
polymorphism.

## 1. Curve Evaluation Interface

Every curve type must implement:

```rust
pub trait CurveEval {
    /// Evaluate point at parameter t.
    fn point_at(&self, t: f64) -> Point3;

    /// First derivative (tangent vector, not necessarily unit length).
    fn derivative_at(&self, t: f64) -> Vector3;

    /// Second derivative (curvature direction).
    fn second_derivative_at(&self, t: f64) -> Vector3;

    /// Unit tangent at parameter t. Returns None at degenerate points (cusps,
    /// inflection points where derivative is zero).
    fn tangent_at(&self, t: f64) -> Option<UnitVector3>;

    /// Curvature (1/radius) at parameter t.
    fn curvature_at(&self, t: f64) -> f64;

    /// Parameter domain [t_min, t_max].
    fn domain(&self) -> Interval;

    /// Is the curve periodic (e.g., full circle)?
    fn is_periodic(&self) -> bool;

    /// Period (if periodic).
    fn period(&self) -> Option<f64>;

    /// Is the curve closed (start == end)?
    fn is_closed(&self) -> bool;

    /// Arc length between two parameters (numerical integration).
    fn arc_length(&self, t0: f64, t1: f64) -> f64;

    /// Find parameter closest to a given point.
    fn project_point(&self, point: &Point3) -> (f64, f64); // (t, distance)

    /// Compute bounding box.
    fn bounding_box(&self) -> BoundingBox3;
}
```

## 2. Surface Evaluation Interface

```rust
pub trait SurfaceEval {
    /// Evaluate point at parameters (u, v).
    fn point_at(&self, u: f64, v: f64) -> Point3;

    /// Partial derivative with respect to u.
    fn du(&self, u: f64, v: f64) -> Vector3;

    /// Partial derivative with respect to v.
    fn dv(&self, u: f64, v: f64) -> Vector3;

    /// Unit normal vector at (u, v). Returns None at singularities
    /// (sphere poles, cone apex, degenerate patches where du × dv = 0).
    /// Each surface type handles its own singularities — e.g., SphereSurface
    /// computes pole normals via L'Hôpital, not the cross product.
    fn normal_at(&self, u: f64, v: f64) -> Option<UnitVector3>;

    /// Second derivatives (for curvature computation).
    fn duu(&self, u: f64, v: f64) -> Vector3;
    fn duv(&self, u: f64, v: f64) -> Vector3;
    fn dvv(&self, u: f64, v: f64) -> Vector3;

    /// Principal curvatures and directions at (u, v).
    fn curvature_at(&self, u: f64, v: f64) -> SurfaceCurvature;

    /// Parameter domain.
    fn domain(&self) -> Domain2;

    /// Periodicity in u and v directions.
    fn is_periodic_u(&self) -> bool;
    fn is_periodic_v(&self) -> bool;

    /// Find (u, v) parameters closest to a given point.
    fn project_point(&self, point: &Point3) -> (f64, f64, f64); // (u, v, distance)

    /// Compute bounding box.
    fn bounding_box(&self) -> BoundingBox3;

    /// Is the point (u, v) at a singularity (e.g., sphere pole)?
    fn is_singular_at(&self, u: f64, v: f64) -> bool;
}

pub struct SurfaceCurvature {
    pub k1: f64,                       // Maximum principal curvature
    pub k2: f64,                       // Minimum principal curvature
    pub dir1: UnitVector3,             // Direction of k1
    pub dir2: UnitVector3,             // Direction of k2
    pub gaussian: f64,                 // k1 * k2
    pub mean: f64,                     // (k1 + k2) / 2
}
```

## 3. NURBS Evaluation

The B-spline basis and NURBS evaluation is the most performance-critical code path.

### 3.1 B-Spline Basis Functions

```rust
/// Evaluate all non-zero B-spline basis functions at parameter t.
/// Uses the Cox-de Boor recursion with the optimized (non-recursive) form.
///
/// Returns: basis[i] for i in [span-degree .. span] where
/// span is the knot span containing t.
pub fn basis_functions(
    degree: u32,
    knots: &[f64],
    t: f64,
) -> (usize, Vec<f64>);  // (span_index, basis_values)

/// Evaluate basis functions and their derivatives up to order k.
pub fn basis_functions_derivs(
    degree: u32,
    knots: &[f64],
    t: f64,
    order: u32,
) -> (usize, Vec<Vec<f64>>);  // (span, derivs[order][basis_index])
```

### 3.2 Curve Evaluation (De Boor's Algorithm)

```rust
/// Evaluate a B-spline curve at parameter t using De Boor's algorithm.
/// This is numerically stable and efficient: O(degree²) per evaluation.
pub fn evaluate_bspline_curve(curve: &BSplineCurve, t: f64) -> Point3;

/// Evaluate a rational B-spline (NURBS) curve.
/// Projects to 4D homogeneous coordinates, evaluates, then divides by w.
pub fn evaluate_nurbs_curve(curve: &BSplineCurve, t: f64) -> Point3;

/// Evaluate curve and derivatives up to order k.
pub fn evaluate_curve_derivs(
    curve: &BSplineCurve,
    t: f64,
    order: u32,
) -> Vec<Vector3>;
```

### 3.3 Surface Evaluation

```rust
/// Evaluate a B-spline surface at (u, v).
/// Uses tensor-product evaluation: evaluate u-direction first, then v-direction.
pub fn evaluate_bspline_surface(surface: &BSplineSurface, u: f64, v: f64) -> Point3;

/// Evaluate surface and partial derivatives.
pub fn evaluate_surface_derivs(
    surface: &BSplineSurface,
    u: f64, v: f64,
    order: u32,
) -> Vec<Vec<Vector3>>;  // derivs[du_order][dv_order]
```

### 3.4 Knot Operations

```rust
/// Insert a knot into a B-spline curve (Boehm's algorithm).
/// Returns new control points and updated knot vector.
pub fn knot_insert_curve(
    curve: &BSplineCurve,
    t: f64,
    multiplicity: u32,
) -> BSplineCurve;

/// Refine a knot vector by inserting multiple knots simultaneously (Oslo algorithm).
pub fn knot_refine_curve(
    curve: &BSplineCurve,
    new_knots: &[f64],
) -> BSplineCurve;

/// Remove a knot from a B-spline curve (within tolerance).
pub fn knot_remove_curve(
    curve: &BSplineCurve,
    t: f64,
    tolerance: f64,
) -> Option<BSplineCurve>;

/// Elevate the degree of a B-spline curve by 1.
pub fn degree_elevate_curve(curve: &BSplineCurve) -> BSplineCurve;

/// Decompose a B-spline curve into Bézier segments.
pub fn decompose_to_bezier(curve: &BSplineCurve) -> Vec<BezierSegment>;
```

## 4. Curve-Curve Intersection

### 4.1 Strategy

Curve-curve intersection is foundational — used by booleans, trims, and projections.
Algorithm selection depends on curve type pair:

| Curve A | Curve B | Algorithm |
|---------|---------|-----------|
| Line | Line | Analytic (closest approach) |
| Line | Circle | Analytic (quadratic) |
| Line | BSpline | Bézier clipping |
| Circle | Circle | Analytic |
| Circle | BSpline | Bézier clipping |
| BSpline | BSpline | Bézier clipping + subdivision |

### 4.2 Bézier Clipping

The workhorse algorithm for NURBS intersection:

```rust
/// Find all intersection points between two curves using Bézier clipping.
///
/// Algorithm:
/// 1. Decompose both curves into Bézier segments
/// 2. Use convex hull property to eliminate non-intersecting pairs
/// 3. For potentially intersecting pairs, clip parameter domain
/// 4. Recurse until parameter interval is below tolerance
/// 5. Refine with Newton-Raphson
///
/// Returns: Vec<(t_a, t_b)> parameter pairs at intersection points
pub fn intersect_curves(
    curve_a: &Curve,
    curve_b: &Curve,
    tolerance: f64,
) -> Vec<CurveIntersection>;

pub struct CurveIntersection {
    pub t_a: f64,                      // Parameter on curve A
    pub t_b: f64,                      // Parameter on curve B
    pub point: Point3,                 // Intersection point
    pub is_tangent: bool,              // Are curves tangent at intersection?
}
```

## 5. Surface-Surface Intersection (SSI)

The hardest single algorithm in the kernel. SSI quality determines boolean robustness.

### 5.1 Overview

SSI produces one or more intersection curves. The algorithm must handle:
- Transverse intersections (typical case)
- Tangent intersections (surfaces touch along a curve)
- Partial overlaps (coincident surface patches)
- Self-intersections (offset surfaces)
- Singular points (where intersection curve has a cusp)

### 5.2 Algorithm: Marching + Subdivision

```rust
/// Compute the intersection curves between two surfaces.
///
/// High-level algorithm:
/// 1. Subdivide both surfaces into Bézier patches
/// 2. Use bounding box hierarchy to find potentially intersecting patch pairs
/// 3. For each pair, find starting points using subdivision or lattice evaluation
/// 4. March along intersection curve using predictor-corrector (Runge-Kutta)
/// 5. Detect boundary crossings and curve endpoints
/// 6. Fit result as B-spline curve with bounded approximation error
///
/// Returns: Vec of intersection curves in both parameter spaces
pub fn intersect_surfaces(
    surface_a: &Surface,
    surface_b: &Surface,
    tolerance: f64,
) -> SurfaceIntersectionResult;

pub struct SurfaceIntersectionResult {
    pub curves: Vec<SSICurve>,
    pub coincident_regions: Vec<CoincidentRegion>,
}

pub struct SSICurve {
    pub curve_3d: BSplineCurve,        // The 3D intersection curve
    pub pcurve_a: Curve2D,             // Curve in surface A's parameter space
    pub pcurve_b: Curve2D,             // Curve in surface B's parameter space
    pub tolerance: f64,                // Max deviation from true intersection
    pub is_closed: bool,
    pub boundary_intersections: Vec<BoundaryIntersection>,
}
```

### 5.3 Marching Method Detail

This is the hardest algorithm in the kernel. The description below is the target
design — implementation should reference Patrikalakis & Maekawa, "Shape Interrogation
for Computer Aided Design and Manufacturing" (2002) as the canonical source.

```
1. STARTING POINT DETECTION

   The lattice method alone misses curves that fall between grid points.
   Use a multi-strategy approach:

   a) Subdivide both surfaces into Bézier patches (knot insertion to full multiplicity)
   b) Build BVH over patches. Test all patch-pair bounding box overlaps.
   c) For each overlapping pair, subdivide recursively until:
      - Patches are "flat enough" (deviation from bilinear < tolerance), OR
      - Patches clearly don't intersect (bounding boxes separate), OR
      - Patch pair is small enough for Newton refinement
   d) For small overlapping pairs: sample center points of each patch,
      project onto the other surface. If projection distance < threshold,
      use as Newton starting point.
   e) Refine each candidate with Newton-Raphson in the 4D parameter space
      (u_a, v_a, u_b, v_b), solving: S_a(u_a, v_a) - S_b(u_b, v_b) = 0

   This guarantees detection of all intersection branches (no misses from
   grid spacing).

2. MARCHING (Predictor-Corrector)

   From each starting point, march in both directions:

   a) TANGENT COMPUTATION:
      t = n_a × n_b (cross product of surface normals)
      If |t| < epsilon: we're at a tangent point (branch point). See step 5.

   b) PREDICTOR (4th-order Runge-Kutta in parameter space):
      The marching ODE in parameter space of surface A is:
        du_a/ds = (∂S_a/∂v_a · t) / |∂S_a/∂u_a × ∂S_a/∂v_a|
        dv_a/ds = -(∂S_a/∂u_a · t) / |∂S_a/∂u_a × ∂S_a/∂v_a|
      Similarly for surface B. Step size h chosen per step 3.

   c) CORRECTOR (Newton-Raphson):
      After predictor gives approximate point, solve the 4D system:
        F(u_a, v_a, u_b, v_b) = S_a(u_a, v_a) - S_b(u_b, v_b) = 0
      Jacobian is [∂S_a/∂u_a, ∂S_a/∂v_a, -∂S_b/∂u_b, -∂S_b/∂v_b] (3×4).
      Solve via least-squares (QR). Max 10 iterations.
      If Newton fails to converge: halve step size and retry from last good point.
      If still fails after 3 halvings: mark as singular point, attempt bypass.

3. STEP SIZE CONTROL

   Adaptive step based on intersection curve curvature:
     h = min(
       h_max,
       max(h_min, chord_tolerance / curvature_estimate)
     )

   Curvature estimate uses the angle between successive tangent vectors.
   Also limit step so that parameter-space step doesn't exceed patch width / 4
   (prevents jumping over narrow features).

   Defaults:
     h_min = tolerance * 10
     h_max = min(domain_a_width, domain_b_width) / 20
     chord_tolerance = ssi_tolerance / 2

4. TERMINATION CONDITIONS

   Stop marching when ANY of:
   a) CLOSURE: new point is within 2×tolerance of the starting point of this
      curve AND we have marched at least 3 steps. Mark curve as closed.
   b) BOUNDARY: parameter hits edge of either surface's domain.
      Record which boundary (u_min, u_max, v_min, v_max of which surface).
   c) SINGULAR POINT: tangent magnitude drops below threshold.
      Record singular point location. May need to restart from other side.
   d) ANOTHER CURVE: new point is within tolerance of a point already on a
      different intersection curve (merge point).
   e) MAX STEPS: safety limit (10,000 steps). Log warning, return partial curve.

5. BRANCH POINT HANDLING (tangent intersections)

   When |n_a × n_b| < angular_threshold at a point:
   a) This is a branch point where intersection curves meet tangentially.
   b) Compute second-order contact direction using surface curvature tensors.
   c) Multiple curves may emanate from this point. For each exit direction:
      - Step slightly along that direction
      - Attempt to restart marching from the offset point
      - If corrector converges, we found a new branch
   d) Tangent intersection (surfaces touch along a curve) is detected when
      branch point handling finds the SAME curve on both sides. In this case,
      the entire curve segment is tangent — record this metadata on the SSICurve.

6. FITTING (marched points → B-spline)

   a) Collect all marched points with their parameter-space coordinates.
   b) Fit as B-spline curve using least-squares approximation:
      - Start with degree 3, minimal knots
      - Insert knots where error exceeds tolerance
      - Iterate until max error < ssi_tolerance
   c) Also fit pcurves (2D curves in each surface's parameter space):
      - Use the (u_a, v_a) coordinates from marching
      - Fit as 2D B-spline with same knot structure
   d) Verify: sample fitted curve at 10× density, check all samples are
      within tolerance of both surfaces. If not, add more knots and refit.
```

### 5.4 Special Cases

```rust
/// Analytic SSI for plane-analytic surface combinations.
/// Much faster and more robust than general marching.
pub fn intersect_plane_cylinder(plane: &PlaneSurface, cyl: &CylinderSurface) -> Vec<Curve>;
pub fn intersect_plane_cone(plane: &PlaneSurface, cone: &ConeSurface) -> Vec<Curve>;
pub fn intersect_plane_sphere(plane: &PlaneSurface, sphere: &SphereSurface) -> Vec<Curve>;
pub fn intersect_plane_torus(plane: &PlaneSurface, torus: &TorusSurface) -> Vec<Curve>;
pub fn intersect_cylinder_cylinder(a: &CylinderSurface, b: &CylinderSurface) -> Vec<Curve>;
// ... etc for all analytic-analytic pairs
```

## 6. Curve-Surface Intersection

```rust
/// Find all intersections of a curve with a surface.
///
/// Algorithm:
/// 1. If curve is a line and surface is analytic: use closed-form solution
/// 2. Otherwise: subdivide curve into Bézier segments, test bounding boxes
///    against surface, use Newton-Raphson refinement
pub fn intersect_curve_surface(
    curve: &Curve,
    surface: &Surface,
    tolerance: f64,
) -> Vec<CurveSurfaceIntersection>;

pub struct CurveSurfaceIntersection {
    pub t_curve: f64,                  // Parameter on curve
    pub uv_surface: (f64, f64),        // Parameters on surface
    pub point: Point3,
    pub is_tangent: bool,
}
```

## 7. Point Projection

```rust
/// Project a point onto a curve — find the closest point.
/// Uses Newton-Raphson with multiple starting points to avoid local minima.
pub fn project_point_to_curve(
    point: &Point3,
    curve: &Curve,
    tolerance: f64,
) -> Vec<ProjectionResult>;

/// Project a point onto a surface.
/// Uses Newton-Raphson in (u,v) space with subdivision-based starting points.
pub fn project_point_to_surface(
    point: &Point3,
    surface: &Surface,
    tolerance: f64,
) -> Vec<SurfaceProjectionResult>;

pub struct ProjectionResult {
    pub t: f64,
    pub point: Point3,
    pub distance: f64,
    pub is_boundary: bool,             // Closest point is at domain boundary
}

pub struct SurfaceProjectionResult {
    pub u: f64,
    pub v: f64,
    pub point: Point3,
    pub distance: f64,
    pub is_boundary: bool,
}
```

## 8. Geometric Queries

```rust
/// Compute minimum distance between two curves.
pub fn min_distance_curve_curve(a: &Curve, b: &Curve, tolerance: f64) -> f64;

/// Compute minimum distance between a curve and a surface.
pub fn min_distance_curve_surface(curve: &Curve, surface: &Surface, tolerance: f64) -> f64;

/// Compute minimum distance between two surfaces.
pub fn min_distance_surface_surface(a: &Surface, b: &Surface, tolerance: f64) -> f64;

/// Determine if a point is inside, outside, or on a solid body.
pub fn point_in_body(point: &Point3, body: &Body, tolerance: f64) -> Containment;

pub enum Containment {
    Inside,
    Outside,
    OnBoundary { face: EntityId<Face>, u: f64, v: f64 },
}
```

## 9. Analytic Surface Properties

For analytic surfaces, many operations have closed-form solutions:

```rust
/// Classify an analytic surface.
pub fn classify_surface(surface: &Surface) -> SurfaceClassification;

pub enum SurfaceClassification {
    Planar,
    Cylindrical { radius: f64 },
    Conical { half_angle: f64 },
    Spherical { radius: f64 },
    Toroidal { major: f64, minor: f64 },
    Freeform,  // NURBS
}

/// For analytic surfaces: compute exact area of a trimmed region.
/// (For NURBS: use numerical integration)
pub fn face_area(face: &Face, kernel: &Kernel) -> f64;

/// Compute surface normal at a point, handling singularities.
/// At singularities (sphere poles, cone apex), uses L'Hôpital's rule
/// to find the limiting normal direction.
pub fn safe_normal_at(
    surface: &Surface,
    u: f64,
    v: f64,
) -> Option<UnitVector3>;
```

## 10. Bounding Volume Hierarchy

All geometric operations use bounding boxes for acceleration:

```rust
pub struct BoundingBox3 {
    pub min: Point3,
    pub max: Point3,
}

impl BoundingBox3 {
    pub fn intersects(&self, other: &BoundingBox3) -> bool;
    pub fn contains_point(&self, point: &Point3) -> bool;
    pub fn union(&self, other: &BoundingBox3) -> BoundingBox3;
    pub fn expand(&self, distance: f64) -> BoundingBox3;
    pub fn diagonal(&self) -> f64;
}

/// Oriented bounding box (tighter fit for rotated geometry).
pub struct OBB3 {
    pub center: Point3,
    pub axes: [UnitVector3; 3],
    pub half_extents: [f64; 3],
}

/// BVH tree for spatial acceleration of face/edge queries.
pub struct BVHTree<T> {
    nodes: Vec<BVHNode<T>>,
}
```
