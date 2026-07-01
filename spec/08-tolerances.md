# 08 — Tolerances and Precision

How the kernel handles floating-point imprecision.

## 1. The Fundamental Problem

B-rep maintains dual bookkeeping:
- **Geometry**: Exact mathematical definitions (curve equations, surface equations)
- **Topology**: Connectivity graph (what's adjacent to what)

These must agree: an edge's curve must lie on both adjacent faces' surfaces. But
floating-point arithmetic means "lies on" is never exact. The kernel must define
what "close enough" means.

## 2. Tolerance Model

### 2.1 System Resolution

The absolute minimum tolerance — the smallest meaningful distance:

```rust
/// System resolution: distances smaller than this are considered zero.
/// This is the precision floor of the kernel.
pub const SYSTEM_RESOLUTION: f64 = 1e-10;  // 0.1 nanometers

/// Angular resolution for direction comparisons.
pub const ANGULAR_RESOLUTION: f64 = 1e-12;  // radians
```

### 2.2 Entity Tolerances

Following Parasolid's model, tolerances are attached to topological entities:

```rust
/// Every edge carries a tolerance value.
/// This is the maximum distance between:
///   - The edge's curve
///   - The actual intersection of its adjacent faces' surfaces
pub struct EdgeTolerance(f64);

/// Every vertex carries a tolerance value.
/// This is the maximum distance between:
///   - The vertex's point
///   - The actual position on any adjacent edge's curve at its endpoint parameter
pub struct VertexTolerance(f64);

impl EdgeTolerance {
    /// A precise edge: tolerance at system resolution.
    pub const PRECISE: Self = Self(SYSTEM_RESOLUTION);

    /// Check if this edge is tolerant (above system resolution).
    pub fn is_tolerant(&self) -> bool { self.0 > SYSTEM_RESOLUTION * 10.0 }
}
```

### 2.3 Body Classification

```rust
pub enum BodyPrecision {
    /// All edges/vertices at system resolution. Gold standard.
    Precise,
    /// Some edges/vertices have elevated tolerances.
    /// The body is valid but has precision gaps.
    Tolerant {
        max_edge_tolerance: f64,
        max_vertex_tolerance: f64,
        tolerant_edge_count: usize,
    },
}

impl Kernel {
    /// Classify a body's precision level.
    pub fn body_precision(&self, body: EntityId<Body>) -> BodyPrecision;

    /// Get the maximum tolerance in a body.
    pub fn max_tolerance(&self, body: EntityId<Body>) -> f64;
}
```

## 3. Tolerance Propagation

Operations can increase tolerances:

### 3.1 Boolean Operations

When two surfaces intersect, the intersection curve is computed numerically (SSI).
The resulting edge has a tolerance equal to the SSI approximation error.

```
Input bodies: precise (tolerance = 1e-10)
After boolean: some edges may have tolerance up to ~1e-6 (SSI fitting error)
```

### 3.2 Import

STEP files from other systems may have tolerance mismatches:
- Different systems use different precision targets
- Unit conversion introduces rounding
- Geometry was approximate in the source system

Import may produce bodies with tolerances up to ~0.01mm.

### 3.3 Tolerance Budgeting

```rust
pub struct ToleranceConfig {
    /// Maximum allowed tolerance on any edge (operations fail if exceeded).
    pub max_allowed_tolerance: f64,          // Default: 0.01 (10 microns)
    /// Tolerance target for SSI computation.
    pub ssi_tolerance: f64,                  // Default: 1e-7
    /// Tolerance for edge-surface consistency checking.
    pub consistency_tolerance: f64,          // Default: 1e-6
    /// Tolerance for vertex merging during sewing.
    pub sew_tolerance: f64,                  // Default: 1e-4
    /// Below this, consider vertices/edges as precise.
    pub precise_threshold: f64,              // Default: 1e-9
}
```

## 4. Tolerance Reduction (Healing)

Operations that reduce tolerances to improve body precision:

```rust
impl Kernel {
    /// Attempt to reduce tolerances on a body.
    ///
    /// Strategies:
    /// 1. Recompute edge curves as exact surface-surface intersection
    /// 2. Refit vertex positions to minimize deviation
    /// 3. Adjust surface parameterizations to improve edge consistency
    ///
    /// Returns: whether tolerances were reduced and by how much.
    pub fn reduce_tolerances(
        &mut self,
        body: EntityId<Body>,
        target: f64,
    ) -> ToleranceReductionResult;
}

pub struct ToleranceReductionResult {
    pub max_before: f64,
    pub max_after: f64,
    pub edges_improved: usize,
    pub vertices_improved: usize,
    pub fully_precise: bool,  // All tolerances now at system resolution
}
```

## 5. Numerical Robustness Techniques

### 5.1 Exact Predicates

For topological decisions (inside/outside, orientation), use exact arithmetic:

```rust
/// Determine the orientation of point D relative to plane ABC.
/// Uses Shewchuk's adaptive precision (robust crate).
/// Returns: positive if D is above, negative if below, zero if coplanar.
pub fn orient3d(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> f64;

/// Determine if points are collinear (within resolution).
pub fn are_collinear(a: &Point3, b: &Point3, c: &Point3) -> bool;

/// Determine if points are coplanar (within resolution).
pub fn are_coplanar(a: &Point3, b: &Point3, c: &Point3, d: &Point3) -> bool;
```

### 5.2 Interval Arithmetic

For bounding computation errors during SSI:

```rust
/// An interval [lo, hi] bounding the true value.
/// Distinct from the parameter-domain `Interval` in opensolid-math — this is
/// for error-bounding arithmetic, not parameter ranges.
pub struct BoundedInterval {
    pub lo: f64,
    pub hi: f64,
}

impl BoundedInterval {
    pub fn contains(&self, value: f64) -> bool;
    pub fn width(&self) -> f64;
    pub fn midpoint(&self) -> f64;
}

/// Interval arithmetic operations (for error bounding).
impl std::ops::Add for BoundedInterval { ... }
impl std::ops::Mul for BoundedInterval { ... }
// etc.
```

### 5.3 Perturbation

When geometric computations produce degenerate results (zero-length vectors, etc.),
use controlled perturbation:

```rust
/// Perturb a point slightly to escape a degenerate configuration.
/// Used as a last resort when exact computation fails.
pub fn perturb_point(point: &Point3, epsilon: f64) -> Point3;
```

## 6. Comparison Operations

```rust
/// Compare two floating-point values within tolerance.
pub fn approx_equal(a: f64, b: f64, tolerance: f64) -> bool {
    (a - b).abs() <= tolerance
}

/// Compare two points within tolerance.
pub fn points_equal(a: &Point3, b: &Point3, tolerance: f64) -> bool {
    a.distance_to(b) <= tolerance
}

/// Compare two unit vectors (angular tolerance).
pub fn directions_equal(a: &UnitVector3, b: &UnitVector3, angular_tol: f64) -> bool {
    let dot = a.dot(b).clamp(-1.0, 1.0);
    dot.acos() <= angular_tol
}

/// Compare two curves (sample and check point-wise distance).
pub fn curves_equal(
    a: &Curve, a_range: Interval,
    b: &Curve, b_range: Interval,
    tolerance: f64,
    sample_count: usize,
) -> bool;
```

## 7. Formal Tolerance Guarantees

The kernel provides these hard guarantees:

### 7.1 Invariants (always true, or the kernel has a bug)

1. **Edge-surface consistency**: For any edge E with tolerance T, every point on
   E's curve is within T of both adjacent faces' surfaces.
2. **Vertex-edge consistency**: For any vertex V with tolerance T, V's point is
   within T of the endpoint of every adjacent edge's curve.
3. **Monotonic tolerance propagation**: An operation's output tolerance is bounded
   by a known function of its input tolerances. Tolerances never grow unboundedly.
4. **Tolerance ordering**: vertex tolerance <= edge tolerance for adjacent entities.

### 7.2 Operation Tolerance Bounds

| Operation | Input Tolerance | Output Tolerance Bound |
|-----------|----------------|------------------------|
| Primitive creation | N/A | System resolution (exact) |
| Boolean (precise inputs) | ≤ system_resolution | ≤ ssi_tolerance (1e-7) |
| Boolean (tolerant inputs) | ≤ T | ≤ max(T, ssi_tolerance) |
| STEP import + heal | Any | ≤ max_acceptable_tolerance |
| Fillet | ≤ T | ≤ max(T, ssi_tolerance) |
| Offset/shell | ≤ T | ≤ max(T, ssi_tolerance) |
| Tolerance reduction | ≤ T | ≤ min(T, target) |

### 7.3 What Happens When Tolerance Would Exceed Limits

```rust
pub enum ToleranceViolation {
    /// Operation would produce edge tolerance above max_allowed_tolerance.
    /// The operation fails with this error — body is not modified.
    ExceedsLimit {
        edge: EntityId<Edge>,
        computed_tolerance: f64,
        limit: f64,
        suggestion: &'static str,
    },
}
```

The kernel NEVER silently produces a body with tolerance above `max_allowed_tolerance`.
It either succeeds within bounds or fails with an actionable error.

### 7.4 Context Table

| Operation | Typical Input Tolerance | Typical Output Tolerance |
|-----------|------------------------|-------------------------|
| Primitive creation | 0 (exact analytic) | System resolution |
| Boolean (precise inputs) | System resolution | 1e-7 to 1e-6 |
| Boolean (tolerant inputs) | Up to 1e-4 | Up to 1e-3 |
| STEP import | Variable (0 to 0.01) | Import tolerance + healing |
| Fillet | System resolution | 1e-7 (fillet surface fitting) |
| Offset/shell | System resolution | 1e-6 (offset intersection) |
| After healing | Variable | Reduced (ideally system resolution) |
