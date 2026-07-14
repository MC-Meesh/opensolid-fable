use crate::primitives::Sdf;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3, Transform3, Vector3};

/// An SDF placed by a rigid transform (rotation + translation).
///
/// Evaluates the inner SDF at the inverse-transformed query point. Rigid
/// isometries preserve Euclidean distance, so the field stays an exact
/// signed distance.
pub struct Transformed<S> {
    pub sdf: S,
    inverse: Transform3,
}

impl<S> Transformed<S> {
    pub fn new(sdf: S, transform: Transform3) -> Self {
        Self {
            sdf,
            inverse: transform.inverse(),
        }
    }
}

impl<S: Sdf> Sdf for Transformed<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(&(self.inverse * p))
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        // Chain rule through the isometry: ∇(g ∘ T⁻¹)(p) = R ∇g(T⁻¹p) with R
        // the forward rotation, i.e. the inverse of the stored inverse's.
        self.inverse
            .inverse_transform_vector(&self.sdf.grad(&(self.inverse * p)))
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // The inverse image of `b` is a rotated box; bound it by the AABB of
        // its corners. Conservative: the AABB is a superset of the rotated
        // box, and rigid motion preserves field values.
        let corners = (0..8).map(|i| {
            let p = Point3::new(
                if i & 1 == 0 { b.min.x } else { b.max.x },
                if i & 2 == 0 { b.min.y } else { b.max.y },
                if i & 4 == 0 { b.min.z } else { b.max.z },
            );
            self.inverse * p
        });
        self.sdf.eval_interval(&BoundingBox3::from_points(corners))
    }

    // Isometries preserve values; gradients rotate forward, as in `grad`.
    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let start = out.len();
        self.sdf.branches(&(self.inverse * p), tol, out);
        for branch in &mut out[start..] {
            branch.1 = self.inverse.inverse_transform_vector(&branch.1);
        }
    }
}

/// An SDF scaled uniformly about the origin by `factor > 0`:
/// `eval(p) = factor * inner.eval(p / factor)`.
///
/// Uniform scaling multiplies every Euclidean distance by the same factor,
/// so rescaling the inner value keeps the field an exact distance.
///
/// Non-uniform scale is deliberately excluded from this wrapper: it
/// stretches space by a direction-dependent amount (spheres become
/// ellipsoids), so no single correction factor can restore the inner value
/// to a distance — `|∇f|` drifts away from 1 and everything that relies on
/// the metric property (blend radii, meshing step bounds, offsets)
/// silently breaks. The dedicated [`AnisotropicScale`] operator provides
/// the re-normalized conservative bound instead.
pub struct UniformScale<S> {
    pub sdf: S,
    factor: f64,
}

impl<S> UniformScale<S> {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `factor` is not positive and finite.
    pub fn new(sdf: S, factor: f64) -> CoreResult<Self> {
        if factor <= 0.0 || !factor.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "factor",
                reason: format!("must be positive and finite, got {factor}"),
            });
        }
        Ok(Self { sdf, factor })
    }
}

impl<S: Sdf> Sdf for UniformScale<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(&Point3::from(p.coords / self.factor)) * self.factor
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        // ∇(k · g(p/k)) = k · (1/k) ∇g(p/k): the factors cancel exactly.
        self.sdf.grad(&Point3::from(p.coords / self.factor))
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Exact given the inner bound: shrink the box into the inner frame
        // and rescale the resulting interval (factor > 0 preserves order).
        let inner = BoundingBox3::new(
            Point3::from(b.min.coords / self.factor),
            Point3::from(b.max.coords / self.factor),
        );
        let i = self.sdf.eval_interval(&inner);
        Interval::new(i.lo * self.factor, i.hi * self.factor)
    }

    // Values scale by `factor` (so the activity tolerance shrinks going
    // in); gradients are unchanged, as in `grad`.
    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let start = out.len();
        self.sdf.branches(
            &Point3::from(p.coords / self.factor),
            tol / self.factor,
            out,
        );
        for branch in &mut out[start..] {
            branch.0 *= self.factor;
        }
    }
}

/// An SDF scaled per-axis about the origin by positive `factors`:
/// `eval(p) = min(factors) * inner(p ⊘ factors)`.
///
/// Unlike [`UniformScale`] the result is **not** an exact distance — no
/// anisotropic rescaling of an SDF can be (see the [`UniformScale`] docs).
/// Multiplying by the *smallest* factor keeps the field a conservative
/// bound: the sign (and therefore the surface) is exact, the Lipschitz
/// constant stays ≤ 1 (so the default `eval_interval` reasoning remains
/// valid), and magnitudes underestimate the true distance by at most the
/// max/min factor ratio. Metric-sensitive operators applied on top
/// (offset, shell, blend radii) will be distorted accordingly.
pub struct AnisotropicScale<S> {
    pub sdf: S,
    factors: Vector3,
    min_factor: f64,
}

impl<S> AnisotropicScale<S> {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if any factor is not positive and
    /// finite.
    pub fn new(sdf: S, factors: Vector3) -> CoreResult<Self> {
        if factors.iter().any(|f| *f <= 0.0 || !f.is_finite()) {
            return Err(CoreError::InvalidArgument {
                argument: "factors",
                reason: format!(
                    "must be positive and finite, got ({}, {}, {})",
                    factors.x, factors.y, factors.z
                ),
            });
        }
        Ok(Self {
            sdf,
            factors,
            min_factor: factors.x.min(factors.y).min(factors.z),
        })
    }

    fn to_inner(&self, p: &Point3) -> Point3 {
        Point3::new(
            p.x / self.factors.x,
            p.y / self.factors.y,
            p.z / self.factors.z,
        )
    }
}

impl<S: Sdf> Sdf for AnisotropicScale<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(&self.to_inner(p)) * self.min_factor
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        // ∇(k · g(A p)) = k · Aᵀ ∇g(A p) with A = diag(1 / factors).
        let g = self.sdf.grad(&self.to_inner(p));
        Vector3::new(
            g.x / self.factors.x,
            g.y / self.factors.y,
            g.z / self.factors.z,
        ) * self.min_factor
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Exact given the inner bound: map the box into the inner frame
        // (positive factors preserve per-axis order) and rescale.
        let inner = BoundingBox3::new(self.to_inner(&b.min), self.to_inner(&b.max));
        let i = self.sdf.eval_interval(&inner);
        Interval::new(i.lo * self.min_factor, i.hi * self.min_factor)
    }

    // Values scale by `min_factor`; gradients map through k · Aᵀ, as in
    // `grad`.
    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let start = out.len();
        self.sdf
            .branches(&self.to_inner(p), tol / self.min_factor, out);
        for branch in &mut out[start..] {
            branch.0 *= self.min_factor;
            let g = branch.1;
            branch.1 = Vector3::new(
                g.x / self.factors.x,
                g.y / self.factors.y,
                g.z / self.factors.z,
            ) * self.min_factor;
        }
    }
}

/// The signed field returned on the degenerate cross-section-collapse plane
/// (`1 + tan(angle)·h = 0`), where the taper's inverse map is undefined. The
/// collapse plane is only reachable for a draft steeper than the body, so
/// treating it as far outside keeps the sign correct there for any real part.
const TAPER_COLLAPSE_OUTSIDE: f64 = 1e30;

/// A linear **taper** (draft) of the field about a neutral plane.
///
/// Picks a pull axis `n` (the neutral-plane normal) and a neutral plane
/// `{ p : n·p = neutral }`, then scales each cross-section perpendicular to
/// `n` about the pull axis by `k(h) = 1 + tan(angle)·h`, where
/// `h = n·p − neutral` is the signed distance from the neutral plane. For
/// `angle > 0` sections on the `+n` side grow and sections on the `−n` side
/// shrink, so a prismatic body's side walls flare outward toward `+n` — the
/// mold-release draft about a parting plane. Points *on* the neutral plane
/// are fixed, and faces whose normal is `±n` (the caps) keep their position.
///
/// Like [`AnisotropicScale`], the taper stretches space by a
/// position-dependent amount, so the result is **not** an exact distance
/// field: the sign and the zero set — hence the meshed surface and every
/// boolean built on it — are exact, but magnitudes, and therefore any
/// offset / shell / blend radius applied *afterward*, are distorted by the
/// local taper factor. A side face at perpendicular distance `r` from the
/// pull axis tilts by `atan(r·tan angle)`, so `angle` is the exact face
/// draft only at unit distance; this is the F-Rep whole-body approximation
/// of the face-selective Draft feature.
///
/// [`eval_interval`](Sdf::eval_interval) stays conservative by
/// inverse-mapping the query box with interval arithmetic; where the box
/// spans the collapse plane it widens to the whole line rather than prune
/// unsoundly.
pub struct Taper<S> {
    pub sdf: S,
    /// Unit pull axis (neutral-plane normal).
    axis: Vector3,
    /// Neutral-plane offset along `axis`: the plane is `{ p : axis·p = neutral }`.
    neutral: f64,
    /// Taper rate `tan(angle)`.
    rate: f64,
}

impl<S> Taper<S> {
    /// Taper `sdf` about the plane through `neutral_point` with normal
    /// `pull`, by draft `angle` in radians.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `pull` is not a non-zero finite
    /// vector, if `neutral_point` is not finite, or if `angle` is not finite
    /// with `|angle| < π/2` (at which a cross-section would collapse or
    /// invert everywhere).
    pub fn new(sdf: S, pull: Vector3, neutral_point: Point3, angle: f64) -> CoreResult<Self> {
        let norm = pull.norm();
        if !norm.is_normal() || pull.iter().any(|c| !c.is_finite()) {
            return Err(CoreError::InvalidArgument {
                argument: "pull",
                reason: format!(
                    "must be a non-zero finite vector, got ({}, {}, {})",
                    pull.x, pull.y, pull.z
                ),
            });
        }
        if neutral_point.coords.iter().any(|c| !c.is_finite()) {
            return Err(CoreError::InvalidArgument {
                argument: "neutral_point",
                reason: format!(
                    "must be finite, got ({}, {}, {})",
                    neutral_point.x, neutral_point.y, neutral_point.z
                ),
            });
        }
        if !angle.is_finite() || angle.abs() >= std::f64::consts::FRAC_PI_2 {
            return Err(CoreError::InvalidArgument {
                argument: "angle",
                reason: format!("must be finite with |angle| < π/2 radians, got {angle}"),
            });
        }
        let axis = pull / norm;
        Ok(Self {
            neutral: axis.dot(&neutral_point.coords),
            rate: angle.tan(),
            axis,
            sdf,
        })
    }

    /// The taper factor `k = 1 + tan(angle)·(axis·p − neutral)` at `p`.
    fn factor(&self, p: &Point3) -> f64 {
        1.0 + self.rate * (self.axis.dot(&p.coords) - self.neutral)
    }

    /// World → inner (pre-taper) point; `None` on the collapse plane `k = 0`.
    fn to_inner(&self, p: &Point3) -> Option<Point3> {
        let k = self.factor(p);
        if k == 0.0 {
            return None;
        }
        let a = self.axis.dot(&p.coords);
        let lat = p.coords - self.axis * a;
        Some(Point3::from(self.axis * a + lat / k))
    }

    /// Map an inner gradient `g` at the pre-image of `p` back to the tapered
    /// field's gradient. Shared by [`grad`](Sdf::grad) and
    /// [`branches`](Sdf::branches): with `M = (1/k)I + c·nᵀ` the Jacobian of
    /// the inverse map, this is `Mᵀg = g/k + (c·g)·n`.
    fn pull_back_grad(&self, p: &Point3, k: f64, g: Vector3) -> Vector3 {
        let a = self.axis.dot(&p.coords);
        let lat = p.coords - self.axis * a;
        let coeff = self.axis.dot(&g) * (1.0 - 1.0 / k) - (self.rate / (k * k)) * lat.dot(&g);
        g / k + self.axis * coeff
    }
}

impl<S: Sdf> Sdf for Taper<S> {
    fn eval(&self, p: &Point3) -> f64 {
        match self.to_inner(p) {
            Some(q) => self.sdf.eval(&q),
            None => TAPER_COLLAPSE_OUTSIDE,
        }
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        let k = self.factor(p);
        if k == 0.0 {
            return self.axis;
        }
        let q = self.to_inner(p).expect("k != 0");
        self.pull_back_grad(p, k, self.sdf.grad(&q))
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        let pt = Interval::point;
        let bx = Interval::new(b.min.x, b.max.x);
        let by = Interval::new(b.min.y, b.max.y);
        let bz = Interval::new(b.min.z, b.max.z);
        // a = axis·p over the box, and the taper factor k(a).
        let a = pt(self.axis.x) * bx + pt(self.axis.y) * by + pt(self.axis.z) * bz;
        let k = pt(1.0) + pt(self.rate) * (a - pt(self.neutral));
        if k.contains_zero() {
            // The box spans the collapse plane; the inverse map blows up.
            return Interval::WHOLE;
        }
        // Inverse map per axis: q_j = a·n_j + (p_j − a·n_j) / k. Interval
        // arithmetic yields an axis-aligned superset of the true pre-image,
        // so the inner field's interval over it contains every eval on `b`.
        let inv = |bj: Interval, nj: f64| {
            let an = a * pt(nj);
            an + (bj - an).div(&k).expect("k has no zero")
        };
        let qx = inv(bx, self.axis.x);
        let qy = inv(by, self.axis.y);
        let qz = inv(bz, self.axis.z);
        self.sdf.eval_interval(&BoundingBox3::new(
            Point3::new(qx.lo, qy.lo, qz.lo),
            Point3::new(qx.hi, qy.hi, qz.hi),
        ))
    }

    // The taper leaves field *values* untouched (it only warps the domain),
    // so branch values and the activity tolerance pass through unchanged;
    // only the branch gradients are pulled back, exactly as `grad` does.
    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let k = self.factor(p);
        let Some(q) = self.to_inner(p) else {
            out.push((TAPER_COLLAPSE_OUTSIDE, self.axis));
            return;
        };
        let start = out.len();
        self.sdf.branches(&q, tol, out);
        for branch in &mut out[start..] {
            branch.1 = self.pull_back_grad(p, k, branch.1);
        }
    }
}

/// Chainable constructors for the transform wrappers. Each call wraps the
/// receiver, so transforms apply to the shape in the order they are chained:
/// `sdf.rotated(r).translated(t)` rotates first, then translates.
pub trait SdfTransformExt: Sdf + Sized {
    /// Apply an arbitrary rigid transform.
    fn transformed(self, transform: Transform3) -> Transformed<Self> {
        Transformed::new(self, transform)
    }

    /// Move by `offset`.
    fn translated(self, offset: Vector3) -> Transformed<Self> {
        self.transformed(Transform3::translation(offset.x, offset.y, offset.z))
    }

    /// Rotate about the origin. `axis_angle`'s direction is the rotation
    /// axis and its norm the angle in radians.
    fn rotated(self, axis_angle: Vector3) -> Transformed<Self> {
        self.transformed(Transform3::rotation(axis_angle))
    }

    /// Scale uniformly about the origin (`factor > 0`).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `factor` is not positive and finite.
    fn scaled(self, factor: f64) -> CoreResult<UniformScale<Self>> {
        UniformScale::new(self, factor)
    }

    /// Scale per-axis about the origin (each factor `> 0`). Sign-exact but
    /// not metric-exact — see [`AnisotropicScale`].
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if any factor is not positive and
    /// finite.
    fn scaled_anisotropic(self, factors: Vector3) -> CoreResult<AnisotropicScale<Self>> {
        AnisotropicScale::new(self, factors)
    }

    /// Taper (draft) the shape about the plane through `neutral_point` with
    /// normal `pull`, by draft `angle` in radians. Sign-exact but not
    /// metric-exact — see [`Taper`].
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `pull` is not a non-zero finite
    /// vector, `neutral_point` is not finite, or `angle` is not finite with
    /// `|angle| < π/2`.
    fn tapered(self, pull: Vector3, neutral_point: Point3, angle: f64) -> CoreResult<Taper<Self>> {
        Taper::new(self, pull, neutral_point, angle)
    }
}

impl<S: Sdf + Sized> SdfTransformExt for S {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eval::gradient;
    use crate::primitives::{Box3, Sphere};
    use std::f64::consts::FRAC_PI_2;

    fn unit_sphere() -> Sphere {
        Sphere {
            center: Point3::origin(),
            radius: 1.0,
        }
    }

    fn assert_unit_gradient(sdf: &dyn Sdf, p: &Point3) {
        let g = gradient(sdf, p).norm();
        assert!((g - 1.0).abs() < 1e-4, "gradient norm {g} at {p:?}");
    }

    #[test]
    fn translated_sphere_surface() {
        let s = unit_sphere().translated(Vector3::new(2.0, 0.0, 0.0));
        assert!(s.eval(&Point3::new(3.0, 0.0, 0.0)).abs() < 1e-12);
        assert!((s.eval(&Point3::new(2.0, 0.0, 0.0)) + 1.0).abs() < 1e-12);
        assert!((s.eval(&Point3::origin()) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn rotated_box_surface() {
        // Box reaching to x = ±2; after +90° about z it reaches to y = ±2.
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [2.0, 1.0, 1.0],
        }
        .rotated(Vector3::new(0.0, 0.0, FRAC_PI_2));
        assert!(b.eval(&Point3::new(0.0, 2.0, 0.0)).abs() < 1e-12);
        assert!((b.eval(&Point3::new(2.0, 0.0, 0.0)) - 1.0).abs() < 1e-12);
        assert!(b.eval(&Point3::origin()) < 0.0);
    }

    #[test]
    fn transformed_preserves_unit_gradient() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 0.5, 0.25],
        }
        .rotated(Vector3::new(0.3, -0.2, 0.9))
        .translated(Vector3::new(1.0, 2.0, -0.5));
        assert_unit_gradient(&b, &Point3::new(2.5, 2.1, -0.3));
        assert_unit_gradient(&b, &Point3::new(0.9, 1.8, -0.6));
    }

    #[test]
    fn composed_transforms_match_single_isometry() {
        let rot = Transform3::rotation(Vector3::new(0.0, 0.0, FRAC_PI_2));
        let tr = Transform3::translation(1.0, 2.0, 3.0);
        let box3 = || Box3 {
            center: Point3::origin(),
            half_extents: [2.0, 1.0, 0.5],
        };
        // Chained rotate-then-translate must equal the single isometry tr * rot.
        let chained = box3()
            .rotated(Vector3::new(0.0, 0.0, FRAC_PI_2))
            .translated(Vector3::new(1.0, 2.0, 3.0));
        let single = box3().transformed(tr * rot);
        for p in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 4.0, 3.0),
            Point3::new(-2.5, 1.3, 4.7),
        ] {
            assert!((chained.eval(&p) - single.eval(&p)).abs() < 1e-12);
        }
    }

    #[test]
    fn scaled_sphere_distance_is_exact_everywhere() {
        // Unit sphere scaled by 2 is a radius-2 sphere; the field must be
        // the exact distance |p| - 2, inside and out.
        let s = unit_sphere().scaled(2.0).expect("valid scale");
        for p in [
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, -3.0, 4.0),
        ] {
            let expected = p.coords.norm() - 2.0;
            assert!((s.eval(&p) - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn scale_is_about_the_origin() {
        // Sphere translated to (1,0,0) then scaled by 2 lands at (2,0,0)
        // with radius 2.
        let s = unit_sphere()
            .translated(Vector3::new(1.0, 0.0, 0.0))
            .scaled(2.0)
            .expect("valid scale");
        assert!(s.eval(&Point3::new(4.0, 0.0, 0.0)).abs() < 1e-12);
        assert!(s.eval(&Point3::origin()).abs() < 1e-12);
        assert!((s.eval(&Point3::new(2.0, 0.0, 0.0)) + 2.0).abs() < 1e-12);
    }

    #[test]
    fn scaled_box_preserves_unit_gradient() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 0.5, 0.25],
        }
        .scaled(3.0)
        .expect("valid scale");
        assert_unit_gradient(&b, &Point3::new(3.2, 0.4, 0.1));
        assert_unit_gradient(&b, &Point3::new(0.5, 0.3, 0.2));
    }

    #[test]
    fn shrinking_scale_works() {
        let s = unit_sphere().scaled(0.25).expect("valid scale");
        assert!(s.eval(&Point3::new(0.25, 0.0, 0.0)).abs() < 1e-12);
        assert!((s.eval(&Point3::new(1.25, 0.0, 0.0)) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn transformed_interval_containment() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 0.5, 0.25],
        }
        .rotated(Vector3::new(0.3, -0.2, 0.9))
        .translated(Vector3::new(0.4, -0.1, 0.2));
        crate::test_util::assert_interval_containment(&b, 31);
    }

    #[test]
    fn scaled_interval_containment() {
        let s = Box3 {
            center: Point3::new(0.2, -0.1, 0.3),
            half_extents: [0.8, 0.4, 0.6],
        }
        .scaled(1.7)
        .expect("valid scale");
        crate::test_util::assert_interval_containment(&s, 32);
    }

    // The corner-AABB of the inverse-rotated box is a superset of the box,
    // so the rotated interval must contain the exact one (conservative),
    // while a pure translation leaves the box axis-aligned and stays exact.
    #[test]
    fn rotated_sphere_interval_contains_exact_translated_stays_exact() {
        use opensolid_core::types::BoundingBox3;
        let b = BoundingBox3::new(Point3::new(1.0, 1.0, 1.0), Point3::new(2.0, 2.0, 2.0));
        let exact = unit_sphere().eval_interval(&b);

        let rotated = unit_sphere().rotated(Vector3::new(0.0, 0.0, 1.2));
        let j = rotated.eval_interval(&b);
        assert!(j.lo <= exact.lo + 1e-12 && exact.hi <= j.hi + 1e-12);

        let shift = Vector3::new(0.5, -0.25, 1.0);
        let translated = unit_sphere().translated(shift);
        let moved_box = BoundingBox3::new(b.min + shift, b.max + shift);
        let k = translated.eval_interval(&moved_box);
        assert!((k.lo - exact.lo).abs() < 1e-12 && (k.hi - exact.hi).abs() < 1e-12);
    }

    // Flat field with a sentinel analytic gradient: the finite-difference
    // fallback would return zero, so any non-zero result through a wrapper
    // proves the wrapper forwarded to the inner `grad`.
    struct GradProbe;
    impl Sdf for GradProbe {
        fn eval(&self, _p: &Point3) -> f64 {
            0.0
        }
        fn grad(&self, p: &Point3) -> Vector3 {
            p.coords
        }
    }

    #[test]
    fn transformed_grad_forwards_and_rotates() {
        // grad(p) = R · inner_grad(T⁻¹p). With inner_grad = identity on
        // coords and T = translate ∘ rotate, the expectation is R(R⁻¹(p-t))
        // = p - t; check against a hand-rotated probe too.
        let t = Vector3::new(1.0, 2.0, 3.0);
        let probe = GradProbe
            .rotated(Vector3::new(0.0, 0.0, FRAC_PI_2))
            .translated(t);
        let p = Point3::new(4.0, 5.0, 6.0);
        let g = probe.grad(&p);
        assert!((g - (p.coords - t)).norm() < 1e-14, "got {g:?}");

        // Pure rotation: inner sees R⁻¹p, result is R(R⁻¹p) = p — but the
        // intermediate must really have been rotated, so probe a point where
        // R⁻¹p differs from p and check via the fallback-is-zero property.
        let rotated = GradProbe.rotated(Vector3::new(0.0, 0.0, FRAC_PI_2));
        let q = Point3::new(1.0, 0.0, 0.0);
        let g = rotated.grad(&q);
        assert!((g - q.coords).norm() < 1e-15, "got {g:?}");
        assert!(g.norm() > 0.5, "fell back to finite differences");
    }

    #[test]
    fn scaled_grad_forwards_at_unscaled_point() {
        // ∇(k·g(p/k)) = ∇g(p/k); with the probe this is p/k, and a
        // finite-difference fallback on the flat field would give zero.
        let s = GradProbe.scaled(4.0).expect("valid scale");
        let p = Point3::new(8.0, -4.0, 2.0);
        let g = s.grad(&p);
        assert!((g - p.coords / 4.0).norm() < 1e-15, "got {g:?}");
    }

    #[test]
    fn transformed_sphere_grad_is_exact_normal() {
        // Sphere carried to center c: gradient at p is (p - c)/|p - c|,
        // exact to machine precision only if forwarding is analytic.
        let c = Point3::new(1.0, 2.0, -0.5);
        let s = unit_sphere()
            .rotated(Vector3::new(0.3, -0.2, 0.9))
            .translated(c.coords);
        for p in [
            Point3::new(3.0, 2.5, 0.1),
            Point3::new(0.9, 1.8, -0.6),
            Point3::new(1.0, 2.0, 4.0),
        ] {
            let g = s.grad(&p);
            let expected = (p - c).normalize();
            assert!((g - expected).norm() < 1e-14, "at {p:?}: got {g:?}");
            assert!((g.norm() - 1.0).abs() < 1e-14);
        }
    }

    #[test]
    fn scaled_sphere_grad_is_exact_normal() {
        let s = unit_sphere().scaled(2.5).expect("valid scale");
        for p in [
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(1.0, -2.0, 0.5),
            Point3::new(0.1, 0.2, 0.3),
        ] {
            let g = s.grad(&p);
            let expected = p.coords.normalize();
            assert!((g - expected).norm() < 1e-14, "at {p:?}: got {g:?}");
        }
    }

    #[test]
    fn anisotropic_scale_surface_and_sign() {
        // Unit sphere scaled (2, 1, 0.5): ellipsoid with semi-axes 2/1/0.5.
        let e = unit_sphere()
            .scaled_anisotropic(Vector3::new(2.0, 1.0, 0.5))
            .expect("valid factors");
        for (p, expect_zero) in [
            (Point3::new(2.0, 0.0, 0.0), true),
            (Point3::new(0.0, 1.0, 0.0), true),
            (Point3::new(0.0, 0.0, 0.5), true),
            (Point3::new(0.0, 0.0, 0.0), false),
        ] {
            let d = e.eval(&p);
            if expect_zero {
                assert!(d.abs() < 1e-12, "at {p:?}: {d}");
            } else {
                assert!(d < 0.0, "at {p:?}: {d}");
            }
        }
        assert!(e.eval(&Point3::new(2.1, 0.0, 0.0)) > 0.0);
        assert!(e.eval(&Point3::new(0.0, 0.0, 0.6)) > 0.0);
        assert!(e.eval(&Point3::new(1.9, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn anisotropic_scale_underestimates_but_never_exceeds_distance() {
        // Along +x the true distance from (4, 0, 0) to the ellipsoid
        // (semi-axis 2) is 2; the conservative field reports
        // min_factor * inner = 0.5 * 1 = 0.5 ≤ 2. It must stay positive
        // outside and Lipschitz ≤ 1 (checked via interval containment).
        let e = unit_sphere()
            .scaled_anisotropic(Vector3::new(2.0, 1.0, 0.5))
            .expect("valid factors");
        let d = e.eval(&Point3::new(4.0, 0.0, 0.0));
        assert!(d > 0.0 && d <= 2.0 + 1e-12, "got {d}");
        assert!((d - 0.5).abs() < 1e-12);
        crate::test_util::assert_interval_containment(&e, 33);
    }

    #[test]
    fn anisotropic_scale_grad_forwards_and_rescales() {
        // With the flat probe (see above) a finite-difference fallback
        // would return zero, so a non-zero result proves forwarding:
        // grad = min_factor * diag(1/f) * inner_grad(p ⊘ f).
        let s = GradProbe
            .scaled_anisotropic(Vector3::new(2.0, 1.0, 0.5))
            .expect("valid factors");
        let p = Point3::new(4.0, -2.0, 1.0);
        let g = s.grad(&p);
        let expected = Vector3::new(4.0 / 2.0 / 2.0, -2.0 / 1.0 / 1.0, 1.0 / 0.5 / 0.5) * 0.5;
        assert!((g - expected).norm() < 1e-14, "got {g:?}");
    }

    #[test]
    fn anisotropic_uniform_factors_match_uniform_scale() {
        let a = unit_sphere()
            .scaled_anisotropic(Vector3::new(1.7, 1.7, 1.7))
            .expect("valid factors");
        let u = unit_sphere().scaled(1.7).expect("valid scale");
        for p in [
            Point3::origin(),
            Point3::new(1.7, 0.0, 0.0),
            Point3::new(-0.4, 2.2, 0.9),
        ] {
            assert!((a.eval(&p) - u.eval(&p)).abs() < 1e-12, "at {p:?}");
        }
    }

    #[test]
    fn anisotropic_scale_rejects_bad_factors() {
        for bad in [
            Vector3::new(0.0, 1.0, 1.0),
            Vector3::new(1.0, -2.0, 1.0),
            Vector3::new(1.0, 1.0, f64::NAN),
            Vector3::new(f64::INFINITY, 1.0, 1.0),
        ] {
            let err = match AnisotropicScale::new(unit_sphere(), bad) {
                Ok(_) => panic!("factors {bad:?}: expected rejection"),
                Err(e) => e,
            };
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "factors",
                        ..
                    }
                ),
                "factors {bad:?}: got {err}"
            );
        }
    }

    // ----- Taper (draft) -----

    use crate::mesh::{MeshOptions, mesh_sdf_indexed};

    fn unit_cube() -> Box3 {
        Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 1.0, 1.0],
        }
    }

    // Taper about the y=0 plane pulling along +Y: the ±x/±z side walls flare
    // out above the neutral plane and pinch in below, while the caps stay put.
    #[test]
    fn taper_flares_side_walls_about_neutral_plane() {
        let angle = 0.2_f64;
        let k = |h: f64| 1.0 + angle.tan() * h;
        let t = unit_cube()
            .tapered(Vector3::new(0.0, 1.0, 0.0), Point3::origin(), angle)
            .expect("valid taper");

        // The +x wall (originally x = 1) sits at x = k(h) at height h.
        for h in [-0.9, -0.3, 0.0, 0.4, 0.9] {
            assert!(
                t.eval(&Point3::new(k(h), h, 0.0)).abs() < 1e-12,
                "wall not at x = k({h}) = {}",
                k(h)
            );
        }
        // On the neutral plane k = 1: the section is unchanged.
        assert!(t.eval(&Point3::new(1.0, 0.0, 0.0)).abs() < 1e-12);
        // Above the plane the wall has moved out past x = 1 (flared);
        // below it has pinched in.
        assert!(t.eval(&Point3::new(1.0, 0.9, 0.0)) < 0.0, "top not flared");
        assert!(
            t.eval(&Point3::new(1.0, -0.9, 0.0)) > 0.0,
            "bottom not pinched"
        );
        // The +y cap keeps its position (axis coordinate is preserved).
        assert!(t.eval(&Point3::new(0.0, 1.0, 0.0)).abs() < 1e-12);
    }

    #[test]
    fn taper_zero_angle_is_identity() {
        let t = unit_cube()
            .tapered(Vector3::new(0.0, 1.0, 0.0), Point3::origin(), 0.0)
            .expect("valid taper");
        let plain = unit_cube();
        for p in [
            Point3::origin(),
            Point3::new(0.5, 0.3, -0.2),
            Point3::new(1.3, -0.7, 0.9),
            Point3::new(-2.0, 1.5, 0.4),
        ] {
            assert!((t.eval(&p) - plain.eval(&p)).abs() < 1e-12, "at {p:?}");
        }
    }

    // A non-zero neutral offset fixes that plane instead of the origin plane.
    #[test]
    fn taper_neutral_plane_offset_is_the_fixed_plane() {
        let angle = 0.25_f64;
        let neutral = Point3::new(0.0, 0.5, 0.0);
        let t = unit_cube()
            .tapered(Vector3::new(0.0, 1.0, 0.0), neutral, angle)
            .expect("valid taper");
        // At y = 0.5 the factor is 1, so the wall is still exactly at x = 1.
        assert!(t.eval(&Point3::new(1.0, 0.5, 0.0)).abs() < 1e-12);
        // Elsewhere it scales by k(h) with h measured from y = 0.5.
        let k = |y: f64| 1.0 + angle.tan() * (y - 0.5);
        assert!(t.eval(&Point3::new(k(-0.5), -0.5, 0.0)).abs() < 1e-12);
        assert!(t.eval(&Point3::new(k(0.9), 0.9, 0.0)).abs() < 1e-12);
    }

    // Draft along an oblique axis still fixes the neutral plane and scales
    // the perpendicular section — checked via the exact inverse relation.
    #[test]
    fn taper_oblique_axis_scales_perpendicular_section() {
        let axis = Vector3::new(1.0, 2.0, 2.0);
        let n = axis.normalize();
        let angle = 0.15_f64;
        let sphere = unit_sphere()
            .tapered(axis, Point3::origin(), angle)
            .expect("valid taper");
        // Forward-map a point known to be on the inner unit sphere and check
        // the tapered field vanishes there: q on the sphere, world image
        // p = a·n + k·lat maps back to q, so eval(p) = inner(q) = 0.
        let q = Point3::new(0.3, -0.4, f64::sqrt(1.0 - 0.09 - 0.16));
        let a = n.dot(&q.coords);
        let lat = q.coords - n * a;
        let k = 1.0 + angle.tan() * a; // neutral = 0
        let world = Point3::from(n * a + lat * k);
        assert!(
            sphere.eval(&world).abs() < 1e-12,
            "forward image off surface"
        );
    }

    #[test]
    fn taper_gradient_matches_finite_differences() {
        // A tapered box: analytic pull-back grad must agree with central
        // differences of the field away from the sharp edges.
        let t = unit_cube()
            .tapered(
                Vector3::new(0.2, 1.0, -0.3),
                Point3::new(0.0, 0.1, 0.0),
                0.3,
            )
            .expect("valid taper");
        let h = 1e-6;
        for p in [
            Point3::new(1.4, 0.2, 0.0),
            Point3::new(0.0, 1.5, 0.3),
            Point3::new(-0.2, -0.6, 1.4),
            Point3::new(2.0, 0.4, -1.5),
        ] {
            let fd = Vector3::new(
                t.eval(&Point3::new(p.x + h, p.y, p.z)) - t.eval(&Point3::new(p.x - h, p.y, p.z)),
                t.eval(&Point3::new(p.x, p.y + h, p.z)) - t.eval(&Point3::new(p.x, p.y - h, p.z)),
                t.eval(&Point3::new(p.x, p.y, p.z + h)) - t.eval(&Point3::new(p.x, p.y, p.z - h)),
            ) / (2.0 * h);
            let g = t.grad(&p);
            assert!(
                (g - fd).norm() < 1e-4,
                "at {p:?}: analytic {g:?} vs fd {fd:?}"
            );
        }
    }

    #[test]
    fn taper_grad_forwards_and_pulls_back() {
        // Flat probe (grad = coords, eval = 0): a finite-difference fallback
        // would give zero, so a non-zero result proves the analytic pull-back
        // ran. At the neutral plane (k = 1) it reduces to the inner grad.
        let axis = Vector3::new(0.0, 1.0, 0.0);
        let t = GradProbe
            .tapered(axis, Point3::origin(), 0.4)
            .expect("valid taper");
        let p = Point3::new(0.7, 0.0, -0.3); // on the neutral plane, k = 1
        let q = t.to_inner(&p).expect("invertible");
        let expected = t.pull_back_grad(&p, 1.0, q.coords);
        let g = t.grad(&p);
        assert!((g - expected).norm() < 1e-12, "got {g:?}");
        assert!(g.norm() > 0.5, "fell back to finite differences");
    }

    #[test]
    fn taper_interval_containment_axis_aligned() {
        // angle 0.2 about +Y keeps the collapse plane (y ≈ −4.93) outside the
        // sampled range, so every box exercises the real interval bound.
        let t = Box3 {
            center: Point3::new(0.1, -0.2, 0.3),
            half_extents: [0.8, 0.5, 0.6],
        }
        .tapered(Vector3::new(0.0, 1.0, 0.0), Point3::origin(), 0.2)
        .expect("valid taper");
        crate::test_util::assert_interval_containment(&t, 71);
    }

    #[test]
    fn taper_interval_containment_oblique_axis() {
        // An oblique axis whose collapse plane can fall inside the sampled
        // range: straddling boxes must widen to WHOLE, the rest stay tight.
        let t = unit_sphere()
            .tapered(
                Vector3::new(1.0, 2.0, 2.0),
                Point3::new(0.0, 0.1, -0.2),
                0.15,
            )
            .expect("valid taper");
        crate::test_util::assert_interval_containment(&t, 72);
    }

    #[test]
    fn taper_interval_spanning_collapse_is_whole() {
        // Steep draft: the collapse plane y = −1/tan(0.6) ≈ −1.53 lies within
        // this box, so the inverse map is undefined and the bound is WHOLE.
        let t = unit_cube()
            .tapered(Vector3::new(0.0, 1.0, 0.0), Point3::origin(), 0.6)
            .expect("valid taper");
        let b = BoundingBox3::new(Point3::new(-1.0, -3.0, -1.0), Point3::new(1.0, 1.0, 1.0));
        let i = t.eval_interval(&b);
        assert_eq!(i.lo, f64::NEG_INFINITY);
        assert_eq!(i.hi, f64::INFINITY);
    }

    #[test]
    fn tapered_box_meshes_to_closed_manifold() {
        let t = unit_cube()
            .tapered(Vector3::new(0.0, 1.0, 0.0), Point3::origin(), 0.25)
            .expect("valid taper");
        // The +Y wall reaches x = 1 + tan(0.25) ≈ 1.26; bound generously.
        let opts = MeshOptions {
            bounds: BoundingBox3 {
                min: Point3::new(-1.6, -1.4, -1.6),
                max: Point3::new(1.6, 1.4, 1.6),
            },
            resolution: 40,
        };
        let mesh = mesh_sdf_indexed(&t, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let cell = 3.2 / 40.0;
        for p in &mesh.positions {
            assert!(
                t.eval(p).abs() < cell,
                "vertex {p:?} off the tapered surface"
            );
        }
        // Confirm the flare: the widest wall vertices sit near the +Y cap.
        let widest = mesh
            .positions
            .iter()
            .max_by(|a, b| a.x.partial_cmp(&b.x).unwrap())
            .expect("non-empty");
        assert!(
            widest.x > 1.15 && widest.y > 0.5,
            "flare not toward +Y: {widest:?}"
        );
    }

    #[test]
    fn taper_rejects_bad_arguments() {
        let bad_pull = [
            Vector3::zeros(),
            Vector3::new(f64::NAN, 0.0, 1.0),
            Vector3::new(f64::INFINITY, 0.0, 0.0),
        ];
        for pull in bad_pull {
            assert!(matches!(
                unit_cube().tapered(pull, Point3::origin(), 0.1),
                Err(CoreError::InvalidArgument {
                    argument: "pull",
                    ..
                })
            ));
        }
        // Non-finite neutral point.
        assert!(matches!(
            unit_cube().tapered(
                Vector3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, f64::NAN, 0.0),
                0.1
            ),
            Err(CoreError::InvalidArgument {
                argument: "neutral_point",
                ..
            })
        ));
        // Out-of-range / non-finite angle.
        for angle in [
            std::f64::consts::FRAC_PI_2,
            std::f64::consts::PI,
            f64::NAN,
            f64::INFINITY,
        ] {
            assert!(matches!(
                unit_cube().tapered(Vector3::new(0.0, 1.0, 0.0), Point3::origin(), angle),
                Err(CoreError::InvalidArgument {
                    argument: "angle",
                    ..
                })
            ));
        }
    }

    #[test]
    fn scale_rejects_nonpositive_factor() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let err = match UniformScale::new(unit_sphere(), bad) {
                Ok(_) => panic!("factor {bad}: expected rejection"),
                Err(e) => e,
            };
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "factor",
                        ..
                    }
                ),
                "factor {bad}: got {err}"
            );
            assert!(
                err.to_string().contains("positive"),
                "missing constraint: {err}"
            );
        }
    }
}
