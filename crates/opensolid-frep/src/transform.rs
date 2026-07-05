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
