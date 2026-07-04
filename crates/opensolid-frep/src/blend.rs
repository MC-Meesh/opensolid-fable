//! Smooth CSG via the polynomial smooth min/max.
//!
//! # Metric properties
//!
//! Both blends preserve the 1-Lipschitz bound when their children satisfy
//! it: the gradient is `h * grad_a ± (1 - h) * grad_b` with `h ∈ [0, 1]`, a
//! convex combination of vectors of norm <= 1, so its norm is <= 1. Inside
//! the blend region the field is *not* an exact distance (the surface is
//! pulled off both operands), and `|grad|` dips below 1 where the child
//! gradients disagree — so gradient-norm ~ 1 holds only for exact
//! primitives, not for blended fields.

use crate::primitives::Sdf;
use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};

pub struct SmoothUnion<A, B> {
    pub a: A,
    pub b: B,
    pub radius: f64,
}

impl<A: Sdf, B: Sdf> Sdf for SmoothUnion<A, B> {
    fn eval(&self, p: &Point3) -> f64 {
        let da = self.a.eval(p);
        let db = self.b.eval(p);
        let h = (0.5 + 0.5 * (db - da) / self.radius).clamp(0.0, 1.0);
        db * (1.0 - h) + da * h - self.radius * h * (1.0 - h)
    }

    // The h-dependence cancels exactly for the polynomial smooth min,
    // leaving the plain mix of child gradients.
    fn grad(&self, p: &Point3) -> Vector3 {
        let da = self.a.eval(p);
        let db = self.b.eval(p);
        let h = (0.5 + 0.5 * (db - da) / self.radius).clamp(0.0, 1.0);
        self.a.grad(p) * h + self.b.grad(p) * (1.0 - h)
    }

    // Conservative widening: the polynomial smooth min deviates from the
    // sharp min by at most radius/4 (attained at da == db), always downward,
    // so [min.lo - r/4, min.hi] contains the blended field.
    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        let sharp = self.a.eval_interval(b).min(&self.b.eval_interval(b));
        Interval::new(sharp.lo - 0.25 * self.radius, sharp.hi)
    }
}

pub struct SmoothSubtraction<A, B> {
    pub a: A,
    pub b: B,
    pub radius: f64,
}

impl<A: Sdf, B: Sdf> Sdf for SmoothSubtraction<A, B> {
    fn eval(&self, p: &Point3) -> f64 {
        let da = self.a.eval(p);
        let db = self.b.eval(p);
        // h must use the raw cutter distance so it clamps to 0 far from the
        // cutter (returning da), not the negated one.
        let h = (0.5 - 0.5 * (da + db) / self.radius).clamp(0.0, 1.0);
        da * (1.0 - h) - db * h + self.radius * h * (1.0 - h)
    }

    // With h on the raw cutter distance (of-9ht) the dd/dh term cancels
    // exactly, as in SmoothUnion, leaving the plain mix of child gradients;
    // the cutter contributes negated. If eval changes, re-derive this.
    fn grad(&self, p: &Point3) -> Vector3 {
        let da = self.a.eval(p);
        let db = self.b.eval(p);
        let h = (0.5 - 0.5 * (da + db) / self.radius).clamp(0.0, 1.0);
        self.a.grad(p) * (1.0 - h) - self.b.grad(p) * h
    }

    // Smooth subtraction is -smin(-da, db, r), so it deviates from the
    // sharp max(da, -db) by at most radius/4, always upward.
    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        let sharp = self.a.eval_interval(b).max(&(-self.b.eval_interval(b)));
        Interval::new(sharp.lo, sharp.hi + 0.25 * self.radius)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::Sphere;

    #[test]
    fn smooth_union_blends() {
        let a = Sphere {
            center: Point3::new(-0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let b = Sphere {
            center: Point3::new(0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let su = SmoothUnion { a, b, radius: 0.3 };
        // Smooth union should be more negative at origin than sharp union
        let sharp_a = Sphere {
            center: Point3::new(-0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let sharp_b = Sphere {
            center: Point3::new(0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let sharp = sharp_a
            .eval(&Point3::origin())
            .min(sharp_b.eval(&Point3::origin()));
        assert!(su.eval(&Point3::origin()) < sharp);
    }

    fn unit_sphere() -> Sphere {
        Sphere {
            center: Point3::origin(),
            radius: 1.0,
        }
    }

    #[test]
    fn smooth_union_interval_containment() {
        let su = SmoothUnion {
            a: Sphere {
                center: Point3::new(-0.5, 0.0, 0.0),
                radius: 1.0,
            },
            b: Sphere {
                center: Point3::new(0.5, 0.2, -0.1),
                radius: 0.8,
            },
            radius: 0.3,
        };
        crate::test_util::assert_interval_containment(&su, 21);
    }

    #[test]
    fn smooth_subtraction_interval_containment() {
        let ss = SmoothSubtraction {
            a: Sphere {
                center: Point3::origin(),
                radius: 1.2,
            },
            b: Sphere {
                center: Point3::new(0.6, 0.1, -0.2),
                radius: 0.7,
            },
            radius: 0.25,
        };
        crate::test_util::assert_interval_containment(&ss, 22);
    }

    // The widening is bounded: exactly radius/4 beyond the sharp interval,
    // on the blending side only.
    #[test]
    fn smooth_interval_widens_by_quarter_radius() {
        use opensolid_core::types::BoundingBox3;
        let a = || Sphere {
            center: Point3::new(-0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let b = || Sphere {
            center: Point3::new(0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let bx = BoundingBox3::new(Point3::new(-0.4, -0.4, -0.4), Point3::new(0.4, 0.4, 0.4));
        let sharp = a().eval_interval(&bx).min(&b().eval_interval(&bx));
        let su = SmoothUnion {
            a: a(),
            b: b(),
            radius: 0.3,
        };
        let i = su.eval_interval(&bx);
        assert_eq!(i.hi, sharp.hi);
        assert!((i.lo - (sharp.lo - 0.075)).abs() < 1e-12);
    }

    #[test]
    fn smooth_subtraction_matches_base_far_from_cutter() {
        // Regression for the sign bug: da=0.5, cutter eval=10, r=0.3 used to
        // return -10 (deep inside) at a point outside the base solid.
        let cutter = Sphere {
            center: Point3::new(10.0, 0.0, 0.0),
            radius: 1.0,
        };
        let ss = SmoothSubtraction {
            a: unit_sphere(),
            b: cutter,
            radius: 0.3,
        };
        let p = Point3::new(-1.5, 0.0, 0.0);
        let da = unit_sphere().eval(&p);
        assert!(da > 0.0);
        assert!((ss.eval(&p) - da).abs() < 1e-12);
    }

    #[test]
    fn smooth_subtraction_matches_negated_cutter_deep_inside_cutter() {
        // Deep inside the cutter (and inside the base) the result is the
        // negated cutter distance: the point has been carved out.
        let cutter = Sphere {
            center: Point3::origin(),
            radius: 0.5,
        };
        let ss = SmoothSubtraction {
            a: unit_sphere(),
            b: cutter,
            radius: 0.1,
        };
        let p = Point3::origin();
        let db = Sphere {
            center: Point3::origin(),
            radius: 0.5,
        }
        .eval(&p);
        assert!((ss.eval(&p) - (-db)).abs() < 1e-12);
        assert!(ss.eval(&p) > 0.0);
    }

    #[test]
    fn smooth_subtraction_fillets_outward_in_blend_region() {
        // Where the two surfaces meet, smooth subtraction removes extra
        // material, so the result is >= the sharp subtraction max(da, -db).
        let cutter = Sphere {
            center: Point3::new(1.0, 0.0, 0.0),
            radius: 1.0,
        };
        let ss = SmoothSubtraction {
            a: unit_sphere(),
            b: cutter,
            radius: 0.3,
        };
        // Point near the intersection circle of the two spheres.
        let p = Point3::new(0.5, 0.866, 0.0);
        let da = unit_sphere().eval(&p);
        let db = Sphere {
            center: Point3::new(1.0, 0.0, 0.0),
            radius: 1.0,
        }
        .eval(&p);
        let sharp = da.max(-db);
        assert!(ss.eval(&p) >= sharp);
        assert!(ss.eval(&p) > sharp + 1e-3, "expected a fillet at the edge");
    }
}
