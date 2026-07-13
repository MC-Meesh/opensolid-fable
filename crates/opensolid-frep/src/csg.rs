//! Sharp CSG via min/max.
//!
//! # Metric properties
//!
//! `min` and `max` of 1-Lipschitz functions are 1-Lipschitz, so every
//! combinator here preserves the 1-Lipschitz bound `|f(p) - f(q)| <= |p - q|`
//! when its children satisfy it. The result is *not* an exact distance in
//! general: e.g. inside an overlapping union, `min` reports the distance to a
//! surface that may lie inside the other operand, underestimating the true
//! distance to the combined boundary. The field is always a conservative
//! (lower-magnitude) bound, which is what sphere tracing and the mesher need.

use crate::primitives::Sdf;
use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};

/// `min(a, b)`. Preserves 1-Lipschitz; exact only where the nearest surface
/// point of the winning operand lies on the union boundary.
pub struct Union<A, B> {
    pub a: A,
    pub b: B,
}

impl<A: Sdf, B: Sdf> Sdf for Union<A, B> {
    fn eval(&self, p: &Point3) -> f64 {
        self.a.eval(p).min(self.b.eval(p))
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        if self.a.eval(p) <= self.b.eval(p) {
            self.a.grad(p)
        } else {
            self.b.grad(p)
        }
    }

    // Pointwise min propagates exactly: no widening beyond the children's.
    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        self.a.eval_interval(b).min(&self.b.eval_interval(b))
    }

    // A child is active if it wins the min within `tol`.
    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let fa = self.a.eval(p);
        let fb = self.b.eval(p);
        if fa <= fb + tol {
            self.a.branches(p, tol, out);
        }
        if fb <= fa + tol {
            self.b.branches(p, tol, out);
        }
    }
}

/// `max(a, b)`. Preserves 1-Lipschitz; underestimates distance near corners
/// where the true nearest boundary point is on the intersection curve.
pub struct Intersection<A, B> {
    pub a: A,
    pub b: B,
}

impl<A: Sdf, B: Sdf> Sdf for Intersection<A, B> {
    fn eval(&self, p: &Point3) -> f64 {
        self.a.eval(p).max(self.b.eval(p))
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        if self.a.eval(p) >= self.b.eval(p) {
            self.a.grad(p)
        } else {
            self.b.grad(p)
        }
    }

    // Pointwise max propagates exactly: no widening beyond the children's.
    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        self.a.eval_interval(b).max(&self.b.eval_interval(b))
    }

    // A child is active if it wins the max within `tol`.
    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let fa = self.a.eval(p);
        let fb = self.b.eval(p);
        if fa >= fb - tol {
            self.a.branches(p, tol, out);
        }
        if fb >= fa - tol {
            self.b.branches(p, tol, out);
        }
    }
}

/// `max(a, -b)`. Negation and `max` both preserve 1-Lipschitz, so the result
/// does too; like the other sharp combinators it is a bound, not an exact
/// distance.
pub struct Subtraction<A, B> {
    pub a: A,
    pub b: B,
}

impl<A: Sdf, B: Sdf> Sdf for Subtraction<A, B> {
    fn eval(&self, p: &Point3) -> f64 {
        self.a.eval(p).max(-self.b.eval(p))
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        if self.a.eval(p) >= -self.b.eval(p) {
            self.a.grad(p)
        } else {
            -self.b.grad(p)
        }
    }

    // max(a, -b): negation and pointwise max both propagate exactly.
    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        self.a.eval_interval(b).max(&(-self.b.eval_interval(b)))
    }

    // `b` enters negated, so its branches are negated too (value and
    // gradient): the branch surfaces are unchanged but oriented outward for
    // the subtracted solid, matching `grad`.
    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let fa = self.a.eval(p);
        let nb = -self.b.eval(p);
        if fa >= nb - tol {
            self.a.branches(p, tol, out);
        }
        if nb >= fa - tol {
            let start = out.len();
            self.b.branches(p, tol, out);
            for branch in &mut out[start..] {
                branch.0 = -branch.0;
                branch.1 = -branch.1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::Sphere;

    #[test]
    fn union_of_spheres() {
        let a = Sphere {
            center: Point3::new(-0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let b = Sphere {
            center: Point3::new(0.5, 0.0, 0.0),
            radius: 1.0,
        };
        let u = Union { a, b };
        assert!(u.eval(&Point3::origin()) < 0.0);
        assert!(u.eval(&Point3::new(5.0, 0.0, 0.0)) > 0.0);
    }

    fn sphere(x: f64, r: f64) -> Sphere {
        Sphere {
            center: Point3::new(x, 0.0, 0.0),
            radius: r,
        }
    }

    #[test]
    fn union_interval_containment() {
        let u = Union {
            a: sphere(-0.5, 1.0),
            b: sphere(0.5, 0.7),
        };
        crate::test_util::assert_interval_containment(&u, 11);
    }

    #[test]
    fn intersection_interval_containment() {
        let i = Intersection {
            a: sphere(-0.3, 1.2),
            b: sphere(0.3, 1.0),
        };
        crate::test_util::assert_interval_containment(&i, 12);
    }

    #[test]
    fn subtraction_interval_containment() {
        let s = Subtraction {
            a: sphere(0.0, 1.5),
            b: sphere(0.2, 0.8),
        };
        crate::test_util::assert_interval_containment(&s, 13);
    }

    // min/max/negate propagation is exact: with exact children the CSG
    // interval equals the min/max of the child intervals, no widening.
    #[test]
    fn csg_intervals_propagate_exactly() {
        let a = sphere(-0.5, 1.0);
        let b = sphere(0.5, 0.7);
        let bx = BoundingBox3::new(Point3::new(0.1, -0.4, -0.2), Point3::new(1.3, 0.6, 0.9));
        let (ia, ib) = (a.eval_interval(&bx), b.eval_interval(&bx));
        let u = Union {
            a: sphere(-0.5, 1.0),
            b: sphere(0.5, 0.7),
        };
        assert_eq!(u.eval_interval(&bx), ia.min(&ib));
        let i = Intersection {
            a: sphere(-0.5, 1.0),
            b: sphere(0.5, 0.7),
        };
        assert_eq!(i.eval_interval(&bx), ia.max(&ib));
        let s = Subtraction {
            a: sphere(-0.5, 1.0),
            b: sphere(0.5, 0.7),
        };
        assert_eq!(s.eval_interval(&bx), ia.max(&(-ib)));
    }

    #[test]
    fn subtraction_cuts_hole() {
        let a = Sphere {
            center: Point3::origin(),
            radius: 2.0,
        };
        let b = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let s = Subtraction { a, b };
        // Center is inside B, so subtraction makes it outside
        assert!(s.eval(&Point3::origin()) > 0.0);
        // Between radii is inside
        assert!(s.eval(&Point3::new(1.5, 0.0, 0.0)) < 0.0);
    }
}
