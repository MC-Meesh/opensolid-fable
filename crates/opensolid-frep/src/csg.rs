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
use opensolid_core::types::{Point3, Vector3};

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
