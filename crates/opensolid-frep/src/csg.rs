use crate::primitives::Sdf;
use opensolid_core::types::{Point3, Vector3};

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
