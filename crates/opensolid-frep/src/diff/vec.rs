//! A minimal 3-vector generic over [`Scalar`].
//!
//! `opensolid_core::types::Vector3` is a nalgebra alias pinned at `f64`, and
//! teaching nalgebra about [`Dual`](super::Dual) would mean implementing its
//! `Scalar`/`RealField` stack (and pulling `num-traits` into the runtime
//! dependency budget). The field tower needs six operations on vectors, so
//! it carries its own.

use super::scalar::Scalar;
use opensolid_core::types::Point3;
use std::ops::{Add, Mul, Neg, Sub};

/// A 3-vector over any [`Scalar`]. Doubles as a point.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Vec3<T> {
    pub x: T,
    pub y: T,
    pub z: T,
}

impl<T: Scalar> Vec3<T> {
    pub fn new(x: T, y: T, z: T) -> Self {
        Self { x, y, z }
    }

    /// A vector of constants — the usual way to lift a fixed sample point
    /// into the dual domain, where it contributes no derivative.
    pub fn cst(x: f64, y: f64, z: f64) -> Self {
        Self::new(T::cst(x), T::cst(y), T::cst(z))
    }

    /// Lift a concrete sample point into the scalar domain `T`.
    ///
    /// The point is a *constant* of the differentiation: we differentiate
    /// with respect to design parameters, not with respect to where we
    /// sampled. See `docs/design/DIFFERENTIABLE.md` §2.
    pub fn from_point(p: &Point3) -> Self {
        Self::cst(p.x, p.y, p.z)
    }

    pub fn zero() -> Self {
        Self::cst(0.0, 0.0, 0.0)
    }

    pub fn splat(v: T) -> Self {
        Self::new(v, v, v)
    }

    pub fn dot(self, o: Self) -> T {
        self.x * o.x + self.y * o.y + self.z * o.z
    }

    /// Euclidean length. Not differentiable at the origin; the
    /// [`sqrt`](Scalar::sqrt) guard keeps the derivative finite there.
    pub fn norm(self) -> T {
        self.dot(self).sqrt()
    }

    pub fn norm_squared(self) -> T {
        self.dot(self)
    }

    /// Componentwise absolute value — the workhorse of box fields.
    pub fn abs(self) -> Self {
        Self::new(self.x.abs(), self.y.abs(), self.z.abs())
    }

    /// Componentwise `max(self, 0)`.
    pub fn relu(self) -> Self {
        Self::new(self.x.relu(), self.y.relu(), self.z.relu())
    }

    /// The largest component.
    pub fn max_component(self) -> T {
        self.x.max(self.y).max(self.z)
    }

    pub fn scale(self, s: T) -> Self {
        Self::new(self.x * s, self.y * s, self.z * s)
    }

    /// Componentwise product.
    pub fn mul_elem(self, o: Self) -> Self {
        Self::new(self.x * o.x, self.y * o.y, self.z * o.z)
    }

    /// The values, dropping derivatives.
    pub fn val(self) -> [f64; 3] {
        [self.x.val(), self.y.val(), self.z.val()]
    }
}

impl<T: Scalar> Add for Vec3<T> {
    type Output = Self;
    fn add(self, o: Self) -> Self {
        Self::new(self.x + o.x, self.y + o.y, self.z + o.z)
    }
}

impl<T: Scalar> Sub for Vec3<T> {
    type Output = Self;
    fn sub(self, o: Self) -> Self {
        Self::new(self.x - o.x, self.y - o.y, self.z - o.z)
    }
}

impl<T: Scalar> Mul<T> for Vec3<T> {
    type Output = Self;
    fn mul(self, s: T) -> Self {
        self.scale(s)
    }
}

impl<T: Scalar> Neg for Vec3<T> {
    type Output = Self;
    fn neg(self) -> Self {
        Self::new(-self.x, -self.y, -self.z)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diff::Dual;

    #[test]
    fn dot_and_norm_over_f64() {
        let a = Vec3::<f64>::new(1.0, 2.0, 2.0);
        assert_eq!(a.dot(a), 9.0);
        assert_eq!(a.norm(), 3.0);
        assert_eq!(a.norm_squared(), 9.0);
    }

    #[test]
    fn from_point_lifts_as_constant() {
        let v = Vec3::<Dual<2>>::from_point(&Point3::new(1.0, 2.0, 3.0));
        assert_eq!(v.val(), [1.0, 2.0, 3.0]);
        // A sample point carries no parameter derivative.
        assert_eq!(v.x.grad(), [0.0, 0.0]);
    }

    #[test]
    fn norm_carries_derivative() {
        // |(r, 0, 0)| = r, so d|v|/dr = 1.
        let r = Dual::<1>::seed(4.0, 0);
        let v = Vec3::new(r, Dual::cst(0.0), Dual::cst(0.0));
        let n = v.norm();
        assert!((n.val() - 4.0).abs() < 1e-12);
        assert!((n.grad()[0] - 1.0).abs() < 1e-9);
    }

    #[test]
    fn norm_at_origin_is_finite() {
        let z = Vec3::<Dual<1>>::zero();
        assert!(z.norm().grad()[0].is_finite());
    }

    #[test]
    fn abs_relu_and_max_component() {
        let v = Vec3::<f64>::new(-3.0, 1.0, -0.5);
        assert_eq!(v.abs().val(), [3.0, 1.0, 0.5]);
        assert_eq!(v.relu().val(), [0.0, 1.0, 0.0]);
        assert_eq!(v.max_component(), 1.0);
    }

    #[test]
    fn arithmetic() {
        let a = Vec3::<f64>::new(1.0, 2.0, 3.0);
        let b = Vec3::<f64>::new(4.0, 5.0, 6.0);
        assert_eq!((a + b).val(), [5.0, 7.0, 9.0]);
        assert_eq!((b - a).val(), [3.0, 3.0, 3.0]);
        assert_eq!((a * 2.0).val(), [2.0, 4.0, 6.0]);
        assert_eq!((-a).val(), [-1.0, -2.0, -3.0]);
        assert_eq!(a.mul_elem(b).val(), [4.0, 10.0, 18.0]);
        assert_eq!(Vec3::<f64>::splat(2.0).val(), [2.0, 2.0, 2.0]);
    }
}
