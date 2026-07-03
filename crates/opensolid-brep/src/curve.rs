//! Parametric 3D curves: analytic primitives and the evaluation trait.
//!
//! Parameterization conventions:
//! - `Line`: parameterized by arc length; `dir` is a unit vector, so
//!   `point(t)` is exactly `t` units from `origin`. Domain is unbounded.
//! - `Circle` / `Ellipse`: parameterized by angle in radians over `[0, 2π)`,
//!   counterclockwise when viewed from the tip of `axis` (right-hand rule).
//!   `t = 0` lies along the reference x-direction of the curve's frame.

use opensolid_core::types::{Point3, Vector3};

/// Full angular period of a closed conic parameterization.
pub const TWO_PI: f64 = 2.0 * std::f64::consts::PI;

/// Evaluation interface for parametric curves.
pub trait CurveEval {
    /// Position on the curve at parameter `t`.
    fn point(&self, t: f64) -> Point3;

    /// First derivative with respect to `t` (tangent, not necessarily unit).
    fn derivative(&self, t: f64) -> Vector3;

    /// Second derivative with respect to `t`.
    fn second_derivative(&self, t: f64) -> Vector3;

    /// Parameter interval `(t_min, t_max)`. Unbounded curves return
    /// infinite endpoints.
    fn domain(&self) -> (f64, f64);

    /// Whether the curve's start and end points coincide.
    fn is_closed(&self) -> bool;

    /// Whether evaluation repeats with period `period()`.
    fn is_periodic(&self) -> bool;

    /// Period of a periodic curve, `None` otherwise.
    fn period(&self) -> Option<f64> {
        None
    }
}

/// Analytic 3D curve primitives.
#[derive(Debug, Clone, PartialEq)]
pub enum Curve3 {
    /// Infinite line through `origin` with unit direction `dir`,
    /// parameterized by arc length.
    Line { origin: Point3, dir: Vector3 },
    /// Circle of `radius` about `center`, in the plane normal to the unit
    /// vector `axis`. The angular reference (t = 0) direction is derived
    /// deterministically from `axis`; see [`plane_basis`].
    Circle {
        center: Point3,
        axis: Vector3,
        radius: f64,
    },
    /// Ellipse about `center` in the plane normal to unit `axis`, with unit
    /// `major_dir` along the major radius (t = 0). `minor_dir` is implied as
    /// `axis × major_dir`.
    Ellipse {
        center: Point3,
        axis: Vector3,
        major_dir: Vector3,
        major_radius: f64,
        minor_radius: f64,
    },
}

/// Deterministic orthonormal basis `(u, v)` spanning the plane normal to
/// `axis` (assumed unit length), with `u × v = axis`. The reference `u` is
/// built from the world X axis unless `axis` is nearly parallel to it, in
/// which case world Y is used.
pub fn plane_basis(axis: &Vector3) -> (Vector3, Vector3) {
    let seed = if axis.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let u = (seed - axis * seed.dot(axis)).normalize();
    let v = axis.cross(&u);
    (u, v)
}

impl Curve3 {
    /// Line through `origin` in the direction of `dir` (normalized here).
    ///
    /// # Panics
    /// Panics if `dir` has zero length.
    pub fn line(origin: Point3, dir: Vector3) -> Self {
        let norm = dir.norm();
        assert!(norm > 0.0, "line direction must be non-zero");
        Curve3::Line {
            origin,
            dir: dir / norm,
        }
    }

    /// Circle of `radius` about `center` in the plane normal to `axis`
    /// (normalized here).
    ///
    /// # Panics
    /// Panics if `axis` has zero length or `radius` is not positive.
    pub fn circle(center: Point3, axis: Vector3, radius: f64) -> Self {
        let norm = axis.norm();
        assert!(norm > 0.0, "circle axis must be non-zero");
        assert!(radius > 0.0, "circle radius must be positive");
        Curve3::Circle {
            center,
            axis: axis / norm,
            radius,
        }
    }

    /// Ellipse about `center` in the plane normal to `axis`, with the major
    /// radius along `major_dir`. `axis` is normalized and `major_dir` is
    /// re-orthogonalized against it (Gram-Schmidt), so `major_dir` only needs
    /// to be non-parallel to `axis`.
    ///
    /// # Panics
    /// Panics if `axis` or `major_dir` is degenerate (zero length or
    /// parallel to each other), if either radius is not positive, or if
    /// `minor_radius > major_radius`.
    pub fn ellipse(
        center: Point3,
        axis: Vector3,
        major_dir: Vector3,
        major_radius: f64,
        minor_radius: f64,
    ) -> Self {
        let axis_norm = axis.norm();
        assert!(axis_norm > 0.0, "ellipse axis must be non-zero");
        let axis = axis / axis_norm;
        let in_plane = major_dir - axis * major_dir.dot(&axis);
        let major_norm = in_plane.norm();
        assert!(
            major_norm > 1e-12,
            "ellipse major_dir must not be parallel to axis"
        );
        assert!(
            major_radius > 0.0 && minor_radius > 0.0,
            "ellipse radii must be positive"
        );
        assert!(
            minor_radius <= major_radius,
            "ellipse minor_radius must not exceed major_radius"
        );
        Curve3::Ellipse {
            center,
            axis,
            major_dir: in_plane / major_norm,
            major_radius,
            minor_radius,
        }
    }

    /// In-plane frame `(u, v)` for conic evaluation: `u` at t = 0, `v` at
    /// t = π/2.
    fn conic_frame(&self) -> Option<(Vector3, Vector3)> {
        match self {
            Curve3::Line { .. } => None,
            Curve3::Circle { axis, .. } => Some(plane_basis(axis)),
            Curve3::Ellipse {
                axis, major_dir, ..
            } => Some((*major_dir, axis.cross(major_dir))),
        }
    }
}

impl CurveEval for Curve3 {
    fn point(&self, t: f64) -> Point3 {
        match self {
            Curve3::Line { origin, dir } => origin + dir * t,
            Curve3::Circle { center, radius, .. } => {
                let (u, v) = self.conic_frame().unwrap();
                center + (u * t.cos() + v * t.sin()) * *radius
            }
            Curve3::Ellipse {
                center,
                major_radius,
                minor_radius,
                ..
            } => {
                let (u, v) = self.conic_frame().unwrap();
                center + u * (major_radius * t.cos()) + v * (minor_radius * t.sin())
            }
        }
    }

    fn derivative(&self, t: f64) -> Vector3 {
        match self {
            Curve3::Line { dir, .. } => *dir,
            Curve3::Circle { radius, .. } => {
                let (u, v) = self.conic_frame().unwrap();
                (v * t.cos() - u * t.sin()) * *radius
            }
            Curve3::Ellipse {
                major_radius,
                minor_radius,
                ..
            } => {
                let (u, v) = self.conic_frame().unwrap();
                v * (minor_radius * t.cos()) - u * (major_radius * t.sin())
            }
        }
    }

    fn second_derivative(&self, t: f64) -> Vector3 {
        match self {
            Curve3::Line { .. } => Vector3::zeros(),
            Curve3::Circle { radius, .. } => {
                let (u, v) = self.conic_frame().unwrap();
                (u * t.cos() + v * t.sin()) * -*radius
            }
            Curve3::Ellipse {
                major_radius,
                minor_radius,
                ..
            } => {
                let (u, v) = self.conic_frame().unwrap();
                -(u * (major_radius * t.cos()) + v * (minor_radius * t.sin()))
            }
        }
    }

    fn domain(&self) -> (f64, f64) {
        match self {
            Curve3::Line { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => (0.0, TWO_PI),
        }
    }

    fn is_closed(&self) -> bool {
        !matches!(self, Curve3::Line { .. })
    }

    fn is_periodic(&self) -> bool {
        self.is_closed()
    }

    fn period(&self) -> Option<f64> {
        if self.is_periodic() {
            Some(TWO_PI)
        } else {
            None
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, PI};

    const EPS: f64 = 1e-10;

    fn assert_point_eq(a: &Point3, b: &Point3) {
        assert!(
            (a - b).norm() < EPS,
            "points differ: {a:?} vs {b:?} (dist {})",
            (a - b).norm()
        );
    }

    fn assert_vec_eq(a: &Vector3, b: &Vector3) {
        assert!(
            (a - b).norm() < EPS,
            "vectors differ: {a:?} vs {b:?} (dist {})",
            (a - b).norm()
        );
    }

    /// Central finite difference should match the analytic derivatives.
    fn check_derivatives_numerically(c: &Curve3, t: f64) {
        let h = 1e-6;
        let fd1 = (c.point(t + h) - c.point(t - h)) / (2.0 * h);
        let d1 = c.derivative(t);
        assert!(
            (fd1 - d1).norm() < 1e-5,
            "first derivative mismatch at t={t}: analytic {d1:?} vs fd {fd1:?}"
        );
        let fd2 = (c.derivative(t + h) - c.derivative(t - h)) / (2.0 * h);
        let d2 = c.second_derivative(t);
        assert!(
            (fd2 - d2).norm() < 1e-5,
            "second derivative mismatch at t={t}: analytic {d2:?} vs fd {fd2:?}"
        );
    }

    #[test]
    fn line_points_by_arc_length() {
        let l = Curve3::line(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 5.0));
        assert_point_eq(&l.point(0.0), &Point3::new(1.0, 2.0, 3.0));
        // dir was length 5 but is normalized: t is arc length.
        assert_point_eq(&l.point(2.0), &Point3::new(1.0, 2.0, 5.0));
        assert_point_eq(&l.point(-1.5), &Point3::new(1.0, 2.0, 1.5));
    }

    #[test]
    fn line_derivatives() {
        let l = Curve3::line(Point3::origin(), Vector3::new(3.0, 0.0, 4.0));
        let d = l.derivative(7.0);
        assert!((d.norm() - 1.0).abs() < EPS, "unit tangent expected");
        assert_vec_eq(&d, &Vector3::new(0.6, 0.0, 0.8));
        assert_vec_eq(&l.second_derivative(-2.0), &Vector3::zeros());
        check_derivatives_numerically(&l, 1.25);
    }

    #[test]
    fn line_domain_and_topology() {
        let l = Curve3::line(Point3::origin(), Vector3::x());
        let (t0, t1) = l.domain();
        assert!(t0.is_infinite() && t0 < 0.0);
        assert!(t1.is_infinite() && t1 > 0.0);
        assert!(!l.is_closed());
        assert!(!l.is_periodic());
        assert_eq!(l.period(), None);
    }

    #[test]
    #[should_panic(expected = "non-zero")]
    fn line_rejects_zero_direction() {
        Curve3::line(Point3::origin(), Vector3::zeros());
    }

    #[test]
    fn circle_analytic_points() {
        // Axis = +Z: plane_basis seeds from world X, so u = X, v = Y.
        let c = Curve3::circle(Point3::new(1.0, 1.0, 0.0), Vector3::z(), 2.0);
        assert_point_eq(&c.point(0.0), &Point3::new(3.0, 1.0, 0.0));
        assert_point_eq(&c.point(FRAC_PI_2), &Point3::new(1.0, 3.0, 0.0));
        assert_point_eq(&c.point(PI), &Point3::new(-1.0, 1.0, 0.0));
        assert_point_eq(&c.point(3.0 * FRAC_PI_2), &Point3::new(1.0, -1.0, 0.0));
    }

    #[test]
    fn circle_arbitrary_axis_stays_on_circle() {
        let center = Point3::new(-2.0, 5.0, 1.0);
        let axis = Vector3::new(1.0, 2.0, -3.0);
        let c = Curve3::circle(center, axis, 1.5);
        let n = axis.normalize();
        for i in 0..12 {
            let t = TWO_PI * f64::from(i) / 12.0;
            let p = c.point(t);
            let r = p - center;
            assert!((r.norm() - 1.5).abs() < EPS, "radius drift at t={t}");
            assert!(r.dot(&n).abs() < EPS, "point off plane at t={t}");
        }
    }

    #[test]
    fn circle_derivatives() {
        let c = Curve3::circle(Point3::origin(), Vector3::z(), 3.0);
        // Tangent has magnitude r, is perpendicular to the radius vector.
        for t in [0.0, 0.4, FRAC_PI_2, 2.0, PI, 5.0] {
            let d = c.derivative(t);
            assert!((d.norm() - 3.0).abs() < EPS);
            let radial = c.point(t) - Point3::origin();
            assert!(d.dot(&radial).abs() < EPS);
            // Second derivative is centripetal: -radial.
            assert_vec_eq(&c.second_derivative(t), &-radial);
            check_derivatives_numerically(&c, t);
        }
    }

    #[test]
    fn circle_counterclockwise_about_axis() {
        let c = Curve3::circle(Point3::origin(), Vector3::z(), 1.0);
        // r × dr/dt must point along +axis (right-hand rule).
        let cross = (c.point(0.3) - Point3::origin()).cross(&c.derivative(0.3));
        assert!(cross.z > 0.0);
        assert!(cross.x.abs() < EPS && cross.y.abs() < EPS);
    }

    #[test]
    fn circle_periodicity_and_domain() {
        let c = Curve3::circle(Point3::new(0.0, 1.0, 2.0), Vector3::new(0.0, 1.0, 1.0), 4.0);
        assert!(c.is_closed());
        assert!(c.is_periodic());
        assert_eq!(c.period(), Some(TWO_PI));
        assert_eq!(c.domain(), (0.0, TWO_PI));
        // Domain edges meet: point(0) == point(2π), and shifting by the
        // period reproduces points and derivatives.
        assert_point_eq(&c.point(0.0), &c.point(TWO_PI));
        assert_point_eq(&c.point(1.1), &c.point(1.1 + TWO_PI));
        assert_vec_eq(&c.derivative(1.1), &c.derivative(1.1 + TWO_PI));
    }

    #[test]
    #[should_panic(expected = "radius must be positive")]
    fn circle_rejects_nonpositive_radius() {
        Curve3::circle(Point3::origin(), Vector3::z(), 0.0);
    }

    #[test]
    fn ellipse_analytic_points() {
        let c = Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 3.0, 1.0);
        assert_point_eq(&c.point(0.0), &Point3::new(3.0, 0.0, 0.0));
        assert_point_eq(&c.point(FRAC_PI_2), &Point3::new(0.0, 1.0, 0.0));
        assert_point_eq(&c.point(PI), &Point3::new(-3.0, 0.0, 0.0));
        assert_point_eq(&c.point(3.0 * FRAC_PI_2), &Point3::new(0.0, -1.0, 0.0));
    }

    #[test]
    fn ellipse_satisfies_implicit_equation() {
        let center = Point3::new(1.0, -2.0, 0.5);
        let axis = Vector3::new(0.0, 1.0, 2.0);
        let major_dir = Vector3::x();
        let c = Curve3::ellipse(center, axis, major_dir, 2.5, 1.5);
        let (u, v) = match &c {
            Curve3::Ellipse {
                axis, major_dir, ..
            } => (*major_dir, axis.cross(major_dir)),
            _ => unreachable!(),
        };
        for i in 0..12 {
            let t = TWO_PI * f64::from(i) / 12.0;
            let r = c.point(t) - center;
            let x = r.dot(&u) / 2.5;
            let y = r.dot(&v) / 1.5;
            assert!((x * x + y * y - 1.0).abs() < EPS, "off ellipse at t={t}");
            assert!(r.dot(&axis.normalize()).abs() < EPS, "off plane at t={t}");
        }
    }

    #[test]
    fn ellipse_derivatives() {
        let c = Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 3.0, 1.0);
        assert_vec_eq(&c.derivative(0.0), &Vector3::new(0.0, 1.0, 0.0));
        assert_vec_eq(&c.derivative(FRAC_PI_2), &Vector3::new(-3.0, 0.0, 0.0));
        assert_vec_eq(&c.second_derivative(0.0), &Vector3::new(-3.0, 0.0, 0.0));
        assert_vec_eq(
            &c.second_derivative(FRAC_PI_2),
            &Vector3::new(0.0, -1.0, 0.0),
        );
        for t in [0.0, 0.7, 2.0, PI, 4.5] {
            check_derivatives_numerically(&c, t);
        }
    }

    #[test]
    fn ellipse_with_equal_radii_matches_circle() {
        let center = Point3::new(2.0, 0.0, -1.0);
        let e = Curve3::ellipse(center, Vector3::z(), Vector3::x(), 2.0, 2.0);
        // plane_basis(z) also yields u = X, v = Y, so evaluations agree.
        let c = Curve3::circle(center, Vector3::z(), 2.0);
        for t in [0.0, 1.0, 2.5, 4.0, 6.0] {
            assert_point_eq(&e.point(t), &c.point(t));
            assert_vec_eq(&e.derivative(t), &c.derivative(t));
            assert_vec_eq(&e.second_derivative(t), &c.second_derivative(t));
        }
    }

    #[test]
    fn ellipse_orthogonalizes_major_dir() {
        // major_dir has a component along the axis; the constructor must
        // project it into the plane and normalize.
        let c = Curve3::ellipse(
            Point3::origin(),
            Vector3::z(),
            Vector3::new(1.0, 0.0, 0.7),
            2.0,
            1.0,
        );
        match &c {
            Curve3::Ellipse {
                axis, major_dir, ..
            } => {
                assert!((major_dir.norm() - 1.0).abs() < EPS);
                assert!(major_dir.dot(axis).abs() < EPS);
                assert_vec_eq(major_dir, &Vector3::x());
            }
            _ => unreachable!(),
        }
    }

    #[test]
    fn ellipse_periodicity() {
        let c = Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 3.0, 1.0);
        assert!(c.is_closed());
        assert!(c.is_periodic());
        assert_eq!(c.period(), Some(TWO_PI));
        assert_point_eq(&c.point(0.0), &c.point(TWO_PI));
    }

    #[test]
    #[should_panic(expected = "parallel to axis")]
    fn ellipse_rejects_major_dir_parallel_to_axis() {
        Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::z(), 2.0, 1.0);
    }

    #[test]
    #[should_panic(expected = "must not exceed")]
    fn ellipse_rejects_minor_greater_than_major() {
        Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 1.0, 2.0);
    }

    #[test]
    fn plane_basis_is_orthonormal_and_right_handed() {
        for axis in [
            Vector3::x(),
            Vector3::y(),
            Vector3::z(),
            Vector3::new(1.0, 1.0, 1.0).normalize(),
            Vector3::new(-0.99, 0.1, 0.05).normalize(),
        ] {
            let (u, v) = plane_basis(&axis);
            assert!((u.norm() - 1.0).abs() < EPS);
            assert!((v.norm() - 1.0).abs() < EPS);
            assert!(u.dot(&v).abs() < EPS);
            assert!(u.dot(&axis).abs() < EPS);
            assert!(v.dot(&axis).abs() < EPS);
            assert_vec_eq(&u.cross(&v), &axis);
        }
    }
}
