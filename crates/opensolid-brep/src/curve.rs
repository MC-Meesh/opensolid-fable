//! Parametric 3D curves: analytic primitives and the evaluation trait.
//!
//! Parameterization conventions:
//! - `Line`: parameterized by arc length; `dir` is a unit vector, so
//!   `point(t)` is exactly `t` units from `origin`. Domain is unbounded.
//! - `Circle` / `Ellipse`: parameterized by angle in radians over `[0, 2π)`,
//!   counterclockwise when viewed from the tip of `axis` (right-hand rule).
//!   `t = 0` lies along the reference x-direction of the curve's frame.
//! - `Nurbs`: parameterized by its own knot vector over the finite domain
//!   `[knots[p], knots[n − p]]`, which is whatever the curve was built with
//!   — not normalized to `[0, 1]`. Evaluation outside it clamps, so a
//!   closed NURBS curve is not periodic.

use crate::nurbs::NurbsCurve;
use opensolid_core::error::{CoreError, CoreResult};
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

/// Analytic 3D curve primitives, plus the sampled polyline curve that
/// carries marched (numerically traced) intersection geometry.
#[derive(Debug, Clone, PartialEq)]
pub enum Curve3 {
    /// Infinite line through `origin` with unit direction `dir`,
    /// parameterized by arc length.
    Line { origin: Point3, dir: Vector3 },
    /// Piecewise-linear curve through `points`, parameterized by vertex
    /// index: `point(t)` interpolates segment `⌊t⌋` at fraction `t − ⌊t⌋`,
    /// so the domain is `[0, len − 1]`. A closed polyline repeats its
    /// first vertex as the last one (`points[len − 1] == points[0]`) and
    /// is periodic with period `len − 1`.
    ///
    /// This is the exact-geometry representation of marched SSI curves
    /// ([`crate::ssi::intersect_marched`]) whose intersections have no
    /// closed form; fitting them as NURBS curves is a later hardening
    /// pass.
    Polyline { points: Vec<Point3>, closed: bool },
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
    /// Freeform clamped NURBS curve. Unlike every analytic variant, its
    /// parameter domain is the finite knot interval and evaluation outside
    /// it clamps rather than extrapolating or wrapping — so a NURBS curve
    /// whose endpoints coincide is [`is_closed`](CurveEval::is_closed) but
    /// never periodic (see [`CurveEval::is_periodic`] below).
    ///
    /// Boxed for the same reason [`crate::surface::Surface3::Nurbs`] is:
    /// [`NurbsCurve`] owns two `Vec`s plus a knot vector, and inlining them
    /// would grow every `Curve3` — the analytic variants are the hot ones.
    Nurbs(Box<NurbsCurve>),
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
    /// # Errors
    /// [`CoreError::Degenerate`] if `dir` has zero or non-finite length.
    pub fn line(origin: Point3, dir: Vector3) -> CoreResult<Self> {
        let norm = dir.norm();
        if norm == 0.0 || !norm.is_finite() {
            return Err(CoreError::Degenerate {
                context: "Curve3::line",
                reason: format!("direction must have non-zero finite length, got {dir}"),
            });
        }
        Ok(Curve3::Line {
            origin,
            dir: dir / norm,
        })
    }

    /// Circle of `radius` about `center` in the plane normal to `axis`
    /// (normalized here).
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `axis` has zero or non-finite length;
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    pub fn circle(center: Point3, axis: Vector3, radius: f64) -> CoreResult<Self> {
        let norm = axis.norm();
        if norm == 0.0 || !norm.is_finite() {
            return Err(CoreError::Degenerate {
                context: "Curve3::circle",
                reason: format!("axis must have non-zero finite length, got {axis}"),
            });
        }
        if radius <= 0.0 || !radius.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "radius",
                reason: format!("must be positive and finite, got {radius}"),
            });
        }
        Ok(Curve3::Circle {
            center,
            axis: axis / norm,
            radius,
        })
    }

    /// Ellipse about `center` in the plane normal to `axis`, with the major
    /// radius along `major_dir`. `axis` is normalized and `major_dir` is
    /// re-orthogonalized against it (Gram-Schmidt), so `major_dir` only needs
    /// to be non-parallel to `axis`.
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `axis` has zero or non-finite length, or
    /// if `major_dir` is (nearly) parallel to `axis`;
    /// [`CoreError::InvalidArgument`] if either radius is not positive and
    /// finite, or if `minor_radius > major_radius`.
    pub fn ellipse(
        center: Point3,
        axis: Vector3,
        major_dir: Vector3,
        major_radius: f64,
        minor_radius: f64,
    ) -> CoreResult<Self> {
        let axis_norm = axis.norm();
        if axis_norm == 0.0 || !axis_norm.is_finite() {
            return Err(CoreError::Degenerate {
                context: "Curve3::ellipse",
                reason: format!("axis must have non-zero finite length, got {axis}"),
            });
        }
        let axis = axis / axis_norm;
        let in_plane = major_dir - axis * major_dir.dot(&axis);
        let major_norm = in_plane.norm();
        if major_norm <= 1e-12 || !major_norm.is_finite() {
            return Err(CoreError::Degenerate {
                context: "Curve3::ellipse",
                reason: format!(
                    "major_dir {major_dir} must not be parallel to axis (or zero/non-finite)"
                ),
            });
        }
        for (name, r) in [
            ("major_radius", major_radius),
            ("minor_radius", minor_radius),
        ] {
            if r <= 0.0 || !r.is_finite() {
                return Err(CoreError::InvalidArgument {
                    argument: name,
                    reason: format!("must be positive and finite, got {r}"),
                });
            }
        }
        if minor_radius > major_radius {
            return Err(CoreError::InvalidArgument {
                argument: "minor_radius",
                reason: format!(
                    "must not exceed major_radius ({minor_radius} > {major_radius}); \
                     swap the radii and rotate major_dir if the minor axis is longer"
                ),
            });
        }
        Ok(Curve3::Ellipse {
            center,
            axis,
            major_dir: in_plane / major_norm,
            major_radius,
            minor_radius,
        })
    }

    /// Piecewise-linear curve through `points` parameterized by vertex
    /// index (see [`Curve3::Polyline`]). `closed` polylines must repeat
    /// their first point as the last one.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] with fewer than 2 points, a
    /// non-finite coordinate, or a closed polyline whose endpoints differ.
    pub fn polyline(points: Vec<Point3>, closed: bool) -> CoreResult<Self> {
        if points.len() < 2 {
            return Err(CoreError::InvalidArgument {
                argument: "points",
                reason: format!("polyline needs at least 2 points, got {}", points.len()),
            });
        }
        if points
            .iter()
            .any(|p| !p.coords.iter().all(|c| c.is_finite()))
        {
            return Err(CoreError::InvalidArgument {
                argument: "points",
                reason: "polyline points must be finite".into(),
            });
        }
        if closed && points[0] != points[points.len() - 1] {
            return Err(CoreError::InvalidArgument {
                argument: "points",
                reason: "a closed polyline must repeat its first point as the last one".into(),
            });
        }
        Ok(Curve3::Polyline { points, closed })
    }

    /// Freeform clamped NURBS curve over its knot interval.
    ///
    /// Accepts any curve [`NurbsCurve`] itself accepts — its constructors
    /// already enforce the invariants (matching control/weight/knot counts,
    /// positive weights, non-empty domain) that evaluation relies on, so
    /// there is nothing left for this wrapper to reject.
    pub fn nurbs(curve: NurbsCurve) -> Self {
        Curve3::Nurbs(Box::new(curve))
    }

    /// In-plane frame `(u, v)` for conic evaluation: `u` at t = 0, `v` at
    /// t = π/2.
    fn conic_frame(&self) -> Option<(Vector3, Vector3)> {
        match self {
            Curve3::Line { .. } | Curve3::Polyline { .. } | Curve3::Nurbs(_) => None,
            Curve3::Circle { axis, .. } => Some(plane_basis(axis)),
            Curve3::Ellipse {
                axis, major_dir, ..
            } => Some((*major_dir, axis.cross(major_dir))),
        }
    }
}

/// Segment index and interior fraction for a polyline parameter: `t`
/// wrapped by the period for closed polylines, clamped to the domain for
/// open ones, then split as `(⌊t⌋, t − ⌊t⌋)` with the final vertex mapped
/// onto the last segment's end.
fn polyline_segment(points: &[Point3], closed: bool, t: f64) -> (usize, f64) {
    let segs = points.len() - 1;
    let t = if closed {
        t.rem_euclid(segs as f64)
    } else {
        t.clamp(0.0, segs as f64)
    };
    let i = (t.floor() as usize).min(segs - 1);
    (i, t - i as f64)
}

impl CurveEval for Curve3 {
    fn point(&self, t: f64) -> Point3 {
        match self {
            Curve3::Line { origin, dir } => origin + dir * t,
            Curve3::Polyline { points, closed } => {
                let (i, f) = polyline_segment(points, *closed, t);
                points[i] + (points[i + 1] - points[i]) * f
            }
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
            Curve3::Nurbs(nurbs) => nurbs.point(t),
        }
    }

    fn derivative(&self, t: f64) -> Vector3 {
        match self {
            Curve3::Line { dir, .. } => *dir,
            // Piecewise constant: the chord vector of the segment under `t`
            // (one parameter unit spans one segment).
            Curve3::Polyline { points, closed } => {
                let (i, _) = polyline_segment(points, *closed, t);
                points[i + 1] - points[i]
            }
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
            Curve3::Nurbs(nurbs) => nurbs.derivative(t),
        }
    }

    fn second_derivative(&self, t: f64) -> Vector3 {
        match self {
            Curve3::Line { .. } | Curve3::Polyline { .. } => Vector3::zeros(),
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
            Curve3::Nurbs(nurbs) => nurbs.second_derivative(t),
        }
    }

    fn domain(&self) -> (f64, f64) {
        match self {
            Curve3::Line { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => (0.0, TWO_PI),
            Curve3::Polyline { points, .. } => (0.0, (points.len() - 1) as f64),
            Curve3::Nurbs(nurbs) => nurbs.domain(),
        }
    }

    fn is_closed(&self) -> bool {
        match self {
            Curve3::Line { .. } => false,
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => true,
            Curve3::Polyline { closed, .. } => *closed,
            // Geometric, not declared: the patch reports whether its two
            // domain ends actually meet.
            Curve3::Nurbs(nurbs) => nurbs.is_closed(),
        }
    }

    /// Closed *and* wrapping. Every variant but the freeform one repeats
    /// with its period once closed; a clamped NURBS curve does not, because
    /// evaluation outside the knot interval clamps to the ends instead of
    /// wrapping — a closed one traces its locus exactly once. Callers that
    /// mean "start meets end" want [`is_closed`](CurveEval::is_closed).
    fn is_periodic(&self) -> bool {
        match self {
            Curve3::Nurbs(nurbs) => nurbs.is_periodic(),
            _ => self.is_closed(),
        }
    }

    fn period(&self) -> Option<f64> {
        match self {
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => Some(TWO_PI),
            Curve3::Polyline {
                points,
                closed: true,
            } => Some((points.len() - 1) as f64),
            _ => None,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::KnotVector;
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
        let l = Curve3::line(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 5.0))
            .expect("valid curve");
        assert_point_eq(&l.point(0.0), &Point3::new(1.0, 2.0, 3.0));
        // dir was length 5 but is normalized: t is arc length.
        assert_point_eq(&l.point(2.0), &Point3::new(1.0, 2.0, 5.0));
        assert_point_eq(&l.point(-1.5), &Point3::new(1.0, 2.0, 1.5));
    }

    #[test]
    fn line_derivatives() {
        let l = Curve3::line(Point3::origin(), Vector3::new(3.0, 0.0, 4.0)).expect("valid curve");
        let d = l.derivative(7.0);
        assert!((d.norm() - 1.0).abs() < EPS, "unit tangent expected");
        assert_vec_eq(&d, &Vector3::new(0.6, 0.0, 0.8));
        assert_vec_eq(&l.second_derivative(-2.0), &Vector3::zeros());
        check_derivatives_numerically(&l, 1.25);
    }

    #[test]
    fn line_domain_and_topology() {
        let l = Curve3::line(Point3::origin(), Vector3::x()).expect("valid curve");
        let (t0, t1) = l.domain();
        assert!(t0.is_infinite() && t0 < 0.0);
        assert!(t1.is_infinite() && t1 > 0.0);
        assert!(!l.is_closed());
        assert!(!l.is_periodic());
        assert_eq!(l.period(), None);
    }

    #[test]
    fn line_rejects_zero_direction() {
        let err = Curve3::line(Point3::origin(), Vector3::zeros()).unwrap_err();
        assert!(matches!(err, CoreError::Degenerate { .. }), "got {err}");
        let msg = err.to_string();
        assert!(msg.contains("Curve3::line"), "missing context: {msg}");
        assert!(msg.contains("non-zero"), "missing constraint: {msg}");
    }

    #[test]
    fn circle_analytic_points() {
        // Axis = +Z: plane_basis seeds from world X, so u = X, v = Y.
        let c = Curve3::circle(Point3::new(1.0, 1.0, 0.0), Vector3::z(), 2.0).expect("valid curve");
        assert_point_eq(&c.point(0.0), &Point3::new(3.0, 1.0, 0.0));
        assert_point_eq(&c.point(FRAC_PI_2), &Point3::new(1.0, 3.0, 0.0));
        assert_point_eq(&c.point(PI), &Point3::new(-1.0, 1.0, 0.0));
        assert_point_eq(&c.point(3.0 * FRAC_PI_2), &Point3::new(1.0, -1.0, 0.0));
    }

    #[test]
    fn circle_arbitrary_axis_stays_on_circle() {
        let center = Point3::new(-2.0, 5.0, 1.0);
        let axis = Vector3::new(1.0, 2.0, -3.0);
        let c = Curve3::circle(center, axis, 1.5).expect("valid curve");
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
        let c = Curve3::circle(Point3::origin(), Vector3::z(), 3.0).expect("valid curve");
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
        let c = Curve3::circle(Point3::origin(), Vector3::z(), 1.0).expect("valid curve");
        // r × dr/dt must point along +axis (right-hand rule).
        let cross = (c.point(0.3) - Point3::origin()).cross(&c.derivative(0.3));
        assert!(cross.z > 0.0);
        assert!(cross.x.abs() < EPS && cross.y.abs() < EPS);
    }

    #[test]
    fn circle_periodicity_and_domain() {
        let c = Curve3::circle(Point3::new(0.0, 1.0, 2.0), Vector3::new(0.0, 1.0, 1.0), 4.0)
            .expect("valid curve");
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
    fn circle_rejects_nonpositive_radius() {
        for bad in [0.0, -2.0, f64::NAN, f64::INFINITY] {
            let err = Curve3::circle(Point3::origin(), Vector3::z(), bad).unwrap_err();
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "radius",
                        ..
                    }
                ),
                "radius {bad}: got {err}"
            );
        }
    }

    #[test]
    fn circle_rejects_zero_axis() {
        let err = Curve3::circle(Point3::origin(), Vector3::zeros(), 1.0).unwrap_err();
        assert!(matches!(err, CoreError::Degenerate { .. }), "got {err}");
        assert!(err.to_string().contains("axis"), "unhelpful message: {err}");
    }

    #[test]
    fn ellipse_analytic_points() {
        let c = Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 3.0, 1.0)
            .expect("valid curve");
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
        let c = Curve3::ellipse(center, axis, major_dir, 2.5, 1.5).expect("valid curve");
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
        let c = Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 3.0, 1.0)
            .expect("valid curve");
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
        let e = Curve3::ellipse(center, Vector3::z(), Vector3::x(), 2.0, 2.0).expect("valid curve");
        // plane_basis(z) also yields u = X, v = Y, so evaluations agree.
        let c = Curve3::circle(center, Vector3::z(), 2.0).expect("valid curve");
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
        )
        .expect("valid curve");
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
        let c = Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 3.0, 1.0)
            .expect("valid curve");
        assert!(c.is_closed());
        assert!(c.is_periodic());
        assert_eq!(c.period(), Some(TWO_PI));
        assert_point_eq(&c.point(0.0), &c.point(TWO_PI));
    }

    #[test]
    fn ellipse_rejects_major_dir_parallel_to_axis() {
        let err =
            Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::z(), 2.0, 1.0).unwrap_err();
        assert!(matches!(err, CoreError::Degenerate { .. }), "got {err}");
        assert!(
            err.to_string().contains("parallel to axis"),
            "unhelpful message: {err}"
        );
    }

    #[test]
    fn ellipse_rejects_minor_greater_than_major() {
        let err =
            Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 1.0, 2.0).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "minor_radius",
                    ..
                }
            ),
            "got {err}"
        );
        assert!(
            err.to_string().contains("must not exceed major_radius"),
            "unhelpful message: {err}"
        );
    }

    #[test]
    fn ellipse_rejects_nonpositive_radii() {
        let err =
            Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), -1.0, 1.0).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "major_radius",
                    ..
                }
            ),
            "got {err}"
        );
        let err =
            Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 2.0, 0.0).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "minor_radius",
                    ..
                }
            ),
            "got {err}"
        );
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

    fn open_polyline() -> Curve3 {
        Curve3::polyline(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
            ],
            false,
        )
        .unwrap()
    }

    /// Closed unit square in the xy-plane, first vertex repeated last.
    fn square_polyline() -> Curve3 {
        Curve3::polyline(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 0.0),
            ],
            true,
        )
        .unwrap()
    }

    #[test]
    fn polyline_evaluates_by_vertex_index() {
        let c = open_polyline();
        assert_point_eq(&c.point(0.0), &Point3::new(0.0, 0.0, 0.0));
        assert_point_eq(&c.point(0.5), &Point3::new(0.5, 0.0, 0.0));
        assert_point_eq(&c.point(1.0), &Point3::new(1.0, 0.0, 0.0));
        assert_point_eq(&c.point(1.25), &Point3::new(1.0, 0.5, 0.0));
        assert_point_eq(&c.point(2.0), &Point3::new(1.0, 2.0, 0.0));
        // Out-of-domain parameters clamp to the endpoints.
        assert_point_eq(&c.point(-3.0), &Point3::new(0.0, 0.0, 0.0));
        assert_point_eq(&c.point(7.0), &Point3::new(1.0, 2.0, 0.0));
        // Piecewise-constant derivative: the chord of the segment under t.
        assert_vec_eq(&c.derivative(0.5), &Vector3::new(1.0, 0.0, 0.0));
        assert_vec_eq(&c.derivative(1.5), &Vector3::new(0.0, 2.0, 0.0));
        assert_vec_eq(&c.second_derivative(0.5), &Vector3::zeros());
        assert_eq!(c.domain(), (0.0, 2.0));
        assert!(!c.is_closed());
        assert!(!c.is_periodic());
        assert_eq!(c.period(), None);
    }

    #[test]
    fn polyline_closed_wraps_periodically() {
        let c = square_polyline();
        assert_eq!(c.domain(), (0.0, 4.0));
        assert!(c.is_closed());
        assert!(c.is_periodic());
        assert_eq!(c.period(), Some(4.0));
        assert_point_eq(&c.point(4.0), &c.point(0.0));
        assert_point_eq(&c.point(4.25), &c.point(0.25));
        assert_point_eq(&c.point(-0.5), &c.point(3.5));
        assert_vec_eq(&c.derivative(4.5), &c.derivative(0.5));
    }

    /// Exact unit circle in the XY plane as a rational quadratic (Piegl &
    /// Tiller §7.5): closed, and rational, so it exercises the weighted
    /// evaluation path rather than a plain B-spline.
    fn nurbs_unit_circle() -> Curve3 {
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let curve = NurbsCurve::new(
            vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(-1.0, 1.0, 0.0),
                Point3::new(-1.0, 0.0, 0.0),
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(0.0, -1.0, 0.0),
                Point3::new(1.0, -1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
            ],
            vec![1.0, s, 1.0, s, 1.0, s, 1.0, s, 1.0],
            KnotVector::new(
                2,
                vec![
                    0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
                ],
            )
            .unwrap(),
        )
        .unwrap();
        Curve3::nurbs(curve)
    }

    /// Open cubic B-spline over the domain `[0, 2]`, so the tests cannot
    /// pass by accident on a curve whose domain happens to be `[0, 1]`.
    fn nurbs_open_cubic() -> Curve3 {
        let curve = NurbsCurve::bspline(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.5),
                Point3::new(3.0, 2.5, -1.0),
                Point3::new(4.0, 0.0, 2.0),
                Point3::new(6.0, -1.0, 1.0),
            ],
            KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0, 2.0]).unwrap(),
        )
        .unwrap();
        Curve3::nurbs(curve)
    }

    #[test]
    fn nurbs_evaluation_matches_the_inner_curve() {
        let inner = match &nurbs_unit_circle() {
            Curve3::Nurbs(n) => (**n).clone(),
            _ => unreachable!(),
        };
        let wrapped = nurbs_unit_circle();
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            assert_point_eq(&wrapped.point(t), &inner.point(t));
            assert_vec_eq(&wrapped.derivative(t), &inner.derivative(t));
            assert_vec_eq(&wrapped.second_derivative(t), &inner.second_derivative(t));
        }
    }

    #[test]
    fn nurbs_circle_is_geometrically_exact() {
        let c = nurbs_unit_circle();
        for i in 0..=40 {
            let t = i as f64 / 40.0;
            let r = c.point(t) - Point3::origin();
            assert!(
                (r.norm() - 1.0).abs() < 1e-12,
                "off the unit circle at t={t}"
            );
            assert!(r.z.abs() < 1e-12, "left the XY plane at t={t}");
        }
    }

    #[test]
    fn nurbs_derivatives_agree_with_finite_differences() {
        // Away from the C1-reducing repeated knots at 0.25/0.5/0.75, where
        // the one-sided derivatives of the circle representation differ and
        // a central difference straddles both.
        let c = nurbs_unit_circle();
        for t in [0.1, 0.35, 0.6, 0.9] {
            check_derivatives_numerically(&c, t);
        }
        let open = nurbs_open_cubic();
        for t in [0.3, 0.7, 1.4, 1.8] {
            check_derivatives_numerically(&open, t);
        }
    }

    #[test]
    fn nurbs_domain_comes_from_the_knot_vector() {
        assert_eq!(nurbs_unit_circle().domain(), (0.0, 1.0));
        assert_eq!(nurbs_open_cubic().domain(), (0.0, 2.0));
    }

    #[test]
    fn nurbs_evaluation_clamps_outside_the_domain() {
        // Clamped curves do not extrapolate: past either end the curve
        // pins to the endpoint rather than continuing.
        let c = nurbs_open_cubic();
        let (t0, t1) = c.domain();
        assert_point_eq(&c.point(t0 - 5.0), &c.point(t0));
        assert_point_eq(&c.point(t1 + 5.0), &c.point(t1));
    }

    #[test]
    fn nurbs_is_closed_but_never_periodic() {
        let circle = nurbs_unit_circle();
        assert!(circle.is_closed(), "the circle's ends meet");
        // The distinction the analytic variants do not need: a clamped
        // NURBS traces its locus once, so nothing wraps.
        assert!(!circle.is_periodic());
        assert_eq!(circle.period(), None);

        let open = nurbs_open_cubic();
        assert!(!open.is_closed());
        assert!(!open.is_periodic());
        assert_eq!(open.period(), None);
    }

    #[test]
    fn nurbs_has_no_conic_frame() {
        // `conic_frame` is the guard the Circle/Ellipse arms unwrap; a
        // freeform curve must never claim one.
        assert!(nurbs_unit_circle().conic_frame().is_none());
    }

    #[test]
    fn polyline_rejects_invalid_input() {
        assert!(Curve3::polyline(vec![Point3::origin()], false).is_err());
        assert!(
            Curve3::polyline(
                vec![Point3::origin(), Point3::new(f64::NAN, 0.0, 0.0)],
                false
            )
            .is_err()
        );
        // Closed polylines must repeat their first point as the last one.
        assert!(
            Curve3::polyline(
                vec![
                    Point3::origin(),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
                true
            )
            .is_err()
        );
    }
}
