//! Closest-point projection: point → curve and point → surface.
//!
//! Projection minimizes the squared distance to the target by Newton
//! iteration on the stationarity conditions (residual orthogonal to the
//! tangent(s), Piegl & Tiller §6.1). Seeds:
//! - Analytic primitives ([`Curve3`], [`Surface3`]) get closed-form seeds:
//!   exact for line/circle/plane/cylinder/sphere/torus, near-exact for
//!   ellipse (stretched-angle) and cone (foot on the nearest of the two
//!   candidate ruling lines), so Newton is a polish step.
//! - NURBS ([`NurbsCurve`], [`NurbsSurface`]) are coarsely sampled per
//!   non-empty knot span (`degree + 2` samples per span) and Newton starts
//!   from the best sample.
//!
//! Parameters are restricted to the domain each step: wrapped by the period
//! for periodic directions, clamped at the ends otherwise, so projections
//! onto bounded (e.g. clamped NURBS) geometry converge to the boundary when
//! the unconstrained optimum lies outside.
//!
//! Results carry a `converged` flag instead of an error: the best parameter
//! found is always returned, and `converged: false` marks the rare cases
//! where the iteration stalled before meeting the tolerances (degenerate
//! Hessian away from any minimum, iteration cap). Ambiguous inputs — the
//! center of a circle or sphere, points on a cylinder or torus axis — are
//! equidistant from a whole locus; projection returns one valid closest
//! point with `converged: true`.

use crate::curve::{Curve3, CurveEval, plane_basis};
use crate::nurbs::{KnotVector, NurbsCurve, NurbsSurface};
use crate::surface::{Surface3, SurfaceEval};
use opensolid_core::SYSTEM_RESOLUTION;
use opensolid_core::types::{Point3, Vector3};

/// Iteration cap for the Newton polish. Analytic seeds converge in a few
/// steps; the cap only matters for slow first-order tails near
/// parameterization singularities (e.g. sphere poles).
const MAX_ITERATIONS: usize = 64;

/// Cosine tolerance for the orthogonality convergence test:
/// `|tangent · residual| ≤ COS_TOL · |tangent| · |residual|`.
const COS_TOL: f64 = 1e-10;

/// Result of projecting a point onto a curve.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct CurveProjection {
    /// Parameter of the closest point, inside the curve domain (wrapped for
    /// periodic curves, clamped for bounded ones).
    pub t: f64,
    /// The closest point, `curve.point(t)`.
    pub point: Point3,
    /// Distance from the query point to `point`.
    pub distance: f64,
    /// Whether the iteration met the convergence tolerances. The best
    /// parameter found is returned either way.
    pub converged: bool,
}

/// Result of projecting a point onto a surface.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SurfaceProjection {
    /// `u` parameter of the closest point, inside the surface domain.
    pub u: f64,
    /// `v` parameter of the closest point, inside the surface domain.
    pub v: f64,
    /// The closest point, `surface.point(u, v)`.
    pub point: Point3,
    /// Distance from the query point to `point`.
    pub distance: f64,
    /// Whether the iteration met the convergence tolerances. The best
    /// parameters found are returned either way.
    pub converged: bool,
}

/// Closest-point projection onto a parametric curve.
pub trait CurveProject {
    /// Closest point on the curve to `point`.
    fn project_point(&self, point: &Point3) -> CurveProjection;
}

/// Closest-point projection onto a parametric surface.
pub trait SurfaceProject {
    /// Closest point on the surface to `point`.
    fn project_point(&self, point: &Point3) -> SurfaceProjection;

    /// Closest point on the surface to `point`, with the Newton iteration
    /// started from `seed` instead of the implementation's own seeding.
    ///
    /// Use this wherever a good seed is already known (the previous point
    /// of a walk, say). Beyond being cheaper than a blind search, it picks
    /// the **branch**: a surface that approaches itself has several local
    /// minima, and unseeded projection can converge onto the wrong sheet.
    ///
    /// The default implementation ignores the seed, which is correct for
    /// surfaces whose own seeding is already a global closed-form answer.
    fn project_point_seeded(&self, point: &Point3, seed: (f64, f64)) -> SurfaceProjection {
        let _ = seed;
        self.project_point(point)
    }
}

/// One parameter direction's domain restriction: wrap by the period if
/// periodic, clamp to the interval otherwise.
#[derive(Debug, Clone, Copy)]
struct ParamBounds {
    lo: f64,
    hi: f64,
    period: Option<f64>,
}

impl ParamBounds {
    fn restrict(&self, t: f64) -> f64 {
        match self.period {
            Some(period) => self.lo + (t - self.lo).rem_euclid(period),
            None => t.clamp(self.lo, self.hi),
        }
    }
}

/// Newton iteration for `argmin_t |C(t) - P|²` from `seed`.
///
/// Stationarity g(t) = C'·(C-P) = 0 is solved with
/// g'(t) = C''·(C-P) + |C'|². Convergence: residual below
/// [`SYSTEM_RESOLUTION`] (point on curve), orthogonality within
/// [`COS_TOL`], or the restricted step moving the point less than
/// [`SYSTEM_RESOLUTION`] (interior floating-point floor, or pinned at a
/// clamped boundary).
fn newton_curve<C: CurveEval>(curve: &C, point: &Point3, seed: f64) -> CurveProjection {
    let (lo, hi) = curve.domain();
    let bounds = ParamBounds {
        lo,
        hi,
        period: if curve.is_periodic() {
            curve.period()
        } else {
            None
        },
    };
    let mut t = bounds.restrict(seed);
    let mut best_t = t;
    let mut best_dist = f64::INFINITY;
    let mut converged = false;
    for _ in 0..MAX_ITERATIONS {
        let r = curve.point(t) - point;
        let dist = r.norm();
        if dist < best_dist {
            best_dist = dist;
            best_t = t;
        }
        let d1 = curve.derivative(t);
        let g = d1.dot(&r);
        if dist <= SYSTEM_RESOLUTION || g.abs() <= COS_TOL * d1.norm() * dist {
            converged = true;
            break;
        }
        let gp = curve.second_derivative(t).dot(&r) + d1.norm_squared();
        let step = -g / gp;
        if !step.is_finite() {
            break;
        }
        let t_next = bounds.restrict(t + step);
        if (t_next - t).abs() * d1.norm() <= SYSTEM_RESOLUTION {
            t = t_next;
            if (curve.point(t) - point).norm() < best_dist {
                best_t = t;
            }
            converged = true;
            break;
        }
        t = t_next;
    }
    let closest = curve.point(best_t);
    CurveProjection {
        t: best_t,
        point: closest,
        distance: (closest - point).norm(),
        converged,
    }
}

/// Second-order evaluation frame at `(u, v)` for surface Newton iteration.
struct SurfaceJet {
    point: Point3,
    su: Vector3,
    sv: Vector3,
    suu: Vector3,
    suv: Vector3,
    svv: Vector3,
}

/// Surfaces that expose second partial derivatives for Newton projection.
trait NewtonSurface: SurfaceEval {
    fn jet(&self, u: f64, v: f64) -> SurfaceJet;
}

/// Newton iteration for `argmin_{u,v} |S(u,v) - P|²` from `(seed_u, seed_v)`.
///
/// Solves the 2×2 system for the stationarity conditions
/// `f = S_u·r = 0`, `g = S_v·r = 0` with the full Hessian (Piegl & Tiller
/// Eq. 6.5). Convergence criteria mirror [`newton_curve`], applied per
/// direction.
fn newton_surface<S: NewtonSurface>(
    surface: &S,
    point: &Point3,
    seed_u: f64,
    seed_v: f64,
) -> SurfaceProjection {
    let (u_lo, u_hi) = surface.domain_u();
    let (v_lo, v_hi) = surface.domain_v();
    let bounds_u = ParamBounds {
        lo: u_lo,
        hi: u_hi,
        period: if surface.is_periodic_u() {
            surface.period_u()
        } else {
            None
        },
    };
    let bounds_v = ParamBounds {
        lo: v_lo,
        hi: v_hi,
        period: if surface.is_periodic_v() {
            surface.period_v()
        } else {
            None
        },
    };
    let mut u = bounds_u.restrict(seed_u);
    let mut v = bounds_v.restrict(seed_v);
    let mut best = (u, v);
    let mut best_dist = f64::INFINITY;
    let mut converged = false;
    for _ in 0..MAX_ITERATIONS {
        let jet = surface.jet(u, v);
        let r = jet.point - point;
        let dist = r.norm();
        if dist < best_dist {
            best_dist = dist;
            best = (u, v);
        }
        let f = jet.su.dot(&r);
        let g = jet.sv.dot(&r);
        let orthogonal =
            f.abs() <= COS_TOL * jet.su.norm() * dist && g.abs() <= COS_TOL * jet.sv.norm() * dist;
        if dist <= SYSTEM_RESOLUTION || orthogonal {
            converged = true;
            break;
        }
        let a = jet.su.norm_squared() + r.dot(&jet.suu);
        let b = jet.su.dot(&jet.sv) + r.dot(&jet.suv);
        let c = jet.sv.norm_squared() + r.dot(&jet.svv);
        let det = a * c - b * b;
        let du = (b * g - c * f) / det;
        let dv = (b * f - a * g) / det;
        if !du.is_finite() || !dv.is_finite() {
            break;
        }
        let mut u_next = bounds_u.restrict(u + du);
        let mut v_next = bounds_v.restrict(v + dv);
        // Active-set fallback: if the joint step runs a clamped (aperiodic)
        // parameter out of its domain, that parameter pins to the bound and
        // the optimum must be re-sought along the free direction alone —
        // the pinned component of the joint solve would otherwise stall the
        // free one short of the constrained minimum.
        let u_pinned = bounds_u.period.is_none() && (u + du < bounds_u.lo || u + du > bounds_u.hi);
        let v_pinned = bounds_v.period.is_none() && (v + dv < bounds_v.lo || v + dv > bounds_v.hi);
        if u_pinned && !v_pinned {
            let dv_1d = -g / c;
            if dv_1d.is_finite() {
                v_next = bounds_v.restrict(v + dv_1d);
            }
        } else if v_pinned && !u_pinned {
            let du_1d = -f / a;
            if du_1d.is_finite() {
                u_next = bounds_u.restrict(u + du_1d);
            }
        }
        let moved = ((u_next - u) * jet.su + (v_next - v) * jet.sv).norm();
        if moved <= SYSTEM_RESOLUTION {
            u = u_next;
            v = v_next;
            if (surface.point(u, v) - point).norm() < best_dist {
                best = (u, v);
            }
            converged = true;
            break;
        }
        u = u_next;
        v = v_next;
    }
    let (u, v) = best;
    let closest = surface.point(u, v);
    SurfaceProjection {
        u,
        v,
        point: closest,
        distance: (closest - point).norm(),
        converged,
    }
}

/// Radial and tangential unit directions at angle `u` about `axis`
/// (matching the frame conventions of [`Surface3`]).
fn radial_tangential(axis: &Vector3, u: f64) -> (Vector3, Vector3) {
    let (e_u, e_v) = plane_basis(axis);
    (e_u * u.cos() + e_v * u.sin(), e_v * u.cos() - e_u * u.sin())
}

impl CurveProject for Curve3 {
    fn project_point(&self, point: &Point3) -> CurveProjection {
        let seed = match self {
            Curve3::Line { origin, dir } => (point - origin).dot(dir),
            // Piecewise linear: the per-segment minimum is exact, so scan
            // every segment instead of Newton (the derivative is
            // discontinuous at the vertices).
            Curve3::Polyline { points, .. } => {
                let mut best = (f64::INFINITY, 0.0);
                for (i, w) in points.windows(2).enumerate() {
                    let ab = w[1] - w[0];
                    let len2 = ab.norm_squared();
                    let s = if len2 > 0.0 {
                        ((point - w[0]).dot(&ab) / len2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    let d = (point - (w[0] + ab * s)).norm();
                    if d < best.0 {
                        best = (d, i as f64 + s);
                    }
                }
                let t = best.1;
                return CurveProjection {
                    t,
                    point: self.point(t),
                    distance: best.0,
                    converged: true,
                };
            }
            Curve3::Circle { center, axis, .. } => {
                let (e_u, e_v) = plane_basis(axis);
                let d = point - center;
                d.dot(&e_v).atan2(d.dot(&e_u))
            }
            Curve3::Ellipse {
                center,
                axis,
                major_dir,
                major_radius,
                minor_radius,
            } => {
                // Stretched-angle seed: exact for points on the ellipse,
                // close elsewhere; Newton finishes the job.
                let minor_dir = axis.cross(major_dir);
                let d = point - center;
                (major_radius * d.dot(&minor_dir)).atan2(minor_radius * d.dot(major_dir))
            }
        };
        newton_curve(self, point, seed)
    }
}

/// Sample parameters covering every non-empty knot span inside the domain:
/// `per_span` evenly spaced values from each span start (inclusive), plus
/// the domain end.
fn span_samples(knots: &KnotVector, per_span: usize) -> Vec<f64> {
    let (t0, t1) = knots.domain();
    let mut samples = Vec::new();
    for window in knots.knots().windows(2) {
        let (a, b) = (window[0].max(t0), window[1].min(t1));
        if b > a {
            for i in 0..per_span {
                samples.push(a + (b - a) * i as f64 / per_span as f64);
            }
        }
    }
    samples.push(t1);
    samples
}

impl CurveProject for NurbsCurve {
    fn project_point(&self, point: &Point3) -> CurveProjection {
        let mut seed = 0.0;
        let mut best = f64::INFINITY;
        for t in span_samples(self.knot_vector(), self.degree() + 2) {
            let d = (self.point(t) - point).norm_squared();
            if d < best {
                best = d;
                seed = t;
            }
        }
        newton_curve(self, point, seed)
    }
}

impl NewtonSurface for Surface3 {
    fn jet(&self, u: f64, v: f64) -> SurfaceJet {
        let zero = Vector3::zeros();
        let (suu, suv, svv) = match self {
            // No closed-form second derivatives to fold into the shared
            // tail below; take the whole jet from the patch instead.
            Surface3::Nurbs(nurbs) => return nurbs.jet(u, v),
            Surface3::Plane { .. } => (zero, zero, zero),
            Surface3::Cylinder { axis, radius, .. } => {
                let (radial, _) = radial_tangential(axis, u);
                (-radial * *radius, zero, zero)
            }
            Surface3::Cone {
                axis,
                half_angle,
                radius,
                ..
            } => {
                let (radial, tangential) = radial_tangential(axis, u);
                let rho = radius + v * half_angle.tan();
                (-radial * rho, tangential * half_angle.tan(), zero)
            }
            Surface3::Sphere { axis, radius, .. } => {
                let (radial, tangential) = radial_tangential(axis, u);
                (
                    -radial * (radius * v.cos()),
                    -tangential * (radius * v.sin()),
                    -(radial * v.cos() + axis * v.sin()) * *radius,
                )
            }
            Surface3::Torus {
                axis,
                major_radius,
                minor_radius,
                ..
            } => {
                let (radial, tangential) = radial_tangential(axis, u);
                (
                    -radial * (major_radius + minor_radius * v.cos()),
                    -tangential * (minor_radius * v.sin()),
                    -(radial * v.cos() + axis * v.sin()) * *minor_radius,
                )
            }
        };
        SurfaceJet {
            point: self.point(u, v),
            su: self.du(u, v),
            sv: self.dv(u, v),
            suu,
            suv,
            svv,
        }
    }
}

impl SurfaceProject for Surface3 {
    fn project_point(&self, point: &Point3) -> SurfaceProjection {
        let (seed_u, seed_v) = match self {
            // No closed-form seed: fall through to the patch's own
            // per-knot-span search.
            Surface3::Nurbs(nurbs) => return nurbs.project_point(point),
            Surface3::Plane { origin, normal } => {
                let (e_u, e_v) = plane_basis(normal);
                let d = point - origin;
                (d.dot(&e_u), d.dot(&e_v))
            }
            Surface3::Cylinder { origin, axis, .. } => {
                let (e_u, e_v) = plane_basis(axis);
                let d = point - origin;
                (d.dot(&e_v).atan2(d.dot(&e_u)), d.dot(axis))
            }
            Surface3::Cone {
                origin,
                axis,
                half_angle,
                radius,
            } => {
                let (e_u, e_v) = plane_basis(axis);
                let d = point - origin;
                let angle = d.dot(&e_v).atan2(d.dot(&e_u));
                // The closest point lies on one of the two ruling lines in
                // the point's axial plane: at `angle` (near nappe) or
                // diametrically opposite (which the same line reaches past
                // the apex as the mirror nappe). Take the closer foot.
                let candidate = |u: f64| {
                    let (radial, _) = radial_tangential(axis, u);
                    let ruling_origin = origin + radial * *radius;
                    let ruling_dir = radial * half_angle.tan() + axis;
                    let v = (point - ruling_origin).dot(&ruling_dir) / ruling_dir.norm_squared();
                    let foot = ruling_origin + ruling_dir * v;
                    ((point - foot).norm_squared(), u, v)
                };
                let near = candidate(angle);
                let far = candidate(angle + std::f64::consts::PI);
                let (_, u, v) = if near.0 <= far.0 { near } else { far };
                (u, v)
            }
            Surface3::Sphere { center, axis, .. } => {
                let (e_u, e_v) = plane_basis(axis);
                let d = point - center;
                let (x, y, z) = (d.dot(&e_u), d.dot(&e_v), d.dot(axis));
                // atan2 against the in-plane radius keeps the latitude seed
                // accurate at the poles (asin of a near-1 ratio is not).
                (y.atan2(x), z.atan2(x.hypot(y)))
            }
            Surface3::Torus {
                center,
                axis,
                major_radius,
                ..
            } => {
                let (e_u, e_v) = plane_basis(axis);
                let d = point - center;
                let u = d.dot(&e_v).atan2(d.dot(&e_u));
                let (radial, _) = radial_tangential(axis, u);
                let w = point - (center + radial * *major_radius);
                (u, w.dot(axis).atan2(w.dot(&radial)))
            }
        };
        newton_surface(self, point, seed_u, seed_v)
    }

    fn project_point_seeded(&self, point: &Point3, seed: (f64, f64)) -> SurfaceProjection {
        match self {
            Surface3::Nurbs(nurbs) => nurbs.project_point_seeded(point, seed),
            // Every analytic seed above is already a closed-form global
            // answer, so a caller's seed can only make it worse.
            Surface3::Plane { .. }
            | Surface3::Cylinder { .. }
            | Surface3::Cone { .. }
            | Surface3::Sphere { .. }
            | Surface3::Torus { .. } => self.project_point(point),
        }
    }
}

impl NewtonSurface for NurbsSurface {
    fn jet(&self, u: f64, v: f64) -> SurfaceJet {
        let ders = self.derivatives(u, v, 2);
        SurfaceJet {
            point: Point3::origin() + ders[0][0],
            su: ders[1][0],
            sv: ders[0][1],
            suu: ders[2][0],
            suv: ders[1][1],
            svv: ders[0][2],
        }
    }
}

impl SurfaceProject for NurbsSurface {
    fn project_point(&self, point: &Point3) -> SurfaceProjection {
        let u_samples = span_samples(self.knot_vector_u(), self.degree_u() + 2);
        let v_samples = span_samples(self.knot_vector_v(), self.degree_v() + 2);
        let mut seed = (u_samples[0], v_samples[0]);
        let mut best = f64::INFINITY;
        for &u in &u_samples {
            for &v in &v_samples {
                let d = (self.point(u, v) - point).norm_squared();
                if d < best {
                    best = d;
                    seed = (u, v);
                }
            }
        }
        newton_surface(self, point, seed.0, seed.1)
    }

    /// Newton straight from `seed`, skipping the per-span search. The seed
    /// is clamped to the knot domains by [`newton_surface`]'s bounds.
    fn project_point_seeded(&self, point: &Point3, seed: (f64, f64)) -> SurfaceProjection {
        newton_surface(self, point, seed.0, seed.1)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, FRAC_PI_4, PI};

    const EPS: f64 = 1e-9;

    fn assert_near(a: f64, b: f64, context: &str) {
        assert!((a - b).abs() < EPS, "{context}: {a} vs {b}");
    }

    fn assert_point_near(a: &Point3, b: &Point3, context: &str) {
        assert!(
            (a - b).norm() < EPS,
            "{context}: {a:?} vs {b:?} (dist {})",
            (a - b).norm()
        );
    }

    /// The projection result must be self-consistent: `point` is the curve
    /// evaluated at `t` inside the domain, `distance` matches, and the
    /// residual is orthogonal to the tangent (or the point coincides).
    fn check_curve_projection(curve: &impl CurveEval, p: &Point3, proj: &CurveProjection) {
        assert!(proj.converged, "did not converge for {p:?}");
        let (lo, hi) = curve.domain();
        assert!(
            proj.t >= lo - EPS && proj.t <= hi + EPS,
            "t={} outside domain [{lo}, {hi}]",
            proj.t
        );
        assert_point_near(&curve.point(proj.t), &proj.point, "point vs t");
        assert_near((proj.point - p).norm(), proj.distance, "distance");
        let r = proj.point - p;
        let d1 = curve.derivative(proj.t);
        // A residual below tolerance is rounding noise with no meaningful
        // direction; only off-curve points get the orthogonality check.
        assert!(
            r.norm() < EPS || d1.dot(&r).abs() <= 1e-8 * d1.norm() * r.norm(),
            "residual not orthogonal to tangent: {}",
            d1.dot(&r)
        );
    }

    /// Same consistency checks for a surface projection.
    fn check_surface_projection(surface: &impl SurfaceEval, p: &Point3, proj: &SurfaceProjection) {
        assert!(proj.converged, "did not converge for {p:?}");
        assert_point_near(&surface.point(proj.u, proj.v), &proj.point, "point vs uv");
        assert_near((proj.point - p).norm(), proj.distance, "distance");
    }

    /// Densely sampled distance over a bounded parameter window: an upper
    /// bound on the true minimum used to catch wrong-local-minimum results.
    fn sampled_curve_min(curve: &impl CurveEval, p: &Point3, lo: f64, hi: f64) -> f64 {
        let n = 2000;
        (0..=n)
            .map(|i| lo + (hi - lo) * f64::from(i) / f64::from(n))
            .map(|t| (curve.point(t) - p).norm())
            .fold(f64::INFINITY, f64::min)
    }

    fn sampled_surface_min(
        surface: &impl SurfaceEval,
        p: &Point3,
        (u_lo, u_hi): (f64, f64),
        (v_lo, v_hi): (f64, f64),
    ) -> f64 {
        let n = 300;
        let mut min = f64::INFINITY;
        for i in 0..=n {
            let u = u_lo + (u_hi - u_lo) * f64::from(i) / f64::from(n);
            for j in 0..=n {
                let v = v_lo + (v_hi - v_lo) * f64::from(j) / f64::from(n);
                min = min.min((surface.point(u, v) - p).norm());
            }
        }
        min
    }

    // --- Curves: analytic ---

    #[test]
    fn line_projection_matches_closed_form() {
        // Non-unit input direction: the constructor normalizes, so t is arc
        // length along +z.
        let line = Curve3::line(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 5.0)).unwrap();
        let p = Point3::new(4.0, 6.0, 7.5);
        let proj = line.project_point(&p);
        check_curve_projection(&line, &p, &proj);
        assert_near(proj.t, 4.5, "t");
        assert_point_near(&proj.point, &Point3::new(1.0, 2.0, 7.5), "foot");
        assert_near(proj.distance, 5.0, "distance");
    }

    #[test]
    fn line_point_on_line_projects_to_itself() {
        let line = Curve3::line(Point3::origin(), Vector3::new(1.0, 1.0, 0.0)).unwrap();
        let p = Point3::new(2.0, 2.0, 0.0);
        let proj = line.project_point(&p);
        check_curve_projection(&line, &p, &proj);
        assert_near(proj.t, 8.0f64.sqrt(), "t");
        assert!(proj.distance < EPS, "distance {}", proj.distance);
    }

    #[test]
    fn circle_projection_matches_closed_form() {
        let center = Point3::new(1.0, 1.0, 0.0);
        let circle = Curve3::circle(center, Vector3::z(), 2.0).unwrap();
        // Radial distance 3, axial offset 1.5, at angle 0.7.
        let angle: f64 = 0.7;
        let p = center + Vector3::new(angle.cos(), angle.sin(), 0.0) * 3.0 + Vector3::z() * 1.5;
        let proj = circle.project_point(&p);
        check_curve_projection(&circle, &p, &proj);
        assert_near(proj.t, 0.7, "t");
        assert_point_near(
            &proj.point,
            &(center + Vector3::new(angle.cos(), angle.sin(), 0.0) * 2.0),
            "foot",
        );
        assert_near(proj.distance, (1.0f64 + 1.5 * 1.5).sqrt(), "distance");
    }

    #[test]
    fn circle_projection_wraps_into_domain() {
        let circle = Curve3::circle(Point3::origin(), Vector3::z(), 1.0).unwrap();
        let angle: f64 = -0.3;
        let p = Point3::new(2.0 * angle.cos(), 2.0 * angle.sin(), 0.0);
        let proj = circle.project_point(&p);
        check_curve_projection(&circle, &p, &proj);
        assert_near(proj.t, crate::curve::TWO_PI - 0.3, "t wrapped");
        assert_near(proj.distance, 1.0, "distance");
    }

    #[test]
    fn circle_center_is_ambiguous_but_converges() {
        let circle = Curve3::circle(Point3::new(0.0, 0.0, 2.0), Vector3::z(), 3.0).unwrap();
        // On the circle's axis: every parameter is equidistant.
        let p = Point3::new(0.0, 0.0, 4.0);
        let proj = circle.project_point(&p);
        check_curve_projection(&circle, &p, &proj);
        assert_near(proj.distance, (9.0f64 + 4.0).sqrt(), "distance");
    }

    #[test]
    fn ellipse_projection_is_stationary_and_globally_optimal() {
        let ellipse = Curve3::ellipse(
            Point3::new(1.0, -1.0, 0.5),
            Vector3::z(),
            Vector3::x(),
            3.0,
            1.0,
        )
        .unwrap();
        for p in [
            Point3::new(4.0, 1.0, 1.0),
            Point3::new(0.0, 2.5, 0.0),
            Point3::new(-3.0, -2.0, 2.0),
            Point3::new(1.5, -0.9, 0.5), // inside the ellipse
        ] {
            let proj = ellipse.project_point(&p);
            check_curve_projection(&ellipse, &p, &proj);
            let sampled = sampled_curve_min(&ellipse, &p, 0.0, crate::curve::TWO_PI);
            assert!(
                proj.distance <= sampled + 1e-6,
                "not globally optimal for {p:?}: {} vs sampled {sampled}",
                proj.distance
            );
        }
    }

    #[test]
    fn ellipse_point_on_curve_projects_to_itself() {
        let ellipse =
            Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 3.0, 1.0).unwrap();
        let p = ellipse.point(1.2);
        let proj = ellipse.project_point(&p);
        check_curve_projection(&ellipse, &p, &proj);
        assert_near(proj.t, 1.2, "t");
        assert!(proj.distance < EPS, "distance {}", proj.distance);
    }

    #[test]
    fn ellipse_interior_point_near_major_axis() {
        // For x > (a² - b²)/a = 8/3 the closest point is the major vertex.
        let ellipse =
            Curve3::ellipse(Point3::origin(), Vector3::z(), Vector3::x(), 3.0, 1.0).unwrap();
        let p = Point3::new(2.9, 0.0, 0.0);
        let proj = ellipse.project_point(&p);
        check_curve_projection(&ellipse, &p, &proj);
        assert_point_near(&proj.point, &Point3::new(3.0, 0.0, 0.0), "foot");
        assert_near(proj.distance, 0.1, "distance");
    }

    // --- Curves: NURBS ---

    /// Exact unit circle in the XY plane: rational quadratic, nine control
    /// points over four 90° arcs (Piegl & Tiller §7.5).
    fn nurbs_unit_circle() -> NurbsCurve {
        let pts = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        ];
        let s = FRAC_1_SQRT_2;
        let weights = vec![1.0, s, 1.0, s, 1.0, s, 1.0, s, 1.0];
        let knots = KnotVector::new(
            2,
            vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ],
        )
        .unwrap();
        NurbsCurve::new(pts, weights, knots).unwrap()
    }

    /// Generic rational cubic (same fixture as the NURBS evaluation tests).
    fn generic_rational_cubic() -> NurbsCurve {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.5),
            Point3::new(3.0, 2.5, -1.0),
            Point3::new(4.0, 0.0, 2.0),
            Point3::new(5.0, -1.5, 1.0),
            Point3::new(7.0, 1.0, 0.0),
        ];
        let weights = vec![1.0, 0.5, 2.0, 1.5, 0.8, 1.0];
        let knots =
            KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 0.4, 0.7, 1.0, 1.0, 1.0, 1.0]).unwrap();
        NurbsCurve::new(pts, weights, knots).unwrap()
    }

    #[test]
    fn nurbs_circle_projection_matches_analytic_circle() {
        let nurbs = nurbs_unit_circle();
        let analytic = Curve3::circle(Point3::origin(), Vector3::z(), 1.0).unwrap();
        for p in [
            Point3::new(2.0, 1.0, 0.5),
            Point3::new(-0.4, 0.3, -1.0),
            Point3::new(0.1, -2.0, 0.0),
        ] {
            let from_nurbs = nurbs.project_point(&p);
            let from_analytic = analytic.project_point(&p);
            check_curve_projection(&nurbs, &p, &from_nurbs);
            // Parameterizations differ (angle vs rational), so compare the
            // geometry, not the parameters.
            assert_point_near(&from_nurbs.point, &from_analytic.point, "foot");
            assert_near(from_nurbs.distance, from_analytic.distance, "distance");
        }
    }

    #[test]
    fn nurbs_open_curve_clamps_to_endpoint() {
        // Degree-1 segment from (0,0,0) to (2,0,0) over [0, 1].
        let segment = NurbsCurve::bspline(
            vec![Point3::origin(), Point3::new(2.0, 0.0, 0.0)],
            KnotVector::clamped_uniform(1, 2).unwrap(),
        )
        .unwrap();
        let p = Point3::new(3.0, 1.0, 0.0);
        let proj = segment.project_point(&p);
        assert!(proj.converged);
        assert_near(proj.t, 1.0, "t clamped to domain end");
        assert_point_near(&proj.point, &Point3::new(2.0, 0.0, 0.0), "endpoint");
        assert_near(proj.distance, 2.0f64.sqrt(), "distance");
    }

    #[test]
    fn nurbs_generic_cubic_projection_is_optimal() {
        let curve = generic_rational_cubic();
        for p in [
            Point3::new(2.0, 3.0, 0.0),
            Point3::new(5.0, -3.0, 1.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(8.0, 1.0, -1.0), // beyond the end: clamps
        ] {
            let proj = curve.project_point(&p);
            assert!(proj.converged, "did not converge for {p:?}");
            let sampled = sampled_curve_min(&curve, &p, 0.0, 1.0);
            assert!(
                proj.distance <= sampled + 1e-6,
                "not optimal for {p:?}: {} vs sampled {sampled}",
                proj.distance
            );
        }
    }

    // --- Surfaces: analytic ---

    #[test]
    fn analytic_surface_jets_match_finite_differences() {
        let surfaces = [
            Surface3::plane(Point3::new(1.0, 0.0, -1.0), Vector3::new(1.0, 2.0, 2.0)).unwrap(),
            Surface3::cylinder(Point3::new(0.0, 1.0, 0.0), Vector3::new(1.0, 1.0, 0.0), 1.5)
                .unwrap(),
            Surface3::cone(Point3::origin(), Vector3::new(0.2, -1.0, 0.4), 0.5, 1.2).unwrap(),
            Surface3::sphere(Point3::new(2.0, 2.0, 2.0), Vector3::new(1.0, 0.0, 1.0), 2.0).unwrap(),
            Surface3::torus(
                Point3::new(0.0, 0.0, 1.0),
                Vector3::new(0.0, 1.0, 2.0),
                3.0,
                0.8,
            )
            .unwrap(),
        ];
        let h = 1e-5;
        for surface in &surfaces {
            for (u, v) in [(0.3, -0.6), (1.9, 0.8), (4.4, 0.1)] {
                let jet = surface.jet(u, v);
                let fd_suu = (surface.du(u + h, v) - surface.du(u - h, v)) / (2.0 * h);
                let fd_suv = (surface.du(u, v + h) - surface.du(u, v - h)) / (2.0 * h);
                let fd_svv = (surface.dv(u, v + h) - surface.dv(u, v - h)) / (2.0 * h);
                assert!(
                    (jet.suu - fd_suu).norm() < 1e-4,
                    "suu mismatch at ({u},{v}): {:?} vs {fd_suu:?}",
                    jet.suu
                );
                assert!(
                    (jet.suv - fd_suv).norm() < 1e-4,
                    "suv mismatch at ({u},{v}): {:?} vs {fd_suv:?}",
                    jet.suv
                );
                assert!(
                    (jet.svv - fd_svv).norm() < 1e-4,
                    "svv mismatch at ({u},{v}): {:?} vs {fd_svv:?}",
                    jet.svv
                );
            }
        }
    }

    #[test]
    fn plane_projection_matches_closed_form() {
        let plane = Surface3::plane(Point3::new(0.0, 0.0, 2.0), Vector3::z()).unwrap();
        let p = Point3::new(3.0, -4.0, 7.0);
        let proj = plane.project_point(&p);
        check_surface_projection(&plane, &p, &proj);
        assert_point_near(&proj.point, &Point3::new(3.0, -4.0, 2.0), "foot");
        assert_near(proj.distance, 5.0, "distance");
        assert_near(proj.u, 3.0, "u");
        assert_near(proj.v, -4.0, "v");
    }

    #[test]
    fn cylinder_projection_matches_closed_form() {
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).unwrap();
        let p = Point3::new(3.0, 4.0, 5.0);
        let proj = cylinder.project_point(&p);
        check_surface_projection(&cylinder, &p, &proj);
        // Radial distance 5: closest point at radius 2 along (3,4)/5.
        assert_point_near(&proj.point, &Point3::new(1.2, 1.6, 5.0), "foot");
        assert_near(proj.distance, 3.0, "distance");
        assert_near(proj.u, 4.0f64.atan2(3.0), "u");
        assert_near(proj.v, 5.0, "v");
    }

    #[test]
    fn cylinder_axis_point_is_ambiguous_but_converges() {
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).unwrap();
        let p = Point3::new(0.0, 0.0, -3.0);
        let proj = cylinder.project_point(&p);
        check_surface_projection(&cylinder, &p, &proj);
        assert_near(proj.distance, 2.0, "distance");
        assert_near(proj.v, -3.0, "v");
    }

    #[test]
    fn cone_projection_matches_closed_form() {
        // half_angle π/4 (tan = 1), radius 1 at v = 0, apex at (0,0,-1).
        let cone = Surface3::cone(Point3::origin(), Vector3::z(), FRAC_PI_4, 1.0).unwrap();
        let p = Point3::new(2.0, 0.0, 0.5);
        let proj = cone.project_point(&p);
        check_surface_projection(&cone, &p, &proj);
        // Foot of the perpendicular onto the ruling through (1,0,0) with
        // direction (1,0,1).
        assert_point_near(&proj.point, &Point3::new(1.75, 0.0, 0.75), "foot");
        assert_near(proj.distance, 0.125f64.sqrt(), "distance");
        assert_near(proj.u, 0.0, "u");
        assert_near(proj.v, 0.75, "v");
    }

    #[test]
    fn cone_apex_point_projects_to_apex() {
        let cone = Surface3::cone(Point3::origin(), Vector3::z(), FRAC_PI_4, 1.0).unwrap();
        let p = Point3::new(0.0, 0.0, -1.0);
        let proj = cone.project_point(&p);
        assert!(proj.converged);
        assert_point_near(&proj.point, &p, "apex is on the surface");
        assert!(proj.distance < EPS, "distance {}", proj.distance);
    }

    #[test]
    fn cone_second_nappe_projection() {
        // (-2,0,-2) is closest to the mirror nappe, reached along the u = 0
        // ruling past the apex (rho < 0), not the u = π ruling the angular
        // seed alone would suggest.
        let cone = Surface3::cone(Point3::origin(), Vector3::z(), FRAC_PI_4, 1.0).unwrap();
        let p = Point3::new(-2.0, 0.0, -2.0);
        let proj = cone.project_point(&p);
        check_surface_projection(&cone, &p, &proj);
        assert_point_near(&proj.point, &Point3::new(-1.5, 0.0, -2.5), "foot");
        assert_near(proj.distance, 0.5f64.sqrt(), "distance");
        assert_near(proj.u, 0.0, "u");
        assert_near(proj.v, -2.5, "v");
    }

    #[test]
    fn sphere_projection_matches_closed_form() {
        let sphere = Surface3::sphere(Point3::new(0.0, 0.0, 1.0), Vector3::z(), 2.0).unwrap();
        let p = Point3::new(0.0, 3.0, 1.0);
        let proj = sphere.project_point(&p);
        check_surface_projection(&sphere, &p, &proj);
        assert_point_near(&proj.point, &Point3::new(0.0, 2.0, 1.0), "foot");
        assert_near(proj.distance, 1.0, "distance");
        assert_near(proj.u, FRAC_PI_2, "u");
        assert_near(proj.v, 0.0, "v");
    }

    #[test]
    fn sphere_pole_points_project_to_poles() {
        // Points on the pole axis: the parameterization is singular at the
        // poles but the projection is still well-defined.
        let sphere = Surface3::sphere(Point3::new(0.0, 0.0, 1.0), Vector3::z(), 2.0).unwrap();

        let north = Point3::new(0.0, 0.0, 4.0);
        let proj = sphere.project_point(&north);
        assert!(proj.converged, "north pole projection did not converge");
        assert_point_near(&proj.point, &Point3::new(0.0, 0.0, 3.0), "north foot");
        assert_near(proj.distance, 1.0, "north distance");
        assert_near(proj.v, FRAC_PI_2, "north v");

        let south = Point3::new(0.0, 0.0, -3.0);
        let proj = sphere.project_point(&south);
        assert!(proj.converged, "south pole projection did not converge");
        assert_point_near(&proj.point, &Point3::new(0.0, 0.0, -1.0), "south foot");
        assert_near(proj.distance, 2.0, "south distance");
        assert_near(proj.v, -FRAC_PI_2, "south v");
    }

    #[test]
    fn sphere_near_pole_off_axis_projection() {
        let sphere = Surface3::sphere(Point3::origin(), Vector3::z(), 2.0).unwrap();
        // Slightly off the pole axis: the closest point is just off the
        // pole, where the u-partial nearly vanishes.
        let p = Point3::new(1e-3, 0.0, 4.0);
        let proj = sphere.project_point(&p);
        check_surface_projection(&sphere, &p, &proj);
        let d = p - Point3::origin();
        assert_near(proj.distance, d.norm() - 2.0, "distance");
        assert_point_near(
            &proj.point,
            &(Point3::origin() + d / d.norm() * 2.0),
            "foot on the center-to-point ray",
        );
    }

    #[test]
    fn sphere_center_is_ambiguous_but_converges() {
        let center = Point3::new(1.0, 2.0, 3.0);
        let sphere = Surface3::sphere(center, Vector3::z(), 2.0).unwrap();
        let proj = sphere.project_point(&center);
        check_surface_projection(&sphere, &center, &proj);
        assert_near(proj.distance, 2.0, "distance");
    }

    #[test]
    fn torus_projection_matches_closed_form() {
        let torus = Surface3::torus(Point3::origin(), Vector3::z(), 3.0, 1.0).unwrap();
        // Outside the outer equator.
        let p = Point3::new(5.0, 0.0, 0.0);
        let proj = torus.project_point(&p);
        check_surface_projection(&torus, &p, &proj);
        assert_point_near(&proj.point, &Point3::new(4.0, 0.0, 0.0), "outer foot");
        assert_near(proj.distance, 1.0, "outer distance");
        // Above the tube center: closest point on top of the tube.
        let p = Point3::new(3.0, 0.0, 2.0);
        let proj = torus.project_point(&p);
        check_surface_projection(&torus, &p, &proj);
        assert_point_near(&proj.point, &Point3::new(3.0, 0.0, 1.0), "top foot");
        assert_near(proj.distance, 1.0, "top distance");
        assert_near(proj.v, FRAC_PI_2, "top v");
    }

    #[test]
    fn torus_axis_point_is_ambiguous_but_converges() {
        let torus = Surface3::torus(Point3::origin(), Vector3::z(), 3.0, 1.0).unwrap();
        let p = Point3::new(0.0, 0.0, 0.5);
        let proj = torus.project_point(&p);
        check_surface_projection(&torus, &p, &proj);
        // Distance from an axis point to the tube: |axis point to tube
        // center circle| - minor radius.
        assert_near(proj.distance, (9.0f64 + 0.25).sqrt() - 1.0, "distance");
    }

    #[test]
    fn tilted_offset_surfaces_project_consistently() {
        // Off-axis frames: verify stationarity and global optimality by
        // dense sampling over the periodic directions.
        let cylinder = Surface3::cylinder(
            Point3::new(1.0, -2.0, 0.5),
            Vector3::new(1.0, 2.0, -1.0),
            1.5,
        )
        .unwrap();
        let torus = Surface3::torus(
            Point3::new(-1.0, 0.0, 2.0),
            Vector3::new(2.0, -1.0, 1.0),
            4.0,
            1.2,
        )
        .unwrap();
        let p = Point3::new(3.0, 1.0, -2.0);

        let proj = cylinder.project_point(&p);
        check_surface_projection(&cylinder, &p, &proj);
        let sampled = sampled_surface_min(&cylinder, &p, (0.0, crate::curve::TWO_PI), (-8.0, 8.0));
        assert!(
            proj.distance <= sampled + 1e-4,
            "cylinder: {} vs sampled {sampled}",
            proj.distance
        );

        let proj = torus.project_point(&p);
        check_surface_projection(&torus, &p, &proj);
        let sampled = sampled_surface_min(
            &torus,
            &p,
            (0.0, crate::curve::TWO_PI),
            (0.0, crate::curve::TWO_PI),
        );
        assert!(
            proj.distance <= sampled + 1e-4,
            "torus: {} vs sampled {sampled}",
            proj.distance
        );
    }

    // --- Surfaces: NURBS ---

    /// Exact NURBS cylinder of radius 1 about the z-axis, `v ∈ [0, 1]`
    /// mapping to `z ∈ [0, 2]` (same fixture as the NURBS surface tests).
    fn nurbs_cylinder_patch() -> NurbsSurface {
        let ring: Vec<(f64, f64)> = vec![
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (-1.0, 1.0),
            (-1.0, 0.0),
            (-1.0, -1.0),
            (0.0, -1.0),
            (1.0, -1.0),
            (1.0, 0.0),
        ];
        let control_points: Vec<Vec<Point3>> = ring
            .iter()
            .map(|&(x, y)| vec![Point3::new(x, y, 0.0), Point3::new(x, y, 2.0)])
            .collect();
        let s = FRAC_1_SQRT_2;
        let weights: Vec<Vec<f64>> = [1.0, s, 1.0, s, 1.0, s, 1.0, s, 1.0]
            .iter()
            .map(|&w| vec![w, w])
            .collect();
        let knots_u = KnotVector::new(
            2,
            vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ],
        )
        .unwrap();
        let knots_v = KnotVector::clamped_uniform(1, 2).unwrap();
        NurbsSurface::new(control_points, weights, knots_u, knots_v).unwrap()
    }

    /// Bilinear non-planar quad: S(u,v) = (2u, v, v(1 + 2u)).
    fn bilinear_patch() -> NurbsSurface {
        let control_points = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 1.0)],
            vec![Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 1.0, 3.0)],
        ];
        let knots = KnotVector::clamped_uniform(1, 2).unwrap();
        NurbsSurface::bspline(control_points, knots.clone(), knots).unwrap()
    }

    #[test]
    fn nurbs_cylinder_patch_matches_analytic_cylinder() {
        let patch = nurbs_cylinder_patch();
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 1.0).unwrap();
        // z inside [0, 2] so the axial clamp is inactive and the geometric
        // answers must coincide.
        for p in [
            Point3::new(2.0, 1.0, 1.0),
            Point3::new(-0.5, 1.5, 0.3),
            Point3::new(0.2, -3.0, 1.8),
        ] {
            let from_patch = patch.project_point(&p);
            let from_analytic = cylinder.project_point(&p);
            check_surface_projection(&patch, &p, &from_patch);
            assert_point_near(&from_patch.point, &from_analytic.point, "foot");
            assert_near(from_patch.distance, from_analytic.distance, "distance");
        }
    }

    #[test]
    fn nurbs_patch_clamps_to_boundary() {
        let patch = bilinear_patch();
        // Beyond the u = 1, v = 0 corner: closest point is the corner.
        let p = Point3::new(3.0, -0.5, 0.0);
        let proj = patch.project_point(&p);
        assert!(proj.converged);
        assert_near(proj.u, 1.0, "u pinned");
        assert_near(proj.v, 0.0, "v pinned");
        assert_point_near(&proj.point, &Point3::new(2.0, 0.0, 0.0), "corner");
        assert_near(proj.distance, 1.25f64.sqrt(), "distance");
    }

    #[test]
    fn nurbs_patch_interior_normal_offset_projects_back() {
        let patch = bilinear_patch();
        let (u, v) = (0.3, 0.4);
        let base = patch.point(u, v);
        let normal = patch.normal(u, v).expect("regular interior point");
        let p = base + normal * 0.2;
        let proj = patch.project_point(&p);
        check_surface_projection(&patch, &p, &proj);
        assert_near(proj.distance, 0.2, "distance equals the offset");
        assert!(
            (proj.point - base).norm() < 1e-6,
            "foot {:?} drifted from base {base:?}",
            proj.point
        );
        assert!((proj.u - u).abs() < 1e-6, "u {} vs {u}", proj.u);
        assert!((proj.v - v).abs() < 1e-6, "v {} vs {v}", proj.v);
    }

    #[test]
    fn nurbs_patch_projection_is_globally_optimal() {
        let patch = bilinear_patch();
        for p in [
            Point3::new(1.0, 0.5, 2.0),
            Point3::new(-1.0, 0.5, 0.5),
            Point3::new(2.5, 1.5, 4.0),
        ] {
            let proj = patch.project_point(&p);
            assert!(proj.converged, "did not converge for {p:?}");
            let sampled = sampled_surface_min(&patch, &p, (0.0, 1.0), (0.0, 1.0));
            assert!(
                proj.distance <= sampled + 1e-4,
                "not optimal for {p:?}: {} vs sampled {sampled}",
                proj.distance
            );
        }
    }

    #[test]
    fn span_samples_cover_domain() {
        let knots = KnotVector::new(
            2,
            vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ],
        )
        .unwrap();
        let samples = span_samples(&knots, 4);
        assert_eq!(samples.first(), Some(&0.0));
        assert_eq!(samples.last(), Some(&1.0));
        // Four non-empty spans × 4 samples + the domain end.
        assert_eq!(samples.len(), 4 * 4 + 1);
        assert!(samples.windows(2).all(|w| w[1] > w[0]), "not increasing");
    }

    #[test]
    fn param_bounds_wrap_and_clamp() {
        let periodic = ParamBounds {
            lo: 0.0,
            hi: crate::curve::TWO_PI,
            period: Some(crate::curve::TWO_PI),
        };
        assert_near(periodic.restrict(-0.3), crate::curve::TWO_PI - 0.3, "wrap");
        assert_near(
            periodic.restrict(crate::curve::TWO_PI + 1.0),
            1.0,
            "wrap forward",
        );
        let clamped = ParamBounds {
            lo: 0.0,
            hi: 1.0,
            period: None,
        };
        assert_near(clamped.restrict(-2.0), 0.0, "clamp low");
        assert_near(clamped.restrict(PI), 1.0, "clamp high");
        assert_near(clamped.restrict(0.4), 0.4, "interior untouched");
    }

    /// Polyline projection scans every segment for the exact per-segment
    /// minimum (no Newton over the kinked parameterization).
    #[test]
    fn projects_onto_polyline() {
        // L-shape: x-run then y-run.
        let c = Curve3::polyline(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(2.0, 2.0, 0.0),
            ],
            false,
        )
        .unwrap();
        // Beside the middle of the second segment.
        let p = Point3::new(3.0, 1.0, 0.0);
        let proj = c.project_point(&p);
        check_curve_projection(&c, &p, &proj);
        assert_near(proj.t, 1.5, "second-segment parameter");
        assert_near(proj.distance, 1.0, "distance to the segment");
        // Nearest to the corner vertex: both segments tie there.
        let proj = c.project_point(&Point3::new(3.0, -1.0, 0.0));
        assert_near(proj.t, 1.0, "corner vertex parameter");
        // Beyond the start: clamps to the first vertex.
        let proj = c.project_point(&Point3::new(-1.0, -1.0, 0.0));
        assert_near(proj.t, 0.0, "clamped start parameter");
        assert_point_near(&proj.point, &Point3::origin(), "clamped start point");
    }
}
