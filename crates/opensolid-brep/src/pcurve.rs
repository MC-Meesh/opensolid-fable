//! 2D parameter-space curves ("pcurves"): the trim geometry a [`Fin`] uses
//! to bound its face in the owning surface's `(u, v)` space.
//!
//! # Why a face needs 2D trim geometry
//!
//! A face is a region of its surface, and the region is delimited in the
//! surface's *parameter* space, not in model space. Recovering that region
//! from 3D edge curves alone is a projection problem with no single answer
//! wherever the surface parameterization folds back on itself: on a closed
//! surface, the seam meridian of a cylinder is `u = 0` and `u = 2π` at once,
//! and the two fins of the seam edge must take *different* branches or the
//! face's boundary is not a closed cycle in parameter space. Storing the
//! pcurve per fin is what makes that distinction representable.
//!
//! # The parameterization invariant
//!
//! > A fin's pcurve is parameterized over the **same interval, with the same
//! > parameter**, as its edge's 3D curve: for every `t` in
//! > `[edge.t_start, edge.t_end]`, `surface.point(pcurve(t)) == curve.point(t)`.
//!
//! Every constructor and every consumer in this module holds to that. It is
//! what lets a caller walk an edge and its pcurve in lockstep without a
//! search, and it is why [`Curve2::Polyline`] carries an explicit parameter
//! per vertex rather than being indexed by vertex number the way
//! [`Curve3::Polyline`](crate::curve::Curve3::Polyline) is.
//!
//! The invariant is also why an imported STEP pcurve cannot be copied across
//! verbatim: STEP parameterizes a `PCURVE`'s definitional representation in
//! the *basis curve's* parameter, and the reader re-parameterizes every curve
//! it maps into the kernel convention (arc length for lines, angle for
//! conics), so the two do not line up. The reader derives pcurves with
//! [`attach_body_pcurves`] instead, taking the one thing the authored 2D
//! geometry knows and projection does not — which edges are seams — from the
//! topology, where a seam shows up as an edge its face uses twice.
//!
//! # No NURBS variant yet
//!
//! [`Curve2`] has no freeform variant. Since of-3qy.8 retired the B-spline
//! mesh fallback, freeform faces *do* reach the exact path, so a trim curve
//! that is genuinely a 2D NURBS is now reachable — it just has nowhere exact
//! to go. [`fit_pcurve`] fits what it can as a line or a circle and
//! otherwise falls back to [`Curve2::Polyline`], whose error is bounded by
//! the sample spacing rather than exact.
//!
//! That is a real (bounded) loss of fidelity on freeform trim, not a
//! representational gap that costs nothing, and it is tracked as of-50u
//! along with the 2D rational B-spline emission the writer needs for the
//! same reason. It is *not* a correctness problem: a polyline pcurve
//! describes a curve that really does lie on the surface, and no consumer is
//! told it is exact.
//!
//! [`Fin`]: crate::topology::Fin

use crate::curve::{Curve3, CurveEval, TWO_PI};
use crate::project::SurfaceProject;
use crate::surface::{Surface3, SurfaceEval};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::types::{Point2, Point3, Vector2};

/// Number of curve samples [`fit_pcurve`] projects into parameter space.
/// One more than a power of two so the samples include both endpoints and
/// the midpoint exactly.
const FIT_SAMPLES: usize = 33;

/// Relative tolerance for accepting an analytic fit in [`fit_pcurve`],
/// measured in parameter space against the sampled span's own extent.
const FIT_TOL_REL: f64 = 1e-9;

/// How nearly constant a periodic direction must be for a curve to count as
/// running *along* that direction's branch cut (see [`shift_to_high_branch`]).
///
/// Deliberately far looser than [`FIT_TOL_REL`]: this is a yes/no question
/// about which direction a seam runs, not a fit, and projection near a
/// parameterization singularity — a sphere's poles, where a meridian's `u`
/// is barely determined — wobbles well past a fit tolerance. Nothing real
/// sits in the gap, because no edge sweeps only a millionth of a period.
const SEAM_CONSTANT_TOL_REL: f64 = 1e-6;

/// Evaluation interface for parameter-space curves, mirroring
/// [`CurveEval`](crate::curve::CurveEval) in 2D.
pub trait Curve2Eval {
    /// Parameter-space position at curve parameter `t`.
    fn point(&self, t: f64) -> Point2;

    /// First derivative with respect to `t`.
    fn derivative(&self, t: f64) -> Vector2;

    /// Parameter interval `(t_min, t_max)`. Unbounded curves return infinite
    /// endpoints.
    fn domain(&self) -> (f64, f64);
}

/// A curve in a surface's `(u, v)` parameter space.
///
/// Parameterization conventions:
/// - `Line`: affine, `point(t) = origin + dir * t`. `dir` is **not**
///   normalized. Unlike [`Curve3::Line`] there is no arc-length convention
///   to normalize to: `u` and `v` are not commensurable units (a step in a
///   cylinder's `u` is radians, in its `v` is millimetres), so
///   parameter-space length is not a meaningful quantity. Carrying the rate
///   in `dir` is also what lets the module invariant hold for pairings that
///   move through `(u, v)` at other than unit rate — a cone's generators,
///   say. Domain is unbounded.
/// - `Circle`: parameterized by angle in radians at unit rate, measured from
///   `x_dir` and turning towards `ccw` (a pcurve traverses either way,
///   unlike [`Curve3::Circle`], because the surface's own `(u, v)`
///   handedness may reverse it). Evaluation is periodic, so an edge range
///   that runs past `2π` evaluates correctly.
/// - `Polyline`: parameterized by the explicit, strictly increasing `params`
///   attached to its vertices, so it can carry the edge's own parameter
///   range (see the module invariant).
#[derive(Debug, Clone, PartialEq)]
pub enum Curve2 {
    /// Infinite line through `origin` at parameter-space velocity `dir`.
    Line { origin: Point2, dir: Vector2 },
    /// Circle of `radius` about `center`, starting along unit `x_dir` at
    /// `t = 0` and turning counterclockwise when `ccw`, clockwise otherwise.
    Circle {
        center: Point2,
        radius: f64,
        x_dir: Vector2,
        ccw: bool,
    },
    /// Piecewise-linear curve through `points`, with `params[i]` the
    /// parameter at `points[i]`. `params` is strictly increasing and the two
    /// vectors are the same length (at least 2). Evaluation outside
    /// `[params[0], params[last]]` clamps.
    Polyline {
        params: Vec<f64>,
        points: Vec<Point2>,
    },
}

impl Curve2 {
    /// Line through `origin` advancing at `dir` per unit parameter. `dir` is
    /// kept as given — see the variant's documentation for why parameter
    /// space has no arc length to normalize to.
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `dir` has zero or non-finite length, or
    /// if `origin` is not finite.
    pub fn line(origin: Point2, dir: Vector2) -> CoreResult<Self> {
        let norm = dir.norm();
        if norm == 0.0 || !norm.is_finite() || !origin.coords.iter().all(|c| c.is_finite()) {
            return Err(CoreError::Degenerate {
                context: "Curve2::line",
                reason: format!(
                    "need a finite origin and non-zero finite rate, got {origin}/{dir}"
                ),
            });
        }
        Ok(Curve2::Line { origin, dir })
    }

    /// Circle of `radius` about `center`, with `t = 0` along `x_dir`
    /// (normalized here) and turning counterclockwise when `ccw`.
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `x_dir` has zero or non-finite length;
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    pub fn circle(center: Point2, radius: f64, x_dir: Vector2, ccw: bool) -> CoreResult<Self> {
        let norm = x_dir.norm();
        if norm == 0.0 || !norm.is_finite() {
            return Err(CoreError::Degenerate {
                context: "Curve2::circle",
                reason: format!(
                    "reference direction must have non-zero finite length, got {x_dir}"
                ),
            });
        }
        if radius <= 0.0 || !radius.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "radius",
                reason: format!("must be positive and finite, got {radius}"),
            });
        }
        Ok(Curve2::Circle {
            center,
            radius,
            x_dir: x_dir / norm,
            ccw,
        })
    }

    /// Piecewise-linear pcurve through `points` at `params`.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if fewer than two vertices are given,
    /// if the two vectors differ in length, or if `params` is not strictly
    /// increasing and finite.
    pub fn polyline(params: Vec<f64>, points: Vec<Point2>) -> CoreResult<Self> {
        if points.len() < 2 {
            return Err(CoreError::InvalidArgument {
                argument: "points",
                reason: format!("a polyline needs at least 2 vertices, got {}", points.len()),
            });
        }
        if params.len() != points.len() {
            return Err(CoreError::InvalidArgument {
                argument: "params",
                reason: format!(
                    "one parameter per vertex required, got {} for {} vertices",
                    params.len(),
                    points.len()
                ),
            });
        }
        // NaN-safe: only a strictly increasing step passes the comparison.
        if !params[0].is_finite()
            || params
                .windows(2)
                .any(|w| w[1].partial_cmp(&w[0]) != Some(std::cmp::Ordering::Greater))
        {
            return Err(CoreError::InvalidArgument {
                argument: "params",
                reason: "must be finite and strictly increasing".to_string(),
            });
        }
        Ok(Curve2::Polyline { params, points })
    }
}

impl Curve2Eval for Curve2 {
    fn point(&self, t: f64) -> Point2 {
        match self {
            Curve2::Line { origin, dir } => origin + dir * t,
            Curve2::Circle {
                center,
                radius,
                x_dir,
                ccw,
            } => {
                let y_dir = perp(x_dir, *ccw);
                center + (x_dir * t.cos() + y_dir * t.sin()) * *radius
            }
            Curve2::Polyline { params, points } => {
                let (i, frac) = polyline_segment(params, t);
                points[i] + (points[i + 1] - points[i]) * frac
            }
        }
    }

    fn derivative(&self, t: f64) -> Vector2 {
        match self {
            Curve2::Line { dir, .. } => *dir,
            Curve2::Circle {
                radius, x_dir, ccw, ..
            } => {
                let y_dir = perp(x_dir, *ccw);
                (y_dir * t.cos() - x_dir * t.sin()) * *radius
            }
            Curve2::Polyline { params, points } => {
                let (i, _) = polyline_segment(params, t);
                (points[i + 1] - points[i]) / (params[i + 1] - params[i])
            }
        }
    }

    fn domain(&self) -> (f64, f64) {
        match self {
            Curve2::Line { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            Curve2::Circle { .. } => (0.0, TWO_PI),
            Curve2::Polyline { params, .. } => (params[0], params[params.len() - 1]),
        }
    }
}

/// The direction a quarter turn from `x_dir`, counterclockwise or not.
fn perp(x_dir: &Vector2, ccw: bool) -> Vector2 {
    if ccw {
        Vector2::new(-x_dir.y, x_dir.x)
    } else {
        Vector2::new(x_dir.y, -x_dir.x)
    }
}

/// Segment index and interpolation fraction for `t` on a polyline with
/// `params` knots. Clamps outside the domain.
fn polyline_segment(params: &[f64], t: f64) -> (usize, f64) {
    let last = params.len() - 1;
    // NaN-safe: only a parameter that compares strictly greater advances, so
    // a NaN lands on the first segment's start.
    if t.partial_cmp(&params[0]) != Some(std::cmp::Ordering::Greater) {
        return (0, 0.0);
    }
    if t >= params[last] {
        return (last - 1, 1.0);
    }
    // `t` is strictly inside, so `partition_point` returns 1..=last.
    let i = params.partition_point(|&p| p <= t) - 1;
    (i, (t - params[i]) / (params[i + 1] - params[i]))
}

// ---------------------------------------------------------------------
// Recomputing a pcurve from 3D geometry
// ---------------------------------------------------------------------

/// Which branch of a periodic parameter direction a seam edge's pcurve takes.
///
/// A seam edge is used *twice by the same face*: it is where a closed
/// surface's parameterization is cut open, so the face's boundary meets it
/// coming and going. Its two fins must take representatives a full period
/// apart, or the boundary is not a closed cycle in parameter space and the
/// enclosed region is empty. Closest-point projection returns one
/// representative and cannot know which fin wants which, so the caller says.
///
/// The branch is expressed as a shift, not as an absolute parameter value,
/// which keeps it independent of where a surface's parameterization happens
/// to place its own origin.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum SeamSide {
    /// The pcurve as projected.
    #[default]
    Low,
    /// The same pcurve shifted a full period along whichever periodic
    /// direction the seam runs constant in — `u` for a cylinder, cone or
    /// sphere generator, either family on a torus.
    High,
}

/// Recompute the pcurve of `curve`, trimmed to `[t_start, t_end]`, on
/// `surface`.
///
/// The curve is sampled, each sample inverted onto the surface by
/// closest-point projection, and the resulting parameter-space samples
/// unwrapped across periodic branch cuts (each sample takes the
/// representative nearest its predecessor, so a curve that crosses the seam
/// runs continuously past it rather than jumping a full period). The samples
/// are then fitted, in order of preference, as a [`Curve2::Line`], a
/// [`Curve2::Circle`], or a [`Curve2::Polyline`] through the samples
/// themselves.
///
/// The first two cover essentially every analytic pairing an exact import
/// produces — a line or circle on a plane, a cylinder's generators and
/// cross-sections, a sphere's meridians and latitudes, a torus's two circle
/// families, and every seam — exactly, not to a tolerance. The polyline
/// fallback bounds the error of everything else by the sample spacing.
///
/// `seam` picks the branch for a seam edge, where projection returns one of
/// two equally valid representatives a period apart. It has no effect on a
/// curve that runs across a periodic direction rather than along it, because
/// such a curve has only one representative.
///
/// # Errors
/// [`CoreError::Degenerate`] if `[t_start, t_end]` is not a finite,
/// increasing interval, or if the whole curve collapses to a single point in
/// parameter space (a pcurve needs extent to bound anything).
pub fn fit_pcurve(
    surface: &Surface3,
    curve: &Curve3,
    t_start: f64,
    t_end: f64,
    seam: SeamSide,
) -> CoreResult<Curve2> {
    // NaN-safe: only a strictly increasing finite range passes.
    if !t_start.is_finite() || t_end.partial_cmp(&t_start) != Some(std::cmp::Ordering::Greater) {
        return Err(CoreError::Degenerate {
            context: "fit_pcurve",
            reason: format!("parameter range [{t_start}, {t_end}] must be finite and increasing"),
        });
    }
    let (params, points) = sample_parameter_space(surface, curve, t_start, t_end, seam);

    let span = points
        .iter()
        .fold(0.0f64, |acc, p| acc.max((p - points[0]).norm()));
    if span <= f64::EPSILON {
        return Err(CoreError::Degenerate {
            context: "fit_pcurve",
            reason: "curve collapses to a single point in parameter space".to_string(),
        });
    }
    let tol = FIT_TOL_REL * span;

    if let Some(line) = fit_line(&params, &points, tol) {
        return Ok(line);
    }
    if let Some(circle) = fit_circle(&params, &points, tol) {
        return Ok(circle);
    }
    Curve2::polyline(params, points)
}

/// Sample `curve` over `[t_start, t_end]` and invert each sample onto
/// `surface`, unwrapping periodic directions so the result is continuous.
fn sample_parameter_space(
    surface: &Surface3,
    curve: &Curve3,
    t_start: f64,
    t_end: f64,
    seam: SeamSide,
) -> (Vec<f64>, Vec<Point2>) {
    let (period_u, period_v) = (surface.period_u(), surface.period_v());
    let mut params = Vec::with_capacity(FIT_SAMPLES);
    let mut points: Vec<Point2> = Vec::with_capacity(FIT_SAMPLES);
    let mut seed: Option<(f64, f64)> = None;

    for i in 0..FIT_SAMPLES {
        let t = t_start + (t_end - t_start) * (i as f64) / ((FIT_SAMPLES - 1) as f64);
        let p: Point3 = curve.point(t);
        let projection = match seed {
            Some(seed) => surface.project_point_seeded(&p, seed),
            None => surface.project_point(&p),
        };
        let mut uv = Point2::new(projection.u, projection.v);
        // Continue from the previous sample's branch, so a curve crossing a
        // branch cut runs past it instead of jumping back a whole period.
        if let Some(previous) = points.last() {
            uv.x = nearest_representative(uv.x, previous.x, period_u);
            uv.y = nearest_representative(uv.y, previous.y, period_v);
        }
        seed = Some((uv.x, uv.y));
        params.push(t);
        points.push(uv);
    }
    repair_singular_samples(surface, &mut points);
    if seam == SeamSide::High {
        shift_to_high_branch(&mut points, period_u, period_v);
    }
    (params, points)
}

/// Replace the `u` of samples that land on a parameterization singularity
/// with the `u` of the nearest sample that does not.
///
/// A singularity collapses a whole `u` line to a single point — a sphere's
/// poles, a cone's apex — so *every* `u` maps there and projection has no
/// grounds to prefer one. Left alone, the arbitrary value it returns is a
/// wild point that turns an otherwise exact fit into a polyline: a sphere's
/// seam meridian runs from pole to pole, so two of its samples are singular
/// and the other thirty-one are collinear.
///
/// Taking `u` from the neighbours is not a fudge — it is the answer the
/// pcurve needs. A boundary reaching the pole arrives along some `u`, and
/// which one is decided by the curve on the way in, not by the pole.
fn repair_singular_samples(surface: &Surface3, points: &mut [Point2]) {
    let singular: Vec<bool> = points
        .iter()
        .map(|p| surface.is_singular(p.x, p.y))
        .collect();
    if !singular.iter().any(|&s| s) || singular.iter().all(|&s| s) {
        return;
    }
    // Nearest non-singular neighbour, scanning in from each side; the first
    // pass to reach a sample wins, so a run of singular samples takes the
    // `u` of whichever end has a real one closer.
    let mut fixed = singular.iter().map(|&s| !s).collect::<Vec<bool>>();
    for pass in 0..2 {
        let mut carry: Option<f64> = None;
        for i in 0..points.len() {
            let i = if pass == 0 { i } else { points.len() - 1 - i };
            if fixed[i] {
                carry = Some(points[i].x);
            } else if let Some(u) = carry {
                points[i].x = u;
                fixed[i] = true;
            }
        }
    }
}

/// The representative of `value` (modulo `period`) nearest `reference`.
/// Aperiodic directions pass through untouched.
fn nearest_representative(value: f64, reference: f64, period: Option<f64>) -> f64 {
    let Some(period) = period else { return value };
    value + period * ((reference - value) / period).round()
}

/// Move sampled parameters onto the far branch of the periodic direction the
/// samples run *constant* in — the direction whose cut the curve lies along,
/// which is the only one with a second representative to move to.
///
/// A curve that varies in every periodic direction is not a seam and is left
/// where it is.
fn shift_to_high_branch(points: &mut [Point2], period_u: Option<f64>, period_v: Option<f64>) {
    let constant = |extent: f64, period: f64| extent <= SEAM_CONSTANT_TOL_REL * period;
    let extent = |values: &[Point2], component: fn(&Point2) -> f64| {
        let first = component(&values[0]);
        values
            .iter()
            .fold(0.0f64, |acc, p| acc.max((component(p) - first).abs()))
    };

    if let Some(period) = period_u
        && constant(extent(points, |p| p.x), period)
    {
        for p in points.iter_mut() {
            p.x += period;
        }
        return;
    }
    if let Some(period) = period_v
        && constant(extent(points, |p| p.y), period)
    {
        for p in points.iter_mut() {
            p.y += period;
        }
    }
}

/// Fit the samples as a line affine in the curve parameter, or `None` if any
/// sample is further than `tol` from it.
fn fit_line(params: &[f64], points: &[Point2], tol: f64) -> Option<Curve2> {
    let last = points.len() - 1;
    let span = params[last] - params[0];
    let slope = (points[last] - points[0]) / span;
    let norm = slope.norm();
    if norm == 0.0 || !norm.is_finite() {
        return None;
    }
    for (&t, p) in params.iter().zip(points) {
        if (p - (points[0] + slope * (t - params[0]))).norm() > tol {
            return None;
        }
    }
    // `origin` is the position at t = 0, so the line evaluates in the
    // curve's own parameter — `point(t) = origin + slope * t` — which is
    // exactly the module invariant.
    let origin = points[0] - slope * params[0];
    Curve2::line(origin, slope).ok()
}

/// Fit the samples as a circle whose angle is affine in the curve parameter,
/// or `None` if any sample is further than `tol` from it.
///
/// Only a unit-rate fit is accepted (angle equal to the curve parameter up to
/// an offset), because [`Curve2::Circle`] has no rate to store — anything
/// else falls through to the polyline, which can represent it exactly.
fn fit_circle(params: &[f64], points: &[Point2], tol: f64) -> Option<Curve2> {
    let last = points.len() - 1;
    // Thirds, not endpoints: a closed edge's first and last samples are the
    // same point, which would leave the circumcenter underdetermined.
    let center = circle_center(points[0], points[last / 3], points[2 * last / 3], tol)?;
    let radius = (points[0] - center).norm();
    if radius <= tol {
        return None;
    }
    // Handedness from the signed area swept over the first sample step,
    // which is short enough that its sign cannot alias past half a turn.
    let a = points[0] - center;
    let b = points[1] - center;
    let ccw = a.x * b.y - a.y * b.x > 0.0;
    // `x_dir` is the position at t = 0, rotated back from the first sample.
    let x_dir = rotate(a / radius, -params[0], ccw);
    let circle = Curve2::circle(center, radius, x_dir, ccw).ok()?;
    for (&t, p) in params.iter().zip(points) {
        if (p - circle.point(t)).norm() > tol {
            return None;
        }
    }
    Some(circle)
}

/// Circumcenter of three parameter-space points, or `None` when they are
/// collinear within `tol`.
fn circle_center(a: Point2, b: Point2, c: Point2, tol: f64) -> Option<Point2> {
    let (ab, ac) = (b - a, c - a);
    let cross = ab.x * ac.y - ab.y * ac.x;
    // Scale-aware: `cross` is twice the triangle area, so compare it with
    // the tolerance times the longest side rather than with `tol` itself.
    let scale = ab.norm().max(ac.norm());
    if cross.abs() <= tol * scale {
        return None;
    }
    let (ab2, ac2) = (ab.norm_squared(), ac.norm_squared());
    let center = a + Vector2::new(ac.y * ab2 - ab.y * ac2, ab.x * ac2 - ac.x * ab2) / (2.0 * cross);
    center
        .coords
        .iter()
        .all(|c| c.is_finite())
        .then_some(center)
}

/// Rotate a parameter-space vector by `angle`, in the sense given by `ccw`.
fn rotate(v: Vector2, angle: f64, ccw: bool) -> Vector2 {
    let signed = if ccw { angle } else { -angle };
    let (sin, cos) = signed.sin_cos();
    Vector2::new(v.x * cos - v.y * sin, v.x * sin + v.y * cos)
}

/// Derive and attach trim geometry to every fin of `body`, replacing
/// whatever each fin carried before.
///
/// Returns the number of fins that got a pcurve. Fins that cannot carry one
/// — no surface on the face, no curve on the edge, geometry that collapses
/// in parameter space — are left with `None` rather than failing the whole
/// body: a pcurve is derived data, and a body is not wrong for having a
/// degenerate edge somewhere.
///
/// Seams are read from the topology: an edge a single face uses twice is
/// where that surface's parameterization is cut open, so the two uses are
/// put on opposite branches, taken in loop-traversal order.
///
/// Run this *after* any repair that rewires fins onto different edges — a
/// pcurve is tied to its edge's curve and parameter range, so welding two
/// edges into one invalidates the pcurves of the fins that moved.
pub fn attach_body_pcurves(
    store: &mut crate::topology::TopologyStore,
    geo: &mut crate::geometry::GeometryStore,
    body: opensolid_core::EntityId<crate::topology::Body>,
) -> usize {
    use crate::topology::{Edge, Fin};
    use opensolid_core::EntityId;
    use std::collections::HashMap;

    let mut attached = 0;
    for face in store.faces_of_body(body) {
        let Some(surface) = store
            .faces
            .get(face)
            .and_then(|f| f.surface)
            .and_then(|id| geo.surface(id))
            .cloned()
        else {
            continue;
        };
        let fins: Vec<EntityId<Fin>> = store
            .loops_of_face(face)
            .into_iter()
            .flat_map(|loop_id| store.fins_of_loop(loop_id).to_vec())
            .collect();

        let mut uses: HashMap<EntityId<Edge>, usize> = HashMap::new();
        for fin_id in fins {
            let edge_id = store.fin_edge(fin_id);
            let count = uses.entry(edge_id).or_insert(0);
            let seam = if *count == 0 {
                SeamSide::Low
            } else {
                SeamSide::High
            };
            *count += 1;

            let pcurve = store
                .edge(edge_id)
                .and_then(|edge| {
                    let curve = geo.curve(edge.curve?)?.clone();
                    fit_pcurve(&surface, &curve, edge.t_start, edge.t_end, seam).ok()
                })
                .map(|pcurve| geo.add_pcurve(pcurve));
            if pcurve.is_some() {
                attached += 1;
            }
            let fin = store
                .fins
                .get_mut(fin_id)
                .expect("fin came from this face's loops");
            // Retire what this fin carried before, so re-running the pass
            // does not leave the arena growing by a full body each time.
            if let Some(stale) = std::mem::replace(&mut fin.pcurve, pcurve) {
                geo.pcurves.remove(stale);
            }
        }
    }
    attached
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_core::types::Vector3;

    const TOL: f64 = 1e-9;

    fn assert_close(a: Point2, b: Point2) {
        assert!((a - b).norm() < TOL, "expected {b}, got {a}");
    }

    /// Every pcurve produced here must satisfy the module invariant:
    /// `surface.point(pcurve(t)) == curve.point(t)` across the edge range.
    fn assert_invariant(surface: &Surface3, curve: &Curve3, pcurve: &Curve2, range: (f64, f64)) {
        for i in 0..=16 {
            let t = range.0 + (range.1 - range.0) * (i as f64) / 16.0;
            let uv = pcurve.point(t);
            let on_surface = surface.point(uv.x, uv.y);
            let on_curve = curve.point(t);
            assert!(
                (on_surface - on_curve).norm() < 1e-7,
                "at t = {t}: surface {on_surface} vs curve {on_curve}"
            );
        }
    }

    // --- Curve2 evaluation -------------------------------------------

    #[test]
    fn line_keeps_its_rate_rather_than_normalizing() {
        let line = Curve2::line(Point2::new(1.0, 2.0), Vector2::new(3.0, 4.0)).expect("valid");
        assert_close(line.point(0.0), Point2::new(1.0, 2.0));
        assert_close(line.point(2.0), Point2::new(7.0, 10.0));
        assert_close(Point2::from(line.derivative(0.0)), Point2::new(3.0, 4.0));
        assert_eq!(line.domain(), (f64::NEG_INFINITY, f64::INFINITY));
    }

    #[test]
    fn line_rejects_a_degenerate_direction() {
        assert!(Curve2::line(Point2::origin(), Vector2::zeros()).is_err());
        assert!(Curve2::line(Point2::origin(), Vector2::new(f64::NAN, 0.0)).is_err());
        assert!(Curve2::line(Point2::new(f64::NAN, 0.0), Vector2::x()).is_err());
    }

    #[test]
    fn circle_turns_both_ways() {
        let ccw = Curve2::circle(Point2::origin(), 2.0, Vector2::x(), true).expect("valid");
        assert_close(ccw.point(0.0), Point2::new(2.0, 0.0));
        assert_close(
            ccw.point(std::f64::consts::FRAC_PI_2),
            Point2::new(0.0, 2.0),
        );

        let cw = Curve2::circle(Point2::origin(), 2.0, Vector2::x(), false).expect("valid");
        assert_close(
            cw.point(std::f64::consts::FRAC_PI_2),
            Point2::new(0.0, -2.0),
        );
        assert_eq!(cw.domain(), (0.0, TWO_PI));
    }

    #[test]
    fn circle_derivative_is_the_tangent() {
        let circle = Curve2::circle(Point2::origin(), 3.0, Vector2::x(), true).expect("valid");
        let d = circle.derivative(0.0);
        assert_close(Point2::from(d), Point2::new(0.0, 3.0));
    }

    #[test]
    fn circle_rejects_a_bad_radius_or_direction() {
        assert!(Curve2::circle(Point2::origin(), 0.0, Vector2::x(), true).is_err());
        assert!(Curve2::circle(Point2::origin(), f64::INFINITY, Vector2::x(), true).is_err());
        assert!(Curve2::circle(Point2::origin(), 1.0, Vector2::zeros(), true).is_err());
    }

    #[test]
    fn polyline_interpolates_between_its_own_parameters() {
        let pl = Curve2::polyline(
            vec![2.0, 4.0, 10.0],
            vec![
                Point2::new(0.0, 0.0),
                Point2::new(1.0, 0.0),
                Point2::new(1.0, 3.0),
            ],
        )
        .expect("valid");
        assert_eq!(pl.domain(), (2.0, 10.0));
        assert_close(pl.point(2.0), Point2::new(0.0, 0.0));
        assert_close(pl.point(3.0), Point2::new(0.5, 0.0));
        assert_close(pl.point(7.0), Point2::new(1.0, 1.5));
        assert_close(pl.point(10.0), Point2::new(1.0, 3.0));
        // Rate is per unit of the curve's own parameter, not per vertex.
        assert_close(Point2::from(pl.derivative(3.0)), Point2::new(0.5, 0.0));
    }

    #[test]
    fn polyline_evaluation_clamps_outside_its_domain() {
        let pl = Curve2::polyline(
            vec![0.0, 1.0],
            vec![Point2::new(0.0, 0.0), Point2::new(1.0, 1.0)],
        )
        .expect("valid");
        assert_close(pl.point(-5.0), Point2::new(0.0, 0.0));
        assert_close(pl.point(5.0), Point2::new(1.0, 1.0));
        assert_close(pl.point(f64::NAN), Point2::new(0.0, 0.0));
    }

    #[test]
    fn polyline_rejects_malformed_input() {
        let p = Point2::origin();
        assert!(Curve2::polyline(vec![0.0], vec![p]).is_err());
        assert!(Curve2::polyline(vec![0.0], vec![p, p]).is_err());
        assert!(Curve2::polyline(vec![1.0, 0.0], vec![p, p]).is_err());
        assert!(Curve2::polyline(vec![0.0, 0.0], vec![p, p]).is_err());
        assert!(Curve2::polyline(vec![0.0, f64::NAN], vec![p, p]).is_err());
    }

    // --- fit_pcurve ---------------------------------------------------

    #[test]
    fn line_on_a_plane_fits_a_parameter_space_line() {
        let plane = Surface3::plane(Point3::origin(), Vector3::z()).expect("valid");
        let curve = Curve3::line(Point3::new(1.0, 0.0, 0.0), Vector3::y()).expect("valid");
        let pcurve = fit_pcurve(&plane, &curve, 0.0, 4.0, SeamSide::Low).expect("fits");
        assert!(
            matches!(pcurve, Curve2::Line { .. }),
            "expected a line, got {pcurve:?}"
        );
        assert_invariant(&plane, &curve, &pcurve, (0.0, 4.0));
    }

    #[test]
    fn circle_on_a_plane_fits_a_parameter_space_circle() {
        let plane = Surface3::plane(Point3::origin(), Vector3::z()).expect("valid");
        let curve = Curve3::circle(Point3::new(1.0, 2.0, 0.0), Vector3::z(), 3.0).expect("valid");
        let pcurve = fit_pcurve(&plane, &curve, 0.0, TWO_PI, SeamSide::Low).expect("fits");
        match &pcurve {
            Curve2::Circle { center, radius, .. } => {
                assert_close(*center, Point2::new(1.0, 2.0));
                assert!((radius - 3.0).abs() < TOL);
            }
            other => panic!("expected a circle, got {other:?}"),
        }
        assert_invariant(&plane, &curve, &pcurve, (0.0, TWO_PI));
    }

    #[test]
    fn cylinder_generator_fits_a_parameter_space_line() {
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        // A vertical ruling at u = π/2, running up the cylinder.
        let curve = Curve3::line(Point3::new(0.0, 2.0, 0.0), Vector3::z()).expect("valid");
        let pcurve = fit_pcurve(&cylinder, &curve, 0.0, 5.0, SeamSide::Low).expect("fits");
        assert!(
            matches!(pcurve, Curve2::Line { .. }),
            "expected a line, got {pcurve:?}"
        );
        assert_invariant(&cylinder, &curve, &pcurve, (0.0, 5.0));
    }

    #[test]
    fn cylinder_cross_section_fits_a_parameter_space_line() {
        // A circular cross-section is a *line* in (u, v): v is constant and
        // u advances one-for-one with the circle's angle parameter.
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        let curve = Curve3::circle(Point3::new(0.0, 0.0, 1.0), Vector3::z(), 2.0).expect("valid");
        let pcurve = fit_pcurve(&cylinder, &curve, 0.5, 2.5, SeamSide::Low).expect("fits");
        match &pcurve {
            Curve2::Line { dir, .. } => {
                assert!((dir.x - 1.0).abs() < 1e-7, "u should advance with t");
                assert!(dir.y.abs() < 1e-7, "v should stay constant");
            }
            other => panic!("expected a line, got {other:?}"),
        }
        assert_invariant(&cylinder, &curve, &pcurve, (0.5, 2.5));
    }

    #[test]
    fn sphere_meridian_fits_a_parameter_space_line() {
        let sphere = Surface3::sphere(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        // A meridian in the x–z plane: the great circle about the y axis.
        let curve = Curve3::circle(Point3::origin(), Vector3::y(), 2.0).expect("valid");
        let pcurve = fit_pcurve(&sphere, &curve, 0.2, 1.2, SeamSide::Low).expect("fits");
        assert_invariant(&sphere, &curve, &pcurve, (0.2, 1.2));
    }

    #[test]
    fn seam_side_selects_the_branch_a_seam_edge_sits_on() {
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        // The seam ruling itself: u = 0 and u = 2π describe it equally.
        let seam_point = cylinder.point(0.0, 0.0);
        let curve = Curve3::line(seam_point, Vector3::z()).expect("valid");

        let low = fit_pcurve(&cylinder, &curve, 0.0, 3.0, SeamSide::Low).expect("fits");
        let high = fit_pcurve(&cylinder, &curve, 0.0, 3.0, SeamSide::High).expect("fits");
        assert!(low.point(1.0).x.abs() < TOL, "low branch sits at u = 0");
        assert!(
            (high.point(1.0).x - TWO_PI).abs() < TOL,
            "high branch sits at u = 2π"
        );
        // Both describe the same 3D curve — that is what makes it a seam.
        assert_invariant(&cylinder, &curve, &low, (0.0, 3.0));
        assert_invariant(&cylinder, &curve, &high, (0.0, 3.0));
    }

    #[test]
    fn seam_side_shifts_a_generator_wherever_the_cut_happens_to_sit() {
        // The branch is a shift, not an absolute parameter: a generator away
        // from u = 0 is just as much a seam candidate as one on it, because
        // where a surface puts its parameter origin is its own business.
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        let curve = Curve3::line(cylinder.point(1.0, 0.0), Vector3::z()).expect("valid");
        let low = fit_pcurve(&cylinder, &curve, 0.0, 3.0, SeamSide::Low).expect("fits");
        let high = fit_pcurve(&cylinder, &curve, 0.0, 3.0, SeamSide::High).expect("fits");
        assert!((low.point(0.0).x - 1.0).abs() < 1e-7);
        assert!((high.point(0.0).x - (1.0 + TWO_PI)).abs() < 1e-7);
        assert_invariant(&cylinder, &curve, &high, (0.0, 3.0));
    }

    #[test]
    fn seam_side_leaves_a_curve_that_crosses_the_periodic_direction_alone() {
        // A cross-section varies in u, so it has only one representative:
        // there is no second branch for `High` to move it to.
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        let curve = Curve3::circle(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        let low = fit_pcurve(&cylinder, &curve, 0.5, 2.5, SeamSide::Low).expect("fits");
        let high = fit_pcurve(&cylinder, &curve, 0.5, 2.5, SeamSide::High).expect("fits");
        assert_eq!(low, high);
    }

    #[test]
    fn a_torus_seam_shifts_along_whichever_direction_it_runs_constant_in() {
        let torus = Surface3::torus(Point3::origin(), Vector3::z(), 5.0, 1.0).expect("valid");
        // A minor (tube) circle: u constant, v sweeping — so the ambiguity
        // is in u, and the shift must land there.
        let minor = Curve3::circle(Point3::new(5.0, 0.0, 0.0), Vector3::y(), 1.0).expect("valid");
        let high = fit_pcurve(&torus, &minor, 0.3, 1.3, SeamSide::High).expect("fits");
        let low = fit_pcurve(&torus, &minor, 0.3, 1.3, SeamSide::Low).expect("fits");
        assert!((high.point(0.5).x - (low.point(0.5).x + TWO_PI)).abs() < 1e-7);
        assert!((high.point(0.5).y - low.point(0.5).y).abs() < 1e-7);
        assert_invariant(&torus, &minor, &high, (0.3, 1.3));

        // A major circle: v constant, u sweeping — the shift moves v.
        let major = Curve3::circle(Point3::origin(), Vector3::z(), 6.0).expect("valid");
        let high = fit_pcurve(&torus, &major, 0.3, 1.3, SeamSide::High).expect("fits");
        let low = fit_pcurve(&torus, &major, 0.3, 1.3, SeamSide::Low).expect("fits");
        assert!((high.point(0.5).x - low.point(0.5).x).abs() < 1e-7);
        assert!((high.point(0.5).y - (low.point(0.5).y + TWO_PI)).abs() < 1e-7);
        assert_invariant(&torus, &major, &high, (0.3, 1.3));
    }

    #[test]
    fn a_curve_crossing_the_seam_unwraps_instead_of_jumping() {
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        // A cross-section starting before the seam and sweeping past it.
        let curve = Curve3::circle(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        let (t0, t1) = (TWO_PI - 1.0, TWO_PI + 1.0);
        let pcurve = fit_pcurve(&cylinder, &curve, t0, t1, SeamSide::Low).expect("fits");
        // Continuous across the cut: u keeps climbing past 2π rather than
        // dropping back to 0, so the fit stays a single line.
        assert!(
            matches!(pcurve, Curve2::Line { .. }),
            "expected a line, got {pcurve:?}"
        );
        assert!(pcurve.point(t1).x > TWO_PI);
        assert_invariant(&cylinder, &curve, &pcurve, (t0, t1));
    }

    /// A sphere's seam meridian runs pole to pole, and `u` is undetermined
    /// at a pole. Without repairing those two samples the fit sees a wild
    /// point and degrades to a 33-vertex polyline, where the answer is a
    /// single vertical line.
    #[test]
    fn polar_samples_do_not_poison_a_meridian_fit() {
        let sphere = Surface3::sphere(Point3::origin(), Vector3::z(), 1.0).expect("valid");
        // The full seam meridian: a great circle about −y, swept from the
        // south pole to the north.
        let curve = Curve3::circle(Point3::origin(), -Vector3::y(), 1.0).expect("valid");
        let (t0, t1) = (
            std::f64::consts::FRAC_PI_2 * 3.0,
            std::f64::consts::FRAC_PI_2 * 5.0,
        );
        let pcurve = fit_pcurve(&sphere, &curve, t0, t1, SeamSide::Low).expect("fits");
        match &pcurve {
            Curve2::Line { dir, .. } => {
                assert!(dir.x.abs() < 1e-9, "a meridian holds u constant");
                assert!((dir.y.abs() - 1.0).abs() < 1e-9, "v advances with t");
            }
            other => panic!("expected a line, got {other:?}"),
        }
        assert_invariant(&sphere, &curve, &pcurve, (t0, t1));
    }

    #[test]
    fn a_freeform_curve_falls_back_to_a_polyline() {
        let plane = Surface3::plane(Point3::origin(), Vector3::z()).expect("valid");
        let curve = Curve3::Polyline {
            points: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(3.0, 1.0, 0.0),
                Point3::new(4.0, 4.0, 0.0),
            ],
            closed: false,
        };
        let pcurve = fit_pcurve(&plane, &curve, 0.0, 3.0, SeamSide::Low).expect("fits");
        match &pcurve {
            Curve2::Polyline { params, points } => {
                assert_eq!(params.len(), FIT_SAMPLES);
                assert_eq!(points.len(), FIT_SAMPLES);
                assert_eq!(params[0], 0.0);
                assert_eq!(params[FIT_SAMPLES - 1], 3.0);
            }
            other => panic!("expected a polyline, got {other:?}"),
        }
        // The samples land on the curve's own corners, so the piecewise-
        // linear fit reproduces it well inside the invariant tolerance.
        assert_invariant(&plane, &curve, &pcurve, (0.0, 3.0));
    }

    #[test]
    fn fit_rejects_a_degenerate_parameter_range() {
        let plane = Surface3::plane(Point3::origin(), Vector3::z()).expect("valid");
        let curve = Curve3::line(Point3::origin(), Vector3::x()).expect("valid");
        assert!(fit_pcurve(&plane, &curve, 1.0, 1.0, SeamSide::Low).is_err());
        assert!(fit_pcurve(&plane, &curve, 2.0, 1.0, SeamSide::Low).is_err());
        assert!(fit_pcurve(&plane, &curve, f64::NAN, 1.0, SeamSide::Low).is_err());
    }

    // --- attach_body_pcurves ------------------------------------------

    #[test]
    fn attaching_covers_every_fin_of_a_body_and_holds_the_invariant() {
        use crate::geometry::GeometryStore;
        use crate::primitives;
        use crate::topology::TopologyStore;

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::cylinder(&mut store, &mut geo, 1.5, 4.0).expect("cylinder");

        let mut fin_count = 0;
        for face in store.faces_of_body(body) {
            for loop_id in store.loops_of_face(face) {
                fin_count += store.fins_of_loop(loop_id).len();
                for &fin in store.fins_of_loop(loop_id) {
                    assert!(
                        store.fin(fin).unwrap().pcurve.is_none(),
                        "a kernel-built body starts with no trim geometry"
                    );
                }
            }
        }

        assert_eq!(attach_body_pcurves(&mut store, &mut geo, body), fin_count);
        for face in store.faces_of_body(body) {
            let surface = geo
                .surface(store.face(face).unwrap().surface.unwrap())
                .unwrap()
                .clone();
            for loop_id in store.loops_of_face(face) {
                for &fin_id in store.fins_of_loop(loop_id) {
                    let fin = store.fin(fin_id).unwrap();
                    let pcurve = geo.pcurve(fin.pcurve.expect("every fin covered")).unwrap();
                    let edge = store.edge(fin.edge).unwrap();
                    let curve = geo.curve(edge.curve.unwrap()).unwrap();
                    assert_invariant(&surface, curve, pcurve, (edge.t_start, edge.t_end));
                }
            }
        }
    }

    /// Every closed primitive has at least one seam, and its two fins must
    /// come out a full period apart — including the sphere, whose meridian
    /// runs through both poles, where projection is at its worst
    /// conditioned.
    #[test]
    fn every_closed_primitive_separates_its_seam_fins() {
        use crate::geometry::GeometryStore;
        use crate::primitives;
        use crate::topology::{Edge, TopologyStore};
        use opensolid_core::EntityId;
        use std::collections::HashMap;

        type Build = fn(&mut TopologyStore, &mut GeometryStore) -> Option<EntityId<crate::Body>>;
        let cases: [(&str, Build); 3] = [
            ("cylinder", |s, g| primitives::cylinder(s, g, 1.5, 4.0).ok()),
            ("sphere", |s, g| primitives::sphere(s, g, 2.0).ok()),
            ("torus", |s, g| primitives::torus(s, g, 3.0, 1.0).ok()),
        ];

        for (name, build) in cases {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let body = build(&mut store, &mut geo).expect("primitive builds");
            attach_body_pcurves(&mut store, &mut geo, body);

            let mut seams = 0;
            for face in store.faces_of_body(body) {
                let mut by_edge: HashMap<EntityId<Edge>, Vec<Point2>> = HashMap::new();
                for loop_id in store.loops_of_face(face) {
                    for &fin_id in store.fins_of_loop(loop_id) {
                        let fin = store.fin(fin_id).unwrap();
                        let Some(pcurve) = fin.pcurve.and_then(|id| geo.pcurve(id)) else {
                            continue;
                        };
                        let edge = store.edge(fin.edge).unwrap();
                        let mid = (edge.t_start + edge.t_end) / 2.0;
                        by_edge.entry(fin.edge).or_default().push(pcurve.point(mid));
                    }
                }
                for (edge, uses) in by_edge {
                    if uses.len() < 2 {
                        continue;
                    }
                    seams += 1;
                    let gap = uses[1] - uses[0];
                    assert!(
                        (gap.norm() - TWO_PI).abs() < 1e-6,
                        "{name} {edge:?}: seam fins are {gap} apart, expected one period"
                    );
                }
            }
            assert!(seams >= 1, "{name} must have at least one seam edge");
        }
    }

    /// Re-attaching replaces what was there — the pass is how a caller
    /// refreshes trim geometry after an edit that moved fins between edges,
    /// so it must not depend on the fins starting bare.
    #[test]
    fn attaching_twice_replaces_rather_than_accumulates() {
        use crate::geometry::GeometryStore;
        use crate::primitives;
        use crate::topology::TopologyStore;

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::cylinder(&mut store, &mut geo, 1.5, 4.0).expect("cylinder");

        let first = attach_body_pcurves(&mut store, &mut geo, body);
        let before: Vec<_> = store
            .faces_of_body(body)
            .into_iter()
            .flat_map(|f| store.loops_of_face(f))
            .flat_map(|l| store.fins_of_loop(l).to_vec())
            .map(|fin| {
                geo.pcurve(store.fin(fin).unwrap().pcurve.unwrap())
                    .unwrap()
                    .clone()
            })
            .collect();

        let arena_size = geo.pcurves.len();
        assert_eq!(attach_body_pcurves(&mut store, &mut geo, body), first);
        assert_eq!(
            geo.pcurves.len(),
            arena_size,
            "a re-run must retire the pcurves it replaces"
        );
        let after: Vec<_> = store
            .faces_of_body(body)
            .into_iter()
            .flat_map(|f| store.loops_of_face(f))
            .flat_map(|l| store.fins_of_loop(l).to_vec())
            .map(|fin| {
                geo.pcurve(store.fin(fin).unwrap().pcurve.unwrap())
                    .unwrap()
                    .clone()
            })
            .collect();
        assert_eq!(before, after, "a second pass must be idempotent in effect");
    }

    #[test]
    fn fit_rejects_a_curve_with_no_parameter_space_extent() {
        // A cylinder's axis projects to a single ambiguous locus; a curve
        // running along it has no extent in (u, v) to bound anything with.
        let sphere = Surface3::sphere(Point3::origin(), Vector3::z(), 2.0).expect("valid");
        let curve = Curve3::Polyline {
            points: vec![Point3::new(0.0, 0.0, 2.0), Point3::new(0.0, 0.0, 2.0)],
            closed: false,
        };
        assert!(fit_pcurve(&sphere, &curve, 0.0, 1.0, SeamSide::Low).is_err());
    }
}
