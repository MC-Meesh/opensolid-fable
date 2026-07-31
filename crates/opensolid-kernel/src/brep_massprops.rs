//! Mass properties measured directly from B-Rep geometry — a second,
//! tessellation-free path to the numbers [`crate::massprops`] gets from a
//! [`TriangleMesh`](opensolid_core::mesh::TriangleMesh).
//!
//! # Why a second path
//!
//! Every volume check in this kernel used to route through
//! [`mass_properties`](crate::massprops::mass_properties), which integrates a
//! *triangle mesh*. That makes tessellation a single point of failure: a
//! tessellator bug that is consistent — a face gridded over the wrong
//! parameter rectangle, a seam sampled on the wrong branch — moves the volume
//! of every body that hits it, and no amount of comparing meshed volumes to
//! each other notices. The two paths here share nothing but the surface
//! evaluators: this one reads the topology graph and the trim curves and never
//! builds a triangle, so a disagreement localizes the fault to one side.
//!
//! # The reduction
//!
//! For a solid `Ω` bounded by `∂Ω` with outward normal `n`, every moment is a
//! surface integral (divergence theorem), with `F` chosen so `∇·F` is the
//! integrand:
//!
//! | quantity | `F` | surface form |
//! |---|---|---|
//! | `V` | `(x, y, z)/3` | `⅓ ∮ x·n dA` |
//! | `∫xᵢ dV` | `xᵢ²/2 · êᵢ` | `½ ∮ xᵢ² nᵢ dA` |
//! | `∫xᵢ² dV` | `xᵢ³/3 · êᵢ` | `⅓ ∮ xᵢ³ nᵢ dA` |
//! | `∫xᵢxⱼ dV` | `xᵢ²xⱼ/2 · êᵢ` | `½ ∮ xᵢ²xⱼ nᵢ dA` |
//!
//! On a parametric face the oriented area element is exactly
//! `n dA = (S_u × S_v) du dv`, so each surface integral becomes a plain
//! double integral over the face's region `D` in `(u, v)`.
//!
//! `D` is not a rectangle — it is whatever the face's loops of trim curves
//! bound — so the double integral is reduced *again*, by Green's theorem, to
//! a contour integral over those trim curves:
//!
//! ```text
//! ∫∫_D g(u,v) du dv  =  ∮_{∂D} G(u,v) dv,   G(u,v) = ∫_{u₀}^{u} g(s,v) ds
//! ```
//!
//! for any constant `u₀` (a different one shifts `G` by a function of `v`
//! alone, whose loop integral is zero). Both integrals are evaluated with
//! 8-point Gauss–Legendre on panels sized to the surface: one panel for a
//! plane, where every integrand is a polynomial of degree ≤ 4 that the rule
//! integrates exactly; `π/4` for the angular directions of the quadrics,
//! where the integrands are trigonometric polynomials of degree ≤ 5 and the
//! rule's error is at machine precision; knot spans for NURBS, whose
//! integrands are only piecewise smooth.
//!
//! # Where the parameter-space loops come from
//!
//! From [`Fin::pcurve`](opensolid_brep::topology::Fin::pcurve) when the body
//! carries trim geometry (STEP imports do), and otherwise from
//! [`fit_pcurve`], which projects the fin's 3D edge curve onto the face's
//! surface. That trim geometry is *exact*, not a sampled approximation, for
//! every pairing on an analytic surface: a `Line` or a `Circle` in `(u, v)`
//! where one fits, and otherwise a `Projected` curve that inverts the
//! surface at each of the 3D curve's points. Only a freeform surface falls
//! back to a polyline, whose error is bounded by the sample spacing (see
//! [`opensolid_brep::pcurve`]).
//!
//! Which of those a trim gets is not cosmetic, and this module is where it
//! shows: a spherical cap trimmed off the pole axis used to be measured
//! 1.3e-3 wrong — twelve orders worse than the *congruent* cap trimmed on
//! it — purely because one image fit a `Line` and the other did not
//! (of-y8qc).
//!
//! Projection alone cannot place a **seam** fin, because both branches (`u`
//! and `u ± 2π`) project to the same point. This module resolves seams
//! geometrically rather than by convention: walking a loop, each fin is
//! shifted by whole periods to meet its predecessor, and where that leaves the
//! choice open — a loop made *entirely* of seam fins, as a whole sphere's is —
//! the branch is fixed by which side of the seam the face lies on. Travelling
//! along a constant-`u` seam in `+v`, the interior is to the left, i.e. at
//! lower `u`, so that fin takes the *upper* branch; along a constant-`v` seam
//! in `+u` the interior is at higher `v`, so that fin takes the *lower* one.
//! Both flip for a [`FaceSense::Negative`] face, whose loops wind the other
//! way.
//!
//! Gaps that survive the walk are closed with a straight segment in `(u, v)`.
//! That is not a patch over a defect: a sphere's boundary really is two
//! meridians, and the pole rows that join them are lines the parameterization
//! collapses to a point, carrying no edge to hang a fin on. A gap that is
//! *not* on such a line is a genuinely open loop and is rejected
//! ([`BrepMassPropertiesError::OpenParameterLoop`]).
//!
//! # Signs come from the winding, not from a flag
//!
//! A loop runs counterclockwise about its *face* normal, and the parameter
//! map takes counterclockwise in `(u, v)` to counterclockwise about
//! `S_u × S_v`. So a [`FaceSense::Positive`] face's loop is counterclockwise
//! in `(u, v)` and Green's theorem returns `+∫∫_D`, which is what a face whose
//! surface normal *is* the outward normal should contribute; a
//! [`FaceSense::Negative`] face's loop is clockwise and returns `−∫∫_D`, which
//! is again exactly right. No sense factor appears anywhere below — the only
//! explicit sign is [`ShellOrientation::Inward`], where outward-from-material
//! opposes the face normal (a void shell subtracting its cavity).
//!
//! Surface area is the one quantity that is *not* signed, so it alone is
//! multiplied by the winding, recovered from the same contour as the signed
//! area of `D` in parameter space.
//!
//! [`FaceSense`] is read in exactly one place — [`seam_branch`], to say which
//! side of a seam the interior is on — and that reading assumes the flag
//! agrees with the loop it labels, which on a well-formed body it does
//! ([`CheckFailure::FaceSenseContradictsLoop`](opensolid_brep::CheckFailure)
//! is what enforces it). On a body where it does not, this is the one place
//! the disagreement shows: a seam fin lands a period from where the walk
//! wants it, and the loop fails to close by twice the period. That is what
//! of-hrgt was — a STEP import whose sense flag had been flipped without its
//! loops, reported here as `OpenParameterLoop { gap: 4π }` on a cylinder wall.
//! The repair is in the reader
//! ([`reconcile_face_senses`](crate::io::step::heal::reconcile_face_senses));
//! nothing about the integration below needed changing.

use std::collections::HashMap;

use nalgebra::Matrix3;
use opensolid_brep::curve::{Curve3, CurveEval};
use opensolid_brep::geometry::GeometryStore;
use opensolid_brep::pcurve::{Curve2, Curve2Eval, SeamSide, fit_pcurve};
use opensolid_brep::surface::{Surface3, SurfaceEval};
use opensolid_brep::topology::{
    Body, Edge, Face, FaceSense, Fin, FinSense, ShellOrientation, TopologyStore,
};
use opensolid_core::types::{Point2, Vector2};
use opensolid_core::{EntityId, Point3, Vector3};
use thiserror::Error;

use crate::massprops::MassProperties;

/// Why a body's B-Rep mass properties could not be measured.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum BrepMassPropertiesError {
    /// The body has no faces, so it bounds nothing.
    #[error("body has no faces")]
    NoFaces,
    /// A face carries no surface (or a stale surface id): there is no
    /// parameterization to integrate over.
    #[error("{face:?} has no surface geometry")]
    MissingSurface { face: EntityId<Face> },
    /// A fin has neither a stored pcurve nor an edge curve that can be
    /// projected into the face's parameter space.
    #[error("{fin:?} has no usable trim geometry")]
    MissingTrim { fin: EntityId<Fin> },
    /// A fin's edge spans an empty parameter range on a curve whose ends do
    /// not meet, so the fin traverses nothing and its loop cannot close.
    #[error("{fin:?} spans the empty parameter range [{t_start}, {t_end}] on an open curve")]
    EmptyEdgeRange {
        fin: EntityId<Fin>,
        t_start: f64,
        t_end: f64,
    },
    /// A loop does not close in parameter space, and the gap is not along a
    /// line the parameterization collapses (a pole row or a cone apex).
    #[error("{face:?} has an open boundary in parameter space (gap {gap:.3e} at {at:?})")]
    OpenParameterLoop {
        face: EntityId<Face>,
        gap: f64,
        at: Point2,
    },
    /// The signed volume came out zero or negative: the body's faces do not
    /// consistently bound a positive region.
    #[error("body encloses non-positive volume {volume:.6e}")]
    NonPositiveVolume { volume: f64 },
}

/// Eight-point Gauss–Legendre abscissae on `[-1, 1]`.
const GL_X: [f64; 8] = [
    -0.960_289_856_497_536_3,
    -0.796_666_477_413_626_7,
    -0.525_532_409_916_329,
    -0.183_434_642_495_649_8,
    0.183_434_642_495_649_8,
    0.525_532_409_916_329,
    0.796_666_477_413_626_7,
    0.960_289_856_497_536_3,
];

/// Weights matching [`GL_X`].
const GL_W: [f64; 8] = [
    0.101_228_536_290_376_3,
    0.222_381_034_453_374_5,
    0.313_706_645_877_887_3,
    0.362_683_783_378_362,
    0.362_683_783_378_362,
    0.313_706_645_877_887_3,
    0.222_381_034_453_374_5,
    0.101_228_536_290_376_3,
];

/// Panel width for an angular parameter direction. The quadrics' integrands
/// are trigonometric polynomials of degree ≤ 5; at this width the 8-point
/// rule's error term is ~1e-15 relative, i.e. at the noise floor.
const ANGULAR_PANEL: f64 = std::f64::consts::FRAC_PI_4;

/// Panels a NURBS knot span is split into. The patch is only `C^{p-m}` across
/// a knot, so the knots themselves are always panel boundaries; this is the
/// extra resolution *within* a span.
const NURBS_PANELS_PER_SPAN: usize = 2;

/// Runaway guard on panels per integration interval.
const MAX_PANELS: usize = 256;

/// Bisection depth cap for the contour panel refinement. Adaptive bisection
/// only descends where the curve is moving, so a single sharp feature costs
/// about two panels per level rather than `2^depth` — twenty levels resolve
/// a feature a millionth of the interval wide inside [`MAX_PANELS`].
const MAX_PANEL_DEPTH: usize = 20;

/// Samples used to measure how far a trim curve travels in `(u, v)` across a
/// candidate panel.
const EXCURSION_SAMPLES: usize = 4;

/// How nearly constant a seam fin's pcurve must be in one parameter to count
/// as running *along* that direction. Deliberately loose: this is a yes/no
/// question about a seam's direction, and a projected pcurve near a
/// parameterization singularity wobbles far past any fit tolerance.
const SEAM_CONSTANT_TOL: f64 = 1e-6;

/// A parameter-space gap wider than this fraction of the face's own
/// parameter extent is treated as a real break rather than fit noise.
const GAP_TOL_REL: f64 = 1e-2;

/// The ten volume moments plus the two areas, accumulated in one pass.
///
/// Everything here is *signed*: a clockwise loop in `(u, v)` negates the lot,
/// which is what makes the winding carry the orientation (see the module
/// docs). `uv_area` is the odd one out only in what it is used for — it is
/// the signed area of `D` in parameter space, whose sign recovers the winding
/// so `area` can be unsigned again.
#[derive(Debug, Clone, Copy, Default)]
struct Moments {
    volume: f64,
    /// `∫x dV`, `∫y dV`, `∫z dV`.
    first: Vector3,
    /// `∫x² dV`, `∫y² dV`, `∫z² dV`.
    diag: Vector3,
    /// `∫xy dV`, `∫yz dV`, `∫zx dV`.
    off: Vector3,
    area: f64,
    uv_area: f64,
}

impl Moments {
    /// `self += w * rhs`.
    fn axpy(&mut self, w: f64, rhs: &Moments) {
        self.volume += w * rhs.volume;
        self.first += rhs.first * w;
        self.diag += rhs.diag * w;
        self.off += rhs.off * w;
        self.area += w * rhs.area;
        self.uv_area += w * rhs.uv_area;
    }
}

/// The integrands of every moment at one parameter-space point, before the
/// `u`-antiderivative and the contour integral.
///
/// `n` is the *unnormalized* `S_u × S_v`, so `n du dv` is already the oriented
/// area element and no Jacobian is applied separately.
fn integrand(surface: &Surface3, u: f64, v: f64) -> Moments {
    let p = surface.point(u, v);
    let n = surface.du(u, v).cross(&surface.dv(u, v));
    let (x, y, z) = (p.x, p.y, p.z);
    Moments {
        volume: (x * n.x + y * n.y + z * n.z) / 3.0,
        first: Vector3::new(x * x * n.x, y * y * n.y, z * z * n.z) * 0.5,
        diag: Vector3::new(x * x * x * n.x, y * y * y * n.y, z * z * z * n.z) / 3.0,
        off: Vector3::new(x * x * y * n.x, y * y * z * n.y, z * z * x * n.z) * 0.5,
        area: n.norm(),
        uv_area: 1.0,
    }
}

/// `a > b`, and false whenever either side is NaN — the comparison every
/// guard below wants, spelled so it cannot silently invert on a NaN.
fn exceeds(a: f64, b: f64) -> bool {
    a.partial_cmp(&b) == Some(std::cmp::Ordering::Greater)
}

/// Panel count for an interval of length `len` at target width `step`.
/// An infinite `step` (a direction the integrands are polynomial in) asks for
/// exactly one panel, which the 8-point rule integrates exactly.
fn subdivisions(len: f64, step: f64) -> usize {
    if !len.is_finite() || !exceeds(step, 0.0) {
        return 1;
    }
    let n = (len.abs() / step).ceil();
    if !n.is_finite() {
        return MAX_PANELS;
    }
    (n as usize).clamp(1, MAX_PANELS)
}

/// Target panel width in `u`: infinite where the integrands are polynomial in
/// `u` (a plane), angular for the quadrics, a fraction of a knot span for a
/// NURBS patch.
fn u_step(surface: &Surface3) -> f64 {
    match surface {
        Surface3::Plane { .. } => f64::INFINITY,
        Surface3::Nurbs(_) => {
            let (a, b) = surface.domain_u();
            (b - a) / (NURBS_PANELS_PER_SPAN as f64 * 4.0)
        }
        _ => ANGULAR_PANEL,
    }
}

/// Target panel width in `v`. Cylinder and cone `v` are ruled directions the
/// integrands are polynomial in; sphere and torus `v` are angular.
fn v_step(surface: &Surface3) -> f64 {
    match surface {
        Surface3::Plane { .. } | Surface3::Cylinder { .. } | Surface3::Cone { .. } => f64::INFINITY,
        Surface3::Nurbs(_) => {
            let (a, b) = surface.domain_v();
            (b - a) / (NURBS_PANELS_PER_SPAN as f64 * 4.0)
        }
        Surface3::Sphere { .. } | Surface3::Torus { .. } => ANGULAR_PANEL,
    }
}

/// `G(u, v) = ∫_{u0}^{u} g(s, v) ds`, the inner half of the Green reduction.
///
/// Panel boundaries land on the NURBS knots inside the interval, where the
/// patch loses smoothness and a rule spanning the knot would lose its order.
fn u_integral(surface: &Surface3, u0: f64, u: f64, v: f64) -> Moments {
    let mut acc = Moments::default();
    let span = u - u0;
    if span == 0.0 || !span.is_finite() {
        return acc;
    }
    let (lo, hi) = if span > 0.0 { (u0, u) } else { (u, u0) };

    let mut cuts: Vec<f64> = vec![lo];
    if let Surface3::Nurbs(nurbs) = surface {
        for &knot in nurbs.knot_vector_u().knots() {
            if knot > lo && knot < hi && cuts.last() != Some(&knot) {
                cuts.push(knot);
            }
        }
    }
    cuts.push(hi);

    let step = u_step(surface);
    for window in cuts.windows(2) {
        let (a, b) = (window[0], window[1]);
        let n = subdivisions(b - a, step);
        for i in 0..n {
            let s0 = a + (b - a) * (i as f64) / (n as f64);
            let s1 = a + (b - a) * ((i + 1) as f64) / (n as f64);
            let half = (s1 - s0) * 0.5;
            let mid = (s0 + s1) * 0.5;
            for k in 0..GL_X.len() {
                acc.axpy(GL_W[k] * half, &integrand(surface, mid + half * GL_X[k], v));
            }
        }
    }

    if span > 0.0 {
        acc
    } else {
        let mut flipped = Moments::default();
        flipped.axpy(-1.0, &acc);
        flipped
    }
}

/// Panels for integrating along `pcurve` over `[t0, t1]`.
///
/// Split first at the trim curve's own kinks (a polyline's vertices) and at
/// an angular step for a circle, then refine each piece until the distance it
/// covers in `(u, v)` is within the surface's panel widths — the smoothness
/// that matters is the *surface's*, and a trim curve's parameter says nothing
/// about how far it drags the integrand.
///
/// The refinement bisects rather than dividing a piece into equal parts,
/// because a trim curve can put all of its motion in a corner of its
/// parameter range and equal parts would spend their resolution where
/// nothing happens. A sphere cut by a plane a distance `δ` from its centre
/// does exactly that: `u` covers half its range within `δ` of the crossing,
/// at a rate of `1/δ`, so a lens between two spheres 1e-3 apart needs its
/// panels a thousand times finer *there* and nowhere else. Equal parts had
/// that lens 5.6e-8 out; bisection puts it back at floating point, for
/// about forty panels.
fn t_panels(surface: &Surface3, pcurve: &Curve2, t0: f64, t1: f64) -> Vec<(f64, f64)> {
    let mut coarse = vec![t0];
    match pcurve {
        Curve2::Polyline { params, .. } => {
            for &p in params {
                if p > t0 && p < t1 {
                    coarse.push(p);
                }
            }
        }
        Curve2::Circle { .. } => {
            let n = subdivisions(t1 - t0, ANGULAR_PANEL);
            for i in 1..n {
                coarse.push(t0 + (t1 - t0) * (i as f64) / (n as f64));
            }
        }
        Curve2::Line { .. } => {}
        // The guide's vertices are where its samples were placed, which is
        // the only structural hint an exactly-inverted curve carries. It is
        // a hint and not a kink list — the curve is smooth through them —
        // but the excursion refinement below is what actually sizes the
        // panels, and starting it from the sample breaks costs nothing.
        Curve2::Projected(p) => {
            for &t in p.guide_params() {
                if t > t0 && t < t1 {
                    coarse.push(t);
                }
            }
        }
    }
    coarse.push(t1);

    let (us, vs) = (u_step(surface), v_step(surface));
    let mut out = Vec::new();
    for window in coarse.windows(2) {
        let (a, b) = (window[0], window[1]);
        if !exceeds(b, a) {
            continue;
        }
        let mut pending = vec![(a, b, 0usize)];
        let mut emitted = 0usize;
        while let Some((a, b, depth)) = pending.pop() {
            let mid = 0.5 * (a + b);
            // A half that is not strictly inside its parent has run out of
            // parameter to split, whatever it does in `(u, v)`.
            let splittable = depth < MAX_PANEL_DEPTH
                && emitted + pending.len() + 1 < MAX_PANELS
                && exceeds(mid, a)
                && exceeds(b, mid);
            if splittable {
                let (du, dv) = excursion(pcurve, a, b);
                if exceeds(du, us) || exceeds(dv, vs) {
                    // Right before left, so popping walks the interval
                    // forwards and `out` comes back ordered — including
                    // where the budget runs out and the rest is emitted
                    // unsplit.
                    pending.push((mid, b, depth + 1));
                    pending.push((a, mid, depth + 1));
                    continue;
                }
            }
            out.push((a, b));
            emitted += 1;
        }
    }
    out
}

/// How far `pcurve` travels in each parameter direction across `[a, b]`,
/// measured as the total variation over [`EXCURSION_SAMPLES`] steps rather
/// than as the distance between the ends — a panel the curve leaves and
/// returns along has moved, whatever its endpoints say.
fn excursion(pcurve: &Curve2, a: f64, b: f64) -> (f64, f64) {
    let (mut du, mut dv) = (0.0, 0.0);
    let mut prev = pcurve.point(a);
    for i in 1..=EXCURSION_SAMPLES {
        let p = pcurve.point(a + (b - a) * (i as f64) / (EXCURSION_SAMPLES as f64));
        du += (p.x - prev.x).abs();
        dv += (p.y - prev.y).abs();
        prev = p;
    }
    (du, dv)
}

/// `∮ G dv` along `pcurve` over `[t0, t1]`, accumulated into `acc` with
/// `weight` (`-1` traverses the curve backwards).
fn contour_integral(
    surface: &Surface3,
    u0: f64,
    pcurve: &Curve2,
    (t0, t1): (f64, f64),
    weight: f64,
    acc: &mut Moments,
) {
    for (a, b) in t_panels(surface, pcurve, t0, t1) {
        let half = (b - a) * 0.5;
        let mid = (a + b) * 0.5;
        for k in 0..GL_X.len() {
            let t = mid + half * GL_X[k];
            let uv = pcurve.point(t);
            let dv = pcurve.derivative(t).y;
            if dv == 0.0 {
                continue;
            }
            let g = u_integral(surface, u0, uv.x, uv.y);
            acc.axpy(weight * GL_W[k] * half * dv, &g);
        }
    }
}

/// Translate a trim curve by `shift` in parameter space — how a seam fin is
/// moved onto the branch its loop needs.
fn shifted(curve: &Curve2, shift: Vector2) -> Curve2 {
    if shift == Vector2::zeros() {
        return curve.clone();
    }
    match curve {
        Curve2::Line { origin, dir } => Curve2::Line {
            origin: origin + shift,
            dir: *dir,
        },
        Curve2::Circle {
            center,
            radius,
            x_dir,
            ccw,
        } => Curve2::Circle {
            center: center + shift,
            radius: *radius,
            x_dir: *x_dir,
            ccw: *ccw,
        },
        Curve2::Polyline { params, points } => Curve2::Polyline {
            params: params.clone(),
            points: points.iter().map(|p| p + shift).collect(),
        },
        // The shift rides on the projected curve's own offset, so the branch
        // guide keeps naming the branch inversion lands on and the exact
        // value is translated afterwards — which is the whole point of
        // keeping the two apart.
        Curve2::Projected(p) => Curve2::Projected(Box::new(p.shifted(shift))),
    }
}

/// One fin's trim curve, already on the branch that makes its loop close.
struct Trim {
    curve: Curve2,
    range: (f64, f64),
    sense: FinSense,
}

impl Trim {
    fn start(&self) -> Point2 {
        match self.sense {
            FinSense::Forward => self.curve.point(self.range.0),
            FinSense::Reversed => self.curve.point(self.range.1),
        }
    }

    fn end(&self) -> Point2 {
        match self.sense {
            FinSense::Forward => self.curve.point(self.range.1),
            FinSense::Reversed => self.curve.point(self.range.0),
        }
    }

    /// `+1` forwards along `range`, `-1` backwards.
    fn weight(&self) -> f64 {
        match self.sense {
            FinSense::Forward => 1.0,
            FinSense::Reversed => -1.0,
        }
    }
}

/// Period along each parameter direction, `None` where the surface does not
/// repeat and no branch choice exists.
fn periods(surface: &Surface3) -> (Option<f64>, Option<f64>) {
    (surface.period_u(), surface.period_v())
}

/// Whole-period shift bringing `start` onto the branch nearest `target`.
fn continuity_shift(target: Point2, start: Point2, periods: (Option<f64>, Option<f64>)) -> Vector2 {
    let axis = |delta: f64, period: Option<f64>| match period {
        Some(p) if p > 0.0 => (delta / p).round() * p,
        _ => 0.0,
    };
    Vector2::new(
        axis(target.x - start.x, periods.0),
        axis(target.y - start.y, periods.1),
    )
}

/// Which parameter a seam fin holds constant, and how far its branch sits
/// from the first use's.
///
/// A seam runs along one constant parameter, and the face lies on one side of
/// it. Walking counterclockwise the interior is to the left: along a
/// constant-`u` seam travelled in `+v`, left is `−u`, so that fin sits on the
/// *upper* branch; along a constant-`v` seam travelled in `+u`, left is `+v`,
/// so that fin sits on the *lower* one. `winding` is `-1` for a
/// [`FaceSense::Negative`] face, whose loops run the other way.
///
/// Returns `(the constant parameter is u, signed offset)`, or `None` when the
/// fin does not look like a seam use after all (it varies in both parameters,
/// or the constant direction is not periodic), leaving the caller on plain
/// continuity.
fn seam_branch(
    net: Vector2,
    periods: (Option<f64>, Option<f64>),
    winding: f64,
) -> Option<(bool, f64)> {
    let scale = net.norm();
    if scale == 0.0 {
        return None;
    }
    if net.x.abs() <= SEAM_CONSTANT_TOL * scale {
        // Constant in u: the +v-going use takes the upper branch.
        let period = periods.0?;
        let sign = if net.y > 0.0 { 1.0 } else { -1.0 };
        Some((true, winding * sign * period))
    } else if net.y.abs() <= SEAM_CONSTANT_TOL * scale {
        // Constant in v: the +u-going use takes the lower branch.
        let period = periods.1?;
        let sign = if net.x > 0.0 { -1.0 } else { 1.0 };
        Some((false, winding * sign * period))
    } else {
        None
    }
}

/// A parameter range this short a fraction of its curve's own domain spans
/// nothing a fit could see, however the two endpoints compare.
const DEGENERATE_RANGE_REL: f64 = 1e-9;

/// The parameter ranges a fin's edge really spans, in forward traversal
/// order — one normally, two when the edge wraps past the end of its curve's
/// domain.
///
/// A range that does not increase is a *wrap*: the edge runs off the end of
/// the domain and resumes at the start. That is how an arc across a circle's
/// `t = 0`, and how every imprint loop the boolean pipeline closes on itself,
/// arrive here. The wrapped part cannot simply be evaluated past the domain
/// end — a sampled [`Curve3::Polyline`](opensolid_brep::Curve3::Polyline)
/// clamps there rather than repeating — so it is split into the two in-domain
/// pieces instead, which trace exactly the same arc.
///
/// Wrapping is only meaningful when the curve's ends actually meet, which is
/// checked *geometrically*: the closed imprint polylines carry `closed:
/// false` and no period despite looping, which is what leaves their ranges
/// unwrapped (and empty, or a rounding step wide) in the first place — the
/// emitter-side defect is of-i7ka, and this recovery goes away with it. A
/// range that neither increases nor wraps onto a closed curve bounds nothing
/// and gets [`BrepMassPropertiesError::EmptyEdgeRange`] rather than a
/// plausible number.
fn edge_ranges(edge: &opensolid_brep::Edge, curve: Option<&Curve3>) -> Option<Vec<(f64, f64)>> {
    let curve = curve?;
    let (lo, hi) = curve.domain();
    let span = hi - lo;
    if !span.is_finite() {
        // An unbounded curve (a line) has nothing to wrap around.
        return (edge.t_end > edge.t_start).then(|| vec![(edge.t_start, edge.t_end)]);
    }
    if !exceeds(span, 0.0) {
        return None;
    }
    let empty = DEGENERATE_RANGE_REL * span;
    if edge.t_end - edge.t_start > empty {
        return Some(vec![(edge.t_start, edge.t_end)]);
    }
    let closes = (curve.point(lo) - curve.point(hi)).norm()
        <= edge.tolerance.max(opensolid_brep::SYSTEM_RESOLUTION);
    if !closes {
        return None;
    }
    let pieces: Vec<(f64, f64)> = [
        (edge.t_start.clamp(lo, hi), hi),
        (lo, edge.t_end.clamp(lo, hi)),
    ]
    .into_iter()
    .filter(|&(a, b)| b - a > empty)
    .collect();
    // Both ends sitting on the domain boundary means the edge is the whole
    // closed curve, not nothing.
    Some(if pieces.is_empty() {
        vec![(lo, hi)]
    } else {
        pieces
    })
}

/// Every fin's trim curve for one face, grouped by loop and unwrapped onto
/// branches that make each loop close in parameter space.
///
/// Prefers the fins' own stored pcurves, but only when *every* fin of the face
/// has one: mixing stored and refitted curves would mix branch conventions on
/// a seam, and a face's loops only mean anything read together.
fn face_trims(
    store: &TopologyStore,
    geo: &GeometryStore,
    surface: &Surface3,
    face_id: EntityId<Face>,
) -> Result<Vec<Vec<Trim>>, BrepMassPropertiesError> {
    let face = store
        .face(face_id)
        .ok_or(BrepMassPropertiesError::MissingSurface { face: face_id })?;
    let winding = match face.sense {
        FaceSense::Positive => 1.0,
        FaceSense::Negative => -1.0,
    };
    let loop_ids = store.loops_of_face(face_id);
    let stored_everywhere = loop_ids.iter().all(|&loop_id| {
        store.fins_of_loop(loop_id).iter().all(|&fin_id| {
            match store.fin(fin_id).and_then(|f| f.pcurve) {
                Some(pcurve) => geo.pcurve(pcurve).is_some(),
                None => false,
            }
        })
    });

    let per = periods(surface);
    // Where each seam edge's *first* use in this face landed, so the second
    // use can be placed a period away from that branch.
    let mut first_use: HashMap<EntityId<Edge>, Point2> = HashMap::new();
    let mut out = Vec::with_capacity(loop_ids.len());

    for loop_id in loop_ids {
        let fin_ids = store.fins_of_loop(loop_id).to_vec();
        if fin_ids.is_empty() {
            // A degenerate vertex loop bounds no area: a cone apex or a
            // sphere pole, already covered by the gap-closing segments.
            continue;
        }
        let mut trims: Vec<Trim> = Vec::with_capacity(fin_ids.len());
        for fin_id in fin_ids {
            let fin = store
                .fin(fin_id)
                .ok_or(BrepMassPropertiesError::MissingTrim { fin: fin_id })?;
            let edge_id = fin.edge;
            let edge = store
                .edge(edge_id)
                .ok_or(BrepMassPropertiesError::MissingTrim { fin: fin_id })?;
            let edge_curve = edge.curve.and_then(|id| geo.curve(id));
            let mut ranges =
                edge_ranges(edge, edge_curve).ok_or(BrepMassPropertiesError::EmptyEdgeRange {
                    fin: fin_id,
                    t_start: edge.t_start,
                    t_end: edge.t_end,
                })?;
            // A wrapped edge's two pieces are listed in the curve's own
            // direction; a reversed fin walks them the other way round.
            if fin.sense == FinSense::Reversed {
                ranges.reverse();
            }
            // Only a whole fin counts as a use of its edge — the pieces of a
            // wrapped one are not two visits to a seam.
            let first_visit = !first_use.contains_key(&edge_id);

            for (piece, range) in ranges.into_iter().enumerate() {
                let base = if stored_everywhere {
                    geo.pcurve(fin.pcurve.expect("checked above"))
                        .expect("checked above")
                        .clone()
                } else {
                    let curve =
                        edge_curve.ok_or(BrepMassPropertiesError::MissingTrim { fin: fin_id })?;
                    fit_pcurve(surface, curve, range.0, range.1, SeamSide::Low)
                        .map_err(|_| BrepMassPropertiesError::MissingTrim { fin: fin_id })?
                };

                let probe = Trim {
                    curve: base,
                    range,
                    sense: fin.sense,
                };
                let net = probe.end() - probe.start();
                let previous_end = trims.last().map(|t: &Trim| t.end());
                let seam_anchor = if piece == 0 && !first_visit {
                    first_use.get(&edge_id).copied()
                } else {
                    None
                };

                let shift = match (seam_anchor, previous_end) {
                    // Second use of an edge this face already walked: a seam.
                    // Its branch is fixed by which side of the seam the face
                    // is on, which continuity cannot see when the two uses
                    // coincide. The target is *absolute* — one period from
                    // where the first use ended up — so it holds whether the
                    // base curves came in on one branch (a fresh fit) or on
                    // two (stored pcurves, which record the split already).
                    (Some(anchor), previous) => match seam_branch(net, per, winding) {
                        Some((constant_in_u, offset)) => {
                            let base = probe.start();
                            let mut shift = if constant_in_u {
                                Vector2::new(anchor.x + offset - base.x, 0.0)
                            } else {
                                Vector2::new(0.0, anchor.y + offset - base.y)
                            };
                            // The seam only pins its own constant direction;
                            // the other one still follows the walk.
                            if let Some(prev) = previous {
                                let free = continuity_shift(prev, base + shift, per);
                                if constant_in_u {
                                    shift.y += free.y;
                                } else {
                                    shift.x += free.x;
                                }
                            }
                            shift
                        }
                        None => previous
                            .map(|prev| continuity_shift(prev, probe.start(), per))
                            .unwrap_or_else(Vector2::zeros),
                    },
                    (None, Some(prev)) => continuity_shift(prev, probe.start(), per),
                    (None, None) => Vector2::zeros(),
                };

                let placed = Trim {
                    curve: shifted(&probe.curve, shift),
                    range,
                    sense: probe.sense,
                };
                if piece == 0 {
                    // Record where this edge's first use *landed*, not the
                    // shift that put it there: the second use is placed a
                    // period from the branch, not from the correction.
                    first_use.entry(edge_id).or_insert_with(|| placed.start());
                }
                trims.push(placed);
            }
        }
        out.push(trims);
    }
    Ok(out)
}

/// Samples taken along a parameter-space gap when deciding whether the
/// surface collapses it to a point.
const BRIDGE_SAMPLES: usize = 8;

/// Whether the straight `(u, v)` segment from `a` to `b` is a line the
/// surface collapses to a single point.
///
/// This is what makes a sphere's pole row, a cone's apex and a NURBS patch's
/// collapsed control row legitimate places for a loop to jump: the row is a
/// real part of the face's boundary that carries no edge, because in model
/// space it is one point. The question is asked *geometrically* — does the
/// whole segment map to the same place? — rather than through
/// [`SurfaceEval::is_singular`], whose threshold a fitted pcurve wobbling near
/// an ill-conditioned pole can miss while still landing on the pole.
///
/// Sampling the interior is what keeps it honest. Two seam fins wrongly left
/// on the same branch also have coincident *endpoints* (they are the same
/// edge), but the segment between them crosses the far side of the surface,
/// and that is what this catches.
fn is_collapsed_bridge(surface: &Surface3, a: Point2, b: Point2) -> bool {
    let anchor = surface.point(a.x, a.y);
    let tolerance = 1e-9 * (1.0 + anchor.coords.norm());
    (0..=BRIDGE_SAMPLES).all(|i| {
        let t = (i as f64) / (BRIDGE_SAMPLES as f64);
        let uv = a + (b - a) * t;
        (surface.point(uv.x, uv.y) - anchor).norm() <= tolerance
    })
}

/// Signed moments of the region one face covers, contributed with its own
/// face normal (see the module docs on why no sense factor appears).
fn face_moments(
    surface: &Surface3,
    loops: &[Vec<Trim>],
    face_id: EntityId<Face>,
) -> Result<Moments, BrepMassPropertiesError> {
    // A well-conditioned origin for the inner integral: the middle of the
    // parameter extent the trim curves actually visit. Any constant works
    // (the module docs say why), so pick the one with the shortest reach.
    let mut lo = Point2::new(f64::INFINITY, f64::INFINITY);
    let mut hi = Point2::new(f64::NEG_INFINITY, f64::NEG_INFINITY);
    for trims in loops {
        for trim in trims {
            for i in 0..=EXCURSION_SAMPLES {
                let t = trim.range.0
                    + (trim.range.1 - trim.range.0) * (i as f64) / (EXCURSION_SAMPLES as f64);
                let p = trim.curve.point(t);
                lo = Point2::new(lo.x.min(p.x), lo.y.min(p.y));
                hi = Point2::new(hi.x.max(p.x), hi.y.max(p.y));
            }
        }
    }
    if !lo.x.is_finite() || !hi.x.is_finite() {
        return Ok(Moments::default());
    }
    let u0 = 0.5 * (lo.x + hi.x);
    let extent = (hi - lo).norm().max(f64::MIN_POSITIVE);

    let mut acc = Moments::default();
    for trims in loops {
        for (index, trim) in trims.iter().enumerate() {
            contour_integral(
                surface,
                u0,
                &trim.curve,
                trim.range,
                trim.weight(),
                &mut acc,
            );

            // Close onto the next fin (wrapping at the end of the loop). A
            // real gap only survives on a line the parameterization collapses.
            let next = &trims[(index + 1) % trims.len()];
            let (from, to) = (trim.end(), next.start());
            let jump = to - from;
            if jump.norm() <= GAP_TOL_REL * extent * 1e-6 {
                continue;
            }
            if jump.norm() > GAP_TOL_REL * extent && !is_collapsed_bridge(surface, from, to) {
                return Err(BrepMassPropertiesError::OpenParameterLoop {
                    face: face_id,
                    gap: jump.norm(),
                    at: from,
                });
            }
            let bridge = Curve2::Line {
                origin: from,
                dir: jump,
            };
            contour_integral(surface, u0, &bridge, (0.0, 1.0), 1.0, &mut acc);
        }
    }
    Ok(acc)
}

/// Mass properties of the solid bounded by `body`'s faces, integrated over
/// the B-Rep surfaces themselves — no tessellation anywhere.
///
/// Unit density, so mass equals volume and the inertia tensor scales linearly
/// with density; the fields mean exactly what
/// [`mass_properties`](crate::massprops::mass_properties) returns for the same
/// solid, which is the point.
///
/// The body must be closed and consistently oriented. Nothing here checks
/// that (it is [`TopologyStore::check`]'s job); what an unclosed body gets is
/// a wrong number or, more often, [`BrepMassPropertiesError::OpenParameterLoop`]
/// or [`BrepMassPropertiesError::NonPositiveVolume`].
///
/// # Errors
/// See [`BrepMassPropertiesError`]: missing surface or trim geometry, a
/// parameter-space loop that does not close along a collapsed line, or a body
/// whose faces do not bound a positive volume.
pub fn brep_mass_properties(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
) -> Result<MassProperties, BrepMassPropertiesError> {
    let shells = store.shells_of_body(body).to_vec();
    let mut total = Moments::default();
    let mut face_count = 0usize;

    for shell_id in shells {
        let orientation = store
            .shell(shell_id)
            .map(|s| s.orientation)
            .unwrap_or(ShellOrientation::Outward);
        // Outward-from-material opposes the face normal on a void shell, so
        // its cavity subtracts. Every other sign rides on the loop winding.
        let flip = match orientation {
            ShellOrientation::Outward => 1.0,
            ShellOrientation::Inward => -1.0,
        };
        for &face_id in store.faces_of_shell(shell_id) {
            let surface = store
                .face(face_id)
                .and_then(|f| f.surface)
                .and_then(|id| geo.surface(id))
                .ok_or(BrepMassPropertiesError::MissingSurface { face: face_id })?;
            let loops = face_trims(store, geo, surface, face_id)?;
            let moments = face_moments(surface, &loops, face_id)?;

            face_count += 1;
            total.axpy(flip, &moments);
            // Area is the one unsigned quantity: undo the winding, which the
            // signed parameter-space area reports.
            let winding = if moments.uv_area < 0.0 { -1.0 } else { 1.0 };
            total.area += (winding - flip) * moments.area;
        }
    }

    if face_count == 0 {
        return Err(BrepMassPropertiesError::NoFaces);
    }
    if !exceeds(total.volume, 0.0) {
        return Err(BrepMassPropertiesError::NonPositiveVolume {
            volume: total.volume,
        });
    }

    let volume = total.volume;
    let centroid = total.first / volume;
    // Second-moment matrix about the origin.
    let second = Matrix3::new(
        total.diag.x,
        total.off.x,
        total.off.z,
        total.off.x,
        total.diag.y,
        total.off.y,
        total.off.z,
        total.off.y,
        total.diag.z,
    );
    // I_origin: diagonal Ixx = ∫(y² + z²) = tr(S) − S_xx, off-diagonal
    // Ixy = −S_xy — both captured by tr(S)·E − S.
    let inertia_origin = Matrix3::identity() * second.trace() - second;
    // Parallel-axis shift to the centroid: I_c = I_o − m·(|d|²·E − d·dᵀ).
    let inertia = inertia_origin
        - (Matrix3::identity() * centroid.norm_squared() - centroid * centroid.transpose())
            * volume;

    Ok(MassProperties {
        volume,
        surface_area: total.area,
        centroid: Point3::from(centroid),
        inertia,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::massprops::mass_properties;
    use opensolid_brep::{TessellationOptions, primitives, tessellate_body, translate_body};
    use std::f64::consts::PI;

    /// The analytic path is exact up to floating-point accumulation, so the
    /// closed-form checks are held to a genuinely tight bound.
    const EXACT: f64 = 1e-9;

    struct Scene {
        store: TopologyStore,
        geo: GeometryStore,
    }

    impl Scene {
        fn new() -> Self {
            Scene {
                store: TopologyStore::new(),
                geo: GeometryStore::new(),
            }
        }

        fn measure(&self, body: EntityId<Body>) -> MassProperties {
            brep_mass_properties(&self.store, &self.geo, body)
                .unwrap_or_else(|e| panic!("brep mass properties: {e}"))
        }

        /// The same body measured the *other* way: tessellate, then integrate
        /// the triangles.
        fn measure_meshed(&self, body: EntityId<Body>, angular_step: f64) -> MassProperties {
            let mesh = tessellate_body(
                &self.store,
                &self.geo,
                body,
                &TessellationOptions { angular_step },
            )
            .expect("tessellation");
            assert!(mesh.is_closed_manifold(), "tessellation is not closed");
            mass_properties(&mesh).expect("mesh mass properties")
        }
    }

    fn assert_rel(got: f64, want: f64, rtol: f64, what: &str) {
        let scale = want.abs().max(1e-300);
        assert!(
            ((got - want) / scale).abs() <= rtol,
            "{what}: {got} vs expected {want} (relative {:.3e}, allowed {rtol:.1e})",
            ((got - want) / scale).abs()
        );
    }

    fn assert_diagonal_inertia(mp: &MassProperties, want: [f64; 3], rtol: f64, what: &str) {
        for i in 0..3 {
            assert_rel(
                mp.inertia[(i, i)],
                want[i],
                rtol,
                &format!("{what} I[{i}{i}]"),
            );
            for j in 0..3 {
                if i != j {
                    let scale = want[i].abs().max(want[j].abs());
                    assert!(
                        mp.inertia[(i, j)].abs() <= rtol * scale,
                        "{what}: product of inertia I[{i}{j}] = {} (allowed {:.3e})",
                        mp.inertia[(i, j)],
                        rtol * scale
                    );
                }
            }
        }
    }

    // -----------------------------------------------------------------
    // Closed-form cross-checks, one per analytic surface family.
    // -----------------------------------------------------------------

    #[test]
    fn block_matches_closed_form() {
        let mut scene = Scene::new();
        let (a, b, c) = (2.0, 3.0, 5.0);
        let body = primitives::block(&mut scene.store, &mut scene.geo, a, b, c).unwrap();
        let mp = scene.measure(body);

        let m = a * b * c;
        assert_rel(mp.volume, m, EXACT, "block volume");
        assert_rel(
            mp.surface_area,
            2.0 * (a * b + b * c + c * a),
            EXACT,
            "block area",
        );
        assert!(mp.centroid.coords.norm() < EXACT * (a + b + c));
        assert_diagonal_inertia(
            &mp,
            [
                m / 12.0 * (b * b + c * c),
                m / 12.0 * (a * a + c * c),
                m / 12.0 * (a * a + b * b),
            ],
            EXACT,
            "block",
        );
    }

    #[test]
    fn cylinder_matches_closed_form() {
        let mut scene = Scene::new();
        let (r, h) = (1.5, 4.0);
        let body = primitives::cylinder(&mut scene.store, &mut scene.geo, r, h).unwrap();
        let mp = scene.measure(body);

        let m = PI * r * r * h;
        assert_rel(mp.volume, m, EXACT, "cylinder volume");
        assert_rel(
            mp.surface_area,
            2.0 * PI * r * r + 2.0 * PI * r * h,
            EXACT,
            "cylinder area",
        );
        assert!(mp.centroid.coords.norm() < EXACT * (r + h));
        let transverse = m * (3.0 * r * r + h * h) / 12.0;
        assert_diagonal_inertia(
            &mp,
            [transverse, transverse, m * r * r / 2.0],
            EXACT,
            "cylinder",
        );
    }

    #[test]
    fn sphere_matches_closed_form() {
        let mut scene = Scene::new();
        let r = 1.25;
        let body = primitives::sphere(&mut scene.store, &mut scene.geo, r).unwrap();
        let mp = scene.measure(body);

        let m = 4.0 / 3.0 * PI * r * r * r;
        assert_rel(mp.volume, m, EXACT, "sphere volume");
        assert_rel(mp.surface_area, 4.0 * PI * r * r, EXACT, "sphere area");
        assert!(mp.centroid.coords.norm() < EXACT * r);
        let i = 0.4 * m * r * r;
        assert_diagonal_inertia(&mp, [i, i, i], EXACT, "sphere");
    }

    #[test]
    fn torus_matches_closed_form() {
        let mut scene = Scene::new();
        let (big, small) = (3.0, 0.75);
        let body = primitives::torus(&mut scene.store, &mut scene.geo, big, small).unwrap();
        let mp = scene.measure(body);

        let m = 2.0 * PI * PI * big * small * small;
        assert_rel(mp.volume, m, EXACT, "torus volume");
        assert_rel(
            mp.surface_area,
            4.0 * PI * PI * big * small,
            EXACT,
            "torus area",
        );
        assert!(mp.centroid.coords.norm() < EXACT * big);
        // Solid torus about its own axis: Izz = m(R² + ¾r²); about a diameter
        // Ixx = Iyy = m(½R² + ⅝r²).
        let axial = m * (big * big + 0.75 * small * small);
        let diametral = m * (0.5 * big * big + 0.625 * small * small);
        assert_diagonal_inertia(&mp, [diametral, diametral, axial], EXACT, "torus");
    }

    #[test]
    fn pointed_cone_matches_closed_form() {
        let mut scene = Scene::new();
        let (r, h) = (2.0, 3.0);
        // Base radius r at z = -h/2, apex at z = +h/2.
        let body = primitives::cone(&mut scene.store, &mut scene.geo, r, 0.0, h).unwrap();
        let mp = scene.measure(body);

        let m = PI * r * r * h / 3.0;
        assert_rel(mp.volume, m, EXACT, "cone volume");
        // Base disc plus the lateral surface π·r·slant.
        let slant = (r * r + h * h).sqrt();
        assert_rel(
            mp.surface_area,
            PI * r * r + PI * r * slant,
            EXACT,
            "cone area",
        );
        // Centroid a quarter of the height up from the base.
        assert_rel(mp.centroid.z, -h / 2.0 + h / 4.0, EXACT, "cone centroid z");
        assert!(mp.centroid.coords.xy().norm() < EXACT * r);
        let transverse = m * (3.0 / 20.0 * r * r + 3.0 / 80.0 * h * h);
        assert_diagonal_inertia(
            &mp,
            [transverse, transverse, 3.0 / 10.0 * m * r * r],
            EXACT,
            "cone",
        );
    }

    #[test]
    fn frustum_matches_closed_form() {
        let mut scene = Scene::new();
        let (r1, r2, h) = (2.0, 0.8, 3.0);
        let body = primitives::cone(&mut scene.store, &mut scene.geo, r1, r2, h).unwrap();
        let mp = scene.measure(body);

        let m = PI * h * (r1 * r1 + r1 * r2 + r2 * r2) / 3.0;
        assert_rel(mp.volume, m, EXACT, "frustum volume");
        let slant = ((r1 - r2) * (r1 - r2) + h * h).sqrt();
        assert_rel(
            mp.surface_area,
            PI * (r1 * r1 + r2 * r2) + PI * (r1 + r2) * slant,
            EXACT,
            "frustum area",
        );
        // Centroid height above the base: h(R₁² + 2R₁R₂ + 3R₂²) / (4(R₁²+R₁R₂+R₂²)).
        let above_base =
            h * (r1 * r1 + 2.0 * r1 * r2 + 3.0 * r2 * r2) / (4.0 * (r1 * r1 + r1 * r2 + r2 * r2));
        assert_rel(
            mp.centroid.z,
            -h / 2.0 + above_base,
            EXACT,
            "frustum centroid z",
        );
    }

    /// A cone whose larger cap is on top: the wall surface's frame axis then
    /// points along `-Z`, reversing which way its `u` runs. Sign handling
    /// must not care.
    #[test]
    fn inverted_cone_matches_closed_form() {
        let mut scene = Scene::new();
        let (r, h) = (2.0, 3.0);
        let body = primitives::cone(&mut scene.store, &mut scene.geo, 0.0, r, h).unwrap();
        let mp = scene.measure(body);

        let m = PI * r * r * h / 3.0;
        assert_rel(mp.volume, m, EXACT, "inverted cone volume");
        assert_rel(mp.centroid.z, h / 2.0 - h / 4.0, EXACT, "inverted centroid");
    }

    // -----------------------------------------------------------------
    // Cross-checks against the mesh path and invariance properties.
    // -----------------------------------------------------------------

    #[test]
    fn agrees_with_the_mesh_path_on_every_primitive() {
        let mut scene = Scene::new();
        let bodies = [
            primitives::block(&mut scene.store, &mut scene.geo, 2.0, 3.0, 5.0).unwrap(),
            primitives::cylinder(&mut scene.store, &mut scene.geo, 1.5, 4.0).unwrap(),
            primitives::sphere(&mut scene.store, &mut scene.geo, 1.25).unwrap(),
            primitives::torus(&mut scene.store, &mut scene.geo, 3.0, 0.75).unwrap(),
            primitives::cone(&mut scene.store, &mut scene.geo, 2.0, 0.8, 3.0).unwrap(),
        ];
        // A fine tessellation still discretizes both curved directions; 0.5%
        // is the same budget the boolean stress suite allows curved solids.
        let step = std::f64::consts::TAU / 256.0;
        for (index, body) in bodies.into_iter().enumerate() {
            let exact = scene.measure(body);
            let meshed = scene.measure_meshed(body, step);
            assert_rel(
                meshed.volume,
                exact.volume,
                5e-3,
                &format!("body {index} volume"),
            );
            assert_rel(
                meshed.surface_area,
                exact.surface_area,
                5e-3,
                &format!("body {index} area"),
            );
            let scale = exact.volume.cbrt();
            assert!(
                (meshed.centroid - exact.centroid).norm() <= 5e-3 * scale,
                "body {index} centroid: {:?} vs {:?}",
                meshed.centroid,
                exact.centroid
            );
            for i in 0..3 {
                assert_rel(
                    meshed.inertia[(i, i)],
                    exact.inertia[(i, i)],
                    1e-2,
                    &format!("body {index} I[{i}{i}]"),
                );
            }
        }
    }

    /// The mesh path converges *to* the exact path as the tessellation is
    /// refined — the strongest statement that the two measure the same solid.
    #[test]
    fn mesh_path_converges_to_the_exact_path() {
        let mut scene = Scene::new();
        let body = primitives::sphere(&mut scene.store, &mut scene.geo, 1.25).unwrap();
        let exact = scene.measure(body).volume;

        let coarse = (scene
            .measure_meshed(body, std::f64::consts::TAU / 32.0)
            .volume
            - exact)
            .abs();
        let fine = (scene
            .measure_meshed(body, std::f64::consts::TAU / 64.0)
            .volume
            - exact)
            .abs();
        // Halving the angular step is second-order: expect ≥3× better, well
        // inside the 4× the theory promises.
        assert!(
            fine * 3.0 < coarse,
            "refining did not converge: coarse error {coarse:.3e}, fine {fine:.3e}"
        );
    }

    /// Volume and inertia are translation invariant (the latter about the
    /// centroid); the centroid rides along. Moving a body far from the origin
    /// is what exercises the parallel-axis shift and the cancellation in the
    /// signed integrals.
    #[test]
    fn translation_moves_the_centroid_and_nothing_else() {
        let mut scene = Scene::new();
        let body = primitives::cylinder(&mut scene.store, &mut scene.geo, 1.5, 4.0).unwrap();
        let here = scene.measure(body);

        let offset = Vector3::new(120.0, -75.0, 43.5);
        translate_body(&mut scene.store, &mut scene.geo, body, offset).unwrap();
        let there = scene.measure(body);

        assert_rel(there.volume, here.volume, EXACT, "translated volume");
        assert_rel(
            there.surface_area,
            here.surface_area,
            EXACT,
            "translated area",
        );
        assert!(
            (there.centroid - (here.centroid + offset)).norm() < 1e-8,
            "centroid {:?} did not follow the offset",
            there.centroid
        );
        for i in 0..3 {
            assert_rel(
                there.inertia[(i, i)],
                here.inertia[(i, i)],
                1e-8,
                &format!("translated I[{i}{i}]"),
            );
        }
    }

    // -----------------------------------------------------------------
    // Rejections.
    // -----------------------------------------------------------------

    /// A body's stored trim geometry and a fresh projection must measure the
    /// same solid. They do not agree on *branches* — [`attach_body_pcurves`]
    /// splits a seam's two fins onto the low and high branch as it writes
    /// them, while a fit puts both on the low one — so this is the check that
    /// the seam placement is derived from the geometry rather than assumed
    /// from whichever convention happened to be in the store.
    #[test]
    fn stored_trim_geometry_measures_the_same_as_a_fresh_fit() {
        use opensolid_brep::attach_body_pcurves;

        let mut scene = Scene::new();
        let bodies = [
            primitives::cylinder(&mut scene.store, &mut scene.geo, 1.5, 4.0).unwrap(),
            primitives::sphere(&mut scene.store, &mut scene.geo, 1.25).unwrap(),
            primitives::torus(&mut scene.store, &mut scene.geo, 3.0, 0.75).unwrap(),
            primitives::cone(&mut scene.store, &mut scene.geo, 2.0, 0.8, 3.0).unwrap(),
        ];
        let fitted: Vec<MassProperties> = bodies.iter().map(|&b| scene.measure(b)).collect();

        for &body in &bodies {
            let attached = attach_body_pcurves(&mut scene.store, &mut scene.geo, body);
            assert!(attached > 0, "no trim geometry was attached");
        }

        for (index, (&body, before)) in bodies.iter().zip(&fitted).enumerate() {
            let after = scene.measure(body);
            assert_rel(
                after.volume,
                before.volume,
                1e-12,
                &format!("body {index} volume from stored pcurves"),
            );
            assert_rel(
                after.surface_area,
                before.surface_area,
                1e-12,
                &format!("body {index} area from stored pcurves"),
            );
            assert!(
                (after.centroid - before.centroid).norm() < 1e-9 * before.volume.cbrt(),
                "body {index} centroid moved: {:?} vs {:?}",
                after.centroid,
                before.centroid
            );
        }
    }

    /// The four shapes an edge's parameter range arrives in, read against a
    /// closed curve: an ordinary interval, a wrap through the domain end, and
    /// the two ways the boolean pipeline spells "the whole loop" when it
    /// cannot unwrap one — the range collapsed onto either end of the domain.
    #[test]
    fn a_range_that_wraps_a_closed_curve_is_read_as_the_arc_it_traces() {
        const TAU: f64 = std::f64::consts::TAU;
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let circle = geo
            .add_curve(Curve3::circle(Point3::origin(), Vector3::z(), 1.0).expect("valid circle"));
        let vertex = store.create_vertex(Point3::new(1.0, 0.0, 0.0), 1e-10);
        let edge_id = store.create_edge_with_curve(vertex, vertex, 1e-10, circle, 0.0, TAU);

        /// A stored range and the pieces it should be read as.
        type Case = ((f64, f64), &'static [(f64, f64)]);
        let cases: [Case; 4] = [
            ((0.5, 2.0), &[(0.5, 2.0)]),
            ((5.0, 1.0), &[(5.0, TAU), (0.0, 1.0)]),
            ((0.0, 0.0), &[(0.0, TAU)]),
            ((TAU, TAU), &[(0.0, TAU)]),
        ];
        for ((t_start, t_end), want) in cases {
            let edge = store.edges.get_mut(edge_id).expect("live edge");
            edge.t_start = t_start;
            edge.t_end = t_end;
            let edge = store.edge(edge_id).expect("live edge");
            let got = edge_ranges(edge, geo.curve(circle)).expect("a closed curve can wrap");
            assert_eq!(got.len(), want.len(), "[{t_start}, {t_end}] -> {got:?}");
            for (got, want) in got.iter().zip(want) {
                assert!(
                    (got.0 - want.0).abs() < 1e-12 && (got.1 - want.1).abs() < 1e-12,
                    "[{t_start}, {t_end}] -> {got:?}, want {want:?}"
                );
            }
        }
    }

    /// And the whole measurement still runs when an edge arrives spelled that
    /// way: the primitive's own cap circles, re-stamped as the empty range
    /// the boolean pipeline hands over for a closed imprint loop, must
    /// measure exactly as before.
    #[test]
    fn a_degenerate_range_on_a_closed_curve_measures_as_before() {
        let mut scene = Scene::new();
        let body = primitives::cylinder(&mut scene.store, &mut scene.geo, 1.5, 4.0).unwrap();
        let before = scene.measure(body);

        // Each circle bounds both a cap and the wall, so dedupe the walk.
        let circles: std::collections::HashSet<_> = scene
            .store
            .faces_of_body(body)
            .into_iter()
            .flat_map(|f| scene.store.edges_of_face(f))
            .filter(|&e| {
                scene
                    .store
                    .edge(e)
                    .is_some_and(|edge| edge.t_end - edge.t_start > 6.0)
            })
            .collect();
        assert_eq!(circles.len(), 2, "expected the two cap circles");
        for edge_id in circles {
            let edge = scene.store.edges.get_mut(edge_id).expect("live edge");
            edge.t_end = edge.t_start;
        }

        let after = scene.measure(body);
        assert_rel(after.volume, before.volume, 1e-12, "wrapped-range volume");
        assert_rel(
            after.surface_area,
            before.surface_area,
            1e-12,
            "wrapped-range area",
        );
    }

    /// The same empty range on a curve whose ends do *not* meet traces
    /// nothing, and is reported rather than quietly measured as zero.
    #[test]
    fn an_empty_range_on_an_open_curve_is_rejected() {
        let mut scene = Scene::new();
        let body = primitives::block(&mut scene.store, &mut scene.geo, 2.0, 3.0, 5.0).unwrap();
        let edge_id = scene
            .store
            .edges_of_face(scene.store.faces_of_body(body)[0])[0];
        let edge = scene.store.edges.get_mut(edge_id).expect("live edge");
        edge.t_end = edge.t_start;

        let failure = brep_mass_properties(&scene.store, &scene.geo, body)
            .expect_err("an empty range on a line bounds nothing");
        assert!(
            matches!(failure, BrepMassPropertiesError::EmptyEdgeRange { .. }),
            "unexpected failure: {failure}"
        );
    }

    /// A seam fin left on the wrong branch leaves the face's boundary open in
    /// parameter space, and the gap is not a line the surface collapses. The
    /// measurement must say so rather than return the plausible-looking
    /// number the broken region would integrate to.
    #[test]
    fn an_open_parameter_loop_is_rejected() {
        let mut scene = Scene::new();
        let body = primitives::cylinder(&mut scene.store, &mut scene.geo, 1.5, 4.0).unwrap();
        // Shorten the seam edge so the wall's boundary no longer reaches the
        // top cap: the loop now has a real hole in it, mid-surface.
        let seam = scene
            .store
            .faces_of_body(body)
            .into_iter()
            .flat_map(|f| scene.store.edges_of_face(f))
            .find(|&e| {
                scene
                    .store
                    .edge(e)
                    .is_some_and(|edge| (edge.t_end - edge.t_start - 4.0).abs() < 1e-9)
            })
            .expect("the axial seam");
        scene.store.edges.get_mut(seam).expect("live edge").t_end = 2.5;

        let failure = brep_mass_properties(&scene.store, &scene.geo, body)
            .expect_err("a wall whose seam stops short bounds nothing well-defined");
        assert!(
            matches!(failure, BrepMassPropertiesError::OpenParameterLoop { .. }),
            "unexpected failure: {failure}"
        );
    }

    #[test]
    fn body_without_faces_is_rejected() {
        let mut store = TopologyStore::new();
        let geo = GeometryStore::new();
        let body = store.create_body(opensolid_brep::BodyType::Solid);
        assert_eq!(
            brep_mass_properties(&store, &geo, body),
            Err(BrepMassPropertiesError::NoFaces)
        );
    }

    #[test]
    fn face_without_a_surface_is_rejected() {
        let mut store = TopologyStore::new();
        let geo = GeometryStore::new();
        let body = store.create_body(opensolid_brep::BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);
        let face = store.create_face(shell, FaceSense::Positive);
        assert_eq!(
            brep_mass_properties(&store, &geo, body),
            Err(BrepMassPropertiesError::MissingSurface { face })
        );
    }
}
