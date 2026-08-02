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
//! # Freeform trim ([`Curve2::Nurbs`], of-50u)
//!
//! Since of-3qy.8 retired the B-spline mesh fallback, freeform faces reach
//! the exact path, and their trim geometry is genuinely a NURBS in
//! `(u, v)`. [`Curve2::Nurbs`] holds it. It arrives two ways:
//!
//! - **Fitted.** [`fit_pcurve`] interpolates its samples with a cubic
//!   B-spline wherever the surface has no closed-form inverse to build a
//!   [`Curve2::Projected`] from — a [`Surface3::Nurbs`]. The fit passes
//!   through every sample at that sample's own parameter (so the module
//!   invariant holds there exactly, as a polyline's did) and its error
//!   between samples is fourth order in the spacing, against the polyline's
//!   second — the same 33 samples that bought 1.3e-3 of a trimmed region
//!   buy ~1e-9 of it. The interpolation parameters ride along in the
//!   variant, so a consumer that must not over-claim (the checker) knows
//!   where the invariant is exact and where it is approximated.
//! - **Transplanted.** The STEP reader adopts an authored 2D B-spline trim
//!   verbatim when its parameterization provably lines up with the kernel's
//!   (freeform curve on a freeform surface — see the reader), which
//!   preserves the author's exact trim rather than a refit. Such a curve
//!   claims the invariant everywhere, and carries no fit parameters.
//!
//! Neither is a correctness problem even when approximate: a fitted pcurve
//! describes a curve that really does lie on the surface at its samples,
//! and no consumer is told more than the variant records.
//!
//! # Why a fitted pcurve is not enough (of-y8qc)
//!
//! `Line` and `Circle` between them cover the pcurves an *axis-aligned*
//! model produces, and nothing else. Tilt one operand and they stop
//! covering anything: a plane section of a sphere is a `v = const` latitude
//! only while the cutting plane is perpendicular to the pole axis, and at
//! any other angle its image in `(u, v)` is a transcendental curve that
//! neither variant can hold. Every such trim used to land on the polyline
//! fallback, whose region error is second order in the sample spacing —
//! 1.3e-3 of the volume of a spherical cap trimmed at 90°, against 5.7e-16
//! for the *congruent* cap trimmed at 0°.
//!
//! Sampling harder does not fix that: second order means 1e-9 costs some
//! 36 000 vertices per trim. [`Curve2::Projected`] fixes it by not fitting.
//! Every analytic surface here inverts in closed form — a sphere's `(u, v)`
//! is a longitude and a latitude of the query point — so the exact image of
//! the edge curve is available at any `t` for about the cost of an
//! `atan2`, and the samples are demoted to what they are actually needed
//! for: choosing the *branch* of a periodic direction, which only has to be
//! right to within half a period.
//!
//! [`Fin`]: crate::topology::Fin

use crate::curve::{Curve3, CurveEval, TWO_PI};
use crate::nurbs::{KnotVector, NurbsCurve2};
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

/// How far a [`Curve2::Projected`] branch guide is allowed to move in one
/// sample interval, as a fraction of the direction's period.
///
/// The guide's whole job is to say which representative of a periodic
/// parameter the exact inverse belongs to, and it does that correctly as
/// long as it is nearer the truth than half a period. A quarter period per
/// interval leaves the linear interpolation between two exact samples at
/// most an eighth of a period out, which is margin enough that no realistic
/// trim can cross it.
const GUIDE_STEP_FRACTION: f64 = 0.25;

/// Ceiling on branch-guide vertices, so refinement of a pathological curve
/// terminates. Reaching it means the guide is coarser than
/// [`GUIDE_STEP_FRACTION`] somewhere, which costs a branch choice rather
/// than accuracy — and an edge whose parameter winds that fast in `(u, v)`
/// is under-resolved everywhere else in the pipeline too.
const GUIDE_MAX_SAMPLES: usize = 4096;

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
/// - `Projected`: parameterized by the 3D curve it is the image of, which
///   is the module invariant stated as a definition rather than as a
///   property to be maintained.
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
    /// The exact image of a 3D curve on a surface, evaluated by inverting
    /// the surface at the curve's point instead of by fitting anything.
    ///
    /// Boxed because it owns a curve, a surface and the guide's two
    /// vectors, and inlining that would grow every `Curve2`.
    Projected(Box<ProjectedCurve2>),
    /// Freeform trim: a NURBS in `(u, v)`, parameterized by the edge's own
    /// parameter like every other variant.
    ///
    /// `fit_params` records where the curve is an *interpolant*: non-empty
    /// (strictly increasing, one entry per construction sample) for a curve
    /// [`fit_pcurve`] fitted, whose invariant claim is exact at those
    /// parameters and fourth-order in their spacing between them. Empty for
    /// a curve that claims the invariant at every parameter — an authored
    /// trim the STEP reader verified and transplanted verbatim.
    Nurbs {
        curve: Box<NurbsCurve2>,
        fit_params: Vec<f64>,
    },
}

/// The payload of [`Curve2::Projected`]: everything needed to invert a
/// surface at a curve's point and land on the right branch.
///
/// # What makes it exact
///
/// `point(t)` is `surface⁻¹(curve(t))`, computed rather than approximated.
/// Every analytic [`Surface3`] has a closed-form inverse (a longitude and a
/// latitude, an angle and a height), which
/// [`project_point`](SurfaceProject::project_point) returns directly for a
/// point that already lies on the surface. So the module invariant —
/// `surface.point(pcurve(t)) == curve.point(t)` — holds to floating point at
/// *every* `t`, not only at sampled ones, and the region a loop of these
/// bounds is the true one.
///
/// # What the guide is for
///
/// An inverse is only unique up to a period: `u` and `u + 2π` name the same
/// point on a cylinder, and a face's loop is a closed cycle in `(u, v)` only
/// if each fin takes the representative its neighbours continue. Closed-form
/// inversion cannot know which, so the guide — the same sampled polyline the
/// fallback fit would have used, refined until no interval moves more than
/// [`GUIDE_STEP_FRACTION`] of a period — says. The exact value snaps to the
/// representative nearest the guide, which is right whenever the guide is
/// within half a period, and never contributes its own error to the result.
///
/// `offset` carries whole-period translations applied after construction
/// (a seam fin moved to the far branch, a loop stitched for continuity),
/// kept separate from the guide so that shifting is exact and reversible.
///
/// # What it costs
///
/// The curve and the surface are owned copies, because a pcurve outlives
/// any particular walk of the topology and [`Curve2`] has no handle on the
/// [`GeometryStore`](crate::geometry::GeometryStore) to borrow them from.
/// For the analytic pairings this variant exists to serve that is a
/// stack-sized struct each; a [`Curve3::Nurbs`] trim on an analytic surface
/// copies its control net, which is the one case where the polyline
/// fallback was cheaper. Evaluation costs one curve evaluation and one
/// closed-form inversion, against a polyline's binary search and lerp.
#[derive(Debug, Clone, PartialEq)]
pub struct ProjectedCurve2 {
    /// The 3D curve this is the image of, in its own parameter.
    curve: Box<Curve3>,
    /// The surface inverted at each of the curve's points.
    surface: Box<Surface3>,
    /// Guide parameters, strictly increasing, spanning the fitted range.
    guide_params: Vec<f64>,
    /// Guide vertices, unwrapped across periodic branch cuts and *not*
    /// carrying `offset`.
    guide_points: Vec<Point2>,
    /// Parameter-space translation applied after inversion.
    offset: Vector2,
}

impl ProjectedCurve2 {
    /// The 3D curve this is the image of.
    pub fn curve(&self) -> &Curve3 {
        &self.curve
    }

    /// The surface the curve is inverted against.
    pub fn surface(&self) -> &Surface3 {
        &self.surface
    }

    /// Parameter-space translation applied after inversion.
    pub fn offset(&self) -> Vector2 {
        self.offset
    }

    /// The branch guide as a plain polyline, `offset` included — what a
    /// consumer that cannot evaluate an inverse (the STEP writer) should
    /// fall back to, and where the panel breaks of a consumer that can
    /// (`brep_massprops`) belong.
    pub fn guide(&self) -> Curve2 {
        Curve2::Polyline {
            params: self.guide_params.clone(),
            points: self.guide_points.iter().map(|p| p + self.offset).collect(),
        }
    }

    /// Guide parameters — the sample breaks, for a consumer that wants to
    /// panel between them without materializing [`Self::guide`].
    pub fn guide_params(&self) -> &[f64] {
        &self.guide_params
    }

    /// A copy translated by `shift` in parameter space.
    pub fn shifted(&self, shift: Vector2) -> Self {
        let mut out = self.clone();
        out.offset += shift;
        out
    }

    /// Linear interpolation of the guide at `t`, on the unwrapped branch and
    /// without `offset`.
    fn guide_point(&self, t: f64) -> Point2 {
        let (i, frac) = polyline_segment(&self.guide_params, t);
        self.guide_points[i] + (self.guide_points[i + 1] - self.guide_points[i]) * frac
    }

    /// The exact inverse at `t`, on the guide's branch, without `offset`.
    fn inverse(&self, t: f64) -> Point2 {
        let guide = self.guide_point(t);
        let periods = (self.surface.period_u(), self.surface.period_v());
        guide_sample(&self.surface, &self.curve, t, guide, periods)
    }
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
        check_samples(&params, &points)?;
        Ok(Curve2::Polyline { params, points })
    }

    /// The image of `curve` on `surface`, evaluated by inversion rather than
    /// fitted, with `params`/`points` the branch guide (see
    /// [`ProjectedCurve2`]).
    ///
    /// The guide is *not* the geometry — it only picks the representative of
    /// a periodic parameter — so it is accepted at whatever resolution the
    /// caller sampled, subject to the same well-formedness a polyline needs.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if fewer than two guide vertices are
    /// given, if the two vectors differ in length, or if `params` is not
    /// strictly increasing and finite.
    pub fn projected(
        surface: &Surface3,
        curve: &Curve3,
        params: Vec<f64>,
        points: Vec<Point2>,
    ) -> CoreResult<Self> {
        check_samples(&params, &points)?;
        Ok(Curve2::Projected(Box::new(ProjectedCurve2 {
            curve: Box::new(curve.clone()),
            surface: Box::new(surface.clone()),
            guide_params: params,
            guide_points: points,
            offset: Vector2::zeros(),
        })))
    }

    /// A freeform trim curve that claims the module invariant at every
    /// parameter — for a caller that has *verified* the claim, the way the
    /// STEP reader samples an authored trim against its edge before
    /// transplanting it. A fitted interpolant should come from
    /// [`fit_pcurve`] instead, which records its fit parameters.
    pub fn nurbs(curve: NurbsCurve2) -> Self {
        Curve2::Nurbs {
            curve: Box::new(curve),
            fit_params: Vec::new(),
        }
    }
}

/// The well-formedness every sampled pcurve needs: at least two vertices,
/// one parameter each, and parameters finite and strictly increasing.
fn check_samples(params: &[f64], points: &[Point2]) -> CoreResult<()> {
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
    Ok(())
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
            Curve2::Projected(p) => p.inverse(t) + p.offset,
            Curve2::Nurbs { curve, .. } => curve.point(t),
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
            // Differentiating `S(u(t), v(t)) = C(t)` gives
            // `S_u u' + S_v v' = C'`, an overdetermined 3x2 system whose
            // normal equations are the surface's first fundamental form:
            //
            //   [E F; F G] [u'; v'] = [C'·S_u; C'·S_v]
            //
            // `EG − F² = |S_u × S_v|²`, so the solve is exactly as
            // well-posed as the parameterization is — it is singular only
            // where the surface itself is, and there `u'` is genuinely
            // undefined (every `u` names the same point at a pole) and the
            // guide's slope is the honest answer.
            Curve2::Projected(p) => {
                let uv = p.inverse(t);
                let guide_slope = || {
                    let (i, _) = polyline_segment(&p.guide_params, t);
                    (p.guide_points[i + 1] - p.guide_points[i])
                        / (p.guide_params[i + 1] - p.guide_params[i])
                };
                if p.surface.is_singular(uv.x, uv.y) {
                    return guide_slope();
                }
                let (su, sv) = (p.surface.du(uv.x, uv.y), p.surface.dv(uv.x, uv.y));
                let (e, f, g) = (su.norm_squared(), su.dot(&sv), sv.norm_squared());
                let det = e * g - f * f;
                let d3 = p.curve.derivative(t);
                let (a, b) = (d3.dot(&su), d3.dot(&sv));
                let duv = Vector2::new((g * a - f * b) / det, (e * b - f * a) / det);
                // NaN-safe: a non-finite component fails the test and defers.
                if det > 0.0 && duv.x.is_finite() && duv.y.is_finite() {
                    duv
                } else {
                    guide_slope()
                }
            }
            Curve2::Nurbs { curve, .. } => curve.derivative(t),
        }
    }

    fn domain(&self) -> (f64, f64) {
        match self {
            Curve2::Line { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            Curve2::Circle { .. } => (0.0, TWO_PI),
            Curve2::Polyline { params, .. } => (params[0], params[params.len() - 1]),
            Curve2::Projected(p) => (p.guide_params[0], p.guide_params[p.guide_params.len() - 1]),
            Curve2::Nurbs { curve, .. } => curve.domain(),
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
/// unwrapped across branch cuts (each sample takes the representative
/// nearest its predecessor, so a curve that crosses the seam runs
/// continuously past it rather than jumping a full period). A cut is either
/// a periodic direction's or the join of a clamped patch that closes on
/// itself — see [`Surface3::wrap_period_u`]. The samples are then fitted, in
/// order of preference, as a [`Curve2::Line`] or a [`Curve2::Circle`];
/// failing that they become the branch guide of a [`Curve2::Projected`];
/// where the pairing has no closed-form inverse, a fitted [`Curve2::Nurbs`]
/// interpolating the samples, or — for a [`Curve3::Polyline`], whose chords
/// are its geometry — a [`Curve2::Polyline`] through the samples themselves.
///
/// The first two cover essentially every *axis-aligned* analytic pairing —
/// a line or circle on a plane, a cylinder's generators and cross-sections,
/// a sphere's meridians and latitudes, a torus's two circle families, and
/// every seam — exactly, not to a tolerance. `Projected` covers everything
/// else on an analytic surface just as exactly, which is what tilting one
/// operand needs (see the module header, of-y8qc). Only a freeform surface
/// is left on a fit, whose between-sample error is fourth order in the
/// sample spacing (see the module header, of-50u).
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
    if is_projectable(surface, curve) {
        let (params, points) = refine_branch_guide(surface, curve, params, points);
        return Curve2::projected(surface, curve, params, points);
    }
    // A smooth curve on a freeform surface: interpolate the samples with a
    // cubic instead of chording them, cutting the between-sample error from
    // second order in the spacing to fourth. A `Curve3::Polyline` stays on
    // the polyline, which is not a downgrade — the curve itself is piecewise
    // linear, so the chords are the geometry.
    if !matches!(curve, Curve3::Polyline { .. })
        && let Some(nurbs) = fit_nurbs_pcurve(&params, &points)
    {
        return Ok(nurbs);
    }
    Curve2::polyline(params, points)
}

/// Degree of the interpolating fit below. Cubic is the conventional choice
/// (Piegl & Tiller §9.2.1): high enough for fourth-order error, low enough
/// that the interpolant cannot oscillate past what the samples support.
const FIT_NURBS_DEGREE: usize = 3;

/// Interpolate the sampled parameter-space points with a B-spline that
/// passes through every sample at that sample's own parameter — global
/// curve interpolation (Piegl & Tiller §9.2.1), with the knot vector by
/// parameter averaging (their eq. 9.8) so the collocation system is
/// guaranteed nonsingular (Schoenberg–Whitney).
///
/// The knots being averages of the curve's *own* parameters is what keeps
/// the module invariant: the interpolant is parameterized by the edge
/// parameter directly, exact at the samples, fourth-order between them.
///
/// `None` if there are too few samples for the degree or the solve fails —
/// the averaged knot vector makes the matrix nonsingular in exact
/// arithmetic, but a caller gets the polyline fallback rather than a panic
/// if floating point disagrees.
fn fit_nurbs_pcurve(params: &[f64], points: &[Point2]) -> Option<Curve2> {
    let n = points.len();
    let p = FIT_NURBS_DEGREE;
    if n < p + 1 {
        return None;
    }
    let mut knots = vec![params[0]; p + 1];
    knots.extend((1..=(n - 1 - p)).map(|j| params[j..j + p].iter().sum::<f64>() / p as f64));
    knots.extend(std::iter::repeat_n(params[n - 1], p + 1));
    let kv = KnotVector::new(p, knots).ok()?;

    // Collocation matrix: row k holds the p + 1 basis functions alive at
    // t_k. Banded, but n is FIT_SAMPLES — dense LU costs nothing here.
    let mut matrix = nalgebra::DMatrix::<f64>::zeros(n, n);
    let mut rhs = nalgebra::DMatrix::<f64>::zeros(n, 2);
    for (k, (&t, q)) in params.iter().zip(points).enumerate() {
        let span = kv.find_span(t);
        for (j, &b) in kv.basis_funs(span, t).iter().enumerate() {
            matrix[(k, span - p + j)] = b;
        }
        rhs[(k, 0)] = q.x;
        rhs[(k, 1)] = q.y;
    }
    let solution = matrix.lu().solve(&rhs)?;
    let control_points: Vec<Point2> = (0..n)
        .map(|i| Point2::new(solution[(i, 0)], solution[(i, 1)]))
        .collect();
    // The constructor's finiteness check is the guard against a solve that
    // technically succeeded but overflowed.
    let curve = NurbsCurve2::bspline(control_points, kv).ok()?;
    Some(Curve2::Nurbs {
        curve: Box::new(curve),
        fit_params: params.to_vec(),
    })
}

/// Whether this pairing can be inverted exactly at an arbitrary parameter,
/// which is what a [`Curve2::Projected`] promises.
///
/// Two things have to hold, and both are about the *inverse* rather than
/// about how nice the geometry looks:
///
/// - The surface must have a closed-form inverse. Every analytic variant
///   does; [`Surface3::Nurbs`] does not, and iterating a Newton search per
///   evaluation would trade a bounded error for an unbounded cost and a
///   convergence question. Freeform pairings take the fitted
///   [`Curve2::Nurbs`] instead (of-50u).
/// - The curve must be smooth, so that the image has a derivative to report
///   between guide vertices. A [`Curve3::Polyline`] is not, and it gains
///   nothing anyway: it already interpolates its own vertices linearly, so
///   the parameter-space polyline through those same vertices is the same
///   order of approximation as anything else could be.
fn is_projectable(surface: &Surface3, curve: &Curve3) -> bool {
    !matches!(surface, Surface3::Nurbs(_)) && !matches!(curve, Curve3::Polyline { .. })
}

/// Subdivide the sampled parameters until no interval moves more than
/// [`GUIDE_STEP_FRACTION`] of a period, so the guide can be trusted to name
/// the branch anywhere between its vertices.
///
/// Only periodic directions are refined, because only they have a branch to
/// choose: along an aperiodic direction the inverse is unique and the guide
/// is inert. Nothing here improves the pcurve's *accuracy* — the exact
/// inverse does that — so a curve whose samples are already within the step
/// (which is nearly all of them) pays one comparison per interval.
///
/// Each pass re-unwraps the whole guide, which is what makes this terminate
/// and what makes it *fix* branch mistakes rather than merely avoid new
/// ones. Unwrapping takes every vertex to the representative nearest its
/// predecessor, so afterwards no interval spans more than half a period —
/// and the initial uniform samples of a curve that swings hard can be a
/// hair the wrong side of that, which is precisely the case refining is for.
fn refine_branch_guide(
    surface: &Surface3,
    curve: &Curve3,
    mut params: Vec<f64>,
    mut points: Vec<Point2>,
) -> (Vec<f64>, Vec<Point2>) {
    let (period_u, period_v) = (surface.period_u(), surface.period_v());
    let step = |period: Option<f64>| period.map(|p| p * GUIDE_STEP_FRACTION);
    let (step_u, step_v) = (step(period_u), step(period_v));
    if step_u.is_none() && step_v.is_none() {
        return (params, points);
    }
    let too_far = |a: &Point2, b: &Point2| {
        step_u.is_some_and(|s| (b.x - a.x).abs() > s)
            || step_v.is_some_and(|s| (b.y - a.y).abs() > s)
    };

    while points.len() < GUIDE_MAX_SAMPLES {
        let mut split = false;
        // Backwards, so an insertion never moves an interval still to visit.
        for i in (1..points.len()).rev() {
            if points.len() >= GUIDE_MAX_SAMPLES {
                break;
            }
            if !too_far(&points[i - 1], &points[i]) {
                continue;
            }
            let mid = 0.5 * (params[i - 1] + params[i]);
            // A parameter interval too narrow to have a midpoint of its own
            // cannot be split further, whatever it does in `(u, v)`.
            if !(mid > params[i - 1] && mid < params[i]) {
                continue;
            }
            let uv = guide_sample(surface, curve, mid, points[i - 1], (period_u, period_v));
            params.insert(i, mid);
            points.insert(i, uv);
            split = true;
        }
        if !split {
            break;
        }
        unwrap_in_place(&mut points, period_u, period_v);
    }
    (params, points)
}

/// Take every vertex after the first to the representative nearest its
/// predecessor, leaving the first where it is (so a seam shift, which moves
/// the whole guide together, survives).
fn unwrap_in_place(points: &mut [Point2], period_u: Option<f64>, period_v: Option<f64>) {
    for i in 1..points.len() {
        let previous = points[i - 1];
        points[i].x = nearest_representative(points[i].x, previous.x, period_u);
        points[i].y = nearest_representative(points[i].y, previous.y, period_v);
    }
}

/// One inversion of `surface` at `curve(t)`, taking the representative
/// nearest `reference` and, at a parameterization singularity,
/// `reference`'s own `u` — for the reason [`repair_singular_samples`] gives,
/// and because at a singularity every `u` maps to the same point, so keeping
/// the guide's costs the invariant nothing and keeps the pcurve continuous
/// through it.
///
/// Used both to place a guide vertex and to evaluate the finished
/// [`Curve2::Projected`], which is what makes the guide self-consistent:
/// evaluating at a vertex's own parameter reproduces that vertex.
fn guide_sample(
    surface: &Surface3,
    curve: &Curve3,
    t: f64,
    reference: Point2,
    (period_u, period_v): (Option<f64>, Option<f64>),
) -> Point2 {
    let projection = surface.project_point_seeded(&curve.point(t), (reference.x, reference.y));
    let v = nearest_representative(projection.v, reference.y, period_v);
    let u = if surface.is_singular(projection.u, projection.v) {
        reference.x
    } else {
        nearest_representative(projection.u, reference.x, period_u)
    };
    Point2::new(u, v)
}

/// Sample `curve` over `[t_start, t_end]` and invert each sample onto
/// `surface`, unwrapping the wrapping directions so the result is continuous
/// and then putting a clamped patch's samples back on its knot rectangle.
fn sample_parameter_space(
    surface: &Surface3,
    curve: &Curve3,
    t_start: f64,
    t_end: f64,
    seam: SeamSide,
) -> (Vec<f64>, Vec<Point2>) {
    // Both branch cuts a walk may cross: a periodic parameterization's, and
    // the join of a clamped patch that closes on itself. The second is why
    // this is not `period_u`/`period_v` — see [`Surface3::wrap_period_u`].
    let (period_u, period_v) = (surface.wrap_period_u(), surface.wrap_period_v());
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
    recenter_on_knot_rectangle(surface, &mut points, period_u, period_v);
    if seam == SeamSide::High {
        shift_to_high_branch(&mut points, period_u, period_v);
    }
    (params, points)
}

/// Shift the unwrapped samples of a closed **NURBS** patch by whole periods
/// until they sit on its knot rectangle.
///
/// Unwrapping keeps a walk continuous across a branch cut, at the price of
/// letting it leave the domain: a boundary curve that starts *on* the join
/// of a closed patch is projected to whichever of the two equal ends the
/// search happened to return, and if that is the low one the rest of the
/// walk trails off below it, down to a full period out.
///
/// For a periodic parameterization that costs nothing, because evaluation is
/// periodic too — a cylinder's `u = 7` is its `u = 7 − 2π` and both are the
/// same point. A clamped patch has no such luxury: outside its knot
/// rectangle it does not extrapolate but *clamps*, so a `u` a period low
/// evaluates to the domain edge and the pcurve reads as a curve that folds
/// onto the seam. Hence the shift, and hence it applies only where
/// evaluation is bounded.
///
/// A run already on the rectangle is left exactly where it is — including
/// one lying *on* either end of the cut, which is what a seam edge is and
/// which [`shift_to_high_branch`] is then free to move deliberately. Only a
/// run that has left the rectangle is moved, and it is recentred rather than
/// dragged in by its nearest sample, so a run that genuinely straddles the
/// join — which a face boundary should not do without a seam edge, but which
/// nothing here can rule out — ends up minimally outside on both sides
/// instead of wholly outside on one.
fn recenter_on_knot_rectangle(
    surface: &Surface3,
    points: &mut [Point2],
    period_u: Option<f64>,
    period_v: Option<f64>,
) {
    if !matches!(surface, Surface3::Nurbs(_)) {
        return;
    }
    let axes = [
        (period_u, surface.domain_u(), true),
        (period_v, surface.domain_v(), false),
    ];
    for (period, (lo, hi), is_u) in axes {
        let Some(period) = period else { continue };
        let component = |p: &Point2| if is_u { p.x } else { p.y };
        let (min, max) = points
            .iter()
            .map(component)
            .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), c| {
                (min.min(c), max.max(c))
            });
        // NaN-safe: a non-finite sample leaves the fold's sentinels in place
        // and there is nothing meaningful to shift by.
        if !(min >= lo && max <= hi) && min.is_finite() && max.is_finite() {
            let shift = period * ((0.5 * (lo + hi) - 0.5 * (min + max)) / period).round();
            for p in points.iter_mut() {
                if is_u {
                    p.x += shift;
                } else {
                    p.y += shift;
                }
            }
        }
    }
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
pub(crate) fn nearest_representative(value: f64, reference: f64, period: Option<f64>) -> f64 {
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

    /// Exact NURBS cylinder of radius 1 about the z-axis, `v ∈ [0, 1]`
    /// mapping to `z ∈ [0, 2]`, with the control ring wound **clockwise** so
    /// that a counterclockwise edge curve runs in *decreasing* `u`. Closed
    /// in `u`: first and last control row coincide.
    fn clockwise_nurbs_cylinder() -> Surface3 {
        let ring: Vec<(f64, f64)> = [
            (1.0, 0.0),
            (1.0, -1.0),
            (0.0, -1.0),
            (-1.0, -1.0),
            (-1.0, 0.0),
            (-1.0, 1.0),
            (0.0, 1.0),
            (1.0, 1.0),
            (1.0, 0.0),
        ]
        .to_vec();
        let control_points: Vec<Vec<Point3>> = ring
            .iter()
            .map(|&(x, y)| vec![Point3::new(x, y, 0.0), Point3::new(x, y, 2.0)])
            .collect();
        let s = std::f64::consts::FRAC_1_SQRT_2;
        let weights: Vec<Vec<f64>> = [1.0, s, 1.0, s, 1.0, s, 1.0, s, 1.0]
            .iter()
            .map(|&w| vec![w, w])
            .collect();
        let knots_u = crate::nurbs::KnotVector::new(
            2,
            vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ],
        )
        .expect("valid knots");
        let knots_v = crate::nurbs::KnotVector::clamped_uniform(1, 2).expect("valid knots");
        Surface3::nurbs(
            crate::nurbs::NurbsSurface::new(control_points, weights, knots_u, knots_v)
                .expect("valid patch"),
        )
    }

    /// of-fid: a boundary curve of a closed NURBS patch, starting *on* the
    /// join and running away from it in decreasing `u`.
    ///
    /// The two ends of the `u` domain are the same point, so the first
    /// sample may be projected to either; take the low one and every later
    /// sample wants a `u` below the domain. Before of-fid the clamp pinned
    /// them all at `u_min` and the pcurve ran a whole diameter from its own
    /// edge — on a patch the edge lies exactly on.
    #[test]
    fn a_closed_patch_boundary_starting_on_the_join_tracks_its_edge() {
        let surface = clockwise_nurbs_cylinder();
        // The top rim, counterclockwise from (1, 0, 2) — which is the join.
        let curve = Curve3::circle(Point3::new(0.0, 0.0, 2.0), Vector3::z(), 1.0).expect("valid");
        let pcurve = fit_pcurve(&surface, &curve, 0.0, TWO_PI, SeamSide::Low).expect("fits");
        assert_invariant(&surface, &curve, &pcurve, (0.0, TWO_PI));

        // And it stays on the knot rectangle, which is the only place a
        // clamped patch evaluates rather than saturating.
        let (u_lo, u_hi) = surface.domain_u();
        for i in 0..=32 {
            let uv = pcurve.point(TWO_PI * (i as f64) / 32.0);
            assert!(
                uv.x >= u_lo - TOL && uv.x <= u_hi + TOL,
                "sample {i} left the u domain at {}",
                uv.x
            );
            assert!(
                (uv.y - 1.0).abs() < TOL,
                "the rim holds v = 1, got {}",
                uv.y
            );
        }
    }

    /// The recentering shift moves the whole run or nothing: an in-domain
    /// pcurve on the same patch must come back untouched.
    #[test]
    fn a_closed_patch_boundary_away_from_the_join_is_left_alone() {
        let surface = clockwise_nurbs_cylinder();
        let curve = Curve3::circle(Point3::new(0.0, 0.0, 2.0), Vector3::z(), 1.0).expect("valid");
        // A quarter of the rim, well clear of the join at both ends.
        let (t0, t1) = (0.5, 2.0);
        let pcurve = fit_pcurve(&surface, &curve, t0, t1, SeamSide::Low).expect("fits");
        assert_invariant(&surface, &curve, &pcurve, (t0, t1));
        let (u_lo, u_hi) = surface.domain_u();
        for i in 0..=16 {
            let u = pcurve.point(t0 + (t1 - t0) * (i as f64) / 16.0).x;
            assert!(u > u_lo && u < u_hi, "sample {i} left the u domain at {u}");
        }
    }

    /// The seam edge of a closed patch lies *on* the cut, so its samples sit
    /// at one end of the `u` domain — inside it, and exactly at the shift's
    /// tie point. Recentering must leave them alone, or the two fins of the
    /// seam would both be handed the same branch and the face's boundary
    /// would not close in parameter space.
    #[test]
    fn a_closed_patch_seam_keeps_its_two_branches() {
        let surface = clockwise_nurbs_cylinder();
        // A generator: the ruling at the join, running up the patch.
        let curve = Curve3::line(Point3::new(1.0, 0.0, 0.0), Vector3::z()).expect("valid");
        let low = fit_pcurve(&surface, &curve, 0.0, 2.0, SeamSide::Low).expect("fits");
        let high = fit_pcurve(&surface, &curve, 0.0, 2.0, SeamSide::High).expect("fits");
        assert_invariant(&surface, &curve, &low, (0.0, 2.0));
        assert_invariant(&surface, &curve, &high, (0.0, 2.0));

        let (u_lo, u_hi) = surface.domain_u();
        let period = u_hi - u_lo;
        for i in 0..=8 {
            let t = 2.0 * (i as f64) / 8.0;
            let (a, b) = (low.point(t).x, high.point(t).x);
            assert!(
                (b - a - period).abs() < TOL,
                "the two seam branches must sit a period apart, got {a} and {b}"
            );
        }
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

    // --- Curve2::Projected --------------------------------------------

    /// The `t = 0` end of a `Curve3::Circle` is fixed by [`plane_basis`], so
    /// the frame the test wants has to be built explicitly. This is the
    /// circle a plane at distance `1 - h` from the centre cuts on a unit
    /// sphere whose pole axis is `tilt` radians off the cutting plane's
    /// normal, i.e. the of-y8qc configuration.
    fn tilted_sphere_cut(tilt: f64, h: f64) -> (Surface3, Curve3, f64) {
        let axis = Vector3::new(tilt.cos(), 0.0, tilt.sin());
        let sphere = Surface3::sphere(Point3::origin(), axis, 1.0).expect("valid");
        let offset = 1.0 - h;
        let radius = (1.0 - offset * offset).sqrt();
        // The cut is the plane x = offset, whose normal is +x throughout.
        let circle = Curve3::Ellipse {
            center: Point3::new(offset, 0.0, 0.0),
            axis: Vector3::x(),
            major_dir: Vector3::y(),
            major_radius: radius,
            minor_radius: radius,
        };
        (sphere, circle, radius)
    }

    /// A latitude fits exactly as a line; tilt the same cut off the pole
    /// axis and its image in `(u, v)` is transcendental, which is where
    /// of-y8qc's 1.3e-3 came from. `Projected` is the answer, and it holds
    /// the invariant to floating point rather than to a sample spacing.
    #[test]
    fn a_tilted_sphere_cut_is_inverted_exactly_instead_of_fitted() {
        let (sphere, circle, _) = tilted_sphere_cut(0.0, 0.95);
        let pole_aligned = fit_pcurve(&sphere, &circle, 0.0, TWO_PI, SeamSide::Low).expect("fits");
        assert!(
            matches!(pole_aligned, Curve2::Line { .. }),
            "a latitude is still a line, got {pole_aligned:?}"
        );

        for tilt_deg in [1.0f64, 15.0, 45.0, 75.0, 90.0] {
            let (sphere, circle, _) = tilted_sphere_cut(tilt_deg.to_radians(), 0.95);
            let pcurve = fit_pcurve(&sphere, &circle, 0.0, TWO_PI, SeamSide::Low).expect("fits");
            assert!(
                matches!(pcurve, Curve2::Projected(_)),
                "{tilt_deg}°: expected an inverted pcurve, got {pcurve:?}"
            );
            // Deliberately off the guide's vertices: 257 samples against a
            // 33-sample guide, so almost none of these were ever projected
            // while the pcurve was built. A fit would show its error here.
            for i in 0..=256 {
                let t = TWO_PI * (i as f64) / 256.0;
                let uv = pcurve.point(t);
                let gap = (sphere.point(uv.x, uv.y) - circle.point(t)).norm();
                assert!(gap < 1e-14, "{tilt_deg}° at t = {t}: off by {gap:e}");
            }
        }
    }

    /// The derivative comes from the first fundamental form, not from the
    /// guide's chords — so it is the true tangent even where the guide's
    /// slope is badly wrong (the 90° cut swings nearly the whole `u` range
    /// between two adjacent samples).
    #[test]
    fn an_inverted_pcurve_differentiates_by_the_metric() {
        let (sphere, circle, _) = tilted_sphere_cut(std::f64::consts::FRAC_PI_2, 0.95);
        let pcurve = fit_pcurve(&sphere, &circle, 0.0, TWO_PI, SeamSide::Low).expect("fits");
        let eps = 1e-6;
        for i in 0..=64 {
            let t = 0.1 + (TWO_PI - 0.2) * (i as f64) / 64.0;
            let central = (pcurve.point(t + eps) - pcurve.point(t - eps)) / (2.0 * eps);
            let analytic = pcurve.derivative(t);
            let scale = analytic.norm().max(1.0);
            assert!(
                (analytic - central).norm() < 1e-5 * scale,
                "at t = {t}: analytic {analytic} vs central difference {central}"
            );
        }
    }

    /// A trim that crosses the seam has to keep climbing past `2π` rather
    /// than snapping back, and the guide is what tells inversion — which
    /// only ever returns the principal branch — which representative to
    /// take.
    #[test]
    fn an_inverted_pcurve_stays_on_one_branch_across_the_seam() {
        // Tilted so the pcurve is not a line or circle, and ranged so it
        // sweeps a full turn starting away from t = 0.
        let (sphere, circle, _) = tilted_sphere_cut(std::f64::consts::FRAC_PI_4, 0.95);
        let (t0, t1) = (0.7, 0.7 + TWO_PI);
        let pcurve = fit_pcurve(&sphere, &circle, t0, t1, SeamSide::Low).expect("fits");
        assert!(matches!(pcurve, Curve2::Projected(_)));
        assert_invariant(&sphere, &circle, &pcurve, (t0, t1));

        // Continuous: no step anywhere near a whole period.
        let mut previous = pcurve.point(t0);
        for i in 1..=2048 {
            let t = t0 + (t1 - t0) * (i as f64) / 2048.0;
            let uv = pcurve.point(t);
            assert!(
                (uv - previous).norm() < 0.5,
                "at t = {t}: jumped from {previous} to {uv}"
            );
            previous = uv;
        }
        // A closed trim comes back to where it started, one period along.
        let (start, end) = (pcurve.point(t0), pcurve.point(t1));
        assert!(
            ((end.x - start.x).abs() - TWO_PI).abs() < 1e-9 && (end.y - start.y).abs() < 1e-9,
            "start {start} and end {end} should differ by exactly one period in u"
        );
    }

    /// Shifting rides on the offset, so the exact inverse is translated
    /// afterwards rather than the guide being asked to carry it.
    #[test]
    fn shifting_an_inverted_pcurve_translates_it_exactly() {
        let (sphere, circle, _) = tilted_sphere_cut(std::f64::consts::FRAC_PI_3, 0.95);
        let pcurve = fit_pcurve(&sphere, &circle, 0.0, TWO_PI, SeamSide::Low).expect("fits");
        let Curve2::Projected(p) = &pcurve else {
            panic!("expected an inverted pcurve, got {pcurve:?}");
        };
        let shift = Vector2::new(TWO_PI, 0.25);
        let moved = Curve2::Projected(Box::new(p.shifted(shift)));
        for i in 0..=32 {
            let t = TWO_PI * (i as f64) / 32.0;
            assert_close(moved.point(t), pcurve.point(t) + shift);
            assert_close(
                Point2::from(moved.derivative(t)),
                Point2::from(pcurve.derivative(t)),
            );
        }
        assert_eq!(moved.domain(), pcurve.domain());
    }

    /// A plane section of a sphere never needs refining, however sharply it
    /// swings: cut just past the centre, `u` covers half its range within a
    /// couple of thousandths of `t`, and the two uniform samples straddling
    /// that still land a quarter period apart, because the pile-up is
    /// bounded by the geometry (the samples either side of the crossing sit
    /// at `π ∓ atan(δ/a)` for the cut's offset `δ`, which cannot span more
    /// than `π/2`). So the uniform samples are the answer here, and the test
    /// says so rather than leaving it to chance.
    #[test]
    fn uniform_samples_already_name_the_branch_for_a_sphere_cut() {
        let (sphere, circle, _) = tilted_sphere_cut(std::f64::consts::FRAC_PI_2, 1.001);
        let pcurve = fit_pcurve(&sphere, &circle, 0.0, TWO_PI, SeamSide::Low).expect("fits");
        let Curve2::Projected(p) = &pcurve else {
            panic!("expected an inverted pcurve, got {pcurve:?}");
        };
        assert_eq!(p.guide_params().len(), FIT_SAMPLES);
        assert_invariant(&sphere, &circle, &pcurve, (0.0, TWO_PI));
    }

    /// Started from a guide too coarse to name anything — three vertices
    /// over a full turn, the middle one deliberately a whole period off the
    /// branch its neighbours are on — refinement does both of its jobs: it
    /// subdivides until every interval is inside the step, and it re-unwraps
    /// as it goes, so the planted vertex is pulled back onto the right
    /// branch instead of dragging a period-wide discontinuity through the
    /// finished pcurve.
    #[test]
    fn refining_the_guide_subdivides_and_repairs_the_branch() {
        let (sphere, circle, _) = tilted_sphere_cut(std::f64::consts::FRAC_PI_2, 1.001);
        let coarse = vec![0.0, std::f64::consts::PI, TWO_PI];
        let mut points: Vec<Point2> = coarse
            .iter()
            .map(|&t| {
                let p = sphere.project_point(&circle.point(t));
                Point2::new(p.u, p.v)
            })
            .collect();
        // Continuous to start with, apart from the planted mistake.
        unwrap_in_place(&mut points, sphere.period_u(), sphere.period_v());
        points[1].x += TWO_PI;

        let (params, points) = refine_branch_guide(&sphere, &circle, coarse, points);
        assert!(
            params.len() > 3,
            "a three-vertex guide over a full turn must be subdivided"
        );
        for w in points.windows(2) {
            assert!(
                (w[1].x - w[0].x).abs() <= TWO_PI * GUIDE_STEP_FRACTION + TOL,
                "guide step {} exceeds a quarter period",
                (w[1].x - w[0].x).abs()
            );
        }

        // The guide is only a guide, so the proof it is good enough is that
        // the pcurve built on it is exact and continuous.
        let pcurve = Curve2::projected(&sphere, &circle, params, points).expect("valid");
        assert_invariant(&sphere, &circle, &pcurve, (0.0, TWO_PI));

        // This cut's `u` genuinely moves fast — most of a radian within a
        // few thousandths of `t` — so continuity is stated as "no step of
        // half a period", which only a branch mistake can produce, and
        // pinned down at the ends: this trim runs out along the branch cut
        // and back rather than winding, so it must close on the `u` it
        // started from, and a stray branch would leave it a period away.
        let mut previous = pcurve.point(0.0);
        for i in 1..=1024 {
            let t = TWO_PI * (i as f64) / 1024.0;
            let uv = pcurve.point(t);
            assert!(
                (uv - previous).norm() < std::f64::consts::PI,
                "at t = {t}: jumped from {previous} to {uv}"
            );
            previous = uv;
        }
        assert_close(previous, pcurve.point(0.0));
    }

    /// The bulged biquadratic patch the freeform fit tests share: nothing
    /// about it is a plane, and it has no closed-form inverse.
    fn bulged_patch() -> Surface3 {
        use crate::nurbs::{KnotVector, NurbsSurface};
        let control = vec![
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 2.0, 1.0),
                Point3::new(0.0, 4.0, 0.0),
            ],
            vec![
                Point3::new(2.0, 0.0, 1.0),
                Point3::new(2.0, 2.0, 3.0),
                Point3::new(2.0, 4.0, 1.0),
            ],
            vec![
                Point3::new(4.0, 0.0, 0.0),
                Point3::new(4.0, 2.0, 1.0),
                Point3::new(4.0, 4.0, 0.0),
            ],
        ];
        let knots = KnotVector::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).expect("valid knots");
        let patch = NurbsSurface::bspline(control, knots.clone(), knots).expect("valid patch");
        Surface3::Nurbs(Box::new(patch))
    }

    /// A NURBS surface has no closed-form inverse, so a smooth curve on it
    /// gets the fitted interpolant: exact at every sample's own parameter,
    /// fourth-order between them (of-50u).
    #[test]
    fn a_freeform_surface_gets_a_fitted_nurbs_pcurve() {
        let surface = bulged_patch();
        // A straight line in model space is not a straight line in this
        // patch's parameters.
        let curve =
            Curve3::line(Point3::new(0.2, 0.3, 6.0), Vector3::new(1.0, 0.9, 0.0)).expect("valid");
        let pcurve = fit_pcurve(&surface, &curve, 0.0, 3.0, SeamSide::Low).expect("fits");
        let Curve2::Nurbs { fit_params, .. } = &pcurve else {
            panic!("expected the fitted NURBS, got {pcurve:?}");
        };
        assert_eq!(fit_params.len(), FIT_SAMPLES);

        // At its own parameters the fit is an interpolant: the projection
        // that produced the samples is the only error left.
        for &t in fit_params {
            let uv = pcurve.point(t);
            let miss = surface
                .point(uv.x, uv.y)
                .coords
                .metric_distance(&surface.project_point(&curve.point(t)).point.coords);
            assert!(miss < 1e-7, "at fit parameter {t}: missed by {miss}");
        }
        // Between them the interpolant stays close to the projected truth —
        // the fourth-order bound, measured where a chord would be worst.
        for i in 0..FIT_SAMPLES - 1 {
            let t = 0.5 * (fit_params[i] + fit_params[i + 1]);
            let uv = pcurve.point(t);
            let projected = surface.project_point(&curve.point(t));
            let miss = (surface.point(uv.x, uv.y) - projected.point).norm();
            assert!(miss < 1e-6, "between samples at t = {t}: missed by {miss}");
        }
    }

    /// A cubic interpolant reproduces anything already in its own space: a
    /// curve whose `(u, v)` image is a parabola comes back exact at *every*
    /// parameter, not only the sampled ones.
    #[test]
    fn the_fitted_nurbs_reproduces_a_polynomial_image_exactly() {
        use crate::nurbs::{KnotVector, NurbsCurve, NurbsSurface};

        // Bilinear patch S(u, v) = (u, v, uv)...
        let control = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 0.0)],
            vec![Point3::new(1.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0)],
        ];
        let kv1 = KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).expect("valid");
        let patch = NurbsSurface::bspline(control, kv1.clone(), kv1).expect("valid patch");
        let surface = Surface3::Nurbs(Box::new(patch));

        // ...and the twisted cubic c(t) = (t, t², t³), which lies on it
        // along (u, v) = (t, t²) — a parabola no line or circle fits.
        let kv3 = KnotVector::new(3, vec![0.0; 4].into_iter().chain(vec![1.0; 4]).collect())
            .expect("valid");
        let cubic = NurbsCurve::bspline(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0 / 3.0, 0.0, 0.0),
                Point3::new(2.0 / 3.0, 1.0 / 3.0, 0.0),
                Point3::new(1.0, 1.0, 1.0),
            ],
            kv3,
        )
        .expect("valid curve");
        let curve = Curve3::nurbs(cubic);

        let pcurve = fit_pcurve(&surface, &curve, 0.0, 1.0, SeamSide::Low).expect("fits");
        assert!(
            matches!(pcurve, Curve2::Nurbs { .. }),
            "expected the fitted NURBS, got {pcurve:?}"
        );
        for i in 0..=257 {
            let t = i as f64 / 257.0;
            assert_close(pcurve.point(t), Point2::new(t, t * t));
        }
        assert_invariant(&surface, &curve, &pcurve, (0.0, 1.0));
    }

    /// A `Curve3::Polyline` stays on the polyline pcurve even on a freeform
    /// surface: its chords *are* the geometry, so interpolating a smooth
    /// curve through them would claim smoothness the edge does not have.
    #[test]
    fn a_polyline_curve_on_a_freeform_surface_keeps_the_polyline() {
        let surface = bulged_patch();
        let line =
            Curve3::line(Point3::new(0.2, 0.3, 6.0), Vector3::new(1.0, 0.9, 0.0)).expect("valid");
        let points: Vec<Point3> = (0..=8).map(|i| line.point(3.0 * i as f64 / 8.0)).collect();
        let curve = Curve3::polyline(points, false).expect("valid polyline");
        let pcurve = fit_pcurve(&surface, &curve, 0.0, 8.0, SeamSide::Low).expect("fits");
        assert!(
            matches!(pcurve, Curve2::Polyline { .. }),
            "expected the polyline fallback, got {pcurve:?}"
        );
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
