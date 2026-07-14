//! Marching surface-surface intersection (transversal MVP).
//!
//! [`intersect_nurbs`] traces the intersection curves of two
//! [`NurbsSurface`]s as dense polylines (fitting the result as a NURBS
//! curve is a later hardening pass), per spec/02-geometry.md §5.3 at the
//! spec/12 "minimum viable" level. [`intersect_marched`] runs the same
//! tracer over analytic primitive pairs whose intersection has no
//! closed form among the [`crate::curve::Curve3`] variants (see its docs
//! for the pair table); both share the algorithm below:
//!
//! 1. **Seeding** — a parameter grid over surface A is evaluated and each
//!    node is projected onto surface B ([`SurfaceProject`], of-uui.5); the
//!    sign of the oriented distance `(P_a - foot_b) · n_b` changes across
//!    the intersection locus, so every sign-changing grid edge yields a
//!    candidate that a least-norm 4D Newton refines onto the intersection.
//! 2. **Marching** — from each seed, a predictor step of arc length `h`
//!    along `t = n_a × n_b` (mapped to parameter increments through each
//!    surface's first fundamental form) followed by a Newton corrector on
//!    `S_a(u_a, v_a) - S_b(u_b, v_b) = 0` constrained to the plane through
//!    the predicted point perpendicular to `t`. The step halves when the
//!    corrector fails and recovers gradually on success.
//! 3. **Termination** — at domain boundaries (the final point is corrected
//!    with the crossing parameter pinned to its bound), on closure back to
//!    the seed (closed curves repeat their first point at the end), or at
//!    the step-count safety cap.
//!
//! **Transversal only.** Near-tangential contact — surface normals closer
//! than [`NEAR_TANGENCY_SIN`] anywhere on the traced locus — aborts with
//! [`CoreError::Degenerate`] carrying the location and the measured
//! `|n_a × n_b|`. Branch points, tangent curves, and coincident regions
//! are the hardening pass of spec/12. Surfaces that do not intersect
//! return an empty curve set, not an error.

use crate::curve::{TWO_PI, plane_basis};
use crate::nurbs::NurbsSurface;
use crate::project::SurfaceProject;
use crate::surface::{Surface3, SurfaceEval};
use nalgebra::{Matrix3, Matrix4, Vector4};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::types::{Point3, Vector3};

/// Tangency threshold on `|n_a × n_b|` (the sine of the angle between the
/// unit normals). Below this the marching tangent is numerically
/// meaningless and the configuration is treated as (near-)tangential:
/// the transversal MVP refuses crossings shallower than ~0.06°.
pub const NEAR_TANGENCY_SIN: f64 = 1e-3;

/// Grid divisions per parameter direction for seed detection. Fine enough
/// that a transversal curve crossing the domain flips the oriented
/// distance sign on at least one grid edge.
const GRID_DIVISIONS: usize = 16;

/// Newton iteration cap for seed refinement and marching correction.
const MAX_CORRECTOR_ITERATIONS: usize = 12;

/// Marching step cap per traced direction (spec/02 §5.3 step 4e).
const MAX_STEPS: usize = 10_000;

/// Consecutive step halvings allowed before a branch is abandoned.
const MAX_STEP_HALVINGS: usize = 6;

/// Surface access required by the marching tracer: first-order evaluation
/// for the stepping frames, plus closest-point projection
/// ([`SurfaceProject`]) and normals ([`SurfaceEval`]) for grid seeding.
trait MarchSurface: SurfaceEval + SurfaceProject {
    /// Point and first partials `(S, S_u, S_v)` at `(u, v)`.
    fn eval1(&self, u: f64, v: f64) -> (Point3, Vector3, Vector3) {
        (self.point(u, v), self.du(u, v), self.dv(u, v))
    }
}

impl MarchSurface for NurbsSurface {
    fn eval1(&self, u: f64, v: f64) -> (Point3, Vector3, Vector3) {
        // Fused basis evaluation: one derivatives() call instead of three.
        let d = self.derivatives(u, v, 1);
        (Point3::origin() + d[0][0], d[1][0], d[0][1])
    }
}

impl MarchSurface for Surface3 {}

/// One intersection curve traced by marching, as a dense polyline with
/// the parameter preimages on both surfaces.
///
/// The three vectors are parallel: `points[i] = a.point(params_a[i])`
/// and lies within the marching gap tolerance of `b.point(params_b[i])`.
/// Closed curves repeat their first vertex as the last one.
#[derive(Debug, Clone, PartialEq)]
pub struct MarchedCurve {
    /// Polyline vertices on the intersection curve (evaluated on A).
    pub points: Vec<Point3>,
    /// `(u, v)` preimage of each vertex on surface A.
    pub params_a: Vec<(f64, f64)>,
    /// `(u, v)` preimage of each vertex on surface B.
    pub params_b: Vec<(f64, f64)>,
    /// Whether marching closed back onto its starting point (the first
    /// vertex is then repeated at the end). Open curves end where the
    /// march left a surface domain or stalled.
    pub closed: bool,
}

/// Marching parameter state: `[u_a, v_a, u_b, v_b]`.
#[derive(Debug, Clone, Copy)]
struct MarchState([f64; 4]);

/// First-order evaluation of both surfaces at a [`MarchState`].
struct Frames {
    pa: Point3,
    au: Vector3,
    av: Vector3,
    pb: Point3,
    bu: Vector3,
    bv: Vector3,
}

impl Frames {
    /// Residual between the surfaces (zero on the intersection).
    fn gap(&self) -> Vector3 {
        self.pa - self.pb
    }

    /// Unit normals of both surfaces; `None` where a parameterization is
    /// degenerate (collapsed partials).
    fn unit_normals(&self) -> Option<(Vector3, Vector3)> {
        let na = self.au.cross(&self.av);
        let nb = self.bu.cross(&self.bv);
        let (la, lb) = (na.norm(), nb.norm());
        if la <= f64::MIN_POSITIVE || lb <= f64::MIN_POSITIVE {
            return None;
        }
        Some((na / la, nb / lb))
    }
}

/// Why a marching direction stopped.
enum StopReason {
    /// A parameter reached its domain bound.
    Boundary,
    /// The march returned to its starting point (closed loop).
    Closed,
    /// Corrector could not recover even at the minimum step, or a
    /// parameterization degeneracy was hit. The polyline up to the last
    /// good point is kept.
    Stalled,
}

fn near_tangency_error(at: &Point3, sin: f64) -> CoreError {
    CoreError::Degenerate {
        context: "ssi::marching",
        reason: format!(
            "near-tangential intersection near ({:.6}, {:.6}, {:.6}): \
             |n_a × n_b| = {sin:.3e} is below {NEAR_TANGENCY_SIN:e}; \
             only transversal intersections are supported (spec/12 MVP)",
            at.x, at.y, at.z
        ),
    }
}

/// Marching context: the surface pair, the domain boxes to march over,
/// and the tolerances/step sizes derived from the tolerance context and
/// the geometry scale.
struct Marcher<'a, A: MarchSurface, B: MarchSurface> {
    a: &'a A,
    b: &'a B,
    /// Domain interval per parameter, indexed like [`MarchState`].
    domains: [(f64, f64); 4],
    /// Convergence tolerance on `|S_a - S_b|`.
    gap_tol: f64,
    /// Nominal marching step (3D arc length).
    h0: f64,
    /// Minimum step before a branch is declared stalled.
    h_min: f64,
}

impl<A: MarchSurface, B: MarchSurface> Marcher<'_, A, B> {
    fn frames(&self, s: &MarchState) -> Frames {
        let (pa, au, av) = self.a.eval1(s.0[0], s.0[1]);
        let (pb, bu, bv) = self.b.eval1(s.0[2], s.0[3]);
        Frames {
            pa,
            au,
            av,
            pb,
            bu,
            bv,
        }
    }

    fn clamp(&self, s: &mut MarchState) {
        for (value, (lo, hi)) in s.0.iter_mut().zip(self.domains) {
            *value = value.clamp(lo, hi);
        }
    }

    /// Unit marching tangent `n_a × n_b` and its magnitude before
    /// normalization (the sine of the normal angle).
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] when the configuration is near-tangential
    /// or a surface parameterization is degenerate at `frames`.
    fn tangent(&self, frames: &Frames) -> CoreResult<Vector3> {
        let (na, nb) = frames
            .unit_normals()
            .ok_or_else(|| near_tangency_error(&frames.pa, 0.0))?;
        let cross = na.cross(&nb);
        let sin = cross.norm();
        if sin < NEAR_TANGENCY_SIN {
            return Err(near_tangency_error(&frames.pa, sin));
        }
        Ok(cross / sin)
    }

    /// Parameter increments on both surfaces that move each surface point
    /// by the 3D displacement `w`, via each first fundamental form.
    /// `None` if a fundamental form is degenerate.
    fn predictor_deltas(&self, frames: &Frames, w: &Vector3) -> Option<[f64; 4]> {
        let solve = |su: &Vector3, sv: &Vector3| -> Option<(f64, f64)> {
            let (e, f, g) = (su.dot(su), su.dot(sv), sv.dot(sv));
            let det = e * g - f * f;
            if det <= f64::MIN_POSITIVE {
                return None;
            }
            let (r1, r2) = (su.dot(w), sv.dot(w));
            Some(((g * r1 - f * r2) / det, (e * r2 - f * r1) / det))
        };
        let (dua, dva) = solve(&frames.au, &frames.av)?;
        let (dub, dvb) = solve(&frames.bu, &frames.bv)?;
        Some([dua, dva, dub, dvb])
    }

    /// Largest fraction of the parameter step `d` that keeps every
    /// parameter inside its domain, and the index of the parameter that
    /// pins first (if any fraction < 1).
    fn boundary_scale(&self, s: &MarchState, d: &[f64; 4]) -> (f64, Option<usize>) {
        let mut scale = 1.0;
        let mut pinned = None;
        for (k, ((&dk, &pk), (lo, hi))) in d.iter().zip(s.0.iter()).zip(self.domains).enumerate() {
            let allowed = if dk > 0.0 {
                (hi - pk) / dk
            } else if dk < 0.0 {
                (lo - pk) / dk
            } else {
                continue;
            };
            let allowed = allowed.max(0.0);
            if allowed < scale {
                scale = allowed;
                pinned = Some(k);
            }
        }
        (scale, pinned)
    }

    /// Least-norm Newton refinement of a seed candidate onto the
    /// intersection: minimal parameter update solving
    /// `J·δ = -(S_a - S_b)` via `δ = Jᵀ(JJᵀ)⁻¹·(-gap)`.
    ///
    /// Returns the refined state, or `None` if the candidate does not
    /// converge (no intersection near it).
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] when the refinement lands on (or stalls
    /// against) a near-tangential contact.
    fn refine_seed(&self, start: MarchState) -> CoreResult<Option<(MarchState, Frames)>> {
        let mut s = start;
        let mut frames = self.frames(&s);
        for _ in 0..MAX_CORRECTOR_ITERATIONS {
            let gap = frames.gap();
            let dist = gap.norm();
            let sin = frames
                .unit_normals()
                .map(|(na, nb)| na.cross(&nb).norm())
                .unwrap_or(0.0);
            // Tangency check while the surfaces are essentially in
            // contact: converging onto parallel normals is exactly the
            // configuration the transversal MVP must reject.
            if sin < NEAR_TANGENCY_SIN && dist <= 100.0 * self.gap_tol {
                return Err(near_tangency_error(&frames.pa, sin));
            }
            if dist <= self.gap_tol {
                return Ok(Some((s, frames)));
            }
            let cols = [frames.au, frames.av, -frames.bu, -frames.bv];
            let mut m = Matrix3::zeros();
            for c in &cols {
                m += c * c.transpose();
            }
            let Some(y) = m.lu().solve(&(-gap)) else {
                return Ok(None);
            };
            for (value, col) in s.0.iter_mut().zip(&cols) {
                *value += col.dot(&y);
            }
            if s.0.iter().any(|v| !v.is_finite()) {
                return Ok(None);
            }
            self.clamp(&mut s);
            frames = self.frames(&s);
        }
        Ok(None)
    }

    /// Marching corrector: full Newton on the 4×4 system of the three gap
    /// equations plus the plane constraint `t · (S_a - target) = 0` that
    /// pins the corrected point to the predictor's cross-section.
    fn correct(
        &self,
        start: MarchState,
        target: &Point3,
        t: &Vector3,
    ) -> Option<(MarchState, Frames)> {
        let mut s = start;
        for _ in 0..MAX_CORRECTOR_ITERATIONS {
            let frames = self.frames(&s);
            let gap = frames.gap();
            let plane = t.dot(&(frames.pa - target));
            if gap.norm() <= self.gap_tol && plane.abs() <= self.gap_tol {
                return Some((s, frames));
            }
            let (au, av, bu, bv) = (frames.au, frames.av, frames.bu, frames.bv);
            #[rustfmt::skip]
            let jac = Matrix4::new(
                au.x,        av.x,        -bu.x, -bv.x,
                au.y,        av.y,        -bu.y, -bv.y,
                au.z,        av.z,        -bu.z, -bv.z,
                t.dot(&au),  t.dot(&av),  0.0,   0.0,
            );
            let rhs = Vector4::new(-gap.x, -gap.y, -gap.z, -plane);
            let delta = jac.lu().solve(&rhs)?;
            for (value, d) in s.0.iter_mut().zip(delta.iter()) {
                *value += d;
            }
            if s.0.iter().any(|v| !v.is_finite()) {
                return None;
            }
            self.clamp(&mut s);
        }
        None
    }

    /// Boundary corrector: parameter `pin` stays fixed at its bound, the
    /// remaining three parameters solve the (square) gap system so the
    /// final polyline vertex lies on the intersection *and* the boundary.
    fn correct_pinned(&self, start: MarchState, pin: usize) -> Option<(MarchState, Frames)> {
        let mut s = start;
        for _ in 0..MAX_CORRECTOR_ITERATIONS {
            let frames = self.frames(&s);
            let gap = frames.gap();
            if gap.norm() <= self.gap_tol {
                return Some((s, frames));
            }
            self.pinned_newton_step(&mut s, &frames, pin)?;
        }
        None
    }

    /// One Newton update of the pinned gap system: solve the 3×3 Jacobian
    /// of the gap against the three parameters other than `pin` and apply
    /// the (clamped) increment. `None` on a singular Jacobian or a
    /// non-finite iterate.
    fn pinned_newton_step(&self, s: &mut MarchState, frames: &Frames, pin: usize) -> Option<()> {
        let free: Vec<usize> = (0..4).filter(|&k| k != pin).collect();
        let cols = [frames.au, frames.av, -frames.bu, -frames.bv];
        let jac = Matrix3::from_columns(&[cols[free[0]], cols[free[1]], cols[free[2]]]);
        let delta = jac.lu().solve(&(-frames.gap()))?;
        for (i, &k) in free.iter().enumerate() {
            s.0[k] += delta[i];
        }
        if s.0.iter().any(|v| !v.is_finite()) {
            return None;
        }
        self.clamp(s);
        Some(())
    }

    /// March from `seed` with initial tangent orientation `dir` (±1),
    /// collecting states after the seed. The seed itself is not included.
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] on near-tangency along the way.
    fn trace(
        &self,
        seed: &(MarchState, Frames),
        dir: f64,
        seed_tangent: &Vector3,
    ) -> CoreResult<(Vec<(MarchState, Point3)>, StopReason)> {
        let seed_point = seed.0;
        let origin = seed.1.pa;
        let mut out = Vec::new();
        let mut state = seed_point;
        let mut frames = self.frames(&state);
        let mut prev_t = seed_tangent * dir;
        let mut h = self.h0;
        for step in 0..MAX_STEPS {
            let mut t = self.tangent(&frames)?;
            if t.dot(&prev_t) < 0.0 {
                t = -t;
            }
            let mut advanced = false;
            for _ in 0..=MAX_STEP_HALVINGS {
                let w = t * h;
                let Some(deltas) = self.predictor_deltas(&frames, &w) else {
                    return Ok((out, StopReason::Stalled));
                };
                let (scale, pinned) = self.boundary_scale(&state, &deltas);
                if let Some(pin) = pinned {
                    // The predictor leaves the domain: land exactly on the
                    // crossed bound and finish there.
                    let mut on_boundary = state;
                    for (value, d) in on_boundary.0.iter_mut().zip(deltas) {
                        *value += d * scale;
                    }
                    let (lo, hi) = self.domains[pin];
                    on_boundary.0[pin] = if deltas[pin] > 0.0 { hi } else { lo };
                    self.clamp(&mut on_boundary);
                    if let Some((s_final, f_final)) = self.correct_pinned(on_boundary, pin) {
                        out.push((s_final, f_final.pa));
                    }
                    return Ok((out, StopReason::Boundary));
                }
                let mut predicted = state;
                for (value, d) in predicted.0.iter_mut().zip(deltas) {
                    *value += d;
                }
                let target = frames.pa + w;
                if let Some((s_next, f_next)) = self.correct(predicted, &target, &t) {
                    state = s_next;
                    frames = f_next;
                    out.push((state, frames.pa));
                    prev_t = t;
                    h = (h * 1.25).min(self.h0);
                    advanced = true;
                    break;
                }
                h *= 0.5;
                if h < self.h_min {
                    return Ok((out, StopReason::Stalled));
                }
            }
            if !advanced {
                return Ok((out, StopReason::Stalled));
            }
            // Closure: back within a step of the start after enough steps
            // to rule out never having left (forward direction only; a
            // closed loop never reaches the backward trace).
            if dir > 0.0 && step >= 5 && (frames.pa - origin).norm() < 0.75 * h {
                return Ok((out, StopReason::Closed));
            }
        }
        Ok((out, StopReason::Stalled))
    }
}

/// Intersect two NURBS surfaces by grid-seeded predictor-corrector
/// marching, returning the intersection set as dense polylines.
///
/// See the module docs for the algorithm. The result is empty when the
/// surfaces do not intersect over their domains. `tol.linear` is the
/// convergence tolerance on the gap `|S_a - S_b|` at every polyline
/// vertex.
///
/// # Errors
/// [`CoreError::Degenerate`] when a (near-)tangential contact is detected
/// — normals closer than [`NEAR_TANGENCY_SIN`] on the intersection — since
/// the MVP only handles transversal crossings.
pub fn intersect_nurbs(
    a: &NurbsSurface,
    b: &NurbsSurface,
    tol: &ToleranceContext,
) -> CoreResult<Vec<MarchedCurve>> {
    march_boxed(
        a,
        b,
        [a.domain_u(), a.domain_v(), b.domain_u(), b.domain_v()],
        tol,
    )
}

/// Shared marching driver: grid-seed over `domains[0..2]` of `a`, refine
/// against `b`, and trace within the given parameter boxes (which must be
/// finite — unbounded primitive directions are clipped by the callers).
fn march_boxed<A: MarchSurface, B: MarchSurface>(
    a: &A,
    b: &B,
    domains: [(f64, f64); 4],
    tol: &ToleranceContext,
) -> CoreResult<Vec<MarchedCurve>> {
    let gap_tol = tol.linear.max(opensolid_core::SYSTEM_RESOLUTION);

    // Oriented-distance samples of A's parameter grid against B. Nodes
    // where the projection fails (rare: ambiguous/singular feet) are
    // excluded from seeding.
    let n = GRID_DIVISIONS;
    let grid_param = |k: usize, (lo, hi): (f64, f64)| lo + (hi - lo) * k as f64 / n as f64;
    let mut nodes = vec![None; (n + 1) * (n + 1)];
    let mut lo = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
    let mut hi = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);
    for i in 0..=n {
        for j in 0..=n {
            let (u, v) = (grid_param(i, domains[0]), grid_param(j, domains[1]));
            let pa = a.point(u, v);
            lo = Point3::new(lo.x.min(pa.x), lo.y.min(pa.y), lo.z.min(pa.z));
            hi = Point3::new(hi.x.max(pa.x), hi.y.max(pa.y), hi.z.max(pa.z));
            let proj = b.project_point(&pa);
            if !proj.converged {
                continue;
            }
            let Some(nb) = b.normal(proj.u, proj.v) else {
                continue;
            };
            let signed = (pa - proj.point).dot(&nb);
            nodes[i * (n + 1) + j] = Some((u, v, signed, proj.u, proj.v));
        }
    }
    let diameter = (hi - lo).norm();
    if !diameter.is_finite() || diameter <= 0.0 {
        return Ok(Vec::new());
    }

    let marcher = Marcher {
        a,
        b,
        domains,
        gap_tol,
        h0: diameter / 100.0,
        h_min: (diameter / 100.0) * 1e-3,
    };

    // Seed candidates: grid nodes already on the intersection, plus the
    // linear zero crossing of every sign-changing grid edge.
    let mut candidates: Vec<MarchState> = Vec::new();
    let node = |i: usize, j: usize| nodes[i * (n + 1) + j];
    for i in 0..=n {
        for j in 0..=n {
            let Some((u, v, d0, bu, bv)) = node(i, j) else {
                continue;
            };
            if d0.abs() <= gap_tol {
                candidates.push(MarchState([u, v, bu, bv]));
                continue;
            }
            for (i2, j2) in [(i + 1, j), (i, j + 1)] {
                if i2 > n || j2 > n {
                    continue;
                }
                let Some((u2, v2, d1, bu2, bv2)) = node(i2, j2) else {
                    continue;
                };
                if d1.abs() <= gap_tol || d0 * d1 >= 0.0 {
                    continue;
                }
                let f = d0 / (d0 - d1);
                candidates.push(MarchState([
                    u + (u2 - u) * f,
                    v + (v2 - v) * f,
                    bu + (bu2 - bu) * f,
                    bv + (bv2 - bv) * f,
                ]));
            }
        }
    }

    let mut curves: Vec<MarchedCurve> = Vec::new();
    let merge_dist = 2.0 * marcher.h0;
    for candidate in candidates {
        let Some(seed) = marcher.refine_seed(candidate)? else {
            continue;
        };
        // Skip seeds landing on an already-traced curve.
        let on_existing = curves.iter().any(|curve| {
            curve
                .points
                .iter()
                .any(|p| (p - seed.1.pa).norm() < merge_dist)
        });
        if on_existing {
            continue;
        }

        let tangent = marcher.tangent(&seed.1)?;
        let (forward, forward_stop) = marcher.trace(&seed, 1.0, &tangent)?;
        let closed = matches!(forward_stop, StopReason::Closed);
        let mut path: Vec<(MarchState, Point3)> = Vec::new();
        if closed {
            path.push((seed.0, seed.1.pa));
            path.extend(forward);
            path.push((seed.0, seed.1.pa));
        } else {
            let (backward, _) = marcher.trace(&seed, -1.0, &tangent)?;
            path.extend(backward.into_iter().rev());
            path.push((seed.0, seed.1.pa));
            path.extend(forward);
        }
        if path.len() < 2 {
            continue;
        }
        let mut curve = MarchedCurve {
            points: Vec::with_capacity(path.len()),
            params_a: Vec::with_capacity(path.len()),
            params_b: Vec::with_capacity(path.len()),
            closed,
        };
        for (s, p) in path {
            curve.points.push(p);
            curve.params_a.push((s.0[0], s.0[1]));
            curve.params_b.push((s.0[2], s.0[3]));
        }
        curves.push(curve);
    }
    Ok(curves)
}

/// Bounding sphere `(center, radius)` of a compact primitive; `None` for
/// the unbounded ones (plane, cylinder, cone).
fn bounding_sphere(s: &Surface3) -> Option<(Point3, f64)> {
    match *s {
        Surface3::Sphere { center, radius, .. } => Some((center, radius)),
        Surface3::Torus {
            center,
            major_radius,
            minor_radius,
            ..
        } => Some((center, major_radius + minor_radius)),
        _ => None,
    }
}

/// Parameter box for an unbounded primitive clipped to where it can meet a
/// partner bounded by `(center, radius)`: any intersection point lies
/// inside the partner's bounding sphere, so the infinite directions are
/// cut to that reach (slightly padded so seeding never sits exactly on a
/// box edge).
fn clipped_domains(
    s: &Surface3,
    partner: (Point3, f64),
    tol: &ToleranceContext,
) -> [(f64, f64); 2] {
    let (center, radius) = partner;
    let reach = 1.05 * radius + tol.linear;
    match s {
        Surface3::Plane { origin, normal } => {
            let (e_u, e_v) = plane_basis(normal);
            let d = center - origin;
            let (cu, cv) = (e_u.dot(&d), e_v.dot(&d));
            [(cu - reach, cu + reach), (cv - reach, cv + reach)]
        }
        Surface3::Cylinder { origin, axis, .. } | Surface3::Cone { origin, axis, .. } => {
            // The axial band reaching the partner's sphere; the apex-side of
            // a cone is clipped the same way (the parabola/hyperbola branch
            // never touches the apex, so no singular `v` is forced in).
            let t = axis.dot(&(center - origin));
            [(0.0, TWO_PI), (t - reach, t + reach)]
        }
        // Compact primitives keep their natural (finite) domains.
        _ => [s.domain_u(), s.domain_v()],
    }
}

/// Swap the per-surface parameter preimages of marched curves, for results
/// traced with the argument order reversed.
fn swap_params(curves: Vec<MarchedCurve>) -> Vec<MarchedCurve> {
    curves
        .into_iter()
        .map(|mut c| {
            std::mem::swap(&mut c.params_a, &mut c.params_b);
            c
        })
        .collect()
}

/// March `a` against `b` (grid-seeding over `a`, which must have finite
/// domains), converting the tracer's runtime tangency detection into the
/// structured `NotImplemented` of the transversal MVP.
fn march_primitives(
    a: &Surface3,
    b: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<Vec<MarchedCurve>> {
    let bounds = bounding_sphere(a).expect("marched SSI grids over a compact primitive");
    let [bu, bv] = clipped_domains(b, bounds, tol);
    let domains = [a.domain_u(), a.domain_v(), bu, bv];
    match march_boxed(a, b, domains, tol) {
        Err(CoreError::Degenerate { .. }) => Err(CoreError::NotImplemented {
            feature: "marched primitive SSI across a tangential contact or \
                      parameterization singularity (transversal MVP)",
        }),
        other => other,
    }
}

/// Marched intersection curves of two analytic primitives whose general
/// configuration has no closed form among the [`crate::curve::Curve3`]
/// variants. Supported pairs (either argument order):
///
/// - cylinder-sphere (general quartic)
/// - plane-torus (general oblique quartic / spiric sections)
/// - cylinder-torus, sphere-torus, torus-torus (degree 8 in general)
/// - sphere-cone, torus-cone (general quartic / degree 8): the cone rides
///   as the clipped partner against the compact sphere or torus, whose
///   finite domain seeds the grid
///
/// Special configurations of these pairs that *do* have closed forms
/// (coaxial arrangements, tangencies) are classified exactly by
/// [`super::intersect`]; this entry point marches whatever it is given and
/// is the fallback when that returns `NotImplemented` for a general
/// position. All other pairs are rejected: plane/sphere/cylinder
/// combinations always have conic closed forms in [`super::intersect`], and
/// the cone pairs whose *both* surfaces are unbounded — plane-cone,
/// cylinder-cone, cone-cone — carry no compact partner to seed the grid, so
/// they march through [`intersect_marched_bounded`] with an explicit region
/// of interest instead.
///
/// **Representation choice (cylinder-sphere)**: the general cylinder-sphere
/// curve admits a closed-form parameterization by cylinder angle — on the
/// cylinder `(r cos u, r sin u, z)` the sphere equation gives
/// `z(u) = z_c ± √(R² − |…|²)` — but that is not expressible with the
/// current `Curve3` variants and needs branch stitching where the two `z`
/// roots merge. Marching reuses the NURBS SSI infrastructure, yields the
/// same dense-polyline representation, and passes the same residual
/// acceptance, so the MVP marches this pair too; a dedicated analytic
/// quartic curve type is a later hardening pass.
///
/// Vertices lie on both surfaces within `tol.linear` (the marching gap
/// tolerance). Curves are transversal by construction. Periodic parameter
/// directions are traced over one period `[0, 2π]` with hard seam bounds,
/// exactly like the clamped-NURBS seam in [`intersect_nurbs`]: a closed
/// curve crossing a seam comes back as open polyline(s) ending on it.
///
/// # Errors
/// [`CoreError::NotImplemented`] for unsupported pairs, for tangential
/// contact detected while marching (including sphere-pole singularities on
/// the trace), and for the plane-torus Villarceau configuration (a plane
/// through the center at `sin θ = r/R` is bitangent, so its circles carry
/// two tangent points each).
pub fn intersect_marched(
    a: &Surface3,
    b: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<Vec<MarchedCurve>> {
    use Surface3::*;
    match (a, b) {
        // Canonical orders: grid over the compact primitive (the torus when
        // both are compact — unlike the sphere it has no parameterization
        // poles for seeds to land on). A cone is never the gridded primitive
        // here (it is unbounded); it rides as the clipped partner against a
        // compact sphere or torus.
        (Sphere { .. }, Cylinder { .. } | Cone { .. })
        | (
            Torus { .. },
            Plane { .. } | Cylinder { .. } | Sphere { .. } | Torus { .. } | Cone { .. },
        ) => {
            if let (Torus { .. }, Plane { .. }) = (a, b) {
                villarceau_check(b, a, tol)?;
            }
            march_primitives(a, b, tol)
        }
        // Swapped orders re-enter canonically and swap the preimages back.
        (Cylinder { .. } | Cone { .. }, Sphere { .. })
        | (Plane { .. } | Cylinder { .. } | Sphere { .. } | Cone { .. }, Torus { .. }) => {
            Ok(swap_params(intersect_marched(b, a, tol)?))
        }
        _ => Err(CoreError::NotImplemented {
            feature: "marched SSI for this surface pair (plane/sphere/cylinder \
                      combinations have closed forms in ssi::intersect; the \
                      unbounded cone pairs — plane-cone, cylinder-cone, \
                      cone-cone — march through ssi::intersect_marched_bounded, \
                      not here)",
        }),
    }
}

/// Marched intersection of two *unbounded* analytic primitives within an
/// explicit region of interest — the plane-cone parabola/hyperbola sections.
///
/// Unlike [`intersect_marched`], where one partner is always compact and
/// bounds the seeding grid, neither a plane nor a cone is bounded, so the
/// caller supplies the region as a bounding sphere `bounds = (center,
/// radius)`. The boolean pipeline derives it from the joint extent of the
/// two clipped faces; only the section inside that region is traced (the
/// full parabola/hyperbola runs to infinity). Both infinite parameter
/// directions are clipped to the sphere's reach exactly as
/// [`intersect_marched`] clips a plane against a compact partner.
///
/// Seeding grids over the cone's clipped axial band × full angle and
/// refines against the plane. Vertices lie on both surfaces within
/// `tol.linear`, curves are transversal by construction, and a section
/// crossing the cone's angular seam comes back as open fragments meeting on
/// it (welded downstream), exactly like the periodic seam handling in
/// [`intersect_marched`].
///
/// Only the parabola/hyperbola branch is routed here; plane-cone circles,
/// ellipses, the generator-line pair and the apex-only contact are exact in
/// [`super::intersect`], which returns `NotImplemented` for precisely this
/// branch.
///
/// # Errors
/// [`CoreError::NotImplemented`] for any pair other than plane-cone, and for
/// a tangential contact detected while marching (transversal MVP).
pub fn intersect_marched_bounded(
    a: &Surface3,
    b: &Surface3,
    bounds: (Point3, f64),
    tol: &ToleranceContext,
) -> CoreResult<Vec<MarchedCurve>> {
    use Surface3::*;
    match (a, b) {
        // Grid over the cone: its axial parameterization covers the whole
        // section, whereas a plane grid would need the same clip and gains
        // nothing.
        (Cone { .. }, Plane { .. }) => march_bounded_pair(a, b, bounds, tol),
        (Plane { .. }, Cone { .. }) => {
            Ok(swap_params(intersect_marched_bounded(b, a, bounds, tol)?))
        }
        _ => Err(CoreError::NotImplemented {
            feature: "bounded marched SSI for this surface pair (only the \
                      unbounded plane-cone sections are marched with an \
                      explicit region; every other pair is closed-form in \
                      ssi::intersect or compact-bounded in intersect_marched)",
        }),
    }
}

/// Grid-seed over `a`, refine against `b`, both clipped to the region-of-
/// interest sphere `bounds`. Mirrors [`march_primitives`] but takes the
/// bound explicitly because neither primitive is compact.
fn march_bounded_pair(
    a: &Surface3,
    b: &Surface3,
    bounds: (Point3, f64),
    tol: &ToleranceContext,
) -> CoreResult<Vec<MarchedCurve>> {
    let [au, av] = clipped_domains(a, bounds, tol);
    let [bu, bv] = clipped_domains(b, bounds, tol);
    match march_boxed(a, b, [au, av, bu, bv], tol) {
        Err(CoreError::Degenerate { .. }) => Err(CoreError::NotImplemented {
            feature: "marched plane-cone SSI across a tangential contact \
                      (transversal MVP)",
        }),
        other => other,
    }
}

/// Re-converge a marched boundary vertex onto the exact intersection with
/// its seam parameter held fixed, far past the tracer's own gap tolerance.
///
/// The tracer's boundary corrector ([`Marcher::correct_pinned`]) stops as
/// soon as the surface gap drops below `tol.linear`, so two fragments cut
/// at the same seam can end up ~`tol.linear` apart even though they
/// terminate at the same geometric point. The boolean pipeline welds
/// fragment junctions at a snap length many orders tighter, so it
/// re-polishes each endpoint here: the parameter sitting exactly on a
/// natural domain bound of its surface (the tracer pins it there
/// bit-exactly) stays fixed while the three free parameters Newton-solve
/// the gap system to `gap_tol`. Returns the polished point evaluated on
/// the **pinned** surface — exactly on that surface's seam curve — or
/// `None` when no parameter sits on a natural bound (a stalled fragment)
/// or the tight solve does not converge.
pub(crate) fn tighten_boundary_point(
    a: &Surface3,
    b: &Surface3,
    params_a: (f64, f64),
    params_b: (f64, f64),
    gap_tol: f64,
) -> Option<Point3> {
    let state = [params_a.0, params_a.1, params_b.0, params_b.1];
    let natural = [a.domain_u(), a.domain_v(), b.domain_u(), b.domain_v()];
    let pin = (0..4).find(|&k| {
        let (lo, hi) = natural[k];
        (state[k] == lo && lo.is_finite()) || (state[k] == hi && hi.is_finite())
    })?;
    pin_intersection_point(a, b, state, pin, state[pin], gap_tol)
}

/// Extra Newton iterations past the acceptance tolerance in
/// [`pin_intersection_point`]. Quadratic convergence reaches the rounding
/// floor in one or two; the loop also stops as soon as an iteration fails
/// to shrink the gap.
const PIN_POLISH_ITERATIONS: usize = 3;

/// Newton-solve the intersection of `a` and `b` with the parameter at
/// index `pin` (into `[u_a, v_a, u_b, v_b]`) held fixed at `pin_value`,
/// from `seed` (which must be within a marching step of the root).
/// Returns the point evaluated on the pinned surface — exactly on its
/// `pin_value` iso-curve — or `None` if the tight solve does not converge.
///
/// The root is polished past `gap_tol` to the numerical fixed point:
/// acceptance at `gap_tol` alone leaves the iterate anywhere in a
/// `gap_tol`-sized blob around the root (the seed itself may already
/// qualify), so two callers solving the *same* junction from different
/// seeds could disagree by ~`gap_tol` — above the boolean pipeline's weld
/// epsilon. The extra iterations are monotone in the gap (kept only while
/// they improve), so every caller lands on the same point to rounding
/// error and downstream welds see bit-consistent junctions.
pub(crate) fn pin_intersection_point(
    a: &Surface3,
    b: &Surface3,
    seed: [f64; 4],
    pin: usize,
    pin_value: f64,
    gap_tol: f64,
) -> Option<Point3> {
    let mut state = seed;
    state[pin] = pin_value;
    // Domains only clamp the Newton updates; the seed is already within a
    // marching step of the root, so a generous window never binds while
    // still guarding against a divergent iterate.
    let domains = [0, 1, 2, 3].map(|k| (state[k] - 10.0, state[k] + 10.0));
    let marcher = Marcher {
        a,
        b,
        domains,
        gap_tol,
        h0: 0.0,
        h_min: 0.0,
    };
    let (accepted, mut frames) = marcher.correct_pinned(MarchState(state), pin)?;
    // Evaluate on the pinned surface so the point lies exactly on that
    // surface's seam curve (the free-side evaluation is a gap away).
    let on_pinned = |f: &Frames| if pin < 2 { f.pa } else { f.pb };
    let mut best_gap = frames.gap().norm();
    let mut best_point = on_pinned(&frames);
    let mut s = accepted;
    for _ in 0..PIN_POLISH_ITERATIONS {
        if marcher.pinned_newton_step(&mut s, &frames, pin).is_none() {
            break;
        }
        frames = marcher.frames(&s);
        let gap = frames.gap().norm();
        if gap >= best_gap {
            break;
        }
        best_gap = gap;
        best_point = on_pinned(&frames);
    }
    Some(best_point)
}

/// Reject the Villarceau configuration: a plane through the torus center
/// inclined at exactly `sin θ = minor/major` is bitangent to the torus, so
/// each of its two circles passes through two tangent points — outside the
/// transversal MVP, and a trap for the tracer's stepwise tangency check.
fn villarceau_check(plane: &Surface3, torus: &Surface3, tol: &ToleranceContext) -> CoreResult<()> {
    let (
        Surface3::Plane { origin, normal },
        &Surface3::Torus {
            center,
            axis,
            major_radius,
            minor_radius,
        },
    ) = (plane, torus)
    else {
        unreachable!("dispatched on Plane/Torus")
    };
    let cos_t = normal.dot(&axis).abs().min(1.0);
    let sin_t = (1.0 - cos_t * cos_t).sqrt();
    let through_center = tol.approx_zero(normal.dot(&(center - origin)));
    let angular = tol
        .angular
        .max(opensolid_core::tolerance::ANGULAR_RESOLUTION);
    if through_center && (sin_t - minor_radius / major_radius).abs() <= angular {
        return Err(CoreError::NotImplemented {
            feature: "plane-torus Villarceau section (bitangent plane through \
                      the center at sin θ = minor/major)",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::KnotVector;
    use std::f64::consts::FRAC_1_SQRT_2;

    fn default_tol() -> ToleranceContext {
        ToleranceContext::default()
    }

    /// Exact NURBS unit cylinder about the z-axis, z ∈ [0, 2]
    /// (rational quadratic circle in u, linear ruling in v).
    fn cylinder_patch() -> NurbsSurface {
        let ring = [
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

    /// Bilinear patch spanning x, y ∈ [-2, 2] on the plane
    /// z = height + slope_x · x.
    fn plane_patch(height: f64, slope_x: f64) -> NurbsSurface {
        let z = |x: f64| height + slope_x * x;
        let control_points = vec![
            vec![
                Point3::new(-2.0, -2.0, z(-2.0)),
                Point3::new(-2.0, 2.0, z(-2.0)),
            ],
            vec![
                Point3::new(2.0, -2.0, z(2.0)),
                Point3::new(2.0, 2.0, z(2.0)),
            ],
        ];
        let knots = KnotVector::clamped_uniform(1, 2).unwrap();
        NurbsSurface::bspline(control_points, knots.clone(), knots).unwrap()
    }

    /// Bicubic patch over x, y ∈ [0, 3] whose height field is given per
    /// control point: `z[i][j]` for the control point at
    /// (x, y) = (i, j). Linear precision of Bernstein bases keeps x and y
    /// exactly linear in the parameters.
    fn bicubic_graph(z: [[f64; 4]; 4]) -> NurbsSurface {
        let control_points: Vec<Vec<Point3>> = (0..4)
            .map(|i| {
                (0..4)
                    .map(|j| Point3::new(i as f64, j as f64, z[i][j]))
                    .collect()
            })
            .collect();
        let knots = KnotVector::clamped_uniform(3, 4).unwrap();
        NurbsSurface::bspline(control_points, knots.clone(), knots).unwrap()
    }

    fn param_on_bound(p: f64, (lo, hi): (f64, f64)) -> bool {
        (p - lo).abs() < 1e-9 || (p - hi).abs() < 1e-9
    }

    /// An endpoint of an open marched curve must sit on a domain bound of
    /// one of the surfaces (whichever parameter pinned first).
    fn assert_endpoint_on_boundary(
        curve: &MarchedCurve,
        index: usize,
        a: &NurbsSurface,
        b: &NurbsSurface,
    ) {
        let (ua, va) = curve.params_a[index];
        let (ub, vb) = curve.params_b[index];
        let on_a = param_on_bound(ua, a.domain_u()) || param_on_bound(va, a.domain_v());
        let on_b = param_on_bound(ub, b.domain_u()) || param_on_bound(vb, b.domain_v());
        assert!(
            on_a || on_b,
            "endpoint {index} not on a domain boundary: \
             params_a = ({ua}, {va}), params_b = ({ub}, {vb})"
        );
    }

    /// Every vertex must satisfy both surface preimages within the gap
    /// tolerance.
    fn assert_vertices_on_both(a: &NurbsSurface, b: &NurbsSurface, curve: &MarchedCurve) {
        for k in 0..curve.points.len() {
            let (ua, va) = curve.params_a[k];
            let (ub, vb) = curve.params_b[k];
            let pa = a.point(ua, va);
            let pb = b.point(ub, vb);
            assert!(
                (pa - curve.points[k]).norm() < 1e-9,
                "vertex {k} does not match params_a"
            );
            assert!(
                (pa - pb).norm() < 3e-6,
                "vertex {k}: surface gap {} exceeds tolerance",
                (pa - pb).norm()
            );
        }
    }

    #[test]
    fn nurbs_cylinder_plane_matches_analytic_ellipse() {
        let cylinder = cylinder_patch();
        // Tilted plane z = 1 + 0.3 x cuts the full cylinder cross-section:
        // the true intersection is the ellipse x² + y² = 1, z = 1 + 0.3 x.
        let plane = plane_patch(1.0, 0.3);
        let curves = intersect_nurbs(&cylinder, &plane, &default_tol()).unwrap();
        assert_eq!(curves.len(), 1, "expected a single intersection curve");
        let curve = &curves[0];
        assert!(curve.points.len() > 50, "polyline too sparse");
        assert_vertices_on_both(&cylinder, &plane, curve);
        for (k, p) in curve.points.iter().enumerate() {
            assert!(
                (p.x * p.x + p.y * p.y - 1.0).abs() < 1e-5,
                "vertex {k} off the cylinder: {p:?}"
            );
            assert!(
                (p.z - 1.0 - 0.3 * p.x).abs() < 1e-5,
                "vertex {k} off the plane: {p:?}"
            );
        }
        // The clamped NURBS cylinder has a parameter seam at u = 0/1, so
        // the geometrically closed ellipse marches seam to seam: both
        // endpoints at (1, 0, 1.3), full angular coverage in between.
        let seam = Point3::new(1.0, 0.0, 1.3);
        assert!(
            (curve.points.first().unwrap() - seam).norm() < 1e-4,
            "first endpoint not at the seam"
        );
        assert!(
            (curve.points.last().unwrap() - seam).norm() < 1e-4,
            "last endpoint not at the seam"
        );
        let mut angles: Vec<f64> = curve.points.iter().map(|p| p.y.atan2(p.x)).collect();
        angles.sort_by(f64::total_cmp);
        let max_gap = angles
            .windows(2)
            .map(|w| w[1] - w[0])
            .fold(0.0f64, f64::max);
        assert!(max_gap < 0.2, "angular coverage has a gap of {max_gap} rad");
    }

    #[test]
    fn bicubic_patches_transversal_crossing_single_branch() {
        // Gently wavy height field around z = 0 ...
        let wavy = bicubic_graph([
            [0.10, -0.08, 0.06, -0.10],
            [-0.06, 0.09, -0.10, 0.07],
            [0.08, -0.07, 0.09, -0.06],
            [-0.09, 0.06, -0.08, 0.10],
        ]);
        // ... crossed by the steep bicubic plane z = x - 1.5 (slope 1
        // against wavy slopes of ~0.2: safely transversal).
        let tilted = bicubic_graph([[-1.5; 4], [-0.5; 4], [0.5; 4], [1.5; 4]]);
        let curves = intersect_nurbs(&wavy, &tilted, &default_tol()).unwrap();
        assert_eq!(curves.len(), 1, "expected a single branch");
        let curve = &curves[0];
        assert!(!curve.closed, "open crossing must not close");
        assert!(curve.points.len() >= 20, "polyline too sparse");
        assert_vertices_on_both(&wavy, &tilted, curve);
        // Continuity: no jumps between consecutive vertices.
        for w in curve.points.windows(2) {
            assert!(
                (w[1] - w[0]).norm() < 0.2,
                "polyline jump of {}",
                (w[1] - w[0]).norm()
            );
        }
        // The branch runs boundary to boundary across the patch.
        assert_endpoint_on_boundary(curve, 0, &wavy, &tilted);
        assert_endpoint_on_boundary(curve, curve.points.len() - 1, &wavy, &tilted);
        // And the crossing stays near x = 1.5 ± the wave amplitude.
        for p in &curve.points {
            assert!(
                (p.x - 1.5).abs() < 0.35,
                "crossing strayed from the tilted plane locus: {p:?}"
            );
        }
    }

    #[test]
    fn marching_terminates_at_domain_boundaries() {
        // Flat patch z = 0 against the tilted plane z = 0.5 x - 0.5: the
        // intersection is the straight line x = 1, z = 0 crossing the
        // shared footprint, ending where the march hits the y-bounds.
        let flat = plane_patch(0.0, 0.0);
        let tilted = plane_patch(-0.5, 0.5);
        let curves = intersect_nurbs(&flat, &tilted, &default_tol()).unwrap();
        assert_eq!(curves.len(), 1);
        let curve = &curves[0];
        assert!(!curve.closed);
        assert_vertices_on_both(&flat, &tilted, curve);
        for p in &curve.points {
            assert!((p.x - 1.0).abs() < 1e-5, "off the line x = 1: {p:?}");
            assert!(p.z.abs() < 1e-5, "off the plane z = 0: {p:?}");
        }
        assert_endpoint_on_boundary(curve, 0, &flat, &tilted);
        assert_endpoint_on_boundary(curve, curve.points.len() - 1, &flat, &tilted);
        // The full y extent of the footprint is covered.
        let mut ys: Vec<f64> = curve.points.iter().map(|p| p.y).collect();
        ys.sort_by(f64::total_cmp);
        assert!(ys[0] < -1.999, "curve stops short of y = -2");
        assert!(ys[ys.len() - 1] > 1.999, "curve stops short of y = +2");
    }

    #[test]
    fn near_tangency_bails_with_structured_error() {
        // z = (x - 1.5)³ crosses z = 0 tangentially along x = 1.5 (equal
        // normals on the crossing): must be rejected, not traced.
        let flat = bicubic_graph([[0.0; 4]; 4]);
        let cubic = bicubic_graph([[-3.375; 4], [3.375; 4], [-3.375; 4], [3.375; 4]]);
        let err = intersect_nurbs(&flat, &cubic, &default_tol()).unwrap_err();
        match &err {
            CoreError::Degenerate { context, reason } => {
                assert_eq!(*context, "ssi::marching");
                assert!(
                    reason.contains("tangential"),
                    "reason does not name tangency: {reason}"
                );
                assert!(
                    reason.contains("transversal"),
                    "reason does not state the MVP limitation: {reason}"
                );
            }
            other => panic!("expected Degenerate near-tangency error, got {other:?}"),
        }
    }

    #[test]
    fn disjoint_surfaces_yield_empty_result() {
        let low = plane_patch(0.0, 0.0);
        let high = plane_patch(1.0, 0.0);
        let curves = intersect_nurbs(&low, &high, &default_tol()).unwrap();
        assert!(curves.is_empty(), "parallel planes must not intersect");
    }

    #[test]
    fn closed_curves_repeat_their_first_vertex() {
        // A cylinder against the horizontal plane z = 1 would march seam
        // to seam like the ellipse; to exercise closure use two bicubic
        // graphs whose intersection is a closed loop strictly inside both
        // domains: a dome z = bump peaking at ~0.59 against z = 0.3.
        let dome = bicubic_graph([
            [0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 1.0, 0.0],
            [0.0, 1.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0],
        ]);
        let level = bicubic_graph([[0.3; 4]; 4]);
        let curves = intersect_nurbs(&dome, &level, &default_tol()).unwrap();
        assert_eq!(curves.len(), 1, "expected a single closed loop");
        let curve = &curves[0];
        assert!(curve.closed, "loop not detected as closed");
        assert!(
            (curve.points.first().unwrap() - curve.points.last().unwrap()).norm() < 1e-9,
            "closed polyline does not repeat its first vertex"
        );
        assert_vertices_on_both(&dome, &level, curve);
        for p in &curve.points {
            assert!(
                (p.z - 0.3).abs() < 1e-5,
                "loop vertex off the level surface: {p:?}"
            );
        }
    }

    // ── marched primitive pairs (intersect_marched) ────────────────────

    /// Residual bound for marched vertices: the corrector converges the
    /// surface gap to `tol.linear` (1e-6 default), with a little headroom.
    const MARCH_RESID: f64 = 5e-6;

    /// Geometric residual of `p` against the primitive's locus: ~0 iff on it.
    fn primitive_residual(s: &Surface3, p: &Point3) -> f64 {
        match *s {
            Surface3::Plane { origin, normal } => normal.dot(&(p - origin)).abs(),
            Surface3::Sphere { center, radius, .. } => ((p - center).norm() - radius).abs(),
            Surface3::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let d = p - origin;
                ((d - axis * axis.dot(&d)).norm() - radius).abs()
            }
            Surface3::Torus {
                center,
                axis,
                major_radius,
                minor_radius,
            } => {
                let d = p - center;
                let h = axis.dot(&d);
                let rho = (d - axis * h).norm();
                ((rho - major_radius).hypot(h) - minor_radius).abs()
            }
            Surface3::Cone {
                origin,
                axis,
                half_angle,
                radius,
            } => {
                // Radial deviation from the cone's rho(v) = radius + v·tan α,
                // scaled by cos α to a true perpendicular distance.
                let d = p - origin;
                let v = axis.dot(&d);
                let rho = (d - axis * v).norm();
                ((rho - (radius + v * half_angle.tan())) * half_angle.cos()).abs()
            }
        }
    }

    /// The bead-level acceptance: every vertex of every curve satisfies
    /// the implicit residual bound on both surfaces, and its parameter
    /// preimages reproduce it on each surface.
    fn assert_marched_on_both(curves: &[MarchedCurve], a: &Surface3, b: &Surface3) {
        assert!(!curves.is_empty(), "expected intersection curves");
        let total: usize = curves.iter().map(|c| c.points.len()).sum();
        assert!(total >= 30, "polylines too sparse: {total} vertices in all");
        for (ci, curve) in curves.iter().enumerate() {
            for (k, p) in curve.points.iter().enumerate() {
                let (ra, rb) = (primitive_residual(a, p), primitive_residual(b, p));
                assert!(ra <= MARCH_RESID, "curve {ci} vertex {k} off A by {ra:e}");
                assert!(rb <= MARCH_RESID, "curve {ci} vertex {k} off B by {rb:e}");
                let (ua, va) = curve.params_a[k];
                let (ub, vb) = curve.params_b[k];
                assert!(
                    (a.point(ua, va) - p).norm() <= MARCH_RESID,
                    "curve {ci} vertex {k}: params_a do not reproduce the vertex"
                );
                assert!(
                    (b.point(ub, vb) - p).norm() <= MARCH_RESID,
                    "curve {ci} vertex {k}: params_b do not reproduce the vertex"
                );
            }
        }
    }

    /// Assert a parameter track covers the full `[0, 2π)` period with no
    /// gap larger than `max_gap` (seam-to-seam traces of a loop around a
    /// periodic direction).
    fn assert_full_period(track: impl Iterator<Item = f64>, max_gap: f64) {
        let mut values: Vec<f64> = track.collect();
        values.sort_by(f64::total_cmp);
        let first = values[0];
        let last = values[values.len() - 1];
        let wrap = (first + TWO_PI - last).max(0.0);
        let interior = values.windows(2).map(|w| w[1] - w[0]).fold(0.0, f64::max);
        assert!(
            interior.max(wrap) <= max_gap,
            "period coverage gap {:.3} exceeds {max_gap}",
            interior.max(wrap)
        );
    }

    fn torus3(center: Point3, major: f64, minor: f64) -> Surface3 {
        Surface3::Torus {
            center,
            axis: Vector3::z(),
            major_radius: major,
            minor_radius: minor,
        }
    }

    #[test]
    fn marched_cylinder_sphere_general_closed_loop() {
        let cyl = Surface3::Cylinder {
            origin: Point3::origin(),
            axis: Vector3::z(),
            radius: 1.0,
        };
        let sph = Surface3::Sphere {
            center: Point3::new(0.0, -1.2, 0.0),
            axis: Vector3::z(),
            radius: 0.8,
        };
        let curves = intersect_marched(&cyl, &sph, &default_tol()).unwrap();
        assert_eq!(curves.len(), 1, "one puncture loop expected");
        assert!(curves[0].closed, "interior loop must close");
        assert_marched_on_both(&curves, &cyl, &sph);
        // The quartic's z-extent: z² = -1.8 - 2.4 y over y ∈ [-1, -0.75].
        let z_max = 0.6_f64.sqrt();
        let reached = curves[0]
            .points
            .iter()
            .map(|p| p.z.abs())
            .fold(0.0, f64::max);
        assert!(
            reached <= z_max + 1e-4,
            "curve exceeds the analytic z bound"
        );
        assert!(reached >= 0.9 * z_max, "curve misses most of its z extent");
    }

    #[test]
    fn marched_plane_torus_oblique_spiric_rings() {
        // Plane z = 0.5 - tan(5°)·x: an oblique (non-Villarceau) section,
        // two nested rings that each encircle the torus axis.
        let tilt = 5.0_f64.to_radians();
        let plane = Surface3::Plane {
            origin: Point3::new(0.0, 0.0, 0.5),
            normal: Vector3::new(tilt.sin(), 0.0, tilt.cos()),
        };
        let torus = torus3(Point3::origin(), 3.0, 1.0);
        let curves = intersect_marched(&torus, &plane, &default_tol()).unwrap();
        assert_eq!(curves.len(), 2, "outer and inner spiric rings");
        assert_marched_on_both(&curves, &torus, &plane);
        // Each ring wraps the (seam-bounded) torus angle u fully.
        for curve in &curves {
            assert_full_period(curve.params_a.iter().map(|&(u, _)| u), 0.3);
        }
    }

    #[test]
    fn marched_bounded_plane_cone_hyperbola() {
        // Cone widening upward from radius 1 at z = 0, half-angle 30°; apex
        // below at z = -1/tan30° ≈ -1.732. A vertical plane parallel to the
        // axis (normal +y) offset 0.5 from it cuts a hyperbola on the upper
        // nappe: x² = (1 + z·tan30°)² − 0.25 on y = 0.5. The plane sits away
        // from the cone's +x seam, so the branch traces as one open fragment.
        let half_angle = 30.0_f64.to_radians();
        let cone = Surface3::Cone {
            origin: Point3::origin(),
            axis: Vector3::z(),
            half_angle,
            radius: 1.0,
        };
        let plane = Surface3::Plane {
            origin: Point3::new(0.0, 0.5, 0.0),
            normal: Vector3::y(),
        };
        let bounds = (Point3::new(0.0, 0.5, 1.0), 2.0);
        let curves = intersect_marched_bounded(&cone, &plane, bounds, &default_tol()).unwrap();
        assert_eq!(curves.len(), 1, "one open hyperbola branch");
        assert!(!curves[0].closed, "a hyperbola section is open");
        assert_marched_on_both(&curves, &cone, &plane);
        // The branch spreads well out in x within the region of interest.
        let x_reach = curves[0]
            .points
            .iter()
            .map(|p| p.x.abs())
            .fold(0.0, f64::max);
        assert!(x_reach >= 0.8, "hyperbola too short in x: {x_reach}");
        // Swapped argument order re-enters canonically and swaps preimages.
        let swapped = intersect_marched_bounded(&plane, &cone, bounds, &default_tol()).unwrap();
        assert_eq!(swapped.len(), 1);
        assert_marched_on_both(&swapped, &plane, &cone);
    }

    #[test]
    fn marched_cylinder_torus_drilled_tube() {
        // A thin vertical cylinder drilled through the top and bottom of
        // the tube at u = π/2: one loop where it enters, one where it
        // exits, each encircling the cylinder.
        let torus = torus3(Point3::origin(), 3.0, 1.0);
        let cyl = Surface3::Cylinder {
            origin: Point3::new(0.0, 3.0, 0.0),
            axis: Vector3::z(),
            radius: 0.5,
        };
        let curves = intersect_marched(&torus, &cyl, &default_tol()).unwrap();
        assert_eq!(curves.len(), 2, "entry and exit loops");
        assert_marched_on_both(&curves, &torus, &cyl);
        let mut mean_z: Vec<f64> = curves
            .iter()
            .map(|c| c.points.iter().map(|p| p.z).sum::<f64>() / c.points.len() as f64)
            .collect();
        mean_z.sort_by(f64::total_cmp);
        assert!(
            mean_z[0] < -0.8 && mean_z[1] > 0.8,
            "loops on top and bottom"
        );
        for curve in &curves {
            assert_full_period(curve.params_b.iter().map(|&(u, _)| u), 0.3);
        }
    }

    #[test]
    fn marched_sphere_torus_general_tube_rings() {
        // Sphere centered on the tube centerline, wider than the tube:
        // two rings around the tube, one on each side of the center.
        let torus = torus3(Point3::origin(), 3.0, 1.0);
        let sph = Surface3::Sphere {
            center: Point3::new(0.0, 3.0, 0.0),
            axis: Vector3::z(),
            radius: 1.5,
        };
        let curves = intersect_marched(&torus, &sph, &default_tol()).unwrap();
        assert_marched_on_both(&curves, &torus, &sph);
        // Fragmentation at the torus v-seam and the sphere u-seam is
        // allowed (each cut is a domain boundary), but grouping fragments
        // by ring — the two rings sit on either side of u = π/2 — must
        // recover full tube-angle coverage for both.
        let (left, right): (Vec<&MarchedCurve>, Vec<&MarchedCurve>) = curves
            .iter()
            .partition(|c| c.params_a[0].0 < std::f64::consts::FRAC_PI_2);
        assert!(
            !left.is_empty() && !right.is_empty(),
            "expected rings on both sides of u = π/2"
        );
        for ring in [left, right] {
            assert_full_period(
                ring.iter().flat_map(|c| c.params_a.iter().map(|&(_, v)| v)),
                0.3,
            );
        }
    }

    #[test]
    fn marched_torus_torus_offset_pair() {
        // Parallel but non-coaxial axes with unequal tubes: an irreducible
        // degree-8 configuration in general position.
        let a = torus3(Point3::origin(), 3.0, 1.0);
        let b = torus3(Point3::new(0.4, 0.0, 0.0), 3.0, 0.8);
        let curves = intersect_marched(&a, &b, &default_tol()).unwrap();
        assert_marched_on_both(&curves, &a, &b);
        // The y = 0 meridian crossings at x = 3.65, z = ±√0.5775 must be
        // reached by some traced vertex.
        for target_z in [0.5775_f64.sqrt(), -(0.5775_f64.sqrt())] {
            let target = Point3::new(3.65, 0.0, target_z);
            let nearest = curves
                .iter()
                .flat_map(|c| c.points.iter())
                .map(|p| (p - target).norm())
                .fold(f64::INFINITY, f64::min);
            assert!(
                nearest < 0.15,
                "no vertex near the analytic crossing {target:?} (nearest {nearest:.3})"
            );
        }
    }

    #[test]
    fn marched_villarceau_plane_rejected() {
        // Plane through the center at sin θ = r/R is bitangent: its two
        // Villarceau circles each carry two tangent points.
        let torus = torus3(Point3::origin(), 3.0, 1.0);
        let sin_t = 1.0 / 3.0;
        let plane = Surface3::Plane {
            origin: Point3::origin(),
            normal: Vector3::new(sin_t, 0.0, (1.0 - sin_t * sin_t).sqrt()),
        };
        for (a, b) in [(&torus, &plane), (&plane, &torus)] {
            let result = intersect_marched(a, b, &default_tol());
            assert!(
                matches!(result, Err(CoreError::NotImplemented { .. })),
                "expected NotImplemented, got {result:?}"
            );
        }
    }

    #[test]
    fn marched_tangential_contact_rejected() {
        // Concentric kissing tubes (majors 2 and 4, tubes 1): tangent along
        // the circle rho = 3 — detected while seeding, reported as the
        // structured NotImplemented of the transversal MVP.
        let a = torus3(Point3::origin(), 2.0, 1.0);
        let b = torus3(Point3::origin(), 4.0, 1.0);
        let result = intersect_marched(&a, &b, &default_tol());
        assert!(
            matches!(result, Err(CoreError::NotImplemented { .. })),
            "expected NotImplemented, got {result:?}"
        );
    }

    #[test]
    fn marched_disjoint_pair_is_empty() {
        // Small sphere floating inside the tube: no surface contact.
        let torus = torus3(Point3::origin(), 3.0, 1.0);
        let sph = Surface3::Sphere {
            center: Point3::new(0.0, 3.0, 0.0),
            axis: Vector3::z(),
            radius: 0.4,
        };
        let curves = intersect_marched(&torus, &sph, &default_tol()).unwrap();
        assert!(curves.is_empty());
    }

    #[test]
    fn marched_closed_form_pairs_rejected() {
        let s1 = Surface3::Sphere {
            center: Point3::origin(),
            axis: Vector3::z(),
            radius: 1.0,
        };
        let s2 = Surface3::Sphere {
            center: Point3::new(1.5, 0.0, 0.0),
            axis: Vector3::z(),
            radius: 1.0,
        };
        let result = intersect_marched(&s1, &s2, &default_tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    // ── marched cone pairs with a compact partner (intersect_marched) ───

    /// Cone about +z, half-angle 30°, radius 1 at `v = 0`; apex below at
    /// `z = -1/tan30° ≈ -1.732`.
    fn cone3() -> Surface3 {
        Surface3::cone(Point3::origin(), Vector3::z(), 30f64.to_radians(), 1.0).unwrap()
    }

    #[test]
    fn marched_sphere_cone_bite_closed_loop() {
        // Sphere just outside the cone wall on the +y side (u = π/2, clear of
        // the u = 0 seam): its near cap pokes through the nappe, so the
        // intersection is a single closed cap loop, apex-free and comfortably
        // inside the clipped axial window.
        let cone = cone3();
        // Cone radius at z = 2 is 1 + 2·tan30° ≈ 2.1547; sit the center 0.4
        // beyond the wall so only a cap intersects.
        let center = Point3::new(0.0, 1.0 + 2.0 * 30f64.to_radians().tan() + 0.4, 2.0);
        let sph = Surface3::sphere(center, Vector3::z(), 0.8).unwrap();
        let curves = intersect_marched(&sph, &cone, &default_tol()).unwrap();
        assert_eq!(curves.len(), 1, "one bite loop expected");
        assert!(curves[0].closed, "the bite must close");
        assert_marched_on_both(&curves, &sph, &cone);
        // The loop hugs the +y side of the cone, on the sphere's near cap.
        for p in &curves[0].points {
            assert!(p.y > 1.0, "loop strayed off the +y wall: {p:?}");
            assert!(
                (p - center).norm() < 0.85,
                "vertex not on the 0.8 sphere: {p:?}"
            );
        }
    }

    #[test]
    fn marched_torus_cone_coaxial_rings() {
        // Coaxial tube around the cone: the axial-plane line (cone) cuts the
        // tube circle at two points, each sweeping a full ring about the
        // axis — two horizontal circles at z ≈ 1.80 and 2.50.
        let cone = cone3();
        let torus = torus3(Point3::new(0.0, 0.0, 2.0), 2.5, 0.5);
        let curves = intersect_marched(&torus, &cone, &default_tol()).unwrap();
        assert_marched_on_both(&curves, &torus, &cone);
        // Group by mean height: one ring below z = 2.15, one above.
        let mut mean_z: Vec<f64> = curves
            .iter()
            .map(|c| c.points.iter().map(|p| p.z).sum::<f64>() / c.points.len() as f64)
            .collect();
        mean_z.sort_by(f64::total_cmp);
        assert_eq!(mean_z.len(), 2, "two rings expected");
        assert!(
            (mean_z[0] - 1.803).abs() < 0.05 && (mean_z[1] - 2.496).abs() < 0.05,
            "ring heights off the analytic crossings: {mean_z:?}"
        );
        // Each ring wraps the full torus angle (seam-to-seam over one period).
        for curve in &curves {
            assert_full_period(curve.params_a.iter().map(|&(u, _)| u), 0.3);
        }
    }

    #[test]
    fn marched_cone_swapped_orders_agree() {
        // (Cone, Sphere) re-enters as (Sphere, Cone) and swaps the preimages
        // back: the point sets must match the canonical order.
        let cone = cone3();
        let center = Point3::new(0.0, 1.0 + 2.0 * 30f64.to_radians().tan() + 0.4, 2.0);
        let sph = Surface3::sphere(center, Vector3::z(), 0.8).unwrap();
        let canon = intersect_marched(&sph, &cone, &default_tol()).unwrap();
        let swapped = intersect_marched(&cone, &sph, &default_tol()).unwrap();
        assert_eq!(canon.len(), swapped.len());
        assert_marched_on_both(&swapped, &cone, &sph);
        // Same loop, same vertices (order preserved by the shared driver).
        assert_eq!(canon[0].points, swapped[0].points);
    }

    #[test]
    fn marched_cone_disjoint_is_empty() {
        // Small sphere floating well inside the nappe: no contact.
        let cone = cone3();
        let sph = Surface3::sphere(Point3::new(0.0, 0.0, 3.0), Vector3::z(), 0.3).unwrap();
        let curves = intersect_marched(&sph, &cone, &default_tol()).unwrap();
        assert!(curves.is_empty(), "interior sphere must not touch the cone");
    }

    #[test]
    fn marched_unbounded_cone_pairs_rejected() {
        // Neither surface is compact, so these carry no partner to seed the
        // grid and are not handled by intersect_marched (they march through
        // intersect_marched_bounded instead).
        let cone = cone3();
        let plane = Surface3::plane(Point3::new(0.0, 0.0, 1.0), Vector3::x()).unwrap();
        let cyl = Surface3::cylinder(Point3::new(2.0, 0.0, 0.0), Vector3::z(), 0.5).unwrap();
        let cone2 = Surface3::cone(
            Point3::new(0.0, 0.0, 4.0),
            -Vector3::z(),
            30f64.to_radians(),
            1.0,
        )
        .unwrap();
        for (a, b) in [(&cone, &plane), (&cone, &cyl), (&cone, &cone2)] {
            assert!(
                matches!(
                    intersect_marched(a, b, &default_tol()),
                    Err(CoreError::NotImplemented { .. })
                ),
                "unbounded cone pair should be NotImplemented"
            );
            assert!(matches!(
                intersect_marched(b, a, &default_tol()),
                Err(CoreError::NotImplemented { .. })
            ));
        }
    }

    /// Fixture for the boundary-point polish: unit sphere and an offset
    /// vertical cylinder whose intersection crosses the sphere's `u = 0`
    /// seam meridian (the `y = 0, x > 0` half-plane) transversally at
    /// `q = (c, 0, √(1 − c²))` with `c = 0.3 + √0.32` (the positive root
    /// of `(x − 0.3)² + 0.2² = 0.6²`).
    fn seam_junction_fixture() -> (Surface3, Surface3, Point3, (f64, f64), (f64, f64)) {
        let sphere = Surface3::sphere(Point3::origin(), Vector3::z(), 1.0).unwrap();
        let cyl_origin = Point3::new(0.3, 0.2, 0.0);
        let cylinder = Surface3::cylinder(cyl_origin, Vector3::z(), 0.6).unwrap();
        let c = 0.3 + 0.32_f64.sqrt();
        let q = Point3::new(c, 0.0, (1.0 - c * c).sqrt());
        // Sphere: point(u, v) = (cos v cos u, cos v sin u, sin v); the
        // junction sits on the seam u = 0 at latitude v = asin(q.z).
        let params_a = (0.0, q.z.asin());
        // Cylinder: u from the radial direction, v the height along z.
        let params_b = ((q.y - cyl_origin.y).atan2(q.x - cyl_origin.x), q.z);
        assert!((sphere.point(params_a.0, params_a.1) - q).norm() < 1e-15);
        assert!((cylinder.point(params_b.0, params_b.1) - q).norm() < 1e-15);
        (sphere, cylinder, q, params_a, params_b)
    }

    /// A seed already inside `gap_tol` must still be polished onto the
    /// exact junction: acceptance alone would return the perturbed seed
    /// (~1e-5 off), and downstream welds need every caller of the same
    /// junction to land on the same point.
    #[test]
    fn tighten_polishes_past_the_acceptance_tolerance() {
        let (sphere, cylinder, q, (ua, va), (ub, vb)) = seam_junction_fixture();
        let p = tighten_boundary_point(
            &sphere,
            &cylinder,
            (ua, va + 3e-5),
            (ub, vb - 2e-5),
            1e-4, // above the seed's gap: accepted at iteration 0
        )
        .expect("pinned solve must converge");
        assert!(
            (p - q).norm() < 1e-10,
            "polished endpoint {p:?} is {:e} from the exact junction {q:?}",
            (p - q).norm()
        );
        // Evaluated on the pinned sphere at u = 0 exactly: on the seam.
        assert_eq!(p.y, 0.0);
    }

    /// Two fragments cut at the same seam approach the junction from the
    /// two cover sides (`u = 0` and `u = 2π`); their tightened endpoints
    /// must agree far below any weld epsilon.
    #[test]
    fn tighten_agrees_across_seam_sides() {
        let (sphere, cylinder, _, (_, va), (ub, vb)) = seam_junction_fixture();
        let lo =
            tighten_boundary_point(&sphere, &cylinder, (0.0, va + 3e-5), (ub, vb - 2e-5), 1e-4)
                .expect("low-side solve must converge");
        let hi = tighten_boundary_point(
            &sphere,
            &cylinder,
            (TWO_PI, va - 2e-5),
            (ub, vb + 3e-5),
            1e-4,
        )
        .expect("high-side solve must converge");
        assert!(
            (lo - hi).norm() < 1e-12,
            "seam-side endpoints disagree by {:e}",
            (lo - hi).norm()
        );
    }

    /// No parameter on a natural domain bound (a stalled fragment): the
    /// tighten has no seam to pin and reports `None`.
    #[test]
    fn tighten_requires_a_boundary_parameter() {
        let (sphere, cylinder, _, (_, va), (ub, vb)) = seam_junction_fixture();
        assert!(tighten_boundary_point(&sphere, &cylinder, (1.0, va), (ub, vb), 1e-4).is_none());
    }

    /// Pinning a free-side parameter (the cylinder angle) evaluates the
    /// result on THAT surface's iso-curve.
    #[test]
    fn pin_intersection_point_evaluates_on_the_pinned_surface() {
        let (sphere, cylinder, q, (ua, va), (ub, vb)) = seam_junction_fixture();
        let p = pin_intersection_point(
            &sphere,
            &cylinder,
            [ua, va + 1e-5, ub, vb - 1e-5],
            2,
            ub,
            1e-4,
        )
        .expect("pinned solve must converge");
        assert!((p - q).norm() < 1e-10);
        // Exactly on the cylinder's u = ub ruling line.
        let ruling = cylinder.point(ub, p.z);
        assert!((p - ruling).norm() < 1e-14);
    }
}
