//! Marching surface-surface intersection for NURBS surfaces
//! (transversal MVP).
//!
//! [`intersect_nurbs`] traces the intersection curves of two
//! [`NurbsSurface`]s as dense polylines (fitting the result as a NURBS
//! curve is a later hardening pass), per spec/02-geometry.md §5.3 at the
//! spec/12 "minimum viable" level:
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

use crate::nurbs::NurbsSurface;
use crate::project::SurfaceProject;
use crate::surface::SurfaceEval;
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

/// Marching context: the surface pair, their domain boxes, and the
/// tolerances/step sizes derived from the tolerance context and the
/// geometry scale.
struct Marcher<'a> {
    a: &'a NurbsSurface,
    b: &'a NurbsSurface,
    /// Domain interval per parameter, indexed like [`MarchState`].
    domains: [(f64, f64); 4],
    /// Convergence tolerance on `|S_a - S_b|`.
    gap_tol: f64,
    /// Nominal marching step (3D arc length).
    h0: f64,
    /// Minimum step before a branch is declared stalled.
    h_min: f64,
}

impl Marcher<'_> {
    fn frames(&self, s: &MarchState) -> Frames {
        let da = self.a.derivatives(s.0[0], s.0[1], 1);
        let db = self.b.derivatives(s.0[2], s.0[3], 1);
        Frames {
            pa: Point3::origin() + da[0][0],
            au: da[1][0],
            av: da[0][1],
            pb: Point3::origin() + db[0][0],
            bu: db[1][0],
            bv: db[0][1],
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
        let free: Vec<usize> = (0..4).filter(|&k| k != pin).collect();
        let mut s = start;
        for _ in 0..MAX_CORRECTOR_ITERATIONS {
            let frames = self.frames(&s);
            let gap = frames.gap();
            if gap.norm() <= self.gap_tol {
                return Some((s, frames));
            }
            let cols = [frames.au, frames.av, -frames.bu, -frames.bv];
            let jac = Matrix3::from_columns(&[cols[free[0]], cols[free[1]], cols[free[2]]]);
            let delta = jac.lu().solve(&(-gap))?;
            for (i, &k) in free.iter().enumerate() {
                s.0[k] += delta[i];
            }
            if s.0.iter().any(|v| !v.is_finite()) {
                return None;
            }
            self.clamp(&mut s);
        }
        None
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
    let domains = [a.domain_u(), a.domain_v(), b.domain_u(), b.domain_v()];
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
}
