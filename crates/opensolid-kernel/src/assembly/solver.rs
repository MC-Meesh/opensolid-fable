//! The rigid-body mate solver: Levenberg–Marquardt over floating-instance
//! poses, with a closed-form fast path for the canonical concentric +
//! coincident fastener seat (Assembly MVP 2, of-fsl.25.2).
//!
//! The solver works on **poses**, not geometry: [`solve_mates`] takes each
//! instance's [`Transform3`] and a `fixed` flag, plus the [`Mate`]s, and
//! returns the resolved poses. It never touches a [`Part`](super::Part) or an
//! SDF, so it is a self-contained numerical layer that is testable without
//! building any geometry. [`Assembly::solve`](super::Assembly::solve) is a thin
//! wrapper that reads the poses out of the assembly's instances.
//!
//! # Parameterization
//!
//! Each floating instance contributes 6 unknowns — a translation increment and
//! a rotation-vector increment applied to its current pose. Rotation is carried
//! as a unit quaternion and renormalized every iteration (per
//! `docs/design/ASSEMBLIES.md` §2), so there is no gimbal degeneracy. A step
//! `δ = (u, ω)` updates a pose by `R ← exp(ω) R`, `t ← t + u`.
//!
//! # Residuals
//!
//! Each mate contributes an analytic residual block that depends only on the
//! two instances it names (so the Jacobian is sparse):
//!
//! - **Coincident (plane–plane)** — `nₐ + n_b` (flush + anti-parallel, 3) and
//!   `nₐ · (pₐ − p_b)` (co-planar, 1).
//! - **Coincident (point–plane)** — `n_plane · (p_point − p_plane)` (1).
//! - **Concentric** — `dₐ × d_b` (collinear, 3) and the rejection of
//!   `pₐ − p_b` off `dₐ` (line coincident, 3).
//! - **Distance (plane–plane)** — `nₐ · (pₐ − p_b) − value` (1).
//! - **Distance (point–point)** — `|pₐ − p_b| − value` (1).
//!
//! # Jacobian
//!
//! The residuals are analytic and sparse; the Jacobian is assembled by
//! central finite differences of those residuals along each floating
//! instance's six tangent directions. Only the mates that touch a perturbed
//! instance are re-evaluated per column, so the assembled Jacobian keeps the
//! same block sparsity the analytic derivative would. For the tens-of-unknowns
//! systems assemblies produce this is exact to finite-difference precision and
//! negligibly cheap, and it removes a large class of hand-derivative sign bugs.

use super::mates::{Feature, Mate, MateKind};
use nalgebra::{DMatrix, DVector, Translation3, UnitQuaternion};
use opensolid_core::types::{Point3, Transform3, Vector3};

/// Tuning knobs for [`solve_mates_with`].
#[derive(Debug, Clone, Copy)]
pub struct SolveOptions {
    /// Maximum LM iterations before giving up.
    pub max_iterations: usize,
    /// Convergence threshold on the residual 2-norm.
    pub tolerance: f64,
    /// Initial Levenberg–Marquardt damping.
    pub initial_lambda: f64,
    /// Relative threshold (times the largest singular value) below which a
    /// singular value counts as a free DOF.
    pub rank_tolerance: f64,
}

impl Default for SolveOptions {
    fn default() -> Self {
        Self {
            max_iterations: 200,
            tolerance: 1e-9,
            initial_lambda: 1e-3,
            // Above the finite-difference Jacobian's roundoff floor
            // (~macheps/FD_EPS ≈ 1e-10) and well below unit-scale genuine
            // constraints, so redundant/null directions read as free DOF.
            rank_tolerance: 1e-6,
        }
    }
}

/// Outcome classification for a solve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SolveStatus {
    /// All residuals reached the tolerance. Free DOF may remain
    /// (see [`SolveResult::free_dof`]); that is normal, not an error.
    Converged,
    /// The mates conflict: LM stalled with a residual above tolerance. The
    /// returned poses are the least-squares best fit, and the residual norm is
    /// reported so callers can surface a mate error instead of crashing.
    OverConstrained,
}

/// The result of a solve: resolved poses plus diagnostics.
#[derive(Debug, Clone)]
pub struct SolveResult {
    /// Resolved pose per instance, in insertion order. Fixed instances are
    /// returned unchanged.
    pub transforms: Vec<Transform3>,
    /// Whether the mates were satisfiable.
    pub status: SolveStatus,
    /// Residual 2-norm at the returned poses (0 for a perfect solve).
    pub residual_norm: f64,
    /// LM iterations taken (0 when the closed-form path or an already-solved
    /// input short-circuits iteration).
    pub iterations: usize,
    /// Remaining unconstrained DOF across all floating instances (rank
    /// deficiency of the constraint Jacobian). A seated bolt free to spin
    /// reports 1; a fully-located part reports 0.
    pub free_dof: usize,
}

impl SolveResult {
    /// True when the mates could not all be satisfied.
    pub fn is_over_constrained(&self) -> bool {
        self.status == SolveStatus::OverConstrained
    }

    /// True when the solved instances retain movable freedom.
    pub fn is_under_constrained(&self) -> bool {
        self.free_dof > 0
    }
}

/// Solve `mates` for the floating instance poses.
///
/// `poses[i]` is instance `i`'s current pose and `fixed[i]` marks ground
/// instances (held constant). Mates reference instances by the index into
/// these slices. See [`SolveStatus`] for how conflicting and under-constrained
/// systems are reported (neither panics).
///
/// # Panics
///
/// Never on numerics. `poses` and `fixed` must have equal length and every
/// mate must reference an in-range instance; a mate with a malformed feature
/// pairing simply contributes no residual (use [`Mate::is_valid`] to check).
pub fn solve_mates(poses: &[Transform3], fixed: &[bool], mates: &[Mate]) -> SolveResult {
    solve_mates_with(poses, fixed, mates, &SolveOptions::default())
}

/// [`solve_mates`] with explicit options.
pub fn solve_mates_with(
    poses_in: &[Transform3],
    fixed: &[bool],
    mates: &[Mate],
    opts: &SolveOptions,
) -> SolveResult {
    assert_eq!(
        poses_in.len(),
        fixed.len(),
        "poses and fixed must describe the same instances"
    );
    let float_indices: Vec<usize> = (0..fixed.len()).filter(|&i| !fixed[i]).collect();
    let mut poses: Vec<Transform3> = poses_in.to_vec();

    // Row layout: each mate occupies a contiguous block.
    let row_of: Vec<usize> = row_offsets(mates);
    let m_rows: usize = mates.iter().map(residual_len).sum();
    let n = 6 * float_indices.len();

    // Nothing to move, or nothing constraining: report the residual as-is.
    if n == 0 || m_rows == 0 {
        let r = residual_vector(mates, &poses, &row_of, m_rows);
        let norm = r.norm();
        return SolveResult {
            transforms: poses,
            status: status_for(norm, opts.tolerance),
            residual_norm: norm,
            iterations: 0,
            // No mates ⇒ every floating DOF is free.
            free_dof: if m_rows == 0 { n } else { 0 },
        };
    }

    // Closed-form fast path: a single floating instance seated by exactly one
    // concentric + one coincident mate against ground.
    if let Some(seated) = seat_concentric_coincident(poses_in, fixed, mates) {
        let mut trial = poses.clone();
        trial[float_indices[0]] = seated;
        let r = residual_vector(mates, &trial, &row_of, m_rows);
        let norm = r.norm();
        if norm <= opts.tolerance {
            let free_dof = free_dof(mates, &trial, &float_indices, opts.rank_tolerance);
            return SolveResult {
                transforms: trial,
                status: SolveStatus::Converged,
                residual_norm: norm,
                iterations: 0,
                free_dof,
            };
        }
        // Precondition not really met (non-planar seat geometry); fall through.
    }

    // General Levenberg–Marquardt.
    let mut lambda = opts.initial_lambda;
    let mut iterations = 0;
    let mut r = residual_vector(mates, &poses, &row_of, m_rows);
    let mut cost = r.norm_squared();

    while iterations < opts.max_iterations {
        if r.norm() <= opts.tolerance {
            break;
        }
        let j = jacobian(mates, &mut poses, &float_indices, &row_of, m_rows, n);
        // Levenberg–Marquardt in the SVD basis: `δ = -V diag(σ/(σ²+λ)) Uᵀ r`.
        // Assemblies are inherently rank-deficient (under-constrained parts
        // retain free DOF), so a null singular value must contribute *zero*
        // step — the `σ/(σ²+λ)` filter does exactly that, and it lets λ shrink
        // toward a full Gauss–Newton step (quadratic terminal convergence)
        // without the normal-equations conditioning blowing up.
        let svd = j.svd(true, true);

        // Inner loop: grow damping until the step reduces cost.
        let mut accepted = false;
        loop {
            let step = lm_step(&svd, &r, lambda);

            let mut trial = poses.clone();
            apply_step(&mut trial, &float_indices, &step);
            let r_trial = residual_vector(mates, &trial, &row_of, m_rows);
            let cost_trial = r_trial.norm_squared();

            if cost_trial < cost {
                poses = trial;
                r = r_trial;
                cost = cost_trial;
                lambda = (lambda / LAMBDA_DOWN).max(LAMBDA_MIN);
                accepted = true;
                break;
            }
            lambda *= LAMBDA_UP;
            if lambda > LAMBDA_MAX {
                break;
            }
        }

        iterations += 1;
        if !accepted {
            // Damping saturated without progress: converged to a (possibly
            // non-zero) least-squares minimum.
            break;
        }
    }

    let residual_norm = r.norm();
    let free_dof = free_dof(mates, &poses, &float_indices, opts.rank_tolerance);
    SolveResult {
        transforms: poses,
        status: status_for(residual_norm, opts.tolerance),
        residual_norm,
        iterations,
        free_dof,
    }
}

fn status_for(residual_norm: f64, tolerance: f64) -> SolveStatus {
    if residual_norm <= tolerance {
        SolveStatus::Converged
    } else {
        SolveStatus::OverConstrained
    }
}

const LAMBDA_UP: f64 = 3.0;
const LAMBDA_DOWN: f64 = 3.0;
const LAMBDA_MIN: f64 = 1e-12;
const LAMBDA_MAX: f64 = 1e12;
/// Finite-difference step for the numeric central-difference Jacobian. Chosen
/// near the central-difference optimum (∛macheps ≈ 6e-6) so truncation and
/// roundoff error both stay small.
const FD_EPS: f64 = 1e-6;

/// The Levenberg–Marquardt step in the Jacobian's SVD basis:
/// `δ = -Σ_i [σ_i / (σ_i² + λ)] (uᵢᵀ r) vᵢ`.
///
/// A null singular value (`σ_i = 0`, a free DOF) contributes nothing, so the
/// step never excites unconstrained directions; as `λ → 0` the constrained
/// directions approach the full Gauss–Newton step `-(uᵢᵀr)/σ_i · vᵢ`.
fn lm_step(
    svd: &nalgebra::SVD<f64, nalgebra::Dyn, nalgebra::Dyn>,
    r: &DVector<f64>,
    lambda: f64,
) -> DVector<f64> {
    let u = svd.u.as_ref().expect("SVD computed with u");
    let v_t = svd.v_t.as_ref().expect("SVD computed with v_t");
    let sv = &svd.singular_values;
    let utr = u.transpose() * r; // length = #singular values
    let k = sv.len();
    let mut y = DVector::zeros(k);
    for i in 0..k {
        let s = sv[i];
        y[i] = -s / (s * s + lambda) * utr[i];
    }
    v_t.transpose() * y // length n
}

/// The starting row of each mate's residual block.
fn row_offsets(mates: &[Mate]) -> Vec<usize> {
    let mut rows = Vec::with_capacity(mates.len());
    let mut acc = 0;
    for m in mates {
        rows.push(acc);
        acc += residual_len(m);
    }
    rows
}

/// Number of scalar residuals a mate contributes.
fn residual_len(mate: &Mate) -> usize {
    match mate.kind {
        MateKind::Coincident => match (mate.a.feature, mate.b.feature) {
            (Feature::Plane { .. }, Feature::Plane { .. }) => 4,
            (Feature::Point { .. }, Feature::Plane { .. })
            | (Feature::Plane { .. }, Feature::Point { .. }) => 1,
            _ => 0, // invalid pairing; Mate::is_valid rejects it upstream
        },
        MateKind::Concentric => match (mate.a.feature, mate.b.feature) {
            (Feature::Axis { .. }, Feature::Axis { .. }) => 6,
            _ => 0,
        },
        MateKind::Distance => match (mate.a.feature, mate.b.feature) {
            (Feature::Plane { .. }, Feature::Plane { .. })
            | (Feature::Point { .. }, Feature::Point { .. }) => 1,
            _ => 0,
        },
    }
}

/// Write a mate's residual block into `out` (length [`residual_len`]).
fn write_residual(mate: &Mate, poses: &[Transform3], out: &mut [f64]) {
    let a = mate.a.feature.to_world(&poses[mate.a.instance]);
    let b = mate.b.feature.to_world(&poses[mate.b.instance]);
    match mate.kind {
        MateKind::Coincident => match (a, b) {
            (
                Feature::Plane {
                    point: pa,
                    normal: na,
                },
                Feature::Plane {
                    point: pb,
                    normal: nb,
                },
            ) => {
                let s = na + nb; // anti-parallel ⇒ zero
                out[0] = s.x;
                out[1] = s.y;
                out[2] = s.z;
                out[3] = na.dot(&(pa - pb)); // co-planar
            }
            (
                Feature::Point { point: pa },
                Feature::Plane {
                    point: pb,
                    normal: nb,
                },
            ) => {
                out[0] = nb.dot(&(pa - pb));
            }
            (
                Feature::Plane {
                    point: pa,
                    normal: na,
                },
                Feature::Point { point: pb },
            ) => {
                out[0] = na.dot(&(pa - pb));
            }
            _ => {}
        },
        MateKind::Concentric => {
            if let (
                Feature::Axis {
                    point: pa,
                    direction: da,
                },
                Feature::Axis {
                    point: pb,
                    direction: db,
                },
            ) = (a, b)
            {
                let cross = da.cross(&db); // collinear ⇒ zero
                out[0] = cross.x;
                out[1] = cross.y;
                out[2] = cross.z;
                let w = pa - pb;
                let rej = w - da * da.dot(&w); // component of w off the axis
                out[3] = rej.x;
                out[4] = rej.y;
                out[5] = rej.z;
            }
        }
        MateKind::Distance => {
            let value = mate.value.unwrap_or(0.0);
            match (a, b) {
                (
                    Feature::Plane {
                        point: pa,
                        normal: na,
                    },
                    Feature::Plane { point: pb, .. },
                ) => {
                    out[0] = na.dot(&(pa - pb)) - value;
                }
                (Feature::Point { point: pa }, Feature::Point { point: pb }) => {
                    out[0] = (pa - pb).norm() - value;
                }
                _ => {}
            }
        }
    }
}

/// The full stacked residual vector `F(x)`.
fn residual_vector(
    mates: &[Mate],
    poses: &[Transform3],
    row_of: &[usize],
    m_rows: usize,
) -> DVector<f64> {
    let mut r = DVector::zeros(m_rows);
    for (mi, mate) in mates.iter().enumerate() {
        let len = residual_len(mate);
        if len == 0 {
            continue;
        }
        let start = row_of[mi];
        write_residual(mate, poses, &mut r.as_mut_slice()[start..start + len]);
    }
    r
}

/// Apply pose increment `d = (u, ω)` to `pose`, renormalizing the quaternion.
fn increment(pose: &Transform3, d: &[f64; 6]) -> Transform3 {
    let u = Vector3::new(d[0], d[1], d[2]);
    let omega = Vector3::new(d[3], d[4], d[5]);
    let rot = UnitQuaternion::from_scaled_axis(omega) * pose.rotation;
    // Renormalize every iteration to hold the unit constraint against drift.
    let rot = UnitQuaternion::new_normalize(*rot.quaternion());
    let trans = pose.translation.vector + u;
    Transform3::from_parts(Translation3::from(trans), rot)
}

/// Apply an LM step (length `6 * #floating`) to the floating instances.
fn apply_step(poses: &mut [Transform3], float_indices: &[usize], step: &DVector<f64>) {
    for (fi, &inst) in float_indices.iter().enumerate() {
        let base = fi * 6;
        let d = [
            step[base],
            step[base + 1],
            step[base + 2],
            step[base + 3],
            step[base + 4],
            step[base + 5],
        ];
        poses[inst] = increment(&poses[inst], &d);
    }
}

/// Central-difference Jacobian. `poses` is mutated transiently (each perturbed
/// pose is restored) so callers see it unchanged on return.
fn jacobian(
    mates: &[Mate],
    poses: &mut [Transform3],
    float_indices: &[usize],
    row_of: &[usize],
    m_rows: usize,
    n: usize,
) -> DMatrix<f64> {
    let mut jac = DMatrix::zeros(m_rows, n);

    // Which mates touch each instance, so a perturbation re-evaluates only the
    // affected residual blocks (preserving sparsity).
    let mut mates_of: Vec<Vec<usize>> = vec![Vec::new(); poses.len()];
    for (mi, mate) in mates.iter().enumerate() {
        if residual_len(mate) == 0 {
            continue;
        }
        mates_of[mate.a.instance].push(mi);
        if mate.b.instance != mate.a.instance {
            mates_of[mate.b.instance].push(mi);
        }
    }

    let mut buf_p = [0.0_f64; 6];
    let mut buf_m = [0.0_f64; 6];
    for (fi, &inst) in float_indices.iter().enumerate() {
        let saved = poses[inst];
        for axis in 0..6 {
            let col = fi * 6 + axis;
            let mut d = [0.0_f64; 6];
            d[axis] = FD_EPS;
            let pose_p = increment(&saved, &d);
            d[axis] = -FD_EPS;
            let pose_m = increment(&saved, &d);

            for &mi in &mates_of[inst] {
                let mate = &mates[mi];
                let len = residual_len(mate);
                let start = row_of[mi];

                poses[inst] = pose_p;
                write_residual(mate, poses, &mut buf_p[..len]);
                poses[inst] = pose_m;
                write_residual(mate, poses, &mut buf_m[..len]);

                for k in 0..len {
                    jac[(start + k, col)] = (buf_p[k] - buf_m[k]) / (2.0 * FD_EPS);
                }
            }
            poses[inst] = saved;
        }
    }
    jac
}

/// Remaining unconstrained DOF: the rank deficiency of the constraint Jacobian
/// at `poses`. Computed from its singular values.
fn free_dof(mates: &[Mate], poses: &[Transform3], float_indices: &[usize], rank_tol: f64) -> usize {
    let n = 6 * float_indices.len();
    if n == 0 {
        return 0;
    }
    let row_of = row_offsets(mates);
    let m_rows: usize = mates.iter().map(residual_len).sum();
    if m_rows == 0 {
        return n;
    }
    let mut poses = poses.to_vec();
    let j = jacobian(mates, &mut poses, float_indices, &row_of, m_rows, n);
    let sv = j.singular_values();
    let max_sv = sv.iter().cloned().fold(0.0_f64, f64::max);
    if max_sv == 0.0 {
        return n;
    }
    let thresh = max_sv * rank_tol;
    let rank = sv.iter().filter(|&&s| s > thresh).count();
    n.saturating_sub(rank)
}

/// Closed-form pose for the canonical "drop a bolt in a hole and seat the head"
/// stack: a single floating instance located by exactly one concentric axis
/// mate and one coincident plane mate, both against fixed (ground) features,
/// with the bolt's local axis parallel to its seat-plane normal.
///
/// `poses[i]` / `fixed[i]` describe the instances (same convention as
/// [`solve_mates`]). Returns the seated pose (rotation about the axis left at
/// the instance's current spin — that DOF is free), or `None` when the
/// assembly does not match the pattern or the geometry is non-canonical (axis
/// not perpendicular to the seat plane), in which case [`solve_mates`] falls
/// back to iterative LM.
pub fn seat_concentric_coincident(
    poses: &[Transform3],
    fixed: &[bool],
    mates: &[Mate],
) -> Option<Transform3> {
    // Exactly one floating instance.
    let mut float_iter = (0..fixed.len()).filter(|&i| !fixed[i]);
    let f_idx = float_iter.next()?;
    if float_iter.next().is_some() {
        return None;
    }

    // Exactly one concentric + one coincident mate, each floating↔fixed.
    let mut concentric: Option<&Mate> = None;
    let mut coincident: Option<&Mate> = None;
    for mate in mates {
        let touches_float = mate.a.instance == f_idx || mate.b.instance == f_idx;
        let other = if mate.a.instance == f_idx {
            mate.b.instance
        } else {
            mate.a.instance
        };
        let other_fixed = fixed.get(other).copied().unwrap_or(false);
        if !touches_float || !other_fixed || mate.a.instance == mate.b.instance {
            return None;
        }
        match mate.kind {
            MateKind::Concentric if concentric.is_none() => concentric = Some(mate),
            MateKind::Coincident if coincident.is_none() => coincident = Some(mate),
            _ => return None, // duplicate or unsupported kind ⇒ not this pattern
        }
    }
    let concentric = concentric?;
    let coincident = coincident?;

    // Bolt (local, on the floating instance) axis + seat plane; hole axis +
    // target plane in world (on the fixed instances).
    let (bolt_pa, bolt_da) = local_axis(concentric, f_idx)?;
    let (hole_pa, hole_da) = world_axis(concentric, f_idx, poses)?;
    let (bolt_pp, bolt_np) = local_plane(coincident, f_idx)?;
    let (tgt_pp, tgt_np) = world_plane(coincident, f_idx, poses)?;

    // Canonical fastener: the bolt's axis is parallel to its seat-plane normal.
    if bolt_da.cross(&bolt_np).norm() > 1e-9 {
        return None;
    }
    // sign so the seated normal is anti-parallel to the target: R·np = −tgt_np.
    // np = s·da locally ⇒ R·da = −tgt_np / s.
    let s = bolt_da.dot(&bolt_np).signum();
    let d_target = -tgt_np / s;
    // The hole axis must be parallel to the target normal (a real counterbore).
    if hole_da.cross(&d_target).norm() > 1e-6 {
        return None;
    }

    // Rotation taking the bolt axis to the target direction; spin about the
    // axis is free, so the minimal rotation is a valid representative.
    let rot = rotation_between(&bolt_da, &d_target)?;

    // Place so the bolt axis point lands on the hole line and the seat plane is
    // flush with the target. With t = hole_pa − R·bolt_pa + μ·hole_da, the perp
    // constraint is satisfied for any μ; μ then makes the planes flush.
    let a = rot * bolt_pa.coords; // R·bolt_pa
    let bpt = rot * bolt_pp.coords; // R·bolt_pp
    let denom = tgt_np.dot(&hole_da);
    if denom.abs() < 1e-9 {
        return None; // axis lies in the seat plane; not a seat
    }
    // tgt_np · (R·bolt_pp + t − tgt_pp) = 0, with t = hole_pa − a + μ·hole_da.
    let base = bpt + (hole_pa.coords - a) - tgt_pp.coords;
    let mu = -tgt_np.dot(&base) / denom;
    let t = hole_pa.coords - a + hole_da * mu;

    Some(Transform3::from_parts(Translation3::from(t), rot))
}

/// Minimal rotation taking unit `from` to unit `to`, handling the exactly
/// anti-parallel case (where [`UnitQuaternion::rotation_between`] is `None`
/// because the axis is ambiguous) with a π rotation about any perpendicular.
fn rotation_between(from: &Vector3, to: &Vector3) -> Option<UnitQuaternion<f64>> {
    if let Some(q) = UnitQuaternion::rotation_between(from, to) {
        return Some(q);
    }
    // Opposite directions: pick a perpendicular axis and spin π.
    let seed = if from.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    let axis = nalgebra::Unit::new_normalize(from.cross(&seed));
    Some(UnitQuaternion::from_axis_angle(&axis, std::f64::consts::PI))
}

// --- feature extraction helpers for the closed form ---

/// The axis feature on the floating side of a concentric mate, in local frame.
fn local_axis(mate: &Mate, f_idx: usize) -> Option<(Point3, Vector3)> {
    let fref = if mate.a.instance == f_idx {
        &mate.a
    } else {
        &mate.b
    };
    match fref.feature {
        Feature::Axis { point, direction } => Some((point, direction)),
        _ => None,
    }
}

/// The axis feature on the fixed side of a concentric mate, in world frame.
fn world_axis(mate: &Mate, f_idx: usize, poses: &[Transform3]) -> Option<(Point3, Vector3)> {
    let fref = if mate.a.instance == f_idx {
        &mate.b
    } else {
        &mate.a
    };
    match fref.feature.to_world(&poses[fref.instance]) {
        Feature::Axis { point, direction } => Some((point, direction)),
        _ => None,
    }
}

/// The plane feature on the floating side of a coincident mate, in local frame.
fn local_plane(mate: &Mate, f_idx: usize) -> Option<(Point3, Vector3)> {
    let fref = if mate.a.instance == f_idx {
        &mate.a
    } else {
        &mate.b
    };
    match fref.feature {
        Feature::Plane { point, normal } => Some((point, normal)),
        _ => None,
    }
}

/// The plane feature on the fixed side of a coincident mate, in world frame.
fn world_plane(mate: &Mate, f_idx: usize, poses: &[Transform3]) -> Option<(Point3, Vector3)> {
    let fref = if mate.a.instance == f_idx {
        &mate.b
    } else {
        &mate.a
    };
    match fref.feature.to_world(&poses[fref.instance]) {
        Feature::Plane { point, normal } => Some((point, normal)),
        _ => None,
    }
}
