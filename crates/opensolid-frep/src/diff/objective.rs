//! Differentiable objectives, computed **on the field** — no meshing.
//!
//! [`mass_properties`](crate::massprops) — in `opensolid-kernel` — integrates
//! over triangles via the divergence theorem. That is exact and it is the
//! right tool for reporting, but it is useless to an optimiser: the mesh
//! connectivity changes discontinuously as parameters move (a vertex crosses
//! a cell, a triangle appears), so `d(volume)/dθ` through a mesher is
//! garbage. The whole mesh is a non-differentiable function of the design.
//!
//! So we integrate the *field* instead. Volume becomes
//!
//! ```text
//! V(θ) = ∫ occupancy(f(p; θ)) dp
//! ```
//!
//! where [`occupancy`](Occupancy) is a smooth 1→0 ramp across the surface.
//! Every term is smooth in `θ`, so forward-mode AD carries `∂V/∂θ` straight
//! through the quadrature. See `docs/design/DIFFERENTIABLE.md` §5.
//!
//! # Accuracy
//!
//! These are Riemann sums with a smeared boundary: expect a percent-level
//! bias against exact analytic volume, controlled by
//! [`Occupancy::resolution`] and [`Occupancy::band`]. That is fine — an
//! optimiser needs a *consistent, differentiable* objective, not an exact
//! one. Report final numbers with the exact mesh-based
//! `mass_properties`; steer with these.

use super::dual::Dual;
use super::param::ParamSdf;
use super::scalar::Scalar;
use super::vec::Vec3;
use opensolid_core::types::{BoundingBox3, Point3};

/// Band width, in grid cells, used by [`Occupancy::for_domain`].
///
/// This constant is set by the *gradient*, not the value. `dV/dθ` is an
/// integral over the band alone — only sample points with `|f| < band` carry
/// any derivative at all — so the midpoint rule has to resolve the ramp
/// derivative across the band. If `∫s' du` does not come out to −1, **every
/// gradient is scaled wrong**, by the same factor, at every resolution.
///
/// That error does not vanish as the grid refines: band and cell shrink
/// together, so their ratio — and the mis-scaling — is fixed. It is a bias
/// you cannot buy your way out of with resolution, which is why it is worth
/// a named constant and a test
/// ([`band_resolves_the_ramp_derivative`](self)).
///
/// Worst-case error in `∫s' du` over all sub-cell alignments of the surface:
///
/// | band (cells) | 2 | **3** | 4 | 6 |
/// |---|---|---|---|---|
/// | cubic C¹ ramp | 6.3% | 2.8% | 1.6% | 0.7% |
/// | quintic C² ramp ([`occupancy_ramp`]) | 0.39% | **0.077%** | 0.02% | 0.005% |
///
/// Three cells of the quintic ramp puts the gradient scale within 0.1% while
/// keeping the band narrow enough that the volume bias stays small (and that
/// bias *does* fall as `1/resolution²`).
///
/// The error is worst for **axis-aligned planar faces**: every point on the
/// face aliases against the grid identically, so the errors add coherently
/// rather than averaging out as they do on a curved surface. A sphere
/// therefore looks fine at settings where a cube is visibly wrong — and CAD
/// parts are mostly axis-aligned faces, so the table above is the case that
/// matters.
pub const BAND_CELLS: f64 = 3.0;

/// The smooth occupancy ramp and the quadrature grid it is sampled on.
#[derive(Debug, Clone, Copy)]
pub struct Occupancy {
    /// Samples per axis. Cost is `resolution³`; error falls roughly as
    /// `1/resolution`.
    pub resolution: usize,
    /// Half-width of the smoothing band, in model units. The occupancy
    /// ramps 1→0 over `f ∈ [-band, band]`.
    ///
    /// This is the key knob. Too small and the ramp derivative is
    /// under-resolved, which mis-scales every gradient; too large and the
    /// value is a blurred version of the true volume. Prefer
    /// [`Occupancy::for_domain`], which sizes it to [`BAND_CELLS`] cells —
    /// see that constant for why the trade-off lands where it does.
    pub band: f64,
}

impl Occupancy {
    pub fn new(resolution: usize, band: f64) -> Self {
        Self { resolution, band }
    }

    /// Band sized to [`BAND_CELLS`] cells of the domain at this resolution.
    /// The right default: wide enough that the gradient quadrature is
    /// accurate, narrow enough that the value is not over-blurred.
    pub fn for_domain(resolution: usize, domain: &BoundingBox3) -> Self {
        let cell = domain.extents().max() / resolution as f64;
        Self::new(resolution, BAND_CELLS * cell)
    }
}

impl Default for Occupancy {
    fn default() -> Self {
        Self::new(48, 0.1)
    }
}

/// Smooth indicator of "inside": 1 well inside, 0 well outside, a C² quintic
/// ramp across the band.
///
/// `s(u) = 1 - S((u+1)/2)` on `u = f/band ∈ [-1, 1]` (clamped outside), where
/// `S(t) = 6t⁵ - 15t⁴ + 10t³` is the quintic smootherstep. So `s(-1) = 1`,
/// `s(1) = 0`, and both `s'` and `s''` vanish at `±1`.
///
/// # Why quintic and not the obvious cubic
///
/// The cubic smoothstep is the reflex choice and it is C¹ — enough that the
/// objective has no kink. But the *quadrature* cares about more than that.
/// The band integral is a midpoint sum of a compactly-supported kernel, and
/// by Euler–Maclaurin its error is governed by the derivatives that fail to
/// vanish at the ends of the support. The cubic leaves `s'' ≠ 0` there and
/// converges as `O(Δu²)`; the quintic kills `s''` too and converges as
/// `O(Δu⁴)`. Same arithmetic cost, one more term — and 36× the accuracy at
/// the band width we use (see [`BAND_CELLS`]).
///
/// `ds/df` is non-zero **only inside the band**, so the volume gradient is
/// supported on a shell around the surface. That is not an accident: as
/// `band → 0` the sum converges to the classical shape derivative, a surface
/// integral over the boundary.
fn occupancy_ramp<T: Scalar>(f: T, band: f64) -> T {
    let u = (f / T::cst(band)).clamp(-1.0, 1.0);
    let t = (u + T::one()) * T::cst(0.5);
    // 1 - S(t), with S(t) = t³(6t² - 15t + 10) in Horner form.
    T::one() - t * t * t * (T::cst(6.0) * t * t - T::cst(15.0) * t + T::cst(10.0))
}

/// Iterate the midpoints of the quadrature grid over `domain`.
fn grid_points(domain: &BoundingBox3, resolution: usize) -> impl Iterator<Item = Point3> + '_ {
    let n = resolution;
    let ext = domain.extents();
    let step = Vec3::<f64>::new(ext.x / n as f64, ext.y / n as f64, ext.z / n as f64);
    (0..n).flat_map(move |i| {
        (0..n).flat_map(move |j| {
            (0..n).map(move |k| {
                Point3::new(
                    domain.min.x + (i as f64 + 0.5) * step.x,
                    domain.min.y + (j as f64 + 0.5) * step.y,
                    domain.min.z + (k as f64 + 0.5) * step.z,
                )
            })
        })
    })
}

/// The volume of one quadrature cell.
fn cell_volume(domain: &BoundingBox3, resolution: usize) -> f64 {
    let e = domain.extents();
    (e.x * e.y * e.z) / (resolution as f64).powi(3)
}

/// Volume enclosed by the field, and `∂V/∂θᵢ` for every parameter.
///
/// `domain` must contain the whole solid — anything outside is simply not
/// counted, silently. Sized from the shape's bounds plus a margin.
///
/// # Example
///
/// ```
/// use opensolid_frep::diff::{objective::{volume, Occupancy}, ParamSdf, Scalar, Vec3, field};
/// use opensolid_core::types::{BoundingBox3, Point3};
///
/// struct Ball;
/// impl ParamSdf<1> for Ball {
///     fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; 1]) -> T {
///         field::sphere(p, Vec3::zero(), params[0])
///     }
/// }
///
/// let domain = BoundingBox3::new(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0));
/// let (v, g) = volume(&Ball, &[1.0], &domain, &Occupancy::for_domain(64, &domain));
///
/// // Close to the analytic 4π/3, and dV/dr close to the analytic 4πr².
/// // Both read ~1% high at this resolution — the band bias; see § Accuracy.
/// assert!((v - 4.0 / 3.0 * std::f64::consts::PI).abs() < 0.1);
/// assert!((g[0] - 4.0 * std::f64::consts::PI).abs() < 0.2);
/// ```
pub fn volume<S: ParamSdf<N> + ?Sized, const N: usize>(
    shape: &S,
    params: &[f64; N],
    domain: &BoundingBox3,
    occ: &Occupancy,
) -> (f64, [f64; N]) {
    let seeded = Dual::<N>::seed_all(params);
    let dv = cell_volume(domain, occ.resolution);

    let mut value = 0.0;
    let mut grad = [0.0; N];
    for p in grid_points(domain, occ.resolution) {
        let f = shape.field(Vec3::from_point(&p), &seeded);
        let o = occupancy_ramp(f, occ.band);
        value += o.v;
        for (g, d) in grad.iter_mut().zip(o.d) {
            *g += d;
        }
    }
    (value * dv, std::array::from_fn(|i| grad[i] * dv))
}

/// Mass and `∂m/∂θ` at uniform `density`.
///
/// Trivially [`volume`] scaled — but it is the quantity design targets are
/// actually written against ("this bracket must come in under 250 g"), so it
/// gets a name.
pub fn mass<S: ParamSdf<N> + ?Sized, const N: usize>(
    shape: &S,
    params: &[f64; N],
    domain: &BoundingBox3,
    occ: &Occupancy,
    density: f64,
) -> (f64, [f64; N]) {
    let (v, g) = volume(shape, params, domain, occ);
    (v * density, std::array::from_fn(|i| g[i] * density))
}

/// Centre of gravity (uniform density) and its parameter gradient.
///
/// Returns `[(value, grad); 3]`, one entry per axis: `out[a].0` is the
/// centroid's `a` coordinate and `out[a].1[i]` is `∂cₐ/∂θᵢ`.
///
/// The centroid is a ratio of two integrals, `∫p·o / ∫o`. Both are
/// accumulated as duals and the division is done *in dual arithmetic*, so
/// the quotient rule is applied for us — no hand-derived sensitivity.
pub fn centroid<S: ParamSdf<N> + ?Sized, const N: usize>(
    shape: &S,
    params: &[f64; N],
    domain: &BoundingBox3,
    occ: &Occupancy,
) -> [(f64, [f64; N]); 3] {
    let seeded = Dual::<N>::seed_all(params);

    let mut moment = [Dual::<N>::cst(0.0); 3];
    let mut total = Dual::<N>::cst(0.0);
    for p in grid_points(domain, occ.resolution) {
        let o = occupancy_ramp(shape.field(Vec3::from_point(&p), &seeded), occ.band);
        total = total + o;
        for (a, coord) in [p.x, p.y, p.z].into_iter().enumerate() {
            moment[a] = moment[a] + o * Dual::cst(coord);
        }
    }
    // The cell volume cancels in the ratio, so it is never applied.
    std::array::from_fn(|a| {
        let c = moment[a] / total;
        (c.v, c.d)
    })
}

/// Smooth minimum of the field over `probes`, and its parameter gradient —
/// a differentiable **clearance**.
///
/// `probes` sample a keep-out region (a connector envelope, a neighbouring
/// part). The result approximates `min_p f(p; θ)`: the signed distance from
/// the solid to the nearest probe. Positive means the solid clears every
/// probe; the constraint is `clearance >= required`.
///
/// A hard `min` is non-differentiable where the winning probe changes, and
/// worse, its gradient sees only *one* probe — so an optimiser pushes the
/// part off one probe and straight into its neighbour, and chatters. The
/// log-sum-exp softmin at temperature `softness` blends the gradients of all
/// near-active probes instead, which is both smooth and better-behaved.
/// As `softness → 0` it converges to the hard min.
///
/// # Panics
///
/// If `probes` is empty or `softness <= 0`.
pub fn clearance<S: ParamSdf<N> + ?Sized, const N: usize>(
    shape: &S,
    params: &[f64; N],
    probes: &[Point3],
    softness: f64,
) -> (f64, [f64; N]) {
    assert!(!probes.is_empty(), "clearance needs at least one probe");
    assert!(softness > 0.0, "softness must be positive");

    let seeded = Dual::<N>::seed_all(params);
    let ds: Vec<Dual<N>> = probes
        .iter()
        .map(|p| shape.field(Vec3::from_point(p), &seeded))
        .collect();

    // Shift by the hard min before exponentiating: exp(-(d - m)/s) is then
    // bounded by 1, so a deeply-inside probe cannot overflow to inf. The
    // shift is exact — it factors straight back out of the log.
    let m = ds
        .iter()
        .copied()
        .reduce(|a, b| a.min(b))
        .expect("non-empty");
    let s = Dual::<N>::cst(softness);
    let sum = ds
        .iter()
        .map(|&d| ((m - d) / s).exp())
        .reduce(|a, b| a + b)
        .expect("non-empty");
    let out = m - s * sum.ln();
    (out.v, out.d)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diff::field;
    use std::f64::consts::PI;

    struct Ball;
    impl ParamSdf<1> for Ball {
        fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; 1]) -> T {
            field::sphere(p, Vec3::zero(), params[0])
        }
    }

    /// A cube of parameterised half-extent — exact volume `(2h)³`.
    struct Cube;
    impl ParamSdf<1> for Cube {
        fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; 1]) -> T {
            field::box3(p, Vec3::zero(), Vec3::splat(params[0]))
        }
    }

    fn domain(r: f64) -> BoundingBox3 {
        BoundingBox3::new(Point3::new(-r, -r, -r), Point3::new(r, r, r))
    }

    #[test]
    fn occupancy_ramp_is_one_inside_zero_outside() {
        assert_eq!(occupancy_ramp(-5.0, 0.1), 1.0);
        assert_eq!(occupancy_ramp(5.0, 0.1), 0.0);
        // Exactly on the surface it is a half — the ramp is centred.
        assert!((occupancy_ramp(0.0, 0.1) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn occupancy_ramp_is_monotone() {
        let mut prev = 1.1;
        for i in 0..=40 {
            let f = -0.2 + 0.01 * i as f64;
            let o = occupancy_ramp(f, 0.1);
            assert!(o <= prev + 1e-12, "ramp must be non-increasing");
            prev = o;
        }
    }

    #[test]
    fn occupancy_gradient_vanishes_outside_the_band() {
        // Only points within the band contribute derivative — the shell.
        let f = Dual::<1>::seed(0.5, 0);
        assert_eq!(occupancy_ramp(f, 0.1).grad(), [0.0]);
        let f = Dual::<1>::seed(0.0, 0);
        assert!(occupancy_ramp(f, 0.1).grad()[0].abs() > 0.0);
    }

    /// Sum `ds/du` over the band the way the volume quadrature samples it:
    /// grid midpoints, spacing `1/BAND_CELLS` in `u`, with the surface
    /// sitting at sub-cell offset `align`. Exact answer is -1.
    ///
    /// Differentiates the real [`occupancy_ramp`] with a dual rather than
    /// re-deriving `s'` by hand — so this cannot silently pass against a
    /// stale formula if the ramp is ever changed.
    fn ramp_derivative_sum(band_cells: f64, align: f64) -> f64 {
        let du = 1.0 / band_cells;
        let mut total = 0.0;
        let mut i = -(band_cells.ceil() as i64) - 2;
        loop {
            let u = (i as f64 + align) * du;
            i += 1;
            if u < -1.0 {
                continue;
            }
            if u > 1.0 {
                break;
            }
            // band = 1 here, so d/df and d/du coincide.
            total += occupancy_ramp(Dual::<1>::seed(u, 0), 1.0).grad()[0];
        }
        total * du
    }

    /// The band must resolve the ramp derivative well enough that
    /// `∫s' du = -1`, or every gradient comes out scaled by the same wrong
    /// factor — at *every* resolution, since band and cell shrink together.
    /// This is the property [`BAND_CELLS`] is chosen for.
    #[test]
    fn band_resolves_the_ramp_derivative() {
        // Worst case over sub-cell alignments: an axis-aligned face puts the
        // whole face at one alignment, so the worst case is the real case.
        for align in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9] {
            let integral = ramp_derivative_sum(BAND_CELLS, align);
            assert!(
                (integral + 1.0).abs() < 2e-3,
                "align {align}: ∫s' du = {integral}, must be ≈ -1 or gradients are mis-scaled"
            );
        }
    }

    /// The quintic ramp is not decoration: the C¹ cubic mis-scales gradients
    /// by ~3% at this band, and no amount of resolution removes it.
    #[test]
    fn quintic_ramp_beats_the_cubic_it_replaced() {
        fn cubic_sum(band_cells: f64, align: f64) -> f64 {
            let du = 1.0 / band_cells;
            let (mut total, mut i) = (0.0, -(band_cells.ceil() as i64) - 2);
            loop {
                let u = (i as f64 + align) * du;
                i += 1;
                if u < -1.0 {
                    continue;
                }
                if u > 1.0 {
                    break;
                }
                total += -0.75 * (1.0 - u * u); // d/du of 1/2 - 3u/4 + u³/4
            }
            total * du
        }
        let worst = |f: fn(f64, f64) -> f64| {
            [0.0, 0.1, 0.25, 0.5, 0.75, 0.9]
                .iter()
                .map(|&a| (f(BAND_CELLS, a) + 1.0).abs())
                .fold(0.0, f64::max)
        };
        let quintic = worst(ramp_derivative_sum);
        let cubic = worst(cubic_sum);
        assert!(
            quintic < cubic * 0.1,
            "quintic {quintic} should be far better than cubic {cubic}"
        );
    }

    /// The failure that set `BAND_CELLS`: an axis-aligned face aliases
    /// coherently against the grid, so a too-narrow band mis-scales the
    /// gradient by ~11% even though the volume looks fine.
    #[test]
    fn narrow_band_mis_scales_the_gradient_of_an_aligned_face() {
        let d = domain(2.0);
        let cell = d.extents().max() / 64.0;
        let narrow = volume(&Cube, &[1.0], &d, &Occupancy::new(64, 0.75 * cell)).1[0];
        let good = volume(&Cube, &[1.0], &d, &Occupancy::new(64, BAND_CELLS * cell)).1[0];
        // Analytic dV/dh = 24.
        assert!((narrow - 24.0).abs() > 1.0, "narrow band was {narrow}");
        assert!((good - 24.0).abs() < 1.0, "default band was {good}");
    }

    /// Tolerances here are the *measured* band bias at this resolution, not
    /// aspirations: the smeared boundary biases volume high by ~1.4% at
    /// res 64, falling as `1/res²`
    /// ([`volume_refines_toward_analytic_with_resolution`](self)).
    #[test]
    fn cube_volume_matches_analytic() {
        let d = domain(2.0);
        let occ = Occupancy::for_domain(64, &d);
        let (v, g) = volume(&Cube, &[1.0], &d, &occ);
        // Exact volume (2h)³ = 8; dV/dh = 3·(2h)²·2 = 24.
        assert!((v - 8.0).abs() < 0.2, "volume {v}");
        assert!((g[0] - 24.0).abs() < 0.5, "dV/dh {}", g[0]);
        // The bias has a known sign — the smeared shell adds volume.
        assert!(v > 8.0, "band bias should read high, got {v}");
    }

    #[test]
    fn sphere_volume_and_gradient_match_analytic() {
        let d = domain(2.0);
        let occ = Occupancy::for_domain(64, &d);
        let (v, g) = volume(&Ball, &[1.0], &d, &occ);
        // V = 4πr³/3 ≈ 4.19; dV/dr = surface area = 4πr² ≈ 12.57.
        assert!((v - 4.0 / 3.0 * PI).abs() < 0.1, "volume {v}");
        assert!((g[0] - 4.0 * PI).abs() < 0.2, "dV/dr {}", g[0]);
    }

    /// The gradient must converge too — the cubic ramp this replaced
    /// plateaued at ~1.45% on an axis-aligned face no matter the resolution.
    #[test]
    fn gradient_refines_toward_analytic_with_resolution() {
        let d = domain(2.0);
        let err =
            |res| (volume(&Cube, &[1.0], &d, &Occupancy::for_domain(res, &d)).1[0] - 24.0).abs();
        let (coarse, fine) = (err(32), err(96));
        assert!(fine < coarse * 0.5, "coarse {coarse}, fine {fine}");
        assert!(fine < 0.15, "gradient should converge, not plateau: {fine}");
    }

    /// The whole point: the AD gradient of a *quadrature* must match finite
    /// differences *of that same quadrature*.
    #[test]
    fn volume_gradient_agrees_with_finite_differences() {
        let d = domain(2.0);
        let occ = Occupancy::new(48, 0.09);
        let (_, g) = volume(&Ball, &[1.0], &d, &occ);

        let h = 1e-4;
        let vp = volume(&Ball, &[1.0 + h], &d, &occ).0;
        let vm = volume(&Ball, &[1.0 - h], &d, &occ).0;
        let fd = (vp - vm) / (2.0 * h);
        assert!(
            (g[0] - fd).abs() < 1e-3 * fd.abs().max(1.0),
            "{} vs {fd}",
            g[0]
        );
    }

    #[test]
    fn volume_refines_toward_analytic_with_resolution() {
        let d = domain(2.0);
        let exact = 4.0 / 3.0 * PI;
        let coarse = (volume(&Ball, &[1.0], &d, &Occupancy::for_domain(16, &d)).0 - exact).abs();
        let fine = (volume(&Ball, &[1.0], &d, &Occupancy::for_domain(64, &d)).0 - exact).abs();
        assert!(fine < coarse, "coarse {coarse}, fine {fine}");
    }

    #[test]
    fn mass_scales_volume() {
        let d = domain(2.0);
        let occ = Occupancy::for_domain(32, &d);
        let (v, gv) = volume(&Ball, &[1.0], &d, &occ);
        let (m, gm) = mass(&Ball, &[1.0], &d, &occ, 7.8);
        assert!((m - v * 7.8).abs() < 1e-9);
        assert!((gm[0] - gv[0] * 7.8).abs() < 1e-9);
    }

    #[test]
    fn centroid_of_a_centred_ball_is_the_origin() {
        let d = domain(2.0);
        let c = centroid(&Ball, &[1.0], &d, &Occupancy::for_domain(48, &d));
        for (a, (v, _)) in c.iter().enumerate() {
            assert!(v.abs() < 1e-6, "axis {a} centroid {v}");
        }
    }

    #[test]
    fn centroid_tracks_a_translating_ball() {
        /// Ball translated along x by θ0.
        struct Sliding;
        impl ParamSdf<1> for Sliding {
            fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; 1]) -> T {
                field::sphere(p, Vec3::new(params[0], T::zero(), T::zero()), T::cst(0.8))
            }
        }
        let d = domain(2.0);
        let occ = Occupancy::for_domain(48, &d);
        let c = centroid(&Sliding, &[0.5], &d, &occ);
        // Centroid sits at the ball centre, and moves 1:1 with the parameter.
        assert!((c[0].0 - 0.5).abs() < 1e-3, "x centroid {}", c[0].0);
        assert!((c[0].1[0] - 1.0).abs() < 1e-2, "dcx/dθ {}", c[0].1[0]);
        assert!(c[1].1[0].abs() < 1e-2, "y must not move");
    }

    #[test]
    fn clearance_approaches_hard_min() {
        let probes = [
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 5.0, 0.0),
            Point3::new(0.0, 0.0, 8.0),
        ];
        // Hard min of (3-1, 5-1, 8-1) = 2.
        let (c, _) = clearance(&Ball, &[1.0], &probes, 1e-3);
        assert!((c - 2.0).abs() < 1e-2, "clearance {c}");
    }

    #[test]
    fn clearance_gradient_agrees_with_finite_differences() {
        let probes = [Point3::new(3.0, 0.0, 0.0), Point3::new(0.0, 3.4, 0.0)];
        let (_, g) = clearance(&Ball, &[1.0], &probes, 0.2);
        let h = 1e-6;
        let fd = (clearance(&Ball, &[1.0 + h], &probes, 0.2).0
            - clearance(&Ball, &[1.0 - h], &probes, 0.2).0)
            / (2.0 * h);
        assert!((g[0] - fd).abs() < 1e-5, "{} vs {fd}", g[0]);
        // Growing the ball eats clearance.
        assert!(g[0] < 0.0);
    }

    #[test]
    fn clearance_softmin_is_a_lower_bound_on_hard_min() {
        // log-sum-exp softmin never exceeds the true min.
        let probes = [Point3::new(3.0, 0.0, 0.0), Point3::new(0.0, 3.0, 0.0)];
        let (c, _) = clearance(&Ball, &[1.0], &probes, 0.5);
        assert!(c <= 2.0 + 1e-12, "softmin {c} exceeded hard min 2.0");
    }

    #[test]
    fn clearance_blends_gradients_of_tied_probes() {
        // Two probes exactly tied: a hard min would credit one; the softmin
        // splits the sensitivity between them.
        let probes = [Point3::new(3.0, 0.0, 0.0), Point3::new(0.0, 3.0, 0.0)];
        let (_, g) = clearance(&Ball, &[1.0], &probes, 0.5);
        // Each probe contributes dδ/dr = -1, blended 50/50 → still -1.
        assert!((g[0] - (-1.0)).abs() < 1e-6, "{}", g[0]);
    }

    #[test]
    #[should_panic(expected = "at least one probe")]
    fn clearance_rejects_empty_probes() {
        let _ = clearance(&Ball, &[1.0], &[], 0.1);
    }

    #[test]
    fn grid_points_covers_the_domain_midpoints() {
        let d = BoundingBox3::new(Point3::origin(), Point3::new(1.0, 1.0, 1.0));
        let pts: Vec<_> = grid_points(&d, 2).collect();
        assert_eq!(pts.len(), 8);
        assert!(pts.iter().all(|p| p.x > 0.0 && p.x < 1.0));
        assert!((cell_volume(&d, 2) - 0.125).abs() < 1e-12);
    }
}
