//! 2D sketch profiles and the sweep solids built from them.
//!
//! [`Profile2D`] is a closed loop of straight and circular-arc segments
//! (DXF-style bulge arcs) with an exact 2D signed distance. [`Extrude`]
//! sweeps a profile linearly along +Y and [`Revolve`] sweeps it around the
//! Y axis; both implement [`Sdf`], so the existing meshing pipeline
//! consumes them unchanged.
//!
//! # Field exactness
//!
//! - [`Extrude`] with no draft is an exact signed distance field (the 2D
//!   profile distance is exact and the linear-sweep combination preserves
//!   exactness). With a draft angle the walls tilt: each wall term is the
//!   *exact* perpendicular distance to that tilted plane, so the field stays
//!   sign-correct and Lipschitz ≤ 1 (the default `eval_interval` remains
//!   valid), but like all `min`/`max` CSG the magnitude near tapered corners
//!   underestimates the true distance.
//! - [`Revolve`] over the full turn is exact. A partial revolve is the
//!   intersection (`max`) of the full solid of revolution with an exact
//!   wedge field: sign-correct everywhere, Lipschitz ≤ 1 (so the default
//!   `eval_interval` stays valid), but like all `max` CSG the interior
//!   magnitude near the cut faces underestimates the true distance.

use crate::primitives::Sdf;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3};
use std::f64::consts::{FRAC_PI_2, PI, TAU};

/// Chord length below which a segment is rejected as degenerate.
const MIN_CHORD: f64 = 1e-9;

fn invalid(argument: &'static str, reason: String) -> CoreError {
    CoreError::InvalidArgument { argument, reason }
}

/// One boundary element of a profile, precomputed for fast queries.
#[derive(Clone, Debug)]
enum Segment {
    Line {
        a: [f64; 2],
        b: [f64; 2],
    },
    /// Circular arc from `a` to `b`. `bulge` is the DXF convention:
    /// `tan(sweep / 4)`, positive for a counter-clockwise sweep.
    Arc {
        a: [f64; 2],
        b: [f64; 2],
        bulge: f64,
        center: [f64; 2],
        radius: f64,
        start_angle: f64,
        /// Signed sweep in radians, `4·atan(bulge)`, in `(-2π, 2π)`.
        sweep: f64,
    },
    /// Elliptical arc from `a` to `b`. The supporting ellipse is centred at
    /// `center` with semi-axes `rx`/`ry` rotated by angle `phi`
    /// (`cphi`/`sphi` cached); the arc runs from eccentric angle
    /// `start_angle` through the signed `sweep` (radians of eccentric angle,
    /// `|sweep| ≤ 2π`), so `a = E(start_angle)` and `b = E(start_angle +
    /// sweep)` where `E(θ) = center + Rot(phi)·(rx·cosθ, ry·sinθ)`.
    Ellipse {
        a: [f64; 2],
        b: [f64; 2],
        center: [f64; 2],
        rx: f64,
        ry: f64,
        cphi: f64,
        sphi: f64,
        start_angle: f64,
        sweep: f64,
    },
    /// Cubic Bézier from `a` to `b` with control points `c1`, `c2`:
    /// `B(t) = (1−t)³·a + 3(1−t)²t·c1 + 3(1−t)t²·c2 + t³·b`, `t ∈ [0, 1]`.
    Cubic {
        a: [f64; 2],
        b: [f64; 2],
        c1: [f64; 2],
        c2: [f64; 2],
    },
}

fn dist2d(p: [f64; 2], q: [f64; 2]) -> f64 {
    (p[0] - q[0]).hypot(p[1] - q[1])
}

/// Axis-aligned bounds `(min, max)` of a segment chain: every endpoint plus
/// each segment's own interior extremes (arc cardinal points, ellipse axis
/// extremes, cubic `dP/dt = 0` points).
fn segments_bounds(segments: &[Segment]) -> ([f64; 2], [f64; 2]) {
    let mut min = [f64::INFINITY; 2];
    let mut max = [f64::NEG_INFINITY; 2];
    let mut include = |p: [f64; 2]| {
        min[0] = min[0].min(p[0]);
        min[1] = min[1].min(p[1]);
        max[0] = max[0].max(p[0]);
        max[1] = max[1].max(p[1]);
    };
    for seg in segments {
        include(seg.start());
        include(seg.end());
        match *seg {
            // Arcs can extend past their endpoints: include each cardinal
            // extreme of the circle that lies on the swept range.
            Segment::Arc {
                center,
                radius,
                start_angle,
                sweep,
                ..
            } => {
                for k in 0..4 {
                    let ang = k as f64 * FRAC_PI_2;
                    let delta = if sweep >= 0.0 {
                        (ang - start_angle).rem_euclid(TAU)
                    } else {
                        (start_angle - ang).rem_euclid(TAU)
                    };
                    if delta <= sweep.abs() {
                        include([
                            center[0] + radius * ang.cos(),
                            center[1] + radius * ang.sin(),
                        ]);
                    }
                }
            }
            Segment::Ellipse { .. } => {
                for p in seg.ellipse_axis_extremes() {
                    include(p);
                }
            }
            Segment::Cubic { .. } => {
                for p in seg.cubic_axis_extremes() {
                    include(p);
                }
            }
            Segment::Line { .. } => {}
        }
    }
    (min, max)
}

/// Real roots of `a·t² + b·t + c` (any that exist), robust to a vanishing
/// leading coefficient (degenerates to the linear root).
fn solve_quadratic(a: f64, b: f64, c: f64) -> Vec<f64> {
    if a.abs() < 1e-14 {
        if b.abs() < 1e-14 {
            return Vec::new();
        }
        return vec![-c / b];
    }
    let disc = b * b - 4.0 * a * c;
    if disc < 0.0 {
        return Vec::new();
    }
    let sq = disc.sqrt();
    vec![(-b + sq) / (2.0 * a), (-b - sq) / (2.0 * a)]
}

/// Point on the ellipse `center + Rot(phi)·(rx·cosθ, ry·sinθ)` at eccentric
/// angle `theta`.
fn ellipse_point(c: [f64; 2], rx: f64, ry: f64, cphi: f64, sphi: f64, theta: f64) -> [f64; 2] {
    let (ct, st) = (theta.cos(), theta.sin());
    [
        c[0] + rx * ct * cphi - ry * st * sphi,
        c[1] + rx * ct * sphi + ry * st * cphi,
    ]
}

/// `dE/dθ` of [`ellipse_point`] (independent of `center`).
fn ellipse_deriv(rx: f64, ry: f64, cphi: f64, sphi: f64, theta: f64) -> [f64; 2] {
    let (ct, st) = (theta.cos(), theta.sin());
    [
        -rx * st * cphi - ry * ct * sphi,
        -rx * st * sphi + ry * ct * cphi,
    ]
}

/// `d²E/dθ²` of [`ellipse_point`] (independent of `center`): the position
/// vector relative to `center` negated.
fn ellipse_deriv2(rx: f64, ry: f64, cphi: f64, sphi: f64, theta: f64) -> [f64; 2] {
    let (ct, st) = (theta.cos(), theta.sin());
    [
        -rx * ct * cphi + ry * st * sphi,
        -rx * ct * sphi - ry * st * cphi,
    ]
}

/// Point on the cubic Bézier `a, c1, c2, b` at parameter `t`.
fn cubic_point(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    let u = 1.0 - t;
    let (w0, w1, w2, w3) = (u * u * u, 3.0 * u * u * t, 3.0 * u * t * t, t * t * t);
    [
        w0 * a[0] + w1 * c1[0] + w2 * c2[0] + w3 * b[0],
        w0 * a[1] + w1 * c1[1] + w2 * c2[1] + w3 * b[1],
    ]
}

/// `dB/dt` of [`cubic_point`].
fn cubic_deriv(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    let u = 1.0 - t;
    let (w0, w1, w2) = (3.0 * u * u, 6.0 * u * t, 3.0 * t * t);
    [
        w0 * (c1[0] - a[0]) + w1 * (c2[0] - c1[0]) + w2 * (b[0] - c2[0]),
        w0 * (c1[1] - a[1]) + w1 * (c2[1] - c1[1]) + w2 * (b[1] - c2[1]),
    ]
}

/// `d²B/dt²` of [`cubic_point`].
fn cubic_deriv2(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    let u = 1.0 - t;
    [
        6.0 * (u * (c2[0] - 2.0 * c1[0] + a[0]) + t * (b[0] - 2.0 * c2[0] + c1[0])),
        6.0 * (u * (c2[1] - 2.0 * c1[1] + a[1]) + t * (b[1] - 2.0 * c2[1] + c1[1])),
    ]
}

/// Closest point on a smooth curve to `q`, returned as `(distance, s)` where
/// `s ∈ [s0, s1]` is the parameter of the nearest point. Robust global search:
/// sample the parameter range, then Newton-polish the best sample on the
/// squared-distance stationarity `(P(s) − q)·P'(s) = 0`. `pt`/`d1`/`d2` give
/// the point and its first/second parameter derivatives.
fn closest_on_curve(
    q: [f64; 2],
    s0: f64,
    s1: f64,
    pt: impl Fn(f64) -> [f64; 2],
    d1: impl Fn(f64) -> [f64; 2],
    d2: impl Fn(f64) -> [f64; 2],
) -> (f64, f64) {
    const SAMPLES: usize = 32;
    let dist_at = |s: f64| {
        let p = pt(s);
        (q[0] - p[0]).hypot(q[1] - p[1])
    };
    // Coarse global scan for the basin of the nearest point.
    let mut best_s = s0;
    let mut best_d = dist_at(s0);
    for i in 1..=SAMPLES {
        let s = s0 + (s1 - s0) * (i as f64 / SAMPLES as f64);
        let d = dist_at(s);
        if d < best_d {
            best_d = d;
            best_s = s;
        }
    }
    // Newton polish on the stationarity of the squared distance.
    let mut t = best_s;
    for _ in 0..12 {
        let p = pt(t);
        let pp = d1(t);
        let ppp = d2(t);
        let diff = [p[0] - q[0], p[1] - q[1]];
        let g = diff[0] * pp[0] + diff[1] * pp[1];
        let h = pp[0] * pp[0] + pp[1] * pp[1] + diff[0] * ppp[0] + diff[1] * ppp[1];
        if h.abs() < 1e-15 {
            break;
        }
        let step = g / h;
        t = (t - step).clamp(s0, s1);
        if step.abs() < 1e-14 {
            break;
        }
    }
    let dt = dist_at(t);
    if dt < best_d {
        (dt, t)
    } else {
        (best_d, best_s)
    }
}

/// Count crossings of the `+x` horizontal ray from `q` with a curve over
/// `s ∈ [s0, s1]` (`s0 < s1`), using the even-odd polygon half-open rule on
/// each `y`-monotone piece (split at the interior `y`-extrema in `crit`).
/// `yf`/`xf` evaluate the curve's `y`/`x` at a parameter.
///
/// The outer endpoint `y`-values are taken from `y_lo`/`y_hi` (the *canonical*
/// shared-vertex heights) rather than `yf(s0)`/`yf(s1)`: consecutive segments
/// share a vertex exactly, so using its one canonical `y` for the half-open
/// test makes the parity robust to the sub-ULP noise a curve's own endpoint
/// evaluation carries. Interior split points use the true `yf`.
fn ray_crossings(
    q: [f64; 2],
    span: [f64; 2],
    crit: &[f64],
    yf: impl Fn(f64) -> f64,
    xf: impl Fn(f64) -> f64,
    y_ends: [f64; 2],
) -> u32 {
    let [s0, s1] = span;
    let mut breaks = vec![(s0, y_ends[0])];
    for &c in crit {
        if c > s0 && c < s1 {
            breaks.push((c, yf(c)));
        }
    }
    breaks.push((s1, y_ends[1]));
    breaks.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap());
    let (qx, qy) = (q[0], q[1]);
    let mut count = 0;
    for w in breaks.windows(2) {
        let ((a, ya), (b, yb)) = (w[0], w[1]);
        // Half-open rule: the lower endpoint is inclusive, the upper exclusive.
        if (ya > qy) == (yb > qy) {
            continue;
        }
        // Monotone piece: bisect on the true curve for the crossing parameter.
        let (mut lo, mut hi) = (a, b);
        let hi_above = yb > qy;
        for _ in 0..60 {
            let mid = 0.5 * (lo + hi);
            if (yf(mid) > qy) == hi_above {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        if xf(0.5 * (lo + hi)) > qx {
            count += 1;
        }
    }
    count
}

impl Segment {
    fn new(a: [f64; 2], b: [f64; 2], bulge: f64) -> Self {
        if bulge == 0.0 {
            return Segment::Line { a, b };
        }
        let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
        let chord = dx.hypot(dy);
        // Sagitta s = bulge·chord/2; radius and center follow from the
        // half-angle identities with sweep = 4·atan(bulge).
        let radius = (chord * (1.0 + bulge * bulge) / (4.0 * bulge)).abs();
        // Signed distance from chord midpoint to center along the left
        // normal of the chord direction.
        let h = chord * (1.0 - bulge * bulge) / (4.0 * bulge);
        let center = [
            0.5 * (a[0] + b[0]) - h * dy / chord,
            0.5 * (a[1] + b[1]) + h * dx / chord,
        ];
        Segment::Arc {
            a,
            b,
            bulge,
            center,
            radius,
            start_angle: (a[1] - center[1]).atan2(a[0] - center[0]),
            sweep: 4.0 * bulge.atan(),
        }
    }

    /// Elliptical-arc segment: from start point `a` (which must lie on the
    /// ellipse), sweep `sweep` radians of eccentric angle about `center` on
    /// the ellipse with semi-axes `rx`/`ry` rotated by `phi`. The start
    /// eccentric angle is recovered from `a`; the endpoint follows.
    fn ellipse(a: [f64; 2], center: [f64; 2], rx: f64, ry: f64, phi: f64, sweep: f64) -> Self {
        let (cphi, sphi) = (phi.cos(), phi.sin());
        // Invert a into the ellipse frame to get its eccentric angle.
        let (dx, dy) = (a[0] - center[0], a[1] - center[1]);
        let lx = cphi * dx + sphi * dy;
        let ly = -sphi * dx + cphi * dy;
        let start_angle = (ly / ry).atan2(lx / rx);
        let b = ellipse_point(center, rx, ry, cphi, sphi, start_angle + sweep);
        Segment::Ellipse {
            a,
            b,
            center,
            rx,
            ry,
            cphi,
            sphi,
            start_angle,
            sweep,
        }
    }

    /// Cubic-Bézier segment from `a` to `b` with control points `c1`, `c2`.
    fn cubic(a: [f64; 2], b: [f64; 2], c1: [f64; 2], c2: [f64; 2]) -> Self {
        Segment::Cubic { a, b, c1, c2 }
    }

    /// Start point of the segment.
    fn start(&self) -> [f64; 2] {
        match *self {
            Segment::Line { a, .. }
            | Segment::Arc { a, .. }
            | Segment::Ellipse { a, .. }
            | Segment::Cubic { a, .. } => a,
        }
    }

    /// End point of the segment.
    fn end(&self) -> [f64; 2] {
        match *self {
            Segment::Line { b, .. }
            | Segment::Arc { b, .. }
            | Segment::Ellipse { b, .. }
            | Segment::Cubic { b, .. } => b,
        }
    }

    /// Eccentric angles (within the swept range) at which the ellipse's `x`
    /// or `y` coordinate is extremal — the candidate bounding-box extremes
    /// beyond the endpoints.
    fn ellipse_axis_extremes(&self) -> Vec<[f64; 2]> {
        let Segment::Ellipse {
            center,
            rx,
            ry,
            cphi,
            sphi,
            start_angle,
            sweep,
            ..
        } = *self
        else {
            return Vec::new();
        };
        let (lo, hi) = (
            start_angle.min(start_angle + sweep),
            start_angle.max(start_angle + sweep),
        );
        let mut out = Vec::new();
        // x-extreme: dEx/dθ = 0 → -rx·sinθ·cphi - ry·cosθ·sphi = 0.
        // y-extreme: dEy/dθ = 0 → -rx·sinθ·sphi + ry·cosθ·cphi = 0.
        for &(base, _) in &[
            ((-ry * sphi).atan2(rx * cphi), 0u8),
            ((ry * cphi).atan2(rx * sphi), 1u8),
        ] {
            for k in -1..=2 {
                let theta = base + k as f64 * PI;
                if theta > lo && theta < hi {
                    out.push(ellipse_point(center, rx, ry, cphi, sphi, theta));
                }
            }
        }
        out
    }

    /// Points on the cubic where `dx/dt = 0` or `dy/dt = 0` (`t ∈ (0, 1)`) —
    /// the candidate bounding-box extremes beyond the endpoints.
    fn cubic_axis_extremes(&self) -> Vec<[f64; 2]> {
        let Segment::Cubic { a, b, c1, c2 } = *self else {
            return Vec::new();
        };
        let mut out = Vec::new();
        for axis in 0..2 {
            // dP/dt is quadratic: 3[(P1−P0)(1−t)² + 2(P2−P1)(1−t)t + (P3−P2)t²].
            let p0 = a[axis];
            let p1 = c1[axis];
            let p2 = c2[axis];
            let p3 = b[axis];
            // Coefficients of the quadratic A t² + B t + C for dP/dt / 3.
            let qa = p3 - 3.0 * p2 + 3.0 * p1 - p0;
            let qb = 2.0 * (p2 - 2.0 * p1 + p0);
            let qc = p1 - p0;
            for t in solve_quadratic(qa, qb, qc) {
                if t > 0.0 && t < 1.0 {
                    out.push(cubic_point(a, c1, c2, b, t));
                }
            }
        }
        out
    }

    /// Interior `y`-monotone break parameters for the ray-cast lune test: the
    /// `y`-extrema strictly inside the segment's parameter range.
    fn y_crit(&self) -> Vec<f64> {
        match *self {
            Segment::Ellipse {
                rx,
                ry,
                cphi,
                sphi,
                start_angle,
                sweep,
                ..
            } => {
                let (lo, hi) = (
                    start_angle.min(start_angle + sweep),
                    start_angle.max(start_angle + sweep),
                );
                // dEy/dθ = 0 → tanθ = (ry·cphi)/(rx·sphi).
                let base = (ry * cphi).atan2(rx * sphi);
                let mut out = Vec::new();
                for k in -1..=2 {
                    let theta = base + k as f64 * PI;
                    if theta > lo && theta < hi {
                        out.push(theta);
                    }
                }
                out
            }
            Segment::Cubic { a, b, c1, c2 } => {
                let (p0, p1, p2, p3) = (a[1], c1[1], c2[1], b[1]);
                let qa = p3 - 3.0 * p2 + 3.0 * p1 - p0;
                let qb = 2.0 * (p2 - 2.0 * p1 + p0);
                let qc = p1 - p0;
                solve_quadratic(qa, qb, qc)
                    .into_iter()
                    .filter(|&t| t > 0.0 && t < 1.0)
                    .collect()
            }
            _ => Vec::new(),
        }
    }

    /// Number of `+x` ray crossings for this segment's true boundary from
    /// canonical vertex `va` to `vb` (the shared endpoints of the closed
    /// loop). Half-open at the endpoints using `va`/`vb` heights so adjacent
    /// segments agree exactly at a shared vertex; interior crossings solved on
    /// the true curve. This is the per-segment term of the whole-boundary
    /// even-odd point-in-region test.
    fn ray_crossings_canon(&self, q: [f64; 2], va: [f64; 2], vb: [f64; 2]) -> u32 {
        let (qx, qy) = (q[0], q[1]);
        match *self {
            Segment::Line { .. } => {
                // Straight chord va→vb.
                if (va[1] > qy) == (vb[1] > qy) {
                    return 0;
                }
                let t = (qy - va[1]) / (vb[1] - va[1]);
                u32::from(va[0] + t * (vb[0] - va[0]) > qx)
            }
            Segment::Arc {
                center,
                radius,
                start_angle,
                sweep,
                ..
            } => {
                let (lo, hi) = (
                    start_angle.min(start_angle + sweep),
                    start_angle.max(start_angle + sweep),
                );
                // Circle y-extrema (top/bottom) inside the swept range.
                let mut crit = Vec::new();
                for k in -1..=2 {
                    let theta = FRAC_PI_2 + k as f64 * PI;
                    if theta > lo && theta < hi {
                        crit.push(theta);
                    }
                }
                let y_ends = if sweep >= 0.0 {
                    [va[1], vb[1]]
                } else {
                    [vb[1], va[1]]
                };
                ray_crossings(
                    q,
                    [lo, hi],
                    &crit,
                    |t| center[1] + radius * t.sin(),
                    |t| center[0] + radius * t.cos(),
                    y_ends,
                )
            }
            Segment::Ellipse {
                center,
                rx,
                ry,
                cphi,
                sphi,
                start_angle,
                sweep,
                ..
            } => {
                let (lo, hi) = (
                    start_angle.min(start_angle + sweep),
                    start_angle.max(start_angle + sweep),
                );
                let y_ends = if sweep >= 0.0 {
                    [va[1], vb[1]]
                } else {
                    [vb[1], va[1]]
                };
                ray_crossings(
                    q,
                    [lo, hi],
                    &self.y_crit(),
                    |t| ellipse_point(center, rx, ry, cphi, sphi, t)[1],
                    |t| ellipse_point(center, rx, ry, cphi, sphi, t)[0],
                    y_ends,
                )
            }
            Segment::Cubic { a, b, c1, c2 } => ray_crossings(
                q,
                [0.0, 1.0],
                &self.y_crit(),
                |t| cubic_point(a, c1, c2, b, t)[1],
                |t| cubic_point(a, c1, c2, b, t)[0],
                [va[1], vb[1]],
            ),
        }
    }

    /// Unsigned distance from `q` to the segment.
    fn distance(&self, q: [f64; 2]) -> f64 {
        match *self {
            Segment::Line { a, b } => {
                let (ex, ey) = (b[0] - a[0], b[1] - a[1]);
                let (wx, wy) = (q[0] - a[0], q[1] - a[1]);
                let t = ((wx * ex + wy * ey) / (ex * ex + ey * ey)).clamp(0.0, 1.0);
                (wx - t * ex).hypot(wy - t * ey)
            }
            Segment::Arc {
                a,
                b,
                center,
                radius,
                start_angle,
                sweep,
                ..
            } => {
                let (vx, vy) = (q[0] - center[0], q[1] - center[1]);
                let ang = vy.atan2(vx);
                // Angular offset from the start, measured along the sweep
                // direction, in [0, 2π).
                let delta = if sweep >= 0.0 {
                    (ang - start_angle).rem_euclid(TAU)
                } else {
                    (start_angle - ang).rem_euclid(TAU)
                };
                if delta <= sweep.abs() {
                    (vx.hypot(vy) - radius).abs()
                } else {
                    dist2d(q, a).min(dist2d(q, b))
                }
            }
            Segment::Ellipse { .. } | Segment::Cubic { .. } => self.closest(q).0,
        }
    }

    /// Closest point on a curved (ellipse/cubic) segment to `q`, returned as
    /// `(distance, closest_point, travel)` where `travel` is the
    /// (unnormalised) tangent in the direction of travel at that point.
    fn closest(&self, q: [f64; 2]) -> (f64, [f64; 2], [f64; 2]) {
        match *self {
            Segment::Ellipse {
                center,
                rx,
                ry,
                cphi,
                sphi,
                start_angle,
                sweep,
                ..
            } => {
                let (d, s) = closest_on_curve(
                    q,
                    0.0,
                    1.0,
                    |s| ellipse_point(center, rx, ry, cphi, sphi, start_angle + s * sweep),
                    |s| {
                        let dd = ellipse_deriv(rx, ry, cphi, sphi, start_angle + s * sweep);
                        [dd[0] * sweep, dd[1] * sweep]
                    },
                    |s| {
                        let dd = ellipse_deriv2(rx, ry, cphi, sphi, start_angle + s * sweep);
                        [dd[0] * sweep * sweep, dd[1] * sweep * sweep]
                    },
                );
                let theta = start_angle + s * sweep;
                let dd = ellipse_deriv(rx, ry, cphi, sphi, theta);
                (
                    d,
                    ellipse_point(center, rx, ry, cphi, sphi, theta),
                    [dd[0] * sweep, dd[1] * sweep],
                )
            }
            Segment::Cubic { a, b, c1, c2 } => {
                let (d, t) = closest_on_curve(
                    q,
                    0.0,
                    1.0,
                    |t| cubic_point(a, c1, c2, b, t),
                    |t| cubic_deriv(a, c1, c2, b, t),
                    |t| cubic_deriv2(a, c1, c2, b, t),
                );
                (
                    d,
                    cubic_point(a, c1, c2, b, t),
                    cubic_deriv(a, c1, c2, b, t),
                )
            }
            // Straight/circular segments have closed forms elsewhere.
            _ => (self.distance(q), self.start(), [0.0, 0.0]),
        }
    }

    /// Unsigned distance from `q` to the segment paired with the signed
    /// *lateral* offset `s`: positive on the left of the segment's travel
    /// direction, negative on the right, `|s|` equal to the perpendicular
    /// distance where `q` projects onto the segment's interior. Open-path
    /// ribs use the sign to pick which side of the sketch line receives
    /// material; the distance magnitude drives the field.
    fn dist_and_side(&self, q: [f64; 2]) -> (f64, f64) {
        match *self {
            Segment::Line { a, b } => {
                let (ex, ey) = (b[0] - a[0], b[1] - a[1]);
                let (wx, wy) = (q[0] - a[0], q[1] - a[1]);
                let len = ex.hypot(ey);
                let t = ((wx * ex + wy * ey) / (ex * ex + ey * ey)).clamp(0.0, 1.0);
                let dist = (wx - t * ex).hypot(wy - t * ey);
                // cross(edge, w) / |edge|: signed perpendicular offset, left
                // of the a→b direction positive.
                let side = (ex * wy - ey * wx) / len;
                (dist, side)
            }
            Segment::Arc {
                a,
                b,
                center,
                radius,
                start_angle,
                sweep,
                ..
            } => {
                let (vx, vy) = (q[0] - center[0], q[1] - center[1]);
                let rho = vx.hypot(vy);
                let ang = vy.atan2(vx);
                let delta = if sweep >= 0.0 {
                    (ang - start_angle).rem_euclid(TAU)
                } else {
                    (start_angle - ang).rem_euclid(TAU)
                };
                // Travelling along the arc, the center is on the left for a
                // CCW sweep and on the right for a CW sweep, so points inside
                // the circle (rho < radius) are on the left iff sweep > 0.
                let side = sweep.signum() * (radius - rho);
                let dist = if delta <= sweep.abs() {
                    (rho - radius).abs()
                } else {
                    dist2d(q, a).min(dist2d(q, b))
                };
                (dist, side)
            }
            Segment::Ellipse { .. } | Segment::Cubic { .. } => {
                let (dist, cp, travel) = self.closest(q);
                let n = [q[0] - cp[0], q[1] - cp[1]];
                let len = travel[0].hypot(travel[1]);
                // cross(travel, q − closest) / |travel|: signed perpendicular
                // offset, left of travel positive (matching the line case).
                let side = if len > 0.0 {
                    (travel[0] * n[1] - travel[1] * n[0]) / len
                } else {
                    0.0
                };
                (dist, side)
            }
        }
    }
}

/// A closed planar profile bounded by straight lines and circular arcs.
///
/// Vertices are listed in loop order; segment `i` runs from vertex `i` to
/// vertex `i + 1` (wrapping) and carries a *bulge*: `0` for a straight
/// line, otherwise `tan(sweep / 4)` with positive values sweeping
/// counter-clockwise (the DXF polyline convention; `1` is a
/// counter-clockwise semicircle).
///
/// The boundary must be simple (non-self-intersecting, arcs not crossing
/// other segments); [`Profile2D::signed_distance`] is then the exact 2D
/// signed distance to the enclosed region, negative inside, regardless of
/// loop orientation.
#[derive(Clone, Debug)]
pub struct Profile2D {
    verts: Vec<[f64; 2]>,
    segments: Vec<Segment>,
    min: [f64; 2],
    max: [f64; 2],
}

impl Profile2D {
    /// Build a profile from loop vertices and per-segment bulges
    /// (`bulges[i]` belongs to the segment leaving `vertices[i]`).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if fewer than two vertices are given,
    /// the vertex and bulge counts differ, any coordinate or bulge is not
    /// finite, consecutive vertices coincide, or a two-vertex profile has
    /// no arc (it would enclose no area).
    pub fn new(vertices: Vec<[f64; 2]>, bulges: Vec<f64>) -> CoreResult<Self> {
        if vertices.len() < 2 {
            return Err(invalid(
                "vertices",
                format!("profile needs at least 2 vertices, got {}", vertices.len()),
            ));
        }
        if bulges.len() != vertices.len() {
            return Err(invalid(
                "bulges",
                format!(
                    "expected one bulge per segment ({}), got {}",
                    vertices.len(),
                    bulges.len()
                ),
            ));
        }
        if vertices.iter().flatten().any(|c| !c.is_finite()) {
            return Err(invalid("vertices", "coordinates must be finite".into()));
        }
        if bulges.iter().any(|b| !b.is_finite()) {
            return Err(invalid("bulges", "bulges must be finite".into()));
        }
        if vertices.len() == 2 && bulges.iter().all(|&b| b == 0.0) {
            return Err(invalid(
                "vertices",
                "a 2-vertex profile needs at least one arc to enclose area".into(),
            ));
        }
        let n = vertices.len();
        let mut segments = Vec::with_capacity(n);
        for i in 0..n {
            let a = vertices[i];
            let b = vertices[(i + 1) % n];
            if dist2d(a, b) < MIN_CHORD {
                return Err(invalid(
                    "vertices",
                    format!("segment {i} is degenerate: consecutive vertices coincide"),
                ));
            }
            segments.push(Segment::new(a, b, bulges[i]));
        }

        let (min, max) = segments_bounds(&segments);
        Ok(Self {
            verts: vertices,
            segments,
            min,
            max,
        })
    }

    /// Assemble a validated profile from prebuilt segments forming a closed
    /// loop (each segment's end is the next segment's start; the last closes
    /// back to the first). The [`Profile2DBuilder`] is the public entry.
    fn from_segments(segments: Vec<Segment>) -> CoreResult<Self> {
        if segments.len() < 2 {
            return Err(invalid(
                "segments",
                format!("profile needs at least 2 segments, got {}", segments.len()),
            ));
        }
        let verts: Vec<[f64; 2]> = segments.iter().map(|s| s.start()).collect();
        let (min, max) = segments_bounds(&segments);
        Ok(Self {
            verts,
            segments,
            min,
            max,
        })
    }

    /// Axis-aligned bounds of the boundary as `(min, max)` corners,
    /// including arc extremes.
    pub fn bounds(&self) -> ([f64; 2], [f64; 2]) {
        (self.min, self.max)
    }

    /// True if `q` is inside the enclosed region.
    ///
    /// Parity of the chord polygon (even-odd rule), then toggled once per
    /// circular segment (the region between an arc and its chord) that
    /// contains `q`: an outward-bulging arc adds its circular segment to
    /// the chord polygon, an inward one removes it, and XOR covers both.
    ///
    /// Arc chords are *interior* lines of the region, so a query exactly on
    /// one must not be misclassified: the side test breaks the σ = 0 tie by
    /// emulating the same virtual `+x` (then `+y`) nudge the even-odd ray
    /// test applies implicitly, keeping the two parity sources consistent.
    fn contains(&self, q: [f64; 2]) -> bool {
        // A profile carrying an ellipse or cubic segment uses the uniform
        // whole-boundary ray cast (robust to sub-ULP vertex noise on the
        // curved endpoints); pure line/arc loops keep the original
        // chord-polygon + arc-lune test unchanged.
        if self
            .segments
            .iter()
            .any(|s| matches!(s, Segment::Ellipse { .. } | Segment::Cubic { .. }))
        {
            let n = self.segments.len();
            let mut crossings = 0;
            for i in 0..n {
                let va = self.verts[i];
                let vb = self.verts[(i + 1) % n];
                crossings += self.segments[i].ray_crossings_canon(q, va, vb);
            }
            return crossings % 2 == 1;
        }
        let n = self.verts.len();
        let mut inside = false;
        for i in 0..n {
            let a = self.verts[i];
            let b = self.verts[(i + 1) % n];
            if (a[1] > q[1]) != (b[1] > q[1]) {
                let t = (q[1] - a[1]) / (b[1] - a[1]);
                if a[0] + t * (b[0] - a[0]) > q[0] {
                    inside = !inside;
                }
            }
        }
        for seg in &self.segments {
            if let Segment::Arc {
                a,
                b,
                bulge,
                center,
                radius,
                ..
            } = *seg
            {
                // Inside the circle and on the arc's side of the chord.
                // The arc apex sits at cross(chord, apex - a)·bulge < 0,
                // for both minor and major arcs.
                if dist2d(q, center) < radius {
                    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                    let cross = dx * (q[1] - a[1]) - dy * (q[0] - a[0]);
                    // σ = 0: on the chord line. Nudging q by (+ε, +ε')
                    // shifts σ by dx·ε' - dy·ε; the x term dominates.
                    let side = if cross != 0.0 {
                        cross
                    } else if dy != 0.0 {
                        -dy
                    } else {
                        dx
                    };
                    if side * bulge < 0.0 {
                        inside = !inside;
                    }
                }
            }
        }
        inside
    }

    /// Exact signed distance to the enclosed region: negative inside,
    /// positive outside, zero on the boundary.
    pub fn signed_distance(&self, u: f64, v: f64) -> f64 {
        self.signed_distance_masked(u, v, &[])
    }

    /// [`Self::signed_distance`], but segments flagged in `skip` do not
    /// contribute to the distance magnitude (they still shape the parity,
    /// which only depends on the region). [`Revolve`] uses this to ignore
    /// boundary segments lying on the revolution axis — they vanish from
    /// the surface when swept.
    fn signed_distance_masked(&self, u: f64, v: f64, skip: &[bool]) -> f64 {
        let q = [u, v];
        let d = self
            .segments
            .iter()
            .enumerate()
            .filter(|(i, _)| skip.get(*i) != Some(&true))
            .map(|(_, s)| s.distance(q))
            .fold(f64::INFINITY, f64::min);
        if self.contains(q) { -d } else { d }
    }

    /// Flags for each segment: a straight segment lying on the vertical
    /// line `u = 0` (the revolution axis).
    fn axis_line_segments(&self) -> Vec<bool> {
        self.segments
            .iter()
            .map(|s| match *s {
                Segment::Line { a, b } => a[0].abs() <= MIN_CHORD && b[0].abs() <= MIN_CHORD,
                Segment::Arc { .. } | Segment::Ellipse { .. } | Segment::Cubic { .. } => false,
            })
            .collect()
    }

    /// Start a mixed-segment profile builder anchored at `start`.
    pub fn builder(start: [f64; 2]) -> Profile2DBuilder {
        Profile2DBuilder::new(start)
    }
}

fn finite2(p: [f64; 2]) -> bool {
    p[0].is_finite() && p[1].is_finite()
}

/// Validate and build a straight segment `a → b`.
fn seg_line(a: [f64; 2], b: [f64; 2]) -> CoreResult<Segment> {
    if !(finite2(a) && finite2(b)) {
        return Err(invalid("point", "coordinates must be finite".into()));
    }
    if dist2d(a, b) < MIN_CHORD {
        return Err(invalid(
            "point",
            "segment is degenerate: endpoints coincide".into(),
        ));
    }
    Ok(Segment::new(a, b, 0.0))
}

/// Validate and build a circular-arc segment `a → b` with the DXF `bulge`.
fn seg_arc(a: [f64; 2], b: [f64; 2], bulge: f64) -> CoreResult<Segment> {
    if !(finite2(a) && finite2(b) && bulge.is_finite()) {
        return Err(invalid(
            "arc",
            "coordinates and bulge must be finite".into(),
        ));
    }
    if dist2d(a, b) < MIN_CHORD {
        return Err(invalid(
            "arc",
            "segment is degenerate: endpoints coincide".into(),
        ));
    }
    Ok(Segment::new(a, b, bulge))
}

/// Validate and build an elliptical-arc segment starting at `a`.
fn seg_ellipse(
    a: [f64; 2],
    center: [f64; 2],
    rx: f64,
    ry: f64,
    rotation: f64,
    sweep: f64,
) -> CoreResult<Segment> {
    if !(finite2(a) && finite2(center) && rotation.is_finite() && sweep.is_finite()) {
        return Err(invalid("ellipse", "parameters must be finite".into()));
    }
    if !(rx.is_finite() && rx > MIN_CHORD && ry.is_finite() && ry > MIN_CHORD) {
        return Err(invalid(
            "ellipse",
            format!("radii must be positive, got ({rx}, {ry})"),
        ));
    }
    if sweep.abs() < 1e-9 || sweep.abs() > TAU + 1e-9 {
        return Err(invalid(
            "ellipse",
            format!("|sweep| must be in (0, 2π], got {sweep}"),
        ));
    }
    Ok(Segment::ellipse(a, center, rx, ry, rotation, sweep))
}

/// Validate and build a cubic-Bézier segment `a → b` with controls `c1`,`c2`.
fn seg_cubic(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2]) -> CoreResult<Segment> {
    if !(finite2(a) && finite2(b) && finite2(c1) && finite2(c2)) {
        return Err(invalid("cubic", "control points must be finite".into()));
    }
    if dist2d(a, b) < MIN_CHORD && dist2d(a, c1) < MIN_CHORD && dist2d(a, c2) < MIN_CHORD {
        return Err(invalid(
            "cubic",
            "segment is degenerate: all points coincide".into(),
        ));
    }
    Ok(Segment::cubic(a, b, c1, c2))
}

/// Builder for a closed [`Profile2D`] mixing straight, circular-arc,
/// elliptical-arc and cubic-Bézier segments.
///
/// Anchor at a start point, chain `line_to` / `arc_to` / `ellipse_to` /
/// `cubic_to` (each continues from the previous endpoint), then [`build`]
/// closes the loop with a straight segment back to the start (a no-op if the
/// path already ends there). The first invalid input is reported by `build`.
///
/// [`build`]: Profile2DBuilder::build
#[derive(Clone, Debug)]
pub struct Profile2DBuilder {
    start: [f64; 2],
    cursor: [f64; 2],
    segments: Vec<Segment>,
    error: Option<CoreError>,
}

impl Profile2DBuilder {
    fn new(start: [f64; 2]) -> Self {
        Self {
            start,
            cursor: start,
            segments: Vec::new(),
            error: None,
        }
    }

    fn push(&mut self, seg: CoreResult<Segment>) {
        match seg {
            Ok(s) => {
                self.cursor = s.end();
                self.segments.push(s);
            }
            Err(e) => {
                if self.error.is_none() {
                    self.error = Some(e);
                }
            }
        }
    }

    /// Straight segment to `to`.
    pub fn line_to(mut self, to: [f64; 2]) -> Self {
        let seg = seg_line(self.cursor, to);
        self.push(seg);
        self
    }

    /// Circular arc to `to` with the DXF `bulge` (`tan(sweep / 4)`, positive
    /// counter-clockwise; `0` is a straight line).
    pub fn arc_to(mut self, to: [f64; 2], bulge: f64) -> Self {
        let seg = seg_arc(self.cursor, to, bulge);
        self.push(seg);
        self
    }

    /// Elliptical arc: from the current point, sweep `sweep` radians of
    /// eccentric angle on the ellipse centred at `center` with semi-axes
    /// `rx`/`ry` rotated by `rotation`. The current point must lie on that
    /// ellipse; the endpoint follows from the sweep.
    pub fn ellipse_to(
        mut self,
        center: [f64; 2],
        rx: f64,
        ry: f64,
        rotation: f64,
        sweep: f64,
    ) -> Self {
        let seg = seg_ellipse(self.cursor, center, rx, ry, rotation, sweep);
        self.push(seg);
        self
    }

    /// Cubic Bézier to `to` with control points `c1`, `c2`.
    pub fn cubic_to(mut self, c1: [f64; 2], c2: [f64; 2], to: [f64; 2]) -> Self {
        let seg = seg_cubic(self.cursor, c1, c2, to);
        self.push(seg);
        self
    }

    /// Close the loop and assemble the profile.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if any segment was invalid, or the loop
    /// has fewer than two segments after closing.
    pub fn build(mut self) -> CoreResult<Profile2D> {
        if let Some(e) = self.error {
            return Err(e);
        }
        // Close with a straight segment unless the path already ends on start.
        if dist2d(self.cursor, self.start) >= MIN_CHORD {
            self.segments
                .push(Segment::new(self.cursor, self.start, 0.0));
        }
        Profile2D::from_segments(self.segments)
    }
}

/// Draft angles beyond this magnitude (~80°) are rejected: `tan` explodes
/// toward 90° and the taper stops being a meaningful wall angle.
pub const MAX_DRAFT: f64 = 1.4;

/// A profile swept linearly along +Y: profile `(u, v)` maps to world
/// `(x, z) = (u, v)` and the solid spans `y ∈ [0, height]`.
///
/// An optional `draft` angle tapers the walls along the sweep: the
/// cross-section at height `y` is the base profile inset radially by
/// `tan(draft)·y`. A positive draft shrinks the section toward the top cap
/// (the standard mold-release taper); a negative draft flares it outward.
/// Zero draft is the exact prism; see the [module docs](self) on exactness.
pub struct Extrude {
    profile: Profile2D,
    height: f64,
    /// `tan(draft)`: radial inset applied per unit height. Zero = straight
    /// walls.
    taper: f64,
    /// `cos(draft)` — scales the tapered-wall term to the exact perpendicular
    /// distance to the tilted plane, which keeps the field Lipschitz ≤ 1.
    /// Always positive for `|draft| < π/2`.
    wall_scale: f64,
}

impl Extrude {
    /// A straight-walled extrusion (no draft).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `height` is not positive and finite.
    pub fn new(profile: Profile2D, height: f64) -> CoreResult<Self> {
        Self::with_draft(profile, height, 0.0)
    }

    /// An extrusion with a `draft` angle in radians (see [`Extrude`]).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `height` is not positive and finite,
    /// or `draft` is not finite with `|draft| < `[`MAX_DRAFT`].
    pub fn with_draft(profile: Profile2D, height: f64, draft: f64) -> CoreResult<Self> {
        if !(height.is_finite() && height > 0.0) {
            return Err(invalid(
                "height",
                format!("must be positive and finite, got {height}"),
            ));
        }
        if !(draft.is_finite() && draft.abs() < MAX_DRAFT) {
            return Err(invalid(
                "draft",
                format!("must be finite with |draft| < {MAX_DRAFT} rad (~80°), got {draft}"),
            ));
        }
        Ok(Self {
            profile,
            height,
            taper: draft.tan(),
            wall_scale: draft.cos(),
        })
    }
}

impl Sdf for Extrude {
    fn eval(&self, p: &Point3) -> f64 {
        // Tapered wall: the section at height y is the profile inset by
        // taper·y, so (profile_distance + taper·y) is the in-plane clearance
        // and multiplying by cos(draft) turns it into the exact perpendicular
        // distance to the tilted wall (an identity for a single plane; the
        // `min` over walls keeps it Lipschitz ≤ 1). taper = 0 recovers the
        // exact prism.
        let d = (self.profile.signed_distance(p.x, p.z) + self.taper * p.y) * self.wall_scale;
        let half = 0.5 * self.height;
        let w = (p.y - half).abs() - half;
        // Standard combination of a 2D SDF with a slab: interior takes the
        // larger (closer-to-boundary) term, exterior the Euclidean
        // combination of the positive parts.
        d.max(w).min(0.0) + d.max(0.0).hypot(w.max(0.0))
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // The wall term is unit-gradient, but where the exterior hypot pairs a
        // tilted wall (which gains a ±sin(draft) component along +Y) with the
        // horizontal cap, the gradients reinforce and the field is Lipschitz
        // L = sqrt(1 + |sin(draft)|). taper = 0 gives L = 1 (the exact prism),
        // recovering the default bound. Widen the half-diagonal by L so the
        // interval stays conservative for octree pruning.
        let sin_draft = (self.taper * self.wall_scale).abs(); // |tan·cos| = |sin|
        let d = self.eval(&b.center());
        let r = 0.5 * (1.0 + sin_draft).sqrt() * b.extents().norm();
        Interval::new(d - r, d + r)
    }
}

/// An *open* planar path — a polyline of straight lines and circular arcs
/// that does **not** close back on itself. Segment `i` runs from vertex `i`
/// to vertex `i + 1` and carries a bulge (the same DXF convention as
/// [`Profile2D`]: `0` is a line, otherwise `tan(sweep / 4)`, positive
/// counter-clockwise). A [`Rib`] thickens this path into a solid.
#[derive(Clone, Debug)]
pub struct OpenPath2D {
    segments: Vec<Segment>,
    min: [f64; 2],
    max: [f64; 2],
}

impl OpenPath2D {
    /// Build an open path from its vertices and per-segment bulges
    /// (`bulges[i]` belongs to the segment leaving `vertices[i]`, so there
    /// is one fewer bulge than vertex).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if fewer than two vertices are given,
    /// `bulges.len() != vertices.len() - 1`, any coordinate or bulge is not
    /// finite, or consecutive vertices coincide.
    pub fn new(vertices: Vec<[f64; 2]>, bulges: Vec<f64>) -> CoreResult<Self> {
        if vertices.len() < 2 {
            return Err(invalid(
                "vertices",
                format!(
                    "open path needs at least 2 vertices, got {}",
                    vertices.len()
                ),
            ));
        }
        if bulges.len() != vertices.len() - 1 {
            return Err(invalid(
                "bulges",
                format!(
                    "expected one bulge per segment ({}), got {}",
                    vertices.len() - 1,
                    bulges.len()
                ),
            ));
        }
        if vertices.iter().flatten().any(|c| !c.is_finite()) {
            return Err(invalid("vertices", "coordinates must be finite".into()));
        }
        if bulges.iter().any(|b| !b.is_finite()) {
            return Err(invalid("bulges", "bulges must be finite".into()));
        }
        let mut segments = Vec::with_capacity(vertices.len() - 1);
        for i in 0..vertices.len() - 1 {
            let a = vertices[i];
            let b = vertices[i + 1];
            if dist2d(a, b) < MIN_CHORD {
                return Err(invalid(
                    "vertices",
                    format!("segment {i} is degenerate: consecutive vertices coincide"),
                ));
            }
            segments.push(Segment::new(a, b, bulges[i]));
        }

        let (min, max) = segments_bounds(&segments);
        Ok(Self { segments, min, max })
    }

    /// Assemble a validated open path from prebuilt segments (each segment's
    /// end is the next segment's start). The [`OpenPath2DBuilder`] is the
    /// public entry.
    fn from_segments(segments: Vec<Segment>) -> CoreResult<Self> {
        if segments.is_empty() {
            return Err(invalid(
                "segments",
                "open path needs at least 1 segment".into(),
            ));
        }
        let (min, max) = segments_bounds(&segments);
        Ok(Self { segments, min, max })
    }

    /// Axis-aligned bounds of the path centreline (arc extremes included)
    /// as `(min, max)` corners — before any thickening.
    pub fn bounds(&self) -> ([f64; 2], [f64; 2]) {
        (self.min, self.max)
    }

    /// Unsigned distance from `q` to the path and the signed lateral offset
    /// of the *nearest* segment (left of travel positive). `|side|` equals
    /// the distance where `q` projects onto a segment interior and only
    /// carries a sign near vertices and endpoint caps.
    fn dist_and_side(&self, q: [f64; 2]) -> (f64, f64) {
        self.segments
            .iter()
            .map(|s| s.dist_and_side(q))
            .fold(
                (f64::INFINITY, 0.0),
                |acc, cur| {
                    if cur.0 < acc.0 { cur } else { acc }
                },
            )
    }

    /// Start a mixed-segment open-path builder anchored at `start`.
    pub fn builder(start: [f64; 2]) -> OpenPath2DBuilder {
        OpenPath2DBuilder::new(start)
    }
}

/// Builder for an open [`OpenPath2D`] mixing straight, circular-arc,
/// elliptical-arc and cubic-Bézier segments — the same segment vocabulary as
/// [`Profile2DBuilder`], but the path stays open (no closing segment).
#[derive(Clone, Debug)]
pub struct OpenPath2DBuilder {
    cursor: [f64; 2],
    segments: Vec<Segment>,
    error: Option<CoreError>,
}

impl OpenPath2DBuilder {
    fn new(start: [f64; 2]) -> Self {
        Self {
            cursor: start,
            segments: Vec::new(),
            error: None,
        }
    }

    fn push(&mut self, seg: CoreResult<Segment>) {
        match seg {
            Ok(s) => {
                self.cursor = s.end();
                self.segments.push(s);
            }
            Err(e) => {
                if self.error.is_none() {
                    self.error = Some(e);
                }
            }
        }
    }

    /// Straight segment to `to`.
    pub fn line_to(mut self, to: [f64; 2]) -> Self {
        let seg = seg_line(self.cursor, to);
        self.push(seg);
        self
    }

    /// Circular arc to `to` with the DXF `bulge`.
    pub fn arc_to(mut self, to: [f64; 2], bulge: f64) -> Self {
        let seg = seg_arc(self.cursor, to, bulge);
        self.push(seg);
        self
    }

    /// Elliptical arc (see [`Profile2DBuilder::ellipse_to`]).
    pub fn ellipse_to(
        mut self,
        center: [f64; 2],
        rx: f64,
        ry: f64,
        rotation: f64,
        sweep: f64,
    ) -> Self {
        let seg = seg_ellipse(self.cursor, center, rx, ry, rotation, sweep);
        self.push(seg);
        self
    }

    /// Cubic Bézier to `to` with control points `c1`, `c2`.
    pub fn cubic_to(mut self, c1: [f64; 2], c2: [f64; 2], to: [f64; 2]) -> Self {
        let seg = seg_cubic(self.cursor, c1, c2, to);
        self.push(seg);
        self
    }

    /// Assemble the open path.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if any segment was invalid or no
    /// segments were added.
    pub fn build(self) -> CoreResult<OpenPath2D> {
        if let Some(e) = self.error {
            return Err(e);
        }
        OpenPath2D::from_segments(self.segments)
    }
}

/// Which side of the sketch path a [`Rib`] grows its material on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RibSide {
    /// Symmetric about the path: `thickness / 2` to each side (SolidWorks'
    /// default, and the only exact-distance option).
    Both,
    /// The full `thickness` on the left of the path's travel direction.
    First,
    /// The full `thickness` on the right of the path's travel direction.
    Second,
}

/// An open sketch path thickened into a thin wall and extruded along +Y
/// over `y ∈ [0, height]` — a support **rib**. The path `(u, v)` maps to
/// world `(x, z)` exactly as [`Extrude`] does, so a rib unions cleanly with
/// extruded/revolved parent bodies.
///
/// # Field exactness
///
/// - [`RibSide::Both`] is an exact signed distance field: the symmetric
///   thick-path distance `d_path − thickness/2` is exact and the linear
///   sweep preserves exactness (identical to [`Extrude`]'s slab combine).
/// - [`RibSide::First`] / [`RibSide::Second`] are sign-exact and
///   Lipschitz ≤ 1 (so the default `eval_interval` stays valid), but like
///   partial [`Revolve`] the interior magnitude near the open side and the
///   endpoint caps underestimates the true distance.
pub struct Rib {
    path: OpenPath2D,
    thickness: f64,
    height: f64,
    side: RibSide,
}

impl Rib {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `thickness` or `height` is not
    /// positive and finite.
    pub fn new(path: OpenPath2D, thickness: f64, height: f64, side: RibSide) -> CoreResult<Self> {
        if !(thickness.is_finite() && thickness > 0.0) {
            return Err(invalid(
                "thickness",
                format!("must be positive and finite, got {thickness}"),
            ));
        }
        if !(height.is_finite() && height > 0.0) {
            return Err(invalid(
                "height",
                format!("must be positive and finite, got {height}"),
            ));
        }
        Ok(Self {
            path,
            thickness,
            height,
            side,
        })
    }

    /// Conservative `(min, max)` world-space bounds: the centreline box
    /// grown by the full `thickness` in the sketch plane (`x`, `z`) and
    /// `y ∈ [0, height]`.
    pub fn world_bounds(&self) -> ([f64; 3], [f64; 3]) {
        let (min, max) = self.path.bounds();
        let t = self.thickness;
        (
            [min[0] - t, 0.0, min[1] - t],
            [max[0] + t, self.height, max[1] + t],
        )
    }

    /// Signed distance of the thickened 2D wall footprint at `(u, v)`.
    fn footprint(&self, u: f64, v: f64) -> f64 {
        let (d, side) = self.path.dist_and_side([u, v]);
        match self.side {
            RibSide::Both => d - 0.5 * self.thickness,
            // Keep a band of width `thickness` on one side of the path:
            // within `thickness` of the centreline AND on the chosen side.
            RibSide::First => (d - self.thickness).max(-side),
            RibSide::Second => (d - self.thickness).max(side),
        }
    }
}

impl Sdf for Rib {
    fn eval(&self, p: &Point3) -> f64 {
        let d = self.footprint(p.x, p.z);
        let half = 0.5 * self.height;
        let w = (p.y - half).abs() - half;
        // Exact slab combine, identical to `Extrude`.
        d.max(w).min(0.0) + d.max(0.0).hypot(w.max(0.0))
    }
}

/// A profile revolved around the Y axis: profile `(u, v)` maps to
/// `(radius, y) = (u, v)`, so the profile must lie in `u ≥ 0`.
///
/// A full turn (`angle = 2π`) is an exact SDF. A partial revolve sweeps
/// from the `+X` half-plane towards `+Z` through `angle` radians and is
/// the `max` of the full solid with an exact infinite wedge — sign-exact
/// and Lipschitz ≤ 1 (see the [module docs](self)).
pub struct Revolve {
    profile: Profile2D,
    /// Straight profile segments lying on the axis enclose the region but
    /// sweep to nothing, so they are excluded from the distance (a solid
    /// cylinder's axis is interior, not surface).
    skip: Vec<bool>,
    /// Half the sweep angle; the wedge is evaluated symmetrically.
    half_angle: f64,
    full: bool,
}

impl Revolve {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `angle` is outside `(0, 2π]` or the
    /// profile extends to negative `u` (it would self-overlap when revolved).
    pub fn new(profile: Profile2D, angle: f64) -> CoreResult<Self> {
        if !(angle.is_finite() && angle > 0.0 && angle <= TAU + 1e-9) {
            return Err(invalid(
                "angle",
                format!("must be in (0, 2π] radians, got {angle}"),
            ));
        }
        let (min, _) = profile.bounds();
        if min[0] < -MIN_CHORD {
            return Err(invalid(
                "profile",
                format!(
                    "revolve profile must lie in u >= 0 (radial coordinate), reaches u = {}",
                    min[0]
                ),
            ));
        }
        let skip = profile.axis_line_segments();
        Ok(Self {
            profile,
            skip,
            half_angle: 0.5 * angle,
            full: angle >= TAU - 1e-9,
        })
    }
}

impl Sdf for Revolve {
    fn eval(&self, p: &Point3) -> f64 {
        let r = p.x.hypot(p.z);
        let d = self.profile.signed_distance_masked(r, p.y, &self.skip);
        if self.full {
            return d;
        }
        // Signed distance to the infinite wedge |θ - half_angle| ≤
        // half_angle (a prism along Y with apex on the axis). Fold the
        // angular offset from the wedge bisector to ψ ∈ [0, π]; past a
        // quarter turn from a face, the axis itself is the closest
        // boundary point.
        let mut phi = p.z.atan2(p.x) - self.half_angle;
        phi = phi.rem_euclid(TAU);
        if phi > PI {
            phi -= TAU;
        }
        let psi = phi.abs();
        let wedge = if psi <= self.half_angle {
            let gap = self.half_angle - psi;
            -(if gap < FRAC_PI_2 { r * gap.sin() } else { r })
        } else {
            let gap = psi - self.half_angle;
            if gap < FRAC_PI_2 { r * gap.sin() } else { r }
        };
        d.max(wedge)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::{Cylinder, Torus};
    use crate::transform::SdfTransformExt;
    use opensolid_core::types::Vector3;

    fn unit_square() -> Profile2D {
        Profile2D::new(
            vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            vec![0.0; 4],
        )
        .expect("valid square")
    }

    /// Two bulge-1 arcs on a horizontal diameter form the full circle of
    /// radius `r` centered at `(cx, cy)`.
    fn circle_profile(cx: f64, cy: f64, r: f64) -> Profile2D {
        Profile2D::new(vec![[cx - r, cy], [cx + r, cy]], vec![1.0, 1.0]).expect("valid circle")
    }

    #[test]
    fn square_signed_distance_is_exact() {
        let p = unit_square();
        assert!((p.signed_distance(0.5, 0.5) + 0.5).abs() < 1e-12);
        assert!((p.signed_distance(0.5, 0.25) + 0.25).abs() < 1e-12);
        assert!((p.signed_distance(2.0, 0.5) - 1.0).abs() < 1e-12);
        assert!((p.signed_distance(2.0, 2.0) - 2f64.sqrt()).abs() < 1e-12);
        assert!(p.signed_distance(1.0, 0.5).abs() < 1e-12);
        assert!(p.signed_distance(0.0, 0.0).abs() < 1e-12);
        assert!((p.signed_distance(-1.0, -1.0) - 2f64.sqrt()).abs() < 1e-12);
    }

    #[test]
    fn square_bounds() {
        let (min, max) = unit_square().bounds();
        assert_eq!(min, [0.0, 0.0]);
        assert_eq!(max, [1.0, 1.0]);
    }

    #[test]
    fn circle_profile_matches_analytic_circle() {
        let p = circle_profile(0.0, 0.0, 1.0);
        for (u, v) in [
            (0.0, 0.0),
            (0.5, 0.0),
            (0.0, -0.7),
            (2.0, 0.0),
            (1.5, 1.5),
            (0.0, 1.0),
        ] {
            let expected = f64::hypot(u, v) - 1.0;
            assert!(
                (p.signed_distance(u, v) - expected).abs() < 1e-12,
                "at ({u}, {v}): {} vs {expected}",
                p.signed_distance(u, v)
            );
        }
        // Arc extremes (top/bottom of the circle) are in the bounds even
        // though no vertex is there.
        let (min, max) = p.bounds();
        assert!((min[1] + 1.0).abs() < 1e-12 && (max[1] - 1.0).abs() < 1e-12);
        assert!((min[0] + 1.0).abs() < 1e-12 && (max[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn outward_bulge_adds_area() {
        // Unit square whose right edge bulges out into a semicircle of
        // radius 0.5 centered at (1, 0.5).
        let p = Profile2D::new(
            vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            vec![0.0, 1.0, 0.0, 0.0],
        )
        .expect("valid profile");
        // Bulge apex is at (1.5, 0.5): inside up to it, surface on it.
        assert!(p.signed_distance(1.25, 0.5) < 0.0);
        assert!(p.signed_distance(1.5, 0.5).abs() < 1e-12);
        assert!((p.signed_distance(2.0, 0.5) - 0.5).abs() < 1e-12);
        // Distance inside the bulge region measures to the arc, not chord.
        assert!((p.signed_distance(1.25, 0.5) + 0.25).abs() < 1e-12);
        let (_, max) = p.bounds();
        assert!((max[0] - 1.5).abs() < 1e-12);
    }

    #[test]
    fn inward_bulge_removes_area() {
        // Same square, right edge bulging inward (clockwise arc).
        let p = Profile2D::new(
            vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            vec![0.0, -1.0, 0.0, 0.0],
        )
        .expect("valid profile");
        // The notch reaches to (0.5, 0.5): that point is on the surface,
        // points radially outward of the arc are outside the region.
        assert!(p.signed_distance(0.5, 0.5).abs() < 1e-12);
        assert!((p.signed_distance(0.75, 0.5) - 0.25).abs() < 1e-12);
        assert!(p.signed_distance(0.25, 0.5) < 0.0);
        // Chord midpoint (1, 0.5) is outside now.
        assert!((p.signed_distance(1.0, 0.5) - 0.5).abs() < 1e-12);
    }

    #[test]
    fn major_arc_parity_and_distance() {
        // A single circle drawn as a major (270°) arc plus a minor (90°)
        // arc: region is still the full disk of radius 1 at origin.
        let s = 0.5f64.sqrt();
        let b_major = (3.0 * PI / 8.0).tan(); // 270° sweep: bulge = tan(sweep/4)
        let b_minor = (PI / 8.0).tan(); // 90° sweep
        let p = Profile2D::new(vec![[s, -s], [s, s]], vec![b_minor, b_major])
            .expect("valid two-arc circle");
        for (u, v) in [(0.0, 0.0), (0.9, 0.0), (-0.9, 0.0), (0.0, 1.5), (-2.0, 0.0)] {
            let expected = f64::hypot(u, v) - 1.0;
            assert!(
                (p.signed_distance(u, v) - expected).abs() < 1e-9,
                "at ({u}, {v}): {} vs {expected}",
                p.signed_distance(u, v)
            );
        }
    }

    #[test]
    fn profile_rejects_bad_input() {
        assert!(Profile2D::new(vec![[0.0, 0.0]], vec![0.0]).is_err());
        assert!(Profile2D::new(vec![[0.0, 0.0], [1.0, 0.0]], vec![0.0, 0.0]).is_err());
        assert!(Profile2D::new(vec![[0.0, 0.0], [1.0, 0.0]], vec![0.0]).is_err());
        assert!(
            Profile2D::new(
                vec![[0.0, 0.0], [0.0, 0.0], [1.0, 1.0]],
                vec![0.0, 0.0, 0.0]
            )
            .is_err()
        );
        assert!(
            Profile2D::new(vec![[0.0, f64::NAN], [1.0, 0.0], [1.0, 1.0]], vec![0.0; 3]).is_err()
        );
        assert!(
            Profile2D::new(
                vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0]],
                vec![0.0, f64::NAN, 0.0]
            )
            .is_err()
        );
        // Two vertices with an arc is fine (a lens / circle).
        assert!(Profile2D::new(vec![[0.0, 0.0], [1.0, 0.0]], vec![1.0, 1.0]).is_ok());
    }

    #[test]
    fn extruded_circle_matches_cylinder_primitive() {
        // Extruding a radius-0.5 circle by 2 along Y equals the cylinder
        // primitive shifted so it spans y ∈ [0, 2]. Both fields are exact,
        // so they must agree to machine precision.
        let e = Extrude::new(circle_profile(0.0, 0.0, 0.5), 2.0).expect("valid extrude");
        let c = Cylinder {
            center: Point3::origin(),
            radius: 0.5,
            half_height: 1.0,
        }
        .translated(Vector3::new(0.0, 1.0, 0.0));
        for p in [
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.4, 0.5, 0.1),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(0.0, 2.5, 0.0),
            Point3::new(0.3, -0.5, -0.2),
            Point3::new(2.0, 3.0, -1.0),
        ] {
            assert!(
                (e.eval(&p) - c.eval(&p)).abs() < 1e-12,
                "at {p:?}: {} vs {}",
                e.eval(&p),
                c.eval(&p)
            );
        }
    }

    #[test]
    fn extruded_square_key_distances() {
        let e = Extrude::new(unit_square(), 3.0).expect("valid extrude");
        // Center of the solid: nearest faces are the profile walls (0.5).
        assert!((e.eval(&Point3::new(0.5, 1.5, 0.5)) + 0.5).abs() < 1e-12);
        // Above the top cap.
        assert!((e.eval(&Point3::new(0.5, 4.0, 0.5)) - 1.0).abs() < 1e-12);
        // Diagonal from a top edge: hypot of the two clearances
        // (1 past the profile in x, 1 above the cap in y).
        assert!((e.eval(&Point3::new(2.0, 4.0, 0.5)) - 1f64.hypot(1.0)).abs() < 1e-12);
        // Near the bottom cap from inside.
        assert!((e.eval(&Point3::new(0.5, 0.25, 0.5)) + 0.25).abs() < 1e-12);
    }

    #[test]
    fn extrude_rejects_bad_height() {
        for h in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(Extrude::new(unit_square(), h).is_err(), "height {h}");
        }
    }

    /// A straight diagonal path from `(0,0)` to `(2,0)`.
    fn straight_path() -> OpenPath2D {
        OpenPath2D::new(vec![[0.0, 0.0], [2.0, 0.0]], vec![0.0]).expect("valid path")
    }

    #[test]
    fn open_path_rejects_bad_input() {
        // Too few vertices.
        assert!(OpenPath2D::new(vec![[0.0, 0.0]], vec![]).is_err());
        // Wrong bulge count (open path has n-1 segments).
        assert!(OpenPath2D::new(vec![[0.0, 0.0], [1.0, 0.0]], vec![0.0, 0.0]).is_err());
        assert!(OpenPath2D::new(vec![[0.0, 0.0], [1.0, 0.0], [2.0, 0.0]], vec![0.0]).is_err());
        // Coincident consecutive vertices.
        assert!(OpenPath2D::new(vec![[0.0, 0.0], [0.0, 0.0]], vec![0.0]).is_err());
        // Non-finite.
        assert!(OpenPath2D::new(vec![[0.0, f64::NAN], [1.0, 0.0]], vec![0.0]).is_err());
        assert!(OpenPath2D::new(vec![[0.0, 0.0], [1.0, 0.0]], vec![f64::INFINITY]).is_err());
        // A single straight segment (two vertices, one bulge) is valid —
        // unlike a closed profile, an open path needs no enclosed area.
        assert!(OpenPath2D::new(vec![[0.0, 0.0], [1.0, 0.0]], vec![0.0]).is_ok());
    }

    #[test]
    fn open_path_bounds_include_arc_extremes() {
        // Semicircle from (0,0) to (1,0) with bulge 1: its apex reaches
        // beyond the chord, and the bounds must capture it.
        let p = OpenPath2D::new(vec![[0.0, 0.0], [1.0, 0.0]], vec![1.0]).expect("valid path");
        let (min, max) = p.bounds();
        // Bulge 1 is a CCW semicircle; centre at (0.5, 0), radius 0.5.
        // Travelling CCW from (0,0) to (1,0) dips through the bottom, so the
        // apex sits at (0.5, -0.5): v spans [-0.5, 0], not [0, 0.5].
        assert!((min[1] + 0.5).abs() < 1e-12, "min v = {}", min[1]);
        assert!((max[1]).abs() < 1e-12, "max v = {}", max[1]);
    }

    #[test]
    fn rib_both_sides_is_symmetric_and_exact() {
        let rib = Rib::new(straight_path(), 0.4, 1.0, RibSide::Both).expect("valid rib");
        // Centreline interior: nearest wall is the ±0.2 face.
        assert!((rib.eval(&Point3::new(1.0, 0.5, 0.0)) + 0.2).abs() < 1e-12);
        // On the +z wall face.
        assert!(rib.eval(&Point3::new(1.0, 0.5, 0.2)).abs() < 1e-12);
        // On the -z wall face (symmetry).
        assert!(rib.eval(&Point3::new(1.0, 0.5, -0.2)).abs() < 1e-12);
        // Outside the wall by 0.1.
        assert!((rib.eval(&Point3::new(1.0, 0.5, 0.3)) - 0.1).abs() < 1e-12);
        // Above the top cap by 0.5.
        assert!((rib.eval(&Point3::new(1.0, 1.5, 0.0)) - 0.5).abs() < 1e-12);
        // Diagonal past a top edge: hypot of the two clearances.
        assert!(
            (rib.eval(&Point3::new(1.0, 1.5, 0.5)) - 0.3f64.hypot(0.5)).abs() < 1e-12,
            "got {}",
            rib.eval(&Point3::new(1.0, 1.5, 0.5))
        );
    }

    #[test]
    fn rib_first_side_grows_left_of_travel() {
        // Path runs +x; left of travel is +z (First), right is -z.
        let rib = Rib::new(straight_path(), 0.4, 1.0, RibSide::First).expect("valid rib");
        // Left side gets the full thickness: material out to z = +0.4.
        assert!(rib.eval(&Point3::new(1.0, 0.5, 0.1)) < 0.0);
        assert!(rib.eval(&Point3::new(1.0, 0.5, 0.4)).abs() < 1e-9);
        // Right side gets nothing: even just off the line is outside.
        assert!(rib.eval(&Point3::new(1.0, 0.5, -0.1)) > 0.0);

        // Second is the mirror image.
        let rib2 = Rib::new(straight_path(), 0.4, 1.0, RibSide::Second).expect("valid rib");
        assert!(rib2.eval(&Point3::new(1.0, 0.5, -0.1)) < 0.0);
        assert!(rib2.eval(&Point3::new(1.0, 0.5, 0.1)) > 0.0);
    }

    #[test]
    fn rib_rejects_bad_input() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(Rib::new(straight_path(), bad, 1.0, RibSide::Both).is_err());
            assert!(Rib::new(straight_path(), 0.4, bad, RibSide::Both).is_err());
        }
    }

    #[test]
    fn rib_world_bounds_grow_by_thickness() {
        let rib = Rib::new(straight_path(), 0.4, 1.5, RibSide::Both).expect("valid rib");
        let (min, max) = rib.world_bounds();
        assert_eq!(min, [-0.4, 0.0, -0.4]);
        assert_eq!(max, [2.4, 1.5, 0.4]);
    }

    #[test]
    fn rib_is_lipschitz_one() {
        // The one-sided field is the delicate case; verify the default
        // interval bound stays conservative there.
        let path = OpenPath2D::new(vec![[-0.6, -0.3], [0.2, 0.1], [0.7, -0.2]], vec![0.0, 0.4])
            .expect("valid path");
        let rib = Rib::new(path, 0.25, 0.9, RibSide::First).expect("valid rib");
        crate::test_util::assert_interval_containment(&rib, 71);
    }

    #[test]
    fn rib_meshes_and_unions_with_extrude() {
        use crate::csg::Union;
        use crate::mesh::{MeshOptions, mesh_sdf_indexed};
        use opensolid_core::types::BoundingBox3;

        // A rib between two walls: two upright plates joined by a diagonal
        // rib thickened symmetrically.
        let rib = Rib::new(
            OpenPath2D::new(vec![[0.2, 0.2], [0.8, 0.8]], vec![0.0]).expect("valid path"),
            0.15,
            0.6,
            RibSide::Both,
        )
        .expect("valid rib");
        let mesh = mesh_sdf_indexed(
            &rib,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-0.2, -0.2, -0.2),
                    Point3::new(1.2, 0.8, 1.2),
                ),
                resolution: 32,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());

        // Union with an extruded base plate composes cleanly.
        let plate = Extrude::new(unit_square(), 0.2).expect("valid extrude");
        let combined = Union { a: &rib, b: &plate };
        let mesh = mesh_sdf_indexed(
            &combined,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-0.3, -0.1, -0.3),
                    Point3::new(1.3, 0.9, 1.3),
                ),
                resolution: 40,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn zero_draft_matches_plain_extrude() {
        // with_draft(.., 0.0) must be identical to the straight extrude.
        let plain = Extrude::new(unit_square(), 2.0).expect("valid");
        let drafted = Extrude::with_draft(unit_square(), 2.0, 0.0).expect("valid");
        for p in [
            Point3::new(0.5, 1.0, 0.5),
            Point3::new(2.0, 3.0, -1.0),
            Point3::new(0.5, 0.25, 0.5),
        ] {
            assert_eq!(plain.eval(&p), drafted.eval(&p), "at {p:?}");
        }
    }

    #[test]
    fn draft_wall_distance_is_exact() {
        // A tall unit-square extrude with positive draft: the left wall tilts
        // inward as y grows. For a point just outside that wall, well clear of
        // the other walls and both caps, the field is the exact perpendicular
        // distance to the tilted plane: (profile_dist + tan(draft)·y)·cos.
        let draft = 0.3_f64;
        let e = Extrude::with_draft(unit_square(), 4.0, draft).expect("valid");
        let p = Point3::new(-0.1, 2.0, 0.5);
        let expected = (0.1 + draft.tan() * 2.0) * draft.cos();
        assert!(
            (e.eval(&p) - expected).abs() < 1e-12,
            "{} vs {expected}",
            e.eval(&p)
        );
    }

    #[test]
    fn positive_draft_narrows_toward_top() {
        // Near the top, positive draft insets the section: a point that is
        // inside the straight prism sits outside the drafted one.
        let plain = Extrude::new(unit_square(), 1.0).expect("valid");
        let drafted = Extrude::with_draft(unit_square(), 1.0, 0.3).expect("valid");
        let p = Point3::new(0.05, 0.9, 0.5);
        assert!(plain.eval(&p) < 0.0, "inside the prism");
        assert!(drafted.eval(&p) > 0.0, "outside the drafted top");
    }

    #[test]
    fn negative_draft_flares_toward_top() {
        // Negative draft flares the section outward: a point outside the base
        // wall is captured by the wider top.
        let drafted = Extrude::with_draft(unit_square(), 1.0, -0.3).expect("valid");
        let p = Point3::new(1.2, 0.9, 0.5);
        assert!(drafted.eval(&p) < 0.0, "inside the flared top");
        // Same point at the base (y≈0) is still outside — the base is the
        // untapered profile.
        assert!(drafted.eval(&Point3::new(1.2, 0.02, 0.5)) > 0.0);
    }

    #[test]
    fn extrude_rejects_bad_draft() {
        for d in [
            f64::NAN,
            f64::INFINITY,
            MAX_DRAFT,
            MAX_DRAFT + 0.1,
            -MAX_DRAFT,
        ] {
            assert!(
                Extrude::with_draft(unit_square(), 1.0, d).is_err(),
                "draft {d}"
            );
        }
        // Well within range is accepted.
        assert!(Extrude::with_draft(unit_square(), 1.0, 0.5).is_ok());
        assert!(Extrude::with_draft(unit_square(), 1.0, -0.5).is_ok());
    }

    #[test]
    fn drafted_extrude_interval_containment() {
        // The default eval_interval relies on Lipschitz ≤ 1; the drafted
        // field must satisfy it for the octree mesher to stay sound.
        let e = Extrude::with_draft(
            Profile2D::new(
                vec![[-0.6, -0.4], [0.6, -0.4], [0.6, 0.4], [-0.6, 0.4]],
                vec![0.0, 0.5, 0.0, -0.3],
            )
            .expect("valid profile"),
            1.2,
            0.4,
        )
        .expect("valid extrude");
        crate::test_util::assert_interval_containment(&e, 61);
    }

    #[test]
    fn full_revolve_of_circle_matches_torus_primitive() {
        // Revolving a radius-0.3 circle centered at u = 1 around Y is the
        // torus with major radius 1, minor 0.3. Both exact: machine equal.
        let r = Revolve::new(circle_profile(1.0, 0.0, 0.3), TAU).expect("valid revolve");
        let t = Torus {
            center: Point3::origin(),
            major_radius: 1.0,
            minor_radius: 0.3,
        };
        let mut probe = crate::test_util::Lcg(7);
        for _ in 0..50 {
            let p = Point3::new(
                probe.in_range(-2.0, 2.0),
                probe.in_range(-1.0, 1.0),
                probe.in_range(-2.0, 2.0),
            );
            assert!(
                (r.eval(&p) - t.eval(&p)).abs() < 1e-12,
                "at {p:?}: {} vs {}",
                r.eval(&p),
                t.eval(&p)
            );
        }
    }

    #[test]
    fn half_revolve_respects_the_cut_planes() {
        // Rectangle u ∈ [0.5, 1], v ∈ [0, 1] revolved 180°: covers z ≥ 0.
        let rect = Profile2D::new(
            vec![[0.5, 0.0], [1.0, 0.0], [1.0, 1.0], [0.5, 1.0]],
            vec![0.0; 4],
        )
        .expect("valid rect");
        let r = Revolve::new(rect, PI).expect("valid revolve");

        // Inside the swept half (points at θ = 90°, i.e. +Z).
        assert!(r.eval(&Point3::new(0.0, 0.5, 0.75)) < 0.0);
        // The mirror point on the unswept side is outside...
        let d = r.eval(&Point3::new(0.0, 0.5, -0.75));
        assert!(d > 0.0);
        // ...at exactly the distance to the nearest cut face (the xz
        // distance to the half-plane z = 0, x >= 0 region is |z| here?
        // nearest solid point is (0.75, 0.5, 0) or (-0.75, 0.5, 0), both
        // cut faces: distance = |z| = 0.75... but radial gap matters:
        // (0, 0.5, -0.75) has r = 0.75 ∈ [0.5, 1], so nearest face point
        // is (±0.75, 0.5, 0) at distance 0.75.
        assert!((d - 0.75).abs() < 1e-12, "got {d}");

        // On the start cut face (θ = 0 half-plane): surface.
        assert!(r.eval(&Point3::new(0.75, 0.5, 0.0)).abs() < 1e-12);
        // Just behind it: outside by |z|.
        assert!((r.eval(&Point3::new(0.75, 0.5, -0.1)) - 0.1).abs() < 1e-9);
        // Outer radius still respected in the swept half.
        assert!((r.eval(&Point3::new(0.0, 0.5, 2.0)) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn reflex_revolve_keeps_most_of_the_turn() {
        // 270° revolve: only the quadrant containing -Z/+X... the sweep
        // runs θ ∈ [0°, 270°], so θ = -45° (i.e. 315°) is the gap middle.
        let rect = Profile2D::new(
            vec![[0.5, 0.0], [1.0, 0.0], [1.0, 1.0], [0.5, 1.0]],
            vec![0.0; 4],
        )
        .expect("valid rect");
        let r = Revolve::new(rect, 1.5 * PI).expect("valid revolve");
        let s = 0.75 * 0.5f64.sqrt();
        // Middle of the gap (θ = -45°): outside.
        assert!(r.eval(&Point3::new(s, 0.5, -s)) > 0.0);
        // θ = 180° (swept): inside.
        assert!(r.eval(&Point3::new(-0.75, 0.5, 0.0)) < 0.0);
        // θ = 45° (swept): inside.
        assert!(r.eval(&Point3::new(s, 0.5, s)) < 0.0);
    }

    #[test]
    fn revolve_rejects_bad_input() {
        for angle in [0.0, -1.0, 7.0, f64::NAN] {
            assert!(
                Revolve::new(circle_profile(1.0, 0.0, 0.3), angle).is_err(),
                "angle {angle}"
            );
        }
        // Profile reaching negative u would self-overlap.
        assert!(Revolve::new(circle_profile(0.1, 0.0, 0.5), TAU).is_err());
    }

    #[test]
    fn axis_touching_full_revolve_is_a_solid_cylinder() {
        // Rectangle u ∈ [0, 1], v ∈ [0, 1] revolved fully: solid cylinder.
        let rect = Profile2D::new(
            vec![[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]],
            vec![0.0; 4],
        )
        .expect("valid rect");
        let r = Revolve::new(rect, TAU).expect("valid revolve");
        // The axis is interior: the u = 0 profile edge sweeps to nothing,
        // so the nearest real surfaces are the caps at 0.5.
        assert!((r.eval(&Point3::new(0.0, 0.5, 0.0)) + 0.5).abs() < 1e-12);
        assert!((r.eval(&Point3::new(0.0, 0.5, 2.0)) - 1.0).abs() < 1e-12);
        assert!((r.eval(&Point3::new(0.0, 2.0, 0.0)) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn extrude_interval_containment() {
        let e = Extrude::new(
            Profile2D::new(
                vec![[-0.6, -0.4], [0.6, -0.4], [0.6, 0.4], [-0.6, 0.4]],
                vec![0.0, 0.5, 0.0, -0.3],
            )
            .expect("valid profile"),
            1.2,
        )
        .expect("valid extrude");
        crate::test_util::assert_interval_containment(&e, 51);
    }

    #[test]
    fn revolve_interval_containment_full_and_partial() {
        let full = Revolve::new(circle_profile(1.0, 0.0, 0.3), TAU).expect("valid revolve");
        crate::test_util::assert_interval_containment(&full, 52);

        let rect = Profile2D::new(
            vec![[0.3, -0.4], [1.0, -0.4], [1.0, 0.4], [0.3, 0.4]],
            vec![0.0; 4],
        )
        .expect("valid rect");
        let partial = Revolve::new(rect, 2.0).expect("valid revolve");
        crate::test_util::assert_interval_containment(&partial, 53);
    }

    #[test]
    fn extrude_and_revolve_mesh_through_existing_pipeline() {
        use crate::mesh::{MeshOptions, mesh_sdf_indexed};
        use opensolid_core::types::BoundingBox3;

        let e = Extrude::new(circle_profile(0.0, 0.0, 0.5), 1.0).expect("valid extrude");
        let mesh = mesh_sdf_indexed(
            &e,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-1.0, -0.5, -1.0),
                    Point3::new(1.0, 1.5, 1.0),
                ),
                resolution: 32,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());

        let rect = Profile2D::new(
            vec![[0.3, -0.3], [0.8, -0.3], [0.8, 0.3], [0.3, 0.3]],
            vec![0.0; 4],
        )
        .expect("valid rect");
        let r = Revolve::new(rect, PI).expect("valid revolve");
        let mesh = mesh_sdf_indexed(
            &r,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-1.2, -0.7, -1.2),
                    Point3::new(1.2, 0.7, 1.2),
                ),
                resolution: 32,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    // ---- Curved sketch entities: ellipse + spline (of-fsl.22) ----

    /// A full circle of radius `r` at origin built from two half-ellipse
    /// arcs with `rx = ry = r`.
    fn ellipse_circle(r: f64) -> Profile2D {
        Profile2D::builder([r, 0.0])
            .ellipse_to([0.0, 0.0], r, r, 0.0, PI)
            .ellipse_to([0.0, 0.0], r, r, 0.0, PI)
            .build()
            .expect("valid ellipse circle")
    }

    #[test]
    fn ellipse_with_equal_radii_matches_circle() {
        // rx = ry ellipse arcs must reproduce the exact circle SDF.
        let p = ellipse_circle(1.0);
        let mut probe = crate::test_util::Lcg(11);
        for _ in 0..200 {
            let (u, v) = (probe.in_range(-2.0, 2.0), probe.in_range(-2.0, 2.0));
            let expected = f64::hypot(u, v) - 1.0;
            assert!(
                (p.signed_distance(u, v) - expected).abs() < 1e-9,
                "at ({u}, {v}): {} vs {expected}",
                p.signed_distance(u, v)
            );
        }
    }

    /// A full ellipse `rx`,`ry` (rotation `phi`) at origin, from two arcs.
    fn ellipse_profile(rx: f64, ry: f64, phi: f64) -> Profile2D {
        let start = ellipse_point([0.0, 0.0], rx, ry, phi.cos(), phi.sin(), 0.0);
        Profile2D::builder(start)
            .ellipse_to([0.0, 0.0], rx, ry, phi, PI)
            .ellipse_to([0.0, 0.0], rx, ry, phi, PI)
            .build()
            .expect("valid ellipse")
    }

    #[test]
    fn ellipse_key_distances_axis_aligned() {
        // Ellipse rx = 2, ry = 1 at origin.
        let p = ellipse_profile(2.0, 1.0, 0.0);
        // On the boundary (major/minor axis ends).
        assert!(p.signed_distance(2.0, 0.0).abs() < 1e-9);
        assert!(p.signed_distance(0.0, 1.0).abs() < 1e-9);
        // Centre: nearest boundary point is a minor-axis end, distance 1.
        assert!((p.signed_distance(0.0, 0.0) + 1.0).abs() < 1e-9);
        // Just outside the major-axis end.
        assert!((p.signed_distance(3.0, 0.0) - 1.0).abs() < 1e-9);
        // Just outside the minor-axis end.
        assert!((p.signed_distance(0.0, 1.5) - 0.5).abs() < 1e-9);
        // Inside near the minor axis.
        assert!(p.signed_distance(0.0, 0.5) < 0.0);
    }

    #[test]
    fn ellipse_rotation_ninety_swaps_axes() {
        // Rotating the rx=2, ry=1 ellipse by π/2 makes it tall (2 along v).
        let p = ellipse_profile(2.0, 1.0, FRAC_PI_2);
        assert!(p.signed_distance(0.0, 2.0).abs() < 1e-9);
        assert!(p.signed_distance(1.0, 0.0).abs() < 1e-9);
        // Centre: nearest boundary is now a horizontal end at distance 1.
        assert!((p.signed_distance(0.0, 0.0) + 1.0).abs() < 1e-9);
        assert!((p.signed_distance(0.0, 3.0) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn ellipse_bounds_include_axis_extremes() {
        // Axis-aligned ellipse rx=2, ry=1: bounds are exactly [-2,2]×[-1,1].
        let (min, max) = ellipse_profile(2.0, 1.0, 0.0).bounds();
        assert!((min[0] + 2.0).abs() < 1e-9 && (max[0] - 2.0).abs() < 1e-9);
        assert!((min[1] + 1.0).abs() < 1e-9 && (max[1] - 1.0).abs() < 1e-9);
        // Rotated 90°: bounds swap to [-1,1]×[-2,2].
        let (min, max) = ellipse_profile(2.0, 1.0, FRAC_PI_2).bounds();
        assert!((min[0] + 1.0).abs() < 1e-9 && (max[0] - 1.0).abs() < 1e-9);
        assert!((min[1] + 2.0).abs() < 1e-9 && (max[1] - 2.0).abs() < 1e-9);
    }

    #[test]
    fn cubic_collinear_controls_matches_straight_square() {
        // A unit square whose top edge is a cubic with collinear, evenly
        // spaced control points is geometrically the plain square.
        let straight = unit_square();
        let curved = Profile2D::builder([0.0, 0.0])
            .line_to([1.0, 0.0])
            .line_to([1.0, 1.0])
            // Top edge (1,1) -> (0,1) as a straight cubic.
            .cubic_to([2.0 / 3.0, 1.0], [1.0 / 3.0, 1.0], [0.0, 1.0])
            .build()
            .expect("valid curved square");
        let mut probe = crate::test_util::Lcg(19);
        for _ in 0..200 {
            let (u, v) = (probe.in_range(-1.0, 2.0), probe.in_range(-1.0, 2.0));
            assert!(
                (straight.signed_distance(u, v) - curved.signed_distance(u, v)).abs() < 1e-9,
                "at ({u}, {v}): {} vs {}",
                straight.signed_distance(u, v),
                curved.signed_distance(u, v)
            );
        }
    }

    #[test]
    fn cubic_bulge_signs_and_distance() {
        // A box with a cubic top edge bowing up to apex (0.5, 0.375).
        let p = Profile2D::builder([0.0, 0.0])
            .cubic_to([0.25, 0.5], [0.75, 0.5], [1.0, 0.0])
            .line_to([1.0, -1.0])
            .line_to([0.0, -1.0])
            .build()
            .expect("valid cubic box");
        // On the curve apex.
        assert!(
            p.signed_distance(0.5, 0.375).abs() < 1e-9,
            "apex on boundary"
        );
        // Inside the bump (below apex, above chord).
        assert!(p.signed_distance(0.5, 0.2) < 0.0);
        // Deep inside the box.
        assert!(p.signed_distance(0.5, -0.5) < 0.0);
        // Above the bump: outside, distance to apex = 0.125.
        assert!(p.signed_distance(0.5, 0.5) > 0.0);
        assert!((p.signed_distance(0.5, 0.5) - 0.125).abs() < 1e-9);
    }

    #[test]
    fn extruded_ellipse_meshes_and_is_manifold() {
        use crate::mesh::{MeshOptions, mesh_sdf_indexed};
        use opensolid_core::types::BoundingBox3;

        let e = Extrude::new(ellipse_profile(1.5, 0.7, 0.4), 1.0).expect("valid extrude");
        let mesh = mesh_sdf_indexed(
            &e,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-2.0, -0.5, -2.0),
                    Point3::new(2.0, 1.5, 2.0),
                ),
                resolution: 40,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn extruded_cubic_profile_meshes_and_is_manifold() {
        use crate::mesh::{MeshOptions, mesh_sdf_indexed};
        use opensolid_core::types::BoundingBox3;

        // A teardrop: two cubics bowing out from a sharp corner.
        let p = Profile2D::builder([0.0, 0.0])
            .cubic_to([0.9, 0.2], [0.6, 0.9], [0.0, 1.0])
            .cubic_to([-0.6, 0.9], [-0.9, 0.2], [0.0, 0.0])
            .build()
            .expect("valid teardrop");
        let e = Extrude::new(p, 0.8).expect("valid extrude");
        let mesh = mesh_sdf_indexed(
            &e,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-1.5, -0.3, -1.5),
                    Point3::new(1.5, 1.1, 1.5),
                ),
                resolution: 44,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn ellipse_extrude_interval_containment() {
        // The numeric closest-point field is exactly Euclidean, hence
        // Lipschitz ≤ 1, so the default eval_interval must stay conservative.
        let e = Extrude::new(ellipse_profile(1.3, 0.8, 0.3), 1.1).expect("valid extrude");
        crate::test_util::assert_interval_containment(&e, 67);
    }

    #[test]
    fn cubic_extrude_interval_containment() {
        let p = Profile2D::builder([0.0, 0.0])
            .cubic_to([0.9, 0.2], [0.6, 0.9], [0.0, 1.0])
            .cubic_to([-0.6, 0.9], [-0.9, 0.2], [0.0, 0.0])
            .build()
            .expect("valid teardrop");
        let e = Extrude::new(p, 0.9).expect("valid extrude");
        crate::test_util::assert_interval_containment(&e, 73);
    }

    #[test]
    fn full_ellipse_revolve_meshes() {
        use crate::mesh::{MeshOptions, mesh_sdf_indexed};
        use opensolid_core::types::BoundingBox3;

        // Half-ellipse profile (u ≥ 0) revolved into an ellipsoid-of-revolution.
        let p = Profile2D::builder([1.0, -0.5])
            .ellipse_to([1.0, 0.0], 0.6, 0.5, 0.0, PI)
            .line_to([1.0, -0.5])
            .build()
            .expect("valid half-ellipse profile");
        let r = Revolve::new(p, TAU).expect("valid revolve");
        let mesh = mesh_sdf_indexed(
            &r,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-2.0, -0.8, -2.0),
                    Point3::new(2.0, 0.8, 2.0),
                ),
                resolution: 40,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn builder_rejects_bad_curved_input() {
        // Ellipse radii must be positive.
        assert!(
            Profile2D::builder([1.0, 0.0])
                .ellipse_to([0.0, 0.0], 0.0, 1.0, 0.0, PI)
                .ellipse_to([0.0, 0.0], 1.0, 1.0, 0.0, PI)
                .build()
                .is_err()
        );
        // Ellipse sweep must be non-zero.
        assert!(
            Profile2D::builder([1.0, 0.0])
                .ellipse_to([0.0, 0.0], 1.0, 1.0, 0.0, 0.0)
                .line_to([0.0, 0.0])
                .build()
                .is_err()
        );
        // Non-finite cubic control point.
        assert!(
            Profile2D::builder([0.0, 0.0])
                .cubic_to([f64::NAN, 0.5], [0.75, 0.5], [1.0, 0.0])
                .line_to([0.0, 0.0])
                .build()
                .is_err()
        );
        // No segments at all (nothing added).
        assert!(Profile2D::builder([0.0, 0.0]).build().is_err());
    }

    #[test]
    fn open_path_builder_rib_meshes() {
        use crate::mesh::{MeshOptions, mesh_sdf_indexed};
        use opensolid_core::types::BoundingBox3;

        // Open path with a cubic wiggle, thickened into a rib.
        let path = OpenPath2D::builder([0.1, 0.1])
            .cubic_to([0.4, 0.5], [0.6, -0.3], [0.9, 0.2])
            .build()
            .expect("valid open path");
        let rib = Rib::new(path, 0.15, 0.6, RibSide::Both).expect("valid rib");
        let mesh = mesh_sdf_indexed(
            &rib,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-0.3, -0.2, -0.6),
                    Point3::new(1.3, 0.8, 0.8),
                ),
                resolution: 40,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }
}
