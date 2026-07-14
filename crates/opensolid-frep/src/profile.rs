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

/// Parametric samples used to seed the Newton closest-point search on
/// ellipse arcs and splines (curves without a closed-form distance).
const CURVE_SEEDS: usize = 24;
/// Newton refinement steps applied to the best seed.
const NEWTON_ITERS: usize = 12;

fn invalid(argument: &'static str, reason: String) -> CoreError {
    CoreError::InvalidArgument { argument, reason }
}

fn dot2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

fn sub2(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

/// 2D cross product `a × b` (the z of the 3D cross of the lifted vectors).
fn cross2(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[1] - a[1] * b[0]
}

/// Real roots of `c2·t² + c1·t + c0` in ascending order (0, 1, or 2), with
/// the linear and constant degeneracies handled.
fn quadratic_roots(c2: f64, c1: f64, c0: f64) -> Vec<f64> {
    if c2.abs() < 1e-300 {
        if c1.abs() < 1e-300 {
            return Vec::new();
        }
        return vec![-c0 / c1];
    }
    let disc = c1 * c1 - 4.0 * c2 * c0;
    if disc < 0.0 {
        return Vec::new();
    }
    let s = disc.sqrt();
    let mut r = [(-c1 - s) / (2.0 * c2), (-c1 + s) / (2.0 * c2)];
    if r[0] > r[1] {
        r.swap(0, 1);
    }
    r.to_vec()
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
    /// Elliptical arc from `a` to `b`, parametrized in the ellipse's local
    /// frame as `p(θ) = center + rx·cos θ·u + ry·sin θ·v` (with `u ⟂ v`
    /// unit), swept from `theta0` through the signed `sweep`.
    EllipseArc {
        a: [f64; 2],
        b: [f64; 2],
        center: [f64; 2],
        u: [f64; 2],
        v: [f64; 2],
        rx: f64,
        ry: f64,
        theta0: f64,
        sweep: f64,
    },
    /// Cubic Bézier from `a` to `b` with control points `c1`, `c2`.
    Spline {
        a: [f64; 2],
        b: [f64; 2],
        c1: [f64; 2],
        c2: [f64; 2],
    },
}

fn dist2d(p: [f64; 2], q: [f64; 2]) -> f64 {
    (p[0] - q[0]).hypot(p[1] - q[1])
}

/// Cubic Bézier point at parameter `t ∈ [0, 1]`.
fn bezier_point(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    let s = 1.0 - t;
    let w0 = s * s * s;
    let w1 = 3.0 * s * s * t;
    let w2 = 3.0 * s * t * t;
    let w3 = t * t * t;
    [
        w0 * a[0] + w1 * c1[0] + w2 * c2[0] + w3 * b[0],
        w0 * a[1] + w1 * c1[1] + w2 * c2[1] + w3 * b[1],
    ]
}

/// Cubic Bézier derivative `dp/dt` at `t`.
fn bezier_deriv(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2], t: f64) -> [f64; 2] {
    let s = 1.0 - t;
    // 3[(c1-a)s² + 2(c2-c1)st + (b-c2)t²]
    let k0 = 3.0 * s * s;
    let k1 = 6.0 * s * t;
    let k2 = 3.0 * t * t;
    [
        k0 * (c1[0] - a[0]) + k1 * (c2[0] - c1[0]) + k2 * (b[0] - c2[0]),
        k0 * (c1[1] - a[1]) + k1 * (c2[1] - c1[1]) + k2 * (b[1] - c2[1]),
    ]
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

    /// Build an elliptical arc from `a` to `b` on the ellipse centered at
    /// `center` with semi-axes `rx`/`ry` and major axis at angle `rotation`,
    /// swept counter-clockwise (`ccw`) or clockwise between the endpoints'
    /// local angles.
    fn ellipse_arc(
        a: [f64; 2],
        b: [f64; 2],
        center: [f64; 2],
        rx: f64,
        ry: f64,
        rotation: f64,
        ccw: bool,
    ) -> Self {
        let u = [rotation.cos(), rotation.sin()];
        let v = [-rotation.sin(), rotation.cos()];
        let theta0 = Self::ellipse_angle(center, u, v, rx, ry, a);
        let theta1 = Self::ellipse_angle(center, u, v, rx, ry, b);
        let sweep = if ccw {
            (theta1 - theta0).rem_euclid(TAU)
        } else {
            -((theta0 - theta1).rem_euclid(TAU))
        };
        Segment::EllipseArc {
            a,
            b,
            center,
            u,
            v,
            rx,
            ry,
            theta0,
            sweep,
        }
    }

    /// Local-frame angle of point `p` on (or projected onto) the ellipse:
    /// `atan2(t / ry, s / rx)` with `(s, t)` the coordinates of `p - center`
    /// in the `(u, v)` basis.
    fn ellipse_angle(
        center: [f64; 2],
        u: [f64; 2],
        v: [f64; 2],
        rx: f64,
        ry: f64,
        p: [f64; 2],
    ) -> f64 {
        let d = sub2(p, center);
        (dot2(d, v) / ry).atan2(dot2(d, u) / rx)
    }

    /// Point on the ellipse at local angle `θ`.
    fn ellipse_point(
        center: [f64; 2],
        u: [f64; 2],
        v: [f64; 2],
        rx: f64,
        ry: f64,
        theta: f64,
    ) -> [f64; 2] {
        let (cx, sx) = (theta.cos(), theta.sin());
        [
            center[0] + rx * cx * u[0] + ry * sx * v[0],
            center[1] + rx * cx * u[1] + ry * sx * v[1],
        ]
    }

    /// True if local angle `theta` lies on the swept range `[theta0, sweep]`.
    fn angle_on_arc(theta0: f64, sweep: f64, theta: f64) -> bool {
        let delta = if sweep >= 0.0 {
            (theta - theta0).rem_euclid(TAU)
        } else {
            (theta0 - theta).rem_euclid(TAU)
        };
        delta <= sweep.abs()
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
            Segment::EllipseArc {
                a,
                b,
                center,
                u,
                v,
                rx,
                ry,
                theta0,
                sweep,
            } => {
                // Newton-minimize |p(θ) - q|² over the swept angular range,
                // seeded from a coarse scan plus the geometric-angle guess.
                let f = |theta: f64| {
                    let p = Self::ellipse_point(center, u, v, rx, ry, theta);
                    dot2(sub2(p, q), sub2(p, q))
                };
                let mut best_theta = theta0;
                let mut best = f(theta0);
                let seed_theta = Self::ellipse_angle(center, u, v, rx, ry, q);
                for k in 0..=CURVE_SEEDS {
                    let theta = theta0 + sweep * (k as f64 / CURVE_SEEDS as f64);
                    let val = f(theta);
                    if val < best {
                        best = val;
                        best_theta = theta;
                    }
                }
                if Self::angle_on_arc(theta0, sweep, seed_theta) && f(seed_theta) < best {
                    best_theta = seed_theta;
                }
                // Newton on g(θ) = (p - q)·p' = 0 (stationary distance).
                let mut theta = best_theta;
                for _ in 0..NEWTON_ITERS {
                    let (cx, sx) = (theta.cos(), theta.sin());
                    let p = [
                        center[0] + rx * cx * u[0] + ry * sx * v[0],
                        center[1] + rx * cx * u[1] + ry * sx * v[1],
                    ];
                    let dp = [
                        -rx * sx * u[0] + ry * cx * v[0],
                        -rx * sx * u[1] + ry * cx * v[1],
                    ];
                    let ddp = [
                        -rx * cx * u[0] - ry * sx * v[0],
                        -rx * cx * u[1] - ry * sx * v[1],
                    ];
                    let g = dot2(sub2(p, q), dp);
                    let gp = dot2(dp, dp) + dot2(sub2(p, q), ddp);
                    if gp.abs() < 1e-300 {
                        break;
                    }
                    let step = g / gp;
                    theta -= step;
                    if step.abs() < 1e-15 {
                        break;
                    }
                }
                let interior = if Self::angle_on_arc(theta0, sweep, theta) {
                    dist2d(Self::ellipse_point(center, u, v, rx, ry, theta), q)
                } else {
                    f64::INFINITY
                };
                interior.min(dist2d(q, a)).min(dist2d(q, b))
            }
            Segment::Spline { a, b, c1, c2 } => {
                let f = |t: f64| {
                    let p = bezier_point(a, c1, c2, b, t);
                    dot2(sub2(p, q), sub2(p, q))
                };
                let mut best_t = 0.0;
                let mut best = f(0.0);
                for k in 1..=CURVE_SEEDS {
                    let t = k as f64 / CURVE_SEEDS as f64;
                    let val = f(t);
                    if val < best {
                        best = val;
                        best_t = t;
                    }
                }
                // Newton on g(t) = (p - q)·p' = 0, clamped to [0, 1].
                let mut t = best_t;
                for _ in 0..NEWTON_ITERS {
                    let p = bezier_point(a, c1, c2, b, t);
                    let dp = bezier_deriv(a, c1, c2, b, t);
                    // p''(t) = 6[(1-t)(c2 - 2c1 + a) + t(b - 2c2 + c1)]
                    let s = 1.0 - t;
                    let ddp = [
                        6.0 * (s * (c2[0] - 2.0 * c1[0] + a[0]) + t * (b[0] - 2.0 * c2[0] + c1[0])),
                        6.0 * (s * (c2[1] - 2.0 * c1[1] + a[1]) + t * (b[1] - 2.0 * c2[1] + c1[1])),
                    ];
                    let g = dot2(sub2(p, q), dp);
                    let gp = dot2(dp, dp) + dot2(sub2(p, q), ddp);
                    if gp.abs() < 1e-300 {
                        break;
                    }
                    let step = (g / gp).clamp(-0.5, 0.5);
                    t = (t - step).clamp(0.0, 1.0);
                    if step.abs() < 1e-15 {
                        break;
                    }
                }
                dist2d(bezier_point(a, c1, c2, b, t), q)
                    .min(dist2d(q, a))
                    .min(dist2d(q, b))
            }
        }
    }

    /// Parity correction for a curved segment: whether `q` lies in the
    /// "lune" between the segment's curve and its straight chord `a → b`.
    ///
    /// The even-odd test over the chord polygon already counts each chord;
    /// XOR-ing this lune parity per segment corrects the chord contribution
    /// to the true curve contribution (see [`Profile2D::contains`]). Returns
    /// `false` for straight segments (curve ≡ chord).
    fn lune_parity(&self, q: [f64; 2]) -> bool {
        match *self {
            Segment::Line { .. } => false,
            Segment::Arc {
                a,
                b,
                bulge,
                center,
                radius,
                ..
            } => {
                // Region between the arc and its chord: inside the circle
                // and on the arc's side of the chord. σ = 0 (on the chord
                // line) is broken with the same virtual +x/+y nudge the
                // even-odd ray test applies implicitly.
                if dist2d(q, center) < radius {
                    let (dx, dy) = (b[0] - a[0], b[1] - a[1]);
                    let cross = dx * (q[1] - a[1]) - dy * (q[0] - a[0]);
                    let side = if cross != 0.0 {
                        cross
                    } else if dy != 0.0 {
                        -dy
                    } else {
                        dx
                    };
                    side * bulge < 0.0
                } else {
                    false
                }
            }
            Segment::EllipseArc {
                a,
                b,
                center,
                u,
                v,
                rx,
                ry,
                theta0,
                sweep,
            } => {
                // Affine image of the circular case: inside the ellipse and
                // on the arc's side of the chord. The arc's side is the side
                // its midpoint falls on.
                let d = sub2(q, center);
                let (s, t) = (dot2(d, u) / rx, dot2(d, v) / ry);
                if s * s + t * t >= 1.0 {
                    return false;
                }
                let mid = Self::ellipse_point(center, u, v, rx, ry, theta0 + 0.5 * sweep);
                let chord = sub2(b, a);
                let mid_side = cross2(chord, sub2(mid, a));
                let cross = cross2(chord, sub2(q, a));
                let side = if cross != 0.0 {
                    cross
                } else if chord[0] != 0.0 {
                    -chord[0]
                } else {
                    chord[1]
                };
                side * mid_side > 0.0
            }
            Segment::Spline { a, b, c1, c2 } => {
                // Lune parity = parity of (ray crossings with the curve) +
                // (ray crossings with the chord); the chord term cancels the
                // even-odd chord contribution, leaving the true curve count.
                (spline_ray_crossings(a, c1, c2, b, q) + chord_ray_crossing(a, b, q)) & 1 == 1
            }
        }
    }

    /// Interior points where the curve reaches an axis extreme (beyond its
    /// endpoints), for tight profile bounds. Empty for straight segments.
    fn extreme_points(&self) -> Vec<[f64; 2]> {
        match *self {
            Segment::Line { .. } => Vec::new(),
            Segment::Arc {
                center,
                radius,
                start_angle,
                sweep,
                ..
            } => {
                let mut pts = Vec::new();
                for k in 0..4 {
                    let ang = k as f64 * FRAC_PI_2;
                    if Self::angle_on_arc(start_angle, sweep, ang) {
                        pts.push([
                            center[0] + radius * ang.cos(),
                            center[1] + radius * ang.sin(),
                        ]);
                    }
                }
                pts
            }
            Segment::EllipseArc {
                center,
                u,
                v,
                rx,
                ry,
                theta0,
                sweep,
                ..
            } => {
                // x'(θ) = 0 at atan2(ry·vx, rx·ux) (+π); likewise y with vy, uy.
                let mut pts = Vec::new();
                for (num, den) in [(ry * v[0], rx * u[0]), (ry * v[1], rx * u[1])] {
                    let base = num.atan2(den);
                    for theta in [base, base + PI] {
                        if Self::angle_on_arc(theta0, sweep, theta) {
                            pts.push(Self::ellipse_point(center, u, v, rx, ry, theta));
                        }
                    }
                }
                pts
            }
            Segment::Spline { a, b, c1, c2 } => {
                // p'(t) = 0 per axis: 3[(c-a) + 2(c2-2c1+a)t + (b-3c2+3c1-a)t²].
                let mut pts = Vec::new();
                for axis in 0..2 {
                    let k2 = b[axis] - 3.0 * c2[axis] + 3.0 * c1[axis] - a[axis];
                    let k1 = 2.0 * (c2[axis] - 2.0 * c1[axis] + a[axis]);
                    let k0 = c1[axis] - a[axis];
                    for t in quadratic_roots(k2, k1, k0) {
                        if t > 0.0 && t < 1.0 {
                            pts.push(bezier_point(a, c1, c2, b, t));
                        }
                    }
                }
                pts
            }
        }
    }
}

/// Half-open crossing count (0 or 1) of the rightward ray from `q` with the
/// straight chord `a → b`, matching the even-odd predicate used in
/// [`Profile2D::contains`].
fn chord_ray_crossing(a: [f64; 2], b: [f64; 2], q: [f64; 2]) -> u32 {
    if (a[1] > q[1]) != (b[1] > q[1]) {
        let x = a[0] + (q[1] - a[1]) / (b[1] - a[1]) * (b[0] - a[0]);
        if x > q[0] {
            return 1;
        }
    }
    0
}

/// Number of times the rightward horizontal ray from `q` crosses the cubic
/// Bézier `a → b` (controls `c1`, `c2`). The curve is split at the roots of
/// `y'(t) = 0` into y-monotone pieces so each transverse crossing is counted
/// once and tangential grazes at extrema are not counted.
fn spline_ray_crossings(a: [f64; 2], c1: [f64; 2], c2: [f64; 2], b: [f64; 2], q: [f64; 2]) -> u32 {
    // y'(t) = 3[(c1-a) + 2(c2-2c1+a)t + (b-3c2+3c1-a)t²]·ŷ, a quadratic.
    let ay = a[1];
    let by = b[1];
    let cy1 = c1[1];
    let cy2 = c2[1];
    let k2 = by - 3.0 * cy2 + 3.0 * cy1 - ay;
    let k1 = 2.0 * (cy2 - 2.0 * cy1 + ay);
    let k0 = cy1 - ay;
    let mut breaks = vec![0.0, 1.0];
    for r in quadratic_roots(k2, k1, k0) {
        if r > 0.0 && r < 1.0 {
            breaks.push(r);
        }
    }
    breaks.sort_by(|x, y| x.partial_cmp(y).unwrap());
    let by_t = |t: f64| bezier_point(a, c1, c2, b, t)[1];
    let mut count = 0u32;
    for w in breaks.windows(2) {
        let (t0, t1) = (w[0], w[1]);
        let (y0, y1) = (by_t(t0), by_t(t1));
        // Half-open crossing on this monotone piece.
        if (y0 > q[1]) != (y1 > q[1]) {
            // Bisect for the parameter where y = q.y, then test x.
            let (mut lo, mut hi) = (t0, t1);
            let ascending = y1 > y0;
            for _ in 0..60 {
                let mid = 0.5 * (lo + hi);
                let ym = by_t(mid);
                if (ym > q[1]) == ascending {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            let t = 0.5 * (lo + hi);
            if bezier_point(a, c1, c2, b, t)[0] > q[0] {
                count += 1;
            }
        }
    }
    count
}

/// One segment of a [`Profile2D`] built via [`Profile2D::from_segments`]:
/// each variant carries the segment's *end* point `to`; its start is the
/// previous segment's end (the first starts at the constructor's `start`,
/// the last closes back to it).
#[derive(Clone, Copy, Debug)]
pub enum SegmentSpec {
    /// Straight line to `to`.
    Line { to: [f64; 2] },
    /// Circular arc to `to`, DXF `bulge = tan(sweep / 4)` (CCW positive).
    Arc { to: [f64; 2], bulge: f64 },
    /// Elliptical arc to `to` on the ellipse centered at `center` with
    /// semi-axes `radius = [rx, ry]` and major axis at `rotation` radians,
    /// traversed counter-clockwise when `ccw`.
    EllipseArc {
        to: [f64; 2],
        center: [f64; 2],
        radius: [f64; 2],
        rotation: f64,
        ccw: bool,
    },
    /// Cubic Bézier to `to` with control points `c1`, `c2`.
    Spline {
        to: [f64; 2],
        c1: [f64; 2],
        c2: [f64; 2],
    },
}

impl SegmentSpec {
    fn end(&self) -> [f64; 2] {
        match *self {
            SegmentSpec::Line { to }
            | SegmentSpec::Arc { to, .. }
            | SegmentSpec::EllipseArc { to, .. }
            | SegmentSpec::Spline { to, .. } => to,
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

        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        let mut include = |p: [f64; 2]| {
            min[0] = min[0].min(p[0]);
            min[1] = min[1].min(p[1]);
            max[0] = max[0].max(p[0]);
            max[1] = max[1].max(p[1]);
        };
        for &v in &vertices {
            include(v);
        }
        // Curved segments can extend past their endpoints: include each
        // axis extreme reached on the swept range.
        for seg in &segments {
            for p in seg.extreme_points() {
                include(p);
            }
        }

        Ok(Self {
            verts: vertices,
            segments,
            min,
            max,
        })
    }

    /// Build a profile from a start point and a loop of typed segments
    /// (lines, circular arcs, elliptical arcs, cubic Béziers). Segment `i`
    /// runs from vertex `i` to vertex `i + 1`; the final segment closes the
    /// loop back to `start`, so its `to` must coincide with `start`.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if fewer than two segments are given,
    /// any coordinate or parameter is not finite, an ellipse radius is not
    /// positive, consecutive vertices coincide, or the loop does not close.
    pub fn from_segments(start: [f64; 2], specs: Vec<SegmentSpec>) -> CoreResult<Self> {
        if specs.len() < 2 {
            return Err(invalid(
                "specs",
                format!("profile needs at least 2 segments, got {}", specs.len()),
            ));
        }
        let finite2 = |p: [f64; 2]| p[0].is_finite() && p[1].is_finite();
        if !finite2(start) {
            return Err(invalid("start", "coordinates must be finite".into()));
        }
        for (i, spec) in specs.iter().enumerate() {
            let ok = finite2(spec.end())
                && match *spec {
                    SegmentSpec::Line { .. } => true,
                    SegmentSpec::Arc { bulge, .. } => bulge.is_finite(),
                    SegmentSpec::EllipseArc {
                        center,
                        radius,
                        rotation,
                        ..
                    } => {
                        finite2(center)
                            && rotation.is_finite()
                            && radius[0].is_finite()
                            && radius[1].is_finite()
                            && radius[0] > 0.0
                            && radius[1] > 0.0
                    }
                    SegmentSpec::Spline { c1, c2, .. } => finite2(c1) && finite2(c2),
                };
            if !ok {
                return Err(invalid(
                    "specs",
                    format!("segment {i} has non-finite or non-positive parameters"),
                ));
            }
        }
        // Loop vertices: start, then every segment end except the last
        // (which closes back to start).
        let n = specs.len();
        if dist2d(specs[n - 1].end(), start) >= MIN_CHORD {
            return Err(invalid(
                "specs",
                "the last segment must close the loop back to start".into(),
            ));
        }
        let mut verts = Vec::with_capacity(n);
        verts.push(start);
        for spec in &specs[..n - 1] {
            verts.push(spec.end());
        }

        let mut segments = Vec::with_capacity(n);
        for i in 0..n {
            let a = verts[i];
            let b = verts[(i + 1) % n];
            if dist2d(a, b) < MIN_CHORD {
                return Err(invalid(
                    "specs",
                    format!("segment {i} is degenerate: consecutive vertices coincide"),
                ));
            }
            segments.push(match specs[i] {
                SegmentSpec::Line { .. } => Segment::Line { a, b },
                SegmentSpec::Arc { bulge, .. } => Segment::new(a, b, bulge),
                SegmentSpec::EllipseArc {
                    center,
                    radius,
                    rotation,
                    ccw,
                    ..
                } => Segment::ellipse_arc(a, b, center, radius[0], radius[1], rotation, ccw),
                SegmentSpec::Spline { c1, c2, .. } => Segment::Spline { a, b, c1, c2 },
            });
        }

        let mut min = [f64::INFINITY; 2];
        let mut max = [f64::NEG_INFINITY; 2];
        let mut include = |p: [f64; 2]| {
            min[0] = min[0].min(p[0]);
            min[1] = min[1].min(p[1]);
            max[0] = max[0].max(p[0]);
            max[1] = max[1].max(p[1]);
        };
        for &v in &verts {
            include(v);
        }
        for seg in &segments {
            for p in seg.extreme_points() {
                include(p);
            }
        }

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
    /// Curved-segment chords are *interior* lines of the region, so a query
    /// exactly on one must not be misclassified: [`Segment::lune_parity`]
    /// breaks the σ = 0 tie by emulating the same virtual `+x` (then `+y`)
    /// nudge the even-odd ray test applies implicitly, keeping the two
    /// parity sources consistent.
    fn contains(&self, q: [f64; 2]) -> bool {
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
            if seg.lune_parity(q) {
                inside = !inside;
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
                _ => false,
            })
            .collect()
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

    // --- Curved segments: ellipse arcs and cubic-Bézier splines ---

    /// Full ellipse (rx, ry) at origin, drawn as an upper + lower CCW arc
    /// between (rx, 0) and (-rx, 0).
    fn ellipse_profile(rx: f64, ry: f64, rotation: f64) -> Profile2D {
        let u = [rotation.cos(), rotation.sin()];
        let end = |sx: f64| [rx * sx * u[0], rx * sx * u[1]];
        Profile2D::from_segments(
            end(1.0),
            vec![
                SegmentSpec::EllipseArc {
                    to: end(-1.0),
                    center: [0.0, 0.0],
                    radius: [rx, ry],
                    rotation,
                    ccw: true,
                },
                SegmentSpec::EllipseArc {
                    to: end(1.0),
                    center: [0.0, 0.0],
                    radius: [rx, ry],
                    rotation,
                    ccw: true,
                },
            ],
        )
        .expect("valid ellipse")
    }

    /// Brute-force closest distance from `q` to an origin-centered ellipse
    /// with axes `(rx, ry)` rotated by `rotation`, by dense sampling.
    fn brute_ellipse_distance(rx: f64, ry: f64, rotation: f64, q: [f64; 2]) -> f64 {
        let (cu, su) = (rotation.cos(), rotation.sin());
        let mut best = f64::INFINITY;
        for k in 0..200_000 {
            let t = k as f64 / 200_000.0 * TAU;
            let (c, s) = (t.cos(), t.sin());
            let p = [rx * c * cu - ry * s * su, rx * c * su + ry * s * cu];
            best = best.min(dist2d(p, q));
        }
        best
    }

    #[test]
    fn ellipse_with_equal_radii_matches_circle() {
        // rx = ry reduces the elliptical arc to a circular one: the field
        // must match the analytic circle distance.
        let p = ellipse_profile(1.0, 1.0, 0.0);
        for (u, v) in [
            (0.0, 0.0),
            (0.5, 0.0),
            (0.0, -0.7),
            (2.0, 0.0),
            (1.5, 1.5),
            (0.0, 1.0),
            (-0.3, 0.4),
        ] {
            let expected = f64::hypot(u, v) - 1.0;
            assert!(
                (p.signed_distance(u, v) - expected).abs() < 1e-9,
                "at ({u}, {v}): {} vs {expected}",
                p.signed_distance(u, v)
            );
        }
    }

    #[test]
    fn ellipse_distance_matches_brute_force() {
        let (rx, ry, rot) = (2.0, 1.0, 0.6);
        let p = ellipse_profile(rx, ry, rot);
        let mut probe = crate::test_util::Lcg(11);
        for _ in 0..40 {
            let q = [probe.in_range(-3.0, 3.0), probe.in_range(-3.0, 3.0)];
            let got = p.signed_distance(q[0], q[1]).abs();
            let want = brute_ellipse_distance(rx, ry, rot, q);
            assert!((got - want).abs() < 1e-4, "at {q:?}: {got} vs {want}");
        }
    }

    #[test]
    fn ellipse_parity_inside_outside() {
        let p = ellipse_profile(2.0, 1.0, 0.0);
        // Inside: within the ellipse but outside its inscribed circle.
        assert!(p.signed_distance(1.5, 0.0) < 0.0);
        assert!(p.signed_distance(0.0, 0.5) < 0.0);
        // Outside: past the minor axis but inside the major-axis span.
        assert!(p.signed_distance(0.0, 1.5) > 0.0);
        assert!(p.signed_distance(1.5, 0.9) > 0.0);
    }

    #[test]
    fn ellipse_bounds_include_axis_extremes() {
        // A 45°-rotated 2×1 ellipse reaches beyond its endpoints; bounds
        // must contain the true extent sqrt(rx²cos²+ ry²sin²) along x.
        let rot = std::f64::consts::FRAC_PI_4;
        let p = ellipse_profile(2.0, 1.0, rot);
        let (min, max) = p.bounds();
        let ext = ((2.0f64 * rot.cos()).powi(2) + (1.0f64 * rot.sin()).powi(2)).sqrt();
        assert!((max[0] - ext).abs() < 1e-9, "max {} vs {ext}", max[0]);
        assert!((min[0] + ext).abs() < 1e-9, "min {} vs {ext}", min[0]);
    }

    /// A closed profile whose top edge is a cubic Bézier bulging up, over a
    /// square base — for parity and distance checks.
    fn bezier_cap_profile(c1: [f64; 2], c2: [f64; 2]) -> Profile2D {
        Profile2D::from_segments(
            [0.0, 0.0],
            vec![
                SegmentSpec::Line { to: [1.0, 0.0] },
                SegmentSpec::Line { to: [1.0, 1.0] },
                SegmentSpec::Spline {
                    to: [0.0, 1.0],
                    c1,
                    c2,
                },
                SegmentSpec::Line { to: [0.0, 0.0] },
            ],
        )
        .expect("valid bezier cap")
    }

    #[test]
    fn spline_collinear_controls_reduce_to_a_line() {
        // Evenly spaced collinear controls make the top "spline" the exact
        // straight edge (1,1) → (0,1); the shape is the unit square.
        let p = bezier_cap_profile([2.0 / 3.0, 1.0], [1.0 / 3.0, 1.0]);
        assert!((p.signed_distance(0.5, 0.5) + 0.5).abs() < 1e-9);
        assert!((p.signed_distance(0.5, 1.25) - 0.25).abs() < 1e-6);
        assert!((p.signed_distance(0.5, 0.75) + 0.25).abs() < 1e-6);
    }

    #[test]
    fn spline_bulge_adds_area_and_distance_is_brute_force_exact() {
        // Controls pull the top edge up to a peak near y ≈ 1.5.
        let (c1, c2) = ([0.75, 2.0], [0.25, 2.0]);
        let p = bezier_cap_profile(c1, c2);
        let a = [1.0, 1.0];
        let b = [0.0, 1.0];
        // Brute-force min distance to the whole boundary.
        let brute = |q: [f64; 2]| {
            let mut best = f64::INFINITY;
            let seg = |s: [f64; 2], e: [f64; 2], best: &mut f64| {
                for k in 0..=4000 {
                    let t = k as f64 / 4000.0;
                    let pt = [s[0] + t * (e[0] - s[0]), s[1] + t * (e[1] - s[1])];
                    *best = best.min(dist2d(pt, q));
                }
            };
            seg([0.0, 0.0], [1.0, 0.0], &mut best);
            seg([1.0, 0.0], [1.0, 1.0], &mut best);
            seg([0.0, 1.0], [0.0, 0.0], &mut best);
            for k in 0..=8000 {
                let t = k as f64 / 8000.0;
                best = best.min(dist2d(bezier_point(a, c1, c2, b, t), q));
            }
            best
        };
        // A point above the chord but below the bulge peak is inside.
        assert!(
            p.signed_distance(0.5, 1.3) < 0.0,
            "under the bulge is inside"
        );
        // Above the peak is outside.
        assert!(
            p.signed_distance(0.5, 2.2) > 0.0,
            "above the bulge is outside"
        );
        let mut probe = crate::test_util::Lcg(23);
        for _ in 0..40 {
            let q = [probe.in_range(-0.5, 1.5), probe.in_range(-0.5, 2.5)];
            let got = p.signed_distance(q[0], q[1]).abs();
            let want = brute(q);
            assert!((got - want).abs() < 2e-3, "at {q:?}: {got} vs {want}");
        }
    }

    #[test]
    fn spline_bounds_cover_the_bulge() {
        let (c1, c2) = ([0.75, 2.0], [0.25, 2.0]);
        let p = bezier_cap_profile(c1, c2);
        let (_, max) = p.bounds();
        // Peak of a symmetric cubic with both controls at y = 2: y_max =
        // 0.25·1 + 0.75·2 = ... = (1 + 3·2 + ... )/8. Just assert it clears
        // the chord (y = 1) by a solid margin and is below the control (2).
        assert!(max[1] > 1.4 && max[1] < 2.0, "peak {}", max[1]);
    }

    #[test]
    fn from_segments_rejects_bad_input() {
        // Fewer than two segments.
        assert!(
            Profile2D::from_segments([0.0, 0.0], vec![SegmentSpec::Line { to: [1.0, 0.0] }])
                .is_err()
        );
        // Loop does not close.
        assert!(
            Profile2D::from_segments(
                [0.0, 0.0],
                vec![
                    SegmentSpec::Line { to: [1.0, 0.0] },
                    SegmentSpec::Line { to: [1.0, 1.0] },
                ],
            )
            .is_err()
        );
        // Non-positive ellipse radius.
        assert!(
            Profile2D::from_segments(
                [1.0, 0.0],
                vec![
                    SegmentSpec::EllipseArc {
                        to: [-1.0, 0.0],
                        center: [0.0, 0.0],
                        radius: [0.0, 1.0],
                        rotation: 0.0,
                        ccw: true,
                    },
                    SegmentSpec::EllipseArc {
                        to: [1.0, 0.0],
                        center: [0.0, 0.0],
                        radius: [1.0, 1.0],
                        rotation: 0.0,
                        ccw: true,
                    },
                ],
            )
            .is_err()
        );
        // Non-finite spline control.
        assert!(
            Profile2D::from_segments(
                [0.0, 0.0],
                vec![
                    SegmentSpec::Line { to: [1.0, 0.0] },
                    SegmentSpec::Spline {
                        to: [0.0, 0.0],
                        c1: [f64::NAN, 0.0],
                        c2: [0.5, 1.0],
                    },
                ],
            )
            .is_err()
        );
    }

    #[test]
    fn extruded_ellipse_meshes_to_closed_manifold() {
        use crate::mesh::{MeshOptions, mesh_sdf_indexed};
        use opensolid_core::types::BoundingBox3;

        let e = Extrude::new(ellipse_profile(1.5, 0.8, 0.3), 1.0).expect("valid extrude");
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
    fn extruded_spline_profile_meshes_to_closed_manifold() {
        use crate::mesh::{MeshOptions, mesh_sdf_indexed};
        use opensolid_core::types::BoundingBox3;

        let p = bezier_cap_profile([0.75, 2.0], [0.25, 2.0]);
        let e = Extrude::new(p, 1.0).expect("valid extrude");
        let mesh = mesh_sdf_indexed(
            &e,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-0.5, -0.5, -0.5),
                    Point3::new(1.5, 1.5, 2.5),
                ),
                resolution: 40,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn ellipse_interval_containment() {
        let e = Extrude::new(ellipse_profile(1.2, 0.7, 0.4), 1.0).expect("valid extrude");
        crate::test_util::assert_interval_containment(&e, 61);
    }
}
