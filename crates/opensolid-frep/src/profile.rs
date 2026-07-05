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
//! - [`Extrude`] is an exact signed distance field (the 2D profile distance
//!   is exact and the linear-sweep combination preserves exactness).
//! - [`Revolve`] over the full turn is exact. A partial revolve is the
//!   intersection (`max`) of the full solid of revolution with an exact
//!   wedge field: sign-correct everywhere, Lipschitz ≤ 1 (so the default
//!   `eval_interval` stays valid), but like all `max` CSG the interior
//!   magnitude near the cut faces underestimates the true distance.

use crate::primitives::Sdf;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::types::Point3;
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
}

fn dist2d(p: [f64; 2], q: [f64; 2]) -> f64 {
    (p[0] - q[0]).hypot(p[1] - q[1])
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
        // Arcs can extend past their endpoints: include each cardinal
        // extreme of the circle that lies on the swept range.
        for seg in &segments {
            if let Segment::Arc {
                center,
                radius,
                start_angle,
                sweep,
                ..
            } = *seg
            {
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
        }

        Ok(Self {
            verts: vertices,
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
                Segment::Arc { .. } => false,
            })
            .collect()
    }
}

/// A profile swept linearly along +Y: profile `(u, v)` maps to world
/// `(x, z) = (u, v)` and the solid spans `y ∈ [0, height]`.
///
/// Exact signed distance field (given the exact profile distance).
pub struct Extrude {
    profile: Profile2D,
    height: f64,
}

impl Extrude {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `height` is not positive and finite.
    pub fn new(profile: Profile2D, height: f64) -> CoreResult<Self> {
        if !(height.is_finite() && height > 0.0) {
            return Err(invalid(
                "height",
                format!("must be positive and finite, got {height}"),
            ));
        }
        Ok(Self { profile, height })
    }
}

impl Sdf for Extrude {
    fn eval(&self, p: &Point3) -> f64 {
        let d = self.profile.signed_distance(p.x, p.z);
        let half = 0.5 * self.height;
        let w = (p.y - half).abs() - half;
        // Standard exact combination of a 2D SDF with a slab: interior
        // takes the larger (closer-to-boundary) term, exterior the
        // Euclidean combination of the positive parts.
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
}
