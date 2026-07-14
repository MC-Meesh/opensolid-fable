//! Edge-selective fillet & chamfer: a boolean whose sharp `min`/`max` is
//! replaced by a smooth (fillet) or beveled (chamfer) blend **only near a
//! selected edge**, leaving the rest of the model crisp.
//!
//! The global blends in [`crate::blend`] round *every* edge a boolean
//! produces, at one radius. A CAD fillet must instead target one edge and
//! leave its neighbours untouched. [`EdgeBlend`] does this by windowing the
//! blend radius with the distance to a selected [`EdgeRegion`] (the
//! feature-edge polyline the mesher's CSG-edge detection already recovers):
//!
//! ```text
//! d(p)      = distance from p to the selected edge polyline
//! w(p)      = 1 on the edge, tapering smoothly to 0 past `influence`
//! r_eff(p)  = radius * w(p)
//! ```
//!
//! Where `w = 0` the effective radius is 0 and the operator is exactly the
//! sharp boolean, so untouched edges stay sharp. See
//! `docs/design/edge-fillet-chamfer.md` for the full design.
//!
//! # Metric properties
//!
//! The windowed field is not globally 1-Lipschitz, so [`EdgeBlend`] supplies
//! its own conservative [`eval_interval`](Sdf::eval_interval): the blend only
//! ever pulls the field off the sharp value, and by a bounded amount, so a
//! one-sided widening of the sharp interval contains it. [`branches`](Sdf::branches)
//! reports a single smooth branch inside the blend (so refinement does not
//! re-sharpen the fillet) and forwards the sharp boolean's branches elsewhere.

use crate::csg::{Intersection, Subtraction, Union};
use crate::primitives::Sdf;
use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use std::f64::consts::FRAC_1_SQRT_2;

/// Below this effective radius the blend collapses to the sharp boolean. Also
/// required for correctness: the chamfer formula does not reduce to `min` at
/// `r = 0`, so it must be short-circuited rather than evaluated there.
const RADIUS_EPS: f64 = 1e-9;

/// The window reaches zero at `INFLUENCE_FACTOR * radius` from the edge. It
/// exceeds 1 so the full fillet cross-section (which bulges ~`radius` off the
/// edge) sits inside the full-weight band `[0, radius]`.
const INFLUENCE_FACTOR: f64 = 2.0;

/// Which boolean the selected edge belongs to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BooleanKind {
    /// `min(a, b)`.
    Union,
    /// `max(a, b)`.
    Intersection,
    /// `max(a, -b)` — `a` minus `b`.
    Subtraction,
}

impl BooleanKind {
    /// Re-express the boolean as `sign * min(x, y)` so one smooth/chamfer
    /// `min` serves all three kinds. `Intersection = -min(-a, -b)` and
    /// `Subtraction = -min(-a, b)`.
    fn signed(self, fa: f64, fb: f64) -> (f64, f64, f64) {
        match self {
            BooleanKind::Union => (fa, fb, 1.0),
            BooleanKind::Intersection => (-fa, -fb, -1.0),
            BooleanKind::Subtraction => (-fa, fb, -1.0),
        }
    }

    /// Interval form of [`signed`](Self::signed): the two `min`-space operand
    /// intervals and whether an outer negation applies.
    fn signed_interval(self, ia: &Interval, ib: &Interval) -> (Interval, Interval, bool) {
        let neg = |i: &Interval| Interval::new(-i.hi, -i.lo);
        match self {
            BooleanKind::Union => (*ia, *ib, false),
            BooleanKind::Intersection => (neg(ia), neg(ib), true),
            BooleanKind::Subtraction => (neg(ia), *ib, true),
        }
    }
}

/// Fillet (rounded blend) or chamfer (planar bevel).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BlendMode {
    /// Rounded blend of radius `radius` (the polynomial smooth-min).
    Fillet,
    /// Planar bevel with setback `radius` (the hg_sdf chamfer-min).
    Chamfer,
}

/// The selected edge, as the set of 3D segments the mesher's CSG-edge
/// detection produces (crease-labeled feature vertices, chained). The blend
/// is localized to the neighbourhood of these segments.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct EdgeRegion {
    segments: Vec<[Point3; 2]>,
}

impl EdgeRegion {
    /// A region from explicit segments. An empty region blends nowhere, so
    /// the boolean stays fully sharp.
    pub fn new(segments: Vec<[Point3; 2]>) -> Self {
        Self { segments }
    }

    /// A region from a polyline, chaining each consecutive pair of points
    /// into a segment. Fewer than two points yields an empty region.
    pub fn from_polyline(points: &[Point3]) -> Self {
        let segments = points.windows(2).map(|w| [w[0], w[1]]).collect();
        Self { segments }
    }

    /// Number of segments in the region.
    pub fn len(&self) -> usize {
        self.segments.len()
    }

    /// True if the region has no segments (blends nowhere).
    pub fn is_empty(&self) -> bool {
        self.segments.is_empty()
    }

    /// Euclidean distance from `p` to the nearest segment; `+∞` if empty.
    pub fn distance(&self, p: &Point3) -> f64 {
        self.segments
            .iter()
            .map(|[a, b]| point_segment_distance(p, a, b))
            .fold(f64::INFINITY, f64::min)
    }
}

/// Distance from a point to the closed segment `[a, b]`.
fn point_segment_distance(p: &Point3, a: &Point3, b: &Point3) -> f64 {
    let ab = b - a;
    let denom = ab.dot(&ab);
    let t = if denom > 0.0 {
        ((p - a).dot(&ab) / denom).clamp(0.0, 1.0)
    } else {
        0.0
    };
    (p - (a + ab * t)).norm()
}

/// The polynomial smooth-min used by [`crate::blend::SmoothUnion`]. Deviates
/// from `min(x, y)` by at most `r/4`, always downward.
fn smooth_min(x: f64, y: f64, r: f64) -> f64 {
    let h = (0.5 + 0.5 * (y - x) / r).clamp(0.0, 1.0);
    y * (1.0 - h) + x * h - r * h * (1.0 - h)
}

/// The hg_sdf chamfer-min: a planar bevel. Deviates from `min(x, y)` by at
/// most `r * √½` at the edge, always downward.
fn chamfer_min(x: f64, y: f64, r: f64) -> f64 {
    x.min(y).min((x + y - r) * FRAC_1_SQRT_2)
}

/// A boolean of `a` and `b` whose sharp edge is filleted or chamfered only
/// within [`EdgeRegion`]. Elsewhere it is exactly the sharp boolean.
///
/// Generic over the operand fields so it stays on the zero-cost path;
/// [`crate::Shape::blend_edge`] builds one from runtime `Shape` handles.
pub struct EdgeBlend<A, B> {
    pub a: A,
    pub b: B,
    pub kind: BooleanKind,
    pub mode: BlendMode,
    /// Fillet radius / chamfer setback. Must be finite and non-negative.
    pub radius: f64,
    pub region: EdgeRegion,
}

impl<A, B> EdgeBlend<A, B> {
    /// Build an edge blend. `radius` must be finite and non-negative; a zero
    /// radius (or empty region) leaves the boolean fully sharp.
    pub fn new(
        a: A,
        b: B,
        kind: BooleanKind,
        mode: BlendMode,
        radius: f64,
        region: EdgeRegion,
    ) -> Self {
        debug_assert!(
            radius.is_finite() && radius >= 0.0,
            "fillet radius must be finite and non-negative, got {radius}"
        );
        Self {
            a,
            b,
            kind,
            mode,
            radius,
            region,
        }
    }

    fn influence(&self) -> f64 {
        INFLUENCE_FACTOR * self.radius
    }

    /// Blend weight at `p`: 1 within `radius` of the edge, a smoothstep taper
    /// to 0 across `[radius, influence]`, 0 beyond.
    fn weight(&self, p: &Point3) -> f64 {
        let d = self.region.distance(p);
        if d <= self.radius {
            1.0
        } else if d >= self.influence() {
            0.0
        } else {
            let t = (d - self.radius) / (self.influence() - self.radius);
            // 1 - smoothstep(t): C1 at both ends of the taper band.
            1.0 - t * t * (3.0 - 2.0 * t)
        }
    }
}

impl<A: Sdf, B: Sdf> Sdf for EdgeBlend<A, B> {
    fn eval(&self, p: &Point3) -> f64 {
        let fa = self.a.eval(p);
        let fb = self.b.eval(p);
        let (x, y, sign) = self.kind.signed(fa, fb);
        let r = self.radius * self.weight(p);
        if r < RADIUS_EPS {
            return sign * x.min(y);
        }
        let blended = match self.mode {
            BlendMode::Fillet => smooth_min(x, y, r),
            BlendMode::Chamfer => chamfer_min(x, y, r),
        };
        sign * blended
    }

    fn eval_interval(&self, bx: &BoundingBox3) -> Interval {
        let ia = self.a.eval_interval(bx);
        let ib = self.b.eval_interval(bx);
        let (ix, iy, neg) = self.kind.signed_interval(&ia, &ib);
        let im = ix.min(&iy); // min-space sharp interval

        // A conservative lower bound on the box-to-edge distance: every point
        // is within a half-diagonal of the center. If even the nearest point
        // is beyond `influence`, the whole box is sharp and needs no widening
        // — this keeps deep-interior cells prunable, as `SmoothUnion` does.
        let half_diag = 0.5 * bx.extents().norm();
        let minspace = if self.region.distance(&bx.center()) - half_diag >= self.influence() {
            im
        } else {
            // Somewhere in the box the blend may be active. The field lies
            // between the sharp op (w = 0) and the full-radius blend (w = 1),
            // both monotone in the effective radius, so hulling the two
            // bounds the windowed field for every intermediate weight.
            let blended = match self.mode {
                // smooth-min ∈ [min - r/4, min], one-sided and radius-bounded.
                BlendMode::Fillet => Interval::new(im.lo - 0.25 * self.radius, im.hi),
                // chamfer-min = min(min(x,y), (x+y-r)·√½), by interval
                // arithmetic on the closed form (the two branches treated
                // independently only widens).
                BlendMode::Chamfer => {
                    let plane = Interval::new(
                        (ix.lo + iy.lo - self.radius) * FRAC_1_SQRT_2,
                        (ix.hi + iy.hi - self.radius) * FRAC_1_SQRT_2,
                    );
                    im.min(&plane)
                }
            };
            im.hull(&blended)
        };

        if neg {
            Interval::new(-minspace.hi, -minspace.lo)
        } else {
            minspace
        }
    }

    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let r = self.radius * self.weight(p);
        if r < RADIUS_EPS {
            // Sharp here: hand the mesher the boolean's branch decomposition
            // so refinement snaps the crease onto the analytic edge.
            match self.kind {
                BooleanKind::Union => Union {
                    a: &self.a,
                    b: &self.b,
                }
                .branches(p, tol, out),
                BooleanKind::Intersection => Intersection {
                    a: &self.a,
                    b: &self.b,
                }
                .branches(p, tol, out),
                BooleanKind::Subtraction => Subtraction {
                    a: &self.a,
                    b: &self.b,
                }
                .branches(p, tol, out),
            }
        } else {
            // Inside the blend the surface is smooth: one branch, so
            // refinement leaves the fillet alone instead of re-sharpening it.
            out.push((self.eval(p), self.grad(p)));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::HalfSpace;

    /// Solid `{x < 0} ∪ {y < 0}`: a convex edge along the z-axis.
    fn union_scene() -> (HalfSpace, HalfSpace) {
        (
            HalfSpace {
                normal: Vector3::new(1.0, 0.0, 0.0),
                offset: 0.0,
            },
            HalfSpace {
                normal: Vector3::new(0.0, 1.0, 0.0),
                offset: 0.0,
            },
        )
    }

    fn z_axis_region() -> EdgeRegion {
        EdgeRegion::from_polyline(&[Point3::new(0.0, 0.0, -10.0), Point3::new(0.0, 0.0, 10.0)])
    }

    fn blend(
        kind: BooleanKind,
        mode: BlendMode,
        radius: f64,
        region: EdgeRegion,
    ) -> EdgeBlend<HalfSpace, HalfSpace> {
        let (a, b) = union_scene();
        EdgeBlend::new(a, b, kind, mode, radius, region)
    }

    #[test]
    fn region_distance_and_polyline() {
        let r = z_axis_region();
        assert_eq!(r.len(), 1);
        assert!(!r.is_empty());
        // On the axis: distance 0.
        assert!(r.distance(&Point3::new(0.0, 0.0, 3.0)) < 1e-12);
        // Off the axis (within the segment's z-span): the radial distance.
        assert!((r.distance(&Point3::new(3.0, 4.0, 1.0)) - 5.0).abs() < 1e-12);
        // Empty region: infinite distance.
        assert!(
            EdgeRegion::default()
                .distance(&Point3::origin())
                .is_infinite()
        );
    }

    #[test]
    fn from_polyline_needs_two_points() {
        assert!(EdgeRegion::from_polyline(&[Point3::origin()]).is_empty());
        assert_eq!(
            EdgeRegion::from_polyline(&[
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0)
            ])
            .len(),
            2
        );
    }

    #[test]
    fn far_from_edge_matches_sharp() {
        let (a, b) = union_scene();
        let eb = blend(BooleanKind::Union, BlendMode::Fillet, 0.3, z_axis_region());
        // A boundary point on the x = 0 face, far (5 units) from the z-axis.
        let p = Point3::new(0.0, 5.0, 0.0);
        let sharp = a.eval(&p).min(b.eval(&p));
        assert!((eb.eval(&p) - sharp).abs() < 1e-12);
    }

    #[test]
    fn empty_region_is_fully_sharp() {
        let (a, b) = union_scene();
        let eb = blend(
            BooleanKind::Union,
            BlendMode::Fillet,
            0.5,
            EdgeRegion::default(),
        );
        for p in [
            Point3::origin(),
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(-0.3, -0.2, 0.0),
        ] {
            let sharp = a.eval(&p).min(b.eval(&p));
            assert!((eb.eval(&p) - sharp).abs() < 1e-12, "at {p:?}");
        }
    }

    #[test]
    fn union_fillet_adds_material_at_edge() {
        let radius = 0.4;
        let eb = blend(
            BooleanKind::Union,
            BlendMode::Fillet,
            radius,
            z_axis_region(),
        );
        let p = Point3::origin();
        // On the convex edge the sharp field is 0; the fillet fills the notch,
        // making the field more negative (inside) by exactly r/4.
        assert!(eb.eval(&p) < 0.0);
        assert!((eb.eval(&p) - (-radius * 0.25)).abs() < 1e-12);
    }

    #[test]
    fn chamfer_cuts_more_than_fillet_at_edge() {
        let radius = 0.4;
        let p = Point3::origin();
        let fillet = blend(
            BooleanKind::Union,
            BlendMode::Fillet,
            radius,
            z_axis_region(),
        )
        .eval(&p);
        let chamfer = blend(
            BooleanKind::Union,
            BlendMode::Chamfer,
            radius,
            z_axis_region(),
        )
        .eval(&p);
        // chamfer = -r·√½ ≈ -0.283r, deeper than the fillet's -0.25r.
        assert!((chamfer - (-radius * FRAC_1_SQRT_2)).abs() < 1e-12);
        assert!(chamfer < fillet);
    }

    #[test]
    fn intersection_fillet_removes_material_at_concave_edge() {
        // {x < 0} ∩ {y < 0}: a concave edge along z. A fillet rounds it
        // inward, removing material — the field rises above the sharp 0.
        let radius = 0.4;
        let eb = blend(
            BooleanKind::Intersection,
            BlendMode::Fillet,
            radius,
            z_axis_region(),
        );
        let p = Point3::origin();
        assert!(eb.eval(&p) > 0.0);
        assert!((eb.eval(&p) - radius * 0.25).abs() < 1e-12);
    }

    #[test]
    fn weight_is_one_on_edge_and_zero_past_influence() {
        let eb = blend(BooleanKind::Union, BlendMode::Fillet, 0.3, z_axis_region());
        assert_eq!(eb.weight(&Point3::new(0.0, 0.0, 1.0)), 1.0);
        // influence = 0.6; a point 1 unit off the axis is well past it.
        assert_eq!(eb.weight(&Point3::new(1.0, 0.0, 0.0)), 0.0);
        // Inside the taper band the weight is strictly between 0 and 1.
        let w = eb.weight(&Point3::new(0.45, 0.0, 0.0));
        assert!(w > 0.0 && w < 1.0);
    }

    #[test]
    fn branches_smooth_inside_sharp_outside() {
        let p = Point3::origin();
        let tol = 1e-6;

        // No region → sharp edge → the union reports both surfaces (a crease).
        let sharp = blend(
            BooleanKind::Union,
            BlendMode::Fillet,
            0.3,
            EdgeRegion::default(),
        );
        let mut out = Vec::new();
        sharp.branches(&p, tol, &mut out);
        assert_eq!(out.len(), 2, "sharp edge should expose two branches");

        // Region covering the edge → smooth blend → a single branch.
        let filleted = blend(BooleanKind::Union, BlendMode::Fillet, 0.3, z_axis_region());
        let mut out2 = Vec::new();
        filleted.branches(&p, tol, &mut out2);
        assert_eq!(
            out2.len(),
            1,
            "filleted edge should be a single smooth branch"
        );
    }

    #[test]
    fn fillet_interval_containment() {
        let eb = blend(BooleanKind::Union, BlendMode::Fillet, 0.3, z_axis_region());
        crate::test_util::assert_interval_containment(&eb, 51);
    }

    #[test]
    fn chamfer_interval_containment() {
        let eb = blend(BooleanKind::Union, BlendMode::Chamfer, 0.3, z_axis_region());
        crate::test_util::assert_interval_containment(&eb, 52);
    }

    #[test]
    fn intersection_fillet_interval_containment() {
        let eb = blend(
            BooleanKind::Intersection,
            BlendMode::Fillet,
            0.35,
            z_axis_region(),
        );
        crate::test_util::assert_interval_containment(&eb, 53);
    }

    #[test]
    fn subtraction_chamfer_interval_containment() {
        let eb = blend(
            BooleanKind::Subtraction,
            BlendMode::Chamfer,
            0.25,
            z_axis_region(),
        );
        crate::test_util::assert_interval_containment(&eb, 54);
    }
}
