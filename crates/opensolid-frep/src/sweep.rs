//! Sweep a constant profile along a 3D path, and loft between two profiles.
//!
//! Both build [`Sdf`] solids from [`Profile2D`] sections, so the existing
//! meshing pipeline consumes them unchanged.
//!
//! # [`Sweep`]
//!
//! A closed profile extruded along a polyline path. Each path segment becomes
//! a rigid extrusion of the profile through a frame perpendicular to that
//! segment, and the whole solid is the union (`min`) of those per-segment
//! prisms. The frame is fixed by a global up reference, so the profile keeps
//! a constant orientation along the path — *no twist* (the MVP contract).
//!
//! Every per-segment field is an exact prism SDF, so the union is 1-Lipschitz
//! and the default [`Sdf::eval_interval`] stays valid.
//!
//! MVP limitations: constant profile, no twist, and joints are mitred by
//! overlap — outer corners bulge slightly, sharp inner corners may notch. The
//! profile's local origin `(0, 0)` rides on the path.
//!
//! # [`Loft`]
//!
//! A blend between a `bottom` profile on the plane `y = 0` and a `top`
//! profile on `y = height`, formed by linearly interpolating their 2D signed
//! distances along `y` (a linear SDF morph), then capping the ends. It is
//! sign-correct everywhere. The morph is generally **not** 1-Lipschitz in `y`
//! (its `y`-gradient is `(top - bottom) / height`), so [`Sdf::eval_interval`]
//! is overridden with a conservative bound.
//!
//! MVP limitations: parallel planes only, linear morph — the intermediate
//! cross-sections are the zero set of the interpolated field, not a
//! corresponding-point sweep, so topology is not guaranteed when the two
//! profiles differ wildly.

use crate::primitives::Sdf;
use crate::profile::Profile2D;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};

/// Path segments shorter than this are rejected as degenerate.
const MIN_SEGMENT: f64 = 1e-9;

fn invalid(argument: &'static str, reason: String) -> CoreError {
    CoreError::InvalidArgument { argument, reason }
}

/// Exact prism SDF: a 2D profile distance `d` combined with a slab offset
/// `w` (`< 0` inside the slab). Shared by [`Sweep`]'s per-segment field and
/// [`Loft`]'s capped morph, and identical to the combination [`crate::Extrude`]
/// uses.
fn prism(d: f64, w: f64) -> f64 {
    d.max(w).min(0.0) + d.max(0.0).hypot(w.max(0.0))
}

/// One polyline segment, precomputed with an orthonormal frame whose
/// `tangent` runs along the segment and whose `u_axis`/`v_axis` span the
/// perpendicular plane the profile lives in.
#[derive(Clone, Debug)]
struct SweptSegment {
    origin: Point3,
    tangent: Vector3,
    u_axis: Vector3,
    v_axis: Vector3,
    length: f64,
}

/// A right-handed frame perpendicular to `tangent` (assumed unit). The up
/// reference switches to `+X` when the tangent is nearly vertical so the
/// cross product never degenerates; this fixes the profile orientation with
/// no torsional twist along the path.
fn frame(tangent: Vector3) -> (Vector3, Vector3) {
    let up = if tangent.y.abs() < 0.9 {
        Vector3::new(0.0, 1.0, 0.0)
    } else {
        Vector3::new(1.0, 0.0, 0.0)
    };
    let u_axis = up.cross(&tangent).normalize();
    let v_axis = tangent.cross(&u_axis);
    (u_axis, v_axis)
}

/// A closed profile swept along a polyline path (see [module docs](self)).
pub struct Sweep {
    profile: Profile2D,
    segments: Vec<SweptSegment>,
}

impl Sweep {
    /// Sweep `profile` along `path` (a polyline of at least two 3D points).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if fewer than two path points are
    /// given, any coordinate is not finite, or two consecutive points
    /// coincide (a zero-length segment has no tangent).
    pub fn new(profile: Profile2D, path: &[[f64; 3]]) -> CoreResult<Self> {
        if path.len() < 2 {
            return Err(invalid(
                "path",
                format!("sweep path needs at least 2 points, got {}", path.len()),
            ));
        }
        if path.iter().flatten().any(|c| !c.is_finite()) {
            return Err(invalid("path", "coordinates must be finite".into()));
        }
        let mut segments = Vec::with_capacity(path.len() - 1);
        for (i, w) in path.windows(2).enumerate() {
            let a = Point3::new(w[0][0], w[0][1], w[0][2]);
            let b = Point3::new(w[1][0], w[1][1], w[1][2]);
            let delta = b - a;
            let length = delta.norm();
            if length < MIN_SEGMENT {
                return Err(invalid(
                    "path",
                    format!("path segment {i} is degenerate: consecutive points coincide"),
                ));
            }
            let tangent = delta / length;
            let (u_axis, v_axis) = frame(tangent);
            segments.push(SweptSegment {
                origin: a,
                tangent,
                u_axis,
                v_axis,
                length,
            });
        }
        Ok(Self { profile, segments })
    }

    /// World-space corners of every per-segment prism (profile bounds box ×
    /// segment length), for a conservative axis-aligned bound of the sweep.
    pub fn corners(&self) -> Vec<Point3> {
        let (min, max) = self.profile.bounds();
        let mut pts = Vec::with_capacity(self.segments.len() * 8);
        for s in &self.segments {
            for &axial in &[0.0, s.length] {
                for &pu in &[min[0], max[0]] {
                    for &pv in &[min[1], max[1]] {
                        pts.push(s.origin + s.tangent * axial + s.u_axis * pu + s.v_axis * pv);
                    }
                }
            }
        }
        pts
    }
}

impl Sdf for Sweep {
    fn eval(&self, p: &Point3) -> f64 {
        let mut best = f64::INFINITY;
        for s in &self.segments {
            let rel = p - s.origin;
            let axial = rel.dot(&s.tangent);
            let pu = rel.dot(&s.u_axis);
            let pv = rel.dot(&s.v_axis);
            let d = self.profile.signed_distance(pu, pv);
            let half = 0.5 * s.length;
            let w = (axial - half).abs() - half;
            best = best.min(prism(d, w));
        }
        best
    }
}

/// A loft between two profiles on parallel planes (see [module docs](self)).
pub struct Loft {
    bottom: Profile2D,
    top: Profile2D,
    height: f64,
}

impl Loft {
    /// Loft from `bottom` (on `y = 0`) to `top` (on `y = height`).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `height` is not positive and finite.
    pub fn new(bottom: Profile2D, top: Profile2D, height: f64) -> CoreResult<Self> {
        if !(height.is_finite() && height > 0.0) {
            return Err(invalid(
                "height",
                format!("must be positive and finite, got {height}"),
            ));
        }
        Ok(Self {
            bottom,
            top,
            height,
        })
    }

    /// Linearly interpolated 2D signed distance at world `(x, z)` and height
    /// `y`: `bottom` at `y = 0`, `top` at `y = height`, clamped outside.
    fn morph(&self, x: f64, z: f64, y: f64) -> f64 {
        let t = (y / self.height).clamp(0.0, 1.0);
        let d0 = self.bottom.signed_distance(x, z);
        let d1 = self.top.signed_distance(x, z);
        (1.0 - t) * d0 + t * d1
    }
}

impl Sdf for Loft {
    fn eval(&self, p: &Point3) -> f64 {
        let d = self.morph(p.x, p.z, p.y);
        let half = 0.5 * self.height;
        let w = (p.y - half).abs() - half;
        prism(d, w)
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // The morphed 2D distance and the slab offset are each bounded
        // conservatively, then combined through `prism`, which is monotone
        // nondecreasing in both arguments — so the field's extremes over the
        // box are attained at the componentwise interval endpoints.
        //
        // Each profile distance is 1-Lipschitz in (x, z), so over the box's
        // xz-rectangle it lies within its center value ± the half-diagonal.
        let cx = 0.5 * (b.min.x + b.max.x);
        let cz = 0.5 * (b.min.z + b.max.z);
        let rxz = 0.5 * (b.max.x - b.min.x).hypot(b.max.z - b.min.z);
        let s0 = {
            let c = self.bottom.signed_distance(cx, cz);
            Interval::new(c - rxz, c + rxz)
        };
        let s1 = {
            let c = self.top.signed_distance(cx, cz);
            Interval::new(c - rxz, c + rxz)
        };
        let t = Interval::new(
            (b.min.y / self.height).clamp(0.0, 1.0),
            (b.max.y / self.height).clamp(0.0, 1.0),
        );
        let d2d = (Interval::point(1.0) - t) * s0 + t * s1;
        let half = 0.5 * self.height;
        let w = Interval::new(b.min.y - half, b.max.y - half).abs() - Interval::point(half);
        Interval::new(prism(d2d.lo, w.lo), prism(d2d.hi, w.hi))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{MeshOptions, mesh_sdf_indexed};
    use crate::primitives::Cylinder;
    use crate::profile::Extrude;
    use crate::transform::SdfTransformExt;

    fn unit_square() -> Profile2D {
        Profile2D::new(
            vec![[-0.5, -0.5], [0.5, -0.5], [0.5, 0.5], [-0.5, 0.5]],
            vec![0.0; 4],
        )
        .expect("valid square")
    }

    /// Two bulge-1 arcs on a horizontal diameter form the full circle of
    /// radius `r` centered at the origin.
    fn circle(r: f64) -> Profile2D {
        Profile2D::new(vec![[-r, 0.0], [r, 0.0]], vec![1.0, 1.0]).expect("valid circle")
    }

    #[test]
    fn straight_sweep_of_circle_matches_cylinder() {
        // A radius-0.5 circle swept straight up +Y over [0, 2] is the
        // cylinder spanning y ∈ [0, 2]. The circle is rotationally
        // symmetric, so the profile-plane orientation is irrelevant and the
        // two exact fields must agree to machine precision.
        let s = Sweep::new(circle(0.5), &[[0.0, 0.0, 0.0], [0.0, 2.0, 0.0]]).expect("valid sweep");
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
                (s.eval(&p) - c.eval(&p)).abs() < 1e-12,
                "at {p:?}: {} vs {}",
                s.eval(&p),
                c.eval(&p)
            );
        }
    }

    #[test]
    fn sweep_along_x_matches_translated_cylinder() {
        // Same circle swept along +X: a cylinder lying on the x axis. Check a
        // few known distances directly.
        let s = Sweep::new(circle(0.5), &[[0.0, 0.0, 0.0], [2.0, 0.0, 0.0]]).expect("valid sweep");
        // On the axis, midway: nearest surface is the circular wall at 0.5.
        assert!((s.eval(&Point3::new(1.0, 0.0, 0.0)) + 0.5).abs() < 1e-12);
        // Just past the far cap along the axis.
        assert!((s.eval(&Point3::new(2.5, 0.0, 0.0)) - 0.5).abs() < 1e-12);
        // Radially outside the wall.
        assert!((s.eval(&Point3::new(1.0, 0.8, 0.0)) - 0.3).abs() < 1e-12);
    }

    #[test]
    fn bent_sweep_meshes_watertight() {
        // An L-shaped path: up then across. The union of the two oriented
        // prisms must mesh to a closed manifold.
        let s = Sweep::new(
            circle(0.25),
            &[[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 1.0, 0.0]],
        )
        .expect("valid sweep");
        let mesh = mesh_sdf_indexed(
            &s,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-0.6, -0.6, -0.6),
                    Point3::new(1.6, 1.6, 0.6),
                ),
                resolution: 48,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn sweep_rejects_bad_path() {
        assert!(Sweep::new(unit_square(), &[[0.0, 0.0, 0.0]]).is_err());
        assert!(
            Sweep::new(unit_square(), &[[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]]).is_err(),
            "coincident points have no tangent"
        );
        assert!(
            Sweep::new(unit_square(), &[[0.0, 0.0, 0.0], [f64::NAN, 1.0, 0.0]]).is_err(),
            "non-finite coordinate"
        );
    }

    #[test]
    fn sweep_interval_containment() {
        let s = Sweep::new(
            circle(0.3),
            &[[0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.8, 1.0, 0.2]],
        )
        .expect("valid sweep");
        crate::test_util::assert_interval_containment(&s, 71);
    }

    #[test]
    fn loft_of_equal_profiles_is_an_extrusion() {
        // Lofting a profile to an identical copy is a plain extrusion: the
        // linear morph of a field with itself is the field, independent of
        // the height parameter, so it must equal Extrude to machine
        // precision at every point.
        let l = Loft::new(unit_square(), unit_square(), 1.5).expect("valid loft");
        let e = Extrude::new(unit_square(), 1.5).expect("valid extrude");
        let mut probe = crate::test_util::Lcg(9);
        for _ in 0..60 {
            let p = Point3::new(
                probe.in_range(-1.5, 1.5),
                probe.in_range(-0.5, 2.0),
                probe.in_range(-1.5, 1.5),
            );
            assert!(
                (l.eval(&p) - e.eval(&p)).abs() < 1e-12,
                "at {p:?}: {} vs {}",
                l.eval(&p),
                e.eval(&p)
            );
        }
    }

    #[test]
    fn loft_cross_section_interpolates() {
        // Loft a small square (half-extent 0.25) up to a large one (0.75)
        // over height 1. At mid-height the interpolated profile half-extent
        // is 0.5, so the wall along +x sits at x = 0.5 on the y = 0.5 plane.
        let small = Profile2D::new(
            vec![[-0.25, -0.25], [0.25, -0.25], [0.25, 0.25], [-0.25, 0.25]],
            vec![0.0; 4],
        )
        .expect("valid");
        let large = Profile2D::new(
            vec![[-0.75, -0.75], [0.75, -0.75], [0.75, 0.75], [-0.75, 0.75]],
            vec![0.0; 4],
        )
        .expect("valid");
        let l = Loft::new(small, large, 1.0).expect("valid loft");
        // On the mid plane the +x wall is the zero crossing at x = 0.5.
        assert!(l.eval(&Point3::new(0.5, 0.5, 0.0)).abs() < 1e-12);
        assert!(l.eval(&Point3::new(0.4, 0.5, 0.0)) < 0.0);
        assert!(l.eval(&Point3::new(0.6, 0.5, 0.0)) > 0.0);
        // At the bottom the wall is at the small profile (0.25).
        assert!(l.eval(&Point3::new(0.25, 0.0, 0.0)).abs() < 1e-12);
        // At the top the wall is at the large profile (0.75).
        assert!(l.eval(&Point3::new(0.75, 1.0, 0.0)).abs() < 1e-12);
    }

    #[test]
    fn loft_meshes_watertight() {
        let small = Profile2D::new(
            vec![[-0.3, -0.3], [0.3, -0.3], [0.3, 0.3], [-0.3, 0.3]],
            vec![0.0; 4],
        )
        .expect("valid");
        let l = Loft::new(small, circle(0.6), 1.0).expect("valid loft");
        let mesh = mesh_sdf_indexed(
            &l,
            &MeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-0.9, -0.3, -0.9),
                    Point3::new(0.9, 1.3, 0.9),
                ),
                resolution: 40,
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn loft_rejects_bad_height() {
        for h in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(
                Loft::new(unit_square(), unit_square(), h).is_err(),
                "height {h}"
            );
        }
    }

    #[test]
    fn loft_interval_containment() {
        // Distinct profiles over a modest height give a steep y-gradient —
        // the exact regime the eval_interval override must cover.
        let small = Profile2D::new(
            vec![[-0.2, -0.2], [0.2, -0.2], [0.2, 0.2], [-0.2, 0.2]],
            vec![0.0; 4],
        )
        .expect("valid");
        let l = Loft::new(small, circle(0.9), 0.5).expect("valid loft");
        crate::test_util::assert_interval_containment(&l, 72);
    }
}
