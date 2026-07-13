use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};

const GRADIENT_EPS: f64 = 1e-6;

pub trait Sdf: Send + Sync {
    fn eval(&self, p: &Point3) -> f64;

    /// Gradient of the distance field. The default uses central finite
    /// differences; implementations with a cheap closed form override it.
    /// On non-smooth loci (edges, branch switches) any subgradient may be
    /// returned.
    fn grad(&self, p: &Point3) -> Vector3 {
        let dx = self.eval(&Point3::new(p.x + GRADIENT_EPS, p.y, p.z))
            - self.eval(&Point3::new(p.x - GRADIENT_EPS, p.y, p.z));
        let dy = self.eval(&Point3::new(p.x, p.y + GRADIENT_EPS, p.z))
            - self.eval(&Point3::new(p.x, p.y - GRADIENT_EPS, p.z));
        let dz = self.eval(&Point3::new(p.x, p.y, p.z + GRADIENT_EPS))
            - self.eval(&Point3::new(p.x, p.y, p.z - GRADIENT_EPS));
        Vector3::new(dx, dy, dz) / (2.0 * GRADIENT_EPS)
    }

    /// Conservative range of the field over a non-empty axis-aligned box:
    /// the result contains `eval(p)` for every `p` in `b`. Octree meshing
    /// uses this to prune cells that provably do not cross the surface
    /// (`lo > 0` or `hi < 0`).
    ///
    /// The default bounds the field by its value at the box center plus or
    /// minus the half-diagonal, which is valid **only for true signed
    /// distance fields** (Lipschitz constant ≤ 1). Fields that violate the
    /// metric property must override this with their own conservative bound
    /// — the default would under-cover and pruning would cut through the
    /// surface.
    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        let d = self.eval(&b.center());
        let r = 0.5 * b.extents().norm();
        Interval::new(d - r, d + r)
    }

    /// The smooth field branches active at `p`: `(value, gradient)` of every
    /// leaf field that wins — or could win, within `tol` — the `min`/`max`
    /// combinators between it and the root. A sharp CSG feature (edge,
    /// corner) is exactly the locus where several branches are zero at once,
    /// so meshing uses the active branches to project vertices onto the
    /// *analytic* intersection of the adjoining surfaces instead of the
    /// kinked composite field.
    ///
    /// The default treats the whole field as one smooth branch, which is
    /// correct for primitives and smooth blends. Sharp CSG combinators
    /// recurse into every child within `tol` of winning (negating through
    /// subtraction); wrappers forward, mapping points, values, and gradients
    /// exactly as their `eval`/`grad` do.
    fn branches(&self, p: &Point3, _tol: f64, out: &mut Vec<(f64, Vector3)>) {
        out.push((self.eval(p), self.grad(p)));
    }
}

impl<T: Sdf + ?Sized> Sdf for &T {
    fn eval(&self, p: &Point3) -> f64 {
        (**self).eval(p)
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        (**self).grad(p)
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        (**self).eval_interval(b)
    }

    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        (**self).branches(p, tol, out)
    }
}

impl<T: Sdf + ?Sized> Sdf for Box<T> {
    fn eval(&self, p: &Point3) -> f64 {
        (**self).eval(p)
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        (**self).grad(p)
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        (**self).eval_interval(b)
    }

    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        (**self).branches(p, tol, out)
    }
}

impl<T: Sdf + ?Sized> Sdf for std::sync::Arc<T> {
    fn eval(&self, p: &Point3) -> f64 {
        (**self).eval(p)
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        (**self).grad(p)
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        (**self).eval_interval(b)
    }

    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        (**self).branches(p, tol, out)
    }
}

/// Interval of `x - c` as `x` ranges over `[lo, hi]`.
fn offset(lo: f64, hi: f64, c: f64) -> Interval {
    Interval::new(lo - c, hi - c)
}

/// `sqrt` of an interval known to be non-negative.
fn sqrt_nonneg(x: Interval) -> Interval {
    Interval::new(x.lo.sqrt(), x.hi.sqrt())
}

fn clamp01(t: Interval) -> Interval {
    Interval::new(t.lo.clamp(0.0, 1.0), t.hi.clamp(0.0, 1.0))
}

/// Range of the Euclidean distance from `c` to points of `b`. Exact: each
/// axis contributes independently, so the nearest and farthest points are
/// assembled per-axis.
fn point_dist(b: &BoundingBox3, c: &Point3) -> Interval {
    let dx = offset(b.min.x, b.max.x, c.x).abs();
    let dy = offset(b.min.y, b.max.y, c.y).abs();
    let dz = offset(b.min.z, b.max.z, c.z).abs();
    sqrt_nonneg(dx.square() + dy.square() + dz.square())
}

/// Range of the xz-plane distance from the vertical axis through
/// `(cx, cz)` over `b`. Exact, same argument as [`point_dist`].
fn radial_dist(b: &BoundingBox3, cx: f64, cz: f64) -> Interval {
    let dx = offset(b.min.x, b.max.x, cx).abs();
    let dz = offset(b.min.z, b.max.z, cz).abs();
    sqrt_nonneg(dx.square() + dz.square())
}

pub struct Sphere {
    pub center: Point3,
    pub radius: f64,
}

impl Sdf for Sphere {
    fn eval(&self, p: &Point3) -> f64 {
        (p - self.center).norm() - self.radius
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        let v = p - self.center;
        let n = v.norm();
        if n == 0.0 { Vector3::zeros() } else { v / n }
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Exact: the range of |p - center| over the box, shifted by -radius.
        let d = point_dist(b, &self.center);
        Interval::new(d.lo - self.radius, d.hi - self.radius)
    }
}

pub struct Box3 {
    pub center: Point3,
    pub half_extents: [f64; 3],
}

/// Box SDF as a function of the per-axis offsets `d_i = |p_i - c_i| - h_i`.
/// Nondecreasing in each component — the basis for exact interval bounds.
fn box_distance(d: [f64; 3]) -> f64 {
    let outside = (d[0].max(0.0).powi(2) + d[1].max(0.0).powi(2) + d[2].max(0.0).powi(2)).sqrt();
    let inside = d[0].max(d[1]).max(d[2]).min(0.0);
    outside + inside
}

/// Per-axis offset intervals `|p_i - c_i| - h_i` over the box. Exact.
fn box_offsets(b: &BoundingBox3, center: &Point3, half_extents: &[f64; 3]) -> [Interval; 3] {
    [
        offset(b.min.x, b.max.x, center.x).abs() - Interval::point(half_extents[0]),
        offset(b.min.y, b.max.y, center.y).abs() - Interval::point(half_extents[1]),
        offset(b.min.z, b.max.z, center.z).abs() - Interval::point(half_extents[2]),
    ]
}

impl Sdf for Box3 {
    fn eval(&self, p: &Point3) -> f64 {
        box_distance([
            (p.x - self.center.x).abs() - self.half_extents[0],
            (p.y - self.center.y).abs() - self.half_extents[1],
            (p.z - self.center.z).abs() - self.half_extents[2],
        ])
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        let q = p - self.center;
        let d = [
            q.x.abs() - self.half_extents[0],
            q.y.abs() - self.half_extents[1],
            q.z.abs() - self.half_extents[2],
        ];
        let s = [q.x.signum(), q.y.signum(), q.z.signum()];
        let outward = Vector3::new(
            d[0].max(0.0) * s[0],
            d[1].max(0.0) * s[1],
            d[2].max(0.0) * s[2],
        );
        let n = outward.norm();
        if n > 0.0 {
            return outward / n;
        }
        // Inside: distance changes along the least-deep axis only.
        let mut axis = 0;
        for i in 1..3 {
            if d[i] > d[axis] {
                axis = i;
            }
        }
        let mut g = Vector3::zeros();
        g[axis] = s[axis];
        g
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // box_distance is nondecreasing in each component and the components
        // range independently (one per axis), so the extremes of the exact
        // image are attained at the componentwise endpoints.
        let d = box_offsets(b, &self.center, &self.half_extents);
        Interval::new(
            box_distance([d[0].lo, d[1].lo, d[2].lo]),
            box_distance([d[0].hi, d[1].hi, d[2].hi]),
        )
    }
}

pub struct Cylinder {
    pub center: Point3,
    pub radius: f64,
    pub half_height: f64,
}

/// Cylinder SDF as a function of the radial and axial offsets.
/// Nondecreasing in both — the basis for exact interval bounds.
fn cylinder_distance(radial: f64, axial: f64) -> f64 {
    let outside = radial.max(0.0).hypot(axial.max(0.0));
    let inside = radial.max(axial).min(0.0);
    outside + inside
}

impl Sdf for Cylinder {
    fn eval(&self, p: &Point3) -> f64 {
        let dx = p.x - self.center.x;
        let dz = p.z - self.center.z;
        let radial = (dx * dx + dz * dz).sqrt() - self.radius;
        let axial = (p.y - self.center.y).abs() - self.half_height;
        cylinder_distance(radial, axial)
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        let dx = p.x - self.center.x;
        let dy = p.y - self.center.y;
        let dz = p.z - self.center.z;
        let rho = dx.hypot(dz);
        let radial = rho - self.radius;
        let axial = dy.abs() - self.half_height;
        let radial_dir = if rho > 0.0 {
            Vector3::new(dx / rho, 0.0, dz / rho)
        } else {
            Vector3::zeros()
        };
        let ro = radial.max(0.0);
        let ao = axial.max(0.0);
        let outside = ro.hypot(ao);
        if outside > 0.0 {
            (radial_dir * ro + Vector3::new(0.0, dy.signum() * ao, 0.0)) / outside
        } else if radial > axial {
            radial_dir
        } else {
            Vector3::new(0.0, dy.signum(), 0.0)
        }
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Exact: cylinder_distance is nondecreasing in both offsets, and
        // radial (x, z) and axial (y) range independently.
        let radial = radial_dist(b, self.center.x, self.center.z) - Interval::point(self.radius);
        let axial =
            offset(b.min.y, b.max.y, self.center.y).abs() - Interval::point(self.half_height);
        Interval::new(
            cylinder_distance(radial.lo, axial.lo),
            cylinder_distance(radial.hi, axial.hi),
        )
    }
}

pub struct Torus {
    pub center: Point3,
    pub major_radius: f64,
    pub minor_radius: f64,
}

impl Sdf for Torus {
    fn eval(&self, p: &Point3) -> f64 {
        let dx = p.x - self.center.x;
        let dy = p.y - self.center.y;
        let dz = p.z - self.center.z;
        let ring = dx.hypot(dz) - self.major_radius;
        ring.hypot(dy) - self.minor_radius
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Exact: the ring offset (x, z) and the vertical offset (y) range
        // independently, and their exact ranges are connected, so squaring,
        // summing, and the monotone sqrt preserve exactness.
        let ring =
            radial_dist(b, self.center.x, self.center.z) - Interval::point(self.major_radius);
        let dy = offset(b.min.y, b.max.y, self.center.y).abs();
        let d = sqrt_nonneg(ring.square() + dy.square());
        Interval::new(d.lo - self.minor_radius, d.hi - self.minor_radius)
    }
}

pub struct Cone {
    pub center: Point3,
    pub half_height: f64,
    pub radius_bottom: f64,
    pub radius_top: f64,
}

// Cone keeps the default `eval_interval`: its `eval` is an exact distance
// (Lipschitz ≤ 1), so the center-plus-half-diagonal bound is valid, and the
// closed form is not monotone in any convenient decomposition, so an
// analytic interval version would not be exact anyway.
impl Sdf for Cone {
    fn eval(&self, p: &Point3) -> f64 {
        let h = self.half_height;
        let r1 = self.radius_bottom;
        let r2 = self.radius_top;
        let qx = (p.x - self.center.x).hypot(p.z - self.center.z);
        let qy = p.y - self.center.y;
        let cap_radius = if qy < 0.0 { r1 } else { r2 };
        let ca = (qx - qx.min(cap_radius), qy.abs() - h);
        let k1 = (r2, h);
        let k2 = (r2 - r1, 2.0 * h);
        let t = (((k1.0 - qx) * k2.0 + (k1.1 - qy) * k2.1) / (k2.0 * k2.0 + k2.1 * k2.1))
            .clamp(0.0, 1.0);
        let cb = (qx - k1.0 + k2.0 * t, qy - k1.1 + k2.1 * t);
        let sign = if cb.0 < 0.0 && ca.1 < 0.0 { -1.0 } else { 1.0 };
        let d2 = (ca.0 * ca.0 + ca.1 * ca.1).min(cb.0 * cb.0 + cb.1 * cb.1);
        sign * d2.sqrt()
    }
}

pub struct Capsule {
    pub start: Point3,
    pub end: Point3,
    pub radius: f64,
}

impl Sdf for Capsule {
    fn eval(&self, p: &Point3) -> f64 {
        let pa = p - self.start;
        let ba = self.end - self.start;
        let t = (pa.dot(&ba) / ba.dot(&ba)).clamp(0.0, 1.0);
        (pa - ba * t).norm() - self.radius
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        let pa = p - self.start;
        let ba = self.end - self.start;
        let t = (pa.dot(&ba) / ba.dot(&ba)).clamp(0.0, 1.0);
        let v = pa - ba * t;
        let n = v.norm();
        if n == 0.0 { Vector3::zeros() } else { v / n }
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Conservative, not exact: `t` and `pa` are correlated, and interval
        // propagation treats them as independent. Still contains the true
        // range, and is far tighter than the Lipschitz default for boxes
        // near the segment. Degenerate capsules (start == end) are as
        // undefined here as in `eval`.
        let pa = [
            offset(b.min.x, b.max.x, self.start.x),
            offset(b.min.y, b.max.y, self.start.y),
            offset(b.min.z, b.max.z, self.start.z),
        ];
        let ba = self.end - self.start;
        let dot = pa[0] * Interval::point(ba.x)
            + pa[1] * Interval::point(ba.y)
            + pa[2] * Interval::point(ba.z);
        let t = clamp01(dot * Interval::point(1.0 / ba.dot(&ba)));
        let vx = pa[0] - Interval::point(ba.x) * t;
        let vy = pa[1] - Interval::point(ba.y) * t;
        let vz = pa[2] - Interval::point(ba.z) * t;
        let d = sqrt_nonneg(vx.square() + vy.square() + vz.square());
        Interval::new(d.lo - self.radius, d.hi - self.radius)
    }
}

pub struct HalfSpace {
    pub normal: Vector3,
    pub offset: f64,
}

impl Sdf for HalfSpace {
    fn eval(&self, p: &Point3) -> f64 {
        (self.normal.dot(&p.coords) - self.offset) / self.normal.norm()
    }

    fn grad(&self, _p: &Point3) -> Vector3 {
        self.normal / self.normal.norm()
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Exact: the field is linear, so each axis contributes an
        // independent interval and the extremes sit at box corners.
        let inv = 1.0 / self.normal.norm();
        let n = self.normal * inv;
        let cx = Interval::from_unordered(n.x * b.min.x, n.x * b.max.x);
        let cy = Interval::from_unordered(n.y * b.min.y, n.y * b.max.y);
        let cz = Interval::from_unordered(n.z * b.min.z, n.z * b.max.z);
        cx + cy + cz - Interval::point(self.offset * inv)
    }
}

/// `half_extents` are the outer extents including the rounding, so
/// `radius` must not exceed the smallest half extent. With `radius == 0`
/// this is identical to [`Box3`].
pub struct RoundedBox {
    pub center: Point3,
    pub half_extents: [f64; 3],
    pub radius: f64,
}

impl Sdf for RoundedBox {
    fn eval(&self, p: &Point3) -> f64 {
        let d = [
            (p.x - self.center.x).abs() - (self.half_extents[0] - self.radius),
            (p.y - self.center.y).abs() - (self.half_extents[1] - self.radius),
            (p.z - self.center.z).abs() - (self.half_extents[2] - self.radius),
        ];
        let outside =
            (d[0].max(0.0).powi(2) + d[1].max(0.0).powi(2) + d[2].max(0.0).powi(2)).sqrt();
        let inside = d[0].max(d[1]).max(d[2]).min(0.0);
        outside + inside - self.radius
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Exact: a rounded box is the box with shrunken extents, offset
        // outward by `radius` — the same monotone argument as [`Box3`].
        let inner = [
            self.half_extents[0] - self.radius,
            self.half_extents[1] - self.radius,
            self.half_extents[2] - self.radius,
        ];
        let d = box_offsets(b, &self.center, &inner);
        Interval::new(
            box_distance([d[0].lo, d[1].lo, d[2].lo]) - self.radius,
            box_distance([d[0].hi, d[1].hi, d[2].hi]) - self.radius,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eval::gradient;

    fn assert_unit_gradient(sdf: &dyn Sdf, p: &Point3) {
        let g = gradient(sdf, p).norm();
        assert!((g - 1.0).abs() < 1e-4, "gradient norm {g} at {p:?}");
    }

    #[test]
    fn sphere_inside_outside() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        assert!(s.eval(&Point3::origin()) < 0.0);
        assert!((s.eval(&Point3::new(1.0, 0.0, 0.0))).abs() < 1e-10);
        assert!(s.eval(&Point3::new(2.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn box_inside_outside() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 1.0, 1.0],
        };
        assert!(b.eval(&Point3::origin()) < 0.0);
        assert!((b.eval(&Point3::new(1.0, 0.0, 0.0))).abs() < 1e-10);
        assert!(b.eval(&Point3::new(2.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn torus_inside_surface_outside() {
        let t = Torus {
            center: Point3::origin(),
            major_radius: 2.0,
            minor_radius: 0.5,
        };
        assert!((t.eval(&Point3::new(2.0, 0.0, 0.0)) + 0.5).abs() < 1e-10);
        assert!((t.eval(&Point3::new(2.5, 0.0, 0.0))).abs() < 1e-10);
        assert!((t.eval(&Point3::new(0.0, 0.5, 2.0))).abs() < 1e-10);
        assert!(t.eval(&Point3::new(3.5, 0.0, 0.0)) > 0.0);
        assert!(t.eval(&Point3::origin()) > 0.0);
    }

    #[test]
    fn torus_gradient_norm() {
        let t = Torus {
            center: Point3::origin(),
            major_radius: 2.0,
            minor_radius: 0.5,
        };
        assert_unit_gradient(&t, &Point3::new(2.4, 0.2, 0.3));
        assert_unit_gradient(&t, &Point3::new(-1.6, -0.1, 0.9));
    }

    #[test]
    fn cone_inside_surface_outside() {
        let c = Cone {
            center: Point3::origin(),
            half_height: 1.0,
            radius_bottom: 1.0,
            radius_top: 0.5,
        };
        assert!(c.eval(&Point3::origin()) < 0.0);
        // Lateral surface: radius interpolates linearly, 0.75 at mid-height.
        assert!((c.eval(&Point3::new(0.75, 0.0, 0.0))).abs() < 1e-9);
        // Caps and rims.
        assert!((c.eval(&Point3::new(0.0, -1.0, 0.0))).abs() < 1e-9);
        assert!((c.eval(&Point3::new(0.0, 1.0, 0.0))).abs() < 1e-9);
        assert!((c.eval(&Point3::new(1.0, -1.0, 0.0))).abs() < 1e-9);
        assert!(c.eval(&Point3::new(2.0, 0.0, 0.0)) > 0.0);
        assert!(c.eval(&Point3::new(0.0, 2.0, 0.0)) > 0.0);
        // Above the top cap: distance is the axial gap.
        assert!((c.eval(&Point3::new(0.0, 2.0, 0.0)) - 1.0).abs() < 1e-10);
    }

    #[test]
    fn cone_degenerates_to_point_tip() {
        let c = Cone {
            center: Point3::origin(),
            half_height: 1.0,
            radius_bottom: 1.0,
            radius_top: 0.0,
        };
        assert!((c.eval(&Point3::new(0.0, 1.0, 0.0))).abs() < 1e-9);
        assert!(c.eval(&Point3::new(0.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn cone_gradient_norm() {
        let c = Cone {
            center: Point3::origin(),
            half_height: 1.0,
            radius_bottom: 1.0,
            radius_top: 0.5,
        };
        assert_unit_gradient(&c, &Point3::new(0.9, 0.1, 0.2));
        assert_unit_gradient(&c, &Point3::new(0.1, -1.3, 0.2));
    }

    #[test]
    fn capsule_inside_surface_outside() {
        let c = Capsule {
            start: Point3::new(0.0, -1.0, 0.0),
            end: Point3::new(0.0, 1.0, 0.0),
            radius: 0.5,
        };
        assert!((c.eval(&Point3::origin()) + 0.5).abs() < 1e-10);
        assert!((c.eval(&Point3::new(0.5, 0.0, 0.0))).abs() < 1e-10);
        // Spherical end cap.
        assert!((c.eval(&Point3::new(0.0, 1.5, 0.0))).abs() < 1e-10);
        assert!(c.eval(&Point3::new(2.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn capsule_gradient_norm() {
        let c = Capsule {
            start: Point3::new(0.0, -1.0, 0.0),
            end: Point3::new(0.0, 1.0, 0.0),
            radius: 0.5,
        };
        assert_unit_gradient(&c, &Point3::new(0.55, 0.3, 0.1));
        assert_unit_gradient(&c, &Point3::new(0.2, 1.4, 0.3));
    }

    #[test]
    fn half_space_inside_surface_outside() {
        let h = HalfSpace {
            normal: Vector3::new(0.0, 0.0, 1.0),
            offset: 0.0,
        };
        assert!(h.eval(&Point3::new(3.0, -2.0, -1.0)) < 0.0);
        assert!((h.eval(&Point3::new(5.0, 7.0, 0.0))).abs() < 1e-10);
        assert!((h.eval(&Point3::new(0.0, 0.0, 2.5)) - 2.5).abs() < 1e-10);
    }

    #[test]
    fn half_space_non_unit_normal_is_still_a_distance() {
        let h = HalfSpace {
            normal: Vector3::new(0.0, 0.0, 2.0),
            offset: 2.0,
        };
        // Plane z = 1; distance must be metric despite the non-unit normal.
        assert!((h.eval(&Point3::new(0.0, 0.0, 3.0)) - 2.0).abs() < 1e-10);
        assert!((h.eval(&Point3::new(4.0, 4.0, 1.0))).abs() < 1e-10);
    }

    #[test]
    fn half_space_gradient_norm() {
        let h = HalfSpace {
            normal: Vector3::new(1.0, 2.0, -2.0),
            offset: 0.5,
        };
        assert_unit_gradient(&h, &Point3::new(0.3, -0.7, 1.1));
    }

    #[test]
    fn rounded_box_inside_surface_outside() {
        let b = RoundedBox {
            center: Point3::origin(),
            half_extents: [1.0, 1.0, 1.0],
            radius: 0.2,
        };
        assert!((b.eval(&Point3::origin()) + 1.0).abs() < 1e-10);
        // Face centers lie exactly on the outer extent.
        assert!((b.eval(&Point3::new(1.0, 0.0, 0.0))).abs() < 1e-10);
        // Corners are rounded: the sharp corner point is outside.
        let corner_gap = 0.2 * (3.0_f64.sqrt() - 1.0);
        assert!((b.eval(&Point3::new(1.0, 1.0, 1.0)) - corner_gap).abs() < 1e-10);
        assert!(b.eval(&Point3::new(2.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn rounded_box_zero_radius_matches_box() {
        let rb = RoundedBox {
            center: Point3::origin(),
            half_extents: [1.0, 0.5, 2.0],
            radius: 0.0,
        };
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 0.5, 2.0],
        };
        for p in [
            Point3::new(0.3, 0.1, -0.4),
            Point3::new(1.5, 0.7, 0.0),
            Point3::new(-2.0, 1.0, 3.0),
        ] {
            assert!((rb.eval(&p) - b.eval(&p)).abs() < 1e-12);
        }
    }

    #[test]
    fn rounded_box_gradient_norm() {
        let b = RoundedBox {
            center: Point3::origin(),
            half_extents: [1.0, 1.0, 1.0],
            radius: 0.2,
        };
        assert_unit_gradient(&b, &Point3::new(1.05, 0.2, 0.1));
        // Near the rounded corner the field is smooth and metric.
        assert_unit_gradient(&b, &Point3::new(1.0, 1.0, 1.0));
    }

    mod interval {
        use super::*;
        use crate::test_util::assert_interval_containment;
        use opensolid_core::types::BoundingBox3;

        // Off-center, anisotropic parameters so the tests do not pass by
        // symmetry accident.

        #[test]
        fn sphere_containment() {
            let s = Sphere {
                center: Point3::new(0.3, -0.2, 0.5),
                radius: 1.1,
            };
            assert_interval_containment(&s, 1);
        }

        #[test]
        fn box_containment() {
            let b = Box3 {
                center: Point3::new(-0.4, 0.1, 0.2),
                half_extents: [1.2, 0.5, 0.8],
            };
            assert_interval_containment(&b, 2);
        }

        #[test]
        fn cylinder_containment() {
            let c = Cylinder {
                center: Point3::new(0.2, -0.3, 0.1),
                radius: 0.7,
                half_height: 1.1,
            };
            assert_interval_containment(&c, 3);
        }

        #[test]
        fn torus_containment() {
            let t = Torus {
                center: Point3::new(0.1, 0.2, -0.3),
                major_radius: 1.5,
                minor_radius: 0.4,
            };
            assert_interval_containment(&t, 4);
        }

        // Cone has no override; this exercises the Lipschitz default on an
        // exact SDF.
        #[test]
        fn cone_default_containment() {
            let c = Cone {
                center: Point3::new(-0.2, 0.3, 0.1),
                half_height: 0.9,
                radius_bottom: 1.0,
                radius_top: 0.3,
            };
            assert_interval_containment(&c, 5);
        }

        #[test]
        fn capsule_containment() {
            let c = Capsule {
                start: Point3::new(-0.8, -0.5, 0.2),
                end: Point3::new(0.7, 0.9, -0.3),
                radius: 0.4,
            };
            assert_interval_containment(&c, 6);
        }

        #[test]
        fn half_space_containment() {
            let h = HalfSpace {
                normal: Vector3::new(1.0, -2.0, 0.5),
                offset: 0.3,
            };
            assert_interval_containment(&h, 7);
        }

        #[test]
        fn rounded_box_containment() {
            let b = RoundedBox {
                center: Point3::new(0.1, -0.2, 0.3),
                half_extents: [1.0, 0.6, 0.8],
                radius: 0.25,
            };
            assert_interval_containment(&b, 8);
        }

        #[test]
        fn sphere_interval_is_exact() {
            let s = Sphere {
                center: Point3::origin(),
                radius: 1.0,
            };
            // Box [1,2]^3: nearest corner (1,1,1), farthest (2,2,2).
            let b = BoundingBox3::new(Point3::new(1.0, 1.0, 1.0), Point3::new(2.0, 2.0, 2.0));
            let i = s.eval_interval(&b);
            assert!((i.lo - (3.0_f64.sqrt() - 1.0)).abs() < 1e-12);
            assert!((i.hi - (2.0 * 3.0_f64.sqrt() - 1.0)).abs() < 1e-12);
        }

        #[test]
        fn half_space_interval_is_exact() {
            let h = HalfSpace {
                normal: Vector3::new(0.0, 0.0, 2.0),
                offset: 2.0, // plane z = 1
            };
            let b = BoundingBox3::new(Point3::new(-5.0, 0.0, 0.0), Point3::new(5.0, 1.0, 4.0));
            let i = h.eval_interval(&b);
            assert!((i.lo - (-1.0)).abs() < 1e-12);
            assert!((i.hi - 3.0).abs() < 1e-12);
        }

        // The property the octree consumer relies on: cells provably inside
        // or outside can be pruned.
        #[test]
        fn interval_signs_allow_pruning() {
            let s = Sphere {
                center: Point3::origin(),
                radius: 1.0,
            };
            let inside =
                BoundingBox3::new(Point3::new(-0.2, -0.2, -0.2), Point3::new(0.2, 0.2, 0.2));
            assert!(s.eval_interval(&inside).hi < 0.0);
            let outside = BoundingBox3::new(Point3::new(2.0, 2.0, 2.0), Point3::new(3.0, 3.0, 3.0));
            assert!(s.eval_interval(&outside).lo > 0.0);
            let straddling =
                BoundingBox3::new(Point3::new(0.5, -0.5, -0.5), Point3::new(1.5, 0.5, 0.5));
            let i = s.eval_interval(&straddling);
            assert!(i.lo < 0.0 && i.hi > 0.0);
        }

        // Forwarding impls must not fall back to the trait default.
        #[test]
        fn reference_box_and_arc_forward_eval_interval() {
            let s = Sphere {
                center: Point3::origin(),
                radius: 1.0,
            };
            let b = BoundingBox3::new(Point3::new(1.0, 1.0, 1.0), Point3::new(2.0, 2.0, 2.0));
            let expected = s.eval_interval(&b);
            assert_eq!((&s as &dyn Sdf).eval_interval(&b), expected);
            let boxed: Box<dyn Sdf> = Box::new(Sphere {
                center: Point3::origin(),
                radius: 1.0,
            });
            assert_eq!(boxed.eval_interval(&b), expected);
            let arc: std::sync::Arc<dyn Sdf> = std::sync::Arc::new(Sphere {
                center: Point3::origin(),
                radius: 1.0,
            });
            assert_eq!(arc.eval_interval(&b), expected);
        }
    }
}
