use opensolid_core::types::{Point3, Vector3};

pub trait Sdf: Send + Sync {
    fn eval(&self, p: &Point3) -> f64;
}

impl<T: Sdf + ?Sized> Sdf for &T {
    fn eval(&self, p: &Point3) -> f64 {
        (**self).eval(p)
    }
}

impl<T: Sdf + ?Sized> Sdf for Box<T> {
    fn eval(&self, p: &Point3) -> f64 {
        (**self).eval(p)
    }
}

impl<T: Sdf + ?Sized> Sdf for std::sync::Arc<T> {
    fn eval(&self, p: &Point3) -> f64 {
        (**self).eval(p)
    }
}

pub struct Sphere {
    pub center: Point3,
    pub radius: f64,
}

impl Sdf for Sphere {
    fn eval(&self, p: &Point3) -> f64 {
        (p - self.center).norm() - self.radius
    }
}

pub struct Box3 {
    pub center: Point3,
    pub half_extents: [f64; 3],
}

impl Sdf for Box3 {
    fn eval(&self, p: &Point3) -> f64 {
        let d = [
            (p.x - self.center.x).abs() - self.half_extents[0],
            (p.y - self.center.y).abs() - self.half_extents[1],
            (p.z - self.center.z).abs() - self.half_extents[2],
        ];
        let outside =
            (d[0].max(0.0).powi(2) + d[1].max(0.0).powi(2) + d[2].max(0.0).powi(2)).sqrt();
        let inside = d[0].max(d[1]).max(d[2]).min(0.0);
        outside + inside
    }
}

pub struct Cylinder {
    pub center: Point3,
    pub radius: f64,
    pub half_height: f64,
}

impl Sdf for Cylinder {
    fn eval(&self, p: &Point3) -> f64 {
        let dx = p.x - self.center.x;
        let dz = p.z - self.center.z;
        let radial = (dx * dx + dz * dz).sqrt() - self.radius;
        let axial = (p.y - self.center.y).abs() - self.half_height;
        let outside = radial.max(0.0).hypot(axial.max(0.0));
        let inside = radial.max(axial).min(0.0);
        outside + inside
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
}

pub struct Cone {
    pub center: Point3,
    pub half_height: f64,
    pub radius_bottom: f64,
    pub radius_top: f64,
}

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
}

pub struct HalfSpace {
    pub normal: Vector3,
    pub offset: f64,
}

impl Sdf for HalfSpace {
    fn eval(&self, p: &Point3) -> f64 {
        (self.normal.dot(&p.coords) - self.offset) / self.normal.norm()
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
}
