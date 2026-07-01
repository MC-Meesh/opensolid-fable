use opensolid_core::types::Point3;

pub trait Sdf: Send + Sync {
    fn eval(&self, p: &Point3) -> f64;
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
        let outside = (d[0].max(0.0).powi(2) + d[1].max(0.0).powi(2) + d[2].max(0.0).powi(2))
            .sqrt();
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

#[cfg(test)]
mod tests {
    use super::*;

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
}
