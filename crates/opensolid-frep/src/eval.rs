use crate::primitives::Sdf;
use opensolid_core::types::Point3;
use nalgebra::Vector3;

const GRADIENT_EPS: f64 = 1e-6;

pub fn gradient(sdf: &dyn Sdf, p: &Point3) -> Vector3<f64> {
    let dx = sdf.eval(&Point3::new(p.x + GRADIENT_EPS, p.y, p.z))
        - sdf.eval(&Point3::new(p.x - GRADIENT_EPS, p.y, p.z));
    let dy = sdf.eval(&Point3::new(p.x, p.y + GRADIENT_EPS, p.z))
        - sdf.eval(&Point3::new(p.x, p.y - GRADIENT_EPS, p.z));
    let dz = sdf.eval(&Point3::new(p.x, p.y, p.z + GRADIENT_EPS))
        - sdf.eval(&Point3::new(p.x, p.y, p.z - GRADIENT_EPS));
    Vector3::new(dx, dy, dz) / (2.0 * GRADIENT_EPS)
}

pub fn normal(sdf: &dyn Sdf, p: &Point3) -> Vector3<f64> {
    gradient(sdf, p).normalize()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives::Sphere;

    #[test]
    fn sphere_normal_points_outward() {
        let s = Sphere { center: Point3::origin(), radius: 1.0 };
        let n = normal(&s, &Point3::new(1.0, 0.0, 0.0));
        assert!((n.x - 1.0).abs() < 1e-4);
        assert!(n.y.abs() < 1e-4);
        assert!(n.z.abs() < 1e-4);
    }
}
