use crate::primitives::Sdf;
use nalgebra::Vector3;
use opensolid_core::types::Point3;

/// Thin wrapper over [`Sdf::grad`] for dyn contexts.
pub fn gradient(sdf: &dyn Sdf, p: &Point3) -> Vector3<f64> {
    sdf.grad(p)
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
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let n = normal(&s, &Point3::new(1.0, 0.0, 0.0));
        assert!((n.x - 1.0).abs() < 1e-4);
        assert!(n.y.abs() < 1e-4);
        assert!(n.z.abs() < 1e-4);
    }
}
