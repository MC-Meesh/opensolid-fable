//! Meshing entry points for the unified kernel.
//!
//! [`TriangleMesh`] is the shared interchange type: F-Rep meshing produces
//! it (see [`mesh_sdf_indexed`]), B-Rep tessellation will produce it, and
//! exporters consume it.

pub use opensolid_core::mesh::{Triangle, TriangleMesh};
pub use opensolid_frep::mesh::{MeshOptions, mesh_sdf, mesh_sdf_indexed};

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_core::types::{BoundingBox3, Point3};
    use opensolid_frep::primitives::Sphere;

    /// End-to-end through the kernel API: mesh an SDF, round-trip through a
    /// triangle soup, weld, and verify the mesh utilities agree.
    #[test]
    fn sdf_to_triangle_mesh_pipeline() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = MeshOptions {
            bounds: BoundingBox3::new(Point3::new(-1.6, -1.6, -1.6), Point3::new(1.6, 1.6, 1.6)),
            resolution: 16,
        };
        let mesh = mesh_sdf_indexed(&s, &opts);
        assert!(mesh.is_closed_manifold());

        let welded = TriangleMesh::from_triangles(&mesh.to_triangles()).weld(1e-12);
        assert!(welded.is_closed_manifold());
        assert_eq!(welded.triangle_count(), mesh.triangle_count());

        let bbox = welded.bounding_box().expect("sphere mesh is non-empty");
        for (lo, hi) in [
            (bbox.min.x, bbox.max.x),
            (bbox.min.y, bbox.max.y),
            (bbox.min.z, bbox.max.z),
        ] {
            assert!(lo < -0.8 && lo > -1.2, "bbox min {lo} not near -1");
            assert!(hi > 0.8 && hi < 1.2, "bbox max {hi} not near 1");
        }

        let expected = 4.0 * std::f64::consts::PI;
        assert!((welded.total_area() - expected).abs() / expected < 0.2);
    }
}
