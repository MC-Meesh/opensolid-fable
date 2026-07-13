//! Interchange I/O.
//!
//! Mesh export: write a [`TriangleMesh`](crate::mesh::TriangleMesh) to standard
//! interchange formats via any [`std::io::Write`] sink.
//!
//! Formats:
//! - Binary STL ([`write_stl_binary`]) — compact, the de-facto 3D-print format.
//! - ASCII STL ([`write_stl_ascii`]) — human-readable STL.
//! - Wavefront OBJ ([`write_obj`]) — positions + per-vertex normals, indexed.
//!
//! CAD import: [`step`] parses the STEP Part 21 (ISO-10303-21) exchange
//! structure into a flat entity graph (syntax only; semantics live elsewhere).
//!
//! STL carries one facet normal per triangle; both STL writers recompute it
//! from the triangle's vertex positions (right-hand rule) rather than trusting
//! stored per-vertex normals, since STL consumers expect geometric facet
//! normals. OBJ preserves the mesh's per-vertex normals.
//!
//! All writers assume a structurally valid mesh (every index in bounds) and
//! panic otherwise; use buffered writers for large meshes.

pub mod obj;
pub mod step;
pub mod stl;

pub use obj::write_obj;
pub use stl::{write_stl_ascii, write_stl_binary};

#[cfg(test)]
pub(crate) mod test_meshes {
    use crate::mesh::{Triangle, TriangleMesh};
    use opensolid_core::types::{Point3, Vector3};

    /// Axis-aligned unit box as a 12-triangle soup mesh (36 vertices,
    /// outward-facing windings, facet normals replicated per vertex).
    pub(crate) fn unit_box() -> TriangleMesh {
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        let corners = [
            p(0.0, 0.0, 0.0), // 0
            p(1.0, 0.0, 0.0), // 1
            p(0.0, 1.0, 0.0), // 2
            p(1.0, 1.0, 0.0), // 3
            p(0.0, 0.0, 1.0), // 4
            p(1.0, 0.0, 1.0), // 5
            p(0.0, 1.0, 1.0), // 6
            p(1.0, 1.0, 1.0), // 7
        ];
        // (corner indices, outward normal) per triangle; windings follow the
        // right-hand rule so the geometric normal matches the stated one.
        #[rustfmt::skip]
        let tris: [([usize; 3], Vector3); 12] = [
            ([0, 2, 3], Vector3::new(0.0, 0.0, -1.0)),
            ([0, 3, 1], Vector3::new(0.0, 0.0, -1.0)),
            ([4, 5, 7], Vector3::new(0.0, 0.0, 1.0)),
            ([4, 7, 6], Vector3::new(0.0, 0.0, 1.0)),
            ([0, 1, 5], Vector3::new(0.0, -1.0, 0.0)),
            ([0, 5, 4], Vector3::new(0.0, -1.0, 0.0)),
            ([2, 6, 7], Vector3::new(0.0, 1.0, 0.0)),
            ([2, 7, 3], Vector3::new(0.0, 1.0, 0.0)),
            ([0, 4, 6], Vector3::new(-1.0, 0.0, 0.0)),
            ([0, 6, 2], Vector3::new(-1.0, 0.0, 0.0)),
            ([1, 3, 7], Vector3::new(1.0, 0.0, 0.0)),
            ([1, 7, 5], Vector3::new(1.0, 0.0, 0.0)),
        ];
        let soup: Vec<Triangle> = tris
            .iter()
            .map(|&(idx, normal)| Triangle {
                positions: idx.map(|i| corners[i]),
                normals: [normal; 3],
            })
            .collect();
        TriangleMesh::from_triangles(&soup)
    }

    #[test]
    fn unit_box_is_a_valid_closed_solid() {
        let welded = unit_box().weld(0.0);
        assert_eq!(welded.vertex_count(), 8);
        assert_eq!(welded.triangle_count(), 12);
        assert!(welded.is_closed_manifold());
    }
}
