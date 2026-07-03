//! Mass properties of the solid enclosed by a closed triangle mesh.
//!
//! Volume, centroid, and inertia are exact polyhedral integrals computed via
//! the divergence theorem: each triangle spans a signed tetrahedron with the
//! origin, and moments accumulate over all tetrahedra. Signed accumulation
//! makes the result independent of where the origin sits relative to the
//! mesh. Surface area is summed directly from the triangles.

use nalgebra::Matrix3;
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{Point3, Vector3};
use thiserror::Error;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum MassPropertiesError {
    /// The mesh has boundary edges, inconsistent winding, degenerate edges,
    /// or out-of-bounds indices, so the enclosed volume is undefined.
    #[error("mesh is not a closed, consistently oriented manifold")]
    NotClosedManifold,
    /// The mesh is closed but encloses no volume (e.g. two coincident
    /// triangles forming a zero-thickness pillow).
    #[error("mesh encloses zero volume")]
    ZeroVolume,
}

/// Mass properties at unit density: mass equals volume, and the inertia
/// tensor scales linearly with density.
#[derive(Debug, Clone, PartialEq)]
pub struct MassProperties {
    /// Enclosed volume. Positive regardless of winding orientation: a
    /// consistently inward-wound mesh is treated the same as its
    /// outward-wound mirror.
    pub volume: f64,
    /// Total surface area.
    pub surface_area: f64,
    /// Center of mass of the enclosed solid.
    pub centroid: Point3,
    /// Inertia tensor about the centroid, unit density. Symmetric.
    pub inertia: Matrix3<f64>,
}

/// Compute the mass properties of the solid enclosed by `mesh`.
///
/// The mesh must be a closed, consistently oriented 2-manifold (see
/// [`TriangleMesh::is_closed_manifold`]); anything else returns
/// [`MassPropertiesError::NotClosedManifold`].
pub fn mass_properties(mesh: &TriangleMesh) -> Result<MassProperties, MassPropertiesError> {
    if !mesh.is_closed_manifold() {
        return Err(MassPropertiesError::NotClosedManifold);
    }

    let mut volume = 0.0;
    // First moments ∫(x, y, z) dV and second moments S[u][v] = ∫ u·v dV.
    let mut first = Vector3::zeros();
    let mut second = Matrix3::<f64>::zeros();
    for tri in &mesh.indices {
        let a = mesh.positions[tri[0]].coords;
        let b = mesh.positions[tri[1]].coords;
        let c = mesh.positions[tri[2]].coords;
        // 6 × signed volume of the tetrahedron (origin, a, b, c).
        let det = a.dot(&b.cross(&c));
        volume += det / 6.0;
        first += (a + b + c) * (det / 24.0);
        // For linear f, g on a tetrahedron with vertices v_k:
        // ∫ f·g dV = V/20 · (Σ f(v_k)·g(v_k) + Σ f(v_k) · Σ g(v_k)),
        // where the origin vertex contributes zero to both sums.
        let s = a + b + c;
        second += (a * a.transpose() + b * b.transpose() + c * c.transpose() + s * s.transpose())
            * (det / 120.0);
    }

    // A consistently inward-wound mesh flips the sign of every integral;
    // normalize so the orientation convention doesn't matter.
    if volume < 0.0 {
        volume = -volume;
        first = -first;
        second = -second;
    }
    if volume == 0.0 || volume.is_nan() {
        return Err(MassPropertiesError::ZeroVolume);
    }

    let centroid = first / volume;
    // I_origin: diagonal Ixx = ∫(y² + z²) = tr(S) − S_xx, off-diagonal
    // Ixy = −S_xy — both captured by tr(S)·E − S.
    let inertia_origin = Matrix3::identity() * second.trace() - second;
    // Parallel-axis shift to the centroid: I_c = I_o − m·(|d|²·E − d·dᵀ).
    let inertia = inertia_origin
        - (Matrix3::identity() * centroid.norm_squared() - centroid * centroid.transpose())
            * volume;

    Ok(MassProperties {
        volume,
        surface_area: mesh.total_area(),
        centroid: Point3::from(centroid),
        inertia,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{MeshOptions, mesh_sdf_indexed};
    use opensolid_core::types::BoundingBox3;
    use opensolid_frep::primitives::Sphere;

    /// Axis-aligned box as 12 outward-wound triangles.
    fn box_mesh(min: Point3, extents: Vector3) -> TriangleMesh {
        let p = |dx: f64, dy: f64, dz: f64| {
            Point3::new(
                min.x + dx * extents.x,
                min.y + dy * extents.y,
                min.z + dz * extents.z,
            )
        };
        TriangleMesh {
            positions: vec![
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
                p(0.0, 0.0, 1.0),
                p(1.0, 0.0, 1.0),
                p(1.0, 1.0, 1.0),
                p(0.0, 1.0, 1.0),
            ],
            normals: vec![Vector3::zeros(); 8],
            indices: vec![
                [0, 3, 2],
                [0, 2, 1], // bottom (−z)
                [4, 5, 6],
                [4, 6, 7], // top (+z)
                [0, 1, 5],
                [0, 5, 4], // front (−y)
                [3, 7, 6],
                [3, 6, 2], // back (+y)
                [0, 4, 7],
                [0, 7, 3], // left (−x)
                [1, 2, 6],
                [1, 6, 5], // right (+x)
            ],
        }
    }

    /// Regular tetrahedron centered on the origin, outward-wound.
    fn tetrahedron() -> TriangleMesh {
        let v = [
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
        ];
        TriangleMesh {
            positions: v.to_vec(),
            normals: vec![Vector3::zeros(); 4],
            indices: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        }
    }

    fn sphere_mesh(center: Point3, radius: f64, resolution: usize) -> TriangleMesh {
        let margin = Vector3::new(1.6, 1.6, 1.6) * radius;
        let opts = MeshOptions {
            bounds: BoundingBox3::new(center - margin, center + margin),
            resolution,
        };
        mesh_sdf_indexed(&Sphere { center, radius }, &opts)
    }

    #[test]
    fn box_mass_properties_are_exact() {
        // Box far from the origin: exercises signed-tetrahedron cancellation
        // and the parallel-axis shift.
        let mesh = box_mesh(Point3::new(10.0, -5.0, 3.0), Vector3::new(2.0, 3.0, 4.0));
        let mp = mass_properties(&mesh).unwrap();

        assert!((mp.volume - 24.0).abs() < 1e-9, "volume {}", mp.volume);
        assert!(
            (mp.surface_area - 52.0).abs() < 1e-9,
            "area {}",
            mp.surface_area
        );
        let centroid_err = (mp.centroid - Point3::new(11.0, -3.5, 5.0)).norm();
        assert!(centroid_err < 1e-9, "centroid {:?}", mp.centroid);

        // Solid box about its centroid: I_xx = m/12·(b² + c²), products zero.
        let m = 24.0;
        let expected = [
            m / 12.0 * (9.0 + 16.0),
            m / 12.0 * (4.0 + 16.0),
            m / 12.0 * (4.0 + 9.0),
        ];
        for (i, want) in expected.into_iter().enumerate() {
            assert!(
                (mp.inertia[(i, i)] - want).abs() < 1e-8,
                "I[{i}][{i}] = {}, want {want}",
                mp.inertia[(i, i)]
            );
        }
        for i in 0..3 {
            for j in 0..3 {
                if i != j {
                    assert!(
                        mp.inertia[(i, j)].abs() < 1e-8,
                        "product of inertia I[{i}][{j}] = {}",
                        mp.inertia[(i, j)]
                    );
                }
            }
        }
    }

    #[test]
    fn regular_tetrahedron_is_exact_and_isotropic() {
        let mp = mass_properties(&tetrahedron()).unwrap();
        // Edge a = 2√2: V = a³/(6√2) = 8/3, I = m·a²/20 = (8/3)·8/20 = 16/15.
        assert!((mp.volume - 8.0 / 3.0).abs() < 1e-12);
        assert!(mp.centroid.coords.norm() < 1e-12);
        for i in 0..3 {
            assert!((mp.inertia[(i, i)] - 16.0 / 15.0).abs() < 1e-12);
            for j in 0..3 {
                if i != j {
                    assert!(mp.inertia[(i, j)].abs() < 1e-12);
                }
            }
        }
    }

    #[test]
    fn meshed_unit_sphere_volume_within_two_percent() {
        let mp = mass_properties(&sphere_mesh(Point3::origin(), 1.0, 32)).unwrap();
        let expected = 4.0 / 3.0 * std::f64::consts::PI;
        let rel = (mp.volume - expected).abs() / expected;
        assert!(rel < 0.02, "volume {} vs {expected} (rel {rel})", mp.volume);

        let area = 4.0 * std::f64::consts::PI;
        let area_rel = (mp.surface_area - area).abs() / area;
        assert!(area_rel < 0.1, "area {} vs {area}", mp.surface_area);
    }

    #[test]
    fn offset_sphere_centroid_at_its_center() {
        let center = Point3::new(1.5, -2.0, 0.75);
        let mp = mass_properties(&sphere_mesh(center, 1.0, 32)).unwrap();
        let err = (mp.centroid - center).norm();
        assert!(err < 0.02, "centroid {:?} off by {err}", mp.centroid);
    }

    #[test]
    fn meshed_sphere_inertia_near_analytic() {
        let mp = mass_properties(&sphere_mesh(Point3::origin(), 1.0, 32)).unwrap();
        // Solid sphere: I = (2/5)·m·r², isotropic. Compare against the mesh's
        // own volume so the check isolates inertia error from volume error.
        let expected = 0.4 * mp.volume;
        for i in 0..3 {
            let rel = (mp.inertia[(i, i)] - expected).abs() / expected;
            assert!(
                rel < 0.05,
                "I[{i}][{i}] = {} vs {expected}",
                mp.inertia[(i, i)]
            );
            for j in 0..3 {
                if i != j {
                    assert!(mp.inertia[(i, j)].abs() < 0.05 * expected);
                }
            }
        }
    }

    #[test]
    fn inward_wound_mesh_gives_same_positive_properties() {
        let mut inward = tetrahedron();
        for tri in &mut inward.indices {
            tri.swap(1, 2);
        }
        let mp = mass_properties(&inward).unwrap();
        assert!((mp.volume - 8.0 / 3.0).abs() < 1e-12);
        assert!((mp.inertia[(0, 0)] - 16.0 / 15.0).abs() < 1e-12);
    }

    #[test]
    fn open_mesh_is_rejected() {
        let mesh = TriangleMesh {
            positions: vec![
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: vec![Vector3::zeros(); 3],
            indices: vec![[0, 1, 2]],
        };
        assert_eq!(
            mass_properties(&mesh),
            Err(MassPropertiesError::NotClosedManifold)
        );
        assert_eq!(
            mass_properties(&TriangleMesh::new()),
            Err(MassPropertiesError::NotClosedManifold)
        );
    }

    #[test]
    fn zero_thickness_pillow_is_rejected() {
        // Two coincident triangles wound opposite ways form a closed manifold
        // that encloses nothing.
        let mesh = TriangleMesh {
            positions: vec![
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ],
            normals: vec![Vector3::zeros(); 3],
            indices: vec![[0, 1, 2], [0, 2, 1]],
        };
        assert!(mesh.is_closed_manifold());
        assert_eq!(mass_properties(&mesh), Err(MassPropertiesError::ZeroVolume));
    }
}
