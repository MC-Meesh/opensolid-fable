//! Mass properties of the solid enclosed by a closed triangle mesh.
//!
//! Volume, centroid, and inertia are exact polyhedral integrals computed via
//! the divergence theorem: each triangle spans a signed tetrahedron with a
//! common apex, and moments accumulate over all tetrahedra. Signed
//! accumulation makes the result independent of where that apex sits relative
//! to the mesh — but only in exact arithmetic, so the apex is placed at the
//! mesh's own bounding-box centre rather than the absolute origin (see
//! [`reference_point`]). Surface area is summed directly from the triangles.

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

/// The apex all the signed tetrahedra are referenced to: the centre of the
/// mesh's axis-aligned bounding box.
///
/// The choice does not change the exact answer — the signed sum telescopes to
/// the same integral for any apex — but it decides the *magnitudes* the sum
/// passes through on the way there. With the absolute origin as apex, a body
/// sitting distance `D` away accumulates terms of size `D³` to land on an
/// answer of size `feature³`, so `log10(D³/feature³)` digits are cancelled
/// away before the result begins; a 4×4×2 slab at `D = 1e6` came out 191×
/// too large, and at `1e8` the volume cancelled to exactly zero. Referencing
/// the tetrahedra to a point inside the mesh caps every intermediate at the
/// mesh's own half-diameter, so the cancellation is bounded by the geometry's
/// aspect ratio instead of by its distance from the origin.
///
/// Halving the extent is why this is the bbox centre and not `positions[0]`:
/// both are `O(diameter)`, but the centre is `O(diameter/2)` and is the
/// natural centre of a symmetric body, where it often lands on an exactly
/// representable coordinate and leaves the shifted vertices exact.
fn reference_point(mesh: &TriangleMesh) -> Vector3 {
    let mut min = Vector3::repeat(f64::INFINITY);
    let mut max = Vector3::repeat(f64::NEG_INFINITY);
    for p in &mesh.positions {
        min = min.inf(&p.coords);
        max = max.sup(&p.coords);
    }
    // A mesh with no vertices never reaches here (it is not a closed
    // manifold), but a non-finite coordinate would leave the bounds as they
    // started; fall back to the origin rather than propagate a NaN apex.
    let mid = (min + max) * 0.5;
    if mid.iter().all(|v| v.is_finite()) {
        mid
    } else {
        Vector3::zeros()
    }
}

/// Compute the mass properties of the solid enclosed by `mesh`.
///
/// The mesh must be a closed, consistently oriented 2-manifold (see
/// [`TriangleMesh::is_closed_manifold`]); anything else returns
/// [`MassPropertiesError::NotClosedManifold`].
///
/// Accuracy does not depend on where the mesh sits: the integration is
/// referenced to the mesh's own bounding-box centre and the centroid is
/// shifted back afterwards (see [`reference_point`]).
pub fn mass_properties(mesh: &TriangleMesh) -> Result<MassProperties, MassPropertiesError> {
    if !mesh.is_closed_manifold() {
        return Err(MassPropertiesError::NotClosedManifold);
    }

    let origin = reference_point(mesh);
    let mut volume = 0.0;
    // Moments of the mesh *translated by −origin*: first ∫(x, y, z) dV and
    // second S[u][v] = ∫ u·v dV, both in that shifted frame.
    let mut first = Vector3::zeros();
    let mut second = Matrix3::<f64>::zeros();
    for tri in &mesh.indices {
        let a = mesh.positions[tri[0]].coords - origin;
        let b = mesh.positions[tri[1]].coords - origin;
        let c = mesh.positions[tri[2]].coords - origin;
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

    // Centroid of the shifted mesh. Volume is translation-invariant, so this
    // is the true centroid measured from `origin`.
    let local_centroid = first / volume;
    // I_apex: diagonal Ixx = ∫(y² + z²) = tr(S) − S_xx, off-diagonal
    // Ixy = −S_xy — both captured by tr(S)·E − S.
    let inertia_apex = Matrix3::identity() * second.trace() - second;
    // Parallel-axis shift to the centroid: I_c = I_o − m·(|d|²·E − d·dᵀ).
    // Inertia *about the centroid* is itself translation-invariant, so this
    // is already the answer — no second shift by `origin` is needed, and
    // deliberately so: it is the shift by the full absolute position that
    // used to subtract two large nearly-equal tensors.
    let inertia = inertia_apex
        - (Matrix3::identity() * local_centroid.norm_squared()
            - local_centroid * local_centroid.transpose())
            * volume;

    Ok(MassProperties {
        volume,
        surface_area: mesh.total_area(),
        // The centroid is the one output that is not translation-invariant;
        // shift it back into absolute coordinates.
        centroid: Point3::from(local_centroid + origin),
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

    /// The same box rebuilt at increasing distance from the origin must give
    /// the same volume, the same inertia, and a centroid that tracks the
    /// offset — this is of-ukcq. Referenced to the absolute origin the
    /// intermediate tetrahedra are `O(offset³)` against a 24-unit answer, and
    /// the volume went 4× wrong by 1e6 and cancelled to zero by 1e8.
    ///
    /// The offsets are irrational on purpose: with integer coordinates every
    /// product below 2^53 is exact, so the arithmetic never rounds and the
    /// test silently proves nothing.
    #[test]
    fn box_properties_are_stable_far_from_origin() {
        let extents = Vector3::new(2.0, 3.0, 4.0);
        let inertia_want = [
            24.0 / 12.0 * (9.0 + 16.0),
            24.0 / 12.0 * (4.0 + 16.0),
            24.0 / 12.0 * (4.0 + 9.0),
        ];
        for exp in [0, 2, 4, 6, 8, 10] {
            let d = 10f64.powi(exp) + std::f64::consts::FRAC_1_PI;
            let min = Point3::new(d, d, d);
            let mp = mass_properties(&box_mesh(min, extents)).unwrap();

            let rel = (mp.volume - 24.0).abs() / 24.0;
            assert!(
                rel < 1e-12,
                "offset {d:e}: volume {} (rel {rel})",
                mp.volume
            );
            assert!(
                (mp.surface_area - 52.0).abs() < 1e-9,
                "offset {d:e}: area {}",
                mp.surface_area
            );

            // The centroid is absolute, so it can only be resolved to the
            // f64 spacing at `d`; ask for that and nothing more.
            let want = min + extents * 0.5;
            let err = (mp.centroid - want).norm();
            assert!(
                err <= 16.0 * d * f64::EPSILON,
                "offset {d:e}: centroid {:?} off by {err}",
                mp.centroid
            );

            // Inertia is about the centroid, hence translation-invariant:
            // the far-field case must be as exact as the near-field one.
            for (i, w) in inertia_want.into_iter().enumerate() {
                let rel = (mp.inertia[(i, i)] - w).abs() / w;
                assert!(
                    rel < 1e-11,
                    "offset {d:e}: I[{i}][{i}] = {} vs {w}",
                    mp.inertia[(i, i)]
                );
                for j in 0..3 {
                    if i != j {
                        assert!(
                            mp.inertia[(i, j)].abs() < 1e-9,
                            "offset {d:e}: I[{i}][{j}] = {}",
                            mp.inertia[(i, j)]
                        );
                    }
                }
            }
        }
    }

    /// A far-field body whose smallest feature is far smaller than its
    /// distance from the origin — the corner where the fix has the least
    /// room, since the apex sits inside the bbox but the body still spans
    /// six decades of extent.
    ///
    /// The budget is the *input* floor, not a fudge factor. A 1e-3 thickness
    /// is recovered as the difference of two coordinates near 1e6, where the
    /// f64 grid is `d·ε` wide, so the thickness is only known to
    /// `d·ε/thickness ≈ 2.2e-7` relative before `mass_properties` is even
    /// called. Passing at a few times under that means the integration
    /// itself added essentially nothing; referenced to the absolute origin
    /// it was 191× wrong at this distance.
    #[test]
    fn far_field_holds_for_a_high_aspect_ratio_body() {
        let d = 1e6 + std::f64::consts::FRAC_1_PI;
        let (long, thin) = (1000.0, 1e-3);
        let extents = Vector3::new(long, 1.0, thin);
        let mp = mass_properties(&box_mesh(Point3::new(d, d, d), extents)).unwrap();
        let want = long * 1.0 * thin;
        let rel = (mp.volume - want).abs() / want;
        let floor = d * f64::EPSILON / thin;
        assert!(
            rel < floor,
            "volume {} vs {want} (rel {rel}, input floor {floor})",
            mp.volume
        );
    }

    /// A mesh straddling the origin is the case the old code was good at.
    /// The fix must not cost it anything: the bbox centre is the origin, so
    /// this is bit-for-bit the same arithmetic as before.
    #[test]
    fn reference_point_is_the_origin_for_a_centered_mesh() {
        let mesh = box_mesh(Point3::new(-1.0, -1.5, -2.0), Vector3::new(2.0, 3.0, 4.0));
        assert_eq!(reference_point(&mesh), Vector3::zeros());
        let mp = mass_properties(&mesh).unwrap();
        assert!((mp.volume - 24.0).abs() < 1e-13);
        assert!(mp.centroid.coords.norm() < 1e-13);
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
