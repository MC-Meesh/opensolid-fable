//! B-Rep → F-Rep: signed distance to a tessellated body.
//!
//! [`MeshSdf`] wraps a closed [`TriangleMesh`] as an [`Sdf`], so any B-Rep
//! body becomes a first-class F-Rep field through tessellation
//! ([`MeshSdf::from_body`]) — ready for CSG composition, blending, and
//! re-meshing. The field is the exact distance to the *mesh*, so it
//! deviates from the true body's distance by at most the tessellation's
//! chordal error.
//!
//! # Sign: angle-weighted pseudonormals
//!
//! The unsigned distance comes from a BVH nearest-triangle query
//! (`O(log n)` per evaluation). The sign is the dot product of the offset
//! `p - closest` with the **angle-weighted pseudonormal** of the closest
//! feature (Bærentzen & Aanæs, "Signed distance computation using the
//! angle weighted pseudonormal", 2005): the face normal for interior hits,
//! the sum of the two adjacent face normals for edge hits, and the
//! incident-angle-weighted normal sum for vertex hits. For a closed,
//! consistently outward-oriented 2-manifold this sign is provably correct
//! everywhere — including queries nearest to sharp edges and corners,
//! where a plain face normal misclassifies.
//!
//! Chosen over the generalized winding number (Jacobson et al. 2013)
//! because one nearest query per evaluation is cheap and the sign is exact
//! for our inputs: tessellated B-Rep bodies are watertight by
//! construction. The trade-off is **zero tolerance for defective input** —
//! holes, self-intersections, or inverted orientation break the sign — so
//! the constructor validates closedness/orientation up front and rejects
//! anything else. A fast-winding-number fallback for imperfect (imported)
//! meshes is a later hardening pass.
//!
//! # Limits
//!
//! - Input must be a closed, consistently oriented 2-manifold with outward
//!   winding and positive enclosed volume ([`MeshSdf::new`] rejects the
//!   rest). Near-zero-area triangles are rejected too: their normals are
//!   numerically meaningless and would poison the pseudonormals.
//! - The field is exact for the mesh, not the smooth body it approximates;
//!   refine the tessellation to tighten the gap.
//! - [`Sdf::eval_interval`] keeps the default center-plus-half-diagonal
//!   bound, which is valid here because the field is a true metric SDF
//!   (Lipschitz constant 1).

use crate::bvh::Bvh;
use opensolid_brep::{GeometryStore, TessellationOptions, TopologyStore, tessellate_body};
use opensolid_core::EntityId;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{Point3, Vector3};
use opensolid_frep::primitives::Sdf;
use std::collections::HashMap;

/// Signed distance field of a closed triangle mesh. See the module docs
/// for the algorithm and its limits.
pub struct MeshSdf {
    positions: Vec<Point3>,
    triangles: Vec<[usize; 3]>,
    /// Unit outward normal per triangle.
    face_normals: Vec<Vector3>,
    /// Unit angle-weighted pseudonormal per vertex.
    vertex_normals: Vec<Vector3>,
    /// Unit pseudonormal per undirected edge `(lo, hi)`: the (normalized)
    /// sum of its two adjacent face normals.
    edge_normals: HashMap<(usize, usize), Vector3>,
    bvh: Bvh<usize>,
}

impl std::fmt::Debug for MeshSdf {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("MeshSdf")
            .field("triangles", &self.triangles.len())
            .field("vertices", &self.positions.len())
            .finish_non_exhaustive()
    }
}

/// Region of a triangle containing the closest point, in local indices.
enum Feature {
    Vertex(usize),
    /// Edge from local vertex `k` to `(k + 1) % 3`.
    Edge(usize),
    Face,
}

impl MeshSdf {
    /// Build the signed distance field of `mesh`.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if the mesh is not a closed,
    /// consistently oriented manifold, or if it encloses non-positive
    /// signed volume (inverted orientation);
    /// [`CoreError::Degenerate`] if any triangle's area is negligible
    /// relative to its edge lengths, or a pseudonormal cancels to zero.
    pub fn new(mesh: &TriangleMesh) -> CoreResult<Self> {
        if !mesh.is_closed_manifold() {
            return Err(CoreError::InvalidArgument {
                argument: "mesh",
                reason: "must be a closed, consistently oriented 2-manifold".to_string(),
            });
        }

        let positions = mesh.positions.clone();
        let triangles = mesh.indices.clone();

        let mut face_normals = Vec::with_capacity(triangles.len());
        let mut signed_volume = 0.0;
        for (t, tri) in triangles.iter().enumerate() {
            let [a, b, c] = tri.map(|i| positions[i]);
            let cross = (b - a).cross(&(c - a));
            let norm = cross.norm();
            let longest_sq = (b - a)
                .norm_squared()
                .max((c - a).norm_squared())
                .max((c - b).norm_squared());
            if norm <= 1e-12 * longest_sq {
                return Err(CoreError::Degenerate {
                    context: "MeshSdf::new",
                    reason: format!("triangle {t} has negligible area (sliver or collinear)"),
                });
            }
            face_normals.push(cross / norm);
            signed_volume += a.coords.dot(&b.coords.cross(&c.coords)) / 6.0;
        }
        if signed_volume <= 0.0 {
            return Err(CoreError::InvalidArgument {
                argument: "mesh",
                reason: format!(
                    "winding must be outward (enclosed signed volume {signed_volume} <= 0)"
                ),
            });
        }

        let mut vertex_sums = vec![Vector3::zeros(); positions.len()];
        let mut edge_sums: HashMap<(usize, usize), Vector3> = HashMap::new();
        for (tri, normal) in triangles.iter().zip(&face_normals) {
            for k in 0..3 {
                let (i, j) = (tri[k], tri[(k + 1) % 3]);
                *edge_sums.entry((i.min(j), i.max(j))).or_default() += normal;

                let corner = positions[tri[k]];
                let u = (positions[tri[(k + 1) % 3]] - corner).normalize();
                let v = (positions[tri[(k + 2) % 3]] - corner).normalize();
                let angle = u.dot(&v).clamp(-1.0, 1.0).acos();
                vertex_sums[tri[k]] += normal * angle;
            }
        }
        let unit = |sum: Vector3, what: &str| -> CoreResult<Vector3> {
            let norm = sum.norm();
            if norm <= 1e-12 {
                return Err(CoreError::Degenerate {
                    context: "MeshSdf::new",
                    reason: format!("{what} pseudonormal cancels to zero"),
                });
            }
            Ok(sum / norm)
        };
        let vertex_normals = vertex_sums
            .into_iter()
            .map(|sum| unit(sum, "vertex"))
            .collect::<CoreResult<Vec<_>>>()?;
        let edge_normals = edge_sums
            .into_iter()
            .map(|(key, sum)| Ok((key, unit(sum, "edge")?)))
            .collect::<CoreResult<HashMap<_, _>>>()?;

        let bvh = Bvh::from_triangle_mesh(mesh);
        Ok(Self {
            positions,
            triangles,
            face_normals,
            vertex_normals,
            edge_normals,
            bvh,
        })
    }

    /// Tessellate `body` and wrap the result as an SDF.
    ///
    /// # Errors
    /// Tessellation errors ([`tessellate_body`]), plus [`MeshSdf::new`]'s
    /// validation of the resulting mesh.
    pub fn from_body(
        store: &TopologyStore,
        geo: &GeometryStore,
        body: EntityId<opensolid_brep::Body>,
        options: &TessellationOptions,
    ) -> CoreResult<Self> {
        Self::new(&tessellate_body(store, geo, body, options)?)
    }

    /// Number of triangles in the wrapped mesh.
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    fn triangle_points(&self, t: usize) -> [Point3; 3] {
        self.triangles[t].map(|i| self.positions[i])
    }

    /// Signed distance, closest surface point, and the pseudonormal of the
    /// closest feature.
    fn query(&self, p: &Point3) -> (f64, Point3, Vector3) {
        let (_, &t) = self
            .bvh
            .nearest(p, |q, &i| {
                let [a, b, c] = self.triangle_points(i);
                (closest_point_on_triangle(q, &a, &b, &c).0 - q).norm()
            })
            .expect("validated mesh is non-empty");

        let [a, b, c] = self.triangle_points(t);
        let (closest, feature) = closest_point_on_triangle(p, &a, &b, &c);
        let tri = self.triangles[t];
        let normal = match feature {
            Feature::Face => self.face_normals[t],
            Feature::Vertex(k) => self.vertex_normals[tri[k]],
            Feature::Edge(k) => {
                let (i, j) = (tri[k], tri[(k + 1) % 3]);
                self.edge_normals[&(i.min(j), i.max(j))]
            }
        };
        let offset = p - closest;
        let unsigned = offset.norm();
        let signed = if offset.dot(&normal) < 0.0 {
            -unsigned
        } else {
            unsigned
        };
        (signed, closest, normal)
    }
}

impl Sdf for MeshSdf {
    fn eval(&self, p: &Point3) -> f64 {
        self.query(p).0
    }

    /// Exact gradient away from the surface: the unit offset from the
    /// closest point, oriented outward. On the surface itself (offset too
    /// short to normalize) the feature pseudonormal is the limit direction.
    fn grad(&self, p: &Point3) -> Vector3 {
        let (signed, closest, normal) = self.query(p);
        let offset = p - closest;
        let unsigned = offset.norm();
        if unsigned > 1e-9 {
            offset * (signed.signum() / unsigned)
        } else {
            normal
        }
    }

    // eval_interval: default is valid — this is a true metric SDF (module
    // docs).
}

/// Closest point of triangle `abc` to `p`, with the containing feature
/// (Ericson, *Real-Time Collision Detection* §5.1.5).
fn closest_point_on_triangle(p: &Point3, a: &Point3, b: &Point3, c: &Point3) -> (Point3, Feature) {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;
    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return (*a, Feature::Vertex(0));
    }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return (*b, Feature::Vertex(1));
    }

    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (a + ab * v, Feature::Edge(0));
    }

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return (*c, Feature::Vertex(2));
    }

    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (a + ac * w, Feature::Edge(2));
    }

    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (b + (c - b) * w, Feature::Edge(1));
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (a + ab * v + ac * w, Feature::Face)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{MeshOptions, mesh_sdf_indexed};
    use opensolid_brep::primitives;
    use opensolid_core::types::BoundingBox3;

    fn tessellated(
        make: impl FnOnce(
            &mut TopologyStore,
            &mut GeometryStore,
        ) -> CoreResult<EntityId<opensolid_brep::Body>>,
    ) -> TriangleMesh {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = make(&mut store, &mut geo).expect("valid primitive");
        tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds")
    }

    /// Deterministic pseudo-random doubles in [0, 1) (no rand dependency).
    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            (self.0 >> 11) as f64 / (1u64 << 53) as f64
        }
        fn point(&mut self, half: f64) -> Point3 {
            Point3::new(
                (self.next() * 2.0 - 1.0) * half,
                (self.next() * 2.0 - 1.0) * half,
                (self.next() * 2.0 - 1.0) * half,
            )
        }
    }

    #[test]
    fn sphere_sdf_matches_analytic_at_random_points() {
        let r = 2.0;
        let sdf =
            MeshSdf::new(&tessellated(|s, g| primitives::sphere(s, g, r))).expect("valid mesh");
        // The mesh is inscribed: per parameter direction the chordal error
        // is r(1 - cos(step/2)) ≈ 0.01 at 32 segments, and the worst-case
        // facet deviation on the lat-long grid is about twice that (both
        // directions curve); 3× gives headroom.
        let chord = r * (1.0 - (std::f64::consts::TAU / 64.0).cos()) + 1e-6;
        let mut rng = Lcg(42);
        for _ in 0..300 {
            let p = rng.point(3.2);
            let analytic = p.coords.norm() - r;
            let got = sdf.eval(&p);
            assert!(
                (got - analytic).abs() <= 3.0 * chord,
                "at {p:?}: mesh sdf {got} vs analytic {analytic} (chord {chord})"
            );
        }
    }

    #[test]
    fn sign_is_correct_deep_inside_and_outside() {
        let sdf =
            MeshSdf::new(&tessellated(|s, g| primitives::sphere(s, g, 2.0))).expect("valid mesh");
        assert!(
            sdf.eval(&Point3::origin()) < -1.9,
            "deep inside is negative"
        );
        assert!(
            sdf.eval(&Point3::new(10.0, 0.0, 0.0)) > 7.9,
            "far outside is positive"
        );
    }

    #[test]
    fn block_distances_are_exact_per_feature_region() {
        // A 2×2×2 block: face, edge, and vertex regions all have closed
        // forms, exercising every pseudonormal branch.
        let sdf = MeshSdf::new(&tessellated(|s, g| primitives::block(s, g, 2.0, 2.0, 2.0)))
            .expect("valid mesh");
        // Face region, outside and inside.
        assert!((sdf.eval(&Point3::new(2.0, 0.0, 0.0)) - 1.0).abs() < 1e-9);
        assert!((sdf.eval(&Point3::origin()) + 1.0).abs() < 1e-9);
        assert!((sdf.eval(&Point3::new(0.5, 0.0, 0.0)) + 0.5).abs() < 1e-9);
        // Edge region: closest point is (1, 1, 0).
        let edge = sdf.eval(&Point3::new(2.0, 2.0, 0.0));
        assert!((edge - 2.0f64.sqrt()).abs() < 1e-9, "edge distance {edge}");
        // Vertex region: closest point is the corner (1, 1, 1).
        let corner = sdf.eval(&Point3::new(2.0, 2.0, 2.0));
        assert!(
            (corner - 3.0f64.sqrt()).abs() < 1e-9,
            "corner distance {corner}"
        );
    }

    #[test]
    fn gradient_points_radially_on_a_sphere() {
        let sdf =
            MeshSdf::new(&tessellated(|s, g| primitives::sphere(s, g, 2.0))).expect("valid mesh");
        for p in [
            Point3::new(3.0, 0.5, -0.2), // outside
            Point3::new(0.8, 0.3, 0.1),  // inside
        ] {
            let grad = sdf.grad(&p);
            let radial = p.coords.normalize();
            assert!((grad.norm() - 1.0).abs() < 1e-9, "gradient not unit");
            assert!(
                grad.dot(&radial) > 0.95,
                "at {p:?}: gradient {grad:?} not outward-radial"
            );
        }
    }

    #[test]
    fn meshing_the_mesh_sdf_reproduces_a_closed_manifold() {
        let r = 1.0;
        let sdf =
            MeshSdf::new(&tessellated(|s, g| primitives::sphere(s, g, r))).expect("valid mesh");
        let opts = MeshOptions {
            bounds: BoundingBox3::new(Point3::new(-1.6, -1.6, -1.6), Point3::new(1.6, 1.6, 1.6)),
            resolution: 20,
        };
        let remeshed = mesh_sdf_indexed(&sdf, &opts);
        assert!(remeshed.is_closed_manifold(), "round trip must stay closed");
        // And it is still (approximately) the same sphere.
        let bbox = remeshed.bounding_box().expect("non-empty");
        for value in [
            bbox.min.x.abs(),
            bbox.min.y.abs(),
            bbox.min.z.abs(),
            bbox.max.x,
            bbox.max.y,
            bbox.max.z,
        ] {
            assert!((value - r).abs() < 0.2, "bbox extent {value} vs radius {r}");
        }
    }

    #[test]
    fn from_body_convenience_matches_manual_pipeline() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::torus(&mut store, &mut geo, 3.0, 1.0).expect("valid torus");
        let sdf = MeshSdf::from_body(&store, &geo, body, &TessellationOptions::default())
            .expect("valid body");
        // Center of the hole: inside-the-hole is outside the material.
        assert!(sdf.eval(&Point3::origin()) > 0.0);
        // Tube center ring is deep inside.
        assert!(sdf.eval(&Point3::new(3.0, 0.0, 0.0)) < -0.9);
    }

    #[test]
    fn eval_interval_bounds_eval() {
        let sdf =
            MeshSdf::new(&tessellated(|s, g| primitives::sphere(s, g, 1.0))).expect("valid mesh");
        let mut rng = Lcg(7);
        for _ in 0..50 {
            let center = rng.point(1.5);
            let half = 0.1 + rng.next() * 0.5;
            let b = BoundingBox3::new(
                center - Vector3::new(half, half, half),
                center + Vector3::new(half, half, half),
            );
            let range = sdf.eval_interval(&b);
            let d = sdf.eval(&center);
            assert!(
                range.lo <= d && d <= range.hi,
                "interval {range:?} excludes center value {d}"
            );
        }
    }

    #[test]
    fn rejects_open_flipped_and_degenerate_meshes() {
        // Open: a single triangle.
        let open = TriangleMesh {
            positions: vec![
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            normals: vec![Vector3::z(); 3],
            indices: vec![[0, 1, 2]],
        };
        assert!(matches!(
            MeshSdf::new(&open),
            Err(CoreError::InvalidArgument {
                argument: "mesh",
                ..
            })
        ));

        // Inverted: a tetrahedron wound inward (negative volume).
        let v = [
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
        ];
        let inverted = TriangleMesh {
            positions: v.to_vec(),
            normals: vec![Vector3::z(); 4],
            indices: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        };
        let err = MeshSdf::new(&inverted).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "mesh",
                    ..
                }
            ),
            "got {err}"
        );
        assert!(err.to_string().contains("outward"), "unhelpful: {err}");

        // The reversed winding of the same tetrahedron is outward: accepted.
        let outward = TriangleMesh {
            positions: v.to_vec(),
            normals: vec![Vector3::z(); 4],
            indices: vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]],
        };
        let sdf = MeshSdf::new(&outward).expect("outward tetrahedron is valid");
        assert!(sdf.eval(&Point3::origin()) < 0.0, "origin is inside");

        // Degenerate: combinatorially closed, but one vertex is collinear
        // with an edge, so two faces have zero area.
        let flat = TriangleMesh {
            positions: vec![
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.5, 0.0, 0.0), // on the edge 0-1
            ],
            normals: vec![Vector3::z(); 4],
            indices: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        };
        // (Winding may or may not pass; accept either rejection reason.)
        assert!(MeshSdf::new(&flat).is_err());
    }
}
