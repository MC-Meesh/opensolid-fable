//! Dual contouring mesher: converts any [`Sdf`] into a triangle mesh.
//!
//! The algorithm samples the SDF on a uniform grid over a bounding region,
//! finds grid edges where the sign changes (surface crossings), places one
//! vertex per crossed cell using Hermite data (linear interpolation of SDF
//! values along crossed edges, normals from the SDF gradient), and connects
//! vertices of the four cells around each crossed edge into a quad.
//!
//! The surface must lie strictly inside `bounds`; crossings on the boundary
//! layer of cells are not stitched and would leave holes.

use crate::eval::gradient;
use crate::primitives::Sdf;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};

pub use opensolid_core::mesh::{Triangle, TriangleMesh};

/// Options controlling SDF meshing.
#[derive(Debug, Clone, Copy)]
pub struct MeshOptions {
    /// Region to mesh. Must strictly contain the surface.
    pub bounds: BoundingBox3,
    /// Number of grid cells along each axis.
    pub resolution: usize,
}

/// Mesh an SDF into triangles using dual contouring on a uniform grid.
///
/// Returns an empty vec if the surface does not cross the sampled region
/// or if `resolution` is zero.
pub fn mesh_sdf(sdf: &dyn Sdf, opts: &MeshOptions) -> Vec<Triangle> {
    mesh_sdf_indexed(sdf, opts).to_triangles()
}

/// Mesh an SDF into an indexed [`TriangleMesh`] using dual contouring on a
/// uniform grid. Vertices are shared between adjacent triangles.
///
/// Returns an empty mesh if the surface does not cross the sampled region
/// or if `resolution` is zero.
pub fn mesh_sdf_indexed(sdf: &dyn Sdf, opts: &MeshOptions) -> TriangleMesh {
    build_mesh(sdf, opts)
}

/// Cell edges as pairs of corner indices. Corner bit layout: bit0 = x,
/// bit1 = y, bit2 = z.
const CELL_EDGES: [(usize, usize); 12] = [
    (0, 1),
    (2, 3),
    (4, 5),
    (6, 7), // x-aligned
    (0, 2),
    (1, 3),
    (4, 6),
    (5, 7), // y-aligned
    (0, 4),
    (1, 5),
    (2, 6),
    (3, 7), // z-aligned
];

fn build_mesh(sdf: &dyn Sdf, opts: &MeshOptions) -> TriangleMesh {
    let n = opts.resolution;
    let mut mesh = TriangleMesh::new();
    if n == 0 {
        return mesh;
    }

    let np = n + 1;
    let min = opts.bounds.min;
    let size = opts.bounds.max - opts.bounds.min;
    let nf = n as f64;
    let step = Vector3::new(size.x / nf, size.y / nf, size.z / nf);

    let point_at = |i: usize, j: usize, k: usize| {
        Point3::new(
            min.x + step.x * i as f64,
            min.y + step.y * j as f64,
            min.z + step.z * k as f64,
        )
    };
    let vidx = |i: usize, j: usize, k: usize| (i * np + j) * np + k;
    let cidx = |i: usize, j: usize, k: usize| (i * n + j) * n + k;
    let corner = |c: usize| (c & 1, (c >> 1) & 1, (c >> 2) & 1);

    // Sample the SDF at every grid point.
    let mut values = vec![0.0f64; np * np * np];
    for i in 0..np {
        for j in 0..np {
            for k in 0..np {
                values[vidx(i, j, k)] = sdf.eval(&point_at(i, j, k));
            }
        }
    }

    // Place one vertex per cell that has a sign change: the centroid of the
    // Hermite edge crossings (linear interpolation of the SDF along each
    // crossed cell edge).
    let mut cell_vertex: Vec<Option<usize>> = vec![None; n * n * n];
    for i in 0..n {
        for j in 0..n {
            for k in 0..n {
                let mut sum = Vector3::zeros();
                let mut count = 0usize;
                for &(a, b) in &CELL_EDGES {
                    let (ax, ay, az) = corner(a);
                    let (bx, by, bz) = corner(b);
                    let va = values[vidx(i + ax, j + ay, k + az)];
                    let vb = values[vidx(i + bx, j + by, k + bz)];
                    if (va < 0.0) == (vb < 0.0) {
                        continue;
                    }
                    let t = va / (va - vb);
                    let pa = point_at(i + ax, j + ay, k + az);
                    let pb = point_at(i + bx, j + by, k + bz);
                    sum += pa.coords + (pb.coords - pa.coords) * t;
                    count += 1;
                }
                if count > 0 {
                    cell_vertex[cidx(i, j, k)] = Some(mesh.positions.len());
                    mesh.positions.push(Point3::from(sum / count as f64));
                }
            }
        }
    }

    for p in &mesh.positions {
        let g = gradient(sdf, p);
        let norm = g.norm();
        mesh.normals
            .push(if norm > 1e-12 { g / norm } else { Vector3::z() });
    }

    // For each interior grid edge with a sign change, connect the vertices of
    // the four cells sharing that edge into a quad (two triangles). Winding:
    // with perpendicular axes u = (d+1)%3 and v = (d+2)%3, the quad order
    // (0,0),(1,0),(1,1),(0,1) faces +d; reversed when the surface faces -d.
    for d in 0..3 {
        let u = (d + 1) % 3;
        let v = (d + 2) % 3;
        for gd in 0..n {
            for gu in 1..n {
                for gv in 1..n {
                    let mut g0 = [0usize; 3];
                    g0[d] = gd;
                    g0[u] = gu;
                    g0[v] = gv;
                    let mut g1 = g0;
                    g1[d] += 1;
                    let v0 = values[vidx(g0[0], g0[1], g0[2])];
                    let v1 = values[vidx(g1[0], g1[1], g1[2])];
                    let inside0 = v0 < 0.0;
                    if inside0 == (v1 < 0.0) {
                        continue;
                    }
                    let cell = |a: usize, b: usize| {
                        let mut c = g0;
                        c[u] = c[u] - 1 + a;
                        c[v] = c[v] - 1 + b;
                        cell_vertex[cidx(c[0], c[1], c[2])]
                            .expect("cell adjacent to a sign-change edge must have a vertex")
                    };
                    let mut quad = [cell(0, 0), cell(1, 0), cell(1, 1), cell(0, 1)];
                    if !inside0 {
                        quad.reverse();
                    }
                    mesh.indices.push([quad[0], quad[1], quad[2]]);
                    mesh.indices.push([quad[0], quad[2], quad[3]]);
                }
            }
        }
    }

    mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg::Union;
    use crate::primitives::{Box3, Sphere};

    fn bounds(half: f64) -> BoundingBox3 {
        BoundingBox3::new(
            Point3::new(-half, -half, -half),
            Point3::new(half, half, half),
        )
    }

    fn total_area(tris: &[Triangle]) -> f64 {
        tris.iter()
            .map(|t| {
                let e1 = t.positions[1] - t.positions[0];
                let e2 = t.positions[2] - t.positions[0];
                e1.cross(&e2).norm() * 0.5
            })
            .sum()
    }

    #[test]
    fn sphere_mesh_watertight() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = MeshOptions {
            bounds: bounds(1.6),
            resolution: 20,
        };
        let mesh = mesh_sdf_indexed(&s, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let cell = 3.2 / 20.0;
        for p in &mesh.positions {
            assert!(s.eval(p).abs() < cell, "vertex {p:?} too far from surface");
        }
    }

    /// Welding the triangle soup from `mesh_sdf` must reconstruct a closed
    /// manifold that agrees with the indexed mesh from `mesh_sdf_indexed`.
    #[test]
    fn welded_soup_agrees_with_indexed_sphere_mesh() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = MeshOptions {
            bounds: bounds(1.6),
            resolution: 20,
        };
        let indexed = mesh_sdf_indexed(&s, &opts);
        let soup = mesh_sdf(&s, &opts);
        assert_eq!(soup.len(), indexed.triangle_count());

        let welded = TriangleMesh::from_triangles(&soup).weld(1e-12);
        assert!(welded.is_closed_manifold());
        assert_eq!(welded.triangle_count(), indexed.triangle_count());
        // Every vertex the indexed mesh actually references must survive
        // the soup round-trip as exactly one welded vertex.
        let referenced: std::collections::HashSet<usize> =
            indexed.indices.iter().flatten().copied().collect();
        assert_eq!(welded.vertex_count(), referenced.len());
        assert!((welded.total_area() - indexed.total_area()).abs() < 1e-9);
    }

    #[test]
    fn sphere_normals_outward_and_area_correct() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = MeshOptions {
            bounds: bounds(1.6),
            resolution: 20,
        };
        let tris = mesh_sdf(&s, &opts);
        assert!(!tris.is_empty());
        for t in &tris {
            for (p, nrm) in t.positions.iter().zip(&t.normals) {
                assert!((nrm.norm() - 1.0).abs() < 1e-9, "normal not unit length");
                let radial = p.coords.normalize();
                assert!(nrm.dot(&radial) > 0.9, "vertex normal not outward");
            }
            // Winding must agree with the SDF gradient (outward).
            let e1 = t.positions[1] - t.positions[0];
            let e2 = t.positions[2] - t.positions[0];
            let geo = e1.cross(&e2);
            let centroid =
                (t.positions[0].coords + t.positions[1].coords + t.positions[2].coords) / 3.0;
            assert!(geo.dot(&centroid) > 0.0, "triangle wound inward");
        }
        let area = total_area(&tris);
        let expected = 4.0 * std::f64::consts::PI;
        assert!(
            (area - expected).abs() / expected < 0.15,
            "sphere area {area} vs expected {expected}"
        );
    }

    #[test]
    fn box_mesh_faces_correct() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 1.0, 1.0],
        };
        let opts = MeshOptions {
            bounds: bounds(1.55),
            resolution: 24,
        };
        let mesh = mesh_sdf_indexed(&b, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());

        let cell = 3.1 / 24.0;
        let mut lo = Point3::new(f64::MAX, f64::MAX, f64::MAX);
        let mut hi = Point3::new(f64::MIN, f64::MIN, f64::MIN);
        for p in &mesh.positions {
            assert!(b.eval(p).abs() < cell, "vertex {p:?} off the box surface");
            lo = Point3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
            hi = Point3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
        }
        // Mesh extents must recover the box faces at x/y/z = ±1.
        for (l, h) in [(lo.x, hi.x), (lo.y, hi.y), (lo.z, hi.z)] {
            assert!((l + 1.0).abs() < cell, "min face at {l}, want -1");
            assert!((h - 1.0).abs() < cell, "max face at {h}, want 1");
        }

        let tris = mesh_sdf(&b, &opts);
        let area = total_area(&tris);
        let expected = 24.0; // 6 faces of a 2x2 box
        assert!(
            (area - expected).abs() / expected < 0.15,
            "box area {area} vs expected {expected}"
        );
    }

    #[test]
    fn csg_union_mesh_manifold() {
        let union = Union {
            a: Sphere {
                center: Point3::new(-0.5, 0.0, 0.0),
                radius: 1.0,
            },
            b: Sphere {
                center: Point3::new(0.5, 0.0, 0.0),
                radius: 1.0,
            },
        };
        let opts = MeshOptions {
            bounds: bounds(1.8),
            resolution: 24,
        };
        let mesh = mesh_sdf_indexed(&union, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let cell = 3.6 / 24.0;
        for p in &mesh.positions {
            assert!(union.eval(p).abs() < cell, "vertex {p:?} off the surface");
        }
    }

    #[test]
    fn empty_when_surface_outside_bounds() {
        let s = Sphere {
            center: Point3::new(10.0, 10.0, 10.0),
            radius: 1.0,
        };
        let opts = MeshOptions {
            bounds: bounds(1.0),
            resolution: 8,
        };
        assert!(mesh_sdf(&s, &opts).is_empty());
    }

    #[test]
    fn zero_resolution_returns_empty() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = MeshOptions {
            bounds: bounds(2.0),
            resolution: 0,
        };
        assert!(mesh_sdf(&s, &opts).is_empty());
    }
}
