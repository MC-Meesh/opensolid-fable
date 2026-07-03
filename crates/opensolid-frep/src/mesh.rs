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

/// A triangle with per-vertex positions and outward unit normals.
#[derive(Debug, Clone)]
pub struct Triangle {
    pub positions: [Point3; 3],
    pub normals: [Vector3; 3],
}

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
    let mesh = build_mesh(sdf, opts);
    mesh.indices
        .iter()
        .map(|tri| Triangle {
            positions: [
                mesh.positions[tri[0]],
                mesh.positions[tri[1]],
                mesh.positions[tri[2]],
            ],
            normals: [
                mesh.normals[tri[0]],
                mesh.normals[tri[1]],
                mesh.normals[tri[2]],
            ],
        })
        .collect()
}

/// Indexed mesh: shared vertices referenced by triangle index triples.
struct IndexedMesh {
    positions: Vec<Point3>,
    normals: Vec<Vector3>,
    indices: Vec<[usize; 3]>,
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

fn build_mesh(sdf: &dyn Sdf, opts: &MeshOptions) -> IndexedMesh {
    let n = opts.resolution;
    let mut mesh = IndexedMesh {
        positions: Vec::new(),
        normals: Vec::new(),
        indices: Vec::new(),
    };
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
    use std::collections::HashMap;

    fn bounds(half: f64) -> BoundingBox3 {
        BoundingBox3::new(
            Point3::new(-half, -half, -half),
            Point3::new(half, half, half),
        )
    }

    /// Every undirected edge must be shared by exactly two triangles with
    /// opposite orientations — i.e. the mesh is closed and consistently wound.
    fn assert_closed_manifold(indices: &[[usize; 3]]) {
        let mut edges: HashMap<(usize, usize), (usize, i64)> = HashMap::new();
        for tri in indices {
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                assert_ne!(a, b, "degenerate triangle edge");
                let key = (a.min(b), a.max(b));
                let dir = if a < b { 1 } else { -1 };
                let entry = edges.entry(key).or_insert((0, 0));
                entry.0 += 1;
                entry.1 += dir;
            }
        }
        for (edge, (count, dir_sum)) in &edges {
            assert_eq!(*count, 2, "edge {edge:?} used {count} times, want 2");
            assert_eq!(*dir_sum, 0, "edge {edge:?} not consistently oriented");
        }
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
        let mesh = build_mesh(&s, &opts);
        assert!(!mesh.indices.is_empty());
        assert_closed_manifold(&mesh.indices);
        let cell = 3.2 / 20.0;
        for p in &mesh.positions {
            assert!(s.eval(p).abs() < cell, "vertex {p:?} too far from surface");
        }
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
        let mesh = build_mesh(&b, &opts);
        assert!(!mesh.indices.is_empty());
        assert_closed_manifold(&mesh.indices);

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
        let mesh = build_mesh(&union, &opts);
        assert!(!mesh.indices.is_empty());
        assert_closed_manifold(&mesh.indices);
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
