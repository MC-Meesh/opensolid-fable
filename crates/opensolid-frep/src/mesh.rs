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
//!
//! Grid cells are kept near-cubic regardless of the aspect ratio of
//! `bounds`: [`MeshOptions::resolution`] sets the cell count along the
//! longest axis and the other axes get proportionally fewer cells.
//! Strongly anisotropic cells alias near-tangent surface regions into
//! ambiguous faces (all four edges of one grid face crossed), which dual
//! contouring cannot stitch into a manifold — four quads would share one
//! mesh edge (of-6f8).

use crate::eval::gradient;
use crate::primitives::Sdf;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use rayon::prelude::*;

pub use opensolid_core::mesh::{Triangle, TriangleMesh};

/// Options controlling SDF meshing.
#[derive(Debug, Clone, Copy)]
pub struct MeshOptions {
    /// Region to mesh. Must strictly contain the surface, with at least one
    /// grid cell of clearance on every side (roughly `longest extent /
    /// resolution`).
    pub bounds: BoundingBox3,
    /// Number of grid cells along the longest axis of `bounds`. Shorter
    /// axes get proportionally fewer cells so cells stay near-cubic; for
    /// cubic bounds every axis gets exactly `resolution` cells.
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
/// bit1 = y, bit2 = z. Shared with the adaptive mesher.
pub(crate) const CELL_EDGES: [(usize, usize); 12] = [
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

/// Geometry shared by the meshing passes. Cell counts are per-axis: the
/// longest axis gets `resolution` cells, shorter axes proportionally fewer
/// (at least one), so cells stay near-cubic on anisotropic bounds.
struct Grid {
    /// Cells along x, y, z.
    n: [usize; 3],
    /// Grid points along x, y, z (`n + 1`).
    np: [usize; 3],
    min: Point3,
    step: Vector3,
}

impl Grid {
    /// `None` if the grid is unusable: zero resolution, or bounds whose
    /// longest extent is not a positive finite number.
    fn new(opts: &MeshOptions) -> Option<Self> {
        let res = opts.resolution;
        let size = opts.bounds.max - opts.bounds.min;
        let longest = size.x.max(size.y).max(size.z);
        if res == 0 || longest <= 0.0 || !longest.is_finite() {
            return None;
        }
        let cells = |extent: f64| ((res as f64 * extent / longest).round() as usize).max(1);
        let n = [cells(size.x), cells(size.y), cells(size.z)];
        Some(Self {
            n,
            np: [n[0] + 1, n[1] + 1, n[2] + 1],
            min: opts.bounds.min,
            step: Vector3::new(
                size.x / n[0] as f64,
                size.y / n[1] as f64,
                size.z / n[2] as f64,
            ),
        })
    }

    fn point_at(&self, i: usize, j: usize, k: usize) -> Point3 {
        Point3::new(
            self.min.x + self.step.x * i as f64,
            self.min.y + self.step.y * j as f64,
            self.min.z + self.step.z * k as f64,
        )
    }

    fn vidx(&self, i: usize, j: usize, k: usize) -> usize {
        (i * self.np[1] + j) * self.np[2] + k
    }

    fn cidx(&self, i: usize, j: usize, k: usize) -> usize {
        (i * self.n[1] + j) * self.n[2] + k
    }
}

pub(crate) fn corner(c: usize) -> (usize, usize, usize) {
    (c & 1, (c >> 1) & 1, (c >> 2) & 1)
}

/// Hermite vertex for one cell: centroid of the sign-change crossings on its
/// edges (linear interpolation of the SDF along each crossed edge), or `None`
/// if no edge crosses the surface.
fn cell_candidate(g: &Grid, values: &[f64], i: usize, j: usize, k: usize) -> Option<Point3> {
    let mut sum = Vector3::zeros();
    let mut count = 0usize;
    for &(a, b) in &CELL_EDGES {
        let (ax, ay, az) = corner(a);
        let (bx, by, bz) = corner(b);
        let va = values[g.vidx(i + ax, j + ay, k + az)];
        let vb = values[g.vidx(i + bx, j + by, k + bz)];
        if (va < 0.0) == (vb < 0.0) {
            continue;
        }
        let t = va / (va - vb);
        let pa = g.point_at(i + ax, j + ay, k + az);
        let pb = g.point_at(i + bx, j + by, k + bz);
        sum += pa.coords + (pb.coords - pa.coords) * t;
        count += 1;
    }
    (count > 0).then(|| Point3::from(sum / count as f64))
}

/// Triangles for one `(d, gd)` slab of the quad-emission pass: for each
/// interior grid edge along axis `d` at depth `gd` with a sign change,
/// connect the vertices of the four cells sharing that edge. Winding: with
/// perpendicular axes u = (d+1)%3 and v = (d+2)%3, the quad order
/// (0,0),(1,0),(1,1),(0,1) faces +d; reversed when the surface faces -d.
fn slab_triangles(
    g: &Grid,
    values: &[f64],
    cell_vertex: &[Option<usize>],
    d: usize,
    gd: usize,
) -> Vec<[usize; 3]> {
    let u = (d + 1) % 3;
    let v = (d + 2) % 3;
    let mut tris = Vec::new();
    for gu in 1..g.n[u] {
        for gv in 1..g.n[v] {
            let mut g0 = [0usize; 3];
            g0[d] = gd;
            g0[u] = gu;
            g0[v] = gv;
            let mut g1 = g0;
            g1[d] += 1;
            let v0 = values[g.vidx(g0[0], g0[1], g0[2])];
            let v1 = values[g.vidx(g1[0], g1[1], g1[2])];
            let inside0 = v0 < 0.0;
            if inside0 == (v1 < 0.0) {
                continue;
            }
            let cell = |a: usize, b: usize| {
                let mut c = g0;
                c[u] = c[u] - 1 + a;
                c[v] = c[v] - 1 + b;
                cell_vertex[g.cidx(c[0], c[1], c[2])]
                    .expect("cell adjacent to a sign-change edge must have a vertex")
            };
            let mut quad = [cell(0, 0), cell(1, 0), cell(1, 1), cell(0, 1)];
            if !inside0 {
                quad.reverse();
            }
            tris.push([quad[0], quad[1], quad[2]]);
            tris.push([quad[0], quad[2], quad[3]]);
        }
    }
    tris
}

/// Parallel mesher. Every pass fans out over independent x-slabs (or
/// vertices) and preserves the serial iteration order, so the output is
/// identical to [`build_mesh_serial`] triangle-for-triangle.
fn build_mesh(sdf: &dyn Sdf, opts: &MeshOptions) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let Some(g) = Grid::new(opts) else {
        return mesh;
    };
    let g = &g;
    let [nx, ny, nz] = g.n;
    let [_, npy, npz] = g.np;

    // Sample the SDF at every grid point, one parallel task per x-slab
    // (each slab is a contiguous run of the i-major value array).
    let mut values = vec![0.0f64; g.np[0] * npy * npz];
    values
        .par_chunks_mut(npy * npz)
        .enumerate()
        .for_each(|(i, slab)| {
            for j in 0..npy {
                for k in 0..npz {
                    slab[j * npz + k] = sdf.eval(&g.point_at(i, j, k));
                }
            }
        });

    // Compute cell vertices in parallel, then number them in serial scan
    // order so vertex indices match the serial mesher exactly.
    let mut candidates: Vec<Option<Point3>> = vec![None; nx * ny * nz];
    candidates
        .par_chunks_mut(ny * nz)
        .enumerate()
        .for_each(|(i, slab)| {
            for j in 0..ny {
                for k in 0..nz {
                    slab[j * nz + k] = cell_candidate(g, &values, i, j, k);
                }
            }
        });
    let mut cell_vertex: Vec<Option<usize>> = vec![None; nx * ny * nz];
    for (c, candidate) in candidates.into_iter().enumerate() {
        if let Some(p) = candidate {
            cell_vertex[c] = Some(mesh.positions.len());
            mesh.positions.push(p);
        }
    }

    // Normals per vertex; parallel collect preserves order.
    mesh.normals = mesh
        .positions
        .par_iter()
        .map(|p| {
            let grad = gradient(sdf, p);
            let norm = grad.norm();
            if norm > 1e-12 {
                grad / norm
            } else {
                Vector3::z()
            }
        })
        .collect();

    // Emit quads per (axis, depth) slab in parallel, concatenated in the
    // serial loop order.
    let slabs: Vec<(usize, usize)> = (0..3)
        .flat_map(|d| (0..g.n[d]).map(move |gd| (d, gd)))
        .collect();
    let per_slab: Vec<Vec<[usize; 3]>> = slabs
        .par_iter()
        .map(|&(d, gd)| slab_triangles(g, &values, &cell_vertex, d, gd))
        .collect();
    for tris in per_slab {
        mesh.indices.extend(tris);
    }

    mesh
}

/// Single-threaded reference implementation. Kept (test-only) as the ground
/// truth the parallel mesher must reproduce exactly.
#[cfg(test)]
fn build_mesh_serial(sdf: &dyn Sdf, opts: &MeshOptions) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    let Some(g) = Grid::new(opts) else {
        return mesh;
    };
    let g = &g;
    let [nx, ny, nz] = g.n;
    let [npx, npy, npz] = g.np;

    let mut values = vec![0.0f64; npx * npy * npz];
    for i in 0..npx {
        for j in 0..npy {
            for k in 0..npz {
                values[g.vidx(i, j, k)] = sdf.eval(&g.point_at(i, j, k));
            }
        }
    }

    let mut cell_vertex: Vec<Option<usize>> = vec![None; nx * ny * nz];
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..nz {
                if let Some(p) = cell_candidate(g, &values, i, j, k) {
                    cell_vertex[g.cidx(i, j, k)] = Some(mesh.positions.len());
                    mesh.positions.push(p);
                }
            }
        }
    }

    for p in &mesh.positions {
        let grad = gradient(sdf, p);
        let norm = grad.norm();
        mesh.normals.push(if norm > 1e-12 {
            grad / norm
        } else {
            Vector3::z()
        });
    }

    for d in 0..3 {
        for gd in 0..g.n[d] {
            mesh.indices
                .extend(slab_triangles(g, &values, &cell_vertex, d, gd));
        }
    }

    mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blend::SmoothSubtraction;
    use crate::csg::Union;
    use crate::primitives::{Box3, Cylinder, Sphere, Torus};

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

    /// The parallel mesher must reproduce the serial reference bit-for-bit:
    /// same vertices in the same order, same normals, same triangles.
    fn assert_parallel_matches_serial(sdf: &dyn Sdf, opts: &MeshOptions) {
        let parallel = mesh_sdf_indexed(sdf, opts);
        let serial = build_mesh_serial(sdf, opts);
        assert!(!serial.is_empty());
        assert_eq!(parallel.positions, serial.positions);
        assert_eq!(parallel.normals, serial.normals);
        assert_eq!(parallel.indices, serial.indices);
    }

    #[test]
    fn parallel_matches_serial_sphere() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = MeshOptions {
            bounds: bounds(1.6),
            resolution: 20,
        };
        assert_parallel_matches_serial(&s, &opts);
    }

    #[test]
    fn parallel_matches_serial_csg_union() {
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
        assert_parallel_matches_serial(&union, &opts);
    }

    /// Regression for of-d62/of-9ht: the blend-factor sign bug made the
    /// SmoothSubtraction field negative at infinity, so the mesher saw sign
    /// crossings on the bounds boundary layer and produced unstitched
    /// boundary edges (not a closed manifold, bad-edge count scaling with
    /// resolution).
    #[test]
    fn smooth_subtraction_mesh_manifold() {
        let shape = SmoothSubtraction {
            a: Box3 {
                center: Point3::origin(),
                half_extents: [1.0, 1.0, 1.0],
            },
            b: Cylinder {
                center: Point3::origin(),
                radius: 0.4,
                half_height: 1.5,
            },
            radius: 0.2,
        };
        let opts = MeshOptions {
            bounds: bounds(1.8),
            resolution: 24,
        };
        // The field must be positive at the sampling-region corners; with
        // the sign bug the whole far field was "inside".
        for sx in [-1.8, 1.8] {
            for sy in [-1.8, 1.8] {
                for sz in [-1.8, 1.8] {
                    assert!(shape.eval(&Point3::new(sx, sy, sz)) > 0.0);
                }
            }
        }
        let mesh = mesh_sdf_indexed(&shape, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let cell = 3.6 / 24.0;
        for p in &mesh.positions {
            assert!(shape.eval(p).abs() < cell, "vertex {p:?} off the surface");
        }
    }

    /// The torus in the x-z plane with its tight bounds dilated per-axis:
    /// a strongly anisotropic sampling region (roughly 5x1x5).
    fn torus_and_tight_bounds(margin_frac: f64) -> (Torus, BoundingBox3) {
        let t = Torus {
            center: Point3::origin(),
            major_radius: 2.0,
            minor_radius: 0.5,
        };
        let half = Vector3::new(2.5, 0.5, 2.5) * (1.0 + 2.0 * margin_frac);
        let b = BoundingBox3::new(Point3::from(-half), Point3::from(half));
        (t, b)
    }

    /// Regression for of-6f8: meshing over tight anisotropic bounds used to
    /// give cells with a ~5:1 aspect ratio, which aliases near-tangent
    /// surface regions into ambiguous faces (all four edges of a grid face
    /// crossed) — the four quads around such a face share one mesh edge, so
    /// the output had edges used by four triangles and was not manifold.
    /// Cells are now kept near-cubic, so tight bounds must mesh closed.
    #[test]
    fn torus_tight_anisotropic_bounds_manifold() {
        // (margin, resolution) pairs that produced 4-triangle edges before
        // the fix, plus surrounding combinations that happened to pass.
        for (margin_frac, resolution) in [
            (0.02, 32),
            (0.02, 48),
            (0.05, 64),
            (0.10, 32),
            (0.20, 32),
            (0.20, 48),
        ] {
            let (t, bounds) = torus_and_tight_bounds(margin_frac);
            let mesh = mesh_sdf_indexed(&t, &MeshOptions { bounds, resolution });
            assert!(
                !mesh.is_empty(),
                "empty mesh at ({margin_frac}, {resolution})"
            );
            assert!(
                mesh.is_closed_manifold(),
                "not a closed manifold at margin {margin_frac}, resolution {resolution}"
            );
            // Vertices must still sit within one (near-cubic) cell of the
            // surface: anisotropy handling must not cost accuracy.
            let size = bounds.max - bounds.min;
            let cell = size.x.max(size.y).max(size.z) / resolution as f64;
            let diagonal = cell * 3.0f64.sqrt();
            for p in &mesh.positions {
                assert!(
                    t.eval(p).abs() < diagonal,
                    "vertex {p:?} too far from surface at ({margin_frac}, {resolution})"
                );
            }
        }
    }

    #[test]
    fn parallel_matches_serial_anisotropic_torus() {
        let (t, bounds) = torus_and_tight_bounds(0.1);
        let opts = MeshOptions {
            bounds,
            resolution: 32,
        };
        assert_parallel_matches_serial(&t, &opts);
    }

    /// Anisotropic bounds must yield near-cubic cells: `resolution` cells
    /// along the longest axes, proportionally fewer along the short one.
    #[test]
    fn anisotropic_bounds_get_near_cubic_cells() {
        let (t, bounds) = torus_and_tight_bounds(0.1);
        let opts = MeshOptions {
            bounds,
            resolution: 32,
        };
        let mesh = mesh_sdf_indexed(&t, &opts);
        // The tight torus bounds are 6x1.2x6, so y gets round(32/5) = 6
        // cells of size 0.2 and no mesh vertex may sit outside the y slab
        // reachable by interior cells.
        let size = bounds.max - bounds.min;
        assert!((size.y / 6.0 - size.x / 32.0) / (size.x / 32.0) < 0.1);
        for p in &mesh.positions {
            assert!(p.y.abs() <= 0.6, "vertex {p:?} outside the y extent");
        }
    }

    #[test]
    fn degenerate_bounds_return_empty() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        // Inverted (max < min) and point-sized bounds must not panic.
        for b in [
            BoundingBox3::new(Point3::new(1.0, 1.0, 1.0), Point3::new(-1.0, -1.0, -1.0)),
            BoundingBox3::new(Point3::origin(), Point3::origin()),
        ] {
            let opts = MeshOptions {
                bounds: b,
                resolution: 8,
            };
            assert!(mesh_sdf(&s, &opts).is_empty());
        }
    }

    /// A zero-extent axis gets one cell rather than dividing by zero; the
    /// surface cannot be strictly interior, so the mesh is empty but sane.
    #[test]
    fn flat_bounds_do_not_panic() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = MeshOptions {
            bounds: BoundingBox3::new(Point3::new(-2.0, 0.0, -2.0), Point3::new(2.0, 0.0, 2.0)),
            resolution: 8,
        };
        assert!(mesh_sdf(&s, &opts).is_empty());
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
