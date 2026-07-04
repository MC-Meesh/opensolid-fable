//! Adaptive octree dual contouring with QEF vertex placement.
//!
//! The uniform mesher in [`crate::mesh`] samples the SDF at every point of a
//! dense grid. This mesher instead refines an octree over the bounding
//! region: at every level, a cell whose [`Sdf::eval_interval`] excludes zero
//! provably does not cross the surface and is pruned along with its entire
//! subtree. Only cells straddling the surface survive to the target depth,
//! so the number of field evaluations scales with the surface area
//! (`O(4^depth)`) instead of the volume (`O(8^depth)`).
//!
//! Each surviving leaf places one vertex by minimizing a quadratic error
//! function (QEF) over the cell's Hermite data: for every sign-changing cell
//! edge, the crossing point `p_i` (linear interpolation of the field) and
//! the unit normal `n_i` (field gradient at the crossing) define a tangent
//! plane, and the vertex minimizes `sum_i (n_i . (x - p_i))^2`. The normal
//! equations are solved through an SVD pseudo-inverse with relative
//! singular-value truncation, seeded at the mass point of the crossings, so
//! flat regions stay stable while edges and corners of the surface attract
//! the vertex exactly onto the sharp feature — the classic dual-contouring
//! advantage over plain crossing-centroid placement.
//!
//! # Leaf-depth choice: uniform, not graded
//!
//! All surface-crossing leaves live at `max_depth`. Mixing leaf depths
//! requires either stitching polygons across octree levels or grading the
//! tree and special-casing transition faces; both add crack-prone
//! connectivity logic. Restricting to uniform leaf depth (the trivially
//! balanced octree) keeps the connectivity identical to the uniform grid —
//! guaranteeing the same watertight topology — while retaining the two wins
//! that motivate the octree: interval pruning of empty space and QEF sharp
//! features. Graded leaves with cross-level stitching can be layered on
//! later without changing this module's public surface.
//!
//! The surface must lie strictly inside `bounds`; crossings on the boundary
//! layer of cells are not stitched and would leave holes.

use std::collections::HashMap;

use crate::mesh::{CELL_EDGES, corner};
use crate::primitives::Sdf;
use nalgebra::Matrix3;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use rayon::prelude::*;

pub use opensolid_core::mesh::{Triangle, TriangleMesh};

/// Fraction of the largest singular value below which QEF singular values
/// are truncated. Rank-deficient normal matrices (faces: rank 1, edges:
/// rank 2) then resolve to the minimum-norm solution about the mass point
/// instead of shooting the vertex along the ill-determined direction.
const QEF_SV_TRUNCATION: f64 = 0.1;

/// Options controlling adaptive SDF meshing.
#[derive(Debug, Clone, Copy)]
pub struct AdaptiveMeshOptions {
    /// Region to mesh. Must strictly contain the surface.
    pub bounds: BoundingBox3,
    /// Octree depth: leaf cells subdivide `bounds` into `2^max_depth` cells
    /// per axis. Depths much beyond 10 (1024^3 virtual cells) are
    /// impractical; the sparse representation only touches surface cells,
    /// but vertex counts still grow with `4^max_depth`.
    pub max_depth: u32,
}

/// Mesh an SDF into triangles using adaptive octree dual contouring.
///
/// Returns an empty vec if the surface does not cross the bounded region or
/// if `max_depth` is zero (a single cell has no interior edges to stitch).
pub fn mesh_sdf_adaptive(sdf: &dyn Sdf, opts: &AdaptiveMeshOptions) -> Vec<Triangle> {
    mesh_sdf_adaptive_indexed(sdf, opts).to_triangles()
}

/// Mesh an SDF into an indexed [`TriangleMesh`] using adaptive octree dual
/// contouring with QEF vertex placement. Vertices are shared between
/// adjacent triangles.
pub fn mesh_sdf_adaptive_indexed(sdf: &dyn Sdf, opts: &AdaptiveMeshOptions) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if opts.bounds.is_empty() {
        return mesh;
    }
    let g = OctGrid::new(opts);

    // Phase 1: octree refinement. Leaves arrive in Morton-ish recursion
    // order; sort into scan order so all later numbering is deterministic.
    let mut leaves: Vec<[u32; 3]> = Vec::new();
    collect_leaves(sdf, &g, 0, opts.max_depth, [0, 0, 0], &mut leaves);
    leaves.sort_unstable();
    if leaves.is_empty() {
        return mesh;
    }

    // Phase 2: sample the field once per unique leaf corner, in parallel.
    let mut corner_keys: Vec<[u32; 3]> = leaves
        .iter()
        .flat_map(|&c| (0..8).map(move |b| cell_corner(c, b)))
        .collect();
    corner_keys.sort_unstable();
    corner_keys.dedup();
    let corner_values: HashMap<[u32; 3], f64> = corner_keys
        .par_iter()
        .map(|&key| (key, sdf.eval(&g.point_at(key))))
        .collect();

    // Phase 3: one QEF vertex per leaf that has sign-changing edges. A leaf
    // can survive pruning without a crossing (intervals are conservative);
    // such cells get no vertex, exactly like uncrossed cells of the uniform
    // grid.
    let candidates: Vec<Option<Point3>> = leaves
        .par_iter()
        .map(|&cell| qef_vertex(sdf, &g, &corner_values, cell))
        .collect();
    let mut cell_vertex: HashMap<[u32; 3], usize> = HashMap::new();
    for (cell, candidate) in leaves.iter().zip(candidates) {
        if let Some(p) = candidate {
            cell_vertex.insert(*cell, mesh.positions.len());
            mesh.positions.push(p);
        }
    }

    mesh.normals = mesh
        .positions
        .par_iter()
        .map(|p| {
            let grad = sdf.grad(p);
            let norm = grad.norm();
            if norm > 1e-12 {
                grad / norm
            } else {
                Vector3::z()
            }
        })
        .collect();

    // Phase 4: stitch. Every interior sign-changing grid edge connects the
    // vertices of its four surrounding cells into a quad, exactly the
    // uniform mesher's rule restricted to edges of surviving leaves.
    let mut edges: Vec<(usize, [u32; 3])> = leaves
        .iter()
        .flat_map(|&cell| {
            CELL_EDGES.iter().enumerate().map(move |(e, _)| {
                let axis = e / 4; // CELL_EDGES groups four edges per axis
                (axis, cell_corner(cell, CELL_EDGES[e].0))
            })
        })
        .collect();
    edges.sort_unstable();
    edges.dedup();
    let per_edge: Vec<Option<[[usize; 3]; 2]>> = edges
        .par_iter()
        .map(|&(d, e0)| edge_quad(&g, &corner_values, &cell_vertex, d, e0))
        .collect();
    for tris in per_edge.into_iter().flatten() {
        mesh.indices.extend(tris);
    }

    mesh
}

/// Lattice geometry at the finest octree level: `n = 2^max_depth` cells per
/// axis over `bounds`.
struct OctGrid {
    n: u32,
    min: Point3,
    step: Vector3,
}

impl OctGrid {
    fn new(opts: &AdaptiveMeshOptions) -> Self {
        let n = 1u32 << opts.max_depth;
        let size = opts.bounds.max - opts.bounds.min;
        let nf = n as f64;
        Self {
            n,
            min: opts.bounds.min,
            step: Vector3::new(size.x / nf, size.y / nf, size.z / nf),
        }
    }

    fn point_at(&self, key: [u32; 3]) -> Point3 {
        Point3::new(
            self.min.x + self.step.x * key[0] as f64,
            self.min.y + self.step.y * key[1] as f64,
            self.min.z + self.step.z * key[2] as f64,
        )
    }

    /// Bounds of the octree cell at `coords` whose edge spans `1 << shift`
    /// finest-lattice steps. Corners are computed from lattice coordinates,
    /// so parent and child boxes share bit-identical corner points.
    fn cell_bounds(&self, shift: u32, coords: [u32; 3]) -> BoundingBox3 {
        let lo = coords.map(|c| c << shift);
        let hi = coords.map(|c| (c + 1) << shift);
        BoundingBox3::new(self.point_at(lo), self.point_at(hi))
    }
}

/// Finest-lattice coordinates of corner `bits` (bit0 = x, bit1 = y,
/// bit2 = z) of the leaf cell at `cell`.
fn cell_corner(cell: [u32; 3], bits: usize) -> [u32; 3] {
    let (dx, dy, dz) = corner(bits);
    [
        cell[0] + dx as u32,
        cell[1] + dy as u32,
        cell[2] + dz as u32,
    ]
}

/// Depth-first octree refinement. Prunes any cell whose field interval
/// excludes zero; pushes surviving cells at `max_depth` as leaves.
fn collect_leaves(
    sdf: &dyn Sdf,
    g: &OctGrid,
    depth: u32,
    max_depth: u32,
    coords: [u32; 3],
    out: &mut Vec<[u32; 3]>,
) {
    let bounds = g.cell_bounds(max_depth - depth, coords);
    if !sdf.eval_interval(&bounds).contains_zero() {
        return;
    }
    if depth == max_depth {
        out.push(coords);
        return;
    }
    for child in 0..8 {
        let (dx, dy, dz) = corner(child);
        let next = [
            2 * coords[0] + dx as u32,
            2 * coords[1] + dy as u32,
            2 * coords[2] + dz as u32,
        ];
        collect_leaves(sdf, g, depth + 1, max_depth, next, out);
    }
}

/// QEF vertex for one leaf cell, or `None` if no cell edge crosses the
/// surface. Crossing points come from linear interpolation of the corner
/// values; normals from the SDF gradient at each crossing.
fn qef_vertex(
    sdf: &dyn Sdf,
    g: &OctGrid,
    values: &HashMap<[u32; 3], f64>,
    cell: [u32; 3],
) -> Option<Point3> {
    let mut crossings: Vec<(Vector3, Vector3)> = Vec::new(); // (point, unit normal)
    for &(a, b) in &CELL_EDGES {
        let ka = cell_corner(cell, a);
        let kb = cell_corner(cell, b);
        let va = values[&ka];
        let vb = values[&kb];
        if (va < 0.0) == (vb < 0.0) {
            continue;
        }
        let t = va / (va - vb);
        let pa = g.point_at(ka);
        let pb = g.point_at(kb);
        let p = pa.coords + (pb.coords - pa.coords) * t;
        let grad = sdf.grad(&Point3::from(p));
        let norm = grad.norm();
        if norm > 1e-12 {
            crossings.push((p, grad / norm));
        } else {
            // Degenerate gradient: the point still anchors the mass point
            // but contributes no plane.
            crossings.push((p, Vector3::zeros()));
        }
    }
    if crossings.is_empty() {
        return None;
    }

    let mass = crossings.iter().map(|(p, _)| p).sum::<Vector3>() / crossings.len() as f64;
    let mut ata = Matrix3::<f64>::zeros();
    let mut atb = Vector3::zeros();
    for (p, n) in &crossings {
        ata += n * n.transpose();
        atb += n * n.dot(&(p - mass));
    }
    let svd = ata.svd(true, true);
    let eps = QEF_SV_TRUNCATION * svd.singular_values.max();
    let delta = svd.solve(&atb, eps).unwrap_or_else(|_| Vector3::zeros());

    // Ill-conditioned QEFs can still land outside the cell; clamping keeps
    // the dual grid non-inverted enough for a manifold stitch.
    let cell_bounds = g.cell_bounds(0, cell);
    let x = mass + delta;
    Some(Point3::new(
        x.x.clamp(cell_bounds.min.x, cell_bounds.max.x),
        x.y.clamp(cell_bounds.min.y, cell_bounds.max.y),
        x.z.clamp(cell_bounds.min.z, cell_bounds.max.z),
    ))
}

/// Two triangles for the interior grid edge along axis `d` starting at
/// lattice point `e0`, or `None` if the edge has no sign change or lies on
/// the boundary layer. Winding matches the uniform mesher: the quad
/// (0,0),(1,0),(1,1),(0,1) over the perpendicular axes faces +d, reversed
/// when the surface faces -d.
fn edge_quad(
    g: &OctGrid,
    values: &HashMap<[u32; 3], f64>,
    cell_vertex: &HashMap<[u32; 3], usize>,
    d: usize,
    e0: [u32; 3],
) -> Option<[[usize; 3]; 2]> {
    let u = (d + 1) % 3;
    let v = (d + 2) % 3;
    if e0[u] == 0 || e0[u] >= g.n || e0[v] == 0 || e0[v] >= g.n {
        return None;
    }
    let mut e1 = e0;
    e1[d] += 1;
    let v0 = values[&e0];
    let v1 = values[&e1];
    let inside0 = v0 < 0.0;
    if inside0 == (v1 < 0.0) {
        return None;
    }
    let cell = |a: u32, b: u32| {
        let mut c = e0;
        c[u] = c[u] - 1 + a;
        c[v] = c[v] - 1 + b;
        // A sign change on this edge puts both signs in every adjacent
        // cell's corner set, so eval_interval must contain zero for all
        // four: none was pruned and each has a crossing, hence a vertex.
        *cell_vertex
            .get(&c)
            .expect("cell adjacent to a sign-change edge must have a vertex")
    };
    let mut quad = [cell(0, 0), cell(1, 0), cell(1, 1), cell(0, 1)];
    if !inside0 {
        quad.reverse();
    }
    Some([[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]])
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg::Subtraction;
    use crate::mesh::{MeshOptions, mesh_sdf_indexed};
    use crate::primitives::{Box3, Cylinder, Sphere};
    use opensolid_core::interval::Interval;
    use std::sync::atomic::{AtomicUsize, Ordering};

    fn bounds(half: f64) -> BoundingBox3 {
        BoundingBox3::new(
            Point3::new(-half, -half, -half),
            Point3::new(half, half, half),
        )
    }

    fn unit_sphere() -> Sphere {
        Sphere {
            center: Point3::origin(),
            radius: 1.0,
        }
    }

    fn max_abs_eval(sdf: &dyn Sdf, mesh: &TriangleMesh) -> f64 {
        mesh.positions
            .iter()
            .map(|p| sdf.eval(p).abs())
            .fold(0.0, f64::max)
    }

    #[test]
    fn sphere_mesh_watertight() {
        let s = unit_sphere();
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.6),
            max_depth: 5,
        };
        let mesh = mesh_sdf_adaptive_indexed(&s, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let cell = 3.2 / 32.0;
        for p in &mesh.positions {
            assert!(s.eval(p).abs() < cell, "vertex {p:?} too far from surface");
        }
    }

    /// With uniform leaf depth the stitch rule is identical to the uniform
    /// grid's, so pruning must not change the topology: same triangle count
    /// as the dense mesher at the matching resolution.
    #[test]
    fn topology_matches_uniform_grid() {
        let s = unit_sphere();
        let adaptive = mesh_sdf_adaptive_indexed(
            &s,
            &AdaptiveMeshOptions {
                bounds: bounds(1.6),
                max_depth: 4,
            },
        );
        let uniform = mesh_sdf_indexed(
            &s,
            &MeshOptions {
                bounds: bounds(1.6),
                resolution: 16,
            },
        );
        assert_eq!(adaptive.triangle_count(), uniform.triangle_count());
        assert_eq!(adaptive.vertex_count(), uniform.vertex_count());
    }

    /// QEF vertex placement must recover the box's sharp edges: at an equal
    /// triangle budget (same effective resolution, hence same connectivity),
    /// the adaptive mesh's worst vertex-to-surface distance must beat the
    /// uniform mesher's crossing-centroid placement decisively.
    #[test]
    fn box_edges_sharper_than_uniform_dc() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 1.0, 1.0],
        };
        let region = bounds(1.55);
        let adaptive = mesh_sdf_adaptive_indexed(
            &b,
            &AdaptiveMeshOptions {
                bounds: region,
                max_depth: 4,
            },
        );
        let uniform = mesh_sdf_indexed(
            &b,
            &MeshOptions {
                bounds: region,
                resolution: 16,
            },
        );
        // Equal budget: identical connectivity, only vertex placement moves.
        assert_eq!(adaptive.triangle_count(), uniform.triangle_count());
        assert!(adaptive.is_closed_manifold());

        let qef_err = max_abs_eval(&b, &adaptive);
        let centroid_err = max_abs_eval(&b, &uniform);
        let cell = 3.1 / 16.0;
        // Centroid placement smears cells that straddle an edge of the box;
        // its worst error is a sizable fraction of a cell. QEF intersects
        // the adjacent face planes and lands on the feature.
        assert!(
            qef_err < 0.25 * centroid_err,
            "QEF err {qef_err} not decisively sharper than centroid err {centroid_err}"
        );
        assert!(
            qef_err < 0.05 * cell,
            "QEF err {qef_err} should be far below a cell ({cell})"
        );

        // The recovered extents must land on the true faces at +/-1 much
        // tighter than one cell.
        let bb = adaptive.bounding_box().unwrap();
        for (lo, hi) in [
            (bb.min.x, bb.max.x),
            (bb.min.y, bb.max.y),
            (bb.min.z, bb.max.z),
        ] {
            assert!((lo + 1.0).abs() < 0.05 * cell, "min face at {lo}, want -1");
            assert!((hi - 1.0).abs() < 0.05 * cell, "max face at {hi}, want 1");
        }
    }

    #[test]
    fn csg_subtraction_mesh_manifold() {
        let shape = Subtraction {
            a: Box3 {
                center: Point3::origin(),
                half_extents: [1.0, 1.0, 1.0],
            },
            b: Cylinder {
                center: Point3::origin(),
                radius: 0.5,
                half_height: 1.5,
            },
        };
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.7),
            max_depth: 5,
        };
        let mesh = mesh_sdf_adaptive_indexed(&shape, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let cell = 3.4 / 32.0;
        for p in &mesh.positions {
            assert!(
                shape.eval(p).abs() < cell,
                "vertex {p:?} too far from surface"
            );
        }
    }

    /// Wraps an SDF and counts `eval` calls, delegating `grad` and
    /// `eval_interval` so only corner sampling is measured.
    struct CountingSdf<T> {
        inner: T,
        evals: AtomicUsize,
    }

    impl<T: Sdf> Sdf for CountingSdf<T> {
        fn eval(&self, p: &Point3) -> f64 {
            self.evals.fetch_add(1, Ordering::Relaxed);
            self.inner.eval(p)
        }

        fn grad(&self, p: &Point3) -> Vector3 {
            self.inner.grad(p)
        }

        fn eval_interval(&self, b: &BoundingBox3) -> Interval {
            self.inner.eval_interval(b)
        }
    }

    /// Interval pruning must keep field sampling near the surface: far
    /// fewer point evaluations than the dense grid the uniform mesher needs.
    #[test]
    fn pruning_samples_far_fewer_points_than_dense_grid() {
        let s = CountingSdf {
            inner: unit_sphere(),
            evals: AtomicUsize::new(0),
        };
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.6),
            max_depth: 5,
        };
        let mesh = mesh_sdf_adaptive_indexed(&s, &opts);
        assert!(mesh.is_closed_manifold());
        let dense = 33usize.pow(3); // (2^5 + 1)^3 grid points
        let sampled = s.evals.load(Ordering::Relaxed);
        assert!(
            sampled < dense / 3,
            "pruning sampled {sampled} points, dense grid would be {dense}"
        );
    }

    #[test]
    fn empty_when_surface_outside_bounds() {
        let s = Sphere {
            center: Point3::new(10.0, 10.0, 10.0),
            radius: 1.0,
        };
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.0),
            max_depth: 4,
        };
        assert!(mesh_sdf_adaptive(&s, &opts).is_empty());
    }

    #[test]
    fn zero_depth_returns_empty() {
        let opts = AdaptiveMeshOptions {
            bounds: bounds(2.0),
            max_depth: 0,
        };
        assert!(mesh_sdf_adaptive(&unit_sphere(), &opts).is_empty());
    }

    #[test]
    fn empty_bounds_returns_empty() {
        let opts = AdaptiveMeshOptions {
            bounds: BoundingBox3::EMPTY,
            max_depth: 4,
        };
        assert!(mesh_sdf_adaptive(&unit_sphere(), &opts).is_empty());
    }

    /// Triangle-soup output must agree with the indexed mesh after welding,
    /// mirroring the uniform mesher's soup/indexed contract.
    #[test]
    fn welded_soup_agrees_with_indexed_mesh() {
        let s = unit_sphere();
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.6),
            max_depth: 4,
        };
        let indexed = mesh_sdf_adaptive_indexed(&s, &opts);
        let soup = mesh_sdf_adaptive(&s, &opts);
        assert_eq!(soup.len(), indexed.triangle_count());
        let welded = TriangleMesh::from_triangles(&soup).weld(1e-12);
        assert!(welded.is_closed_manifold());
        assert_eq!(welded.triangle_count(), indexed.triangle_count());
    }

    #[test]
    fn sphere_normals_outward() {
        let s = unit_sphere();
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.6),
            max_depth: 4,
        };
        let mesh = mesh_sdf_adaptive_indexed(&s, &opts);
        for (p, nrm) in mesh.positions.iter().zip(&mesh.normals) {
            assert!((nrm.norm() - 1.0).abs() < 1e-9, "normal not unit length");
            assert!(nrm.dot(&p.coords.normalize()) > 0.9, "normal not outward");
        }
        // Winding must agree with the outward normals.
        for tri in mesh.to_triangles() {
            let e1 = tri.positions[1] - tri.positions[0];
            let e2 = tri.positions[2] - tri.positions[0];
            let centroid =
                (tri.positions[0].coords + tri.positions[1].coords + tri.positions[2].coords) / 3.0;
            assert!(e1.cross(&e2).dot(&centroid) > 0.0, "triangle wound inward");
        }
    }
}
