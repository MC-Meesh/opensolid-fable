//! Adaptive octree dual contouring with QEF vertex placement.
//!
//! The uniform mesher in [`crate::mesh`] samples the SDF at every point of a
//! dense grid. This mesher instead refines an octree over the bounding
//! region: at every level, a cell whose [`Sdf::eval_interval`] excludes zero
//! provably does not cross the surface and is pruned along with its entire
//! subtree. Only cells straddling the surface survive refinement, so the
//! number of field evaluations scales with the surface area
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
//! # Leaf-depth choice: uniform or graded
//!
//! With [`AdaptiveMeshOptions::accuracy`] unset, all surface-crossing leaves
//! live at `max_depth`: the connectivity is identical to the uniform grid —
//! guaranteeing the same watertight topology — while retaining interval
//! pruning of empty space and QEF sharp features.
//!
//! With an accuracy target set, leaf depths are graded to the local shape of
//! the surface. Refinement of a surface-crossing cell stops as soon as its
//! one-vertex model is provably within the target: the field magnitude at
//! every interpolated edge crossing and at the QEF vertex must not exceed
//! the target (a direct chordal-deviation bound for distance-like fields),
//! and the crossing normals must agree within [`FEATURE_NORMAL_DOT`].
//! Gradient discontinuities of CSG min/max fields violate the normal test,
//! so sharp feature edges and high-curvature bands refine all the way to
//! `max_depth` while flat regions stay coarse. Cells whose interval contains
//! zero but whose corners show no sign change keep subdividing: the surface
//! may cross their boundary, and a neighbor's fine stitch edge may reference
//! them, so stopping early there could leave holes.
//!
//! Mixed leaf depths are stitched crack-free by the recursive octree
//! traversal of Ju et al. (2002): `cell_proc`/`face_proc`/`edge_proc`
//! enumerate every minimal (finest) interior edge exactly once, and each
//! sign-changing minimal edge connects the vertices of its (up to four
//! distinct) adjacent leaves into a quad — or a triangle where a coarse leaf
//! spans two quadrants. Because polygons are dual (one vertex per leaf) no
//! T-junction cracks can occur, and corner signs are sampled on one global
//! finest lattice with bit-identical coordinates, so adjacent cells always
//! agree about crossings.
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

/// Graded refinement never terminates a surface cell above this depth: the
/// error estimate needs a few crossings per cell to be meaningful, and an
/// `8^3` base grid is cheap.
const MIN_GRADED_DEPTH: u32 = 3;

/// Two crossing normals with a dot product below this mark a sharp feature
/// (CSG edge, small fillet) or strong curvature crossing the cell; such
/// cells refine to `max_depth` so feature lines stay crisp. `0.9` is about
/// 26 degrees of normal spread.
const FEATURE_NORMAL_DOT: f64 = 0.9;

/// Depth above which graded octree children are built in parallel. `2`
/// yields up to 64 independent subtrees, plenty for a thread pool while
/// keeping redundant sibling corner evaluations negligible.
const GRADED_PAR_DEPTH: u32 = 2;

/// Options controlling adaptive SDF meshing.
#[derive(Debug, Clone, Copy)]
pub struct AdaptiveMeshOptions {
    /// Region to mesh. Must strictly contain the surface.
    pub bounds: BoundingBox3,
    /// Octree depth: leaf cells subdivide `bounds` into at most
    /// `2^max_depth` cells per axis. Depths much beyond 10 (1024^3 virtual
    /// cells) are impractical; the sparse representation only touches
    /// surface cells, but vertex counts still grow with `4^max_depth`.
    pub max_depth: u32,
    /// Target accuracy: maximum chordal deviation of the mesh from the
    /// surface, in model units of a distance-like field. `Some(tol)` grades
    /// leaf depth — coarse on flat regions, `max_depth` at sharp features —
    /// stopping refinement once the cell's surface model deviates by at
    /// most `tol`. `None` keeps every surface leaf at `max_depth`.
    pub accuracy: Option<f64>,
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
    if opts.bounds.is_empty() {
        return TriangleMesh::new();
    }
    match opts.accuracy {
        Some(acc) if acc.is_finite() && acc > 0.0 && opts.max_depth > MIN_GRADED_DEPTH => {
            mesh_graded(sdf, opts, acc)
        }
        _ => mesh_uniform_depth(sdf, opts),
    }
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
/// bit2 = z) of the cell at `coords` whose edge spans `1 << shift` finest
/// steps.
fn corner_key(coords: [u32; 3], shift: u32, bits: usize) -> [u32; 3] {
    let (dx, dy, dz) = corner(bits);
    [
        (coords[0] + dx as u32) << shift,
        (coords[1] + dy as u32) << shift,
        (coords[2] + dz as u32) << shift,
    ]
}

fn has_sign_change(corners: &[f64; 8]) -> bool {
    let first = corners[0] < 0.0;
    corners.iter().any(|&v| (v < 0.0) != first)
}

/// QEF minimizer over Hermite crossings (`points` with matching unit
/// `normals`; zero normals anchor the mass point but contribute no plane),
/// clamped into `cell_bounds`. Ill-conditioned QEFs can still land outside
/// the cell; clamping keeps the dual grid non-inverted enough for a
/// manifold stitch.
fn solve_qef(points: &[Vector3], normals: &[Vector3], cell_bounds: &BoundingBox3) -> Point3 {
    let mass = points.iter().sum::<Vector3>() / points.len() as f64;
    let mut ata = Matrix3::<f64>::zeros();
    let mut atb = Vector3::zeros();
    for (p, n) in points.iter().zip(normals) {
        ata += n * n.transpose();
        atb += n * n.dot(&(p - mass));
    }
    let svd = ata.svd(true, true);
    let eps = QEF_SV_TRUNCATION * svd.singular_values.max();
    let delta = svd.solve(&atb, eps).unwrap_or_else(|_| Vector3::zeros());
    let x = mass + delta;
    Point3::new(
        x.x.clamp(cell_bounds.min.x, cell_bounds.max.x),
        x.y.clamp(cell_bounds.min.y, cell_bounds.max.y),
        x.z.clamp(cell_bounds.min.z, cell_bounds.max.z),
    )
}

/// Unit-normalized SDF gradient, or `Vector3::z()` on degenerate gradients.
fn unit_grad(sdf: &dyn Sdf, p: &Point3) -> Vector3 {
    let grad = sdf.grad(p);
    let norm = grad.norm();
    if norm > 1e-12 {
        grad / norm
    } else {
        Vector3::z()
    }
}

// ---------------------------------------------------------------------------
// Uniform leaf depth (`accuracy: None`): interval-pruned descent to
// `max_depth`, then the same per-edge stitch rule as the uniform grid.
// ---------------------------------------------------------------------------

fn mesh_uniform_depth(sdf: &dyn Sdf, opts: &AdaptiveMeshOptions) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
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
        .map(|p| unit_grad(sdf, p))
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

/// Finest-lattice coordinates of corner `bits` (bit0 = x, bit1 = y,
/// bit2 = z) of the leaf cell at `cell`.
fn cell_corner(cell: [u32; 3], bits: usize) -> [u32; 3] {
    corner_key(cell, 0, bits)
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
    let mut points: Vec<Vector3> = Vec::new();
    let mut normals: Vec<Vector3> = Vec::new();
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
        points.push(p);
        // Degenerate gradient: the point still anchors the mass point but
        // contributes no plane.
        normals.push(if norm > 1e-12 {
            grad / norm
        } else {
            Vector3::zeros()
        });
    }
    if points.is_empty() {
        return None;
    }
    Some(solve_qef(&points, &normals, &g.cell_bounds(0, cell)))
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

// ---------------------------------------------------------------------------
// Graded leaf depth (`accuracy: Some`): error-driven top-down refinement
// with recursive crack-free stitching across depth transitions.
// ---------------------------------------------------------------------------

enum Node {
    /// Provably surface-free (interval pruned), or a `max_depth` cell whose
    /// corners show no sign change. No stitch edge with a sign change can
    /// abut an `Empty` node: pruned regions contain no crossing at all, and
    /// a crossingless `max_depth` cell owns every minimal edge on its
    /// boundary in its own corner set.
    Empty,
    Leaf(Box<Leaf>),
    Internal(Box<[Node; 8]>),
}

struct Leaf {
    depth: u32,
    /// Field values at the cell corners ([`corner`] bit layout).
    corners: [f64; 8],
    vertex: Point3,
    /// Position of `vertex` in the mesh, assigned by [`number_leaves`].
    index: usize,
}

fn as_leaf(node: &Node) -> Option<&Leaf> {
    match node {
        Node::Leaf(l) => Some(l),
        _ => None,
    }
}

struct GradedBuilder<'a> {
    sdf: &'a dyn Sdf,
    g: OctGrid,
    max_depth: u32,
    accuracy: f64,
}

fn mesh_graded(sdf: &dyn Sdf, opts: &AdaptiveMeshOptions, accuracy: f64) -> TriangleMesh {
    let mut mesh = TriangleMesh::new();
    if !sdf.eval_interval(&opts.bounds).contains_zero() {
        return mesh;
    }
    let builder = GradedBuilder {
        sdf,
        g: OctGrid::new(opts),
        max_depth: opts.max_depth,
        accuracy,
    };
    let root_corners: [f64; 8] = std::array::from_fn(|c| {
        sdf.eval(&builder.g.point_at(corner_key([0, 0, 0], opts.max_depth, c)))
    });
    let mut root = builder.build(0, [0, 0, 0], root_corners);

    number_leaves(&mut root, &mut mesh.positions);
    if mesh.positions.is_empty() {
        return mesh;
    }
    mesh.normals = mesh
        .positions
        .par_iter()
        .map(|p| unit_grad(sdf, p))
        .collect();
    cell_proc(&root, &mut mesh.indices);
    mesh
}

/// Assign mesh vertex indices to leaves in deterministic recursion order.
fn number_leaves(node: &mut Node, positions: &mut Vec<Point3>) {
    match node {
        Node::Empty => {}
        Node::Leaf(l) => {
            l.index = positions.len();
            positions.push(l.vertex);
        }
        Node::Internal(children) => {
            for child in children.iter_mut() {
                number_leaves(child, positions);
            }
        }
    }
}

impl GradedBuilder<'_> {
    /// Build the subtree for the cell at `coords`/`depth`, whose interval
    /// was already checked by the parent and whose corner field values are
    /// `corners`.
    fn build(&self, depth: u32, coords: [u32; 3], corners: [f64; 8]) -> Node {
        if depth == self.max_depth {
            return self.max_depth_leaf(depth, coords, corners);
        }
        if depth >= MIN_GRADED_DEPTH && has_sign_change(&corners) {
            if let Some(node) = self.try_leaf(depth, coords, corners) {
                return node;
            }
        }
        self.subdivide(depth, coords, corners)
    }

    /// Linear-interpolation surface crossings on the sign-changing edges of
    /// the cell.
    fn hit_points(&self, depth: u32, coords: [u32; 3], corners: &[f64; 8]) -> Vec<Vector3> {
        let shift = self.max_depth - depth;
        let mut points = Vec::new();
        for &(a, b) in &CELL_EDGES {
            let va = corners[a];
            let vb = corners[b];
            if (va < 0.0) == (vb < 0.0) {
                continue;
            }
            let t = va / (va - vb);
            let pa = self.g.point_at(corner_key(coords, shift, a));
            let pb = self.g.point_at(corner_key(coords, shift, b));
            points.push(pa.coords + (pb.coords - pa.coords) * t);
        }
        points
    }

    /// Unit normals at the crossing points; zero where the gradient is
    /// degenerate (those anchor the QEF mass point but contribute no plane
    /// and are skipped by the feature test).
    fn hit_normals(&self, points: &[Vector3]) -> Vec<Vector3> {
        points
            .iter()
            .map(|p| {
                let grad = self.sdf.grad(&Point3::from(*p));
                let norm = grad.norm();
                if norm > 1e-12 {
                    grad / norm
                } else {
                    Vector3::zeros()
                }
            })
            .collect()
    }

    /// Terminate refinement here if the cell's one-vertex surface model is
    /// within the accuracy target and no sharp feature crosses the cell;
    /// `None` means the cell must subdivide. Only called on sign-changing
    /// cells, so a returned leaf always carries a vertex — which the stitch
    /// relies on: any sign-changing minimal edge abutting this leaf can
    /// reference it.
    fn try_leaf(&self, depth: u32, coords: [u32; 3], corners: [f64; 8]) -> Option<Node> {
        let points = self.hit_points(depth, coords, &corners);
        debug_assert!(!points.is_empty(), "sign change must produce crossings");

        // Chordal deviation of the linear edge model: |f| at an interpolated
        // crossing measures how far the local linearization strays from the
        // true surface (cheap first: skips gradient work on cells that
        // clearly refine).
        let err = points
            .iter()
            .map(|p| self.sdf.eval(&Point3::from(*p)).abs())
            .fold(0.0, f64::max);
        if err > self.accuracy {
            return None;
        }

        // Sharp feature or strong curvature: refine to max_depth.
        let normals = self.hit_normals(&points);
        for (i, a) in normals.iter().enumerate() {
            if a.norm_squared() == 0.0 {
                continue;
            }
            for b in &normals[i + 1..] {
                if b.norm_squared() > 0.0 && a.dot(b) < FEATURE_NORMAL_DOT {
                    return None;
                }
            }
        }

        let shift = self.max_depth - depth;
        let vertex = solve_qef(&points, &normals, &self.g.cell_bounds(shift, coords));
        if self.sdf.eval(&vertex).abs() > self.accuracy {
            return None;
        }
        Some(Node::Leaf(Box::new(Leaf {
            depth,
            corners,
            vertex,
            index: usize::MAX,
        })))
    }

    /// Finest-level cell: a leaf if any edge crosses, otherwise `Empty`
    /// (conservative intervals let crossingless cells survive to the
    /// bottom, exactly like uncrossed cells of the uniform grid).
    fn max_depth_leaf(&self, depth: u32, coords: [u32; 3], corners: [f64; 8]) -> Node {
        if !has_sign_change(&corners) {
            return Node::Empty;
        }
        let points = self.hit_points(depth, coords, &corners);
        let normals = self.hit_normals(&points);
        let vertex = solve_qef(&points, &normals, &self.g.cell_bounds(0, coords));
        Node::Leaf(Box::new(Leaf {
            depth,
            corners,
            vertex,
            index: usize::MAX,
        }))
    }

    fn subdivide(&self, depth: u32, coords: [u32; 3], corners: [f64; 8]) -> Node {
        let child_shift = self.max_depth - depth - 1;
        let base = [coords[0] << 1, coords[1] << 1, coords[2] << 1];

        let children: [Node; 8] = if depth < GRADED_PAR_DEPTH {
            // Shallow cells are few: build subtrees in parallel and let each
            // child evaluate its own corners (the handful of duplicated
            // sibling evaluations is noise at this level).
            let mut built: Vec<Node> = (0..8usize)
                .into_par_iter()
                .map(|c| {
                    let (dx, dy, dz) = corner(c);
                    let cc = [
                        base[0] + dx as u32,
                        base[1] + dy as u32,
                        base[2] + dz as u32,
                    ];
                    if !self
                        .sdf
                        .eval_interval(&self.g.cell_bounds(child_shift, cc))
                        .contains_zero()
                    {
                        return Node::Empty;
                    }
                    let child_corners: [f64; 8] = std::array::from_fn(|k| {
                        self.sdf
                            .eval(&self.g.point_at(corner_key(cc, child_shift, k)))
                    });
                    self.build(depth + 1, cc, child_corners)
                })
                .collect();
            let mut drain = built.drain(..);
            std::array::from_fn(|_| drain.next().expect("eight children"))
        } else {
            // Sequential: share field samples between siblings through the
            // 3x3x3 lattice of child corners, seeded with the parent's
            // corners and filled lazily for interval-surviving children only.
            let lidx = |i: usize, j: usize, k: usize| (i * 3 + j) * 3 + k;
            let mut vals: [Option<f64>; 27] = [None; 27];
            for (c, &v) in corners.iter().enumerate() {
                let (dx, dy, dz) = corner(c);
                vals[lidx(2 * dx, 2 * dy, 2 * dz)] = Some(v);
            }
            let mut children: [Node; 8] = std::array::from_fn(|_| Node::Empty);
            for (c, child) in children.iter_mut().enumerate() {
                let (dx, dy, dz) = corner(c);
                let cc = [
                    base[0] + dx as u32,
                    base[1] + dy as u32,
                    base[2] + dz as u32,
                ];
                if !self
                    .sdf
                    .eval_interval(&self.g.cell_bounds(child_shift, cc))
                    .contains_zero()
                {
                    continue;
                }
                let child_corners: [f64; 8] = std::array::from_fn(|k| {
                    let (ex, ey, ez) = corner(k);
                    let (i, j, l) = (dx + ex, dy + ey, dz + ez);
                    *vals[lidx(i, j, l)].get_or_insert_with(|| {
                        self.sdf.eval(&self.g.point_at([
                            (base[0] + i as u32) << child_shift,
                            (base[1] + j as u32) << child_shift,
                            (base[2] + l as u32) << child_shift,
                        ]))
                    })
                });
                *child = self.build(depth + 1, cc, child_corners);
            }
            children
        };

        if children.iter().all(|c| matches!(c, Node::Empty)) {
            Node::Empty
        } else {
            Node::Internal(Box::new(children))
        }
    }
}

// --- Recursive stitching (Ju et al. 2002) ---
//
// Conventions: child/corner index bits are x = bit0, y = bit1, z = bit2
// (matching [`corner`]). For an edge parallel to axis `e`, the four
// surrounding nodes are slotted as `2*sp + sq`, where `sp`/`sq` are the
// node's side (0 = negative) along axes `(e+1)%3` and `(e+2)%3`.

fn cell_proc(node: &Node, out: &mut Vec<[usize; 3]>) {
    let Node::Internal(children) = node else {
        return;
    };
    for child in children.iter() {
        cell_proc(child, out);
    }
    for axis in 0..3 {
        let da = 1usize << axis;
        for c in 0..8 {
            if c & da == 0 {
                face_proc([&children[c], &children[c | da]], axis, out);
            }
        }
    }
    for e in 0..3usize {
        let de = 1usize << e;
        let dp = 1usize << ((e + 1) % 3);
        let dq = 1usize << ((e + 2) % 3);
        for alpha in [0, de] {
            edge_proc(
                [
                    &children[alpha],
                    &children[alpha | dq],
                    &children[alpha | dp],
                    &children[alpha | dp | dq],
                ],
                e,
                out,
            );
        }
    }
}

/// The child of an internal node, or the node itself when it is a leaf
/// spanning the whole sub-region (or `Empty`).
fn child_or_self(node: &Node, idx: usize) -> &Node {
    match node {
        Node::Internal(children) => &children[idx],
        other => other,
    }
}

/// `n[0]`/`n[1]` are on the negative/positive side of their shared face,
/// which is normal to `axis`.
fn face_proc(n: [&Node; 2], axis: usize, out: &mut Vec<[usize; 3]>) {
    if n.iter().any(|x| matches!(x, Node::Empty)) {
        return;
    }
    if !n.iter().any(|x| matches!(x, Node::Internal(_))) {
        return;
    }
    let da = 1usize << axis;
    let dp = 1usize << ((axis + 1) % 3);
    let dq = 1usize << ((axis + 2) % 3);

    // Four sub-face pairs.
    for pq in 0..4usize {
        let off = (pq >> 1) * dp + (pq & 1) * dq;
        face_proc(
            [child_or_self(n[0], da | off), child_or_self(n[1], off)],
            axis,
            out,
        );
    }

    // Four interior edges lying in the shared face: two parallel to each
    // in-plane axis `e`, split at the face center along the other in-plane
    // axis `o`.
    for e in [(axis + 1) % 3, (axis + 2) % 3] {
        let o = 3 - axis - e;
        let de = 1usize << e;
        let do_ = 1usize << o;
        for beta in [0, de] {
            let mut nodes = [n[0]; 4];
            for sa in 0..2usize {
                for so in 0..2usize {
                    // Side along `axis`: n[0]'s children touch the face from
                    // below (sa = 0), n[1]'s from above.
                    let node = if sa == 0 {
                        child_or_self(n[0], da | beta | (so * do_))
                    } else {
                        child_or_self(n[1], beta | (so * do_))
                    };
                    let (sp, sq) = if (e + 1) % 3 == axis {
                        (sa, so)
                    } else {
                        (so, sa)
                    };
                    nodes[2 * sp + sq] = node;
                }
            }
            edge_proc(nodes, e, out);
        }
    }
}

/// Four nodes around a common edge parallel to `e` (slot layout in the
/// module comment above). Recurses along the edge until all four nodes are
/// leaves, then emits the polygon for the minimal edge.
fn edge_proc(n: [&Node; 4], e: usize, out: &mut Vec<[usize; 3]>) {
    // A surface-free node has no crossing anywhere in its closure, so every
    // (sub-)edge it borders is sign-change-free: nothing to emit below here.
    if n.iter().any(|x| matches!(x, Node::Empty)) {
        return;
    }
    if let (Some(a), Some(b), Some(c), Some(d)) =
        (as_leaf(n[0]), as_leaf(n[1]), as_leaf(n[2]), as_leaf(n[3]))
    {
        process_edge([a, b, c, d], e, out);
        return;
    }
    let de = 1usize << e;
    let dp = 1usize << ((e + 1) % 3);
    let dq = 1usize << ((e + 2) % 3);
    for alpha in [0, de] {
        let m: [&Node; 4] = std::array::from_fn(|k| match n[k] {
            // The child adjacent to the common edge mirrors the node's side.
            Node::Internal(children) => &children[alpha + (1 - (k >> 1)) * dp + (1 - (k & 1)) * dq],
            leaf => leaf,
        });
        edge_proc(m, e, out);
    }
}

/// Emit the dual polygon for the minimal edge shared by four leaves. The
/// sign test reads the deepest leaf's own corners (the minimal edge is one
/// of its lattice edges); a coarse leaf spanning two quadrants appears
/// twice and degenerates the quad into a triangle. Winding matches
/// [`edge_quad`]: the quad `(0,0),(1,0),(1,1),(0,1)` over the perpendicular
/// axes faces `+e`, reversed when the surface faces `-e`.
fn process_edge(leaves: [&Leaf; 4], e: usize, out: &mut Vec<[usize; 3]>) {
    let deepest = (0..4)
        .max_by_key(|&k| leaves[k].depth)
        .expect("four leaves");
    let mut bits = [0usize; 3];
    bits[(e + 1) % 3] = 1 - (deepest >> 1);
    bits[(e + 2) % 3] = 1 - (deepest & 1);
    let idx0 = bits[0] | bits[1] << 1 | bits[2] << 2;
    bits[e] = 1;
    let idx1 = bits[0] | bits[1] << 1 | bits[2] << 2;
    let v0 = leaves[deepest].corners[idx0];
    let v1 = leaves[deepest].corners[idx1];
    let inside0 = v0 < 0.0;
    if inside0 == (v1 < 0.0) {
        return;
    }
    let mut quad = [
        leaves[0].index,
        leaves[2].index,
        leaves[3].index,
        leaves[1].index,
    ];
    if !inside0 {
        quad.reverse();
    }
    for tri in [[quad[0], quad[1], quad[2]], [quad[0], quad[2], quad[3]]] {
        if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
            out.push(tri);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blend::SmoothUnion;
    use crate::csg::{Subtraction, Union};
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

    /// Worst field magnitude over triangle vertices, edge midpoints, and
    /// centroids: a dense probe of the mesh's chordal deviation from the
    /// zero set of a distance-like field.
    fn max_chordal_deviation(sdf: &dyn Sdf, mesh: &TriangleMesh) -> f64 {
        let mut worst: f64 = 0.0;
        for tri in &mesh.indices {
            let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
            for q in [
                a,
                b,
                c,
                (a + b) / 2.0,
                (b + c) / 2.0,
                (a + c) / 2.0,
                (a + b + c) / 3.0,
            ] {
                worst = worst.max(sdf.eval(&Point3::from(q)).abs());
            }
        }
        worst
    }

    #[test]
    fn sphere_mesh_watertight() {
        let s = unit_sphere();
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.6),
            max_depth: 5,
            accuracy: None,
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
                accuracy: None,
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
                accuracy: None,
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
            accuracy: None,
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
            accuracy: None,
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
            accuracy: None,
        };
        assert!(mesh_sdf_adaptive(&s, &opts).is_empty());
    }

    #[test]
    fn zero_depth_returns_empty() {
        let opts = AdaptiveMeshOptions {
            bounds: bounds(2.0),
            max_depth: 0,
            accuracy: None,
        };
        assert!(mesh_sdf_adaptive(&unit_sphere(), &opts).is_empty());
    }

    #[test]
    fn empty_bounds_returns_empty() {
        let opts = AdaptiveMeshOptions {
            bounds: BoundingBox3::EMPTY,
            max_depth: 4,
            accuracy: None,
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
            accuracy: None,
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
            accuracy: None,
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

    // --- Graded refinement ---

    /// A sphere is curved everywhere, so an unreachable accuracy target
    /// forces every surface cell to max_depth: the recursive stitcher must
    /// then reproduce the uniform grid's topology exactly, validating the
    /// cross-level traversal against the proven flat stitch.
    #[test]
    fn graded_forced_fine_matches_uniform_topology() {
        let s = unit_sphere();
        let graded = mesh_sdf_adaptive_indexed(
            &s,
            &AdaptiveMeshOptions {
                bounds: bounds(1.6),
                max_depth: 5,
                accuracy: Some(1e-12),
            },
        );
        let uniform = mesh_sdf_adaptive_indexed(
            &s,
            &AdaptiveMeshOptions {
                bounds: bounds(1.6),
                max_depth: 5,
                accuracy: None,
            },
        );
        assert_eq!(graded.triangle_count(), uniform.triangle_count());
        assert_eq!(graded.vertex_count(), uniform.vertex_count());
        assert!(graded.is_closed_manifold());
    }

    /// Graded meshing must hit the accuracy target with far fewer triangles
    /// than uniform leaf depth at the same max_depth: the sphere's curvature
    /// satisfies a 1e-2 target several levels above the floor.
    #[test]
    fn graded_sphere_meets_accuracy_with_fewer_triangles() {
        let s = unit_sphere();
        let acc = 0.01;
        let graded = mesh_sdf_adaptive_indexed(
            &s,
            &AdaptiveMeshOptions {
                bounds: bounds(1.6),
                max_depth: 7,
                accuracy: Some(acc),
            },
        );
        assert!(!graded.is_empty());
        assert!(graded.is_closed_manifold());
        let dev = max_chordal_deviation(&s, &graded);
        assert!(dev <= 2.0 * acc, "deviation {dev} exceeds 2x target {acc}");

        let uniform = mesh_sdf_adaptive_indexed(
            &s,
            &AdaptiveMeshOptions {
                bounds: bounds(1.6),
                max_depth: 7,
                accuracy: None,
            },
        );
        assert!(
            graded.triangle_count() * 4 < uniform.triangle_count(),
            "graded {} vs uniform {} triangles: expected >4x savings",
            graded.triangle_count(),
            uniform.triangle_count()
        );
    }

    /// Union of two boxes: flat faces must stay coarse, the sharp feature
    /// edges must refine and stay crisp (recovered extents land on the true
    /// faces), and the whole mesh must stay watertight across the coarse-to
    /// -fine transitions.
    #[test]
    fn graded_box_union_crisp_edges_fewer_triangles() {
        let shape = Union {
            a: Box3 {
                center: Point3::origin(),
                half_extents: [1.0, 0.4, 0.6],
            },
            b: Box3 {
                center: Point3::origin(),
                half_extents: [0.5, 0.9, 0.5],
            },
        };
        let acc = 0.005;
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.4),
            max_depth: 7,
            accuracy: Some(acc),
        };
        let mesh = mesh_sdf_adaptive_indexed(&shape, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let dev = max_chordal_deviation(&shape, &mesh);
        assert!(dev <= 2.0 * acc, "deviation {dev} exceeds 2x target {acc}");

        // Crisp features: the union's outer extents are recovered far
        // tighter than the finest cell (2.8 / 128 = 0.022).
        let bb = mesh.bounding_box().unwrap();
        for (got, want) in [
            (bb.max.x, 1.0),
            (bb.max.y, 0.9),
            (bb.max.z, 0.6),
            (bb.min.x, -1.0),
            (bb.min.y, -0.9),
            (bb.min.z, -0.6),
        ] {
            assert!(
                (got - want).abs() < 2e-3,
                "extent {got}, want {want} (sharp feature smeared)"
            );
        }

        let uniform = mesh_sdf_adaptive_indexed(
            &shape,
            &AdaptiveMeshOptions {
                accuracy: None,
                ..opts
            },
        );
        // Feature bands are one-dimensional (they double per level while a
        // uniform surface quadruples), so the ratio grows with depth:
        // measured 4.1x at depth 7, 10x at depth 8.
        assert!(
            mesh.triangle_count() * 3 < uniform.triangle_count(),
            "graded {} vs uniform {} triangles: expected >3x savings",
            mesh.triangle_count(),
            uniform.triangle_count()
        );
    }

    /// Smooth blends have no sharp edges but a curved blend band: graded
    /// meshing must meet the accuracy target and stay manifold.
    #[test]
    fn graded_smooth_blend_within_accuracy() {
        let shape = SmoothUnion {
            a: unit_sphere(),
            b: Box3 {
                center: Point3::new(0.9, 0.0, 0.0),
                half_extents: [0.6, 0.6, 0.6],
            },
            radius: 0.3,
        };
        let acc = 0.01;
        let mesh = mesh_sdf_adaptive_indexed(
            &shape,
            &AdaptiveMeshOptions {
                bounds: BoundingBox3::new(
                    Point3::new(-1.6, -1.6, -1.6),
                    Point3::new(2.1, 1.6, 1.6),
                ),
                max_depth: 7,
                accuracy: Some(acc),
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let dev = max_chordal_deviation(&shape, &mesh);
        assert!(dev <= 2.0 * acc, "deviation {dev} exceeds 2x target {acc}");
    }

    /// Graded CSG subtraction (curved feature edges from the drilled
    /// cylinder) must stay watertight across depth transitions.
    #[test]
    fn graded_subtraction_manifold() {
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
        let acc = 0.01;
        let mesh = mesh_sdf_adaptive_indexed(
            &shape,
            &AdaptiveMeshOptions {
                bounds: bounds(1.7),
                max_depth: 6,
                accuracy: Some(acc),
            },
        );
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let dev = max_chordal_deviation(&shape, &mesh);
        assert!(dev <= 2.0 * acc, "deviation {dev} exceeds 2x target {acc}");
    }

    /// Flat-dominated scenes must also save field evaluations, not just
    /// triangles: early leaf termination skips the fine levels entirely
    /// outside the feature bands.
    #[test]
    fn graded_box_union_samples_fewer_points_than_uniform_depth() {
        let make = || Union {
            a: Box3 {
                center: Point3::origin(),
                half_extents: [1.0, 0.4, 0.6],
            },
            b: Box3 {
                center: Point3::origin(),
                half_extents: [0.5, 0.9, 0.5],
            },
        };
        let graded = CountingSdf {
            inner: make(),
            evals: AtomicUsize::new(0),
        };
        let uniform = CountingSdf {
            inner: make(),
            evals: AtomicUsize::new(0),
        };
        let opts = AdaptiveMeshOptions {
            bounds: bounds(1.4),
            max_depth: 8,
            accuracy: Some(0.005),
        };
        mesh_sdf_adaptive_indexed(&graded, &opts);
        mesh_sdf_adaptive_indexed(
            &uniform,
            &AdaptiveMeshOptions {
                accuracy: None,
                ..opts
            },
        );
        let g = graded.evals.load(Ordering::Relaxed);
        let u = uniform.evals.load(Ordering::Relaxed);
        // Graded descent pays per-level corner samples and error probes, so
        // the eval win needs depth to amortize: measured 2.4x at depth 8
        // (and slightly *worse* than uniform at depth 6 — that is expected).
        assert!(
            g * 2 < u,
            "graded sampled {g} points, uniform depth sampled {u}: expected >2x savings"
        );
    }

    /// Degenerate accuracy targets must fall back to uniform depth rather
    /// than panic or grade nonsensically.
    #[test]
    fn degenerate_accuracy_falls_back_to_uniform() {
        let s = unit_sphere();
        let uniform = mesh_sdf_adaptive_indexed(
            &s,
            &AdaptiveMeshOptions {
                bounds: bounds(1.6),
                max_depth: 4,
                accuracy: None,
            },
        );
        for acc in [Some(0.0), Some(-1.0), Some(f64::NAN), Some(f64::INFINITY)] {
            let mesh = mesh_sdf_adaptive_indexed(
                &s,
                &AdaptiveMeshOptions {
                    bounds: bounds(1.6),
                    max_depth: 4,
                    accuracy: acc,
                },
            );
            assert_eq!(mesh.triangle_count(), uniform.triangle_count());
        }
    }
}
