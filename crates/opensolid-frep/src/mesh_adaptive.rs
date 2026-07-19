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
//! Each surviving leaf places one vertex per surface component by minimizing
//! a quadratic error function (QEF) over the component's Hermite data: for
//! every sign-changing cell edge, the crossing point `p_i` (linear
//! interpolation of the field) and the unit normal `n_i` (field gradient at
//! the crossing) define a tangent plane, and the vertex minimizes
//! `sum_i (n_i . (x - p_i))^2`. The normal equations are solved through an
//! SVD pseudo-inverse with relative singular-value truncation, seeded at the
//! mass point of the crossings, so flat regions stay stable while edges and
//! corners of the surface attract the vertex exactly onto the sharp feature —
//! the classic dual-contouring advantage over plain crossing-centroid
//! placement.
//!
//! A cell's surface components are the connected groups of its crossed edges
//! ([`classify_components`], shared with the uniform mesher), traced across
//! the cell faces. A single vertex per cell cannot represent two surface
//! sheets crossing one cell — e.g. the band beside a CSG crease where both
//! surfaces pass through a cell that does not contain the crease itself — and
//! would pinch them together into non-manifold four-triangle edges (of-54d).
//! Placing one vertex per component and routing each stitch edge to the
//! component that owns it keeps such bands manifold. Multi-component cells in
//! the graded octree always refine to `max_depth` (their crossing normals
//! disagree, tripping the feature test), so only finest-level leaves ever
//! carry more than one vertex.
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
//! sign-changing minimal edge connects the owning component vertex of its
//! (up to four distinct) adjacent leaves into a quad — or a triangle where a
//! coarse leaf spans two quadrants. Because polygons are dual (one vertex per
//! leaf component) no T-junction cracks can occur, and corner signs are
//! sampled on one global finest lattice with bit-identical coordinates, so
//! adjacent cells always agree about crossings.
//!
//! The surface must lie strictly inside `bounds`; crossings on the boundary
//! layer of cells are not stitched and would leave holes.

use std::collections::HashMap;
use std::sync::Mutex;

use crate::mesh::{CELL_EDGES, NO_COMP, classify_components, corner};
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

/// Perpendicular axes to `e` in increasing order. Within the group of four
/// [`CELL_EDGES`] slots parallel to `e`, the slot of the edge at
/// perpendicular position `pos` is `4*e + pos[p] + 2*pos[q]` — the same
/// layout the uniform mesher's stitch uses, so the two meshers index cell
/// edges identically.
fn perp_axes(e: usize) -> (usize, usize) {
    match e {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    }
}

/// Crossing point and unit gradient normal for each sign-changing cell edge
/// (`None` on uncrossed edges). The crossing is the linear interpolation of
/// the corner field values; `corner_pt(bits)` gives the world position of
/// corner `bits`. A degenerate gradient yields a zero normal, which anchors
/// the QEF mass point but contributes no plane.
fn edge_crossings(
    sdf: &dyn Sdf,
    corners: &[f64; 8],
    corner_pt: impl Fn(usize) -> Point3,
) -> [Option<(Vector3, Vector3)>; 12] {
    let mut out = [None; 12];
    for (e, &(a, b)) in CELL_EDGES.iter().enumerate() {
        if (corners[a] < 0.0) == (corners[b] < 0.0) {
            continue;
        }
        let t = corners[a] / (corners[a] - corners[b]);
        let pa = corner_pt(a).coords;
        let pb = corner_pt(b).coords;
        let p = pa + (pb - pa) * t;
        let grad = sdf.grad(&Point3::from(p));
        let norm = grad.norm();
        let n = if norm > 1e-12 {
            grad / norm
        } else {
            Vector3::zeros()
        };
        out[e] = Some((p, n));
    }
    out
}

/// One QEF vertex per surface component: for each component `c` in
/// `0..count`, solve [`solve_qef`] over the crossings whose edge belongs to
/// `c`. `crossings[e]` must be `Some` for every edge with
/// `comp_of_edge[e] == c` (guaranteed when both come from the same corner
/// values). Vertices land in `[0..count]`.
fn component_qef(
    comp_of_edge: &[u8; 12],
    count: u8,
    crossings: &[Option<(Vector3, Vector3)>; 12],
    cell_bounds: &BoundingBox3,
) -> [Point3; 4] {
    let mut out = [Point3::origin(); 4];
    for (c, slot) in out.iter_mut().enumerate().take(count as usize) {
        let mut points: Vec<Vector3> = Vec::new();
        let mut normals: Vec<Vector3> = Vec::new();
        for (e, cross) in crossings.iter().enumerate() {
            if comp_of_edge[e] as usize == c {
                let (p, n) = cross.expect("component edge must carry a crossing");
                points.push(p);
                normals.push(n);
            }
        }
        *slot = solve_qef(&points, &normals, cell_bounds);
    }
    out
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

    // Phase 3: one QEF vertex per surface component of each leaf that has
    // sign-changing edges. A leaf can survive pruning without a crossing
    // (intervals are conservative); such cells get no vertex, exactly like
    // uncrossed cells of the uniform grid. Two-sheet cells get one vertex per
    // sheet so the stitch never pinches them (of-54d).
    let candidates: Vec<Option<CellComponents>> = leaves
        .par_iter()
        .map(|&cell| cell_components(sdf, &g, &corner_values, cell))
        .collect();
    let mut cell_vertex: HashMap<[u32; 3], LeafRef> = HashMap::new();
    for (cell, candidate) in leaves.iter().zip(candidates) {
        if let Some(cc) = candidate {
            let base = mesh.positions.len();
            mesh.positions
                .extend_from_slice(&cc.points[..cc.count as usize]);
            cell_vertex.insert(
                *cell,
                LeafRef {
                    base,
                    comp_of_edge: cc.comp_of_edge,
                },
            );
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

/// Per-component QEF vertices for one leaf cell.
struct CellComponents {
    /// Component id per [`CELL_EDGES`] slot ([`NO_COMP`] on uncrossed edges).
    comp_of_edge: [u8; 12],
    /// Number of surface components (1..=4).
    count: u8,
    /// One QEF vertex per component, in `[0..count]`.
    points: [Point3; 4],
}

/// Numbered dual vertices for one leaf: component `c`'s vertex is mesh
/// position `base + c`, and a crossed edge slot maps to its component through
/// `comp_of_edge`.
struct LeafRef {
    base: usize,
    comp_of_edge: [u8; 12],
}

/// Per-component QEF vertices for one leaf cell, or `None` if no cell edge
/// crosses the surface. Crossing points come from linear interpolation of the
/// corner values; normals from the SDF gradient at each crossing; components
/// from [`classify_components`].
fn cell_components(
    sdf: &dyn Sdf,
    g: &OctGrid,
    values: &HashMap<[u32; 3], f64>,
    cell: [u32; 3],
) -> Option<CellComponents> {
    let corners: [f64; 8] = std::array::from_fn(|c| values[&cell_corner(cell, c)]);
    if !has_sign_change(&corners) {
        return None;
    }
    let (comp_of_edge, count) = classify_components(&corners);
    let crossings = edge_crossings(sdf, &corners, |b| g.point_at(cell_corner(cell, b)));
    let points = component_qef(&comp_of_edge, count, &crossings, &g.cell_bounds(0, cell));
    Some(CellComponents {
        comp_of_edge,
        count,
        points,
    })
}

/// Two triangles for the interior grid edge along axis `d` starting at
/// lattice point `e0`, or `None` if the edge has no sign change or lies on
/// the boundary layer. Each surrounding cell contributes the vertex of the
/// surface component that owns this edge, so two-sheet cells no longer pinch.
/// Winding matches the uniform mesher: the quad (0,0),(1,0),(1,1),(0,1) over
/// the perpendicular axes faces +d, reversed when the surface faces -d.
fn edge_quad(
    g: &OctGrid,
    values: &HashMap<[u32; 3], f64>,
    cell_vertex: &HashMap<[u32; 3], LeafRef>,
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
    let (p, q) = perp_axes(d);
    let cell = |a: u32, b: u32| {
        let mut c = e0;
        c[u] = c[u] - 1 + a;
        c[v] = c[v] - 1 + b;
        // A sign change on this edge puts both signs in every adjacent
        // cell's corner set, so eval_interval must contain zero for all
        // four: none was pruned and each has a crossing, hence a vertex.
        let lr = cell_vertex
            .get(&c)
            .expect("cell adjacent to a sign-change edge must have a vertex");
        // This grid edge's slot within the cell's CELL_EDGES, then the
        // component that owns it.
        let mut delta = [0usize; 3];
        delta[u] = 1 - a as usize;
        delta[v] = 1 - b as usize;
        let slot = 4 * d + delta[p] + 2 * delta[q];
        let comp = lr.comp_of_edge[slot];
        debug_assert_ne!(comp, NO_COMP, "crossed edge missing from its cell");
        lr.base + comp as usize
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
    /// Component id per [`CELL_EDGES`] slot ([`NO_COMP`] on uncrossed edges).
    /// Only meaningful for `count > 1`; single-component leaves always use
    /// their one vertex regardless of slot.
    comp_of_edge: [u8; 12],
    /// Number of surface components (1..=4). Coarse (non-`max_depth`) leaves
    /// are always 1; only finest leaves can hold two sheets.
    count: u8,
    /// One QEF vertex per component, in `[0..count]`.
    vertices: [Point3; 4],
    /// Mesh position of each component vertex, assigned by [`number_leaves`].
    indices: [usize; 4],
}

impl Leaf {
    /// Mesh vertex index of the component owning the minimal edge parallel to
    /// `e` on side `k` (the slot layout of [`edge_proc`]). Single-component
    /// leaves ignore the slot; multi-component leaves (always finest, so the
    /// minimal edge is one of their own cell edges) look the owner up.
    fn comp_index(&self, e: usize, k: usize) -> usize {
        if self.count == 1 {
            return self.indices[0];
        }
        let (p, q) = perp_axes(e);
        let mut pos = [0usize; 3];
        pos[(e + 1) % 3] = 1 - (k >> 1);
        pos[(e + 2) % 3] = 1 - (k & 1);
        let slot = 4 * e + pos[p] + 2 * pos[q];
        let c = self.comp_of_edge[slot];
        let c = if c == NO_COMP { 0 } else { c as usize };
        self.indices[c]
    }
}

fn as_leaf(node: &Node) -> Option<&Leaf> {
    match node {
        Node::Leaf(l) => Some(l),
        _ => None,
    }
}

/// Concurrent memo of field values at integer finest-lattice coordinates.
///
/// The graded octree evaluates a cell corner or interior-probe point at
/// `g.point_at(key)` many times over: sibling cells share the faces between
/// them, neighbouring cells at the same depth share the edges between them,
/// and the interior-feature probe ([`GradedBuilder::model_holds_inside`])
/// resamples exactly the points the next refinement level would take. Keyed by
/// the absolute finest-lattice integer coordinate — bit-identical across depths
/// and cells for the same physical point — the memo evaluates each point at
/// most once over the whole tree, recovering for the sparse graded descent the
/// single-global-grid economy the uniform mesher gets for free (of-9gw). This
/// is what pays back the probe's ~19-evals-per-terminating-leaf cost: the
/// shared face and edge points among those 19 (all but the cell centre) are
/// evaluated once and reused by every cell that touches them.
///
/// Sharded by a hash of the key so the parallel subtrees above
/// [`GRADED_PAR_DEPTH`] rarely contend: neighbouring subtrees touch mostly
/// disjoint regions, and even a shared boundary point lands in one shard while
/// unrelated inserts proceed in the others. The SDF is evaluated *outside* the
/// lock, so two threads racing the same fresh point compute it twice (the SDF
/// is pure, so the result is identical and the map converges on one stored
/// value) rather than serializing every field evaluation behind the lock.
struct LatticeMemo {
    shards: Vec<Mutex<HashMap<[u32; 3], f64>>>,
}

impl LatticeMemo {
    /// One shard per ~4 potential worker threads (minimum 16), so the handful
    /// of parallel subtrees seldom hash to the same lock. Empty maps do not
    /// allocate, so an ample shard count costs nothing on small meshes.
    fn new() -> Self {
        let shard_count = (rayon::current_num_threads() * 4).max(16);
        Self {
            shards: (0..shard_count)
                .map(|_| Mutex::new(HashMap::new()))
                .collect(),
        }
    }

    fn shard(&self, key: [u32; 3]) -> &Mutex<HashMap<[u32; 3], f64>> {
        let h = key[0].wrapping_mul(0x9E37_79B1)
            ^ key[1].wrapping_mul(0x85EB_CA77)
            ^ key[2].wrapping_mul(0xC2B2_AE3D);
        &self.shards[h as usize % self.shards.len()]
    }

    /// Field value at `key`, computing it (outside the lock) only on the first
    /// request across the whole tree. Racing computes converge: `or_insert`
    /// keeps whichever value landed first, and all are bit-identical.
    fn eval_or(&self, key: [u32; 3], compute: impl FnOnce() -> f64) -> f64 {
        let shard = self.shard(key);
        if let Some(&v) = shard.lock().unwrap().get(&key) {
            return v;
        }
        let v = compute();
        *shard.lock().unwrap().entry(key).or_insert(v)
    }
}

struct GradedBuilder<'a> {
    sdf: &'a dyn Sdf,
    g: OctGrid,
    max_depth: u32,
    accuracy: f64,
    memo: LatticeMemo,
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
        memo: LatticeMemo::new(),
    };
    let root_corners: [f64; 8] =
        std::array::from_fn(|c| builder.eval_key(corner_key([0, 0, 0], opts.max_depth, c)));
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

/// Assign mesh vertex indices to leaf component vertices in deterministic
/// recursion order.
fn number_leaves(node: &mut Node, positions: &mut Vec<Point3>) {
    match node {
        Node::Empty => {}
        Node::Leaf(l) => {
            for c in 0..l.count as usize {
                l.indices[c] = positions.len();
                positions.push(l.vertices[c]);
            }
        }
        Node::Internal(children) => {
            for child in children.iter_mut() {
                number_leaves(child, positions);
            }
        }
    }
}

/// Field values on the `3x3x3` lattice of a cell's child corners, indexed
/// `(i * 3 + j) * 3 + k` with each of `i`, `j`, `k` in `0..=2`, filled
/// lazily. Even coordinates on every axis are the cell's own corners, so
/// those eight arrive already known.
struct SubLattice {
    vals: [Option<f64>; 27],
}

impl SubLattice {
    fn seeded(corners: &[f64; 8]) -> Self {
        let mut vals: [Option<f64>; 27] = [None; 27];
        for (c, &v) in corners.iter().enumerate() {
            let (dx, dy, dz) = corner(c);
            vals[(2 * dx * 3 + 2 * dy) * 3 + 2 * dz] = Some(v);
        }
        Self { vals }
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
        // The child-corner lattice serves both the leaf probe and, if the
        // cell goes on to subdivide, the children themselves — the probe
        // samples exactly the points refinement would need anyway, so a
        // cell rejected by the probe re-reads them for free.
        let mut lattice = SubLattice::seeded(&corners);
        if depth >= MIN_GRADED_DEPTH && has_sign_change(&corners) {
            if let Some(node) = self.try_leaf(depth, coords, corners, &mut lattice) {
                return node;
            }
        }
        self.subdivide(depth, coords, lattice)
    }

    /// Field value at the finest-lattice point `key`, evaluated once per
    /// point across the whole tree and reused by every cell that shares it
    /// ([`LatticeMemo`]).
    fn eval_key(&self, key: [u32; 3]) -> f64 {
        self.memo
            .eval_or(key, || self.sdf.eval(&self.g.point_at(key)))
    }

    /// Field value at child-corner lattice point `(i, j, k)` (each in
    /// `0..=2`) of the cell at `coords`/`depth`. The per-cell `lattice` is an
    /// L1 cache in front of the global [`LatticeMemo`], so points a cell reads
    /// more than once (the probe then subdivision) cost one array lookup, not
    /// a repeated shard lock.
    fn lattice_at(
        &self,
        lattice: &mut SubLattice,
        depth: u32,
        coords: [u32; 3],
        (i, j, k): (usize, usize, usize),
    ) -> f64 {
        let child_shift = self.max_depth - depth - 1;
        let base = [coords[0] << 1, coords[1] << 1, coords[2] << 1];
        let slot = &mut lattice.vals[(i * 3 + j) * 3 + k];
        *slot.get_or_insert_with(|| {
            self.eval_key([
                (base[0] + i as u32) << child_shift,
                (base[1] + j as u32) << child_shift,
                (base[2] + k as u32) << child_shift,
            ])
        })
    }

    /// Crossing point and unit normal for each sign-changing cell edge, at
    /// this cell's depth.
    fn crossings(
        &self,
        depth: u32,
        coords: [u32; 3],
        corners: &[f64; 8],
    ) -> [Option<(Vector3, Vector3)>; 12] {
        let shift = self.max_depth - depth;
        edge_crossings(self.sdf, corners, |b| {
            self.g.point_at(corner_key(coords, shift, b))
        })
    }

    /// Terminate refinement here if the cell's one-vertex surface model is
    /// within the accuracy target and no sharp feature crosses the cell;
    /// `None` means the cell must subdivide. Only called on sign-changing
    /// cells, so a returned leaf always carries a vertex — which the stitch
    /// relies on: any sign-changing minimal edge abutting this leaf can
    /// reference it. A returned leaf is always single-component: two-sheet
    /// cells (whose crossing normals disagree) are forced to subdivide so
    /// each sheet gets its own vertex at `max_depth`.
    fn try_leaf(
        &self,
        depth: u32,
        coords: [u32; 3],
        corners: [f64; 8],
        lattice: &mut SubLattice,
    ) -> Option<Node> {
        let (comp_of_edge, count) = classify_components(&corners);
        debug_assert!(count >= 1, "sign change must produce a component");
        // Two sheets in one cell cannot be modeled by a single vertex; refine
        // so the finest level places one vertex per sheet.
        if count > 1 {
            return None;
        }
        let crossings = self.crossings(depth, coords, &corners);

        // Chordal deviation of the linear edge model: |f| at an interpolated
        // crossing measures how far the local linearization strays from the
        // true surface (cheap first: skips gradient work on cells that
        // clearly refine).
        let err = crossings
            .iter()
            .filter_map(|c| c.map(|(p, _)| self.sdf.eval(&Point3::from(p)).abs()))
            .fold(0.0, f64::max);
        if err > self.accuracy {
            return None;
        }

        // Sharp feature or strong curvature: refine to max_depth.
        let normals: Vec<Vector3> = crossings.iter().filter_map(|c| c.map(|(_, n)| n)).collect();
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
        let vertices = component_qef(
            &comp_of_edge,
            count,
            &crossings,
            &self.g.cell_bounds(shift, coords),
        );
        if self.sdf.eval(&vertices[0]).abs() > self.accuracy {
            return None;
        }
        if !self.model_holds_inside(depth, coords, &crossings, &vertices[0], lattice) {
            return None;
        }
        Some(Node::Leaf(Box::new(Leaf {
            depth,
            corners,
            comp_of_edge,
            count,
            vertices,
            indices: [usize::MAX; 4],
        })))
    }

    /// Does the cell's one-vertex plane model predict the field across the
    /// cell's *interior*, not just on its boundary lattice?
    ///
    /// Corner signs and edge crossings only sample the cell's boundary. A
    /// feature living entirely inside the cell — the rim of a hole whose
    /// footprint clears all eight corners — leaves every corner and every
    /// crossing agreeing on one flat sheet, so the error and normal tests
    /// above pass and the cell terminates carrying a single vertex for what
    /// are really two sheets. Neighbouring fine leaves then route both
    /// sheets' stitch edges onto that one vertex, pinching them into
    /// four-triangle (non-manifold) edges (of-obv).
    ///
    /// Probing the child-corner sublattice — the points the next refinement
    /// level would sample anyway — catches such interiors: the model here is
    /// the dual patch itself, a plane through the QEF vertex with the mean
    /// crossing normal, so this is the same chordal-deviation bound the edge
    /// test applies, extended from the boundary to the cell's inside.
    ///
    /// Sampling is not a proof: a feature thinner than the sublattice can
    /// still hide. It is the same bet the edge and vertex tests already make,
    /// taken on a strictly finer point set.
    fn model_holds_inside(
        &self,
        depth: u32,
        coords: [u32; 3],
        crossings: &[Option<(Vector3, Vector3)>; 12],
        vertex: &Point3,
        lattice: &mut SubLattice,
    ) -> bool {
        let normal_sum = crossings
            .iter()
            .filter_map(|c| c.map(|(_, n)| n))
            .fold(Vector3::zeros(), |acc, n| acc + n);
        // No usable mean normal (opposed or all-zero crossings): the cell has
        // no trustworthy plane model, so refine rather than guess.
        let Some(normal) = normal_sum.try_normalize(1e-12) else {
            return false;
        };

        // Corners of this cell's eight children: the 3x3x3 sublattice. The
        // eight parent corners (even index on every axis) sit on the plane
        // model's own data and are already covered by the tests above.
        let child_shift = self.max_depth - depth - 1;
        let base = [coords[0] << 1, coords[1] << 1, coords[2] << 1];
        for i in 0..3usize {
            for j in 0..3usize {
                for k in 0..3usize {
                    if i != 1 && j != 1 && k != 1 {
                        continue;
                    }
                    let p = self.g.point_at([
                        (base[0] + i as u32) << child_shift,
                        (base[1] + j as u32) << child_shift,
                        (base[2] + k as u32) << child_shift,
                    ]);
                    let model = normal.dot(&(p - vertex));
                    if (self.lattice_at(lattice, depth, coords, (i, j, k)) - model).abs()
                        > self.accuracy
                    {
                        return false;
                    }
                }
            }
        }
        true
    }

    /// Finest-level cell: a leaf if any edge crosses, otherwise `Empty`
    /// (conservative intervals let crossingless cells survive to the
    /// bottom, exactly like uncrossed cells of the uniform grid). This is the
    /// only place two-sheet cells materialize, so it places one vertex per
    /// surface component.
    fn max_depth_leaf(&self, depth: u32, coords: [u32; 3], corners: [f64; 8]) -> Node {
        if !has_sign_change(&corners) {
            return Node::Empty;
        }
        let (comp_of_edge, count) = classify_components(&corners);
        let crossings = self.crossings(depth, coords, &corners);
        let vertices = component_qef(
            &comp_of_edge,
            count,
            &crossings,
            &self.g.cell_bounds(0, coords),
        );
        Node::Leaf(Box::new(Leaf {
            depth,
            corners,
            comp_of_edge,
            count,
            vertices,
            indices: [usize::MAX; 4],
        }))
    }

    /// Build the cell's eight children. `lattice` carries whatever child
    /// corners the leaf probe already sampled (it is seeded with this cell's
    /// own corners), so no field point is evaluated twice.
    fn subdivide(&self, depth: u32, coords: [u32; 3], lattice: SubLattice) -> Node {
        let child_shift = self.max_depth - depth - 1;
        let base = [coords[0] << 1, coords[1] << 1, coords[2] << 1];

        let children: [Node; 8] = if depth < GRADED_PAR_DEPTH {
            // Shallow cells are few: build subtrees in parallel. Sibling
            // corners are shared through the global memo, so a face or edge
            // point one child evaluates is reused by the next rather than
            // recomputed.
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
                    let child_corners: [f64; 8] =
                        std::array::from_fn(|k| self.eval_key(corner_key(cc, child_shift, k)));
                    self.build(depth + 1, cc, child_corners)
                })
                .collect();
            let mut drain = built.drain(..);
            std::array::from_fn(|_| drain.next().expect("eight children"))
        } else {
            // Sequential: share field samples between siblings through the
            // 3x3x3 lattice of child corners, carried over from the leaf
            // probe and filled lazily for interval-surviving children only.
            let mut lattice = lattice;
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
                    self.lattice_at(&mut lattice, depth, coords, (dx + ex, dy + ey, dz + ez))
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
/// twice and degenerates the quad into a triangle. Each leaf contributes the
/// vertex of the component that owns the minimal edge, so two-sheet cells no
/// longer pinch. Winding matches [`edge_quad`]: the quad
/// `(0,0),(1,0),(1,1),(0,1)` over the perpendicular axes faces `+e`, reversed
/// when the surface faces `-e`.
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
        leaves[0].comp_index(e, 0),
        leaves[2].comp_index(e, 2),
        leaves[3].comp_index(e, 3),
        leaves[1].comp_index(e, 1),
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

    /// Undirected edges shared by exactly four triangles: the pinched-edge
    /// signature of two surface sheets forced through one cell vertex (of-54d).
    /// Zero on a correctly component-split mesh.
    fn four_triangle_edges(mesh: &TriangleMesh) -> usize {
        let mut edges: HashMap<(usize, usize), u32> = HashMap::new();
        for tri in &mesh.indices {
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                *edges.entry((a.min(b), a.max(b))).or_insert(0) += 1;
            }
        }
        edges.values().filter(|&&c| c == 4).count()
    }

    /// Two overlapping unit spheres, subtracted: the difference has a sharp
    /// concave crease circle where the sheets meet, so cells beside the crease
    /// are crossed by both sheets — the two-sheets-per-cell band that pinched
    /// with one vertex per cell (of-54d).
    fn subtracted_spheres() -> Subtraction<Sphere, Sphere> {
        Subtraction {
            a: Sphere {
                center: Point3::origin(),
                radius: 1.0,
            },
            b: Sphere {
                center: Point3::new(1.0, 0.0, 0.0),
                radius: 1.0,
            },
        }
    }

    /// Regression for of-54d: the uniform-depth adaptive mesher shares the
    /// uniform grid's per-edge stitch, so — like the uniform grid — one vertex
    /// per surface component must keep the CSG crease band free of
    /// four-triangle (pinched) edges at every resolution. Before the port a
    /// single vertex per cell fused the two sheets into non-manifold pinches.
    #[test]
    fn uniform_depth_crease_band_has_no_pinched_edges() {
        let shape = subtracted_spheres();
        for max_depth in [4, 5, 6, 7] {
            let mesh = mesh_sdf_adaptive_indexed(
                &shape,
                &AdaptiveMeshOptions {
                    bounds: bounds(1.4),
                    max_depth,
                    accuracy: None,
                },
            );
            assert!(!mesh.is_empty(), "empty mesh at depth {max_depth}");
            assert_eq!(
                four_triangle_edges(&mesh),
                0,
                "pinched edges at depth {max_depth}"
            );
            assert!(
                mesh.is_closed_manifold(),
                "not a closed manifold at depth {max_depth}"
            );
        }
    }

    /// Graded refinement must also place one vertex per sheet: the two-sheet
    /// crease band comes out manifold with no pinched edges. Multi-sheet cells
    /// refine to `max_depth` and split into per-component vertices there.
    #[test]
    fn graded_crease_band_has_no_pinched_edges() {
        let shape = subtracted_spheres();
        for (max_depth, acc) in [(5, 0.02), (5, 0.005), (6, 0.01), (6, 0.005)] {
            let mesh = mesh_sdf_adaptive_indexed(
                &shape,
                &AdaptiveMeshOptions {
                    bounds: bounds(1.4),
                    max_depth,
                    accuracy: Some(acc),
                },
            );
            assert!(
                !mesh.is_empty(),
                "empty mesh at depth {max_depth}, acc {acc}"
            );
            assert_eq!(
                four_triangle_edges(&mesh),
                0,
                "pinched edges at depth {max_depth}, acc {acc}"
            );
            assert!(
                mesh.is_closed_manifold(),
                "not a closed manifold at depth {max_depth}, acc {acc}"
            );
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

    /// Flat-dominated scenes must pay off: grading collapses the flat
    /// regions into a fraction of the triangles *and* into a fraction of the
    /// field evaluations.
    ///
    /// The of-obv interior-feature probe ([`GradedBuilder::model_holds_inside`])
    /// costs ~19 evals on a *terminating* leaf — the one case with no skipped
    /// subtree to amortize them against — and once erased the eval win here
    /// entirely (parity at depth 8: 222k vs 232k). But those 19 points are all
    /// shared with sibling and neighbour cells except the cell centre, so the
    /// global lattice memo (of-9gw, [`LatticeMemo`]) evaluates each once and
    /// hands it back for free to every cell that touches it. Measured on this
    /// shape at depth 8: evals drop from 222k to 148k, a 1.57x win over the
    /// 232k uniform reference, with no change to the mesh (28,988 triangles,
    /// 8.0x fewer than uniform's 232k).
    ///
    /// The probe is not optional: without it the mesher pinches two surface
    /// sheets onto one vertex and faceted STEP export fails outright. This test
    /// guards both payoffs at a depth cheap enough to run — triangle count,
    /// which is what a STEP body actually pays for, and the restored eval
    /// savings.
    #[test]
    fn graded_box_union_collapses_flat_regions_without_eval_blowup() {
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
        let gm = mesh_sdf_adaptive_indexed(&graded, &opts);
        let um = mesh_sdf_adaptive_indexed(
            &uniform,
            &AdaptiveMeshOptions {
                accuracy: None,
                ..opts
            },
        );
        let (gt, ut) = (gm.triangle_count(), um.triangle_count());
        // Measured 8.0x here; assert half of it so ordinary drift in the
        // grading heuristics does not trip the gate.
        assert!(
            ut > 4 * gt,
            "graded produced {gt} triangles, uniform depth {ut}: expected >4x fewer"
        );
        let g = graded.evals.load(Ordering::Relaxed);
        let u = uniform.evals.load(Ordering::Relaxed);
        // Measured 1.57x fewer (148k vs 232k) once the lattice memo recovered
        // the probe's shared points; assert only >1.25x so drift in the
        // grading heuristics or a few racing double-computes never trip it.
        assert!(
            5 * g < 4 * u,
            "graded sampled {g} points, uniform depth sampled {u}: expected >1.25x fewer"
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

    /// A plate with a through-hole — the most common feature in mechanical
    /// CAD — must mesh closed at every grid alignment (of-obv).
    ///
    /// The hole's rim can fall entirely between a coarse cell's corners, so
    /// the cell reads as one flat sheet and used to terminate with a single
    /// vertex, pinching the plate face and the bore wall onto it. Whether a
    /// corner lands in the bore depends only on grid alignment, which is why
    /// this used to pass or fail on identity transforms of the same part;
    /// the offsets sweep alignments that previously failed.
    #[test]
    fn plate_with_through_hole_closes_at_any_grid_alignment() {
        for (i, &offset) in [0.0, 0.37, 1.4, 2.6, -0.9].iter().enumerate() {
            let shape = Subtraction {
                a: Box3 {
                    center: Point3::new(0.0, 0.0, 0.0),
                    half_extents: [30.0, 2.5, 20.0],
                },
                b: Cylinder {
                    center: Point3::new(15.0, 0.0, 0.0),
                    radius: 2.5,
                    half_height: 10.0,
                },
            };
            // Mirrors the STEP export path: 10% pad, 0.5%-of-extent accuracy.
            let pad = 6.0 + offset;
            let bounds = BoundingBox3::new(
                Point3::new(-30.0 - pad, -2.5 - pad, -20.0 - pad),
                Point3::new(30.0 + pad, 2.5 + pad, 20.0 + pad),
            );
            let extent = 60.0 + 2.0 * pad;
            let mesh = mesh_sdf_adaptive_indexed(
                &shape,
                &AdaptiveMeshOptions {
                    bounds,
                    max_depth: 8,
                    accuracy: Some(5e-3 * extent),
                },
            );
            assert!(!mesh.is_empty(), "case {i}: empty mesh");
            assert!(
                mesh.is_closed_manifold(),
                "case {i} (pad {pad}): plate with through-hole did not close"
            );
        }
    }
}
