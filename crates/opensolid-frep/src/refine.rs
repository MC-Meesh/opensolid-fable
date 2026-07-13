//! Post-meshing refinement: exact feature-edge snapping and isotropic
//! remeshing of dual-contouring output.
//!
//! Dual contouring places one QEF vertex per cell, which leaves two visible
//! artifacts. First, along a sharp CSG edge the per-cell QEF minimizes
//! against *sampled* crossing planes, so consecutive vertices oscillate
//! around the true intersection curve — a sawtooth "scalloping" of what
//! should be a clean polyline (e.g. the rim of a drilled hole). Second, the
//! triangulation inherits the grid: vertex placement inside each cell is
//! unconstrained, so the wireframe mixes slivers, needles, and abrupt
//! density changes at octree depth transitions.
//!
//! [`refine_mesh`] fixes both in place:
//!
//! 1. **Feature snap.** Every vertex is classified through
//!    [`Sdf::branches`]: the smooth field branches whose surfaces pass
//!    nearby. Two branches with distinct normals mark a crease, three or
//!    more a corner. Crease and corner vertices are Newton-projected onto
//!    the *analytic* intersection of those branch surfaces (the exact CSG
//!    edge), with a trust region of a fraction of a cell so vertices whose
//!    dual cell does not actually straddle the feature are left alone.
//! 2. **Sliver collapse.** Edges much shorter than a cell (QEF vertices
//!    from neighboring cells that nearly coincide) are collapsed under the
//!    standard link condition, preserving the closed manifold.
//! 3. **Tangential smoothing + Delaunay flips.** A few iterations of
//!    feature-aware Laplacian smoothing — regular vertices move only in
//!    their tangent plane and are re-projected onto the isosurface along
//!    the gradient; crease vertices move only along their crease polyline
//!    and are re-snapped onto the exact curve; corners stay fixed — each
//!    followed by edge flips toward the intrinsic Delaunay triangulation
//!    (feature edges are never flipped).
//!
//! Normals are recomputed from the field gradient at the end. The pass
//! preserves closed-manifoldness: flips and collapses are only applied when
//! their local validity conditions hold, and every failed projection falls
//! back to leaving the vertex where it was.

use crate::primitives::Sdf;
use nalgebra::{Matrix2, Matrix3, Vector2};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{Point3, Vector3};
use rayon::prelude::*;
use std::collections::{HashMap, HashSet};

/// Options for [`refine_mesh`].
#[derive(Debug, Clone, Copy)]
pub struct RefineOptions {
    /// Finest cell size of the source mesh: the length scale for feature
    /// detection tolerances, snap trust regions, and sliver collapse.
    pub cell: f64,
    /// Smoothing + flip iterations. 0 disables everything except the
    /// feature snap and sliver collapse.
    pub smooth_iterations: usize,
}

impl RefineOptions {
    /// Defaults tuned for interactive remeshing: three smoothing rounds.
    pub fn for_cell(cell: f64) -> Self {
        Self {
            cell,
            smooth_iterations: 3,
        }
    }
}

/// Two branch normals with a dot product below this describe distinct
/// surfaces meeting at a feature; above it they are the same local surface
/// sampled twice. Matches the mesher's `FEATURE_NORMAL_DOT`.
const FEATURE_DOT: f64 = 0.9;

/// Fraction of a neighbor-centroid step applied per smoothing iteration.
const SMOOTH_LAMBDA: f64 = 0.5;

/// Newton iterations for feature projection; convergence is quadratic, so
/// this is a generous cap.
const NEWTON_ITERS: usize = 8;

/// Snap trust region as a fraction of a cell: QEF already places genuine
/// feature-cell vertices within a small fraction of a cell of the true
/// edge, while vertices one cell away sit half a cell or more out — this
/// separates the two so near-feature sheet vertices are not dragged onto
/// the edge (which would collapse a band of triangles).
const SNAP_TRUST: f64 = 0.4;

/// Edges shorter than this fraction of a cell are slivers to collapse.
const COLLAPSE_FRACTION: f64 = 0.2;

/// Vertex classification from the active field branches at the vertex.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Kind {
    /// One smooth surface: free to smooth tangentially.
    Regular,
    /// On the intersection curve of exactly two branch surfaces (snap
    /// succeeded): moves only along the curve.
    Crease,
    /// On a corner of three or more surfaces: never moves.
    Corner,
    /// Near a feature but not on it (snap rejected by the trust region):
    /// frozen, so it neither pollutes the crease polyline nor gets smoothed
    /// with a one-sided gradient.
    Fixed,
}

impl Kind {
    fn is_feature(self) -> bool {
        !matches!(self, Kind::Regular)
    }
}

/// Refine a dual-contouring mesh in place against the field it was meshed
/// from. `opts.cell` must be the finest lattice step of the source grid.
/// No-op on empty meshes and non-positive cell sizes.
pub fn refine_mesh(sdf: &dyn Sdf, mesh: &mut TriangleMesh, opts: &RefineOptions) {
    if mesh.is_empty() || !opts.cell.is_finite() || opts.cell <= 0.0 {
        return;
    }
    let cell = opts.cell;

    repair_pinched_edges(mesh);
    let mut kinds = snap_features(sdf, mesh, cell);
    collapse_short_edges(sdf, mesh, &mut kinds, COLLAPSE_FRACTION * cell);

    // Interleave one flip pass with each smoothing pass (Botsch-Kobbelt
    // style), then flip to convergence once positions have settled.
    for _ in 0..opts.smooth_iterations {
        flip_pass(sdf, mesh, &kinds);
        smooth_pass(sdf, mesh, &kinds, cell);
    }
    if opts.smooth_iterations > 0 {
        for _ in 0..3 {
            if flip_pass(sdf, mesh, &kinds) == 0 {
                break;
            }
        }
    }

    mesh.normals = mesh
        .positions
        .par_iter()
        .map(|p| {
            let g = sdf.grad(p);
            let n = g.norm();
            if n > 1e-12 { g / n } else { Vector3::z() }
        })
        .collect();
}

/// Split pinched edges — edges shared by four triangles — into their two
/// surface sheets, duplicating the shared vertices per sheet.
///
/// Dual contouring with one vertex per cell cannot represent two surface
/// sheets crossing the same cell (the band around a CSG crease where both
/// surfaces pass through a cell that does not contain the crease itself,
/// of-1ad): it forces both sheets through one vertex, emitting edges used
/// by four triangles. The uniform mesher places one vertex per surface
/// component; the adaptive mesher does not (yet), so its output is repaired
/// here: at each four-triangle edge the two coherently-wound triangle pairs
/// are matched into sheets by geometric normal, triangle fans around every
/// vertex are traced with sheet-mates connected *across* the pinched edges,
/// and each extra fan gets its own copy of the vertex. Positions are
/// unchanged (the pinch stays geometrically, at sub-cell scale on the
/// crease band); the topology becomes two clean sheets with exactly two
/// triangles per edge.
///
/// Edges with three, five, or more triangles, or without balanced
/// orientations, are left untouched — no repair is guessed for genuinely
/// broken input.
fn repair_pinched_edges(mesh: &mut TriangleMesh) {
    // Directed incidences per undirected edge: (triangle, forward?).
    let mut edges: HashMap<(usize, usize), Vec<(usize, bool)>> = HashMap::new();
    for (t, tri) in mesh.indices.iter().enumerate() {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            edges
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push((t, a < b));
        }
    }
    if !edges.values().any(|inc| inc.len() == 4) {
        return;
    }

    let geo_normal = |t: usize| {
        let [a, b, c] = mesh.indices[t].map(|i| mesh.positions[i]);
        (b - a).cross(&(c - a)).normalize()
    };
    // Triangle adjacency used for fan tracing: across a regular edge, its
    // two triangles; across a pinched edge, each triangle and its
    // sheet-mate (the opposite-wound triangle with the closest normal).
    let mut mates: HashMap<(usize, usize), Vec<(usize, usize)>> = HashMap::new();
    for (&key, incident) in &edges {
        match incident[..] {
            [(t1, _), (t2, _)] => {
                mates.insert(key, vec![(t1, t2)]);
            }
            [_, _, _, _] => {
                let fwd: Vec<usize> = incident
                    .iter()
                    .filter(|&&(_, f)| f)
                    .map(|&(t, _)| t)
                    .collect();
                let bwd: Vec<usize> = incident
                    .iter()
                    .filter(|&&(_, f)| !f)
                    .map(|&(t, _)| t)
                    .collect();
                if fwd.len() != 2 || bwd.len() != 2 {
                    continue; // unbalanced orientations: not a pinch
                }
                let n = geo_normal(fwd[0]);
                let (m0, m1) = if n.dot(&geo_normal(bwd[0])) >= n.dot(&geo_normal(bwd[1])) {
                    (bwd[0], bwd[1])
                } else {
                    (bwd[1], bwd[0])
                };
                mates.insert(key, vec![(fwd[0], m0), (fwd[1], m1)]);
            }
            _ => {}
        }
    }

    // Fan-trace each vertex: union incident triangles that are connected
    // through an incident edge, then give every fan beyond the first its
    // own vertex copy.
    let mut vertex_tris: Vec<Vec<usize>> = vec![Vec::new(); mesh.positions.len()];
    for (t, tri) in mesh.indices.iter().enumerate() {
        for &v in tri {
            vertex_tris[v].push(t);
        }
    }
    let mut new_tris = mesh.indices.clone();
    for (v, tris) in vertex_tris.iter().enumerate() {
        if tris.len() < 2 {
            continue;
        }
        // Tiny union-find over this vertex's incident triangles.
        let slot_of: HashMap<usize, usize> =
            tris.iter().enumerate().map(|(s, &t)| (t, s)).collect();
        let mut parent: Vec<usize> = (0..tris.len()).collect();
        fn find(parent: &mut [usize], mut x: usize) -> usize {
            while parent[x] != x {
                parent[x] = parent[parent[x]];
                x = parent[x];
            }
            x
        }
        for &t in tris {
            let tri = mesh.indices[t];
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                if a != v && b != v {
                    continue;
                }
                let Some(pairs) = mates.get(&(a.min(b), a.max(b))) else {
                    continue;
                };
                for &(t1, t2) in pairs {
                    if let (Some(&s1), Some(&s2)) = (slot_of.get(&t1), slot_of.get(&t2)) {
                        let (r1, r2) = (find(&mut parent, s1), find(&mut parent, s2));
                        parent[r1] = r2;
                    }
                }
            }
        }
        let first_root = find(&mut parent, 0);
        let mut copy_of_root: HashMap<usize, usize> = HashMap::new();
        for (s, &t) in tris.iter().enumerate() {
            let root = find(&mut parent, s);
            if root == first_root {
                continue;
            }
            let copy = *copy_of_root.entry(root).or_insert_with(|| {
                mesh.positions.push(mesh.positions[v]);
                if !mesh.normals.is_empty() {
                    mesh.normals.push(mesh.normals[v]);
                }
                mesh.positions.len() - 1
            });
            for slot in new_tris[t].iter_mut() {
                if *slot == v {
                    *slot = copy;
                }
            }
        }
    }
    mesh.indices = new_tris;
}

/// Active branch surfaces near `p`, deduplicated by normal direction:
/// branches within `tol` of winning their min/max whose surface passes
/// within `tol` of `p`. Two branches whose unit normals agree within
/// [`FEATURE_DOT`] describe the same local surface; the closer one is kept.
fn distinct_branches(sdf: &dyn Sdf, p: &Point3, tol: f64) -> Vec<(f64, Vector3)> {
    let mut all = Vec::new();
    sdf.branches(p, tol, &mut all);
    let mut kept: Vec<(f64, Vector3, Vector3)> = Vec::new(); // (value, grad, unit)
    for (v, g) in all {
        if v.abs() > tol {
            continue;
        }
        let n = g.norm();
        if n < 1e-12 {
            continue;
        }
        let unit = g / n;
        match kept
            .iter_mut()
            .find(|(_, _, ku)| ku.dot(&unit) > FEATURE_DOT)
        {
            Some(k) => {
                if v.abs() < k.0.abs() {
                    *k = (v, g, unit);
                }
            }
            None => kept.push((v, g, unit)),
        }
    }
    // Nearest surfaces first, so creases near corners pick the right pair.
    kept.sort_by(|a, b| {
        a.0.abs()
            .partial_cmp(&b.0.abs())
            .expect("finite branch values")
    });
    kept.into_iter().map(|(v, g, _)| (v, g)).collect()
}

/// Newton-project `start` onto the common zero set of the `targets` branch
/// surfaces (two → crease curve, three → corner point), taking the
/// minimal-norm step of the underdetermined system at each iteration (so
/// crease projections move perpendicular to the curve, never along it).
/// Returns `None` if branches can no longer be matched by normal, the
/// system degenerates, the iterate leaves the `max_move` trust region, or
/// the residual fails to converge.
fn newton_snap(
    sdf: &dyn Sdf,
    start: Point3,
    targets: &[(f64, Vector3)],
    tol: f64,
    max_move: f64,
) -> Option<Point3> {
    let k = targets.len().min(3);
    let target_units: Vec<Vector3> = targets[..k].iter().map(|(_, g)| g.normalize()).collect();
    let eps = 1e-9 * max_move;
    let mut x = start;
    for _ in 0..NEWTON_ITERS {
        // Re-evaluate the branches at the current iterate and match each
        // target surface by normal direction.
        let mut avail = Vec::new();
        sdf.branches(&x, tol, &mut avail);
        let mut rows: Vec<(f64, Vector3)> = Vec::with_capacity(k);
        for tu in &target_units {
            let best = avail
                .iter()
                .filter(|(_, g)| g.norm() > 1e-12)
                .max_by(|a, b| {
                    let da = a.1.normalize().dot(tu);
                    let db = b.1.normalize().dot(tu);
                    da.partial_cmp(&db).expect("finite normal dots")
                })?;
            if best.1.normalize().dot(tu) < 0.5 {
                return None;
            }
            rows.push(*best);
        }
        if rows.iter().map(|(v, _)| v.abs()).fold(0.0, f64::max) < eps {
            return Some(x);
        }
        // Minimal-norm solution of J·delta = -r: delta = Jᵀ (J Jᵀ)⁻¹ (-r).
        let delta = match rows[..] {
            [(v0, g0), (v1, g1)] => {
                let m = Matrix2::new(g0.dot(&g0), g0.dot(&g1), g0.dot(&g1), g1.dot(&g1));
                let scale = g0.norm_squared() * g1.norm_squared();
                if m.determinant() < 1e-6 * scale {
                    return None; // normals (near-)parallel: no crease here
                }
                let lambda = m.try_inverse()? * Vector2::new(-v0, -v1);
                g0 * lambda.x + g1 * lambda.y
            }
            [(v0, g0), (v1, g1), (v2, g2)] => {
                let m = Matrix3::new(
                    g0.dot(&g0),
                    g0.dot(&g1),
                    g0.dot(&g2),
                    g1.dot(&g0),
                    g1.dot(&g1),
                    g1.dot(&g2),
                    g2.dot(&g0),
                    g2.dot(&g1),
                    g2.dot(&g2),
                );
                let scale = g0.norm_squared() * g1.norm_squared() * g2.norm_squared();
                if m.determinant() < 1e-6 * scale {
                    return None;
                }
                let lambda = m.try_inverse()? * Vector3::new(-v0, -v1, -v2);
                g0 * lambda.x + g1 * lambda.y + g2 * lambda.z
            }
            _ => return None,
        };
        x += delta;
        if (x - start).norm() > max_move {
            return None;
        }
    }
    None
}

/// Classify every vertex through its active branches and snap crease and
/// corner vertices onto the exact feature. Returns the vertex kinds.
fn snap_features(sdf: &dyn Sdf, mesh: &mut TriangleMesh, cell: f64) -> Vec<Kind> {
    let max_move = SNAP_TRUST * cell;
    let snapped: Vec<(Kind, Point3)> = mesh
        .positions
        .par_iter()
        .map(|p| {
            let branches = distinct_branches(sdf, p, cell);
            match branches.len() {
                0 | 1 => (Kind::Regular, *p),
                n => {
                    let k = n.min(3);
                    match newton_snap(sdf, *p, &branches[..k], cell, max_move) {
                        Some(x) if k == 2 => (Kind::Crease, x),
                        Some(x) => (Kind::Corner, x),
                        None => (Kind::Fixed, *p),
                    }
                }
            }
        })
        .collect();
    let mut kinds = Vec::with_capacity(snapped.len());
    for (i, (kind, p)) in snapped.into_iter().enumerate() {
        kinds.push(kind);
        mesh.positions[i] = p;
    }
    kinds
}

/// Vertex -> vertex adjacency from the index buffer.
fn vertex_neighbors(mesh: &TriangleMesh) -> Vec<Vec<usize>> {
    let mut neighbors: Vec<Vec<usize>> = vec![Vec::new(); mesh.positions.len()];
    for tri in &mesh.indices {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            if !neighbors[a].contains(&b) {
                neighbors[a].push(b);
            }
            if !neighbors[b].contains(&a) {
                neighbors[b].push(a);
            }
        }
    }
    neighbors
}

/// One Jacobi-style pass of feature-aware tangential Laplacian smoothing.
/// Regular vertices move toward their neighbor centroid within the tangent
/// plane and are re-projected onto the isosurface; crease vertices move
/// toward the midpoint of their two crease neighbors and are re-snapped
/// onto the exact curve; corners and frozen vertices stay put.
fn smooth_pass(sdf: &dyn Sdf, mesh: &mut TriangleMesh, kinds: &[Kind], cell: f64) {
    let neighbors = vertex_neighbors(mesh);
    let positions = &mesh.positions;
    let new_positions: Vec<Point3> = (0..positions.len())
        .into_par_iter()
        .map(|i| {
            let p = positions[i];
            match kinds[i] {
                Kind::Corner | Kind::Fixed => p,
                Kind::Crease => {
                    let curve: Vec<usize> = neighbors[i]
                        .iter()
                        .copied()
                        .filter(|&j| matches!(kinds[j], Kind::Crease | Kind::Corner))
                        .collect();
                    // Only an unambiguous polyline interior moves; junctions
                    // and frayed classifications stay fixed.
                    let [a, b] = curve[..] else { return p };
                    let mid = Point3::from((positions[a].coords + positions[b].coords) / 2.0);
                    let moved = p + (mid - p) * SMOOTH_LAMBDA;
                    let branches = distinct_branches(sdf, &moved, cell);
                    if branches.len() < 2 {
                        return p;
                    }
                    newton_snap(sdf, moved, &branches[..2], cell, SNAP_TRUST * cell).unwrap_or(p)
                }
                Kind::Regular => {
                    if neighbors[i].is_empty() {
                        return p;
                    }
                    let centroid = neighbors[i]
                        .iter()
                        .map(|&j| positions[j].coords)
                        .sum::<Vector3>()
                        / neighbors[i].len() as f64;
                    let delta = centroid - p.coords;
                    let g = sdf.grad(&p);
                    let n2 = g.norm_squared();
                    if n2 < 1e-18 {
                        return p;
                    }
                    let n = g / n2.sqrt();
                    let tangential = delta - n * delta.dot(&n);
                    let mut x = p + tangential * SMOOTH_LAMBDA;
                    // Two first-order projection steps back onto f = 0.
                    for _ in 0..2 {
                        let f = sdf.eval(&x);
                        let g = sdf.grad(&x);
                        let g2 = g.norm_squared();
                        if g2 < 1e-18 {
                            return p;
                        }
                        x -= g * (f / g2);
                    }
                    x
                }
            }
        })
        .collect();
    mesh.positions = new_positions;
}

/// An undirected edge `(a, b)` with `a < b` and its two incident oriented
/// triangles: `t_ab` contains the directed edge a->b (opposite vertex `c`),
/// `t_ba` contains b->a (opposite `d`). `usize::MAX` marks a missing side
/// (open input), which callers skip.
struct EdgeRec {
    a: usize,
    b: usize,
    t_ab: usize,
    c: usize,
    t_ba: usize,
    d: usize,
}

/// Every undirected edge with its incident triangles, sorted by `(a, b)`
/// (so existence checks are binary searches). Edges with the same direction
/// used twice (non-manifold input) are dropped.
fn edge_records(mesh: &TriangleMesh) -> Vec<EdgeRec> {
    // Vertex indices fit u32 comfortably (meshes are a few 100k vertices),
    // so the undirected edge packs into one u64 sort key.
    debug_assert!(mesh.positions.len() < u32::MAX as usize);
    let mut directed: Vec<(u64, bool, usize, usize)> = Vec::with_capacity(mesh.indices.len() * 3);
    for (t, tri) in mesh.indices.iter().enumerate() {
        for e in 0..3 {
            let a = tri[e];
            let b = tri[(e + 1) % 3];
            let opp = tri[(e + 2) % 3];
            let key = ((a.min(b) as u64) << 32) | a.max(b) as u64;
            directed.push((key, a < b, t, opp));
        }
    }
    directed.sort_unstable_by_key(|&(key, ..)| key);
    let mut recs = Vec::with_capacity(directed.len() / 2);
    let mut i = 0;
    while i < directed.len() {
        let key = directed[i].0;
        let mut rec = EdgeRec {
            a: (key >> 32) as usize,
            b: (key & u32::MAX as u64) as usize,
            t_ab: usize::MAX,
            c: usize::MAX,
            t_ba: usize::MAX,
            d: usize::MAX,
        };
        let mut valid = true;
        while i < directed.len() && directed[i].0 == key {
            let (_, forward, t, opp) = directed[i];
            let (slot_t, slot_o) = if forward {
                (&mut rec.t_ab, &mut rec.c)
            } else {
                (&mut rec.t_ba, &mut rec.d)
            };
            if *slot_t != usize::MAX {
                valid = false; // same direction twice: non-manifold
            }
            *slot_t = t;
            *slot_o = opp;
            i += 1;
        }
        if valid {
            recs.push(rec);
        }
    }
    recs
}

/// True if the sorted `recs` contain the undirected edge between `u`, `v`.
fn has_edge(recs: &[EdgeRec], u: usize, v: usize) -> bool {
    let key = (u.min(v), u.max(v));
    recs.binary_search_by(|r| (r.a, r.b).cmp(&key)).is_ok()
}

/// Angle at `apex` in the triangle `(apex, u, v)`. Test-only: the flip
/// criterion itself is evaluated trig-free.
#[cfg(test)]
fn corner_angle(apex: &Point3, u: &Point3, v: &Point3) -> f64 {
    let e1 = u - apex;
    let e2 = v - apex;
    let denom = e1.norm() * e2.norm();
    if denom < 1e-30 {
        return 0.0;
    }
    (e1.dot(&e2) / denom).clamp(-1.0, 1.0).acos()
}

/// True if the triangle `(a, b, c)` is non-degenerate and its winding
/// agrees with the outward field gradient at its centroid.
fn triangle_valid(sdf: &dyn Sdf, a: &Point3, b: &Point3, c: &Point3) -> bool {
    let cross = (b - a).cross(&(c - a));
    let area2 = cross.norm();
    let scale = (b - a).norm().max((c - a).norm()).max((c - b).norm());
    if area2 < 1e-10 * scale * scale {
        return false;
    }
    let centroid = Point3::from((a.coords + b.coords + c.coords) / 3.0);
    cross.dot(&sdf.grad(&centroid)) > 0.0
}

/// One pass of Delaunay edge flips (opposite-angle criterion) in the
/// surface tangent plane. Feature edges are preserved; a flip is only
/// applied when both replacement triangles are valid and the new diagonal
/// does not already exist. Returns the number of flips applied.
fn flip_pass(sdf: &dyn Sdf, mesh: &mut TriangleMesh, kinds: &[Kind]) -> usize {
    let recs = edge_records(mesh);
    // Diagonals created this pass: the stale `recs` cannot see them, and a
    // duplicate diagonal from two different quads would be non-manifold.
    let mut created: HashSet<(usize, usize)> = HashSet::new();
    let mut touched = vec![false; mesh.indices.len()];
    let mut flips = 0usize;
    for rec in &recs {
        if rec.t_ab == usize::MAX || rec.t_ba == usize::MAX {
            continue; // boundary edge (open input): leave it
        }
        if touched[rec.t_ab] || touched[rec.t_ba] {
            continue;
        }
        let (a, b, c, d) = (rec.a, rec.b, rec.c, rec.d);
        // Never flip away a feature edge; never create a diagonal between
        // two feature vertices (it would read as a false crease segment).
        if kinds[a].is_feature() && kinds[b].is_feature() {
            continue;
        }
        if kinds[c].is_feature() && kinds[d].is_feature() {
            continue;
        }
        if c == d || has_edge(&recs, c, d) || created.contains(&(c.min(d), c.max(d))) {
            continue;
        }
        let (pa, pb, pc, pd) = (
            mesh.positions[a],
            mesh.positions[b],
            mesh.positions[c],
            mesh.positions[d],
        );
        // Delaunay: flip when the opposite angles sum past pi. Trig-free:
        // with both angles in (0, pi), C + D > pi iff sin(C + D) < 0, and
        // sinC·cosD + cosC·sinD scales by the (positive) edge norms only.
        // The relative margin keeps near-ties from oscillating.
        let (e1, e2) = (pa - pc, pb - pc);
        let (f1, f2) = (pa - pd, pb - pd);
        let (sin_c, cos_c) = (e1.cross(&e2).norm(), e1.dot(&e2));
        let (sin_d, cos_d) = (f1.cross(&f2).norm(), f1.dot(&f2));
        let sin_sum = sin_c * cos_d + cos_c * sin_d;
        if sin_sum >= -1e-3 * (sin_c * sin_d).max(cos_c.abs() * cos_d.abs()) {
            continue;
        }
        if !triangle_valid(sdf, &pa, &pd, &pc) || !triangle_valid(sdf, &pd, &pb, &pc) {
            continue;
        }
        // t_ab was (a, b, c), t_ba was (b, a, d); the flipped pair keeps
        // every outer directed edge and replaces a<->b with the c<->d
        // diagonal, so orientation stays consistent.
        mesh.indices[rec.t_ab] = [a, d, c];
        mesh.indices[rec.t_ba] = [d, b, c];
        touched[rec.t_ab] = true;
        touched[rec.t_ba] = true;
        created.insert((c.min(d), c.max(d)));
        flips += 1;
    }
    flips
}

/// Collapse sliver edges (shorter than `threshold`) under the link
/// condition, then drop degenerate triangles and unreferenced vertices.
/// Feature vertices absorb regular ones (keeping the exact feature
/// position); feature-feature edges are never collapsed.
fn collapse_short_edges(
    sdf: &dyn Sdf,
    mesh: &mut TriangleMesh,
    kinds: &mut Vec<Kind>,
    threshold: f64,
) {
    let mut neighbors = vertex_neighbors(mesh);
    for n in &mut neighbors {
        n.sort_unstable();
    }
    // Sorted-merge intersection of two neighbor lists.
    let common_neighbors = |a: usize, b: usize| {
        let (na, nb) = (&neighbors[a], &neighbors[b]);
        let mut common = Vec::new();
        let (mut i, mut j) = (0, 0);
        while i < na.len() && j < nb.len() {
            match na[i].cmp(&nb[j]) {
                std::cmp::Ordering::Less => i += 1,
                std::cmp::Ordering::Greater => j += 1,
                std::cmp::Ordering::Equal => {
                    common.push(na[i]);
                    i += 1;
                    j += 1;
                }
            }
        }
        common
    };
    let recs = edge_records(mesh);
    // Triangles incident to each vertex, so the fold guard only inspects
    // the two endpoint rings instead of the whole index buffer.
    let mut vertex_tris: Vec<Vec<usize>> = vec![Vec::new(); mesh.positions.len()];
    for (t, tri) in mesh.indices.iter().enumerate() {
        for &v in tri {
            vertex_tris[v].push(t);
        }
    }

    // Only sliver edges are candidates; shortest first.
    let mut short: Vec<(f64, usize)> = recs
        .iter()
        .enumerate()
        .filter_map(|(i, r)| {
            let len2 = (mesh.positions[r.a] - mesh.positions[r.b]).norm_squared();
            (len2 < threshold * threshold && r.t_ab != usize::MAX && r.t_ba != usize::MAX)
                .then_some((len2, i))
        })
        .collect();
    short.sort_unstable_by(|x, y| x.partial_cmp(y).expect("finite edge lengths"));

    let mut locked = vec![false; mesh.positions.len()];
    // Old index -> surviving index for collapsed vertices.
    let mut remap: Vec<usize> = (0..mesh.positions.len()).collect();
    let mut any = false;
    for (_, rec_index) in short {
        let et = &recs[rec_index];
        let (a, b) = (et.a, et.b);
        if locked[a] || locked[b] {
            continue;
        }
        // Survivor keeps its position: features absorb regular vertices.
        let (survivor, gone) = match (kinds[a].is_feature(), kinds[b].is_feature()) {
            (true, true) => continue, // never merge two feature vertices
            (true, false) => (a, b),
            (false, true) => (b, a),
            (false, false) => (a, b),
        };
        // Link condition: the endpoints' common neighbors must be exactly
        // the two opposite vertices, or the collapse pinches the surface.
        let common = common_neighbors(a, b);
        if common.len() != 2 || !common.contains(&et.c) || !common.contains(&et.d) {
            continue;
        }
        // Regular-regular collapses meet at the midpoint, re-projected onto
        // the surface so the collapse does not dent it.
        let target = if kinds[survivor].is_feature() {
            mesh.positions[survivor]
        } else {
            let mut x =
                Point3::from((mesh.positions[survivor].coords + mesh.positions[gone].coords) / 2.0);
            for _ in 0..2 {
                let f = sdf.eval(&x);
                let g = sdf.grad(&x);
                let g2 = g.norm_squared();
                if g2 < 1e-18 {
                    break;
                }
                x -= g * (f / g2);
            }
            x
        };
        // Fold guard: every surviving triangle around either endpoint must
        // stay valid with the merged position.
        let survives = |tri: &[usize; 3]| {
            let mapped = tri.map(|v| {
                if v == gone || v == survivor {
                    usize::MAX
                } else {
                    v
                }
            });
            let merged_count = mapped.iter().filter(|&&v| v == usize::MAX).count();
            if merged_count != 1 {
                return true; // dies (both endpoints) or untouched elsewhere
            }
            let ps: Vec<Point3> = tri
                .iter()
                .map(|&v| {
                    if v == gone || v == survivor {
                        target
                    } else {
                        mesh.positions[v]
                    }
                })
                .collect();
            triangle_valid(sdf, &ps[0], &ps[1], &ps[2])
        };
        let ring_ok = vertex_tris[a].iter().chain(&vertex_tris[b]).all(|&t| {
            let tri = &mesh.indices[t];
            if tri.contains(&a) && tri.contains(&b) {
                return true; // this triangle is removed by the collapse
            }
            survives(tri)
        });
        if !ring_ok {
            continue;
        }
        remap[gone] = survivor;
        mesh.positions[survivor] = target;
        // Lock the whole neighborhood: one collapse per region per pass
        // keeps the precomputed adjacency valid.
        locked[a] = true;
        locked[b] = true;
        for &v in neighbors[a].iter().chain(&neighbors[b]) {
            locked[v] = true;
        }
        any = true;
    }
    if !any {
        return;
    }

    // Apply the remap, drop degenerate triangles, compact vertices.
    for tri in &mut mesh.indices {
        for v in tri.iter_mut() {
            *v = remap[*v];
        }
    }
    mesh.indices
        .retain(|t| t[0] != t[1] && t[1] != t[2] && t[0] != t[2]);
    compact(mesh, kinds);
}

/// Drop vertices not referenced by any triangle, remapping indices and
/// kinds in place.
fn compact(mesh: &mut TriangleMesh, kinds: &mut Vec<Kind>) {
    let mut new_index: Vec<Option<usize>> = vec![None; mesh.positions.len()];
    let mut positions = Vec::new();
    let mut normals = Vec::new();
    let mut new_kinds = Vec::new();
    for tri in &mut mesh.indices {
        for v in tri.iter_mut() {
            *v = *new_index[*v].get_or_insert_with(|| {
                positions.push(mesh.positions[*v]);
                if *v < mesh.normals.len() {
                    normals.push(mesh.normals[*v]);
                }
                new_kinds.push(kinds[*v]);
                positions.len() - 1
            });
        }
    }
    mesh.positions = positions;
    if !mesh.normals.is_empty() {
        mesh.normals = normals;
    }
    *kinds = new_kinds;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::csg::{Subtraction, Union};
    use crate::mesh_adaptive::{AdaptiveMeshOptions, mesh_sdf_adaptive_indexed};
    use crate::primitives::{Box3, Cylinder, Sphere};
    use opensolid_core::types::BoundingBox3;

    fn bounds(half: f64) -> BoundingBox3 {
        BoundingBox3::new(
            Point3::new(-half, -half, -half),
            Point3::new(half, half, half),
        )
    }

    /// Box with a through-drilled cylinder: the top and bottom rims are
    /// exact circles (radius 0.5 at y = +/-1), the outer edges exact lines.
    fn drilled_box() -> Subtraction<Box3, Cylinder> {
        Subtraction {
            a: Box3 {
                center: Point3::origin(),
                half_extents: [1.0, 1.0, 1.0],
            },
            b: Cylinder {
                center: Point3::origin(),
                radius: 0.5,
                half_height: 1.5,
            },
        }
    }

    fn adaptive_mesh(
        sdf: &dyn Sdf,
        half: f64,
        max_depth: u32,
        accuracy: f64,
    ) -> (TriangleMesh, f64) {
        let opts = AdaptiveMeshOptions {
            bounds: bounds(half),
            max_depth,
            accuracy: Some(accuracy),
        };
        let mesh = mesh_sdf_adaptive_indexed(sdf, &opts);
        let cell = 2.0 * half / (1u64 << max_depth) as f64;
        (mesh, cell)
    }

    /// Worst distance of the meshed rim polyline from the exact rim circle
    /// — the scalloping amplitude this bead is about. The rim polyline is
    /// recovered from the mesh itself: endpoints of sharp-dihedral edges
    /// (adjacent triangle normals disagree) in the top-rim region.
    fn rim_deviation(mesh: &TriangleMesh, cell: f64) -> f64 {
        let geo_normal = |t: &[usize; 3]| {
            let [a, b, c] = t.map(|i| mesh.positions[i]);
            (b - a).cross(&(c - a)).normalize()
        };
        let mut rim: HashSet<usize> = HashSet::new();
        for rec in &edge_records(mesh) {
            if rec.t_ab == usize::MAX || rec.t_ba == usize::MAX {
                continue;
            }
            if geo_normal(&mesh.indices[rec.t_ab]).dot(&geo_normal(&mesh.indices[rec.t_ba])) > 0.7 {
                continue;
            }
            rim.insert(rec.a);
            rim.insert(rec.b);
        }
        let mut worst: f64 = 0.0;
        let mut count = 0;
        for &v in &rim {
            let p = mesh.positions[v];
            let radial = (p.x * p.x + p.z * p.z).sqrt();
            let (dr, dy) = (radial - 0.5, p.y - 1.0);
            // Restrict to the top rim (excludes the box's outer edges).
            if dr.abs() < 3.0 * cell && dy.abs() < 3.0 * cell {
                worst = worst.max((dr * dr + dy * dy).sqrt());
                count += 1;
            }
        }
        assert!(count > 8, "expected rim-polyline vertices, found {count}");
        worst
    }

    #[test]
    fn branches_decompose_subtraction_at_rim() {
        let shape = drilled_box();
        // A point on the rim circle: both surfaces are zero there.
        let p = Point3::new(0.5, 1.0, 0.0);
        let branches = distinct_branches(&shape, &p, 0.05);
        assert_eq!(branches.len(), 2, "rim point must see two branch surfaces");
        for (v, _) in &branches {
            assert!(v.abs() < 1e-12, "branch value {v} should be ~0 on the rim");
        }
        // Their normals: box top face (0,1,0) and inward cylinder radial.
        let normals: Vec<Vector3> = branches.iter().map(|(_, g)| g.normalize()).collect();
        assert!(normals.iter().any(|n| n.dot(&Vector3::y()) > 0.99));
        assert!(
            normals
                .iter()
                .any(|n| n.dot(&Vector3::new(-1.0, 0.0, 0.0)) > 0.99)
        );
    }

    #[test]
    fn newton_snap_lands_exactly_on_rim() {
        let shape = drilled_box();
        // Start off the rim in both directions.
        let start = Point3::new(0.503, 0.996, 0.021);
        let branches = distinct_branches(&shape, &start, 0.05);
        assert_eq!(branches.len(), 2);
        let snapped = newton_snap(&shape, start, &branches[..2], 0.05, 0.05)
            .expect("snap must converge from a near-rim start");
        let radial = (snapped.x * snapped.x + snapped.z * snapped.z).sqrt();
        assert!((radial - 0.5).abs() < 1e-9, "radial {radial} not on circle");
        assert!(
            (snapped.y - 1.0).abs() < 1e-9,
            "y {} not on top face",
            snapped.y
        );
    }

    /// The headline acceptance: refining a drilled-box mesh must land the
    /// rim vertices on the exact circle (no sawtooth), keep the mesh a
    /// closed manifold, and keep every vertex on the surface.
    #[test]
    fn refine_removes_rim_scalloping() {
        let shape = drilled_box();
        let (mut mesh, cell) = adaptive_mesh(&shape, 1.4, 7, 0.005);
        assert!(mesh.is_closed_manifold());
        let before = rim_deviation(&mesh, cell);

        refine_mesh(&shape, &mut mesh, &RefineOptions::for_cell(cell));
        assert!(mesh.is_closed_manifold(), "refine broke the manifold");
        let after = rim_deviation(&mesh, cell);
        // Feature vertices land analytically on the curve; the rim band
        // must tighten decisively, not marginally.
        assert!(
            after < 0.25 * before.max(1e-9),
            "rim deviation before {before}, after {after}: no decisive improvement"
        );

        // Vertices stay on the surface.
        for p in &mesh.positions {
            assert!(
                shape.eval(p).abs() < cell,
                "vertex {p:?} pushed off the surface"
            );
        }
    }

    /// A pile of rim vertices must sit exactly (1e-9) on the analytic
    /// circle after refinement.
    #[test]
    fn refined_rim_vertices_are_exact() {
        let shape = drilled_box();
        let (mut mesh, cell) = adaptive_mesh(&shape, 1.4, 7, 0.005);
        refine_mesh(&shape, &mut mesh, &RefineOptions::for_cell(cell));
        let mut exact = 0;
        for p in &mesh.positions {
            let radial = (p.x * p.x + p.z * p.z).sqrt();
            if (radial - 0.5).abs() < 1e-9 && (p.y - 1.0).abs() < 1e-9 {
                exact += 1;
            }
        }
        // The rim at this resolution crosses well over 32 cells.
        assert!(
            exact > 32,
            "only {exact} vertices exactly on the rim circle"
        );
    }

    fn min_angle(mesh: &TriangleMesh, tri: &[usize; 3]) -> f64 {
        let [a, b, c] = tri.map(|i| mesh.positions[i]);
        corner_angle(&a, &b, &c)
            .min(corner_angle(&b, &c, &a))
            .min(corner_angle(&c, &a, &b))
    }

    /// Triangle quality: smoothing + flips must reduce the share of bad
    /// (min angle < 15 degrees) triangles substantially while preserving
    /// the manifold and the chordal deviation budget.
    #[test]
    fn refine_improves_triangle_quality() {
        let shape = Union {
            a: Sphere {
                center: Point3::origin(),
                radius: 1.0,
            },
            b: Box3 {
                center: Point3::new(0.8, 0.0, 0.0),
                half_extents: [0.6, 0.6, 0.6],
            },
        };
        let (mut mesh, cell) = adaptive_mesh(&shape, 1.8, 7, 0.005);
        assert!(mesh.is_closed_manifold());
        let bad_share = |m: &TriangleMesh| {
            let bad = m
                .indices
                .iter()
                .filter(|t| min_angle(m, t) < 15f64.to_radians())
                .count();
            bad as f64 / m.triangle_count() as f64
        };
        let before = bad_share(&mesh);
        refine_mesh(&shape, &mut mesh, &RefineOptions::for_cell(cell));
        assert!(mesh.is_closed_manifold(), "refine broke the manifold");
        let after = bad_share(&mesh);
        assert!(
            after < 0.5 * before.max(1e-9),
            "bad-triangle share before {before:.3}, after {after:.3}"
        );
        // Quality must not cost accuracy: vertices stay within a cell.
        for p in &mesh.positions {
            assert!(shape.eval(p).abs() < cell, "vertex {p:?} off the surface");
        }
    }

    /// A smooth shape (no features at all) must survive refinement: no
    /// vertex classified as feature, manifold preserved, surface kept.
    #[test]
    fn refine_smooth_sphere_is_safe() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let (mut mesh, cell) = adaptive_mesh(&s, 1.6, 6, 0.005);
        let before_tris = mesh.triangle_count();
        refine_mesh(&s, &mut mesh, &RefineOptions::for_cell(cell));
        assert!(mesh.is_closed_manifold());
        // Flips preserve the triangle count; only collapses reduce it, and
        // a clean sphere mesh has few to none.
        assert!(mesh.triangle_count() <= before_tris);
        for p in &mesh.positions {
            assert!(s.eval(p).abs() < 0.5 * cell, "vertex {p:?} off the sphere");
        }
    }

    /// Per-phase timing for tuning, not a gate:
    /// `cargo test -p opensolid-frep --release refine_phase_timings -- --ignored --nocapture`
    #[test]
    #[ignore = "perf measurement; run with --release -- --ignored --nocapture"]
    fn refine_phase_timings() {
        let shape = drilled_box();
        let (mut mesh, cell) = adaptive_mesh(&shape, 1.4, 9, 0.005);
        eprintln!(
            "{} tris, {} verts",
            mesh.triangle_count(),
            mesh.vertex_count()
        );
        let t = std::time::Instant::now();
        let mut kinds = snap_features(&shape, &mut mesh, cell);
        eprintln!("snap: {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
        let t = std::time::Instant::now();
        collapse_short_edges(&shape, &mut mesh, &mut kinds, COLLAPSE_FRACTION * cell);
        eprintln!("collapse: {:.1} ms", t.elapsed().as_secs_f64() * 1e3);
        for i in 0..3 {
            let t = std::time::Instant::now();
            flip_pass(&shape, &mut mesh, &kinds);
            let flip_ms = t.elapsed().as_secs_f64() * 1e3;
            let t = std::time::Instant::now();
            smooth_pass(&shape, &mut mesh, &kinds, cell);
            eprintln!(
                "iter {i}: flip {flip_ms:.1} ms, smooth {:.1} ms",
                t.elapsed().as_secs_f64() * 1e3
            );
        }
    }

    /// The pinch configuration the adaptive mesher emits in
    /// two-sheets-per-cell bands: two locally flat surface sheets crossing
    /// at a shared edge, four triangles on that edge (one coherently-wound
    /// pair per sheet, near-coplanar within each sheet). Repair must pair
    /// the sheets by normal, duplicate the shared vertices, and leave two
    /// clean two-triangle edges.
    #[test]
    fn repair_splits_pinched_edge_between_crossing_sheets() {
        let positions = vec![
            Point3::new(0.0, 0.0, 0.0),  // 0: shared edge start
            Point3::new(0.0, 1.0, 0.0),  // 1: shared edge end
            Point3::new(-1.0, 0.5, 0.0), // sheet 1 (z = 0 plane, normal +z)
            Point3::new(1.0, 0.5, 0.0),
            Point3::new(0.0, 0.5, 1.0), // sheet 2 (x = 0 plane, normal +x)
            Point3::new(0.0, 0.5, -1.0),
        ];
        let indices = vec![
            [0, 1, 2], // sheet 1, left of the edge
            [1, 0, 3], // sheet 1, right of the edge
            [0, 1, 4], // sheet 2, front
            [1, 0, 5], // sheet 2, back
        ];
        let mut mesh = TriangleMesh {
            normals: positions.iter().map(|_| Vector3::z()).collect(),
            positions,
            indices,
        };
        repair_pinched_edges(&mut mesh);
        assert_eq!(mesh.triangle_count(), 4, "repair must not add triangles");
        assert_eq!(mesh.vertex_count(), 8, "both shared vertices get one copy");
        // Copies sit exactly on the originals: geometry unchanged.
        assert_eq!(mesh.positions[6], mesh.positions[0]);
        assert_eq!(mesh.positions[7], mesh.positions[1]);
        // No edge carries four triangles anymore, and each sheet kept its
        // own coherently wound pair.
        for rec in edge_records(&mesh) {
            let interior = rec.t_ab != usize::MAX && rec.t_ba != usize::MAX;
            if interior {
                let same = |t: usize, u: usize| {
                    let (nt, nu) = (mesh.indices[t], mesh.indices[u]);
                    let plane = |tri: [usize; 3]| {
                        let [a, b, c] = tri.map(|i| mesh.positions[i]);
                        (b - a).cross(&(c - a)).normalize()
                    };
                    plane(nt).dot(&plane(nu)) > 0.99
                };
                assert!(
                    same(rec.t_ab, rec.t_ba),
                    "sheet pairing split a coherent sheet"
                );
            }
        }
        // The two sheets are now edge-disjoint: sheet 2's triangles all
        // reference the duplicated vertices.
        assert_eq!(mesh.indices[2], [6, 7, 4]);
        assert_eq!(mesh.indices[3], [7, 6, 5]);
    }

    #[test]
    fn repair_leaves_clean_meshes_alone() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let (mut mesh, _) = adaptive_mesh(&s, 1.6, 5, 0.01);
        let (verts, tris) = (mesh.vertex_count(), mesh.triangle_count());
        repair_pinched_edges(&mut mesh);
        assert_eq!(mesh.vertex_count(), verts);
        assert_eq!(mesh.triangle_count(), tris);
        assert!(mesh.is_closed_manifold());
    }

    #[test]
    fn refine_empty_and_degenerate_inputs_are_noops() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let mut empty = TriangleMesh::new();
        refine_mesh(&s, &mut empty, &RefineOptions::for_cell(0.01));
        assert!(empty.is_empty());

        let (mut mesh, _) = adaptive_mesh(&s, 1.6, 5, 0.01);
        let tris = mesh.triangle_count();
        for cell in [0.0, -1.0, f64::NAN] {
            refine_mesh(&s, &mut mesh, &RefineOptions::for_cell(cell));
            assert_eq!(
                mesh.triangle_count(),
                tris,
                "degenerate cell {cell} not a no-op"
            );
        }
    }
}
