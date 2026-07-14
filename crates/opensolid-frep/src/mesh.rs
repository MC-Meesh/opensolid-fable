//! Dual contouring mesher: converts any [`Sdf`] into a triangle mesh.
//!
//! The algorithm samples the SDF on a uniform grid over a bounding region,
//! finds grid edges where the sign changes (surface crossings), places one
//! vertex per surface component of each crossed cell using Hermite data
//! (linear interpolation of SDF values along crossed edges, normals from
//! the SDF gradient), and connects, for each crossed edge, the vertices of
//! that edge's component in the four surrounding cells into a quad.
//!
//! A cell's surface components are the connected groups of its crossed
//! edges, traced across the cell faces: a face crossed twice links its two
//! crossings, a face crossed four times (checkerboard corner signs) is
//! split by the asymptotic decider on the face's bilinear interpolant. The
//! decider uses only face-local data, so the two cells sharing a face
//! always agree and every mesh edge is stitched by exactly two quads.
//! Single-vertex-per-cell dual contouring cannot represent two surface
//! sheets crossing one cell — e.g. the crease circle of a CSG subtraction —
//! and emitted non-manifold four-quad edges there (of-1ad).
//!
//! One doubly-crossed face remains: a component may pass through a face
//! twice (all four crossings of an ambiguous face in one component). If
//! that happens on *both* sides of the face, its four quads would all
//! connect the same two cell vertices — a four-quad edge again. Such faces
//! are split before stitching: each contour arc gets an extra vertex at
//! the midpoint of its two crossings, and the arc's two quads route
//! through it as pentagons, restoring exactly-two-polygons-per-edge.
//!
//! The surface must lie strictly inside `bounds`; crossings on the boundary
//! layer of cells are not stitched and would leave holes.
//!
//! Grid cells are kept near-cubic regardless of the aspect ratio of
//! `bounds`: [`MeshOptions::resolution`] sets the cell count along the
//! longest axis and the other axes get proportionally fewer cells.
//! Strongly anisotropic cells alias near-tangent surface regions into
//! ambiguous faces, needlessly splitting surface components (of-6f8).

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

/// Cell faces as (corner cycle, edge cycle): `corners[i]` and
/// `corners[(i+1)%4]` are the endpoints of `CELL_EDGES[edges[i]]`, so
/// `edges[(i+3)%4]` and `edges[i]` are the two face edges meeting at
/// `corners[i]`. Order: x=0, x=1, y=0, y=1, z=0, z=1.
const CELL_FACES: [([usize; 4], [usize; 4]); 6] = [
    ([0, 2, 6, 4], [4, 10, 6, 8]),
    ([1, 3, 7, 5], [5, 11, 7, 9]),
    ([0, 1, 5, 4], [0, 9, 2, 8]),
    ([2, 3, 7, 6], [1, 11, 3, 10]),
    ([0, 1, 3, 2], [0, 5, 1, 4]),
    ([4, 5, 7, 6], [2, 7, 3, 6]),
];

/// Marker for cell edges without a sign change in [`CellVerts::comp_of_edge`].
pub(crate) const NO_COMP: u8 = u8::MAX;

/// Dual vertices for one cell: one Hermite vertex (centroid of the edge
/// crossings) per surface component. Every crossed edge is paired with
/// exactly one partner on each of its two faces, so components are cycles
/// of at least three edges — at most four fit in twelve slots.
struct CellVerts {
    /// Component id per [`CELL_EDGES`] slot; [`NO_COMP`] where the edge has
    /// no sign change.
    comp_of_edge: [u8; 12],
    /// Number of components (1..=4).
    count: u8,
    /// Vertex per component, in first-crossed-edge order.
    points: [Point3; 4],
}

fn uf_find(parent: &mut [u8; 12], mut x: u8) -> u8 {
    while parent[x as usize] != x {
        parent[x as usize] = parent[parent[x as usize] as usize];
        x = parent[x as usize];
    }
    x
}

fn uf_union(parent: &mut [u8; 12], a: u8, b: u8) {
    let ra = uf_find(parent, a);
    let rb = uf_find(parent, b);
    parent[ra as usize] = rb;
}

/// Group a cell's sign-changing edges into surface components from its eight
/// corner field values, returning the component id of each [`CELL_EDGES`]
/// slot ([`NO_COMP`] where the edge has no sign change) and the component
/// count (0 if no edge crosses).
///
/// Components are traced across the six faces: a face with two crossings
/// links them; a face with four (checkerboard corner signs) is resolved by
/// the asymptotic decider — the sign of the face's bilinear interpolant at
/// its saddle point picks which diagonal corner pair the two contour arcs
/// separate. The decider reads only face data, so adjacent cells always
/// agree. Purely a function of the corner signs (and saddle magnitudes on
/// ambiguous faces), so the uniform and adaptive meshers share it and stitch
/// consistently across a shared face.
pub(crate) fn classify_components(v: &[f64; 8]) -> ([u8; 12], u8) {
    let inside = |c: usize| v[c] < 0.0;
    let crossed = |e: usize| {
        let (a, b) = CELL_EDGES[e];
        inside(a) != inside(b)
    };

    let mut parent: [u8; 12] = core::array::from_fn(|e| e as u8);
    for &(corners, edges) in &CELL_FACES {
        let cross: Vec<usize> = (0..4).filter(|&n| crossed(edges[n])).collect();
        match cross[..] {
            [p, q] => uf_union(&mut parent, edges[p] as u8, edges[q] as u8),
            [_, _, _, _] => {
                // Checkerboard face: corners alternate inside/outside, all
                // four edges crossed. The two contour arcs wrap the diagonal
                // corner pair whose sign differs from the bilinear saddle.
                let f: [f64; 4] = core::array::from_fn(|n| v[corners[n]]);
                let saddle = (f[0] * f[2] - f[1] * f[3]) / (f[0] + f[2] - f[1] - f[3]);
                let wrap_even = inside(corners[0]) != (saddle < 0.0);
                let (m, n) = if wrap_even { (0, 2) } else { (1, 3) };
                for w in [m, n] {
                    uf_union(&mut parent, edges[(w + 3) % 4] as u8, edges[w] as u8);
                }
            }
            _ => {}
        }
    }

    let mut comp_of_edge = [NO_COMP; 12];
    let mut root_comp = [NO_COMP; 12];
    let mut ncomp = 0u8;
    for (e, slot) in comp_of_edge.iter_mut().enumerate() {
        if !crossed(e) {
            continue;
        }
        let r = uf_find(&mut parent, e as u8) as usize;
        if root_comp[r] == NO_COMP {
            debug_assert!(ncomp < 4, "more than four edge components in a cell");
            root_comp[r] = ncomp;
            ncomp += 1;
        }
        *slot = root_comp[r];
    }
    (comp_of_edge, ncomp)
}

/// Dual vertices for one cell, or `None` if no edge crosses the surface.
/// One Hermite vertex (centroid of the edge crossings) per surface component
/// as grouped by [`classify_components`].
fn cell_verts(g: &Grid, values: &[f64], i: usize, j: usize, k: usize) -> Option<Box<CellVerts>> {
    let v: [f64; 8] = core::array::from_fn(|c| {
        let (cx, cy, cz) = corner(c);
        values[g.vidx(i + cx, j + cy, k + cz)]
    });
    let inside = |c: usize| v[c] < 0.0;

    let mut crossing = [None::<Vector3>; 12];
    let mut any = false;
    for (e, &(a, b)) in CELL_EDGES.iter().enumerate() {
        if inside(a) == inside(b) {
            continue;
        }
        let t = v[a] / (v[a] - v[b]);
        let (ax, ay, az) = corner(a);
        let (bx, by, bz) = corner(b);
        let pa = g.point_at(i + ax, j + ay, k + az);
        let pb = g.point_at(i + bx, j + by, k + bz);
        crossing[e] = Some(pa.coords + (pb.coords - pa.coords) * t);
        any = true;
    }
    if !any {
        return None;
    }

    let (comp_of_edge, ncomp) = classify_components(&v);
    let mut sums = [Vector3::zeros(); 4];
    let mut counts = [0usize; 4];
    for e in 0..12 {
        let Some(x) = crossing[e] else { continue };
        let c = comp_of_edge[e] as usize;
        sums[c] += x;
        counts[c] += 1;
    }
    let mut points = [Point3::origin(); 4];
    for c in 0..ncomp as usize {
        points[c] = Point3::from(sums[c] / counts[c] as f64);
    }
    Some(Box::new(CellVerts {
        comp_of_edge,
        count: ncomp,
        points,
    }))
}

/// Numbered dual vertices for one cell: the component vertices occupy mesh
/// positions `base..base + count`, so the vertex for a crossed edge slot is
/// `base + comp_of_edge[slot]`.
struct CellRef {
    base: usize,
    comp_of_edge: [u8; 12],
}

/// Extra vertices splitting doubly-crossed faces, keyed by (face axis,
/// minus-side cell, crossed-edge slot in that cell). Both crossed edges of
/// one contour arc map to the arc's shared vertex.
type FaceSplits = std::collections::HashMap<(usize, [usize; 3], usize), usize>;

/// Find faces whose four crossings belong to a single component in *both*
/// adjacent cells — the component passes through the face twice, so the four
/// quads around the face would all share one mesh edge (a four-quad edge).
/// For each contour arc of such a face, append an extra vertex at the
/// midpoint of the arc's two crossings; [`slab_triangles`] routes the arc's
/// two quads through it.
fn doubled_face_splits(
    g: &Grid,
    values: &[f64],
    cell_vertex: &[Option<CellRef>],
    positions: &mut Vec<Point3>,
) -> FaceSplits {
    let mut splits = FaceSplits::new();
    for i in 0..g.n[0] {
        for j in 0..g.n[1] {
            for k in 0..g.n[2] {
                let Some(c1) = cell_vertex[g.cidx(i, j, k)].as_ref() else {
                    continue;
                };
                for axis in 0..3 {
                    let mut nb = [i, j, k];
                    nb[axis] += 1;
                    if nb[axis] >= g.n[axis] {
                        continue;
                    }
                    let Some(c2) = cell_vertex[g.cidx(nb[0], nb[1], nb[2])].as_ref() else {
                        continue;
                    };
                    // The shared face: this cell's plus face and the
                    // neighbor's minus face along `axis`.
                    let (corners, edges) = CELL_FACES[2 * axis + 1];
                    let nb_edges = CELL_FACES[2 * axis].1;
                    let single = |cr: &CellRef, es: [usize; 4]| {
                        cr.comp_of_edge[es[0]] != NO_COMP
                            && es
                                .iter()
                                .all(|&e| cr.comp_of_edge[e] == cr.comp_of_edge[es[0]])
                    };
                    if !single(c1, edges) || !single(c2, nb_edges) {
                        continue;
                    }
                    // Same asymptotic decider as `cell_verts`, so the arcs
                    // match the component tracing on both sides.
                    let f: [f64; 4] = core::array::from_fn(|n| {
                        let (cx, cy, cz) = corner(corners[n]);
                        values[g.vidx(i + cx, j + cy, k + cz)]
                    });
                    let saddle = (f[0] * f[2] - f[1] * f[3]) / (f[0] + f[2] - f[1] - f[3]);
                    let wrap_even = (f[0] < 0.0) != (saddle < 0.0);
                    let wraps = if wrap_even { [0, 2] } else { [1, 3] };
                    for w in wraps {
                        let slots = [edges[(w + 3) % 4], edges[w]];
                        let mut sum = Vector3::zeros();
                        for &s in &slots {
                            let (a, b) = CELL_EDGES[s];
                            let (ax, ay, az) = corner(a);
                            let (bx, by, bz) = corner(b);
                            let va = values[g.vidx(i + ax, j + ay, k + az)];
                            let vb = values[g.vidx(i + bx, j + by, k + bz)];
                            let pa = g.point_at(i + ax, j + ay, k + az);
                            let pb = g.point_at(i + bx, j + by, k + bz);
                            sum += pa.coords + (pb.coords - pa.coords) * (va / (va - vb));
                        }
                        let split_vertex = positions.len();
                        positions.push(Point3::from(sum / 2.0));
                        for &s in &slots {
                            splits.insert((axis, [i, j, k], s), split_vertex);
                        }
                    }
                }
            }
        }
    }
    splits
}

/// Triangles for one `(d, gd)` slab of the quad-emission pass: for each
/// interior grid edge along axis `d` at depth `gd` with a sign change,
/// connect the vertices of the four cells sharing that edge — a quad, with
/// a split vertex inserted on any side crossing a doubled face (see
/// [`doubled_face_splits`]) — and fan-triangulate. Winding: with
/// perpendicular axes u = (d+1)%3 and v = (d+2)%3, the order
/// (0,0),(1,0),(1,1),(0,1) faces +d; reversed when the surface faces -d.
fn slab_triangles(
    g: &Grid,
    values: &[f64],
    cell_vertex: &[Option<CellRef>],
    splits: &FaceSplits,
    d: usize,
    gd: usize,
) -> Vec<[usize; 3]> {
    let u = (d + 1) % 3;
    let v = (d + 2) % 3;
    // Perpendicular axes in increasing order: within each CELL_EDGES axis
    // group the slot layout is `offset(p) + 2 * offset(q)`.
    let (p, q) = match d {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
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
                // This grid edge's slot within the cell's CELL_EDGES.
                let mut delta = [0usize; 3];
                delta[u] = 1 - a;
                delta[v] = 1 - b;
                let slot = 4 * d + delta[p] + 2 * delta[q];
                let cr = cell_vertex[g.cidx(c[0], c[1], c[2])]
                    .as_ref()
                    .expect("cell adjacent to a sign-change edge must have a vertex");
                let comp = cr.comp_of_edge[slot];
                debug_assert_ne!(comp, NO_COMP, "crossed edge missing from its cell");
                (cr.base + comp as usize, c, slot)
            };
            let cells = [cell(0, 0), cell(1, 0), cell(1, 1), cell(0, 1)];
            // Quad side `s` joins `cells[s]` and `cells[(s + 1) % 4]`; the
            // face between them is keyed in `splits` by its minus-side cell.
            let side_minus = [(0, u), (1, v), (3, u), (0, v)];
            let mut poly: Vec<usize> = Vec::with_capacity(8);
            let mut lead = None;
            for (s, &(vertex, _, _)) in cells.iter().enumerate() {
                poly.push(vertex);
                let (m, axis) = side_minus[s];
                let (_, coords, slot) = cells[m];
                if let Some(&split_vertex) = splits.get(&(axis, coords, slot)) {
                    poly.push(split_vertex);
                    lead = Some(split_vertex);
                }
            }
            if !inside0 {
                poly.reverse();
            }
            // Fan from a split vertex when there is one: every chord is then
            // incident to it, so no chord can recreate the doubled edge the
            // split vertex exists to divide.
            if let Some(sv) = lead {
                let at = poly.iter().position(|&x| x == sv).unwrap();
                poly.rotate_left(at);
            }
            for t in 1..poly.len() - 1 {
                tris.push([poly[0], poly[t], poly[t + 1]]);
            }
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
    let mut candidates: Vec<Option<Box<CellVerts>>> = Vec::with_capacity(nx * ny * nz);
    candidates.resize_with(nx * ny * nz, || None);
    candidates
        .par_chunks_mut(ny * nz)
        .enumerate()
        .for_each(|(i, slab)| {
            for j in 0..ny {
                for k in 0..nz {
                    slab[j * nz + k] = cell_verts(g, &values, i, j, k);
                }
            }
        });
    let mut cell_vertex: Vec<Option<CellRef>> = Vec::with_capacity(nx * ny * nz);
    cell_vertex.resize_with(nx * ny * nz, || None);
    for (c, candidate) in candidates.into_iter().enumerate() {
        if let Some(cv) = candidate {
            let base = mesh.positions.len();
            mesh.positions
                .extend_from_slice(&cv.points[..cv.count as usize]);
            cell_vertex[c] = Some(CellRef {
                base,
                comp_of_edge: cv.comp_of_edge,
            });
        }
    }

    let splits = doubled_face_splits(g, &values, &cell_vertex, &mut mesh.positions);

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
        .map(|&(d, gd)| slab_triangles(g, &values, &cell_vertex, &splits, d, gd))
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

    let mut cell_vertex: Vec<Option<CellRef>> = Vec::with_capacity(nx * ny * nz);
    cell_vertex.resize_with(nx * ny * nz, || None);
    for i in 0..nx {
        for j in 0..ny {
            for k in 0..nz {
                if let Some(cv) = cell_verts(g, &values, i, j, k) {
                    let base = mesh.positions.len();
                    mesh.positions
                        .extend_from_slice(&cv.points[..cv.count as usize]);
                    cell_vertex[g.cidx(i, j, k)] = Some(CellRef {
                        base,
                        comp_of_edge: cv.comp_of_edge,
                    });
                }
            }
        }
    }

    let splits = doubled_face_splits(g, &values, &cell_vertex, &mut mesh.positions);

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
                .extend(slab_triangles(g, &values, &cell_vertex, &splits, d, gd));
        }
    }

    mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::blend::SmoothSubtraction;
    use crate::csg::{Intersection, Subtraction, Union};
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

    /// Two overlapping unit spheres, subtracted: the boundary of the
    /// difference has a sharp concave crease circle where the two sphere
    /// sheets meet, so cells along the crease are crossed by both sheets.
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

    /// Regression for of-1ad: single-vertex-per-cell dual contouring could
    /// not represent the crease circle of a CSG subtraction — grid faces
    /// there have checkerboard corner signs (all four edges crossed), so
    /// four quads shared one mesh edge and the mesh was not manifold at
    /// most resolutions. Cells now place one vertex per surface component.
    #[test]
    fn csg_subtraction_crease_manifold() {
        let shape = subtracted_spheres();
        for resolution in [16, 24, 32, 48, 64] {
            let opts = MeshOptions {
                bounds: bounds(1.4),
                resolution,
            };
            let mesh = mesh_sdf_indexed(&shape, &opts);
            assert!(!mesh.is_empty(), "empty mesh at resolution {resolution}");
            assert!(
                mesh.is_closed_manifold(),
                "not a closed manifold at resolution {resolution}"
            );
            // Component vertices must stay on the surface: within one cell
            // diagonal, like single-vertex cells.
            let diagonal = 2.8 / resolution as f64 * 3.0f64.sqrt();
            for p in &mesh.positions {
                assert!(
                    shape.eval(p).abs() < diagonal,
                    "vertex {p:?} too far from surface at resolution {resolution}"
                );
            }
        }
    }

    /// The convex counterpart creases too (lens-shaped intersection): must
    /// also mesh closed across the same resolutions.
    #[test]
    fn csg_intersection_crease_manifold() {
        let shape = Intersection {
            a: Sphere {
                center: Point3::origin(),
                radius: 1.0,
            },
            b: Sphere {
                center: Point3::new(1.0, 0.0, 0.0),
                radius: 1.0,
            },
        };
        for resolution in [16, 24, 32, 48, 64] {
            let opts = MeshOptions {
                bounds: bounds(1.6),
                resolution,
            };
            let mesh = mesh_sdf_indexed(&shape, &opts);
            assert!(!mesh.is_empty(), "empty mesh at resolution {resolution}");
            assert!(
                mesh.is_closed_manifold(),
                "not a closed manifold at resolution {resolution}"
            );
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

    /// Exercises the multi-component cell path (crease cells hold two
    /// vertices) in both meshers.
    #[test]
    fn parallel_matches_serial_csg_subtraction() {
        let shape = subtracted_spheres();
        let opts = MeshOptions {
            bounds: bounds(1.4),
            resolution: 32,
        };
        assert_parallel_matches_serial(&shape, &opts);
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

    /// Off-axis subtraction that lands one surface component on both sides
    /// of a doubly-crossed face at these resolutions: a cell along the
    /// crease is crossed by a single surface sheet that passes through one
    /// face twice (all four crossings of an ambiguous face in one
    /// component, in both adjacent cells). Without splitting such faces the
    /// four quads around them shared one mesh edge (of-1ad residual).
    fn offset_subtracted_spheres() -> Subtraction<Sphere, Sphere> {
        Subtraction {
            a: Sphere {
                center: Point3::origin(),
                radius: 1.0,
            },
            b: Sphere {
                center: Point3::new(0.7, 0.31, 0.13),
                radius: 0.8,
            },
        }
    }

    #[test]
    fn doubled_face_split_keeps_mesh_manifold() {
        let shape = offset_subtracted_spheres();
        for resolution in [48, 84, 88, 94] {
            let opts = MeshOptions {
                bounds: bounds(1.4),
                resolution,
            };
            let mesh = mesh_sdf_indexed(&shape, &opts);
            assert!(!mesh.is_empty(), "empty mesh at resolution {resolution}");
            assert!(
                mesh.is_closed_manifold(),
                "not a closed manifold at resolution {resolution}"
            );
            // Split vertices sit at crossing midpoints, so they obey the
            // same one-cell-diagonal surface distance bound as cell
            // vertices.
            let diagonal = 2.8 / resolution as f64 * 3.0f64.sqrt();
            for p in &mesh.positions {
                assert!(
                    shape.eval(p).abs() < diagonal,
                    "vertex {p:?} too far from surface at resolution {resolution}"
                );
            }
        }
    }

    #[test]
    fn parallel_matches_serial_doubled_face_split() {
        let shape = offset_subtracted_spheres();
        let opts = MeshOptions {
            bounds: bounds(1.4),
            resolution: 48,
        };
        assert_parallel_matches_serial(&shape, &opts);
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
