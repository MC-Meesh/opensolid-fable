//! Shared indexed triangle mesh: the interchange type between F-Rep meshing,
//! B-Rep tessellation, and exporters.

use std::collections::{BTreeMap, HashMap};

use crate::types::{BoundingBox3, Point3, Vector3};

/// A triangle with per-vertex positions and outward unit normals.
#[derive(Debug, Clone)]
pub struct Triangle {
    pub positions: [Point3; 3],
    pub normals: [Vector3; 3],
}

/// Indexed triangle mesh: shared vertices referenced by index triples.
///
/// `positions` and `normals` are parallel arrays; `indices` holds one
/// `[usize; 3]` per triangle, referencing both.
#[derive(Debug, Clone, Default)]
pub struct TriangleMesh {
    pub positions: Vec<Point3>,
    pub normals: Vec<Vector3>,
    pub indices: Vec<[usize; 3]>,
}

impl TriangleMesh {
    /// An empty mesh.
    pub fn new() -> Self {
        Self::default()
    }

    /// Number of triangles.
    pub fn triangle_count(&self) -> usize {
        self.indices.len()
    }

    /// Number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.positions.len()
    }

    /// True if the mesh has no triangles.
    pub fn is_empty(&self) -> bool {
        self.indices.is_empty()
    }

    /// Build an indexed mesh from a triangle soup, three vertices per
    /// triangle with no sharing. Use [`TriangleMesh::weld`] afterwards to
    /// merge coincident vertices.
    pub fn from_triangles(triangles: &[Triangle]) -> Self {
        let mut mesh = Self {
            positions: Vec::with_capacity(triangles.len() * 3),
            normals: Vec::with_capacity(triangles.len() * 3),
            indices: Vec::with_capacity(triangles.len()),
        };
        for tri in triangles {
            let base = mesh.positions.len();
            mesh.positions.extend_from_slice(&tri.positions);
            mesh.normals.extend_from_slice(&tri.normals);
            mesh.indices.push([base, base + 1, base + 2]);
        }
        mesh
    }

    /// Expand the indexed mesh into a triangle soup.
    pub fn to_triangles(&self) -> Vec<Triangle> {
        self.indices
            .iter()
            .map(|tri| Triangle {
                positions: [
                    self.positions[tri[0]],
                    self.positions[tri[1]],
                    self.positions[tri[2]],
                ],
                normals: [
                    self.normals[tri[0]],
                    self.normals[tri[1]],
                    self.normals[tri[2]],
                ],
            })
            .collect()
    }

    /// Merge vertices whose positions are within `epsilon` of each other and
    /// drop triangles left degenerate (two or more identical indices) by the
    /// merge. `epsilon <= 0` merges only bit-identical positions.
    ///
    /// Merged vertices average their normals (renormalized; zero if the
    /// average cancels). Unreferenced vertices are discarded.
    pub fn weld(&self, epsilon: f64) -> Self {
        let mut positions: Vec<Point3> = Vec::new();
        let mut normal_sums: Vec<Vector3> = Vec::new();
        // Old vertex index -> welded index, filled lazily so unreferenced
        // vertices never enter the output.
        let mut remap: Vec<Option<usize>> = vec![None; self.positions.len()];
        // Spatial hash on cells of size epsilon; a match can sit in any of
        // the 27 cells around a query point.
        let mut grid: HashMap<(i64, i64, i64), Vec<usize>> = HashMap::new();
        let cell = |p: &Point3| -> (i64, i64, i64) {
            if epsilon > 0.0 {
                (
                    (p.x / epsilon).floor() as i64,
                    (p.y / epsilon).floor() as i64,
                    (p.z / epsilon).floor() as i64,
                )
            } else {
                (
                    p.x.to_bits() as i64,
                    p.y.to_bits() as i64,
                    p.z.to_bits() as i64,
                )
            }
        };

        let mut weld_vertex =
            |old: usize, positions: &mut Vec<Point3>, normal_sums: &mut Vec<Vector3>| -> usize {
                if let Some(new) = remap[old] {
                    return new;
                }
                let p = self.positions[old];
                let (kx, ky, kz) = cell(&p);
                let mut found = None;
                if epsilon > 0.0 {
                    'search: for dx in -1..=1i64 {
                        for dy in -1..=1i64 {
                            for dz in -1..=1i64 {
                                let Some(cands) = grid.get(&(kx + dx, ky + dy, kz + dz)) else {
                                    continue;
                                };
                                for &cand in cands {
                                    if (positions[cand] - p).norm_squared() <= epsilon * epsilon {
                                        found = Some(cand);
                                        break 'search;
                                    }
                                }
                            }
                        }
                    }
                } else if let Some(cands) = grid.get(&(kx, ky, kz)) {
                    found = cands.iter().copied().find(|&cand| positions[cand] == p);
                }
                let new = found.unwrap_or_else(|| {
                    let new = positions.len();
                    positions.push(p);
                    normal_sums.push(Vector3::zeros());
                    grid.entry((kx, ky, kz)).or_default().push(new);
                    new
                });
                normal_sums[new] += self.normals[old];
                remap[old] = Some(new);
                new
            };

        let mut indices = Vec::with_capacity(self.indices.len());
        for tri in &self.indices {
            let mapped = [
                weld_vertex(tri[0], &mut positions, &mut normal_sums),
                weld_vertex(tri[1], &mut positions, &mut normal_sums),
                weld_vertex(tri[2], &mut positions, &mut normal_sums),
            ];
            if mapped[0] != mapped[1] && mapped[1] != mapped[2] && mapped[0] != mapped[2] {
                indices.push(mapped);
            }
        }

        let normals = normal_sums
            .into_iter()
            .map(|sum| {
                let norm = sum.norm();
                if norm > 1e-12 {
                    sum / norm
                } else {
                    Vector3::zeros()
                }
            })
            .collect();

        Self {
            positions,
            normals,
            indices,
        }
    }

    /// True if the mesh is a closed, consistently oriented 2-manifold:
    /// every undirected edge is shared by exactly two triangles with
    /// opposite directions, no triangle has a degenerate edge, and all
    /// indices are in bounds. An empty mesh is not considered closed.
    pub fn is_closed_manifold(&self) -> bool {
        if self.indices.is_empty() {
            return false;
        }
        let n = self.positions.len();
        // Per undirected edge: (use count, sum of directions).
        let mut edges: HashMap<(usize, usize), (u32, i64)> = HashMap::new();
        for tri in &self.indices {
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                if a == b || a >= n || b >= n {
                    return false;
                }
                let entry = edges.entry((a.min(b), a.max(b))).or_insert((0, 0));
                entry.0 += 1;
                entry.1 += if a < b { 1 } else { -1 };
            }
        }
        edges
            .values()
            .all(|&(count, dir_sum)| count == 2 && dir_sum == 0)
    }

    /// Axis-aligned bounding box of all vertices referenced by triangles;
    /// `None` if the mesh has no triangles.
    pub fn bounding_box(&self) -> Option<BoundingBox3> {
        let mut bbox: Option<BoundingBox3> = None;
        for tri in &self.indices {
            for &i in tri {
                let p = self.positions[i];
                bbox = Some(match bbox {
                    None => BoundingBox3::new(p, p),
                    Some(b) => BoundingBox3::new(
                        Point3::new(b.min.x.min(p.x), b.min.y.min(p.y), b.min.z.min(p.z)),
                        Point3::new(b.max.x.max(p.x), b.max.y.max(p.y), b.max.z.max(p.z)),
                    ),
                });
            }
        }
        bbox
    }

    /// Sum of all triangle areas.
    pub fn total_area(&self) -> f64 {
        self.indices
            .iter()
            .map(|tri| {
                let e1 = self.positions[tri[1]] - self.positions[tri[0]];
                let e2 = self.positions[tri[2]] - self.positions[tri[0]];
                e1.cross(&e2).norm() * 0.5
            })
            .sum()
    }

    /// Geometric (position-derived) unit normal of a triangle, or `None`
    /// when the triangle is degenerate (near-zero area). Independent of the
    /// stored per-vertex normals so feature/silhouette walks see the actual
    /// facet orientation.
    fn face_normal(&self, tri: &[usize; 3]) -> Option<Vector3> {
        let a = self.positions[tri[0]];
        let e1 = self.positions[tri[1]] - a;
        let e2 = self.positions[tri[2]] - a;
        let n = e1.cross(&e2);
        let len = n.norm();
        if len > 1e-12 { Some(n / len) } else { None }
    }

    /// Map each undirected edge (min-index, max-index) to the triangles that
    /// use it. Deterministic order (BTreeMap) so downstream edge buffers are
    /// reproducible. Operates on index topology, so coincident-but-unshared
    /// vertices break adjacency — [`weld`](Self::weld) a soup first.
    fn edge_adjacency(&self) -> BTreeMap<(usize, usize), Vec<usize>> {
        let mut adjacency: BTreeMap<(usize, usize), Vec<usize>> = BTreeMap::new();
        for (fi, tri) in self.indices.iter().enumerate() {
            for e in 0..3 {
                let a = tri[e];
                let b = tri[(e + 1) % 3];
                adjacency.entry((a.min(b), a.max(b))).or_default().push(fi);
            }
        }
        adjacency
    }

    /// Crease (feature) edges: the drawing solid-line source. Returns the
    /// endpoint positions of every undirected edge whose two adjacent faces
    /// meet at a dihedral angle of at least `min_dihedral` radians, together
    /// with boundary edges (used by a single face) and non-manifold edges
    /// (used by three or more), which are always creases. Coplanar
    /// tessellation seams (dihedral below the threshold) are dropped.
    ///
    /// The dihedral test compares geometric face normals: two faces are a
    /// crease when `n0 · n1 <= cos(min_dihedral)`. Operates on index
    /// topology (see [`edge_adjacency`](Self::edge_adjacency)).
    pub fn feature_edges(&self, min_dihedral: f64) -> Vec<[Point3; 2]> {
        let cos_threshold = min_dihedral.cos();
        let mut out = Vec::new();
        for (&(a, b), faces) in &self.edge_adjacency() {
            let is_feature = match faces.as_slice() {
                [f0, f1] => match (
                    self.face_normal(&self.indices[*f0]),
                    self.face_normal(&self.indices[*f1]),
                ) {
                    (Some(n0), Some(n1)) => n0.dot(&n1) <= cos_threshold,
                    // A degenerate neighbour has no reliable orientation;
                    // treat the shared edge as a crease rather than smoothing
                    // over a hole in the topology.
                    _ => true,
                },
                // Boundary (single face) or non-manifold (3+): always a crease.
                _ => true,
            };
            if is_feature {
                out.push([self.positions[a], self.positions[b]]);
            }
        }
        out
    }

    /// Silhouette (outline) edges for an orthographic view along `view_dir`:
    /// the endpoint positions of every undirected edge whose two adjacent
    /// faces face opposite ways relative to the view — the sign of
    /// `face_normal · view_dir` flips across the edge — plus boundary edges,
    /// which always bound the outline. This is the mesh-resolution
    /// approximation of the smooth-surface silhouette locus `n·view_dir = 0`;
    /// it is view-dependent and must be recomputed per view. `view_dir` need
    /// not be normalized. Operates on index topology.
    pub fn silhouette_edges(&self, view_dir: Vector3) -> Vec<[Point3; 2]> {
        let mut out = Vec::new();
        for (&(a, b), faces) in &self.edge_adjacency() {
            let is_silhouette = match faces.as_slice() {
                [f0, f1] => match (
                    self.face_normal(&self.indices[*f0]),
                    self.face_normal(&self.indices[*f1]),
                ) {
                    (Some(n0), Some(n1)) => (n0.dot(&view_dir) > 0.0) != (n1.dot(&view_dir) > 0.0),
                    _ => false,
                },
                // Boundary edge: part of the outline for any view.
                [_] => true,
                // Non-manifold: no well-defined front/back pair; skip.
                _ => false,
            };
            if is_silhouette {
                out.push([self.positions[a], self.positions[b]]);
            }
        }
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_normal() -> Vector3 {
        Vector3::z()
    }

    fn tri(a: Point3, b: Point3, c: Point3) -> Triangle {
        Triangle {
            positions: [a, b, c],
            normals: [unit_normal(); 3],
        }
    }

    /// Regular tetrahedron with outward-wound faces, as an indexed mesh.
    fn tetrahedron() -> TriangleMesh {
        let v = [
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
        ];
        let normals = v.iter().map(|p| p.coords.normalize()).collect::<Vec<_>>();
        TriangleMesh {
            positions: v.to_vec(),
            normals,
            indices: vec![[0, 1, 2], [0, 3, 1], [0, 2, 3], [1, 3, 2]],
        }
    }

    #[test]
    fn from_to_triangles_roundtrip() {
        let soup = vec![
            tri(
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ),
            tri(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ),
        ];
        let mesh = TriangleMesh::from_triangles(&soup);
        assert_eq!(mesh.triangle_count(), 2);
        assert_eq!(mesh.vertex_count(), 6);
        let back = mesh.to_triangles();
        assert_eq!(back.len(), soup.len());
        for (a, b) in soup.iter().zip(&back) {
            assert_eq!(a.positions, b.positions);
            assert_eq!(a.normals, b.normals);
        }
    }

    #[test]
    fn weld_collapses_duplicates() {
        // Two triangles sharing an edge, with the shared vertices duplicated
        // and perturbed by less than epsilon.
        let d = 1e-9;
        let soup = vec![
            tri(
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ),
            tri(
                Point3::new(1.0 + d, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(d, 1.0, 0.0),
            ),
        ];
        let welded = TriangleMesh::from_triangles(&soup).weld(1e-6);
        assert_eq!(welded.triangle_count(), 2);
        assert_eq!(welded.vertex_count(), 4, "shared edge vertices not merged");
        for n in &welded.normals {
            assert!((n.norm() - 1.0).abs() < 1e-9, "welded normal not unit");
        }
    }

    #[test]
    fn weld_zero_epsilon_merges_only_exact() {
        let d = 1e-9;
        let soup = vec![
            tri(
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ),
            tri(
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(d, 1.0, 0.0),
            ),
        ];
        let welded = TriangleMesh::from_triangles(&soup).weld(0.0);
        // Exactly one duplicate is bit-identical; the perturbed one stays.
        assert_eq!(welded.vertex_count(), 5);
    }

    #[test]
    fn weld_drops_degenerate_triangles() {
        let d = 1e-9;
        let soup = vec![
            tri(
                Point3::origin(),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ),
            // Sliver whose vertices all collapse onto the first triangle's.
            tri(
                Point3::new(d, 0.0, 0.0),
                Point3::new(1.0 + d, 0.0, 0.0),
                Point3::new(1.0, d, 0.0),
            ),
        ];
        let welded = TriangleMesh::from_triangles(&soup).weld(1e-3);
        assert_eq!(welded.triangle_count(), 1, "degenerate triangle kept");
        assert_eq!(welded.vertex_count(), 3);
    }

    #[test]
    fn weld_discards_unreferenced_vertices() {
        let mut mesh = TriangleMesh::from_triangles(&[tri(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        )]);
        mesh.positions.push(Point3::new(5.0, 5.0, 5.0));
        mesh.normals.push(unit_normal());
        let welded = mesh.weld(1e-6);
        assert_eq!(welded.vertex_count(), 3);
    }

    #[test]
    fn tetrahedron_is_closed_manifold() {
        let mesh = tetrahedron();
        assert!(mesh.is_closed_manifold());
        // Welding an already-indexed closed mesh must keep it closed.
        assert!(mesh.weld(1e-9).is_closed_manifold());
        // Round-tripping through a soup and welding recovers closedness.
        let soup = mesh.to_triangles();
        assert!(
            TriangleMesh::from_triangles(&soup)
                .weld(1e-9)
                .is_closed_manifold()
        );
    }

    #[test]
    fn open_mesh_is_not_closed_manifold() {
        let mesh = TriangleMesh::from_triangles(&[tri(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        )]);
        assert!(!mesh.is_closed_manifold());
    }

    #[test]
    fn inconsistent_winding_is_not_manifold() {
        let mut mesh = tetrahedron();
        mesh.indices[0].swap(1, 2); // flip one face
        assert!(!mesh.is_closed_manifold());
    }

    #[test]
    fn empty_mesh_is_not_closed_manifold() {
        let mesh = TriangleMesh::new();
        assert!(!mesh.is_closed_manifold());
        assert!(mesh.is_empty());
        assert!(mesh.bounding_box().is_none());
        assert_eq!(mesh.total_area(), 0.0);
    }

    #[test]
    fn degenerate_edge_is_not_manifold() {
        let mesh = TriangleMesh {
            positions: vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            normals: vec![unit_normal(); 2],
            indices: vec![[0, 0, 1]],
        };
        assert!(!mesh.is_closed_manifold());
    }

    #[test]
    fn out_of_bounds_index_is_not_manifold() {
        let mesh = TriangleMesh {
            positions: vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)],
            normals: vec![unit_normal(); 2],
            indices: vec![[0, 1, 7]],
        };
        assert!(!mesh.is_closed_manifold());
    }

    #[test]
    fn bounding_box_covers_referenced_vertices() {
        let mesh = tetrahedron();
        let bbox = mesh.bounding_box().expect("non-empty mesh has a bbox");
        assert_eq!(bbox.min, Point3::new(-1.0, -1.0, -1.0));
        assert_eq!(bbox.max, Point3::new(1.0, 1.0, 1.0));
    }

    #[test]
    fn bounding_box_ignores_unreferenced_vertices() {
        let mut mesh = TriangleMesh::from_triangles(&[tri(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        )]);
        mesh.positions.push(Point3::new(100.0, 0.0, 0.0));
        mesh.normals.push(unit_normal());
        let bbox = mesh.bounding_box().unwrap();
        assert_eq!(bbox.max, Point3::new(1.0, 1.0, 0.0));
    }

    /// Canonicalize an edge segment set into an order-independent set of
    /// sorted endpoint-index pairs, resolving positions back to `mesh`
    /// vertices by exact coordinate match.
    fn edge_key_set(mesh: &TriangleMesh, edges: &[[Point3; 2]]) -> Vec<(usize, usize)> {
        let index_of = |p: &Point3| mesh.positions.iter().position(|q| q == p).unwrap();
        let mut keys: Vec<(usize, usize)> = edges
            .iter()
            .map(|[a, b]| {
                let (ia, ib) = (index_of(a), index_of(b));
                (ia.min(ib), ia.max(ib))
            })
            .collect();
        keys.sort_unstable();
        keys
    }

    /// A unit cube centered at the origin as a welded, closed manifold: 8
    /// shared vertices, 12 triangles. Outward-wound, geometric face normals
    /// axis-aligned.
    fn cube() -> TriangleMesh {
        let v = [
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
        ];
        // Outward-wound (CCW seen from outside) two triangles per face.
        let indices = vec![
            [0, 3, 2],
            [0, 2, 1], // -z (bottom)
            [4, 5, 6],
            [4, 6, 7], // +z (top)
            [0, 1, 5],
            [0, 5, 4], // -y
            [2, 3, 7],
            [2, 7, 6], // +y
            [1, 2, 6],
            [1, 6, 5], // +x
            [0, 4, 7],
            [0, 7, 3], // -x
        ];
        TriangleMesh {
            positions: v.to_vec(),
            normals: vec![Vector3::z(); 8],
            indices,
        }
    }

    #[test]
    fn cube_is_closed_manifold() {
        assert!(cube().is_closed_manifold());
    }

    #[test]
    fn feature_edges_are_the_cube_wireframe() {
        let mesh = cube();
        // A 90° dihedral between the box faces; the coplanar face-diagonal
        // seams (0°) must be dropped, leaving exactly the 12 cube edges.
        let edges = mesh.feature_edges(45.0_f64.to_radians());
        assert_eq!(edges.len(), 12, "expected the 12 cube edges");
        let keys = edge_key_set(&mesh, &edges);
        // No face-diagonal (a seam like 0-2) survives.
        assert!(
            !keys.contains(&(0, 2)) && !keys.contains(&(4, 6)),
            "coplanar seam leaked into feature edges"
        );
        // All emitted edges are real cube edges (length 2), never diagonals.
        for [a, b] in &edges {
            assert!(((a - b).norm() - 2.0).abs() < 1e-9, "edge is a diagonal");
        }
    }

    #[test]
    fn feature_edges_threshold_above_ninety_drops_everything() {
        // With a threshold steeper than any real dihedral (90°), no crease
        // qualifies — a fully smooth read of the surface.
        let mesh = cube();
        assert!(mesh.feature_edges(91.0_f64.to_radians()).is_empty());
    }

    #[test]
    fn feature_edges_include_open_boundary() {
        // A single triangle: all three edges are boundary edges, hence
        // features regardless of dihedral threshold.
        let mesh = TriangleMesh::from_triangles(&[tri(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        )]);
        assert_eq!(mesh.feature_edges(45.0_f64.to_radians()).len(), 3);
    }

    #[test]
    fn silhouette_edges_ring_the_cube_outline() {
        let mesh = cube();
        // Looking straight down +z: only the top (+z) faces are back-facing
        // (n·dir > 0); the bottom and (edge-on) side faces are not. The sign
        // flips exactly around the top rim, which projects to the square
        // outline — 4 edges.
        let keys = edge_key_set(&mesh, &mesh.silhouette_edges(Vector3::z()));
        assert_eq!(keys, vec![(4, 5), (4, 7), (5, 6), (6, 7)]);
        // No vertical edge (e.g. 0-4, along z) flips.
        assert!(!keys.contains(&(0, 4)));
    }

    #[test]
    fn silhouette_edges_corner_view_is_a_hexagon() {
        // Viewed corner-on (1,1,1), three faces are back-facing and three
        // front-facing; the outline is the classic 6-edge hexagon.
        let mesh = cube();
        let keys = edge_key_set(&mesh, &mesh.silhouette_edges(Vector3::new(1.0, 1.0, 1.0)));
        assert_eq!(keys, vec![(1, 2), (1, 5), (2, 3), (3, 7), (4, 5), (4, 7)]);
    }

    #[test]
    fn silhouette_edges_flip_with_view_direction() {
        // The +x outline (4 edges around the +x face) differs from the +z
        // outline (top rim) — silhouettes are view-dependent.
        let mesh = cube();
        let sx = edge_key_set(&mesh, &mesh.silhouette_edges(Vector3::x()));
        let sz = edge_key_set(&mesh, &mesh.silhouette_edges(Vector3::z()));
        assert_eq!(sx.len(), 4);
        assert_eq!(sz.len(), 4);
        assert_ne!(sx, sz, "silhouette must be view-dependent");
    }

    #[test]
    fn silhouette_edges_include_open_boundary() {
        let mesh = TriangleMesh::from_triangles(&[tri(
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        )]);
        // A lone triangle's three edges are all boundary — always silhouette.
        assert_eq!(mesh.silhouette_edges(Vector3::z()).len(), 3);
    }

    #[test]
    fn counts_and_area() {
        let mesh = tetrahedron();
        assert_eq!(mesh.triangle_count(), 4);
        assert_eq!(mesh.vertex_count(), 4);
        // Regular tetrahedron with edge length 2*sqrt(2): area = sqrt(3) * a^2.
        let a: f64 = 8.0f64.sqrt();
        let expected = 3.0f64.sqrt() * a * a;
        assert!((mesh.total_area() - expected).abs() < 1e-9);
    }
}
