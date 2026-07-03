//! Shared indexed triangle mesh: the interchange type between F-Rep meshing,
//! B-Rep tessellation, and exporters.

use std::collections::HashMap;

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
            indices: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
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
