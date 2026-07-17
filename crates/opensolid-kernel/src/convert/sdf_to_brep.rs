//! F-Rep → B-Rep: mesh-backed body with planar region recovery.
//!
//! [`sdf_to_brep`] converts an implicit body into a faceted B-Rep solid:
//!
//! 1. **Mesh** the SDF with adaptive octree dual contouring
//!    ([`mesh_sdf_adaptive_indexed`]). QEF vertex placement puts mesh
//!    vertices exactly onto sharp features, so the facets of a polyhedral
//!    field land exactly in its bounding planes — which is what makes the
//!    next step recover them. The mesh is welded at a tiny epsilon to
//!    remove near-coincident vertices and the needle slivers they carry;
//!    slivers that survive (nearly collinear vertices) get their normal
//!    from the field gradient instead of their degenerate winding.
//! 2. **Cluster** triangles into planar regions by region growing: a
//!    triangle joins a region when its normal is within
//!    [`planar_angle_tol`](SdfToBrepOptions::planar_angle_tol) of the
//!    region seed's normal and all its vertices lie within
//!    [`planar_offset_tol`](SdfToBrepOptions::planar_offset_tol) of the
//!    seed's plane. Comparing against the fixed seed plane (not a running
//!    average) prevents tolerance drift from chaining gently curved facets
//!    into one bogus "plane".
//! 3. **Recover faces**: each region becomes one planar face. Its loops are
//!    traced from the region's boundary (directed triangle edges whose
//!    neighbor lies in another region); the loop winding positively about
//!    the region normal is the outer loop, the rest are holes. Curved
//!    regions never merge past a single triangle, so they remain as
//!    triangulated face sets — one planar face per facet. Regions whose
//!    boundary is not a set of simple loops (pinched vertices, or a region
//!    swallowing a whole closed component) dissolve back into per-triangle
//!    faces rather than producing invalid topology.
//! 4. **Assemble** a [`TopologyStore`] body: one closed outward shell per
//!    connected mesh component with its genus derived from the component's
//!    Euler characteristic, shared vertices and edges keyed by mesh
//!    indices, a [`Surface3::Plane`] on every face and a line [`Curve3`]
//!    (arc-length parameter range) on every edge. The result passes
//!    [`TopologyStore::check`].
//!
//! # Roadmap to real NURBS fitting
//!
//! This MVP recovers exact planes only; everything curved stays faceted.
//! The planned follow-ups, in dependency order:
//!
//! - **Boundary simplification**: merge runs of collinear boundary edges of
//!   a planar face into single line edges (today each mesh lattice segment
//!   is its own B-Rep edge), then fit circular arcs to boundary runs whose
//!   adjacent region is a recognized quadric.
//! - **Quadric recognition**: detect cylinder/sphere/cone/torus regions
//!   from principal-curvature statistics of the mesh (the same
//!   normal+offset clustering generalizes: cluster in curvature space) and
//!   replace their triangle fans with a single analytic face.
//! - **General NURBS fitting**: least-squares fit of a
//!   [`NurbsSurface`](opensolid_brep::NurbsSurface) over each remaining
//!   curved region's parameterized triangle patch, with the fitted
//!   deviation recorded as edge/vertex tolerance (tolerant modeling,
//!   `spec/08-tolerances.md`).
//! - **Feature-line extraction**: snap region boundaries to SDF feature
//!   curves (gradient discontinuities) instead of mesh lattice polylines,
//!   decoupling B-Rep edge accuracy from the meshing resolution.

use std::collections::HashMap;

use opensolid_brep::{
    Body, BodyType, Curve3, Edge, FaceSense, FinSense, GeometryStore, LoopType, SYSTEM_RESOLUTION,
    ShellOrientation, Surface3, TopologyStore, Vertex,
};
use opensolid_core::EntityId;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_frep::mesh_adaptive::{AdaptiveMeshOptions, mesh_sdf_adaptive_indexed};
use opensolid_frep::primitives::Sdf;

/// Options controlling F-Rep → B-Rep conversion.
#[derive(Debug, Clone, Copy)]
pub struct SdfToBrepOptions {
    /// Adaptive meshing options. The surface must lie strictly inside
    /// `mesh.bounds`, or the mesh comes out open and conversion fails.
    pub mesh: AdaptiveMeshOptions,
    /// Maximum angle (radians) between a triangle's normal and its region
    /// seed's normal for the triangle to join the planar region.
    pub planar_angle_tol: f64,
    /// Maximum distance from any of a triangle's vertices to the region
    /// seed's plane for the triangle to join the planar region.
    pub planar_offset_tol: f64,
}

impl SdfToBrepOptions {
    /// Options with default clustering tolerances: an angle tolerance far
    /// below any real facet dihedral (adjacent facets of a curved surface
    /// at practical depths differ by ~`2π / 2^max_depth` radians) and an
    /// offset tolerance proportional to the region diagonal, both a few
    /// orders above QEF/f64 noise.
    pub fn new(bounds: BoundingBox3, max_depth: u32) -> Self {
        let diagonal = (bounds.max - bounds.min).norm();
        Self {
            mesh: AdaptiveMeshOptions {
                bounds,
                max_depth,
                accuracy: None,
            },
            planar_angle_tol: 1e-4,
            planar_offset_tol: 1e-7 * diagonal,
        }
    }
}

/// Convert an SDF into a faceted B-Rep solid body. See the [module
/// docs](self) for the algorithm; the returned body passes
/// [`TopologyStore::check`].
///
/// The geometry is a faceted approximation: every face is planar, and the
/// body's boundary deviates from the SDF's zero set by at most the meshing
/// chordal error (shrink it by raising `opts.mesh.max_depth`). Planar
/// regions of the field are recovered exactly, one face each.
///
/// Both stores are only mutated on success.
///
/// # Errors
/// [`CoreError::InvalidArgument`] if the surface does not cross
/// `opts.mesh.bounds`; [`CoreError::Degenerate`] if meshing does not
/// produce a closed manifold (surface not strictly inside the bounds) or
/// produces a zero-area triangle.
pub fn sdf_to_brep(
    sdf: &dyn Sdf,
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    opts: &SdfToBrepOptions,
) -> CoreResult<EntityId<Body>> {
    // Weld near-coincident vertices (adjacent cells' QEF vertices can clamp
    // onto virtually the same feature point), dropping the needle slivers
    // and zero-length edges they would otherwise inject into the topology.
    let diagonal = (opts.mesh.bounds.max - opts.mesh.bounds.min).norm();
    let mesh = mesh_sdf_adaptive_indexed(sdf, &opts.mesh).weld(1e-12 * diagonal);
    if mesh.is_empty() {
        return Err(CoreError::InvalidArgument {
            argument: "sdf",
            reason: "surface does not cross the meshing bounds".to_string(),
        });
    }
    // Name the actual defect rather than asserting one cause. The old text
    // blamed the meshing bounds unconditionally, which is wrong — and
    // misleading — for the pinched-edge case, where the surface is strictly
    // inside the bounds and no accuracy resolves it (of-o0o).
    let defects = mesh.manifold_defects();
    if let Some(reason) = defects.describe() {
        return Err(CoreError::Degenerate {
            context: "sdf_to_brep",
            reason: format!("adaptive meshing did not produce a closed manifold: {reason}"),
        });
    }

    let normals = triangle_normals(sdf, &mesh)?;
    let neighbors = triangle_neighbors(&mesh);
    let components = connected_components(&neighbors);
    let genus = component_genus(&mesh, &components)?;
    let (mut region_of, mut regions) = cluster_planar_regions(&mesh, &normals, &neighbors, opts);
    let faces = trace_faces(&mesh, &normals, &neighbors, &mut region_of, &mut regions);
    build_body(store, geo, &mesh, &components, &genus, &faces)
}

/// One recovered face: a planar region's fitted plane plus its boundary
/// loops as closed mesh-vertex cycles, outer loop first.
struct FaceSpec {
    /// Any triangle of the region (for shell assignment).
    tri: usize,
    /// Unit area-weighted region normal (outward).
    normal: Vector3,
    /// Area-weighted centroid, the plane's origin.
    origin: Point3,
    /// Vertex cycles; `loops[0]` winds positively about `normal` (outer),
    /// the rest negatively (holes).
    loops: Vec<Vec<usize>>,
}

/// Unit geometric normal per triangle (from winding, which the mesher
/// orients outward). Slivers that survive welding (three nearly collinear
/// vertices) have numerically meaningless winding normals; those fall back
/// to the field gradient at the centroid, which keeps them clusterable
/// into the region they geometrically belong to.
fn triangle_normals(sdf: &dyn Sdf, mesh: &TriangleMesh) -> CoreResult<Vec<Vector3>> {
    mesh.indices
        .iter()
        .enumerate()
        .map(|(t, tri)| {
            let [a, b, c] = tri.map(|i| mesh.positions[i]);
            let cross = (b - a).cross(&(c - a));
            let norm = cross.norm();
            let longest_sq = (b - a)
                .norm_squared()
                .max((c - a).norm_squared())
                .max((c - b).norm_squared());
            if norm > 1e-12 * longest_sq {
                return Ok(cross / norm);
            }
            let centroid = Point3::from((a.coords + b.coords + c.coords) / 3.0);
            let grad = sdf.grad(&centroid);
            let grad_norm = grad.norm();
            if grad_norm <= 1e-12 {
                return Err(CoreError::Degenerate {
                    context: "sdf_to_brep",
                    reason: format!(
                        "mesh triangle {t} has negligible area and a vanishing field gradient"
                    ),
                });
            }
            Ok(grad / grad_norm)
        })
        .collect()
}

/// For each triangle, the neighbor across each of its three directed edges
/// `(tri[k], tri[k+1])`. Total (two triangles per undirected edge) is
/// guaranteed by the closed-manifold check.
fn triangle_neighbors(mesh: &TriangleMesh) -> Vec<[usize; 3]> {
    let mut edge_tris: HashMap<(usize, usize), (usize, Option<usize>)> =
        HashMap::with_capacity(mesh.indices.len() * 3 / 2);
    for (t, tri) in mesh.indices.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            edge_tris
                .entry((a.min(b), a.max(b)))
                .and_modify(|e| e.1 = Some(t))
                .or_insert((t, None));
        }
    }
    let mut neighbors = vec![[usize::MAX; 3]; mesh.indices.len()];
    for (t, tri) in mesh.indices.iter().enumerate() {
        for k in 0..3 {
            let (a, b) = (tri[k], tri[(k + 1) % 3]);
            let (t0, t1) = edge_tris[&(a.min(b), a.max(b))];
            let t1 = t1.expect("closed manifold: two triangles per edge");
            neighbors[t][k] = if t0 == t { t1 } else { t0 };
        }
    }
    neighbors
}

/// Connected-component id per triangle (flood fill over edge adjacency).
/// Ids are contiguous from zero in first-triangle order.
fn connected_components(neighbors: &[[usize; 3]]) -> Vec<usize> {
    let mut comp_of = vec![usize::MAX; neighbors.len()];
    let mut count = 0;
    for seed in 0..neighbors.len() {
        if comp_of[seed] != usize::MAX {
            continue;
        }
        comp_of[seed] = count;
        let mut stack = vec![seed];
        while let Some(t) = stack.pop() {
            for &u in &neighbors[t] {
                if comp_of[u] == usize::MAX {
                    comp_of[u] = count;
                    stack.push(u);
                }
            }
        }
        count += 1;
    }
    comp_of
}

/// Genus per connected component from the Euler characteristic of its
/// triangle mesh: `χ = V - E + F` with `E = 3F/2`, and `χ = 2 - 2g` for a
/// closed orientable surface.
fn component_genus(mesh: &TriangleMesh, comp_of: &[usize]) -> CoreResult<Vec<u32>> {
    let count = comp_of.iter().copied().max().map_or(0, |m| m + 1);
    let mut tri_count = vec![0i64; count];
    let mut vert_count = vec![0i64; count];
    // A vertex belongs to exactly one component (triangles sharing it are
    // edge-connected around it on a manifold), so a single marker suffices.
    let mut seen = vec![usize::MAX; mesh.positions.len()];
    for (t, tri) in mesh.indices.iter().enumerate() {
        let c = comp_of[t];
        tri_count[c] += 1;
        for &v in tri {
            if seen[v] != c {
                seen[v] = c;
                vert_count[c] += 1;
            }
        }
    }
    (0..count)
        .map(|c| {
            // χ = V - E + F = V - 3F/2 + F = V - F/2.
            let chi = vert_count[c] - tri_count[c] / 2;
            let two_g = 2 - chi;
            if tri_count[c] % 2 != 0 || two_g < 0 || two_g % 2 != 0 {
                return Err(CoreError::Degenerate {
                    context: "sdf_to_brep",
                    reason: format!(
                        "mesh component {c} is not a closed orientable surface (χ = {chi})"
                    ),
                });
            }
            Ok((two_g / 2) as u32)
        })
        .collect()
}

/// Partition triangles into planar regions by region growing against each
/// region's seed plane. Returns (region id per triangle, member lists).
fn cluster_planar_regions(
    mesh: &TriangleMesh,
    normals: &[Vector3],
    neighbors: &[[usize; 3]],
    opts: &SdfToBrepOptions,
) -> (Vec<usize>, Vec<Vec<usize>>) {
    let cos_tol = opts.planar_angle_tol.cos();
    let mut region_of = vec![usize::MAX; mesh.indices.len()];
    let mut regions: Vec<Vec<usize>> = Vec::new();
    for seed in 0..mesh.indices.len() {
        if region_of[seed] != usize::MAX {
            continue;
        }
        let rid = regions.len();
        let seed_normal = normals[seed];
        let seed_offset = seed_normal.dot(&mesh.positions[mesh.indices[seed][0]].coords);
        region_of[seed] = rid;
        let mut members = vec![seed];
        let mut stack = vec![seed];
        while let Some(t) = stack.pop() {
            for &u in &neighbors[t] {
                if region_of[u] != usize::MAX || normals[u].dot(&seed_normal) < cos_tol {
                    continue;
                }
                let on_plane = mesh.indices[u].iter().all(|&v| {
                    (seed_normal.dot(&mesh.positions[v].coords) - seed_offset).abs()
                        <= opts.planar_offset_tol
                });
                if on_plane {
                    region_of[u] = rid;
                    members.push(u);
                    stack.push(u);
                }
            }
        }
        regions.push(members);
    }
    (region_of, regions)
}

/// Turn every region into a [`FaceSpec`]. A region whose boundary does not
/// trace into simple loops with exactly one outer dissolves into
/// per-triangle regions (appended and processed in the same pass; a single
/// triangle always traces).
fn trace_faces(
    mesh: &TriangleMesh,
    normals: &[Vector3],
    neighbors: &[[usize; 3]],
    region_of: &mut [usize],
    regions: &mut Vec<Vec<usize>>,
) -> Vec<FaceSpec> {
    let mut specs = Vec::new();
    let mut rid = 0;
    while rid < regions.len() {
        let members = regions[rid].clone();
        match trace_region(mesh, normals, neighbors, region_of, rid, &members) {
            Some(spec) => specs.push(spec),
            None => {
                for &t in &members {
                    let fresh = regions.len();
                    region_of[t] = fresh;
                    regions.push(vec![t]);
                }
                regions[rid].clear();
            }
        }
        rid += 1;
    }
    specs
}

/// Fit the region's plane and trace its boundary loops. `None` if the
/// boundary is empty (region covers a whole closed component), pinched
/// (some vertex starts two boundary edges), broken, or does not classify
/// into exactly one outer loop plus holes.
fn trace_region(
    mesh: &TriangleMesh,
    normals: &[Vector3],
    neighbors: &[[usize; 3]],
    region_of: &[usize],
    rid: usize,
    members: &[usize],
) -> Option<FaceSpec> {
    // Area-weighted plane fit: within clustering tolerance of every
    // triangle's plane, and exact for single-triangle regions. Regions with
    // negligible total area (surviving slivers) fall back to their first
    // triangle's (gradient-derived) normal and vertex.
    let mut area_normal = Vector3::zeros();
    let mut centroid = Vector3::zeros();
    let mut total_area = 0.0;
    for &t in members {
        let [a, b, c] = mesh.indices[t].map(|i| mesh.positions[i]);
        let cross = (b - a).cross(&(c - a));
        let area = cross.norm() / 2.0;
        area_normal += cross;
        centroid += (a.coords + b.coords + c.coords) * (area / 3.0);
        total_area += area;
    }
    let norm = area_normal.norm();
    let normal = if norm > 1e-12 {
        area_normal / norm
    } else {
        normals[members[0]]
    };
    let origin = if total_area > 0.0 {
        Point3::from(centroid / total_area)
    } else {
        mesh.positions[mesh.indices[members[0]][0]]
    };

    // Directed boundary edges, sorted for deterministic loop order. The
    // triangles' outward winding makes outer boundaries counterclockwise
    // and hole boundaries clockwise about `normal`, exactly the B-Rep loop
    // convention.
    let mut boundary: Vec<(usize, usize)> = Vec::new();
    for &t in members {
        let tri = mesh.indices[t];
        for k in 0..3 {
            if region_of[neighbors[t][k]] != rid {
                boundary.push((tri[k], tri[(k + 1) % 3]));
            }
        }
    }
    if boundary.is_empty() {
        return None;
    }
    boundary.sort_unstable();

    // Successor lookup: boundary edge starting at each vertex. A vertex
    // starting two boundary edges is a pinch — not a disk-with-holes.
    let mut edge_from: HashMap<usize, usize> = HashMap::with_capacity(boundary.len());
    for (i, &(a, _)) in boundary.iter().enumerate() {
        if edge_from.insert(a, i).is_some() {
            return None;
        }
    }

    let mut used = vec![false; boundary.len()];
    let mut loops: Vec<Vec<usize>> = Vec::new();
    for start in 0..boundary.len() {
        if used[start] {
            continue;
        }
        let mut cycle = Vec::new();
        let mut i = start;
        loop {
            if used[i] {
                // Walked into an already-consumed edge without closing:
                // in-degree > 1 somewhere (pinch on the incoming side).
                return None;
            }
            used[i] = true;
            let (a, b) = boundary[i];
            cycle.push(a);
            i = *edge_from.get(&b)?;
            if i == start {
                break;
            }
        }
        loops.push(cycle);
    }

    // A single simple loop bounds a topological disk: it is the outer loop
    // regardless of its (possibly numerically degenerate) area sign. With
    // holes present, exactly one loop must wind positively about the
    // normal: the outer loop.
    if loops.len() > 1 {
        let mut outer = None;
        for (li, cycle) in loops.iter().enumerate() {
            let mut area2 = Vector3::zeros();
            for (w, &v) in cycle.iter().enumerate() {
                let p = mesh.positions[v].coords;
                let q = mesh.positions[cycle[(w + 1) % cycle.len()]].coords;
                area2 += p.cross(&q);
            }
            if area2.dot(&normal) > 0.0 {
                if outer.is_some() {
                    return None;
                }
                outer = Some(li);
            }
        }
        loops.swap(0, outer?);
    }

    Some(FaceSpec {
        tri: members[0],
        normal,
        origin,
        loops,
    })
}

/// Assemble the B-Rep body from the recovered faces. All fallible geometry
/// construction happens before the first store mutation, so a failure
/// leaves both stores untouched.
fn build_body(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    mesh: &TriangleMesh,
    comp_of: &[usize],
    genus: &[u32],
    faces: &[FaceSpec],
) -> CoreResult<EntityId<Body>> {
    let mut planes = Vec::with_capacity(faces.len());
    for spec in faces {
        planes.push(Surface3::plane(spec.origin, spec.normal)?);
    }

    // Unique undirected edges in first-use order (deterministic).
    let mut edge_index: HashMap<(usize, usize), usize> = HashMap::new();
    let mut edge_keys: Vec<(usize, usize)> = Vec::new();
    for spec in faces {
        for cycle in &spec.loops {
            for (w, &a) in cycle.iter().enumerate() {
                let b = cycle[(w + 1) % cycle.len()];
                let key = (a.min(b), a.max(b));
                edge_index.entry(key).or_insert_with(|| {
                    edge_keys.push(key);
                    edge_keys.len() - 1
                });
            }
        }
    }
    let mut lines = Vec::with_capacity(edge_keys.len());
    for &(lo, hi) in &edge_keys {
        let (p, q) = (mesh.positions[lo], mesh.positions[hi]);
        lines.push((Curve3::line(p, q - p)?, (q - p).norm()));
    }

    let body = store.create_body(BodyType::Solid);
    let shells: Vec<_> = genus
        .iter()
        .map(|&g| {
            let shell = store.create_shell(body, true, ShellOrientation::Outward);
            store.shells.get_mut(shell).expect("just created").genus = g;
            shell
        })
        .collect();

    let mut vertex_ids: HashMap<usize, EntityId<Vertex>> = HashMap::new();
    let mut vertex_id = |store: &mut TopologyStore, v: usize| -> EntityId<Vertex> {
        if let Some(&id) = vertex_ids.get(&v) {
            return id;
        }
        let id = store.create_vertex(mesh.positions[v], SYSTEM_RESOLUTION);
        vertex_ids.insert(v, id);
        id
    };
    let edge_ids: Vec<EntityId<Edge>> = edge_keys
        .iter()
        .zip(lines)
        .map(|(&(lo, hi), (line, length))| {
            let start = vertex_id(store, lo);
            let end = vertex_id(store, hi);
            let id = store.create_edge(start, end, SYSTEM_RESOLUTION);
            let edge = store.edges.get_mut(id).expect("just created");
            edge.curve = Some(geo.add_curve(line));
            edge.t_start = 0.0;
            edge.t_end = length;
            id
        })
        .collect();

    for (spec, plane) in faces.iter().zip(planes) {
        let face = store.create_face(shells[comp_of[spec.tri]], FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface = Some(geo.add_surface(plane));
        for (li, cycle) in spec.loops.iter().enumerate() {
            let loop_edges: Vec<(EntityId<Edge>, FinSense)> = cycle
                .iter()
                .enumerate()
                .map(|(w, &a)| {
                    let b = cycle[(w + 1) % cycle.len()];
                    let edge = edge_ids[edge_index[&(a.min(b), a.max(b))]];
                    let sense = if a < b {
                        FinSense::Forward
                    } else {
                        FinSense::Reversed
                    };
                    (edge, sense)
                })
                .collect();
            let loop_type = if li == 0 {
                LoopType::Outer
            } else {
                LoopType::Inner
            };
            store.create_loop(face, loop_type, &loop_edges);
        }
    }

    Ok(body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_frep::csg::{Subtraction, Union};
    use opensolid_frep::primitives::{Box3, Cylinder, Sphere, Torus};

    fn bounds(half: f64) -> BoundingBox3 {
        BoundingBox3::new(
            Point3::new(-half, -half, -half),
            Point3::new(half, half, half),
        )
    }

    fn convert(
        sdf: &dyn Sdf,
        opts: &SdfToBrepOptions,
    ) -> (TopologyStore, GeometryStore, EntityId<Body>) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = sdf_to_brep(sdf, &mut store, &mut geo, opts).expect("conversion succeeds");
        (store, geo, body)
    }

    #[test]
    fn box_converts_to_six_planar_faces() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 1.0, 1.0],
        };
        let opts = SdfToBrepOptions::new(bounds(1.55), 4);
        let (store, geo, body) = convert(&b, &opts);

        assert_eq!(store.check(body), Vec::new());
        let faces = store.faces_of_body(body);
        assert_eq!(faces.len(), 6, "box must recover exactly its six planes");

        // Each face: a single outer loop on an axis-aligned outward plane at
        // offset 1, one face per side.
        let mut seen = [false; 6];
        for &face_id in &faces {
            let face = store.face(face_id).unwrap();
            assert!(face.inner_loops.is_empty(), "box face has no holes");
            let fins = store.fins_of_loop(face.outer_loop.unwrap()).len();
            assert!(fins >= 4, "outer loop must be a closed polyline");

            let surface = geo.surface(face.surface.expect("face carries geometry"));
            let Some(Surface3::Plane { origin, normal }) = surface else {
                panic!("expected a plane, got {surface:?}");
            };
            let comps = [normal.x, normal.y, normal.z];
            let axis = (0..3)
                .max_by(|&i, &j| comps[i].abs().total_cmp(&comps[j].abs()))
                .unwrap();
            assert!(
                comps[axis].abs() > 1.0 - 1e-9,
                "normal not axis-aligned: {normal:?}"
            );
            let offset = normal.dot(&origin.coords);
            assert!(
                (offset - 1.0).abs() < 0.02,
                "plane not on a box side (outward, offset {offset})"
            );
            let side = 2 * axis + usize::from(comps[axis] > 0.0);
            assert!(!seen[side], "two faces recovered for the same box side");
            seen[side] = true;
        }
        assert_eq!(seen, [true; 6]);
    }

    #[test]
    fn sphere_converts_to_valid_faceted_body() {
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let opts = SdfToBrepOptions::new(bounds(1.6), 4);
        let (store, geo, body) = convert(&s, &opts);

        assert_eq!(store.check(body), Vec::new());

        // The sphere stays a triangulated face set: at most a few
        // exactly-coplanar facet pairs (the two halves of a planar DC quad)
        // merge, everything else is one face per facet.
        let mesh = mesh_sdf_adaptive_indexed(&s, &opts.mesh);
        let faces = store.faces_of_body(body);
        assert!(faces.len() > 100, "sanity: a real faceting, not a fluke");
        assert!(
            faces.len() > mesh.triangle_count() / 2 && faces.len() <= mesh.triangle_count(),
            "{} faces from {} facets: only quad-planar merges expected",
            faces.len(),
            mesh.triangle_count()
        );
        for &face_id in &faces {
            let face = store.face(face_id).unwrap();
            let surface = geo.surface(face.surface.expect("face carries geometry"));
            assert!(matches!(surface, Some(Surface3::Plane { .. })));
        }

        // All B-Rep vertices sit on the meshed sphere.
        let cell = 3.2 / 16.0;
        for (_, v) in store.vertices.iter() {
            assert!(
                (v.point.coords.norm() - 1.0).abs() < cell,
                "vertex {:?} too far from the sphere",
                v.point
            );
        }
    }

    /// A through-hole makes the shell genus 1, and the two flat annular
    /// caps must come back as single faces with one hole loop each — the
    /// inner-loop and genus paths exercised together.
    #[test]
    fn through_hole_yields_genus_one_and_ring_faces() {
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
        let opts = SdfToBrepOptions::new(bounds(1.7), 5);
        let (store, _geo, body) = convert(&shape, &opts);

        assert_eq!(store.check(body), Vec::new());
        let shells = store.shells_of_body(body);
        assert_eq!(shells.len(), 1);
        assert_eq!(store.shell(shells[0]).unwrap().genus, 1);

        let ring_faces = store
            .faces_of_body(body)
            .iter()
            .filter(|&&f| store.face(f).unwrap().inner_loops.len() == 1)
            .count();
        assert_eq!(ring_faces, 2, "top and bottom caps are annuli");
    }

    #[test]
    fn torus_shell_genus_is_one() {
        let t = Torus {
            center: Point3::origin(),
            major_radius: 1.0,
            minor_radius: 0.4,
        };
        let opts = SdfToBrepOptions::new(bounds(1.6), 5);
        let (store, _geo, body) = convert(&t, &opts);

        assert_eq!(store.check(body), Vec::new());
        let shells = store.shells_of_body(body);
        assert_eq!(shells.len(), 1);
        assert_eq!(store.shell(shells[0]).unwrap().genus, 1);
    }

    #[test]
    fn disjoint_solids_become_separate_shells() {
        let two = Union {
            a: Sphere {
                center: Point3::new(-1.5, 0.0, 0.0),
                radius: 0.6,
            },
            b: Sphere {
                center: Point3::new(1.5, 0.0, 0.0),
                radius: 0.6,
            },
        };
        let opts = SdfToBrepOptions::new(
            BoundingBox3::new(Point3::new(-2.4, -1.2, -1.2), Point3::new(2.4, 1.2, 1.2)),
            5,
        );
        let (store, _geo, body) = convert(&two, &opts);

        assert_eq!(store.check(body), Vec::new());
        let shells = store.shells_of_body(body);
        assert_eq!(shells.len(), 2);
        for &shell in shells {
            assert_eq!(store.shell(shell).unwrap().genus, 0);
        }
    }

    #[test]
    fn surface_outside_bounds_is_rejected() {
        let s = Sphere {
            center: Point3::new(10.0, 10.0, 10.0),
            radius: 1.0,
        };
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let opts = SdfToBrepOptions::new(bounds(1.0), 4);
        let err = sdf_to_brep(&s, &mut store, &mut geo, &opts).unwrap_err();
        assert!(matches!(err, CoreError::InvalidArgument { .. }), "{err}");
        assert_eq!(store.bodies.len(), 0, "stores untouched on failure");
    }

    #[test]
    fn surface_crossing_bounds_is_rejected_as_open() {
        // The sphere pokes through the meshing region: boundary-layer
        // crossings are not stitched, so the mesh has holes.
        let s = Sphere {
            center: Point3::origin(),
            radius: 1.0,
        };
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let opts = SdfToBrepOptions::new(bounds(0.8), 4);
        let err = sdf_to_brep(&s, &mut store, &mut geo, &opts).unwrap_err();
        assert!(matches!(err, CoreError::Degenerate { .. }), "{err}");
        assert_eq!(store.bodies.len(), 0, "stores untouched on failure");
    }
}
