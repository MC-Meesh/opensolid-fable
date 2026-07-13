//! B-Rep tessellation MVP (`spec/07-tessellation.md`): convert bodies with
//! analytic face geometry into [`TriangleMesh`]es.
//!
//! Strategy, per face by surface kind:
//!
//! - **Planar faces**: the outer loop is sampled into a polygon (lines as
//!   single segments, circles/ellipses at the angular step) and
//!   ear-clip triangulated (correct for concave outlines).
//! - **Quadric faces** (cylinder, cone, sphere, torus): sampled on a
//!   parameter grid. Periodic directions wrap by index, so seams close
//!   exactly; parameterization singularities (sphere poles, cone apex)
//!   collapse their grid row to a single vertex with the limit normal.
//!   Ruled directions (cylinder/cone `v`) use one segment; angular
//!   directions honor the angular step. The `v` range of an unbounded
//!   surface is recovered by projecting boundary-edge samples onto the
//!   surface.
//!
//! Per-vertex normals come from [`SurfaceEval::normal`], negated for
//! [`FaceSense::Negative`] faces so they point outward from the material;
//! triangle winding follows the same outward direction. Boolean outputs
//! routinely bind tool-derived faces with Negative sense (of-as6).
//!
//! [`tessellate_body`] concatenates the per-face meshes and welds them:
//! adjacent faces sample their shared edges at identical curve parameters,
//! so rim vertices coincide and welding stitches the body watertight.
//! Welded boundary vertices average the adjoining faces' normals.
//!
//! # MVP limitations (later hardening passes)
//!
//! - Planar faces are **ear-clip** triangulated (correct for concave outer
//!   loops), but faces with inner loops (holes) are still rejected with
//!   [`CoreError::NotImplemented`]. Full constrained Delaunay triangulation
//!   (hole bridging, as in [`crate::boolean`]) is a later pass.
//! - Quadric faces are assumed to cover their surface's **full angular
//!   range** (the full `u` period, and the full `v` domain/period for
//!   spheres and tori), as the primitive and sweep constructors produce.
//!   Trimmed quadric faces (from booleans) arrive with the CDT pass.
//! - The only fidelity control is [`TessellationOptions::angular_step`];
//!   chord tolerance, edge-length bounds, and adaptive refinement are
//!   deferred.

use crate::curve::{Curve3, CurveEval, plane_basis};
use crate::geometry::GeometryStore;
use crate::project::SurfaceProject;
use crate::surface::{Surface3, SurfaceEval};
use crate::topology::{Body, Face, FaceSense, Fin, Loop, TopologyStore};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::{EntityId, Point3, Vector3};

/// Fidelity controls for tessellation.
///
/// The MVP exposes a single knob; the spec's full option set (chord
/// tolerance, edge-length bounds) is a later hardening pass.
#[derive(Debug, Clone)]
pub struct TessellationOptions {
    /// Maximum parameter step, in radians, when sampling angular directions
    /// (circular edges, quadric parameter grids). Smaller is finer: the
    /// default `2π/32` gives 32 segments around a full circle.
    pub angular_step: f64,
}

impl Default for TessellationOptions {
    fn default() -> Self {
        Self {
            angular_step: std::f64::consts::TAU / 32.0,
        }
    }
}

impl TessellationOptions {
    fn validate(&self) -> CoreResult<()> {
        if self.angular_step <= 0.0 || !self.angular_step.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "angular_step",
                reason: format!("must be positive and finite, got {}", self.angular_step),
            });
        }
        Ok(())
    }
}

/// Segment count for sweeping an angular range at the configured step.
/// At least 3, so closed circles always produce a real polygon.
fn angular_segments(sweep: f64, options: &TessellationOptions) -> usize {
    ((sweep.abs() / options.angular_step).ceil() as usize).max(3)
}

/// Tessellate every face of `body` into one welded mesh.
///
/// For the closed solids produced by [`crate::primitives`] and
/// [`crate::sweep`], the result is a closed, consistently oriented
/// manifold (see [`TriangleMesh::is_closed_manifold`]).
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `body` is stale, or any reached face
/// or edge lacks attached geometry; [`CoreError::NotImplemented`] for
/// planar faces with holes (see the module docs).
pub fn tessellate_body(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    options: &TessellationOptions,
) -> CoreResult<TriangleMesh> {
    options.validate()?;
    if store.body(body).is_none() {
        return Err(CoreError::InvalidArgument {
            argument: "body",
            reason: format!("stale body id {body:?}"),
        });
    }

    let mut mesh = TriangleMesh::new();
    for face in store.faces_of_body(body) {
        tessellate_face_into(store, geo, face, options, &mut mesh)?;
    }

    // Adjacent faces sample shared edges at identical parameters, so their
    // rim vertices agree to floating-point noise; weld at a tolerance far
    // below any feature size to stitch them.
    let epsilon = mesh
        .bounding_box()
        .map(|b| (b.max - b.min).norm() * 1e-9)
        .unwrap_or(0.0);
    Ok(mesh.weld(epsilon))
}

/// Tessellate a single face (unwelded, open along its boundary unless the
/// face alone closes the surface).
///
/// # Errors
/// As [`tessellate_body`], for this face.
pub fn tessellate_face(
    store: &TopologyStore,
    geo: &GeometryStore,
    face: EntityId<Face>,
    options: &TessellationOptions,
) -> CoreResult<TriangleMesh> {
    options.validate()?;
    let mut mesh = TriangleMesh::new();
    tessellate_face_into(store, geo, face, options, &mut mesh)?;
    Ok(mesh)
}

fn invalid_face(face: EntityId<Face>, what: &str) -> CoreError {
    CoreError::InvalidArgument {
        argument: "body",
        reason: format!("face {face:?} {what}"),
    }
}

fn tessellate_face_into(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    options: &TessellationOptions,
    mesh: &mut TriangleMesh,
) -> CoreResult<()> {
    let face = store
        .face(face_id)
        .ok_or_else(|| invalid_face(face_id, "is stale"))?;
    let surface_id = face
        .surface
        .ok_or_else(|| invalid_face(face_id, "has no attached surface geometry"))?;
    let surface = geo
        .surface(surface_id)
        .ok_or_else(|| invalid_face(face_id, "references a stale surface id"))?;

    // A Negative-sense face's outward normal opposes its surface normal
    // (boolean outputs encode tool-derived faces this way — see
    // `crate::boolean`): flip emitted normals and winding to stay outward.
    let flip = face.sense == FaceSense::Negative;
    match surface {
        Surface3::Plane { .. } => {
            fan_planar_face(store, geo, face_id, face, surface, flip, options, mesh)
        }
        Surface3::Cylinder { .. } | Surface3::Cone { .. } => {
            let (u_anchor, v_lo, v_hi) = boundary_param_range(store, geo, face_id, face, surface)?;
            grid_face(surface, u_anchor, v_lo, v_hi, false, 1, flip, options, mesh);
            Ok(())
        }
        Surface3::Sphere { .. } => {
            let (v_lo, v_hi) = surface.domain_v();
            let n_v = angular_segments(v_hi - v_lo, options);
            grid_face(surface, 0.0, v_lo, v_hi, false, n_v, flip, options, mesh);
            Ok(())
        }
        Surface3::Torus { .. } => {
            let period = surface.period_v().expect("torus is v-periodic");
            let n_v = angular_segments(period, options);
            grid_face(surface, 0.0, 0.0, period, true, n_v, flip, options, mesh);
            Ok(())
        }
    }
}

/// Ear-clip triangulate a planar face's outer loop polygon. Correct for
/// concave outlines, unlike the old first-vertex fan (of-6dw).
#[allow(clippy::too_many_arguments)]
fn fan_planar_face(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    face: &Face,
    surface: &Surface3,
    flip: bool,
    options: &TessellationOptions,
    mesh: &mut TriangleMesh,
) -> CoreResult<()> {
    if !face.inner_loops.is_empty() {
        return Err(CoreError::NotImplemented {
            feature: "tessellating planar faces with holes (needs constrained triangulation)",
        });
    }
    let loop_id = face
        .outer_loop
        .ok_or_else(|| invalid_face(face_id, "has no outer loop"))?;
    let polygon = sample_loop(store, geo, face_id, loop_id, options)?;
    if polygon.len() < 3 {
        return Err(invalid_face(
            face_id,
            "outer loop samples to fewer than 3 points",
        ));
    }

    let surface_normal = surface
        .normal(0.0, 0.0)
        .expect("planes have a normal everywhere");
    // The face's outward normal (of-as6): Negative-sense faces oppose their
    // surface normal, and their loops wind CCW about *outward* — building
    // the basis about outward keeps ear_clip's winding outward-facing.
    let normal = if flip {
        -surface_normal
    } else {
        surface_normal
    };
    let base = mesh.positions.len();
    for point in &polygon {
        mesh.positions.push(*point);
        mesh.normals.push(normal);
    }
    // Ear-clip the loop polygon so concave faces (U/S/C outlines) tile
    // without overlap; a first-vertex fan was silently wrong for any loop
    // not star-shaped from that vertex (of-6dw). Project onto a plane
    // basis with e_u × e_v = normal so ear_clip's counterclockwise
    // triples come out wound about +normal — the outward winding, since
    // the loop runs counterclockwise about the outward normal.
    let (e_u, e_v) = plane_basis(&normal);
    let origin = polygon[0];
    let uv: Vec<(f64, f64)> = polygon
        .iter()
        .map(|p| {
            let d = p - origin;
            (d.dot(&e_u), d.dot(&e_v))
        })
        .collect();
    for [a, b, c] in crate::triangulate::ear_clip(&uv) {
        mesh.indices.push([base + a, base + b, base + c]);
    }
    Ok(())
}

/// Sample a loop's boundary as a closed polygon, in loop order, one open
/// run of points per fin (each fin's end point is supplied by the next).
fn sample_loop(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    loop_id: EntityId<Loop>,
    options: &TessellationOptions,
) -> CoreResult<Vec<Point3>> {
    let mut points = Vec::new();
    for &fin_id in store.fins_of_loop(loop_id) {
        let (curve, t_from, t_to) = fin_curve(store, geo, face_id, fin_id)?;
        let segments = match curve {
            Curve3::Line { .. } => 1,
            Curve3::Circle { .. } | Curve3::Ellipse { .. } => {
                angular_segments(t_to - t_from, options)
            }
        };
        for k in 0..segments {
            let t = t_from + (t_to - t_from) * k as f64 / segments as f64;
            points.push(curve.point(t));
        }
    }
    Ok(points)
}

/// A fin's curve and its parameter sweep in traversal direction.
fn fin_curve<'g>(
    store: &TopologyStore,
    geo: &'g GeometryStore,
    face_id: EntityId<Face>,
    fin_id: EntityId<Fin>,
) -> CoreResult<(&'g Curve3, f64, f64)> {
    let fin = store
        .fin(fin_id)
        .ok_or_else(|| invalid_face(face_id, "loop references a stale fin"))?;
    let edge = store
        .edge(fin.edge)
        .ok_or_else(|| invalid_face(face_id, "fin references a stale edge"))?;
    let curve_id = edge
        .curve
        .ok_or_else(|| invalid_face(face_id, "has an edge with no attached curve geometry"))?;
    let curve = geo
        .curve(curve_id)
        .ok_or_else(|| invalid_face(face_id, "has an edge referencing a stale curve id"))?;
    let (t_from, t_to) = match fin.sense {
        crate::topology::FinSense::Forward => (edge.t_start, edge.t_end),
        crate::topology::FinSense::Reversed => (edge.t_end, edge.t_start),
    };
    Ok((curve, t_from, t_to))
}

/// The `u` anchor and `v` range spanned by a face's boundary, recovered by
/// projecting boundary-edge samples onto the surface (for surfaces with an
/// unbounded `v` domain: cylinders and cones).
///
/// The anchor is the `u` of the first boundary sample — a rim vertex. The
/// boundary circles of a transformed body are re-anchored to start at an
/// arbitrary angle ([`crate::transform`]), so the grid's `u` columns must
/// start at the same angle for its rim vertices to coincide with the
/// adjacent faces' boundary samples and weld watertight.
fn boundary_param_range(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
    face: &Face,
    surface: &Surface3,
) -> CoreResult<(f64, f64, f64)> {
    let mut u_anchor = None;
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for loop_id in face
        .outer_loop
        .into_iter()
        .chain(face.inner_loops.iter().copied())
    {
        for &fin_id in store.fins_of_loop(loop_id) {
            let (curve, t_from, t_to) = fin_curve(store, geo, face_id, fin_id)?;
            for k in 0..=4 {
                let t = t_from + (t_to - t_from) * k as f64 / 4.0;
                let projected = surface.project_point(&curve.point(t));
                if u_anchor.is_none() {
                    u_anchor = Some(projected.u);
                }
                lo = lo.min(projected.v);
                hi = hi.max(projected.v);
            }
        }
    }
    if !(lo.is_finite() && hi.is_finite() && hi > lo) {
        return Err(invalid_face(
            face_id,
            "boundary does not span a v range on its unbounded surface",
        ));
    }
    Ok((u_anchor.expect("v range implies samples"), lo, hi))
}

/// Tessellate a quadric face over its parameter rectangle:
/// `u` over the full period starting at `u_anchor` (wrapped by index), `v`
/// over `[v_lo, v_hi]` with `n_v` segments (wrapped if `wrap_v`). Singular
/// rows (sphere poles, cone apex) collapse to a single vertex. `flip`
/// reverses emitted normals and winding, for Negative-sense faces whose
/// outward direction opposes the surface normal.
#[allow(clippy::too_many_arguments)]
fn grid_face(
    surface: &Surface3,
    u_anchor: f64,
    v_lo: f64,
    v_hi: f64,
    wrap_v: bool,
    n_v: usize,
    flip: bool,
    options: &TessellationOptions,
    mesh: &mut TriangleMesh,
) {
    let period = surface.period_u().expect("quadric surfaces are u-periodic");
    let n_u = angular_segments(period, options);
    let row_count = if wrap_v { n_v } else { n_v + 1 };

    // rows[j] holds one vertex index per u column, or exactly one index for
    // a collapsed singular row.
    let mut rows: Vec<Vec<usize>> = Vec::with_capacity(row_count);
    for j in 0..row_count {
        let v = if !wrap_v && j == n_v {
            v_hi // exact endpoint, no accumulation error
        } else {
            v_lo + (v_hi - v_lo) * j as f64 / n_v as f64
        };
        let singular = surface.is_singular(u_anchor, v);
        let columns = if singular { 1 } else { n_u };
        let mut row = Vec::with_capacity(columns);
        for i in 0..columns {
            let u = u_anchor + period * i as f64 / n_u as f64;
            row.push(mesh.positions.len());
            mesh.positions.push(surface.point(u, v));
            let normal = grid_normal(surface, u, v, v_lo, v_hi);
            mesh.normals.push(if flip { -normal } else { normal });
        }
        rows.push(row);
    }

    let at = |j: usize, i: usize| -> usize {
        let row = &rows[j % row_count];
        row[i % row.len()]
    };
    for j in 0..n_v {
        for i in 0..n_u {
            // Quad corners in (u, v): a --u--> b, then +v to c/d. Winding
            // follows du × dv, the surface normal — reversed when the
            // face's outward direction opposes it.
            let (a, b) = (at(j, i), at(j, i + 1));
            let (d, c) = (at(j + 1, i), at(j + 1, i + 1));
            for [p, q, r] in [[a, b, c], [a, c, d]] {
                let tri = if flip { [p, r, q] } else { [p, q, r] };
                if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                    mesh.indices.push(tri);
                }
            }
        }
    }
}

/// Surface normal for a grid vertex. Where the parameterization is
/// degenerate *and* has no limit normal (cone apex — sphere poles do have
/// one), nudge `v` toward the range interior for a usable shading normal.
fn grid_normal(surface: &Surface3, u: f64, v: f64, v_lo: f64, v_hi: f64) -> Vector3 {
    surface.normal(u, v).unwrap_or_else(|| {
        let mid = (v_lo + v_hi) / 2.0;
        let nudged = v + (mid - v) * 1e-6;
        surface.normal(u, nudged).unwrap_or_else(Vector3::zeros)
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::primitives;
    use std::f64::consts::{PI, TAU};

    fn build(
        make: impl FnOnce(&mut TopologyStore, &mut GeometryStore) -> CoreResult<EntityId<Body>>,
    ) -> TriangleMesh {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = make(&mut store, &mut geo).expect("valid primitive");
        tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds")
    }

    /// Signed volume via the divergence theorem: positive iff triangles
    /// wind outward consistently.
    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    /// Euler characteristic V - E + F of a closed mesh.
    fn euler_characteristic(mesh: &TriangleMesh) -> i64 {
        let mut edges = std::collections::HashSet::new();
        for tri in &mesh.indices {
            for e in 0..3 {
                let (a, b) = (tri[e], tri[(e + 1) % 3]);
                edges.insert((a.min(b), a.max(b)));
            }
        }
        mesh.vertex_count() as i64 - edges.len() as i64 + mesh.triangle_count() as i64
    }

    fn assert_within(actual: f64, expected: f64, fraction: f64, what: &str) {
        assert!(
            (actual - expected).abs() <= expected.abs() * fraction,
            "{what}: {actual} vs expected {expected} (>{:.1}%)",
            fraction * 100.0
        );
    }

    #[test]
    fn block_mesh_is_exact() {
        let mesh = build(|s, g| primitives::block(s, g, 2.0, 3.0, 4.0));
        assert!(mesh.is_closed_manifold());
        assert_eq!(mesh.triangle_count(), 12, "two triangles per face");
        assert_eq!(mesh.vertex_count(), 8, "corners welded across faces");
        assert_eq!(euler_characteristic(&mesh), 2);
        // Flat faces tessellate exactly, not approximately.
        let area = 2.0 * (2.0 * 3.0 + 3.0 * 4.0 + 4.0 * 2.0);
        assert!((mesh.total_area() - area).abs() < 1e-9);
        assert!((signed_volume(&mesh) - 24.0).abs() < 1e-9);
        let bbox = mesh.bounding_box().unwrap();
        assert!((bbox.min - Point3::new(-1.0, -1.5, -2.0)).norm() < 1e-9);
        assert!((bbox.max - Point3::new(1.0, 1.5, 2.0)).norm() < 1e-9);
    }

    #[test]
    fn cylinder_mesh_is_closed_and_accurate() {
        let (r, h) = (1.5, 5.0);
        let mesh = build(|s, g| primitives::cylinder(s, g, r, h));
        assert!(mesh.is_closed_manifold());
        assert_eq!(euler_characteristic(&mesh), 2);
        assert_within(
            mesh.total_area(),
            TAU * r * h + TAU * r * r,
            0.05,
            "cylinder area",
        );
        assert_within(
            signed_volume(&mesh),
            PI * r * r * h,
            0.05,
            "cylinder volume",
        );
    }

    #[test]
    fn sphere_mesh_is_closed_and_accurate() {
        let r = 2.5;
        let mesh = build(|s, g| primitives::sphere(s, g, r));
        assert!(mesh.is_closed_manifold());
        assert_eq!(euler_characteristic(&mesh), 2);
        assert_within(mesh.total_area(), 2.0 * TAU * r * r, 0.05, "sphere area");
        assert_within(
            signed_volume(&mesh),
            2.0 / 3.0 * TAU * r * r * r,
            0.05,
            "sphere volume",
        );
    }

    #[test]
    fn torus_mesh_is_closed_genus_one_and_accurate() {
        let (major, minor) = (3.0, 1.0);
        let mesh = build(|s, g| primitives::torus(s, g, major, minor));
        assert!(mesh.is_closed_manifold());
        assert_eq!(euler_characteristic(&mesh), 0, "torus has genus 1");
        assert_within(
            mesh.total_area(),
            TAU * TAU * major * minor,
            0.05,
            "torus area",
        );
        assert_within(
            signed_volume(&mesh),
            PI * TAU * major * minor * minor,
            0.05,
            "torus volume",
        );
    }

    #[test]
    fn convex_body_normals_point_outward() {
        // All four bodies are centered at the origin; for the convex ones
        // every outward direction has positive dot with its position.
        for mesh in [
            build(|s, g| primitives::block(s, g, 2.0, 3.0, 4.0)),
            build(|s, g| primitives::cylinder(s, g, 1.5, 5.0)),
            build(|s, g| primitives::sphere(s, g, 2.5)),
        ] {
            for (position, normal) in mesh.positions.iter().zip(&mesh.normals) {
                assert!((normal.norm() - 1.0).abs() < 1e-9, "vertex normal not unit");
                assert!(
                    normal.dot(&position.coords) > 0.0,
                    "inward vertex normal at {position:?}"
                );
            }
            for tri in &mesh.indices {
                let [a, b, c] = tri.map(|i| mesh.positions[i]);
                let geometric = (b - a).cross(&(c - a));
                let centroid = (a.coords + b.coords + c.coords) / 3.0;
                assert!(
                    geometric.dot(&centroid) > 0.0,
                    "inward triangle winding at {centroid:?}"
                );
            }
        }
    }

    #[test]
    fn torus_normals_agree_with_surface() {
        // The inner ring's normals point toward the axis, so the convex
        // dot-with-position test does not apply; check against the exact
        // tube normal instead: (p - ring_center)/minor for each vertex.
        let (major, minor) = (3.0, 1.0);
        let mesh = build(|s, g| primitives::torus(s, g, major, minor));
        for (position, normal) in mesh.positions.iter().zip(&mesh.normals) {
            let ring = Vector3::new(position.x, position.y, 0.0).normalize() * major;
            let exact = (position.coords - ring) / minor;
            assert!(
                (normal - exact).norm() < 1e-6,
                "normal {normal:?} vs tube normal {exact:?}"
            );
        }
    }

    #[test]
    fn finer_angular_step_converges() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");
        let exact = 2.0 * TAU;
        let area = |step: f64| {
            tessellate_body(
                &store,
                &geo,
                body,
                &TessellationOptions { angular_step: step },
            )
            .expect("tessellation succeeds")
            .total_area()
        };
        let coarse = (area(TAU / 16.0) - exact).abs();
        let fine = (area(TAU / 64.0) - exact).abs();
        assert!(
            fine < coarse / 4.0,
            "quadratic convergence expected: coarse err {coarse}, fine err {fine}"
        );
    }

    #[test]
    fn concave_planar_face_tiles_without_overlap() {
        use crate::topology::{
            BodyType, FaceSense, FinSense, LoopType, SYSTEM_RESOLUTION, ShellOrientation,
        };
        // A concave U outline in the z=0 plane (of-6dw): the old
        // first-vertex fan spilled across the notch and emitted
        // overlapping, mixed-winding triangles that inflate the area. Ear
        // clipping tiles the polygon exactly.
        let outline = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 3.0, 0.0),
            Point3::new(2.0, 3.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 3.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
        ];
        let n = outline.len();

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);
        let verts: Vec<_> = outline
            .iter()
            .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
            .collect();
        let plane = Surface3::plane(outline[0], Vector3::z()).expect("valid plane");
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface = Some(geo.add_surface(plane));
        let loop_edges: Vec<_> = (0..n)
            .map(|i| {
                let (a, b) = (outline[i], outline[(i + 1) % n]);
                let curve = geo.add_curve(Curve3::line(a, b - a).expect("valid line"));
                let edge = store.create_edge_with_curve(
                    verts[i],
                    verts[(i + 1) % n],
                    SYSTEM_RESOLUTION,
                    curve,
                    0.0,
                    (b - a).norm(),
                );
                (edge, FinSense::Forward)
            })
            .collect();
        store.create_loop(face, LoopType::Outer, &loop_edges);

        let mesh = tessellate_face(&store, &geo, face, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert_eq!(
            mesh.triangle_count(),
            n - 2,
            "n-2 triangles tile the polygon"
        );
        // Exact area of the U outline (shoelace = 7).
        assert!(
            (mesh.total_area() - 7.0).abs() < 1e-9,
            "cap triangles overlap: area {} != 7",
            mesh.total_area()
        );
        // Every triangle winds counterclockwise about +z (outward).
        for tri in &mesh.indices {
            let [a, b, c] = tri.map(|i| mesh.positions[i]);
            let facing = (b - a).cross(&(c - a));
            assert!(facing.z > 0.0, "triangle winds inward: {facing:?}");
        }
    }

    /// of-as6: a subtract's tool-derived faces bind the tool's surfaces
    /// with `FaceSense::Negative` (outward opposes the surface normal).
    /// Ignoring the sense wound those caps inward, so the welded mesh
    /// failed the manifold orientation check on every imprinted edge.
    #[test]
    fn boolean_corner_notch_tessellates_closed() {
        use opensolid_core::Transform3;
        use opensolid_core::tolerance::ToleranceContext;

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let a = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("valid block");
        let b = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("valid block");
        crate::transform::transform_body(
            &mut store,
            &mut geo,
            b,
            &Transform3::translation(1.0, 1.0, 1.0),
        )
        .expect("rigid translation");
        let out = crate::boolean::subtract(&store, &geo, a, b, &ToleranceContext::default())
            .expect("transversal subtract");
        assert!(out.check().is_empty(), "boolean result must be valid");

        let mesh = tessellate_body(
            &out.store,
            &out.geo,
            out.body,
            &TessellationOptions::default(),
        )
        .expect("tessellation succeeds");
        assert!(mesh.is_closed_manifold(), "L-shape mesh must be watertight");
        assert_eq!(euler_characteristic(&mesh), 2);
        // Unit corner removed from the 2×2×2 block: volume 8 - 1, area
        // unchanged at 24 (three notch walls replace the removed corner).
        assert!((signed_volume(&mesh) - 7.0).abs() < 1e-9);
        assert!((mesh.total_area() - 24.0).abs() < 1e-9);
    }

    /// A valid body re-encoded with inward surface normals (negated plane
    /// normal + Negative sense, as an importer may produce — of-alr) must
    /// tessellate identically to its all-Positive twin.
    #[test]
    fn flipped_encoding_block_tessellates_identically() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("valid block");
        for face_id in store.faces_of_body(body) {
            let surface_id = store.face(face_id).unwrap().surface.expect("bound surface");
            let flipped = match geo.surface(surface_id).expect("live surface") {
                Surface3::Plane { origin, normal } => {
                    Surface3::plane(*origin, -*normal).expect("valid plane")
                }
                other => panic!("block faces are planes, got {other:?}"),
            };
            let new_id = geo.add_surface(flipped);
            let face = store.faces.get_mut(face_id).expect("live face");
            face.surface = Some(new_id);
            face.sense = crate::topology::FaceSense::Negative;
        }

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert!(mesh.is_closed_manifold());
        assert!((signed_volume(&mesh) - 24.0).abs() < 1e-9);
        for (position, normal) in mesh.positions.iter().zip(&mesh.normals) {
            assert!(
                normal.dot(&position.coords) > 0.0,
                "inward vertex normal at {position:?}"
            );
        }
    }

    /// The quadric grid honors face sense too: flipping a sphere's faces to
    /// Negative reverses winding and normals wholesale, yielding a still-
    /// manifold mesh that bounds the same region from the other side.
    #[test]
    fn negative_sense_sphere_winds_inward() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");
        let outward = tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds");
        for face_id in store.faces_of_body(body) {
            store.faces.get_mut(face_id).expect("live face").sense =
                crate::topology::FaceSense::Negative;
        }
        let inward = tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert!(inward.is_closed_manifold(), "flip preserves manifoldness");
        assert!(
            (signed_volume(&inward) + signed_volume(&outward)).abs() < 1e-9,
            "Negative sense reverses the enclosed signed volume"
        );
        // On a unit sphere at the origin the exact outward normal is the
        // position itself; Negative sense must emit the negation. (The two
        // meshes' vertex orders differ — weld numbers vertices in triangle
        // order — so compare against the analytic normal, not index-wise.)
        for (position, normal) in inward.positions.iter().zip(&inward.normals) {
            assert!(
                (normal + position.coords).norm() < 1e-9,
                "normal {normal:?} at {position:?} does not point inward"
            );
        }
    }

    #[test]
    fn single_face_mesh_is_open() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::cylinder(&mut store, &mut geo, 1.0, 2.0).expect("valid cylinder");
        // Face order from the builder: bottom cap, top cap, wall.
        let wall = store.faces_of_body(body)[2];
        let mesh = tessellate_face(&store, &geo, wall, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert!(!mesh.is_empty());
        assert!(!mesh.is_closed_manifold(), "a lone wall is an open tube");
    }

    #[test]
    fn rejects_invalid_options_and_stale_body() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::sphere(&mut store, &mut geo, 1.0).expect("valid sphere");

        for bad in [0.0, -0.1, f64::NAN] {
            let err = tessellate_body(
                &store,
                &geo,
                body,
                &TessellationOptions { angular_step: bad },
            )
            .unwrap_err();
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "angular_step",
                        ..
                    }
                ),
                "step {bad}: got {err}"
            );
        }

        let stale = body;
        store.bodies.remove(body);
        let err =
            tessellate_body(&store, &geo, stale, &TessellationOptions::default()).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "body",
                    ..
                }
            ),
            "got {err}"
        );
    }

    #[test]
    fn rejects_faces_without_geometry() {
        // An mvfs-seeded body has a face but no attached surface.
        let mut store = TopologyStore::new();
        let geo = GeometryStore::new();
        let (body, ..) = store.mvfs(Point3::origin());
        let err = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "body",
                    ..
                }
            ),
            "got {err}"
        );
        assert!(err.to_string().contains("surface"), "unhelpful: {err}");
    }
}
