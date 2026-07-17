//! Rigid placement of store-backed bodies.
//!
//! The primitive builders ([`crate::primitives`]) construct every solid
//! centered at the origin; [`transform_body`] applies an arbitrary rigid
//! transform (rotation + translation, [`Transform3`]) to a finished body,
//! with [`translate_body`] and [`rotate_body`] as conveniences. Uniform
//! scale and general affine maps are a later issue.
//!
//! A transform mutates the body's geometry **in place**: its vertex points
//! and every [`Curve3`]/[`Surface3`] its topology references. This assumes
//! the body does not share geometry ids with another body — true for
//! everything the current builders produce, which insert fresh geometry per
//! body.
//!
//! # Circle parameter re-anchoring
//!
//! [`Curve3::Circle`] derives its angular reference direction (t = 0) from
//! its axis via [`plane_basis`], so the parameterization does not rotate
//! covariantly: `plane_basis(R·axis)` is generally not `R·plane_basis(axis)`.
//! Edge parameters (`t_start`/`t_end`) index into that parameterization, so
//! after rotating a circle every edge on it is shifted by the angle between
//! the rotated old reference and the newly derived one, keeping
//! `curve.point(t_start) == start_vertex.point` exact. Lines (arc-length
//! parameterization) and ellipses (explicit stored `major_dir`) transform
//! covariantly and need no shift.

use crate::curve::{Curve3, TWO_PI, plane_basis};
use crate::geometry::GeometryStore;
use crate::surface::Surface3;
use crate::topology::{Body, Edge, TopologyStore};
use nalgebra::{Translation3, Unit, UnitQuaternion};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::{EntityId, Point3, Transform3, Vector3};

/// Apply a rigid `transform` to `body`: every vertex point and every
/// curve/surface referenced by its edges and faces.
///
/// Edges lying on rotated circles have their `t_start`/`t_end` re-anchored
/// to the circle's new derived parameterization (module docs), so
/// vertex/curve consistency is preserved exactly.
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `transform` has a non-finite
/// translation or rotation component.
///
/// # Panics
/// Panics if `body` is stale or its topology references dead geometry ids
/// (a corrupt store is a caller bug, matching the navigation methods).
pub fn transform_body(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    body: EntityId<Body>,
    transform: &Transform3,
) -> CoreResult<()> {
    let finite = transform.translation.vector.iter().all(|c| c.is_finite())
        && transform
            .rotation
            .quaternion()
            .coords
            .iter()
            .all(|c| c.is_finite());
    if !finite {
        return Err(CoreError::InvalidArgument {
            argument: "transform",
            reason: format!("must be finite, got {transform}"),
        });
    }
    let rotation = transform.rotation;

    let (vertex_ids, edge_ids, curve_ids, surface_ids) = body_geometry_ids(store, body);

    for v in vertex_ids {
        let vertex = store.vertices.get_mut(v).expect("stale Vertex id");
        vertex.point = transform * vertex.point;
    }

    // Angular parameter shift per rotated circle, applied to its edges below.
    let mut circle_shifts: Vec<(EntityId<Curve3>, f64)> = Vec::new();
    for id in curve_ids {
        match geo.curves.get_mut(id).expect("stale Curve3 id") {
            Curve3::Line { origin, dir } => {
                *origin = transform * *origin;
                *dir = rotation * *dir;
            }
            Curve3::Circle { center, axis, .. } => {
                let old_axis = *axis;
                *center = transform * *center;
                *axis = rotation * old_axis;
                let shift = circle_param_shift(&rotation, &old_axis, axis);
                if shift != 0.0 {
                    circle_shifts.push((id, shift));
                }
            }
            Curve3::Ellipse {
                center,
                axis,
                major_dir,
                ..
            } => {
                *center = transform * *center;
                *axis = rotation * *axis;
                *major_dir = rotation * *major_dir;
            }
            Curve3::Polyline { points, .. } => {
                for p in points.iter_mut() {
                    *p = transform * *p;
                }
            }
        }
    }

    if !circle_shifts.is_empty() {
        for edge_id in edge_ids {
            let edge = store.edges.get_mut(edge_id).expect("stale Edge id");
            let Some(curve) = edge.curve else { continue };
            if let Some(&(_, shift)) = circle_shifts.iter().find(|(id, _)| *id == curve) {
                shift_edge_params(edge, shift);
            }
        }
    }

    for id in surface_ids {
        match geo.surfaces.get_mut(id).expect("stale Surface3 id") {
            Surface3::Plane { origin, normal } => {
                *origin = transform * *origin;
                *normal = rotation * *normal;
            }
            Surface3::Cylinder { origin, axis, .. } | Surface3::Cone { origin, axis, .. } => {
                *origin = transform * *origin;
                *axis = rotation * *axis;
            }
            Surface3::Sphere { center, axis, .. } | Surface3::Torus { center, axis, .. } => {
                *center = transform * *center;
                *axis = rotation * *axis;
            }
            // A rigid transform is affine, so moving the control points
            // moves the patch exactly; knots and weights are unaffected,
            // leaving the parameterization (and every uv in the topology)
            // intact.
            Surface3::Nurbs(nurbs) => nurbs.map_control_points(|p| transform * p),
        }
    }
    Ok(())
}

/// Translate `body` by `offset`. Convenience for [`transform_body`] with a
/// pure translation.
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `offset` is not finite.
///
/// # Panics
/// As [`transform_body`].
pub fn translate_body(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    body: EntityId<Body>,
    offset: Vector3,
) -> CoreResult<()> {
    if !offset.iter().all(|c| c.is_finite()) {
        return Err(CoreError::InvalidArgument {
            argument: "offset",
            reason: format!("must be finite, got {offset}"),
        });
    }
    transform_body(
        store,
        geo,
        body,
        &Transform3::translation(offset.x, offset.y, offset.z),
    )
}

/// Rotate `body` by `angle` radians (right-hand rule) about the axis
/// through `point` with direction `axis` (normalized here). Convenience for
/// [`transform_body`] with a pure rotation about an arbitrary line.
///
/// # Errors
/// [`CoreError::Degenerate`] if `axis` has zero or non-finite length;
/// [`CoreError::InvalidArgument`] if `angle` or `point` is not finite.
///
/// # Panics
/// As [`transform_body`].
pub fn rotate_body(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    body: EntityId<Body>,
    point: Point3,
    axis: Vector3,
    angle: f64,
) -> CoreResult<()> {
    let norm = axis.norm();
    if norm == 0.0 || !norm.is_finite() {
        return Err(CoreError::Degenerate {
            context: "rotate_body",
            reason: format!("axis must have non-zero finite length, got {axis}"),
        });
    }
    if !angle.is_finite() {
        return Err(CoreError::InvalidArgument {
            argument: "angle",
            reason: format!("must be finite, got {angle}"),
        });
    }
    if !point.iter().all(|c| c.is_finite()) {
        return Err(CoreError::InvalidArgument {
            argument: "point",
            reason: format!("must be finite, got {point}"),
        });
    }
    let rotation = UnitQuaternion::from_axis_angle(&Unit::new_unchecked(axis / norm), angle);
    // T(p) = R·(p - point) + point.
    let translation = Translation3::from(point.coords - rotation * point.coords);
    transform_body(
        store,
        geo,
        body,
        &Transform3::from_parts(translation, rotation),
    )
}

/// Unique vertex, edge, curve, and surface ids reachable from `body`.
///
/// # Panics
/// Panics on stale topology ids (corrupt store, caller bug).
#[allow(clippy::type_complexity)]
fn body_geometry_ids(
    store: &TopologyStore,
    body: EntityId<Body>,
) -> (
    Vec<EntityId<crate::topology::Vertex>>,
    Vec<EntityId<Edge>>,
    Vec<EntityId<Curve3>>,
    Vec<EntityId<Surface3>>,
) {
    let mut vertex_ids = Vec::new();
    let mut edge_ids = Vec::new();
    let mut curve_ids: Vec<EntityId<Curve3>> = Vec::new();
    let mut surface_ids: Vec<EntityId<Surface3>> = Vec::new();

    for face in store.faces_of_body(body) {
        if let Some(surface) = store.face(face).expect("stale Face id").surface {
            if !surface_ids.contains(&surface) {
                surface_ids.push(surface);
            }
        }
        for loop_id in store.loops_of_face(face) {
            // Degenerate vertex loops carry a vertex but no fins.
            if let Some(v) = store.loop_(loop_id).expect("stale Loop id").vertex {
                if !vertex_ids.contains(&v) {
                    vertex_ids.push(v);
                }
            }
        }
        for edge_id in store.edges_of_face(face) {
            let edge = store.edge(edge_id).expect("stale Edge id");
            if !edge_ids.contains(&edge_id) {
                edge_ids.push(edge_id);
            }
            if let Some(curve) = edge.curve {
                if !curve_ids.contains(&curve) {
                    curve_ids.push(curve);
                }
            }
            for v in [edge.start_vertex, edge.end_vertex] {
                if !vertex_ids.contains(&v) {
                    vertex_ids.push(v);
                }
            }
        }
    }
    (vertex_ids, edge_ids, curve_ids, surface_ids)
}

/// Angular offset from a rotated circle's old reference direction to its
/// newly derived one: rotating by `rotation` maps the point at old
/// parameter `t` to the point at new parameter `t + shift`.
fn circle_param_shift(
    rotation: &UnitQuaternion<f64>,
    old_axis: &Vector3,
    new_axis: &Vector3,
) -> f64 {
    let (u_old, _) = plane_basis(old_axis);
    let (u_new, v_new) = plane_basis(new_axis);
    let rotated_ref = rotation * u_old;
    rotated_ref.dot(&v_new).atan2(rotated_ref.dot(&u_new))
}

/// Shift an edge's parameter range by `shift`, renormalizing `t_start` into
/// the circle's canonical `[0, 2π)` domain while preserving the span.
fn shift_edge_params(edge: &mut Edge, shift: f64) {
    let span = edge.t_end - edge.t_start;
    let start = (edge.t_start + shift).rem_euclid(TWO_PI);
    edge.t_start = start;
    edge.t_end = start + span;
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::boolean;
    use crate::curve::CurveEval;
    use crate::primitives;
    use crate::project::SurfaceProject;
    use crate::tessellate::{TessellationOptions, tessellate_body};
    use opensolid_core::ToleranceContext;
    use opensolid_core::mesh::TriangleMesh;

    fn stores() -> (TopologyStore, GeometryStore) {
        (TopologyStore::new(), GeometryStore::new())
    }

    /// Signed volume via the divergence theorem: positive iff triangles
    /// wind outward consistently. Origin-independent for closed meshes.
    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    /// Tessellate `body`, asserting the mesh is closed and manifold, and
    /// return its volume.
    fn closed_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> f64 {
        let mesh = tessellate_body(store, geo, body, &TessellationOptions::default())
            .expect("tessellation succeeds");
        assert!(
            mesh.is_closed_manifold(),
            "transformed body must tessellate watertight"
        );
        signed_volume(&mesh)
    }

    /// Assert the body passes the checker and its geometry is numerically
    /// consistent: every edge's curve interpolates its endpoint vertices at
    /// `t_start`/`t_end`, and edge samples lie on adjacent face surfaces.
    /// This is the invariant circle re-anchoring exists to preserve.
    fn assert_consistent(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) {
        assert!(
            store.check(body).is_empty(),
            "transform must keep validity: {:?}",
            store.check(body)
        );
        for face in store.faces_of_body(body) {
            let surface_id = store.face(face).unwrap().surface.expect("bound surface");
            let surface = geo.surface(surface_id).expect("live surface");
            for edge_id in store.edges_of_face(face) {
                let edge = store.edge(edge_id).unwrap();
                let curve = geo.curve(edge.curve.expect("bound curve")).unwrap();
                let start = store.vertex(edge.start_vertex).unwrap().point;
                let end = store.vertex(edge.end_vertex).unwrap().point;
                assert!(
                    (curve.point(edge.t_start) - start).norm() < 1e-9,
                    "curve.point(t_start) drifted from start vertex"
                );
                assert!(
                    (curve.point(edge.t_end) - end).norm() < 1e-9,
                    "curve.point(t_end) drifted from end vertex"
                );
                for k in 0..=4 {
                    let t = edge.t_start + (edge.t_end - edge.t_start) * k as f64 / 4.0;
                    assert!(
                        surface.project_point(&curve.point(t)).distance < 1e-9,
                        "edge sample left its adjacent surface"
                    );
                }
            }
        }
    }

    #[test]
    fn translated_block_moves_vertices_and_geometry_together() {
        let (mut store, mut geo) = stores();
        let body = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("valid block");
        let offset = Vector3::new(3.0, -1.0, 0.5);
        translate_body(&mut store, &mut geo, body, offset).expect("finite offset");

        assert_consistent(&store, &geo, body);
        for (_, v) in store.vertices.iter() {
            assert_eq!((v.point.x - 3.0).abs(), 1.0);
            assert_eq!((v.point.y + 1.0).abs(), 1.0);
            assert_eq!((v.point.z - 0.5).abs(), 1.0);
        }
    }

    #[test]
    fn translated_cylinder_moves_caps_and_wall() {
        let (mut store, mut geo) = stores();
        let body = primitives::cylinder(&mut store, &mut geo, 1.0, 4.0).expect("valid cylinder");
        translate_body(&mut store, &mut geo, body, Vector3::new(2.0, 2.0, 1.0)).expect("finite");

        assert_consistent(&store, &geo, body);
        let mut zs: Vec<f64> = store.vertices.iter().map(|(_, v)| v.point.z).collect();
        zs.sort_by(f64::total_cmp);
        assert_eq!(zs, vec![-1.0, 3.0], "seam vertices at shifted cap heights");

        // The wall surface moved with the body.
        let wall = store
            .faces_of_body(body)
            .iter()
            .find_map(|&f| {
                let id = store.face(f).unwrap().surface.unwrap();
                match geo.surface(id).unwrap() {
                    Surface3::Cylinder { origin, .. } => Some(*origin),
                    _ => None,
                }
            })
            .expect("cylinder has a wall face");
        assert!((wall - Point3::new(2.0, 2.0, -1.0)).norm() < 1e-12);
    }

    #[test]
    fn rotated_cylinder_round_trips_through_tessellation() {
        // The of-ipt.5 repro shape: a cylinder tilted off-axis. Rotation
        // re-anchors the cap circles' parameters, and tessellation anchors
        // the wall grid to the re-anchored boundary — both are needed for
        // the mesh to weld watertight.
        let (mut store, mut geo) = stores();
        let body = primitives::cylinder(&mut store, &mut geo, 1.0, 4.0).expect("valid cylinder");
        let aligned_volume = closed_volume(&store, &geo, body);

        rotate_body(
            &mut store,
            &mut geo,
            body,
            Point3::new(0.5, -0.3, 0.2),
            Vector3::new(1.0, 1.0, 0.0),
            0.7,
        )
        .expect("valid rotation");

        assert_consistent(&store, &geo, body);
        let tilted_volume = closed_volume(&store, &geo, body);
        // Rigid motion maps the sample lattice congruently, so the two
        // tessellations enclose the same volume to floating-point noise.
        assert!(
            (tilted_volume - aligned_volume).abs() < 1e-6 * aligned_volume,
            "volume changed under rotation: {tilted_volume} vs {aligned_volume}"
        );
    }

    #[test]
    fn rotated_sphere_and_torus_tessellate_closed() {
        for make in [
            (|s: &mut TopologyStore, g: &mut GeometryStore| primitives::sphere(s, g, 1.5))
                as fn(&mut TopologyStore, &mut GeometryStore) -> CoreResult<EntityId<Body>>,
            |s, g| primitives::torus(s, g, 3.0, 1.0),
        ] {
            let (mut store, mut geo) = stores();
            let body = make(&mut store, &mut geo).expect("valid primitive");
            let before = closed_volume(&store, &geo, body);
            rotate_body(
                &mut store,
                &mut geo,
                body,
                Point3::new(1.0, 2.0, -0.5),
                Vector3::new(1.0, -2.0, 3.0),
                1.1,
            )
            .expect("valid rotation");
            assert_consistent(&store, &geo, body);
            let after = closed_volume(&store, &geo, body);
            assert!(
                (after - before).abs() < 1e-6 * before,
                "volume changed under rotation: {after} vs {before}"
            );
        }
    }

    #[test]
    fn rotated_block_subtract_matches_axis_aligned() {
        let tol = ToleranceContext::default();
        // Axis-aligned reference: [-2,2]^3 minus a 2-cube protruding through
        // the +x face (overlap [0.5,2]×[-1,1]² = 6): 64 - 6 = 58.
        let subtract_volume = |rotate: bool| -> f64 {
            let (mut store, mut geo) = stores();
            let a = primitives::block(&mut store, &mut geo, 4.0, 4.0, 4.0).expect("valid block");
            let b = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("valid block");
            translate_body(&mut store, &mut geo, b, Vector3::new(1.5, 0.0, 0.0)).expect("finite");
            if rotate {
                for body in [a, b] {
                    rotate_body(
                        &mut store,
                        &mut geo,
                        body,
                        Point3::new(0.3, -0.2, 0.5),
                        Vector3::new(1.0, 2.0, 3.0),
                        0.6,
                    )
                    .expect("valid rotation");
                }
            }
            let out = boolean::subtract(&store, &geo, a, b, &tol).expect("subtract succeeds");
            assert!(out.check().is_empty(), "boolean result must be valid");
            let mesh = out.tessellate().expect("tessellation succeeds");
            assert!(mesh.is_closed_manifold());
            signed_volume(&mesh)
        };

        let aligned = subtract_volume(false);
        assert!(
            (aligned - 58.0).abs() < 1e-9,
            "axis-aligned reference: {aligned}"
        );
        let rotated = subtract_volume(true);
        // Both inputs rotated rigidly by the same transform, so the result
        // is the rotated image of the axis-aligned result: same volume.
        assert!(
            (rotated - 58.0).abs() < 1e-9,
            "rotated subtract volume: {rotated}"
        );
    }

    #[test]
    fn rotation_composition_matches_combined_rotation() {
        let p1 = Point3::new(1.0, 0.0, -2.0);
        let axis1 = Vector3::new(0.0, 0.0, 1.0);
        let angle1 = std::f64::consts::FRAC_PI_3;
        let p2 = Point3::new(-0.5, 2.0, 0.0);
        let axis2 = Vector3::new(1.0, 1.0, 0.0);
        let angle2 = 0.4;

        // Body 1: two successive rotations.
        let (mut store1, mut geo1) = stores();
        let body1 = primitives::cylinder(&mut store1, &mut geo1, 1.0, 3.0).expect("valid");
        rotate_body(&mut store1, &mut geo1, body1, p1, axis1, angle1).expect("valid");
        rotate_body(&mut store1, &mut geo1, body1, p2, axis2, angle2).expect("valid");

        // Body 2: the single combined isometry T2 ∘ T1.
        let (mut store2, mut geo2) = stores();
        let body2 = primitives::cylinder(&mut store2, &mut geo2, 1.0, 3.0).expect("valid");
        let iso = |p: Point3, axis: Vector3, angle: f64| {
            let rotation = UnitQuaternion::from_axis_angle(&Unit::new_normalize(axis), angle);
            Transform3::from_parts(Translation3::from(p.coords - rotation * p.coords), rotation)
        };
        let combined = iso(p2, axis2, angle2) * iso(p1, axis1, angle1);
        transform_body(&mut store2, &mut geo2, body2, &combined).expect("valid");

        assert_consistent(&store1, &geo1, body1);
        assert_consistent(&store2, &geo2, body2);
        // Identical construction order means identical vertex ids; every
        // vertex must land in the same place along both paths.
        for ((_, v1), (_, v2)) in store1.vertices.iter().zip(store2.vertices.iter()) {
            assert!(
                (v1.point - v2.point).norm() < 1e-12,
                "composition diverged: {:?} vs {:?}",
                v1.point,
                v2.point
            );
        }
    }

    #[test]
    fn rotate_about_point_fixes_that_point() {
        let (mut store, mut geo) = stores();
        let body = primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("valid block");
        let corner = Point3::new(1.0, 1.5, 2.0);
        let fixed_id = store
            .vertices
            .iter()
            .find(|(_, v)| (v.point - corner).norm() < 1e-12)
            .map(|(id, _)| id)
            .expect("block has a corner vertex at (1, 1.5, 2)");

        rotate_body(
            &mut store,
            &mut geo,
            body,
            corner,
            Vector3::new(2.0, -1.0, 0.5),
            1.9,
        )
        .expect("valid rotation");

        assert_consistent(&store, &geo, body);
        let fixed = store.vertex(fixed_id).unwrap().point;
        assert!(
            (fixed - corner).norm() < 1e-12,
            "rotation center moved to {fixed:?}"
        );
        // And the body genuinely rotated: some other vertex moved.
        assert!(
            store
                .vertices
                .iter()
                .any(|(_, v)| (v.point - corner).norm() > 1.0),
            "no vertex moved"
        );
    }

    #[test]
    fn full_turn_is_identity() {
        let (mut store, mut geo) = stores();
        let body = primitives::cylinder(&mut store, &mut geo, 1.0, 2.0).expect("valid cylinder");
        let before: Vec<Point3> = store.vertices.iter().map(|(_, v)| v.point).collect();
        rotate_body(
            &mut store,
            &mut geo,
            body,
            Point3::new(0.7, -1.2, 3.0),
            Vector3::new(1.0, 2.0, -1.0),
            TWO_PI,
        )
        .expect("valid rotation");
        assert_consistent(&store, &geo, body);
        for ((_, v), original) in store.vertices.iter().zip(&before) {
            assert!(
                (v.point - original).norm() < 1e-12,
                "full turn moved a vertex"
            );
        }
    }

    #[test]
    fn rejects_non_finite_and_degenerate_inputs() {
        let (mut store, mut geo) = stores();
        let body = primitives::block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("valid block");

        let err = translate_body(&mut store, &mut geo, body, Vector3::new(f64::NAN, 0.0, 0.0))
            .unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "offset",
                    ..
                }
            ),
            "got {err}"
        );

        let err = rotate_body(
            &mut store,
            &mut geo,
            body,
            Point3::origin(),
            Vector3::zeros(),
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, CoreError::Degenerate { .. }), "got {err}");

        let err = rotate_body(
            &mut store,
            &mut geo,
            body,
            Point3::origin(),
            Vector3::x(),
            f64::INFINITY,
        )
        .unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "angle",
                    ..
                }
            ),
            "got {err}"
        );

        let err = rotate_body(
            &mut store,
            &mut geo,
            body,
            Point3::new(f64::NAN, 0.0, 0.0),
            Vector3::x(),
            1.0,
        )
        .unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "point",
                    ..
                }
            ),
            "got {err}"
        );

        let mut bad = Transform3::identity();
        bad.translation.vector.x = f64::NAN;
        let err = transform_body(&mut store, &mut geo, body, &bad).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "transform",
                    ..
                }
            ),
            "got {err}"
        );

        // Failed validation must not have mutated the body.
        assert_consistent(&store, &geo, body);
        for (_, v) in store.vertices.iter() {
            assert!(v.point.iter().all(|c| c.abs() == 0.5), "body was mutated");
        }
    }
}
