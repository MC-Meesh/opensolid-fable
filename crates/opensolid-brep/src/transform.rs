//! Rigid placement of store-backed bodies.
//!
//! The primitive builders ([`crate::primitives`]) construct every solid
//! centered at the origin; [`translate_body`] moves a finished body to its
//! place in the model. Only translation exists for now — the full transform
//! stack (rotation, general body transforms per `spec/03-topology.md`) is a
//! later issue.
//!
//! Translation mutates the body's geometry **in place**: its vertex points
//! and every [`Curve3`]/[`Surface3`] its topology references. This assumes
//! the body does not share geometry ids with another body — true for
//! everything the current builders produce, which insert fresh geometry per
//! body.

use crate::curve::Curve3;
use crate::geometry::GeometryStore;
use crate::surface::Surface3;
use crate::topology::{Body, TopologyStore};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::{EntityId, Vector3};

/// Translate `body` by `offset`: every vertex point and every curve/surface
/// referenced by its edges and faces.
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `offset` is not finite.
///
/// # Panics
/// Panics if `body` is stale or its topology references dead geometry ids
/// (a corrupt store is a caller bug, matching the navigation methods).
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

    let mut curve_ids: Vec<EntityId<Curve3>> = Vec::new();
    let mut surface_ids: Vec<EntityId<Surface3>> = Vec::new();
    let mut vertex_ids = Vec::new();

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

    for v in vertex_ids {
        store.vertices.get_mut(v).expect("stale Vertex id").point += offset;
    }
    for id in curve_ids {
        match geo.curves.get_mut(id).expect("stale Curve3 id") {
            Curve3::Line { origin, .. } => *origin += offset,
            Curve3::Circle { center, .. } | Curve3::Ellipse { center, .. } => *center += offset,
        }
    }
    for id in surface_ids {
        match geo.surfaces.get_mut(id).expect("stale Surface3 id") {
            Surface3::Plane { origin, .. } | Surface3::Cylinder { origin, .. } => *origin += offset,
            Surface3::Cone { origin, .. } => *origin += offset,
            Surface3::Sphere { center, .. } | Surface3::Torus { center, .. } => *center += offset,
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::CurveEval;
    use crate::primitives;
    use crate::project::SurfaceProject;
    use opensolid_core::Point3;

    #[test]
    fn translated_block_moves_vertices_and_geometry_together() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("valid block");
        let offset = Vector3::new(3.0, -1.0, 0.5);
        translate_body(&mut store, &mut geo, body, offset).expect("finite offset");

        assert!(
            store.check(body).is_empty(),
            "translation must keep validity"
        );

        // Vertices span the shifted extents.
        for (_, v) in store.vertices.iter() {
            assert_eq!((v.point.x - 3.0).abs(), 1.0);
            assert_eq!((v.point.y + 1.0).abs(), 1.0);
            assert_eq!((v.point.z - 0.5).abs(), 1.0);
        }

        // Edge curves still interpolate their (moved) vertices, and edge
        // curves still lie on the (moved) adjacent surfaces.
        for face in store.faces_of_body(body) {
            let surface_id = store.face(face).unwrap().surface.expect("bound surface");
            let surface = geo.surface(surface_id).expect("live surface");
            for edge_id in store.edges_of_face(face) {
                let edge = store.edge(edge_id).unwrap();
                let curve = geo.curve(edge.curve.expect("bound curve")).unwrap();
                let start = store.vertex(edge.start_vertex).unwrap().point;
                assert!((curve.point(edge.t_start) - start).norm() < 1e-9);
                assert!(surface.project_point(&start).distance < 1e-9);
            }
        }
    }

    #[test]
    fn translated_cylinder_moves_caps_and_wall() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::cylinder(&mut store, &mut geo, 1.0, 4.0).expect("valid cylinder");
        translate_body(&mut store, &mut geo, body, Vector3::new(2.0, 2.0, 1.0)).expect("finite");

        assert!(store.check(body).is_empty());
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
    fn rejects_non_finite_offset() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("valid block");
        let err = translate_body(&mut store, &mut geo, body, Vector3::new(f64::NAN, 0.0, 0.0))
            .unwrap_err();
        assert!(
            matches!(err, CoreError::InvalidArgument { .. }),
            "got {err}"
        );
    }
}
