//! B-Rep primitive solids (`spec/03-topology.md` §4): block, cylinder,
//! sphere, torus.
//!
//! Every builder produces a closed, manifold, consistently-oriented solid
//! that passes [`TopologyStore::check`] and carries full geometry: a
//! [`Surface3`] on every face and a [`Curve3`] (with exact parameter range)
//! on every edge, both stored in a [`GeometryStore`].
//!
//! Conventions (matching the center-based F-Rep primitives):
//!
//! - All primitives are centered at the origin with `+Z` as the main axis.
//! - Face senses are [`FaceSense::Positive`]: every surface is constructed
//!   so its own normal points out of the material.
//! - Outer loops run counterclockwise about the outward face normal.
//! - Periodic faces are closed with **seam edges** rather than left as
//!   parameter-space annuli: the cylinder wall has an axial seam line, the
//!   sphere a meridian seam through both poles, and the torus one seam
//!   circle per parameter direction. Each seam edge appears twice in its
//!   face's loop, once per direction, so every edge is two-finned (manifold)
//!   and mate senses oppose.
//!
//! Resulting topologies (V, E, F, loops, shell genus):
//!
//! | Primitive | V | E | F | L | genus | Euler `V-E+F-R = 2(S-H)` |
//! |-----------|---|---|---|---|-------|--------------------------|
//! | block     | 8 | 12| 6 | 6 | 0     | 8-12+6 = 2               |
//! | cylinder  | 2 | 3 | 3 | 3 | 0     | 2-3+3 = 2                |
//! | sphere    | 2 | 1 | 1 | 1 | 0     | 2-1+1 = 2                |
//! | torus     | 1 | 2 | 1 | 1 | 1     | 1-2+1 = 0                |
//!
//! The sphere's poles need no edges: the seam meridian ends in a vertex at
//! each pole, and the surface parameterization's polar singularities lie in
//! the face interior (handled by [`SurfaceEval::is_singular`], see
//! [`crate::surface`]).
//!
//! Builders validate all arguments and construct all geometry *before*
//! touching either store, so a failed call leaves both stores exactly as
//! they were.
//!
//! [`SurfaceEval::is_singular`]: crate::surface::SurfaceEval::is_singular

use crate::curve::{Curve3, TWO_PI};
use crate::geometry::GeometryStore;
use crate::surface::Surface3;
use crate::topology::{
    Body, BodyType, Edge, FaceSense, FinSense, LoopType, SYSTEM_RESOLUTION, ShellOrientation,
    TopologyStore, Vertex,
};
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::{EntityId, Point3, Vector3};
use std::f64::consts::FRAC_PI_2;

/// Reject a size-like argument that is not positive and finite.
fn positive_dim(name: &'static str, value: f64) -> CoreResult<f64> {
    if value <= 0.0 || !value.is_finite() {
        return Err(CoreError::InvalidArgument {
            argument: name,
            reason: format!("must be positive and finite, got {value}"),
        });
    }
    Ok(value)
}

/// Create an edge with attached curve geometry and exact parameter range.
///
/// Unlike [`TopologyStore::create_edge`], handles closed edges
/// (`start == end`): the edge registers on the shared vertex once, not
/// twice.
fn make_edge(
    store: &mut TopologyStore,
    start: EntityId<Vertex>,
    end: EntityId<Vertex>,
    curve: EntityId<Curve3>,
    t_start: f64,
    t_end: f64,
) -> EntityId<Edge> {
    let edge = store.edges.insert(Edge {
        curve: Some(curve),
        start_vertex: start,
        end_vertex: end,
        t_start,
        t_end,
        tolerance: SYSTEM_RESOLUTION,
        fins: Vec::new(),
    });
    store
        .vertices
        .get_mut(start)
        .expect("make_edge: stale start Vertex id")
        .edges
        .push(edge);
    if end != start {
        store
            .vertices
            .get_mut(end)
            .expect("make_edge: stale end Vertex id")
            .edges
            .push(edge);
    }
    edge
}

/// Axis-aligned rectangular block centered at the origin, with extents
/// `x_size` × `y_size` × `z_size`. Six planar faces, twelve line edges.
///
/// # Errors
/// [`CoreError::InvalidArgument`] if any size is not positive and finite.
pub fn block(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    x_size: f64,
    y_size: f64,
    z_size: f64,
) -> CoreResult<EntityId<Body>> {
    let hx = positive_dim("x_size", x_size)? / 2.0;
    let hy = positive_dim("y_size", y_size)? / 2.0;
    let hz = positive_dim("z_size", z_size)? / 2.0;

    let corners: [Point3; 8] = [
        Point3::new(-hx, -hy, -hz),
        Point3::new(hx, -hy, -hz),
        Point3::new(hx, hy, -hz),
        Point3::new(-hx, hy, -hz),
        Point3::new(-hx, -hy, hz),
        Point3::new(hx, -hy, hz),
        Point3::new(hx, hy, hz),
        Point3::new(-hx, hy, hz),
    ];

    /// Undirected edges as (low, high) corner-index pairs: bottom ring, top
    /// ring, verticals.
    const EDGE_PAIRS: [(usize, usize); 12] = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];

    // Vertex cycles counterclockwise viewed from outside, with the outward
    // normal each cycle implies (right-hand rule).
    let face_specs: [([usize; 4], Vector3); 6] = [
        ([0, 3, 2, 1], -Vector3::z()), // bottom
        ([4, 5, 6, 7], Vector3::z()),  // top
        ([0, 1, 5, 4], -Vector3::y()), // front
        ([1, 2, 6, 5], Vector3::x()),  // right
        ([2, 3, 7, 6], Vector3::y()),  // back
        ([3, 0, 4, 7], -Vector3::x()), // left
    ];

    // Validate-then-mutate: all geometry constructed before any insertion.
    let mut lines = Vec::with_capacity(EDGE_PAIRS.len());
    for &(a, b) in &EDGE_PAIRS {
        lines.push(Curve3::line(corners[a], corners[b] - corners[a])?);
    }
    let mut planes = Vec::with_capacity(face_specs.len());
    for &(cycle, normal) in &face_specs {
        planes.push(Surface3::plane(corners[cycle[0]], normal)?);
    }

    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);
    let vertices = corners.map(|p| store.create_vertex(p, SYSTEM_RESOLUTION));

    let edges: Vec<EntityId<Edge>> = EDGE_PAIRS
        .iter()
        .zip(lines)
        .map(|(&(a, b), line)| {
            let length = (corners[b] - corners[a]).norm();
            let curve = geo.add_curve(line);
            make_edge(store, vertices[a], vertices[b], curve, 0.0, length)
        })
        .collect();

    let directed_edge = |from: usize, to: usize| -> (EntityId<Edge>, FinSense) {
        let (index, &(a, _)) = EDGE_PAIRS
            .iter()
            .enumerate()
            .find(|&(_, &(a, b))| (a, b) == (from, to) || (a, b) == (to, from))
            .expect("face cycles only use listed edges");
        let sense = if a == from {
            FinSense::Forward
        } else {
            FinSense::Reversed
        };
        (edges[index], sense)
    };

    for ((cycle, _), plane) in face_specs.into_iter().zip(planes) {
        let surface = geo.add_surface(plane);
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface = Some(surface);
        let loop_edges: Vec<_> = (0..4)
            .map(|i| directed_edge(cycle[i], cycle[(i + 1) % 4]))
            .collect();
        store.create_loop(face, LoopType::Outer, &loop_edges);
    }

    Ok(body)
}

/// Circular cylinder of `radius` about the `+Z` axis, centered at the
/// origin (`z ∈ [-height/2, height/2]`). Two planar caps plus a periodic
/// wall closed by an axial seam edge at `u = 0` (the `+X` direction).
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `radius` or `height` is not positive
/// and finite.
pub fn cylinder(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    radius: f64,
    height: f64,
) -> CoreResult<EntityId<Body>> {
    let r = positive_dim("radius", radius)?;
    let h = positive_dim("height", height)?;
    let hz = h / 2.0;
    let bottom_center = Point3::new(0.0, 0.0, -hz);
    let top_center = Point3::new(0.0, 0.0, hz);
    let axis = Vector3::z();

    // Validate-then-mutate.
    let bottom_circle = Curve3::circle(bottom_center, axis, r)?;
    let top_circle = Curve3::circle(top_center, axis, r)?;
    let seam_line = Curve3::line(Point3::new(r, 0.0, -hz), axis)?;
    let bottom_plane = Surface3::plane(bottom_center, -axis)?;
    let top_plane = Surface3::plane(top_center, axis)?;
    let wall_surface = Surface3::cylinder(bottom_center, axis, r)?;

    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);

    // Both circle curves start (t = 0) at the +X radial direction, where
    // the seam meets them.
    let v_bottom = store.create_vertex(Point3::new(r, 0.0, -hz), SYSTEM_RESOLUTION);
    let v_top = store.create_vertex(Point3::new(r, 0.0, hz), SYSTEM_RESOLUTION);

    let e_bottom = {
        let curve = geo.add_curve(bottom_circle);
        make_edge(store, v_bottom, v_bottom, curve, 0.0, TWO_PI)
    };
    let e_top = {
        let curve = geo.add_curve(top_circle);
        make_edge(store, v_top, v_top, curve, 0.0, TWO_PI)
    };
    let e_seam = {
        let curve = geo.add_curve(seam_line);
        make_edge(store, v_bottom, v_top, curve, 0.0, h)
    };

    // Bottom cap looks along -Z: counterclockwise about -Z is clockwise
    // about +Z, i.e. against the circle's natural direction.
    let f_bottom = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(f_bottom).expect("just created").surface =
        Some(geo.add_surface(bottom_plane));
    store.create_loop(f_bottom, LoopType::Outer, &[(e_bottom, FinSense::Reversed)]);

    let f_top = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(f_top).expect("just created").surface = Some(geo.add_surface(top_plane));
    store.create_loop(f_top, LoopType::Outer, &[(e_top, FinSense::Forward)]);

    // Wall boundary (outward normal radial): along the bottom circle, up
    // the seam, back along the top circle, down the seam.
    let f_wall = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(f_wall).expect("just created").surface =
        Some(geo.add_surface(wall_surface));
    store.create_loop(
        f_wall,
        LoopType::Outer,
        &[
            (e_bottom, FinSense::Forward),
            (e_seam, FinSense::Forward),
            (e_top, FinSense::Reversed),
            (e_seam, FinSense::Reversed),
        ],
    );

    Ok(body)
}

/// Sphere of `radius` centered at the origin, north pole along `+Z`.
/// A single face closed by a seam meridian edge from the south pole to the
/// north pole through `+X` (the `u = 0` meridian).
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
pub fn sphere(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    radius: f64,
) -> CoreResult<EntityId<Body>> {
    let r = positive_dim("radius", radius)?;
    let center = Point3::origin();

    // Meridian circle in the XZ plane: axis -Y gives the deterministic
    // basis (X, Z), so point(t) = center + r(cos t · X + sin t · Z) and the
    // curve parameter equals the sphere's latitude v.
    let meridian = Curve3::circle(center, -Vector3::y(), r)?;
    let surface = Surface3::sphere(center, Vector3::z(), r)?;

    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);

    let v_south = store.create_vertex(Point3::new(0.0, 0.0, -r), SYSTEM_RESOLUTION);
    let v_north = store.create_vertex(Point3::new(0.0, 0.0, r), SYSTEM_RESOLUTION);

    let e_seam = {
        let curve = geo.add_curve(meridian);
        make_edge(store, v_south, v_north, curve, -FRAC_PI_2, FRAC_PI_2)
    };

    let face = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(face).expect("just created").surface = Some(geo.add_surface(surface));
    store.create_loop(
        face,
        LoopType::Outer,
        &[(e_seam, FinSense::Forward), (e_seam, FinSense::Reversed)],
    );

    Ok(body)
}

/// Torus about the `+Z` axis centered at the origin: `major_radius` from
/// the axis to the tube center, `minor_radius` of the tube. A single face
/// closed by two seam circles meeting at one vertex on the outer equator
/// (`+X` direction): the major seam at `v = 0` and the minor (tube) seam at
/// `u = 0`. The face's loop traverses them as the fundamental polygon
/// `a b a⁻¹ b⁻¹`, and the shell records genus 1.
///
/// # Errors
/// [`CoreError::InvalidArgument`] if either radius is not positive and
/// finite, or `major_radius <= minor_radius` (spindle/horn tori are not
/// supported).
pub fn torus(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    major_radius: f64,
    minor_radius: f64,
) -> CoreResult<EntityId<Body>> {
    let center = Point3::origin();
    let axis = Vector3::z();
    // Validates both radii and the major > minor constraint.
    let surface = Surface3::torus(center, axis, major_radius, minor_radius)?;

    // Major seam: the v = 0 circle (outer equator, radius R + r about the
    // axis). Minor seam: the u = 0 tube circle in the XZ plane, centered at
    // the tube center (R, 0, 0); axis -Y gives point(t) =
    // (R + r cos t, 0, r sin t), matching the surface's v parameter.
    let outer = major_radius + minor_radius;
    let major_circle = Curve3::circle(center, axis, outer)?;
    let minor_circle = Curve3::circle(
        Point3::new(major_radius, 0.0, 0.0),
        -Vector3::y(),
        minor_radius,
    )?;

    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);
    store.shells.get_mut(shell).expect("just created").genus = 1;

    let v0 = store.create_vertex(Point3::new(outer, 0.0, 0.0), SYSTEM_RESOLUTION);

    let e_major = {
        let curve = geo.add_curve(major_circle);
        make_edge(store, v0, v0, curve, 0.0, TWO_PI)
    };
    let e_minor = {
        let curve = geo.add_curve(minor_circle);
        make_edge(store, v0, v0, curve, 0.0, TWO_PI)
    };

    let face = store.create_face(shell, FaceSense::Positive);
    store.faces.get_mut(face).expect("just created").surface = Some(geo.add_surface(surface));
    store.create_loop(
        face,
        LoopType::Outer,
        &[
            (e_major, FinSense::Forward),
            (e_minor, FinSense::Forward),
            (e_major, FinSense::Reversed),
            (e_minor, FinSense::Reversed),
        ],
    );

    Ok(body)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::CurveEval;
    use crate::euler::EulerCounts;
    use crate::project::SurfaceProject;
    use crate::surface::SurfaceEval;

    /// Build all four primitives into one shared store pair.
    fn build_all() -> (TopologyStore, GeometryStore, [EntityId<Body>; 4]) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let bodies = [
            block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("valid block"),
            cylinder(&mut store, &mut geo, 1.5, 5.0).expect("valid cylinder"),
            sphere(&mut store, &mut geo, 2.5).expect("valid sphere"),
            torus(&mut store, &mut geo, 3.0, 1.0).expect("valid torus"),
        ];
        (store, geo, bodies)
    }

    #[test]
    fn all_primitives_pass_check() {
        let (store, _geo, bodies) = build_all();
        for body in bodies {
            let failures = store.check(body);
            assert!(failures.is_empty(), "{body:?} failed check: {failures:?}");
        }
    }

    #[test]
    fn entity_counts_match_theory() {
        let (store, _geo, [block, cylinder, sphere, torus]) = build_all();
        let counts = |body| store.euler_counts(body);
        let expect = |v, e, f, l, genus| EulerCounts {
            vertices: v,
            edges: e,
            faces: f,
            loops: l,
            rings: l - f,
            shells: 1,
            genus,
        };
        assert_eq!(counts(block), expect(8, 12, 6, 6, 0));
        assert_eq!(counts(cylinder), expect(2, 3, 3, 3, 0));
        assert_eq!(counts(sphere), expect(2, 1, 1, 1, 0));
        assert_eq!(counts(torus), expect(1, 2, 1, 1, 1));
        for body in [block, cylinder, sphere, torus] {
            assert!(counts(body).euler_poincare_holds(), "{body:?}");
        }
    }

    #[test]
    fn every_face_has_the_expected_surface() {
        let (store, geo, [block, cylinder, sphere, torus]) = build_all();
        let kinds = |body| -> Vec<&'static str> {
            store
                .faces_of_body(body)
                .iter()
                .map(|&f| {
                    let id = store
                        .face(f)
                        .unwrap()
                        .surface
                        .expect("face must carry a surface");
                    match geo.surface(id).expect("surface id must resolve") {
                        Surface3::Plane { .. } => "plane",
                        Surface3::Cylinder { .. } => "cylinder",
                        Surface3::Cone { .. } => "cone",
                        Surface3::Sphere { .. } => "sphere",
                        Surface3::Torus { .. } => "torus",
                    }
                })
                .collect()
        };
        assert_eq!(kinds(block), vec!["plane"; 6]);
        assert_eq!(kinds(cylinder), vec!["plane", "plane", "cylinder"]);
        assert_eq!(kinds(sphere), vec!["sphere"]);
        assert_eq!(kinds(torus), vec!["torus"]);
    }

    #[test]
    fn every_body_is_a_closed_outward_solid() {
        let (store, _geo, bodies) = build_all();
        for body in bodies {
            assert_eq!(store.body(body).unwrap().body_type, BodyType::Solid);
            let shells = store.shells_of_body(body);
            assert_eq!(shells.len(), 1);
            let shell = store.shell(shells[0]).unwrap();
            assert!(shell.is_closed);
            assert_eq!(shell.orientation, ShellOrientation::Outward);
            for &face in store.faces_of_body(body).iter() {
                assert_eq!(store.face(face).unwrap().sense, FaceSense::Positive);
            }
        }
    }

    #[test]
    fn edge_curves_interpolate_their_vertices() {
        let (store, geo, bodies) = build_all();
        for body in bodies {
            for face in store.faces_of_body(body) {
                for edge_id in store.edges_of_face(face) {
                    let edge = store.edge(edge_id).unwrap();
                    let curve = geo
                        .curve(edge.curve.expect("edge must carry a curve"))
                        .expect("curve id must resolve");
                    let start = store.vertex(edge.start_vertex).unwrap().point;
                    let end = store.vertex(edge.end_vertex).unwrap().point;
                    assert!(
                        (curve.point(edge.t_start) - start).norm() < 1e-9,
                        "{edge_id:?}: curve start off vertex"
                    );
                    assert!(
                        (curve.point(edge.t_end) - end).norm() < 1e-9,
                        "{edge_id:?}: curve end off vertex"
                    );
                }
            }
        }
    }

    #[test]
    fn edge_curves_lie_on_adjacent_surfaces() {
        let (store, geo, bodies) = build_all();
        for body in bodies {
            for face in store.faces_of_body(body) {
                for edge_id in store.edges_of_face(face) {
                    let edge = store.edge(edge_id).unwrap();
                    let curve = geo.curve(edge.curve.unwrap()).unwrap();
                    for adjacent in store.faces_of_edge(edge_id) {
                        let surface_id = store.face(adjacent).unwrap().surface.unwrap();
                        let surface = geo.surface(surface_id).unwrap();
                        for k in 0..=8 {
                            let t = edge.t_start + (edge.t_end - edge.t_start) * f64::from(k) / 8.0;
                            let p = curve.point(t);
                            let proj = surface.project_point(&p);
                            assert!(
                                proj.distance < 1e-8,
                                "{edge_id:?} at t={t}: {:.2e} off {adjacent:?}",
                                proj.distance
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn block_dimensions_and_outward_normals() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 2.0, 4.0, 6.0).expect("valid block");

        // Vertices span the centered extents.
        let faces = store.faces_of_body(body);
        for &face in &faces {
            for v in store.vertices_of_face(face) {
                let p = store.vertex(v).unwrap().point;
                assert_eq!(p.x.abs(), 1.0);
                assert_eq!(p.y.abs(), 2.0);
                assert_eq!(p.z.abs(), 3.0);
            }
        }

        // Each face's plane normal points away from the body center: the
        // loop cycles in the builder agree with the outward surfaces.
        for &face in &faces {
            let corners: Vec<Point3> = store
                .vertices_of_face(face)
                .iter()
                .map(|&v| store.vertex(v).unwrap().point)
                .collect();
            let centroid = Point3::from(
                corners.iter().map(|p| p.coords).sum::<Vector3>() / corners.len() as f64,
            );
            let surface_id = store.face(face).unwrap().surface.unwrap();
            let normal = geo.surface(surface_id).unwrap().normal(0.0, 0.0).unwrap();
            assert!(
                normal.dot(&centroid.coords) > 0.0,
                "{face:?}: plane normal points inward"
            );
        }
    }

    #[test]
    fn cylinder_caps_at_expected_heights() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = cylinder(&mut store, &mut geo, 1.5, 5.0).expect("valid cylinder");

        let faces = store.faces_of_body(body);
        let normals: Vec<Vector3> = faces
            .iter()
            .take(2)
            .map(|&f| {
                let id = store.face(f).unwrap().surface.unwrap();
                geo.surface(id).unwrap().normal(0.0, 0.0).unwrap()
            })
            .collect();
        assert!((normals[0] - -Vector3::z()).norm() < 1e-12, "bottom cap");
        assert!((normals[1] - Vector3::z()).norm() < 1e-12, "top cap");

        // The two vertices sit on the seam at z = ±h/2.
        let mut zs: Vec<f64> = store.vertices.iter().map(|(_, v)| v.point.z).collect();
        zs.sort_by(f64::total_cmp);
        assert_eq!(zs, vec![-2.5, 2.5]);
    }

    #[test]
    fn sphere_seam_spans_the_poles() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = sphere(&mut store, &mut geo, 2.5).expect("valid sphere");

        let face = store.faces_of_body(body)[0];
        let loop_id = store.face(face).unwrap().outer_loop.unwrap();
        let fins = store.fins_of_loop(loop_id);
        assert_eq!(fins.len(), 2, "seam out and back");
        assert_eq!(store.fin_edge(fins[0]), store.fin_edge(fins[1]));

        let edge = store.edge(store.fin_edge(fins[0])).unwrap();
        let south = store.vertex(edge.start_vertex).unwrap().point;
        let north = store.vertex(edge.end_vertex).unwrap().point;
        assert!((south - Point3::new(0.0, 0.0, -2.5)).norm() < 1e-12);
        assert!((north - Point3::new(0.0, 0.0, 2.5)).norm() < 1e-12);
    }

    #[test]
    fn torus_seams_meet_on_the_outer_equator() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = torus(&mut store, &mut geo, 3.0, 1.0).expect("valid torus");

        let shells = store.shells_of_body(body);
        assert_eq!(store.shell(shells[0]).unwrap().genus, 1);

        let face = store.faces_of_body(body)[0];
        let loop_id = store.face(face).unwrap().outer_loop.unwrap();
        let fins = store.fins_of_loop(loop_id).to_vec();
        assert_eq!(fins.len(), 4, "fundamental polygon a b a⁻¹ b⁻¹");
        // Fins 0/2 share the major seam, 1/3 the minor seam, with opposite
        // senses.
        assert_eq!(store.fin_edge(fins[0]), store.fin_edge(fins[2]));
        assert_eq!(store.fin_edge(fins[1]), store.fin_edge(fins[3]));
        assert_ne!(store.fin_edge(fins[0]), store.fin_edge(fins[1]));

        // Single vertex on the outer equator.
        assert_eq!(store.vertices.len(), 1);
        let (_, v0) = store.vertices.iter().next().unwrap();
        assert!((v0.point - Point3::new(4.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn builders_reject_bad_dimensions() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();

        assert!(block(&mut store, &mut geo, 0.0, 1.0, 1.0).is_err());
        assert!(block(&mut store, &mut geo, 1.0, -2.0, 1.0).is_err());
        assert!(block(&mut store, &mut geo, 1.0, 1.0, f64::NAN).is_err());
        assert!(cylinder(&mut store, &mut geo, 0.0, 1.0).is_err());
        assert!(cylinder(&mut store, &mut geo, 1.0, f64::INFINITY).is_err());
        assert!(sphere(&mut store, &mut geo, -1.0).is_err());
        // Spindle torus (major <= minor) rejected by the surface constructor.
        assert!(torus(&mut store, &mut geo, 1.0, 1.0).is_err());
        assert!(torus(&mut store, &mut geo, 0.5, 1.0).is_err());

        // Failed builders leave both stores untouched.
        assert!(store.bodies.is_empty());
        assert!(store.vertices.is_empty());
        assert!(geo.curves.is_empty());
        assert!(geo.surfaces.is_empty());
    }
}
