//! End-to-end acceptance tests for the hybrid F-Rep + B-Rep story
//! (of-ipt.3): mixed-representation booleans through [`hybrid::boolean`],
//! forced fallback on exact-pipeline failure, and representation
//! round-trip stability. These exercise only the public kernel API — they
//! are the "kernel MVP is actually done" gate.

use std::f64::consts::PI;

use opensolid_kernel::brep::boolean::{subtract as brep_subtract, unite as brep_unite};
use opensolid_kernel::brep::{
    Body, GeometryStore, TessellationOptions, TopologyStore, primitives, tessellate_body,
    translate_body,
};
use opensolid_kernel::builder::shape;
use opensolid_kernel::core::EntityId;
use opensolid_kernel::core::mesh::TriangleMesh;
use opensolid_kernel::core::types::{BoundingBox3, Point3, Vector3};
use opensolid_kernel::hybrid::{self, HybridBody, HybridOptions, HybridPath};
use opensolid_kernel::{MeshSdf, SdfToBrepOptions, mass_properties, sdf_to_brep};

fn opts() -> HybridOptions {
    HybridOptions::default()
}

fn volume(mesh: &TriangleMesh) -> f64 {
    mass_properties(mesh).expect("closed manifold mesh").volume
}

fn assert_volume_within(mesh: &TriangleMesh, exact: f64, rel_tol: f64, context: &str) {
    let got = volume(mesh);
    assert!(
        (got - exact).abs() / exact < rel_tol,
        "{context}: volume {got} not within {:.1}% of analytic {exact}",
        rel_tol * 100.0
    );
}

/// Volume of a body via mass properties on its tessellation.
///
/// Faceted bodies recovered by `sdf_to_brep` now ear-clip into a closed,
/// consistently oriented manifold (of-6sq), so the strict manifold gate in
/// `mass_properties` accepts them directly.
fn body_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> f64 {
    let mesh = tessellate_body(store, geo, body, &TessellationOptions::default())
        .expect("body tessellates");
    assert!(
        mesh.is_closed_manifold(),
        "recovered faceted body must tessellate to a closed manifold"
    );
    volume(&mesh)
}

/// Acceptance (1a): an implicit sphere minus an exact B-Rep block covering
/// the (+,+,+) octant. Mixed representations must take the F-Rep path and
/// deliver a watertight mesh whose volume matches the analytic 7/8 ball.
#[test]
fn frep_sphere_minus_brep_block_is_closed_and_volume_accurate() {
    let ball: HybridBody = shape::sphere(1.0).unwrap().into();
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let block = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
    translate_body(&mut store, &mut geo, block, Vector3::new(1.0, 1.0, 1.0)).unwrap();

    let out = hybrid::subtract(&ball, &HybridBody::brep(&store, &geo, block), &opts()).unwrap();

    assert!(
        matches!(out.path, HybridPath::Frep { .. }),
        "mixed representations must take the F-Rep path"
    );
    assert!(out.mesh.is_closed_manifold(), "result must be watertight");
    assert_volume_within(
        &out.mesh,
        (7.0 / 8.0) * (4.0 / 3.0) * PI,
        0.03,
        "F-Rep sphere minus B-Rep octant block",
    );
}

/// Acceptance (1b): an exact B-Rep cylinder united with an implicit torus
/// threaded around it (chain-link style: interlocked but disjoint, so the
/// union volume is exactly the sum). The result must be one watertight
/// mesh containing both genus-carrying components.
#[test]
fn brep_cylinder_united_with_frep_torus_is_closed_and_volume_accurate() {
    // B-Rep cylinder along +Z: radius 0.5, z ∈ [-1.5, 1.5].
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let cyl = primitives::cylinder(&mut store, &mut geo, 0.5, 3.0).unwrap();

    // F-Rep torus, ring rotated from the XZ into the XY plane so its hole
    // wraps the cylinder axis: tube center circle radius 1.5, tube radius
    // 0.5 — closest approach to the cylinder surface is 0.5.
    let torus: HybridBody = shape::torus(1.5, 0.5)
        .unwrap()
        .rotate_x(90.0)
        .unwrap()
        .into();

    let out = hybrid::unite(&HybridBody::brep(&store, &geo, cyl), &torus, &opts()).unwrap();

    assert!(
        matches!(out.path, HybridPath::Frep { .. }),
        "mixed representations must take the F-Rep path"
    );
    assert!(out.mesh.is_closed_manifold(), "result must be watertight");
    let cylinder_volume = PI * 0.5 * 0.5 * 3.0;
    let torus_volume = 2.0 * PI * PI * 1.5 * 0.5 * 0.5;
    assert_volume_within(
        &out.mesh,
        cylinder_volume + torus_volume,
        0.03,
        "B-Rep cylinder united with linked F-Rep torus",
    );
}

/// Acceptance (2): force an exact-pipeline failure — the tool block shares
/// four coincident side planes with the target, which the B-Rep boolean
/// rejects — and verify the kernel diverts to the F-Rep fallback and still
/// returns a valid, volume-accurate result.
#[test]
fn coincident_face_failure_falls_back_to_frep_and_stays_valid() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let target = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
    let tool = primitives::block(&mut store, &mut geo, 1.0, 2.0, 2.0).unwrap();
    translate_body(&mut store, &mut geo, tool, Vector3::new(0.75, 0.0, 0.0)).unwrap();

    // Precondition: the exact pipeline really does refuse this input. If
    // it ever learns to handle coincident faces, this test should be
    // rethought rather than silently passing.
    assert!(
        brep_subtract(&store, &geo, target, tool, &opts().tol).is_err(),
        "precondition: exact B-Rep subtract must reject coincident faces"
    );

    let out = hybrid::subtract(
        &HybridBody::brep(&store, &geo, target),
        &HybridBody::brep(&store, &geo, tool),
        &opts(),
    )
    .unwrap();

    assert!(
        matches!(out.path, HybridPath::Frep { .. }),
        "exact-pipeline failure must divert to the F-Rep fallback"
    );
    assert!(out.mesh.is_closed_manifold(), "result must be watertight");
    // Tool overlaps x ∈ [0.25, 1.0] of the target: 8 − 0.75·2·2 = 5.
    assert_volume_within(&out.mesh, 5.0, 0.03, "subtract with coincident side faces");
}

/// Same forced-failure check for unite, mirroring the classic coincident
/// overlap: two blocks of equal cross-section partially overlapping along
/// X, all four side planes coincident.
#[test]
fn coincident_face_union_falls_back_to_frep_and_stays_valid() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = primitives::block(&mut store, &mut geo, 1.0, 1.0, 1.0).unwrap();
    let b = primitives::block(&mut store, &mut geo, 1.2, 1.0, 1.0).unwrap();
    translate_body(&mut store, &mut geo, b, Vector3::new(0.35, 0.0, 0.0)).unwrap();
    assert!(
        brep_unite(&store, &geo, a, b, &opts().tol).is_err(),
        "precondition: exact B-Rep unite must reject coincident faces"
    );

    let out = hybrid::unite(
        &HybridBody::brep(&store, &geo, a),
        &HybridBody::brep(&store, &geo, b),
        &opts(),
    )
    .unwrap();
    assert!(matches!(out.path, HybridPath::Frep { .. }));
    assert!(out.mesh.is_closed_manifold());
    // Union spans x ∈ [-0.5, 0.95] with unit cross-section.
    assert_volume_within(&out.mesh, 1.45, 0.03, "unite with coincident side faces");
}

/// One representation conversion cycle: tessellate the body into a mesh
/// SDF, then recover a faceted B-Rep from the field by adaptive dual
/// contouring. The sampling cube ±1.4 keeps the test bodies' surfaces
/// (extent ≤ ±1) strictly inside at depth 5 (32 cells).
fn cycle(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
) -> (TopologyStore, GeometryStore, EntityId<Body>) {
    let sdf = MeshSdf::from_body(store, geo, body, &TessellationOptions::default())
        .expect("B-Rep body wraps as a signed distance field");
    let mut out_store = TopologyStore::new();
    let mut out_geo = GeometryStore::new();
    let bounds = BoundingBox3::new(Point3::new(-1.4, -1.4, -1.4), Point3::new(1.4, 1.4, 1.4));
    let recovered = sdf_to_brep(
        &sdf,
        &mut out_store,
        &mut out_geo,
        &SdfToBrepOptions::new(bounds, 5),
    )
    .expect("field recovers a faceted B-Rep body");
    assert!(
        out_store.check(recovered).is_empty(),
        "recovered body must pass the topology checker"
    );
    (out_store, out_geo, recovered)
}

/// Acceptance (3a): one full representation cycle. A curved B-Rep body
/// converted B-Rep → SDF → B-Rep must come back checker-clean with its
/// volume within tolerance of the analytic value.
#[test]
fn brep_sdf_round_trip_preserves_volume_one_cycle() {
    let analytic = PI * 2.0; // cylinder: radius 1, height 2
    let mut store0 = TopologyStore::new();
    let mut geo0 = GeometryStore::new();
    let cyl = primitives::cylinder(&mut store0, &mut geo0, 1.0, 2.0).unwrap();

    let (store1, geo1, body1) = cycle(&store0, &geo0, cyl);
    let v1 = body_volume(&store1, &geo1, body1);
    assert!(
        (v1 - analytic).abs() / analytic < 0.03,
        "cycle 1 volume {v1} not within 3% of analytic {analytic}"
    );
}

/// Acceptance (3b): two full cycles — the second re-imaging must not
/// compound the error.
///
/// The second cycle re-images an already-faceted body: `tessellate_body`
/// ear-clips its planar region boundaries into a closed manifold (of-6sq),
/// so `MeshSdf::from_body` accepts it and the second B-Rep → SDF conversion
/// runs through the public API.
#[test]
fn brep_sdf_round_trip_preserves_volume_across_two_cycles() {
    let analytic = PI * 2.0; // cylinder: radius 1, height 2
    let mut store0 = TopologyStore::new();
    let mut geo0 = GeometryStore::new();
    let cyl = primitives::cylinder(&mut store0, &mut geo0, 1.0, 2.0).unwrap();

    let (store1, geo1, body1) = cycle(&store0, &geo0, cyl);
    let v1 = body_volume(&store1, &geo1, body1);
    assert!(
        (v1 - analytic).abs() / analytic < 0.03,
        "cycle 1 volume {v1} not within 3% of analytic {analytic}"
    );

    let (store2, geo2, body2) = cycle(&store1, &geo1, body1);
    let v2 = body_volume(&store2, &geo2, body2);
    assert!(
        (v2 - analytic).abs() / analytic < 0.03,
        "cycle 2 volume {v2} not within 3% of analytic {analytic}"
    );
    // Stability: re-imaging an already-faceted body must not compound the
    // error — the second cycle stays within 1% of the first.
    assert!(
        (v2 - v1).abs() / v1 < 0.01,
        "round-trip drift: cycle 1 volume {v1} vs cycle 2 volume {v2}"
    );
}

/// The F-Rep fallback field itself round-trips: a hybrid result's faceted
/// B-Rep recovery converts back into a field whose re-mesh keeps the
/// volume. This ties acceptance (2) and (3) together — the fallback output
/// is a first-class citizen of both representations.
#[test]
fn hybrid_fallback_result_round_trips_through_faceted_brep() {
    let ball: HybridBody = shape::sphere(1.0).unwrap().into();
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let block = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
    translate_body(&mut store, &mut geo, block, Vector3::new(1.0, 1.0, 1.0)).unwrap();
    let out = hybrid::subtract(&ball, &HybridBody::brep(&store, &geo, block), &opts()).unwrap();
    let reference = volume(&out.mesh);

    let mut fac_store = TopologyStore::new();
    let mut fac_geo = GeometryStore::new();
    let faceted = out
        .faceted_brep(&mut fac_store, &mut fac_geo, 6)
        .expect("fallback result recovers a faceted B-Rep");
    assert!(fac_store.check(faceted).is_empty());

    let v = body_volume(&fac_store, &fac_geo, faceted);
    assert!(
        (v - reference).abs() / reference < 0.03,
        "faceted recovery volume {v} strays from hybrid mesh volume {reference}"
    );
}
