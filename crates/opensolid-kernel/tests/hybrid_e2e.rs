//! End-to-end acceptance tests for the hybrid F-Rep + B-Rep story
//! (of-ipt.3): mixed-representation booleans through [`hybrid::boolean`],
//! forced fallback on exact-pipeline failure, and representation
//! round-trip stability. These exercise only the public kernel API — they
//! are the "kernel MVP is actually done" gate.

use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2, PI};

use opensolid_kernel::brep::boolean::{subtract as brep_subtract, unite as brep_unite};
use opensolid_kernel::brep::{
    Body, BodyType, Curve3, FaceSense, FinSense, GeometryStore, KnotVector, LoopType, NurbsSurface,
    SYSTEM_RESOLUTION, ShellOrientation, Surface3, TessellationOptions, TopologyStore, primitives,
    tessellate_body, translate_body,
};
use opensolid_kernel::builder::shape;
use opensolid_kernel::core::EntityId;
use opensolid_kernel::core::error::CoreError;
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

/// Acceptance (2): the tool block shares four coincident side planes with
/// the target. This was the kernel's headline gap — the exact pipeline
/// refused the input and the kernel quietly served a mesh-derived answer
/// where an exact one was asked for — and of-bxl.4 closed it, so the same
/// input must now come back down the *exact* path.
///
/// Rewritten rather than deleted, per COINCIDENT.md §7: the comment this
/// replaces anticipated exactly this change. What it asserted (that the
/// coincident case diverts) is now false; what it was really protecting —
/// that the answer is watertight and volume-accurate whichever path serves
/// it — is kept, and tightened, since the exact path owes a far better
/// tolerance than the F-Rep fallback's 3%.
#[test]
fn coincident_face_subtract_now_takes_the_exact_path() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let target = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
    let tool = primitives::block(&mut store, &mut geo, 1.0, 2.0, 2.0).unwrap();
    translate_body(&mut store, &mut geo, tool, Vector3::new(0.75, 0.0, 0.0)).unwrap();

    let exact = brep_subtract(&store, &geo, target, tool, &opts().tol)
        .expect("of-bxl.4: exact B-Rep subtract handles coincident side faces");
    assert!(
        exact.check().is_empty(),
        "exact result must be a valid solid: {:?}",
        exact.check()
    );

    let out = hybrid::subtract(
        &HybridBody::brep(&store, &geo, target),
        &HybridBody::brep(&store, &geo, tool),
        &opts(),
    )
    .unwrap();

    assert!(
        matches!(out.path, HybridPath::Brep(_)),
        "coincident faces must no longer divert to the F-Rep fallback"
    );
    assert!(out.mesh.is_closed_manifold(), "result must be watertight");
    // Tool overlaps x ∈ [0.25, 1.0] of the target: 8 − 0.75·2·2 = 5.
    assert_volume_within(
        &out.mesh,
        5.0,
        1e-9,
        "exact subtract with coincident side faces",
    );
}

/// The same for unite: the classic coincident overlap — two blocks of equal
/// cross-section partially overlapping along X, all four side planes
/// coincident — now runs exactly (of-bxl.4).
#[test]
fn coincident_face_union_now_takes_the_exact_path() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = primitives::block(&mut store, &mut geo, 1.0, 1.0, 1.0).unwrap();
    let b = primitives::block(&mut store, &mut geo, 1.2, 1.0, 1.0).unwrap();
    translate_body(&mut store, &mut geo, b, Vector3::new(0.35, 0.0, 0.0)).unwrap();
    let exact = brep_unite(&store, &geo, a, b, &opts().tol)
        .expect("of-bxl.4: exact B-Rep unite handles coincident side faces");
    assert!(
        exact.check().is_empty(),
        "exact result must be a valid solid: {:?}",
        exact.check()
    );

    let out = hybrid::unite(
        &HybridBody::brep(&store, &geo, a),
        &HybridBody::brep(&store, &geo, b),
        &opts(),
    )
    .unwrap();
    assert!(matches!(out.path, HybridPath::Brep(_)));
    assert!(out.mesh.is_closed_manifold());
    // Union spans x ∈ [-0.5, 0.95] with unit cross-section.
    assert_volume_within(
        &out.mesh,
        1.45,
        1e-9,
        "exact unite with coincident side faces",
    );
}

/// The F-Rep fallback acceptance the two tests above used to carry.
///
/// Their coincident-block inputs no longer reach the fallback, but the
/// property they were really pinning — that an exact-pipeline shortfall
/// diverts to F-Rep and still returns a watertight, volume-accurate body —
/// is permanent, and must keep a live test. So it moves to a contact the
/// exact path genuinely still refuses: a cylinder tangent to a block's face
/// plane, which is of-bxl.6's tier and stays `NotImplemented` by design
/// (COINCIDENT.md §5 — the fallback is not a gap to close, it is the
/// backstop).
#[test]
fn exact_pipeline_shortfall_still_falls_back_to_frep_and_stays_valid() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let block = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).unwrap();
    let tool = primitives::cylinder(&mut store, &mut geo, 0.5, 4.0).unwrap();
    // Wall tangent to the block's x = 1 face plane from outside.
    translate_body(&mut store, &mut geo, tool, Vector3::new(1.5, 0.0, 0.0)).unwrap();

    // Precondition: the exact pipeline really does refuse this input. If it
    // ever learns tangent contact (of-bxl.6), rethink this test rather than
    // let it silently pass.
    let err = brep_subtract(&store, &geo, block, tool, &opts().tol)
        .expect_err("precondition: exact B-Rep must still reject tangent contact");
    assert!(
        matches!(err, CoreError::NotImplemented { .. }),
        "expected a structured shortfall, got {err:?}"
    );

    let out = hybrid::subtract(
        &HybridBody::brep(&store, &geo, block),
        &HybridBody::brep(&store, &geo, tool),
        &opts(),
    )
    .unwrap();
    assert!(
        matches!(out.path, HybridPath::Frep { .. }),
        "an exact-pipeline shortfall must divert to the F-Rep fallback"
    );
    assert!(out.mesh.is_closed_manifold(), "result must be watertight");
    // Tangent from outside: the cylinder removes nothing.
    assert_volume_within(&out.mesh, 8.0, 0.03, "subtract with tangent contact");
}

/// Tier-2 tangent contact (of-bxl.6, COINCIDENT.md §6): a sphere resting
/// on a plate, united, is a body with a non-manifold vertex — not
/// representable in the exact topology, and by design NOT a gap the exact
/// path closes. It must divert to the F-Rep fallback. The fallback is the
/// assertion here, not the result's quality.
#[test]
fn sphere_resting_on_plate_union_falls_back_to_frep() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let plate = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).unwrap();
    let ball = primitives::sphere(&mut store, &mut geo, 1.0).unwrap();
    // Plate top face at z = 1; the ball's south pole touches its center.
    translate_body(&mut store, &mut geo, ball, Vector3::new(0.0, 0.0, 2.0)).unwrap();

    let err = brep_unite(&store, &geo, plate, ball, &opts().tol)
        .expect_err("precondition: exact B-Rep must refuse in-trim tangent point contact");
    assert!(
        matches!(err, CoreError::NotImplemented { .. }),
        "expected a structured shortfall, got {err:?}"
    );

    let out = hybrid::unite(
        &HybridBody::brep(&store, &geo, plate),
        &HybridBody::brep(&store, &geo, ball),
        &opts(),
    )
    .unwrap();
    assert!(
        matches!(out.path, HybridPath::Frep { .. }),
        "tier-2 tangent contact must divert to the F-Rep fallback"
    );
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

// =====================================================================
// FREEFORM §9 promotion: NURBS operands on the exact path (of-ew7)
// =====================================================================
//
// The §9 promotion gate is the section-(14) campaign in
// `crates/opensolid-brep/tests/boolean_stress.rs`, and it is green. But
// that suite calls `boolean_with_inside_tests` directly, handing each
// NURBS operand a hand-written closed-form inside test — it proves the
// *pipeline* classifies and reconstructs NURBS-hosted solids correctly,
// not that a caller of the kernel gets the exact path.
//
// These tests close that gap: they go through the public
// [`hybrid::boolean`] entry point with no inside tests supplied, so the
// kernel must build the of-3oj `MeshSdf` sign crutch itself, run the
// exact pipeline, and clear all three acceptance gates (closed manifold,
// chordal deviation within an F-Rep cell, and the `validate_exact`
// volume cross-check) before it may return [`HybridPath::Brep`]. An
// assertion of `Brep` here is therefore the promotion itself: NURBS
// operands are no longer routed to the F-Rep fallback as a class.
//
// The router keeps the exact-or-fallback deviation check regardless —
// promotion removes the *class* divert, not the per-result quality bar.
// Curved NURBS walls still tessellate at the angular pitch, so their
// deviation is a phase-4 accuracy question (of-37i.6), not a phase-3
// one; the operands here are planar-patch hexahedra, whose chords are
// exact.

/// The 8 corners of the axis-aligned box `min..max` in `primitives::block`
/// corner order: bottom ring (`z = min`) counterclockwise from
/// `(min, min)`, then the top ring above it.
fn box_corners(min: [f64; 3], max: [f64; 3]) -> [Point3; 8] {
    [
        Point3::new(min[0], min[1], min[2]),
        Point3::new(max[0], min[1], min[2]),
        Point3::new(max[0], max[1], min[2]),
        Point3::new(min[0], max[1], min[2]),
        Point3::new(min[0], min[1], max[2]),
        Point3::new(max[0], min[1], max[2]),
        Point3::new(max[0], max[1], max[2]),
        Point3::new(min[0], max[1], max[2]),
    ]
}

/// Hexahedral solid whose six faces are all degree-1 B-spline patches over
/// the unit knot domain — the same construction as
/// `boolean_stress.rs`'s `Scene::nurbs_hexahedron` at `spans = 1`, reduced
/// to the axis-aligned case this file needs. Geometrically identical to
/// `primitives::block`, but every surface the pipeline sees is
/// [`Surface3::Nurbs`], so `body_has_nurbs_face` is true and the exact
/// path cannot classify it by ray parity.
///
/// Control points are ordered so each patch's `du × dv` points out of the
/// solid, matching `FaceSense::Positive` + `ShellOrientation::Outward`.
fn nurbs_block(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    min: [f64; 3],
    max: [f64; 3],
) -> EntityId<Body> {
    /// Undirected edges as (low, high) corner-index pairs: bottom ring,
    /// top ring, then the four verticals.
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
    /// Vertex cycles counterclockwise viewed from outside, identical to
    /// `primitives::block`'s `face_specs`.
    const FACE_CYCLES: [[usize; 4]; 6] = [
        [0, 3, 2, 1], // bottom (−Z)
        [4, 5, 6, 7], // top (+Z)
        [0, 1, 5, 4], // front (−Y)
        [1, 2, 6, 5], // right (+X)
        [2, 3, 7, 6], // back (+Y)
        [3, 0, 4, 7], // left (−X)
    ];

    let corners = box_corners(min, max);
    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);
    let vertices = corners.map(|p| store.create_vertex(p, SYSTEM_RESOLUTION));

    let edges: Vec<_> = EDGE_PAIRS
        .iter()
        .map(|&(a, b)| {
            let line = Curve3::line(corners[a], corners[b] - corners[a]).expect("distinct corners");
            let length = (corners[b] - corners[a]).norm();
            let curve = geo.add_curve(line);
            store.create_edge_with_curve(
                vertices[a],
                vertices[b],
                SYSTEM_RESOLUTION,
                curve,
                0.0,
                length,
            )
        })
        .collect();

    let directed_edge = |from: usize, to: usize| -> (EntityId<_>, FinSense) {
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

    // Clamped degree-1 knots over [0, 1] for a 2x2 control grid.
    let knots = || KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).expect("valid clamped deg-1 knots");

    for cycle in FACE_CYCLES {
        // Row-major `[i][j]` with `i↔u`, `j↔v`: row u=0 is (c0, c3), row
        // u=1 is (c1, c2), so `(u,v)=(0,0)→c0`, `(1,0)→c1`, `(1,1)→c2`,
        // `(0,1)→c3` and the normal at the origin is `(c1−c0)×(c3−c0)`.
        let grid = vec![
            vec![corners[cycle[0]], corners[cycle[3]]],
            vec![corners[cycle[1]], corners[cycle[2]]],
        ];
        let patch = NurbsSurface::bspline(grid, knots(), knots()).expect("rectangular 2x2 grid");
        let surface = geo.add_surface(Surface3::nurbs(patch));
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface = Some(surface);
        let loop_edges: Vec<_> = (0..4)
            .map(|i| directed_edge(cycle[i], cycle[(i + 1) % 4]))
            .collect();
        store.create_loop(face, LoopType::Outer, &loop_edges);
    }
    body
}

/// Promotion, half of it: a NURBS-hosted solid subtracted from an
/// **analytic** block through the public hybrid API must land on the exact
/// path. Only operand A is NURBS, so the kernel builds one `MeshSdf` sign
/// test and B still classifies by exact ray parity — the mixed case, and
/// the one that would silently regress first if `body_has_nurbs_face`
/// stopped firing.
///
/// Volumes: the NURBS box is `2³ = 8` and the analytic tool covers its
/// `x ≥ 1` half, so `A − B = 4`. Planar patches tessellate exactly, so
/// this is checked at the planar tolerance, not a curved one.
#[test]
fn nurbs_operand_against_analytic_takes_the_exact_path() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = nurbs_block(&mut store, &mut geo, [0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b = primitives::block(&mut store, &mut geo, 2.0, 4.0, 4.0).unwrap();
    translate_body(&mut store, &mut geo, b, Vector3::new(2.0, 1.0, 1.0)).unwrap();

    let out = hybrid::subtract(
        &HybridBody::brep(&store, &geo, a),
        &HybridBody::brep(&store, &geo, b),
        &opts(),
    )
    .expect("NURBS minus analytic block");

    assert!(
        matches!(out.path, HybridPath::Brep(_)),
        "FREEFORM §9 promotion (of-ew7): a NURBS operand must take the \
         exact B-Rep path, not the F-Rep fallback — got {:?}",
        out.diagnostic
    );
    assert!(out.mesh.is_closed_manifold());
    assert_volume_within(&out.mesh, 4.0, 1e-9, "NURBS box minus analytic block");
}

/// Promotion, the rest of it: **both** operands NURBS — every cut is a
/// NURBS patch meeting a NURBS patch, so the kernel builds two `MeshSdf`
/// sign tests and the whole boolean runs through NURBS↔NURBS SSI and
/// NURBS-on-NURBS region tracing with no analytic surface anywhere.
#[test]
fn nurbs_pair_boolean_takes_the_exact_path() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = nurbs_block(&mut store, &mut geo, [0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b = nurbs_block(&mut store, &mut geo, [1.0, -1.0, -1.0], [3.0, 3.0, 3.0]);
    let (ha, hb) = (
        HybridBody::brep(&store, &geo, a),
        HybridBody::brep(&store, &geo, b),
    );

    // vol(A) = 8, vol(B) = 2·4·4 = 32, overlap = A's +x half = 4.
    for (op, expected, context) in [
        (
            hybrid::subtract(&ha, &hb, &opts()),
            4.0,
            "NURBS box minus NURBS box",
        ),
        (
            hybrid::intersect(&ha, &hb, &opts()),
            4.0,
            "NURBS box meet NURBS box",
        ),
        (
            hybrid::unite(&ha, &hb, &opts()),
            36.0,
            "NURBS box union NURBS box",
        ),
    ] {
        let out = op.unwrap_or_else(|e| panic!("{context}: {e}"));
        assert!(
            matches!(out.path, HybridPath::Brep(_)),
            "FREEFORM §9 promotion (of-ew7): {context} must take the exact \
             B-Rep path — got {:?}",
            out.diagnostic
        );
        assert!(out.mesh.is_closed_manifold(), "{context}: not watertight");
        assert_volume_within(&out.mesh, expected, 1e-9, context);
    }
}

/// The promotion does not disable the safety net. `validate_exact` is on
/// by default and the deviation gate is unconditional, so a NURBS result
/// that clears them carries `diagnostic: None` — the gates ran and stayed
/// silent, rather than being skipped for NURBS operands.
#[test]
fn promoted_nurbs_result_still_passes_the_validation_gate() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = nurbs_block(&mut store, &mut geo, [0.0, 0.0, 0.0], [2.0, 2.0, 2.0]);
    let b = nurbs_block(&mut store, &mut geo, [0.5, 0.5, -0.5], [1.5, 1.5, 2.5]);

    let options = opts();
    assert!(
        options.validation.is_some(),
        "the validation gate must be on by default for this to mean anything"
    );
    let out = hybrid::subtract(
        &HybridBody::brep(&store, &geo, a),
        &HybridBody::brep(&store, &geo, b),
        &options,
    )
    .expect("NURBS box bored by a NURBS bar");

    assert!(matches!(out.path, HybridPath::Brep(_)));
    assert!(
        out.diagnostic.is_none(),
        "an accepted exact result must carry no discard diagnostic: {:?}",
        out.diagnostic
    );
    // Box 2³ = 8 minus the 1×1×2 through-bore = 6.
    assert_volume_within(&out.mesh, 6.0, 1e-9, "NURBS box bored by NURBS bar");
}

/// Solid cylinder whose wall is four **exact** rational quadratic NURBS
/// quarter patches (the of-pb7.3 construction: 90° arcs with the middle
/// control point at the tangent intersection, weight `1/√2`) swept linearly
/// in `v`, with planar caps. Geometrically identical to
/// `primitives::cylinder`, but the walls the pipeline sees are *curved*
/// NURBS — the §9 gate's "NURBS patch of exact analytic form", and the
/// operand that separates a promotion that covers freeform geometry from one
/// that only covers planar patches.
fn nurbs_cylinder(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    r: f64,
    h: f64,
) -> EntityId<Body> {
    let axis = Vector3::z();
    let dirs = [
        Vector3::new(1.0, 0.0, 0.0),
        Vector3::new(0.0, 1.0, 0.0),
        Vector3::new(-1.0, 0.0, 0.0),
        Vector3::new(0.0, -1.0, 0.0),
    ];
    let (bottom, top) = (Point3::origin(), Point3::new(0.0, 0.0, h));
    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);

    let v_bot: Vec<_> = dirs
        .iter()
        .map(|d| store.create_vertex(bottom + d * r, SYSTEM_RESOLUTION))
        .collect();
    let v_top: Vec<_> = dirs
        .iter()
        .map(|d| store.create_vertex(top + d * r, SYSTEM_RESOLUTION))
        .collect();

    // Quarter-arc `k` spans `[kπ/2, (k+1)π/2]`: `Curve3::circle`'s parameter
    // origin is `+X`, so the arc parameter ranges line up with `dirs`.
    let mut arc = |center: Point3, verts: &[EntityId<_>], k: usize, geo: &mut GeometryStore| {
        let curve = geo.add_curve(Curve3::circle(center, axis, r).expect("valid circle"));
        store.create_edge_with_curve(
            verts[k],
            verts[(k + 1) % 4],
            SYSTEM_RESOLUTION,
            curve,
            k as f64 * FRAC_PI_2,
            (k + 1) as f64 * FRAC_PI_2,
        )
    };
    let e_bot: Vec<_> = (0..4).map(|k| arc(bottom, &v_bot, k, geo)).collect();
    let e_top: Vec<_> = (0..4).map(|k| arc(top, &v_top, k, geo)).collect();
    let e_seam: Vec<_> = (0..4)
        .map(|k| {
            let curve =
                geo.add_curve(Curve3::line(bottom + dirs[k] * r, axis).expect("valid seam"));
            store.create_edge_with_curve(v_bot[k], v_top[k], SYSTEM_RESOLUTION, curve, 0.0, h)
        })
        .collect();

    // Bottom cap looks along −Z, so its arcs run reversed in reversed order.
    let cap = |store: &mut TopologyStore,
               geo: &mut GeometryStore,
               center: Point3,
               normal: Vector3,
               fins: Vec<(EntityId<_>, FinSense)>| {
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface =
            Some(geo.add_surface(Surface3::plane(center, normal).expect("valid plane")));
        store.create_loop(face, LoopType::Outer, &fins);
    };
    cap(
        store,
        geo,
        bottom,
        -axis,
        (0..4)
            .rev()
            .map(|k| (e_bot[k], FinSense::Reversed))
            .collect(),
    );
    cap(
        store,
        geo,
        top,
        axis,
        (0..4).map(|k| (e_top[k], FinSense::Forward)).collect(),
    );

    let knots_u = KnotVector::clamped_uniform(2, 3).expect("degree-2 knots, 3 controls");
    let knots_v = KnotVector::clamped_uniform(1, 2).expect("degree-1 knots, 2 controls");
    for k in 0..4 {
        let (d0, d1) = (dirs[k], dirs[(k + 1) % 4]);
        // Middle control point sits at the tangent intersection (radius r√2).
        let control_points: Vec<Vec<Point3>> = [d0, d0 + d1, d1]
            .iter()
            .map(|d| vec![bottom + d * r, top + d * r])
            .collect();
        let weights: Vec<Vec<f64>> = [1.0, FRAC_1_SQRT_2, 1.0]
            .iter()
            .map(|&w| vec![w, w])
            .collect();
        let patch = NurbsSurface::new(control_points, weights, knots_u.clone(), knots_v.clone())
            .expect("valid rational quarter-cylinder patch");
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("just created").surface =
            Some(geo.add_surface(Surface3::nurbs(patch)));
        store.create_loop(
            face,
            LoopType::Outer,
            &[
                (e_bot[k], FinSense::Forward),
                (e_seam[(k + 1) % 4], FinSense::Forward),
                (e_top[k], FinSense::Reversed),
                (e_seam[k], FinSense::Reversed),
            ],
        );
    }
    body
}

/// How far the promotion reaches on **curved** NURBS. Held as an executable
/// bug report for of-dvj until of-37i.6; **live since**.
///
/// The planar-patch tests above cannot speak for curved operands, and this
/// one used to fail: the tool's wall patch and its planar cap share a
/// circular edge and sampled it *differently* — the cap ear-clips the curve
/// at uniform angles (11.25° steps), while the NURBS wall gridded uniformly
/// in its own rational parameter, which is not angle-uniform. The rims did
/// not weld (128 open edges on a 124-triangle mesh), `MeshSdf::new` rejected
/// the operand, and so neither the exact path's inside-test crutch nor the
/// F-Rep fallback's field could be built. It was the of-2i3 lesson in a new
/// place: adjacent faces must sample a shared edge at the same positions,
/// which the quadric path gets for free by parameterizing both sides by
/// angle.
///
/// The fix took the grid away from NURBS faces entirely: they go through the
/// same constrained-Delaunay pass a boolean *result*'s faces take, whose
/// boundary is sampled from the **edge curves** and therefore agrees with
/// whatever is on the other side of them by construction
/// (`tessellate::nurbs_face_cdt`).
///
/// The assertion is deliberately weak — succeeds, watertight,
/// volume-accurate on *whichever* path — because diverting to F-Rep on the
/// deviation gate would be a legitimate outcome; only the hard error is not.
#[test]
fn curved_nurbs_operand_produces_a_correct_result_on_whichever_path() {
    let (r, h) = (1.0, 4.0);
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let block = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).unwrap();
    let tool = nurbs_cylinder(&mut store, &mut geo, r, h);
    translate_body(&mut store, &mut geo, tool, Vector3::new(0.0, 0.0, -h / 2.0)).unwrap();

    let out = hybrid::subtract(
        &HybridBody::brep(&store, &geo, block),
        &HybridBody::brep(&store, &geo, tool),
        &opts(),
    )
    .expect("a NURBS-walled tool must have *some* path through the kernel");

    assert!(out.mesh.is_closed_manifold(), "result is not watertight");
    // Block 4×4×2 minus the through-bore π r² · 2. The F-Rep path pays dual
    // contouring's cell error on top of the tessellation's, so the bar is the
    // fallback's accuracy, not the exact path's.
    assert_volume_within(
        &out.mesh,
        4.0 * 4.0 * 2.0 - PI * r * r * 2.0,
        0.05,
        "block bored by a NURBS-walled cylinder",
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
