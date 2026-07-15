//! STEP corpus + round-trip stress tests (of-3qy.4).
//!
//! Adversarial validation of the STEP read/write pipeline, in three parts:
//!
//! 1. **Round-trip identity** — primitives and boolean outputs are written
//!    with [`write_step`], re-imported with [`read_step`], and required to
//!    come back as exact B-Reps with identical Euler counts. Where the
//!    standalone tessellator produces a closed manifold on both sides the
//!    volumes must agree within 1e-9 relative; everywhere the emitted text
//!    must reach a **fixed point** of `write ∘ read` (writing the re-imported
//!    body reproduces the file byte for byte), which pins every coordinate
//!    to the exact `f64` and the whole topology graph to the same traversal.
//!    Stores that share geometry across faces reach it from the second write
//!    rather than the first (of-kb8, below); every other case is immediate.
//! 2. **Synthetic adversarial files** — missing entities, cyclic references,
//!    degenerate geometry, unit mismatches, huge coordinates, overflowing
//!    reals, truncation, garbage. The reader must return structured errors
//!    ([`StepError`] / [`Diagnostic`]s / [`SolidOutcome::Failed`]) or clean
//!    fallbacks. It must NEVER panic and NEVER silently import wrong
//!    geometry.
//! 3. **Vendored real-world files** — CATIA V5-authored CAx-IF test parts
//!    under `tests/data/step/` (see the README there for provenance and
//!    licensing). Analytic parts must import as exact B-Reps and survive a
//!    write round trip; NURBS-bearing parts must degrade to structured
//!    diagnostics.
//!
//! Protocol (same as `boolean_stress.rs`): a failing case is documented as
//! a `bd` bug bead with a minimal repro and the test is `#[ignore]`d
//! referencing the bug ID. Failures are expected and are the point — tests
//! must not be softened to pass. Run known-broken cases with
//! `cargo test --test step_corpus -- --ignored`.
//!
//! Bugs filed from this suite (first run, 2026-07-12):
//! - of-1dd (fixed): parser stack overflow on ~500-deep nested aggregates —
//!   a 1KB crafted file aborted the process. `parse_value` now routes both
//!   recursion sites through a depth counter capped at 64, returning a
//!   structured 'aggregate nesting too deep' error instead.
//! - of-83h (fixed): reader ignored declared length units; metre and
//!   millimetre files imported identical geometry. The reader now scales
//!   coordinates into millimetres from the GLOBAL_UNIT_ASSIGNED_CONTEXT.
//! - of-as6 (fixed): `tessellate_body` ignored `FaceSense::Negative`, so
//!   planar boolean outputs (L-shape subtract) meshed with inward-wound
//!   tool faces and failed the manifold check even though
//!   `BooleanOutput::tessellate()` was closed. Iso-rectangular trimmed
//!   quadric faces (cylinder edge notch) now tessellate faithfully as
//!   partial arcs — of-2i3 (fixed). The volume half of the round-trip gate
//!   stays conditional because non-rectangular trims and sphere/torus caps
//!   still defer to the CDT pass.
//! - of-kb8: the reader duplicates shared geometry instances (one
//!   `Curve3`/`Surface3` per referencing edge/face), so `write ∘ read` is
//!   only byte-identical from the second write onwards when the source
//!   store shares geometry across faces (e.g. a boolean splitting a
//!   cylinder band into two faces on one surface).

use opensolid_kernel::brep::boolean::{BooleanOutput, intersect, subtract, unite};
use opensolid_kernel::brep::{
    Body, GeometryStore, TessellationOptions, TopologyStore, primitives, tessellate_body,
    translate_body,
};
use opensolid_kernel::core::EntityId;
use opensolid_kernel::core::tolerance::ToleranceContext;
use opensolid_kernel::core::types::Vector3;
use opensolid_kernel::io::step::read::{
    Severity, SolidOutcome, StepImport, StepReadOptions, read_step, read_step_bytes,
};
use opensolid_kernel::io::step::write::{LengthUnit, StepWriteOptions, write_step};
use opensolid_kernel::mass_properties;

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

fn tol() -> ToleranceContext {
    ToleranceContext::default()
}

/// Wrap a DATA-section body in a minimal, syntactically complete Part 21
/// envelope.
fn envelope(data: &str) -> String {
    format!(
        "ISO-10303-21;\n\
         HEADER;\n\
         FILE_DESCRIPTION((''),'2;1');\n\
         FILE_NAME('','',(''),(''),'','','');\n\
         FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\n\
         ENDSEC;\n\
         DATA;\n\
         {data}\n\
         ENDSEC;\n\
         END-ISO-10303-21;\n"
    )
}

/// Import into fresh stores. Panics only on Part 21 syntax errors — the
/// adversarial semantic cases must all get past parsing.
fn import(source: &str) -> (TopologyStore, GeometryStore, StepImport) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step(source, &mut store, &mut geo, &StepReadOptions::default())
        .expect("source must be syntactically valid Part 21");
    (store, geo, report)
}

/// Every diagnostic must carry a non-empty message, and every `Failed`
/// solid must be explained by at least one Warning/Error diagnostic —
/// "structured errors, not silence".
fn assert_structured(report: &StepImport) {
    for d in &report.diagnostics {
        assert!(
            !d.message.is_empty(),
            "diagnostic with empty message: {d:?}"
        );
    }
    for solid in &report.solids {
        if matches!(solid.outcome, SolidOutcome::Failed) {
            assert!(
                report
                    .diagnostics
                    .iter()
                    .any(|d| d.severity >= Severity::Warning),
                "solid #{} failed with no Warning/Error diagnostic",
                solid.step_id
            );
        }
    }
}

/// The single solid of `report` as an exact B-Rep body.
fn only_brep(report: &StepImport) -> EntityId<Body> {
    assert_eq!(report.solids.len(), 1, "expected exactly one solid");
    match &report.solids[0].outcome {
        SolidOutcome::BRep(body) => *body,
        other => panic!(
            "expected exact B-Rep import, got {other:?}; diagnostics: {:?}",
            report.diagnostics
        ),
    }
}

fn assert_counts_equal(
    store: &TopologyStore,
    body: EntityId<Body>,
    store2: &TopologyStore,
    body2: EntityId<Body>,
    context: &str,
) {
    let a = store.euler_counts(body);
    let b = store2.euler_counts(body2);
    assert_eq!(a.vertices, b.vertices, "{context}: vertex count");
    assert_eq!(a.edges, b.edges, "{context}: edge count");
    assert_eq!(a.faces, b.faces, "{context}: face count");
    assert_eq!(a.loops, b.loops, "{context}: loop count");
    assert_eq!(a.rings, b.rings, "{context}: ring count");
    assert_eq!(a.shells, b.shells, "{context}: shell count");
    assert_eq!(a.genus, b.genus, "{context}: genus");
}

/// Volume via the standalone store tessellator, only when it produces a
/// closed manifold — it may not, on bodies with non-rectangular trimmed
/// quadric faces or sphere/torus caps that still defer to the CDT pass
/// (of-2i3 handled the iso-rectangular cylinder/cone case).
fn closed_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> Option<f64> {
    let mesh = tessellate_body(store, geo, body, &TessellationOptions::default()).ok()?;
    mass_properties(&mesh).ok().map(|mp| mp.volume)
}

/// The full round-trip gate: write → read (exact B-Rep, no error
/// diagnostics, clean check) → identical Euler counts → write again and
/// require the byte-identical file (fixed point). Volume is compared when
/// the tessellator can measure both sides (the CDT-pass cases gate the rest).
fn assert_round_trip(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    context: &str,
) {
    assert_round_trip_gate(store, geo, body, context, FixedPoint::Immediate);
}

/// Whether `write ∘ read` must reproduce the file on the first re-write or
/// only from the second one (of-kb8: stores sharing one surface/curve
/// across faces/edges re-import with duplicated geometry instances, which
/// shifts the fixed point one iteration out).
#[derive(Clone, Copy, PartialEq)]
enum FixedPoint {
    Immediate,
    AfterOneTrip,
}

fn assert_round_trip_gate(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    context: &str,
    fixed_point: FixedPoint,
) {
    assert!(
        store.check(body).is_empty(),
        "{context}: original body must pass check: {:?}",
        store.check(body)
    );
    let text = write_step(store, geo, &[body], &StepWriteOptions::default())
        .unwrap_or_else(|e| panic!("{context}: body must serialize: {e}"));

    let (store2, geo2, report) = import(&text);
    assert!(
        !report.has_errors(),
        "{context}: reader reported errors: {:?}",
        report.diagnostics
    );
    let body2 = only_brep(&report);
    assert!(
        store2.check(body2).is_empty(),
        "{context}: re-imported body must pass check: {:?}",
        store2.check(body2)
    );
    assert_counts_equal(store, body, &store2, body2, context);

    if let (Some(v1), Some(v2)) = (closed_volume(store, geo, body), {
        closed_volume(&store2, &geo2, body2)
    }) {
        assert!(v1 > 0.0, "{context}: original volume must be positive");
        let drift = (v1 - v2).abs() / v1.max(1.0);
        assert!(
            drift <= 1e-9,
            "{context}: volume drift {drift:e} exceeds 1e-9 ({v1} vs {v2})"
        );
    }

    let text2 = write_step(&store2, &geo2, &[body2], &StepWriteOptions::default())
        .unwrap_or_else(|e| panic!("{context}: re-imported body must serialize: {e}"));
    match fixed_point {
        FixedPoint::Immediate => assert_eq!(
            text, text2,
            "{context}: write ∘ read must be a fixed point (geometry or topology drifted)"
        ),
        FixedPoint::AfterOneTrip => {
            let (store3, geo3, report3) = import(&text2);
            let body3 = only_brep(&report3);
            let text3 = write_step(&store3, &geo3, &[body3], &StepWriteOptions::default())
                .unwrap_or_else(|e| panic!("{context}: third write must succeed: {e}"));
            assert_eq!(
                text2, text3,
                "{context}: write ∘ read must stabilize after one round trip \
                 (geometry or topology keeps drifting)"
            );
        }
    }
}

/// Round-trip gate for a boolean output (its own store/geo pair), with the
/// analytic volume cross-checked against `BooleanOutput::tessellate()`.
fn assert_boolean_round_trip(out: &BooleanOutput, expected_volume: f64, context: &str) {
    assert_boolean_round_trip_gate(out, expected_volume, context, FixedPoint::Immediate);
}

fn assert_boolean_round_trip_gate(
    out: &BooleanOutput,
    expected_volume: f64,
    context: &str,
    fixed_point: FixedPoint,
) {
    let failures = out.check();
    assert!(
        failures.is_empty(),
        "{context}: boolean output must pass check: {failures:?}"
    );
    let mesh = out
        .tessellate()
        .unwrap_or_else(|e| panic!("{context}: boolean output must tessellate: {e:?}"));
    assert!(
        mesh.is_closed_manifold(),
        "{context}: boolean tessellation must be a closed manifold"
    );
    let volume = mass_properties(&mesh)
        .unwrap_or_else(|e| panic!("{context}: mass_properties failed: {e}"))
        .volume;
    let rel = (volume - expected_volume).abs() / expected_volume;
    assert!(
        rel <= 5e-3,
        "{context}: boolean volume {volume} differs from analytic {expected_volume} by {rel:e}"
    );

    assert_round_trip_gate(&out.store, &out.geo, out.body, context, fixed_point);
}

// ---------------------------------------------------------------------
// 1a. Round trips: primitives (integration level, through the public API)
// ---------------------------------------------------------------------

#[test]
fn round_trip_every_primitive_in_one_file() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let bodies = [
        primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("block"),
        primitives::cylinder(&mut store, &mut geo, 1.5, 4.0).expect("cylinder"),
        primitives::sphere(&mut store, &mut geo, 2.0).expect("sphere"),
        primitives::torus(&mut store, &mut geo, 3.0, 1.0).expect("torus"),
    ];
    let text = write_step(&store, &geo, &bodies, &StepWriteOptions::default())
        .expect("all primitives must serialize into one file");

    let (store2, geo2, report) = import(&text);
    assert!(!report.has_errors(), "{:?}", report.diagnostics);
    assert_eq!(report.solids.len(), 4, "one MANIFOLD_SOLID_BREP per body");
    let mut bodies2 = Vec::new();
    for (original, solid) in bodies.iter().zip(&report.solids) {
        let SolidOutcome::BRep(body2) = &solid.outcome else {
            panic!("solid #{} did not re-import exactly", solid.step_id);
        };
        assert_counts_equal(&store, *original, &store2, *body2, "multi-solid");
        let v1 = closed_volume(&store, &geo, *original).expect("primitive volume");
        let v2 = closed_volume(&store2, &geo2, *body2).expect("re-imported volume");
        assert!(
            ((v1 - v2) / v1).abs() <= 1e-9,
            "volume drift for solid #{}: {v1} vs {v2}",
            solid.step_id
        );
        bodies2.push(*body2);
    }

    let text2 = write_step(&store2, &geo2, &bodies2, &StepWriteOptions::default())
        .expect("re-imported bodies must serialize");
    assert_eq!(text, text2, "write ∘ read must be a fixed point");
}

#[test]
fn round_trip_translated_primitives_off_origin() {
    // Placement (not just shape) must survive: same primitives, pushed far
    // from the origin in all three axes.
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let block = primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("block");
    translate_body(
        &mut store,
        &mut geo,
        block,
        Vector3::new(107.5, -33.25, 9.125),
    )
    .expect("translate block");
    assert_round_trip(&store, &geo, block, "translated block");

    let cyl = primitives::cylinder(&mut store, &mut geo, 1.5, 4.0).expect("cylinder");
    translate_body(&mut store, &mut geo, cyl, Vector3::new(-250.0, 1.0e3, 0.5))
        .expect("translate cylinder");
    assert_round_trip(&store, &geo, cyl, "translated cylinder");

    let torus = primitives::torus(&mut store, &mut geo, 3.0, 1.0).expect("torus");
    translate_body(&mut store, &mut geo, torus, Vector3::new(0.0, 0.0, -77.7))
        .expect("translate torus");
    assert_round_trip(&store, &geo, torus, "translated torus");
}

#[test]
fn round_trip_large_but_finite_coordinates() {
    // 1e6-scale block: coordinates and volume (1e18) stay finite and must
    // survive exactly — fmt_real must not lose bits at this magnitude.
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = primitives::block(&mut store, &mut geo, 1.0e6, 1.0e6, 1.0e6).expect("block");
    translate_body(
        &mut store,
        &mut geo,
        body,
        Vector3::new(3.0e6, -2.0e6, 1.0e6),
    )
    .expect("translate");
    assert_round_trip(&store, &geo, body, "1e6-scale block");
}

// ---------------------------------------------------------------------
// 1b. Round trips: boolean outputs
// ---------------------------------------------------------------------

/// Two unit-overlap 2×2×2 blocks: A at the origin, B shifted by (1,1,1).
fn corner_blocks() -> (TopologyStore, GeometryStore, EntityId<Body>, EntityId<Body>) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("block a");
    let b = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("block b");
    translate_body(&mut store, &mut geo, b, Vector3::new(1.0, 1.0, 1.0)).expect("translate b");
    (store, geo, a, b)
}

#[test]
fn round_trip_union_of_overlapping_blocks() {
    let (store, geo, a, b) = corner_blocks();
    let out = unite(&store, &geo, a, b, &tol()).expect("unite");
    // 8 + 8 − 1 (unit cube overlap)
    assert_boolean_round_trip(&out, 15.0, "block ∪ block");
}

#[test]
fn round_trip_intersection_of_blocks() {
    let (store, geo, a, b) = corner_blocks();
    let out = intersect(&store, &geo, a, b, &tol()).expect("intersect");
    assert_boolean_round_trip(&out, 1.0, "block ∩ block");
}

#[test]
fn round_trip_subtraction_l_shape() {
    let (store, geo, a, b) = corner_blocks();
    let out = subtract(&store, &geo, a, b, &tol()).expect("subtract");
    assert_boolean_round_trip(&out, 7.0, "block − block");
}

/// Block with a cylinder poking through both faces: the union splits the
/// cylinder band into two faces that SHARE one cylindrical surface, and
/// its two seam edges share one line — the case where the writer's
/// emit-once geometry sharing is actually exercised on import.
fn block_cylinder_union() -> BooleanOutput {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).expect("block");
    let b = primitives::cylinder(&mut store, &mut geo, 0.8, 4.0).expect("cylinder");
    unite(&store, &geo, a, b, &tol()).expect("unite")
}

#[test]
fn round_trip_union_of_block_and_cylinder() {
    let out = block_cylinder_union();
    // Block plus the two cylinder stubs protruding 1 above and below.
    let expected = 4.0 * 4.0 * 2.0 + std::f64::consts::PI * 0.8 * 0.8 * 2.0;
    // of-kb8: the shared band surface re-imports as two Surface3 instances,
    // so the byte-identical fixed point only holds from the second write.
    assert_boolean_round_trip_gate(&out, expected, "block ∪ cylinder", FixedPoint::AfterOneTrip);
}

/// of-kb8: the reader materializes one Curve3/Surface3 per referencing
/// edge/face instead of memoizing by STEP instance id, so a body whose
/// faces share a surface does not reproduce its own file on the first
/// re-write (the duplicate records appear; topology and volume are
/// unaffected). Un-ignore when the reader deduplicates shared geometry.
#[test]
#[ignore = "of-kb8: reader duplicates shared geometry instances"]
fn write_read_write_is_byte_identical_even_with_shared_geometry() {
    let out = block_cylinder_union();
    let expected = 4.0 * 4.0 * 2.0 + std::f64::consts::PI * 0.8 * 0.8 * 2.0;
    assert_boolean_round_trip(&out, expected, "block ∪ cylinder (strict)");
}

#[test]
fn round_trip_block_minus_cylinder_through_hole() {
    // Ring loops (faces with holes) are the hard part here: FACE_BOUND vs
    // FACE_OUTER_BOUND must survive, or genus/ring counts diverge.
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = primitives::block(&mut store, &mut geo, 4.0, 4.0, 2.0).expect("block");
    let b = primitives::cylinder(&mut store, &mut geo, 0.8, 4.0).expect("cylinder");
    let out = subtract(&store, &geo, a, b, &tol()).expect("subtract");
    let expected = 4.0 * 4.0 * 2.0 - std::f64::consts::PI * 0.8 * 0.8 * 2.0;
    assert_boolean_round_trip(&out, expected, "block − cylinder through-hole");
}

#[test]
fn round_trip_edge_notch() {
    // Cylinder centered on a vertical block edge: quarter-cylinder notch,
    // partial-wrap cylindrical band + notched planar loops (the of-ipt.8
    // configuration).
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let a = primitives::block(&mut store, &mut geo, 2.0, 2.0, 2.0).expect("block");
    let c = primitives::cylinder(&mut store, &mut geo, 0.4, 3.0).expect("cylinder");
    translate_body(&mut store, &mut geo, c, Vector3::new(1.0, 1.0, 0.0)).expect("translate");
    let out = subtract(&store, &geo, a, c, &tol()).expect("subtract");
    let expected = 8.0 - std::f64::consts::PI * 0.4 * 0.4 / 4.0 * 2.0;
    assert_boolean_round_trip(&out, expected, "edge notch");
}

// ---------------------------------------------------------------------
// 2. Synthetic adversarial files
// ---------------------------------------------------------------------
// The reader contract under attack: parse errors are Err(StepError),
// semantic problems are per-solid Failed outcomes plus diagnostics.
// Nothing here may panic, hang, or silently import wrong geometry.

mod adversarial {
    use super::*;

    #[test]
    fn syntactic_garbage_is_a_parse_error_not_a_panic() {
        for (name, source) in [
            ("empty", ""),
            ("not step at all", "solid STL\nfacet normal 0 0 1\n"),
            (
                "envelope only, no sections",
                "ISO-10303-21;END-ISO-10303-21;",
            ),
            (
                "truncated mid-instance",
                "ISO-10303-21;HEADER;ENDSEC;DATA;#1=CARTESIAN_POINT('',(",
            ),
            (
                "truncated mid-string",
                "ISO-10303-21;HEADER;ENDSEC;DATA;#1=CARTESIAN_POINT('unterminated",
            ),
            (
                "missing ENDSEC",
                "ISO-10303-21;HEADER;ENDSEC;DATA;#1=CARTESIAN_POINT('',(0.,0.,0.));END-ISO-10303-21;",
            ),
            (
                "binary junk",
                "ISO-10303-21;\u{0}\u{1}\u{2}\u{3}\u{4}garbage\u{7f}",
            ),
        ] {
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let result = read_step(source, &mut store, &mut geo, &StepReadOptions::default());
            assert!(result.is_err(), "{name}: expected a StepError");
        }
    }

    #[test]
    fn duplicate_instance_names_are_a_parse_error() {
        let source =
            envelope("#1 = CARTESIAN_POINT('',(0.,0.,0.));\n#1 = CARTESIAN_POINT('',(1.,0.,0.));");
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let result = read_step(&source, &mut store, &mut geo, &StepReadOptions::default());
        let err = result.expect_err("duplicate #1 must be rejected");
        assert!(
            err.message.contains("duplicate"),
            "unexpected message: {err}"
        );
    }

    #[test]
    fn solid_referencing_missing_shell_fails_with_diagnostics() {
        let source = envelope("#1 = MANIFOLD_SOLID_BREP('ghost', #999);");
        let (_, _, report) = import(&source);
        assert_eq!(report.solids.len(), 1);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert!(report.has_errors(), "missing shell must be an Error");
        assert_structured(&report);
    }

    #[test]
    fn face_referencing_missing_surface_fails_with_diagnostics() {
        let source = envelope(
            "#4 = ADVANCED_FACE('', (), #999, .T.);\n\
             #5 = CLOSED_SHELL('', (#4));\n\
             #6 = MANIFOLD_SOLID_BREP('holey', #5);",
        );
        let (_, _, report) = import(&source);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert!(!report.diagnostics.is_empty());
        assert_structured(&report);
    }

    #[test]
    fn shell_attribute_of_wrong_type_fails_with_diagnostics() {
        let source = envelope("#1 = MANIFOLD_SOLID_BREP('typed', 'not a ref');");
        let (_, _, report) = import(&source);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert!(report.has_errors());
        assert_structured(&report);
    }

    #[test]
    fn empty_closed_shell_does_not_import_as_a_valid_body() {
        let source =
            envelope("#5 = CLOSED_SHELL('', ());\n#6 = MANIFOLD_SOLID_BREP('hollow', #5);");
        let (store, _, report) = import(&source);
        match &report.solids[0].outcome {
            SolidOutcome::BRep(body) => panic!(
                "empty shell imported as a body ({:?}) — silently wrong geometry",
                store.euler_counts(*body)
            ),
            SolidOutcome::Mesh { mesh, .. } => panic!(
                "empty shell imported as a {}-triangle mesh",
                mesh.triangle_count()
            ),
            SolidOutcome::Failed => {}
        }
        assert_structured(&report);
    }

    #[test]
    fn cyclic_references_fail_without_hanging() {
        // The placement of the plane is the placement itself; the shell's
        // face list contains the shell. Resolution must terminate.
        let source = envelope(
            "#2 = AXIS2_PLACEMENT_3D('', #2, $, $);\n\
             #3 = PLANE('', #2);\n\
             #4 = ADVANCED_FACE('', (#5), #3, .T.);\n\
             #5 = CLOSED_SHELL('', (#5, #4));\n\
             #6 = MANIFOLD_SOLID_BREP('ouroboros', #5);",
        );
        let (_, _, report) = import(&source);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert!(!report.diagnostics.is_empty());
        assert_structured(&report);
    }

    #[test]
    fn mutually_recursive_edges_fail_without_hanging() {
        let source = envelope(
            "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #2 = VERTEX_POINT('', #1);\n\
             #10 = EDGE_CURVE('', #2, #2, #11, .T.);\n\
             #11 = EDGE_CURVE('', #2, #2, #10, .T.);\n\
             #12 = ORIENTED_EDGE('', *, *, #10, .T.);\n\
             #13 = EDGE_LOOP('', (#12));\n\
             #14 = FACE_OUTER_BOUND('', #13, .T.);\n\
             #15 = PLANE('', #16);\n\
             #16 = AXIS2_PLACEMENT_3D('', #1, $, $);\n\
             #17 = ADVANCED_FACE('', (#14), #15, .T.);\n\
             #18 = CLOSED_SHELL('', (#17));\n\
             #19 = MANIFOLD_SOLID_BREP('strange loop', #18);",
        );
        let (_, _, report) = import(&source);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert_structured(&report);
    }

    #[test]
    fn degenerate_geometry_fails_with_diagnostics() {
        // Zero-radius circle, zero-length direction, coincident edge
        // vertices: every geometry constructor must reject its input and
        // the reader must surface that, not build junk.
        let source = envelope(
            "#1 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #2 = DIRECTION('',(0.,0.,0.));\n\
             #3 = AXIS2_PLACEMENT_3D('', #1, #2, $);\n\
             #4 = CIRCLE('', #3, 0.0);\n\
             #5 = VERTEX_POINT('', #1);\n\
             #6 = EDGE_CURVE('', #5, #5, #4, .T.);\n\
             #7 = ORIENTED_EDGE('', *, *, #6, .T.);\n\
             #8 = EDGE_LOOP('', (#7));\n\
             #9 = FACE_OUTER_BOUND('', #8, .T.);\n\
             #10 = PLANE('', #3);\n\
             #11 = ADVANCED_FACE('', (#9), #10, .T.);\n\
             #12 = CLOSED_SHELL('', (#11));\n\
             #13 = MANIFOLD_SOLID_BREP('degenerate', #12);",
        );
        let (_, _, report) = import(&source);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert!(!report.diagnostics.is_empty());
        assert_structured(&report);
    }

    #[test]
    fn huge_coordinates_fail_cleanly_not_wrongly() {
        // 1e300 coordinates: any cross product or squared norm overflows
        // to inf. Import may succeed only if the geometry is genuinely
        // representable; otherwise it must be a structured failure.
        let source = envelope(
            "#1 = CARTESIAN_POINT('',(1.0E300,1.0E300,1.0E300));\n\
             #2 = DIRECTION('',(0.,0.,1.));\n\
             #3 = DIRECTION('',(1.,0.,0.));\n\
             #4 = AXIS2_PLACEMENT_3D('', #1, #2, #3);\n\
             #5 = PLANE('', #4);\n\
             #7 = ADVANCED_FACE('', (), #5, .T.);\n\
             #8 = CLOSED_SHELL('', (#7));\n\
             #9 = MANIFOLD_SOLID_BREP('huge', #8);",
        );
        let (_, _, report) = import(&source);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert_structured(&report);
    }

    #[test]
    fn overflowing_real_literals_parse_without_panicking() {
        // 1.0E999 exceeds f64 range. Whatever the policy (inf or error),
        // the pipeline must stay structured.
        let source = envelope("#1 = CARTESIAN_POINT('overflow',(1.0E999,-1.0E999,0.));");
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let result = read_step(&source, &mut store, &mut geo, &StepReadOptions::default());
        if let Ok(report) = result {
            assert_structured(&report);
        }
    }

    #[test]
    fn latin1_bytes_in_strings_parse_via_read_step_bytes() {
        // STEP files are ASCII/Latin-1; a 0xE9 ('é') in a name must not
        // break byte-level parsing.
        let source = envelope("#1 = CARTESIAN_POINT('caf\u{e9}',(0.,0.,0.));");
        let mut latin1: Vec<u8> = Vec::with_capacity(source.len());
        for ch in source.chars() {
            latin1.push(if (ch as u32) < 256 { ch as u8 } else { b'?' });
        }
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let report = read_step_bytes(&latin1, &mut store, &mut geo, &StepReadOptions::default())
            .expect("Latin-1 bytes must parse");
        assert_structured(&report);
    }

    #[test]
    fn moderately_nested_aggregates_parse() {
        // Real files nest 2–4 levels; 64 is far beyond any legitimate
        // writer while staying inside the parser's (currently unlimited,
        // see of-1dd) recursion budget.
        let depth = 64;
        let source = envelope(&format!(
            "#1 = THING('',{}0.{});",
            "(".repeat(depth),
            ")".repeat(depth)
        ));
        let file = opensolid_kernel::io::step::parse(&source).expect("depth-64 must parse");
        assert_eq!(file.len(), 1);
    }

    /// of-1dd (fixed): parse_value recursion used to have no depth limit;
    /// ~500 levels overflowed a 2MB test-thread stack and ABORTED the
    /// process. The parser now rejects absurd nesting with a structured
    /// [`StepError`] instead of recursing to death.
    #[test]
    fn deeply_nested_aggregates_must_not_crash_the_process() {
        let depth = 100_000;
        let source = envelope(&format!(
            "#1 = THING('',{}0.{});",
            "(".repeat(depth),
            ")".repeat(depth)
        ));
        let result = opensolid_kernel::io::step::parse(&source);
        assert!(
            result.is_err(),
            "absurd nesting should be rejected with a StepError, not accepted"
        );
    }

    /// of-83h (fixed): the reader resolves the GLOBAL_UNIT_ASSIGNED_CONTEXT
    /// length unit and scales coordinates into the kernel convention
    /// (millimetres), so the metre file's volume comes back 1e9 times the
    /// millimetre file's.
    #[test]
    fn declared_length_unit_should_scale_geometry() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("block");

        let volume_in = |text: &str| {
            let (store2, geo2, report) = import(text);
            closed_volume(&store2, &geo2, only_brep(&report)).expect("volume")
        };
        let mm = StepWriteOptions {
            length_unit: LengthUnit::Millimetre,
            ..Default::default()
        };
        let m = StepWriteOptions {
            length_unit: LengthUnit::Metre,
            ..Default::default()
        };
        let v_mm = volume_in(&write_step(&store, &geo, &[body], &mm).expect("write mm"));
        let v_m = volume_in(&write_step(&store, &geo, &[body], &m).expect("write m"));
        let ratio = v_m / v_mm;
        assert!(
            (ratio - 1.0e9).abs() / 1.0e9 <= 1e-6,
            "a metre-unit file must import 1e9× the volume of the same part \
             declared in millimetres; got ratio {ratio:e} (units ignored?)"
        );
    }
}

// ---------------------------------------------------------------------
// 3. Vendored real-world corpus (tests/data/step/, see README.md there)
// ---------------------------------------------------------------------

mod corpus {
    use super::*;

    fn load(name: &str) -> Vec<u8> {
        let path = format!("{}/tests/data/step/{name}", env!("CARGO_MANIFEST_DIR"));
        std::fs::read(&path).unwrap_or_else(|e| panic!("cannot read {path}: {e}"))
    }

    fn import_bytes(bytes: &[u8]) -> (TopologyStore, GeometryStore, StepImport) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let report = read_step_bytes(bytes, &mut store, &mut geo, &StepReadOptions::default())
            .expect("vendored file must parse");
        (store, geo, report)
    }

    /// Every solid must land in one of the three structured outcomes;
    /// exact imports must pass the checker and mesh fallbacks must be
    /// closed manifolds. Returns the exact bodies.
    fn assert_all_outcomes_structured(
        store: &TopologyStore,
        report: &StepImport,
    ) -> Vec<EntityId<Body>> {
        assert_structured(report);
        let mut breps = Vec::new();
        for solid in &report.solids {
            match &solid.outcome {
                SolidOutcome::BRep(body) => {
                    assert!(
                        store.check(*body).is_empty(),
                        "solid #{} imported exactly but fails check: {:?}",
                        solid.step_id,
                        store.check(*body)
                    );
                    breps.push(*body);
                }
                SolidOutcome::Mesh { mesh, .. } => {
                    assert!(
                        mesh.is_closed_manifold(),
                        "solid #{}: fallback mesh is not a closed manifold",
                        solid.step_id
                    );
                }
                SolidOutcome::Failed => {}
            }
        }
        breps
    }

    /// A CATIA-authored exact-import part: one solid, all-analytic
    /// geometry, no diagnostics at all — and it must survive our own
    /// write → read round trip with identical topology.
    fn assert_exact_single_solid_and_round_trips(name: &str) {
        let (store, geo, report) = import_bytes(&load(name));
        assert_eq!(report.solids.len(), 1, "{name}: expected one solid");
        assert!(
            report.diagnostics.is_empty(),
            "{name}: expected a clean exact import, got: {:?}",
            report.diagnostics
        );
        let breps = assert_all_outcomes_structured(&store, &report);
        assert_eq!(breps.len(), 1, "{name}: expected an exact B-Rep import");
        let body = breps[0];

        let counts = store.euler_counts(body);
        assert!(counts.faces >= 6, "{name}: implausibly few faces");
        assert_eq!(counts.shells, 1, "{name}: expected a single shell");

        assert_round_trip(&store, &geo, body, name);
    }

    #[test]
    fn sg1_c5_analytic_part_imports_exactly_and_round_trips() {
        // Planes + cylinders + one cone.
        assert_exact_single_solid_and_round_trips("sg1-c5-214.stp");
    }

    #[test]
    fn io1_cm_analytic_part_imports_exactly_and_round_trips() {
        // Planes + cylinders + one torus.
        assert_exact_single_solid_and_round_trips("io1-cm-214.stp");
    }

    #[test]
    fn dm1_id_nurbs_part_degrades_to_structured_diagnostics() {
        // Three solids carrying B-spline surfaces (including complex
        // instances and QUASI_UNIFORM_SURFACE) the kernel cannot represent
        // yet. Whatever the per-solid outcome, it must be structured —
        // today all three fail with unsupported-surface diagnostics; if
        // NURBS support lands they must import as valid B-Reps instead.
        let bytes = load("dm1-id-214.stp");
        let (store, _, report) = import_bytes(&bytes);
        assert_eq!(report.solids.len(), 3, "expected three solids");
        assert_all_outcomes_structured(&store, &report);
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.severity == Severity::Warning && d.message.contains("unsupported")),
            "expected unsupported-geometry warnings, got: {:?}",
            report.diagnostics.iter().take(5).collect::<Vec<_>>()
        );
    }
}
