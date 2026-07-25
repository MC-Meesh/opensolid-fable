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
/// (of-2i3 handled the iso-rectangular cylinder/cone case; of-fc8 the
/// planar-faces-with-holes case, so drilled parts now measure).
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

    let (m1, m2) = (
        closed_volume(store, geo, body),
        closed_volume(&store2, &geo2, body2),
    );
    // Whether a body can be measured at all must survive the round trip. The
    // deferred CDT cases make *both* sides unmeasurable and are skipped below,
    // but a body that measures going in and not coming out (or the reverse) is
    // a round-trip defect the volume comparison would otherwise swallow by
    // silently not running (of-fc8).
    assert_eq!(
        m1.is_some(),
        m2.is_some(),
        "{context}: tessellability must survive the round trip \
         (original {m1:?}, re-imported {m2:?})"
    );
    if let (Some(v1), Some(v2)) = (m1, m2) {
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

    // -----------------------------------------------------------------
    // Malformed product structure (of-3qy.13)
    // -----------------------------------------------------------------
    // Broken assembly wiring must degrade the *placement* only: the solids
    // still import, still get an occurrence each, and the reader says so.

    /// A one-solid file whose root representation holds `items`, with the
    /// given extra records appended.
    fn assembly_probe_items(items: &str, extra: &str) -> StepImport {
        let (_, _, report) = import(&envelope(&format!(
            "#1 = MANIFOLD_SOLID_BREP('part',#2);\n\
             #10 = PRODUCT_DEFINITION('design','',#11,#0);\n\
             #11 = PRODUCT_DEFINITION_FORMATION('','',#12);\n\
             #12 = PRODUCT('root','root','',());\n\
             #13 = PRODUCT_DEFINITION_SHAPE('','',#10);\n\
             #14 = ADVANCED_BREP_SHAPE_REPRESENTATION('',({items}),#0);\n\
             #15 = SHAPE_DEFINITION_REPRESENTATION(#13,#14);\n\
             {extra}"
        )));
        report
    }

    /// [`assembly_probe_items`] with the solid as the only root item.
    fn assembly_probe(extra: &str) -> StepImport {
        assembly_probe_items("#1", extra)
    }

    #[test]
    fn nauo_naming_a_missing_product_does_not_lose_the_solid() {
        let report =
            assembly_probe("#20 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','ghost','',#10,#999,$);\n");
        assert_eq!(report.solids.len(), 1);
        assert_eq!(report.instances.len(), 1, "the real solid still places");
        assert_structured(&report);
    }

    #[test]
    fn dangling_placement_reference_degrades_to_a_diagnostic() {
        // The CDSR's ITEM_DEFINED_TRANSFORMATION points at an axis that is
        // not in the file at all.
        let report = assembly_probe(
            "#20 = PRODUCT_DEFINITION('design','',#21,#0);\n\
             #21 = PRODUCT_DEFINITION_FORMATION('','',#22);\n\
             #22 = PRODUCT('child','child','',());\n\
             #23 = PRODUCT_DEFINITION_SHAPE('','',#20);\n\
             #24 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(),#0);\n\
             #25 = SHAPE_DEFINITION_REPRESENTATION(#23,#24);\n\
             #30 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','child_1','',#10,#20,$);\n\
             #31 = PRODUCT_DEFINITION_SHAPE('Placement','',#30);\n\
             #32 = ITEM_DEFINED_TRANSFORMATION('','',#998,#999);\n\
             #33 = ( REPRESENTATION_RELATIONSHIP('','',#24,#14) \
             REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#32) \
             SHAPE_REPRESENTATION_RELATIONSHIP() );\n\
             #34 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#33,#31);\n",
        );
        assert_eq!(report.instances.len(), 1);
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.severity >= Severity::Warning && d.message.contains("dangling")),
            "expected a dangling-reference diagnostic, got {:?}",
            report.diagnostics
        );
        assert_structured(&report);
    }

    #[test]
    fn a_representation_referring_to_itself_terminates() {
        // A MAPPED_ITEM whose REPRESENTATION_MAP maps the very
        // representation the item belongs to. Without a guard this is an
        // infinite descent.
        let report = assembly_probe_items(
            "#1,#25",
            "#20 = REPRESENTATION_MAP(#24,#14);\n\
             #21 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #22 = DIRECTION('',(0.,0.,1.));\n\
             #23 = DIRECTION('',(1.,0.,0.));\n\
             #24 = AXIS2_PLACEMENT_3D('',#21,#22,#23);\n\
             #25 = MAPPED_ITEM('self',#20,#24);\n",
        );
        // The solid is reachable; how many times is unimportant, that the
        // call returns at all is the gate.
        assert!(!report.instances.is_empty());
        assert_structured(&report);
    }
}

// ---------------------------------------------------------------------
// 2b. Healable adversarial files (of-3qy.12)
//
// The defects real exporters emit that are *repairable*: shells whose faces
// never shared a boundary, corners that disagree at the last written decimal,
// face uses authored backwards. The healer must promote these to exact
// B-Reps — and, just as importantly, must refuse to "repair" a defect that is
// really a modelling error, degrading to the structured fallback instead of
// inventing geometry.
// ---------------------------------------------------------------------

mod healing {
    use super::*;
    use opensolid_kernel::io::step::heal::{HealOptions, HealStrategy};
    use std::fmt::Write as _;

    /// Corner-cycles of a unit tetrahedron, counterclockwise from outside.
    const TET: [[usize; 3]; 4] = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
    const TET_CORNERS: [[f64; 3]; 4] = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
    ];

    /// An AP203 tetrahedron whose four faces never share a boundary: each
    /// authors its own three `VERTEX_POINT`s and `EDGE_CURVE`s, so the
    /// mapped shell has twelve one-fin edges instead of six two-fin ones.
    ///
    /// `jitter` displaces each face's private copy of a shared corner along a
    /// per-face direction; `reversed` lists faces whose whole use (surface
    /// sense *and* loop traversal) is authored backwards.
    fn unsewn_tetrahedron(jitter: f64, reversed: &[usize]) -> String {
        let mut b = String::new();
        let mut next = 1u64;
        let mut id = || {
            next += 1;
            next - 1
        };
        let mut faces = Vec::new();

        for (f, cycle) in TET.iter().enumerate() {
            let k = (f + 1) as f64;
            let raw = [(k * 0.7).sin(), (k * 1.3).sin(), (k * 2.1).sin()];
            let len = raw.iter().map(|c| c * c).sum::<f64>().sqrt();
            let nudge = raw.map(|c| c / len * jitter);
            let p: Vec<[f64; 3]> = cycle
                .iter()
                .map(|&c| {
                    let corner = TET_CORNERS[c];
                    [
                        corner[0] + nudge[0],
                        corner[1] + nudge[1],
                        corner[2] + nudge[2],
                    ]
                })
                .collect();

            // Outward normal of a counterclockwise-from-outside cycle.
            let (u, v) = (
                [p[1][0] - p[0][0], p[1][1] - p[0][1], p[1][2] - p[0][2]],
                [p[2][0] - p[0][0], p[2][1] - p[0][1], p[2][2] - p[0][2]],
            );
            let n = [
                u[1] * v[2] - u[2] * v[1],
                u[2] * v[0] - u[0] * v[2],
                u[0] * v[1] - u[1] * v[0],
            ];
            let n_len = n.iter().map(|c| c * c).sum::<f64>().sqrt();
            let n = n.map(|c| c / n_len);

            let points: Vec<u64> = p
                .iter()
                .map(|q| {
                    let pid = id();
                    writeln!(
                        b,
                        "#{pid} = CARTESIAN_POINT('', ({:.9}, {:.9}, {:.9}));",
                        q[0], q[1], q[2]
                    )
                    .unwrap();
                    pid
                })
                .collect();
            let vertices: Vec<u64> = points
                .iter()
                .map(|&pid| {
                    let vid = id();
                    writeln!(b, "#{vid} = VERTEX_POINT('', #{pid});").unwrap();
                    vid
                })
                .collect();

            let mut edges = Vec::new();
            for k in 0..3 {
                let (a, c) = (k, (k + 1) % 3);
                let d = [p[c][0] - p[a][0], p[c][1] - p[a][1], p[c][2] - p[a][2]];
                let dir = id();
                writeln!(
                    b,
                    "#{dir} = DIRECTION('', ({:.9}, {:.9}, {:.9}));",
                    d[0], d[1], d[2]
                )
                .unwrap();
                let vec = id();
                writeln!(b, "#{vec} = VECTOR('', #{dir}, 1.);").unwrap();
                let line = id();
                writeln!(b, "#{line} = LINE('', #{}, #{vec});", points[a]).unwrap();
                let edge = id();
                writeln!(
                    b,
                    "#{edge} = EDGE_CURVE('', #{}, #{}, #{line}, .T.);",
                    vertices[a], vertices[c]
                )
                .unwrap();
                edges.push(edge);
            }

            let normal = id();
            writeln!(
                b,
                "#{normal} = DIRECTION('', ({:.9}, {:.9}, {:.9}));",
                n[0], n[1], n[2]
            )
            .unwrap();
            let placement = id();
            writeln!(
                b,
                "#{placement} = AXIS2_PLACEMENT_3D('', #{}, #{normal}, $);",
                points[0]
            )
            .unwrap();
            let plane = id();
            writeln!(b, "#{plane} = PLANE('', #{placement});").unwrap();

            let backwards = reversed.contains(&f);
            let flag = if backwards { ".F." } else { ".T." };
            let mut oriented: Vec<u64> = edges
                .iter()
                .map(|&edge| {
                    let oe = id();
                    writeln!(b, "#{oe} = ORIENTED_EDGE('', *, *, #{edge}, {flag});").unwrap();
                    oe
                })
                .collect();
            if backwards {
                oriented.reverse();
            }
            let edge_loop = id();
            writeln!(
                b,
                "#{edge_loop} = EDGE_LOOP('', (#{}, #{}, #{}));",
                oriented[0], oriented[1], oriented[2]
            )
            .unwrap();
            let bound = id();
            writeln!(b, "#{bound} = FACE_OUTER_BOUND('', #{edge_loop}, .T.);").unwrap();
            let face = id();
            writeln!(
                b,
                "#{face} = ADVANCED_FACE('', (#{bound}), #{plane}, {flag});"
            )
            .unwrap();
            faces.push(face);
        }

        let shell = id();
        let refs: Vec<String> = faces.iter().map(|f| format!("#{f}")).collect();
        writeln!(b, "#{shell} = CLOSED_SHELL('', ({}));", refs.join(", ")).unwrap();
        let solid = id();
        writeln!(b, "#{solid} = MANIFOLD_SOLID_BREP('tet', #{shell});").unwrap();
        envelope(&b)
    }

    fn import_unhealed(source: &str) -> (TopologyStore, GeometryStore, StepImport) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let options = StepReadOptions {
            heal: HealOptions {
                strategy: HealStrategy::Off,
                ..HealOptions::default()
            },
            ..StepReadOptions::default()
        };
        let report =
            read_step(source, &mut store, &mut geo, &options).expect("adversarial file parses");
        (store, geo, report)
    }

    /// The baseline the healer exists to move: unhealed, an unsewn shell
    /// cannot import exactly.
    #[test]
    fn unsewn_shell_cannot_import_exactly_without_healing() {
        let (_store, _geo, report) = import_unhealed(&unsewn_tetrahedron(0.0, &[]));
        assert_structured(&report);
        assert!(
            !matches!(report.solids[0].outcome, SolidOutcome::BRep(_)),
            "an unsewn shell has one fin per edge; it must not pass check"
        );
        assert_eq!(report.heal_operations, 0);
    }

    /// Healed, the same file imports exactly, passes the checker, and
    /// survives the write → read round trip like any other exact body.
    #[test]
    fn unsewn_shell_heals_into_an_exact_brep_and_round_trips() {
        let (store, geo, report) = import(&unsewn_tetrahedron(0.0, &[]));
        assert_structured(&report);
        let body = only_brep(&report);
        let counts = store.euler_counts(body);
        assert_eq!(
            (counts.vertices, counts.edges, counts.faces),
            (4, 6, 4),
            "12 private corners sew to 4, 12 half-edges weld to 6"
        );
        assert!(report.heal_operations > 0);
        // The tetrahedron's 45° edge directions renormalize by one ULP on
        // first import (a `dir/|dir|` round trip through a decimal literal,
        // nothing to do with healing), so the fixed point arrives one trip
        // out exactly as of-kb8's shared-geometry cases do.
        assert_round_trip_gate(
            &store,
            &geo,
            body,
            "healed unsewn tetrahedron",
            FixedPoint::AfterOneTrip,
        );
    }

    /// Signed volume by the divergence theorem, straight off the
    /// tessellation. Unlike [`closed_volume`] this does not require the mesh
    /// to weld watertight — a *tolerant* healed body (one whose curves still
    /// pass through the pre-merge points) tessellates to rim samples that
    /// disagree by the closed gap, which is far above the tessellator's
    /// exact weld epsilon. Tolerance-aware welding is of-61f.
    fn mesh_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> f64 {
        let mesh = tessellate_body(store, geo, body, &TessellationOptions::default())
            .expect("healed body tessellates");
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    /// Gaps at the last written decimal, plus two faces authored backwards:
    /// both passes together, still exact, still the right way out.
    #[test]
    fn gapped_and_misoriented_shell_heals_completely() {
        let (store, geo, report) = import(&unsewn_tetrahedron(1e-7, &[1, 2]));
        assert_structured(&report);
        let body = only_brep(&report);
        let counts = store.euler_counts(body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (4, 6, 4));
        let volume = mesh_volume(&store, &geo, body);
        assert!(
            (volume - 1.0 / 6.0).abs() < 1e-6,
            "outward, not inside out: got {volume}"
        );
    }

    /// A file that is consistently oriented but wholly inside out passes
    /// every combinatorial check there is. Only the geometric volume-sign
    /// pass can catch it, and it must — importing an inverted solid is
    /// exactly the "silently wrong geometry" this suite exists to forbid.
    #[test]
    fn wholly_inverted_shell_is_uprighted_not_imported_inside_out() {
        let (store, geo, report) = import(&unsewn_tetrahedron(0.0, &[0, 1, 2, 3]));
        assert_structured(&report);
        let body = only_brep(&report);
        let volume = closed_volume(&store, &geo, body).expect("healed tetrahedron tessellates");
        assert!(
            (volume - 1.0 / 6.0).abs() < 1e-9,
            "an inverted import must be righted, not accepted: got {volume}"
        );
    }

    /// The refusal case. A tenth of a millimetre on a 1 mm part is a
    /// modelling error, not export round-off: healing must leave it alone and
    /// let the import degrade, never weld a hole shut and call it exact.
    #[test]
    fn a_gap_too_wide_to_be_round_off_is_never_welded() {
        let (store, _geo, report) = import(&unsewn_tetrahedron(0.1, &[]));
        assert_structured(&report);
        match &report.solids[0].outcome {
            SolidOutcome::BRep(body) => panic!(
                "a 0.1 mm gap was welded shut and imported as exact: {:?}",
                store.euler_counts(*body)
            ),
            SolidOutcome::Mesh { mesh, .. } => {
                assert!(mesh.is_closed_manifold(), "fallback must still be closed")
            }
            SolidOutcome::Failed => {}
        }
    }

    /// Healing past [`MAX_ALLOWED_TOLERANCE`] would produce a body the kernel
    /// rejects for tolerance alone, so an over-wide `max_gap` is clamped
    /// rather than honoured.
    #[test]
    fn an_over_wide_max_gap_is_clamped_not_honoured() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let options = StepReadOptions {
            heal: HealOptions {
                strategy: HealStrategy::Auto,
                max_gap: Some(1.0),
            },
            ..StepReadOptions::default()
        };
        let report = read_step(
            &unsewn_tetrahedron(0.1, &[]),
            &mut store,
            &mut geo,
            &options,
        )
        .expect("adversarial file parses");
        assert_structured(&report);
        assert!(
            !matches!(report.solids[0].outcome, SolidOutcome::BRep(_)),
            "a 1 mm merge tolerance must be clamped to the kernel limit"
        );
    }

    /// Every repair is attributable: one `Info` diagnostic per operation,
    /// each naming the solid it belongs to.
    #[test]
    fn every_repair_is_reported_per_entity() {
        let (_store, _geo, report) = import(&unsewn_tetrahedron(1e-7, &[3]));
        assert_structured(&report);
        let healed: Vec<&opensolid_kernel::io::step::read::Diagnostic> = report
            .diagnostics
            .iter()
            .filter(|d| d.message.starts_with("healed:"))
            .collect();
        assert_eq!(healed.len(), report.heal_operations);
        for diagnostic in healed {
            assert_eq!(diagnostic.severity, Severity::Info);
            assert!(
                diagnostic.entity.is_some(),
                "a repair must name the solid it belongs to"
            );
        }
    }

    /// `ReportOnly` is a dry run: it says what it would fix and the body is
    /// left exactly as authored.
    #[test]
    fn report_only_never_promotes_a_body() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let options = StepReadOptions {
            heal: HealOptions {
                strategy: HealStrategy::ReportOnly,
                ..HealOptions::default()
            },
            ..StepReadOptions::default()
        };
        let report = read_step(
            &unsewn_tetrahedron(0.0, &[]),
            &mut store,
            &mut geo,
            &options,
        )
        .expect("adversarial file parses");
        assert_structured(&report);
        assert!(!matches!(report.solids[0].outcome, SolidOutcome::BRep(_)));
        assert_eq!(
            report.heal_operations, 10,
            "4 vertex merges + 6 edge welds, planned but not applied"
        );
    }

    /// Files that were already valid must never acquire a repair — healing
    /// runs only for bodies the checker rejected.
    #[test]
    fn well_formed_files_are_never_healed() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("block");
        let text = write_step(&store, &geo, &[body], &StepWriteOptions::default()).expect("write");
        let (_, _, report) = import(&text);
        assert_eq!(report.heal_operations, 0);
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

    /// Every vendored corpus file — including the NIST tree — must parse and
    /// land every solid in a structured outcome. Files the reader cannot
    /// import yet must fail with diagnostics, never with a panic, a hang, or
    /// silently wrong geometry. This is the test that makes growing the
    /// corpus cheap: drop a file under `tests/data/step/` and it is covered.
    #[test]
    fn every_vendored_file_imports_structurally() {
        let files = corpus_files();
        assert!(
            files.len() >= 17,
            "corpus shrank? found only {} files",
            files.len()
        );
        for file in &files {
            let bytes = std::fs::read(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
            let (store, _, report) = import_bytes(&bytes);
            let name = file.file_name().unwrap_or_default().to_string_lossy();
            assert!(
                !report.solids.is_empty(),
                "{name}: no MANIFOLD_SOLID_BREP found — wrong file vendored?"
            );
            assert_all_outcomes_structured(&store, &report);
        }
    }

    /// The corpus-wide pass-rate floor (spec/06-step-io.md §Pass-rate
    /// targets). A file passes when every solid in it imports (exactly or as
    /// a closed-manifold mesh fallback). The floor pins the current figure so
    /// a reader regression fails loudly; raise it as import coverage grows.
    /// The full per-file report: `cargo run --release --example
    /// step_import_report -- crates/opensolid-kernel/tests/data/step`.
    #[test]
    fn corpus_pass_rate_does_not_regress() {
        let files = corpus_files();
        let mut passed = Vec::new();
        for file in &files {
            let bytes = std::fs::read(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
            let (_, _, report) = import_bytes(&bytes);
            let ok = !report.solids.is_empty()
                && report
                    .solids
                    .iter()
                    .all(|s| !matches!(s.outcome, SolidOutcome::Failed));
            if ok {
                passed.push(
                    file.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned(),
                );
            }
        }
        // 2026-07-25 baseline: sg1, io1, nist_ctc_03 (both editions),
        // nist_ftc_09, nist_ftc_11 — 6 of 17.
        const FLOOR: usize = 6;
        assert!(
            passed.len() >= FLOOR,
            "corpus pass count regressed below {FLOOR}: only {passed:?} pass"
        );
    }

    fn corpus_files() -> Vec<std::path::PathBuf> {
        let root = format!("{}/tests/data/step", env!("CARGO_MANIFEST_DIR"));
        let mut files = Vec::new();
        let mut stack = vec![std::path::PathBuf::from(root)];
        while let Some(dir) = stack.pop() {
            for entry in std::fs::read_dir(&dir).unwrap_or_else(|e| panic!("read {dir:?}: {e}")) {
                let path = entry.expect("dir entry").path();
                if path.is_dir() {
                    stack.push(path);
                } else if path
                    .extension()
                    .and_then(|e| e.to_str())
                    .is_some_and(|e| e.eq_ignore_ascii_case("stp"))
                {
                    files.push(path);
                }
            }
        }
        files.sort();
        files
    }

    /// NIST parts that import exactly today must keep doing so, and must
    /// survive our write → read round trip. Unlike the CATIA files these
    /// carry Info diagnostics (unit scaling), so only Warning+ is rejected.
    /// Parts whose faces share surfaces re-import with duplicated geometry
    /// (of-kb8), so their byte-identical fixed point arrives one write late.
    fn assert_nist_exact_and_round_trips_gate(name: &str, fixed_point: FixedPoint) {
        let (store, geo, report) = import_bytes(&load(name));
        assert_eq!(report.solids.len(), 1, "{name}: expected one solid");
        assert!(
            report
                .diagnostics
                .iter()
                .all(|d| d.severity < Severity::Warning),
            "{name}: expected no Warning/Error diagnostics, got: {:?}",
            report.diagnostics
        );
        let breps = assert_all_outcomes_structured(&store, &report);
        assert_eq!(breps.len(), 1, "{name}: expected an exact B-Rep import");
        assert_round_trip_gate(&store, &geo, breps[0], name, fixed_point);
    }

    #[test]
    fn nist_ctc_03_imports_exactly_and_round_trips() {
        assert_nist_exact_and_round_trips_gate(
            "nist/nist_ctc_03_asme1_rc.stp",
            FixedPoint::AfterOneTrip,
        );
    }

    #[test]
    fn nist_ftc_09_imports_exactly_and_round_trips() {
        assert_nist_exact_and_round_trips_gate(
            "nist/nist_ftc_09_asme1_rd.stp",
            FixedPoint::AfterOneTrip,
        );
    }

    #[test]
    fn nist_ftc_11_imports_exactly_and_round_trips() {
        assert_nist_exact_and_round_trips_gate(
            "nist/nist_ftc_11_asme1_rb.stp",
            FixedPoint::Immediate,
        );
    }

    #[test]
    fn nist_ctc_03_ap242_imports_exactly_and_round_trips() {
        assert_nist_exact_and_round_trips_gate(
            "nist/nist_ctc_03_asme1_ap242-e2.stp",
            FixedPoint::AfterOneTrip,
        );
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

    // -----------------------------------------------------------------
    // Assembly structure (of-3qy.13)
    // -----------------------------------------------------------------
    // Product structure is resolved independently of whether the geometry
    // imports, so these gates hold even while AS1's SURFACE_CURVE edges
    // and DM1's NURBS surfaces still defeat the geometry mapper.

    /// Every solid a file declares must be accounted for by at least one
    /// placed occurrence, and every placement must be a finite rigid
    /// transform. This is the invariant that makes `instances` safe to
    /// consume without cross-checking `solids`.
    fn assert_instances_cover_every_solid(report: &StepImport, name: &str) {
        for (index, solid) in report.solids.iter().enumerate() {
            assert!(
                report.instances_of(index).next().is_some(),
                "{name}: solid #{} has no placed occurrence",
                solid.step_id
            );
        }
        for instance in &report.instances {
            assert!(
                instance.solid < report.solids.len(),
                "{name}: instance indexes solid {} of {}",
                instance.solid,
                report.solids.len()
            );
            assert_eq!(
                report.solids[instance.solid].step_id, instance.step_id,
                "{name}: instance's solid index and step id disagree"
            );
            let t = instance.transform;
            assert!(
                t.translation.vector.iter().all(|c| c.is_finite())
                    && t.rotation.quaternion().coords.iter().all(|c| c.is_finite()),
                "{name}: non-finite placement {t}"
            );
            // Rigid: the rotation must preserve lengths and handedness.
            let matrix = t.rotation.to_rotation_matrix();
            assert!(
                (matrix.matrix().determinant() - 1.0).abs() < 1e-9,
                "{name}: placement rotation is not a proper rotation"
            );
        }
    }

    /// AS1 is the canonical CAx-IF assembly: a plate, two L-bracket
    /// sub-assemblies of three nut-bolt sub-assemblies each, and a rod
    /// sub-assembly holding a rod and two nuts — five distinct parts in
    /// eighteen places, nested three deep.
    #[test]
    fn as1_oc_resolves_the_full_assembly_tree() {
        let (_, _, report) = import_bytes(&load("as1-oc-214.stp"));
        assert_eq!(report.solids.len(), 5, "five distinct parts");
        assert_eq!(report.instances.len(), 18, "eighteen placed occurrences");
        assert!(report.is_assembly());
        assert_instances_cover_every_solid(&report, "as1-oc-214.stp");

        // Occurrence counts per part, by product name.
        let mut counts: std::collections::BTreeMap<&str, usize> = Default::default();
        for instance in &report.instances {
            *counts.entry(instance.product.as_str()).or_default() += 1;
        }
        assert_eq!(counts["nut"], 8, "two on the rod, six in the brackets");
        assert_eq!(counts["bolt"], 6);
        assert_eq!(counts["l-bracket"], 2);
        assert_eq!(counts["rod"], 1);
        assert_eq!(counts["plate"], 1);

        // Nesting depth: the plate hangs off the root, the bracket nuts are
        // three levels down.
        let depths: std::collections::BTreeSet<usize> =
            report.instances.iter().map(|i| i.path.len()).collect();
        assert_eq!(
            depths,
            std::collections::BTreeSet::from([1, 2, 3]),
            "one-, two- and three-deep occurrences"
        );

        // The composed pose of the first nut on the rod, computed by hand
        // from the file: the rod-assembly is placed by #15 (origin
        // (-10,75,60), z along +X, x along -Z) and the nut by #45 (origin
        // (-10,-7.5,185), unrotated), so the nut lands at
        //     R(#15) · (-10,-7.5,185) + (-10,75,60) = (175, 67.5, 70).
        let nut_1 = report
            .instances
            .iter()
            .find(|i| i.path == ["rod-assembly_1", "nut_1"])
            .expect("rod-assembly_1/nut_1");
        let origin = nut_1.transform * opensolid_kernel::core::Point3::origin();
        assert!(
            (origin - opensolid_kernel::core::Point3::new(175.0, 67.5, 70.0)).norm() < 1e-9,
            "nut_1 landed at {origin:?}"
        );
        // Its frame is the rod-assembly's quarter turn.
        assert!(
            (nut_1.transform.rotation.angle() - std::f64::consts::FRAC_PI_2).abs() < 1e-9,
            "nut_1 rotation is {}",
            nut_1.transform.rotation.angle()
        );

        // No two occurrences of one part may coincide — that would mean a
        // placement was silently dropped to identity.
        let nuts: Vec<_> = report
            .instances
            .iter()
            .filter(|i| i.product == "nut")
            .map(|i| i.transform * opensolid_kernel::core::Point3::origin())
            .collect();
        for (i, a) in nuts.iter().enumerate() {
            for b in &nuts[i + 1..] {
                assert!((a - b).norm() > 1e-6, "two nuts coincide at {a:?}");
            }
        }
    }

    /// DM1 is a one-level assembly authored in inches: the placements must
    /// be scaled into millimetres alongside the geometry, or components
    /// land 25.4× too close to the origin.
    #[test]
    fn dm1_id_places_components_in_millimetres() {
        let (_, _, report) = import_bytes(&load("dm1-id-214.stp"));
        assert!((report.length_scale - 25.4).abs() < 1e-12, "an inch file");
        assert_eq!(report.solids.len(), 3);
        assert_eq!(
            report.instances.len(),
            7,
            "three bolts, three nuts, a bracket"
        );
        assert!(report.is_assembly());
        assert_instances_cover_every_solid(&report, "dm1-id-214.stp");
        assert!(
            report.instances.iter().all(|i| i.path.len() == 1),
            "DM1 is a single-level assembly"
        );

        // The bolts sit ~10mm and ~30mm out in X; unscaled they would be at
        // ~0.4mm and ~1.2mm.
        let mut bolt_x: Vec<f64> = report
            .instances
            .iter()
            .filter(|i| i.product == "bolt")
            .map(|i| i.transform.translation.vector.x)
            .collect();
        bolt_x.sort_by(f64::total_cmp);
        assert_eq!(bolt_x.len(), 3);
        assert!(
            bolt_x[0] > 5.0 && bolt_x[2] > 25.0,
            "bolt placements were not scaled into millimetres: {bolt_x:?}"
        );
    }

    /// A part file is not an assembly, however the reader models it: one
    /// identity occurrence per solid and nothing else.
    #[test]
    fn part_files_yield_one_identity_occurrence_per_solid() {
        for name in [
            "sg1-c5-214.stp",
            "io1-cm-214.stp",
            "nist/nist_ftc_11_asme1_rb.stp",
        ] {
            let (_, _, report) = import_bytes(&load(name));
            assert_eq!(
                report.instances.len(),
                report.solids.len(),
                "{name}: one occurrence per solid"
            );
            assert!(
                report.instances.iter().all(|i| i.is_identity()),
                "{name}: a part file must import at the origin"
            );
            assert!(!report.is_assembly(), "{name}: not an assembly");
        }
    }

    /// The corpus-wide structural invariant, so a new vendored file is
    /// covered the moment it is dropped in.
    #[test]
    fn every_vendored_file_accounts_for_every_solid() {
        for file in corpus_files() {
            let bytes = std::fs::read(&file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
            let (_, _, report) = import_bytes(&bytes);
            let name = file.file_name().unwrap_or_default().to_string_lossy();
            assert_instances_cover_every_solid(&report, &name);
        }
    }
}

// ---------------------------------------------------------------------
// 4. Schema breadth (of-3qy.9): swept surfaces, conic and composite curves
// ---------------------------------------------------------------------
// Entities real mechanical-CAD exporters emit beyond the quadric core.
// Conic (parabola/hyperbola) walls have no exact kernel form yet, so these
// solids must degrade to a *watertight* tessellated import with structured
// diagnostics — never Failed, never silently wrong.

mod schema_breadth {
    use super::*;
    use opensolid_kernel::core::mesh::TriangleMesh;

    /// Signed volume via the divergence theorem: positive iff the mesh is
    /// consistently outward-wound.
    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    /// A prism over the region between a conic arc and its chord, extruded
    /// one unit along +z. The curved wall is a SURFACE_OF_LINEAR_EXTRUSION
    /// of the TRIMMED conic; the caps are planes ear-clipped from the arc
    /// polyline. `(ax, ay)`: the arc endpoints (±ay at x = ax), which for
    /// both conics here correspond to parameter t = ±1.
    fn conic_prism(bottom: &str, top: &str, ax: f64, ay: f64) -> String {
        let nay = -ay;
        envelope(&format!(
            "\
#1 = CARTESIAN_POINT('', (0., 0., 0.));
#2 = CARTESIAN_POINT('', (0., 0., 1.));
#3 = CARTESIAN_POINT('', ({ax:.9}, {nay:.9}, 0.));
#4 = CARTESIAN_POINT('', ({ax:.9}, {ay:.9}, 0.));
#5 = CARTESIAN_POINT('', ({ax:.9}, {nay:.9}, 1.));
#6 = CARTESIAN_POINT('', ({ax:.9}, {ay:.9}, 1.));
#7 = DIRECTION('', (0., 0., 1.));
#8 = DIRECTION('', (0., 0., -1.));
#9 = DIRECTION('', (1., 0., 0.));
#10 = DIRECTION('', (0., 1., 0.));
#11 = VERTEX_POINT('', #3);
#12 = VERTEX_POINT('', #4);
#13 = VERTEX_POINT('', #5);
#14 = VERTEX_POINT('', #6);
#15 = AXIS2_PLACEMENT_3D('', #1, #7, #9);
#16 = AXIS2_PLACEMENT_3D('', #2, #7, #9);
#17 = {bottom};
#18 = {top};
#19 = VECTOR('', #7, 1.);
#20 = TRIMMED_CURVE('', #17, (#3), (#4), .T., .CARTESIAN.);
#21 = SURFACE_OF_LINEAR_EXTRUSION('', #20, #19);
#22 = LINE('', #3, #19);
#23 = LINE('', #4, #19);
#24 = VECTOR('', #10, 1.);
#25 = LINE('', #3, #24);
#26 = LINE('', #5, #24);
#27 = EDGE_CURVE('', #11, #12, #17, .T.);
#28 = EDGE_CURVE('', #13, #14, #18, .T.);
#29 = EDGE_CURVE('', #11, #13, #22, .T.);
#30 = EDGE_CURVE('', #12, #14, #23, .T.);
#31 = EDGE_CURVE('', #11, #12, #25, .T.);
#32 = EDGE_CURVE('', #13, #14, #26, .T.);
#33 = AXIS2_PLACEMENT_3D('', #1, #8, #9);
#34 = PLANE('', #33);
#35 = PLANE('', #16);
#36 = AXIS2_PLACEMENT_3D('', #3, #9, #7);
#37 = PLANE('', #36);
#39 = ORIENTED_EDGE('', *, *, #27, .T.);
#40 = ORIENTED_EDGE('', *, *, #31, .F.);
#41 = EDGE_LOOP('', (#39, #40));
#42 = FACE_OUTER_BOUND('', #41, .T.);
#43 = ADVANCED_FACE('', (#42), #34, .T.);
#44 = ORIENTED_EDGE('', *, *, #32, .T.);
#45 = ORIENTED_EDGE('', *, *, #28, .F.);
#46 = EDGE_LOOP('', (#44, #45));
#47 = FACE_OUTER_BOUND('', #46, .T.);
#48 = ADVANCED_FACE('', (#47), #35, .T.);
#49 = ORIENTED_EDGE('', *, *, #29, .T.);
#50 = ORIENTED_EDGE('', *, *, #28, .T.);
#51 = ORIENTED_EDGE('', *, *, #30, .F.);
#52 = ORIENTED_EDGE('', *, *, #27, .F.);
#53 = EDGE_LOOP('', (#49, #50, #51, #52));
#54 = FACE_OUTER_BOUND('', #53, .T.);
#55 = ADVANCED_FACE('', (#54), #21, .F.);
#56 = ORIENTED_EDGE('', *, *, #31, .T.);
#57 = ORIENTED_EDGE('', *, *, #30, .T.);
#58 = ORIENTED_EDGE('', *, *, #32, .F.);
#59 = ORIENTED_EDGE('', *, *, #29, .F.);
#60 = EDGE_LOOP('', (#56, #57, #58, #59));
#61 = FACE_OUTER_BOUND('', #60, .T.);
#62 = ADVANCED_FACE('', (#61), #37, .T.);
#63 = CLOSED_SHELL('', (#43, #48, #55, #62));
#64 = MANIFOLD_SOLID_BREP('prism', #63);"
        ))
    }

    /// Expect the mesh-fallback outcome with a watertight mesh whose
    /// signed volume is `expected` within `rel_tol`.
    fn assert_watertight_mesh_volume(source: &str, expected: f64, rel_tol: f64) {
        let (store, _geo, report) = import(source);
        assert_structured(&report);
        assert!(
            !report.has_errors(),
            "unexpected errors: {:?}",
            report.diagnostics
        );
        assert_eq!(report.solids.len(), 1);
        match &report.solids[0].outcome {
            SolidOutcome::Mesh { mesh, .. } => {
                assert!(mesh.is_closed_manifold(), "fallback mesh not watertight");
                let volume = signed_volume(mesh);
                assert!(
                    (volume - expected).abs() / expected < rel_tol,
                    "volume {volume} vs expected {expected}"
                );
            }
            other => panic!("expected the mesh fallback, got {other:?}"),
        }
        let _ = store;
    }

    #[test]
    fn parabola_extrusion_wall_imports_as_watertight_mesh() {
        // p(t) = (t^2, 2t, 0), t in [-1, 1]: endpoints (1, ±2). Region
        // between arc and chord x = 1: area 8/3, height 1.
        let src = conic_prism("PARABOLA('', #15, 1.)", "PARABOLA('', #16, 1.)", 1.0, 2.0);
        assert_watertight_mesh_volume(&src, 8.0 / 3.0, 0.02);
    }

    #[test]
    fn hyperbola_extrusion_wall_imports_as_watertight_mesh() {
        // p(t) = (cosh t, sinh t, 0), t in [-1, 1]: endpoints
        // (cosh 1, ±sinh 1). Area between arc and chord x = cosh 1 is
        // cosh(1)·sinh(1) − 1, height 1.
        let (ax, ay) = (1.0f64.cosh(), 1.0f64.sinh());
        let src = conic_prism(
            "HYPERBOLA('', #15, 1., 1.)",
            "HYPERBOLA('', #16, 1., 1.)",
            ax,
            ay,
        );
        assert_watertight_mesh_volume(&src, ax * ay - 1.0, 0.02);
    }

    #[test]
    fn composite_curve_seam_imports_as_watertight_mesh() {
        // The doc-example sphere with its seam meridian spelled as a
        // COMPOSITE_CURVE of two TRIMMED quarter arcs (south → equator →
        // north). Exact import has no multi-segment curve, so this must
        // degrade to a watertight tessellated sphere.
        let src = envelope(
            "\
#1 = CARTESIAN_POINT('', (0., 0., 0.));
#2 = CARTESIAN_POINT('', (0., 0., -2.));
#3 = CARTESIAN_POINT('', (0., 0., 2.));
#4 = DIRECTION('', (0., 0., 1.));
#5 = DIRECTION('', (0., -1., 0.));
#6 = DIRECTION('', (1., 0., 0.));
#7 = VERTEX_POINT('', #2);
#8 = VERTEX_POINT('', #3);
#9 = AXIS2_PLACEMENT_3D('', #1, #4, #6);
#10 = AXIS2_PLACEMENT_3D('', #1, #5, #6);
#11 = CIRCLE('', #10, 2.);
#12 = SPHERICAL_SURFACE('', #9, 2.);
#21 = CARTESIAN_POINT('', (2., 0., 0.));
#22 = TRIMMED_CURVE('', #11, (#2), (#21), .T., .CARTESIAN.);
#23 = TRIMMED_CURVE('', #11, (#21), (#3), .T., .CARTESIAN.);
#24 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS., .T., #22);
#25 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS., .T., #23);
#26 = COMPOSITE_CURVE('', (#24, #25), .F.);
#13 = EDGE_CURVE('', #7, #8, #26, .T.);
#14 = ORIENTED_EDGE('', *, *, #13, .T.);
#15 = ORIENTED_EDGE('', *, *, #13, .F.);
#16 = EDGE_LOOP('', (#14, #15));
#17 = FACE_OUTER_BOUND('', #16, .T.);
#18 = ADVANCED_FACE('', (#17), #12, .T.);
#19 = CLOSED_SHELL('', (#18));
#20 = MANIFOLD_SOLID_BREP('ball', #19);",
        );
        let expected = 4.0 / 3.0 * std::f64::consts::PI * 8.0;
        assert_watertight_mesh_volume(&src, expected, 0.05);
    }
}
