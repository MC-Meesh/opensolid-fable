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
//!    Freeform (NURBS) bodies now close the same loop: the writer emits them
//!    exactly (of-3qy.7) and the reader's exact path hangs `Surface3::Nurbs`
//!    on the face (of-3qy.8), so they return as B-Reps and are gated on
//!    volume through the tessellator wherever it can measure them.
//! 2. **Synthetic adversarial files** — missing entities, cyclic references,
//!    degenerate geometry, unit mismatches, huge coordinates, overflowing
//!    reals, truncation, garbage. The reader must return structured errors
//!    ([`StepError`] / [`Diagnostic`]s / [`SolidOutcome::Failed`]) or clean
//!    fallbacks. It must NEVER panic and NEVER silently import wrong
//!    geometry.
//! 3. **Vendored real-world files** — CATIA V5-authored CAx-IF test parts
//!    under `tests/data/step/` (see the README there for provenance and
//!    licensing). Analytic parts must import as exact B-Reps and survive a
//!    write round trip; so must NURBS-bearing parts, since of-3qy.7 and
//!    of-3qy.8 closed the freeform loop on both sides.
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
    Body, CheckFailure, Curve3, CurveEval, GeometryStore, MAX_ALLOWED_TOLERANCE, SYSTEM_RESOLUTION,
    Surface3, SurfaceProject, TessellationOptions, TopologyStore, primitives, tessellate_body,
    translate_body,
};
use opensolid_kernel::core::EntityId;
use opensolid_kernel::core::tolerance::ToleranceContext;
use opensolid_kernel::core::types::Vector3;
use opensolid_kernel::io::step::read::{
    Severity, SolidOutcome, StepImport, StepReadOptions, read_step, read_step_bytes,
};
use opensolid_kernel::io::step::write::{LengthUnit, StepWriteOptions, write_step};
use opensolid_kernel::{brep_mass_properties, mass_properties};

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

/// Volume the second way (of-ipt.17): surface integrals over the B-Rep faces,
/// reduced to contour integrals over each face's trim curves.
///
/// This is the corpus's only measurement of the bodies the standalone
/// tessellator defers to the CDT pass — [`closed_volume`] returns `None` for
/// those and every volume gate above silently skipped them. It is also the
/// one that reads an imported body's **stored** pcurves, which nothing else
/// in the round trip does: the writer emits them, the reader rebuilds them,
/// and until now no gate ever measured anything through them.
fn exact_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> Option<f64> {
    brep_mass_properties(store, geo, body)
        .ok()
        .map(|mp| mp.volume)
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
    assert_round_trip_gate(store, geo, body, context, FixedPoint::AfterOneTrip);
}

/// Whether `write ∘ read` must reproduce the file on the first re-write or
/// only from the second one.
///
/// Two things push it out by one iteration, and a body built in the kernel
/// hits at least the second every time:
/// - of-kb8: stores sharing one surface/curve across faces/edges re-import
///   with duplicated geometry instances.
/// - of-3qy.11: a kernel-built body carries no fin pcurves, while an
///   imported one always does, so the second file gains the `SURFACE_CURVE`
///   wrappers the first had nothing to write.
///
/// `Immediate` therefore only applies to a body that has already been
/// through an import — a corpus file, not a kernel-built fixture — where it
/// stays the stronger gate.
#[derive(Clone, Copy, PartialEq)]
enum FixedPoint {
    Immediate,
    AfterOneTrip,
    /// No fixed point yet: the file's re-writes oscillate in the last ULP of
    /// an `ELLIPSE` placement's directions and the pcurves fitted from them
    /// (of-qb5). Everything the gate checks before the text comparison —
    /// `check`, entity counts, volume — still applies; only the byte
    /// comparison is skipped. Tighten to `AfterOneTrip` once of-qb5 lands.
    UlpDrift,
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

    // The same two questions asked of the measurement that does not go
    // through a mesh (of-ipt.17). It reaches the bodies `closed_volume`
    // cannot, so this is the *only* volume gate several corpus files get, and
    // on an imported body it is the only thing that reads the pcurves the
    // round trip just wrote and re-read.
    let (e1, e2) = (
        exact_volume(store, geo, body),
        exact_volume(&store2, &geo2, body2),
    );
    assert_eq!(
        e1.is_some(),
        e2.is_some(),
        "{context}: B-Rep measurability must survive the round trip \
         (original {e1:?}, re-imported {e2:?})"
    );
    if let (Some(v1), Some(v2)) = (e1, e2) {
        assert!(
            v1 > 0.0,
            "{context}: original B-Rep volume must be positive, got {v1}"
        );
        let drift = (v1 - v2).abs() / v1.max(1.0);
        assert!(
            drift <= 1e-9,
            "{context}: B-Rep volume drift {drift:e} exceeds 1e-9 ({v1} vs {v2})"
        );
    }
    // Where both paths can measure, they must agree to the tessellation's own
    // fidelity: the default 32 samples per circle sit a sagitta
    // `R(1 − cos(π/32))` ≈ `4.8e-3·R` inside a curved face, which costs about
    // 1.3% of a doubly-curved body's volume. 3% covers that with room for a
    // body that is mostly curved surface.
    if let (Some(meshed), Some(exact)) = (m1, e1) {
        let gap = (meshed - exact).abs() / exact.abs().max(1e-300);
        assert!(
            gap <= 3e-2,
            "{context}: meshed volume {meshed} and B-Rep-native volume {exact} \
             disagree by {gap:e}, far past tessellation error"
        );
    }

    let text2 = write_step(&store2, &geo2, &[body2], &StepWriteOptions::default())
        .unwrap_or_else(|e| panic!("{context}: re-imported body must serialize: {e}"));
    match fixed_point {
        FixedPoint::Immediate => assert_eq!(
            text, text2,
            "{context}: write ∘ read must be a fixed point (geometry or topology drifted)"
        ),
        FixedPoint::UlpDrift => {}
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
///
/// A boolean output is kernel-built, so its fixed point is one trip out —
/// see [`FixedPoint`].
fn assert_boolean_round_trip(out: &BooleanOutput, expected_volume: f64, context: &str) {
    assert_boolean_round_trip_gate(out, expected_volume, context, FixedPoint::AfterOneTrip);
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

    // One trip out, like every kernel-built fixture — see [`FixedPoint`].
    let text2 = write_step(&store2, &geo2, &bodies2, &StepWriteOptions::default())
        .expect("re-imported bodies must serialize");
    let (store3, geo3, report3) = import(&text2);
    assert!(!report3.has_errors(), "{:?}", report3.diagnostics);
    let bodies3: Vec<_> = report3
        .solids
        .iter()
        .map(|solid| match &solid.outcome {
            SolidOutcome::BRep(body) => *body,
            other => panic!(
                "solid #{} did not re-import exactly: {other:?}",
                solid.step_id
            ),
        })
        .collect();
    let text3 = write_step(&store3, &geo3, &bodies3, &StepWriteOptions::default())
        .expect("third write must succeed");
    assert_eq!(text2, text3, "write ∘ read must stabilize after one trip");
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
// 1c. Round trips: freeform (NURBS) geometry (of-3qy.7)
// ---------------------------------------------------------------------
// The writer used to refuse `Curve3::Nurbs` / `Surface3::Nurbs` outright
// rather than approximate them; it now emits the real
// `B_SPLINE_CURVE_WITH_KNOTS` / `B_SPLINE_SURFACE_WITH_KNOTS` (and the
// `RATIONAL_B_SPLINE_*` complex instance when weights are present).
//
// The reader's *exact* path still refuses to hang a `Surface3::Nurbs` on a
// face (that is of-3qy.8), so today these bodies re-import through the
// tessellated fallback. That still gates the emission end to end: the
// fallback evaluates the patch the file actually carries, so a transposed
// control grid, a dropped knot multiplicity or a lost weight lands as the
// wrong volume. The outcome assertion accepts an exact B-Rep too, so these
// tests keep gating once of-3qy.8 flips the import side.

mod freeform {
    use super::*;
    use opensolid_kernel::brep::curve::plane_basis;
    use opensolid_kernel::brep::{
        Curve3, CurveEval, KnotVector, NurbsCurve, NurbsSurface, Surface3,
    };
    use opensolid_kernel::core::mesh::TriangleMesh;

    /// Signed volume by the divergence theorem: works on a fallback mesh,
    /// which is not indexed as a manifold but is closed and outward-wound.
    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    /// A control point's weight, as a function of *where it is* rather than
    /// which patch owns it.
    ///
    /// That is what keeps the six rational patches meshing into a closed
    /// manifold. Each block edge is shared by two faces, and the boundary
    /// of a rational bilinear patch is the rational degree-1 curve carrying
    /// its two corner weights; position-keyed weights make both faces
    /// assign that edge the same pair, so both sample it at the same points
    /// (the sample set is symmetric under t ↦ 1−t, so the two traversal
    /// directions agree) and the fallback's weld finds them. A weight grid
    /// keyed to each face's own (u, v) frame instead reparameterizes shared
    /// edges differently on either side and tears the mesh at T-junctions.
    ///
    /// The coefficients are arbitrary and small: all eight corners get
    /// distinct, positive weights well away from 1, so a writer that
    /// dropped or transposed the weight grid changes the parameterization
    /// visibly.
    fn corner_weight(p: &opensolid_kernel::core::types::Point3) -> f64 {
        1.0 + 0.05 * p.x + 0.03 * p.y + 0.02 * p.z
    }

    /// A 2×3×4 block whose six faces are bilinear NURBS patches **exactly
    /// coincident with the planes they replace**, and one of whose edges
    /// carries a degree-1 NURBS curve coincident with its line.
    ///
    /// Coincident on purpose: the solid's volume stays exactly 24, so the
    /// round trip is gated against an analytic number rather than against
    /// itself. Every freeform code path is still exercised — the writer
    /// takes both NURBS arms, and the reader re-evaluates the patches and
    /// the curve it finds in the file. Replacing *every* planar face rather
    /// than one keeps the tessellated re-import a closed manifold: the
    /// fallback grids a NURBS face over its whole domain, so a lone patch
    /// among plane neighbours would meet them at T-junctions.
    ///
    /// `rational` weights each control point by [`corner_weight`], which
    /// makes every patch a genuine `RATIONAL_B_SPLINE_SURFACE`. The locus
    /// is unchanged — a positive-weight rational combination of coplanar
    /// control points stays in their plane and still spans the same quad —
    /// so the volume gate holds either way.
    fn nurbs_block(rational: bool) -> (TopologyStore, GeometryStore, EntityId<Body>) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("block");

        let faces = store.faces_of_body(body);
        for &face in &faces {
            let sid = store
                .face(face)
                .expect("live face")
                .surface
                .expect("surface");
            let Surface3::Plane { origin, normal } = *geo.surface(sid).expect("live surface")
            else {
                panic!("a block's faces are all planar");
            };

            // The face's rectangle in its own frame. `plane_basis`
            // guarantees du × dv = normal, so each patch's normal agrees
            // with the plane's and the face's stored sense stays correct.
            let (du, dv) = plane_basis(&normal);
            let (mut u_lo, mut u_hi, mut v_lo, mut v_hi) = (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            );
            for vertex in store.vertices_of_face(face) {
                let d = store.vertex(vertex).expect("live vertex").point - origin;
                let (u, v) = (d.dot(&du), d.dot(&dv));
                u_lo = u_lo.min(u);
                u_hi = u_hi.max(u);
                v_lo = v_lo.min(v);
                v_hi = v_hi.max(v);
            }
            let at = |u: f64, v: f64| origin + du * u + dv * v;
            let grid = vec![
                vec![at(u_lo, v_lo), at(u_lo, v_hi)],
                vec![at(u_hi, v_lo), at(u_hi, v_hi)],
            ];
            let knots = || KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).expect("bilinear knots");
            let patch = if rational {
                let weights = grid
                    .iter()
                    .map(|row| row.iter().map(corner_weight).collect())
                    .collect();
                NurbsSurface::new(grid, weights, knots(), knots()).expect("rational patch")
            } else {
                NurbsSurface::bspline(grid, knots(), knots()).expect("patch")
            };
            store.faces.get_mut(face).expect("live face").surface =
                Some(geo.add_surface(Surface3::nurbs(patch)));
        }

        // One edge becomes a degree-1 NURBS curve through its own
        // endpoints, clamped over the edge's own parameter range — the same
        // locus and the same parameterization as the line it replaces.
        let edge_id = store.edges_of_face(faces[0])[0];
        let edge = store.edge(edge_id).expect("live edge").clone();
        let line = geo
            .curve(edge.curve.expect("edge curve"))
            .expect("live curve")
            .clone();
        let spline = NurbsCurve::bspline(
            vec![line.point(edge.t_start), line.point(edge.t_end)],
            KnotVector::new(1, vec![edge.t_start, edge.t_start, edge.t_end, edge.t_end])
                .expect("clamped knots"),
        )
        .expect("degree-1 spline");
        store.edges.get_mut(edge_id).expect("live edge").curve =
            Some(geo.add_curve(Curve3::nurbs(spline)));

        (store, geo, body)
    }

    /// Write, re-import, and require the solid's volume to survive. Since
    /// of-3qy.8 wired the reader's exact path this lands as a B-Rep rather
    /// than the mesh fallback; both outcomes are measured here.
    fn assert_freeform_round_trip(
        store: &TopologyStore,
        geo: &GeometryStore,
        body: EntityId<Body>,
        expected_volume: f64,
        context: &str,
    ) -> String {
        assert!(
            store.check(body).is_empty(),
            "{context}: original body must pass check: {:?}",
            store.check(body)
        );
        let text = write_step(store, geo, &[body], &StepWriteOptions::default())
            .unwrap_or_else(|e| panic!("{context}: freeform body must serialize: {e}"));

        let (store2, geo2, report) = import(&text);
        assert!(
            !report.has_errors(),
            "{context}: reader reported errors: {:?}",
            report.diagnostics
        );
        assert_structured(&report);
        assert_eq!(report.solids.len(), 1, "{context}: expected one solid");
        let measured = match &report.solids[0].outcome {
            SolidOutcome::BRep(body2) => closed_volume(&store2, &geo2, *body2)
                .unwrap_or_else(|| panic!("{context}: exact re-import must tessellate")),
            SolidOutcome::Mesh { mesh, .. } => signed_volume(mesh),
            other => panic!("{context}: unstructured outcome {other:?}"),
        };
        let drift = (measured - expected_volume).abs() / expected_volume;
        assert!(
            drift <= 1e-9,
            "{context}: volume {measured} is not {expected_volume} \
             — the emitted control grid, knots or weights did not survive"
        );
        text
    }

    #[test]
    fn round_trip_nurbs_faced_block() {
        let (store, geo, body) = nurbs_block(false);
        let text = assert_freeform_round_trip(&store, &geo, body, 24.0, "NURBS block");
        assert!(
            text.contains("B_SPLINE_SURFACE_WITH_KNOTS"),
            "the faces must be emitted as real B-spline surfaces"
        );
        assert!(
            text.contains("B_SPLINE_CURVE_WITH_KNOTS"),
            "the freeform edge must be emitted as a real B-spline curve"
        );
        assert!(
            !text.contains("RATIONAL"),
            "unweighted geometry needs no complex instance"
        );
        // Emission is deterministic: the same stores produce the same file.
        let again =
            write_step(&store, &geo, &[body], &StepWriteOptions::default()).expect("second write");
        assert_eq!(text, again, "writing twice must produce the same file");
    }

    #[test]
    fn round_trip_rational_nurbs_faced_block() {
        let (store, geo, body) = nurbs_block(true);
        let text = assert_freeform_round_trip(&store, &geo, body, 24.0, "rational NURBS block");
        // A weighted patch has nowhere to put its weights in the plain
        // entity, so it must take the complex-instance form.
        assert!(
            text.contains("BOUNDED_SURFACE()") && text.contains("RATIONAL_B_SPLINE_SURFACE((("),
            "a rational patch needs the complex-instance form"
        );
        assert!(
            !text.contains("B_SPLINE_SURFACE_WITH_KNOTS('',"),
            "no face may fall back to the unweighted entity and lose its weights"
        );
        // Weights are what make the re-imported parameterization match, and
        // a patch written with all weights 1 would still pass the volume
        // gate (same locus) — so pin every emitted weight grid exactly
        // against the patch it came from.
        let mut checked = 0;
        for (_, surface) in geo.surfaces.iter() {
            let Surface3::Nurbs(patch) = surface else {
                continue;
            };
            let (rows, cols) = patch.grid_size();
            let grid: Vec<String> = (0..rows)
                .map(|i| {
                    let row: Vec<String> = (0..cols)
                        .map(|j| {
                            let w = patch.weight(i, j);
                            assert_ne!(w, 1.0, "the fixture must carry non-unit weights");
                            format!("{w:?}")
                        })
                        .collect();
                    format!("({})", row.join(","))
                })
                .collect();
            let expected = format!("RATIONAL_B_SPLINE_SURFACE(({}))", grid.join(","));
            assert!(
                text.contains(&expected),
                "weight grid missing from the emitted file: {expected}"
            );
            checked += 1;
        }
        assert_eq!(checked, 6, "every face of the block must be a patch");
    }
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

    /// Gaps at the last written decimal, plus two faces authored backwards:
    /// both passes together, still exact, still the right way out.
    ///
    /// Gated through [`closed_volume`], which needs the tessellation to weld
    /// watertight. That used to be impossible here (of-61f): the healed
    /// vertices sit at their cluster centroids while each adjacent edge's
    /// curve still runs to its own pre-merge endpoint, so faces meeting at one
    /// reached it along different edges and landed up to the closed gap apart
    /// — 1.17e-7 here against a weld epsilon of 1.7e-9. The test measured
    /// volume with a local divergence sum to sidestep it. `sample_loop` now
    /// starts each fin's run at the fin's *vertex* point, so every loop
    /// through a vertex emits the identical corner and the weld is exact.
    #[test]
    fn gapped_and_misoriented_shell_heals_completely() {
        let (store, geo, report) = import(&unsewn_tetrahedron(1e-7, &[1, 2]));
        assert_structured(&report);
        let body = only_brep(&report);
        let counts = store.euler_counts(body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (4, 6, 4));
        let volume = closed_volume(&store, &geo, body)
            .expect("a healed tolerant body must still weld watertight (of-61f)");
        assert!(
            (volume - 1.0 / 6.0).abs() < 1e-6,
            "outward, not inside out: got {volume}"
        );
    }

    /// The weld itself, across the whole range of gaps healing will close: a
    /// tolerant body tessellates to exactly the vertex count of the exact one
    /// and closes, instead of leaving one rim sample per (face, vertex) pair
    /// stranded. of-61f — the regression gate for the corner-snapping in
    /// `sample_loop`, measured before the fix at 8 unwelded vertices and
    /// `NotClosedManifold` for every gap from 1e-9 up.
    #[test]
    fn a_healed_tolerant_body_welds_watertight_at_every_gap() {
        let exact = {
            let (store, geo, report) = import(&unsewn_tetrahedron(0.0, &[]));
            let body = only_brep(&report);
            tessellate_body(&store, &geo, body, &TessellationOptions::default())
                .expect("exact body tessellates")
                .positions
                .len()
        };
        for gap in [1e-9, 1e-8, 1e-7, 1e-6, 1e-5] {
            let (store, geo, report) = import(&unsewn_tetrahedron(gap, &[]));
            assert_structured(&report);
            let body = only_brep(&report);
            let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default())
                .expect("healed body tessellates");
            assert_eq!(
                mesh.positions.len(),
                exact,
                "gap {gap:e}: a tolerant body must weld to the same vertices as an exact one"
            );
            assert!(
                mesh.is_closed_manifold(),
                "gap {gap:e}: healed body tessellated to an open mesh"
            );
            let volume = closed_volume(&store, &geo, body).expect("welded mesh measures");
            assert!(
                (volume - 1.0 / 6.0).abs() < 1e-4,
                "gap {gap:e}: volume {volume}"
            );
        }
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
                ..HealOptions::default()
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
            files.len() >= 34,
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
        // 2026-08-04: 34 of 34 — the whole corpus. The last file in was
        // nist_ctc_05 (of-5cn5): one CIRCLE edge (#4444) whose end
        // VERTEX_POINT sits 7.2e-4 in off the circle's *plane* — 0.0182 mm,
        // nearly twice MAX_ALLOWED_TOLERANCE, so no tolerance the reader
        // could derive would let the kernel carry it, and no consistent
        // shell survived it for the fallback either. Snapping the vertex was
        // no answer: its CARTESIAN_POINT #374 is the last control point of
        // B_SPLINE_CURVE #375 and the other three edges meeting there
        // interpolate it exactly, so a snap only moves the 18 um onto edges
        // that carried none. What admits it is vertex reconciliation
        // (read.rs `reconcile_vertices`): the miss is connectivity slop
        // inside the file's own declared 3.67e-3 in closure, so the reader
        // moves the vertex to the minimax point among its four edges'
        // curves and every edge carries ~9 um as ordinary vertex tolerance,
        // under the kernel limit. The file imports on the exact path,
        // geometrically clean (see the floor below).
        const FLOOR: usize = 34;
        assert!(
            passed.len() >= FLOOR,
            "corpus pass count regressed below {FLOOR}: only {passed:?} pass"
        );
    }

    /// The geometric counterpart of the pass-rate floor (of-ipt.13). A file
    /// passes when every B-Rep solid in it also survives
    /// [`TopologyStore::check_geometry`]: edges on their faces' surfaces
    /// within the tolerance they claim, vertices on their edges' endpoints,
    /// pcurves tracking their edges, face senses agreeing with their
    /// boundaries.
    ///
    /// This is a much harder gate than `check` and part of the corpus does
    /// not clear it yet. That is the point of pinning it: the files that do
    /// are a floor a reader or checker regression cannot quietly drop below,
    /// and the ones that do not are the campaign's work list (of-bb6
    /// unmeasured import tolerances). Raise the floor as those land —
    /// of-fid did, bringing dm1-id-214 in.
    ///
    /// of-he8 came off that list without moving this count: choosing outer
    /// bounds by area silenced every `FaceSenseContradictsLoop` in the corpus
    /// (see `no_imported_face_contradicts_its_outer_loop`), but the four NIST
    /// files it fixed still fail this gate on the defects above.
    ///
    /// of-bbh8 is the second of those: carrying the trim residual as the
    /// vertex's tolerance took every `VertexOffEdge` in the corpus to zero
    /// (116 of them, see [`no_imported_vertex_is_off_its_edges`]) and again
    /// moved this count by nothing. Both times the same six files were held
    /// down by a *second* defect underneath — of-bb6's edge half, which is
    /// what finally moved it: 20 to 31, eleven files at once, once the reader
    /// measured how far each edge really sits from its faces' surfaces and
    /// carried that as the edge's tolerance. Every `EdgeOffSurface` and every
    /// `PcurveDeviation` in the corpus went with it (a pcurve's image lies on
    /// the surface, so it can never be closer to the curve than the surface
    /// itself is — the pcurve reports were the same gap seen twice).
    ///
    /// The 20 it started from is measured on of-zdx's main, not the 18 that
    /// main's own constant still claimed: of-zdx brought the two occ/tangent
    /// parts in as clean B-Reps without raising the floor it passed.
    ///
    /// The three files still outside are the three that no longer produce a
    /// B-Rep at all, listed in [`corpus_pass_rate_does_not_regress`]; this
    /// gate reads them as vacuous rather than clean. So the two counts move
    /// together now, and the next lift on either is of-05ac's.
    #[test]
    fn corpus_geometric_pass_rate_does_not_regress() {
        let mut clean = Vec::new();
        for file in &corpus_files() {
            let bytes = std::fs::read(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
            let (store, geo, report) = import_bytes(&bytes);
            let breps: Vec<EntityId<Body>> = report
                .solids
                .iter()
                .filter_map(|s| match &s.outcome {
                    SolidOutcome::BRep(body) => Some(*body),
                    _ => None,
                })
                .collect();
            // A file with no exact solid clears this vacuously; only files
            // that actually produce B-Reps count as geometrically clean.
            if !breps.is_empty()
                && breps
                    .iter()
                    .all(|&body| store.check_geometry(&geo, body).is_empty())
            {
                clean.push(
                    file.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned(),
                );
            }
        }
        // 2026-08-04: 33 files clear this — every corpus file that imports
        // as a B-Rep at all is also geometrically clean. of-5cn5 added
        // nist_ctc_05: its reconciled vertex (see the pass-rate floor above)
        // carries the split miss as its own tolerance, so the geometry gates
        // hold for it like any other authored slop. (nist_ctc_02 stays a
        // fallback mesh, vacuous here, so the two counts differ by one.)
        // Before that, 2026-08-01: 32, up from a measured 31 on of-bb6's
        // main — the one that bead added was bspline_patch_prism: its four
        // extrusion walls were patches one `VECTOR` long and did not
        // contain their own faces, which showed up here as 12 EdgeOffSurface
        // (to 20 mm) and 8 PcurveDeviation. Sizing the patch to the face it
        // carries took all 20 to zero.
        const FLOOR: usize = 33;
        assert!(
            clean.len() >= FLOOR,
            "geometrically clean corpus count regressed below {FLOOR}: only {clean:?} pass"
        );
    }

    /// No imported vertex may sit further from the endpoint of an adjacent
    /// edge's curve than its own tolerance permits — `spec/08-tolerances.md`
    /// §7.1 invariant 2, the vertex half of of-bb6.
    ///
    /// The reader has to *accept* such a miss: STEP writes finite decimals, so
    /// a `VERTEX_POINT` and the `EDGE_CURVE` it sits on are rounded apart, and
    /// `verify_trim` allows the difference up to `TRIM_TOL_REL`. What it used
    /// to do was accept it and then create the vertex at `SYSTEM_RESOLUTION`
    /// anyway, which is a claim the file does not support: 116 `VertexOffEdge`
    /// reports across six vendored files, 78 of them in nist_ctc_02 alone. The
    /// trim residual is carried as the vertex's tolerance now (of-bbh8) and the
    /// count is zero.
    ///
    /// An absolute gate, not a floor, like
    /// [`no_imported_face_contradicts_its_outer_loop`]: the reader measures
    /// this deviation directly, so there is no file it cannot be right about.
    /// [`no_imported_edge_is_off_its_faces`] is the other half.
    #[test]
    fn no_imported_vertex_is_off_its_edges() {
        let mut offenders: Vec<(String, usize)> = Vec::new();
        for file in &corpus_files() {
            let bytes = std::fs::read(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
            let (store, geo, report) = import_bytes(&bytes);
            let count: usize = report
                .solids
                .iter()
                .filter_map(|s| match &s.outcome {
                    SolidOutcome::BRep(body) => Some(*body),
                    _ => None,
                })
                .map(|body| {
                    store
                        .check_geometry(&geo, body)
                        .iter()
                        .filter(|f| matches!(f, CheckFailure::VertexOffEdge { .. }))
                        .count()
                })
                .sum();
            if count > 0 {
                offenders.push((
                    file.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned(),
                    count,
                ));
            }
        }
        assert!(
            offenders.is_empty(),
            "imported vertices off their edges' curves: {offenders:?}"
        );
    }

    /// No imported edge's curve may stray further from the surface of a face
    /// it bounds than its own tolerance permits — `spec/08-tolerances.md`
    /// §7.1 invariant 1, the edge half of of-bb6 and the counterpart of
    /// [`no_imported_vertex_is_off_its_edges`].
    ///
    /// The same shape of defect as the vertex half, and the same fix. STEP
    /// rounds an `EDGE_CURVE` and the surfaces of the faces it bounds into
    /// decimal text independently, and plenty of producers author a curve
    /// that never sat exactly on either surface to begin with — 828 reports
    /// on nist_ctc_02 alone, and gaps from 3e-9 mm (nist_ftc_09, four orders
    /// above what the file's own written precision explains) to 5.9e-3 mm
    /// (nist_ftc_07). The reader used to accept every one of those and stamp
    /// `SYSTEM_RESOLUTION` on the edge regardless. It measures now, and
    /// carries the measurement.
    ///
    /// Absolute, again because the measurement is available: an edge is
    /// either given a tolerance that covers where its curve actually is, or —
    /// past `MAX_ALLOWED_TOLERANCE`, where no honest tolerance exists — the
    /// solid does not take the exact path at all, so it is not here to be
    /// asked. Both nist_ctc_02 and bspline_patch_prism leave by that door
    /// (of-05ac); if either returns, it returns measured.
    #[test]
    fn no_imported_edge_is_off_its_faces() {
        let mut offenders: Vec<(String, usize)> = Vec::new();
        for file in &corpus_files() {
            let bytes = std::fs::read(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
            let (store, geo, report) = import_bytes(&bytes);
            let count: usize = report
                .solids
                .iter()
                .filter_map(|s| match &s.outcome {
                    SolidOutcome::BRep(body) => Some(*body),
                    _ => None,
                })
                .map(|body| {
                    store
                        .check_geometry(&geo, body)
                        .iter()
                        .filter(|f| matches!(f, CheckFailure::EdgeOffSurface { .. }))
                        .count()
                })
                .sum();
            if count > 0 {
                offenders.push((
                    file.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned(),
                    count,
                ));
            }
        }
        assert!(
            offenders.is_empty(),
            "imported edges off their faces' surfaces: {offenders:?}"
        );
    }

    /// Every tolerance the reader writes must be one the kernel would accept:
    /// at least `SYSTEM_RESOLUTION`, at most `MAX_ALLOWED_TOLERANCE`. The
    /// point of of-bb6 is that a tolerance is a *bound* on where an entity
    /// is, and a bound past the kernel's own cap is not one it can honour —
    /// so an import that would need it refuses the exact path rather than
    /// writing the number down (which `check` would reject anyway, one step
    /// later and less clearly).
    #[test]
    fn no_imported_tolerance_is_outside_the_kernels_range() {
        for file in &corpus_files() {
            let bytes = std::fs::read(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
            let (store, _, report) = import_bytes(&bytes);
            let name = file.file_name().unwrap_or_default().to_string_lossy();
            for body in report.solids.iter().filter_map(|s| match &s.outcome {
                SolidOutcome::BRep(body) => Some(*body),
                _ => None,
            }) {
                for shell in &store.body(body).expect("imported body").shells {
                    for face in &store.shell(*shell).expect("shell").faces {
                        for edge_id in store.edges_of_face(*face) {
                            let edge = store.edge(edge_id).expect("edge of face");
                            assert!(
                                (SYSTEM_RESOLUTION..=MAX_ALLOWED_TOLERANCE)
                                    .contains(&edge.tolerance),
                                "{name}: edge {edge_id:?} imported with tolerance {}",
                                edge.tolerance
                            );
                        }
                    }
                }
            }
        }
    }

    /// No imported face may disagree with its own outer boundary (of-he8).
    ///
    /// A face's sense and the winding of its outer loop are two statements of
    /// the same fact, so a contradiction means one of them was read wrong.
    /// It was always the loop: the reader took the outer bound to be the one
    /// the file tagged, or the first one listed, and on a face with holes
    /// both land on a hole often enough to matter. 61 faces reported across
    /// four NIST files before the outer bound was chosen by parameter-space
    /// area instead — 38 of them untagged (nist_ctc_01, nist_ctc_03,
    /// nist_ftc_06), 23 mis-tagged (nist_stc_06).
    ///
    /// This is an absolute gate, not a floor: unlike the pass-rate tests
    /// above, zero is reachable today and there is no reason for a file to
    /// take it back off zero.
    #[test]
    fn no_imported_face_contradicts_its_outer_loop() {
        let mut offenders: Vec<(String, usize)> = Vec::new();
        for file in &corpus_files() {
            let bytes = std::fs::read(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
            let (store, geo, report) = import_bytes(&bytes);
            let contradictions: usize = report
                .solids
                .iter()
                .filter_map(|s| match &s.outcome {
                    SolidOutcome::BRep(body) => Some(*body),
                    _ => None,
                })
                .map(|body| {
                    store
                        .check_geometry(&geo, body)
                        .iter()
                        .filter(|f| matches!(f, CheckFailure::FaceSenseContradictsLoop { .. }))
                        .count()
                })
                .sum();
            if contradictions > 0 {
                offenders.push((
                    file.file_name()
                        .unwrap_or_default()
                        .to_string_lossy()
                        .into_owned(),
                    contradictions,
                ));
            }
        }
        assert!(
            offenders.is_empty(),
            "faces whose sense contradicts their outer loop (file, count): {offenders:?}"
        );
    }

    /// The tangent-boolean parts import with every edge two-sided (of-zdx).
    ///
    /// A tangency pinches the solid, and OCC spells the pinch by hanging four
    /// fins on one `EDGE_CURVE`: the hole's cylindrical face uses it twice as
    /// its seam, and the two coplanar halves the tangency splits the block's
    /// wall into use it once each. The reader moves the seam onto its own
    /// edge, so `hole_tangent_to_wall` comes out with one edge more than the
    /// file declares and nothing over two fins. `check` would refuse the body
    /// otherwise, which is how the part used to fail.
    #[test]
    fn tangent_parts_import_with_no_edge_over_two_fins() {
        for (name, declared_edges) in [
            ("occ/tangent/cylinder_tangent_to_wall.stp", 21),
            ("occ/tangent/hole_tangent_to_wall.stp", 18),
        ] {
            let path = format!("{}/tests/data/step/{name}", env!("CARGO_MANIFEST_DIR"));
            let bytes = std::fs::read(&path).unwrap_or_else(|e| panic!("read {path}: {e}"));
            let (store, _, report) = import_bytes(&bytes);
            let [solid] = &report.solids[..] else {
                panic!("{name}: expected exactly one solid");
            };
            let SolidOutcome::BRep(body) = solid.outcome else {
                panic!("{name}: expected an exact B-Rep import");
            };
            let overshared: Vec<usize> = store
                .edges
                .iter()
                .map(|(_, edge)| edge.fins.len())
                .filter(|&fins| fins > 2)
                .collect();
            assert!(
                overshared.is_empty(),
                "{name}: edges with more than two fins: {overshared:?}"
            );
            assert_eq!(
                store.euler_counts(body).edges,
                declared_edges,
                "{name}: edge count moved — the seam split is off"
            );
        }
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

    /// of-26t: both parts have full cones whose apex bound is a `VERTEX_LOOP`
    /// — the degenerate loop the reader refused before, dropping the whole
    /// part to the mesh fallback.
    #[test]
    fn nist_ctc_01_imports_exactly_and_round_trips() {
        assert_nist_exact_and_round_trips_gate(
            "nist/nist_ctc_01_asme1_rd.stp",
            FixedPoint::UlpDrift,
        );
    }

    #[test]
    fn nist_ftc_06_imports_exactly_and_round_trips() {
        assert_nist_exact_and_round_trips_gate(
            "nist/nist_ftc_06_asme1_rd.stp",
            FixedPoint::AfterOneTrip,
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
    fn dm1_id_nurbs_part_imports_exactly() {
        // Three solids whose walls are B-splines in every AP203 spelling:
        // simple `B_SPLINE_SURFACE_WITH_KNOTS`, the `RATIONAL_B_SPLINE_-
        // SURFACE` complex instance, and knot-free `QUASI_UNIFORM_SURFACE`
        // (with the matching curve forms on the edges). Before of-3qy.8
        // every one of them was refused off the exact path; now all three
        // solids must land as valid B-Reps carrying real NURBS geometry —
        // no tessellated fallback, no unsupported-geometry warnings.
        let bytes = load("dm1-id-214.stp");
        let (store, geo, report) = import_bytes(&bytes);
        assert_eq!(report.solids.len(), 3, "expected three solids");
        assert!(
            !report
                .diagnostics
                .iter()
                .any(|d| d.message.contains("unsupported")),
            "expected no unsupported-geometry diagnostics, got: {:?}",
            report
                .diagnostics
                .iter()
                .filter(|d| d.message.contains("unsupported"))
                .take(5)
                .collect::<Vec<_>>()
        );
        let breps = assert_all_outcomes_structured(&store, &report);
        assert_eq!(breps.len(), 3, "expected three exact B-Rep imports");

        // The exactness is the point: the walls must be NURBS in the
        // geometry store, not quadrics some reduction guessed at, and the
        // rational ones must carry their non-unit weights.
        let nurbs_surfaces = geo
            .surfaces
            .iter()
            .filter(|(_, s)| matches!(s, Surface3::Nurbs(_)))
            .count();
        assert!(
            nurbs_surfaces >= 3,
            "expected NURBS surfaces in the store, got {nurbs_surfaces}"
        );
        let nurbs_curves = geo
            .curves
            .iter()
            .filter(|(_, c)| matches!(c, Curve3::Nurbs(_)))
            .count();
        assert!(
            nurbs_curves >= 3,
            "expected NURBS curves in the store, got {nurbs_curves}"
        );
        assert!(
            geo.curves.iter().any(|(_, c)| match c {
                Curve3::Nurbs(nurbs) => nurbs.weights().iter().any(|&w| w != 1.0),
                _ => false,
            }),
            "expected a rational (non-unit weight) NURBS curve from the \
             RATIONAL_B_SPLINE_CURVE complex instances"
        );

        // `check` is topological, so it would pass a mis-parameterized
        // patch just as happily (swapped u/v, a knot vector expanded from
        // the wrong multiplicity list, weights read off the wrong record).
        // Every one of those moves the surface off its own edges, so
        // require the geometry to agree: each bounding edge must lie on
        // the face it bounds.
        for &body in &breps {
            assert_edges_lie_on_faces(&store, &geo, body);
        }

        // These three solids are exactly the trimmed-NURBS bodies the
        // standalone tessellator used to refuse (`nurbs_lattice` rejects any
        // boundary that leaves the knot-domain border), which left the
        // edge-on-surface sampling above as this file's only geometric gate
        // (of-znb). Routing freeform faces through the constrained-Delaunay
        // pass (of-37i.6) gave them a standalone path, so require what that
        // bead's repro asked for: every solid meshes closed and measures a
        // positive volume.
        //
        // Where the B-Rep-native contour integral can also measure, the two
        // must agree to tessellation fidelity (the same 3% calibration
        // `assert_round_trip_gate` uses). It cannot on the two solids whose
        // walls close in `u`: their seam fins' pcurves sit on the same branch
        // instead of period-separated ones, so `brep_mass_properties` refuses
        // the loop as open (of-z6zg, which also tracks the sub-percent
        // disagreement of all three volumes with OCC's — a reader-side
        // geometry difference this self-consistency check cannot see).
        let mut cross_checked = 0usize;
        for &body in &breps {
            let meshed = closed_volume(&store, &geo, body).unwrap_or_else(|| {
                panic!("dm1 solid must tessellate closed and measure a volume (of-znb)")
            });
            assert!(meshed > 0.0, "dm1 meshed volume must be positive: {meshed}");
            if let Some(exact) = exact_volume(&store, &geo, body) {
                let gap = (meshed - exact).abs() / exact.abs().max(1e-300);
                assert!(
                    gap <= 3e-2,
                    "dm1 meshed volume {meshed} and B-Rep-native volume {exact} \
                     disagree by {gap:e}, far past tessellation error"
                );
                cross_checked += 1;
            }
        }
        assert!(
            cross_checked >= 1,
            "no dm1 solid measured through its B-Rep faces — the contour \
             integral regressed past even the u-open solid (see of-z6zg)"
        );

        // The freeform loop closes here on *authored* CAD geometry rather
        // than a synthetic patch: export (of-3qy.7) re-emits what this
        // import read, and reading that back must reproduce the same
        // surfaces on the same edges. A knot vector or weight list that
        // survived import but not the write would show up as a re-import
        // whose edges no longer lie on their faces.
        for &body in &breps {
            let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
                .expect("NURBS body must serialize");
            let (store2, geo2, report2) = import_bytes(text.as_bytes());
            assert!(
                !report2.has_errors(),
                "re-import reported errors: {:?}",
                report2.diagnostics
            );
            let breps2 = assert_all_outcomes_structured(&store2, &report2);
            assert_eq!(breps2.len(), 1, "re-import must be one exact B-Rep");
            assert_counts_equal(&store, body, &store2, breps2[0], "dm1 NURBS round trip");
            assert_edges_lie_on_faces(&store2, &geo2, breps2[0]);
            // A re-imported solid must stay measurable, and at the same
            // volume: the writer re-emitting a knot vector or weight list
            // imprecisely would move the walls without disturbing the counts
            // or the edge-on-surface samples above (of-znb).
            let v1 = closed_volume(&store, &geo, body)
                .expect("original dm1 solid measures (asserted above)");
            let v2 = closed_volume(&store2, &geo2, breps2[0])
                .unwrap_or_else(|| panic!("dm1 solid must stay tessellable across the round trip"));
            let drift = (v1 - v2).abs() / v1.max(1.0);
            assert!(
                drift <= 1e-9,
                "dm1 volume drift {drift:e} across the round trip ({v1} vs {v2})"
            );
        }
    }

    /// A prism whose four walls are `SURFACE_OF_LINEAR_EXTRUSION`s of
    /// degree-5 B-splines (of-8ulj).
    ///
    /// The entity states no extent: it is unbounded along the sweep, and
    /// this file — like most — spells a `VECTOR` of magnitude 1 pointing
    /// `+z` for faces that reach 20 mm along `−z`. Sweeping the profile by
    /// that vector, as the reader used to, built four patches that contained
    /// none of their own faces but the `z = 0` rim: 12 `EdgeOffSurface`
    /// reports up to 20 mm, and a tessellation that called the geometry
    /// degenerate. The patches are sized from the faces' own bounds now, so
    /// the exact import has to be geometrically clean, not merely
    /// structurally valid.
    #[test]
    fn bspline_extrusion_prism_imports_on_patches_that_reach_its_faces() {
        let name = "occ/nurbs/bspline_patch_prism.stp";
        let (store, geo, report) = import_bytes(&load(name));
        assert!(
            report.diagnostics.is_empty(),
            "{name}: expected a clean exact import, got: {:?}",
            report.diagnostics
        );
        let breps = assert_all_outcomes_structured(&store, &report);
        assert_eq!(breps.len(), 1, "{name}: expected an exact B-Rep import");
        let body = breps[0];

        let failures = store.check_geometry(&geo, body);
        assert!(
            failures.is_empty(),
            "{name}: extrusion patches do not reach their faces: {failures:?}"
        );

        // OCC reads this part as 50000 mm³ (`reference/occ/nurbs/`), and
        // both of our measurements have to find the same solid. The
        // tessellated one carries the chord error of a degree-5 profile at
        // the default 32-segments-per-turn pitch (0.36% here; `occ_reference`
        // measures it at its own finer pitch and lands inside 0.01%). The
        // exact one integrates the B-Rep faces through their stored pcurves
        // and has no such excuse.
        let volume = closed_volume(&store, &geo, body).expect("the prism must tessellate closed");
        assert!(
            (volume - 50_000.0).abs() / 50_000.0 < 1e-2,
            "{name}: tessellated volume {volume} is not OCC's 50000"
        );
        let exact = exact_volume(&store, &geo, body).expect("the prism must integrate");
        assert!(
            (exact - 50_000.0).abs() / 50_000.0 < 1e-9,
            "{name}: exact volume {exact} is not OCC's 50000"
        );
    }

    /// An exact import is only *usable* once it has a mesh: the wasm layer
    /// builds its `MeshSdf` from one, and every measurement the MCP server
    /// reports goes through it. Both of these files read as exact B-Reps
    /// with zero diagnostics and still had no shape at all, because two
    /// separate tessellator gaps refused every solid in them (of-6fcu).
    ///
    /// - `io1-cm-214` carries two torus fillet quarter-rounds. A trimmed
    ///   sphere/torus face used to be refused outright; it now takes the
    ///   constrained-Delaunay pass.
    /// - `dm1-id-214`'s three solids are built from ruled patches closed in
    ///   `u`, whose seam edge is traversed once each way. Projecting each
    ///   boundary sample independently gave both traversals the same `u`,
    ///   so the parameter ring bounded nothing; the ring is now walked with
    ///   continuity across the patch's join.
    ///
    /// Closed *and* manifold is the assertion that matters: it says the
    /// faces welded to their neighbours, which is what separates a real
    /// tessellation from a pile of triangles that merely stopped erroring.
    /// The volumes these meshes measure are gated against OpenCASCADE in
    /// `occ_reference.rs`.
    #[test]
    fn every_exactly_imported_solid_tessellates_closed() {
        for name in ["io1-cm-214.stp", "dm1-id-214.stp"] {
            let (store, geo, report) = import_bytes(&load(name));
            let breps = assert_all_outcomes_structured(&store, &report);
            assert!(!breps.is_empty(), "{name}: expected exact B-Rep imports");
            for body in breps {
                let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default())
                    .unwrap_or_else(|e| panic!("{name}: exactly-imported body must mesh: {e}"));
                assert!(
                    mesh.is_closed_manifold(),
                    "{name}: mesh is not a closed manifold: {:?}",
                    mesh.manifold_defects().describe()
                );
            }
        }
    }

    /// Sample every face's bounding edges and require each sample to lie on
    /// that face's surface. The tolerance is relative: this part's
    /// coordinates are inches-scaled-to-mm, and STEP carries only finite
    /// decimal text.
    fn assert_edges_lie_on_faces(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) {
        const SAMPLES: usize = 7;
        let mut checked = 0usize;
        for shell_id in &store.body(body).expect("imported body").shells {
            for face_id in &store.shell(*shell_id).expect("shell").faces {
                let face = store.face(*face_id).expect("face");
                let surface = geo
                    .surfaces
                    .get(face.surface.expect("imported face has a surface"))
                    .expect("surface");
                let loops = face.outer_loop.iter().chain(&face.inner_loops);
                for loop_id in loops {
                    for fin_id in &store.loop_(*loop_id).expect("loop").fins {
                        let fin = store.fin(*fin_id).expect("fin");
                        let edge = store.edge(fin.edge).expect("edge");
                        let curve = geo
                            .curves
                            .get(edge.curve.expect("imported edge has a curve"))
                            .expect("curve");
                        for i in 0..=SAMPLES {
                            let t = edge.t_start
                                + (edge.t_end - edge.t_start) * i as f64 / SAMPLES as f64;
                            let point = curve.point(t);
                            let distance = surface.project_point(&point).distance;
                            let tol = 1e-6 * (1.0 + point.coords.norm());
                            assert!(
                                distance <= tol,
                                "edge sample at t={t} is {distance} off its face's surface"
                            );
                            checked += 1;
                        }
                    }
                }
            }
        }
        assert!(checked > 0, "body had no edge samples to check");
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
