//! Cross-feature integration tests (of-yxb): the capstone for the STEP
//! (of-3qy) and sphere/torus exact-boolean (of-7ld) campaigns. Each test
//! chains features that shipped independently and asserts the composite
//! result against closed-form geometry:
//!
//! 1. STEP import → exact sphere subtract → STEP export → re-import,
//!    with the re-imported solid enclosing the analytic volume to within
//!    1e-6 relative.
//! 2. A hybrid boolean with a torus operand taking the exact B-Rep path,
//!    whose result round-trips through STEP write/read.
//! 3. A sketch-extrude (sweep) body exported to STEP and re-imported
//!    cleanly, via the faceted-conversion pipeline (sweep topology carries
//!    placeholder geometry slots, so direct sweep → STEP export is not
//!    possible yet — of-xyx tracks binding real geometry).
//!
//! Measurement notes. `tessellate_body` deliberately rejects trimmed
//! quadric faces (of-q6u), so a re-imported body with a spherical dimple
//! cannot be measured standalone. Test 1 therefore measures the boolean
//! result's volume before export through [`BooleanOutput::tessellate`]
//! (whose CDT pass handles trimmed faces) and pins the re-imported body to
//! that measurement by requiring `write ∘ read` to be a **byte-identical
//! fixed point**: the re-imported solid serializes to the same file, so it
//! is the same geometry down to every `f64`, and its enclosed volume IS
//! the measured one. Measuring the re-imported body directly — through a
//! second exact boolean — currently produces a non-manifold tessellation
//! (of-6cf); that path is covered by an `#[ignore]`d test referencing the
//! bead.
//!
//! The 1e-6 volume gates hold because every face of the measured results
//! is planar except a small spherical cap: planar faces triangulate
//! exactly, so the only discretization error is the cap's, and the cap is
//! sized so that error stays far below the budget.

use std::f64::consts::{FRAC_PI_2, PI};

use opensolid_kernel::brep::boolean::{BooleanOutput, subtract};
use opensolid_kernel::brep::sweep::{Profile, ProfileSegment, extrude};
use opensolid_kernel::brep::{
    Body, GeometryStore, TessellationOptions, TopologyStore, primitives, tessellate_body,
    translate_body,
};
use opensolid_kernel::core::EntityId;
use opensolid_kernel::core::tolerance::ToleranceContext;
use opensolid_kernel::core::types::{BoundingBox3, Point3, Vector3};
use opensolid_kernel::hybrid::{self, HybridBody, HybridOptions, HybridPath};
use opensolid_kernel::io::step::read::{SolidOutcome, StepImport, StepReadOptions, read_step};
use opensolid_kernel::io::step::write::{StepWriteOptions, write_step};
use opensolid_kernel::{MeshSdf, SdfToBrepOptions, mass_properties, sdf_to_brep};

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

/// Relative volume tolerance for the composite gates. Holds because the
/// measured results are planar except for a deliberately small spherical
/// cap (see the module docs).
const E2E_VOLUME_RTOL: f64 = 1e-6;

fn tol() -> ToleranceContext {
    ToleranceContext::default()
}

/// Import STEP text into fresh stores.
fn import(source: &str) -> (TopologyStore, GeometryStore, StepImport) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step(source, &mut store, &mut geo, &StepReadOptions::default())
        .expect("source must be syntactically valid Part 21");
    (store, geo, report)
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

/// Closed-manifold volume of a boolean output via its CDT tessellation
/// (handles trimmed quadric faces, unlike the standalone tessellator).
fn boolean_volume(out: &BooleanOutput, context: &str) -> f64 {
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
    mass_properties(&mesh)
        .unwrap_or_else(|e| panic!("{context}: mass_properties failed: {e}"))
        .volume
}

fn assert_volume_close(got: f64, expected: f64, rtol: f64, context: &str) {
    let rel = (got - expected).abs() / expected.abs().max(f64::MIN_POSITIVE);
    assert!(
        rel <= rtol,
        "{context}: volume {got} differs from analytic {expected} by {rel:e} (allowed {rtol:e})"
    );
}

/// Spherical cap of height `h` cut from a sphere of radius `r`.
fn spherical_cap_volume(r: f64, h: f64) -> f64 {
    PI * h * h * (3.0 * r - h) / 3.0
}

/// Volume of the part of a torus (axis +Z, tube center plane z = 0) below
/// the plane `z = c`, for `|c| <= minor`.
fn torus_below_plane_volume(major: f64, minor: f64, c: f64) -> f64 {
    let r = minor;
    let c = c.clamp(-r, r);
    let integral =
        (r * r / 2.0) * ((c / r).asin() + FRAC_PI_2) + (c / 2.0) * (r * r - c * c).sqrt();
    4.0 * PI * major * integral
}

/// How strong a `write ∘ read` byte gate the round trip must clear.
#[derive(Clone, Copy, PartialEq)]
enum FixedPointGate {
    /// The second write must reproduce the first file byte for byte:
    /// the re-imported body is the exported one down to every `f64`.
    ByteIdentical,
    /// Byte-identical immediately, or at worst from the second write
    /// (of-kb8: re-imported stores duplicate geometry shared across
    /// faces/edges, shifting the fixed point out by one iteration).
    Stabilizes,
    /// No byte gate: bodies with non-axis-aligned plane normals oscillate
    /// in the last ULP between writes and never reach a byte fixed point
    /// (of-9qi). Topology, check, and caller-side volume gates still apply.
    UlpOscillation,
}

/// Write `body`, re-import it, and require: no error diagnostics, an exact
/// B-Rep outcome, a clean check, identical Euler counts, and the requested
/// [`FixedPointGate`]. Returns the re-imported stores and body.
fn round_trip(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    gate: FixedPointGate,
    context: &str,
) -> (TopologyStore, GeometryStore, EntityId<Body>) {
    assert!(
        store.check(body).is_empty(),
        "{context}: body must pass check before export: {:?}",
        store.check(body)
    );
    let text = write_step(store, geo, &[body], &StepWriteOptions::default())
        .unwrap_or_else(|e| panic!("{context}: body must serialize to STEP: {e}"));

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

    match gate {
        FixedPointGate::UlpOscillation => {}
        FixedPointGate::ByteIdentical | FixedPointGate::Stabilizes => {
            let text2 = write_step(&store2, &geo2, &[body2], &StepWriteOptions::default())
                .unwrap_or_else(|e| panic!("{context}: re-imported body must serialize: {e}"));
            if gate == FixedPointGate::ByteIdentical {
                assert_eq!(
                    text, text2,
                    "{context}: write ∘ read must be a byte-identical fixed point"
                );
            } else if text != text2 {
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

    (store2, geo2, body2)
}

/// STEP-import a freshly exported 6×6×2 block and bite a shallow sphere
/// cap (radius `r`, cap height `h`) out of the middle of its top face with
/// an exact B-Rep subtract. Shared setup for the volume tests below.
fn imported_block_minus_sphere_cap(r: f64, h: f64) -> BooleanOutput {
    let block_text = {
        let mut s = TopologyStore::new();
        let mut g = GeometryStore::new();
        let b = primitives::block(&mut s, &mut g, 6.0, 6.0, 2.0).expect("block");
        write_step(&s, &g, &[b], &StepWriteOptions::default()).expect("block serializes")
    };
    let (mut store, mut geo, report) = import(&block_text);
    assert!(!report.has_errors(), "{:?}", report.diagnostics);
    let block = only_brep(&report);

    let sphere = primitives::sphere(&mut store, &mut geo, r).expect("sphere");
    translate_body(
        &mut store,
        &mut geo,
        sphere,
        Vector3::new(0.0, 0.0, 1.0 + r - h),
    )
    .expect("translate sphere");

    subtract(&store, &geo, block, sphere, &tol()).expect("exact subtract")
}

// ---------------------------------------------------------------------
// 1. STEP block → exact sphere subtract → STEP → re-import → volume
// ---------------------------------------------------------------------

/// The full STEP + sphere-boolean chain, volume-gated at 1e-6 relative:
/// a block arrives as STEP text, an exact B-Rep sphere subtract bites a
/// shallow cap out of its top face, and the result — measured against the
/// closed-form volume — survives export and re-import as a byte-identical
/// fixed point, which pins the re-imported solid (and hence its enclosed
/// volume) to the measured body exactly.
#[test]
fn step_block_minus_sphere_reimports_with_analytic_volume() {
    let context = "STEP block − sphere round trip";
    let (r, h) = (0.3, 0.05);
    let out = imported_block_minus_sphere_cap(r, h);

    let analytic = 6.0 * 6.0 * 2.0 - spherical_cap_volume(r, h);
    assert_volume_close(
        boolean_volume(&out, context),
        analytic,
        E2E_VOLUME_RTOL,
        context,
    );

    // Byte-identical export/import: the re-imported body is this geometry
    // down to every f64, so it encloses the volume just measured.
    round_trip(
        &out.store,
        &out.geo,
        out.body,
        FixedPointGate::ByteIdentical,
        context,
    );
}

/// Direct volume measurement of the re-imported solid, by using it as an
/// operand of a second exact boolean (a corner nibble far from the dimple)
/// and measuring that output's CDT tessellation. Blocked on of-6cf: a
/// second boolean on a body carrying circular-arc edges passes `check()`
/// but tessellates non-manifold — even without the STEP trip.
#[test]
#[ignore = "of-6cf: chained boolean on a curved-edge body tessellates non-manifold"]
fn reimported_step_solid_works_as_boolean_operand() {
    let context = "re-imported STEP solid as boolean operand";
    let (r, h) = (0.3, 0.05);
    let out = imported_block_minus_sphere_cap(r, h);

    let (mut store2, mut geo2, body2) = round_trip(
        &out.store,
        &out.geo,
        out.body,
        FixedPointGate::ByteIdentical,
        context,
    );

    let nibble = primitives::block(&mut store2, &mut geo2, 1.0, 1.0, 1.0).expect("nibble");
    translate_body(&mut store2, &mut geo2, nibble, Vector3::new(3.0, 3.0, 1.0))
        .expect("translate nibble");
    let out2 = subtract(&store2, &geo2, body2, nibble, &tol())
        .expect("exact subtract on the re-imported body");
    // The nibble block overlaps the corner octant 0.5 × 0.5 × 0.5.
    let analytic = 6.0 * 6.0 * 2.0 - spherical_cap_volume(r, h) - 0.125;
    assert_volume_close(
        boolean_volume(&out2, context),
        analytic,
        E2E_VOLUME_RTOL,
        context,
    );
}

// ---------------------------------------------------------------------
// 2. Hybrid boolean with a torus operand → STEP round trip
// ---------------------------------------------------------------------

/// A torus sunk into a slab, subtracted through the hybrid front door:
/// both operands are exact B-Reps, the sphere/torus chart promotion
/// (of-7ld.4) must carry the torus through the exact pipeline, and the
/// winning exact result must survive a STEP write/read round trip.
#[test]
fn hybrid_torus_subtract_round_trips_through_step() {
    let context = "hybrid slab − torus STEP round trip";

    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let slab = primitives::block(&mut store, &mut geo, 12.0, 12.0, 4.0).expect("slab");
    translate_body(&mut store, &mut geo, slab, Vector3::new(0.0, 0.0, -2.0))
        .expect("translate slab");
    let (major, minor, drop) = (2.0, 0.5, 0.2);
    let ring = primitives::torus(&mut store, &mut geo, major, minor).expect("torus");
    translate_body(&mut store, &mut geo, ring, Vector3::new(0.0, 0.0, -drop))
        .expect("translate torus");

    let out = hybrid::subtract(
        &HybridBody::brep(&store, &geo, slab),
        &HybridBody::brep(&store, &geo, ring),
        &HybridOptions::default(),
    )
    .expect("hybrid subtract");

    assert!(
        out.diagnostic.is_none(),
        "{context}: validation gate must accept the exact result: {:?}",
        out.diagnostic
    );
    let HybridPath::Brep(exact) = &out.path else {
        panic!("{context}: torus operand must take the exact B-Rep path (of-7ld.4)");
    };

    assert!(
        out.mesh.is_closed_manifold(),
        "{context}: hybrid mesh must be watertight"
    );
    let below = torus_below_plane_volume(major, minor, drop);
    let analytic = 12.0 * 12.0 * 4.0 - below;
    // The groove wall is a large trimmed torus patch meshed at the default
    // angular step, so this gate uses the stress-suite budget, not 1e-6.
    assert_volume_close(
        boolean_volume(exact, context),
        analytic,
        5e-3,
        &format!("{context}: exact result vs analytic"),
    );

    // The exact result (planes + trimmed torus faces) must round-trip
    // through STEP with identical topology.
    round_trip(
        &exact.store,
        &exact.geo,
        exact.body,
        FixedPointGate::Stabilizes,
        context,
    );
}

// ---------------------------------------------------------------------
// 3. Sketch-extrude body → STEP round trip
// ---------------------------------------------------------------------

/// A sketch-extrude body reaches STEP through the faceted-conversion
/// pipeline: profile → [`extrude`] → tessellation → [`MeshSdf`] →
/// [`sdf_to_brep`] → [`write_step`] → [`read_step`]. Sweep topology keeps
/// placeholder geometry slots (binding analytic geometry is of-xyx), so
/// the faceted bridge is the supported export path today. The recovered
/// body is all-planar with arbitrary face normals, which puts it in
/// of-9qi territory: the round trip preserves topology and volume but not
/// the exact file bytes.
#[test]
fn sketch_extrude_body_exports_and_reimports_through_step() {
    let context = "sketch-extrude STEP round trip";

    // Convex trapezoid sketch in the XY plane, extruded straight up.
    // Convexity keeps the cap fans of `SweptBody::tessellate` exact, so
    // the mesh is a faithful boundary of the prism.
    let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
    let corners = [p(0.0, 0.0), p(4.0, 0.0), p(3.0, 2.0), p(1.0, 2.0)];
    let segments: Vec<ProfileSegment> = (0..corners.len())
        .map(|i| ProfileSegment::line_between(corners[i], corners[(i + 1) % corners.len()]))
        .collect::<Result<_, _>>()
        .expect("trapezoid segments");
    let sketch = Profile::new(segments).expect("trapezoid profile");
    let prism = extrude(&sketch, Vector3::new(0.0, 0.0, 2.0)).expect("extrude");
    assert!(prism.check().is_empty(), "{context}: swept body must check");

    // Trapezoid area ((4 + 2) / 2) · 2 = 6, height 2.
    let analytic = 12.0;
    let mesh = prism.tessellate(8).expect("swept body tessellates");
    assert!(mesh.is_closed_manifold(), "{context}: sweep mesh closed");
    let mesh_volume = mass_properties(&mesh).expect("mass properties").volume;
    assert_volume_close(
        mesh_volume,
        analytic,
        1e-9,
        &format!("{context}: sweep mesh"),
    );

    // Faceted bridge: the prism's boundary as an SDF, recovered as an
    // all-planar B-Rep body with real geometry attached.
    let sdf = MeshSdf::new(&mesh).expect("prism mesh wraps as an SDF");
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    // Sampling cube strictly containing the prism ([0,4] × [0,2] × [0,2]).
    let bounds = BoundingBox3::new(Point3::new(-1.0, -2.0, -2.0), Point3::new(5.0, 4.0, 4.0));
    let body = sdf_to_brep(
        &sdf,
        &mut store,
        &mut geo,
        &SdfToBrepOptions::new(bounds, 6),
    )
    .expect("prism recovers a faceted B-Rep");
    assert!(
        store.check(body).is_empty(),
        "{context}: faceted body checks"
    );

    let faceted_mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default())
        .expect("faceted body tessellates");
    let faceted_volume = mass_properties(&faceted_mesh)
        .expect("mass properties")
        .volume;
    assert_volume_close(
        faceted_volume,
        analytic,
        0.03,
        &format!("{context}: faceted recovery"),
    );

    // STEP round trip of the faceted body, with the re-imported side's
    // volume equal to the exported side's (all faces planar, so the
    // standalone tessellator measures both sides; geometric drift across
    // the trip is at most one ULP per coordinate — of-9qi).
    let (store2, geo2, body2) =
        round_trip(&store, &geo, body, FixedPointGate::UlpOscillation, context);
    let mesh2 = tessellate_body(&store2, &geo2, body2, &TessellationOptions::default())
        .expect("re-imported body tessellates");
    let volume2 = mass_properties(&mesh2).expect("mass properties").volume;
    assert_volume_close(
        volume2,
        faceted_volume,
        1e-9,
        &format!("{context}: re-import volume drift"),
    );
}
