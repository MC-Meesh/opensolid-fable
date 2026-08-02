//! Adversarial campaign for the heal-scale full-circle seam bound (of-ce7c,
//! pairing of-5rnp).
//!
//! of-5rnp moved [`trim_curve`]'s "two distinct seam vertices are really the
//! same point" test off the declared closure and onto `seam_tol`: the
//! `HEAL_GAP_REL` ratio (1e-5) over the solid's model scale. (As first
//! landed that scale was the origin-distance proxy `trim_tol` uses; of-85rt
//! — found by this campaign — replaced it with the vertex bounding-box
//! diagonal heal's own merge measures, clamped like heal's, so the bound is
//! translation invariant.) This campaign attacks the boundary itself rather
//! than re-proving the of-5rnp repro:
//!
//! - **Arcs just above the bound must read literally** — exact, checker
//!   clean, correct volume, across randomized radii, heights and chords.
//! - **The collapse cliff must sit at the bound**, flip once, and never
//!   pass through a silently-wrong volume on the way.
//! - **A declared closure must not move the cliff in either direction** —
//!   neither a loose closure widening it (of-5rnp's bug) nor a tight
//!   closure narrowing it.
//! - **The bound must be a model-scale bound, not a placement bound** — a
//!   translated part must keep the reading, and the exact volume, it has at
//!   the origin, because the heal merge the bound mirrors uses the
//!   translation-invariant body diagonal.
//! - **Round-trip volume certification** — everything that imports exactly
//!   must measure the authored volume and survive write→read unchanged.
//!
//! Protocol as `step_closure_adversarial.rs`: deterministic seeded [`Rng`]
//! (remixed by `OPENSOLID_CAMPAIGN_SEED`), a repro string on every
//! assertion, found failures become `bd` beads and their repro is
//! `#[ignore]`d referencing the bead — never softened into a passing test.
//!
//! # Failures this campaign found (both deterministic, no seed required)
//!
//! - **of-85rt** (P1, fixed): `seam_tol`'s original origin-distance proxy
//!   resurrected the of-5rnp corruption for off-origin parts. The identical
//!   clean sector (r = 1.6, chord 8e-3, closure 9e-3) that imported
//!   correctly at the origin imported *checker-clean at 24.1466 mm³ vs
//!   authored 0.0192 mm³* — the same 1257× silent corruption — at
//!   `cx = 1200`; without a declaration it silently lost its exact import
//!   there. Heal's merge gap is diagonal-based and clamped; `seam_tol` was
//!   neither, and now is both. Repros (kept as regression tests):
//!   [`translating_the_of_5rnp_sector_must_not_resurrect_the_collapse`],
//!   [`translating_a_clean_short_arc_sector_must_not_cost_its_exact_import`].
//! - **of-a5me** (P3): inside the bound the ambiguity window is not merely
//!   a degraded import — with a covering closure a genuine sub-bound arc
//!   (chord 4.2e-5 under a 6e-5 bound, closure 1e-3) imports checker-clean
//!   at 235.62 mm³ vs authored 3.15e-4 mm³ (~750,000×). Loop context (the
//!   arc's endpoints each anchor a radial wall) disambiguates what the
//!   local trim decision cannot. Repro:
//!   [`a_sub_bound_arc_under_a_covering_closure_must_not_be_silently_rewritten`].

use opensolid_brep::{GeometryStore, TopologyStore};
use opensolid_kernel::brep_mass_properties;
use opensolid_kernel::io::step::read::{SolidOutcome, StepImport, StepReadOptions, read_step};
use opensolid_kernel::io::step::write::{StepWriteOptions, write_step};
use std::f64::consts::PI;

/// `read.rs`'s seam-identity ratio (`HEAL_GAP_REL`), restated so the
/// campaign can aim chords at the bound.
const SEAM_TOL_REL: f64 = 1e-5;

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `step_closure_adversarial.rs`.
// ---------------------------------------------------------------------

/// Campaign remix (of-5rim): `OPENSOLID_CAMPAIGN_SEED=<hex>` XORs every suite
/// seed so the same properties walk fresh configurations each run. Unset, the
/// suite is byte-for-byte deterministic.
fn campaign_seed() -> u64 {
    static MIX: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    *MIX.get_or_init(|| match std::env::var("OPENSOLID_CAMPAIGN_SEED") {
        Ok(raw) => {
            let hex = raw.trim();
            let hex = hex.strip_prefix("0x").unwrap_or(hex);
            u64::from_str_radix(&hex.replace('_', ""), 16)
                .unwrap_or_else(|_| panic!("OPENSOLID_CAMPAIGN_SEED must be hex, got {raw:?}"))
        }
        Err(_) => 0,
    })
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed ^ campaign_seed())
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn pick(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

// ---------------------------------------------------------------------
// Fixtures
// ---------------------------------------------------------------------

/// Wrap DATA-section body text in a minimal Part 21 envelope.
fn wrap(data: &str) -> String {
    format!(
        "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\n\
         DATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
    )
}

/// The units / representation-context tail (millimetres), optionally
/// declaring `closure_mm` as a `GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT`.
fn context_tail(msb: u64, closure_mm: Option<f64>) -> String {
    let (uncertainty, in_parts) = match closure_mm {
        Some(closure) => (
            format!(
                "#9120 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({closure:.12E}),#9100,\
                 'distance_accuracy_value','maximum model space distance between \
                 geometric entities at asserted connectivities');\n"
            ),
            "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#9120)) ",
        ),
        None => (String::new(), ""),
    };
    format!(
        "#9100 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
         #9101 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
         #9102 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n\
         {uncertainty}\
         #9110 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) {in_parts}\
         GLOBAL_UNIT_ASSIGNED_CONTEXT((#9100,#9101,#9102)) \
         REPRESENTATION_CONTEXT('','3D Context') );\n\
         #9111 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{msb}),#9110);"
    )
}

/// An AP203 cylinder *sector* (generalizing `step_closure_adversarial.rs`'s
/// origin-anchored one): the pie slice of a cylinder of radius `r`, height
/// `h`, swept from angle `0` to `sweep`, with its axis translated to
/// `(cx, 0, *)`. Five faces, two genuinely short `CIRCLE` arcs of chord
/// `2 r sin(sweep/2)`, every vertex exactly on every curve that names it.
/// Millimetres, optionally declaring `closure_mm`.
fn offset_cylinder_sector(cx: f64, r: f64, h: f64, sweep: f64, closure_mm: Option<f64>) -> String {
    let (c, sn) = (sweep.cos(), sweep.sin());
    let (ax, ay) = (cx + r * c, r * sn);
    let rx = cx + r;
    // Outward normal of the swept-side flat wall.
    let (nx, ny) = (-sn, c);
    let body = format!(
        "\
#1 = CARTESIAN_POINT('', ({cx:.12}, 0., 0.));
#2 = CARTESIAN_POINT('', ({cx:.12}, 0., {h:.12}));
#3 = CARTESIAN_POINT('', ({rx:.12}, 0., 0.));
#4 = CARTESIAN_POINT('', ({rx:.12}, 0., {h:.12}));
#5 = CARTESIAN_POINT('', ({ax:.12}, {ay:.12}, 0.));
#6 = CARTESIAN_POINT('', ({ax:.12}, {ay:.12}, {h:.12}));
#7 = DIRECTION('', (0., 0., 1.));
#8 = DIRECTION('', (0., 0., -1.));
#9 = DIRECTION('', (1., 0., 0.));
#10 = DIRECTION('', (0., -1., 0.));
#11 = DIRECTION('', ({c:.12}, {sn:.12}, 0.));
#12 = DIRECTION('', ({nx:.12}, {ny:.12}, 0.));
#13 = VERTEX_POINT('', #1);
#14 = VERTEX_POINT('', #2);
#15 = VERTEX_POINT('', #3);
#16 = VERTEX_POINT('', #4);
#17 = VERTEX_POINT('', #5);
#18 = VERTEX_POINT('', #6);
#19 = AXIS2_PLACEMENT_3D('', #1, #7, #9);
#20 = AXIS2_PLACEMENT_3D('', #2, #7, #9);
#21 = AXIS2_PLACEMENT_3D('', #1, #8, #9);
#22 = AXIS2_PLACEMENT_3D('', #1, #10, #9);
#23 = AXIS2_PLACEMENT_3D('', #1, #12, #11);
#24 = CIRCLE('', #19, {r:.12});
#25 = CIRCLE('', #20, {r:.12});
#26 = VECTOR('', #7, 1.);
#27 = VECTOR('', #9, 1.);
#28 = VECTOR('', #11, 1.);
#29 = LINE('', #1, #26);
#30 = LINE('', #1, #27);
#31 = LINE('', #1, #28);
#32 = LINE('', #2, #27);
#33 = LINE('', #2, #28);
#34 = LINE('', #3, #26);
#35 = LINE('', #5, #26);
#36 = EDGE_CURVE('', #13, #14, #29, .T.);
#37 = EDGE_CURVE('', #13, #15, #30, .T.);
#38 = EDGE_CURVE('', #13, #17, #31, .T.);
#39 = EDGE_CURVE('', #14, #16, #32, .T.);
#40 = EDGE_CURVE('', #14, #18, #33, .T.);
#41 = EDGE_CURVE('', #15, #16, #34, .T.);
#42 = EDGE_CURVE('', #17, #18, #35, .T.);
#43 = EDGE_CURVE('', #15, #17, #24, .T.);
#44 = EDGE_CURVE('', #16, #18, #25, .T.);
#45 = PLANE('', #21);
#46 = PLANE('', #20);
#47 = PLANE('', #22);
#48 = PLANE('', #23);
#49 = CYLINDRICAL_SURFACE('', #19, {r:.12});
#50 = ORIENTED_EDGE('', *, *, #38, .T.);
#51 = ORIENTED_EDGE('', *, *, #43, .F.);
#52 = ORIENTED_EDGE('', *, *, #37, .F.);
#53 = EDGE_LOOP('', (#50, #51, #52));
#54 = FACE_OUTER_BOUND('', #53, .T.);
#55 = ADVANCED_FACE('', (#54), #45, .T.);
#56 = ORIENTED_EDGE('', *, *, #39, .T.);
#57 = ORIENTED_EDGE('', *, *, #44, .T.);
#58 = ORIENTED_EDGE('', *, *, #40, .F.);
#59 = EDGE_LOOP('', (#56, #57, #58));
#60 = FACE_OUTER_BOUND('', #59, .T.);
#61 = ADVANCED_FACE('', (#60), #46, .T.);
#62 = ORIENTED_EDGE('', *, *, #37, .T.);
#63 = ORIENTED_EDGE('', *, *, #41, .T.);
#64 = ORIENTED_EDGE('', *, *, #39, .F.);
#65 = ORIENTED_EDGE('', *, *, #36, .F.);
#66 = EDGE_LOOP('', (#62, #63, #64, #65));
#67 = FACE_OUTER_BOUND('', #66, .T.);
#68 = ADVANCED_FACE('', (#67), #47, .T.);
#69 = ORIENTED_EDGE('', *, *, #36, .T.);
#70 = ORIENTED_EDGE('', *, *, #40, .T.);
#71 = ORIENTED_EDGE('', *, *, #42, .F.);
#72 = ORIENTED_EDGE('', *, *, #38, .F.);
#73 = EDGE_LOOP('', (#69, #70, #71, #72));
#74 = FACE_OUTER_BOUND('', #73, .T.);
#75 = ADVANCED_FACE('', (#74), #48, .T.);
#76 = ORIENTED_EDGE('', *, *, #43, .T.);
#77 = ORIENTED_EDGE('', *, *, #42, .T.);
#78 = ORIENTED_EDGE('', *, *, #44, .F.);
#79 = ORIENTED_EDGE('', *, *, #41, .F.);
#80 = EDGE_LOOP('', (#76, #77, #78, #79));
#81 = FACE_OUTER_BOUND('', #80, .T.);
#82 = ADVANCED_FACE('', (#81), #49, .T.);
#83 = CLOSED_SHELL('', (#55, #61, #68, #75, #82));
#84 = MANIFOLD_SOLID_BREP('sector', #83);
{tail}",
        tail = context_tail(84, closure_mm),
    );
    wrap(&body)
}

/// An AP203 cylinder (radius `r`, height `h`, axis through the origin)
/// whose *bottom* full-circle edge uses OCC's two-distinct-`VERTEX_POINT`
/// seam spelling: the second seam vertex sits exactly on the circle, swept
/// `gap` along it from the first, so only the seam identity — never a
/// vertex-off-curve miss — is at stake.
fn two_vertex_seam_cylinder(r: f64, h: f64, gap: f64, closure_mm: Option<f64>) -> String {
    let (lo, hi) = (-h / 2.0, h / 2.0);
    let phi = gap / r;
    let (sx, sy) = (r * phi.cos(), r * phi.sin());
    let body = format!(
        "\
#1 = CARTESIAN_POINT('', (0., 0., {lo:.12}));
#2 = CARTESIAN_POINT('', (0., 0., {hi:.12}));
#3 = CARTESIAN_POINT('', ({r:.12}, 0., {lo:.12}));
#4 = CARTESIAN_POINT('', ({r:.12}, 0., {hi:.12}));
#5 = DIRECTION('', (0., 0., 1.));
#6 = DIRECTION('', (1., 0., 0.));
#7 = DIRECTION('', (0., 0., -1.));
#8 = VERTEX_POINT('', #3);
#9 = VERTEX_POINT('', #4);
#10 = AXIS2_PLACEMENT_3D('', #1, #5, #6);
#11 = AXIS2_PLACEMENT_3D('', #2, #5, #6);
#12 = AXIS2_PLACEMENT_3D('', #1, #7, #6);
#13 = CIRCLE('', #10, {r:.12});
#14 = CIRCLE('', #11, {r:.12});
#15 = VECTOR('', #5, 1.);
#16 = LINE('', #3, #15);
#41 = CARTESIAN_POINT('', ({sx:.12}, {sy:.12}, {lo:.12}));
#42 = VERTEX_POINT('', #41);
#17 = EDGE_CURVE('', #8, #42, #13, .T.);
#18 = EDGE_CURVE('', #9, #9, #14, .T.);
#19 = EDGE_CURVE('', #8, #9, #16, .T.);
#20 = PLANE('', #12);
#21 = PLANE('', #11);
#22 = CYLINDRICAL_SURFACE('', #10, {r:.12});
#23 = ORIENTED_EDGE('', *, *, #17, .F.);
#24 = EDGE_LOOP('', (#23));
#25 = FACE_OUTER_BOUND('', #24, .T.);
#26 = ADVANCED_FACE('', (#25), #20, .T.);
#27 = ORIENTED_EDGE('', *, *, #18, .T.);
#28 = EDGE_LOOP('', (#27));
#29 = FACE_OUTER_BOUND('', #28, .T.);
#30 = ADVANCED_FACE('', (#29), #21, .T.);
#31 = ORIENTED_EDGE('', *, *, #17, .T.);
#32 = ORIENTED_EDGE('', *, *, #19, .T.);
#33 = ORIENTED_EDGE('', *, *, #18, .F.);
#34 = ORIENTED_EDGE('', *, *, #19, .F.);
#35 = EDGE_LOOP('', (#31, #32, #33, #34));
#36 = FACE_OUTER_BOUND('', #35, .T.);
#37 = ADVANCED_FACE('', (#36), #22, .T.);
#38 = CLOSED_SHELL('', (#26, #30, #37));
#39 = MANIFOLD_SOLID_BREP('cyl', #38);
{tail}",
        tail = context_tail(39, closure_mm),
    );
    wrap(&body)
}

// ---------------------------------------------------------------------
// The bound, restated per arc
// ---------------------------------------------------------------------

/// The seam-identity bound `trim_curve` applies to *both* arcs of an offset
/// sector: `SEAM_TOL_REL` × the diagonal of the sector's vertex bounding
/// box, which is translation invariant — `cx` does not appear (of-85rt).
/// The sliver's y extent (≈ the chord, from the swept arc endpoints) shifts
/// the diagonal by under one part in 10⁹ at every chord this campaign aims,
/// so it is omitted.
fn seam_bound(r: f64, h: f64) -> f64 {
    SEAM_TOL_REL * (r * r + h * h).sqrt()
}

/// The sweep whose chord is `chord`.
fn sweep_for_chord(r: f64, chord: f64) -> f64 {
    2.0 * (chord / (2.0 * r)).asin()
}

/// The authored volume of the sector.
fn sector_volume(r: f64, h: f64, sweep: f64) -> f64 {
    sweep / 2.0 * r * r * h
}

// ---------------------------------------------------------------------
// Import and classification
// ---------------------------------------------------------------------

fn import(src: &str, context: &str) -> (TopologyStore, GeometryStore, StepImport) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step(src, &mut store, &mut geo, &StepReadOptions::default())
        .unwrap_or_else(|e| panic!("{context}: the fixture must PARSE: {e:?}"));
    assert_eq!(
        report.solids.len(),
        1,
        "{context}: expected exactly one solid, got {}",
        report.solids.len()
    );
    (store, geo, report)
}

/// The exact body's volume when the import is an exact, *geometrically
/// checked* B-Rep; `None` when it degraded or failed. Panics if the exact
/// path hands back a body the checker then rejects — an import must never
/// certify what `check_with_geometry` refuses.
fn exact_checked_volume(src: &str, context: &str) -> Option<f64> {
    let (store, geo, report) = import(src, context);
    match report.solids[0].outcome {
        SolidOutcome::BRep(body) => {
            let failures = store.check_with_geometry(&geo, body);
            assert!(
                failures.is_empty(),
                "{context}: an EXACT import must be geometrically valid, but \
                 check_with_geometry() reported {} failures: {:#?}",
                failures.len(),
                failures
            );
            Some(
                brep_mass_properties(&store, &geo, body)
                    .unwrap_or_else(|e| panic!("{context}: measurement failed: {e:?}"))
                    .volume,
            )
        }
        _ => None,
    }
}

fn assert_within(got: f64, want: f64, rtol: f64, context: &str) {
    let scale = want.abs().max(1e-300);
    assert!(
        ((got - want) / scale).abs() <= rtol,
        "{context}: got {got}, expected {want} \
         ({:.3e} relative, allowed {rtol:.1e})",
        ((got - want) / scale).abs()
    );
}

/// The volume certification every case in this campaign must satisfy: an
/// exact import measures the authored volume; a degraded or refused import
/// is honest. Only a *checker-clean exact body at the wrong volume* — a
/// silently-misread solid — is a failure.
fn assert_never_silently_wrong(src: &str, want: f64, context: &str) {
    if let Some(volume) = exact_checked_volume(src, context) {
        assert_within(volume, want, 1.0e-6, context);
    }
}

// =====================================================================
// (0) Baseline: the offset sector fixture is sound
// =====================================================================

/// An ordinary-sweep sector must import exactly at every offset this
/// campaign uses — otherwise the translation attacks below would be
/// measuring fixture defects, not the seam bound.
#[test]
fn an_offset_sector_at_an_ordinary_sweep_imports_exactly() {
    let (r, h, sweep) = (1.6, 3.0, 0.6);
    for cx in [0.0, 300.0, 1200.0] {
        for closure in [None, Some(9.0e-3)] {
            let repro = format!("sector(cx = {cx}, r = {r}, h = {h}, sweep = {sweep}), {closure:?}");
            let volume =
                exact_checked_volume(&offset_cylinder_sector(cx, r, h, sweep, closure), &repro)
                    .unwrap_or_else(|| panic!("{repro}: the clean sector must import exactly"));
            assert_within(volume, sector_volume(r, h, sweep), 1.0e-6, &repro);
        }
    }
}

// =====================================================================
// (1) Chords above the bound read literally
// =====================================================================

/// Randomized radii, heights, near-origin offsets and chords from just
/// above the bound (1.001×) to three decades above it: the arc must read
/// literally — exact, checker clean, authored volume — with or without a
/// declaration. The 0.1% margin is three decades above the fixture's
/// 12-decimal print round-off, so print noise cannot decide a case.
#[test]
fn a_chord_just_above_the_bound_reads_literally() {
    let mut rng = Rng::new(0x_5EA3_B0DD_0001);
    for case in 0..24 {
        let r = rng.range(1.0, 8.0);
        let h = rng.range(2.0, 12.0);
        let cx = rng.range(0.0, 40.0);
        let bound = seam_bound(r, h);
        // Log-uniform in [1.001, 1000] × the bound, capped well under the
        // radius so the sweep stays a genuine sliver.
        let factor = 10.0f64.powf(rng.range(0.000434, 3.0));
        let chord = (bound * factor).min(0.4 * r);
        let closure = match rng.pick(3) {
            0 => None,
            1 => Some(9.0e-3),
            _ => Some(1.0e-3),
        };
        let sweep = sweep_for_chord(r, chord);
        let repro = format!(
            "case {case}: sector(cx = {cx:.6}, r = {r:.6}, h = {h:.6}), chord {chord:.6e} \
             = {factor:.3}x the bound {bound:.6e}, closure {closure:?}"
        );
        let text = offset_cylinder_sector(cx, r, h, sweep, closure);
        let volume = exact_checked_volume(&text, &repro).unwrap_or_else(|| {
            panic!("{repro}: an arc above the seam bound must import exactly")
        });
        assert_within(volume, sector_volume(r, h, sweep), 1.0e-6, &repro);
    }
}

// =====================================================================
// (2) The collapse cliff sits at the bound and is never silently wrong
// =====================================================================

/// Scan chords across the bound (no declaration). Both arcs see the one
/// vertex-bbox-diagonal bound — placement and z height no longer split them
/// into a mixed zone — so below it both collapse and above it both read
/// literally. The literal side must import exactly at the authored volume;
/// below the bound the import must be honest — `verify_trim` refuses the
/// collapsed circle's chord-sized end miss, so no exact body may appear.
/// Acceptance must flip exactly once, at the bound.
#[test]
fn the_collapse_cliff_sits_at_the_seam_bound() {
    let (r, h) = (5.0, 3.0);
    let bound = seam_bound(r, h);
    let mut seen_exact = false;
    for (zone, chord) in [
        ("collapse", 0.5 * bound),
        ("collapse", 0.999 * bound),
        ("literal", 1.001 * bound),
        ("literal", 1.05 * bound),
        ("literal", 2.0 * bound),
    ] {
        let sweep = sweep_for_chord(r, chord);
        let repro = format!(
            "sector(r = {r}, h = {h}), chord {chord:.6e} ({zone}; bound \
             {bound:.6e}), no declaration"
        );
        let text = offset_cylinder_sector(0.0, r, h, sweep, None);
        let expected = sector_volume(r, h, sweep);
        let exact = exact_checked_volume(&text, &repro);
        if let Some(volume) = exact {
            assert_within(volume, expected, 1.0e-6, &repro);
        }
        let literal = zone == "literal";
        assert_eq!(
            exact.is_some(),
            literal,
            "{repro}: expected {}",
            if literal {
                "a literal exact import"
            } else {
                "an honest refusal of the collapsed arc"
            }
        );
        assert!(
            !seen_exact || exact.is_some(),
            "{repro}: acceptance must flip exactly once, monotonically"
        );
        seen_exact = exact.is_some();
    }
}

// =====================================================================
// (3) A declared closure must not move the cliff
// =====================================================================

/// The of-5rnp assertion, generalized: a closure that *covers the chord* —
/// the exact configuration that used to rewrite the arc into a full circle —
/// must not widen the seam bound. Randomized radii and chords between the
/// bound and the declaration, including the corpus-real thousandth-of-an-inch
/// declaration (2.54e-2, clamped to the kernel limit).
#[test]
fn a_loose_closure_must_not_widen_the_seam_bound() {
    let mut rng = Rng::new(0x_5EA3_B0DD_0003);
    for case in 0..24 {
        let r = rng.range(1.0, 8.0);
        let h = rng.range(2.0, 12.0);
        let closure = if rng.pick(2) == 0 { 9.0e-3 } else { 2.54e-2 };
        let bound = seam_bound(r, h);
        // Between the bound and the declaration: covered by the closure,
        // above the seam bound — the collapse would be silent corruption.
        let chord = rng.range(1.05 * bound, 8.0e-3);
        let sweep = sweep_for_chord(r, chord);
        let repro = format!(
            "case {case}: sector(r = {r:.6}, h = {h:.6}), chord {chord:.6e} above \
             bound {bound:.6e}, declared closure {closure:.3e} covers the chord"
        );
        let text = offset_cylinder_sector(0.0, r, h, sweep, Some(closure));
        let volume = exact_checked_volume(&text, &repro).unwrap_or_else(|| {
            panic!("{repro}: the closure must not decide seam identity (of-5rnp)")
        });
        assert_within(volume, sector_volume(r, h, sweep), 1.0e-6, &repro);
    }
}

/// The other direction: a closure *tighter than the gap* must not narrow
/// the bound either. A two-vertex seam whose gap sits inside both the seam
/// bound and the round-off rule, in a file declaring a 1e-9 closure — the
/// declaration calls the two vertices distinct, but seam identity is
/// `seam_tol`'s question and the cylinder must still import whole.
#[test]
fn a_tight_closure_must_not_narrow_the_seam_bound() {
    let (r, h, gap) = (5.0f64, 8.0f64, 5.0e-6f64);
    let repro = format!(
        "two-vertex seam cylinder(r = {r}, h = {h}), gap {gap:.1e} inside the seam \
         bound {:.3e}, declared closure 1e-9",
        // The fixture's three vertices all sit on the seam: its bbox is the
        // gap wide and the full height tall.
        SEAM_TOL_REL * (h * h + gap * gap).sqrt()
    );
    let text = two_vertex_seam_cylinder(r, h, gap, Some(1.0e-9));
    let volume = exact_checked_volume(&text, &repro)
        .unwrap_or_else(|| panic!("{repro}: the seam must still collapse and import"));
    assert_within(volume, PI * r * r * h, 1.0e-4, &repro);
}

/// Inside the bound the collapse is the designed reading — but it must
/// never *certify* a wrong volume. With a covering closure, `verify_trim`
/// accepts the collapsed circle's chord-sized end miss and the fabricated
/// body imports checker-clean at full-circle volume.
///
/// FOUND FAILING, filed as **of-a5me** (P3): sector r = 5, h = 3, chord
/// 4.2e-5 under the 6e-5 bound, closure 1e-3 → exact, checker clean,
/// 235.6198 mm³ vs authored 3.15e-4 mm³ (~750,000×). The heal-gap ambiguity
/// window the of-5rnp fix accepts by design, made an explicit tracked
/// decision: loop context (each arc endpoint anchors a radial wall chain)
/// disambiguates what the local trim decision cannot. Not softened;
/// un-ignore when of-a5me lands (as a fix or an explicit wontfix pin).
#[test]
#[ignore = "of-a5me: a sub-bound arc under a covering closure imports checker-clean at full-circle volume"]
fn a_sub_bound_arc_under_a_covering_closure_must_not_be_silently_rewritten() {
    let (r, h, chord, closure) = (5.0, 3.0, 4.2e-5, 1.0e-3);
    let sweep = sweep_for_chord(r, chord);
    let repro = format!(
        "sector(r = {r}, h = {h}), chord {chord:.1e} under the seam bound \
         {:.3e}, declared closure {closure:.1e} covers the chord",
        seam_bound(r, h)
    );
    assert_never_silently_wrong(
        &offset_cylinder_sector(0.0, r, h, sweep, Some(closure)),
        sector_volume(r, h, sweep),
        &repro,
    );
}

// =====================================================================
// (4) The bound must not depend on where the part sits
// =====================================================================

/// A moderate offset must not change the reading: the extent bound
/// (~3.4e-5) is well under the 8e-3 chord at any placement, so the of-5rnp
/// sector stays literal, exact and correct at `cx = 300` — with and without
/// its covering closure. Guarded the passing side of the of-85rt cliff when
/// the bound still moved with `cx`.
#[test]
fn a_moderate_offset_keeps_the_short_arc_literal() {
    let (r, h, chord) = (1.6, 3.0, 8.0e-3);
    let sweep = sweep_for_chord(r, chord);
    for closure in [None, Some(9.0e-3)] {
        let repro = format!(
            "sector(cx = 300, r = {r}, h = {h}), chord {chord:.1e} above bound \
             {:.3e}, closure {closure:?}",
            seam_bound(r, h)
        );
        let volume =
            exact_checked_volume(&offset_cylinder_sector(300.0, r, h, sweep, closure), &repro)
                .unwrap_or_else(|| panic!("{repro}: must stay a literal exact import"));
        assert_within(volume, sector_volume(r, h, sweep), 1.0e-6, &repro);
    }
}

/// Translating a clean part must never change its measured volume: the
/// seam bound claims to mirror heal's vertex merge, and heal's merge gap is
/// the (translation-invariant) body diagonal.
///
/// FOUND FAILING, filed and fixed as **of-85rt** (P1): at `cx = 1200` the
/// original origin-distance proxy inflated `seam_tol` to ~1.2e-2 > the 8e-3
/// chord, the arc was rewritten into a full circle, the declared closure
/// covered the chord-sized end miss in `verify_trim`, and the import was
/// *checker-clean at 24.1466 mm³ vs authored 0.0192 mm³* — the exact 1257×
/// of-5rnp corruption, resurrected by translation. The bound is now the
/// vertex-bbox diagonal (~3.4e-5 here, at any `cx`), so the arc reads
/// literally; kept as the regression test. Also reproduced under the
/// corpus-real 2.54e-2 declaration.
#[test]
fn translating_the_of_5rnp_sector_must_not_resurrect_the_collapse() {
    let (r, h, chord) = (1.6, 3.0, 8.0e-3);
    let sweep = sweep_for_chord(r, chord);
    for closure in [9.0e-3, 2.54e-2] {
        let repro = format!(
            "sector(cx = 1200, r = {r}, h = {h}), chord {chord:.1e}, declared \
             closure {closure:.3e} — the of-5rnp geometry, translated"
        );
        assert_never_silently_wrong(
            &offset_cylinder_sector(1200.0, r, h, sweep, Some(closure)),
            sector_volume(r, h, sweep),
            &repro,
        );
    }
}

/// The declaration-free half of of-85rt: with no closure the collapse was
/// refused by `verify_trim`, so the translated part was not corrupted — but
/// it silently lost the exact import it gets at the origin. Fidelity must
/// not depend on placement.
///
/// FOUND FAILING, filed and fixed as **of-85rt** (P1) with the corruption
/// above; kept as the regression test.
#[test]
fn translating_a_clean_short_arc_sector_must_not_cost_its_exact_import() {
    let (r, h, chord) = (1.6, 3.0, 8.0e-3);
    let sweep = sweep_for_chord(r, chord);
    let repro = format!(
        "sector(cx = 1200, r = {r}, h = {h}), chord {chord:.1e}, no declaration — \
         imports exactly at cx = 0 and cx = 300"
    );
    let volume =
        exact_checked_volume(&offset_cylinder_sector(1200.0, r, h, sweep, None), &repro)
            .unwrap_or_else(|| panic!("{repro}: translation must not cost the exact import"));
    assert_within(volume, sector_volume(r, h, sweep), 1.0e-6, &repro);
}

// =====================================================================
// (5) Round-trip volume certification
// =====================================================================

/// Whatever imports exactly must survive the kernel's own STEP writer and
/// come back at the same volume — including short-arc sectors near the
/// bound, whose arcs a re-read must again refuse to collapse.
#[test]
fn an_exact_import_round_trips_through_write_and_read() {
    let mut rng = Rng::new(0x_5EA3_B0DD_0005);
    for case in 0..12 {
        let r = rng.range(1.0, 8.0);
        let h = rng.range(2.0, 12.0);
        let cx = rng.range(0.0, 40.0);
        // Ordinary sweeps and near-bound slivers alike.
        let sweep = if rng.pick(2) == 0 {
            rng.range(0.1, 1.2)
        } else {
            sweep_for_chord(r, seam_bound(r, h) * rng.range(1.5, 20.0))
        };
        let repro = format!(
            "case {case}: round trip sector(cx = {cx:.6}, r = {r:.6}, h = {h:.6}, \
             sweep = {sweep:.6e})"
        );
        let text = offset_cylinder_sector(cx, r, h, sweep, None);
        let (store, geo, report) = import(&text, &repro);
        let SolidOutcome::BRep(body) = report.solids[0].outcome else {
            panic!("{repro}: the authored sector must import exactly");
        };
        let authored = sector_volume(r, h, sweep);
        let volume = brep_mass_properties(&store, &geo, body)
            .unwrap_or_else(|e| panic!("{repro}: measurement failed: {e:?}"))
            .volume;
        assert_within(volume, authored, 1.0e-6, &repro);

        let written = write_step(&store, &geo, &[body], &StepWriteOptions::default())
            .unwrap_or_else(|e| panic!("{repro}: write_step failed: {e:?}"));
        let reread = exact_checked_volume(&written, &format!("{repro} [re-read]"))
            .unwrap_or_else(|| panic!("{repro}: the written file must re-import exactly"));
        assert_within(reread, authored, 1.0e-6, &format!("{repro} [re-read]"));
    }
}
