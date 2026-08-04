//! Adversarial campaign for the declared-closure trim tolerance (of-7aja,
//! pairing of-kwn).
//!
//! of-kwn taught the reader to honour a STEP file's own
//! `GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT`: the declared closure now floors
//! [`trim_tol`]'s relative round-off rule, so a CIRCLE vertex the producer
//! parked inside its declared slop imports instead of refusing the whole
//! solid. That fix draws a new acceptance boundary, and this campaign
//! attacks the boundary rather than the happy path:
//!
//! - **Slop just inside the declaration must import** — exactly, checker
//!   clean, with the miss carried as the vertex's tolerance rather than
//!   smoothed over. Randomized radial / off-plane / tangential vertex slop
//!   on a cylinder's seam vertex, magnitudes drawn up to the declared bound.
//! - **Slop just outside must still refuse.** A floor that quietly widens
//!   past the declaration un-fixes what `verify_trim` is for.
//! - **The cliff sits where the file says** — scanned from 0.90× to 1.20×
//!   the declaration, acceptance must flip exactly once, at 1.0×.
//! - **A file authored in inches behaves like its millimetre twin.** The
//!   declaration is converted out of the file's own unit (nist_ctc_05
//!   declares thousandths of an inch); a conversion bug moves the cliff by
//!   25.4× in one direction only.
//! - **The kernel limit still binds.** A declaration past
//!   [`MAX_ALLOWED_TOLERANCE`] is clamped, so slop between the limit and a
//!   looser declaration must refuse even though the file blesses it.
//! - **Declaring a closure must never break a body that imported without
//!   one.** The floor only ever widens acceptance of vertex misses — until
//!   of-5rnp it also widened [`trim_curve`]'s "two distinct seam vertices
//!   are really the same point" test, which *rewrote an arc into a full
//!   circle*, destroying any genuinely short arc whose chord sat under the
//!   declaration. That branch now uses a heal-scale seam bound the closure
//!   does not feed; see [`a_declared_closure_must_not_collapse_a_short_arc`].
//!
//! Protocol as `step_heal_random.rs` / `boolean_stress.rs`: deterministic
//! seeded [`Rng`] (remixed by `OPENSOLID_CAMPAIGN_SEED`), a repro string on
//! every assertion, found failures become `bd` beads and their repro is
//! `#[ignore]`d referencing the bead — never softened into a passing test.
//!
//! # Failures this campaign found (both deterministic, no seed required)
//!
//! - **of-5rnp** (P1, FIXED): a declared closure rewrote a genuinely short
//!   arc (chord under the declaration) into a full circle, and the
//!   fabricated body imported *checker-clean* at 1257× the authored volume —
//!   silent corruption. The seam test no longer consults the closure; the
//!   repro [`a_declared_closure_must_not_collapse_a_short_arc`] now passes.
//! - **of-00pu** (P2, FIXED): the OCC two-vertex full-circle seam spelling
//!   died whenever the seam gap exceeded heal's merge allowance (~1e-5 ×
//!   diagonal), because neither heal's vertex merge nor (since of-5rnp) the
//!   seam test consulted the declaration. Both ends now do, each behind its
//!   own guard: the seam test widens to the closure only for an edge that
//!   stands alone in some `EDGE_LOOP` (a short read there is a
//!   guaranteed-unclosable loop, never a real arc), and heal's vertex-merge
//!   gap is floored by the closure (`HealOptions::gap_floor`) without
//!   flooring sliver collapse. The repro
//!   [`a_declared_closure_admits_a_slopped_two_vertex_seam`] now passes.

use opensolid_brep::MAX_ALLOWED_TOLERANCE;
use opensolid_brep::{GeometryStore, TopologyStore};
use opensolid_kernel::brep_mass_properties;
use opensolid_kernel::io::step::read::{
    Severity, SolidOutcome, StepImport, StepReadOptions, read_step,
};
use std::f64::consts::PI;

/// `read.rs`'s relative round-off rule, restated here so the campaign can
/// steer clear of the region where it (rather than the declaration) decides.
const TRIM_TOL_REL: f64 = 1e-6;

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `step_heal_random.rs`.
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

/// The unit the fixture's coordinates (and its closure declaration) are
/// authored in. The reader converts both out of the file unit, so a
/// millimetre file and its inch twin must land on the same side of the
/// cliff — [`an_inch_authored_file_agrees_with_its_millimetre_twin`].
#[derive(Clone, Copy, PartialEq, Debug)]
enum Unit {
    Mm,
    Inch,
}

impl Unit {
    fn in_mm(self) -> f64 {
        match self {
            Unit::Mm => 1.0,
            Unit::Inch => 25.4,
        }
    }

    /// The `#9100` length-unit declaration. The inch spelling chains a
    /// `CONVERSION_BASED_UNIT` into an SI millimetre, which is how real
    /// AP203 exporters write it (and how nist_ctc_05 declares its
    /// thousandth-of-an-inch closure).
    fn declaration(self) -> &'static str {
        match self {
            Unit::Mm => "#9100 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n",
            Unit::Inch => {
                "#9105 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
                 #9106 = DIMENSIONAL_EXPONENTS(1.0,0.0,0.0,0.0,0.0,0.0,0.0);\n\
                 #9107 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#9105);\n\
                 #9100 = (CONVERSION_BASED_UNIT('INCH',#9107) LENGTH_UNIT() NAMED_UNIT(#9106));\n"
            }
        }
    }
}

/// Wrap DATA-section body text in a minimal Part 21 envelope.
fn wrap(data: &str) -> String {
    format!(
        "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\n\
         DATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
    )
}

/// The units / representation-context tail every fixture shares: declares
/// `unit` as the file's length unit over solid `#{msb}`, and — where
/// `closure_mm` is given — a `GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT` whose
/// `UNCERTAINTY_MEASURE_WITH_UNIT` is authored in that same unit, exactly
/// as a real exporter writes its closure.
fn context_tail(msb: u64, unit: Unit, closure_mm: Option<f64>) -> String {
    let (uncertainty, in_parts) = match closure_mm {
        Some(closure) => (
            format!(
                "#9120 = UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE({:.12E}),#9100,\
                 'distance_accuracy_value','maximum model space distance between \
                 geometric entities at asserted connectivities');\n",
                closure / unit.in_mm()
            ),
            "GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#9120)) ",
        ),
        None => (String::new(), ""),
    };
    format!(
        "{units}\
         #9101 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
         #9102 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n\
         {uncertainty}\
         #9110 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) {in_parts}\
         GLOBAL_UNIT_ASSIGNED_CONTEXT((#9100,#9101,#9102)) \
         REPRESENTATION_CONTEXT('','3D Context') );\n\
         #9111 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#{msb}),#9110);",
        units = unit.declaration(),
    )
}

/// Which way the seam vertex is pushed off its circle, in the frame of the
/// bottom seam point `(r, 0, -h/2)`.
#[derive(Clone, Copy, Debug)]
enum Slop {
    /// Along `+x`: off the circle radially by the whole amount, and off the
    /// seam line by the same — nist_ctc_05's own defect shape (of-kwn).
    Radial,
    /// Along `-z`: off the circle's *plane* by the whole amount while
    /// staying exactly on the seam line — of-5cn5's defect shape.
    OffPlane,
    /// Swept along the circle by the given arc length: exactly on the
    /// circle (the closed edge re-anchors its seam parameter), but off the
    /// seam line by the chord.
    Tangential,
}

const SLOPS: [Slop; 3] = [Slop::Radial, Slop::OffPlane, Slop::Tangential];

impl Slop {
    /// The perturbed bottom-seam vertex position, in millimetres.
    fn apply(self, r: f64, h: f64, amount: f64) -> [f64; 3] {
        let lo = -h / 2.0;
        match self {
            Slop::Radial => [r + amount, 0.0, lo],
            Slop::OffPlane => [r, 0.0, lo - amount],
            Slop::Tangential => {
                let phi = amount / r;
                [r * phi.cos(), r * phi.sin(), lo]
            }
        }
    }

    /// The worst distance the perturbed vertex sits from any curve that
    /// names it: the radial and off-plane pushes miss the circle by the
    /// whole amount; the tangential push stays on the circle and misses
    /// the seam line by the chord.
    fn worst_miss(self, r: f64, amount: f64) -> f64 {
        match self {
            Slop::Radial | Slop::OffPlane => amount,
            Slop::Tangential => 2.0 * r * (amount / (2.0 * r)).sin(),
        }
    }
}

/// An AP203 cylinder (radius `r`, height `h`, both in millimetres) whose
/// *bottom seam vertex* is given a `CARTESIAN_POINT` of its own at
/// `vertex_mm`, while the seam `LINE` and both `CIRCLE`s stay anchored
/// where the clean fixture puts them — so the vertex is off its curves by
/// exactly the perturbation and nothing else about the solid changes.
/// Authored in `unit`, optionally declaring `closure_mm`.
fn cylinder_with_a_loose_seam_vertex(
    r: f64,
    h: f64,
    vertex_mm: [f64; 3],
    unit: Unit,
    closure_mm: Option<f64>,
) -> String {
    let s = unit.in_mm();
    let (lo, hi) = (-h / 2.0 / s, h / 2.0 / s);
    let rr = r / s;
    let (vx, vy, vz) = (vertex_mm[0] / s, vertex_mm[1] / s, vertex_mm[2] / s);
    let body = format!(
        "\
#1 = CARTESIAN_POINT('', (0., 0., {lo:.12}));
#2 = CARTESIAN_POINT('', (0., 0., {hi:.12}));
#3 = CARTESIAN_POINT('', ({rr:.12}, 0., {lo:.12}));
#4 = CARTESIAN_POINT('', ({rr:.12}, 0., {hi:.12}));
#5 = DIRECTION('', (0., 0., 1.));
#6 = DIRECTION('', (1., 0., 0.));
#7 = DIRECTION('', (0., 0., -1.));
#40 = CARTESIAN_POINT('', ({vx:.12}, {vy:.12}, {vz:.12}));
#8 = VERTEX_POINT('', #40);
#9 = VERTEX_POINT('', #4);
#10 = AXIS2_PLACEMENT_3D('', #1, #5, #6);
#11 = AXIS2_PLACEMENT_3D('', #2, #5, #6);
#12 = AXIS2_PLACEMENT_3D('', #1, #7, #6);
#13 = CIRCLE('', #10, {rr:.12});
#14 = CIRCLE('', #11, {rr:.12});
#15 = VECTOR('', #5, 1.);
#16 = LINE('', #3, #15);
#17 = EDGE_CURVE('', #8, #8, #13, .T.);
#18 = EDGE_CURVE('', #9, #9, #14, .T.);
#19 = EDGE_CURVE('', #8, #9, #16, .T.);
#20 = PLANE('', #12);
#21 = PLANE('', #11);
#22 = CYLINDRICAL_SURFACE('', #10, {rr:.12});
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
        tail = context_tail(39, unit, closure_mm),
    );
    wrap(&body)
}

/// An AP203 cylinder *sector*: the pie slice of a cylinder (radius `r`,
/// height `h`, millimetres) swept from angle `0` to `sweep` radians. Five
/// faces — two pie caps, two flat radial walls meeting along the axis, one
/// cylindrical wall — and two genuinely short `CIRCLE` arcs whose chord is
/// `2 r sin(sweep/2)`. Every vertex sits exactly on every curve that names
/// it; there is no slop anywhere. This is the fixture whose short arcs a
/// declared closure can destroy ([`a_declared_closure_must_not_collapse_a_short_arc`]).
fn cylinder_sector(r: f64, h: f64, sweep: f64, closure_mm: Option<f64>) -> String {
    let (c, sn) = (sweep.cos(), sweep.sin());
    let (ax, ay) = (r * c, r * sn);
    // Outward normal of the swept-side flat wall.
    let (nx, ny) = (-sn, c);
    let body = format!(
        "\
#1 = CARTESIAN_POINT('', (0., 0., 0.));
#2 = CARTESIAN_POINT('', (0., 0., {h:.12}));
#3 = CARTESIAN_POINT('', ({r:.12}, 0., 0.));
#4 = CARTESIAN_POINT('', ({r:.12}, 0., {h:.12}));
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
        tail = context_tail(84, Unit::Mm, closure_mm),
    );
    wrap(&body)
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

/// Assert the import refused the exact path *because of* the vertex miss:
/// no exact body, and an `Error` diagnostic naming the trim failure.
fn assert_refused(src: &str, context: &str) {
    let (_, _, report) = import(src, context);
    assert!(
        !matches!(report.solids[0].outcome, SolidOutcome::BRep(_)),
        "{context}: slop beyond the bound must not import exactly"
    );
    assert!(
        report.diagnostics.iter().any(|d| {
            d.severity >= Severity::Error
                && (d
                    .message
                    .contains("does not pass through the edge's vertex points")
                    || d.message.contains("misses its vertex points"))
        }),
        "{context}: the refusal must name the vertex miss: {:#?}",
        report.diagnostics
    );
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

/// A random cylinder small enough that the relative round-off rule
/// (`TRIM_TOL_REL × (1 + ‖p‖)` ≈ 1.6e-5 mm at worst here) stays two decades
/// under the declared closures this campaign draws, so the declaration —
/// not the round-off rule — is always the bound under test.
fn random_cylinder(rng: &mut Rng) -> (f64, f64) {
    (rng.range(1.0, 8.0), rng.range(2.0, 12.0))
}

/// An upper bound on the round-off allowance anywhere on such a cylinder.
fn round_off_ceiling(r: f64, h: f64) -> f64 {
    TRIM_TOL_REL * (1.0 + (r * r + h * h / 4.0).sqrt()) * 2.0
}

// =====================================================================
// (0) Baseline: the clean fixtures import exactly, declared or not
// =====================================================================

/// An unperturbed cylinder must import exactly and checker-clean whether or
/// not it declares a closure — otherwise every acceptance below would be
/// measuring fixture defects, not the tolerance under test.
#[test]
fn clean_cylinders_import_exactly_with_and_without_a_declaration() {
    let mut rng = Rng::new(0x_C105_0BA5);
    for case in 0..8 {
        let (r, h) = random_cylinder(&mut rng);
        let closure = match rng.pick(3) {
            0 => None,
            _ => Some(rng.range(1.0e-4, 9.0e-3)),
        };
        let unit = if rng.pick(2) == 0 {
            Unit::Mm
        } else {
            Unit::Inch
        };
        let repro = format!(
            "case {case}: clean cylinder(r = {r:.6}, h = {h:.6}) in {unit:?}, \
             closure {closure:?}"
        );
        let text = cylinder_with_a_loose_seam_vertex(r, h, [r, 0.0, -h / 2.0], unit, closure);
        let volume = exact_checked_volume(&text, &repro).unwrap_or_else(|| {
            panic!("{repro}: a CLEAN cylinder did not import as an exact B-Rep")
        });
        assert_within(volume, PI * r * r * h, 1.0e-6, &repro);
    }
}

// =====================================================================
// (1) Slop inside the declared closure imports, carrying its miss
// =====================================================================

/// Randomized seam-vertex slop — radial, off-plane, tangential — at up to
/// 0.9× the declared closure must import exactly, pass the geometric
/// check, measure the right volume, and carry the miss as some vertex's
/// tolerance rather than claiming `SYSTEM_RESOLUTION` precision it does
/// not have.
#[test]
fn slop_inside_the_declared_closure_imports_exactly() {
    let mut rng = Rng::new(0x_0F_7A_1A_01);
    for case in 0..24 {
        let (r, h) = random_cylinder(&mut rng);
        let closure = rng.range(1.0e-3, 9.0e-3);
        let slop = SLOPS[rng.pick(SLOPS.len())];
        // Well above the round-off rule (so the declaration is doing the
        // work) and well below the declaration (so print round-off cannot
        // tip the case over the cliff).
        let amount = rng.range(round_off_ceiling(r, h) * 20.0, 0.9 * closure);
        let vertex = slop.apply(r, h, amount);
        let repro = format!(
            "case {case}: cylinder(r = {r:.6}, h = {h:.6}), {slop:?} slop {amount:.6e} \
             inside declared closure {closure:.6e}"
        );
        let text = cylinder_with_a_loose_seam_vertex(r, h, vertex, Unit::Mm, Some(closure));

        let (store, geo, report) = import(&text, &repro);
        let SolidOutcome::BRep(body) = report.solids[0].outcome else {
            panic!(
                "{repro}: slop the file's own declaration covers must import \
                 exactly, got {:?} with {:#?}",
                report.solids[0].outcome, report.diagnostics
            );
        };
        let failures = store.check_with_geometry(&geo, body);
        assert!(
            failures.is_empty(),
            "{repro}: the accepted body must be geometrically valid, got {failures:#?}"
        );
        let volume = brep_mass_properties(&store, &geo, body)
            .unwrap_or_else(|e| panic!("{repro}: measurement failed: {e:?}"))
            .volume;
        assert_within(volume, PI * r * r * h, 1.0e-4, &repro);

        // The miss must be carried, not smoothed over: some vertex's
        // tolerance covers (and does not wildly exceed) the worst miss.
        let worst = slop.worst_miss(r, amount);
        let max_tolerance = store
            .faces_of_body(body)
            .into_iter()
            .flat_map(|f| store.edges_of_face(f))
            .flat_map(|e| {
                let edge = store.edge(e).expect("live edge");
                [edge.start_vertex, edge.end_vertex]
            })
            .map(|v| store.vertex(v).expect("live vertex").tolerance)
            .fold(0.0f64, f64::max);
        assert!(
            max_tolerance >= worst * 0.99,
            "{repro}: the vertex miss ({worst:.6e}) must be carried as a vertex \
             tolerance, but the widest tolerance is {max_tolerance:.6e}"
        );
        assert!(
            max_tolerance <= worst * 1.01 + round_off_ceiling(r, h),
            "{repro}: the carried tolerance ({max_tolerance:.6e}) claims a far \
             larger miss than the authored one ({worst:.6e})"
        );
    }
}

// =====================================================================
// (2) Slop beyond the declared closure still refuses
// =====================================================================

/// The declaration is a floor, not an amnesty: slop at 1.2–3× the declared
/// closure must refuse the exact path with a diagnostic naming the vertex
/// miss. A floor that drifts wider than the declaration un-fixes
/// `verify_trim`.
#[test]
fn slop_beyond_the_declared_closure_is_refused() {
    let mut rng = Rng::new(0x_0F_7A_1A_02);
    for case in 0..24 {
        let (r, h) = random_cylinder(&mut rng);
        let closure = rng.range(1.0e-3, 6.0e-3);
        let slop = SLOPS[rng.pick(SLOPS.len())];
        // Beyond the declaration but inside MAX_ALLOWED_TOLERANCE, so the
        // *declaration* is what refuses (the kernel limit has its own test).
        let amount = (rng.range(1.2, 3.0) * closure).min(MAX_ALLOWED_TOLERANCE * 0.98);
        assert!(amount > 1.1 * closure, "case {case}: degenerate draw");
        let vertex = slop.apply(r, h, amount);
        let repro = format!(
            "case {case}: cylinder(r = {r:.6}, h = {h:.6}), {slop:?} slop {amount:.6e} \
             beyond declared closure {closure:.6e}"
        );
        let text = cylinder_with_a_loose_seam_vertex(r, h, vertex, Unit::Mm, Some(closure));
        assert_refused(&text, &repro);
    }
}

/// Without a declaration the round-off rule stands alone, exactly as before
/// of-kwn: the same slop magnitudes must refuse. Guards against the floor
/// acquiring a default.
#[test]
fn slop_without_a_declaration_is_still_refused_past_round_off() {
    let mut rng = Rng::new(0x_0F_7A_1A_03);
    for case in 0..12 {
        let (r, h) = random_cylinder(&mut rng);
        let slop = SLOPS[rng.pick(SLOPS.len())];
        let amount = rng.range(round_off_ceiling(r, h) * 20.0, 8.0e-3);
        let vertex = slop.apply(r, h, amount);
        let repro = format!(
            "case {case}: cylinder(r = {r:.6}, h = {h:.6}), {slop:?} slop {amount:.6e}, \
             no declaration"
        );
        let text = cylinder_with_a_loose_seam_vertex(r, h, vertex, Unit::Mm, None);
        assert_refused(&text, &repro);
    }
}

// =====================================================================
// (3) The cliff sits exactly where the file declares it
// =====================================================================

/// Scan the acceptance boundary: slop at 0.90×, 0.95×, 0.99× the declared
/// closure imports; 1.01×, 1.05×, 1.20× refuses. One flip, at 1.0×. The 1%
/// margin keeps the fixture's 12-decimal print round-off from deciding a
/// case; anything that moves the cliff further than that is a tolerance
/// bug, not print noise.
#[test]
fn the_acceptance_cliff_sits_at_the_declared_closure() {
    let closure = 4.0e-3;
    let (r, h) = (5.0, 8.0);
    for slop in SLOPS {
        let mut last_accepted = true;
        for factor in [0.90, 0.95, 0.99, 1.01, 1.05, 1.20] {
            let amount = closure * factor;
            let vertex = slop.apply(r, h, amount);
            let repro = format!("{slop:?} slop at {factor}x the declared closure {closure:.1e}");
            let text = cylinder_with_a_loose_seam_vertex(r, h, vertex, Unit::Mm, Some(closure));
            let accepted = exact_checked_volume(&text, &repro).is_some();
            let expect = factor < 1.0;
            assert_eq!(
                accepted,
                expect,
                "{repro}: expected {}",
                if expect { "acceptance" } else { "refusal" }
            );
            assert!(
                last_accepted || !accepted,
                "{repro}: acceptance must flip exactly once, monotonically"
            );
            last_accepted = accepted;
        }
    }
}

// =====================================================================
// (4) Unit conversion: the inch twin agrees with the millimetre file
// =====================================================================

/// The same geometry authored in inches — coordinates *and* closure
/// declaration divided by 25.4, unit chain declared the way real exporters
/// write it — must land on the same side of the cliff as the millimetre
/// original. A conversion bug in the uncertainty path (but not the
/// coordinate path, or vice versa) moves the cliff 25.4× in one direction,
/// which 0.6× / 1.5× draws straddle.
#[test]
fn an_inch_authored_file_agrees_with_its_millimetre_twin() {
    let mut rng = Rng::new(0x_0F_7A_1A_04);
    for case in 0..16 {
        let (r, h) = random_cylinder(&mut rng);
        let closure = rng.range(1.0e-3, 6.0e-3);
        let slop = SLOPS[rng.pick(SLOPS.len())];
        let inside = rng.pick(2) == 0;
        let amount = if inside {
            (0.6 * closure).max(round_off_ceiling(r, h) * 20.0)
        } else {
            (1.5 * closure).min(MAX_ALLOWED_TOLERANCE * 0.98)
        };
        let vertex = slop.apply(r, h, amount);
        let repro = format!(
            "case {case}: cylinder(r = {r:.6}, h = {h:.6}), {slop:?} slop {amount:.6e}, \
             closure {closure:.6e}, expecting {}",
            if inside { "acceptance" } else { "refusal" }
        );
        for unit in [Unit::Mm, Unit::Inch] {
            let text = cylinder_with_a_loose_seam_vertex(r, h, vertex, unit, Some(closure));
            let accepted = exact_checked_volume(&text, &format!("{repro} [{unit:?}]")).is_some();
            assert_eq!(
                accepted, inside,
                "{repro} [{unit:?}]: the file unit must not move the cliff"
            );
        }
    }
}

// =====================================================================
// (5) The kernel limit binds through any declaration
// =====================================================================

/// A declaration looser than [`MAX_ALLOWED_TOLERANCE`] is clamped: slop the
/// file blesses but the kernel cannot carry must refuse, while slop inside
/// the limit imports under the same declaration. nist_ctc_01 declares
/// 5.08e-2 mm — twice the limit — so the clamp is a corpus-real path, and a
/// clamp that only fired in the diagnostic would let a vertex through that
/// `check` rejects.
#[test]
fn a_declaration_past_the_kernel_limit_is_clamped_in_effect() {
    let (r, h) = (5.0, 8.0);
    for declared in [2.54e-2, 1.0e3] {
        for slop in SLOPS {
            let inside = MAX_ALLOWED_TOLERANCE * 0.9;
            let beyond = MAX_ALLOWED_TOLERANCE * 1.2;
            let ok_repro =
                format!("{slop:?} slop {inside:.3e} under declaration {declared:.3e} (clamped)");
            let text = cylinder_with_a_loose_seam_vertex(
                r,
                h,
                slop.apply(r, h, inside),
                Unit::Mm,
                Some(declared),
            );
            assert!(
                exact_checked_volume(&text, &ok_repro).is_some(),
                "{ok_repro}: slop inside the kernel limit must import"
            );

            let bad_repro =
                format!("{slop:?} slop {beyond:.3e} under declaration {declared:.3e} (clamped)");
            let text = cylinder_with_a_loose_seam_vertex(
                r,
                h,
                slop.apply(r, h, beyond),
                Unit::Mm,
                Some(declared),
            );
            assert_refused(&text, &bad_repro);
        }
    }
}

// =====================================================================
// (6) A declaration must not break what imported without one
// =====================================================================

/// The declared closure only ever *widens* vertex acceptance — so adding a
/// declaration to a file that already imports exactly must never turn it
/// into a refusal or change its measured volume. Runs the loose-vertex
/// cylinder at slop the round-off rule itself accepts, with and without a
/// declaration of every magnitude class.
#[test]
fn a_declaration_never_undoes_an_import_the_round_off_rule_accepts() {
    let mut rng = Rng::new(0x_0F_7A_1A_06);
    for case in 0..12 {
        let (r, h) = random_cylinder(&mut rng);
        let slop = SLOPS[rng.pick(SLOPS.len())];
        // Inside the round-off rule at the seam point, so the file imports
        // with no declaration at all.
        let amount = TRIM_TOL_REL * (1.0 + (r * r + h * h / 4.0).sqrt()) * rng.range(0.1, 0.8);
        let vertex = slop.apply(r, h, amount);
        let base_repro = format!(
            "case {case}: cylinder(r = {r:.6}, h = {h:.6}), {slop:?} slop {amount:.3e} \
             inside round-off"
        );
        let bare = cylinder_with_a_loose_seam_vertex(r, h, vertex, Unit::Mm, None);
        let bare_volume = exact_checked_volume(&bare, &base_repro)
            .unwrap_or_else(|| panic!("{base_repro}: must import with no declaration"));
        for closure in [1.0e-4, 5.0e-3, 2.54e-2] {
            let repro = format!("{base_repro}, declaring {closure:.3e}");
            let text = cylinder_with_a_loose_seam_vertex(r, h, vertex, Unit::Mm, Some(closure));
            let volume = exact_checked_volume(&text, &repro).unwrap_or_else(|| {
                panic!("{repro}: adding a declaration must not break the import")
            });
            assert_within(volume, bare_volume, 1.0e-9, &repro);
        }
    }
}

/// The cylinder-sector fixture must import exactly at an ordinary sweep —
/// the baseline that proves the fixture itself is sound before the short-arc
/// cases below lean on it.
#[test]
fn the_cylinder_sector_fixture_imports_exactly() {
    let (r, h, sweep) = (1.6, 3.0, 0.6);
    let repro = format!("sector(r = {r}, h = {h}, sweep = {sweep})");
    let volume = exact_checked_volume(&cylinder_sector(r, h, sweep, None), &repro)
        .unwrap_or_else(|| panic!("{repro}: the clean sector must import exactly"));
    assert_within(volume, sweep / 2.0 * r * r * h, 1.0e-6, &repro);
}

/// The short-arc sector's parameters, shared by the passing baseline and
/// the of-5rnp repro: chord 8e-3 mm ≈ a 0.29° arc at r = 1.6 mm — an
/// ordinary boolean sliver, three decades above round-off scale.
fn short_arc_sector() -> (f64, f64, f64, f64) {
    let (r, h) = (1.6f64, 3.0);
    let chord = 8.0e-3f64;
    let sweep = 2.0 * (chord / (2.0 * r)).asin();
    (r, h, chord, sweep)
}

/// A sector whose arc is genuinely short — chord under the closure a real
/// file might declare — imports exactly *without* a declaration: the arc
/// is three decades above round-off, so the literal reading stands. The
/// baseline the of-5rnp repro below contrasts against.
#[test]
fn a_short_arc_sector_imports_exactly_without_a_declaration() {
    let (r, h, chord, sweep) = short_arc_sector();
    let repro = format!("sector(r = {r}, h = {h}, chord = {chord:.1e}), no declaration");
    let volume = exact_checked_volume(&cylinder_sector(r, h, sweep, None), &repro)
        .unwrap_or_else(|| panic!("{repro}: the short-arc sector must import exactly"));
    assert_within(volume, sweep / 2.0 * r * r * h, 1.0e-6, &repro);
}

/// Declaring a closure must only ever *widen* acceptance — with one
/// covering the chord, [`trim_curve`]'s seam test must still read the short
/// arc's two distinct vertices as an arc, never rewrite it as a **full
/// circle**.
///
/// FOUND FAILING, filed as **of-5rnp** (now fixed): the closure-widened
/// seam test handed back a *checker-clean exact B-Rep* measuring 24.147 mm³
/// where the authored solid is 0.0192 mm³ — 1257× the volume, silently: the
/// 8e-3 end miss was carried as vertex tolerance, so `check_with_geometry`
/// certified the fabricated solid. Any thousandth-of-an-inch declaration
/// (0.0254 mm, clamped to 0.01) covers such a chord, so the corpus-real
/// declarations of-kwn cites reached this. The seam test now uses a
/// heal-scale bound the declaration does not feed.
#[test]
fn a_declared_closure_must_not_collapse_a_short_arc() {
    let (r, h, chord, sweep) = short_arc_sector();
    let expected = sweep / 2.0 * r * r * h;
    // The declaration covers the chord — as any thousandth-of-an-inch
    // closure (0.0254 mm, clamped to 0.01) would.
    let closure = 9.0e-3;
    let repro =
        format!("sector(r = {r}, h = {h}, chord = {chord:.1e}), declaring closure {closure:.1e}");
    let volume = exact_checked_volume(&cylinder_sector(r, h, sweep, Some(closure)), &repro)
        .unwrap_or_else(|| {
            panic!(
                "{repro}: declaring a closure must not destroy a body that \
                 imports exactly without one"
            )
        });
    assert_within(volume, expected, 1.0e-6, &repro);
}

/// Rewrite the clean cylinder fixture's bottom full-circle edge into OCC's
/// two-distinct-`VERTEX_POINT` seam spelling: the second seam vertex sits
/// exactly on the circle, swept `gap` along it from the first, so only the
/// seam identity — never a vertex-off-curve miss — is at stake.
fn with_a_two_vertex_seam(text: &str, r: f64, h: f64, gap: f64) -> String {
    let phi = gap / r;
    text.replace(
        "#17 = EDGE_CURVE('', #8, #8, #13, .T.);",
        &format!(
            "#41 = CARTESIAN_POINT('', ({:.12}, {:.12}, {:.12}));\n\
             #42 = VERTEX_POINT('', #41);\n\
             #17 = EDGE_CURVE('', #8, #42, #13, .T.);",
            r * phi.cos(),
            r * phi.sin(),
            -h / 2.0
        ),
    )
}

// =====================================================================
// (7) The seam spelling the closure deliberately widens
// =====================================================================

/// Import a two-vertex-seam cylinder and require the full-cylinder result:
/// exact, checker clean, correct volume — never a sliver arc read literally.
fn assert_seam_imports_as_a_full_cylinder(r: f64, h: f64, gap: f64, closure: f64, repro: &str) {
    let text = with_a_two_vertex_seam(
        &cylinder_with_a_loose_seam_vertex(r, h, [r, 0.0, -h / 2.0], Unit::Mm, Some(closure)),
        r,
        h,
        gap,
    );
    let (store, geo, report) = import(&text, repro);
    match report.solids[0].outcome {
        SolidOutcome::BRep(body) => {
            let failures = store.check_with_geometry(&geo, body);
            assert!(
                failures.is_empty(),
                "{repro}: accepted body must be valid, got {failures:#?}"
            );
            let volume = brep_mass_properties(&store, &geo, body)
                .unwrap_or_else(|e| panic!("{repro}: measurement failed: {e:?}"))
                .volume;
            assert_within(volume, PI * r * r * h, 1.0e-4, repro);
        }
        ref other => panic!(
            "{repro}: the slopped seam the declaration covers must import \
             exactly, got {other:?} with {:#?}",
            report.diagnostics
        ),
    }
}

/// The always-working region of the two-vertex seam spelling, pinned: a gap
/// inside the *heal* vertex-merge allowance (~1e-5 × diagonal ≈ 1.3e-4 mm
/// here) imports as a full cylinder without leaning on the declaration.
/// Since of-00pu the declared closure decides the boundary beyond this —
/// [`a_declared_closure_admits_a_slopped_two_vertex_seam`].
#[test]
fn a_two_vertex_seam_within_the_heal_gap_imports() {
    let (r, h) = (5.0, 8.0);
    assert_seam_imports_as_a_full_cylinder(
        r,
        h,
        5.0e-5,
        1.0e-3,
        "two-vertex seam 5.0e-5 apart, inside the heal merge gap",
    );
}

/// A full circle written with two *distinct* seam vertices that agree only
/// to within the declared closure reads as a full circle and imports
/// exactly. This is OCC's spelling of a tangent seam, with the slop real
/// files carry.
///
/// Originally FOUND FAILING as **of-00pu** (gap 5e-4 > the ~1.3e-4 heal
/// merge gap: the loop failed vertex continuity and the solid died). Fixed
/// by teaching both ends about the declaration, each behind its own guard —
/// the seam test widens to the closure only for a sole-loop edge, and
/// heal's vertex-merge gap is floored by it (`HealOptions::gap_floor`) —
/// so the acceptance boundary is now the declaration, not the heal gap.
#[test]
fn a_declared_closure_admits_a_slopped_two_vertex_seam() {
    let (r, h) = (5.0, 8.0);
    let closure = 1.0e-3;
    // The whole gap range the of-00pu sweep measured as failing: past the
    // heal-derived merge gap (~1.3e-4 here), up to just inside the closure.
    for gap in [1.0e-4, 3.0e-4, 5.0e-4, 9.0e-4] {
        let repro = format!("two-vertex seam {gap:.1e} apart under declared closure {closure:.1e}");
        assert_seam_imports_as_a_full_cylinder(r, h, gap, closure, &repro);
    }
}

/// The adversarial half: seam vertices *further apart than the declared
/// closure* are not the same point — the file says so itself. Whatever the
/// reader does with the resulting sliver arc, it must never hand back a
/// checker-clean exact body whose volume is wrong (a silently-misread
/// solid). Refusal or degradation are both honest; a wrong volume is not.
#[test]
fn a_seam_gap_beyond_the_declared_closure_is_never_silently_misread() {
    let (r, h) = (5.0, 8.0);
    let closure = 1.0e-3;
    for gap in [2.5e-3, 8.0e-3] {
        let text = with_a_two_vertex_seam(
            &cylinder_with_a_loose_seam_vertex(r, h, [r, 0.0, -h / 2.0], Unit::Mm, Some(closure)),
            r,
            h,
            gap,
        );
        let repro =
            format!("two-vertex seam {gap:.1e} apart, beyond declared closure {closure:.1e}");
        if let Some(volume) = exact_checked_volume(&text, &repro) {
            assert_within(volume, PI * r * r * h, 1.0e-4, &repro);
        }
    }
}
