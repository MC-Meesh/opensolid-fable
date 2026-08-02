//! Seeded adversarial campaign for the mesh fallback's closure contract
//! (of-q7yz, pairing of-05ac).
//!
//! The reader's contract is that a solid the exact B-Rep path cannot take
//! degrades to tessellation rather than disappearing (`spec/06-step-io.md`
//! §4). of-05ac closed the fallback over trimmed faces after two corpus
//! files (nist_ctc_02, bspline_patch_prism) fell off the exact path and
//! were lost outright; this campaign stresses the same closure with
//! *generated* refusals instead of the two vendored ones, so the invariant
//! is measured across a family of solids rather than pinned to two files.
//!
//! # How a refusal is manufactured
//!
//! The exact path refuses a body whose edge strays from the surface of a
//! face it bounds by more than [`MAX_ALLOWED_TOLERANCE`]. That is the
//! nist_ctc_02 defect class — authored curve/surface gaps — and it is
//! reproduced here surgically: the file's *surface* records are displaced,
//! not its curves or vertices. OpenSolid's writer re-emits every
//! `CARTESIAN_POINT` per use, so moving the anchor of one `PLANE` or one
//! `CYLINDRICAL_SURFACE` placement moves that surface alone; every
//! `EDGE_CURVE` still agrees with its `VERTEX_POINT`s, the fallback's
//! shared-ring weld stays exact, and a consistent shell exists for the
//! tessellation to close over. Only the *exact* path has grounds to
//! refuse. A campaign that tore curves off their vertices instead would be
//! manufacturing the nist_ctc_05 class — no consistent shell at all —
//! which the fallback shares `trim_curve` with the exact path precisely so
//! it can refuse too ([`torn_curves_fail_structured_never_silent`] measures
//! that boundary separately).
//!
//! # The corpus is generated, not stored
//!
//! Campaign cases start from bodies this kernel wrote itself (block,
//! cylinder, cone, sphere) plus two hand-authored families the writer has
//! no primitive for, chosen because they exercise the two arms of-05ac
//! repaired:
//!
//! - **Partial-cylinder wedges** — an angular trim of a quadric wall, the
//!   `quadric_u_span` arm. Every quadric the writer emits is full-period;
//!   nist_ctc_02's 476 refused faces were all partial trims, so the
//!   campaign must author its own.
//! - **Tilted parabola prisms** — a `SURFACE_OF_LINEAR_EXTRUSION` whose
//!   profile *rises along the sweep axis*, the exact trap `sweep_offset`
//!   exists for (bspline_patch_prism's arch rises 6.25 mm along the axis
//!   it is extruded down). A conic profile guarantees the fallback runs:
//!   the exact path has no `Curve3` for a parabola. Ground truth is the
//!   oblique-prism volume, so a sweep mis-measured by the profile's rise
//!   fails the gate arithmetically rather than by inspection.
//!
//! Ground truth — volume, and that a closed shell exists — is known
//! exactly for every case, so "degraded" is asserted quantitatively:
//! the outcome must be a closed-manifold mesh whose signed volume matches
//! the solid within the tessellation's own chordal deficit (32 segments
//! per full circle under-measures a cylinder by ~0.6%; gates are set at
//! 2–3%), and `SolidOutcome::Failed` is unconditionally a campaign
//! failure.
//!
//! Protocol as `step_heal_random.rs`: deterministic seeded [`Rng`],
//! `OPENSOLID_CAMPAIGN_SEED` remix, a repro string on every failure;
//! failures become `bd` beads and the case is `#[ignore]`d referencing the
//! bead rather than softened.
//!
//! # Findings (first sweep, 2026-08-01)
//!
//! Spheres failed this campaign both ways on the first sweep, so they are
//! held out of the must-degrade gate and pinned by their own tests:
//!
//! - **of-wtu0** (fixed) — a *refused* sphere vanished: its face is
//!   bounded only by the seam, the CDT refuses the ring ("does not bound a
//!   triangulable region"), and the grid arm classified the seam boundary
//!   from samples projected onto the *displaced* surface — near-pole
//!   scatter read as a partial skewed trim, so a partial grid with 32 open
//!   edges, `SolidOutcome::Failed`. The grid arm now detects a seam-only
//!   boundary topologically and wraps the full period.
//!   [`a_refused_sphere_must_degrade_not_vanish`] pins the fix.
//! - **of-0een** — a sphere *kept* on the exact path (stray inside the
//!   tolerance the reader may raise) imports as a B-Rep that
//!   `brep_mass_properties` then refuses: `OpenParameterLoop` with a gap
//!   of exactly 2π on the seam. [`an_exactly_kept_sphere_must_still_measure`]
//!   pins it.

use opensolid_kernel::brep::{GeometryStore, MAX_ALLOWED_TOLERANCE, TopologyStore, primitives};
use opensolid_kernel::brep_mass_properties;
use opensolid_kernel::core::mesh::TriangleMesh;
use opensolid_kernel::io::step::read::{
    Severity, SolidOutcome, StepImport, StepReadOptions, read_step,
};
use opensolid_kernel::io::step::write::{StepWriteOptions, write_step};
use std::f64::consts::PI;

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `step_heal_random.rs`.
// ---------------------------------------------------------------------

/// Campaign remix (of-5rim): `OPENSOLID_CAMPAIGN_SEED=<hex>` XORs every
/// suite seed so the same properties walk fresh configurations each run.
/// Unset (CI, plain `cargo test`), the suite is byte-for-byte
/// deterministic.
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

    /// A uniformly random unit vector (rejection-sampled from the cube).
    fn direction(&mut self) -> [f64; 3] {
        loop {
            let v = [
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            ];
            let n2 = v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
            if n2 > 0.01 && n2 <= 1.0 {
                let n = n2.sqrt();
                return [v[0] / n, v[1] / n, v[2] / n];
            }
        }
    }
}

// ---------------------------------------------------------------------
// Case generation: a solid with exactly known volume, as STEP text.
// ---------------------------------------------------------------------

/// One generated case: STEP text, the exact volume the import must
/// recover, and the relative gate that volume is held to (set per shape
/// family from the tessellator's own chordal deficit, not from taste).
struct Case {
    label: String,
    text: String,
    volume: f64,
    rel_tol: f64,
}

/// Wrap a DATA-section body in a minimal Part 21 envelope (the same one
/// `step_corpus.rs` uses for its adversarial files).
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

/// A writer-emitted sphere on its own: the shape family both campaign
/// findings live in (of-wtu0, of-0een), kept out of the must-degrade gate
/// and pinned by its own `#[ignore]`d tests until those beads land.
fn writer_sphere(rng: &mut Rng) -> Case {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let r = rng.range(4.0, 10.0);
    let body = primitives::sphere(&mut store, &mut geo, r).expect("valid radius");
    let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
        .unwrap_or_else(|e| panic!("sphere: write_step failed: {e:?}"));
    Case {
        label: format!("sphere(r = {r:.4})"),
        text,
        volume: 4.0 / 3.0 * PI * r * r * r,
        rel_tol: 0.03,
    }
}

/// A writer-emitted primitive. Sizes start at 8 mm so the damage
/// amplitudes below (≤ 0.08 mm) stay under 1% of any dimension and the
/// ground-truth volume survives the bend within the volume gates.
///
/// `include_sphere` is false for the must-degrade gate: a sphere *kept* on
/// the exact path is unmeasurable today (of-0een), so the gate would fail
/// whenever the random displacement stays inside the reader's allowance.
/// The refused-sphere degrade itself is covered by
/// [`a_refused_sphere_must_degrade_not_vanish`].
fn writer_primitive(rng: &mut Rng, include_sphere: bool) -> Case {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let arms = if include_sphere { 4 } else { 3 };
    let (body, volume, rel_tol, label) = match rng.pick(arms) {
        0 => {
            let s = [
                rng.range(8.0, 20.0),
                rng.range(8.0, 20.0),
                rng.range(8.0, 20.0),
            ];
            let body =
                primitives::block(&mut store, &mut geo, s[0], s[1], s[2]).expect("valid extents");
            (
                body,
                s[0] * s[1] * s[2],
                0.01,
                format!("block({:.4}, {:.4}, {:.4})", s[0], s[1], s[2]),
            )
        }
        1 => {
            let (r, h) = (rng.range(4.0, 10.0), rng.range(8.0, 16.0));
            let body = primitives::cylinder(&mut store, &mut geo, r, h).expect("valid dimensions");
            (
                body,
                PI * r * r * h,
                0.03,
                format!("cylinder(r = {r:.4}, h = {h:.4})"),
            )
        }
        2 => {
            let (r0, h) = (rng.range(4.0, 10.0), rng.range(8.0, 14.0));
            let r1 = rng.range(0.3, 0.8) * r0;
            let body = primitives::cone(&mut store, &mut geo, r0, r1, h).expect("valid dimensions");
            (
                body,
                PI * h * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0,
                0.03,
                format!("cone(r0 = {r0:.4}, r1 = {r1:.4}, h = {h:.4})"),
            )
        }
        _ => return writer_sphere(rng),
    };
    let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
        .unwrap_or_else(|e| panic!("{label}: write_step failed: {e:?}"));
    Case {
        label,
        text,
        volume,
        rel_tol,
    }
}

/// A partial-cylinder wedge: angular span `phi` of a radius-`r` cylinder
/// wall, closed by two axial flats and two pie-slice caps. The one shape
/// family in this suite whose quadric wall is *trimmed in u* — the
/// writer's own quadrics are all full-period, and the partial span is the
/// `quadric_u_span` arm of-05ac added.
///
/// Every curve and surface record carries a private anchor
/// `CARTESIAN_POINT` (as the writer's own output does), so the damage
/// operators below can move one surface without moving anything else.
fn wedge(rng: &mut Rng) -> Case {
    let r = rng.range(4.0, 10.0);
    let h = rng.range(6.0, 14.0);
    let phi = rng.range(0.7, 5.0);
    let (c, s) = (phi.cos(), phi.sin());
    let (bx, by) = (r * c, r * s);
    // flat2's outward normal: the wedge interior spans angles (0, phi), so
    // the plane at angle phi faces +phi-ward.
    let (nx, ny) = (-s, c);
    let data = format!(
        "\
#1 = CARTESIAN_POINT('', (0., 0., 0.));
#2 = VERTEX_POINT('', #1);
#3 = CARTESIAN_POINT('', (0., 0., {h:.9}));
#4 = VERTEX_POINT('', #3);
#5 = CARTESIAN_POINT('', ({r:.9}, 0., 0.));
#6 = VERTEX_POINT('', #5);
#7 = CARTESIAN_POINT('', ({bx:.9}, {by:.9}, 0.));
#8 = VERTEX_POINT('', #7);
#9 = CARTESIAN_POINT('', ({r:.9}, 0., {h:.9}));
#10 = VERTEX_POINT('', #9);
#11 = CARTESIAN_POINT('', ({bx:.9}, {by:.9}, {h:.9}));
#12 = VERTEX_POINT('', #11);
#13 = DIRECTION('', (0., 0., 1.));
#14 = DIRECTION('', (0., 0., -1.));
#15 = DIRECTION('', (1., 0., 0.));
#16 = DIRECTION('', (0., -1., 0.));
#17 = DIRECTION('', ({c:.9}, {s:.9}, 0.));
#18 = DIRECTION('', ({nx:.9}, {ny:.9}, 0.));
#19 = CARTESIAN_POINT('', (0., 0., 0.));
#20 = AXIS2_PLACEMENT_3D('', #19, #13, #15);
#21 = CIRCLE('', #20, {r:.9});
#22 = CARTESIAN_POINT('', (0., 0., {h:.9}));
#23 = AXIS2_PLACEMENT_3D('', #22, #13, #15);
#24 = CIRCLE('', #23, {r:.9});
#25 = CARTESIAN_POINT('', ({r:.9}, 0., 0.));
#26 = VECTOR('', #13, 1.);
#27 = LINE('', #25, #26);
#28 = CARTESIAN_POINT('', ({bx:.9}, {by:.9}, 0.));
#29 = VECTOR('', #13, 1.);
#30 = LINE('', #28, #29);
#31 = CARTESIAN_POINT('', (0., 0., 0.));
#32 = VECTOR('', #15, 1.);
#33 = LINE('', #31, #32);
#34 = CARTESIAN_POINT('', (0., 0., 0.));
#35 = VECTOR('', #17, 1.);
#36 = LINE('', #34, #35);
#37 = CARTESIAN_POINT('', (0., 0., {h:.9}));
#38 = VECTOR('', #15, 1.);
#39 = LINE('', #37, #38);
#40 = CARTESIAN_POINT('', (0., 0., {h:.9}));
#41 = VECTOR('', #17, 1.);
#42 = LINE('', #40, #41);
#43 = CARTESIAN_POINT('', (0., 0., 0.));
#44 = VECTOR('', #13, 1.);
#45 = LINE('', #43, #44);
#46 = EDGE_CURVE('', #6, #8, #21, .T.);
#47 = EDGE_CURVE('', #10, #12, #24, .T.);
#48 = EDGE_CURVE('', #6, #10, #27, .T.);
#49 = EDGE_CURVE('', #8, #12, #30, .T.);
#50 = EDGE_CURVE('', #2, #6, #33, .T.);
#51 = EDGE_CURVE('', #2, #8, #36, .T.);
#52 = EDGE_CURVE('', #4, #10, #39, .T.);
#53 = EDGE_CURVE('', #4, #12, #42, .T.);
#54 = EDGE_CURVE('', #2, #4, #45, .T.);
#55 = CARTESIAN_POINT('', (0., 0., 0.));
#56 = AXIS2_PLACEMENT_3D('', #55, #13, #15);
#57 = CYLINDRICAL_SURFACE('', #56, {r:.9});
#58 = CARTESIAN_POINT('', (0., 0., 0.));
#59 = AXIS2_PLACEMENT_3D('', #58, #14, #15);
#60 = PLANE('', #59);
#61 = CARTESIAN_POINT('', (0., 0., {h:.9}));
#62 = AXIS2_PLACEMENT_3D('', #61, #13, #15);
#63 = PLANE('', #62);
#64 = CARTESIAN_POINT('', (0., 0., 0.));
#65 = AXIS2_PLACEMENT_3D('', #64, #16, #15);
#66 = PLANE('', #65);
#67 = CARTESIAN_POINT('', (0., 0., 0.));
#68 = AXIS2_PLACEMENT_3D('', #67, #18, #17);
#69 = PLANE('', #68);
#70 = ORIENTED_EDGE('', *, *, #46, .T.);
#71 = ORIENTED_EDGE('', *, *, #49, .T.);
#72 = ORIENTED_EDGE('', *, *, #47, .F.);
#73 = ORIENTED_EDGE('', *, *, #48, .F.);
#74 = EDGE_LOOP('', (#70, #71, #72, #73));
#75 = FACE_OUTER_BOUND('', #74, .T.);
#76 = ADVANCED_FACE('', (#75), #57, .T.);
#77 = ORIENTED_EDGE('', *, *, #51, .T.);
#78 = ORIENTED_EDGE('', *, *, #46, .F.);
#79 = ORIENTED_EDGE('', *, *, #50, .F.);
#80 = EDGE_LOOP('', (#77, #78, #79));
#81 = FACE_OUTER_BOUND('', #80, .T.);
#82 = ADVANCED_FACE('', (#81), #60, .T.);
#83 = ORIENTED_EDGE('', *, *, #52, .T.);
#84 = ORIENTED_EDGE('', *, *, #47, .T.);
#85 = ORIENTED_EDGE('', *, *, #53, .F.);
#86 = EDGE_LOOP('', (#83, #84, #85));
#87 = FACE_OUTER_BOUND('', #86, .T.);
#88 = ADVANCED_FACE('', (#87), #63, .T.);
#89 = ORIENTED_EDGE('', *, *, #50, .T.);
#90 = ORIENTED_EDGE('', *, *, #48, .T.);
#91 = ORIENTED_EDGE('', *, *, #52, .F.);
#92 = ORIENTED_EDGE('', *, *, #54, .F.);
#93 = EDGE_LOOP('', (#89, #90, #91, #92));
#94 = FACE_OUTER_BOUND('', #93, .T.);
#95 = ADVANCED_FACE('', (#94), #66, .T.);
#96 = ORIENTED_EDGE('', *, *, #54, .T.);
#97 = ORIENTED_EDGE('', *, *, #53, .T.);
#98 = ORIENTED_EDGE('', *, *, #49, .F.);
#99 = ORIENTED_EDGE('', *, *, #51, .F.);
#100 = EDGE_LOOP('', (#96, #97, #98, #99));
#101 = FACE_OUTER_BOUND('', #100, .T.);
#102 = ADVANCED_FACE('', (#101), #69, .T.);
#103 = CLOSED_SHELL('', (#76, #82, #88, #95, #102));
#104 = MANIFOLD_SOLID_BREP('wedge', #103);"
    );
    Case {
        label: format!("wedge(r = {r:.4}, h = {h:.4}, phi = {phi:.4})"),
        text: envelope(&data),
        volume: 0.5 * phi * r * r * h,
        rel_tol: 0.03,
    }
}

/// A prism over the region between a parabolic arc and its chord, extruded
/// a length `l` along a *tilted* direction, so the profile rises along the
/// sweep axis by `ax·dx + …` — the `sweep_offset` trap. The parabola
/// `p(t) = (f·t², 2f·t, 0)`, `t ∈ [−T, T]`, spans endpoints `(ax, ±ay, 0)`
/// with `ax = f·T²`, `ay = 2f·T`; both caps stay planar (z = const) because
/// the tilt only translates the profile.
///
/// A parabola has no exact-path `Curve3`, so every one of these bodies is
/// guaranteed to exercise the fallback — no damage required — and the
/// oblique-prism ground truth `area · l · d̂z` catches a sweep measured
/// from the profile's own rise.
fn tilted_parabola_prism(rng: &mut Rng) -> Case {
    let f = rng.range(0.8, 2.0);
    let t = rng.range(0.7, 1.3);
    let l = rng.range(4.0, 10.0);
    let (tx, ty) = (rng.range(-0.6, 0.6), rng.range(-0.6, 0.6));
    let dn = (tx * tx + ty * ty + 1.0).sqrt();
    let (dx, dy, dz) = (tx / dn, ty / dn, 1.0 / dn);
    let (ax, ay) = (f * t * t, 2.0 * f * t);
    // Top-cap translation: the profile swept to the far rim.
    let (ox, oy, oz) = (l * dx, l * dy, l * dz);
    // Chord-wall normal: perpendicular to both the chord (ŷ) and the sweep
    // direction, facing +x-ward (out of the parabolic region).
    let wn = (dz * dz + dx * dx).sqrt();
    let (wx, wz) = (dz / wn, -dx / wn);
    let (p3x, p3y) = (ax, -ay);
    let data = format!(
        "\
#1 = CARTESIAN_POINT('', (0., 0., 0.));
#2 = CARTESIAN_POINT('', ({ox:.9}, {oy:.9}, {oz:.9}));
#3 = CARTESIAN_POINT('', ({p3x:.9}, {p3y:.9}, 0.));
#4 = CARTESIAN_POINT('', ({ax:.9}, {ay:.9}, 0.));
#5 = CARTESIAN_POINT('', ({x5:.9}, {y5:.9}, {oz:.9}));
#6 = CARTESIAN_POINT('', ({x6:.9}, {y6:.9}, {oz:.9}));
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
#17 = PARABOLA('', #15, {f:.9});
#18 = PARABOLA('', #16, {f:.9});
#70 = DIRECTION('', ({dx:.9}, {dy:.9}, {dz:.9}));
#19 = VECTOR('', #70, {l:.9});
#20 = TRIMMED_CURVE('', #17, (#3), (#4), .T., .CARTESIAN.);
#21 = SURFACE_OF_LINEAR_EXTRUSION('', #20, #19);
#71 = VECTOR('', #70, 1.);
#22 = LINE('', #3, #71);
#23 = LINE('', #4, #71);
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
#72 = DIRECTION('', ({wx:.9}, 0., {wz:.9}));
#36 = AXIS2_PLACEMENT_3D('', #3, #72, #10);
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
#64 = MANIFOLD_SOLID_BREP('prism', #63);",
        x5 = p3x + ox,
        y5 = p3y + oy,
        x6 = ax + ox,
        y6 = ay + oy,
    );
    Case {
        label: format!("prism(f = {f:.4}, t = {t:.4}, l = {l:.4}, tilt = ({tx:.4}, {ty:.4}))"),
        text: envelope(&data),
        volume: 8.0 / 3.0 * f * f * t * t * t * l * dz,
        rel_tol: 0.02,
    }
}

// ---------------------------------------------------------------------
// Damage: displace one record's anchor point in the STEP text.
// ---------------------------------------------------------------------

/// The `#id` a data-section line defines, if it defines one.
fn record_id(line: &str) -> Option<u64> {
    let rest = line.strip_prefix('#')?;
    let end = rest.find(|c: char| !c.is_ascii_digit())?;
    rest[..end].parse().ok()
}

/// The first `#id` referenced after the `=`.
fn first_ref(line: &str) -> Option<u64> {
    let (_, body) = line.split_once('=')?;
    let hash = body.find('#')?;
    let digits: String = body[hash + 1..]
        .chars()
        .take_while(|c| c.is_ascii_digit())
        .collect();
    digits.parse().ok()
}

/// The data-section line defining `#id`.
fn record_line(text: &str, id: u64) -> &str {
    let prefix = format!("#{id} ");
    text.lines()
        .find(|l| l.starts_with(&prefix) || l.starts_with(&format!("#{id}=")))
        .unwrap_or_else(|| panic!("no record #{id}"))
}

/// All record ids whose definition contains `needle` (e.g. `"= PLANE("`).
fn records_matching(text: &str, needles: &[&str]) -> Vec<(u64, &'static str)> {
    let mut out = Vec::new();
    for line in text.lines() {
        if let Some(id) = record_id(line) {
            for needle in needles {
                if line.contains(needle) {
                    let name: &'static str = match *needle {
                        "= PLANE(" => "PLANE",
                        "= CYLINDRICAL_SURFACE(" => "CYLINDRICAL_SURFACE",
                        "= CONICAL_SURFACE(" => "CONICAL_SURFACE",
                        "= SPHERICAL_SURFACE(" => "SPHERICAL_SURFACE",
                        "= LINE(" => "LINE",
                        "= CIRCLE(" => "CIRCLE",
                        _ => "?",
                    };
                    out.push((id, name));
                }
            }
        }
    }
    out
}

/// Rewrite the `CARTESIAN_POINT` record `#point_id`, adding `delta` to its
/// coordinates.
fn displace_point(text: &str, point_id: u64, delta: [f64; 3]) -> String {
    let target = record_line(text, point_id).to_string();
    assert!(
        target.contains("CARTESIAN_POINT"),
        "#{point_id} is not a CARTESIAN_POINT: {target}"
    );
    let open = target.rfind('(').expect("coordinate tuple");
    let close = target[open..].find(')').expect("closed tuple") + open;
    let coords: Vec<f64> = target[open + 1..close]
        .split(',')
        .map(|c| c.trim().parse().expect("numeric coordinate"))
        .collect();
    assert_eq!(coords.len(), 3, "3D point");
    let moved = format!(
        "({:.9},{:.9},{:.9})",
        coords[0] + delta[0],
        coords[1] + delta[1],
        coords[2] + delta[2]
    );
    let replaced = format!("{}{}{}", &target[..open], moved, &target[close + 1..]);
    text.replace(&target, &replaced)
}

/// The anchor `CARTESIAN_POINT` of a surface or curve record: `LINE`
/// references its point directly; `PLANE`/quadrics/`CIRCLE` go through an
/// `AXIS2_PLACEMENT_3D` first.
fn anchor_point(text: &str, id: u64, kind: &str) -> u64 {
    let direct = first_ref(record_line(text, id)).expect("record references geometry");
    if kind == "LINE" {
        return direct;
    }
    first_ref(record_line(text, direct)).expect("placement references its point")
}

/// Displace one randomly chosen *surface* by `amplitude` in a random
/// direction: the nist_ctc_02 defect class (an edge now strays from a face
/// it bounds; every ring is still consistent). Returns the damaged text
/// and a description for repro messages.
fn offset_random_surface(text: &str, rng: &mut Rng, amplitude: f64) -> (String, String) {
    let surfaces = records_matching(
        text,
        &[
            "= PLANE(",
            "= CYLINDRICAL_SURFACE(",
            "= CONICAL_SURFACE(",
            "= SPHERICAL_SURFACE(",
        ],
    );
    assert!(!surfaces.is_empty(), "case has surfaces");
    let (id, kind) = surfaces[rng.pick(surfaces.len())];
    let dir = rng.direction();
    let delta = [dir[0] * amplitude, dir[1] * amplitude, dir[2] * amplitude];
    let point = anchor_point(text, id, kind);
    (
        displace_point(text, point, delta),
        format!(
            "{kind} #{id} moved {amplitude:.4} mm along ({:.4}, {:.4}, {:.4})",
            dir[0], dir[1], dir[2]
        ),
    )
}

/// Displace one randomly chosen *curve* instead: the curve leaves its own
/// vertices, so no consistent shell survives — the nist_ctc_05 / of-kwn
/// class, which the fallback is *specified* to refuse rather than close.
fn offset_random_curve(text: &str, rng: &mut Rng, amplitude: f64) -> (String, String) {
    let curves = records_matching(text, &["= LINE(", "= CIRCLE("]);
    assert!(!curves.is_empty(), "case has analytic curves");
    let (id, kind) = curves[rng.pick(curves.len())];
    let dir = rng.direction();
    let delta = [dir[0] * amplitude, dir[1] * amplitude, dir[2] * amplitude];
    let point = anchor_point(text, id, kind);
    (
        displace_point(text, point, delta),
        format!(
            "{kind} #{id} moved {amplitude:.4} mm along ({:.4}, {:.4}, {:.4})",
            dir[0], dir[1], dir[2]
        ),
    )
}

// ---------------------------------------------------------------------
// Import and measurement.
// ---------------------------------------------------------------------

fn import(source: &str) -> (TopologyStore, GeometryStore, StepImport) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step(source, &mut store, &mut geo, &StepReadOptions::default())
        .expect("generated file must be syntactically valid Part 21");
    (store, geo, report)
}

/// Signed volume by the divergence theorem: works on a fallback mesh,
/// which is not indexed as a manifold but is closed and outward-wound.
/// Taken *signed*, so an inside-out degrade fails the volume gate too.
fn signed_volume(mesh: &TriangleMesh) -> f64 {
    mesh.indices
        .iter()
        .map(|tri| {
            let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
            a.dot(&b.cross(&c)) / 6.0
        })
        .sum()
}

fn took_fallback(report: &StepImport) -> bool {
    report
        .diagnostics
        .iter()
        .any(|d| d.message.contains("falling back to tessellated import"))
}

/// The degraded-not-vanished gate: the one solid must come back `BRep` or
/// `Mesh` — never `Failed` — and either outcome must measure the ground
/// truth.
fn assert_degraded_not_vanished(case: &Case, damage: &str, repro: &str) -> bool {
    let (store, geo, report) = import(&case.text);
    assert_eq!(
        report.solids.len(),
        1,
        "{repro}: {label} [{damage}]: expected one solid",
        label = case.label
    );
    let measured = match &report.solids[0].outcome {
        SolidOutcome::BRep(body) => {
            brep_mass_properties(&store, &geo, *body)
                .unwrap_or_else(|e| {
                    panic!(
                        "{repro}: {label} [{damage}]: exact import failed to measure: {e:?}",
                        label = case.label
                    )
                })
                .volume
        }
        SolidOutcome::Mesh { mesh, .. } => {
            assert!(
                mesh.is_closed_manifold(),
                "{repro}: {label} [{damage}]: fallback mesh is not a closed manifold",
                label = case.label
            );
            signed_volume(mesh)
        }
        SolidOutcome::Failed => panic!(
            "{repro}: {label} [{damage}]: the solid VANISHED (SolidOutcome::Failed) — \
             the contract is degrade-to-mesh, never lose the solid. Diagnostics:\n{}",
            report
                .diagnostics
                .iter()
                .map(|d| format!("  [{:?}] {}", d.severity, d.message))
                .collect::<Vec<_>>()
                .join("\n"),
            label = case.label,
        ),
    };
    let drift = (measured - case.volume).abs() / case.volume;
    assert!(
        drift <= case.rel_tol,
        "{repro}: {label} [{damage}]: volume {measured:.6} vs expected {expected:.6} \
         ({drift:.4} relative, gate {gate})",
        label = case.label,
        expected = case.volume,
        gate = case.rel_tol,
    );
    took_fallback(&report)
}

// ---------------------------------------------------------------------
// Campaigns
// ---------------------------------------------------------------------

/// Repro prefix: the failing case regenerates from the suite seed and case
/// index alone (plus `OPENSOLID_CAMPAIGN_SEED` if one was set).
fn repro(test: &str, seed: u64, index: usize) -> String {
    format!(
        "[repro: OPENSOLID_CAMPAIGN_SEED={:#x} cargo test -p opensolid-kernel \
         --test step_mesh_fallback_random {test} — case {index}, suite seed {seed:#x}]",
        campaign_seed()
    )
}

/// The core of-q7yz campaign: a solid whose only defect is one surface
/// sitting `2–8 × MAX_ALLOWED_TOLERANCE` away from where its edges say it
/// should be. The exact path must refuse most of these (the displacement's
/// off-surface component usually exceeds the 0.01 mm cap) and every refusal
/// must land in a closed, correctly-measuring fallback mesh.
#[test]
fn a_refused_solid_degrades_to_mesh_not_failed() {
    let mut rng = Rng::new(0x51ef_05ac_0f97_a7b2);
    let mut fallbacks = 0;
    const CASES: usize = 48;
    for index in 0..CASES {
        let case = match rng.pick(3) {
            0 => wedge(&mut rng),
            // No spheres here: a sphere the exact path *keeps* is
            // unmeasurable (of-0een). The refused-sphere degrade is
            // covered by `a_refused_sphere_must_degrade_not_vanish` below.
            _ => writer_primitive(&mut rng, false),
        };
        let amplitude = rng.range(2.0, 8.0) * MAX_ALLOWED_TOLERANCE;
        let (text, damage) = offset_random_surface(&case.text, &mut rng, amplitude);
        let damaged = Case { text, ..case };
        let r = repro("a_refused_solid_degrades_to_mesh_not_failed", 0x51EF, index);
        if assert_degraded_not_vanished(&damaged, &damage, &r) {
            fallbacks += 1;
        }
    }
    // A campaign in which nothing fell off the exact path measured nothing.
    // Random displacement leaves ~10–20% of cases inside the exact path's
    // 0.01 mm allowance (in-surface moves are invisible); anything below a
    // quarter says the damage operator has stopped manufacturing refusals.
    assert!(
        fallbacks >= CASES / 4,
        "only {fallbacks}/{CASES} cases exercised the fallback — the damage \
         operator no longer produces exact-path refusals"
    );
}

/// The bspline_patch_prism class, generated: every tilted parabola prism
/// is fallback-only by construction (no exact-path conics), so this is a
/// direct randomized measure of `sweep_offset` + the CDT caps + the swept
/// wall — no damage involved.
#[test]
fn tilted_extrusion_prisms_degrade_watertight() {
    let mut rng = Rng::new(0x9127_44d1_0b5e_77aa);
    for index in 0..18 {
        let case = tilted_parabola_prism(&mut rng);
        let r = repro("tilted_extrusion_prisms_degrade_watertight", 0x9127, index);
        let (_, _, report) = import(&case.text);
        assert_eq!(report.solids.len(), 1, "{r}: {}: one solid", case.label);
        match &report.solids[0].outcome {
            SolidOutcome::Mesh { mesh, .. } => {
                assert!(
                    mesh.is_closed_manifold(),
                    "{r}: {}: fallback mesh is not a closed manifold",
                    case.label
                );
                let measured = signed_volume(mesh);
                let drift = (measured - case.volume).abs() / case.volume;
                assert!(
                    drift <= case.rel_tol,
                    "{r}: {}: volume {measured:.6} vs expected {:.6} ({drift:.4} relative)",
                    case.label,
                    case.volume,
                );
            }
            other => panic!(
                "{r}: {}: expected the mesh fallback (parabola walls have no \
                 exact path), got {other:?}",
                case.label
            ),
        }
    }
}

/// The boundary the fallback must *not* cross: a curve torn off its own
/// vertices leaves no consistent shell (nist_ctc_05 / of-kwn class), and
/// the loss must be structured — a `Failed` outcome explained by at least
/// one Warning/Error diagnostic — never a panic, never silence, and any
/// solid that *does* survive must still measure correctly.
#[test]
fn torn_curves_fail_structured_never_silent() {
    let mut rng = Rng::new(0x77aa_3c1d_5e02_9b48);
    for index in 0..24 {
        let case = match rng.pick(2) {
            0 => writer_primitive(&mut rng, true),
            _ => wedge(&mut rng),
        };
        let amplitude = rng.range(2.0, 8.0) * MAX_ALLOWED_TOLERANCE;
        let (text, damage) = offset_random_curve(&case.text, &mut rng, amplitude);
        let r = repro("torn_curves_fail_structured_never_silent", 0x77aa, index);
        let (store, geo, report) = import(&text);
        assert_eq!(
            report.solids.len(),
            1,
            "{r}: {} [{damage}]: one solid",
            case.label
        );
        match &report.solids[0].outcome {
            SolidOutcome::Failed => {
                assert!(
                    report
                        .diagnostics
                        .iter()
                        .any(|d| matches!(d.severity, Severity::Warning | Severity::Error)),
                    "{r}: {} [{damage}]: a lost solid must be explained by a \
                     Warning/Error diagnostic",
                    case.label
                );
            }
            SolidOutcome::Mesh { mesh, .. } => {
                assert!(
                    mesh.is_closed_manifold(),
                    "{r}: {} [{damage}]: a degraded solid must still be watertight",
                    case.label
                );
                let measured = signed_volume(mesh);
                let drift = (measured - case.volume).abs() / case.volume;
                assert!(
                    drift <= case.rel_tol.max(0.05),
                    "{r}: {} [{damage}]: degraded volume {measured:.6} vs {:.6}",
                    case.label,
                    case.volume,
                );
            }
            SolidOutcome::BRep(body) => {
                // Small off-vertex components can stay inside the trim
                // tolerance; an exact import must then measure exactly.
                let measured = brep_mass_properties(&store, &geo, *body)
                    .unwrap_or_else(|e| {
                        panic!(
                            "{r}: {} [{damage}]: exact import failed to measure: {e:?}",
                            case.label
                        )
                    })
                    .volume;
                let drift = (measured - case.volume).abs() / case.volume;
                assert!(
                    drift <= case.rel_tol.max(0.02),
                    "{r}: {} [{damage}]: exact volume {measured:.6} vs {:.6}",
                    case.label,
                    case.volume,
                );
            }
        }
    }
}

/// Scratch survey (development tool, `#[ignore]`d): sweeps many more cases
/// than the gate and tallies outcome classes per shape × surface family
/// instead of stopping at the first failure. Run it when triaging a new
/// campaign failure to see the whole landscape:
/// `cargo test -p opensolid-kernel --test step_mesh_fallback_random survey -- --ignored --nocapture`
#[test]
#[ignore = "diagnostic sweep, not a gate"]
fn survey() {
    let mut rng = Rng::new(0xdead_beef_0102_0304);
    let mut tally: std::collections::BTreeMap<String, usize> = Default::default();
    let mut samples = 0;
    for index in 0..300 {
        let case = match rng.pick(6) {
            0 | 1 => wedge(&mut rng),
            _ => writer_primitive(&mut rng, true),
        };
        let amplitude = rng.range(2.0, 8.0) * MAX_ALLOWED_TOLERANCE;
        let (text, damage) = offset_random_surface(&case.text, &mut rng, amplitude);
        let (store, geo, report) = import(&text);
        let shape = case.label.split('(').next().unwrap().to_string();
        let surface = damage.split(' ').next().unwrap().to_string();
        let fell = took_fallback(&report);
        let outcome = match &report.solids[0].outcome {
            SolidOutcome::BRep(body) => {
                match brep_mass_properties(&store, &geo, *body).map(|m| m.volume) {
                    Ok(v) if ((v - case.volume) / case.volume).abs() <= case.rel_tol => "brep-ok",
                    Ok(_) => "brep-WRONG-VOLUME",
                    Err(e) => {
                        println!(
                            "--- case {index}: {} [{damage}]: unmeasurable: {e:?}",
                            case.label
                        );
                        "brep-UNMEASURABLE"
                    }
                }
            }
            SolidOutcome::Mesh { mesh, .. } => {
                if !mesh.is_closed_manifold() {
                    "mesh-OPEN"
                } else {
                    let v = signed_volume(mesh);
                    if ((v - case.volume) / case.volume).abs() <= case.rel_tol {
                        "mesh-ok"
                    } else if ((-v - case.volume) / case.volume).abs() <= case.rel_tol {
                        "mesh-INVERTED"
                    } else {
                        "mesh-WRONG-VOLUME"
                    }
                }
            }
            SolidOutcome::Failed => "FAILED",
        };
        *tally
            .entry(format!(
                "{shape:9} {surface:20} fallback={fell:5} {outcome}"
            ))
            .or_default() += 1;
        if (outcome.contains("FAILED") || outcome.contains("WRONG") || outcome.contains("OPEN"))
            && samples < 12
        {
            samples += 1;
            println!("--- case {index}: {} [{damage}]", case.label);
            for d in &report.diagnostics {
                if !matches!(d.severity, Severity::Info)
                    || d.message.contains("manifold")
                    || d.message.contains("gridded")
                {
                    println!("    [{:?}] {}", d.severity, d.message);
                }
            }
        }
    }
    println!();
    for (k, v) in tally {
        println!("{v:4}  {k}");
    }
}

/// Pins of-wtu0: a sphere whose surface record is displaced past
/// [`MAX_ALLOWED_TOLERANCE`] is correctly refused by the exact path and
/// must then degrade to a closed mesh, not vanish. It used to vanish: the
/// quadric grid arm classified the face's `u` span from boundary samples
/// projected onto the *displaced* surface, whose near-pole scatter turned
/// the seam-only boundary into a "partial skewed trim" — a partial grid
/// with 32 open edges no weld could bridge. The reader now detects a
/// seam-only boundary topologically (every edge traversed net-zero) and
/// grids the full period with wrap.
#[test]
fn a_refused_sphere_must_degrade_not_vanish() {
    let mut rng = Rng::new(0x0f_37b0_51ef_0001);
    let mut refused = 0;
    for index in 0..24 {
        let case = writer_sphere(&mut rng);
        let amplitude = rng.range(2.0, 8.0) * MAX_ALLOWED_TOLERANCE;
        let (text, damage) = offset_random_surface(&case.text, &mut rng, amplitude);
        let damaged = Case { text, ..case };
        let r = repro("a_refused_sphere_must_degrade_not_vanish", 0x0f37, index);
        if assert_degraded_not_vanished(&damaged, &damage, &r) {
            refused += 1;
        }
    }
    assert!(
        refused >= 6,
        "only {refused}/24 spheres were refused — the damage operator no \
         longer manufactures refusals"
    );
}

/// Pins of-0een: when the displacement stays inside the tolerance the
/// reader may raise, the sphere imports as an exact B-Rep — which
/// `brep_mass_properties` must then be able to measure. Today it refuses
/// with `OpenParameterLoop { gap: 2π }` on the seam: an import that
/// reports exact but cannot be measured is a silent degrade. Un-ignore
/// when of-0een lands.
#[test]
#[ignore = "of-0een: kept-exact sphere is unmeasurable (OpenParameterLoop gap 2π)"]
fn an_exactly_kept_sphere_must_still_measure() {
    let mut rng = Rng::new(0x0e_e12a_77fd_0002);
    let mut kept = 0;
    for index in 0..300 {
        let case = writer_sphere(&mut rng);
        let amplitude = rng.range(2.0, 8.0) * MAX_ALLOWED_TOLERANCE;
        let (text, damage) = offset_random_surface(&case.text, &mut rng, amplitude);
        let r = repro("an_exactly_kept_sphere_must_still_measure", 0x0ee1, index);
        let (store, geo, report) = import(&text);
        if let SolidOutcome::BRep(body) = &report.solids[0].outcome {
            kept += 1;
            let measured = brep_mass_properties(&store, &geo, *body)
                .unwrap_or_else(|e| {
                    panic!(
                        "{r}: {} [{damage}]: an exact import must be measurable: {e:?}",
                        case.label
                    )
                })
                .volume;
            let drift = (measured - case.volume).abs() / case.volume;
            assert!(
                drift <= 0.01,
                "{r}: {} [{damage}]: exact volume {measured:.6} vs {:.6}",
                case.label,
                case.volume,
            );
            if kept >= 3 {
                return;
            }
        }
    }
    assert!(
        kept > 0,
        "no sphere stayed on the exact path in 300 cases — widen the scan"
    );
}

/// The undamaged templates are themselves gated: a wedge and a prism must
/// import and measure before any damage arm is allowed to draw conclusions
/// from them. (The writer primitives are already covered by
/// `step_heal_random.rs` and the round-trip suites.)
#[test]
fn undamaged_templates_import_and_measure() {
    let mut rng = Rng::new(0x5eed_ba5e_0000_0001);
    for index in 0..6 {
        let case = if index % 2 == 0 {
            wedge(&mut rng)
        } else {
            tilted_parabola_prism(&mut rng)
        };
        let r = repro("undamaged_templates_import_and_measure", 0x5eed, index);
        assert_degraded_not_vanished(&case, "undamaged", &r);
    }
}
