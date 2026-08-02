//! Seeded randomized campaign for STEP import healing (of-ipt.18).
//!
//! `heal.rs` is 69 KB of repair logic — vertex merging, edge welding,
//! tolerance elevation, face reorientation, shell reversal — reached from
//! every `read_step` call. Its tests are hand-authored files with one
//! defect apiece. The defects a real exporter produces are not one at a
//! time and not at a hand-picked magnitude, and the two things that decide
//! whether healing is *correct* rather than merely active are both
//! quantitative: it must repair everything inside its gap tolerance, and it
//! must refuse everything outside it.
//!
//! # The corpus is generated, not stored
//!
//! Every case starts from a primitive body this kernel wrote itself, so the
//! ground truth — volume, genus, validity — is known exactly. The file is
//! then damaged the way a real exporter damages one:
//!
//! - **Unsewing.** Every `ORIENTED_EDGE` is given its own private copy of the
//!   `EDGE_CURVE` it references and of that edge's two `VERTEX_POINT`s, each
//!   independently perturbed. This is the shell a foreign exporter writes:
//!   each face authors its own copy of every boundary edge and corner,
//!   agreeing with its neighbour only to within export round-off. See
//!   [`unsew`].
//! - **Face sense flips.** Flipping the orientation flag of random
//!   `ADVANCED_FACE` records is what a producer with untrustworthy sense
//!   flags emits, and is exactly what the orientation pass is for.
//!
//! Only topology is unsewn; the underlying `LINE`/`CIRCLE` geometry stays
//! shared. Perturbing the curve and surface placements too would produce a
//! file whose *geometry* is inconsistent rather than whose *topology* is
//! unsewn — a different defect, and not the one healing is specified to fix.
//!
//! **Perturbing coordinates alone does nothing here, and that is worth
//! recording.** OpenSolid's own writer emits a properly sewn file: faces that
//! meet on an edge share one `VERTEX_POINT` and one `EDGE_CURVE` record, and
//! only the underlying `CARTESIAN_POINT`s are re-emitted per use. Moving those
//! coordinates moves one shared vertex and opens no gap at all, so a campaign
//! built on jitter alone imports cleanly without a single repair being
//! invoked — passing while testing nothing. Producing work for the healer
//! requires duplicating the topological records, which is what [`unsew`] does.
//!
//! # Two tolerances, not one
//!
//! The perturbation amplitude is bounded above by the *reader's* trim
//! tolerance, not by the healer's gap. A moved vertex leaves the curve its
//! edge carries, and the reader validates that a vertex lies on its edge's
//! curve at `TRIM_TOL_REL` — "the same idea one decade tighter" than
//! `HEAL_GAP_REL` (`heal.rs`). Amplitudes above `1e-6 × diagonal` therefore
//! produce a file the reader rejects on *geometry* before healing is ever
//! consulted, and a campaign that picked its amplitude from the gap alone
//! would be measuring that rejection instead of the repair. Amplitudes here
//! stay under the trim tolerance while leaving the *separation between copies
//! of a corner* — which is what healing merges — inside the gap.
//!
//! # A defect this campaign found, and what it turned out to be
//!
//! The face-sense campaigns used to run on planar-faced bodies only, because
//! a *periodic* face (a cylinder wall and its seam) came back from a healed
//! reversal as a body `check()` certified and `brep_mass_properties` then
//! refused, with a parameter-loop gap of exactly `4π` — of-hrgt.
//!
//! The reversal was never healed at all. Flipping an `ADVANCED_FACE` sense
//! flag changes no fin sense, so the orientation pass's two-colouring saw a
//! consistent shell; and the reader only consults the healer once the
//! *topology-only* `TopologyStore::check` has failed, which that body did not.
//! It came through as an exact import with one face's flag contradicting its
//! own loop's winding. The periodic face failed loudly (the seam's branch is
//! read off the flag); the planar ones failed silently, measuring correctly
//! while being just as malformed, which is why flipping their senses had
//! looked safe. `reconcile_face_senses` (`heal.rs`) is the repair, run from
//! the reader once pcurves make the winding readable.
//!
//! Both face-sense campaigns now run the whole corpus — cylinders and cones
//! included — and [`a_reversed_face_sense_flag_is_corrected_on_import`] pins
//! the original repro. They assert `check_with_geometry`, which is the check
//! that would have caught this on day one.
//!
//! # Both directions are asserted
//!
//! A healer that merges everything passes any "does it heal?" test. The
//! contract in `HealOptions::max_gap` is two-sided — repairs within the gap,
//! refuses beyond it ("healing may not create a body the kernel would reject
//! for tolerance alone") — so
//! [`gaps_beyond_the_tolerance_are_not_silently_merged`] asserts the refusal, and
//! is the half that fails if healing ever becomes indiscriminate.
//!
//! # The geometric check is the one that measures this campaign
//!
//! `TopologyStore::check` reads the graph and holds no geometry, so it cannot
//! see a vertex that has drifted off its edge's curve — which is most of what
//! this campaign's damage does. Every case therefore asserts
//! [`exact_import_volume_checked`], which runs `check_with_geometry` as well.
//!
//! The jittered ones did not, until of-bbh8, and that fence is what it hid: a
//! body the healer reported as fully repaired came back with `VertexOffEdge` at
//! ~1.01× the elevated allowance. Two tolerance shortfalls compounded, and
//! of-hrgt above is the same shape a third time — a validator certifying a body
//! the stricter one rejects. The reader accepts a vertex
//! up to `TRIM_TOL_REL` off its edge's curve — it must, since STEP writes
//! finite decimals and rounds a vertex point and its curve independently — and
//! then created the vertex at `SYSTEM_RESOLUTION`, claiming a precision the
//! file never had. Healing then moved a merged corner to its cluster's
//! centroid and elevated the survivor to `max(existing, displacement)` when
//! what it owed was `existing + displacement`.
//! [`sewn_jitter_is_carried_as_tolerance`] pins the reader half on a file with
//! nothing to heal at all.
//!
//! Protocol as `boolean_stress.rs`: deterministic seeded [`Rng`], a repro
//! string on every failure, failures become `bd` beads and the case is
//! `#[ignore]`d referencing the bead rather than softened.

use opensolid_brep::{GeometryStore, TopologyStore, primitives};
use opensolid_kernel::brep_mass_properties;
use opensolid_kernel::io::step::{
    HealOptions, HealStrategy, SolidOutcome, StepReadOptions, StepWriteOptions, read_step,
    write_step,
};
use std::f64::consts::PI;

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `boolean_stress.rs`.
// ---------------------------------------------------------------------

/// Campaign remix (of-5rim): `OPENSOLID_CAMPAIGN_SEED=<hex>` XORs every suite
/// seed so the same properties walk fresh configurations each run. Unset (CI,
/// plain `cargo test`), the suite is byte-for-byte deterministic. A campaign
/// failure reproduces with the same variable value plus the test name — the
/// campaign driver (`tools/campaign/`) records both in the bead it files.
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
// Corpus generation: a primitive body, exported, with known ground truth.
// ---------------------------------------------------------------------

/// One generated case: the exported STEP text and the exact properties the
/// import must recover.
struct Exported {
    label: String,
    text: String,
    volume: f64,
    /// Bounding-box diagonal, the scale every gap tolerance is relative to.
    diagonal: f64,
}

/// Every case is exported at millimetre scale, which is what `write_step`
/// declares and what the reader assumes, so `length_scale` is 1 and no unit
/// conversion sits between the ground truth and the assertion.
fn export_random_primitive(rng: &mut Rng) -> Exported {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let choice = rng.pick(3);
    let (body, volume, diagonal, label) = match choice {
        0 => {
            let s = [
                rng.range(2.0, 20.0),
                rng.range(2.0, 20.0),
                rng.range(2.0, 20.0),
            ];
            let body =
                primitives::block(&mut store, &mut geo, s[0], s[1], s[2]).expect("valid extents");
            (
                body,
                s[0] * s[1] * s[2],
                (s[0] * s[0] + s[1] * s[1] + s[2] * s[2]).sqrt(),
                format!("block({:.4}, {:.4}, {:.4})", s[0], s[1], s[2]),
            )
        }
        1 => {
            let (r, h) = (rng.range(1.0, 8.0), rng.range(2.0, 16.0));
            let body = primitives::cylinder(&mut store, &mut geo, r, h).expect("valid dimensions");
            (
                body,
                PI * r * r * h,
                (4.0 * r * r + h * h).sqrt(),
                format!("cylinder(r = {r:.4}, h = {h:.4})"),
            )
        }
        _ => {
            let (r0, h) = (rng.range(1.5, 8.0), rng.range(2.0, 12.0));
            let r1 = rng.range(0.2, 0.8) * r0;
            let body = primitives::cone(&mut store, &mut geo, r0, r1, h).expect("valid dimensions");
            (
                body,
                PI * h * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0,
                (4.0 * r0 * r0 + h * h).sqrt(),
                format!("cone(r0 = {r0:.4}, r1 = {r1:.4}, h = {h:.4})"),
            )
        }
    };
    let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
        .unwrap_or_else(|e| panic!("{label}: write_step failed: {e:?}"));
    Exported {
        label,
        text,
        volume,
        diagonal,
    }
}

// ---------------------------------------------------------------------
// Damage
// ---------------------------------------------------------------------

/// The `#id` a data-section line defines, if it defines one.
fn record_id(line: &str) -> Option<u64> {
    let rest = line.strip_prefix('#')?;
    let end = rest.find(|c: char| !c.is_ascii_digit())?;
    rest[..end].parse().ok()
}

/// Every `#id` a line references, in order of appearance after the `=`.
fn referenced_ids(line: &str) -> Vec<u64> {
    let (_, body) = line.split_once('=').unwrap_or(("", line));
    let mut out = Vec::new();
    let bytes = body.as_bytes();
    let mut i = 0;
    while i < bytes.len() {
        if bytes[i] == b'#' {
            let start = i + 1;
            let mut end = start;
            while end < bytes.len() && bytes[end].is_ascii_digit() {
                end += 1;
            }
            if end > start
                && let Ok(id) = body[start..end].parse()
            {
                out.push(id);
            }
            i = end;
        } else {
            i += 1;
        }
    }
    out
}

/// The three coordinates of a `CARTESIAN_POINT` line.
fn point_coords(line: &str) -> [f64; 3] {
    let (_, rest) = line.split_once("',(").expect("a CARTESIAN_POINT record");
    let (coords, _) = rest.split_once("))").expect("closed coordinate list");
    let values: Vec<f64> = coords
        .split(',')
        .map(|s| s.trim().parse().expect("STEP writes plain reals"))
        .collect();
    [values[0], values[1], values[2]]
}

/// **Unsew** the shell: give every `ORIENTED_EDGE` its own private copy of
/// the `EDGE_CURVE` it references and of that edge's two `VERTEX_POINT`s and
/// their `CARTESIAN_POINT`s, each independently jittered by at most
/// `amplitude`. Returns the damaged text and the largest displacement
/// applied.
///
/// This — not coordinate jitter — is the defect healing exists for.
/// OpenSolid's own writer emits a *properly sewn* file: `VERTEX_POINT` and
/// `EDGE_CURVE` records are shared between the faces that meet on them, so
/// perturbing a coordinate moves one shared vertex and creates no gap at all
/// (a campaign built on that premise passes without ever invoking a repair).
/// A foreign exporter typically writes the opposite: each face authors its
/// own copy of every boundary edge and corner, agreeing only to within
/// export round-off. That is what this produces, and it is what
/// `GeometryHealer::fix_gaps` — vertex merging plus the duplicate-edge weld
/// that "makes an unsewn shell watertight" — is specified to repair.
///
/// The underlying curve (`LINE`, `CIRCLE`) stays shared: it is the
/// *topology* that is unsewn, not the geometry.
fn unsew(text: &str, rng: &mut Rng, amplitude: f64) -> (String, f64) {
    use std::collections::HashMap;

    let by_id: HashMap<u64, &str> = text
        .lines()
        .filter_map(|l| record_id(l).map(|id| (id, l)))
        .collect();
    let mut next_id = by_id.keys().copied().max().unwrap_or(0) + 1;
    let mut added: Vec<String> = Vec::new();
    let mut largest: f64 = 0.0;

    // A private, jittered copy of one VERTEX_POINT; returns its new id.
    let copy_vertex = |vertex_id: u64,
                       rng: &mut Rng,
                       next_id: &mut u64,
                       added: &mut Vec<String>,
                       largest: &mut f64|
     -> u64 {
        let vertex_line = by_id[&vertex_id];
        let point_id = referenced_ids(vertex_line)[0];
        let [x, y, z] = point_coords(by_id[&point_id]);
        // Uniform in a cube of half-width amplitude/√3, so the displacement
        // magnitude never exceeds `amplitude`.
        let step = |rng: &mut Rng| rng.range(-1.0, 1.0) * amplitude / 3.0_f64.sqrt();
        let (dx, dy, dz) = (step(rng), step(rng), step(rng));
        *largest = largest.max((dx * dx + dy * dy + dz * dz).sqrt());
        let new_point = *next_id;
        *next_id += 1;
        // Fixed-point, not `{:?}`: Rust's shortest-round-trip form switches
        // to exponent notation for small magnitudes, and a coordinate near
        // zero would come out as `1e-6` rather than as a STEP real.
        added.push(format!(
            "#{new_point} = CARTESIAN_POINT('',({:.12},{:.12},{:.12}));",
            x + dx,
            y + dy,
            z + dz
        ));
        let new_vertex = *next_id;
        *next_id += 1;
        added.push(format!("#{new_vertex} = VERTEX_POINT('',#{new_point});"));
        new_vertex
    };

    let mut rewritten = Vec::with_capacity(text.lines().count());
    for line in text.lines() {
        if !line.contains("= ORIENTED_EDGE(") {
            rewritten.push(line.to_string());
            continue;
        }
        let edge_id = referenced_ids(line)[0];
        let edge_line = by_id[&edge_id];
        let edge_refs = referenced_ids(edge_line);
        let (v1, v2, curve) = (edge_refs[0], edge_refs[1], edge_refs[2]);
        let sense = if edge_line.ends_with(".T.);") {
            ".T."
        } else {
            ".F."
        };

        let n1 = copy_vertex(v1, rng, &mut next_id, &mut added, &mut largest);
        // A closed (seam) edge has start == end; one copy must serve both,
        // or the circle's endpoints would no longer coincide.
        let n2 = if v1 == v2 {
            n1
        } else {
            copy_vertex(v2, rng, &mut next_id, &mut added, &mut largest)
        };

        let new_edge = next_id;
        next_id += 1;
        added.push(format!(
            "#{new_edge} = EDGE_CURVE('',#{n1},#{n2},#{curve},{sense});"
        ));
        rewritten.push(line.replacen(&format!("#{edge_id}"), &format!("#{new_edge}"), 1));
    }

    // STEP data sections are reference-ordered, not line-ordered, so the new
    // records can simply precede the closing ENDSEC.
    let end = rewritten
        .iter()
        .rposition(|l| l.trim() == "ENDSEC;")
        .expect("the data section closes");
    for (i, record) in added.into_iter().enumerate() {
        rewritten.insert(end + i, record);
    }
    (rewritten.join("\n"), largest)
}

/// Jitter every `CARTESIAN_POINT` a `VERTEX_POINT` refers to, by at most
/// `amplitude`, leaving the file otherwise **sewn**. Returns the damaged text
/// and the largest displacement applied.
///
/// The complement of [`unsew`], and the reason this exists despite the module
/// docs recording that jitter alone gives healing nothing to do: that is the
/// point. Every corner stays one shared `VERTEX_POINT`, so no gap opens and no
/// repair is planned — but each vertex has still moved off the curves its
/// edges carry, and the import has to say so in the vertex's tolerance rather
/// than in silence (of-bbh8).
fn jitter_vertex_points(text: &str, rng: &mut Rng, amplitude: f64) -> (String, f64) {
    use std::collections::HashSet;

    let referenced: HashSet<u64> = text
        .lines()
        .filter(|l| l.contains("= VERTEX_POINT("))
        .filter_map(|l| referenced_ids(l).first().copied())
        .collect();
    let mut largest: f64 = 0.0;
    let out = text
        .lines()
        .map(|line| {
            let Some(id) = record_id(line) else {
                return line.to_string();
            };
            if !line.contains("= CARTESIAN_POINT(") || !referenced.contains(&id) {
                return line.to_string();
            }
            let [x, y, z] = point_coords(line);
            let step = |rng: &mut Rng| rng.range(-1.0, 1.0) * amplitude / 3.0_f64.sqrt();
            let (dx, dy, dz) = (step(rng), step(rng), step(rng));
            largest = largest.max((dx * dx + dy * dy + dz * dz).sqrt());
            // Fixed-point for the same reason `unsew` uses it: a coordinate
            // near zero must not come out in exponent notation.
            format!(
                "#{id} = CARTESIAN_POINT('',({:.12},{:.12},{:.12}));",
                x + dx,
                y + dy,
                z + dz
            )
        })
        .collect::<Vec<_>>()
        .join("\n");
    (out, largest)
}

/// Flip the orientation flag of a random subset of `ADVANCED_FACE` records.
/// Returns the damaged text and how many faces were flipped.
fn flip_face_senses(text: &str, rng: &mut Rng) -> (String, usize) {
    let mut flipped = 0;
    let out = text
        .lines()
        .map(|line| {
            if !line.contains("= ADVANCED_FACE(") || rng.pick(2) == 0 {
                return line.to_string();
            }
            let (replaced, hit) = if let Some(head) = line.strip_suffix(".T.);") {
                (format!("{head}.F.);"), true)
            } else if let Some(head) = line.strip_suffix(".F.);") {
                (format!("{head}.T.);"), true)
            } else {
                (line.to_string(), false)
            };
            if hit {
                flipped += 1;
            }
            replaced
        })
        .collect::<Vec<_>>()
        .join("\n");
    (out, flipped)
}

// ---------------------------------------------------------------------
// Import
// ---------------------------------------------------------------------

/// Upper bound on jitter amplitude, as a fraction of the bounding-box
/// diagonal: the reader's own `TRIM_TOL_REL`, one decade tighter than
/// [`HEAL_GAP_REL`](opensolid_kernel::io::step::heal::HEAL_GAP_REL). Above
/// this a jittered vertex no longer lies on its edge's curve and the file is
/// rejected on geometry, never reaching the repair under test.
const TRIM_TOL_REL: f64 = 1e-6;

fn read_options(strategy: HealStrategy, max_gap: Option<f64>) -> StepReadOptions {
    StepReadOptions {
        heal: HealOptions { strategy, max_gap },
        ..StepReadOptions::default()
    }
}

/// Import `text` and return the exact body's volume, additionally requiring
/// that the *geometric* check clears it.
///
/// The topology-only `TopologyStore::check` holds no geometry, so it can see
/// neither a face whose sense flag contradicts its own loop's winding — of-hrgt
/// was exactly that body, certified by `check()` and then refused by the
/// measurement it was certified for — nor a vertex that has drifted off its
/// edge's curve, which is the whole subject of a jitter campaign.
/// `check_with_geometry` measures both.
///
/// Every case in this file uses this now. The jittered ones were fenced off
/// from it until of-bbh8: healing absorbed a merged corner's displacement into
/// the survivor's tolerance up to, rather than on top of, what its members were
/// already carrying, so `VertexOffEdge` fired at about 1.01× the elevated
/// allowance on a body the healer reported as repaired. Repairing the graph
/// while leaving the surviving entities claiming a precision they no longer
/// have is not a repair, and the fence is retired.
fn exact_import_volume_checked(
    text: &str,
    options: &StepReadOptions,
    context: &str,
) -> Option<f64> {
    exact_import(text, options, context, true)
}

/// Import `text` and return the exact body's volume, or `None` if the import
/// did not produce a checker-clean exact B-Rep solid.
fn exact_import_volume(text: &str, options: &StepReadOptions, context: &str) -> Option<f64> {
    exact_import(text, options, context, false)
}

fn exact_import(
    text: &str,
    options: &StepReadOptions,
    context: &str,
    geometric: bool,
) -> Option<f64> {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let import = read_step(text, &mut store, &mut geo, options)
        .unwrap_or_else(|e| panic!("{context}: the damaged file must still PARSE: {e:?}"));
    assert_eq!(
        import.solids.len(),
        1,
        "{context}: expected exactly one solid, got {}",
        import.solids.len()
    );
    match import.solids[0].outcome {
        SolidOutcome::BRep(body) => {
            let failures = store.check(body);
            assert!(
                failures.is_empty(),
                "{context}: an EXACT import must be valid, but check() reported {} \
                 failures: {:#?}",
                failures.len(),
                failures
            );
            if geometric {
                let failures = store.check_with_geometry(&geo, body);
                assert!(
                    failures.is_empty(),
                    "{context}: an EXACT import must be geometrically valid, but \
                     check_with_geometry() reported {} failures: {:#?}",
                    failures.len(),
                    failures
                );
            }
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

// =====================================================================
// (0) Baseline: the undamaged corpus must round-trip
// =====================================================================

/// Every generated case must export and re-import exactly before any damage
/// is applied. Without this, a healing test that fails tells you nothing
/// about healing.
#[test]
fn undamaged_exports_round_trip_exactly() {
    let mut rng = Rng::new(0x_BA5E_11E0);
    for case in 0..12 {
        let export = export_random_primitive(&mut rng);
        let repro = format!("case {case}: baseline round trip of {}", export.label);
        let volume = exact_import_volume_checked(
            &export.text,
            &read_options(HealStrategy::Auto, None),
            &repro,
        )
        .unwrap_or_else(|| {
            panic!("{repro}: an UNDAMAGED export did not re-import as an exact B-Rep")
        });
        assert_within(volume, export.volume, 1e-9, &repro);
    }
}

// =====================================================================
// (1) Vertex gaps within the healer's tolerance
// =====================================================================

/// Independently jittered corner copies, well inside the configured gap,
/// must heal to a valid exact body of the original volume.
///
/// The volume assertion is the point. Merging vertices moves material: each
/// merged corner lands at the centroid of the copies, so the healed solid is
/// a slightly different polyhedron. It must differ by no more than the gap
/// allows — a healer that repairs the topology while quietly deforming the
/// part is not a healer.
#[test]
fn unsewn_shells_within_the_gap_heal_to_an_exact_import() {
    let mut rng = Rng::new(0x_9AF0_1CE1);
    for case in 0..16 {
        let export = export_random_primitive(&mut rng);
        // An explicit gap, so the test controls both sides of the contract
        // rather than depending on `HEAL_GAP_REL`. The amplitude is capped
        // by the reader's trim tolerance, not by the gap (see module docs):
        // corner copies then separate by up to 2× the amplitude, which is
        // what healing merges, while each stays on its edge's curve.
        let max_gap = export.diagonal * 1e-4;
        let amplitude = export.diagonal * TRIM_TOL_REL * rng.range(0.05, 0.4);
        let (damaged, largest) = unsew(&export.text, &mut rng, amplitude);
        assert!(
            largest > 0.0,
            "case {case}: the jitter moved nothing — the corpus has no VERTEX_POINT \
             records to damage"
        );
        let repro = format!(
            "case {case}: {} jittered by up to {largest:.3e} (gap {max_gap:.3e})",
            export.label
        );

        let volume = exact_import_volume_checked(
            &damaged,
            &read_options(HealStrategy::Auto, Some(max_gap)),
            &repro,
        )
        .unwrap_or_else(|| {
            panic!(
                "{repro}: healing did not recover an exact B-Rep from a gap well \
                 inside its own tolerance"
            )
        });

        // A merged corner moves at most `largest`; over a body of this
        // diagonal that is a vanishing fraction of the volume, but bound it
        // generously by the linearized worst case (relative displacement,
        // three dimensions).
        let budget = 6.0 * largest / export.diagonal;
        assert_within(volume, export.volume, budget.max(1e-9), &repro);
    }
}

/// The other half of the `max_gap` contract: a gap far outside the tolerance
/// must **not** be silently merged.
///
/// Healing that fuses whatever it finds passes every "did it repair?" test
/// while quietly welding features that were meant to be distinct. The
/// documented behaviour is to refuse — the import then degrades to the mesh
/// fallback rather than returning a confidently wrong solid.
#[test]
fn gaps_beyond_the_tolerance_are_not_silently_merged() {
    let mut rng = Rng::new(0x_DEAD_9A70);
    for case in 0..12 {
        let export = export_random_primitive(&mut rng);
        // A deliberately tight gap, with the jitter still under the reader's
        // trim tolerance so the file is geometrically acceptable and only
        // the merge decision is under test.
        let max_gap = export.diagonal * 1e-9;
        let amplitude = export.diagonal * TRIM_TOL_REL * rng.range(0.1, 0.4);
        let (damaged, largest) = unsew(&export.text, &mut rng, amplitude);
        let repro = format!(
            "case {case}: {} jittered by up to {largest:.3e}, {:.0}× the configured \
             gap {max_gap:.3e}",
            export.label,
            largest / max_gap
        );

        if let Some(volume) = exact_import_volume(
            &damaged,
            &read_options(HealStrategy::Auto, Some(max_gap)),
            &repro,
        ) {
            panic!(
                "{repro}: healing returned an exact B-Rep of volume {volume} from gaps \
                 two orders of magnitude outside its own tolerance — it merged what it \
                 was told not to merge"
            );
        }
    }
}

// =====================================================================
// (2) Face orientation
// =====================================================================

/// Random `ADVANCED_FACE` sense flips must be repaired: the orientation pass
/// makes every shared edge consistent and reverses any shell left enclosing
/// the wrong sign of volume.
///
/// The volume must come back **positive and exact**. A shell that is
/// consistently oriented but globally inside-out passes a per-edge
/// consistency check and encloses `-V`; asserting the signed volume against
/// the closed form is what distinguishes the two.
#[test]
fn random_face_sense_flips_are_repaired() {
    let mut rng = Rng::new(0x_F11E_D001);
    for case in 0..16 {
        let export = export_random_primitive(&mut rng);
        let (damaged, flipped) = flip_face_senses(&export.text, &mut rng);
        if flipped == 0 {
            continue;
        }
        let repro = format!(
            "case {case}: {} with {flipped} ADVANCED_FACE sense(s) flipped",
            export.label
        );

        let volume = exact_import_volume(&damaged, &read_options(HealStrategy::Auto, None), &repro)
            .unwrap_or_else(|| {
                panic!("{repro}: the orientation pass did not recover an exact B-Rep")
            });
        assert!(
            volume > 0.0,
            "{repro}: healed body encloses NEGATIVE volume {volume} — the shell is \
             consistently oriented but globally inside-out"
        );
        assert_within(volume, export.volume, 1e-9, &repro);
    }
}

/// Flipping *every* face sense produces a fully-reversed but internally
/// consistent shell — the case the per-edge orientation check cannot see and
/// only the enclosed-volume sign test catches.
#[test]
fn wholly_reversed_shells_are_repaired() {
    let mut rng = Rng::new(0x_5E11_0001);
    for case in 0..8 {
        let export = export_random_primitive(&mut rng);
        let damaged = export
            .text
            .lines()
            .map(|line| {
                if line.contains("= ADVANCED_FACE(") {
                    if let Some(head) = line.strip_suffix(".T.);") {
                        return format!("{head}.F.);");
                    }
                    if let Some(head) = line.strip_suffix(".F.);") {
                        return format!("{head}.T.);");
                    }
                }
                line.to_string()
            })
            .collect::<Vec<_>>()
            .join("\n");
        let repro = format!(
            "case {case}: {} with EVERY face sense flipped",
            export.label
        );

        let volume =
            exact_import_volume_checked(&damaged, &read_options(HealStrategy::Auto, None), &repro)
                .unwrap_or_else(|| panic!("{repro}: a wholly reversed shell was not repaired"));
        assert!(
            volume > 0.0,
            "{repro}: healed body still encloses NEGATIVE volume {volume}"
        );
        assert_within(volume, export.volume, 1e-9, &repro);
    }
}

/// A reversed sense flag on **any single face** of a cylinder must import to
/// the same body the clean file does — the regression for of-hrgt.
///
/// Deterministic, no randomness: a cylinder exported to STEP, then re-imported
/// three times with one `ADVANCED_FACE` sense flag flipped each time.
///
/// This is the defect the bead was filed on, and it was worse than it looked.
/// Flipping a sense flag changes no fin sense, so the orientation pass's
/// two-colouring sees a perfectly consistent shell and plans nothing — and
/// the reader only consults the healer when the *topology-only*
/// `TopologyStore::check` fails, which it does not here, so healing was never
/// even invoked. The body came through as an "exact" import with one face's
/// flag contradicting its own loop's winding:
/// `check_with_geometry` reported `FaceSenseContradictsLoop`, tessellation
/// produced a non-manifold mesh, and `brep_mass_properties` refused the
/// *periodic* face with `OpenParameterLoop { gap: 4π }` — two revolutions of
/// `u`, from reading the seam's branch off the flag rather than the loop.
///
/// The two planar caps were the quieter half: `brep_mass_properties` takes its
/// signs from the winding alone, so those bodies measured *correctly* while
/// being just as malformed. Asserting all three faces is what keeps that half
/// covered.
#[test]
fn a_reversed_face_sense_flag_is_corrected_on_import() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = primitives::cylinder(&mut store, &mut geo, 3.0, 8.0).expect("valid cylinder");
    let text =
        write_step(&store, &geo, &[body], &StepWriteOptions::default()).expect("cylinder exports");

    // `primitives::cylinder` emits the two planar caps first and the periodic
    // wall last, so face 2 is the seam-carrying one.
    let face_lines: Vec<usize> = text
        .lines()
        .enumerate()
        .filter(|(_, l)| l.contains("= ADVANCED_FACE("))
        .map(|(i, _)| i)
        .collect();
    assert_eq!(face_lines.len(), 3, "the cylinder has three faces");

    for (which, &target) in face_lines.iter().enumerate() {
        let damaged = text
            .lines()
            .enumerate()
            .map(|(i, l)| {
                if i != target {
                    return l.to_string();
                }
                l.strip_suffix(".T.);")
                    .map(|h| format!("{h}.F.);"))
                    .or_else(|| l.strip_suffix(".F.);").map(|h| format!("{h}.T.);")))
                    .unwrap_or_else(|| l.to_string())
            })
            .collect::<Vec<_>>()
            .join("\n");
        let repro = format!("cylinder with face #{which}'s ADVANCED_FACE sense flipped");

        let volume =
            exact_import_volume_checked(&damaged, &read_options(HealStrategy::Auto, None), &repro)
                .unwrap_or_else(|| panic!("{repro}: healing must recover an exact B-Rep"));
        assert_within(volume, PI * 3.0 * 3.0 * 8.0, 1e-9, &repro);
    }
}

// =====================================================================
// (3) Strategy contracts
// =====================================================================

/// `HealStrategy::ReportOnly` must not mutate: the import may not come back
/// as a repaired exact body. `HealStrategy::Off` must not repair either.
/// Both are asserted against the *same damaged file* that `Auto` heals, so
/// the comparison isolates the strategy and nothing else.
#[test]
fn non_applying_strategies_do_not_repair_what_auto_repairs() {
    let mut rng = Rng::new(0x_5772_A7E9);
    for case in 0..10 {
        let export = export_random_primitive(&mut rng);
        let max_gap = export.diagonal * 1e-4;
        let amplitude = export.diagonal * TRIM_TOL_REL * 0.2;
        let (damaged, largest) = unsew(&export.text, &mut rng, amplitude);
        let repro = format!(
            "case {case}: {} jittered by up to {largest:.3e}",
            export.label
        );

        // Auto repairs it — establishing that there is something to repair.
        let healed = exact_import_volume_checked(
            &damaged,
            &read_options(HealStrategy::Auto, Some(max_gap)),
            &format!("{repro}: Auto"),
        )
        .unwrap_or_else(|| panic!("{repro}: Auto failed to heal, so the contrast is moot"));
        assert!(healed > 0.0, "{repro}: healed volume must be positive");

        for strategy in [HealStrategy::ReportOnly, HealStrategy::Off] {
            let ctx = format!("{repro}: {strategy:?}");
            if let Some(volume) =
                exact_import_volume(&damaged, &read_options(strategy, Some(max_gap)), &ctx)
            {
                panic!(
                    "{ctx}: a non-applying strategy returned a repaired exact B-Rep of \
                     volume {volume} — it mutated the body it promised not to touch"
                );
            }
        }
    }
}

/// The sense repair obeys the strategy like every other pass: only the
/// strategies that both orient and apply may correct a flag.
///
/// This one needs its own case because it is the one repair that runs from the
/// *reader* rather than from `GeometryHealer::heal` — it reads a loop's
/// winding, which only exists once pcurves are attached, which happens after
/// healing. A pass wired in at a different place is a pass that can miss the
/// strategy gate, and this is what would catch that.
///
/// A non-applying strategy still returns an exact B-Rep here (the flag defect
/// is invisible to the topology-only check that gates the exact path), so the
/// contrast is drawn on `check_with_geometry`, which sees the contradiction
/// the flag makes with its own loop.
#[test]
fn only_applying_strategies_correct_a_face_sense_flag() {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = primitives::cylinder(&mut store, &mut geo, 3.0, 8.0).expect("valid cylinder");
    let text =
        write_step(&store, &geo, &[body], &StepWriteOptions::default()).expect("cylinder exports");
    let (damaged, flipped) = flip_face_senses(&text, &mut Rng::new(0x_5E11_5E00));
    assert!(flipped > 0, "the damage must actually flip something");

    for (strategy, repaired) in [
        (HealStrategy::Auto, true),
        (HealStrategy::Minimal, false),
        (HealStrategy::ReportOnly, false),
        (HealStrategy::Off, false),
    ] {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let import = read_step(
            &damaged,
            &mut store,
            &mut geo,
            &read_options(strategy, None),
        )
        .expect("the damaged file still parses");
        let SolidOutcome::BRep(body) = import.solids[0].outcome else {
            panic!("{strategy:?}: expected an exact B-Rep import");
        };
        let failures = store.check_with_geometry(&geo, body);
        assert_eq!(
            failures.is_empty(),
            repaired,
            "{strategy:?}: expected repaired = {repaired}, check_with_geometry() reported \
             {failures:#?}"
        );
    }
}

/// Healing must reach a **fixed point**: once a body has been sewn, further
/// export/import cycles must leave it alone. A repair pass that keeps finding
/// work to do on its own output has not converged, and the damage compounds
/// with every trip through a file.
///
/// The first cycle is allowed to move the body a little and the ones after it
/// are not, which is the honest statement rather than a uniform tolerance.
/// Healing merges each corner's copies to their centroid, so a healed vertex
/// no longer sits exactly on the analytic surface it came from; re-importing
/// re-derives every edge's trim parameters from that moved vertex, and on a
/// curved body (a cone's radius enters its volume squared) that shows up in
/// the eighth digit. It is a one-time cost of the repair, not a drift: from
/// the second cycle on, the file already describes a sewn body, healing has
/// nothing to merge, and the volume must be reproduced to floating point.
#[test]
fn healing_reaches_a_fixed_point_under_repeated_round_trips() {
    /// One-time cost of the first re-export: the healed body's vertices have
    /// moved off their analytic surfaces by up to the merge gap.
    const FIRST_CYCLE: f64 = 1e-6;
    let mut rng = Rng::new(0x1DE1_1DE0);
    for case in 0..8 {
        let export = export_random_primitive(&mut rng);
        let max_gap = export.diagonal * 1e-4;
        let (damaged, _) = unsew(
            &export.text,
            &mut rng,
            export.diagonal * TRIM_TOL_REL * 0.25,
        );
        let repro = format!("case {case}: fixed point over {}", export.label);

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let import = read_step(
            &damaged,
            &mut store,
            &mut geo,
            &read_options(HealStrategy::Auto, Some(max_gap)),
        )
        .unwrap_or_else(|e| panic!("{repro}: read failed: {e:?}"));
        let SolidOutcome::BRep(body) = import.solids[0].outcome else {
            panic!("{repro}: first import was not exact, so convergence is untestable");
        };
        let mut previous = brep_mass_properties(&store, &geo, body)
            .unwrap_or_else(|e| panic!("{repro}: measurement failed: {e:?}"))
            .volume;
        let mut text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
            .unwrap_or_else(|e| panic!("{repro}: re-export failed: {e:?}"));

        for cycle in 1..=3 {
            let ctx = format!("{repro}: cycle {cycle}");
            let mut store = TopologyStore::new();
            let mut geo = GeometryStore::new();
            let import = read_step(
                &text,
                &mut store,
                &mut geo,
                &read_options(HealStrategy::Auto, Some(max_gap)),
            )
            .unwrap_or_else(|e| panic!("{ctx}: read failed: {e:?}"));
            let SolidOutcome::BRep(body) = import.solids[0].outcome else {
                panic!("{ctx}: a HEALED body failed to re-import exactly");
            };
            let failures = store.check_with_geometry(&geo, body);
            assert!(
                failures.is_empty(),
                "{ctx}: re-imported body failed check_with_geometry() with {} \
                 failures: {failures:#?}",
                failures.len()
            );
            let volume = brep_mass_properties(&store, &geo, body)
                .unwrap_or_else(|e| panic!("{ctx}: measurement failed: {e:?}"))
                .volume;

            // Cycle 1 pays the one-time re-trim cost; every later cycle must
            // be an exact fixed point.
            let budget = if cycle == 1 { FIRST_CYCLE } else { 1e-9 };
            assert_within(
                volume,
                previous,
                budget,
                &format!("{ctx}: volume vs the previous cycle"),
            );
            previous = volume;
            text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
                .unwrap_or_else(|e| panic!("{ctx}: re-export failed: {e:?}"));
        }

        // And the fixed point must still be the part that was exported.
        assert_within(
            previous,
            export.volume,
            FIRST_CYCLE,
            &format!("{repro}: converged volume vs the original part"),
        );
    }
}

// =====================================================================
// (4) The tolerance an import carries
// =====================================================================

/// A **sewn** file whose vertex coordinates have been jittered must import to
/// a body that passes `check_with_geometry` — the reader half of of-bbh8.
///
/// Nothing here is healing's to fix: every corner is still one shared
/// `VERTEX_POINT`, so no gap opens, the topology-only check passes, and the
/// healer is never even consulted. What the jitter does is move each vertex
/// off the curves its edges carry, by up to the reader's own `TRIM_TOL_REL` —
/// which the reader *accepts*, then recorded nowhere, creating every vertex at
/// `SYSTEM_RESOLUTION`. The result was a body certified by `check()` and
/// rejected by `check_with_geometry()` with a `VertexOffEdge` per corner, on a
/// file no repair had touched.
///
/// The volume assertion is the second half: carrying the miss as tolerance is
/// the correct repair only if the vertex is *not* moved to make it true. The
/// jitter is bounded by the trim tolerance, so the part must still measure to
/// within it.
#[test]
fn sewn_jitter_is_carried_as_tolerance() {
    let mut rng = Rng::new(0x_7011_E7A0);
    for case in 0..12 {
        let export = export_random_primitive(&mut rng);
        let amplitude = export.diagonal * TRIM_TOL_REL * rng.range(0.05, 0.4);
        let (damaged, largest) = jitter_vertex_points(&export.text, &mut rng, amplitude);
        assert!(
            largest > 0.0,
            "case {case}: the jitter moved nothing — the corpus has no VERTEX_POINT \
             records to damage"
        );
        let repro = format!(
            "case {case}: {} sewn, jittered by up to {largest:.3e}",
            export.label
        );

        // `HealStrategy::Off`: there is no repair to make, and saying so here
        // is what keeps this a statement about the reader.
        let volume =
            exact_import_volume_checked(&damaged, &read_options(HealStrategy::Off, None), &repro)
                .unwrap_or_else(|| {
                    panic!("{repro}: a sewn file within the trim tolerance must import exactly")
                });

        let budget = 6.0 * largest / export.diagonal;
        assert_within(volume, export.volume, budget.max(1e-9), &repro);
    }
}
