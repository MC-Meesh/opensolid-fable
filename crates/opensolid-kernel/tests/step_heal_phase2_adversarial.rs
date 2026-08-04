//! Adversarial campaign for STEP healing **phase 2** (of-x3go, pairing the
//! implementation bead of-3qy.14): edge/surface consistency repair, edge-curve
//! recomputation from surface/surface intersection, and pcurve refitting —
//! plus their interaction with the phase-1 heals (vertex merge, edge weld,
//! orientation) on the same file.
//!
//! # What phase 2 is, in one paragraph
//!
//! Phase 1 repairs the *graph*: it merges duplicated corners, welds duplicated
//! edges, and flips faces until every mate is traversed twice in opposite
//! directions. Phase 2 repairs the *geometry the graph carries*: an edge whose
//! curve strays from the surfaces of the faces it bounds is either covered
//! honestly by tolerance (within [`MAX_ALLOWED_TOLERANCE`]) or has its curve
//! replaced outright by the intersection of those two surfaces — and a fin
//! whose 2D trim no longer tracks its edge's 3D curve gets the trim refit.
//!
//! # The hostile construction this file adds: the bulged arc
//!
//! The phase-1 campaign (`step_heal_random.rs`) records why plain coordinate
//! jitter cannot produce edge/surface inconsistency: moving a shared
//! `VERTEX_POINT` moves the corner *and* the trim together, and moving it far
//! enough to matter trips the reader's vertex-on-curve validation
//! (`TRIM_TOL_REL`) before healing is ever consulted. Producing a file whose
//! edge *curve* is wrong while its vertices are right takes a different
//! weapon: replace a straight block edge's `LINE` with a `CIRCLE` arc through
//! the same two corners. The endpoints sit on the arc exactly — the reader's
//! trim validation passes — while the arc's midpoint bulges off both adjacent
//! planes by the sagitta's projection. The sagitta dials the deviation:
//! under [`MAX_ALLOWED_TOLERANCE`] is the elevate band (the reader absorbs it
//! as edge tolerance), past it is the rescue band (only a curve recompute can
//! save the body).
//!
//! Protocol as `step_heal_random.rs`: deterministic seeded [`Rng`] remixed by
//! `OPENSOLID_CAMPAIGN_SEED`, a repro string on every failure; failures become
//! `bd` beads and the case is `#[ignore]`d referencing the bead, never
//! softened.

use opensolid_brep::topology::{Body, Edge, Fin};
use opensolid_brep::{
    Curve2, Curve3, GeometryStore, MAX_ALLOWED_TOLERANCE, TessellationOptions, TopologyStore,
    attach_body_pcurves, primitives, tessellate_body,
};
use opensolid_core::{EntityId, Point2, Vector2, Vector3};
use opensolid_kernel::brep_mass_properties;
use opensolid_kernel::io::step::{
    GeometryHealer, HealOperation, HealOptions, HealStrategy, SolidOutcome, StepReadOptions,
    StepWriteOptions, read_step, write_step,
};
use std::collections::HashSet;
use std::f64::consts::{PI, TAU};

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), protocol-identical to the phase-1
// campaign in `step_heal_random.rs`.
// ---------------------------------------------------------------------

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

    fn sign(&mut self) -> f64 {
        if self.pick(2) == 0 { 1.0 } else { -1.0 }
    }
}

// ---------------------------------------------------------------------
// STEP text plumbing, shared with the phase-1 campaign.
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

/// **Unsew** the shell exactly as the phase-1 campaign does: every
/// `ORIENTED_EDGE` gets a private copy of its `EDGE_CURVE` and of that edge's
/// two `VERTEX_POINT`s, each independently jittered by at most `amplitude`.
/// The underlying curve stays shared. Returns the damaged text and the
/// largest displacement applied.
fn unsew(text: &str, rng: &mut Rng, amplitude: f64) -> (String, f64) {
    use std::collections::HashMap;

    let by_id: HashMap<u64, &str> = text
        .lines()
        .filter_map(|l| record_id(l).map(|id| (id, l)))
        .collect();
    let mut next_id = by_id.keys().copied().max().unwrap_or(0) + 1;
    let mut added: Vec<String> = Vec::new();
    let mut largest: f64 = 0.0;

    let copy_vertex = |vertex_id: u64,
                       rng: &mut Rng,
                       next_id: &mut u64,
                       added: &mut Vec<String>,
                       largest: &mut f64|
     -> u64 {
        let vertex_line = by_id[&vertex_id];
        let point_id = referenced_ids(vertex_line)[0];
        let [x, y, z] = point_coords(by_id[&point_id]);
        let step = |rng: &mut Rng| rng.range(-1.0, 1.0) * amplitude / 3.0_f64.sqrt();
        let (dx, dy, dz) = (step(rng), step(rng), step(rng));
        *largest = largest.max((dx * dx + dy * dy + dz * dz).sqrt());
        let new_point = *next_id;
        *next_id += 1;
        // Fixed-point, not `{:?}`: a coordinate near zero must not come out
        // in exponent notation.
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

    let end = rewritten
        .iter()
        .rposition(|l| l.trim() == "ENDSEC;")
        .expect("the data section closes");
    for (i, record) in added.into_iter().enumerate() {
        rewritten.insert(end + i, record);
    }
    (rewritten.join("\n"), largest)
}

/// Flip the orientation flag of a random subset of `ADVANCED_FACE` records.
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
// The phase-2 weapon: bulge a straight block edge into a circular arc.
// ---------------------------------------------------------------------

/// What [`bulge_edge_into_arc`] did, for assertions and repro strings.
struct Bulge {
    /// Which STEP edge was rewritten.
    edge_id: u64,
    /// Off-plane deviation at the arc midpoint — what
    /// `measure_edges_off_surfaces` will report for this edge.
    deviation: f64,
    /// Chord length of the replaced edge.
    chord: f64,
}

/// Replace one straight block edge's `LINE` with a `CIRCLE` arc through the
/// same two corner `VERTEX_POINT`s, bulging off **both** adjacent planes by
/// `deviation` at the midpoint.
///
/// The construction that slips past the reader: the corners sit on the arc
/// *exactly*, so the vertex-on-curve trim validation (`TRIM_TOL_REL`)
/// passes, while the interior of the edge is off both faces' surfaces by up
/// to `deviation` — plain coordinate displacement can do no such thing,
/// because moving a vertex off its curve is rejected before healing runs.
///
/// Geometry: for a chord of length `L` and a bulge direction `d` (unit,
/// perpendicular to the chord, at 45° to each adjacent plane's normal so the
/// midpoint leaves both planes by `s/√2`), the sagitta is `s = deviation·√2`,
/// the radius `R = L²/(8s) + s/2`, and the center sits at `M − (R−s)·d`.
/// The circle's axis is `d × u` (chord direction `u`), which makes the
/// counterclockwise sweep from the start vertex to the end vertex — the arc
/// [`trim_curve`] recovers — the *minor* arc through the bulge, not the far
/// side of the circle.
fn bulge_edge_into_arc(text: &str, rng: &mut Rng, deviation: f64) -> (String, Bulge) {
    use std::collections::HashMap;

    let by_id: HashMap<u64, &str> = text
        .lines()
        .filter_map(|l| record_id(l).map(|id| (id, l)))
        .collect();

    // Candidate edges: every EDGE_CURVE between two distinct vertices whose
    // chord is axis-aligned (all of a block's are).
    let mut candidates = Vec::new();
    for line in text.lines() {
        if !line.contains("= EDGE_CURVE(") {
            continue;
        }
        let id = record_id(line).expect("EDGE_CURVE lines define ids");
        let refs = referenced_ids(line);
        let (v1, v2) = (refs[0], refs[1]);
        if v1 == v2 {
            continue;
        }
        let p1 = point_coords(by_id[&referenced_ids(by_id[&v1])[0]]);
        let p2 = point_coords(by_id[&referenced_ids(by_id[&v2])[0]]);
        let differs: Vec<usize> = (0..3).filter(|&i| (p1[i] - p2[i]).abs() > 1e-9).collect();
        if differs.len() == 1 {
            candidates.push((id, v1, v2, p1, p2, differs[0]));
        }
    }
    let (edge_id, v1, v2, p1, p2, axis_index) = candidates[rng.pick(candidates.len())];

    let chord = (p2[axis_index] - p1[axis_index]).abs();
    let mut u = [0.0; 3];
    u[axis_index] = (p2[axis_index] - p1[axis_index]).signum();
    // Bulge direction: equal parts of the two axes perpendicular to the
    // chord, random signs — 45° to each adjacent plane's normal.
    let (j, k) = match axis_index {
        0 => (1, 2),
        1 => (0, 2),
        _ => (0, 1),
    };
    let mut d = [0.0; 3];
    let inv_sqrt2 = 1.0 / 2.0_f64.sqrt();
    d[j] = rng.sign() * inv_sqrt2;
    d[k] = rng.sign() * inv_sqrt2;

    let s = deviation * 2.0_f64.sqrt();
    let radius = chord * chord / (8.0 * s) + s / 2.0;
    let mid = [
        (p1[0] + p2[0]) / 2.0,
        (p1[1] + p2[1]) / 2.0,
        (p1[2] + p2[2]) / 2.0,
    ];
    let center = [
        mid[0] - (radius - s) * d[0],
        mid[1] - (radius - s) * d[1],
        mid[2] - (radius - s) * d[2],
    ];
    // axis = d × u: the ccw sweep start → end then runs through the bulge.
    let axis = [
        d[1] * u[2] - d[2] * u[1],
        d[2] * u[0] - d[0] * u[2],
        d[0] * u[1] - d[1] * u[0],
    ];

    let mut next_id = by_id.keys().copied().max().unwrap_or(0) + 1;
    let mut added = Vec::new();
    let center_id = next_id;
    added.push(format!(
        "#{center_id} = CARTESIAN_POINT('',({:.12},{:.12},{:.12}));",
        center[0], center[1], center[2]
    ));
    next_id += 1;
    let axis_id = next_id;
    added.push(format!(
        "#{axis_id} = DIRECTION('',({:.12},{:.12},{:.12}));",
        axis[0], axis[1], axis[2]
    ));
    next_id += 1;
    let placement_id = next_id;
    added.push(format!(
        "#{placement_id} = AXIS2_PLACEMENT_3D('',#{center_id},#{axis_id},$);"
    ));
    next_id += 1;
    let circle_id = next_id;
    added.push(format!(
        "#{circle_id} = CIRCLE('',#{placement_id},{radius:.12});"
    ));

    let rewritten = text
        .lines()
        .map(|line| {
            if record_id(line) == Some(edge_id) {
                format!("#{edge_id} = EDGE_CURVE('',#{v1},#{v2},#{circle_id},.T.);")
            } else {
                line.to_string()
            }
        })
        .collect::<Vec<_>>();
    let mut rewritten = rewritten;
    let end = rewritten
        .iter()
        .rposition(|l| l.trim() == "ENDSEC;")
        .expect("the data section closes");
    for (i, record) in added.into_iter().enumerate() {
        rewritten.insert(end + i, record);
    }
    (
        rewritten.join("\n"),
        Bulge {
            edge_id,
            deviation,
            chord,
        },
    )
}

// ---------------------------------------------------------------------
// Corpus + import plumbing.
// ---------------------------------------------------------------------

/// Amplitude cap for vertex jitter, as a fraction of the diagonal — the
/// reader's own trim tolerance, above which a jittered vertex is rejected on
/// geometry before healing is consulted.
const TRIM_TOL_REL: f64 = 1e-6;

struct Exported {
    label: String,
    text: String,
    volume: f64,
    diagonal: f64,
}

/// A random block, exported at millimetre scale. Blocks only: the bulged-arc
/// construction needs an edge whose two adjacent surfaces are planes, so the
/// analytic intersection (the rescue's ground truth) is the original line.
fn export_random_block(rng: &mut Rng) -> Exported {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let s = [
        rng.range(2.0, 20.0),
        rng.range(2.0, 20.0),
        rng.range(2.0, 20.0),
    ];
    let body = primitives::block(&mut store, &mut geo, s[0], s[1], s[2]).expect("valid extents");
    let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
        .expect("a primitive block exports");
    Exported {
        label: format!("block({:.4}, {:.4}, {:.4})", s[0], s[1], s[2]),
        text,
        volume: s[0] * s[1] * s[2],
        diagonal: (s[0] * s[0] + s[1] * s[1] + s[2] * s[2]).sqrt(),
    }
}

fn read_options(strategy: HealStrategy, max_gap: Option<f64>) -> StepReadOptions {
    StepReadOptions {
        heal: HealOptions { strategy, max_gap, ..HealOptions::default() },
        ..StepReadOptions::default()
    }
}

/// Import to an exact body, then hand the store/geo/body to `inspect` for
/// case-specific assertions, and return the measured volume. Requires the
/// geometric check — the one that sees edges off their surfaces — to pass.
fn exact_import_inspected(
    text: &str,
    options: &StepReadOptions,
    context: &str,
    inspect: impl FnOnce(&TopologyStore, &GeometryStore, EntityId<Body>),
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
            let failures = store.check_with_geometry(&geo, body);
            assert!(
                failures.is_empty(),
                "{context}: an EXACT import must be geometrically valid, but \
                 check_with_geometry() reported {} failures: {:#?}",
                failures.len(),
                failures
            );
            tessellate_body(&store, &geo, body, &TessellationOptions::default())
                .unwrap_or_else(|e| panic!("{context}: healed body failed to tessellate: {e:?}"));
            inspect(&store, &geo, body);
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

/// Every edge of `body`, via its faces' loops (deduplicated).
fn body_edges(store: &TopologyStore, body: EntityId<Body>) -> Vec<EntityId<Edge>> {
    let mut seen = HashSet::new();
    let mut out = Vec::new();
    for face in store.faces_of_body(body) {
        for loop_id in store.loops_of_face(face) {
            for &fin in store.fins_of_loop(loop_id) {
                let edge = store.fin_edge(fin);
                if seen.insert(edge) {
                    out.push(edge);
                }
            }
        }
    }
    out
}

/// Every fin of `body`, in face → loop → fin order.
fn body_fins(store: &TopologyStore, body: EntityId<Body>) -> Vec<EntityId<Fin>> {
    let mut out = Vec::new();
    for face in store.faces_of_body(body) {
        for loop_id in store.loops_of_face(face) {
            out.extend(store.fins_of_loop(loop_id).iter().copied());
        }
    }
    out
}

// =====================================================================
// (1) The elevate band: a sub-cap bulged arc on a SEWN file
// =====================================================================

/// A sewn file whose edge curve bulges off both adjacent planes by less than
/// [`MAX_ALLOWED_TOLERANCE`] must import exactly, with the deviation recorded
/// **honestly as edge tolerance** — not silently, not by moving anything.
///
/// Nothing here is healing's to fix: the topology is perfect, so the healer
/// is never consulted, and `record_edge_tolerances` is the reader half under
/// test. The geometric check then holds the body to the tolerance it claims.
///
/// The volume budget is what a *tolerant* body can honestly promise, no
/// more: the two faces' trims follow the arc's projections, which meet only
/// within the edge's tolerance, so the boundary has a tolerance-covered gap
/// strip along the bulge — area `(2/3)·L·s` with sagitta `s = deviation·√2`
/// — and the divergence-theorem volume owns a flux error of up to that area
/// times the lever arm `|r·n| ≤ diagonal`, over 3. [`subcap_volume_budget`]
/// is that bound with 2× slack; measured errors sit at roughly a third of
/// it. Asserting tighter would demand a precision the body's own tolerance
/// says it does not have.
#[test]
fn a_subcap_bulged_arc_on_a_sewn_file_is_absorbed_as_edge_tolerance() {
    let mut rng = Rng::new(0x_B019_ED01);
    for case in 0..12 {
        let export = export_random_block(&mut rng);
        let deviation = rng.range(0.002, 0.008);
        let (damaged, bulge) = bulge_edge_into_arc(&export.text, &mut rng, deviation);
        let repro = format!(
            "case {case}: {} with edge #{} bulged into an arc, deviation {:.3e} mm \
             (sub-cap; chord {:.3} mm)",
            export.label, bulge.edge_id, bulge.deviation, bulge.chord
        );

        let volume = exact_import_inspected(
            &damaged,
            &read_options(HealStrategy::Auto, None),
            &repro,
            |store, _geo, body| {
                // The deviation must be carried openly: some edge now owns a
                // tolerance close to the bulge (record_edge_tolerances raises
                // to the *measured* deviation, sampled, so allow slack below).
                let max_tol = body_edges(store, body)
                    .into_iter()
                    .filter_map(|e| store.edge(e).map(|edge| edge.tolerance))
                    .fold(0.0f64, f64::max);
                assert!(
                    max_tol >= deviation * 0.8,
                    "{repro}: the import claims max edge tolerance {max_tol:.3e} for a \
                     curve that measurably strays {deviation:.3e} — the deviation was \
                     swallowed silently"
                );
            },
        )
        .unwrap_or_else(|| {
            panic!(
                "{repro}: a sewn file with a sub-cap geometric defect must import \
                 exactly (the elevate band exists for exactly this file)"
            )
        });
        let budget = subcap_volume_budget(&bulge, export.diagonal, export.volume);
        assert_within(volume, export.volume, budget, &repro);
    }
}

/// Relative volume budget for a body carrying a bulged-arc edge as
/// tolerance: `2 · (2/3)·chord·sagitta · diagonal / 3` over the volume —
/// the flux through the tolerance-covered gap strip along the edge, with 2×
/// slack. See [`a_subcap_bulged_arc_on_a_sewn_file_is_absorbed_as_edge_tolerance`].
fn subcap_volume_budget(bulge: &Bulge, diagonal: f64, volume: f64) -> f64 {
    let sagitta = bulge.deviation * 2.0_f64.sqrt();
    (2.0 * (2.0 / 3.0) * bulge.chord * sagitta * diagonal / 3.0 / volume).max(1e-9)
}

// =====================================================================
// (2) The rescue band: a past-cap bulged arc on an UNSEWN file
// =====================================================================

/// An unsewn file whose bulged edge strays **past** the kernel cap must come
/// back exact: healing sews the shell (phase 1), then the rescue pass
/// replaces the arc with the adjacent planes' intersection — the original
/// line — because no tolerance under the cap can cover where the arc is.
///
/// This is the one repair that changes geometry rather than tolerances, so
/// the assertions are the strong ones: the recompute must be *reported*, the
/// healed body must measure to the true volume within the vertex-merge
/// budget, and a re-export/re-import cycle must be a fixed point (a rescue
/// that leaves work for the next import to find has not converged).
#[test]
fn a_pastcap_bulged_arc_on_an_unsewn_file_is_rescued_by_ssi_recompute() {
    let mut rng = Rng::new(0x_5E5C_0E01);
    for case in 0..12 {
        let export = export_random_block(&mut rng);
        let deviation = rng.range(0.02, 0.05);
        let (bulged, bulge) = bulge_edge_into_arc(&export.text, &mut rng, deviation);
        let amplitude = export.diagonal * TRIM_TOL_REL * rng.range(0.05, 0.25);
        let (damaged, largest) = unsew(&bulged, &mut rng, amplitude);
        let repro = format!(
            "case {case}: {} with edge #{} bulged to {:.3e} mm (past-cap) then unsewn \
             with jitter {largest:.3e}",
            export.label, bulge.edge_id, bulge.deviation
        );

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let import = read_step(
            &damaged,
            &mut store,
            &mut geo,
            &read_options(HealStrategy::Auto, None),
        )
        .unwrap_or_else(|e| panic!("{repro}: the damaged file must still PARSE: {e:?}"));
        let SolidOutcome::BRep(body) = import.solids[0].outcome else {
            panic!(
                "{repro}: healing did not rescue the body — an unsewn shell brings the \
                 healer in, and the rescue pass exists for exactly this deviation"
            );
        };
        assert!(
            import
                .diagnostics
                .iter()
                .any(|d| d.message.contains("recomputed")),
            "{repro}: the import must REPORT the curve recompute it performed; \
             diagnostics: {:#?}",
            import.diagnostics
        );
        let failures = store.check_with_geometry(&geo, body);
        assert!(
            failures.is_empty(),
            "{repro}: rescued body failed check_with_geometry(): {failures:#?}"
        );
        tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .unwrap_or_else(|e| panic!("{repro}: rescued body failed to tessellate: {e:?}"));
        let volume = brep_mass_properties(&store, &geo, body)
            .unwrap_or_else(|e| panic!("{repro}: measurement failed: {e:?}"))
            .volume;
        // The rescue restores the exact line; what remains is the
        // vertex-merge budget, as in the phase-1 campaign.
        let budget = 6.0 * largest / export.diagonal;
        assert_within(volume, export.volume, budget.max(1e-9), &repro);

        // Fixed point: the healed body re-exports to a file with nothing
        // left to rescue. Cycle 1 may pay the one-time re-trim cost the
        // phase-1 campaign documents; the recompute must not recur.
        let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
            .unwrap_or_else(|e| panic!("{repro}: re-export failed: {e:?}"));
        let mut store2 = TopologyStore::new();
        let mut geo2 = GeometryStore::new();
        let reimport = read_step(
            &text,
            &mut store2,
            &mut geo2,
            &read_options(HealStrategy::Auto, None),
        )
        .unwrap_or_else(|e| panic!("{repro}: re-import failed: {e:?}"));
        let SolidOutcome::BRep(body2) = reimport.solids[0].outcome else {
            panic!("{repro}: a RESCUED body failed to re-import exactly");
        };
        assert!(
            !reimport
                .diagnostics
                .iter()
                .any(|d| d.message.contains("recomputed")),
            "{repro}: the second import recomputed a curve AGAIN — the rescue did not \
             converge; diagnostics: {:#?}",
            reimport.diagnostics
        );
        let volume2 = brep_mass_properties(&store2, &geo2, body2)
            .unwrap_or_else(|e| panic!("{repro}: cycle-1 measurement failed: {e:?}"))
            .volume;
        assert_within(
            volume2,
            volume,
            1e-6,
            &format!("{repro}: fixed-point cycle"),
        );
    }
}

// =====================================================================
// (3) The gate: the same past-cap defect on a SEWN file
// =====================================================================

/// The same past-cap bulged arc, on a file whose topology is perfect, must
/// still import exactly — "import heals, it does not reject"
/// (`spec/06-step-io.md` §4) does not say "…but only if the topology is also
/// broken".
///
/// Today it cannot: the reader consults the healer only when the
/// topology-only `TopologyStore::check` fails, so a sewn file goes straight
/// to `record_edge_tolerances`, which sees the past-cap deviation and drops
/// the whole solid to the mesh fallback. The rescue pass — built for exactly
/// this defect — is unreachable. Adding *more* damage (unsewing the same
/// file) makes the import strictly better, which is the inconsistency in one
/// sentence; the sibling test above proves the unsewn half.
#[test]
#[ignore = "of-du3v: phase-2 rescue is unreachable on topologically-sewn files"]
fn a_pastcap_bulged_arc_on_a_sewn_file_must_still_import_exactly() {
    let mut rng = Rng::new(0x_9A7E_D001);
    for case in 0..6 {
        let export = export_random_block(&mut rng);
        let deviation = rng.range(0.02, 0.05);
        let (damaged, bulge) = bulge_edge_into_arc(&export.text, &mut rng, deviation);
        let repro = format!(
            "case {case}: {} with edge #{} bulged to {:.3e} mm (past-cap), topology sewn",
            export.label, bulge.edge_id, bulge.deviation
        );

        let volume = exact_import_inspected(
            &damaged,
            &read_options(HealStrategy::Auto, None),
            &repro,
            |_, _, _| {},
        )
        .unwrap_or_else(|| {
            panic!(
                "{repro}: the solid degraded to the mesh fallback — the rescue \
                 recompute never ran, because healing is gated on the topology-only \
                 check that this (sewn) file passes"
            )
        });
        assert_within(volume, export.volume, 1e-6, &repro);
    }
}

// =====================================================================
// (4) Closed-edge recompute: cap circles, the branch with no unit coverage
// =====================================================================

/// Cap circles of cylinders and cones are **closed** edges — one vertex,
/// `t_end = t_start + period` — and the recompute's closed branch has no
/// other coverage (every in-file unit test uses open arcs and lines on
/// blocks). Corrupt each cap circle's curve (wrong radius, lifted off its
/// plane) past the kernel cap and demand the standalone consistency repair
/// recompute it from the plane/quadric intersection: deviation squeezed to
/// numerical zero, the closed range preserved as one full period, the vertex
/// untouched, and the body measuring exactly again.
#[test]
fn closed_cap_circles_recompute_to_the_analytic_intersection() {
    let mut rng = Rng::new(0x_C10C_ED04);
    for case in 0..10 {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let (body, volume, label) = if case % 2 == 0 {
            let (r, h) = (rng.range(1.0, 8.0), rng.range(2.0, 16.0));
            let body = primitives::cylinder(&mut store, &mut geo, r, h).expect("valid dimensions");
            (
                body,
                PI * r * r * h,
                format!("cylinder(r = {r:.4}, h = {h:.4})"),
            )
        } else {
            let (r0, h) = (rng.range(1.5, 8.0), rng.range(2.0, 12.0));
            let r1 = rng.range(0.2, 0.8) * r0;
            let body = primitives::cone(&mut store, &mut geo, r0, r1, h).expect("valid dimensions");
            (
                body,
                PI * h * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0,
                format!("cone(r0 = {r0:.4}, r1 = {r1:.4}, h = {h:.4})"),
            )
        };
        attach_body_pcurves(&mut store, &mut geo, body);
        assert!(
            store.check_with_geometry(&geo, body).is_empty(),
            "case {case}: {label}: the pristine fixture must be clean"
        );

        // Every closed circular edge is a cap circle (the seam is a line).
        let caps: Vec<EntityId<Edge>> = body_edges(&store, body)
            .into_iter()
            .filter(|&e| {
                let edge = store.edge(e).expect("live edge");
                edge.start_vertex == edge.end_vertex
                    && matches!(
                        edge.curve.and_then(|id| geo.curve(id)),
                        Some(Curve3::Circle { .. })
                    )
            })
            .collect();
        assert!(
            !caps.is_empty(),
            "case {case}: {label}: the fixture has closed cap circles"
        );
        let target = caps[rng.pick(caps.len())];
        let (center, axis, radius) = {
            let edge = store.edge(target).expect("live edge");
            let Some(Curve3::Circle {
                center,
                axis,
                radius,
            }) = edge.curve.and_then(|id| geo.curve(id)).cloned()
            else {
                unreachable!("filtered to circles");
            };
            (center, axis, radius)
        };
        // Corrupt: fatten past the cap and lift off the cap plane — an
        // exporter that wrote the trim circle against the wrong support.
        let fatten = rng.range(1.5, 5.0) * MAX_ALLOWED_TOLERANCE;
        let lift = rng.range(0.5, 3.0) * MAX_ALLOWED_TOLERANCE;
        let corrupt = Curve3::circle(center + axis * lift, axis, radius + fatten)
            .expect("a valid corrupted circle");
        let corrupt_id = geo.add_curve(corrupt);
        store.edges.get_mut(target).expect("live edge").curve = Some(corrupt_id);
        let repro = format!(
            "case {case}: {label}, cap circle {target:?} fattened by {fatten:.3e} and \
             lifted by {lift:.3e} (both past-cap capable)"
        );

        let result = GeometryHealer::fix_edge_surface_consistency(
            body,
            &mut store,
            &mut geo,
            &HealOptions::default(),
        );
        assert!(
            result.operations.iter().any(|op| matches!(
                op,
                HealOperation::EdgeCurveRecomputed { edge, .. } if *edge == target
            )),
            "{repro}: the repair must RECOMPUTE the closed edge, not elevate or skip; \
             operations: {:?}, notes: {:?}",
            result.operations,
            result.notes
        );

        let edge = store.edge(target).expect("the edge survives");
        assert_eq!(
            edge.start_vertex, edge.end_vertex,
            "{repro}: the recompute must preserve the closed topology"
        );
        let span = edge.t_end - edge.t_start;
        assert!(
            (span - TAU).abs() < 1e-9,
            "{repro}: a recomputed closed circle must span one full period, got {span}"
        );
        let worst = store
            .measure_edges_off_surfaces(&geo, body)
            .into_iter()
            .find(|(e, _)| *e == target)
            .map(|(_, d)| d)
            .unwrap_or(0.0);
        assert!(
            worst < 1e-9,
            "{repro}: the recomputed curve still strays {worst:.3e} from its faces' \
             surfaces — the analytic intersection is exact, so this must be numerical \
             zero"
        );
        let failures = store.check_with_geometry(&geo, body);
        assert!(
            failures.is_empty(),
            "{repro}: repaired body failed check_with_geometry(): {failures:#?}"
        );
        tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .unwrap_or_else(|e| panic!("{repro}: repaired body failed to tessellate: {e:?}"));
        let measured = brep_mass_properties(&store, &geo, body)
            .unwrap_or_else(|e| panic!("{repro}: measurement failed: {e:?}"))
            .volume;
        assert_within(measured, volume, 1e-9, &repro);
    }
}

// =====================================================================
// (5) Pcurve refits under hostile trims, including both seam uses
// =====================================================================

/// Corrupt fins' 2D trims — including the two uses of the cylinder seam,
/// whose branch (`SeamSide::Low`/`High`) is assigned by per-face **use
/// order** — and demand `fix_pcurves` refit every one of them back to a body
/// that checks clean, measures exactly, and tessellates.
///
/// The seam is the attack: both seam fins ride one edge on one face, and the
/// refit tells them apart only by the order the face's loops present them.
/// If that convention ever diverges from the one `attach_body_pcurves`
/// derives under, one branch lands on the wrong side of the parameter square
/// and the wall's trim collapses — lockstep alone cannot catch it, because
/// `surface.point(u=0) == surface.point(u=2π)` on a periodic surface, so the
/// measurement and tessellation assertions are the ones doing the work.
#[test]
fn corrupted_pcurves_are_refit_across_seam_uses() {
    // Which fins to corrupt: every fin, only the first seam use, only the
    // second, or every non-seam fin — each stresses the use-order bookkeeping
    // differently.
    #[derive(Debug, Clone, Copy)]
    enum Target {
        All,
        FirstSeamUse,
        SecondSeamUse,
        NonSeam,
    }
    let mut rng = Rng::new(0x_5EA3_51DE);
    for (case, target) in [
        Target::All,
        Target::FirstSeamUse,
        Target::SecondSeamUse,
        Target::NonSeam,
    ]
    .into_iter()
    .enumerate()
    {
        let (r, h) = (rng.range(1.0, 8.0), rng.range(2.0, 16.0));
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::cylinder(&mut store, &mut geo, r, h).expect("valid dimensions");
        attach_body_pcurves(&mut store, &mut geo, body);
        assert!(
            store.check_with_geometry(&geo, body).is_empty(),
            "case {case}: the pristine cylinder must be clean"
        );

        // The seam edge: the one whose two fins sit on the same face. Its
        // uses in face → loop → fin order are what the refit's use-order
        // convention keys on.
        let seam_edge = body_edges(&store, body)
            .into_iter()
            .find(|&e| {
                let fins = store.fins_of_edge(e);
                fins.len() == 2 && store.fin_face(fins[0]) == store.fin_face(fins[1])
            })
            .expect("a cylinder has a seam edge");
        let mut seam_uses = Vec::new();
        for fin in body_fins(&store, body) {
            if store.fin_edge(fin) == seam_edge {
                seam_uses.push(fin);
            }
        }
        assert_eq!(seam_uses.len(), 2, "the seam is used twice by the wall");

        let victims: Vec<EntityId<Fin>> = body_fins(&store, body)
            .into_iter()
            .filter(|&fin| store.fin(fin).is_some_and(|f| f.pcurve.is_some()))
            .filter(|&fin| match target {
                Target::All => true,
                Target::FirstSeamUse => fin == seam_uses[0],
                Target::SecondSeamUse => fin == seam_uses[1],
                Target::NonSeam => store.fin_edge(fin) != seam_edge,
            })
            .collect();
        assert!(
            !victims.is_empty(),
            "case {case} ({target:?}): the target selection found fins to corrupt"
        );
        for &fin in &victims {
            // A rogue line nowhere near the true trim: departure is O(r),
            // decades past any edge tolerance.
            let rogue = geo.add_pcurve(Curve2::Line {
                origin: Point2::new(rng.range(-2.0, 2.0), rng.range(-2.0, 2.0)),
                dir: Vector2::new(rng.range(0.1, 1.0), rng.range(0.1, 1.0)),
            });
            let stale = store
                .fins
                .get_mut(fin)
                .expect("live fin")
                .pcurve
                .replace(rogue)
                .expect("victims carry pcurves");
            geo.pcurves.remove(stale);
        }
        let repro = format!(
            "cylinder(r = {r:.4}, h = {h:.4}), {target:?}: {} pcurve(s) replaced with \
             rogue lines",
            victims.len()
        );

        let result =
            GeometryHealer::fix_pcurves(body, &mut store, &mut geo, &HealOptions::default());
        for &fin in &victims {
            assert!(
                result.operations.iter().any(|op| matches!(
                    op,
                    HealOperation::PcurveRecomputed { fin: f, .. } if *f == fin
                )),
                "{repro}: fin {fin:?}'s rogue pcurve was not refit; operations: {:?}, \
                 notes: {:?}",
                result.operations,
                result.notes
            );
        }
        let failures = store.check_geometry(&geo, body);
        assert!(
            failures.is_empty(),
            "{repro}: repaired body failed check_geometry(): {failures:#?}"
        );
        tessellate_body(&store, &geo, body, &TessellationOptions::default())
            .unwrap_or_else(|e| panic!("{repro}: repaired body failed to tessellate: {e:?}"));
        let measured = brep_mass_properties(&store, &geo, body)
            .unwrap_or_else(|e| panic!("{repro}: measurement failed: {e:?}"))
            .volume;
        assert_within(measured, PI * r * r * h, 1e-9, &repro);
    }
}

// =====================================================================
// (6) Interaction: phase-1 and phase-2 defects in ONE file
// =====================================================================

/// One file carrying an unsewn shell, flipped face senses, *and* a sub-cap
/// bulged arc — the phase-1 heals must run without disturbing the phase-2
/// elevate band, and vice versa. Every assertion from the single-defect
/// campaigns holds at once: exact import, geometric check, honest tolerance,
/// positive volume within the merge budget, and a tessellatable result.
///
/// Whenever the random flip set reaches a majority of the six faces (case 4
/// of this seed does), the heal used to come back inside-out (of-8jqc) — see
/// [`unsew_plus_majority_sense_flips_must_not_heal_to_an_inside_out_body`]
/// for the minimal deterministic pin and the mechanism.
#[test]
fn combined_phase1_and_phase2_damage_heals_in_one_import() {
    let mut rng = Rng::new(0x_C0B1_9ED6);
    for case in 0..10 {
        let export = export_random_block(&mut rng);
        let deviation = rng.range(0.002, 0.008);
        let (bulged, bulge) = bulge_edge_into_arc(&export.text, &mut rng, deviation);
        let amplitude = export.diagonal * TRIM_TOL_REL * rng.range(0.05, 0.25);
        let (unsewn, largest) = unsew(&bulged, &mut rng, amplitude);
        let (damaged, flipped) = flip_face_senses(&unsewn, &mut rng);
        let repro = format!(
            "case {case}: {} with edge #{} bulged to {:.3e} mm (sub-cap), unsewn with \
             jitter {largest:.3e}, {flipped} face sense(s) flipped",
            export.label, bulge.edge_id, bulge.deviation
        );

        let volume = exact_import_inspected(
            &damaged,
            &read_options(HealStrategy::Auto, None),
            &repro,
            |store, _geo, body| {
                let max_tol = body_edges(store, body)
                    .into_iter()
                    .filter_map(|e| store.edge(e).map(|edge| edge.tolerance))
                    .fold(0.0f64, f64::max);
                assert!(
                    max_tol >= deviation * 0.8,
                    "{repro}: the bulge's deviation ({deviation:.3e}) was swallowed \
                     silently (max edge tolerance {max_tol:.3e})"
                );
            },
        )
        .unwrap_or_else(|| panic!("{repro}: combined damage was not healed to an exact import"));
        assert!(volume > 0.0, "{repro}: healed volume must be positive");
        // Two budgets compound: the vertex-merge displacement (phase 1) and
        // the tolerance-covered gap along the bulged edge (phase 2).
        let budget = 6.0 * largest / export.diagonal
            + subcap_volume_budget(&bulge, export.diagonal, export.volume);
        assert_within(volume, export.volume, budget, &repro);
    }
}

/// The API-level twin: one body carrying a past-cap displaced edge curve
/// *and* rogue pcurves on other fins, pushed through the one-shot
/// `GeometryHealer::heal` entry point. The full pass order (collapse → sew →
/// consistency rescue → pcurve refit → orientation) must land every repair in
/// a single call: the displaced line is recomputed from its planes'
/// intersection, the rogue trims are refit, and the body comes back clean,
/// exact, and tessellatable.
///
/// This is also the only test that can exercise `heal()`'s own pcurve pass
/// with pcurves actually present: on the import path fins have no trims yet
/// when the healer runs (they are attached afterwards), so the pass is
/// import-dead and only an API caller ever feeds it work.
#[test]
fn combined_api_damage_heals_in_one_pass() {
    let mut rng = Rng::new(0x_A91D_0007);
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = primitives::block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("unit block");
    attach_body_pcurves(&mut store, &mut geo, body);
    assert!(
        store.check_with_geometry(&geo, body).is_empty(),
        "the pristine block must be clean"
    );

    // Displace one edge's curve past the cap: a parallel line 0.02 mm off
    // both adjacent planes — only the rescue recompute can save it. Any
    // x-aligned edge serves; its adjacent faces are the y and z planes.
    let (target, start_point, span) = body_edges(&store, body)
        .into_iter()
        .find_map(|e| {
            let edge = store.edge(e).expect("live edge");
            let s = store.vertex(edge.start_vertex).expect("live vertex").point;
            let t = store.vertex(edge.end_vertex).expect("live vertex").point;
            let d = t - s;
            (d.y.abs() < 1e-9 && d.z.abs() < 1e-9 && d.x.abs() > 1e-9).then(|| (e, s, d.x))
        })
        .expect("the block has an x-aligned edge");
    let offset = 0.02;
    let bad = geo.add_curve(
        Curve3::line(
            start_point + Vector3::new(0.0, offset, offset),
            Vector3::new(span.signum(), 0.0, 0.0),
        )
        .expect("unit direction"),
    );
    {
        let e = store.edges.get_mut(target).expect("live edge");
        e.curve = Some(bad);
        e.t_start = 0.0;
        e.t_end = span.abs();
    }

    // And three rogue pcurves on fins of OTHER edges.
    let victims: Vec<EntityId<Fin>> = body_fins(&store, body)
        .into_iter()
        .filter(|&fin| store.fin_edge(fin) != target)
        .filter(|&fin| store.fin(fin).is_some_and(|f| f.pcurve.is_some()))
        .take(3)
        .collect();
    assert_eq!(victims.len(), 3, "three fins to corrupt");
    for &fin in &victims {
        let rogue = geo.add_pcurve(Curve2::Line {
            origin: Point2::new(rng.range(-2.0, 2.0), rng.range(-2.0, 2.0)),
            dir: Vector2::new(rng.range(0.1, 1.0), rng.range(0.1, 1.0)),
        });
        let stale = store
            .fins
            .get_mut(fin)
            .expect("live fin")
            .pcurve
            .replace(rogue)
            .expect("victims carry pcurves");
        geo.pcurves.remove(stale);
    }
    let repro = format!(
        "unit block, edge {target:?} displaced {offset} mm off both planes, 3 rogue \
         pcurves on other fins"
    );

    let result = GeometryHealer::heal(body, &mut store, &mut geo, &HealOptions::default());
    assert!(
        result.operations.iter().any(|op| matches!(
            op,
            HealOperation::EdgeCurveRecomputed { edge, .. } if *edge == target
        )),
        "{repro}: the displaced curve must be recomputed; operations: {:?}, notes: {:?}",
        result.operations,
        result.notes
    );
    for &fin in &victims {
        assert!(
            result.operations.iter().any(|op| matches!(
                op,
                HealOperation::PcurveRecomputed { fin: f, .. } if *f == fin
            )),
            "{repro}: fin {fin:?}'s rogue pcurve was not refit in the same heal; \
             operations: {:?}, notes: {:?}",
            result.operations,
            result.notes
        );
    }
    let failures = store.check_with_geometry(&geo, body);
    assert!(
        failures.is_empty(),
        "{repro}: healed body failed check_with_geometry(): {failures:#?}"
    );
    tessellate_body(&store, &geo, body, &TessellationOptions::default())
        .unwrap_or_else(|e| panic!("{repro}: healed body failed to tessellate: {e:?}"));
    let measured = brep_mass_properties(&store, &geo, body)
        .unwrap_or_else(|e| panic!("{repro}: measurement failed: {e:?}"))
        .volume;
    // The recomputed intersection is the exact original line, so the volume
    // is exact — no merge happened, no budget is owed.
    assert_within(measured, 1.0, 1e-9, &repro);
}

/// The minimal pin for of-8jqc: an **unsewn** file with a **majority** of
/// its `ADVANCED_FACE` sense flags flipped must not heal to an inside-out
/// body — which it used to do, certified clean.
///
/// Neither defect alone misbehaved: flips on a sewn file never reach the
/// healer (the topology check passes) and `reconcile_face_senses` repairs
/// the flags against the authoritative loop winding; an unsewn file with
/// honest flags sews and measures correctly. Together, the unsewn topology
/// brought `heal()` in, and its shell-reversal pass measured the enclosed
/// volume *through the lying flags* (`tessellate_face` reads them), saw a
/// negative flag-weighted sum, and reversed the whole shell — inverting the
/// loop windings that were never wrong. `reconcile_face_senses` then
/// corrected every flag to match the inside-out windings, and the result
/// passed `check()` and `check_with_geometry()` while enclosing negative
/// volume; `brep_mass_properties` was the first thing that refused. The fix
/// re-measures the volume sign in `finish_exact_body`, after the flags are
/// reconciled against the windings (`fix_shell_volume_signs`), where the
/// wrong mid-heal reversal is caught and undone.
///
/// Deterministic: fixed block, fixed jitter, the first four (of six) face
/// flags flipped.
#[test]
fn unsew_plus_majority_sense_flips_must_not_heal_to_an_inside_out_body() {
    let mut rng = Rng::new(0x_1D51_DE00);
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = primitives::block(&mut store, &mut geo, 4.0, 5.0, 6.0).expect("valid extents");
    let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
        .expect("a primitive block exports");
    let diagonal = (16.0f64 + 25.0 + 36.0).sqrt();
    let (unsewn_text, largest) = unsew(&text, &mut rng, diagonal * TRIM_TOL_REL * 0.2);

    // Flip the FIRST FOUR face flags — a majority of six, which is what
    // drives the flag-weighted signed volume negative.
    let mut flipped = 0;
    let damaged = unsewn_text
        .lines()
        .map(|line| {
            if !line.contains("= ADVANCED_FACE(") || flipped >= 4 {
                return line.to_string();
            }
            flipped += 1;
            if let Some(head) = line.strip_suffix(".T.);") {
                format!("{head}.F.);")
            } else if let Some(head) = line.strip_suffix(".F.);") {
                format!("{head}.T.);")
            } else {
                line.to_string()
            }
        })
        .collect::<Vec<_>>()
        .join("\n");
    assert_eq!(flipped, 4, "the block writes six ADVANCED_FACE records");
    let repro = "block(4, 5, 6) unsewn (jitter under trim tol) with 4 of 6 face senses flipped";

    let volume = exact_import_inspected(
        &damaged,
        &read_options(HealStrategy::Auto, None),
        repro,
        |_, _, _| {},
    )
    .unwrap_or_else(|| panic!("{repro}: combined damage was not healed to an exact import"));
    assert!(
        volume > 0.0,
        "{repro}: the healed body encloses NEGATIVE volume {volume} — the shell \
         reversal inverted windings that were never wrong, and every validator \
         certified the result"
    );
    let budget = 6.0 * largest / diagonal;
    assert_within(volume, 120.0, budget.max(1e-9), repro);
}
