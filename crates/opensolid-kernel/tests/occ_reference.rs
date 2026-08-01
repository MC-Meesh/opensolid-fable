//! OpenCASCADE as a per-file geometry oracle (spec/11-testing.md §7, of-ipt.16).
//!
//! `tests/step_corpus.rs` asks whether a corpus file imports *at all* and
//! whether our own write → read loop is stable. Neither question can catch a
//! reader that imports a plausible-looking body with the wrong geometry: the
//! round trip is self-consistent by construction, and structured outcomes only
//! prove nothing panicked. This suite asks the other question — is what we
//! imported the same solid an independent kernel sees in the same bytes?
//!
//! The ground truth is OpenCASCADE, recorded per corpus file by
//! `scripts/occ_reference.py` and checked into
//! `tests/data/step/reference/<same path>.json`: per-solid volume, area,
//! centroid, and face/edge/vertex counts, plus one
//! `reference/booleans.json` holding `BRepAlgoAPI` results for a fixed
//! operand set. Because the references are checked in, this suite is
//! hermetic and runs in the default `cargo test` — that is the per-PR smoke
//! half of the oracle. The weekly `external-step-validation` job runs the
//! generator with `--check` to prove the checked-in numbers still match live
//! OCC.
//!
//! What is compared, and why each gate is where it is:
//!
//! - **Solid count** — exact. OCC's solid list is one entry per *placed
//!   occurrence*, which is our `instances` list, not our `solids` list
//!   (as1-oc-214 is 5 parts placed 18 times). Both must agree.
//! - **Face count** — exact, on every file. Two kernels reading the same
//!   `ADVANCED_FACE` set have no license to disagree, and today none do.
//! - **Edge / vertex count** — exact, against OCC's count minus a per-file
//!   delta recorded in [`SEAM_DELTAS`]. OCC rebuilds closed faces with
//!   explicit seam and degenerate edges that the file never spelled; we keep
//!   the file's own topology. Every delta below was verified to be exactly
//!   that: our count equals the file's `EDGE_CURVE`/`VERTEX_POINT` count —
//!   with one file, `hole_tangent_to_wall`, one edge *over* it, where the
//!   reader splits a seam OCC leaves double-booked (see [`SEAM_DELTAS`]).
//! - **Volume, centroid, area** — 0.1% relative, the tolerance spec §7.4
//!   sets for STEP import, on the [`MEASURED_FILES`] the tessellator can
//!   close (18 of 34 today; the rest hit the CDT deferral). Our figures come
//!   from the tessellator, so the residual is chord error; at
//!   [`ANGULAR_STEP`] it lands around 0.045% on the worst curved part.
//! - **Booleans** — our `unite`/`subtract`/`intersect` against
//!   `BRepAlgoAPI_Fuse`/`Cut`/`Common` on the same operands: exact face
//!   count, 0.1% volume and area.
//!
//! Files that do not import, and results the kernel cannot produce yet, are
//! listed explicitly with their bug IDs rather than being tolerated: the
//! lists are gated as *equalities*, so both a regression and a fix fail the
//! test until the list is updated.
//!
//! Cost: 12 s in release, ~10 min in a debug build (measured under load).
//! Essentially all of it is the measurement tessellation — the same 34 files
//! import 57 times in `step_corpus.rs` in 8 s — so trimming it means either
//! profiling `tessellate_body` or measuring fewer bodies, never loosening
//! [`TOLERANCE`] past what spec §7.4 sets. Tracked in of-ywf4.

#[path = "support/json.rs"]
mod json;

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use json::Json;
use opensolid_kernel::brep::boolean::{intersect, subtract, unite};
use opensolid_kernel::brep::{
    Body, GeometryStore, TessellationOptions, TopologyStore, primitives, tessellate_body,
    translate_body,
};
use opensolid_kernel::core::EntityId;
use opensolid_kernel::core::tolerance::ToleranceContext;
use opensolid_kernel::core::types::{Point3, Vector3};
use opensolid_kernel::io::step::read::{SolidOutcome, StepReadOptions, read_step_bytes};
use opensolid_kernel::{brep_mass_properties, mass_properties};

/// Relative tolerance for every measured quantity (spec/11 §7.4).
const TOLERANCE: f64 = 1.0e-3;

/// Angular sampling for the measurement tessellation: 192 segments around a
/// full circle.
///
/// The inscribed-polygon error falls as 1/n² and the grid cost rises as n²,
/// so this is the balance point: measured against OCC, the worst curved
/// corpus part (a full sphere, whose single face costs n² quads) lands near
/// 0.045% — half of [`TOLERANCE`], with the whole suite still finishing in
/// seconds on a release build and a couple of minutes in debug-profile CI.
const ANGULAR_STEP: f64 = std::f64::consts::TAU / 192.0;

/// Corpus files with at least one solid the reader cannot import today.
/// Every entry names the bug that explains it; the test asserts this set
/// *equals* the observed one.
const KNOWN_IMPORT_FAILURES: &[(&str, &str)] = &[
    // of-5cn5: one CIRCLE edge (#4444) whose end VERTEX_POINT sits 7.2e-4 in
    // off the circle's *plane* — 0.0182 mm, nearly twice
    // MAX_ALLOWED_TOLERANCE, so no trim tolerance the reader could derive
    // lets the kernel carry it. of-kwn cleared the five edges underneath it,
    // whose ~1e-5 in misses are inside the 3.669e-3 in closure this file
    // declares; this one is a different kind of defect. Like the file below,
    // what turns the refusal into a lost solid is of-05ac.
    ("nist/nist_ctc_05_asme1_rd.stp", "of-5cn5"),
    // of-05ac: this lost the exact path when of-bb6 started measuring imported
    // edge tolerances — it carries an edge further from its face's surface
    // than MAX_ALLOWED_TOLERANCE, so there is no tolerance the kernel could
    // give it honestly — and the mesh fallback does not close for it either,
    // which is what turns a degrade into a loss. The refusal is right (the
    // part has authored gaps to 0.0386 mm); the fallback failing is not, and
    // of-05ac takes the file off this list and back into the step_corpus
    // floors.
    //
    // bspline_patch_prism was listed here alongside it for a different reason
    // — its extrusion walls were patches one `VECTOR` long and did not contain
    // their own faces. That was of-8ulj's bug, not a fallback gap, and
    // sizing the patch to its face restores the *exact* path, so the file
    // leaves this list with of-8ulj rather than with of-05ac. It is measured
    // in the step_corpus floors.
    ("nist/nist_ctc_02_asme1_rc.stp", "of-05ac"),
];

/// Per-file (edges, vertices) that OCC reports *in addition* to ours.
///
/// OCC's B-Rep model requires every closed face to carry a seam edge, and
/// every cone apex or sphere pole a degenerate one. STEP does not: a full
/// cylindrical face bounded by two circles spells no seam at all, and our
/// reader keeps exactly the topology the file declares. Each delta here was
/// checked against the file's own `EDGE_CURVE`/`VERTEX_POINT` entity count —
/// e.g. `nist_ftc_11` declares 6 edges and 6 vertices for its 6 faces, and
/// that is what we import, while OCC seams its two cylindrical faces up to
/// 12 edges and 8 vertices.
///
/// A delta is normally positive — OCC adds seams, we do not — with one
/// documented exception in the other direction, `hole_tangent_to_wall`, where
/// we split an edge OCC keeps whole. Anything not listed must match exactly.
const SEAM_DELTAS: &[(&str, isize, isize)] = &[
    ("nist/nist_ctc_01_asme1_ap242-e1.stp", 4, 0),
    ("nist/nist_ctc_01_asme1_rd.stp", 20, 6),
    // nist_ctc_02's delta was (148, 0). It is a `KNOWN_IMPORT_FAILURES`
    // entry now (of-05ac) and so counts nothing to compare; the entry comes
    // back with the file.
    ("nist/nist_ctc_03_asme1_rc.stp", 20, 5),
    ("nist/nist_ctc_04_asme1_rd.stp", 66, 0),
    ("nist/nist_ftc_06_asme1_rd.stp", 71, 23),
    ("nist/nist_ftc_07_asme1_rd.stp", 47, 12),
    ("nist/nist_ftc_08_asme1_rc.stp", 20, 0),
    ("nist/nist_ftc_10_asme1_rb.stp", 64, 14),
    ("nist/nist_ftc_11_asme1_rb.stp", 6, 2),
    ("nist/nist_stc_06_asme1_ap242-e3.stp", 60, 13),
    ("occ/blend/filleted_box.stp", 8, 0),
    ("occ/periodic/cone_apex.stp", 1, 0),
    ("occ/periodic/sphere_full.stp", 3, 1),
    ("occ/tangent/sphere_in_matching_pocket.stp", 1, 0),
    // The one negative delta (of-zdx). The tangency splits the block's wall
    // into two coplanar faces, and OCC writes the hole's cylindrical seam on
    // the very same `EDGE_CURVE` as that split, giving one edge four fins.
    // The reader gives the seam its own edge so both the seam and the shared
    // wall boundary are two-sided; OCC keeps the single edge and reports 17
    // where we have 18.
    ("occ/tangent/hole_tangent_to_wall.stp", -1, 0),
];

/// Measured files whose surface area does not agree with OCC, with the bug.
///
/// Empty today, and this time because nothing is wrong. It was not:
/// `nist_ctc_03_asme1_ap242-e2.stp` tessellated closed with the right volume
/// and 61% too much area, then stopped closing at all once the welding and
/// pole work landed — both faces of the same hole-bridging defect, fixed in
/// of-kll8. The mechanism stays here for the next one.
const AREA_EXCEPTIONS: &[(&str, &str)] = &[];

/// Boolean cases the kernel cannot produce yet, with the reason it reports.
const KNOWN_BOOLEAN_GAPS: &[(&str, &str)] = &[(
    "cylinders-cross-union",
    "not implemented: cylinder-cylinder intersection with unequal radii",
)];

/// Corpus files whose every solid tessellates into a closed manifold, so the
/// volume/centroid/area gates can actually run on them. Each is asserted to
/// *stay* measurable: a body that quietly stopped closing would otherwise
/// drop out of the oracle without failing anything.
///
/// The rest of the corpus is not measured, and measurement is not even
/// attempted — the CDT-deferred faces documented in `step_corpus.rs` leave
/// those bodies unclosed, and tessellating a 663-face part to prove it a
/// second time costs more than the debug-profile CI budget allows. When a fix
/// makes another file measurable, add it here; the geometry gates then cover
/// it with no other change. [`measurability_scan`] prints the current list.
const MEASURED_FILES: &[&str] = &[
    "as1-oc-214.stp",
    // The four of-6fcu brought in, by routing trimmed sphere/torus faces
    // and closed-patch NURBS faces through the CDT pass: two fillet-bearing
    // parts (io1-cm's torus quarter-rounds, filleted_box's eight spherical
    // corners), one spline part whose seam edges used to collapse its uv
    // ring, and the tangent sphere pocket.
    //
    // `dm1-id-214.stp` became measurable in the same pass and is
    // deliberately **not** here: its three prototypes come out 0.02%,
    // 0.59%, and 0.29% above OCC's volumes, and the middle two are past
    // this suite's 0.1%. That is not a meshing error — the smallest solid's
    // *exact* B-Rep volume (`brep_mass_properties`, no tessellation
    // involved) is 0.37% high too, and the mesh's surface area is 0.12%
    // *under* OCC's, which an over-covering mesh could not be. It is a
    // reader-side geometry difference on that file, tracked by of-z6zg.
    "io1-cm-214.stp",
    "nist/nist_ctc_03_asme1_ap242-e2.stp",
    "nist/nist_ctc_03_asme1_rc.stp",
    "occ/blend/chamfered_box.stp",
    "occ/blend/filleted_box.stp",
    "occ/coincident/blocks_partial_overlap.stp",
    "occ/coincident/blocks_shared_face.stp",
    "occ/nurbs/lofted_vase.stp",
    // of-8ulj sized this one's four `SURFACE_OF_LINEAR_EXTRUSION` walls to
    // the faces they carry instead of to their `VECTOR`, which is what
    // makes it measurable: the patches used to stop 20 mm short of their
    // own faces, and the tessellator called that degenerate.
    "occ/nurbs/bspline_patch_prism.stp",
    "occ/periodic/cone_apex.stp",
    "occ/periodic/cone_truncated.stp",
    "occ/periodic/cylinder_full.stp",
    "occ/periodic/sphere_full.stp",
    "occ/periodic/torus_full.stp",
    "occ/tangent/sphere_in_matching_pocket.stp",
    "occ/thin/plate_10um.stp",
    "occ/thin/rib_50um.stp",
    "sg1-c5-214.stp",
];

/// Corpus size floor, mirroring `step_corpus.rs`.
const CORPUS_FLOOR: usize = 34;

// ---------------------------------------------------------------------
// Corpus + reference loading
// ---------------------------------------------------------------------

fn corpus_root() -> PathBuf {
    PathBuf::from(format!("{}/tests/data/step", env!("CARGO_MANIFEST_DIR")))
}

/// Every `.stp`/`.step` under the corpus, excluding the reference tree.
fn corpus_files() -> Vec<PathBuf> {
    let mut files = Vec::new();
    let mut stack = vec![corpus_root()];
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(&dir).unwrap_or_else(|e| panic!("read {dir:?}: {e}")) {
            let path = entry.expect("dir entry").path();
            if path.is_dir() {
                if path.file_name().is_some_and(|n| n != "reference") {
                    stack.push(path);
                }
            } else if path
                .extension()
                .and_then(|e| e.to_str())
                .is_some_and(|e| e.eq_ignore_ascii_case("stp") || e.eq_ignore_ascii_case("step"))
            {
                files.push(path);
            }
        }
    }
    files.sort();
    assert!(
        files.len() >= CORPUS_FLOOR,
        "corpus shrank? found only {} files",
        files.len()
    );
    files
}

/// Corpus-relative path, the key every table above is written in.
fn key(file: &Path) -> String {
    file.strip_prefix(corpus_root())
        .expect("corpus file")
        .to_string_lossy()
        .replace('\\', "/")
}

fn reference_path(file: &Path) -> PathBuf {
    corpus_root()
        .join("reference")
        .join(key(file))
        .with_extension("json")
}

fn load_reference(file: &Path) -> Json {
    let path = reference_path(file);
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: no OCC reference at {} ({e}) — run `python3 scripts/occ_reference.py corpus`",
            key(file),
            path.display()
        )
    });
    let value = Json::parse(&text);
    assert_eq!(
        value.field("schema").as_str(),
        "opensolid-occ-reference/1",
        "{}: unexpected reference schema",
        key(file)
    );
    value
}

/// FNV-1a, 64-bit — the same hash `scripts/occ_reference.py` records, so a
/// corpus file edited without regenerating its reference is caught rather
/// than silently compared against stale numbers.
fn fnv1a64(bytes: &[u8]) -> u64 {
    let mut hash: u64 = 0xcbf2_9ce4_8422_2325;
    for &byte in bytes {
        hash ^= byte as u64;
        hash = hash.wrapping_mul(0x0000_0100_0000_01b3);
    }
    hash
}

fn lookup<'a, T: Copy>(table: &'a [(&'a str, T)], name: &str) -> Option<T> {
    table.iter().find(|(k, _)| *k == name).map(|(_, v)| *v)
}

// ---------------------------------------------------------------------
// Our side of the comparison
// ---------------------------------------------------------------------

/// What we imported from one corpus file, expressed the way OCC expresses
/// it: one entry per placed occurrence.
struct OurImport {
    /// Placed occurrences, in file order.
    solids: Vec<OurSolid>,
    /// Some solid failed to import at all.
    has_failure: bool,
}

/// One solid, either as the reader imported it (a prototype) or as the
/// product structure placed it (an occurrence — same counts, moved centroid).
#[derive(Clone, Copy)]
struct OurSolid {
    faces: usize,
    edges: usize,
    vertices: usize,
    /// `None` when the tessellator cannot close this body (the CDT-deferred
    /// cases) or when measurement was not requested, so nothing measurable is
    /// claimed for it.
    measured: Option<Measured>,
}

#[derive(Clone, Copy)]
struct Measured {
    volume: f64,
    area: f64,
    centroid: Point3,
}

impl OurImport {
    fn totals(&self) -> (usize, usize, usize) {
        self.solids.iter().fold((0, 0, 0), |(f, e, v), s| {
            (f + s.faces, e + s.edges, v + s.vertices)
        })
    }

    /// Measurements for every occurrence, or `None` if any is unmeasurable.
    fn all_measured(&self) -> Option<Vec<Measured>> {
        self.solids.iter().map(|s| s.measured).collect()
    }
}

/// One import + measurement pass over the whole corpus, shared by every test
/// in this binary.
///
/// Importing 5 MB of Part 21 and tessellating what can be tessellated costs
/// real seconds; doing it once per assertion would multiply that by the test
/// count for no extra coverage. Test threads race to initialize and the
/// losers block, so the pass runs exactly once per `cargo test`.
fn corpus() -> &'static [(String, Json, OurImport)] {
    static CORPUS: std::sync::OnceLock<Vec<(String, Json, OurImport)>> = std::sync::OnceLock::new();
    CORPUS.get_or_init(|| {
        corpus_files()
            .into_iter()
            .map(|file| {
                let name = key(&file);
                let measure = MEASURED_FILES.contains(&name.as_str());
                let reference = load_reference(&file);
                (name, reference, import_file(&file, measure))
            })
            .collect()
    })
}

fn import_file(file: &Path, measure: bool) -> OurImport {
    let bytes = std::fs::read(file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step_bytes(&bytes, &mut store, &mut geo, &StepReadOptions::default())
        .unwrap_or_else(|e| panic!("{}: must parse: {e}", key(file)));

    // Measure each *prototype* once; an instance is (part, transform), so
    // volume and area carry over and only the centroid moves.
    let options = TessellationOptions {
        angular_step: ANGULAR_STEP,
    };
    let mut prototypes: Vec<Option<OurSolid>> = Vec::new();
    let mut has_failure = false;
    for solid in &report.solids {
        match &solid.outcome {
            SolidOutcome::BRep(body) => {
                let counts = store.euler_counts(*body);
                prototypes.push(Some(OurSolid {
                    faces: counts.faces,
                    edges: counts.edges,
                    vertices: counts.vertices,
                    measured: measure
                        .then(|| measure_body(&store, &geo, *body, &options))
                        .flatten(),
                }));
            }
            SolidOutcome::Mesh { mesh, .. } => {
                // A fallback mesh carries no B-Rep topology to count, but it
                // is still measurable.
                prototypes.push(Some(OurSolid {
                    faces: 0,
                    edges: 0,
                    vertices: 0,
                    measured: measure
                        .then(|| {
                            mass_properties(mesh).ok().map(|mp| Measured {
                                volume: mp.volume,
                                area: mp.surface_area,
                                centroid: mp.centroid,
                            })
                        })
                        .flatten(),
                }));
            }
            SolidOutcome::Failed => {
                has_failure = true;
                prototypes.push(None);
            }
        }
    }

    let mut solids = Vec::new();
    for instance in &report.instances {
        let Some(prototype) = prototypes[instance.solid] else {
            continue;
        };
        solids.push(OurSolid {
            measured: prototype.measured.map(|m| Measured {
                centroid: instance.transform * m.centroid,
                ..m
            }),
            ..prototype
        });
    }
    OurImport {
        solids,
        has_failure,
    }
}

fn measure_body(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    options: &TessellationOptions,
) -> Option<Measured> {
    let mesh = tessellate_body(store, geo, body, options).ok()?;
    let mp = mass_properties(&mesh).ok()?;
    Some(Measured {
        volume: mp.volume,
        area: mp.surface_area,
        centroid: mp.centroid,
    })
}

/// Volume-weighted centroid of a set of measurements — the quantity that is
/// comparable across kernels for a multi-solid file, and the one that only
/// agrees if every instance transform is right.
fn combined_centroid(measured: &[Measured]) -> (f64, Point3) {
    let total: f64 = measured.iter().map(|m| m.volume).sum();
    let mut sum = Vector3::zeros();
    for m in measured {
        sum += m.centroid.coords * m.volume;
    }
    (total, Point3::from(sum / total))
}

fn relative(ours: f64, theirs: f64) -> f64 {
    (ours - theirs).abs() / theirs.abs().max(1.0)
}

// ---------------------------------------------------------------------
// 1. The references themselves
// ---------------------------------------------------------------------

/// Every corpus file has a reference, and that reference was generated from
/// the bytes on disk right now. Without this the whole suite could pass
/// against numbers taken from a file that has since been replaced.
#[test]
fn every_corpus_file_has_a_reference_for_its_current_bytes() {
    for file in corpus_files() {
        let name = key(&file);
        let reference = load_reference(&file);
        let source = reference.field("source");
        let bytes = std::fs::read(&file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
        assert_eq!(
            source.field("bytes").as_usize(),
            bytes.len(),
            "{name}: reference is stale (byte length changed) — regenerate with \
             `python3 scripts/occ_reference.py corpus`"
        );
        assert_eq!(
            source.field("fnv1a64").as_str(),
            format!("{:016x}", fnv1a64(&bytes)),
            "{name}: reference is stale (content changed) — regenerate with \
             `python3 scripts/occ_reference.py corpus`"
        );
        assert!(
            reference.field("totals").field("solids").as_usize() > 0,
            "{name}: OCC found no solids — wrong file vendored?"
        );
    }
}

/// No reference may outlive its corpus file: a leftover JSON would quietly
/// stop being checked while still looking like coverage.
#[test]
fn no_reference_is_orphaned() {
    let expected: Vec<PathBuf> = corpus_files().iter().map(|f| reference_path(f)).collect();
    let mut stack = vec![corpus_root().join("reference")];
    let boolean_reference = corpus_root().join("reference").join("booleans.json");
    while let Some(dir) = stack.pop() {
        for entry in std::fs::read_dir(&dir).unwrap_or_else(|e| panic!("read {dir:?}: {e}")) {
            let path = entry.expect("dir entry").path();
            if path.is_dir() {
                stack.push(path);
            } else if path != boolean_reference {
                assert!(
                    expected.contains(&path),
                    "{}: reference without a corpus file",
                    path.display()
                );
            }
        }
    }
}

// ---------------------------------------------------------------------
// 2. Topology: what the two kernels think is in the file
// ---------------------------------------------------------------------

/// The files that fail to import are exactly the documented ones. Gated as
/// an equality so a fix is as loud as a regression.
#[test]
fn only_the_documented_files_fail_to_import() {
    let mut failing: Vec<&str> = corpus()
        .iter()
        .filter(|(_, _, ours)| ours.has_failure)
        .map(|(name, _, _)| name.as_str())
        .collect();
    let mut expected: Vec<&str> = KNOWN_IMPORT_FAILURES.iter().map(|(f, _)| *f).collect();
    expected.sort();
    failing.sort();
    assert_eq!(
        failing, expected,
        "the set of files that fail to import changed — if one now imports, \
         drop it from KNOWN_IMPORT_FAILURES (and raise the floors in \
         step_corpus.rs); if a new one fails, that is a reader regression"
    );
}

/// Solid count and face count must match OCC exactly, on every file that
/// imports. The solid count is compared against our *instances*: OCC lists
/// placed occurrences, so an assembly's 5 parts placed 18 times is 18.
#[test]
fn occ_agrees_on_solid_and_face_counts() {
    for (name, reference, ours) in corpus() {
        if lookup(KNOWN_IMPORT_FAILURES, name).is_some() {
            continue;
        }
        let totals = reference.field("totals");
        assert_eq!(
            ours.solids.len(),
            totals.field("solids").as_usize(),
            "{name}: solid occurrence count differs from OCC"
        );
        let (faces, _, _) = ours.totals();
        assert_eq!(
            faces,
            totals.field("faces").as_usize(),
            "{name}: face count differs from OCC"
        );
    }
}

/// Edge and vertex counts, against OCC minus the recorded seam delta.
#[test]
fn occ_agrees_on_edge_and_vertex_counts_up_to_seams() {
    let mut unused: Vec<&str> = SEAM_DELTAS.iter().map(|(f, _, _)| *f).collect();
    for (name, reference, ours) in corpus() {
        if lookup(KNOWN_IMPORT_FAILURES, name).is_some() {
            continue;
        }
        let totals = reference.field("totals");
        let (_, edges, vertices) = ours.totals();
        let (edge_delta, vertex_delta) = SEAM_DELTAS
            .iter()
            .find(|(f, _, _)| f == name)
            .map(|(_, e, v)| (*e, *v))
            .unwrap_or((0, 0));
        unused.retain(|f| f != name);
        assert_eq!(
            edges as isize + edge_delta,
            totals.field("edges").as_usize() as isize,
            "{name}: edge count differs from OCC by other than the recorded \
             seam delta of {edge_delta}"
        );
        assert_eq!(
            vertices as isize + vertex_delta,
            totals.field("vertices").as_usize() as isize,
            "{name}: vertex count differs from OCC by other than the recorded \
             seam delta of {vertex_delta}"
        );
    }
    assert!(
        unused.is_empty(),
        "SEAM_DELTAS lists files that are no longer in the corpus (or no \
         longer import): {unused:?}"
    );
}

/// Corpus files whose solids are pinched — the material touches itself along
/// a tangency — so a mesh of them is non-manifold by construction and they
/// can never join [`MEASURED_FILES`]. [`tangent_parts_have_occs_volume`]
/// measures them the other way instead.
const PINCHED_FILES: &[&str] = &[
    "occ/tangent/cylinder_tangent_to_wall.stp",
    "occ/tangent/hole_tangent_to_wall.stp",
];

/// The tangent parts' volumes, integrated over the imported B-Rep surfaces
/// rather than a mesh, against OCC (of-zdx).
///
/// They are the only corpus files the volume gate above cannot reach. Their
/// tangency pinches the solid — the hole's wall touches the block's, so the
/// boundary is non-manifold along one line — and a mesh of that is
/// non-manifold too, which is what [`MEASURED_FILES`] requires the absence
/// of. That is a property of the shape, not a meshing defect, so waiting for
/// it to become measurable would mean never checking these two at all. The
/// surface integral has no such requirement, and it is the sharper instrument
/// besides: with no chord error in the way, both parts land within 1e-11
/// relative of OCC rather than spending most of [`TOLERANCE`] on
/// discretization.
#[test]
fn tangent_parts_have_occs_volume() {
    for name in PINCHED_FILES {
        let file = corpus_root().join(name);
        let bytes = std::fs::read(&file).unwrap_or_else(|e| panic!("read {file:?}: {e}"));
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let report = read_step_bytes(&bytes, &mut store, &mut geo, &StepReadOptions::default())
            .unwrap_or_else(|e| panic!("{name}: must parse: {e}"));
        let [solid] = &report.solids[..] else {
            panic!("{name}: expected exactly one solid");
        };
        let SolidOutcome::BRep(body) = solid.outcome else {
            panic!("{name}: no longer imports as an exact B-Rep");
        };
        let volume = brep_mass_properties(&store, &geo, body)
            .unwrap_or_else(|e| panic!("{name}: exact mass properties: {e}"))
            .volume;
        let expected = load_reference(&file)
            .field("totals")
            .field("volume")
            .as_f64();
        assert!(
            (volume - expected).abs() <= TOLERANCE * expected.abs(),
            "{name}: exact B-Rep volume {volume} differs from OCC's {expected} \
             by more than {TOLERANCE}"
        );
    }
}

// ---------------------------------------------------------------------
// 3. Geometry: volume, centroid, area
// ---------------------------------------------------------------------

/// Per-solid volumes and the assembly-wide centroid, against OCC.
///
/// Volumes are matched sorted, largest first — the same order the generator
/// writes — because neither kernel's traversal order is normative. The
/// centroid is volume-weighted over every placed occurrence, so it only
/// agrees if the product structure put each part where OCC put it.
#[test]
fn occ_agrees_on_volume_and_centroid() {
    let mut visited = 0usize;
    for (name, reference, ours) in corpus() {
        if !MEASURED_FILES.contains(&name.as_str()) {
            continue;
        }
        visited += 1;
        let mut mine = ours.all_measured().unwrap_or_else(|| {
            panic!(
                "{name} is listed in MEASURED_FILES but the tessellator no \
                 longer closes every one of its solids"
            )
        });
        assert!(!mine.is_empty(), "{name}: nothing measured");

        let theirs: Vec<&Json> = reference.field("solids").as_array().iter().collect();
        assert_eq!(
            mine.len(),
            theirs.len(),
            "{name}: measured {} solids, OCC recorded {}",
            mine.len(),
            theirs.len()
        );
        mine.sort_by(|a, b| b.volume.total_cmp(&a.volume));
        for (index, (ours, theirs)) in mine.iter().zip(&theirs).enumerate() {
            let expected = theirs.field("volume").as_f64();
            let drift = relative(ours.volume, expected);
            assert!(
                drift <= TOLERANCE,
                "{name}: solid {index} volume {} differs from OCC's {expected} \
                 by {drift:e} (> {TOLERANCE:e})",
                ours.volume
            );
        }

        let (total, centroid) = combined_centroid(&mine);
        let occ_total = reference.field("totals").field("volume").as_f64();
        let drift = relative(total, occ_total);
        assert!(
            drift <= TOLERANCE,
            "{name}: total volume {total} differs from OCC's {occ_total} by {drift:e}"
        );
        let occ_measured: Vec<Measured> = theirs
            .iter()
            .map(|s| {
                let c = s.field("centroid").as_array();
                Measured {
                    volume: s.field("volume").as_f64(),
                    area: s.field("area").as_f64(),
                    centroid: Point3::new(c[0].as_f64(), c[1].as_f64(), c[2].as_f64()),
                }
            })
            .collect();
        let (_, occ_centroid) = combined_centroid(&occ_measured);
        // Scale the centroid tolerance by the part's own size: an absolute
        // millimetre bound would be meaningless across a 10 mm plate and a
        // 500 mm assembly.
        let scale = total.cbrt().max(1.0);
        let offset = (centroid - occ_centroid).norm();
        assert!(
            offset <= TOLERANCE * scale,
            "{name}: centroid {centroid:?} is {offset} mm from OCC's \
             {occ_centroid:?} (limit {} mm)",
            TOLERANCE * scale
        );
    }
    assert_eq!(
        visited,
        MEASURED_FILES.len(),
        "MEASURED_FILES names a file that is not in the corpus"
    );
}

/// Diagnostic: try to measure *every* corpus file and print what happened.
///
/// This is how [`MEASURED_FILES`] is refreshed — tessellating the whole
/// corpus at [`ANGULAR_STEP`] costs minutes, which is why the gates work off
/// a list instead of rediscovering it on every run.
///
/// ```bash
/// cargo test --release --test occ_reference -- --ignored --nocapture
/// ```
#[test]
#[ignore = "diagnostic: minutes of tessellation, prints the MEASURED_FILES table"]
fn measurability_scan() {
    let mut measurable = Vec::new();
    for file in corpus_files() {
        let name = key(&file);
        let ours = import_file(&file, true);
        let state = match ours.all_measured() {
            Some(mine) if !mine.is_empty() => {
                measurable.push(name.clone());
                let volume: f64 = mine.iter().map(|m| m.volume).sum();
                let area: f64 = mine.iter().map(|m| m.area).sum();
                format!("measurable: volume {volume:.4}, area {area:.4}")
            }
            _ if ours.has_failure => "import failed".to_owned(),
            _ => "not a closed manifold".to_owned(),
        };
        println!("{name:48} {state}");
    }
    println!("\nMEASURED_FILES would be:");
    for name in &measurable {
        println!("    {name:?},");
    }
}

/// Surface area, where it is measurable, against OCC.
#[test]
fn occ_agrees_on_surface_area() {
    let mut checked = 0usize;
    let mut unused: Vec<&str> = AREA_EXCEPTIONS.iter().map(|(f, _)| *f).collect();
    for (name, reference, imported) in corpus() {
        if !MEASURED_FILES.contains(&name.as_str()) {
            continue;
        }
        let mine = imported
            .all_measured()
            .unwrap_or_else(|| panic!("{name}: listed in MEASURED_FILES but not measurable"));
        let ours: f64 = mine.iter().map(|m| m.area).sum();
        let theirs = reference.field("totals").field("area").as_f64();
        let drift = relative(ours, theirs);
        if let Some(bug) = lookup(AREA_EXCEPTIONS, name) {
            unused.retain(|f| f != name);
            assert!(
                drift > TOLERANCE,
                "{name} is listed as an area exception ({bug}) but now agrees \
                 with OCC to {drift:e} — drop the entry"
            );
            continue;
        }
        checked += 1;
        assert!(
            drift <= TOLERANCE,
            "{name}: surface area {ours} differs from OCC's {theirs} by \
             {drift:e} (> {TOLERANCE:e})"
        );
    }
    assert!(
        unused.is_empty(),
        "AREA_EXCEPTIONS lists files that are no longer measured: {unused:?}"
    );
    assert_eq!(
        checked,
        MEASURED_FILES.len() - AREA_EXCEPTIONS.len(),
        "every measured file except the documented exceptions must have its \
         area compared"
    );
}

// ---------------------------------------------------------------------
// 4. Booleans against BRepAlgoAPI
// ---------------------------------------------------------------------

fn boolean_reference() -> Json {
    let path = corpus_root().join("reference").join("booleans.json");
    let text = std::fs::read_to_string(&path).unwrap_or_else(|e| {
        panic!(
            "{}: {e} — run `scripts/occ_reference.py booleans`",
            path.display()
        )
    });
    let value = Json::parse(&text);
    assert_eq!(
        value.field("schema").as_str(),
        "opensolid-occ-boolean-reference/1"
    );
    value
}

/// Rebuild one reference operand with our primitives. The generator builds
/// the OCC side from the same numbers under the same convention: centered on
/// the origin, then translated by `at`.
fn build_operand(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    spec: &Json,
) -> EntityId<Body> {
    let body = match spec.field("kind").as_str() {
        "block" => {
            let size = spec.field("size").as_array();
            primitives::block(
                store,
                geo,
                size[0].as_f64(),
                size[1].as_f64(),
                size[2].as_f64(),
            )
            .expect("block")
        }
        "cylinder" => primitives::cylinder(
            store,
            geo,
            spec.field("radius").as_f64(),
            spec.field("height").as_f64(),
        )
        .expect("cylinder"),
        "sphere" => primitives::sphere(store, geo, spec.field("radius").as_f64()).expect("sphere"),
        other => panic!("unknown operand kind {other:?}"),
    };
    let at = spec.field("at").as_array();
    let offset = Vector3::new(at[0].as_f64(), at[1].as_f64(), at[2].as_f64());
    if offset != Vector3::zeros() {
        translate_body(store, geo, body, offset).expect("translate operand");
    }
    body
}

/// Every boolean in the reference set: same operands, same operation,
/// compared against `BRepAlgoAPI`. Face count exact; volume and area to
/// 0.1%.
#[test]
fn booleans_match_brepalgoapi() {
    let reference = boolean_reference();
    let mut gaps = BTreeMap::new();
    let mut compared = 0usize;
    for case in reference.field("cases").as_array() {
        let name = case.field("name").as_str().to_owned();
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let a = build_operand(&mut store, &mut geo, case.field("a"));
        let b = build_operand(&mut store, &mut geo, case.field("b"));
        let tol = ToleranceContext::default();
        let out = match case.field("op").as_str() {
            "unite" => unite(&store, &geo, a, b, &tol),
            "subtract" => subtract(&store, &geo, a, b, &tol),
            "intersect" => intersect(&store, &geo, a, b, &tol),
            other => panic!("{name}: unknown op {other:?}"),
        };
        let out = match out {
            Ok(out) => out,
            Err(e) => {
                gaps.insert(name, e.to_string());
                continue;
            }
        };

        let expected = case.field("result");
        assert!(
            out.store.check(out.body).is_empty(),
            "{name}: boolean output must pass check: {:?}",
            out.store.check(out.body)
        );
        assert_eq!(
            out.store.euler_counts(out.body).faces,
            expected.field("faces").as_usize(),
            "{name}: face count differs from BRepAlgoAPI"
        );

        let mesh = out
            .tessellate()
            .unwrap_or_else(|e| panic!("{name}: result must tessellate: {e:?}"));
        assert!(
            mesh.is_closed_manifold(),
            "{name}: result tessellation must be a closed manifold"
        );
        let mp = mass_properties(&mesh).unwrap_or_else(|e| panic!("{name}: {e}"));
        for (label, ours, theirs) in [
            ("volume", mp.volume, expected.field("volume").as_f64()),
            ("area", mp.surface_area, expected.field("area").as_f64()),
        ] {
            let drift = relative(ours, theirs);
            assert!(
                drift <= TOLERANCE,
                "{name}: {label} {ours} differs from BRepAlgoAPI's {theirs} by \
                 {drift:e} (> {TOLERANCE:e})"
            );
        }

        let centre = expected.field("centroid").as_array();
        let occ_centroid = Point3::new(centre[0].as_f64(), centre[1].as_f64(), centre[2].as_f64());
        let offset = (mp.centroid - occ_centroid).norm();
        let scale = mp.volume.cbrt().max(1.0);
        assert!(
            offset <= TOLERANCE * scale,
            "{name}: centroid {:?} is {offset} mm from BRepAlgoAPI's \
             {occ_centroid:?} (limit {} mm)",
            mp.centroid,
            TOLERANCE * scale
        );
        compared += 1;
    }

    let expected_gaps: BTreeMap<String, String> = KNOWN_BOOLEAN_GAPS
        .iter()
        .map(|(name, reason)| ((*name).to_owned(), (*reason).to_owned()))
        .collect();
    assert_eq!(
        gaps, expected_gaps,
        "the set of boolean cases the kernel cannot produce changed — if one \
         now works, drop it from KNOWN_BOOLEAN_GAPS; if a new one fails, that \
         is a boolean regression"
    );
    assert_eq!(
        compared,
        reference.field("cases").as_array().len() - KNOWN_BOOLEAN_GAPS.len(),
        "every non-gap case must be compared"
    );
}
