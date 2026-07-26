# Vendored STEP test corpus

Real-world STEP Part 21 files exercised by `tests/step_corpus.rs`. Every file
is license-checked and unmodified; provenance below. The corpus feeds the
pass-rate metric the spec tracks (spec/06-step-io.md §Pass-rate targets) —
`cargo run --release --example step_import_report -- crates/opensolid-kernel/tests/data/step`
prints the current figure.

## STEPcode files (BSD-3-Clause)

Vendored from the [STEPcode](https://github.com/stepcode/stepcode) repository
(`data/ap214e3/`, `develop` branch — sg1/io1/dm1 fetched 2026-07-12,
as1 fetched 2026-07-25 at commit `74b6fe45`), which distributes them under the
BSD-3-Clause license. All four were written by CATIA V5R20 (AP214,
`AUTOMOTIVE_DESIGN` schema) and are long-standing CAx-IF / PDES
interoperability test parts.

| File | Size | Contents |
|------|------|----------|
| `sg1-c5-214.stp` | 24 KB | 1 solid — planes, cylinders, one cone (all analytic) |
| `io1-cm-214.stp` | 41 KB | 1 solid — planes, cylinders, one torus (all analytic) |
| `dm1-id-214.stp` | 86 KB | 3 solids — includes B-spline curves/surfaces (exercises the mesh/diagnostic fallback paths) |
| `as1-oc-214.stp` | 442 KB | 5 solids — the classic AS1 assembly (bolt, nut, plate, L-bracket, rod); `SURFACE_CURVE` geometry |

## NIST MBE PMI test cases (`nist/`, public domain)

Vendored from the [NIST MBE PMI Validation and Conformance Testing
Project](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0)
(`NIST-PMI-STEP-Files.zip`, fetched 2026-07-25 from
https://www.nist.gov/document/nist-pmi-step-files). NIST states the test
cases, CAD models, and STEP files "can be used without any restrictions";
as U.S. government works they carry no copyright. Files are unmodified.

Eleven are the "AP203 geometry only" exports of the five Combined Test Case
(CTC) and six Fully-Toleranced Test Case (FTC) part geometries (exported
2015); two are AP242 editions of CTC-01/CTC-03 with PMI included (exported
2021), covering the AP242 schema declaration and PMI entity skeleton; one is a
2024 Semantic Tolerancing Test Case (STC) export under AP242 edition 3.

| File | Size | Schema |
|------|------|--------|
| `nist_ctc_01_asme1_rd.stp` | 230 KB | AP203e2 MIM_LF |
| `nist_ctc_02_asme1_rc.stp` | 1.2 MB | AP203e2 MIM_LF |
| `nist_ctc_03_asme1_rc.stp` | 246 KB | AP203e2 MIM_LF |
| `nist_ctc_04_asme1_rd.stp` | 775 KB | AP203e2 MIM_LF |
| `nist_ctc_05_asme1_rd.stp` | 319 KB | AP203e2 MIM_LF |
| `nist_ftc_06_asme1_rd.stp` | 218 KB | AP203e2 MIM_LF |
| `nist_ftc_07_asme1_rd.stp` | 393 KB | CONFIG_CONTROL_DESIGN |
| `nist_ftc_08_asme1_rc.stp` | 440 KB | AP203e2 MIM_LF |
| `nist_ftc_09_asme1_rd.stp` | 258 KB | AP203e2 MIM_LF |
| `nist_ftc_10_asme1_rb.stp` | 316 KB | CONFIG_CONTROL_DESIGN |
| `nist_ftc_11_asme1_rb.stp` | 8 KB | CONFIG_CONTROL_DESIGN |
| `nist_ctc_01_asme1_ap242-e1.stp` | 387 KB | AP242 MIM_LF |
| `nist_ctc_03_asme1_ap242-e2.stp` | 658 KB | AP242 MIM_LF |
| `nist_stc_06_asme1_ap242-e3.stp` | 983 KB | AP242 MIM_LF (edition 3) |

`nist_stc_06_asme1_ap242-e3.stp` (fetched 2026-07-26 from the same archive) is
the 2024 Semantic Tolerancing Test Case export of the FTC-06 geometry: the
same 144 faces as `nist_ftc_06_asme1_rd.stp` written under the AP242 edition-3
schema, so the two are a schema differential on identical geometry.

## Self-generated edge cases (`occ/`, no license — we made them)

Written by `python3 scripts/occ_reference.py generate`, which builds each
shape with OpenCASCADE and exports it through OCC's STEP writer. Generated
rather than downloaded because the families spec/11-testing.md §8.1 calls for
are precisely the ones nobody publishes: tangency, coincidence, sub-tolerance
features, closed periodic surfaces with no seam spelled out.

The two provenance records OCC stamps with the wall clock (`FILE_NAME`,
`FILE_DESCRIPTION`) are rewritten to fixed values so regeneration is
byte-reproducible — `scripts/occ_reference.py generate --check` proves the
checked-in files still match what the generator produces. Nothing else is
touched, and `generate --check` is the test that says so.

| File | Family | Schema | What it exercises |
|------|--------|--------|-------------------|
| `periodic/cylinder_full.stp` | periodic | AP203 | closed cylindrical face, no seam edge in the file |
| `periodic/sphere_full.stp` | periodic | AP203 | one face, zero `EDGE_CURVE`s, both poles |
| `periodic/torus_full.stp` | periodic | AP214 | genus-1 solid from a single face |
| `periodic/cone_apex.stp` | periodic | AP203 | full cone: apex bounded by a `VERTEX_LOOP` |
| `periodic/cone_truncated.stp` | periodic | AP203 | conical band between two circles |
| `tangent/cylinder_tangent_to_wall.stp` | tangent | AP203 | fused cylinder tangent to a planar wall |
| `tangent/hole_tangent_to_wall.stp` | tangent | AP203 | the same tangency as a subtraction |
| `tangent/sphere_in_matching_pocket.stp` | tangent | AP214 | sphere tangent to a cylinder along a full circle |
| `coincident/blocks_shared_face.stp` | coincident | AP203 | two blocks fused across an entire shared face |
| `coincident/blocks_partial_overlap.stp` | coincident | AP203 | shared region is a sub-rectangle of both faces |
| `thin/plate_10um.stp` | thin | AP203 | 50 × 50 × 0.01 mm — 5000:1 aspect |
| `thin/rib_50um.stp` | thin | AP203 | 0.05 mm rib on a 40 mm block |
| `nurbs/bspline_patch_prism.stp` | freeform | AP203 | degree-5+ B-spline patch swept into a solid |
| `nurbs/lofted_vase.stp` | freeform | AP242 | ThruSections loft: one B-spline wall, 3 faces total |
| `blend/filleted_box.stp` | blend | AP214 | 12 edge fillets + 8 corner patches |
| `blend/chamfered_box.stp` | blend | AP203 | 12 chamfers + 8 corner planes |

## OCC reference data (`reference/`)

Every corpus file has a checked-in JSON under `reference/` mirroring its
path, holding what OpenCASCADE reads out of the same bytes: per-solid volume,
area, centroid, and face/edge/vertex counts. `reference/booleans.json` holds
`BRepAlgoAPI` results for a fixed operand set. `tests/occ_reference.rs`
compares our import and our booleans against them on every PR — see
spec/11-testing.md §7 and the header of that test for what each gate means.

Regenerate after adding or replacing a corpus file (the test fails loudly if
you forget: each reference records the file's length and FNV-1a hash):

```bash
python3 scripts/occ_reference.py corpus     # per-file references
python3 scripts/occ_reference.py booleans   # BRepAlgoAPI differential
python3 scripts/occ_reference.py generate   # the occ/ edge-case files themselves
```

## Schema note

The reader maps `MANIFOLD_SOLID_BREP` entities regardless of the declared
schema, so AP214/AP242 files are valid input for the AP203-oriented importer;
the product/assembly/PMI skeleton entities are simply ignored.

## Growing the corpus

Additions must state source, fetch date, and license here, and third-party
files must be added unmodified. Then regenerate the OCC references — a file
without one fails `every_corpus_file_has_a_reference_for_its_current_bytes`.

Candidate sources with clean licenses: STEPcode `data/` (BSD-3-Clause, though
`data/ap214e3/` holds only the four files already vendored here), the rest of
the NIST MBE PMI set (public domain — the AP242 editions of the other ten
parts and the other four 2024 `stc_*` semantic-tolerancing parts are not
vendored yet, at 1–6 MB each), NIST hole test cases (public domain), and anything
`scripts/occ_reference.py generate` can build (self-generated, license-free).

Still missing against spec/11 §8.1: the ABC Dataset sample (distributed only
as multi-GB chunk archives) and real vendor exports from SolidWorks / NX /
Fusion, which need files whose redistribution terms we can actually check.
