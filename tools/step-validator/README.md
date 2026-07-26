# External STEP validator

Cross-validation of the STEP pipeline against an external CAD kernel
(headless FreeCAD, i.e. OpenCASCADE), in both directions (of-3qy.10):

1. **ours → FreeCAD** — `step_export_samples` writes primitives and boolean
   results with known analytic volumes; `freecad_check.py check` imports each
   into FreeCAD and verifies it is a valid, closed solid whose exactly
   computed volume matches `manifest.json` within 1e-6 relative.
2. **FreeCAD → ours** — `freecad_check.py generate` builds parametric parts
   with FreeCAD's Part workbench (including a filleted box and a revolved
   ring, which exercise blend and revolve exports) and exports them as STEP;
   `step_import_report` imports them with our reader and prints the pass rate
   tracked by spec/06-step-io.md §Pass-rate targets (month-3 target: 80% on
   FreeCAD exports).

## Running

Opt-in by design — the default `cargo test` suite stays hermetic and never
requires external binaries.

```bash
# needs freecadcmd on PATH (macOS: FreeCAD.app bundles it; override with
# FREECAD_CMD=/Applications/FreeCAD.app/Contents/Resources/bin/freecadcmd)
OPENSOLID_STEP_VALIDATOR=1 tools/step-validator/run.sh

# gate the FreeCAD→ours direction once coverage supports it:
OPENSOLID_STEP_VALIDATOR=1 STEP_IMPORT_MIN_RATE=80 tools/step-validator/run.sh
```

In CI this runs as the `external-step-validation` workflow: manually via
workflow_dispatch, or weekly when the repository variable
`OPENSOLID_STEP_VALIDATOR` is `1`. The pass-rate lines land in the job
summary.

## The other external kernel: OCC as a per-file oracle

This validator answers "does the file survive a trip through another
kernel?". It does not answer "is the geometry we imported the same geometry
that kernel sees?" — that is `scripts/occ_reference.py` (of-ipt.16), which
records OpenCASCADE's account of every corpus file (per-solid volume, area,
centroid, face/edge/vertex counts) plus `BRepAlgoAPI` results for a fixed
boolean operand set as JSON checked in under
`crates/opensolid-kernel/tests/data/step/reference/`.

Because those references are checked in, the comparison
(`crates/opensolid-kernel/tests/occ_reference.rs`) is hermetic and runs in
the default `cargo test`. OCC is only needed to refresh them, which the
`occ-references` job in the same workflow as this validator does weekly with
`--check`.

```bash
pip install cadquery-ocp
python3 scripts/occ_reference.py corpus     # refresh per-file references
python3 scripts/occ_reference.py booleans   # refresh the boolean differential
python3 scripts/occ_reference.py generate   # rebuild the occ/ edge-case files
python3 scripts/occ_reference.py corpus --check   # CI: fail on drift
```

## Related

- `crates/opensolid-kernel/examples/step_import_report.rs` — per-file import
  report + pass rate over any directory of STEP files (also used for the
  vendored corpus under `crates/opensolid-kernel/tests/data/step/`).
- `crates/opensolid-kernel/examples/step_export_samples.rs` — the sample
  exporter for direction 1.
- `crates/opensolid-kernel/tests/step_corpus.rs` — the hermetic corpus suite
  (structured outcomes + pass-rate floor) that runs in default CI.
- `crates/opensolid-kernel/tests/occ_reference.rs` — the hermetic OCC
  differential described above, also in default CI.
