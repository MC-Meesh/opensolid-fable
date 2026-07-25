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

## Related

- `crates/opensolid-kernel/examples/step_import_report.rs` — per-file import
  report + pass rate over any directory of STEP files (also used for the
  vendored corpus under `crates/opensolid-kernel/tests/data/step/`).
- `crates/opensolid-kernel/examples/step_export_samples.rs` — the sample
  exporter for direction 1.
- `crates/opensolid-kernel/tests/step_corpus.rs` — the hermetic corpus suite
  (structured outcomes + pass-rate floor) that runs in default CI.
