#!/usr/bin/env bash
# External STEP validator orchestrator (of-3qy.10).
#
# Round-trips STEP both ways through an external CAD kernel:
#   1. ours → FreeCAD:  export sample parts with our writer, verify each is a
#      valid closed solid with the analytic volume in headless FreeCAD/OCC.
#   2. FreeCAD → ours:  generate parametric parts with FreeCAD's Part
#      workbench, import them with our reader, report the pass rate the spec
#      tracks (spec/06-step-io.md §Pass-rate targets: 80% on FreeCAD exports).
#
# Opt-in by design — default `cargo test` stays hermetic (no external
# binaries). Requires OPENSOLID_STEP_VALIDATOR=1 and a `freecadcmd` on PATH
# (override with FREECAD_CMD). Run from the repository root:
#
#   OPENSOLID_STEP_VALIDATOR=1 tools/step-validator/run.sh
#
set -euo pipefail

if [[ "${OPENSOLID_STEP_VALIDATOR:-}" != "1" ]]; then
  echo "external validator is opt-in: set OPENSOLID_STEP_VALIDATOR=1 to run" >&2
  exit 2
fi

FREECAD="${FREECAD_CMD:-freecadcmd}"
if ! command -v "$FREECAD" >/dev/null 2>&1; then
  echo "error: '$FREECAD' not found — install FreeCAD or set FREECAD_CMD" >&2
  exit 2
fi

here="$(cd "$(dirname "$0")" && pwd)"
root="$(cd "$here/../.." && pwd)"
work="${STEP_VALIDATOR_WORK:-$(mktemp -d "${TMPDIR:-/tmp}/step-validator.XXXXXX")}"
mkdir -p "$work/ours" "$work/theirs"
echo "work dir: $work"

status=0

echo
echo "── 1/2 ours → FreeCAD ─────────────────────────────────────────────"
cargo run --release --manifest-path "$root/Cargo.toml" \
  --example step_export_samples -- "$work/ours"
STEP_VALIDATOR_MODE=check STEP_VALIDATOR_DIR="$work/ours" \
  "$FREECAD" "$here/freecad_check.py" || status=1

echo
echo "── 2/2 FreeCAD → ours ─────────────────────────────────────────────"
STEP_VALIDATOR_MODE=generate STEP_VALIDATOR_DIR="$work/theirs" \
  "$FREECAD" "$here/freecad_check.py" || status=1
# STEP_IMPORT_MIN_RATE gates the FreeCAD→ours direction (default 0 =
# report-only; the spec's month-3 target is 80).
cargo run --release --manifest-path "$root/Cargo.toml" \
  --example step_import_report -- \
  --min-rate "${STEP_IMPORT_MIN_RATE:-0}" "$work/theirs" || status=1

echo
if [[ $status -eq 0 ]]; then
  echo "external validation PASSED"
else
  echo "external validation FAILED (see per-file lines above)" >&2
fi
exit $status
