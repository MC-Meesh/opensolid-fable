# External STEP validator — FreeCAD side (of-3qy.10).
#
# Runs headless under FreeCAD's Python (`freecadcmd freecad_check.py`). Both
# arguments come from the environment because freecadcmd's sys.argv handling
# differs across versions:
#
#   STEP_VALIDATOR_MODE=check    STEP_VALIDATOR_DIR=<dir>   — import every
#       *.step in <dir> and verify each is a valid, closed solid; if the dir
#       has a manifest.json ([{"file", "expected_volume"}, ...]) also compare
#       volumes. Prints one line per file and a final pass-rate line.
#
#   STEP_VALIDATOR_MODE=generate STEP_VALIDATOR_DIR=<dir>   — build a set of
#       parametric parts with FreeCAD's Part workbench and export each as
#       STEP into <dir>, to be imported by our reader (step_import_report).
#
# Exit code: 0 when every file passes (check) or every export writes
# (generate); 1 otherwise. The orchestrator (run.sh) turns pass rates into
# metrics; this script only reports facts.

import json
import os
import sys

import FreeCAD  # noqa: F401  (initializes the headless application)
import Part

REL_VOLUME_TOL = 1e-6


def fail(msg):
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(2)


def check(directory):
    manifest = {}
    manifest_path = os.path.join(directory, "manifest.json")
    if os.path.exists(manifest_path):
        with open(manifest_path) as fh:
            manifest = {e["file"]: e["expected_volume"] for e in json.load(fh)}

    files = sorted(
        f for f in os.listdir(directory) if f.lower().endswith((".step", ".stp"))
    )
    if not files:
        fail(f"no STEP files under {directory}")

    passed = 0
    for name in files:
        path = os.path.join(directory, name)
        try:
            shape = Part.Shape()
            shape.read(path)
        except Exception as e:  # noqa: BLE001 — report and continue
            print(f"FAIL  {name}: read error: {e}")
            continue
        problems = []
        if not shape.isValid():
            problems.append("BRepCheck reports invalid")
        solids = shape.Solids
        if not solids:
            problems.append("no solids")
        else:
            open_shells = sum(
                1 for s in solids for sh in s.Shells if not sh.isClosed()
            )
            if open_shells:
                problems.append(f"{open_shells} open shell(s)")
        volume = sum(s.Volume for s in solids) if solids else 0.0
        expected = manifest.get(name)
        if expected is not None and expected > 0.0:
            drift = abs(volume - expected) / expected
            if drift > REL_VOLUME_TOL:
                problems.append(
                    f"volume {volume:.6f} vs expected {expected:.6f} "
                    f"(drift {drift:.2e} > {REL_VOLUME_TOL:.0e})"
                )
        if problems:
            print(f"FAIL  {name}: {'; '.join(problems)}")
        else:
            passed += 1
            print(f"PASS  {name}: {len(solids)} solid(s), volume {volume:.6f}")

    print(f"\nfreecad-check pass rate: {passed}/{len(files)} "
          f"({100.0 * passed / len(files):.0f}%)")
    sys.exit(0 if passed == len(files) else 1)


def generate(directory):
    os.makedirs(directory, exist_ok=True)
    V = FreeCAD.Vector

    box = Part.makeBox(20, 30, 40)
    cylinder = Part.makeCylinder(15, 40)
    sphere = Part.makeSphere(20)
    torus = Part.makeTorus(30, 10)
    cone = Part.makeCone(15, 5, 30)
    fused = box.fuse(Part.makeCylinder(8, 60, V(10, 15, -10)))
    drilled = Part.makeBox(40, 40, 20).cut(
        Part.makeCylinder(8, 20, V(20, 20, 0))
    )
    common = Part.makeBox(20, 20, 20).common(
        Part.makeBox(20, 20, 20, V(10, 10, 10))
    )
    filleted = Part.makeBox(30, 30, 30)
    filleted = filleted.makeFillet(4.0, filleted.Edges)
    profile = Part.Face(
        Part.Wire(
            [
                Part.makeLine((10, 0, 0), (25, 0, 0)),
                Part.makeLine((25, 0, 0), (25, 0, 35)),
                Part.makeLine((25, 0, 35), (10, 0, 35)),
                Part.makeLine((10, 0, 35), (10, 0, 0)),
            ]
        )
    )
    revolved = profile.revolve(V(0, 0, 0), V(0, 0, 1), 360)

    samples = {
        "fc-box": box,
        "fc-cylinder": cylinder,
        "fc-sphere": sphere,
        "fc-torus": torus,
        "fc-cone": cone,
        "fc-fuse-box-cylinder": fused,
        "fc-box-through-hole": drilled,
        "fc-common-blocks": common,
        "fc-filleted-box": filleted,
        "fc-revolved-ring": revolved,
    }

    manifest = []
    failures = 0
    for name, shape in samples.items():
        path = os.path.join(directory, f"{name}.step")
        try:
            shape.exportStep(path)
            manifest.append(
                {"file": f"{name}.step", "expected_volume": shape.Volume}
            )
            print(f"wrote {name}.step (volume {shape.Volume:.6f})")
        except Exception as e:  # noqa: BLE001 — report and continue
            failures += 1
            print(f"FAIL  {name}: export error: {e}", file=sys.stderr)
    with open(os.path.join(directory, "manifest.json"), "w") as fh:
        json.dump(manifest, fh, indent=2)
    print(f"wrote manifest.json ({len(manifest)} samples) to {directory}")
    sys.exit(0 if failures == 0 else 1)


mode = os.environ.get("STEP_VALIDATOR_MODE")
directory = os.environ.get("STEP_VALIDATOR_DIR")
if not mode or not directory:
    fail("set STEP_VALIDATOR_MODE={check,generate} and STEP_VALIDATOR_DIR")
if mode == "check":
    check(directory)
elif mode == "generate":
    generate(directory)
else:
    fail(f"unknown STEP_VALIDATOR_MODE {mode!r}")
