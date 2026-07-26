#!/usr/bin/env python3
"""OCC reference data generator (spec/11-testing.md §7, of-ipt.16).

OpenCASCADE is the ground truth for geometry: this script asks it what a STEP
file *contains* (per-solid volume, area, centroid, face/edge/vertex counts)
and what a boolean of a fixed operand set *produces*, and writes the answers
as JSON next to the corpus. Those JSONs are checked into git, so the Rust
side (`crates/opensolid-kernel/tests/occ_reference.rs`) compares our import
and our booleans against a real external kernel on every PR without needing
OCC installed.

OCC is only needed to *refresh* the references — that is the weekly
`external-step-validation` job, which also runs `--check` to prove the
checked-in data still matches live OCC.

Requires the OCP bindings (`pip install cadquery-ocp`; pythonocc's `OCC.Core`
is accepted too).

    # refresh every reference under the corpus tree
    python3 scripts/occ_reference.py corpus

    # one file to stdout (the spec's original single-file mode)
    python3 scripts/occ_reference.py analyze path/to/part.stp

    # refresh the boolean differential references
    python3 scripts/occ_reference.py booleans

    # rebuild the self-generated edge-case corpus files under occ/
    python3 scripts/occ_reference.py generate

    # CI: regenerate in memory, fail if what is checked in drifted
    python3 scripts/occ_reference.py corpus --check
    python3 scripts/occ_reference.py booleans --check
    python3 scripts/occ_reference.py generate --check

Exit codes: 0 success, 1 drift (under `--check`) or a file OCC could not
read, 2 usage or missing-bindings error.
"""

from __future__ import annotations

import argparse
import json
import os
import re
import sys

SCHEMA_CORPUS = "opensolid-occ-reference/1"
SCHEMA_BOOLEAN = "opensolid-occ-boolean-reference/1"

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CORPUS_ROOT = os.path.join(
    REPO_ROOT, "crates", "opensolid-kernel", "tests", "data", "step"
)
REFERENCE_DIRNAME = "reference"
BOOLEAN_REFERENCE = "booleans.json"

# Decimal digits kept for floating point fields. 12 significant digits is far
# below OCC's own integration error yet keeps the checked-in JSON stable
# across OCC patch releases and platforms.
SIGNIFICANT_DIGITS = 12


def die(msg: str, code: int = 2) -> None:
    print(f"error: {msg}", file=sys.stderr)
    sys.exit(code)


# ---------------------------------------------------------------------------
# OCC bindings (OCP = cadquery-ocp, OCC.Core = pythonocc)
# ---------------------------------------------------------------------------


def load_occ():
    """Import the OCC API from whichever binding is installed."""
    for package in ("OCP", "OCC.Core"):
        try:
            mods = {
                name: __import__(f"{package}.{name}", fromlist=[name])
                for name in (
                    "STEPControl",
                    "TopExp",
                    "TopAbs",
                    "TopTools",
                    "BRepGProp",
                    "GProp",
                    "BRepCheck",
                    "BRepPrimAPI",
                    "BRepAlgoAPI",
                    "BRepFilletAPI",
                    "BRepOffsetAPI",
                    "BRepBuilderAPI",
                    "GeomAPI",
                    "GeomAbs",
                    "TColgp",
                    "TopoDS",
                    "Interface",
                    "gp",
                )
            }
        except ImportError:
            continue
        return package, mods
    die(
        "no OCC bindings found — install one of:\n"
        "  pip install cadquery-ocp     (OCP, what this script is generated with)\n"
        "  pip install pythonocc-core   (OCC.Core)"
    )


def occ_version(package: str) -> str:
    """Best-effort version string for the binding, for provenance."""
    import importlib.metadata as md

    for dist in ("cadquery-ocp", "pythonocc-core"):
        try:
            return f"{dist} {md.version(dist)}"
        except md.PackageNotFoundError:
            continue
    return package


class Occ:
    """Thin façade over the handful of OCC calls this script makes.

    Both bindings expose the same OCCT classes; they differ only in whether
    static methods carry the `_s` suffix (OCP) or not (pythonocc).
    """

    def __init__(self) -> None:
        package, mods = load_occ()
        self.package = package
        self.version = occ_version(package)
        self.m = mods

    def _static(self, cls, name: str):
        fn = getattr(cls, f"{name}_s", None)
        return fn if fn is not None else getattr(cls, name)

    def read_step(self, path: str):
        """Return (root_shape, status_name). Raises RuntimeError on failure."""
        reader = self.m["STEPControl"].STEPControl_Reader()
        status = reader.ReadFile(path)
        name = getattr(status, "name", str(status))
        if "RetDone" not in name:
            raise RuntimeError(f"OCC could not read the file: {name}")
        reader.TransferRoots()
        if reader.NbShapes() < 1:
            raise RuntimeError("OCC transferred no shapes")
        return reader.OneShape(), name

    def sub_count(self, shape, kind: str) -> int:
        """Unique sub-shapes of `kind` — the map dedupes shared topology."""
        topabs = self.m["TopAbs"]
        mapping = self.m["TopTools"].TopTools_IndexedMapOfShape()
        self._static(self.m["TopExp"].TopExp, "MapShapes")(
            shape, getattr(topabs, f"TopAbs_{kind}"), mapping
        )
        return mapping.Extent()

    def solids(self, shape) -> list:
        topabs = self.m["TopAbs"]
        explorer = self.m["TopExp"].TopExp_Explorer(shape, topabs.TopAbs_SOLID)
        out = []
        while explorer.More():
            out.append(explorer.Current())
            explorer.Next()
        return out

    def volume_props(self, shape):
        props = self.m["GProp"].GProp_GProps()
        self._static(self.m["BRepGProp"].BRepGProp, "VolumeProperties")(shape, props)
        return props

    def area_props(self, shape):
        props = self.m["GProp"].GProp_GProps()
        self._static(self.m["BRepGProp"].BRepGProp, "SurfaceProperties")(shape, props)
        return props

    def is_valid(self, shape) -> bool:
        return bool(self.m["BRepCheck"].BRepCheck_Analyzer(shape).IsValid())

    # -- primitives + booleans, for the differential reference ---------------

    def box(self, size, at):
        gp = self.m["gp"]
        corner = gp.gp_Pnt(
            at[0] - size[0] / 2.0, at[1] - size[1] / 2.0, at[2] - size[2] / 2.0
        )
        return self.m["BRepPrimAPI"].BRepPrimAPI_MakeBox(corner, *size).Shape()

    def cylinder(self, radius, height, at):
        gp = self.m["gp"]
        # Our `primitives::cylinder` is centered on the origin and spans
        # ±height/2 along +Z; OCC's grows from its axis origin along +Z.
        axis = gp.gp_Ax2(
            gp.gp_Pnt(at[0], at[1], at[2] - height / 2.0), gp.gp_Dir(0.0, 0.0, 1.0)
        )
        return (
            self.m["BRepPrimAPI"].BRepPrimAPI_MakeCylinder(axis, radius, height).Shape()
        )

    def sphere(self, radius, at):
        gp = self.m["gp"]
        return (
            self.m["BRepPrimAPI"]
            .BRepPrimAPI_MakeSphere(gp.gp_Pnt(*at), radius)
            .Shape()
        )

    def as_edge(self, shape):
        """Downcast a TopoDS_Shape from an explorer to a TopoDS_Edge."""
        topods = self.m["TopoDS"].TopoDS
        return self._static(topods, "Edge")(shape)

    def torus(self, major, minor, at):
        gp = self.m["gp"]
        axis = gp.gp_Ax2(gp.gp_Pnt(*at), gp.gp_Dir(0.0, 0.0, 1.0))
        return self.m["BRepPrimAPI"].BRepPrimAPI_MakeTorus(axis, major, minor).Shape()

    def cone(self, r1, r2, height, at):
        gp = self.m["gp"]
        axis = gp.gp_Ax2(
            gp.gp_Pnt(at[0], at[1], at[2] - height / 2.0), gp.gp_Dir(0.0, 0.0, 1.0)
        )
        return (
            self.m["BRepPrimAPI"]
            .BRepPrimAPI_MakeCone(axis, r1, r2, height)
            .Shape()
        )

    def boolean(self, op: str, a, b):
        algo = self.m["BRepAlgoAPI"]
        builder = {
            "unite": algo.BRepAlgoAPI_Fuse,
            "subtract": algo.BRepAlgoAPI_Cut,
            "intersect": algo.BRepAlgoAPI_Common,
        }[op](a, b)
        builder.Build()
        if not builder.IsDone():
            raise RuntimeError(f"BRepAlgoAPI {op} failed")
        return builder.Shape()


# ---------------------------------------------------------------------------
# Reference records
# ---------------------------------------------------------------------------


def rounded(value: float) -> float:
    """Round to SIGNIFICANT_DIGITS significant digits (not decimal places)."""
    if value == 0.0 or not (value == value) or value in (float("inf"), float("-inf")):
        return 0.0 if value == 0.0 else value
    return float(f"%.{SIGNIFICANT_DIGITS}g" % value)


def fnv1a64(data: bytes) -> int:
    """FNV-1a, 64-bit. Trivially reimplementable on the Rust side, which is
    the point: the test detects a corpus file edited without regenerating
    its reference."""
    h = 0xCBF29CE484222325
    for byte in data:
        h = ((h ^ byte) * 0x100000001B3) & 0xFFFFFFFFFFFFFFFF
    return h


def shape_record(occ: Occ, shape) -> dict:
    vprops = occ.volume_props(shape)
    aprops = occ.area_props(shape)
    com = vprops.CentreOfMass()
    return {
        "volume": rounded(vprops.Mass()),
        "area": rounded(aprops.Mass()),
        "centroid": [rounded(c) for c in (com.X(), com.Y(), com.Z())],
        "faces": occ.sub_count(shape, "FACE"),
        "edges": occ.sub_count(shape, "EDGE"),
        "vertices": occ.sub_count(shape, "VERTEX"),
        "shells": occ.sub_count(shape, "SHELL"),
        "valid": occ.is_valid(shape),
    }


def analyze(occ: Occ, path: str, source_name: str) -> dict:
    """OCC's account of one STEP file, as the reference record."""
    shape, status = occ.read_step(path)
    with open(path, "rb") as fh:
        data = fh.read()

    solids = [shape_record(occ, solid) for solid in occ.solids(shape)]
    # Traversal order is an OCC implementation detail; volume descending is
    # stable and is the order the Rust comparison matches on.
    solids.sort(key=lambda s: (-s["volume"], -s["faces"], -s["edges"]))
    for index, solid in enumerate(solids):
        solid["index"] = index

    return {
        "schema": SCHEMA_CORPUS,
        "generator": "scripts/occ_reference.py",
        "occ": occ.version,
        "source": {
            "file": source_name,
            "bytes": len(data),
            "fnv1a64": f"{fnv1a64(data):016x}",
        },
        "read_status": status,
        "totals": {
            "solids": len(solids),
            "faces": sum(s["faces"] for s in solids),
            "edges": sum(s["edges"] for s in solids),
            "vertices": sum(s["vertices"] for s in solids),
            "volume": rounded(sum(s["volume"] for s in solids)),
            "area": rounded(sum(s["area"] for s in solids)),
        },
        "solids": solids,
    }


# ---------------------------------------------------------------------------
# The boolean differential operand set
# ---------------------------------------------------------------------------

# Fixed operands, in millimetres, with the same conventions as
# `opensolid_kernel::brep::primitives`: every primitive is built centered on
# the origin and then translated by `at`. The Rust side (`occ_reference.rs`)
# reconstructs exactly these and compares its boolean result against OCC's.
#
# Keep this list append-only: the checked-in JSON and the Rust case table are
# matched by `name`.
BOOLEAN_CASES = [
    {
        "name": "blocks-union-corner",
        "op": "unite",
        "a": {"kind": "block", "size": [20.0, 20.0, 20.0], "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "block", "size": [20.0, 20.0, 20.0], "at": [10.0, 10.0, 10.0]},
    },
    {
        "name": "blocks-intersect-corner",
        "op": "intersect",
        "a": {"kind": "block", "size": [20.0, 20.0, 20.0], "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "block", "size": [20.0, 20.0, 20.0], "at": [10.0, 10.0, 10.0]},
    },
    {
        "name": "blocks-subtract-l-shape",
        "op": "subtract",
        "a": {"kind": "block", "size": [20.0, 20.0, 20.0], "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "block", "size": [20.0, 20.0, 20.0], "at": [10.0, 10.0, 10.0]},
    },
    {
        "name": "block-union-cylinder",
        "op": "unite",
        "a": {"kind": "block", "size": [40.0, 40.0, 20.0], "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "cylinder", "radius": 8.0, "height": 60.0, "at": [0.0, 0.0, 0.0]},
    },
    {
        "name": "block-through-hole",
        "op": "subtract",
        "a": {"kind": "block", "size": [40.0, 40.0, 20.0], "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "cylinder", "radius": 8.0, "height": 60.0, "at": [0.0, 0.0, 0.0]},
    },
    {
        "name": "block-offset-hole",
        "op": "subtract",
        "a": {"kind": "block", "size": [40.0, 40.0, 20.0], "at": [0.0, 0.0, 0.0]},
        "b": {
            "kind": "cylinder",
            "radius": 6.0,
            "height": 60.0,
            "at": [7.0, -5.0, 0.0],
        },
    },
    {
        "name": "block-intersect-cylinder",
        "op": "intersect",
        "a": {"kind": "block", "size": [40.0, 40.0, 20.0], "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "cylinder", "radius": 8.0, "height": 60.0, "at": [0.0, 0.0, 0.0]},
    },
    {
        "name": "block-subtract-sphere",
        "op": "subtract",
        "a": {"kind": "block", "size": [40.0, 40.0, 40.0], "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "sphere", "radius": 15.0, "at": [0.0, 0.0, 0.0]},
    },
    {
        "name": "sphere-union-cylinder",
        "op": "unite",
        "a": {"kind": "sphere", "radius": 20.0, "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "cylinder", "radius": 8.0, "height": 60.0, "at": [0.0, 0.0, 0.0]},
    },
    {
        "name": "cylinders-cross-union",
        "op": "unite",
        "a": {"kind": "cylinder", "radius": 10.0, "height": 40.0, "at": [0.0, 0.0, 0.0]},
        "b": {"kind": "cylinder", "radius": 6.0, "height": 60.0, "at": [4.0, 0.0, 0.0]},
    },
]


def build_operand(occ: Occ, spec: dict):
    kind = spec["kind"]
    if kind == "block":
        return occ.box(spec["size"], spec["at"])
    if kind == "cylinder":
        return occ.cylinder(spec["radius"], spec["height"], spec["at"])
    if kind == "sphere":
        return occ.sphere(spec["radius"], spec["at"])
    die(f"unknown operand kind {kind!r}")


def boolean_reference(occ: Occ) -> dict:
    cases = []
    for case in BOOLEAN_CASES:
        a = build_operand(occ, case["a"])
        b = build_operand(occ, case["b"])
        shape = occ.boolean(case["op"], a, b)
        record = shape_record(occ, shape)
        record["solids"] = occ.sub_count(shape, "SOLID")
        cases.append({**case, "result": record})
        print(
            f"{case['name']}: volume {record['volume']:.6f}, area "
            f"{record['area']:.6f}, {record['faces']} face(s)"
        )
    return {
        "schema": SCHEMA_BOOLEAN,
        "generator": "scripts/occ_reference.py",
        "occ": occ.version,
        "note": (
            "BRepAlgoAPI results for a fixed operand set. Operands follow "
            "opensolid_kernel::brep::primitives conventions: built centered "
            "on the origin, then translated by `at`."
        ),
        "cases": cases,
    }


# ---------------------------------------------------------------------------
# Edge-case corpus generation (spec/11-testing.md §8.1)
# ---------------------------------------------------------------------------
#
# The families the spec calls for — periodic surfaces, tangent booleans,
# coincident faces, thin features, high-degree NURBS, blends — as small
# self-generated STEP files. Self-generated means no licensing question, and
# the exporting kernel is the same one that writes the reference JSON, so
# every file lands in the corpus with an exact oracle attached.
#
# Written across three schemas on purpose: the reader must map
# MANIFOLD_SOLID_BREP regardless of the declared schema.


def bspline_face(occ: Occ, wave: float):
    """A degree-5+ B-spline patch fitted through a 6×6 point grid."""
    gp = occ.m["gp"]
    array = occ.m["TColgp"].TColgp_Array2OfPnt(1, 6, 1, 6)
    for i in range(1, 7):
        for j in range(1, 7):
            x = (i - 1) * 10.0
            y = (j - 1) * 10.0
            # A deterministic non-planar height field: nothing here is
            # random, so regenerating the corpus reproduces the same patch.
            z = wave * ((i - 3.5) ** 2 - (j - 3.5) ** 2) / 6.0
            array.SetValue(i, j, gp.gp_Pnt(x, y, z))
    surface = occ.m["GeomAPI"].GeomAPI_PointsToBSplineSurface(
        array, 5, 8, occ.m["GeomAbs"].GeomAbs_Shape.GeomAbs_C2, 1.0e-4
    ).Surface()
    return (
        occ.m["BRepBuilderAPI"]
        .BRepBuilderAPI_MakeFace(surface, 1.0e-6)
        .Shape()
    )


def edge_case_shapes(occ: Occ) -> list[tuple[str, str, object]]:
    """(relative path, STEP schema, shape) for every generated edge case."""
    gp = occ.m["gp"]
    prim = occ.m["BRepPrimAPI"]
    out: list[tuple[str, str, object]] = []

    # -- periodic surfaces: every closed quadric, including the apex cone --
    out.append(("periodic/cylinder_full", "AP203", occ.cylinder(12.0, 30.0, (0, 0, 0))))
    out.append(("periodic/sphere_full", "AP203", occ.sphere(14.0, (0, 0, 0))))
    out.append(("periodic/torus_full", "AP214IS", occ.torus(30.0, 8.0, (0, 0, 0))))
    out.append(("periodic/cone_apex", "AP203", occ.cone(15.0, 0.0, 25.0, (0, 0, 0))))
    out.append(
        ("periodic/cone_truncated", "AP203", occ.cone(15.0, 6.0, 25.0, (0, 0, 0)))
    )

    # -- tangency: the classic near-degenerate boolean configurations ------
    # A cylinder whose wall is exactly tangent to the block's side face.
    block = occ.box((40.0, 40.0, 20.0), (0, 0, 0))
    tangent_cyl = occ.cylinder(6.0, 40.0, (14.0, 0.0, 0.0))
    out.append(
        ("tangent/cylinder_tangent_to_wall", "AP203", occ.boolean("unite", block, tangent_cyl))
    )
    # A hole tangent to the block wall from the inside.
    out.append(
        (
            "tangent/hole_tangent_to_wall",
            "AP203",
            occ.boolean(
                "subtract",
                occ.box((40.0, 40.0, 20.0), (0, 0, 0)),
                occ.cylinder(6.0, 40.0, (14.0, 0.0, 0.0)),
            ),
        )
    )
    # Sphere seated in a cylindrical pocket of exactly its own radius:
    # tangent along a whole circle rather than at a point.
    out.append(
        (
            "tangent/sphere_in_matching_pocket",
            "AP214IS",
            occ.boolean(
                "unite",
                occ.cylinder(10.0, 20.0, (0, 0, 0)),
                occ.sphere(10.0, (0.0, 0.0, 10.0)),
            ),
        )
    )

    # -- coincident faces: two blocks fused across a shared full face -----
    out.append(
        (
            "coincident/blocks_shared_face",
            "AP203",
            occ.boolean(
                "unite",
                occ.box((20.0, 20.0, 20.0), (0, 0, 0)),
                occ.box((20.0, 20.0, 20.0), (0.0, 0.0, 20.0)),
            ),
        )
    )
    # Partial overlap: the shared region is a sub-rectangle of both faces.
    out.append(
        (
            "coincident/blocks_partial_overlap",
            "AP203",
            occ.boolean(
                "unite",
                occ.box((20.0, 20.0, 20.0), (0, 0, 0)),
                occ.box((10.0, 30.0, 20.0), (0.0, 5.0, 20.0)),
            ),
        )
    )

    # -- thin features: sub-millimetre extents next to 50 mm ones ---------
    out.append(("thin/plate_10um", "AP203", occ.box((50.0, 50.0, 0.01), (0, 0, 0))))
    out.append(
        (
            "thin/rib_50um",
            "AP203",
            occ.boolean(
                "unite",
                occ.box((40.0, 40.0, 4.0), (0, 0, 0)),
                occ.box((0.05, 40.0, 12.0), (0.0, 0.0, 8.0)),
            ),
        )
    )

    # -- freeform: high-degree NURBS, as a prism and as a loft ------------
    prism = prim.BRepPrimAPI_MakePrism(
        bspline_face(occ, 6.0), gp.gp_Vec(0.0, 0.0, -20.0)
    ).Shape()
    out.append(("nurbs/bspline_patch_prism", "AP203", prism))

    # ThruSections over three circles smooths a B-spline lateral surface.
    loft = occ.m["BRepOffsetAPI"].BRepOffsetAPI_ThruSections(True, False, 1.0e-6)
    for radius, height in ((10.0, 0.0), (16.0, 12.0), (7.0, 24.0)):
        circle = occ.m["gp"].gp_Circ(
            gp.gp_Ax2(gp.gp_Pnt(0.0, 0.0, height), gp.gp_Dir(0.0, 0.0, 1.0)), radius
        )
        edge = occ.m["BRepBuilderAPI"].BRepBuilderAPI_MakeEdge(circle).Edge()
        loft.AddWire(occ.m["BRepBuilderAPI"].BRepBuilderAPI_MakeWire(edge).Wire())
    loft.Build()
    out.append(("nurbs/lofted_vase", "AP242DIS", loft.Shape()))

    # -- blends: fillets and chamfers, the surfaces exporters get wrong ---
    box = occ.box((30.0, 30.0, 30.0), (0, 0, 0))
    fillet = occ.m["BRepFilletAPI"].BRepFilletAPI_MakeFillet(box)
    explorer = occ.m["TopExp"].TopExp_Explorer(box, occ.m["TopAbs"].TopAbs_EDGE)
    while explorer.More():
        fillet.Add(4.0, occ.as_edge(explorer.Current()))
        explorer.Next()
    fillet.Build()
    out.append(("blend/filleted_box", "AP214IS", fillet.Shape()))

    box2 = occ.box((30.0, 30.0, 30.0), (0, 0, 0))
    chamfer = occ.m["BRepFilletAPI"].BRepFilletAPI_MakeChamfer(box2)
    explorer = occ.m["TopExp"].TopExp_Explorer(box2, occ.m["TopAbs"].TopAbs_EDGE)
    while explorer.More():
        chamfer.Add(3.0, occ.as_edge(explorer.Current()))
        explorer.Next()
    chamfer.Build()
    out.append(("blend/chamfered_box", "AP203", chamfer.Shape()))

    return out


# The FILE_NAME timestamp OCC stamps into every export would make the corpus
# churn on each regeneration. These files are self-generated, so normalizing
# the header costs nothing and keeps `generate` reproducible: rerunning it
# rewrites byte-identical files unless the geometry actually changed.
FIXED_TIMESTAMP = "2026-07-26T00:00:00"
# The same instant as CALENDAR_DATE spells it: (year, day, month).
FIXED_DATE = (2026, 26, 7)


def split_records(text: str) -> list[str]:
    """Split Part 21 text into `;`-terminated records.

    Quoting matters: a header's `'2;1'` implementation level carries a
    semicolon inside a string, so a naive split truncates the record.
    """
    records = []
    start = 0
    in_string = False
    index = 0
    while index < len(text):
        char = text[index]
        if in_string:
            if char == "'":
                # '' is an escaped quote inside a Part 21 string.
                if text[index + 1 : index + 2] == "'":
                    index += 1
                else:
                    in_string = False
        elif char == "'":
            in_string = True
        elif char == ";":
            records.append(text[start : index + 1])
            start = index + 1
        index += 1
    if text[start:]:
        records.append(text[start:])
    return records


def normalize_step(text: str, name: str) -> str:
    """Strip every wall-clock stamp OCC writes, so `generate` is reproducible.

    Two in the header (`FILE_NAME`, `FILE_DESCRIPTION`), and — only in the
    AP203 CONFIG_CONTROL_DESIGN flavour — a `CALENDAR_DATE`/`LOCAL_TIME` pair
    in the DATA section carrying the creation and approval times, which
    otherwise change every minute. Everything else is OCC's own output,
    untouched.

    `FILE_NAME` and `FILE_DESCRIPTION` wrap across lines in OCC's writer, so
    the header part works record-wise, not line-wise.
    """
    out = []
    for record in split_records(text):
        stripped = record.lstrip()
        if stripped.startswith("FILE_NAME("):
            out.append(
                f"\nFILE_NAME('{name}','{FIXED_TIMESTAMP}',('OpenSolid'),"
                "('scripts/occ_reference.py generate'),"
                "'Open CASCADE STEP processor','OpenSolid','');"
            )
        elif stripped.startswith("FILE_DESCRIPTION("):
            out.append(
                "\nFILE_DESCRIPTION(('OpenSolid generated edge-case corpus file'),'2;1');"
            )
        else:
            out.append(record)
    normalized = "".join(out)
    normalized = re.sub(
        r"CALENDAR_DATE\(\s*\d+\s*,\s*\d+\s*,\s*\d+\s*\)",
        f"CALENDAR_DATE({FIXED_DATE[0]},{FIXED_DATE[1]},{FIXED_DATE[2]})",
        normalized,
    )
    return re.sub(
        r"LOCAL_TIME\(\s*\d+\s*,\s*\d+\s*,\s*[^,]*,\s*(#\d+)\s*\)",
        r"LOCAL_TIME(0,0,$,\1)",
        normalized,
    )


def mode_generate(args) -> int:
    occ = Occ()
    root = os.path.abspath(args.root)
    target = os.path.join(root, args.dir)
    # The STEP controller must be initialized before `write.step.schema`
    # exists as a static; without it every SetCVal silently returns False and
    # every file comes out in the writer's default schema.
    controller = occ.m["STEPControl"].STEPControl_Controller
    occ._static(controller, "Init")()
    static = occ.m["Interface"].Interface_Static
    set_cval = getattr(static, "SetCVal_s", None) or static.SetCVal

    shapes = edge_case_shapes(occ)
    written = 0
    for relative, schema, shape in shapes:
        path = os.path.join(target, f"{relative}.stp")
        os.makedirs(os.path.dirname(path), exist_ok=True)
        assert set_cval("write.step.schema", schema), f"unknown STEP schema {schema}"
        writer = occ.m["STEPControl"].STEPControl_Writer()
        writer.Transfer(shape, occ.m["STEPControl"].STEPControl_StepModelType.STEPControl_AsIs)
        tmp = path + ".tmp"
        writer.Write(tmp)
        with open(tmp, encoding="utf-8") as fh:
            text = fh.read()
        os.remove(tmp)
        name = f"{os.path.basename(relative)}.stp"
        normalized = normalize_step(text, name)
        if args.check:
            current = open(path, encoding="utf-8").read() if os.path.exists(path) else None
            if current != normalized:
                print(f"DRIFTED   {os.path.relpath(path, REPO_ROOT)}")
                continue
            print(f"ok        {os.path.relpath(path, REPO_ROOT)}")
        else:
            with open(path, "w", encoding="utf-8") as fh:
                fh.write(normalized)
            print(f"wrote     {os.path.relpath(path, REPO_ROOT)} ({schema})")
        written += 1

    verb = "checked" if args.check else "wrote"
    print(f"\n{verb} {written}/{len(shapes)} edge-case file(s) with {occ.version}")
    if args.check and written != len(shapes):
        print(
            "edge-case files no longer match the generator — rerun "
            "`python3 scripts/occ_reference.py generate` and commit the result",
            file=sys.stderr,
        )
    return 0 if written == len(shapes) else 1


# ---------------------------------------------------------------------------
# Modes
# ---------------------------------------------------------------------------


def corpus_files(root: str) -> list[str]:
    out = []
    for dirpath, dirnames, filenames in os.walk(root):
        dirnames[:] = sorted(d for d in dirnames if d != REFERENCE_DIRNAME)
        for name in sorted(filenames):
            if name.lower().endswith((".stp", ".step")):
                out.append(os.path.join(dirpath, name))
    return sorted(out)


def reference_path(root: str, step_path: str) -> str:
    """`<root>/foo/bar.stp` → `<root>/reference/foo/bar.json`."""
    relative = os.path.relpath(step_path, root)
    stem = os.path.splitext(relative)[0]
    return os.path.join(root, REFERENCE_DIRNAME, f"{stem}.json")


def serialize(record: dict) -> str:
    return json.dumps(record, indent=2, sort_keys=False) + "\n"


def write_or_check(path: str, record: dict, check: bool) -> bool:
    """Write `record` to `path`, or (under --check) report whether it drifted.

    Returns True when the checked-in file is up to date / was written.
    """
    text = serialize(record)
    if check:
        if not os.path.exists(path):
            print(f"MISSING   {os.path.relpath(path, REPO_ROOT)}")
            return False
        with open(path, encoding="utf-8") as fh:
            current = fh.read()
        if current != text:
            print(f"DRIFTED   {os.path.relpath(path, REPO_ROOT)}")
            return False
        print(f"ok        {os.path.relpath(path, REPO_ROOT)}")
        return True
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(text)
    print(f"wrote     {os.path.relpath(path, REPO_ROOT)}")
    return True


def mode_corpus(args) -> int:
    occ = Occ()
    root = os.path.abspath(args.root)
    files = corpus_files(root)
    if not files:
        die(f"no STEP files under {root}", 1)
    failures = 0
    for step_path in files:
        name = os.path.relpath(step_path, root)
        try:
            record = analyze(occ, step_path, name)
        except RuntimeError as e:
            print(f"FAIL      {name}: {e}", file=sys.stderr)
            failures += 1
            continue
        if not write_or_check(reference_path(root, step_path), record, args.check):
            failures += 1
    verb = "checked" if args.check else "wrote"
    print(f"\n{verb} {len(files) - failures}/{len(files)} reference(s) with {occ.version}")
    if failures and args.check:
        print(
            "references are stale — run `python3 scripts/occ_reference.py corpus` "
            "and commit the result",
            file=sys.stderr,
        )
    return 1 if failures else 0


def mode_analyze(args) -> int:
    occ = Occ()
    path = os.path.abspath(args.file)
    try:
        record = analyze(occ, path, os.path.basename(path))
    except RuntimeError as e:
        die(str(e), 1)
    if args.out:
        write_or_check(os.path.abspath(args.out), record, args.check)
    else:
        sys.stdout.write(serialize(record))
    return 0


def mode_booleans(args) -> int:
    occ = Occ()
    record = boolean_reference(occ)
    path = args.out or os.path.join(
        os.path.abspath(args.root), REFERENCE_DIRNAME, BOOLEAN_REFERENCE
    )
    return 0 if write_or_check(path, record, args.check) else 1


def main(argv: list[str]) -> int:
    # Shared options, accepted both before and after the subcommand.
    common = argparse.ArgumentParser(add_help=False)
    common.add_argument(
        "--root",
        default=CORPUS_ROOT,
        help="corpus root (default: crates/opensolid-kernel/tests/data/step)",
    )
    common.add_argument(
        "--check",
        action="store_true",
        help="do not write; exit non-zero if the checked-in JSON differs",
    )
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0], parents=[common]
    )
    sub = parser.add_subparsers(dest="mode", required=True)
    sub.add_parser("corpus", help="refresh every per-file reference", parents=[common])
    analyze_parser = sub.add_parser(
        "analyze", help="analyze one STEP file", parents=[common]
    )
    analyze_parser.add_argument("file")
    analyze_parser.add_argument("--out", help="write JSON here instead of stdout")
    booleans_parser = sub.add_parser(
        "booleans",
        help="refresh the BRepAlgoAPI differential reference",
        parents=[common],
    )
    booleans_parser.add_argument("--out", help="write JSON here")
    generate_parser = sub.add_parser(
        "generate",
        help="(re)write the self-generated edge-case corpus files",
        parents=[common],
    )
    generate_parser.add_argument(
        "--dir",
        default="occ",
        help="subdirectory of the corpus root to write into (default: occ)",
    )
    args = parser.parse_args(argv)

    return {
        "corpus": mode_corpus,
        "analyze": mode_analyze,
        "booleans": mode_booleans,
        "generate": mode_generate,
    }[args.mode](args)


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
