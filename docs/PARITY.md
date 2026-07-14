# SolidWorks Parity Matrix — Playground CAD GUI

Audit of the OpenSolid playground (`web/playground`) against the core SolidWorks
part-modeling workflow. Scope: what a machinist/engineer expects from
sketch → feature → inspect, mapped to what the playground actually does today.

**Method.** Source-verified against the WASM API (`crates/opensolid-wasm/src/lib.rs`),
the kernel builder (`crates/opensolid-kernel/src/builder.rs`), and the React GUI
(`web/playground/src/`), cross-checked against the running `npm run dev` build.
The two prior user-reported gaps (sketch/extrude on curved surfaces; extrude has
no cut mode) are confirmed in code and included below.

**Status legend:** `complete` · `partial` · `missing` · `broken`.
**Severity:** how much the gap hurts a real part-modeling session
(`critical` blocks core workflow → `low` nice-to-have).

Audit bead: **of-fsl.1**. Epic: **of-fsl**. Every significant gap is tracked by a
child bead (column *Bead*). Beads of-fsl.2–8 pre-existed; of-fsl.9–20 filed by
this audit.

---

## What exists today

**Primitives (WASM):** sphere, box3, roundedBox, cylinder, torus, capsule.
**Sketch → feature:** `Profile(line/arc)` → `extrude(height)` / `revolve(deg)`.
**Transforms:** translate, rotate, scale, uniformScale.
**Booleans:** union, intersect, subtract, smoothUnion.
**GUI features toolbar:** Sketch · Extrude · Revolve · View · Export(STL/STEP).
**Sketch tools:** line, rect, circle, arc. **Constraints:** H, V, coincident,
tangent, length, radius. **Undo/redo:** sketch mode only.
**Kernel-only (not exposed to WASM/GUI):** `shell(thickness)`, `smooth(radius)`
(global all-edge rounding), `cone`.

---

## Sketching

| Feature | SolidWorks behavior | Status | Severity | Bead |
|---|---|---|---|---|
| Line / rectangle / circle / arc | Full primitive set + inference | **complete** | — | — |
| Slot / polygon / ellipse / spline | Dedicated tools | **missing** | medium | of-fsl.13 |
| Construction / centerline geometry | Reference geometry inside sketch | **missing** | medium | of-fsl.13 |
| Trim / extend / offset / sketch-mirror / convert-entities | Core edit tools | **missing** | high | of-fsl.13 |
| Constraints: horizontal, vertical, coincident, tangent | Auto + manual relations | **complete** | — | — |
| Constraints: parallel, perpendicular, equal, concentric, midpoint, symmetric, collinear, fix | Everyday relations | **missing** | high | of-fsl.11 |
| Dimension: length (line), radius (circle/arc) | Smart dimension | **partial** | — | — |
| Dimension: diameter, angle, point-to-point distance, driven/reference | Smart dimension | **missing** | med-high | of-fsl.12 |
| Fully-defined feedback (under/over-defined coloring) | Solver-driven | **partial** | medium | of-fsl.11 |

Sketching is the weakest-relative-to-SW area: only 4 draw tools, no edit tools,
6 of ~15 constraints, 2 of ~6 dimension types. A profile can be drawn and swept,
but constraining it to intent (parallel walls, equal holes, concentric bosses) is
mostly impossible.

## Modeling features

| Feature | SolidWorks behavior | Status | Severity | Bead |
|---|---|---|---|---|
| Extrude boss (blind) | Add material by height | **complete** | — | — |
| **Extrude cut** | Remove material with a sketch | **missing** ⚠️ | **critical** | of-fsl.2 |
| Extrude up-to-face / through-all / offset / symmetric | End conditions | **missing** | high | of-fsl.2 |
| Extrude draft angle | Taper during extrude | **missing** | medium | of-fsl.2 |
| Revolve boss (angle) | Spin profile about axis | **partial** | — | — |
| Revolve cut / thin / axis selection | Variants | **missing** | medium | of-fsl.2 |
| Sweep along path | Profile + guide path | **missing** | med-high | of-fsl.3 |
| Loft between profiles | Blend N profiles | **missing** | medium | of-fsl.3 |
| **Fillet / chamfer (edge-selective)** | Round/bevel picked edges | **missing** ⚠️ | **critical** | of-fsl.4 |
| Fillet (all-edges, global) | — | **partial** (kernel `smooth()` + `roundedBox`, unexposed) | high | of-fsl.4 |
| Shell | Hollow to wall thickness | **missing** (kernel `shell()` exists, unwired) | high | of-fsl.9 |
| Rib | Thicken open profile | **missing** | low-med | of-fsl.16 |
| Draft (standalone) | Taper existing faces | **missing** | medium | of-fsl.15 |
| Linear / circular pattern | Repeat features | **missing** (script loops only) | high | of-fsl.6 |
| Mirror | Reflect features/bodies | **missing** (script only) | high | of-fsl.6 |
| Boolean union/intersect/subtract | Combine bodies | **complete** | — | — |
| Parametric feature rebuild (edit → downstream regen) | Feature replay | **partial** (tree edit/suppress/delete; no true replay) | high | of-fsl.8 |

Extrude-**cut** and **selective fillet/chamfer** are the two headline gaps: they
are the two most-used SolidWorks features and neither exists. Extrude always
*unions* (`sweepTreeNode` in `src/lib/sweep.js` hardcodes `op:'union'`), so a
sketched pocket or hole is unreachable from the GUI.

## Curved-surface workflows

| Feature | SolidWorks behavior | Status | Severity | Bead |
|---|---|---|---|---|
| **Sketch on a curved (cylindrical/conical) face** | Wrap/planar-project sketch | **missing** ⚠️ | high | of-fsl.5 |
| Sketch on a planar face | Pick face → sketch | **complete** | — | — |
| Extrude up-to-surface | Terminate at a curved face | **missing** | high | of-fsl.5 |

`src/lib/facePlane.js` explicitly rejects non-planar regions
(`reason: 'face is curved'`, `SPREAD_TOL_DEG = 0.8°`), so the Sketch button is
inert on any cylinder/sphere/torus wall — confirming the user report.

## Reference geometry

| Feature | SolidWorks behavior | Status | Severity | Bead |
|---|---|---|---|---|
| Standard planes (front/top/right) | 3 datum planes | **complete** (XY/XZ/YZ) | — | — |
| Reference plane (offset / angled / mid / tangent) | User datum planes | **missing** | high | of-fsl.14 |
| Reference axis / point / coordinate system | User datums | **missing** | high | of-fsl.14 |

Only the 3 fixed origin planes plus a planar picked face are sketchable. There is
no way to, e.g., sketch on a plane offset 5 units above a face — a routine need.

## Inspection & display

| Feature | SolidWorks behavior | Status | Severity | Bead |
|---|---|---|---|---|
| Measure (distance/angle/radius/area) | Evaluate → Measure | **missing** (`bounds()` in API only) | medium | of-fsl.17 |
| Mass properties (volume/mass/CoG/inertia) | Evaluate → Mass Props | **missing** | low-med | of-fsl.19 |
| Materials (density assignment) | Appearance + physical | **missing** (render color only) | low-med | of-fsl.19 |
| Section view | Clip plane inspection | **missing** | medium | of-fsl.18 |
| Selection filters (face/edge/vertex/body) | Filter toolbar | **partial** (body + planar face; no edge/vertex 3D pick) | medium | of-fsl.17 |

## Environment & session

| Feature | SolidWorks behavior | Status | Severity | Bead |
|---|---|---|---|---|
| Units (document unit system) | mm/inch, unit-aware entry | **missing** (fully unitless; STEP lacks SI_UNIT) | medium | of-fsl.20 |
| Undo/redo — sketch mode | Deep history | **complete** | — | — |
| **Undo/redo — feature/model level** | Session-wide Ctrl+Z | **missing** ⚠️ | high | of-fsl.10 |
| View: orbit/pan/zoom, standard views, fit, iso, triad | Navigation | **complete** | — | — |
| View: wireframe toggle | Display style | **complete** | — | — |
| View: named views, perspective toggle, zoom-to-selection | Navigation extras | **missing** | low | — |
| Body visibility (show/hide) | Hide/show | **complete** (feature-tree eye toggle) | — | — |
| Export STL / STEP | Save geometry | **complete** | — | — |

Feature-level undo/redo is a notable data-loss hazard: pressing **Delete** on a
selected body (`App.jsx`) is irreversible — `Ctrl+Z` only works inside sketch mode.

---

## Summary

**Complete / near-complete:** boolean ops, blind extrude boss, basic revolve,
sketch-on-planar-face, view navigation, wireframe, body hide/show, STL/STEP
export, sketch-mode undo/redo, standard planes.

**Critical gaps (block core workflow):**
1. **Extrude cut** — of-fsl.2 (always unions; no sketched pockets/holes)
2. **Selective fillet/chamfer** — of-fsl.4 (the single most-used SW feature)

**High-severity gaps:** shell (kernel-ready) · sketch-on-curved-surface ·
feature-level undo/redo · linear/circular pattern · mirror · reference geometry ·
extrude end-conditions · sketch constraint set · sketch edit tools · parametric
rebuild.

**Two quick wins** (kernel already implements, only WASM binding + GUI missing):
- **Shell** — `Part::shell(thickness)` at `builder.rs:598` → of-fsl.9
- **Global fillet** — `Part::smooth(radius)` at `builder.rs:571` → of-fsl.4

**Bead coverage:** all significant gaps tracked under epic **of-fsl** —
of-fsl.2–8 (pre-existing verbs) + of-fsl.9–20 (filed by this audit). No fixes were
made; this is discovery only.
