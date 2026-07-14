# 2D engineering drawings — views, dimensions, export

Status: design note only (of-fsl.26). No implementation in this change.
Scope: generating 2D orthographic drawings (front/top/right/iso), dimensioned
and exported (SVG first), from a modeled body. Both model paths are covered —
the F-Rep mesh path (fast, approximate) and the exact B-Rep path (precise
analytic edges) — with an honest split between what the MVP ships and what is
deferred.

A drawing is the deliverable a machinist actually receives: a dimensioned,
scaled, multi-view sheet. It is *not* the 3D model with a camera pointed at it.
The two hard problems are (1) turning a solid into clean 2D line-work with
correct **visible/hidden edge classification** (hidden-line removal, HLR), and
(2) keeping dimensions **associative** to the model so an edit regenerates the
drawing. Everything else — sheet layout, title block, SVG serialization — is
bookkeeping. This note concentrates on the two hard problems and slots the rest
into the existing kernel + playground.

## 1. What a drawing is made of

A single view is a set of 2D polylines in sheet coordinates, each tagged
`visible` (solid line) or `hidden` (dashed), produced by projecting the body's
edges along a view direction and removing the ones occluded by the body itself.
The pipeline is the same for every view; only the view transform changes:

```
edges3d  = extract edges of the body        (§2.1)
V        = view transform (world → view)     (§2.2, reuse views.js directions)
seg2d    = drop depth of V·edges3d           (orthographic projection)
class    = occlusion test each seg vs body   (§2.3, HLR)
place    = seg2d · scale + view_origin        (§3, sheet layout)
```

Nothing in this pipeline exists yet — but every input does. `views.js`
(`web/playground/src/lib/views.js`, `VIEW_DIRECTIONS` = front/back/left/right/
top/bottom/iso) already defines the standard orthographic directions the CAD
world expects; the mesher already recovers feature edges; the B-Rep store
already holds analytic edges and can project onto them.

## 2. View generation

### 2.1 Edge sources — two paths

**F-Rep / mesh path (fast, MVP).** The dual-contouring mesher
(`crates/opensolid-frep/src/mesh.rs`) plus `refine.rs`
(`crates/opensolid-frep/src/refine.rs`) already classifies vertices as
crease/corner via `is_feature()` and tracks **crease polylines** — these are
exactly the CSG feature edges a drawing wants for its solid lines. The catch:
`MeshData` (the WASM output in `crates/opensolid-wasm/src/lib.rs`, ~line 120)
today surfaces only `positions`/`normals`/`indices`. **The feature edges are
computed internally and thrown away at the WASM boundary.** The first concrete
task is to export a crease-edge buffer (`feature_edges: Vec<f32>`, flat
`[x0,y0,z0, x1,y1,z1, …]`) alongside the mesh — the same flat-polyline
convention `filletEdge`/`chamferEdge` already use for CSG-edge input.

Feature edges alone are not the full drawing outline. The other required class
is the **silhouette** (outline) edge: where the surface turns away from the
view, i.e. the locus `n(p)·view_dir = 0`. On a mesh this is cheap and robust:
a triangle mesh silhouette is the set of edges whose two adjacent faces have
`sign(n_face·view_dir)` differing — a per-view mesh walk, no analytic solve. On
a smooth F-Rep surface (a cylinder, a fillet) the silhouette is *view-dependent*
and must be recomputed per view; the mesh-edge sign test approximates it to mesh
resolution, which is exactly the accuracy the rest of the F-Rep path already
commits to.

**Exact B-Rep path (precise, follow-up).** The B-Rep store
(`crates/opensolid-brep/src/topology.rs`) holds `Edge`/`Fin`/`Face` topology and
`crates/opensolid-brep/src/curve.rs` holds analytic `Curve3 { Line, Polyline,
Circle, Ellipse }`. Model edges are already exact — projecting an analytic edge
is evaluating the curve (`CurveEval`) at samples and applying the view
transform, and true silhouettes of analytic surfaces (a cylinder's outline is
two lines; a sphere's is a circle) can be derived closed-form rather than
sampled. `crates/opensolid-brep/src/project.rs` (`CurveProject`) gives
point→edge projection for snapping dimension anchors. This path yields
publication-clean vector output but inherits the classically hard problem of
**exact HLR** (curve–curve visibility), deferred in §2.3.

### 2.2 The view transform

Orthographic projection is a rigid view rotation followed by dropping the depth
axis. Reuse `Transform3` (nalgebra `Isometry3`, `crates/opensolid-core/src/
types.rs`) to build `world → view` from a `VIEW_DIRECTIONS` vector plus an
up-vector, then project `(x, y, z_view) → (x, y)` keeping `z_view` as the depth
key for occlusion sorting. Iso is the same machinery with the iso direction.
There is no `Point2`/`Transform2` in `opensolid-core` today (2D is ad-hoc
`[f64; 2]`, e.g. `Profile2D` in `crates/opensolid-frep/src/profile.rs`); the
drawing code can stay with `[f64; 2]` to match, or a small `Point2` is a
reasonable first citizen if drawings warrant it.

### 2.3 Hidden-line removal (the hard part)

An edge segment is *hidden* where the body lies between it and the viewer.

**Mesh path (MVP) — sampled occlusion against a BVH.** `opensolid-core` already
has a BVH with `ray_intersect` (`crates/opensolid-core/src/bvh.rs`) over the
triangle mesh. For each projected edge segment, sample points along it, cast a
ray *toward the viewer* (along `−view_dir`) from each sample, and mark the
sample visible iff no triangle is hit before the eye (with a small bias so the
edge's own faces don't self-occlude). Contiguous runs of same-visibility
samples become `visible`/`hidden` sub-segments. This is approximate — visibility
transitions land on sample boundaries — but robust and trivially correct: it can
never crash, and refining the sample density trades speed for crispness. This is
the F-Rep philosophy the rest of the kernel already follows (cf.
`edge-fillet-chamfer.md`: robust approximate over fragile exact).

**Exact path (follow-up) — analytic curve visibility.** True HLR intersects
projected curves, sorts by depth, and splits each curve at every crossing and
silhouette-tangency into maximal visible/hidden arcs. This is the precise, hard,
literature-heavy path (quantitative invisibility / Appel's algorithm). Deferred;
the mesh path covers the MVP and every view a user will eyeball.

## 3. Sheet model

A sheet is a coordinate frame (paper size + units), a scale, a title block, and
an ordered list of placed views:

```
Sheet   { size, scale, title_block, views: Vec<PlacedView> }
PlacedView { view_dir, origin_on_sheet, edges: Vec<Segment2D{ pts, style }> }
```

Standard first/third-angle layout places front/top/right at shared origins so
they align (top above front, right beside front) — pure arithmetic on
`origin_on_sheet` once each view's 2D bounds are known. Scale is one multiplier
applied at placement (`place` step, §1). The title block is a static template
filled from document metadata (part name, scale, date, author). None of this
needs kernel support; it is a JS data model in the playground, parallel to how
the sketch model (`web/playground/src/lib/sketch/model.js`) is pure JS over a
baked kernel result.

## 4. Dimension annotations

Dimensions must be **associative**: edit the model, the dimension value updates
(or the dimension flags itself dangling). This reuses two existing pieces.

**Dimension machinery.** The sketch layer already has dimensional constraints —
`length` and `radius` with numeric values, solved in `web/playground/src/lib/
sketch/solver.js`, defined in `.../sketch/model.js`. A drawing dimension is
simpler than a sketch dimension: it is *driven* (it measures, it does not
constrain), so it needs the residual evaluators' geometry (point-to-point
distance, radius, angle) but not the solver. of-fsl.12 (open) is adding
diameter/angle/point-to-point/driven-dim types to the sketch layer; a drawing
dimension is the driven, read-only sibling of those and should share the
value-formatting and arrow/witness-line rendering.

**Association to model geometry.** A drawing dimension anchors to *model* edges,
not sketch entities, so it needs the persistent-naming machinery from of-fsl.8:
`web/playground/src/lib/persistentRef.js` stores a geometric reference (anchor +
orientation, `faceRefFromPlane` / `resolveFaceRef`) that re-resolves against the
rebuilt mesh by nearest-point, flagging dangling refs — the same scheme
documented in `docs/parametric-rebuild.md`. Drawing dims need the **edge/vertex**
analogue of that face reference: store `{ view, seed_point_on_edge }`, re-resolve
after rebuild by finding the nearest feature edge in the regenerated view (the
mesh path) or by `CurveProject` onto the analytic edge (the exact path). This is
the identical "name by geometric anchor, re-resolve by proximity" strategy the
fillet edge-selection (`edge-fillet-chamfer.md` §2) and the face refs already
use — drawings are a third consumer of one persistent-naming system, which is
the right amount of reuse.

## 5. Section & detail views

**Section view.** Note that of-fsl.18 ("Section view") shipped a *display-only*
three.js viewport clip (`clippingPlanes` + capped shading) — it reveals
internals on screen but produces **no 2D geometry**. A drawing section is
different: it needs the actual cross-section curve where the cutting plane meets
the solid, drawn as a bounded region with hatching. The raw material exists:
`crates/opensolid-brep/src/ssi/` (surface–surface intersection: `analytic.rs`
handles the plane-through-axis → circle cases, `marching.rs` produces polyline
intersection curves as `Curve3::Polyline`) and the boolean imprint pipeline
(`crates/opensolid-brep/src/boolean.rs`, ~line 699) already yield ordered 3D
section polylines. The F-Rep analogue is trivial: intersect with a half-space
(`WasmShape::half_space`, `crates/opensolid-wasm/src/lib.rs` ~line 300) and read
the new boundary loop off the mesher. Section geometry is therefore "run the cut,
take the loop, project it into the sheet, hatch the interior" — reusing the same
projection pipeline (§1) with a known-planar loop (no HLR needed for the cut
face itself). Hatching is a 2D scanline fill of the closed section polygon.

**Detail view.** A zoomed, clipped copy of a region of another view at a larger
scale — pure 2D: clip the parent view's segments to a circle/rectangle and
re-place at the detail scale. No new kernel work.

## 6. Export

No 2D vector export exists in the repo today (existing exporters — STEP
`crates/opensolid-kernel/src/io/step/`, STL/OBJ `io/stl.rs`/`io/obj.rs` — are all
3D). Everything a drawing exports is already 2D polylines + text, so:

- **SVG (MVP).** Trivial and lossless from `Segment2D` lists: `<path>` per
  polyline, `stroke-dasharray` for hidden lines, `<text>` + `<line>` for
  dimensions, one `<g>` per view. Pure string building; no dependency. Ships
  first. Can live JS-side in the playground (the natural home, next to the sheet
  model) or as a Rust exporter if headless/MCP drawing export is wanted — the
  MVP puts it JS-side to match where the sheet model lives.
- **PDF (follow-up).** SVG → PDF via `svg2pdf` (or a print-to-PDF path in the
  browser for the GUI). The drawing is already SVG, so this is a format shim.
- **DXF (follow-up).** For CAD interop. `LINE`/`ARC`/`LWPOLYLINE`/`TEXT`
  entities map directly from the same `Segment2D`/dimension data. Note the DXF
  bulge-arc convention is already used internally by `Profile2D`
  (`crates/opensolid-frep/src/profile.rs`) so arc export has a precedent to
  follow.

## 7. GUI — drawing mode

A drawing is a distinct document *view*, so it slots into `App.jsx`
(`web/playground/src/App.jsx`) as a mode parallel to `sketchOpen` — call it
`drawingOpen` — with its own full-canvas overlay component modeled on
`SketchCanvas.jsx` and its 2D pan/zoom (`web/playground/src/lib/sketchView.js`,
`sketchWorldToScreen` etc.). The flow: user picks standard views from a palette
(reusing `VIEW_DIRECTIONS`), views drop onto the sheet, user clicks model edges
to add driven dimensions, exports SVG. The drawing is persisted as data in the
document (the sheet model of §3), not as script — a drawing is an *output view*
of the model, not a modeling operation, so it does not belong in the modeling
script the way sketches/extrudes do. (A separate-document model is the eventual
SolidWorks-parity answer; the MVP embeds one sheet in the current session.)

## 8. MVP scope

Per the bead's stated MVP: **3 standard views (front/top/right) + iso, visible
edges only, manual dimensions, SVG export.** Concretely that is:

1. Surface `feature_edges` from the mesher through `MeshData` (§2.1).
2. Per-view: project feature edges + compute mesh silhouette, **visible edges
   only** (skip HLR entirely for v1 — draw every projected edge solid; add the
   BVH occlusion pass in the next phase).
3. A JS sheet model + drawing-mode overlay: place the four views, pan/zoom.
4. Manual point-to-point / radius dimensions anchored by seed point (associative
   re-resolve deferred to the persistent-ref phase — v1 dims are static).
5. SVG export.

Deferring HLR and associativity for v1 is deliberate: it makes the first drawing
land as pure projection + serialization (no occlusion, no rebuild plumbing),
proving the sheet/view/export loop end to end, then hidden lines and
associativity layer on without reworking it.

## 9. Out of scope / follow-ups

- **Exact-B-Rep drawings + exact HLR** (analytic curve visibility / quantitative
  invisibility). The precision path for released engineering drawings. Deferred.
- **Hidden-line removal** beyond the sampled-BVH approximation (MVP v2).
- **Associative dimensions** via edge persistent-refs (needs the of-fsl.8
  edge-ref analogue; MVP dims are static).
- **Section & detail views** (§5) — reuse SSI/boolean section polylines; not in
  the first drawing MVP.
- **GD&T, tolerances, surface finish, weld symbols, BOM tables**, first vs
  third-angle projection toggle, auto-dimensioning, broken/crop views.
- **Multi-sheet, multi-document drawings** and the separate-drawing-document
  model (couples to the of-fsl.25 assembly/document work).
- **PDF/DXF export** (SVG ships first; both are format shims off the SVG data).
