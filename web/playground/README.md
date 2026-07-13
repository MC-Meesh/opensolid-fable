# OpenSolid Playground

Interactive browser playground for the OpenSolid F-Rep kernel: edit a small
JS script that builds a shape with the `opensolid-wasm` API, mesh it in WASM,
orbit it in a three.js viewer, and download the result as binary STL.

React + Vite SPA. The UI is componentized under `src/components/`:

- **App** — owns all state (script, accuracy, wireframe, mesh, stats,
  feature selection) and the WASM shape lifecycle. The chrome is one
  window-height, no document scrolling: a resizable tabbed side panel
  (**Code | Tree**) plus the viewport, separated by a draggable splitter.
- **FeatureTree** — CAD feature history in the side panel's Tree tab
  (SolidWorks FeatureManager style): chronological features (Box1, Sketch1,
  Extrude1, Union1 …) derived from the construction tree, with renameable
  rows, an eye visibility toggle, suppress and delete actions. Clicking a
  sketch feature re-enters sketch mode on it; clicking any other feature
  isolates it and opens its parameters. The tab also hosts the shape palette
  (add primitives).
- **ScriptEditor** — CodeMirror 6 editor with JS syntax highlighting (the
  Code tab)
- **Viewport3D** — three.js canvas with OrbitControls and the orientation triad
- **MainToolbar** — single-row workflow-grouped toolbar over the viewport
  (Sketch | Features | View | Export) with an overflow menu for the meshing
  settings
- **MeshSettings** — exact-booleans toggle (in the toolbar's overflow menu);
  meshing accuracy is fixed at a high-precision default
- **StatusBar** — triangle/vertex counts, accuracy, mesh time

The only non-npm generated piece is `pkg/`, the wasm-bindgen output for the
`opensolid-wasm` crate.

## Run it

Prerequisites: Node 20+, Rust with the wasm target (`rustup target add
wasm32-unknown-unknown`) and [wasm-pack](https://rustwasm.github.io/wasm-pack/)
(`cargo install wasm-pack`).

From `web/playground/`:

```sh
npm install
npm run wasm    # builds crates/opensolid-wasm into pkg/ (rerun after Rust changes)
npm run dev     # Vite dev server at http://localhost:5173
```

`npm run wasm` is a required setup step: `pkg/` is generated build output
(written into the playground tree, not symlinked) and is not checked in.
`npm run dev` warns loudly if it's missing (and `npm run build` refuses to
run); the app itself shows an error screen with these instructions instead
of loading. If you generate `pkg/` while the dev server is already running,
restart the dev server.

### WASM init lifecycle

WASM initialization is single-flight and owned by `src/wasm/loader.js`
(surfaced to React through `src/wasm/WasmContext.jsx`). It has three states:
**loading** (overlay), **ready**, or **failed** — failure shows a
full-viewport error screen with the reason (failing URL, HTTP status when
one exists), the `npm run wasm` fix, and a Retry button. Init that hangs
times out after 10 s with a diagnostic; there is never an infinite spinner.

The main panels (3D viewport, sketch canvas, script editor) are each wrapped
in a React error boundary, so a crash in one degrades to an inline error
card with a reset button instead of blanking the app.

Production build (outputs a static site to `web/playground/dist/`):

```sh
npm run build
npm run preview   # serve dist/ locally to check it
```

Tests (vitest, covers the pure JS modules in `src/lib/`):

```sh
npm test
```

### Without wasm-pack

The same `pkg/` can be produced with cargo plus the wasm-bindgen CLI. The CLI
version must match the `wasm-bindgen` crate version cargo resolved (check
with `cargo tree -p opensolid-wasm | grep wasm-bindgen`). From the repo root:

```sh
cargo install wasm-bindgen-cli --version <that version>
cargo build --release --target wasm32-unknown-unknown -p opensolid-wasm
wasm-bindgen --target web --no-typescript --out-dir web/playground/pkg \
    target/wasm32-unknown-unknown/release/opensolid_wasm.wasm
```

## Viewport conventions (SolidWorks-style)

**World convention: Y is up.** The ground grid lies in the XZ plane, the
front view looks along −Z, and every primitive's "up" is +Y (matching
three.js and the Shape API). Standard view directions are defined in
`src/lib/views.js`.

Mouse (SolidWorks mapping):

- **Middle-drag** rotates; **Shift+middle-drag** pans; **scroll** zooms
  toward the cursor. Left-drag also rotates, right-drag also pans.
- **Hover** shows a faint ghost of the body under the cursor; **click**
  selects it (accent ghost + gizmo + property panel); **click empty space**
  deselects.

Keyboard (outside sketch mode):

| Key | Action |
|---|---|
| `F` or `Space` | Zoom to fit |
| `1`–`7` | Front / Back / Left / Right / Top / Bottom / Isometric |
| `T` / `R` / `S` | Translate / rotate / scale gizmo |
| `Delete` | Delete the selected body |
| `Esc` | Deselect (or cancel the pending sweep) |

The **orientation triad** (bottom-left) tracks the camera; click an axis tip
to snap to the view looking down that axis (hollow tip = negative direction).

## Using the playground

- **Code tab (side panel)** — a JS snippet that must `return` a shape. It
  runs with one binding in scope, `Shape` (the `WasmShape` class). Press
  **Run** (side panel header) or Ctrl/Cmd+Enter to re-evaluate and re-mesh.
  Errors (syntax, thrown exceptions, wrong return type) appear at the bottom
  of the side panel. Drag the splitter to resize the panel.
- **Tree tab (side panel)** — every run traces the script's shape
  operations into a construction tree (see `src/lib/sceneTree.js`), and
  `src/lib/featureTree.js` presents it as a chronological feature history.
  Click a feature to isolate it and edit its parameters in the property
  panel; click a sketch feature to re-open its profile in the sketch canvas
  (apply replaces just that profile). The eye toggle and *suppress* recompute
  the displayed mesh with that feature bypassed — the script itself is never
  modified; *delete* rewrites the script without the feature. Double-click a
  name to rename it (display-only). A shape reused in several places (e.g.
  built in a loop) appears once, at its creation position. The palette at
  the top adds primitives as new script statements.
- **Exact booleans** (toolbar overflow menu) — routes sharp booleans through
  the kernel's exact B-Rep pipeline. Meshing accuracy is fixed at a
  high-precision default (no user knob); scripts needing another target can
  call the wasm `meshAdaptive(accuracy)` API directly.
- **Wireframe** — toggles wireframe rendering (View group).
- **Export STL / STEP** (toolbar Export group) — STL assembles a binary STL
  in JS from the current mesh buffers; STEP serializes the shape itself
  (analytic surfaces for exact B-Rep chains, faceted otherwise).

### Shape API

Constructors (all centered at the origin; y is up):

| Call | Shape |
|---|---|
| `Shape.sphere(r)` | sphere |
| `Shape.box3(hx, hy, hz)` | axis-aligned box, half-extents |
| `Shape.roundedBox(hx, hy, hz, r)` | box with edge radius `r` |
| `Shape.cylinder(r, halfHeight)` | cylinder along y |
| `Shape.torus(major, minor)` | ring in the xz plane |
| `Shape.capsule(x1,y1,z1, x2,y2,z2, r)` | sphere-swept segment |

Operations (immutable — each returns a new shape, operands stay usable):
`.translate(x, y, z)`, `.rotate(ax, ay, az, angleRad)` (about the axis
through the origin), `.scale(sx, sy, sz)` / `.uniformScale(f)` (factors
`> 0`, about the origin), `.union(s)`, `.intersect(s)`, `.subtract(s)`,
`.smoothUnion(s, radius?)` (default radius: 10% of the combined bounding
box's largest extent).

Meshing bounds are derived automatically from the shape's tracked bounding
box. Every shape a run creates is retained by its scene-tree node (that is
what makes click-to-isolate instant) and freed on the next successful run.

### The scene-tree model

`src/lib/sceneTree.js` is the shared model intended as the single source of
truth for script ↔ GUI sync: `runTracedScript()` executes the script with a
tracing `Shape` wrapper that records every operation as a node (so loops,
variables and helper functions all work — no static parsing), and
`serializeTree()` emits a canonical script back from any tree, hoisting
shared subtrees and every boolean step into `const` bindings so the emitted
program stays one readable statement per feature. GUI features (property
editing, gizmos, bidirectional sync) read and write this model; when they
regenerate the script they pass the leading comment block (`scriptHeader()`)
back through, so the API-reference header survives every GUI edit.
