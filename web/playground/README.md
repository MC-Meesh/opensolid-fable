# OpenSolid Playground

Interactive browser playground for the OpenSolid F-Rep kernel: edit a small
JS script that builds a shape with the `opensolid-wasm` API, mesh it in WASM,
orbit it in a three.js viewer, and download the result as binary STL.

React + Vite SPA. The UI is componentized under `src/components/`:

- **App** — owns all state (script, resolution, wireframe, mesh, stats,
  scene tree selection) and the WASM shape lifecycle
- **SceneTree** — collapsible view of the script's construction tree; click
  a node to isolate that intermediate shape in the viewport
- **ScriptEditor** — CodeMirror 6 editor with JS syntax highlighting
- **Viewport3D** — three.js canvas with OrbitControls
- **Toolbar** — Run / Download STL buttons, resolution slider, wireframe toggle
- **StatusBar** — triangle/vertex counts, grid size, mesh time

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

## Using the playground

- **Left pane** — a JS snippet that must `return` a shape. It runs with one
  binding in scope, `Shape` (the `WasmShape` class). Press **Run** or
  Ctrl/Cmd+Enter to re-evaluate and re-mesh. Errors (syntax, thrown
  exceptions, wrong return type) appear below the editor.
- **Scene panel** — every run traces the script's shape operations into a
  construction tree (see `src/lib/sceneTree.js`); the final shape is the
  root. Click a step to isolate that intermediate shape in the viewport
  (it re-meshes just that subtree); click it again, click the root, or use
  the *Show full model* button to go back. A shape reused in several places
  (e.g. built in a loop) is one shared node shown under each parent.
- **Resolution slider** — dual-contouring grid resolution (32–128 cells per
  axis). Re-meshes the current shape without re-running the script.
- **Wireframe** — toggles wireframe rendering.
- **Download STL** — assembles a binary STL in JS from the current mesh
  buffers (facet normals recomputed from geometry) and downloads it.

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
`.translate(x, y, z)`, `.union(s)`, `.intersect(s)`, `.subtract(s)`,
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
shared subtrees into `const` bindings. GUI features (property editing,
gizmos, bidirectional sync) should read and write this model.
