# OpenSolid MCP Server

An [MCP](https://modelcontextprotocol.io) (Model Context Protocol) server that
exposes the OpenSolid CAD kernel as an agentic API surface. Point any
MCP-capable agent (Claude, etc.) at it and the agent becomes a **headless CAD
operator**: it writes a script, gets back mesh stats and a validation report,
renders screenshots, measures mass properties, and exports STEP/STL/OBJ — no
GUI, no browser.

The kernel it drives is the *same* WebAssembly build the browser
[playground](../../web/playground) runs, so a script an agent writes here
produces the identical shape in the GUI, and vice-versa.

## Tools

| Tool             | Purpose |
|------------------|---------|
| `create_model`   | Build a model from a playground JS script → `model_id` + mesh stats + validation summary. |
| `get_screenshot` | Render a model to a PNG from a named view (`iso`, `front`, `top`, …). |
| `export`         | Write a model to a file: `step` \| `stl` \| `obj`. |
| `measure`        | Mass properties: volume, surface area, centroid, inertia, bounding box. |
| `validate`       | Structural check: is the mesh a closed, consistently-oriented manifold enclosing a finite non-zero volume? |
| `list_models`    | List the models registered this session. |

Every tool except `create_model` and `list_models` takes a `model_id` returned
by an earlier `create_model` call. Models live for the lifetime of the server
process (in memory, no persistence).

## The script format

`create_model` takes a `script`: a **JavaScript function body** (not a module)
that must `return` a `Shape`. It runs in strict mode with exactly two bindings
in scope — `Shape` and `Profile` — identical to the playground's **Code** tab
(see [`runScript.js`](../../web/playground/src/lib/runScript.js)). No imports,
no `require`, no filesystem or network.

```js
// The classic "bolt boss": a cylinder with a bolt hole through it.
const boss = Shape.cylinder(8, 10);              // radius 8, half-height 10
const hole = Shape.cylinder(3, 12);              // radius 3, taller so it cuts clean
return boss.subtract(hole);
```

### `Shape` — primitives

All dimensions are model units. Box/cylinder/torus arguments are **half-extents
/ half-heights** (the shape is centered on the origin).

| Call | Shape |
|------|-------|
| `Shape.sphere(r)` | sphere, radius `r` |
| `Shape.box3(hx, hy, hz)` | box, half-extents `hx,hy,hz` (full size `2hx × 2hy × 2hz`) |
| `Shape.roundedBox(hx, hy, hz, r)` | box with fillet radius `r` |
| `Shape.cylinder(r, hh)` | cylinder, radius `r`, half-height `hh` (full height `2·hh`), axis +Z |
| `Shape.torus(major, minor)` | torus in the XY plane |
| `Shape.capsule(x1,y1,z1, x2,y2,z2, r)` | capsule (swept sphere) between two points |
| `Shape.extrude(profile, height)` | extrude a `Profile` along +Z |
| `Shape.revolve(profile, angleDeg)` | revolve a `Profile` about the Y axis |

### `Shape` — transforms (return a new shape; never mutate)

| Call | Effect |
|------|--------|
| `s.translate(x, y, z)` | translate |
| `s.rotate(ax, ay, az, angleDeg)` | rotate `angleDeg` about axis `(ax,ay,az)` |
| `s.scale(sx, sy, sz)` | non-uniform scale |
| `s.uniformScale(f)` | uniform scale |

### `Shape` — booleans

| Call | Effect |
|------|--------|
| `a.union(b)` | union |
| `a.subtract(b)` | `a` minus `b` |
| `a.intersect(b)` | intersection |
| `a.smoothUnion(b, r)` | union with a smooth blend of radius `r` (organic fillet) |

### `Profile` — 2D profiles for extrude / revolve

A `Profile` is a closed polyline with optional circular-arc segments (`bulge` is
the tangent of a quarter of the arc's swept angle; `0` = straight).

```js
// An L-bracket profile, extruded 20mm thick.
const p = new Profile(0, 0);   // start at the origin
p.lineTo(40, 0);
p.lineTo(40, 10);
p.lineTo(10, 10);
p.lineTo(10, 40);
p.lineTo(0, 40);
p.close();
return Shape.extrude(p, 20);
```

| Call | Effect |
|------|--------|
| `new Profile(x, y)` | start a profile at `(x, y)` |
| `p.lineTo(x, y)` | straight segment to `(x, y)` |
| `p.arcTo(x, y, bulge)` | circular arc to `(x, y)` (`bulge` = tan(θ/4)) |
| `p.close()` | close the loop back to the start |

### Exact vs. SDF booleans

By default the kernel meshes via its signed-distance-field (SDF) path — robust,
organic, but edges are tessellated. Pass `exact: true` to `create_model` to
route sharp booleans through the **exact B-Rep pipeline**: crisp edges and an
analytic STEP export, for shapes inside the kernel's exact coverage
(sphere/box/cylinder/torus, rigid transforms, uniform scale, sharp booleans).
Anything outside that coverage falls back to the SDF path automatically.

## Setup

```bash
cd tools/mcp-server
npm run build      # compiles crates/opensolid-wasm to ./pkg via wasm-pack
npm test           # runs the unit + end-to-end tests
```

`npm run build` needs [`wasm-pack`](https://rustwasm.github.io/wasm-pack/)
(`cargo install wasm-pack`) and the wasm target
(`rustup target add wasm32-unknown-unknown`). `pkg/` is build output — rerun the
build after any change under `crates/`.

### Registering with an MCP client

The server speaks the MCP **stdio** transport. Point your client at
`src/server.js`:

```jsonc
// e.g. an MCP client config
{
  "mcpServers": {
    "opensolid": {
      "command": "node",
      "args": ["/absolute/path/to/tools/mcp-server/src/server.js"],
      "env": {
        // where export/screenshot files land (default: $TMPDIR/opensolid-mcp)
        "OPENSOLID_MCP_OUTPUT_DIR": "/absolute/path/to/output"
      }
    }
  }
}
```

For Claude Code:

```bash
claude mcp add opensolid -- node /absolute/path/to/tools/mcp-server/src/server.js
```

## Examples

[`examples/agent-gallery/`](examples/agent-gallery/) is a gallery of **five**
worked agent transcripts — a mounting bracket, a hinge leaf, a shelled enclosure
with a press-fit lid, a toothed disk built from a circular pattern, and a
revolved-and-shelled bottle. Each is real, unedited output from this server,
captured by [`build-gallery.mjs`](examples/agent-gallery/build-gallery.mjs): the
agent writes a script, gets mesh stats and a validity flag, renders screenshots,
measures mass properties, and exports STEP/STL/OBJ. They show the intended loop
— *script → validate → measure → adjust → export* — and one genuine export
limitation and how the tool reports it.

Regenerate the whole gallery (renders, exports, and transcripts):

```bash
node examples/agent-gallery/build-gallery.mjs
```

Exported files and renders land in [`examples/output/`](examples/output/). For
connecting a client, the full tool reference, the script API, and the failure
modes these examples exercise, see the
[Agent Guide](../../docs/AGENT_GUIDE.md).
