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
| `import_step`    | Read an existing STEP file → `model_id` (plus one per solid) + per-solid outcomes + diagnostics + measure/validate summary. |
| `get_screenshot` | Render a model to a PNG: named or arbitrary view, framed on a feature (`region`/`target`/`zoom`), optionally cut by a `section` plane or drawn as hidden-line-removed `edges`. Deterministic — same request, same bytes. |
| `export`         | Write a model to a file: `step` \| `stl` \| `obj`, with the document `unit` declared in the STEP header. |
| `measure`        | Mass properties: volume, surface area, centroid, inertia, bounding box. |
| `validate`       | Structural check: is the mesh a closed, consistently-oriented manifold enclosing a finite non-zero volume — and does the exact B-Rep body pass the kernel's own validation? |
| `inspect_topology` | Structure: planar faces, circular rims, holes with their **axes** and diameters, shell count, genus, plus axis probes. |
| `assert_model`   | Check a model against expected values (volume, bbox, genus, hole count *and axis*, clearance, …) and report pass/fail per expectation. |
| `diff_models`    | What changed between two models: volume delta, area, bbox, centroid, and the structural counts. |
| `measure_clearance` | Signed distances from probe points to the solid, or interference between two models. |
| `optimize`       | Drive a model's `param()` design variables onto a mass/volume/centroid target under constraints, and write the result back. |
| `get_capabilities` | The machine-readable manifest: every tool's input schema and every script op's signature. |
| `list_models`    | List the models registered this session. |
| `get_model`      | A model's own source: the script it was built from (with its params' current values), or where it was imported from. |

Every tool except `create_model`, `import_step`, `get_capabilities`, and
`list_models` takes a `model_id` returned by an earlier `create_model` or
`import_step` call. Models live for the lifetime of the server process (in
memory, no persistence) — `get_model` is how a design leaves the session as
something reproducible.

### Importing an existing part

`import_step` takes a `path` or the file `text`. Each `MANIFOLD_SOLID_BREP`
comes back as `brep` (exact B-Rep — analytic surfaces, re-exports as analytic
STEP), `mesh` (valid STEP the kernel cannot represent exactly, imported as a
closed tessellation wrapped as an SDF) or `failed`, alongside the reader's
per-entity diagnostics, the repairs it applied, the units the file declared, and
an immediate measure/validate summary of the part. Every solid gets its own
`model_id`; the top-level one is the whole file, with assembly occurrences
placed. See the [agent guide](../../docs/AGENT_GUIDE.md#import_step) for the
full payload.

**Which oracle to reach for.** `validate` and `measure` report scalars over the
whole part, and a part can be badly wrong while every scalar looks right: the
gallery's angle bracket shipped with its four mounting holes bored sideways,
reporting `valid: true`, rendering plausibly, exporting a clean STL, and
measuring only ~4% light. When a feature has a *direction* — a hole, a bore, a
slot — check the direction: `assert_model` with
`{"type": "through_holes", "value": 4, "axis": [0,1,0], "diameter": 5}`, or
`inspect_topology` to see what the axes actually are. See
[the friction log](../../docs/dogfood-bracket-friction-log.md) for the whole
account and the [agent guide](../../docs/AGENT_GUIDE.md#which-oracle-answers-which-question)
for the full table.

## The script format

`create_model` takes a `script`: a **JavaScript function body** (not a module)
that must `return` a `Shape`. It runs in strict mode with five bindings in
scope — `Shape`, `Profile` (closed 2D profiles), `Path` (3D polyline for
`Shape.sweep`), `OpenPath` (open 2D polyline for `Shape.rib`), and `param`
(design variables for `optimize`). The first four match the playground's
**Code** tab (see [`runScript.js`](../../web/playground/src/lib/runScript.js)).
No imports, no `require`, no filesystem or network.

```js
// The classic "bolt boss": a cylinder with a bolt hole through it.
const boss = Shape.cylinder(8, 10);              // radius 8, half-height 10, axis +Y
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
| `Shape.cylinder(r, hh)` | cylinder, radius `r`, half-height `hh` (full height `2·hh`), axis **+Y** |
| `Shape.torus(major, minor)` | torus with its ring in the **XZ** plane |
| `Shape.cone(rBottom, rTop, hh)` | cone or frustum along **+Y** (either radius may be `0`, not both) |
| `Shape.capsule(x1,y1,z1, x2,y2,z2, r)` | capsule (swept sphere) between two points |
| `Shape.halfSpace(px,py,pz, nx,ny,nz)` | the solid half behind a plane — unbounded; intersect it to clip a through-all extrude |
| `Shape.extrude(profile, height, draftDeg?)` | extrude a `Profile` along **+Y**, from `y=0` to `y=height`, with optional mould draft |
| `Shape.revolve(profile, angleDeg)` | revolve a `Profile` about the Y axis |
| `Shape.sweep(profile, path)` | sweep a `Profile` along a 3D `Path` polyline |
| `Shape.loft(bottom, top, height)` | blend two `Profile`s on parallel planes |
| `Shape.rib(openPath, thickness, height, side)` | thicken an `OpenPath` into a support rib swept **+Y** (`side`: `both`/`first`/`second`) |

> **⚠️ The axis convention is +Y — and getting it wrong fails silently.**
>
> `cylinder` is radial in **xz** and axial in **y**; `extrude` maps the profile's
> `(u, v)` to **`(x, z)`** and sweeps **+Y**; `revolve` turns about **Y**; `torus`
> rings in **xz**. The renderer matches (`y` is up in model space, so the named
> views assume a y-up part).
>
> This cuts against STEP/FreeCAD, which are **z-up**, and the STEP writer emits
> coordinates verbatim. Model a plate flat in xy with thickness in z — the CAD
> habit — and its through-holes need `.rotate(1, 0, 0, 90)` to swing `+Y` onto
> `+Z`. Note that rotating a shape about the axis it already lies on is a **no-op**:
> `cylinder(...).rotate(0, 1, 0, 90)` does nothing.
>
> A hole on the wrong axis still reports `valid: true`, still renders plausibly,
> and still exports. Only `measure` against a hand-computed volume catches it —
> see [`test/bracket-acceptance.test.js`](test/bracket-acceptance.test.js).

### `Shape` — transforms (return a new shape; never mutate)

| Call | Effect |
|------|--------|
| `s.translate(x, y, z)` | translate |
| `s.rotate(ax, ay, az, angleDeg)` | rotate `angleDeg` about axis `(ax,ay,az)` |
| `s.scale(sx, sy, sz)` | non-uniform scale |
| `s.uniformScale(f)` | uniform scale |
| `s.taper(px,py,pz, nx,ny,nz, angleDeg)` | mould draft about a parting plane (pull direction first, then the plane point) |

### `Shape` — booleans and blends

| Call | Effect |
|------|--------|
| `a.union(b)` | union |
| `a.subtract(b)` | `a` minus `b` |
| `a.intersect(b)` | intersection |
| `a.smoothUnion(b, r)` | union with a smooth blend of radius `r` along the whole intersection curve |
| `a.filletEdge(b, r, edge)` | union with a rounded blend on **one selected edge** (`edge` = flat `[x0,y0,z0, …]` polyline) |
| `a.chamferEdge(b, setback, edge)` | same, but a planar bevel |

### `Shape` — thin-wall, patterns, and queries

| Call | Effect |
|------|--------|
| `s.shell(thickness)` | hollow to a wall of `thickness`, centred on the surface |
| `s.linearPattern(dx, dy, dz, count)` | `count` copies, copy `k` at `k·(dx,dy,dz)` |
| `s.circularPattern(ax,ay,az, cx,cy,cz, count, angleDeg?)` | `count` copies about an axis (direction, then point) spanning `angleDeg` (default 360) |
| `s.mirror(nx,ny,nz, px,py,pz)` | union with the reflection across a plane (normal, then point) |
| `s.distance(x, y, z)` | signed distance — negative inside |
| `s.normalAt(x, y, z)` | outward unit normal `[nx,ny,nz]` |
| `s.bounds()` | tracked (conservative) bounding box; use `measure` for the real extent |
| `s.isExact()` | whether an exact B-Rep tessellation will be served |

### `Profile` / `Path` / `OpenPath` — the sketch builders

A `Profile` is a **closed** polyline with optional arc, elliptical-arc, and
Bézier segments (`bulge` is the tangent of a quarter of the arc's swept angle;
`0` = straight).

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
| `new Profile(x, y)` | start a closed profile at `(x, y)` |
| `p.lineTo(x, y)` | straight segment to `(x, y)` |
| `p.arcTo(x, y, bulge)` | circular arc to `(x, y)` (`bulge` = tan(θ/4)) |
| `p.ellipseArcTo(x, y, cx, cy, rx, ry, rotationDeg, ccw)` | elliptical arc — centre-parameterised, **not** SVG endpoint form |
| `p.cubicTo(c1x, c1y, c2x, c2y, x, y)` | cubic Bézier |
| `p.close()` | close the loop back to the start (required before extrude/revolve/loft/sweep) |

A `Path` is the 3D polyline `Shape.sweep` follows — `new Path(x, y, z)` then
`path.lineTo(x, y, z)` per vertex (straight segments only). An `OpenPath` is the
open 2D polyline `Shape.rib` thickens — `new OpenPath(x, y)` plus the same
segment methods as `Profile` minus `close()`.

`OpenPath` is bound in the playground's Code tab too, so any script here —
`Shape.rib` included — pastes straight into the browser (`param` calls
excepted).

### Exact vs. SDF booleans

By default the kernel meshes via its signed-distance-field (SDF) path — robust,
organic, but edges are tessellated. Pass `exact: true` to `create_model` to
route sharp booleans through the **exact B-Rep pipeline**: crisp edges and an
analytic STEP export, for shapes inside the kernel's exact coverage
(sphere/box/cylinder/torus, rigid transforms, uniform scale, sharp booleans).
Anything outside that coverage falls back to the SDF path automatically.

## Install

The published package bundles the kernel as prebuilt WebAssembly, so the only
requirement is **Node ≥ 18**. No Rust, no `wasm-pack`, no build step:

```bash
npx opensolid-mcp
```

For Claude Code:

```bash
claude mcp add opensolid -- npx -y opensolid-mcp
```

### Registering with an MCP client

The server speaks the MCP **stdio** transport:

```jsonc
// e.g. an MCP client config
{
  "mcpServers": {
    "opensolid": {
      "command": "npx",
      "args": ["-y", "opensolid-mcp"],
      "env": {
        // where export/screenshot files land (default: $TMPDIR/opensolid-mcp)
        "OPENSOLID_MCP_OUTPUT_DIR": "/absolute/path/to/output"
      }
    }
  }
}
```

## Building from source

Only needed to work *on* the kernel — a change under `crates/` is not in the
published package until it is released.

```bash
cd tools/mcp-server
npm run build      # compiles crates/opensolid-wasm to ./pkg via wasm-pack
npm test           # unit, end-to-end, and distribution tests
```

`npm run build` needs [`wasm-pack`](https://rustwasm.github.io/wasm-pack/)
(`cargo install wasm-pack`) and the wasm target
(`rustup target add wasm32-unknown-unknown`). `pkg/` is build output — rerun the
build after any change under `crates/`. Running the tests against a **stale**
`pkg/` is the classic way to see a wall of unrelated failures; rebuild first.

Point a client at a source checkout with `node /absolute/path/to/tools/mcp-server/src/server.js`
instead of `npx opensolid-mcp`.

### Releasing

`.github/workflows/release-mcp.yml` publishes to npm when a
`opensolid-mcp-v<version>` tag is pushed, after rebuilding the wasm and running
the full test suite. Bump `version` in `package.json`, merge, then tag. The
`prepack` gate refuses to build a tarball that has no wasm kernel in it, and
`test/package.test.js` packs a real tarball, unpacks it, and drives it over
stdio — because a published npm version is immutable and a kernel-less package
installs cleanly and only fails on the user's machine.

## Examples

[`examples/agent-gallery/`](examples/agent-gallery/) is a gallery of **seven**
worked agent transcripts — a mounting bracket, a hinge leaf, a shelled enclosure
with a press-fit lid, a toothed disk built from a circular pattern, a
revolved-and-shelled bottle, a right-angle bracket with a gusset, and an
optimize-to-mass-target run. Each is real, unedited output from this server,
captured by [`build-gallery.mjs`](examples/agent-gallery/build-gallery.mjs): the
agent writes a script, gets mesh stats and a validity flag, renders screenshots,
measures mass properties, and exports STEP/STL/OBJ. They show the intended loop
— *script → validate → measure → adjust → export* — and one genuine export
limitation and how the tool reports it.

Regenerate the whole gallery (renders, exports, and transcripts):

```bash
npm run gallery    # ~4 min; needs a built pkg/
```

Renders land in [`examples/output/`](examples/output/) and are committed; the
STEP/STL/OBJ exports land there too but are **not** committed — they are ~100 MB
of regenerable build output that the STEP writer rewrites wholesale whenever
entity ordering changes. Run the gallery to produce them. For
connecting a client, the full tool reference, the script API, and the failure
modes these examples exercise, see the
[Agent Guide](../../docs/AGENT_GUIDE.md).
