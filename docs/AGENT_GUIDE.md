# Agent Guide — OpenSolid as an AI-first CAD kernel

OpenSolid ships an [MCP](https://modelcontextprotocol.io) server that exposes the
CAD kernel as a small set of tools. Point any MCP-capable agent at it and the
agent becomes a **headless CAD operator**: it writes a script, gets back mesh
statistics and a validity report, renders screenshots, measures mass properties,
and exports STEP/STL/OBJ — no GUI, no browser, no human in the loop.

The kernel the agent drives is the *same* WebAssembly build the browser
[playground](../web/playground) runs, so a script an agent writes here produces
the identical shape in the GUI, and vice-versa. This guide covers connecting a
client, the tool reference, the script API, and the failure modes an agent will
actually hit — and exactly how each one is reported.

See it in action first: the [agent gallery](../tools/mcp-server/examples/agent-gallery/)
has five end-to-end transcripts (bracket, hinge, enclosure, gear, bottle), each
real unedited output from the server.

---

## 1. Connecting a client

### Prerequisites

The server is a Node process that loads a prebuilt wasm bundle. Build it once:

```bash
cd tools/mcp-server
npm install        # no runtime deps today, but keeps the lockfile honest
npm run build      # compiles crates/opensolid-wasm → ./pkg via wasm-pack
npm test           # optional: unit + end-to-end tests
```

`npm run build` needs [`wasm-pack`](https://rustwasm.github.io/wasm-pack/)
(`cargo install wasm-pack`) and the wasm target
(`rustup target add wasm32-unknown-unknown`). `pkg/` is generated build output —
rerun the build after any change under `crates/`.

### Claude Code

```bash
claude mcp add opensolid -- node /absolute/path/to/tools/mcp-server/src/server.js
```

Then, in a session, ask Claude to build something ("design a 60×40×8 bracket with
two mounting holes and give me the STEP file") — it will discover and call the
tools below.

### Any MCP client (stdio)

The server speaks the MCP **stdio** transport. Register `src/server.js`:

```jsonc
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

Models live in memory for the lifetime of the server process — there is no
persistence. Exports and screenshots are written to `OPENSOLID_MCP_OUTPUT_DIR`.

---

## 2. Tool reference

| Tool             | Input (required **bold**)                          | Returns |
|------------------|----------------------------------------------------|---------|
| `create_model`   | **`script`**, `name`, `exact`                      | `model_id` + mesh stats + validation summary |
| `get_screenshot` | **`model_id`**, `view`, `width`, `height`          | inline PNG image |
| `export`         | **`model_id`**, **`format`**, `path`, `accuracy`   | file path + byte size |
| `measure`        | **`model_id`**, `query`, `accuracy`                | mass properties |
| `validate`       | **`model_id`**, `accuracy`                          | structural report |
| `list_models`    | —                                                  | models registered this session |

Every tool except `create_model` and `list_models` takes a `model_id` handed
back by an earlier `create_model` call.

### `create_model`

Builds a model from a script (see §3) and registers it under a `model_id`. The
response is the agent's first oracle — it arrives without rendering anything:

```json
{
  "model_id": "model-1-8f3a",
  "name": "angle-bracket",
  "exact": false,
  "mesh": { "triangles": 21820, "vertices": 10908 },
  "boundingBox": { "min": [...], "max": [...], "size": [60, 40, 40] },
  "volume": 31764.39,
  "valid": true,
  "issues": []
}
```

- `boundingBox` is measured off the same mesh the mass properties integrate, so
  it is the part's real extent to within the meshing accuracy (~0.5% of the
  extent by default; pass a finer `accuracy` to tighten it). It is *not* the
  kernel's internal tracked bounds, which are a conservative enclosure and can
  overstate a blended or repeatedly-rotated part badly. It is `null` only when
  the mesh has no triangles.
- `exact: true` routes sharp booleans through the exact B-Rep pipeline (crisp
  edges, analytic STEP) for shapes inside the kernel's exact coverage
  (sphere/box/cylinder/torus, rigid transforms, uniform scale, sharp booleans).
  Anything outside it falls back to the SDF path automatically. Default `false`.
- `valid` / `issues` are the same check `validate` runs — a failed boolean shows
  up here immediately, not as a silently-wrong mesh downstream.

### `get_screenshot`

Renders a model to a PNG from a named view and returns the image inline (no file
written). Views: `iso` (default), `front`, `back`, `right`, `left`, `top`,
`bottom`. `width`/`height` default to 800×600. The renderer is a pure-JS
software rasterizer — a screenshot is a few milliseconds, no GPU, no headless
browser.

### `export`

Writes a model to a file. `format` is `step`, `stl`, or `obj`. `path` is
optional (absolute, or relative to the output dir; defaults to
`<name>.<format>`). Returns `{ model_id, format, path, bytes }`.

- **STEP** serializes analytic surfaces when the shape carries an exact B-Rep
  companion, otherwise a faceted-but-valid B-Rep via SDF→B-Rep planar-region
  recovery. See §4 for when the faceted path declines.
- **STL / OBJ** write the current mesh.

`accuracy` sets the target chordal deviation of the exported facets in model
units, defaulting to 0.5% of the model's extent. It is the file-size lever —
pass a coarser value when the export only needs to be eyeballed. The exact-B-Rep
STEP path ignores it; analytic surfaces have no tessellation error.

The lever saturates. Meshing depth is `ceil(log2(extent / accuracy))` clamped to
a minimum of 4, so any accuracy coarser than about `extent / 16` produces the
same file, and the useful range spans roughly 4× in size rather than orders of
magnitude. On a 1.3-unit organic solid: 5.8 MB at `accuracy: 0.002`, 3.0 MB at
the default, 765 KB at `0.2` — and `0.5` is byte-identical to `0.2`.

Accuracy also changes the meshing depth and grid, so it is worth trying when the
faceted STEP path declines (§4) — though it is a coarse instrument, not a
guaranteed fix.

### `measure`

Mass properties computed as exact polyhedral integrals over the mesh. `query`
narrows the result: `all` (default), `volume`, `surface_area`, `bbox`,
`centroid`, `mass` (volume + area + centroid + inertia). `accuracy` sets the
target chordal deviation of the measured mesh in model units.

Volume and centroid are the agent's cheapest correctness oracles — a volume
*delta* confirms a cut actually removed material; a centroid confirms a feature
landed where intended.

### `validate`

Checks whether the mesh is a closed, consistently-oriented manifold enclosing a
finite non-zero volume:

```json
{
  "valid": true,
  "closedManifold": true,
  "triangles": 16752,
  "vertices": 8376,
  "volume": 7008.29,
  "exact": false,
  "issues": []
}
```

Call it before trusting a boolean result. A model that looks right in a
screenshot but isn't watertight will fail here with named `issues`.

### `list_models`

Returns `{ models: [{ model_id, name, exact, createdAt }] }` for everything
registered this session.

---

## 3. Script API crash course

`create_model` takes a **JavaScript function body** (not a module) that must
`return` a `Shape`. It runs in strict mode with exactly two bindings in scope —
`Shape` and `Profile` — identical to the playground's **Code** tab. No imports,
no `require`, no filesystem or network. Because it's real JavaScript, patterns
(loops, arrays, math) are just code:

```js
// A bolt boss with a hole, then four holes on a rectangular pattern.
const boss = Shape.cylinder(8, 10);              // radius 8, half-height 10, axis +Y
let part = boss.subtract(Shape.cylinder(3, 12)); // central hole, taller so it cuts clean
const bolt = Shape.cylinder(1.5, 12);
for (const x of [-5, 5]) for (const z of [-5, 5]) {
  part = part.subtract(bolt.translate(x, 0, z));  // pattern in xz: bolts run parallel to the boss
}
return part;
```

Dimensions are model units. Box/cylinder/torus arguments are **half-extents /
half-heights** — the shape is centered on the origin.

> ### ⚠️ The axis convention: `cylinder` and `extrude` are **+Y**
>
> This is the single most common way to ship a wrong part, so read it once and
> remember it:
>
> - **`Shape.cylinder(r, hh)`** is radial in **xz**, axial in **y** — a **+Y** cylinder.
> - **`Shape.extrude(profile, height)`** maps the profile's `(u, v)` to **`(x, z)`**
>   and sweeps along **+Y**, from `y = 0` to `y = height`.
> - **`Shape.revolve`** turns about **Y**; **`Shape.torus`** rings in **xz**. Same convention.
> - The renderer agrees: `y` is up in model space, so the named views (`top`, `front`, …)
>   are relative to a **y-up** part.
>
> This cuts against STEP/FreeCAD, which are **z-up** — and the STEP writer emits
> coordinates verbatim. So a part you model "flat on the xy plane with thickness
> in z" (the CAD-interchange habit) needs its holes **rotated onto +Z**:
>
> ```js
> const through = Shape.cylinder(2.5, 10).rotate(1, 0, 0, 90);  // +Y -> +Z
> const sideways = Shape.cylinder(2.5, 10).rotate(0, 0, 1, 90);  // +Y -> +X
> ```
>
> **Rotating about the axis a shape is already on is a no-op.** `cylinder(...).rotate(0, 1, 0, 90)`
> looks like it aims the cylinder somewhere but does nothing at all.
>
> **This failure is silent.** A hole bored on the wrong axis still reports
> `valid: true`, still renders plausibly, and still exports. Neither a screenshot
> nor `validate` will catch it — **only `measure` checked against a volume you
> computed by hand will.** Do that for any part that matters.

**`Shape` — primitives**

| Call | Shape |
|------|-------|
| `Shape.sphere(r)` | sphere, radius `r` |
| `Shape.box3(hx, hy, hz)` | box, half-extents (full size `2hx × 2hy × 2hz`) |
| `Shape.roundedBox(hx, hy, hz, r)` | box with fillet radius `r` |
| `Shape.cylinder(r, hh)` | cylinder, radius `r`, half-height `hh`, axis **+Y** |
| `Shape.torus(major, minor)` | torus with its ring in the **XZ** plane |
| `Shape.capsule(x1,y1,z1, x2,y2,z2, r)` | capsule between two points |
| `Shape.extrude(profile, height)` | extrude a `Profile` along **+Y**, from `y=0` to `y=height` |
| `Shape.revolve(profile, angleDeg)` | revolve a `Profile` about the Y axis |

**`Shape` — transforms** (return a new shape; never mutate)

| Call | Effect |
|------|--------|
| `s.translate(x, y, z)` | translate |
| `s.rotate(ax, ay, az, angleDeg)` | rotate `angleDeg` about axis `(ax,ay,az)` |
| `s.scale(sx, sy, sz)` / `s.uniformScale(f)` | scale |

**`Shape` — booleans**

| Call | Effect |
|------|--------|
| `a.union(b)` / `a.subtract(b)` / `a.intersect(b)` | CSG |
| `a.smoothUnion(b, r)` | union with a smooth blend of radius `r` (organic fillet) |

**`Profile` — 2D profiles for extrude / revolve**

A closed polyline with optional circular-arc segments. `bulge` is
`tan(θ/4)` for the arc's swept angle (`0` = straight):

```js
const p = new Profile(0, 0);   // start at the origin
p.lineTo(40, 0);
p.lineTo(40, 10);
p.arcTo(10, 40, 0.4);          // arc segment
p.lineTo(0, 40);
p.close();
return Shape.extrude(p, 20);
```

Full reference (with the exact-vs-SDF discussion) lives in the
[server README](../tools/mcp-server/README.md#the-script-format).

---

## 4. Failure modes and how they're reported

Every tool returns an MCP content result. **Errors set `isError: true`** and put
a human-readable message in the text content; they never throw across the wire.
An agent should branch on `isError` and read the message — each one names a
specific, actionable cause.

### Script errors — caught at `create_model`

| Situation | Reported as |
|-----------|-------------|
| Syntax error in the script | `Error: script failed: script has a syntax error: <detail>` |
| Script doesn't `return` a Shape | `Error: script failed: script must return a Shape, e.g. end with:\n  return solid;` |
| Runtime error in the script | `Error: script failed: <the thrown message>` |

Fix the script and call `create_model` again — nothing is registered on failure.

### Degenerate geometry — caught by `valid` / `validate`

A boolean that produces something that *isn't* a solid does **not** error. The
model registers, but `create_model`'s `valid` flag is `false` and `issues` names
the problem — the same report `validate` returns. For example, intersecting two
boxes that don't overlap yields an empty mesh:

```json
{
  "valid": false,
  "issues": [
    "mesh is empty",
    "mesh is not a closed, consistently oriented manifold"
  ]
}
```

This is the single most important habit for an agent: **check `valid` (or call
`validate`) before exporting.** A screenshot can look plausible while the mesh is
open; the validity report cannot be fooled. Trying to screenshot an empty model
is itself a clean error — `Error: model produced an empty mesh; nothing to
render`.

### Export limitations — reported by `export`

STL and OBJ export any mesh the model produced. **STEP** is stricter: when the
shape has no exact B-Rep companion, STEP goes through the faceted SDF→B-Rep path,
which needs the surface to lie strictly *inside* the meshing region. Thin
features that sit right at the model's bounding box can fail to close, and the
tool **declines rather than emitting a broken file**:

```json
{
  "isError": true,
  "text": "Error: export failed: STEP export failed: degenerate geometry in sdf_to_brep: adaptive meshing did not produce a closed manifold; the surface must lie strictly inside the meshing bounds"
}
```

STL is unaffected when this happens — meshing and STEP's planar-region recovery
are different code paths. To get an analytic STEP of such a part, thicken the
feature slightly or model it as an extruded `Profile` so it carries an exact
B-Rep.

The same root cause bites *before* export, too. Meshing accuracy is derived from
the model's overall bounding box, so a small feature inside a large part gets
proportionally less resolution and can fail to close on its own — `create_model`
returns `valid: false` with `mesh is not a closed, consistently oriented
manifold` and a `null` volume. A Ø3.2 bore that meshes cleanly on a single
knuckle will fail once that knuckle is one of three spread across a 62 mm leaf.
Widen the feature, or model the part smaller and scale it up.

Other export errors:

| Situation | Reported as |
|-----------|-------------|
| Unknown `model_id` | `Error: unknown model_id: <id>` |
| Unsupported `format` | `Error: unsupported format '<x>'; use one of step, stl, obj` |

### The recommended loop

1. **`create_model`** → read `valid` and `volume`. If `valid: false`, fix the
   script before doing anything else.
2. **`validate`** (or trust `create_model`'s summary) to confirm a closed
   manifold after a nontrivial boolean.
3. **`measure`** to check intent — a volume delta proves a cut removed material;
   a centroid proves a feature is where you meant it.
4. **`get_screenshot`** for a human-readable gut check from any named view.
5. **`export`** to STEP/STL/OBJ, branching on `isError` for the STEP faceting
   limitation above.

The five gallery transcripts each walk this loop on a real part. Start there.
