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
has seven end-to-end transcripts (bracket, hinge, enclosure, gear, bottle,
right-angle bracket, gradient optimization), each real unedited output from the
server.

---

## 1. Connecting a client

### Prerequisites

**Node ≥ 18.** That is the whole list. The published `opensolid-mcp` package
carries the CAD kernel with it as prebuilt WebAssembly, so there is no Rust
toolchain to install, no `wasm-pack`, and no build step before an agent can
call a tool.

### Claude Code

```bash
claude mcp add opensolid -- npx -y opensolid-mcp
```

Then, in a session, ask Claude to build something ("design a 60×40×8 bracket with
two mounting holes and give me the STEP file") — it will discover and call the
tools below.

### Any MCP client (stdio)

The server speaks the MCP **stdio** transport:

```jsonc
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

### From a source checkout

Only needed to drive a kernel change that has not been released yet. Build the
wasm first — the tools import the kernel through it, and running against a stale
or absent `pkg/` fails in ways that look like unrelated tool bugs:

```bash
cd tools/mcp-server
npm run build      # compiles crates/opensolid-wasm → ./pkg via wasm-pack
npm test           # unit, end-to-end, and distribution tests
```

`npm run build` needs [`wasm-pack`](https://rustwasm.github.io/wasm-pack/)
(`cargo install wasm-pack`) and the wasm target
(`rustup target add wasm32-unknown-unknown`). Then register
`node /absolute/path/to/tools/mcp-server/src/server.js` as the command instead
of `npx opensolid-mcp`.

Models live in memory for the lifetime of the server process — there is no
persistence. Exports and screenshots are written to `OPENSOLID_MCP_OUTPUT_DIR`.

---

## 2. Tool reference

| Tool             | Input (required **bold**)                          | Returns |
|------------------|----------------------------------------------------|---------|
| `create_model`   | **`script`**, `name`, `exact`                      | `model_id` + mesh stats + validation summary |
| `get_screenshot` | **`model_id`**, `view`, `width`, `height`          | inline PNG image |
| `export`         | **`model_id`**, **`format`**, `path`, `accuracy`, `unit` | file path + byte size |
| `measure`        | **`model_id`**, `query`, `accuracy`                | mass properties |
| `optimize`       | **`model_id`**, **`params`**, **`objective`**, `constraints`, `options` | converged params + achieved objective + trajectory |
| `validate`       | **`model_id`**, `accuracy`                          | structural report |
| `get_capabilities` | `section`                                        | machine-readable manifest of every tool and script op |
| `list_models`    | —                                                  | models registered this session |

Every tool except `create_model`, `get_capabilities`, and `list_models` takes a
`model_id` handed back by an earlier `create_model` call.

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

`unit` declares the **document unit** in the STEP header: `mm` (default), `cm`,
`m`, or `in`. It is metadata, not a conversion — the kernel is unitless and
coordinates are written verbatim, so exporting a 60-unit part as `in` declares a
60-**inch** part, not 2.36. That is the point: without a declaration, an importer
resolves the file as millimetres, inches, or metres essentially at random.
Anything but the four keys is rejected rather than quietly defaulted. STL and OBJ
carry no unit declaration at all, so passing `unit` with them returns a `note`
saying so. See [units.md](units.md).

### `measure`

Mass properties computed as exact polyhedral integrals over the mesh. `query`
narrows the result: `all` (default), `volume`, `surface_area`, `bbox`,
`centroid`, `mass` (volume + area + centroid + inertia). `accuracy` sets the
target chordal deviation of the measured mesh in model units.

Volume and centroid are the agent's cheapest correctness oracles — a volume
*delta* confirms a cut actually removed material; a centroid confirms a feature
landed where intended.

### `optimize`

`measure` reports; `optimize` **moves**. It drives a model's `param()` design
variables (see §3) onto a target under constraints, using gradient descent on
the smooth F-Rep field, and writes the converged values back into the model — so
the next `measure`/`export`/`get_screenshot` shows the optimized part.

```jsonc
{
  "model_id": "bracket-1",
  "params": [ { "name": "thickness", "min": 2, "max": 8 } ], // what may move; bounds required*
  "objective": { "type": "target_mass", "value": 45, "density": 0.0027 }, // grams, g/mm³
  "constraints": [ { "type": "clearance", "probes": [[20, 0, 0]], "min": 1.5 } ],
  "options": { "max_iters": 60, "resolution": 40 }
}
```

- **Objectives:** `target_mass` (needs `density`, mass per model unit³),
  `target_volume`, `centroid_at` (`value` is `[x,y,z]`; use `null` for an axis
  you don't constrain).
- **Constraints:** `clearance` (the solid stays `min` away from keep-out
  `probes` — points, `[[x,y,z],…]` or flat), and `mass`/`volume` bounds
  (`min`/`max`). Constraints are soft quadratic penalties.
- **\*Bounds are required** — a param with no `min`/`max` in the request *or* its
  `param()` declaration is rejected. A negative wall thickness is not a design.
- **Guardrails:** `max_iters` (default 60), `time_budget_ms` (default 30 s),
  `resolution` (field quadrature, default 32; higher is more accurate but ~res³
  slower), `penalty_weight` (constraint stiffness, default 50).

The result reports the achieved objective from the **exact mesh** (the field
that steered the search is biased high; the reported number is the real one),
`converged` and `feasible` flags, any parameters `pinned` to a bound, a
per-iteration `trajectory`, and `warnings`. Read them honestly: `converged:
false` or `feasible: false` means the reported point is a starting place, not an
answer — a pinned parameter usually means the design wants to go past a bound you
set, and an unsatisfiable constraint (`feasible: false`) means the objective and
the constraint genuinely conflict. Every op is supported, `rotate` included.
Topology is yours: `optimize` moves numbers — to change structure, edit the
script and optimize again. Worked example:
[`optimize-bracket`](../tools/mcp-server/examples/agent-gallery/optimize-bracket.md).

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

### `get_capabilities`

The whole surface as JSON, for an agent that would rather read a manifest than
prose. Returns the server identity, the conventions below, the export units,
every tool with its real `inputSchema`, and every script op with its signature,
argument names and notes — grouped by `kind` (`primitive`, `feature`,
`transform`, `boolean`, `blend`, `pattern`, `query`, `builder`, `internal`).
`section` narrows it to `tools`, `script`, `conventions`, or `units` — worth
doing, since the whole manifest is ~29 KB of (compact) JSON.

The inventory is not a hand-maintained copy of the docs: the tool entries *are*
the live definitions, and
[`test/capabilities.test.js`](../tools/mcp-server/test/capabilities.test.js)
diffs the script inventory against the actually-bound classes in both
directions, so a kernel op cannot ship undocumented and a manifest entry cannot
outlive the op it names.

### `list_models`

Returns `{ models: [{ model_id, name, exact, createdAt }] }` for everything
registered this session.

---

## 3. Script API crash course

`create_model` takes a **JavaScript function body** (not a module) that must
`return` a `Shape`. It runs in strict mode with five bindings in scope:

| Binding | What it is |
|---------|------------|
| `Shape` | the solid — primitives, features, transforms, booleans, queries |
| `Profile` | closed 2D profile builder, for `extrude` / `revolve` / `loft` |
| `Path` | 3D polyline builder, for `sweep` |
| `OpenPath` | open 2D polyline builder, for `rib` |
| `param` | declare a design variable `optimize` may move |

The first three match the playground's **Code** tab exactly. No imports, no
`require`, no filesystem or network. Because it's real JavaScript, patterns
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

> ### `param()` — declare a design variable for `optimize`
>
> `param(name, default, { min, max })` marks a number as a tunable design
> variable and returns the value to use. The model builds at the `default`, so a
> script with `param()` calls still runs normally in `create_model` — but the
> `optimize` tool can now *move* those numbers to hit a target:
>
> ```js
> const t = param('thickness', 4, { min: 2, max: 8 });  // builds at 4 mm
> const base = Shape.box3(30, 20, t / 2);
> return base.union(Shape.box3(30, t / 2, 20).translate(0, -(20 - t / 2), 20 - t / 2));
> ```
>
> Only wrap the dimensions you want a search to own — most numbers are intent (a
> bolt circle that must stay a bolt circle), and declaring them all optimizes the
> wrong thing. Bounds are optional at declaration but then **must** be supplied
> in the `optimize` call; a design variable with no bound anywhere is an error.
> `create_model` echoes the declared params back so you can see what is tunable.

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

| Call | Shape | Exact¹ |
|------|-------|:--:|
| `Shape.sphere(r)` | sphere, radius `r` | ✓ |
| `Shape.box3(hx, hy, hz)` | box, half-extents (full size `2hx × 2hy × 2hz`) | ✓ |
| `Shape.roundedBox(hx, hy, hz, r)` | box with every edge rounded to radius `r` (half-extents include the rounding) | |
| `Shape.cylinder(r, hh)` | cylinder, radius `r`, half-height `hh`, axis **+Y** | ✓ |
| `Shape.torus(major, minor)` | torus with its ring in the **XZ** plane | ✓ |
| `Shape.cone(rBottom, rTop, hh)` | cone or frustum along **+Y**: `rBottom` at `y=-hh`, `rTop` at `y=+hh`. Either radius may be `0` for a point (not both) | ✓ |
| `Shape.capsule(x1,y1,z1, x2,y2,z2, r)` | sphere swept along the segment between two points | |
| `Shape.halfSpace(px,py,pz, nx,ny,nz)` | the solid half *behind* the plane through `(px,py,pz)` with outward normal `(nx,ny,nz)` — unbounded on its own | |

¹ *Exact* marks ops that can carry an analytic B-Rep companion, so an
`exact: true` model keeps crisp edges and exports analytic STEP. Everything else
is SDF-only and exports a faceted-but-valid B-Rep.

`halfSpace` is the "up to face" terminator: intersect a through-all extrude with
one to clip the extrude at a plane, instead of guessing a height.

**`Shape` — sketch features** (a `Profile`/`Path` becomes a solid)

| Call | Shape |
|------|-------|
| `Shape.extrude(profile, height, draftDeg?)` | profile swept along **+Y** from `y=0` to `y=height`; profile `(x,y)` → world `(x,z)`. Positive `draftDeg` narrows toward the top cap (mould-release draft), negative flares; its magnitude must stay under ~80° |
| `Shape.revolve(profile, angleDeg)` | profile revolved about **Y** through `angleDeg` ∈ `(0,360]`, sweeping +X toward +Z. Profile `(x,y)` → `(radius, y)`, so it must lie in `x ≥ 0` |
| `Shape.sweep(profile, path)` | profile swept along a 3D `Path` polyline, twist-free per segment; joints mitre by unioning the per-segment prisms. Constant profile, no twist |
| `Shape.loft(bottom, top, height)` | blend between two closed profiles on parallel planes (`bottom` at `y=0`, `top` at `y=height`) by morphing their signed distances. Parallel planes, linear morph |
| `Shape.rib(openPath, thickness, height, side)` | an **open** 2D path thickened into a support rib and swept **+Y**; path `(x,y)` → world `(x,z)`. `side` is `"both"` (`thickness/2` each way), `"first"` (full thickness left of travel) or `"second"` (right). Union it with the parent body yourself |

**`Shape` — transforms** (return a new shape; never mutate)

| Call | Effect | Exact |
|------|--------|:--:|
| `s.translate(x, y, z)` | translate | ✓ |
| `s.rotate(ax, ay, az, angleDeg)` | rotate `angleDeg` about the axis `(ax,ay,az)` through the origin | ✓ |
| `s.uniformScale(f)` | uniform scale about the origin (`f > 0`) | ✓ |
| `s.scale(sx, sy, sz)` | per-axis scale. Booleans and meshing stay correct, but the field is no longer an exact distance, so blends applied *after* it are distorted — prefer `uniformScale` when the factors are equal | |
| `s.taper(px,py,pz, nx,ny,nz, angleDeg)` | mould-release draft about a parting plane: walls flare toward the pull direction `(px,py,pz)` above the neutral plane through `(nx,ny,nz)` and pinch below. **Pull direction first, then the plane point.** Whole-body — the F-Rep approximation of a face-selective draft | |

**`Shape` — booleans and blends**

| Call | Effect | Exact |
|------|--------|:--:|
| `a.union(b)` / `a.subtract(b)` / `a.intersect(b)` | CSG | ✓ |
| `a.smoothUnion(b, r)` | union with a smooth blend of radius `r` along the *whole* intersection curve (organic fillet). Omitting `r` picks 10% of the combined bounding box's largest extent | |
| `a.filletEdge(b, r, edge)` | union of `a` and `b` with a rounded blend of radius `r` on **one selected edge**; every other edge stays sharp. `edge` is a flat `[x0,y0,z0, x1,y1,z1, …]` polyline of the picked feature edge | |
| `a.chamferEdge(b, setback, edge)` | same selection, but a planar bevel of `setback` instead of a round | |

`smoothUnion` blends everywhere the two bodies meet; `filletEdge`/`chamferEdge`
are the edge-selective pair — reach for them when only one corner should break.

**`Shape` — thin-wall and patterns**

| Call | Effect |
|------|--------|
| `s.shell(thickness)` | hollow into a shell of total wall `thickness`, **centred on the surface** (`thickness/2` each side, so the outer extent grows by `thickness/2`). Closed all round — intersect or subtract to open a face |
| `s.linearPattern(dx, dy, dz, count)` | `count` copies, copy `k` translated by `k·(dx,dy,dz)`. Copy 0 is the original; `count` rounds to the nearest integer and must be ≥ 1 |
| `s.circularPattern(ax,ay,az, cx,cy,cz, count, angleDeg?)` | `count` copies spaced evenly about the axis with direction `(ax,ay,az)` through the point `(cx,cy,cz)`, spanning `angleDeg` total (default `360`). **Axis direction first, then the axis point** |
| `s.mirror(nx,ny,nz, px,py,pz)` | this shape **unioned with** its reflection across the plane through `(px,py,pz)` with normal `(nx,ny,nz)` — a mirrored *copy*, not a reflection in place. **Normal first, then the plane point** |

**`Shape` — in-script queries** (answer a question without a tool round trip)

| Call | Returns |
|------|---------|
| `s.distance(x, y, z)` | signed distance to the surface — negative inside, positive outside. After smooth blends or anisotropic scaling it is not an exact Euclidean distance, but the sign and zero set stay correct, so it still answers "is this point inside?" and "which point is nearer?" |
| `s.normalAt(x, y, z)` | outward unit surface normal `[nx, ny, nz]` at the point — the frame for sketching on a curved face |
| `s.bounds()` | tracked bounding box `[minX,minY,minZ, maxX,maxY,maxZ]`. **Conservative**: it encloses the surface and can overstate a blended or repeatedly-rotated part. For the part's real extent use `measure`'s `boundingBox`, which is measured off the mesh |
| `s.isExact()` | whether this shape will serve a validated exact B-Rep tessellation |

These run inside the script, so a script can *decide* with them — e.g. keep only
the pattern copies that clear an obstacle:

```js
let part = Shape.box3(40, 4, 20);
const keepOut = Shape.sphere(6).translate(20, 0, 0);
for (let i = -3; i <= 3; i++) {
  const at = [i * 10, 0, 0];
  if (keepOut.distance(...at) > 3) {              // 3 units of clearance
    part = part.subtract(Shape.cylinder(2, 6).translate(...at));
  }
}
return part;
```

**`Profile` — closed 2D profiles for extrude / revolve / loft**

A closed polyline with optional arc, elliptical-arc, and Bézier segments.
`bulge` is `tan(θ/4)` for the arc's swept angle (`0` = straight, positive =
counter-clockwise):

```js
const p = new Profile(0, 0);   // start at the origin
p.lineTo(40, 0);
p.lineTo(40, 10);
p.arcTo(10, 40, 0.4);          // arc segment
p.lineTo(0, 40);
p.close();                     // required — building an unclosed profile errors
return Shape.extrude(p, 20);
```

| Call | Effect |
|------|--------|
| `new Profile(x, y)` | start a closed profile at `(x, y)` |
| `p.lineTo(x, y)` | straight segment |
| `p.arcTo(x, y, bulge)` | circular arc (`bulge` = `tan(θ/4)`) |
| `p.ellipseArcTo(x, y, cx, cy, rx, ry, rotationDeg, ccw)` | elliptical arc. **Centre-parameterised, not SVG endpoint form**: both the current point and `(x,y)` must lie on the ellipse centred at `(cx,cy)` with semi-axes `rx`/`ry` rotated by `rotationDeg`; `ccw` picks the direction |
| `p.cubicTo(c1x, c1y, c2x, c2y, x, y)` | cubic Bézier with the two control points |
| `p.close()` | close the loop back to the start |

Segments added after `close()` are ignored.

**`Path` — 3D polyline for `Shape.sweep`**

| Call | Effect |
|------|--------|
| `new Path(x, y, z)` | start the path at `(x, y, z)` |
| `path.lineTo(x, y, z)` | straight segment to `(x, y, z)` — straight segments only, no arcs in 3D yet |

```js
const tube = new Profile(-2, -2);                  // 4×4 section
tube.lineTo(2, -2); tube.lineTo(2, 2); tube.lineTo(-2, 2); tube.close();
const route = new Path(0, 0, 0);
route.lineTo(0, 30, 0);
route.lineTo(20, 30, 0);                            // mitred elbow
return Shape.sweep(tube, route);
```

**`OpenPath` — open 2D polyline for `Shape.rib`**

Same segment vocabulary as `Profile` minus `close()` — it is never closed.

| Call | Effect |
|------|--------|
| `new OpenPath(x, y)` | start the path at `(x, y)` |
| `p.lineTo(x, y)` / `p.arcTo(x, y, bulge)` | straight / circular-arc segment |
| `p.ellipseArcTo(x, y, cx, cy, rx, ry, rotationDeg, ccw)` | elliptical arc (as `Profile`) |
| `p.cubicTo(c1x, c1y, c2x, c2y, x, y)` | cubic Bézier |

```js
const base = Shape.box3(20, 1, 20);                 // 40 × 2 × 40 plate
const spine = new OpenPath(-15, 0);
spine.lineTo(15, 0);
return base.union(Shape.rib(spine, 2, 10, 'both')); // 2 thick, 10 tall
```

> `OpenPath` is bound here but not yet in the playground's **Code** tab, so a
> script using `Shape.rib` is the one thing that will not paste straight into the
> browser. Everything else in this section is common to both.

Full reference (with the exact-vs-SDF discussion) lives in the
[server README](../tools/mcp-server/README.md#the-script-format), and
`get_capabilities` returns all of the above as JSON.

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

The seven gallery transcripts each walk this loop on a real part. Start there —
or call `get_capabilities` first if you would rather have the surface as JSON.
