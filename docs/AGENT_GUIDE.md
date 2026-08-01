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
| `import_step`    | `path` \| `text`, `name`, `circle_segments`        | `model_id` + per-solid outcomes + diagnostics + measure/validate summary |
| `get_screenshot` | **`model_id`**, `view` \| `direction`, `region` \| `target` + `zoom`, `mode`, `section`, `accuracy`, `line_width`, `width`, `height` | inline PNG image + the camera that produced it |
| `export`         | **`model_id`**, **`format`**, `path`, `accuracy`, `unit` | file path + byte size |
| `measure`        | **`model_id`**, `query`, `accuracy`                | mass properties |
| `optimize`       | **`model_id`**, **`params`**, **`objective`**, `constraints`, `options` | converged params + achieved objective + trajectory |
| `validate`       | **`model_id`**, `accuracy`, `deep`                 | mesh + B-Rep structural report |
| `inspect_topology` | **`model_id`**, `accuracy`, `probes`, `include_faces` | faces, rims, holes with axes, shells, genus |
| `assert_model`   | **`model_id`**, **`expect`**, `accuracy`           | pass/fail per expectation |
| `diff_models`    | **`model_id_a`**, **`model_id_b`**, `accuracy`, `expect_volume_delta` | what changed between two models |
| `measure_clearance` | **`model_id`**, `probes` \| `against_model_id`, `accuracy`, `softness` | signed distances / interference |
| `get_capabilities` | `section`                                        | machine-readable manifest of every tool and script op |
| `list_models`    | —                                                  | models registered this session |
| `get_model`      | **`model_id`**                                      | the model's own source: its script, or where it was imported from |

Every tool except `create_model`, `import_step`, `get_capabilities`, and
`list_models` takes a `model_id` handed back by an earlier `create_model` or
`import_step` call.

### Which oracle answers which question

`validate` and `measure` report *scalars over the whole part*, and a part can be
badly wrong while every scalar looks fine. The gallery's angle bracket shipped
with its four mounting holes bored sideways through the plates: `validate` said
`valid: true`, the screenshot rendered plausibly, STL wrote fine, and the volume
was only ~4% light, because a wrong-axis hole removes nearly the right amount of
material. See [the friction log](dogfood-bracket-friction-log.md) for the full
account. Reach for the tool that can actually see the mistake you might be
making:

| Question | Tool |
|---|---|
| Is the mesh watertight? Is the B-Rep body sound? | `validate` (`deep: true` for self-intersection) |
| Is this **imported** STEP body actually sound? | `validate` — the B-Rep check exists for topology of unknown provenance, and "the file parsed" says nothing about whether its geometry holds together |
| How much material is there? Where is its centre of mass? | `measure` |
| **How many holes go through it, and along which axis?** | `inspect_topology` |
| Is there a hole through *this point* in *this direction*? | `inspect_topology` `probes`, or `assert_model` `hole_at` |
| Did my cut remove the volume it should have? | `diff_models` with `expect_volume_delta` |
| Does the finished part match what I intended? | `assert_model` |
| Do two parts collide? Does the solid stay clear of a keep-out? | `measure_clearance` |
| Do the several meshers agree this is a solid? | `validate` (`mesher` field), `meshAgreement` in-script |
| Does the part *look* like the thing I described? Is a feature on the wrong face? | `get_screenshot` — framed on the feature, and sectioned if the feature is interior. The one probabilistic oracle: it is a smoke test between the machine checks and a human, and it never overrules a measurement |

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

### `import_step`

Reads an existing STEP (Part 21) file and registers it as a model — the other
way to get a `model_id`, for when the job starts from a part someone handed you
rather than from a script. Pass `path` (a file: absolute, or relative to the
output dir, so a file `export` just wrote reads back by bare name) **or** `text`
(the contents inline). Not both.

```json
{
  "model_id": "model-3-1c4d",
  "name": "bracket",
  "exact": true,
  "source": { "path": "/out/bracket.step", "bytes": 48213 },
  "solids": [
    { "index": 0, "step_id": 503, "name": "bracket", "outcome": "brep",
      "exact": true, "triangles": 1284, "model_id": "model-2-9be1" }
  ],
  "counts": { "brep": 1, "mesh": 0, "failed": 0 },
  "healing": { "operations": 0 },
  "units": { "lengthScale": 1, "angleScale": 1, "note": "…already applied…" },
  "assembly": { "isAssembly": false, "instances": 1 },
  "diagnostics": { "error": 0, "warning": 0, "info": 2, "items": [ … ] },
  "mesh": { "triangles": 1284, "vertices": 646 },
  "boundingBox": { "min": [...], "max": [...], "size": [60, 40, 8] },
  "volume": 17280, "valid": true, "issues": []
}
```

Read it in this order:

- **`outcome`, per solid.** `brep` means the exact path won: analytic surfaces,
  and an `export` of that model writes analytic STEP rather than facets. `mesh`
  means the file is valid but holds geometry the kernel cannot represent
  exactly, so the solid arrives as a closed tessellation wrapped as an SDF —
  fine to measure, render and boolean against, but its surfaces are gone.
  `failed` means the solid did not survive; the diagnostics say why. A solid can
  also come back `brep` with a `shapeError`: the body imported but the
  tessellator could not handle a face, so there is no shape to hand you.
- **`volume` / `valid` / `issues`.** The same oracle `create_model` returns.
  An import is where the trust loop starts, so it runs immediately: a part that
  arrives measuring the wrong volume is the failure worth catching in the first
  call, not the fifth.
- **`diagnostics`.** The reader's per-entity findings, `error`/`warning`/`info`
  counted in full with a bounded, severity-first sample in `items`. `healing.operations`
  counts the repairs the importer applied to make bodies valid.
- **`units`.** What the file *declared*, already applied: imported geometry is
  millimetres whatever the header said. An inch file's 20-unit box arrives 508 mm
  across — that is correct, not a bug, and it is why the field is reported.

Every solid also gets its own `model_id`, so a multi-solid file is addressable
part by part. The top-level `model_id` is the whole file: for an assembly, every
occurrence placed into root coordinates (`assembly.isAssembly`,
`assembly.instances`), while the per-solid models stay part-local, exactly as the
file stores them. Its volume sums the placed parts, so interpenetrating parts
count twice — for an assembly of solid parts that is the total material.

`circle_segments` (default 32, clamped 3..512) sets how finely imported bodies
are tessellated. That mesh is what every measurement and screenshot of the import
reads, so raise it when a curved part measures short; it does not touch the
analytic surfaces, so `export` of a `brep` solid is unaffected.

Imported models work with `measure`, `validate`, `get_screenshot` and `export`
like any other. They carry no `param()`s — nothing about an imported file is
parametric — so `optimize` has nothing to move.

### `get_screenshot`

Renders a model to a PNG and returns it inline (no file written), followed by a
short text block naming the exact camera that produced it. The renderer is a
pure-JS software rasterizer — a screenshot is a few milliseconds, no GPU, no
headless browser.

**This is a probabilistic oracle, and it sits below the machine checks, not
above them.** A screenshot cannot adjudicate a hole's axis, a wall thickness, or
a 0.2 mm interference — `inspect_topology`, `assert_model` and
`measure_clearance` can. What it *can* do is catch the class of defect that
passes every scalar check because the scalar was never wrong: a pocket in the
right place on the wrong face, a rib that fused into its neighbour, a fillet
that ate a feature. Take the shot after the assertions pass, not instead of
them.

| Argument | Effect |
|---|---|
| `view` | `iso` (default), `front`, `back`, `right`, `left`, `top`, `bottom` |
| `direction` | arbitrary `[x,y,z]` the camera looks *along* — overrides `view` |
| `up` | screen-up for a custom `direction` (default `+Y`, or `±Z` looking straight down/up) |
| `region` | `{min:[x,y,z], max:[x,y,z]}` world box to frame instead of the whole part |
| `target` | world point to put at the centre of the frame |
| `zoom` | magnification on top of the fit — `2` is twice as close, `0.5` pulls back |
| `mode` | `shaded` (default), `shaded_edges`, `edges` |
| `section` | `{axis:'X'\|'Y'\|'Z', offset, flip}` — an axis-aligned cut |
| `line_width` | ink thickness in px, 1..8 (default 2 in `edges`, 1 overlaid) |
| `accuracy` | chordal deviation of the rendered mesh (default 0.5% of the extent) |
| `width`/`height` | pixels, default 800×600 |

**Framing.** By default the whole part is fitted to the frame, which at 800×600
gives a 3 mm hole in a 200 mm plate about four pixels. That is not an image
anything can be judged from. Frame from what the machine tools already told you:

- `region` takes the same `{min, max}` shape `measure`'s `boundingBox` comes in,
  so a box you measured can be pasted straight back in.
- `inspect_topology` reports a hole or boss as `{center, axis, diameter}`, not as
  a box — for one of those, `target: center` with `zoom` is the direct
  translation, and `view`/`direction` set to its `axis` is the shot that shows
  whether the axis is the one you meant.
- `zoom` is a multiplier on the fit: `2` is twice as close and shows a quarter of
  the area, `0.5` pulls back.

**Sections.** `section` cuts the model with an axis-aligned plane and shades the
cut face **amber**, so a cut surface can never be mistaken for material. It
keeps the half with the *smaller* coordinate on that axis; `flip` keeps the
other. An omitted `offset` puts the plane at the model's midpoint on that axis,
which is the only offset guaranteed to actually intersect the part. This is the
only way to see interior geometry: a blind hole's floor, a wall's real
thickness, whether two internal pockets broke into each other.

A cut plane the camera lies *in* has no visible area — a `front` view with an
`X` section shows the kept half but no cut face. Look along the plane's normal,
or from `iso`, to see the cut.

**Edge modes.** `edges` draws feature (crease) and silhouette line-work in black
on a light ground, with hidden lines removed — the dimension-drawing look, where
a boundary is a line rather than a shading gradient. It is markedly easier to
read than shading for anything with parallel faces or a shallow step.
`shaded_edges` overlays the same line-work on the solid, which is the best
single shot of a part with both curved and prismatic features.

**Determinism.** The same arguments against the same model produce
byte-identical PNGs: the palette, the light, the margin and the rasterizer
settings are fixed constants, not options, and every framing input is explicit.
Two shots are therefore comparable — a pixel that changed means the *geometry*
changed. Pin `accuracy` when the comparison spans a model rebuild at a different
size, since the default accuracy tracks the part's extent.

The trailing text block reports what was resolved, not what was asked:

```jsonc
{"model_id":"model-1-8f3a","accuracy":0.1,
 "camera":{"view":"top","direction":[0,-1,0],"up":[0,0,-1],"target":[12,0,0],
           "zoom":3,"scale":36,"visibleExtent":[22.2,16.7],"width":800,"height":600,
           "mode":"shaded_edges","lineWidth":1,"section":null}}
```

`scale` is pixels per model unit and `visibleExtent` is how much of the world the
frame covers — together they say whether the feature you meant to inspect is
actually big enough on screen to judge. Re-issue with the same values to
reproduce a shot exactly, or nudge one of them to move.

#### Framing a purposeful inspection shot

A shot is worth taking when it could come back *wrong*. Pick the framing from
what the feature would look like if it had failed:

| What you built | Shot that could show it wrong |
|---|---|
| A through-hole | `view` along the hole's axis, `region` = the hole's bbox padded ~2×, `shaded` — a hole that goes through is a *dark* disc (you are seeing the background through it); one that stopped short is a disc lit like the face around it, and a mis-axised one is an ellipse or is simply absent. Do not take this shot in an edge mode: the near and far rims project to the same circle, so blind and through draw identically |
| A blind hole or pocket | `section` on an axis perpendicular to it, `offset` through its centre — the only view where the floor exists, and the only reliable way to read its depth |
| A wall between two features | `section` across the wall, `mode: 'shaded_edges'` — a wall that broke through is a gap, not a thin line |
| A fillet or chamfer | `region` on the edge, `zoom: 2`+, `mode: 'shaded_edges'` — a radius that swallowed a neighbouring feature is obvious close up and invisible whole-part |
| A boss, rib or lug | `view` face-on to the face it stands on, plus one `iso` — a rib that fused into a wall loses its outline in exactly one of the two |
| A pattern (linear/circular) | whole-part `view` down the pattern's axis, `mode: 'edges'` — miscounts and overlaps are countable in line-work and mush in shading |
| A thin plate or bracket | `mode: 'edges'` from `front`/`top`/`right` — three orthographic line drawings, the way a machinist would read it |
| An imported STEP body | `iso` `shaded_edges` first (does it look like a part at all?), then `section` through the middle (is it a solid or a hollow shell?) |

Two habits that make the difference:

- **Take a pair, not a shot.** One view answers "does this look right"; two
  views at right angles answer "is this the right shape". A feature that is
  wrong on one axis usually looks perfect from the axis it is wrong about.
- **Frame before and after.** Screenshot the same `region` from the same camera
  before and after a cut. Identical bytes means the cut did nothing — which is a
  real and otherwise quiet failure mode, and cheaper to check than
  `diff_models` when you already have both models.

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

Two checks in one report: the mesh is a closed, consistently-oriented manifold
enclosing a finite non-zero volume, **and** — when the model carries an exact
B-Rep — that body passes the kernel's own validation.

```json
{
  "valid": true,
  "closedManifold": true,
  "triangles": 16752,
  "vertices": 8376,
  "volume": 7008.29,
  "exact": false,
  "mesher": "adaptive-sdf",
  "brep": { "available": true, "source": "boolean-result",
            "counts": { "shells": 1, "faces": 7, "edges": 15, "vertices": 8, "genus": 1,
                        "innerLoops": 2, "surfaceKinds": { "plane": 6, "cylinder": 1 } },
            "failures": [], "selfIntersectionChecked": false },
  "issues": []
}
```

- `mesher` names which mesh answered — `exact-brep` (an exact boolean's own
  validated tessellation), `step-reader` (the STEP reader's tessellation of a
  body from `import_step`), or `adaptive-sdf`. It matters: `valid` is a statement about
  *that* mesh, and the several meshers do not always agree (§4).
- `brep` is the body report. The kernel checks referential integrity, loop
  bookkeeping, shell closure and manifoldness, orientation consistency across
  every edge, tolerance sanity, the Euler–Poincaré formula, and the geometric
  invariants: every edge lying on the surfaces of the faces it bounds, every
  vertex on its edges' endpoints, pcurve fidelity, and face sense against loop
  winding. `deep: true` adds face-face **self-intersection** — the body passing
  through itself — which is a pairwise search over faces and so is opt-in.
  Failures appear both here (with a machine-readable `kind`) and in `issues`,
  and they make `valid` false.
- `counts` is the body's real entity census. This is the one place a *true* face
  or edge count comes from: an SDF mesher renders a sharp edge as a one-cell
  bevel, so nothing recovered from a tessellation can count them exactly.
  `surfaceKinds` is often the fastest sanity check there is — four Ø5 bores are
  four cylindrical faces.
- `brep.available: false` carries a `reason` (the op chain left exact coverage, a
  boolean gated back to F-Rep, the mode is off). It is deliberately not reported
  as "no failures": *nothing was checked* and *checked, sound* are different
  answers, and conflating them is how a wrong part passes.

Call it before trusting a boolean result. But note what it still cannot see: a
closed manifold with the right volume can have its holes on the wrong axis. That
is `inspect_topology`'s job.

### `inspect_topology`

Structure instead of scalars — the oracle the other tools do not provide.

```jsonc
{
  "counts": { "planarFaces": 6, "circularRims": 8, "throughHoles": 4, "pockets": 0,
              "cavities": 0, "bosses": 0, "unpairedRims": 0, "shells": 1, "genus": 4 },
  "mesh": { "components": 1, "vertices": 16200, "edges": 48600, "triangles": 32396,
            "eulerCharacteristic": -6, "genus": 4, "closed": true },
  "cylinders": [
    { "kind": "through-hole", "axis": [0, 1, 0], "radius": 2.501, "diameter": 5.002,
      "center": [10, 0, 6], "depth": 4.96 }
    // ...three more
  ],
  "planarFaces": { "count": 6, "totalArea": 1256.4, "planarAreaFraction": 0.813,
                   "remainderArea": 235.1,
                   "faces": [ { "normal": [0, 1, 0], "offset": 2.5, "area": 510.3, "triangles": 8804 } ] },
  "brep": { "available": false, "reason": "…" }
}
```

- **`genus`** is the number of handles in the surface — for a plate, exactly how
  many holes go through it. It comes from `V − E + F` over the welded mesh, so it
  is pure combinatorics: no tolerance, no fit, and it does not drift with
  `accuracy`. `shells` is the connected-component count; a `shells: 2` on a part
  you meant to be one solid means a cut went all the way through and severed it.
- **`cylinders`** is the census that catches a mis-drilled hole. Rims are fitted
  from the mesh and paired into coaxial features, then the field is asked two
  questions about each: is the bore empty (else it is a `boss`), and does it open
  to air at both ends (`through-hole`), one end (`pocket`), or neither
  (`cavity`)? So `kind`, `axis`, `diameter` and `depth` are all measured rather
  than assumed. A hole meant to run through a 5 mm plate along +Y and bored along
  +Z shows up here with an axis of `[0,0,1]`, immediately.
- **`probes`** casts lines: `[{ "axis": [0,1,0], "at": [15,0,10] }]` returns the
  ordered solid/void `spans` that line crosses and a `throughHole` verdict
  (material → gap → material). This is independent of meshing entirely — it
  samples the distance field — and it is the most direct possible answer to "is
  there a hole here, going this way?".
- **`planarFaces`** grows each face from a seed plane, so it is stable against
  the mesher's bevel: 6 faces for a box, not the 194 a crease-cutting algorithm
  reports. `planarAreaFraction` is what the planar faces account for; the
  remainder is genuine curvature plus that bevel. Face areas run a percent or so
  under their analytic values for the same reason, so compare them with that in
  mind — or use `brep.counts` where the model has a B-Rep.

### `assert_model`

State what you intended; get back which expectations the geometry meets.

```jsonc
{
  "model_id": "bracket-1",
  "expect": [
    { "type": "closed_solid" },
    { "type": "volume", "value": 19750.08, "tolerance": 1 },
    { "type": "bbox_size", "value": [60, 40, 40], "tolerance": 0.5 },
    { "type": "genus", "value": 4 },
    { "type": "through_holes", "value": 4, "axis": [0, 1, 0], "diameter": 5 },
    { "type": "hole_at", "at": [22, 0, 14], "axis": [0, 1, 0], "diameter": 5 },
    { "type": "material_at", "at": [0, 0, 0], "value": true },
    { "type": "clearance", "probes": [[0, 30, 0]], "min": 2 },
    { "type": "brep_sound" }
  ]
}
```

Returns `{ ok, passed, failed, checks: [{ type, ok, expected, actual, message }] }`.
Full type list: `volume`, `surface_area`, `bbox_size`, `centroid`,
`closed_solid`, `shells`, `genus`, `planar_faces`, `through_holes`, `hole_at`,
`material_at`, `clearance`, `brep_sound` (see `get_capabilities` for the schema).

- Continuous quantities take an absolute `tolerance` or a `relative_tolerance`,
  defaulting to **1%** — these are integrals over a tessellation, not analytic
  values. Counts must match exactly.
- `bbox_size` and `centroid` take `[x,y,z]`; a `null` component skips that axis.
- `hole_at` takes the bore's **own** axis and a point on its centreline. It checks
  two things, because either alone is meaningless: the line along the axis is
  clear of material (so it is a bore), *and* the void around that point is
  enclosed by material in both directions across the axis (so it is a bore
  through the part, not empty space beside it).
- `through_holes` with an `axis` is the assertion that would have caught the
  bracket bug, and when it fails it tells you what is actually there: *"expected
  4 through-holes matching axis ≈ [0.000,1.000,0.000], found 0. The part has 4
  through-hole(s) in total: Ø5.002 along [0.00,0.00,1.00] at [22.00,0.00,14.00];
  …"*.
- **An expectation that cannot be evaluated fails.** A null volume, an
  unmeasurable genus, a `brep_sound` on a model with no B-Rep — all `ok: false`
  with the reason. A check that quietly abstains is how the wrong part passed the
  first time.

### `diff_models`

Compares two models measured at the same `accuracy` and reports the deltas:
volume (absolute and as a ratio of A), surface area, bounding-box size,
centroid, and the structural counts from `inspect_topology` (`shells`, `genus`,
`planarFaces`, `throughHoles`).

```jsonc
{
  "model_id_a": "plate-blank",
  "model_id_b": "plate-drilled",
  "expect_volume_delta": { "value": -392.7, "tolerance": 5 }
}
```

`expect_volume_delta` turns "these four Ø5 holes should remove 392.7 mm³" into a
pass/fail (`volumeDeltaCheck` in the response). Negative for material removed.
Pass the same `accuracy` for both — that is what the shared argument is for;
comparing a fine mesh against a coarse one turns a meshing difference into an
apparent design difference. `counts` deltas catch the structural half: drilling
four holes should move `genus` by exactly +4.

### `measure_clearance`

The passive counterpart to `optimize`'s `clearance` constraint — measuring,
not moving.

- With **`probes`** (`[[x,y,z],…]` or flat): each point's signed distance to the
  solid (negative = the point is inside the material), the `minDistance` and
  which probe it was, how many probes are inside, a `clear` verdict, and the
  smooth `softMin` the optimiser descends on.
- With **`against_model_id`**: samples that model's mesh vertices against this
  model's field and reports `interferes`, `minDistance` (negative = overlap
  depth), `deepestPoint`, and how many vertices are inside. Nothing else in the
  toolset answers "do these two parts collide?". Resolution is that mesh's
  `accuracy`, so an overlap thinner than its triangle spacing can be missed —
  pass a finer `accuracy` to tighten it.

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

Returns `{ models: [{ model_id, name, exact, source, createdAt }] }` for
everything registered this session. `source` is `script` or `step` — which kind
of origin `get_model` will hand back for that id.

### `get_model`

Returns a model's own source: the `script` it was built from, verbatim and ready
to edit and re-submit, or where it was `imported` from. Also its params **with
their current values**.

That last part is the point. `optimize` writes the converged numbers back into
the model, and until you ask for them here they exist only in that one response
— so a session that ends takes the design with it. `get_model` turns a
`model_id` back into the two things that reproduce the part: the script, and the
numbers to run it at.

```json
{
  "model_id": "model-1-8f3a", "name": "plate", "exact": false,
  "source": "script",
  "script": "const t = param('thickness', 4, {min: 2, max: 8});\nreturn Shape.box3(t, 10, 10);",
  "params": [{ "name": "thickness", "value": 6.31, "default": 4, "min": 2, "max": 8 }]
}
```

For an imported model there is no script — nothing generated it — so `imported`
carries the provenance instead: the file, its size, and for a per-solid model
which solid of it (`solidIndex`, `stepId`, `outcome`).

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

The first four match the playground's **Code** tab exactly. No imports, no
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

> `OpenPath` is bound in the playground's **Code** tab too, so everything in
> this section is common to both surfaces. Only `param()` (above) is
> MCP-specific.

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

### Import problems — graded by `import_step`

An import has three degrees of failure, and only the first is an `isError`:

| Situation | Reported as |
|-----------|-------------|
| Not syntactically valid Part 21, or the file cannot be read | `isError` — `Error: import failed: STEP import failed: <detail>` / `Error: cannot read STEP file: <detail>` |
| Valid file, but nothing usable in it (no solids, or none survived) | `isError` — a JSON payload with `error`, the per-solid outcomes, and the diagnostics |
| Valid file, some solids degraded | **not** an error — the model registers, and the per-solid `outcome` / `shapeError` and the `diagnostics` say what was lost |

The third row is the one to actually read. `counts: { brep, mesh, failed }` is the
one-line summary: a solid that came back `mesh` is real geometry with its analytic
surfaces gone (it will export STEP as facets), and a `failed` one is not there at
all — so a part that "imported fine" can still be missing a boss the file
described. Check `counts` and `valid` before treating an import as the part.

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

Check `valid` (or call `validate`) before exporting. A screenshot can look
plausible while the mesh is open; the validity report cannot be fooled about
*closure*. Trying to screenshot an empty model is itself a clean error — `Error:
model produced an empty mesh; nothing to render`.

### Structurally wrong but perfectly valid — caught by `inspect_topology`

`valid: true` is a narrow claim: this mesh is closed, oriented, and bounds a
volume. It is not a claim that the part is the one you designed. The gallery's
angle bracket had its four mounting holes bored along the wrong axis and reported
`valid: true`, rendered plausibly, exported STL fine, and measured only ~4%
light. Every oracle available at the time agreed with it.

So when a feature has a *direction* — a hole, a bore, a slot, a pocket — verify
the direction:

```jsonc
// Did the four Ø5 holes actually go through the plate, along +Y?
{ "type": "through_holes", "value": 4, "axis": [0, 1, 0], "diameter": 5 }
```

`assert_model` with that expectation, or `inspect_topology`'s `cylinders` and
`probes`, will say so in one call. `genus` is the cheapest version: four holes
through a plate is genus 4, and no meshing accuracy can make it read otherwise.

### Meshers that disagree — `mesher`, and `meshAgreement`

A shape is meshed by more than one route and they are not interchangeable:
`mesh` dual-contours a fixed uniform grid (what the playground viewport shows),
`measure`/`validate` read an adaptive octree mesh, faceted STEP export recovers
planar regions through `sdf_to_brep`, and an exact boolean result serves its own
validated analytic tessellation. "`valid: true`, STL fine, STEP declines" is a
real signature (of-obv, of-2i8) and it used to be undiagnosable, because a caller
could only ever see one of those answers at a time.

`validate`'s `mesher` field names the one that answered. To see all of them side
by side, call `shape.meshAgreement(accuracy)` from inside a script:

```js
const s = Shape.box3(15, 2.5, 10).subtract(Shape.cylinder(2.5, 10));
// { agree, accuracy, paths: [{name, available, closed, triangles, volume}], disagreement: [...] }
console.log(s.meshAgreement(0.2));
return s;
```

`agree` weighs closure across every path, and volume only across the paths
`accuracy` governs — the uniform grid has no accuracy knob, so it is legitimately
coarser (measured ~3% light on a 30 mm plate with a Ø5 hole) and its volume is
reported but excluded from the verdict. A `step-facet` path that comes back
`available: false` is telling you STEP export will decline, before you spend the
export.

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

1. **`create_model`** (or **`import_step`**, when the part already exists) →
   read `valid` and `volume`. If `valid: false`, fix the script before doing
   anything else; on an import, read the per-solid `outcome` first — a `mesh`
   fallback is a different part from a `brep` one in everything but shape.
2. **`validate`** (or trust `create_model`'s summary) to confirm a closed
   manifold — and a sound B-Rep body, where the model has one — after a
   nontrivial boolean.
3. **`assert_model`** with what you actually intended: the volume you computed
   from the spec, the bounding box, and — for every directional feature — the
   hole count *with its axis*. This is the step that was missing. Steps 1, 2, 4
   and 5 all passed on a bracket drilled the wrong way.
4. **`inspect_topology`** when an assertion fails and you need to see what the
   geometry actually is: the hole axes, the genus, the shell count. Or
   **`diff_models`** against the model before the cut, to check the cut removed
   what it should have.
5. **`get_screenshot`** for a gut check — *framed on the feature you are unsure
   of*, not on the whole part, and `section`ed when the feature is interior. It
   cannot adjudicate a hole's axis, but it is the only oracle that sees a
   feature on the wrong face, a rib fused into a wall, or a fillet that ate its
   neighbour. Whole-part `iso` thumbnails are the shot that missed the bracket.
6. **`export`** to STEP/STL/OBJ, branching on `isError` for the STEP faceting
   limitation above.

The seven gallery transcripts each walk this loop on a real part. Start there —
or call `get_capabilities` first if you would rather have the surface as JSON.
