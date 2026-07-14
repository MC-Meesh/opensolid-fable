# Edge-selective fillet & chamfer

Status: design note + F-Rep MVP (of-fsl.4)
Scope: F-Rep (implicit) blends on edges arising from boolean ops between
analytic primitives. Exact B-Rep blends are **out of scope** (follow-up).

Fillet/chamfer is the most-used CAD feature after extrude. In full generality
it is hard: an exact B-Rep fillet must roll a variable-radius ball along a
selected edge, build the blend surface, trim the two adjacent faces back to
the tangency curves, and stitch the topology — all while handling radius
overflow, edge chains that cross vertices, and setback corners. This note
describes the *robust fast path*: a localized F-Rep blend that needs none of
that surgery, and how it slots into the existing kernel.

## 1. The F-Rep idea

Booleans in this kernel are `min`/`max` on signed distance fields (see
`crates/opensolid-frep/src/csg.rs`). A sharp edge is exactly the locus where
the two operand fields are simultaneously zero — the kink in `min(a, b)`.

A **global** smooth blend already exists (`blend.rs`, `SmoothUnion` /
`SmoothSubtraction`): it replaces the kink everywhere the two surfaces are
within `radius`. That is not what a CAD user means by "fillet this edge" — it
rounds *every* edge the boolean produces, at one radius, with no way to leave
neighbours sharp or give edges different radii.

**Edge-selective** blending localizes the smooth min/max to the neighbourhood
of a chosen edge:

```
d(p)      = distance from p to the selected edge region (a polyline)
w(p)      = window(d(p))         # 1 on the edge, tapering to 0 past `influence`
r_eff(p)  = radius * w(p)        # full radius on the edge, 0 away from it
```

- **Fillet**: `smooth_min(a, b, r_eff(p))` — the polynomial smooth-min already
  used by `SmoothUnion`. `radius` **is** the fillet radius.
- **Chamfer**: the hg_sdf chamfer-min `min(min(a,b), (a+b−r_eff)·√½)`, which
  introduces a planar bevel instead of a round. `radius` is the chamfer
  setback.

Where `w = 0` the effective radius is 0 and the operator collapses back to the
sharp `min`/`max`, so untouched edges (and the rest of the same boolean's
intersection curve) stay crisp. This is implemented as one SDF combinator,
`EdgeBlend`, in `crates/opensolid-frep/src/fillet.rs`.

### Why this is robust

No topology surgery, no tangency solve, no trimming. The blend is just a
different scalar field; the existing adaptive mesher + `refine_mesh` recover
the boundary. Coincident faces and tangencies — the classic B-Rep fillet
failure modes — cannot fail here because there is no B-Rep to break.

### Metric / meshing correctness

The mesher relies on `eval_interval` being *conservative* and on `branches`
correctly reporting where the field is sharp. `EdgeBlend`:

- **`eval_interval`**: the blend only ever pulls the field away from the sharp
  value, and by at most `radius/4` (fillet) or `radius·√½` (chamfer), always in
  one direction. So `[sharp.lo − Δ, sharp.hi]` (union) contains the field
  regardless of the window — the same bound `SmoothUnion` uses, widened by the
  worst-case `r_eff = radius`. Pruning stays sound even though the windowed
  field is not globally 1-Lipschitz.
- **`branches`**: inside the influence region the surface is *smooth*, so
  `EdgeBlend` reports a single smooth branch there — otherwise `refine_mesh`
  would Newton-snap the filleted surface back onto the sharp intersection curve
  and undo the fillet. Outside the region it forwards the underlying boolean's
  branch decomposition, so sharp edges still snap exactly.

## 2. Edge selection representation (the hard part)

A fillet must name *which edge*. This is the persistent-naming problem: the
reference has to survive re-evaluation, parameter edits, and remeshing, none of
which preserve mesh vertex indices or triangle IDs.

### What an edge *is* here

An edge produced by a boolean between analytic primitives is a segment of the
analytic intersection curve of two operand surfaces. So an edge is identified
by **(the boolean node, the pair of operand surfaces, a spatial locator)**:

- **Boolean node**: which `union`/`intersect`/`subtract` in the CSG tree. The
  scene tree (`web/.../src/lib/sceneTree.js`) already assigns each op a stable
  `type:ordinal` feature key; the fillet stores that key.
- **Operand surfaces**: the two children of that node. Their fields `a`, `b`
  are exactly what `EdgeBlend` needs.
- **Spatial locator**: *which segment* of the intersection curve. A boolean
  can produce a curve that wraps around (multiple CAD "edges"). We locate the
  selected segment by a **seed point** on the edge (from the pick ray) plus the
  polyline the mesher's CSG-edge detection already produces (`refine.rs`
  Newton-snaps feature vertices onto the analytic curve — those crease-labeled
  vertices, chained, *are* the edge polyline).

### Stability across edits

The stored reference is therefore `{ boolean_key, seed_point, radius, mode }`,
**not** a vertex/triangle index. On re-evaluation we:

1. Resolve `boolean_key` → the two operand fields (structural, survives param
   edits as long as the op still exists).
2. Re-extract the edge polyline near `seed_point` from the current mesh's
   crease vertices (geometry moves with the parameters; the seed tracks it as
   long as the edge doesn't disappear).

This is the F-Rep analogue of Parasolid/OCC persistent naming: name by
*generating operation + geometric anchor*, re-resolve by proximity. It
degrades gracefully — if an edit deletes the edge, resolution finds no nearby
crease and the fillet is flagged stale rather than corrupting the model.

MVP note: the kernel `EdgeBlend` takes the resolved polyline directly (a
`Vec<[Point3; 2]>` of segments). Seed→polyline resolution and stale detection
live in the GUI/session layer; the kernel stays purely geometric and testable.

## 3. Multiple edges

Two orthogonal cases:

- **Same radius, several edges**: one `EdgeBlend` node whose `EdgeRegion` holds
  all the selected segments. `d(p)` is the min distance over every segment, so
  the window covers the union of edges. Cheap — one combinator.
- **Different radii / mixed fillet+chamfer**: separate `EdgeBlend` nodes,
  chained. Each localizes to its own region, so they compose without
  interfering *provided their influence regions don't overlap*. Overlapping
  blends of different radii is a genuine CAD ambiguity (setback corners); the
  MVP does not attempt corner setbacks — overlapping regions simply superimpose
  their fields, which is acceptable for well-separated edges and flagged in the
  UI when regions are close.

Ordering: because each node only perturbs its own neighbourhood, fillet nodes
are order-independent to first order. Corner blends (three edges meeting) are a
known follow-up.

## 4. Radius limits

A fillet radius is not free:

- **Geometric ceiling**: the radius cannot exceed the local feature size — the
  distance to the opposite wall, or to the next edge, or the reciprocal of the
  concave curvature being filleted. Past that the rolling ball no longer fits
  and the F-Rep field self-intersects (the smooth-min pulls through the far
  surface). We clamp `radius > 0` and document that the caller is responsible
  for staying under the local feature size; a `max_radius` estimator (sample
  the field along the edge normal to the nearest opposing surface) is a
  follow-up.
- **Window vs radius**: `influence` must exceed `radius` so the full fillet
  cross-section (which bulges ~`radius` off the edge) sits inside the
  full-weight band. MVP uses `influence = 2·radius` with a smoothstep taper
  over `[radius, 2·radius]`.
- **Numerical floor**: for `r_eff` below a small epsilon we short-circuit to
  the sharp op (also required for the chamfer formula, which does *not* reduce
  to `min` at `r = 0`).

Degenerate/oversize radii do not crash — worst case the mesh shows a visibly
wrong bulge, which is recoverable by lowering the radius, unlike a B-Rep
fillet that would hard-fail.

## 5. GUI flow (playground)

1. **Pick**: click near an edge in the viewport. Existing face-region picking
   (`web/.../src/lib/facePlane.js`) grows coplanar triangle regions; an edge is
   the crease between two adjacent regions. The click ray → nearest crease →
   seed point + the two bounding face regions (hence the two operand surfaces).
2. **Panel**: a small fillet/chamfer panel — mode toggle, radius input, live
   preview. Preview re-emits the script with a `.filletEdge(...)` /
   `.chamferEdge(...)` call and remeshes (the playground already remeshes on
   every edit via `meshAdaptive`).
3. **Persist**: the call is written into the script (the single source of
   truth), storing `{ seed, radius, mode }`; re-evaluation re-resolves the edge.

MVP delivers the kernel + WASM binding + script API; the full click-to-panel
interaction may land incrementally (tracked separately if not in this change).

## 6. Out of scope / follow-ups

- **Exact B-Rep blends** (rolling-ball surface, trim, stitch): the precision
  path for engineering surfaces. Explicitly deferred (follow-up bead).
- **Variable-radius and conic fillets**, **setback corners** where 3+ filleted
  edges meet, **face-face and full-round blends**.
- **`max_radius` estimator** and automatic radius clamping.
- **Edge chains crossing primitive-intrinsic edges** (e.g. the 12 edges of a
  `Box3` primitive, which are not boolean edges) — needs primitive-level edge
  identification, separate from this boolean-edge MVP.
