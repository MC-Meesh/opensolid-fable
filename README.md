# OpenSolid

An open CAD kernel with a **hybrid F-Rep + B-Rep** architecture, written in Rust.

OpenSolid combines two geometry representations that are usually treated as
rivals:

- **F-Rep** (functional / implicit) — a solid is a signed distance field
  (SDF): a function `f(p)` that is negative inside, positive outside, zero on
  the boundary. See `crates/opensolid-frep/src/primitives.rs:6` (the `Sdf`
  trait).
- **B-Rep** (boundary representation) — a solid is an exact topological graph
  of faces, edges, and vertices bound to analytic surfaces and curves. See
  `crates/opensolid-brep/src/topology.rs` (`Body → Shell → Face → Loop → Fin →
  Edge → Vertex`).

## Why hybrid

The two representations have complementary failure modes:

| | F-Rep (SDF) | B-Rep (analytic) |
|---|---|---|
| Booleans | `min`/`max` on distance fields — **cannot fail** | surface intersection + topology surgery — **fragile** |
| Accuracy | approximate (meshed at a grid resolution) | **exact** |
| Best for | robust CSG, organic blends, offsets/shells | precision engineering surfaces, interchange |

A boolean in classical B-Rep kernels is the hardest thing they do: it requires
robust surface–surface intersection, exact classification, and topology
reconstruction, and it hard-fails on coincident faces, tangencies, and
near-degenerate contacts. F-Rep booleans, by contrast, are `min`/`max` of two
distance fields (`crates/opensolid-frep/src/csg.rs:19`) — they are trivially
robust but only as accurate as the grid you mesh them on.

OpenSolid runs **exact-when-possible with automatic F-Rep fallback**, so a
boolean **never hard-fails**. The exact analytic pipeline is tried first; if it
errors, produces a bad mesh, or produces a *silently wrong* result, the
operation falls back to converting both operands to SDFs and doing the
trivially-robust CSG. The entry point is `hybrid::boolean`
(`crates/opensolid-kernel/src/hybrid.rs:331`).

This is not marketing. Every claim below cites the file that implements it.

---

## Architecture

### Crate map

```
crates/
├── opensolid-core/     Points, vectors, transforms, bounding boxes, intervals,
│                       tolerance context, generational arena, BVH, triangle mesh.
├── opensolid-frep/     F-Rep: SDF primitives, CSG (min/max), smooth blending,
│                       offset/shell ops, uniform-grid + adaptive-octree meshers.
├── opensolid-brep/     B-Rep: analytic curves/surfaces, NURBS, SSI, topology
│                       graph, Euler operators, the exact boolean pipeline.
├── opensolid-kernel/   Unified: hybrid booleans, F-Rep↔B-Rep conversion,
│                       mass properties, STL/OBJ IO, session with undo/redo.
└── opensolid-wasm/     wasm-bindgen surface for the browser playground.

web/playground/         React 18 + Vite SPA: script → mesh in WASM → three.js
                        viewport → download STL.
```

Dependency direction: `core ← frep`, `core ← brep`, and `kernel` sits on top of
all three (`crates/opensolid-kernel/src/lib.rs:11-13`). Runtime dependencies are
deliberately minimal — `nalgebra`, `thiserror`, `rayon` only (`Cargo.toml`);
`criterion` and `proptest` are dev-only.

### Data flow

```
  ┌─────────────┐         ┌──────────────────┐        ┌─────────────┐
  │  F-Rep       │         │   B-Rep           │        │  Mesh /      │
  │  Shape (SDF) │         │   TopologyStore   │        │  TriangleMesh│
  └──────┬──────┘         └────────┬─────────┘        └──────┬──────┘
         │                          │                          │
         │  sdf_to_brep  ◄──────────┤                          │
         │  (octree DC + planar     │  tessellate_body         │
         │   region recovery)       │  ───────────────────────►│
         │                          │                          │
         │  ──────────────────────► │  brep_to_sdf (MeshSdf)    │
         │      MeshSdf::new         ◄─────────────────────────┤
         │                          │                          │
         ▼                          ▼                          ▼
     mesh_sdf_indexed          hybrid::boolean            write_stl / write_obj
     (dual contouring)         (exact ▸ validate ▸        mass_properties
                                fallback)
```

Conversion both ways: `MeshSdf` wraps a closed triangle mesh as an SDF
(`crates/opensolid-kernel/src/convert/brep_to_sdf.rs`), and `sdf_to_brep`
recovers a faceted B-Rep body from a field
(`crates/opensolid-kernel/src/convert/sdf_to_brep.rs`).

### The `HybridBoolean` entry point and its gate ladder

`hybrid::boolean(op, a, b, opts)` (`crates/opensolid-kernel/src/hybrid.rs:331`)
takes two operands, each either F-Rep or B-Rep (`HybridBody`, `hybrid.rs:71`),
and returns a `HybridBoolean` (`hybrid.rs:246`) that **always** carries a
watertight mesh plus whichever richer representation won (`HybridPath`,
`hybrid.rs:221`).

The exact B-Rep path is taken only when **both** operands are B-Rep bodies in
the **same** store pair (`hybrid.rs:343-357`). It must then clear every rung of
this ladder before its result is kept — any shortfall diverts to the F-Rep
fallback:

1. **`Ok`** — the exact analytic pipeline (`opensolid_brep::boolean`) returns a
   result, not a `NotImplemented`/`Degenerate` error (`hybrid.rs:358-368`).
2. **Tessellates** — `out.tessellate_measured()` succeeds, returning the mesh
   and its worst chordal deviation (`hybrid.rs:369`).
3. **Closed manifold** — `mesh.is_closed_manifold()` (`hybrid.rs:371`).
4. **Chordal deviation ≤ one grid cell** — the mesh's deviation from the
   analytic surfaces must not exceed the error the F-Rep fallback would itself
   commit (`cell_size`, `hybrid.rs:558`); otherwise the fallback is *more*
   accurate and wins (`hybrid.rs:372`).
5. **`check()` passes** — the result body passes the full topology checker
   (`hybrid.rs:427`, via `BooleanOutput::check`,
   `crates/opensolid-brep/src/boolean.rs:300`).
6. **Volume cross-check** — the result's enclosed volume (`mass_properties`)
   must agree with a coarse F-Rep grid estimate of the same boolean to within
   5% by default (`validate_exact`, `hybrid.rs:441-448`; `ValidationOptions`,
   `hybrid.rs:173`).

Only a result that passes **all six** is accepted as `HybridPath::Brep` with its
exact topology intact. Anything else takes `frep_fallback` (`hybrid.rs:491`):
both operands become SDFs, the operation becomes `min`/`max` CSG, and the
combined field is dual-contoured back into a mesh. When rungs 5–6 (the runtime
*validation gate*) reject an otherwise-clean exact result, the reason is
recorded in `HybridBoolean::diagnostic` (`ValidationDiagnostic`,
`hybrid.rs:200`).

The validation gate exists because the exact pipeline can return `Ok` with
**geometrically wrong** faces that still tessellate to a closed, manifold,
chord-faithful mesh — the mesh-quality rungs (2–4) cannot see that. See
`hybrid.rs:31-45` and the Hard-Won Lessons below.

---

## The boolean pipeline

The exact B-Rep pipeline (`crates/opensolid-brep/src/boolean.rs`) combines two
transversal solids in one `TopologyStore`/`GeometryStore` pair. `unite`,
`subtract`, and `intersect` (`boolean.rs:357`, `:368`, `:379`) all delegate to
the private driver `boolean()` (`boolean.rs:787`), whose whole flow is six calls
(`boolean.rs:795-801`):

**1. Clash (broad phase).** Each solid's faces are indexed in a `Bvh` over
dilated bounding boxes; candidate face pairs come from a dual-tree
`overlap_pairs` descent (`find_imprints`, `boolean.rs:859-871`, using
`Bvh::overlap_pairs`).

**2. SSI (surface–surface intersection).** Each candidate pair is intersected
analytically into exact `Curve3` geometry (`ssi::intersect`,
`crates/opensolid-brep/src/ssi/analytic.rs:90`). The analytic kernel handles
plane×plane, plane×sphere, plane×cylinder, plane×cone, plane×torus, and
equal-radius cylinder×cylinder (Steinmetz); other pairs return
`NotImplemented` (`analytic.rs:96-111`). A separate numeric marcher for NURBS
surfaces exists (`ssi/marching.rs:435`) but is not wired into the boolean
driver yet.

**3. Imprint.** Intersection curves are clipped to the trimmed regions of
*both* faces (`clip_imprint`, `boolean.rs:921`), then imprint curves and
original edges are split at their mutual meeting points **globally** — one
canonical 3D split set per curve (`collect_splits`, `boolean.rs:1072`) — so
both sides of every future shared edge agree exactly. For a closed imprint ring
on a cylinder face, a **seam meridian cut** splits the ring where it crosses the
cylinder's seam (`seam_crossing`, `boolean.rs:1557`, refined against the exact
curve by `refine_seam_point`, `boolean.rs:1623`) so the periodic parameter cover
sees the ring as a boundary-to-boundary chord.

**4. Region splitting on the universal cover.** Each face's parameter-space
arrangement of boundary and imprint polylines is traced into regions
(`reconstruct` → `apply_chain`, `boolean.rs:1167`, `:1984`). Imprint chains that
reach the outer boundary split a region along a **chord**; chains that close on
themselves carve a **ring** (interior disk + hole). Cylinder charts are periodic
(`period = TWO_PI`, `boolean.rs:1992`); `localize_to_window` (`boolean.rs:1945`)
shifts polylines by whole periods so a seam-wrapping region and its holes share
one cover window.

**5. Ray-parity classification.** Each region is classified inside/outside the
*other* solid by casting a ray from an interior sample point and counting
surface crossings — interior iff the parity is odd (`contains_point`,
`boolean.rs:1266`, `hits % 2 == 1` at `:1303`). Six jittered ray directions
(`RAY_DIRECTIONS`, `boolean.rs:100`) dodge grazing/on-boundary degeneracies. A
per-operation keep table decides which regions survive (`keep_table`,
`boolean.rs:1327`).

**6. Shell reconstruction.** Kept regions sharing an atom edge are merged into
shells by **union-find** (`build_output`, `boolean.rs:2245`, union-find at
`:2257-2280`). Each shell's **genus** comes from the Euler–Poincaré
characteristic: `χ = V − E + F − R`, then `genus = 1 − χ/2`
(`shell_genus_from_euler`, `boolean.rs:2523`; `shell_counts`, `:2533`). The
resulting body carries a validated `TopologyStore` and a tessellation payload.

**Transversal only.** Coincident faces, tangent contacts, and single-point
tangencies are rejected with a structured `NotImplemented`
(`boolean.rs:33-38`) — exactly the cases the hybrid fallback rescues. Face
charts today are **plane and cylinder**; cone/sphere/torus charts return
`NotImplemented` (`Chart::new`, `boolean.rs:411-444`).

---

## Hard-won lessons

These are real bugs found and fixed, distilled from the closed issue tracker.
They are the reason the validation gate and stress suite exist.

**1. Refine seam crossings against the exact curve, not fixed sampling
(of-k3u).** Circular edges were sampled at a fixed 96 points and seam crossings
linearly interpolated between samples, so a crossing vertex sat on a chord, off
the true curve by up to the sagitta `r·(1−cos(π/96)) ≈ 5.35e-4·r`. The pipeline
honestly recorded that gap as the edge tolerance — and above radius ≈ 19 it
exceeded the checker's absolute tolerance cap, so a *geometrically fine* boolean
failed its own `check()`. Axis-aligned unit tests missed it because their
imprint samples land phase-aligned with the seam. Fix: `seam_crossing` now
bisects against the exact imprint curve (`boolean.rs:1557`); chord interpolation
survives only as a degenerate-bracket fallback.

**2. UV metrics must not mix radians and lengths (of-9n8).** On a cylinder chart
`u` is an angle (radians) and `v` is a length (model units), but several
computations treated `(u,v)` as an isotropic metric. On a tall, large-radius
band a chain endpoint could false-match a wrong cycle vertex (an `ε` of 0.01
covers 10 units of arc at `r=1000`), and a thin sliver region could put every
interior probe outside and return `Degenerate`. Fix: scale `u` by the radius
(arc-length metric) and probe interior points with per-axis, extent-normalized
offsets. There were three distinct sites of the confusion.

**3. Scale from feature extent, not distance to the origin (of-260).** The
snapping tolerance was derived from `max |p.norm()|` over sampled points, so
`snap = 1e-9·scale` grew with distance from the origin, not feature size.
Micrometer features one meter from the origin already failed classification.
Fix: `geometric_snap()` derives the tolerance from the joint bounding-box extent
(feature size), floored at 100 ULPs of the largest coordinate so welding never
demands sub-`f64` merges far from the origin.

**4. Ear-clip with collinear-run deferral, not fan triangulation
(of-6dw, of-6sq).** Faces were triangulated with a centroid/first-vertex fan,
which is wrong for any non-star-shaped profile (U/S/C shapes) — yet it passed
`is_closed_manifold` and volume checks because signed volume cancels and every
edge is still used twice. Fix (of-6dw): a standalone O(n²) ear-clipper
(`crates/opensolid-brep/src/triangulate.rs`). Refinement (of-6sq): collinear
vertices are **deferred** rather than clipped while collinear, so no zero-area
slivers are emitted in the first place.

**5. Zero-area slivers pass the mesh checks but the SDF bridge rightly rejects
them (of-ipt.9).** A sliver triangle keeps a mesh combinatorially closed (every
edge still used twice, signed volume cancels), so `is_closed_manifold` and
volume checks wave it through. But `MeshSdf::new` rejects any triangle with
`2·area ≤ 1e-12·longest²`
(`crates/opensolid-kernel/src/convert/brep_to_sdf.rs:115`), because a degenerate
triangle has no reliable normal for an SDF. The right fix is to **never emit**
slivers (lesson 4's collinear deferral), not to weld them after the fact —
welding at any epsilon does not restore manifoldness.

**6. Adversarial stress testing found bugs unit tests structurally could not
(the of-ipt.* campaign).** The 2500-line boolean pipeline had ~10 tests, all
axis-aligned blocks and cylinders, written by the author of the code. A
randomized stress suite (`crates/opensolid-brep/tests/`) found roughly seven
severe bugs across ~five root causes — including the canonical through-hole
config that passed `check()`, tessellated to a closed manifold, and reported the
correct face/shell counts and genus while removing **~12× too much volume**
(of-ipt.4). The lesson: **face counts and genus are insufficient acceptance
criteria.** The real bar is a volume identity — the inclusion–exclusion law
`vol(A) + vol(B) = vol(A∪B) + vol(A∩B)` — holding to `1e-9`. That identity is
now a hard gate; it is also the sixth rung of the hybrid ladder above.

---

## Validation philosophy

**Stress-suite-first promotion.** A new surface class does not enter the exact
boolean pipeline until its randomized stress suite (rotations, scales, volume
identities, round-trips) is green. Until then it routes through the F-Rep
fallback, which already works. This is the rule that carried spheres and tori
into the exact pipeline (bead of-7ld) and still gates cones.

**Runtime validation gate.** Correctness is not assumed at build time — it is
re-checked at *run* time. Every accepted exact result passes `check()` and a
volume cross-check against an independent F-Rep estimate before it is kept
(`validate_exact`, `crates/opensolid-kernel/src/hybrid.rs:418`). A silently
wrong result is discarded and the operation quietly produces the robust
fallback answer instead.

**`check()` hardening.** The topology checker
(`crates/opensolid-brep/src/check.rs`) verifies structural invariants, tolerance
bounds, and the Euler–Poincaré relation `V − E + F − R = 2(S − H)`
(`crates/opensolid-brep/src/euler.rs:103`). Bugs found in the field become new
checker rules, so the same class of error cannot silently return `Ok` twice.

**Every function is tested.** 1184 tests pass across the Rust workspace
(`cargo test --workspace`), plus the playground's vitest suite. CI runs `fmt`,
`clippy -D warnings`, `build`, and `test` on every push
(`.github/workflows/ci.yml`). Eight tests are `#[ignore]`d: three are on-demand
perf measurements, and five are known-broken cases held as executable bug
reports — each names the open bead blocking it (of-s89, of-9ia, of-kb8), per
the stress-suite-first policy of never softening a test to make it pass.

---

## Status & roadmap

| Capability | Exact B-Rep path | F-Rep path |
|---|---|---|
| Planes | ✅ today | ✅ |
| Cylinders | ✅ today | ✅ |
| Spheres / tori | ✅ today (of-7ld) | ✅ |
| Cones | ✅ today (of-dtj); non-coaxial cone–cone → of-9ia | ✅ |
| Coincident / tangent contacts | rejected → fallback | ✅ |
| Organic blends, offsets, shells | — | ✅ |
| STEP (AP203) read/write | ✅ today (of-3qy) | mesh fallback on read |

The exact analytic pipeline covers **plane, cylinder, sphere, torus, and
cone** faces today: the sphere/torus stress campaign (bead of-7ld) promoted
both classes through the stress-suite-first policy, and marched SSI curves
carry the oblique plane–torus and torus–torus configurations. **Cones** have
since been promoted the same way (of-dtj): every **plane–cone** boolean is
exact — the parabola/hyperbola/generator sections that arise when a planar
face cuts a cone off-axis march through the bounded SSI entry point
(of-dtj.1) — sphere–cone / torus–cone pairs march against their compact
partner (of-dtj.2), and **coaxial cone–cone** overlaps take the exact path
through the analytic cone–cone SSI. The one remaining cone gap is
**non-coaxial cone–cone**: the SSI itself marches correctly, but hosting the
marched imprint on the two curved cone faces leaves an open chain (of-9ia), so
those configurations fall to the F-Rep path. **STEP (AP203)
interchange** shipped (bead of-3qy): analytic parts round-trip through
`write_step`/`read_step` as exact B-Reps — byte-identical on re-export for
primitive-derived geometry — with a welded-mesh fallback for files the kernel
cannot yet represent exactly, unit scaling from the file's declared length
unit, and cross-feature integration tests chaining STEP with exact booleans,
hybrid booleans, and sweeps
(`crates/opensolid-kernel/tests/integration_e2e.rs`). Meanwhile the F-Rep
fallback covers **everything** — any pair of valid inputs produces a
watertight result, because `min`/`max` on distance fields cannot fail.

See `ROADMAP.md` for the epic-level plan; the beads tracker (`bd ready`) is the
source of truth for task state.

---

## Quickstart

### Build & test (Rust)

```sh
cargo build            # build the workspace
cargo test             # run all 1184 tests
cargo clippy -- -D warnings
```

### Run the demo

Builds a box smooth-unioned with a sphere bump, drills a cylindrical hole,
meshes it watertight, and writes `demo.stl` + `demo.obj`
(`crates/opensolid-kernel/examples/demo.rs`):

```sh
cargo run -p opensolid-kernel --example demo
```

### Web playground

An interactive browser playground: edit a JS script that builds a shape with the
`opensolid-wasm` API, mesh it in WASM, orbit it in a three.js viewport, and
download binary STL.

Prerequisites: Node 20+, the Rust wasm target
(`rustup target add wasm32-unknown-unknown`), and
[wasm-pack](https://rustwasm.github.io/wasm-pack/) (`cargo install wasm-pack`).

```sh
cd web/playground
npm install
npm run wasm      # build crates/opensolid-wasm into pkg/ (rerun after Rust changes)
npm run dev       # Vite dev server at http://localhost:5173
```

`npm run wasm` is a required setup step — `pkg/` is generated build output and
is not checked in. See `web/playground/README.md` for the full UI reference
(feature tree, sketch mode, view navigation) and a no-wasm-pack fallback.

---

## AI-first CAD

OpenSolid is designed to be driven by an agent, not just a mouse. An
[MCP](https://modelcontextprotocol.io) server exposes the kernel as a handful of
tools — `create_model`, `get_screenshot`, `measure`, `validate`, `export` — so
any MCP-capable agent (Claude, etc.) becomes a **headless CAD operator**: it
writes a script, gets back mesh stats and a validity report, renders
screenshots, checks mass properties, and exports STEP/STL/OBJ, with no GUI in the
loop. The kernel is the same wasm build the playground runs, so anything an agent
builds opens unchanged in the browser.

- **[Agent Guide](docs/AGENT_GUIDE.md)** — connect a client, the tool reference,
  the script API, and every failure mode with how it's reported.
- **[Agent gallery](tools/mcp-server/examples/agent-gallery/)** — five worked
  transcripts (bracket, hinge, enclosure, gear, bottle), each real unedited
  output from the server: prompt in, manufacturable part out.
- **[MCP server](tools/mcp-server/)** — the server itself, setup, and tests.

```bash
cd tools/mcp-server && npm run build
claude mcp add opensolid -- node "$PWD/src/server.js"
```

---

## Repository layout

```
crates/            The five Rust crates (see Architecture)
web/playground/    React + Vite browser playground
tools/mcp-server/  MCP server + agent gallery (see AI-first CAD)
research/          Landscape analysis (read-only reference)
spec/              v1 spec — Parasolid mapping, tolerance philosophy, targets
                   (the v1 crate structure predates the hybrid pivot; ignore it)
ROADMAP.md         Living plan, mapped to the beads tracker
```

## License

MPL-2.0 (`Cargo.toml`).
