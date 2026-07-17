# Friction log: building a real part through the MCP agent interface

One engineer-agent, one prompt, one part: the right-angle bracket in
[`bracket-right-angle.md`](../tools/mcp-server/examples/agent-gallery/bracket-right-angle.md)
— 60×40×5 base plate, 40×40×5 vertical plate, triangular gusset, 4× M5 holes,
3 mm fillets. Built end to end over the **real MCP stdio transport** (spawn
`src/server.js`, speak JSON-RPC), the way an actual client would, not by calling
the tool handlers in-process.

The part shipped. This is what it cost, in the order it hurt. Every item is a
filed bug; this document exists so the *pattern* across them is visible, which
no individual bead shows.

## The headline

**The agent interface has no working correctness oracle for a part with a hole
in it.** Not one. That is the thread connecting almost everything below:

| Oracle | What it did on this part |
|---|---|
| `valid` / `validate` | `true` for a bracket whose holes were bored sideways through it |
| screenshot | rendered that same wrong part plausibly |
| `boundingBox` | reported 61.5 × 41.5 × 41.5 for a 60 × 40 × 40 part |
| `measure` volume | `null`, four times, with no reason given |
| STEP export | declined on a box with one through-hole |

The only thing that actually caught the bug was a volume compared against a
number computed by hand, on paper, from the spec. An agent that trusts the
tools' own reports builds wrong parts confidently. That is the finding.

## 1. The docs point the drill the wrong way — `of-4tu` (P1)

`docs/AGENT_GUIDE.md` says `Shape.cylinder` is "axis +Z" and `Shape.extrude`
sweeps "along +Z". Both are **+Y** (`primitives.rs:255-258` takes the radial
term from x/z and the axial from y; `bounded.rs:184` sweeps y from `0..height`
and maps profile `(u,v) → (x,z)`).

So the first bracket was built with four holes bored lengthwise *through* the
plates. It reported `valid: true`. It rendered fine. Nothing errored. The only
symptom was a volume ~800 mm³ light, which is why the discrepancy got chased at
all.

This is not a private mistake — it is **shipped in the gallery**. The
`angle-bracket` example's "four Ø6 mounting holes in the base" are Y-axis
channels, and its own published volume (18032 vs 18747.6 analytic) records the
error. `hinge-leaf` narrates "a cylinder whose default +Z axis I rotate onto +X"
via `rotate(0,1,0,90)` — rotating a +Y cylinder about Y is a no-op, so its
knuckles are wrong too. The docs taught the mistake, and the examples that
demonstrate the docs encode it.

Worth deciding deliberately: +Y-up is unusual, and it cuts directly against
STEP/FreeCAD/CAD-at-large, which are z-up. Whatever the answer, say it once,
loudly. The current state — docs say one thing, kernel does another, renderer
agrees with the kernel, STEP output implies a third — is the worst of all
worlds.

## 2. Faceted STEP declines on a plate with a hole — `of-obv` (P1)

```js
Shape.box3(30,20,2.5).subtract(Shape.cylinder(2.5,10).rotate(1,0,0,90).translate(15,0,0))
```

A 60×40×5 plate, one Ø5 through-hole. `valid: true`, STL fine, STEP:

> `sdf_to_brep: adaptive meshing did not produce a closed manifold; the surface
> must lie strictly inside the meshing bounds`

Three things are wrong here.

**The message is misleading.** `export_step` already pads via `mesh_bounds(64)`
— 10% of max extent, ~6 mm. The surface *is* strictly inside. That string is
hardcoded for any non-manifold result (`sdf_to_brep.rs:140-145`) and sent the
investigation the wrong way for a while.

**It is grid alignment, not geometry.** Two identical parts, differing only in
how the last transform is *spelled*:

```js
box3(30,2.5,20).subtract(cyl)                      // STEP FAIL
box3(30,2.5,20).rotate(0,1,0,360).subtract(cyl)    // STEP ok
```

`rotate(…,360)` is the identity. All it changes is the tracked AABB, which
shifts the octree grid. Nor is "looser" a fix: the bracket rotated upright has a
*much* looser box and still fails; eight identity-equivalent spellings of the
upright part were tried and all eight failed.

**Two meshers disagree and the agent cannot see it.** `valid` comes from the
uniform mesher; STEP goes through `mesh_sdf_adaptive_indexed`; STL is a third
path. "valid + STL ok + STEP declines" is the signature, and there is no knob
to reconcile them — see `of-2i8`.

The gallery documents this as a thin-feature limitation of `gear-disk`. It
isn't. It reproduces on a box. A plate with a through-hole is the most common
feature in mechanical CAD, and faceted STEP is the *only* STEP path for any part
carrying a fillet.

**The shipped bracket script ends in `.rotate(0, 1, 0, 360)` purely to perturb
its bounds.** It is the only spelling found that exports. That line should not
survive the fix, and it is in the transcript as a standing reminder.

## 3. `boundingBox` is not the bounding box — `of-b06` (P2)

It is the conservative *tracked meshing box* (`lib.rs:1077`). `smoothUnion` pads
it by `radius/4`; `rotate` takes the AABB of the rotated corners of the previous
AABB, so error compounds. `cylinder(2.5,10).rotate(1,0,0,90)` reports y ±6.72
where the truth is ±2.5. The upright bracket reports 55.7 for a 40 mm dimension.

It reads exactly right for `box3`/`cylinder` unions — which is all the existing
examples do — so it looks trustworthy until the first blend, then quietly lies
by up to ~40%. The bracket transcript has to tell readers not to believe it.

## 4. `volume: null`, no reason — `of-ysr` (P2)

`measure` returned `{"volume": null, "exact": false}` at default, 0.2, 0.1, and
0.05 accuracy, then `19750.08` at 0.02. Same model, same tool. `massError`
exists in the kernel and explains why; it never reaches the agent. A bare `null`
reads as a broken model rather than a meshing-resolution artifact, and cost real
time.

## 5. No `accuracy` on `export` — `of-2i8` (P2)

`export_step(inner, exact, accuracy, unit)` takes one. `measure` and `validate`
both expose one. `export` does not. So the obvious lever for both diagnosing and
working around `of-obv` is absent — and faceted STEP files cannot be made
smaller. This bracket's STEP is **20.8 MB** (hinge-leaf 11.2, angle-bracket
6.5), all committed to the repo, when a coarse export would serve most uses.

## 6. Volume identity is unverifiable where it matters — `of-fc8` (P2)

The acceptance bar was "STEP re-imports through our reader with volume
identity". The file re-imports as a clean exact B-Rep. It cannot be **measured**:
`tessellate_body` refuses planar faces with hole loops ("needs constrained
triangulation"), which every drilled plate has.

`step_corpus.rs` already concedes this — `closed_volume()` returns `Option` and
the gate *skips* the volume comparison when tessellation fails. So round-trip
volume is silently unchecked on precisely the bodies most likely to be wrong.

The bracket's gate does what can be done today: pin the exported body against
the hand-computed section, then require `write ∘ read` to be a fixed point —
**numerically**, not bytewise, because the reader re-normalizes `DIRECTION`
vectors and moves scattered facet normals by ~1 ULP. (Only the exact path is
byte-identical; the existing exact test's idiom does not transfer.)

## 7. Duplicated issue strings — `of-6t8` (P3)

`issues` comes back as the same sentence twice. Reads as two defects; it is one.

## What was left undone

**FreeCAD import is unverified.** FreeCAD is not installed on this machine and
the acceptance criterion says to document manual verification — so this is the
open item, not a passed check. `tools/mcp-server/examples/output/bracket-right-angle.step`
(20.8 MB, faceted, mm units, z-up) is committed and ready for someone to open.
The reader-side round-trip is covered by
`bracket_faceted_step_round_trips_with_volume_identity`.

## What guards this now

- `tools/mcp-server/test/bracket-acceptance.test.js` — drives the real stdio
  server; asserts closed solid, volume against the analytic section, **the four
  holes remove ~392.7 mm³**, renders, and both exports. Mutating the axes back
  to the docs' "+Z" fails 4 of its 5 subtests.
- `crates/opensolid-wasm/src/step.rs::bracket_faceted_step_round_trips_with_volume_identity`
  — the Rust half: same part via `BoundedShape`, faceted STEP, clean re-import,
  numeric fixed point.

The hole-volume assertion is the one that matters. It is the only test in the
repo that would have caught `of-4tu`, and it exists because a hand calculation
disagreed with every oracle the tools offered.
