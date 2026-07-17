# Coincident and tangent faces in booleans

Status: design note (of-bxl.1, phase 0 of of-bxl)
Scope: strategy only — no code changes. Implementation is phased into
child beads (§8).

Union of two boxes that touch is the most common real CAD operation the
exact B-Rep pipeline cannot do. It is `NotImplemented` today
(`boolean.rs:1519`), and the kernel quietly re-runs the operation as
approximate F-Rep CSG (`hybrid.rs:331`). The user gets a mesh-derived
answer where they asked for an exact one. This note explains why, weighs
three strategies, and picks one.

## 1. What actually fails

The rejection is **at the wrong level of the model**.

`Pipeline::find_imprints` (`boolean.rs:1499`) walks BVH-overlapping face
pairs and calls analytic SSI on the pair's *surfaces*. For two planes,
`plane_plane` (`ssi/analytic.rs:139`) decides:

```rust
if tol.vectors_parallel(n1, n2) {
    return if tol.approx_zero(n1.dot(&(o2 - o1))) {
        SurfaceIntersection::Coincident     // ssi/analytic.rs:156
    } else {
        SurfaceIntersection::Empty
    };
}
```

That is a statement about two *infinite* planes. It never consults the
trimmed regions the faces actually occupy. `find_imprints` then aborts
the entire boolean on it (`boolean.rs:1518-1523`).

The consequence is worse than "coincident faces are unsupported":

- Two boxes stacked face-to-face — coplanar faces, regions **overlap**.
  Genuinely needs new machinery.
- Two boxes side by side, sharing only an edge — coplanar faces, regions
  **disjoint**. Nothing to do; the boolean is ordinary transversal work.
- Two boxes on a common baseplate, far apart in x — their bottom faces
  are coplanar and their bounding boxes may still overlap. Regions
  disjoint. Also nothing to do.

All three abort identically. The last two are pure false positives: the
transversality gate is applied to surfaces, but transversality is a
property of *faces*. A large share of the blocked operations need no new
geometry at all — only the question asked in the right place.

Four other analytic pairs reach the same `Coincident` verdict the same
way: `cylinder_cylinder:482` (coaxial, equal radius), `coaxial_profiles:586`
(the shared coaxial helper), `sphere_sphere:654` (concentric, equal
radius), `cone_cone:980`. Whatever we build must key off the enum
variant, not off planes specifically.

## 2. The second problem: classification is binary

Even with overlap regions in hand, the classifier has nowhere to put the
answer. There is no verdict enum. Classification is a `bool`:

```rust
fn contains_point(&self, s: SolidTag, p: &Point3) -> CoreResult<bool>  // boolean.rs:2386
fn keep_table(op: BooleanOp, solid: SolidTag, inside_other: bool) -> (bool, bool)  // boolean.rs:2448
```

A face region lying *on* the other solid's boundary is neither in nor
out. Today such a point is not classified — it is *evaded*.
`contains_point` abandons a ray direction when the sample sits on a
surface (`:2397-2405`), when incidence is grazing (`:2410`), or when a
hit lands near a face boundary (`:2418`). If all six `RAY_DIRECTIONS`
are exhausted it returns `CoreError::Degenerate` (`:2426`). An
on-boundary point is rescued by luck or it is an error.

So an ON verdict is not a tweak to one function. It is a three-place
change: the verdict type, `contains_point`, and `keep_table`.

## 3. Option (a) — imprint-and-merge

Classify the coplanar overlap in 2D, imprint its boundary onto both
faces, and let the existing reconstruction drop the interior walls.

**Why this fits the existing pipeline unusually well.** The five stages
(`boolean.rs:1-48`) are extract → imprint → split → atomize →
classify/reconstruct. Only the imprint stage and the classify verdict
need to change; splits, atoms, and reconstruction are untouched.

And the imprint has a closed form we already have. For a coincident face
pair, the overlap region's boundary is made of *the other face's trim
edges*. Those edges already lie exactly in this face's surface — that is
what coincidence means. So the imprint curves are literally B's existing
`Curve3` edge geometry, and they can be fed to the existing clipper:

```rust
// today, for a transversal pair:
self.clip_imprint(&ic.curve, fa, fb, box_a, box_b);      // boolean.rs:1538
// coincident pair: same call, curves taken from fb's loops (and fa's, onto fb)
```

No new intersection code, no new curve type, no 2D polygon-boolean
library. `collect_splits`, `build_atoms`, `merge_imprint_chains`, and
`apply_chain` (`boolean.rs:2341-2343`) then produce the face regions
exactly as they do for transversal imprints. This is the single most
important property of this option: **the overlap arrangement is a
special case of the arrangement we already build, not a parallel path.**

> **Amended by of-bxl.4 (implementation).** The claim above holds for the
> *arrangement* — the four stages did produce the regions unchanged — but
> the sentence in §8's table row that "splits, atoms, and reconstruction
> are untouched" was **wrong about reconstruction**, and three gaps had to
> be closed before touching boxes would fuse. Each is worth stating here
> because each is invisible from design altitude:
>
> 1. **The clip is one-sided.** A partner's trim edge lies on the
>    *partner's own* region boundary, so `contains_for_clip` is a float
>    coin flip along its whole length there (its nudge rescue fires on
>    periodic axes, not on planes). Only the *host*'s trim may clip it, and
>    only the host may be cut by it — hosting it on the partner drives
>    `apply_chain` to split that region along its own outline. Hence
>    `ImprintKind::{Transversal, CoincidentEdge { host }}`.
> 2. **Boundary-lying runs must be dropped.** An imprint run lying entirely
>    along a host's own outline cuts nothing and splits off a zero-area
>    sliver with no interior sample. This is *not* coincidence-specific:
>    two cubes meeting face to face also make A's `x = 1` plane meet B's
>    `y = 0` plane **transversally** in a line that is already an edge of
>    both. Judged per-run, not per-station, so an imprint that merely
>    *ends* on a boundary — how every imprint anchors — is untouched.
> 3. **Coincident atoms must weld across solids**, and this is the one the
>    "reconstruction is untouched" claim missed entirely. `build_output`
>    partitions shells by union-find over **shared atom id** and dedups
>    edges by atom id. A's and B's coincident boundary edges are *distinct
>    atoms at the same place*, so A's five surviving faces never union with
>    B's: each group closes into its own open shell and the Euler check
>    fires (`chi = 8 - 12 + 5 = 1`) instead of the fused box appearing.
>    `canonical_atoms` groups atoms tracing the same curve and — load-
>    bearing — records that they generally run **reversed** relative to each
>    other, since the shared edge of two adjacent coplanar faces is
>    traversed oppositely by each. That opposition *is* the manifold
>    condition, and a dart reusing the class representative's edge must flip
>    its fin sense to preserve it.

**Classification of the resulting regions.** Do *not* rediscover ON by
sampling — that is precisely the degenerate case ray-parity evades.
Coincidence is known by construction, so propagate it. For a region of
face `fa` whose partner is coincident face `fb`, test the region's
interior sample against `fb`'s trimmed region with the 2D containment
test that already exists (`FaceRegionPoly::contains`, `boolean.rs:848`):

```rust
enum Sense { Same, Opposite }        // sign of n_a · n_b at the sample
enum Verdict { In, Out, On(Sense) }

// classify: structural first, ray-parity only as the fallback
let verdict = match self.coincident_partner_region(s, f, &sample) {
    Some(sense) => Verdict::On(sense),
    None => if self.contains_point(other, &sample)? { Verdict::In } else { Verdict::Out },
};
```

Point-in-polygon in a shared chart, no ray casting, no degeneracy. The
ON case becomes the *easy* case rather than the hard one.

**The keep table.** `keep_table` grows from `bool` to `Verdict`. The
rule is the standard one (Requicha/Voelcker, and what ACIS and Parasolid
converge on): union keeps OUT ∪ ON-same-sense; intersection keeps IN ∪
ON-same-sense; difference A−B keeps A's OUT ∪ B's IN (reversed) ∪
ON-opposite-sense. Same-sense ON regions are kept **from solid A only**
— a canonical tie-break, without which the shared face is emitted twice
and the shell is non-manifold.

| op | solid | In | Out | On(Same) | On(Opposite) |
|---|---|---|---|---|---|
| Unite | A | drop | keep | keep | drop |
| Unite | B | drop | keep | drop (dup of A) | drop |
| Intersect | A | keep | drop | keep | drop |
| Intersect | B | keep | drop | drop (dup of A) | drop |
| Subtract | A | drop | keep | drop | keep |
| Subtract | B | keep, reversed | drop | drop | drop |

Worked checks, with A = box `x∈[0,1]`, unit in y and z:

- **B = `x∈[1,2]`, touching.** A's `x=1` face is ON B, opposite sense
  (+X vs −X). Union: both ON-opposite regions drop, the wall vanishes,
  result is the fused `x∈[0,2]` box. Subtract: A's face is kept
  (row 5, On(Opposite)), B's dropped — `A−B = A`, correct. Intersect:
  everything drops, result is empty — correct, since the true
  intersection is a zero-volume square (§6).
- **B = `x∈[0.5,1.5]`, overlapping, flush at `y=0`.** A's `y=0` region
  over `x∈[0.5,1]` is ON B same-sense; for subtract it drops (that
  material is gone), and the result's `y=0` face is A's OUT region over
  `x∈[0,0.5]`. Correct.

## 4. Option (b) — symbolic perturbation (SoS)

Reject. Not merely worse here — inapplicable to this architecture.

SoS makes degeneracy vanish by perturbing B by a symbolic ε along a
generic direction, so every predicate resolves consistently and
classification needs no special cases. Its guarantee rests on **exact
predicates**: the perturbation is only meaningful if the sign of every
geometric test is computed exactly, so that "the ε term decides it" is a
real statement rather than noise.

This kernel does not have that footing. Faces are `f64` analytic
surfaces, imprints are *sampled polylines* (`SAMPLES_PER_CIRCLE = 96`,
`IMPRINT_LINE_SAMPLES = 64`, `boolean.rs:73-77`), containment is
ray-parity with a tolerance band. The adaptive-predicate work that does
exist (`ICC_ERRBOUND_A`, `boolean.rs:4471`) is 2D incircle — nowhere
near a general exact predicate over curved surfaces. Applying SoS on top
of sampled curves perturbs a value already uncertain by far more than ε;
the consistency guarantee is simply absent.

It also collides with the kernel's stated tolerance philosophy
(`tolerance.rs:1-9`): every comparison goes through a `ToleranceContext`
so equality is explicit and configurable. SoS's premise is that equality
never happens.

And the geometric surprise is real, not theoretical: a user who unions
two flush boxes gets a sliver face of thickness ε that survives into the
result unless a snapping pass removes it — at which point the snapping
pass, not SoS, is doing the work.

## 5. Option (c) — tolerant merging + hybrid fallback

Not a rival to (a). It is the *ingredient* (a) needs, plus the
backstop.

Coincidence must be a tolerant judgment — nothing else is meaningful for
a modeller whose inputs come from user transforms. The question is which
tolerance. It must be `Pipeline::snap`, the feature-derived weld length
from `geometric_snap` (`boolean.rs:1144`, computed once in
`Pipeline::new` at `:1444-1449`), **not** an absolute epsilon and not
raw `tol.linear`. The rationale is already documented at `:1132-1143`:
the result must weld and classify the same at `(1e6, 0, 0)` as at the
origin (bugs of-lxk, of-260). A coincidence test keyed off an absolute
epsilon reintroduces exactly that bug class.

Note the mismatch this exposes: `plane_plane` decides coincidence with
`tol.approx_zero` (`ssi/analytic.rs:155`) — absolute, `linear = 1e-6` by
default — while the arrangement welds at `snap`. Faces can be
"coincident" to SSI but not weld, or vice versa. The two must be
reconciled or the arrangement will disagree with the classifier. This
is a latent bug in the current code, not something the new work
introduces.

The **hybrid fallback stays**, permanently. `hybrid.rs` already catches
any B-Rep shortfall and re-runs as F-Rep. The truly degenerate — genuine
non-manifold results (§6) — should keep returning `NotImplemented` and
land there by design. The goal of this work is not to eliminate the
fallback; it is to stop hitting it for cases with an exact answer.

**Recommendation: (a) with (c)'s snap-keyed detection and (c)'s fallback
retained. (b) rejected.**

## 6. Tangent contact — scoping

Tangency is a different problem wearing similar clothes, and mostly
*not* worth solving. Coincident faces overlap in 2D; tangent contact is
measure-zero — a point or a curve.

`TangentPoint` (`boolean.rs:1524`) and tangential intersection curves
(`:1533`) are rejected the same way, and the same false-positive
argument applies: the contact locus frequently falls outside one of the
trimmed regions, in which case there is nothing to imprint and the
boolean is ordinary. `plane_sphere`, for instance, returns
`TangentPoint(foot)` from the infinite plane (`ssi/analytic.rs:186`)
regardless of where the face's trim lies.

Three tiers, and they should not be conflated:

1. **Contact locus outside either trimmed region** → empty imprint,
   proceed. Cheap, purely a placement of the existing test, and
   unblocks a real share of traffic. Worth doing (phase 3).
2. **Contact producing a genuinely non-manifold result** — a sphere
   resting on a plate, unioned. The answer is a body with a
   non-manifold vertex. `check()` enforces at most two fins per edge
   (`check.rs:325`), so this is **not representable** in the current
   topology. Keep `NotImplemented`; let the hybrid path serve it. Not a
   gap to close — a rep limitation to document.
3. **Tangential intersection curves through overlapping regions** — a
   cylinder tangent inside a slot. The imprint curve is real, but the
   faces meet without crossing, so adjacent regions are ON-like along a
   curve and the sense test degenerates. Hardest tier, least common;
   deferred (phase 4) and possibly never worth it against the F-Rep
   fallback.

One semantic decision belongs to §3's table and is worth stating
plainly: **intersection of two merely-touching solids returns an empty
body, not a zero-thickness sheet.** The kernel models solids; a square
of zero volume is not one.

## 7. Stress gates — and why volume is the weak one

The instinct is to gate this work on the volume oracle
(`validate_exact`/`grid_volume`, `hybrid.rs:418,459`). That instinct is
wrong here, and the reason is specific to this feature class.

**A leftover interior wall has zero volume.** Union two stacked boxes,
fail to drop the shared wall, and the volume is exactly right. A
relative volume gate cannot see the defining failure of this feature.
The same holds for a doubled same-sense face.

The gate that *does* work is the one §5 might lead you to discount.
`check()` is combinatorial only — geometric checks are explicitly
deferred (`check.rs:56-57`) — but the dominant failure modes here are
combinatorial, and it catches them squarely:

- **Wall retained** → the wall's edges carry four fins → the ≤2-fins-per-edge
  manifoldness check fires.
- **ON region dropped from both solids** → open shell → the closure
  check fires.
- **Same-sense face kept twice** (tie-break missed) → duplicate face,
  four fins → fires.
- **Euler–Poincaré** `V − E + F − R = 2(S − H)` catches the residue.

So for coincident-face work the asymmetry inverts: `check()` is the
primary gate and the volume oracle is secondary. Volume still earns its
place against *sense* errors — a kept face with a flipped normal, or the
same-sense region kept from the wrong solid — which are geometric and
which `check()` will happily pass. Each phase therefore needs both, plus
explicit expected-volume asserts (not just relative divergence) and
face-count asserts.

**Three tests assert the current failure and will break by design.**
They are precondition tripwires, not bugs, and must be *rewritten* to
assert the exact path now succeeds — not deleted, since each also
verifies the F-Rep fallback still produces the right volume:

- `hybrid_e2e.rs:121` — `coincident_face_failure_falls_back_to_frep_and_stays_valid`
  (precondition `:129-134`; the comment at `:129-131` anticipates this
  exact change)
- `hybrid_e2e.rs:156` — `coincident_face_union_falls_back_to_frep_and_stays_valid`
- `hybrid.rs:703` — `coincident_faces_fall_back_to_frep`

New cases belong in `tests/boolean_stress.rs`, under its standing
protocol (`:1-39`): failures are the point, and a failing case earns a
`bd` bug bead plus `#[ignore]` naming it — never a softened assertion.
Minimum matrix per phase: touching boxes (union/subtract/intersect),
flush-overlapping boxes, coplanar-but-disjoint faces (the false-positive
case — must now pass with no new geometry), L-shaped partial overlap,
inclusion–exclusion `vol(A)+vol(B) == vol(A∪B)+vol(A∩B)`, and rotation
invariance to catch snap-scaling regressions.

## 8. Phasing

Child beads of of-bxl, in dependency order:

| bead | scope | gate |
|---|---|---|
| **of-bxl.2** | **Face-level transversality gate.** Move the `Coincident` rejection from surface level to face level: if the trimmed regions do not overlap, proceed as transversal. No new geometry, no verdict change. | coplanar-disjoint and edge-adjacent boxes pass; the three tripwires still fail (unchanged) |
| **of-bxl.3** | **ON verdict.** `Verdict`/`Sense` enums, `coincident_partner_region`, `contains_point` returns `Verdict`, `keep_table` on `Verdict` (§3 table) + A-only tie-break. Dead code until of-bxl.4 produces ON regions. | unit tests on `keep_table`; no behaviour change |
| **of-bxl.4** | **Coincident planar imprint.** *Done.* Imprint each coincident face with the partner's trim edges via `clip_imprint`, one-sided (`ImprintKind`). Reconcile the tolerance mismatch by re-running SSI at `snap` (§5, §9). Weld coincident atoms across solids in `build_output` (`canonical_atoms`) — not anticipated by §3, see its amendment. Four tripwires rewritten, not three. | full §7 matrix for planes in `boolean_stress.rs` §(11); `check()` clean; explicit volume asserts |
| **of-bxl.5** | **Coincident curved surfaces.** Extend to the other four `Coincident` producers (`cylinder_cylinder:482`, `coaxial_profiles:586`, `sphere_sphere:654`, `cone_cone:980`). Chart-sharing on periodic surfaces is the new risk — seam handling per `contains_for_clip` (`boolean.rs:873`). | coaxial cylinders, concentric spheres; §7 matrix |
| **of-bxl.6** | **Tangent triage (tier 1).** Contact locus outside trimmed regions → empty imprint, proceed. Tiers 2 and 3 stay `NotImplemented`. | tangent-outside-trim cases pass; non-manifold cases still fall back |
| **of-bxl.7** | **Tangential curves (tier 3).** Deferred; open only if traffic justifies it against the F-Rep fallback. | — |

of-bxl.2 is worth landing alone and first: it is small, needs no new
concepts, and by itself converts a share of today's fallbacks into exact
results.

## 9. Open questions

- **Tolerance reconciliation (§5)** — ~~should `plane_plane` take the
  pipeline's `snap`?~~ **Decided in of-bxl.4: neither.** `find_imprints`
  re-tests coincidence by re-running the *whole* SSI with `linear` set to
  `snap` (`Pipeline::coincident_at_snap`). SSI stays a pure surface-level
  function with no snap parameter threaded through it, and because the test
  is the SSI verdict itself rather than a hand-written plane comparison, it
  covers all five `Coincident` producers for free instead of special-casing
  planes. `angular` is left alone: parallelism is scale-free, and only the
  positional `approx_zero` test keys off `linear`.

  Worth recording which direction actually bites. On a unit-scale part
  `snap` (~1e-9 of the feature extent) is *three orders of magnitude
  tighter* than `tol.linear` (1e-6), so **SSI is the permissive one**: it
  calls surfaces 1e-7 apart coincident when nothing about them welds.
  Those are now rejected as two distinct parallel surfaces. The converse —
  SSI reporting `Empty` for surfaces closer than `snap` — needs
  `snap > tol.linear`, i.e. a feature extent above ~1e3, and is not
  reachable from the `Coincident` arm at all. Left to of-bxl.5 rather than
  re-testing every `Empty` pair.
- **Partial coincidence of curved faces** — two cylinders coaxial and
  equal-radius are coincident over their whole surface; two *nearly*
  coaxial ones are transversal with a near-tangential curve. The
  transition is discontinuous in the inputs, and `snap` decides which
  side we land on. Whether that discontinuity is acceptable, or whether
  near-coaxial should snap to coaxial, is a of-bxl.5 question.
- **Non-manifold results** — §6 tier 2 is a rep limitation. If the
  roadmap ever admits non-manifold bodies, tier 2 reopens. Worth a
  pointer from the topology docs so the constraint is discoverable from
  both ends.
