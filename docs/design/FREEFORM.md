# Freeform surfaces through the exact boolean pipeline

Status: design note (of-37i.2), phase 0 of the of-37i epic. No code changes.
Scope: taking **NURBS surface bodies** through the *exact* B-Rep boolean path —
chart abstraction, SSI, imprint hosting, region split, classification,
tessellation, tolerances, and where the F-Rep fallback keeps the floor.
Out of scope: how spline profiles *become* NURBS surfaces (loft/sweep/revolve —
of-37i phase 1), STEP round-trip (phase 4), GUI (phase 5).

This is the boss fight. The note exists to make it a sequence of small, gated
fights instead of one big one.

## 0. The headline

**A NURBS surface cannot enter the boolean pipeline today, and it will not fail
with a `panic!` when it tries — it cannot even be constructed.** There are zero
`todo!()`/`unimplemented!()` markers in `opensolid-brep`. The pipeline is
statically closed over `Surface3`, a five-variant enum of analytic primitives
(`crates/opensolid-brep/src/surface.rs:87`), and `GeometryStore` holds exactly
two arenas — `Arena<Curve3>` and `Arena<Surface3>`
(`crates/opensolid-brep/src/geometry.rs:24`). `Face::surface` is
`Option<EntityId<Surface3>>` (`src/topology.rs:152`). `NurbsSurface`
(`src/nurbs/surface.rs:29`) is a standalone struct that no face can name.

That is a *good* failure mode to build on: adding a `Surface3::Nurbs(...)`
variant turns the entire work list into non-exhaustive-match **compile errors**
rather than runtime surprises. Section 8 uses that property as the phase-1 gate.

Three independent walls stand between here and an exact NURBS boolean. They are
not equally hard, and the third one is the one that gets forgotten:

1. **Representation** — `Surface3` / `GeometryStore` must admit NURBS (§1).
2. **The chart** — `Chart::param`'s *infallible closed-form* point→uv inverse is
   assumed by imprint clipping, region seeding, and classification. NURBS has no
   closed-form inverse. This is the deepest assumption in `boolean.rs` (§2).
3. **Ray classification** — `ray_surface_hits` and `surface_residual` need
   closed-form implicit surfaces. NURBS has none. This is a *separate*
   algorithmic gap from SSI and is easy to miss because SSI dominates the
   conversation (§5).

Two things are further along than the epic's framing suggests, and both are
cheap early wins:

- **The marcher is already NURBS-capable and generic.** `trait MarchSurface`
  (`src/ssi/marching.rs:66`) is implemented for both `NurbsSurface` and
  `Surface3`, and the driver `march_boxed<A: MarchSurface, B: MarchSurface>`
  (`:483`) already accepts **mixed** pairs. `intersect_nurbs` (`:467`) works and
  has six tests — it is a fully-built, fully-unwired component with no
  non-test callers anywhere in the workspace. Wiring is a dispatch problem, not
  a numerics problem.
- **The of-lcx constrained-Delaunay tessellator is surface-agnostic.**
  `FlipMesh` (`src/boolean.rs:4415`), `boundary_cdt` (`:5247`), and
  `refine_curved_region` (`:4825`) are pure 2D uv combinatorics. They touch the
  surface only to lift uv→3D and measure deviation. They transfer to NURBS
  essentially for free once the chart does (§4).

The combinatorial core — `build_atoms`, `merge_imprint_chains`, `apply_chain`,
`keep_table`, `build_output`, union-find shells, Euler checks — is entirely
surface-agnostic and needs **no change**. That is the payoff for the phasing
below: most of the pipeline is already freeform-ready.

## 1. Representation

`Surface3` gains a variant:

```rust
pub enum Surface3 {
    Plane { .. }, Cylinder { .. }, Cone { .. }, Sphere { .. }, Torus { .. },
    Nurbs(Box<NurbsSurface>),   // boxed: NurbsSurface owns three Vecs
}
```

`NurbsSurface` already implements the full `SurfaceEval` trait
(`src/nurbs/surface.rs:195`) — `point`, `du`, `dv`, `normal`, `domain_u/v`,
`is_periodic_u/v`, `is_singular` — with nothing unimplemented, plus
`SurfaceProject` (`src/project.rs:509`) and `derivatives(u, v, order)` (`:144`).
So the variant is mostly a plumbing change. Two semantic snags:

**Periodicity.** `NurbsSurface::is_periodic_u/v` are hardcoded `false`
(`src/nurbs/surface.rs:242`) — clamped representation only; a geometrically
closed surface is still evaluated over a single pass of its domain. This is
*load-bearing and good* (§2): a clamped NURBS chart is non-periodic, so
`period_u() == None` and every seam mechanism in the pipeline degenerates to the
plane path. **Do not add periodic NURBS.** A closed spline body should be built
as a clamped patch whose two ends meet at a real seam **edge** in the topology,
exactly as a cylinder's seam is an edge today. That keeps the wrap in the
topology, where the pipeline already handles it, instead of in the chart, where
it would need a period the knot vector can't honestly supply.

**Curves.** `Curve3` (`src/curve.rs:46`) has no NURBS variant either, and adding
one has a trap: `Curve3::is_periodic()` is defined as `self.is_closed()`
(`:348`), but `NurbsCurve::is_closed()` is a *geometric* test (`point(t0) ≈
point(t1)`, `src/nurbs/curve.rs:439`) while `NurbsCurve::is_periodic()` is
hardcoded `false` (`:444`). A `Curve3::Nurbs` would be the first variant that
can be closed-but-not-periodic, breaking an identity every other variant
upholds.

**We do not need `Curve3::Nurbs` for booleans.** `Curve3::Polyline` (`:60`) is
already documented as *"the exact-geometry representation of marched SSI curves
whose intersections have no closed form"*, and `MarchedCurve`
(`src/ssi/marching.rs:90`) lands in it directly. NURBS **edge** curves are only
needed for spline profiles (of-37i phase 1) and STEP (phase 4). Keep the variant
out of this workstream; if phase 1 lands it first, this note's phases are
unaffected.

**Bounding boxes.** `broad_phase_face_box` (`src/boolean.rs:1270`) falls back to
dilated boundary samples when `surface.bounding_box()` returns `None`. For a
NURBS patch that bulges away from its trim boundary, that box can be **too
tight** — a missed face pair is a silently wrong boolean, not an error. The
control-hull box is the correct answer, is exact (convex hull property), and is
cheap. This is a small fix with an outsized correctness role; §8 makes it part
of phase 1 rather than an afterthought.

## 2. The chart abstraction

`Chart` (`src/boolean.rs:399`) is described in its own doc comment as the
*"invertible parameterization of the supported analytic surfaces."* Where
`Surface3` gives forward eval, `Chart` gives **point → (u,v)**, in closed form,
one variant per `Surface3` variant, built by an exhaustive `Chart::build`
(`:459`).

### 2.1 What the parameter domain *is*

For analytic charts, the domain is the surface's natural extent — infinite for a
plane, `(0, 2π) × (-∞, ∞)` for a cylinder — and the *face* is a trimmed region
inside it, embedded as even-odd uv polygons (`FaceRegionPoly`, `:840`). There is
no rectangle-domain concept in the region machinery at all.

For NURBS, the bead framing asks: *"no closed-form seam/period — the parameter
domain is the trim rectangle."* Sharpen that. The chart domain is the **knot
domain** `knots_u.domain() × knots_v.domain()` — a genuine, finite rectangle,
unlike any existing chart. The face's trim is a region inside it, exactly as for
a plane. So the NURBS chart is **the plane case with a bounded domain**, not a
new kind of thing:

| Chart property | Plane | Cylinder / Sphere / Cone / Torus | Clamped NURBS |
|---|---|---|---|
| `period_u` (`:691`) | `None` | `Some(2π)` | **`None`** |
| `period_v` (`:701`) | `None` | `Some(2π)` (torus only) | **`None`** |
| Domain | unbounded | angular × (bounded or not) | **finite knot rectangle** |
| `param` inverse | closed form | closed form | **iterative** |
| Seam | none | u=0 meridian | none (seam is a topology edge) |
| Poles | none | sphere ±π/2, cone apex | **collapsed control rows** |

Three consequences fall straight out, and all three are favourable:

- **`FaceRegionPoly::localize` (`:901`) becomes a no-op.** It early-returns when
  both periods are `None`. The whole shift-into-window apparatus (`:930`) is
  skipped.
- **`seam_crossings` (`:2703`) and the seam-barrier machinery yield nothing.**
  `reconstruct` (`:2331`) collects an empty barrier set — correct, because a
  clamped NURBS chart has no seam to cross.
- **`marched_polylines`' seam-cut loop (`:1593`) degenerates cleanly.** It is
  already parameter-generic (it reads `period_u()/period_v()` and skips axes
  returning `None`), and it operates on `MarchedCurve::params_a/params_b`
  directly — **it never inverts**. This is the single most NURBS-ready function
  in the imprint path; it is typed `&Surface3` only because its helpers
  (`pin_intersection_point`, `tighten_boundary_point`) are.

The **domain boundary** replaces the seam as the thing that needs care. A
marched curve can exit through a knot-domain edge; the marcher already
terminates and re-corrects there (of-pb7.3's boundary termination, pinning the
crossing parameter with a 3×3 solve). What is new is that a trim region can
*abut* the domain boundary, so a face's uv polygon may run exactly along
`u = knots_u.domain().1`. That is a clip-tolerance question (§6), not a
structural one.

### 2.2 The one hard problem: `Chart::param` must become iterative

```rust
// src/boolean.rs:548 — infallible, closed-form, hot.
fn param(&self, p: &Point3, hint: Option<(f64, f64)>) -> (f64, f64)
```

Every arm is direct algebra (dot products, `atan2`, `asin`). NURBS has no such
inverse. The replacement exists and is already trusted — `SurfaceProject::
project_point(&self, point: &Point3) -> SurfaceProjection` (`src/project.rs:80`)
returns `{ u, v, point, distance, converged }`, is implemented for
`NurbsSurface` (`:509`) via per-knot-span seeding plus Newton, and is what
`march_boxed` already uses for grid seeding. But three things change, and they
ripple:

1. **Fallibility.** `project_point` can come back `converged: false`. `param`
   returns `(f64, f64)` with no error path, and is called in hot loops —
   `clip_imprint`'s `inside` closure (`:1737`), `contains_point`'s ray loop
   (`:2400`), `CoverEmbedder::push` (`:1018`). Threading `CoreResult` through
   touches most consumers in `boolean.rs`. **Recommended signature:**

   ```rust
   fn param(&self, p: &Point3, hint: Option<(f64, f64)>) -> CoreResult<(f64, f64)>
   ```

   Analytic arms return `Ok(...)` unconditionally (zero behaviour change, and
   the compiler finds every callsite). The NURBS arm maps a non-converged or
   too-distant projection to `CoreError::Degenerate` naming the location. A
   failed inversion must **abort to the F-Rep fallback**, never guess: a wrong
   uv silently corrupts region parity, which is exactly the of-ipt.4 failure
   mode the README's volume-identity gate was written for (right topology,
   ~12× wrong volume).

2. **`hint` changes meaning** — from *"which period to unwrap into"* to
   *"Newton seed"*. For analytic charts it stays the former. For NURBS the
   seeded path is both faster and more robust than blind projection, so
   **pass the hint everywhere it exists**. Note that two current callsites pass
   `None` in hot loops (`clip_imprint`'s `inside`, `:1737`; the ray-hit
   inversion in `contains_point`, `:2400`) — for NURBS those become full
   span-seeded projections per sample. That is a real cost (§7) and a real
   robustness risk: a self-approaching patch has multiple local minima, and an
   unseeded projection can land on the wrong sheet. Threading a hint through
   these two callsites is part of the work, not an optimization.

3. **`uv_scale(v) -> (f64, f64)` (`:673`) is closed-form in `v` alone** — the
   per-axis arc-length metric (e.g. `(radius, 1.0)` for a cylinder). For NURBS
   it is `(|S_u|, |S_v|)` **at a point**, so the signature must become
   `uv_scale(&self, uv: (f64, f64))`. Cheap to evaluate (`derivatives(u,v,1)`),
   but it is a signature change with consumers in `contains_for_clip` (`:873`)
   and `region_interior_point` (`:3502`).

**Poles.** `Chart::pole_v` (`:721`) / `apex` (`:753`) / `pole_points` (`:771`)
have no NURBS analogue *by construction*, and the honest answer is to return
`None` / empty. A NURBS patch **can** have a degenerate edge (a collapsed
control-point row — the classic lofted-to-a-point tip), and `NurbsSurface::
is_singular` (`:252`) detects it via the same `|S_u × S_v|` test as `normal`.
But note the deliberate contrast already in the code: at a degenerate edge
`NurbsSurface::normal` returns `None` (`:225`), whereas `Surface3::Sphere`
returns the limit normal `±axis`. The sphere's pole machinery relies on that
limit and on a *known* pole location. For phase 1, **reject NURBS patches with
degenerate edges at chart-build time** and let them fall to F-Rep. Degenerate
tips are a distinct campaign (§8, phase 5) and conflating them with the base
promotion is how phases slip.

## 3. Marching SSI: the coverage matrix

`of-pb7.3` delivered `intersect_nurbs` (`src/ssi/marching.rs:467`) — grid-seeded
(16×16, oriented-distance sign), least-norm 4D Newton seed refinement, predictor
along `cross(n1, n2)` through the first fundamental forms, plane-constrained 4×4
Newton corrector, step halving, boundary termination, closure detection.
Transversal only: it bails with `CoreError::Degenerate` when
`|n1 × n2| < NEAR_TANGENCY_SIN = 1e-3` (`:47`).

It is **`pub`, re-exported twice (`src/ssi/mod.rs:17`, `src/lib.rs:53`), tested
six ways — and called from nothing outside its own test module.** The reason is
purely typing: it takes `&NurbsSurface`, while the pipeline dispatches through
`ssi_intersect` / `intersect_marched` / `intersect_marched_bounded`
(`src/boolean.rs:1516`, `:1548`, `:1564`), all typed `&Surface3`.

The generic driver underneath is already there and already accepts mixed pairs:

```rust
// src/ssi/marching.rs:483
fn march_boxed<A: MarchSurface, B: MarchSurface>(
    a: &A, b: &B, domains: [(f64, f64); 4], tol: &ToleranceContext,
) -> CoreResult<Vec<MarchedCurve>>
```

`intersect_nurbs` and `intersect_marched` are both thin wrappers over it. So
**NURBS↔analytic SSI needs no new numerics** — once `Surface3::Nurbs` exists,
`Surface3` *is* a `MarchSurface`, and both pairings are one call.

### Coverage matrix

| Pair | Today | After phase 2 | Route |
|---|---|---|---|
| NURBS ↔ NURBS, transversal | `intersect_nurbs`, unwired | ✅ exact | `march_boxed`, both knot domains |
| NURBS ↔ plane | ✗ nothing | ✅ exact | `march_boxed`; plane domain clipped to the joint face box |
| NURBS ↔ cylinder/sphere/cone/torus | ✗ nothing | ✅ exact | `march_boxed`; same clipping |
| NURBS ↔ anything, tangential | ✗ | **→ F-Rep** | `Degenerate` at `NEAR_TANGENCY_SIN` |
| NURBS ↔ anything, coincident | ✗ | **→ F-Rep** | existing `Coincident` rejection (`:1520`) |
| NURBS with degenerate edge | ✗ | **→ F-Rep** | rejected at `Chart::build` (§2) |

**The unbounded-domain trap.** `march_boxed` requires *finite* parameter boxes —
its doc says so, and the existing callers clip unbounded primitive directions.
`intersect_marched_bounded` (`:802`) already does this for cone sections, using
`box_a.intersection(box_b)` as `(center, radius)` (`src/boolean.rs:1564`). A
NURBS patch is always finite (knot domain), but its *partner* may not be — a
plane is `(-∞, ∞)²`, a cylinder's `v` is unbounded. **Every NURBS↔analytic pair
must go through the bounded entry point**, seeded from the joint face box. This
is the mistake to expect: a NURBS↔plane pair looks compact because one side is,
and routing it through `intersect_marched` yields infinite grid parameters and
NaN seeds rather than an honest error.

### The dispatch traps

These are `matches!`/`_ =>` fallthroughs, so they produce **no compile error**
when the variant is added — they are the dangerous half of the work list:

- `ssi::analytic::intersect`'s `_ =>` arm (`src/ssi/analytic.rs:125`) catches
  every NURBS pair with the message *"analytic SSI for cone pairs other than
  plane-cone and coaxial cone-cone"* — wrong and actively misleading. The
  cascade at `src/boolean.rs:1514` only escalates to marching when the guards
  say so, so a NURBS pair currently falls to `Err(err) => return Err(err)`
  wearing a cone's error message.
- `marched_ssi_supported` (`src/boolean.rs:1384`) and `is_bounded_marched`
  (`:1409`) are `matches!` over primitive shapes — both return `false` for NURBS.
- `intersect_marched`'s `_ =>` (`src/ssi/marching.rs:763`) and
  `intersect_marched_bounded`'s `_ =>` (`:823`).

Phase 2's first commit should be a test asserting that an *unsupported* NURBS
pair produces an error naming NURBS. Otherwise the fallthrough looks like a pass.

## 4. Imprint hosting on NURBS faces

There is no p-curve. `Fin::pcurve` points at a unit-struct marker
(`src/topology.rs:189`, `:58`) and `GeometryStore`'s doc is explicit: *"2D
parameter-space curves are not stored here yet; the SP-curve representation is a
later issue"* (`src/geometry.rs:11`). Faces are covered by `FaceRegionPoly`
(`:840`) — one closed `Vec<((f64,f64), Point3)>` polyline per loop — with uv
**recomputed on demand from the 3D point** via `Chart::param`. `Imprint`
(`:1113`) stores a 3D `Curve3` plus a `SampledCurve`.

That recompute-on-demand design is exactly what makes NURBS expensive, and it is
also what makes NURBS *possible* without inventing p-curves: every uv is a
projection away. The bead's framing — *"curve-in-parameter-space via projection;
closest-point infra exists"* — is right, with one correction: **we should not
build p-curves for this workstream.** The pipeline never asks for a parametric
curve; it asks for *the uv of this 3D point on this face*. Three reasons to
keep it that way:

1. `MarchedCurve` **already carries** `params_a` and `params_b` — the exact uv
   preimages on both surfaces, from the marcher, converged. For imprint
   vertices, the uv is not recomputed at all; it is *known*. The right move is
   to **carry the marched params forward** rather than re-project the 3D points
   through `Chart::param`. Re-projecting throws away converged information and
   can land on the wrong sheet of a self-approaching patch.
2. A real SP-curve representation is a large independent change (storage,
   arenas, `check()` rules, STEP mapping) with its own bead.
3. Everything else in the imprint path already works on `(uv, Point3)` pairs.

So the concrete change to `Imprint` is to make the sampled curve carry optional
per-vertex uv on each host face, populated from `MarchedCurve` when the source
was marched, and left `None` (falling back to `Chart::param`) otherwise.

**`marched_polylines` (`:1593`) is where this lands** and, as noted in §2.1, it
is already parameter-generic and never inverts. Its `Surface3` typing comes from
two helpers:

- `surface_residual` (`:1289`) and `surface_residual_gradient` (`:1333`) —
  exhaustive matches returning **closed-form implicit residuals**. NURBS has no
  implicit form. Replace with a two-surface Newton on `(u_a, v_a, u_b, v_b)`
  driving `|S_a − S_b|` to zero — which is *precisely what the marcher's
  corrector already does* (`march_boxed`'s plane-constrained 4×4 solve). This
  should be extracted and shared, not rewritten.
- `polish_clip_endpoint` (`:1890`) consumes those residuals; it follows.

**`clip_imprint` (`:1693`)** needs no structural change but is where the cost
lands: its `inside` closure calls `chart.param` per sample, with
`IMPRINT_LINE_SAMPLES = 64` for lines / `SAMPLES_PER_CIRCLE = 96` for conics /
per-vertex for polylines, then `refine_crossing` bisects up to
`CLIP_REFINE_ITERATIONS = 50` times. For marched curves against a NURBS host,
the uv comes from `params_a`/`params_b` and the projection disappears (per point
1 above). For an *analytic* curve crossing a NURBS face — e.g. a cylinder-plane
circle clipped against a NURBS face's region — it does not, and each sample is a
seeded Newton.

**The known gap to expect.** `apply_chain` (`:3385`) already errors with
*"boolean imprints chording a hole boundary (transversal MVP)"*, and of-9ia
records that non-coaxial cone–cone SSI marches correctly but **hosting the
marched imprint on two curved faces leaves an open chain**. NURBS↔NURBS is the
same shape of problem: two curved hosts, a marched imprint, chains that must
close. **Expect of-9ia to bite here, and expect it to be the phase-3 blocker.**
It is plausible that fixing chain closure for NURBS fixes of-9ia too — both are
"marched imprint on two curved hosts" — and phase 3 should check that
explicitly rather than route around it.

## 5. Classification: the wall nobody budgets for

`Pipeline::contains_point` (`:2386`) decides inside/outside by **ray parity**
over six fixed directions, and its inner call is:

```rust
// src/boolean.rs:3621 — exhaustive analytic match.
fn ray_surface_hits(surface: &Surface3, p: &Point3, dir: &Vector3) -> Vec<f64>
```

Plane → linear solve. Cylinder/sphere/cone → quadratic. Torus → quartic,
sign-sampled and bisected (`ray_torus_hits`, `:3723`). **NURBS → nothing.** Then
each hit is re-inverted through `chart.param` twice (grazing test, face test).

This is a hard requirement fully independent of SSI, and it is the item most
likely to be discovered late. Three options:

1. **Ray–NURBS via Bézier clipping / subdivision.** Correct and exact-ish;
   a substantial numerical component with its own robustness surface (multiple
   roots, tangential grazes, silhouettes). Not phase 1.
2. **Parity against the tessellated operand.** Ray-cast the NURBS operand's
   *tessellation*. Cheap, reuses of-lcx, but parity is only as good as the mesh:
   a ray passing within a chord's deviation of the true surface flips parity and
   silently corrupts the result — the of-ipt.4 failure mode again. Would need
   the ray offset from the surface by ≫ deviation, which the grazing-retry
   ladder can partly supply but cannot guarantee.
3. **Parity against the F-Rep field.** `HybridBody` already carries an SDF for
   the fallback path; `sign(sdf(p))` *is* an inside test, needs no ray at all,
   and is exactly as accurate as the F-Rep resolution.

**Recommendation: (3) for phase 1, (1) as its own campaign.** Option 3 is a
handful of lines, it is honest about its accuracy (the F-Rep grid cell is
already the hybrid path's stated accuracy bar — `hybrid.rs:369` gates the exact
path on `deviation <= cell_size(&bounds, opts.resolution)`), and it removes
ray–NURBS from the critical path of the first promotion. Its cost is that
classification accuracy for NURBS operands is F-Rep-resolution-bound while the
*geometry* is exact — an asymmetry worth stating plainly in the phase-1 bead
rather than discovering in a stress failure. Note also that option 3 only
classifies against a NURBS *operand*; a NURBS region classified against an
all-analytic operand keeps the exact ray path unchanged.

`keep_table` (`:2448`) is pure combinatorics and is unaffected.

## 6. Tolerance strategy

The bead's framing is exactly right: **there is no exact arithmetic here.**
Analytic SSI produces conics with closed-form parameters; NURBS SSI produces
polylines from a Newton corrector. Every NURBS quantity is marching plus
refinement, so acceptance must be stated as **residuals**, not as equalities.

`ToleranceContext` (`crates/opensolid-core/src/tolerance.rs:38`) supplies
`linear: 1e-6`, `angular: 1e-8`, `parametric: 1e-9`. `geometric_snap` (`:1144`)
derives the pipeline's welding scale from operand extent (not origin distance).
The proposed acceptance ladder, each rung a testable assertion:

| Quantity | Residual | Bar | Where |
|---|---|---|---|
| SSI vertex gap | `\|S_a(u_a,v_a) − S_b(u_b,v_b)\|` | `≤ tol.linear` | already enforced by `march_boxed`'s `gap_tol` (`:489`) |
| SSI vertex on-surface | `dist(p, S)` via `project_point` | `≤ tol.linear` | new phase-2 test |
| Chart inversion | `\|S(param(p)) − p\|` | `≤ tol.linear`, else `Degenerate` | new, in the NURBS `param` arm |
| Imprint endpoint on host boundary | `dist` to the boundary polyline | `≤ snap` | existing `polish_clip_endpoint` semantics |
| Chain closure | endpoint gap | `≤ snap` | existing `merge_imprint_chains` |
| Tessellation chord | worst deviation | `≤` F-Rep cell | existing `tessellate_measured` gate (`hybrid.rs:369`) |
| Volume identity | `\|vol(A)+vol(B) − vol(A∪B) − vol(A∩B)\|` | `1e-9` relative | the hard gate (README) |

Two NURBS-specific pressures:

- **Marching step vs. curvature.** The marcher's fixed 16×16 seed grid
  (`GRID_DIVISIONS`) is tuned for primitives. A high-knot-count patch can hide a
  whole intersection branch between grid nodes — a **missed branch is a silently
  wrong boolean**, and unlike a missed face pair there is no downstream check
  that catches it. Seeding should be **per-knot-span** (as
  `NurbsSurface::project_point` already does, `src/project.rs:509`: `degree+2`
  samples per span) rather than a fixed grid over the whole domain. This is a
  phase-2 must, not a hardening nicety.
- **Parametric tolerance is not scale-free.** `tol.parametric` is documented as
  *"relative to a unit-scale parameter domain."* A knot domain is arbitrary
  (`[0, 1]`, `[0, 17.3]`, whatever the loft produced). Every parametric test on
  a NURBS chart must normalize by the domain span, or the same patch will pass
  or fail depending on its knot scaling. Cheap to get right up front,
  miserable to retrofit — and a rotation/scale-invariance stress case will find
  it.

## 7. Tessellation

Two independent tessellators exist, and only one matters here.

`src/tessellate.rs` (the store-backed path, `tessellate_face_into`, `:173`) is a
`match surface` that handles trimming only for planes (ear clip) and
cylinder/cone iso-rectangles (`QuadricUSpan`, `:442`), and **rejects any trim at
all** on spheres/tori (`require_seam_closed_boundary`, `:377`). Its
`boundary_param_range` doc already concedes that a curved-in-uv cut *"cannot be
gridded without hole bridging and is rejected for the CDT pass (of-q6u)."* This
path is a dead end for freeform; a NURBS arm here should grid the **untrimmed**
patch over its knot domain (which STEP's mesh fallback already does,
`io/step/read.rs:33`) and defer trimmed faces to the CDT.

`BooleanOutput::tessellate_measured` (`:328`) is the one that matters, and it is
in good shape. `boundary_cdt` (`:5247`) recovers every ring edge as a constraint
and removes hole/exterior triangles by parity, so bridging across a hole is
impossible by construction; `refine_curved_region` (`:4825`) inserts an interior
lattice and Delaunay-refines via `FlipMesh` (`:4415`) with adaptive-precision
`orient2d`/`in_circle` predicates. Per of-lcx, this replaced the unsafe
ear-clip+bisect refinement and dropped the through-hole band deviation from
~22% volume error to `<5e-3`.

`refine_curved_region` takes `chart: &Chart` and uses it **only** to lift uv→3D
and measure deviation. Everything else — `FlipMesh`, `ring_contains` (`:4793`),
`bridge_is_clear` (`:5463`), `seg_properly_intersect` (`:5225`),
`triangulate::ear_clip` — is pure 2D uv combinatorics. **The of-lcx CDT is ~90%
NURBS-ready.** Two couplings to break:

1. **`pitch = TWO_PI / SAMPLES_PER_CIRCLE` (`:4837`)** assumes `u` is radians.
   For NURBS, derive the lattice pitch from **curvature over the knot domain** —
   a span-based pitch refined until chord deviation meets the bar. The
   `derivatives(u, v, 2)` needed for a curvature estimate already exists
   (`src/nurbs/surface.rs:144`), and `NewtonSurface` for `NurbsSurface`
   (`src/project.rs:495`) already uses second derivatives.
2. **`Chart::uv_scale`** (§2.2) becomes point-valued, which
   `refine_curved_region`'s `scale: (f64, f64)` argument must follow.

The existing deviation gate then does the rest of the work for free: an
under-refined NURBS face simply fails `deviation <= cell_size(...)` in
`hybrid.rs:369` and diverts to F-Rep. **Bad tessellation costs accuracy, not
correctness** — which is why tessellation is *not* on the critical path and
should not be allowed to block the phase-2/3 promotion.

## 8. Fallback boundaries — what stays F-Rep

The fallback is not a consolation prize; it is the reason this can be phased at
all. `hybrid::boolean` (`crates/opensolid-kernel/src/hybrid.rs:331`) tries the
exact path and diverts on **any** shortfall: an error, a non-manifold
tessellation, chords straying past an F-Rep cell, or a `validate_exact` volume
mismatch. Each item below is therefore a *deliberate* fallback, not a bug:

| Configuration | Path | Why |
|---|---|---|
| NURBS ↔ NURBS/analytic, transversal, clamped, non-degenerate | **exact** (phase 3) | the target |
| Tangential or coincident NURBS contact | **F-Rep** | `NEAR_TANGENCY_SIN` bail; matches the analytic MVP's own limit |
| NURBS with degenerate edges (collapsed rows) | **F-Rep** | pole machinery has no analogue (§2) |
| Periodic/unclamped NURBS | **F-Rep** (or rejected at construction) | seams belong in topology, not the chart (§1) |
| Self-intersecting / self-approaching patches | **F-Rep** | multi-sheet projection is unreliable |
| Chart inversion non-convergent | **F-Rep** | never guess a uv (§2.2) |
| Blends, offsets, shells on freeform | **F-Rep** | unchanged; out of scope |

Two properties this preserves, and they are why the phasing is safe:

- **No regression is possible for analytic bodies.** Every change proposed here
  is either additive (a new variant, a new dispatch arm) or a signature change
  whose analytic arms are `Ok(...)` wrappers over today's exact code.
- **No silently-wrong result is possible for NURBS bodies.** The runtime
  `validate_exact` gate re-checks every accepted exact result against an
  independent F-Rep volume estimate. Even if a NURBS boolean is subtly wrong, it
  is discarded rather than returned. That is exactly the safety net that lets us
  promote incrementally — but per the README's stress-suite-first policy, it is
  a *net*, not a gate. The gate is the stress suite.

## 9. Phasing and gates

Per the README's **stress-suite-first promotion** rule: *"A new surface class
does not enter the exact boolean pipeline until its randomized stress suite
(rotations, scales, volume identities, round-trips) is green. Until then it
routes through the F-Rep fallback, which already works."* That rule carried
spheres/tori (of-7ld) and cones (of-dtj). It gates NURBS the same way, which
means **the stress suite is written before the pipeline is enabled, not after**.

Each phase below has a gate that is a *test*, not a review. Phases 1–2 are
independently landable and useful on their own.

**Phase 1 — representation + chart (`of-37i.3`).**
`Surface3::Nurbs(Box<NurbsSurface>)`; `Chart::Nurbs`; `Chart::param` becomes
`CoreResult`, iterative for NURBS via `project_point`, seeded by `hint` (and a
hint threaded through the two callsites that pass `None`); `uv_scale` becomes
point-valued; `chart_point`/`normal`/`period_*` arms (easy); `pole_*` return
`None` and `Chart::build` **rejects** degenerate-edge patches;
`broad_phase_face_box` uses the control hull. Booleans on NURBS bodies still
error → F-Rep.
**Gate:** the compile-error inventory is empty (every match exhaustive); a
NURBS body can be *constructed and checked*; `param`/`chart_point` round-trip to
`tol.linear` on a randomized patch corpus over randomized knot scalings; all
1187 existing tests unchanged; `clippy -D warnings`.

**Phase 2 — SSI wiring (`of-37i.4`).**
`ssi::intersect` gets an explicit NURBS arm erroring with a NURBS-specific
message (kill the cone-message fallthrough); `marched_ssi_supported` /
`is_bounded_marched` admit NURBS pairs, routing **every** NURBS↔analytic pair
through the bounded entry point; per-knot-span seeding replaces the fixed 16×16
grid; extract the marcher's two-surface Newton corrector to replace
`surface_residual`/`_gradient` for NURBS.
**Gate:** NURBS↔plane matches an analytic section to `tol.linear`; NURBS↔NURBS
transversal yields a single continuous branch, boundary-to-boundary; a patch
with an intersection branch narrower than a 16×16 cell is *found* (the
missed-branch regression test); every unsupported pair errors naming NURBS.

**Phase 3 — imprint, region split, classification (`of-37i.5`).**
Carry `MarchedCurve::params_a/params_b` into `Imprint` instead of re-projecting;
`marched_polylines` de-`Surface3`-ified; classification via the F-Rep sign test
for NURBS operands (§5). **Expect of-9ia (open chain from a marched imprint on
two curved hosts) to bite** — check explicitly whether the fix generalizes.
**Gate:** the stress suite (below) green. **This is the promotion gate.**

**Phase 3 stress suite** — extend `crates/opensolid-brep/tests/boolean_stress.rs`
(2584 lines, already the model: `assert_valid`, `volume`, randomized rotations,
`random_transversal_block_pairs_volume_identity`,
`random_block_pairs_rotation_invariance`):
- A **NURBS patch of exact analytic form** (a bicubic cylinder, as of-pb7.3's
  tests already build) subtracted from a block must match the analytic
  cylinder's boolean to `tol.linear`. *This is the highest-value test in the
  suite* — it makes the exact path check itself against a known-good answer.
- Volume identity `vol(A)+vol(B) = vol(A∪B)+vol(A∩B)` to `1e-9` over randomized
  spline-profile bodies.
- Rotation and **knot-scaling** invariance (catches the `tol.parametric`
  normalization bug, §6).
- Closed manifold + `check()` + genus on every output.
- Trim regions abutting the knot-domain boundary.

**Phase 4 — tessellation quality (`of-37i.6`).** Curvature-derived lattice pitch
replacing the angular pitch; a NURBS arm in `tessellate.rs` for untrimmed
patches. **Gate:** worst chord deviation `≤` F-Rep cell on the stress corpus, so
NURBS booleans stop diverting to F-Rep on the deviation gate. *Accuracy only —
must not gate phase 3.*

**Phase 5 — hardening campaigns (`of-37i.7`; split into separate beads when
scheduled).** Degenerate-edge (collapsed
row) patches; tangential/coincident NURBS contact; ray–NURBS via Bézier clipping
to replace the F-Rep classification crutch; NURBS curve fitting of marched
polylines (the deferral recorded at `src/curve.rs:57`); `Curve3::Nurbs` and real
SP-curves.

## 10. Open questions

- **Does the of-9ia fix generalize?** Non-coaxial cone–cone and NURBS↔NURBS are
  the same shape — marched imprint, two curved hosts, chain must close. If one
  fix serves both, phase 3 shrinks and of-9ia closes for free. If not, phase 3
  is the longest pole in the epic. **This is the single biggest schedule
  uncertainty and should be probed first, in phase 2, before phase 3 is scoped.**
- **Is F-Rep-sign classification (§5) acceptable long-term?** It makes NURBS
  classification resolution-bound while the geometry is exact. Acceptable for
  promotion; the asymmetry should be stated in the phase-1 bead, and ray–NURBS
  scheduled rather than assumed.
- **What does `check()` assert about a NURBS face?** `src/check.rs` (1485 lines)
  verifies structural invariants, tolerance bounds, and Euler–Poincaré. Euler is
  surface-agnostic; the tolerance bounds may need a NURBS notion of "edge lies on
  face" (a projection residual, not a closed-form evaluation). Worth an early
  read in phase 1 — the README's policy is that field bugs become checker rules,
  so getting the rule right up front is cheaper than a campaign later.
- **Where do NURBS surfaces come from?** This note assumes they arrive from
  of-37i phase 1 (loft/sweep of spline profiles) or STEP. If phase 1 lands
  `Curve3::Nurbs` first, §1's snag (closed-but-not-periodic breaking the
  `is_periodic == is_closed` identity) is theirs to resolve, not this
  workstream's.
