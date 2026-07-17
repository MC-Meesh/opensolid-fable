# Differentiable CAD

Status: design note + MVP implemented (of-37i.1)
Scope: how OpenSolid computes derivatives of a part with respect to its
design parameters, and what that buys. Covers the AD machinery, parameter
extraction from the operation graph, field-based objectives, the smooth-
boolean temperature trick, the fixed-topology limitation and how an agent
works around it, B-Rep sensitivities, and the MCP surface. §8–§9 are design
sketches; §1–§6 ship in `opensolid-frep::diff`.

Ask a CAD kernel "how heavy is this bracket?" and it answers. Ask it "how
should I change the bracket to make it lighter without losing clearance?" and
it has nothing to say — you change a number, rebuild, measure, and guess
again. That loop is the bottleneck in every parametric workflow, and it is
the one an agent falls into too: an LLM nudging dimensions and re-measuring
is doing derivative-free optimisation by hand, badly, at one rebuild per
tool call.

A derivative closes the loop. If the kernel can report `∂mass/∂thickness`
alongside `mass`, the search stops being a guess: gradient descent walks a
ten-parameter part onto a mass target in tens of iterations, and the agent's
job changes from *nudging numbers* to *deciding what to optimise*. That is
the split this note argues for, and the MVP demonstrates: **gradients handle
the continuous inner loop; the agent handles the discrete outer loop.**

## 1. Two different gradients

The kernel already has a `grad`, so it is worth being blunt about why it is
not the one we need.

| | `Sdf::grad` (existing) | `ParamSdf::value_and_grad` (new) |
|---|---|---|
| Differentiates w.r.t. | the sample point `p` | the design parameters `θ` |
| Answers | "which way is out?" | "which way should the design move?" |
| Shape | `∇f ∈ ℝ³` | `∂f/∂θ ∈ ℝᴺ` |
| Used by | meshing, normals, projection | optimisation |

They are derivatives of the same function along different axes. `∇f` tells
you the surface normal at a point; it says nothing about what happens if the
fillet radius grows. Both exist, they do not replace each other, and the
naming in `diff` keeps them apart deliberately.

## 2. Forward-mode AD over a generic scalar

The field is a composition of arithmetic, `sqrt`, `abs`, `min`/`max`. Any
such composition can be differentiated exactly by carrying **dual numbers**
`v + Σ dᵢεᵢ` (with `εᵢεⱼ = 0`) through it: the nilpotent algebra applies the
chain rule automatically. No step size, no truncation error, no cancellation
— `dV/dr` comes out to machine precision, where a finite difference gives
maybe 8 digits and forces you to pick an `h`.

To get one implementation that runs at both `f64` and `Dual<N>`, field code
is written against a `Scalar` trait:

```rust
pub trait Scalar: Copy + Send + Sync + Add + Sub + Mul + Div + Neg + PartialOrd {
    fn cst(x: f64) -> Self;
    fn val(self) -> f64;
    fn sqrt(self) -> Self;  fn abs(self) -> Self;
    fn sin(self) -> Self;   fn cos(self) -> Self;
    fn exp(self) -> Self;   fn ln(self) -> Self;
    // min/max/clamp/relu default in terms of PartialOrd
}
```

`Scalar` is deliberately *not* `num_traits::Float`: the runtime dependency
budget is nalgebra + thiserror + rayon (`ROADMAP.md`), and the tower needs
only these dozen operations.

**Forward mode, not reverse.** Reverse mode (backprop) computes a gradient
in one pass regardless of `N`, and forward mode costs `N`-wide arithmetic.
Reverse wins when `N` is in the thousands — a neural net, or a topology
optimisation with a design variable per voxel. A CAD script has *tens* of
parameters, where forward mode is simpler (no tape, no graph, no
allocation), cache-friendly, trivially parallel, and `Copy`. `Dual<N>`
carries all `N` partials through a single evaluation, so a full gradient is
one forward pass. If per-voxel topology optimisation ever lands, it needs
reverse mode and that is a separate machine; nothing here blocks it.

### Why the point is a constant

Only the *parameters* are seeded as duals. The sample point is lifted as a
constant (`Vec3::from_point`). We differentiate the design, not where we
happened to sample it — `p` is a quadrature node, not a variable.

This is what makes the whole thing affordable. `Point3` is
`nalgebra::Point3<f64>`, pinned at `f64` and used everywhere; making the
*point* dual would mean genericising the core type aliases repo-wide.

### Why a parallel trait and not `Sdf<T: Scalar>`

The bead proposed genericising the existing trait to `Sdf<T: Scalar>`. That
turns out to be the wrong shape, for a concrete reason: **`Sdf` must stay
object-safe.** `Shape` is an `Arc<dyn Sdf>` and it is the composition handle
the entire kernel is built on; `mesh_sdf` takes `&dyn Sdf`. A generic method
is not dyn-compatible, so `Sdf<T: Scalar>` would either delete `Shape` or
force a `Shape<T>` monomorphised per scalar — infecting the mesher, the
refiner, the profile/sweep code, and the wasm boundary with a type parameter
that only the optimiser cares about. That is a repo-wide refactor of 12k
lines to serve one feature.

So the scalar generic goes on a **parallel trait** instead:

```rust
pub trait ParamSdf<const N: usize>: Send + Sync {
    fn field<T: Scalar>(&self, p: Vec3<T>, params: &[T; N]) -> T;
}
```

`ParamSdf` is not object-safe and does not need to be — it is used
generically. `Sdf` is untouched. `freeze(params)` bridges back:

```
ParamSdf<N> --freeze(θ)--> Frozen: impl Sdf --> mesh / render / export / STEP
```

An optimiser's output is an ordinary `Sdf`, so it flows through the existing
pipeline with no special cases.

**The cost, stated honestly:** `diff::field` restates every primitive's
closed form. That is duplication, and duplication drifts — a formula could be
fixed in `primitives.rs` and missed in `field.rs`, with nothing in the type
system to catch it. The mitigation is `tests/param_grad_fd.rs`, which pins
every tower function against its `Sdf` counterpart at 400 random points, so
drift fails the build. If the tower ever grows to cover profiles and sweeps
too, the better end state is to invert the relationship — write the geometry
*once* in the tower and define `Sdf` as its `f64` instantiation. That is a
mechanical follow-up (`of-37i.2`), not a redesign.

## 3. Where the derivative does not exist

Sharp CSG is `min`/`max`, which is not differentiable where the branches tie
— exactly on the edges and corners a solid model is made of. `Dual`'s
`min`/`max` compare **values only** and carry the winning branch's
derivative, which yields a *subgradient*: one of the one-sided derivatives,
chosen by a tie-break.

This is the same promise `Sdf::grad` already makes for spatial gradients on
edges, and in practice it is fine: the tie locus is measure-zero, so a
quadrature almost never lands on it, and an optimiser that steps onto a kink
steps off it again. What it is *not* is a licence to ignore. Two real
consequences:

1. **A subgradient can point at only one branch.** At a union seam,
   `∂f/∂θ` credits the winning child and reports exactly zero for the other,
   even though both surfaces are right there. An optimiser reading that
   gradient will happily shrink the ignored feature into a collision.
2. **FD is not a valid reference on a kink.** A central difference straddles
   the tie and averages two different one-sided slopes, so it disagrees with
   the (correct) subgradient. The gradient tests detect kinks by comparing FD
   at two step sizes and skip those samples — otherwise they would be
   asserting the wrong thing.

## 4. Smooth booleans as a temperature

The fix for both problems is already in the kernel: `SmoothUnion` is C¹, so
its parameter gradient is continuous *and* it mixes both children's
sensitivities (`h·∂a + (1-h)·∂b`) instead of picking one. Nothing on the seam
is invisible.

This makes the blend radius a **smoothing temperature**, in the
annealing sense:

- **Large radius** → a soft, well-conditioned field. Every feature within
  the band contributes gradient, so the optimiser sees the whole design and
  cannot get trapped behind a kink. But the geometry is visibly rounded — it
  is not the part you asked for.
- **Small radius** → the true sharp part, with a gradient that is
  informative only away from the seams.

So: **optimise warm, finish cold.** Run the search at a blend radius of a few
grid cells, then anneal it toward the real design value (or to sharp CSG) over
the last iterations. The converged geometry is the sharp one; the smooth field
was scaffolding for the search. This is the same trick as temperature in
simulated annealing or `σ` in a Gaussian-smoothed objective, and it costs
nothing to try because `SmoothUnion` is already the fast path.

A caveat worth stating: annealing changes the objective between iterations,
so the loss is not monotone across a temperature drop. Anneal on a schedule,
not inside the line search.

## 5. Objectives on the field, not the mesh

`mass_properties` (in `opensolid-kernel`) integrates over triangles via the
divergence theorem. It is exact, it is the right tool for *reporting*, and it
is **useless for optimisation**: mesh connectivity changes discontinuously as
parameters move — a vertex crosses a cell, a triangle appears — so `dV/dθ`
through a mesher is garbage. The mesh is a non-differentiable function of the
design. Differentiating through a mesher is a known research problem and not
one we need: the field is right there.

So integrate the field. With a smooth occupancy `s(f)` that ramps 1→0 across
the surface:

```
V(θ) = ∫ s(f(p; θ)) dp        m(θ) = ρ·V(θ)        c(θ) = ∫ p·s(f) dp / ∫ s(f) dp
```

Every term is smooth in `θ`, so duals carry `∂/∂θ` straight through the
quadrature. The centroid is a ratio of two integrals and the division is done
*in dual arithmetic*, so the quotient rule applies itself — no hand-derived
sensitivity to get wrong. Inertia tensors are the same pattern with a
`p⊗p` weight and are a mechanical addition.

`ds/df` is non-zero **only inside the band**, so the volume gradient is
supported on a shell around the surface. That is not an artefact: as
`band → 0` the sum converges to the classical **shape derivative**, a surface
integral over the boundary. The band is a discretised surface integral, and
the shell is where all the information is.

### The band is set by the gradient, not the value

The one genuinely subtle parameter. Because `dV/dθ` lives entirely in the
band, the midpoint rule must resolve the ramp derivative across it: if
`∫s' du ≠ -1`, **every gradient is scaled wrong by the same factor, at every
resolution**. Band and cell shrink together, so refining the grid does not
help — this is a bias you cannot buy your way out of.

That is not hypothetical. The first cut used a 1.5-cell band and a cubic
ramp: only two sample points landed in the band, `∫s' du` came to −0.889, and
a cube reported `dV/dh = 21.5` against an analytic 24 — 11% low, while the
*volume* looked fine. Worst-case error in `∫s' du`, over all sub-cell
alignments:

| band (cells) | 2 | **3** | 4 | 6 |
|---|---|---|---|---|
| cubic C¹ ramp | 6.3% | 2.8% | 1.6% | 0.7% |
| quintic C² ramp | 0.39% | **0.077%** | 0.02% | 0.005% |

The kernel's smoothness drives the quadrature order (Euler–Maclaurin: the
error is governed by the derivatives that fail to vanish at the ends of the
support). The cubic smoothstep is C¹ and leaves `s'' ≠ 0` there, converging
as `O(Δu²)`; the quintic smootherstep kills `s''` too and converges as
`O(Δu⁴)`. One extra polynomial term, 36× the accuracy, identical cost — so
`occupancy_ramp` is quintic and `BAND_CELLS = 3`.

The error is worst for **axis-aligned planar faces**, where every point
aliases against the grid identically and the errors add coherently; curved
surfaces sample the band at varying offsets and average it out. A sphere
looks fine at settings where a cube is 11% wrong. CAD parts are mostly
axis-aligned faces, so the aligned case is the one to tune for.

### What accuracy to expect

At resolution 64 on a 4-unit domain, measured against analytic values:

| | volume | `dV/dθ` |
|---|---|---|
| sphere | +1.5% | +0.5% |
| cube | +1.4% | +0.6% |

Both fall as `1/res²` (at res 160: +0.24% / +0.08%). The bias is *positive*
and has a clear cause — the smeared shell adds volume, and the outer half of
the band has more area than the inner half on a convex surface.

This is fine, and the reason is worth internalising: **an optimiser needs a
consistent, differentiable objective, not an exact one.** A 1% bias that
moves smoothly with `θ` steers perfectly well. The contract the tests enforce
is not "matches analytic" but the two things that actually matter — the AD
gradient matches FD *of the same quadrature* to 1e-3, and both value and
gradient converge as the grid refines. Steer with these; report final numbers
with the exact mesh-based `mass_properties`.

### Clearance

Clearance is a `min` of the field over a keep-out region — non-differentiable
where the winning probe changes, and worse, its gradient sees only *one*
probe, so an optimiser pushes the part off one probe straight into its
neighbour and chatters between them. The log-sum-exp **softmin** at
temperature `s` blends all near-active probes' gradients:

```
softmin(d) = m - s·ln Σ exp(-(dᵢ - m)/s),    m = min dᵢ
```

The shift by `m` is what makes it safe to evaluate: `exp(-(dᵢ-m)/s) ≤ 1`, so
a deeply-interior probe cannot overflow to `inf`. It factors straight back
out of the log, so the shift is exact, not an approximation. As `s → 0` it
converges to the hard min from below.

## 6. The optimiser is the weak link, not the gradients

The MVP ships `optimize::descend`: projected gradient descent, heavy-ball
momentum, Armijo backtracking line search. On the demo it converges in **19
iterations** onto an exact mass target. It is also, deliberately, the least
sophisticated part of this note, and it is worth being precise about where it
runs out — because the answer is *not* "the gradients are bad".

**What works.** Unconstrained or slack-constrained targeting is easy. The
bracket goes from 15.6% under its mass target to 0.0% in 19 iterations, with
two parameters moving together and no rebuild per step. Momentum matters here:
on a 600:1 valley it is worth >10× over plain steepest descent.

**What does not.** An **active** inequality constraint. Under a quadratic
penalty the iterate ends up riding the penalty wall, and progress requires
moving *along* the constraint boundary — a direction in which the objective
barely improves, while the gradient across the boundary is enormous. For the
bracket, holding clearance while gaining mass buys ~2.9 g/mm along the
boundary against a raw 10 g/mm straight up, with a curvature ratio of ~600
across it. Measured:

| | result |
|---|---|
| constraint slack | converges, 19–41 iters, mass exact |
| constraint active, 300 iters | −3.5% off target |
| constraint active, 20,000 iters | −1.78% off target |
| constraint active, + momentum | −3.2% — no real help |
| penalty weight 1 → 20 | no real help |

Momentum does not rescue it because the wall triggers adaptive restart almost
every iteration, so the method degenerates to plain descent. Nor is the
penalty weight a knob that helps: raising it tightens the constraint but
stiffens the wall, and the conditioning gets *worse* in exact proportion.
This is the textbook failure of a quadratic penalty plus a first-order method,
and the textbook answer is a method built for constraints — SLSQP, MMA, or an
augmented Lagrangian — none of which is 40 lines, and all of which want a
dependency the budget does not have (`ROADMAP.md`).

The reason to be relaxed about this: `descend` takes a plain
`Fn(&[f64; N]) -> (f64, [f64; N])` closure. Everything hard — the AD tower,
the field objectives, the shape derivative — produces exactly that pair, and
none of it changes when the optimiser is replaced. **The gradients are the
asset; the optimiser is a consumer of them, and a swappable one.** The demo is
scoped to the slack-constraint case so that what it claims, it actually shows.

## 7. Parameter extraction from the operation graph

The MVP hand-writes `impl ParamSdf<N>` per shape, which is fine for a demo
and wrong as a product. Users write playground scripts
(`web/playground/src/lib/runScript.js`), not Rust impls. The missing piece is
extracting `θ` from a script automatically.

The operation graph already has the information: a feature tree of ops with
numeric arguments (`Extrude { height: 20 }`, `Fillet { radius: 3 }`). What is
needed is:

1. **Declaration.** Mark which arguments are design variables, with bounds:
   `height: param(20, 5..40)`. Bounds are not optional in CAD — a wall
   thickness of −3 mm is not a worse design, it is not a design. Explicit
   declaration beats "every number is a parameter": most numbers are intent
   (a bolt circle that must stay a bolt circle), and a 200-dimensional
   optimisation over every literal in a script optimises the wrong thing.
2. **Collection.** Walk the tree, assign each declared param an index, and
   produce `[f64; N]` plus the bounds.
3. **Rebuild over `T`.** Evaluate the tree with `T` in place of `f64` at the
   declared slots. Every op's `field` already exists in the tower, so this is
   a graph walk, not new math — but it requires the tower to cover the ops
   the tree can hold, which today means extending it to profiles, sweeps and
   patterns (`of-37i.2`).

The natural end state is `const N` becoming runtime-sized, since a script's
parameter count is not known at compile time — either a `SmallVec`-backed
dual, or monomorphising over a few `N` buckets. That is a real design
decision and it is deferred, not solved here.

## 8. B-Rep sensitivities (future work)

Everything above is F-Rep. The B-Rep side can support the same story at fixed
topology: if the face/edge/vertex graph does not change, a control point's
position is a smooth function of the parameters, and surface/curve evaluation
is arithmetic that duals flow through exactly as they do here. The blocker is
not the math but the same generic-scalar question — NURBS eval is written
against `f64` — plus the fact that `SSI` and the boolean pipeline reconstruct
topology, so a boolean's output is not a smooth function of its inputs even
at "fixed" topology. The honest scope for B-Rep is: **sensitivities of a
parameterised feature that does not re-run a boolean.** That is `of-37i.3`,
and the hybrid F-Rep path covers the interesting cases meanwhile.

## 9. The fixed-topology limitation, and the agent loop

The deepest limitation is not accuracy, it is expressiveness. **A gradient
cannot change topology.** `∂mass/∂θ` can thin a rib; it cannot decide the
part wants *three* ribs instead of two, or that a hole should become a slot.
`N` is fixed for the whole descent, and the space of parts with 3 holes is
not reachable by walking continuously through the space of parts with 2. Nor
can a gradient add a feature whose parameter does not exist yet.

That is not a defect to engineer around — it is a clean seam, and it maps
exactly onto what each side is good at:

```
agent (outer loop, discrete)         gradients (inner loop, continuous)
  ├── propose topology               ├── fixed N parameters
  │   "add a gusset"                 │   thickness, radius, position
  │   "make it 3 ribs"               ├── descend to a local optimum
  │   "turn the hole into a slot"    │   tens of iterations, no rebuilds
  ├── declare params + bounds        └── report mass / CoG / clearance
  ├── ──────── hand off ───────────────────▶
  ◀─────────── converged design + achieved objectives ────────
  └── evaluate, revise topology, repeat
```

The agent does what it is good at — proposing discrete structure, reading a
spec, judging whether a design is sane. The gradient does what it is good at
— finding the best version of *that* structure, fast, without a rebuild per
step. Neither is doing the other's job badly.

This is why the gradient work matters beyond mass targeting: it is what makes
an agentic CAD loop tractable. An agent nudging dimensions burns a tool call
per iteration and converges like a random walk. An agent that proposes a
topology and gets back *the best part of that topology* is doing search over
a space small enough to actually cover.

## 10. MCP surface: an `optimize` tool

The MCP server exposes `create_model`, `measure`, `export`, `validate`,
`get_screenshot`, `list_models` (`tools/mcp-server/src/tools.js`). `optimize`
slots in as the tool that closes the loop: it is `measure`'s active
counterpart — `measure` reports, `optimize` *moves*.

**Sketch (not implemented):**

```jsonc
{
  "name": "optimize",
  "arguments": {
    "model_id": "bracket-1",
    "params": [                        // what may move, and how far
      { "name": "thickness", "min": 2.0,  "max": 12.0 },
      { "name": "fillet",    "min": 0.5,  "max": 8.0  }
    ],
    "objective": { "target": "mass", "value": 0.25, "units": "kg" },
    "constraints": [
      { "type": "clearance", "against": "keepout-A", "min": 1.5 },
      { "type": "mass", "max": 0.30 }
    ],
    "material": "aluminium-6061",
    "max_iters": 100
  }
}
```

Returns the converged parameters, the achieved objective and per-constraint
values, whether it converged or hit a bound/iteration cap, and the loss
history. It should **write the parameters back into the model** so the next
`get_screenshot`/`export` shows the optimised part — the whole point is that
the agent does not re-drive the geometry by hand.

Design constraints that fall out of §1–§9:

- **Params must be named and bounded by the caller.** Per §7, guessing which
  numbers are design variables optimises the wrong thing.
- **Report `converged: false` honestly**, with which parameters pinned to
  bounds. An agent that cannot tell "this is optimal" from "I ran out of
  iterations" will confidently ship the latter — and a parameter pinned at a
  bound is usually the interesting finding ("it wants to be thinner than you
  allowed"), not a failure.
- **Report the objective from the exact mesh-based `mass_properties`**, not
  the field quadrature that steered the search (§5). Steer with the biased
  estimate; report the exact one.
- **Anneal the blend temperature internally** (§4) and finish cold, so the
  returned model is the sharp part.
- **Topology is the caller's job** (§9). `optimize` moves numbers. An agent
  wanting a different structure edits the model and calls `optimize` again.

## 11. What ships now

In `opensolid-frep::diff`:

- `Scalar` / `Dual<N>` / `Vec3<T>` — forward-mode AD over a generic scalar.
- `field` — every primitive (sphere, box, rounded box, cylinder, torus, cone,
  capsule, half-space) and operator (sharp + smooth CSG, offset/shell/rounded,
  translate/scale), written once over `T: Scalar`, pinned against the existing
  `Sdf` impls so they cannot drift.
- `ParamSdf<N>` — exact `∂f/∂θ` in one pass; `freeze` bridges back to `Sdf`.
- `objective` — differentiable volume, mass, centroid, softmin clearance.
- `optimize` — projected gradient descent with an Armijo line search.
- `crates/opensolid-kernel/examples/optimize_bracket.rs` — a right-angle
  bracket driven onto a mass target under a clearance constraint, meshed and
  exported.

Not covered: profiles/sweeps/patterns in the tower (`of-37i.2`), script-level
parameter extraction (§7), B-Rep sensitivities (§8, `of-37i.3`), inertia
tensors (§5, mechanical), the `optimize` MCP tool (§10).
