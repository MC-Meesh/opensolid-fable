# Assembly modeling

Status: design note only, no implementation (of-fsl.25)
Scope: how OpenSolid composes independent parts into an assembly document —
document model, mates, the rigid-body solve, GUI, script/module representation,
kernel needs, and the MCP surface. MVP scoping is called out explicitly in §8.

An assembly is a document that positions several *parts* relative to one
another and constrains how they fit — the bolt sits *concentric* to the hole,
its head face *coincident* with the boss. SolidWorks-parity means we need a
part/assembly split (each part is its own model), placed **instances** of those
parts, and **mates** that solve for where the instances actually go. This note
shows how each piece lands on the existing kernel, and — as with the fillet
note — leans on the fact that F-Rep instancing is nearly free.

## 1. Document model — parts and instances

Two document kinds:

- A **part document** is what the playground builds today: a script that
  `return`s one `Shape` (see `web/playground/src/lib/runScript.js`). Nothing
  changes about parts.
- An **assembly document** owns a set of **instances** and a set of **mates**.
  An instance is

  ```
  Instance = { part_ref, transform: Transform3, fixed: bool, name }
  ```

  where `part_ref` names a part document (another script/model) and `transform`
  is the placement in assembly space. `fixed` pins the instance (its transform
  is a solver constant — the assembly's ground); everything else is *floating*
  and its transform is solved from the mates.

Crucially an instance is **(part ref, transform)** — a reference plus a pose,
*not* a copy of the geometry. The same part referenced twice (two identical
bolts) is two instances sharing one part model. This is the whole reason
assemblies are cheap here: see §6.

The session already registers multiple named models — `Model { name, shape }`
in `crates/opensolid-kernel/src/session.rs`, keyed in an arena with undo/redo
over the whole registry. An assembly is a new session payload kind (`Model` is
generic over the payload, exactly so this can grow without touching the session
machinery): an `Assembly { instances, mates }` alongside the F-Rep `Model`s it
references. Undo/redo, journaling, and checkpoints come for free.

## 2. Mates

The MVP mate vocabulary, each a constraint between a **feature on instance A**
and a **feature on instance B** (a face, an edge, a reference plane/axis/point —
the reference geometry from of-fsl.14):

| Mate | Constrains | Removes DOF |
|------|------------|-------------|
| **Coincident** | two planar faces flush (or a point on a plane) | 3 (1 trans + 2 rot) for face–face |
| **Concentric** | two axes collinear (cylindrical/conical, or axis–axis) | 4 (2 trans + 2 rot) |
| **Distance** | two faces/points a fixed offset apart | 1 |
| **Angle** | two faces/axes at a fixed angle | 1 |
| **Parallel** | two faces/axes parallel | 2 |

A mate is stored as `{ kind, a: FeatureRef, b: FeatureRef, value? }` where
`value` carries the offset (distance) or angle. `FeatureRef` names *(instance,
feature)* using the same persistent-naming scheme the fillet note describes —
a face/axis identified by its generating operation plus a geometric anchor, so
it survives re-evaluation of the underlying part. Reference geometry (of-fsl.14)
gives us stable named planes/axes to mate against, which is far more robust than
mating to boolean-derived faces; **the MVP mates to reference geometry and
primitive faces first.**

### How a mate reduces to equations

Each floating instance carries a 6-DOF rigid transform: 3 translation + 3
rotation (we parameterize rotation as a unit quaternion, renormalized each
iteration, to avoid gimbal degeneracy). A mate contributes scalar residual
equations on the transformed features:

- **Coincident (face–face)**: the two face planes must be *anti-parallel and
  flush*. With outward normals `n_A`, `n_B` and points `p_A`, `p_B`:
  `n_A + n_B = 0` (they face into each other) and `n_A · (p_A − p_B) = 0`.
- **Concentric**: axis directions collinear (`d_A × d_B = 0`) and axis lines
  coincident (rejection of `p_A − p_B` off `d_A` is zero).
- **Distance**: `n_A · (p_A − p_B) − value = 0` for face–face; `|p_A − p_B| −
  value = 0` for point–point.
- **Angle**: `d_A · d_B − cos(value) = 0`.
- **Parallel**: `d_A × d_B = 0`.

All residuals are functions of the instances' transforms. Stack them into
`F(x) = 0` where `x` is the concatenation of every floating instance's 6 DOF.

### The solver

`F(x) = 0` is a square-ish nonlinear system solved by **Gauss–Newton /
Levenberg–Marquardt**: linearize `F` about the current pose (the Jacobian is
sparse and analytic — each residual touches only the two instances it names),
solve the normal equations for a step, apply, renormalize quaternions, repeat
to tolerance. This is the standard rigid-body constraint solve; it is small
(6 × #floating unknowns, typically tens) and fast.

Two properties worth stating honestly:

- **Common mate stacks are closed-form**, and we special-case them before
  reaching for the iterative solver: concentric + coincident (the canonical
  "drop a bolt in a hole and seat the head") fully determines a bolt's pose up
  to the free spin about the axis; a fastener seated this way needs no
  iteration. The iterative solver is the general fallback, not the common path.
- **Under-constrained is normal and fine.** A part with remaining DOF (the bolt
  free to spin, a slider free to slide) is not an error — the solver leaves
  those DOF at their current value and the GUI shows the part as movable
  (drag-to-move within the remaining freedom). **Over-constrained/conflicting**
  mates (residuals that cannot all reach zero) are detected by a non-converging
  LM with nonzero residual and surfaced as a mate error, not a crash.

Redundant-but-consistent mates (the usual SolidWorks "this mate is redundant"
case) are absorbed by LM's damping without special handling.

## 3. GUI (playground)

- **Assembly tree**: a left panel listing instances, each expandable to show
  its mates (mirrors the feature tree, but the top level is instances rather
  than features). The fixed instance is badged as ground. This reuses the
  existing tree component from the part feature tree.
- **Insert part**: an "insert component" action opens the model list (the
  session's other registered models, plus an import path for external
  parts/STEP later) and drops a new floating instance at the origin, ready to
  mate.
- **Mate creation**: the SolidWorks flow — click **face/edge on instance A**,
  then **face/edge on instance B**, and the panel offers the mates valid for
  that pair (two planar faces → coincident/distance/parallel; two cylindrical
  faces → concentric). Picking reuses the viewport face-region picking already
  built for fillets (`web/playground/src/lib/facePlane.js`); the pick resolves
  to a `FeatureRef` on the owning instance. On commit, the solver runs and the
  viewport snaps the instance into place with a short animation.
- **Live drag**: dragging an under-constrained instance re-runs the solver each
  frame with the drag as a soft target, so parts slide/spin within their
  remaining DOF.

## 4. Script / module representation

The playground is single-model today: one script returns one `Shape`. Assemblies
need **multiple named models per session** and a way for one script to reference
another — a small module system:

```js
// part: bolt
const bolt = Shape.cylinder(0.25, 2).union(
  Shape.cylinder(0.5, 0.3).translate(0, 0, 2) // head
);

// assembly script
const asm = Assembly.create();
const b1 = asm.insert(bolt, { fixed: false });   // instance = (part, transform)
const plate = asm.insert(plateModel, { fixed: true });
asm.mate.concentric(b1.axis('shaft'), plate.axis('hole1'));
asm.mate.coincident(b1.face('head_bottom'), plate.face('top'));
return asm; // solver runs, poses resolve
```

- A named-model registry backs `insert(model, …)`; the script references other
  models by handle, and the assembly script *is* the persisted source of truth
  (same single-source-of-truth principle as the fillet call written into the
  part script). Re-running the assembly script re-resolves feature refs and
  re-solves the mates.
- `asm.insert` returns an **instance handle** exposing named features
  (`.face(name)`, `.axis(name)`, `.plane(name)`) that resolve against that
  instance's part — these are the `FeatureRef`s the mates consume.
- The module boundary is deliberately thin: parts are plain scripts returning
  `Shape`s; the assembly layer only adds instancing + mates on top. No new
  geometry kernel concepts leak into part authoring.

## 5. Kernel needs

Almost everything the kernel needs already exists:

### Instancing without geometry duplication

An instance renders/evaluates its part *through its transform* — and the F-Rep
`Transformed` combinator (`crates/opensolid-frep/src/transform.rs`) already does
exactly this: `eval(p) = inner.eval(inverse * p)`, gradients rotate through the
chain rule, and `eval_interval` bounds the rotated box conservatively. So an
instance is `Transformed { sdf: part_shape, inverse }` — **zero geometry copied,
the transform applied at query time.** Ten instances of one bolt are ten thin
wrappers over one shared field. This is the F-Rep payoff: in a B-Rep kernel each
instance would clone and transform a topology graph; here it's a pointer plus a
matrix, applied lazily during the point query the mesher already makes.

Meshing an assembly is either (a) mesh each instance's transformed field
independently and concatenate — the honest MVP, one mesh per instance, correct
and trivial — or (b) union all instance fields into one field and mesh once for
a watertight combined body (needed for combined mass properties of a welded
assembly, not for a bolted one). MVP does (a).

### Interference (clash) detection

Two instances interfere iff their solids overlap — iff there exists a point
inside both, i.e. `max(sdf_A(p), sdf_B(p)) < 0` somewhere (the same `max` that
implements CSG intersection in `crates/opensolid-frep/src/csg.rs`). So
interference reduces to: **does the intersection field go negative?** We answer
it with the mesher's own interval machinery — `eval_interval` over the pair's
overlapping bounding boxes, subdividing only where the intersection interval
straddles zero, exactly like the adaptive octree prunes empty space. A negative
minimum is a clash; the region where it's negative is the interference volume
(and its mass properties give the clash volume). Cheap, robust, no B-Rep
surface–surface intersection required.

### Mass properties aggregation

`mass_properties` (`crates/opensolid-kernel/src/massprops.rs`) already returns
volume, surface area, centroid, and the inertia tensor about the centroid for
one closed mesh. Aggregating over an assembly is textbook rigid-body composition
*without* re-meshing the union:

- **Volume / area**: sum over instances (per-part density optional).
- **Centroid**: mass-weighted mean of instance centroids (each part's centroid
  pushed through its instance transform).
- **Inertia**: transform each part's tensor into assembly frame (rotate by the
  instance rotation) then **parallel-axis** shift to the assembly centroid, and
  sum. Standard `I = R Iₚ Rᵀ + m(|d|²E − d dᵀ)`.

This composes cached per-part results, so moving an instance re-aggregates in
microseconds — no geometry is touched. (Overlapping instances double-count in
the overlap; for a bolted assembly the overlap is the interference volume, which
should be ~zero — clash detection and correct mass properties reinforce each
other.)

## 6. MCP surface

One new agent-facing tool, alongside the existing `create_model` /
`get_screenshot` / `export` / `measure` / `validate` / `list_models`
(`tools/mcp-server/src/tools.js`):

- **`assemble`** — inputs: a list of `{ model_id, fixed? }` instances and a list
  of mates `{ kind, a: {instance, feature}, b: {instance, feature}, value? }`.
  Builds the assembly, runs the solver, and returns the resolved instance
  transforms, a convergence/over-constrained status, an interference report
  (clashing instance pairs + clash volume), and aggregate mass properties. As
  with `create_model`, the returned `assembly_id` is addressable by
  `get_screenshot` / `export` / `measure`.

This gives an agent the full loop: `create_model` each part → `assemble` with
mates → `get_screenshot` to see it → `measure`/interference to check the fit.
The mate-by-named-feature interface is more agent-legible than raw transforms:
"concentric(bolt.shaft, plate.hole1)" states intent, and the solver figures out
the pose.

## 7. Why this is robust (the F-Rep payoff, restated)

The two operations that make B-Rep assemblies expensive and fragile —
duplicating/transforming topology per instance, and surface–surface
intersection for clash detection — both collapse to trivial F-Rep queries here:
instancing is a lazy query-point remap (`Transformed`, already built), and
interference is `max(a, b) < 0` evaluated with the existing interval mesher.
The only genuinely new machinery is the rigid-body mate solver, which is a
small, well-understood Gauss–Newton system decoupled from the geometry kernel
and independently testable.

## 8. Honest MVP scoping

**In the MVP:**

- Fixed + floating instances; multiple instances of one part.
- Mates: **coincident, concentric, distance** (the trio that seats 90% of
  fasteners and brackets). Parallel/angle are a fast follow.
- Mate to **reference geometry (of-fsl.14) and primitive faces/axes** first —
  the most stable feature refs.
- Iterative LM solver with closed-form special-cases for concentric+coincident.
- Per-instance meshing (concatenate), interference detection, aggregate mass
  properties.
- Assembly script + `insert`/`mate` API; assembly tree + insert-part + mate-pick
  GUI.
- MCP `assemble` tool.

**Explicitly deferred (follow-up beads):**

- **Motion, mate limits, and DOF animation** — no kinematic simulation; the
  solver finds *a* valid static pose, it does not animate mechanisms.
- **Parallel/angle/tangent/width/gear/cam mates**, and **mate references**
  (pre-tagged snap features).
- **Sub-assemblies** (an assembly instanced inside another) — the model is
  recursive by design but the MVP is one level deep.
- **Exploded views, BOM, and interference *animation*.**
- **Belt/chain, path, and slot mates.**
- **In-context (top-down) editing** — editing a part from within the assembly.
- **STEP assembly import/export** (assembly structure, not just per-part solids).

## 9. Child beads

MVP phases filed as children of this bead (of-fsl.25):

1. **Kernel: instancing + interference + mass-property aggregation** — assembly
   data model, `Transformed`-backed instances, `max<0` clash detection, inertia
   composition. Pure kernel, fully testable.
2. **Mate solver** *(landed, of-fsl.25.2)* — 6-DOF rigid-body Levenberg–Marquardt
   over coincident / concentric / distance residuals, with closed-form
   concentric+coincident and over-constrained detection. Lives in the landed
   assembly module (`crates/opensolid-kernel/src/assembly/{mates,solver}.rs`):
   the solver works on instance *poses* (`Transform3` + `fixed`) and abstract
   `Feature`s, decoupled from part geometry, and `Assembly::solve` reads poses
   out of the MVP-1 instances. The LM step is taken in the Jacobian's SVD basis
   so the rank-deficiency of under-constrained parts (free DOF) is handled
   exactly — null directions get zero step and damping can relax to a full
   Gauss–Newton step.
3. **Script/module system + WASM binding** — multiple named models per session,
   `Assembly` / `insert` / `mate` API, instance-handle feature refs.
4. **GUI** — assembly tree, insert-part flow, face/edge-pick mate creation, live
   drag within remaining DOF.
5. **MCP `assemble` tool** — agent-facing assemble + interference + mass-props.
