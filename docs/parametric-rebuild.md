# Parametric Feature Rebuild & Persistent References (of-fsl.8)

> Design note. Read this before touching `persistentRef.js`, `rebuildState.js`,
> or the face-sketch wiring in `App.jsx`.

The defining SolidWorks behavior: **edit an upstream feature and everything
downstream regenerates.** This note scopes what that means in the OpenSolid
playground, where the model is a traced construction tree (of-4eh.19) and the
script is a regenerated view over it.

## 1. Downstream re-evaluation — already load-bearing

The playground already replays the whole graph on every edit. `commitScript`
is the single store commit: any mutation (script keystroke, gizmo, property
panel, palette add, sweep apply, feature delete) rewrites the script and calls
`evaluateScript`, which re-runs the *entire* traced program top to bottom.

Because the script is a linear program (`const s1 = ...; const s2 = s1...;
return ...`) and `serializeTree` emits statements in creation order, "re-run the
whole program" **is** topological re-evaluation: a node cannot be defined before
its inputs, so replaying the statement list respects the dependency order. There
is no separate scheduler to build. Editing `Box1`'s height and watching every
downstream boolean/transform/sweep regenerate works today and is covered by the
`propertyEdit` / `transformEdit` / `sceneTree` suites.

What that machinery does **not** handle is references that point at *geometry*
rather than at a named variable — the persistent-naming problem below.

## 2. The persistent-naming problem

A feature can reference a **picked face or edge** rather than a named plane:

- **Sketch on a face** (shipped, of-4eh.16): click a planar face, sketch on it,
  extrude. The face's plane `{ origin, normal, u, v }` is detected from the
  displayed mesh and **baked into the sweep as literal `rotate`/`translate`
  numbers** (see `lib/sweep.js`).
- **Up-to-face / fillet-edge** (future): same shape of problem — the feature is
  anchored to a piece of boundary that has no stable name.

Baked coordinates are the failure. Grow the box the face sits on and the face
moves, but the baked numbers stay put — the extrude floats where the *old* face
was, and nothing tells the user. A parametric kernel must instead store a
**stable reference** that re-resolves to "the same face" after the upstream
feature changes. This is the classic persistent-naming (a.k.a. topological
naming) problem; no scheme solves it in full generality, so we scope an honest
MVP and name its failure mode explicitly.

### Reference scheme (MVP)

A face reference is intentionally geometric, not topological (we have no stable
B-Rep face ids at the F-Rep meshing layer yet):

```
FaceRef = { owner, normal: [x,y,z], anchor: [x,y,z], extent }
```

- `owner` — the feature key the reference belongs to (`extrude:1`), so it
  survives re-evaluation the same way renames/visibility do (deterministic keys).
- `normal` — the unit outward normal of the picked planar region (**orientation**).
- `anchor` — the region centroid in world space (**nearest-point heuristic**).
- `extent` — the region radius, used only to scale the match tolerance.

**Resolution.** On rebuild we enumerate the planar regions of the current mesh
and pick the region that (a) has a normal within `NORMAL_TOL_DEG` of the stored
`normal` and (b) whose centroid is nearest the stored `anchor`, provided that
distance is within `ANCHOR_TOL` (a fraction of the model's size). The winning
region yields a fresh plane and a refreshed `anchor`, so the reference **tracks**
the face across successive edits (each rebuild re-locks onto where the face is
now, not where it was first picked).

Orientation gates before distance on purpose: two parallel faces an edit-width
apart (e.g. top and bottom of a thin plate) are disambiguated by proximity, but
a coplanar-but-opposite face (a hole's far wall) is rejected outright by normal.

**Failure mode — dangling.** If no region clears both gates (the face was
booleaned away, shrank below the planar threshold, or flipped), the reference is
**dangling**: it keeps its last-known plane so the feature still renders, but the
feature is flagged and the tree shows a dangling badge (SolidWorks paints such
features with a red "!" error overlay). The user then re-picks a face or deletes
the feature — exactly the SW recovery gesture.

### Known limits (honest MVP)

1. **Resolution is against the full displayed mesh, not the rollback body.** The
   robust answer resolves each reference against the model *rolled back to just
   before its own feature* (so the feature's own geometry can't mask the face it
   sits on). We have the machinery for that (`pruneTree` + `serializeTree` +
   re-mesh) but it costs a mesh per reference per rebuild, so the MVP resolves
   against the whole mesh and documents this. Detecting a *vanished* face (the
   dangling case) is unaffected; only a face fully swallowed by its own feature
   could mis-resolve, and that is called out here rather than hidden.
2. **No automatic geometric re-bake yet.** A live reference updates its stored
   plane, but the MVP does **not** rewrite the baked sweep coordinates in the
   script — so a face-sketch's geometry does not yet slide with the face on its
   own. Re-baking is a fixpoint (mesh depends on the sweep, the sweep's plane
   depends on the mesh) and must be done against the rollback body from limit 1
   to converge safely; it is the next increment, not this one.
3. **Faces only.** Edge references (fillet selections, up-to-edge) reuse the same
   `owner/orientation/anchor` shape but need an edge extractor; out of scope here.

## 3. Rebuild state in the feature tree

Every feature carries one of three rebuild states, surfaced as a badge:

| State      | Meaning                                                        | Badge |
|------------|----------------------------------------------------------------|-------|
| `ok`       | evaluated cleanly; any references resolved                     | none  |
| `dangling` | a persistent reference could not be re-resolved                | red ! |
| `error`    | the feature itself failed to evaluate                          | red ⊘ |

`error` is modeled for completeness. Today evaluation fails whole-script (the
error banner already covers that), so only `ok`/`dangling` are reachable in the
MVP; per-feature error isolation is a later increment that will light up `error`.

## Module map

- `lib/persistentRef.js` — `faceRefFromPlane`, `planarRegionsOf`, `resolveFaceRef`.
  Pure; unit-tested without WASM.
- `lib/rebuildState.js` — folds resolved references into a per-feature-key status
  map for the tree. Pure; unit-tested.
- `components/FeatureTree.jsx` — renders the badge.
- `App.jsx` — captures a `FaceRef` when a sketch is applied on a face, re-resolves
  every reference after each rebuild, and feeds the status map to the tree.
