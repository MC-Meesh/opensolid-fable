# Sketch edit tools: offset, trim/extend, convert-entities (of-fsl.21)

SolidWorks-parity sketch **edit** verbs — reshaping existing geometry rather
than drawing it. All the geometry is pure and unit-tested in
`src/lib/sketch/edit.js` (+ `src/lib/faceBoundary.js` for the convert source);
`SketchCanvas.jsx` wires them to the toolbar.

## Offset (`offsetEntity`)

Selection-based action (Edit group): select any entities, type a signed
distance, click **Offset**. Each selected entity gets a parallel copy:

- **Line** — perpendicular shift by `dist` along the left normal of its p1→p2
  direction (negative flips the side).
- **Circle / arc** — concentric copy at `radius + dist`; rejected (no-op) if
  that drives the radius to ≤ 0.

Offsets use fresh points, so they are independent of the original — no implicit
concentric/parallel constraint is added (the user can constrain afterwards).

## Trim (`trimEntityAt`) — tool `T`

Power-trim: pick the portion of an entity to remove; it trims back to the
nearest crossings with other geometry.

- **Line** — split at the crossings bracketing the pick; keep the outer
  piece(s) (0–2 survivors). No bracketing crossings → the whole line is
  deleted.
- **Circle** — with ≥ 2 crossings, becomes the complementary **arc** (the pick
  side is removed). Fewer than 2 crossings is a no-op.
- **Arc** — keep the outer sub-arc(s) around the removed span.

Length/radius dimension constraints on a trimmed entity are dropped (their
value no longer matches the new size); horizontal/vertical survive.

## Extend (`extendEntityAt`) — tool `X`

Pick near the end of a **line** or **arc**; the nearer endpoint is moved
outward to the next crossing with another entity's drawn extent. Nothing to
meet → no-op with a status hint. Circles (closed) cannot extend.

## Convert Entities (`convertEntities` + `faceBoundaryLoops`)

Projects outside geometry onto the sketch. The **Convert** button is enabled
only when the sketch was opened **on a face**: at entry, `App.jsx` extracts the
face region's boundary loops (`faceBoundaryLoops` — edges bordering exactly one
region triangle, chained and projected into the sketch's (u, v) frame) and
hands them to the canvas as `faceLoops`. Clicking Convert drops those loops in
as shared-endpoint lines.

### Honest MVP limits (next increments)

- Convert projects only the **sketched-on face's own outline**, not arbitrary
  picked model edges/loops of adjacent faces (SolidWorks lets you select any
  edge to convert). Named-plane sketches have no Convert source.
- The projected outline is faceted (marching-cubes boundary), emitted as line
  segments — not reconstructed analytic arcs/circles.
- Offset does not add a driving parallel/concentric relation; trim/extend are
  point-picked (no box/chain trim yet).
