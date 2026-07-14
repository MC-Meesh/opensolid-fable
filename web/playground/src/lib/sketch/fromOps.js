// Rebuild an editable sketch from a sweep feature's profile snapshot —
// the inverse of profileToOps (lib/sweep.js), enabling "edit sketch" on an
// existing Extrude/Revolve feature.
//
// A snapshot is `{ start: [x, y], segs: [{ x, y, bulge }] }` where each seg
// runs from the previous point to (x, y); `bulge = tan(sweep / 4)` (DXF
// convention, positive counter-clockwise, 0 for a line). Segments become
// chained line/arc entities sharing endpoint ids; when the last seg does not
// land back on the start (scripts rely on `close()` for the final leg), a
// closing line is added so the sketch extracts as a closed profile again.

import { createSketch, addPoint, addLine, addArc } from './model.js';

/**
 * Arc center for a chord A→B with the given bulge.
 * Included angle θ = 4·atan(bulge) (signed); the center sits at distance
 * d = |AB| / (2·tan(θ/2)) along the chord's left normal — the sign of d
 * puts it on the correct side for both windings and major/minor arcs.
 */
export function bulgeArcCenter(ax, ay, bx, by, bulge) {
  const theta = 4 * Math.atan(bulge);
  const dx = bx - ax;
  const dy = by - ay;
  const len = Math.hypot(dx, dy);
  const d = len / (2 * Math.tan(theta / 2));
  return [
    (ax + bx) / 2 + (-dy / len) * d,
    (ay + by) / 2 + (dx / len) * d,
  ];
}

const CLOSE_TOL = 1e-9;

/**
 * True if any segment is a curved (ellipse / spline) kind that the sketch
 * model cannot represent yet — such profiles cannot round-trip back into an
 * editable sketch (see the of-fsl.22 follow-up for interactive curve tools).
 */
export function opsHaveCurvedSegs(ops) {
  return ops.segs.some((s) => s.kind === 'ellipse' || s.kind === 'spline');
}

/**
 * Build a sketch whose entities trace the profile snapshot. Returns a fresh
 * sketch (model.js shape) with chained lines/arcs; consecutive segments and
 * the closing joint share point ids, so the loop is closed by construction.
 */
export function sketchFromOps(ops) {
  if (opsHaveCurvedSegs(ops)) {
    throw new Error(
      'this profile contains ellipse/spline segments, which cannot be edited as a sketch yet'
    );
  }
  const sketch = createSketch();
  const startId = addPoint(sketch, ops.start[0], ops.start[1]);
  let prevId = startId;
  let prev = ops.start;

  const segs = ops.segs.slice();
  // A snapshot whose last seg already lands on the start closes through it.
  const last = segs[segs.length - 1];
  const lastClosed =
    last &&
    Math.hypot(last.x - ops.start[0], last.y - ops.start[1]) <= CLOSE_TOL;

  segs.forEach((seg, i) => {
    const isLast = i === segs.length - 1;
    const endId =
      isLast && lastClosed ? startId : addPoint(sketch, seg.x, seg.y);
    if (seg.bulge === 0) {
      addLine(sketch, prevId, endId);
    } else {
      const [cx, cy] = bulgeArcCenter(prev[0], prev[1], seg.x, seg.y, seg.bulge);
      const centerId = addPoint(sketch, cx, cy);
      addArc(sketch, centerId, prevId, endId, seg.bulge > 0);
    }
    prevId = endId;
    prev = [seg.x, seg.y];
  });

  if (!lastClosed && segs.length > 0) {
    addLine(sketch, prevId, startId);
  }
  return sketch;
}
