// Driven drawing dimensions (of-fsl.26.3, DRAWINGS.md §4).
//
// A drawing dimension *measures* model geometry — it is driven (read-only), the
// opposite of a sketch constraint which *drives* geometry. It shares the sketch
// layer's value formatting (sketch/format.js) and the same arrow/witness-line
// visual language, minus the solver: no residual is minimized, the value is
// simply read off the placed view once (DRAWINGS.md §4).
//
// A dimension anchors to two points in *sheet* coordinates (the same space the
// placed view segments live in — sheet.js). v1 dims are **static**: the anchors
// are the raw points the user clicked and the value is measured at creation;
// re-resolving anchors against a rebuilt view (persistent edge refs) is deferred
// to of-fsl.26.5 (DRAWINGS.md §8 item 4). Sheet coordinates already include the
// sheet scale, so the model value is `sheetDistance / sheetScale`.
//
//   Dimension { id, kind: 'linear' | 'radius', a: [x,y], b: [x,y],
//               offset, value }
//     - linear: a,b are the two measured points; `offset` is the signed
//       perpendicular distance of the dimension line from the a→b segment.
//     - radius: a is the arc center, b a point on the arc; `offset` extends the
//       leader past the arc to seat the label.
//     - value: the model-unit measurement, frozen at creation.
//
// This module is framework-free (no three.js / React) so both the on-canvas
// overlay and the SVG exporter consume one geometry description, and it is unit
// testable in isolation like project.js / sheet.js.

import { formatNumber } from '../sketch/format.js';

function sub(a, b) {
  return [a[0] - b[0], a[1] - b[1]];
}
function len2(a) {
  return Math.hypot(a[0], a[1]);
}
function normalize(a) {
  const l = len2(a);
  return l > 0 ? [a[0] / l, a[1] / l] : [0, 0];
}
// Left-hand normal (90° CCW) of a 2D vector.
function perp(a) {
  return [-a[1], a[0]];
}
function add(a, b) {
  return [a[0] + b[0], a[1] + b[1]];
}
function scaleV(a, s) {
  return [a[0] * s, a[1] * s];
}

let dimSeq = 0;
/** Monotonic id for a new dimension (stable within a session). */
export function nextDimId() {
  dimSeq += 1;
  return `dim${dimSeq}`;
}

/**
 * Model-unit measurement between two sheet-space points, undoing the sheet
 * scale (`sheetDistance / scale`). Used for both linear length and radius.
 */
export function measure(a, b, scale = 1) {
  const s = scale > 0 ? scale : 1;
  return len2(sub(b, a)) / s;
}

/**
 * Human label for a dimension value, reusing the sketch readout formatting:
 * radius is prefixed `R`, linear is bare. Matches SketchCanvas dimension glyphs.
 */
export function formatDimLabel(kind, value) {
  const n = formatNumber(value);
  return kind === 'radius' ? `R${n}` : n;
}

/**
 * Build a linear (point-to-point) dimension anchored at sheet points `a`,`b`
 * offset perpendicular by `offset`; measures model length via `scale`.
 */
export function createLinearDim(a, b, offset = 0, scale = 1) {
  return {
    id: nextDimId(),
    kind: 'linear',
    a: [a[0], a[1]],
    b: [b[0], b[1]],
    offset,
    value: measure(a, b, scale),
  };
}

/**
 * Build a radius dimension: `center`→`rim` in sheet coordinates, `offset`
 * extends the leader past the rim; value is the model radius via `scale`.
 */
export function createRadiusDim(center, rim, offset = 0, scale = 1) {
  return {
    id: nextDimId(),
    kind: 'radius',
    a: [center[0], center[1]],
    b: [rim[0], rim[1]],
    offset,
    value: measure(center, rim, scale),
  };
}

// A filled arrowhead triangle whose tip is at `tip`, pointing along unit `dir`,
// with the given `size` (length). Returned as three points for a <polygon>.
function arrowhead(tip, dir, size) {
  const back = sub(tip, scaleV(dir, size));
  const wing = scaleV(perp(dir), size * 0.34);
  return [tip, add(back, wing), sub(back, wing)];
}

/**
 * Geometry primitives for a dimension, in the same (sheet) coordinate space as
 * its anchors, ready for either renderer:
 *
 *   { lines: [[[x,y],[x,y]], …],           // witness + dimension lines
 *     arrowheads: [[[x,y],[x,y],[x,y]], …], // filled triangles
 *     text: { pos: [x,y], angle, label } }  // angle in radians, label string
 *
 * `sizes` supplies the visual metrics in the anchor space (so an on-canvas
 * caller can pass screen-constant sizes and the exporter drawing-relative
 * ones): `{ arrow, ext, gap, textGap }`.
 *   - arrow:  arrowhead length
 *   - ext:    how far the witness line extends past the dimension line
 *   - gap:    gap between the measured point and the start of its witness line
 *   - textGap: offset of the label from the dimension/leader line
 */
export function dimensionGeometry(dim, sizes = {}) {
  const { arrow = 1, ext = 0.5, gap = 0, textGap = 0.6 } = sizes;
  if (dim.kind === 'radius') return radiusGeometry(dim, { arrow, textGap });
  return linearGeometry(dim, { arrow, ext, gap, textGap });
}

function linearGeometry(dim, { arrow, ext, gap, textGap }) {
  const a = dim.a;
  const b = dim.b;
  const dir = normalize(sub(b, a));
  // Degenerate (coincident) anchors: nothing sensible to draw.
  if (dir[0] === 0 && dir[1] === 0) {
    return { lines: [], arrowheads: [], text: null };
  }
  const n = perp(dir);
  const off = scaleV(n, dim.offset);
  const a2 = add(a, off); // dimension-line endpoints
  const b2 = add(b, off);

  // Witness lines run from a small gap off each measured point to just past the
  // dimension line (extension `ext`), along the offset normal.
  const sign = dim.offset >= 0 ? 1 : -1;
  const witnessDir = scaleV(n, sign);
  const w1a = add(a, scaleV(witnessDir, gap));
  const w1b = add(a2, scaleV(witnessDir, ext));
  const w2a = add(b, scaleV(witnessDir, gap));
  const w2b = add(b2, scaleV(witnessDir, ext));

  const lines = [
    [a2, b2], // dimension line
    [w1a, w1b], // witness 1
    [w2a, w2b], // witness 2
  ];
  const arrowheads = [
    arrowhead(a2, scaleV(dir, -1), arrow), // points outward toward witness 1
    arrowhead(b2, dir, arrow), // points outward toward witness 2
  ];

  // Label sits centered on the dimension line, nudged to its outward side, and
  // reads left-to-right (flip 180° when the segment points leftward).
  let angle = Math.atan2(dir[1], dir[0]);
  if (angle > Math.PI / 2) angle -= Math.PI;
  else if (angle < -Math.PI / 2) angle += Math.PI;
  const mid = scaleV(add(a2, b2), 0.5);
  const pos = add(mid, scaleV(n, textGap));
  return {
    lines,
    arrowheads,
    text: { pos, angle, label: formatDimLabel('linear', dim.value) },
  };
}

function radiusGeometry(dim, { arrow, textGap }) {
  const center = dim.a;
  const rim = dim.b;
  const dir = normalize(sub(rim, center));
  if (dir[0] === 0 && dir[1] === 0) {
    return { lines: [], arrowheads: [], text: null };
  }
  const outer = add(rim, scaleV(dir, dim.offset)); // leader past the arc
  const lines = [[center, outer]];
  // Arrowhead seats on the arc (rim), pointing outward along the radius.
  const arrowheads = [arrowhead(rim, dir, arrow)];
  const angle = 0; // radius labels read horizontally
  const pos = add(outer, scaleV(dir, textGap));
  return {
    lines,
    arrowheads,
    text: { pos, angle, label: formatDimLabel('radius', dim.value) },
  };
}

/** Sheet-coordinate bounds `{minX,minY,maxX,maxY}` of a dimension's geometry. */
export function dimensionBounds(dim, sizes = {}) {
  const g = dimensionGeometry(dim, sizes);
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  const acc = ([x, y]) => {
    if (x < minX) minX = x;
    if (y < minY) minY = y;
    if (x > maxX) maxX = x;
    if (y > maxY) maxY = y;
  };
  for (const line of g.lines) line.forEach(acc);
  for (const head of g.arrowheads) head.forEach(acc);
  if (g.text) acc(g.text.pos);
  return Number.isFinite(minX) ? { minX, minY, maxX, maxY } : null;
}
