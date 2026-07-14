// Driven drawing dimensions — geometry only (of-fsl.26.3, DRAWINGS.md §4).
//
// A drawing dimension *measures*, it does not constrain (contrast the sketch
// solver's dimensional constraints): it is the driven, read-only sibling of a
// sketch dimension, so it reuses the sketch value-formatting (sketch/format.js)
// but none of the solver. Two kinds ship in the MVP:
//
//   linear  { kind, a, b, offset }        point-to-point distance
//   radius  { kind, center, rim }         radius from a center to an arc point
//
// Anchors (`a`/`b`/`center`/`rim`) are **sheet-space seed points** — the same
// scaled+placed coordinates the view line-work lives in (sheet.js). The value
// is therefore `sheetDistance / scale` (undoing the placement scale to recover
// model units). v1 dims are *static*: measured once from where the user clicked
// (associative re-resolve against a rebuilt view is deferred to of-fsl.26.5,
// DRAWINGS.md §8).
//
// This module is pure geometry — no React, no DOM — so both consumers render
// from one source of truth: the on-canvas overlay (DrawingCanvas, sheet→screen)
// and the SVG exporter (svg.js, sheet→paper). Each dimension expands to plain
// line segments, filled arrowhead triangles, and a positioned text label, all
// in sheet coordinates.

import { formatNumber } from '../sketch/format.js';

function sub(a, b) {
  return [a[0] - b[0], a[1] - b[1]];
}
function add(a, b) {
  return [a[0] + b[0], a[1] + b[1]];
}
function mul(a, s) {
  return [a[0] * s, a[1] * s];
}
function len(a) {
  return Math.hypot(a[0], a[1]);
}
function normalize(a) {
  const l = len(a);
  return l > 1e-12 ? [a[0] / l, a[1] / l] : [0, 0];
}
// Left normal: rotate 90° CCW.
function perp(a) {
  return [-a[1], a[0]];
}

/**
 * Distance between two sheet points, in **model units**: the raw sheet-space
 * distance divided by the placement `scale`. `scale` of 0/undefined is treated
 * as 1 (an unscaled sheet).
 */
export function sheetDistance(a, b, scale) {
  const d = len(sub(b, a));
  const s = scale || 1;
  return d / s;
}

/**
 * The measured value of a dimension in model units: point-to-point distance
 * for `linear`, center-to-rim radius for `radius`. Unknown kinds return 0.
 */
export function dimensionValue(dim, scale) {
  if (!dim) return 0;
  if (dim.kind === 'linear') return sheetDistance(dim.a, dim.b, scale);
  if (dim.kind === 'radius') return sheetDistance(dim.center, dim.rim, scale);
  return 0;
}

/** Display text for a dimension: bare number for linear, `R…` for radius. */
export function dimensionText(dim, scale) {
  const v = dimensionValue(dim, scale);
  return dim?.kind === 'radius' ? `R${formatNumber(v)}` : formatNumber(v);
}

/**
 * Metric lengths (in sheet units) for dimension rendering, scaled to a sheet's
 * bounding `diagonal` so annotations read the same regardless of drawing size.
 * Callers pass these to `dimensionGeometry`; `offset` is the default stand-off
 * used when a linear dim has no explicit `offset`.
 */
export function defaultMetrics(diagonal) {
  const base = diagonal > 0 ? diagonal * 0.018 : 1;
  return {
    arrow: base * 1.3, // arrowhead length
    halfWidth: base * 0.45, // arrowhead half-width
    gap: base * 0.5, // witness-line gap off the measured point
    extend: base * 0.7, // witness overrun past the dimension line
    offset: base * 5, // default linear stand-off
    textHeight: base * 1.7, // label font size
    textGap: base * 0.9, // label offset off the dimension line
  };
}

// Arrowhead triangle: tip at `tip`, pointing along unit `dir`, of length
// `m.arrow` and half-width `m.halfWidth`. Returns three sheet points.
function arrowhead(tip, dir, m) {
  const back = sub(tip, mul(dir, m.arrow));
  const side = mul(perp(dir), m.halfWidth);
  return [tip, add(back, side), sub(back, side)];
}

// Geometry of a linear (point-to-point) dimension.
function linearGeometry(dim, scale, m) {
  const { a, b } = dim;
  const off = dim.offset ?? m.offset;
  const s = off < 0 ? -1 : 1;
  const dir = normalize(sub(b, a));
  // Degenerate (coincident) anchors: nothing to draw sensibly.
  if (dir[0] === 0 && dir[1] === 0) {
    return { value: 0, text: formatNumber(0), lines: [], arrows: [], label: null };
  }
  const n = perp(dir); // left normal
  const oa = add(a, mul(n, off)); // dimension-line endpoints
  const ob = add(b, mul(n, off));

  // Witness lines: a small gap off each anchor, overrunning the dim line.
  const wa = [add(a, mul(n, s * m.gap)), add(oa, mul(n, s * m.extend))];
  const wb = [add(b, mul(n, s * m.gap)), add(ob, mul(n, s * m.extend))];
  const dimLine = [oa, ob];

  // Inward-pointing arrowheads at each dimension-line end.
  const arrows = [arrowhead(oa, dir, m), arrowhead(ob, mul(dir, -1), m)];

  const mid = mul(add(oa, ob), 0.5);
  const label = {
    pos: add(mid, mul(n, s * m.textGap)),
    text: formatNumber(dimensionValue(dim, scale)),
    height: m.textHeight,
  };
  return {
    value: dimensionValue(dim, scale),
    text: label.text,
    lines: [wa, wb, dimLine],
    arrows,
    label,
  };
}

// Geometry of a radius dimension: a leader from the center out through the arc
// point, arrowhead at the rim, `R…` label just past it.
function radiusGeometry(dim, scale, m) {
  const { center, rim } = dim;
  const dir = normalize(sub(rim, center));
  if (dir[0] === 0 && dir[1] === 0) {
    return { value: 0, text: 'R0', lines: [], arrows: [], label: null };
  }
  const leader = [center, rim];
  const arrows = [arrowhead(rim, dir, m)];
  const label = {
    pos: add(rim, add(mul(dir, m.arrow * 1.5), mul(perp(dir), m.textGap))),
    text: dimensionText(dim, scale),
    height: m.textHeight,
  };
  return {
    value: dimensionValue(dim, scale),
    text: label.text,
    lines: [leader],
    arrows,
    label,
  };
}

/**
 * Expand a dimension into drawable primitives in **sheet coordinates**:
 *
 *   { value, text,
 *     lines:  [[[x,y],[x,y]], …],       witness + dimension/leader lines
 *     arrows: [[[x,y],[x,y],[x,y]], …], filled arrowhead triangles
 *     label:  { pos:[x,y], text, height } | null }
 *
 * `metrics` comes from `defaultMetrics(diagonal)`. Both the on-canvas overlay
 * and the SVG exporter consume this so the two renderings never diverge.
 */
export function dimensionGeometry(dim, scale, metrics) {
  const m = metrics ?? defaultMetrics(0);
  if (dim?.kind === 'linear') return linearGeometry(dim, scale, m);
  if (dim?.kind === 'radius') return radiusGeometry(dim, scale, m);
  return { value: 0, text: '', lines: [], arrows: [], label: null };
}
