// Drawing sheet data model + view placement (of-fsl.26.2, DRAWINGS.md §3).
//
// A sheet is a scale plus an ordered list of placed views. Each view is
// projected 2D line-work (project.js) positioned so the standard orthographic
// set aligns the way engineers read it: front anchors the sheet; top stacks
// with front (shared X); right sits beside front (shared Y); iso fills a
// corner. Placement is pure arithmetic on view bounds — no kernel support.
//
//   Sheet   { size, scale, angle, views: PlacedView[], bounds }
//   PlacedView { view, origin: [x, y], segments: Segment2D[], bounds }
//   Segment2D  { pts: [[x, y], …], style: 'visible' | 'hidden' }
//
// Third-angle (US default): top ABOVE front, right to the RIGHT of front.
// First-angle (ISO): top BELOW, right to the LEFT — the same arithmetic with a
// sign flip. Segments are emitted already in sheet coordinates (scaled +
// translated) so the overlay just draws them.

import { projectView } from './project.js';

/** Standard MVP view set, in reading order. */
export const DEFAULT_VIEWS = ['front', 'top', 'right', 'iso'];

// Gap between adjacent views, as a fraction of the largest view's scaled span.
const GAP_FRAC = 0.18;

function viewSpan(bounds) {
  if (!bounds) return { w: 0, h: 0, cx: 0, cy: 0 };
  return {
    w: bounds.maxX - bounds.minX,
    h: bounds.maxY - bounds.minY,
    cx: (bounds.minX + bounds.maxX) / 2,
    cy: (bounds.minY + bounds.maxY) / 2,
  };
}

// Place a projected view's segments into sheet coordinates: recenter on the
// view's own center, scale, then translate to the sheet origin `[ox, oy]`.
function placeSegments(projected, span, scale, ox, oy) {
  return projected.segments.map((seg) => ({
    pts: seg.pts.map(([x, y]) => [
      (x - span.cx) * scale + ox,
      (y - span.cy) * scale + oy,
    ]),
    style: seg.style,
  }));
}

// Sheet-coordinate bounds of a list of placed segments.
function segmentsBounds(segments) {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const seg of segments) {
    for (const [x, y] of seg.pts) {
      minX = Math.min(minX, x);
      minY = Math.min(minY, y);
      maxX = Math.max(maxX, x);
      maxY = Math.max(maxY, y);
    }
  }
  return Number.isFinite(minX) ? { minX, minY, maxX, maxY } : null;
}

/**
 * Compute sheet origins `[x, y]` for each requested view, aligned per
 * projection angle. Exposed for testing the layout arithmetic directly.
 *
 * `spans` maps view name -> `{ w, h, cx, cy }` in unscaled view units.
 * Returns a map view -> `[ox, oy]` in sheet units for the views present.
 */
export function viewOrigins(spans, scale, angle = 'third', gap) {
  const present = (name) => spans[name] && (spans[name].w > 0 || spans[name].h > 0);
  const maxSpan = Object.values(spans).reduce(
    (m, s) => Math.max(m, s.w, s.h),
    0
  );
  const g = gap ?? maxSpan * scale * GAP_FRAC;
  const sign = angle === 'first' ? -1 : 1;
  const origins = {};
  const front = spans.front;
  origins.front = [0, 0];

  if (present('top') && front) {
    // Shared X with front; stacked vertically (third: above → +y).
    const dy = ((front.h + spans.top.h) / 2) * scale + g;
    origins.top = [0, sign * dy];
  } else if (present('top')) {
    origins.top = [0, 0];
  }

  if (present('right') && front) {
    // Shared Y with front; beside front (third: right → +x).
    const dx = ((front.w + spans.right.w) / 2) * scale + g;
    origins.right = [sign * dx, 0];
  } else if (present('right')) {
    origins.right = [0, 0];
  }

  if (present('iso')) {
    // Diagonal corner: horizontally by the right column, vertically by the
    // top row, so it never overlaps the orthographic set.
    const dx = origins.right ? origins.right[0] : (front ? (front.w / 2) * scale + g : 0);
    const dy = origins.top ? origins.top[1] : (front ? (front.h / 2) * scale + g : 0);
    origins.iso = [dx, dy];
  }

  // Views with no anchor (e.g. front absent) fall back to the origin.
  for (const name of Object.keys(spans)) {
    if (present(name) && !origins[name]) origins[name] = [0, 0];
  }
  return origins;
}

/**
 * Build a drawing sheet from a mesh.
 *
 * @param mesh  `{ positions, indices, featureEdges? }` (the App mesh state).
 * @param opts  `{ views, scale, angle, size, gap }`.
 *              - `views`: view names to place (default DEFAULT_VIEWS).
 *              - `scale`: sheet units per model unit (default 1).
 *              - `angle`: `'third'` (default) or `'first'`.
 * @returns `{ size, scale, angle, views: PlacedView[], bounds }`. Views that
 *          project to nothing (empty mesh, edge-on) are omitted.
 */
export function createSheet(mesh, opts = {}) {
  const {
    views = DEFAULT_VIEWS,
    scale = 1,
    angle = 'third',
    size = null,
    gap,
  } = opts;

  const projected = {};
  const spans = {};
  for (const name of views) {
    const p = projectView(mesh, name);
    if (!p.bounds) continue; // nothing drawn in this view
    projected[name] = p;
    spans[name] = viewSpan(p.bounds);
  }

  const origins = viewOrigins(spans, scale, angle, gap);

  const placed = [];
  for (const name of views) {
    const p = projected[name];
    if (!p) continue;
    const [ox, oy] = origins[name] ?? [0, 0];
    const segments = placeSegments(p, spans[name], scale, ox, oy);
    placed.push({
      view: name,
      origin: [ox, oy],
      segments,
      bounds: segmentsBounds(segments),
    });
  }

  const bounds = segmentsBounds(placed.flatMap((v) => v.segments));
  return { size, scale, angle, views: placed, bounds };
}

/**
 * A pan/zoom view `{ cx, cy, scale }` (sheet center + px per sheet unit) that
 * frames `bounds` in a viewport `size` `{ w, h }` with `pad` fractional margin.
 * Returns a centered unit-scale view when bounds/size are missing or empty.
 */
export function fitView(bounds, size, pad = 0.12) {
  if (!bounds || !size || size.w <= 0 || size.h <= 0) {
    return { cx: 0, cy: 0, scale: 60 };
  }
  const w = Math.max(bounds.maxX - bounds.minX, 1e-6);
  const h = Math.max(bounds.maxY - bounds.minY, 1e-6);
  const scale = Math.min(size.w / w, size.h / h) * (1 - pad);
  return {
    cx: (bounds.minX + bounds.maxX) / 2,
    cy: (bounds.minY + bounds.maxY) / 2,
    scale: Number.isFinite(scale) && scale > 0 ? scale : 60,
  };
}
