// SVG export for 2D drawings (of-fsl.26.3, DRAWINGS.md §6) — the first 2D
// vector exporter in the repo (the existing STEP/STL/OBJ exporters are all 3D).
//
// A drawing is already 2D polylines + dimensions in sheet coordinates, so SVG
// is lossless and dependency-free string building: one `<g>` per placed view
// with a `<polyline>` per edge (dashed for hidden), one `<g>` per dimension
// with `<line>`/`<polygon>`/`<text>`, and an optional title block. Styling is
// inlined as presentation attributes so the file renders standalone, outside
// the app's CSS.
//
// SVG's y-axis grows downward; sheet y grows up. Rather than a flipping group
// transform (which would also mirror text), every coordinate is flipped once in
// JS — `Y = maxY - y` — keeping labels upright. Content bounds (line-work plus
// dimension geometry) drive the viewBox with a uniform margin.

import { dimensionGeometry, defaultMetrics } from './dimensions.js';

function esc(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}

// Grow a mutable bounds box `{minX,minY,maxX,maxY}` by a sheet point.
function grow(bb, [x, y]) {
  if (x < bb.minX) bb.minX = x;
  if (y < bb.minY) bb.minY = y;
  if (x > bb.maxX) bb.maxX = x;
  if (y > bb.maxY) bb.maxY = y;
}

// Round to a compact fixed precision (avoids 0.30000000000000004 in output).
function n(v) {
  return Number.isFinite(v) ? Math.round(v * 1000) / 1000 : 0;
}

/**
 * Serialize a placed sheet (sheet.js `createSheet` result) plus a list of
 * driven dimensions (dimensions.js) into a standalone SVG document string.
 *
 * @param sheet `{ scale, views: PlacedView[], bounds }`.
 * @param dims  array of `{ kind, … }` dimensions in sheet coordinates.
 * @param opts  `{ scale, margin, title, author, date, titleBlock }`.
 *              - `scale`: for the title block readout (defaults to sheet.scale).
 *              - `titleBlock`: include the title block (default false).
 * @returns an `<?xml?><svg>…</svg>` string. Empty sheet → a minimal valid SVG.
 */
export function sheetToSvg(sheet, dims = [], opts = {}) {
  const views = sheet?.views ?? [];
  const scale = opts.scale ?? sheet?.scale ?? 1;

  // Combined content bounds: view line-work + expanded dimension geometry.
  const bb = { minX: Infinity, minY: Infinity, maxX: -Infinity, maxY: -Infinity };
  for (const v of views) {
    for (const seg of v.segments) for (const p of seg.pts) grow(bb, p);
  }
  const diag =
    sheet?.bounds
      ? Math.hypot(
          sheet.bounds.maxX - sheet.bounds.minX,
          sheet.bounds.maxY - sheet.bounds.minY
        )
      : 0;
  const metrics = defaultMetrics(diag);
  const geoms = (dims ?? []).map((d) => dimensionGeometry(d, scale, metrics));
  for (const g of geoms) {
    for (const ln of g.lines) for (const p of ln) grow(bb, p);
    for (const ar of g.arrows) for (const p of ar) grow(bb, p);
    if (g.label) grow(bb, g.label.pos);
  }

  if (!Number.isFinite(bb.minX)) {
    // Nothing to draw — a minimal but valid document.
    return `<?xml version="1.0" encoding="UTF-8"?>\n<svg xmlns="http://www.w3.org/2000/svg" width="100" height="100" viewBox="0 0 100 100"></svg>\n`;
  }

  const margin = opts.margin ?? Math.max(diag * 0.06, metrics.arrow * 2, 4);
  const width = bb.maxX - bb.minX + margin * 2;
  const height = bb.maxY - bb.minY + margin * 2;
  // Map a sheet point into paper space: shift into the margin box and flip y.
  const fx = (x) => n(x - bb.minX + margin);
  const fy = (y) => n(bb.maxY - y + margin);

  const stroke = Math.max(diag * 0.0016, 0.15);
  const dash = `${n(stroke * 4)} ${n(stroke * 3)}`;

  const out = [];
  out.push('<?xml version="1.0" encoding="UTF-8"?>');
  out.push(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${n(width)}" height="${n(
      height
    )}" viewBox="0 0 ${n(width)} ${n(height)}">`
  );

  // --- views -----------------------------------------------------------------
  for (const v of views) {
    out.push(`  <g class="view view-${esc(v.view)}">`);
    for (const seg of v.segments) {
      const pts = seg.pts.map(([x, y]) => `${fx(x)},${fy(y)}`).join(' ');
      const dashAttr =
        seg.style === 'hidden' ? ` stroke-dasharray="${dash}"` : '';
      out.push(
        `    <polyline points="${pts}" fill="none" stroke="#1a1a1a" stroke-width="${n(
          seg.style === 'hidden' ? stroke * 0.7 : stroke
        )}" stroke-linecap="round" stroke-linejoin="round"${dashAttr} />`
      );
    }
    out.push('  </g>');
  }

  // --- dimensions ------------------------------------------------------------
  for (const g of geoms) {
    if (!g.lines.length && !g.arrows.length && !g.label) continue;
    out.push('  <g class="dim">');
    for (const ln of g.lines) {
      const [p, q] = ln;
      out.push(
        `    <line x1="${fx(p[0])}" y1="${fy(p[1])}" x2="${fx(q[0])}" y2="${fy(
          q[1]
        )}" stroke="#1157c4" stroke-width="${n(stroke * 0.8)}" />`
      );
    }
    for (const ar of g.arrows) {
      const pts = ar.map(([x, y]) => `${fx(x)},${fy(y)}`).join(' ');
      out.push(`    <polygon points="${pts}" fill="#1157c4" />`);
    }
    if (g.label) {
      out.push(
        `    <text x="${fx(g.label.pos[0])}" y="${fy(
          g.label.pos[1]
        )}" font-family="sans-serif" font-size="${n(
          g.label.height
        )}" fill="#1157c4" text-anchor="middle" dominant-baseline="middle">${esc(
          g.label.text
        )}</text>`
      );
    }
    out.push('  </g>');
  }

  // --- title block -----------------------------------------------------------
  if (opts.titleBlock) {
    out.push(titleBlockSvg({ width, height, scale, opts }));
  }

  out.push('</svg>');
  return out.join('\n') + '\n';
}

// Static title block anchored to the bottom-right of the paper. Coordinates are
// already in flipped paper space (top-left origin, y-down), so it is emitted
// directly rather than through the sheet-space flip.
function titleBlockSvg({ width, height, scale, opts }) {
  const bw = Math.min(Math.max(width * 0.34, 40), width - 4);
  const bh = Math.min(Math.max(height * 0.14, 18), height - 4);
  const x = width - bw - 2;
  const y = height - bh - 2;
  const fs = n(bh * 0.22);
  const rows = [
    ['Part', opts.title ?? 'Drawing'],
    ['Scale', `${scale}:1`],
    ['Date', opts.date ?? ''],
    ['By', opts.author ?? ''],
  ];
  const lines = [];
  lines.push('  <g class="title-block">');
  lines.push(
    `    <rect x="${n(x)}" y="${n(y)}" width="${n(bw)}" height="${n(
      bh
    )}" fill="none" stroke="#1a1a1a" stroke-width="${n(bh * 0.03)}" />`
  );
  const rowH = bh / rows.length;
  rows.forEach(([k, val], i) => {
    const ty = y + rowH * (i + 0.65);
    lines.push(
      `    <text x="${n(x + bw * 0.04)}" y="${n(
        ty
      )}" font-family="sans-serif" font-size="${fs}" fill="#1a1a1a">${esc(
        k
      )}: ${esc(val)}</text>`
    );
    if (i > 0) {
      const ly = y + rowH * i;
      lines.push(
        `    <line x1="${n(x)}" y1="${n(ly)}" x2="${n(x + bw)}" y2="${n(
          ly
        )}" stroke="#1a1a1a" stroke-width="${n(bh * 0.02)}" />`
      );
    }
  });
  lines.push('  </g>');
  return lines.join('\n');
}
