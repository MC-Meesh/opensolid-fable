// SVG export for 2D drawings (of-fsl.26.3, DRAWINGS.md §6).
//
// A drawing is already 2D polylines + dimensions, so SVG is a lossless,
// dependency-free serialization: `<polyline>` per view segment (solid for
// visible, `stroke-dasharray` for hidden), one `<g>` per view, and per
// dimension a `<line>`/`<polygon>`/`<text>` group. This is the first 2D vector
// exporter in the repo (STEP/STL/OBJ are all 3D) and lives JS-side next to the
// sheet model, per §6 ("the MVP puts it JS-side to match where the sheet model
// lives"). PDF/DXF are format shims off this same data and are deferred (§9).
//
// Sheet coordinates are y-up (math convention); SVG is y-down, so points are
// flipped through `maxY - y` and translated by a margin. Visual metrics
// (stroke width, font size, arrowheads) are derived from the drawing's own
// diagonal so the output reads correctly regardless of model size, and are
// overridable via `opts`.

import { dimensionGeometry, dimensionBounds } from './dimensions.js';

// Merge sheet bounds with the bounds of every dimension so nothing clips.
function unionBounds(a, b) {
  if (!a) return b;
  if (!b) return a;
  return {
    minX: Math.min(a.minX, b.minX),
    minY: Math.min(a.minY, b.minY),
    maxX: Math.max(a.maxX, b.maxX),
    maxY: Math.max(a.maxY, b.maxY),
  };
}

// Round to a compact fixed precision to keep the SVG small and stable.
function n(v) {
  return Number(v.toFixed(3));
}

function escapeXml(s) {
  return String(s)
    .replace(/&/g, '&amp;')
    .replace(/</g, '&lt;')
    .replace(/>/g, '&gt;')
    .replace(/"/g, '&quot;');
}

/**
 * Serialize a sheet (sheet.js) plus its dimensions to a standalone SVG string.
 *
 * @param sheet `{ views: PlacedView[], bounds, scale }` from createSheet.
 * @param dims  array of Dimension (dimensions.js); may be empty/undefined.
 * @param opts  `{ margin, strokeWidth, hiddenWidth, fontSize, arrow, title,
 *                 titleBlock, date }` — all optional; visual metrics default to
 *                 fractions of the drawing diagonal.
 * @returns an `<svg>…</svg>` document string, or a minimal empty sheet when
 *          there is nothing to draw.
 */
export function sheetToSvg(sheet, dims = [], opts = {}) {
  const dimList = dims ?? [];
  // Size the dimension geometry relative to the drawing before measuring its
  // extent, so metrics and bounds agree.
  const base = sheet?.bounds ?? null;
  const roughDiag = base
    ? Math.hypot(base.maxX - base.minX, base.maxY - base.minY)
    : 1;
  const diag = roughDiag > 0 ? roughDiag : 1;

  const {
    margin = diag * 0.06 || 8,
    strokeWidth = diag * 0.003 || 0.4,
    hiddenWidth = diag * 0.002 || 0.3,
    fontSize = diag * 0.028 || 3,
    arrow = diag * 0.018 || 2,
    stroke = '#111111',
    dimStroke = '#1560c0',
    background = '#ffffff',
    title = null,
    titleBlock = false,
    date = null,
  } = opts;

  const dimSizes = {
    arrow,
    ext: arrow * 0.6,
    gap: arrow * 0.3,
    textGap: fontSize * 0.5,
  };

  // Overall bounds: sheet line-work unioned with every dimension's geometry.
  let bounds = base;
  for (const dim of dimList) {
    bounds = unionBounds(bounds, dimensionBounds(dim, dimSizes));
  }
  if (!bounds) {
    // Nothing to draw — emit a tiny valid empty sheet.
    return (
      `<svg xmlns="http://www.w3.org/2000/svg" width="16" height="16" ` +
      `viewBox="0 0 16 16"><rect width="16" height="16" fill="${background}"/></svg>`
    );
  }

  const w = bounds.maxX - bounds.minX + 2 * margin;
  const h = bounds.maxY - bounds.minY + 2 * margin;
  // Sheet (y-up) → SVG (y-down) point transform.
  const tx = (x) => n(x - bounds.minX + margin);
  const ty = (y) => n(bounds.maxY - y + margin);

  const parts = [];
  parts.push(
    `<svg xmlns="http://www.w3.org/2000/svg" width="${n(w)}" height="${n(h)}" ` +
      `viewBox="0 0 ${n(w)} ${n(h)}">`
  );
  parts.push(`<rect width="${n(w)}" height="${n(h)}" fill="${background}"/>`);

  // ---- views --------------------------------------------------------------
  for (const view of sheet?.views ?? []) {
    parts.push(`<g class="view view-${escapeXml(view.view)}">`);
    for (const seg of view.segments) {
      const pts = seg.pts.map(([x, y]) => `${tx(x)},${ty(y)}`).join(' ');
      const hidden = seg.style === 'hidden';
      const attrs =
        `fill="none" stroke="${stroke}" ` +
        `stroke-width="${hidden ? n(hiddenWidth) : n(strokeWidth)}" ` +
        `stroke-linecap="round" stroke-linejoin="round"` +
        (hidden ? ` stroke-dasharray="${n(arrow)} ${n(arrow * 0.6)}"` : '');
      parts.push(`<polyline points="${pts}" ${attrs}/>`);
    }
    parts.push('</g>');
  }

  // ---- dimensions ---------------------------------------------------------
  if (dimList.length) {
    parts.push('<g class="dimensions">');
    for (const dim of dimList) {
      const g = dimensionGeometry(dim, dimSizes);
      for (const line of g.lines) {
        const [p0, p1] = line;
        parts.push(
          `<line x1="${tx(p0[0])}" y1="${ty(p0[1])}" ` +
            `x2="${tx(p1[0])}" y2="${ty(p1[1])}" ` +
            `stroke="${dimStroke}" stroke-width="${n(strokeWidth * 0.75)}"/>`
        );
      }
      for (const head of g.arrowheads) {
        const pts = head.map(([x, y]) => `${tx(x)},${ty(y)}`).join(' ');
        parts.push(`<polygon points="${pts}" fill="${dimStroke}"/>`);
      }
      if (g.text) {
        const [x, y] = g.text.pos;
        // Negate the sheet-space angle for the y-flipped SVG frame.
        const deg = n((-g.text.angle * 180) / Math.PI);
        const px = tx(x);
        const py = ty(y);
        const rot = deg !== 0 ? ` transform="rotate(${deg} ${px} ${py})"` : '';
        parts.push(
          `<text x="${px}" y="${py}" font-size="${n(fontSize)}" ` +
            `fill="${dimStroke}" text-anchor="middle" ` +
            `font-family="sans-serif"${rot}>${escapeXml(g.text.label)}</text>`
        );
      }
    }
    parts.push('</g>');
  }

  // ---- optional title block -----------------------------------------------
  if (titleBlock || title) {
    const bh = fontSize * 3;
    const bw = Math.min(w * 0.5, fontSize * 16);
    const bx = n(w - bw);
    const by = n(h - bh);
    parts.push(
      `<g class="title-block"><rect x="${bx}" y="${by}" width="${n(bw)}" ` +
        `height="${n(bh)}" fill="none" stroke="${stroke}" ` +
        `stroke-width="${n(strokeWidth)}"/>`
    );
    const lines = [];
    if (title) lines.push(escapeXml(title));
    const meta = [];
    if (sheet?.scale != null) meta.push(`Scale ${sheet.scale}:1`);
    if (date) meta.push(escapeXml(date));
    if (meta.length) lines.push(meta.join('   '));
    let ly = by + fontSize * 1.2;
    for (const text of lines) {
      parts.push(
        `<text x="${n(bx + fontSize * 0.4)}" y="${n(ly)}" ` +
          `font-size="${n(fontSize)}" fill="${stroke}" ` +
          `font-family="sans-serif">${text}</text>`
      );
      ly += fontSize * 1.3;
    }
    parts.push('</g>');
  }

  parts.push('</svg>');
  return parts.join('\n');
}
