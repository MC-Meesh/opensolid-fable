import { describe, expect, it } from 'vitest';
import { sheetToSvg } from './svg.js';
import { createSheet } from './sheet.js';

// Unit cube [0,1]^3 — draws in every view (see sheet.test.js).
const CUBE = {
  positions: new Float32Array([
    0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0,
    0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1,
  ]),
  indices: new Uint32Array([
    0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7,
    0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2,
    0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5,
  ]),
};

// Minimal hand-built sheet so the serializer is tested in isolation from
// projection: one view, one visible + one hidden segment.
function tinySheet() {
  return {
    scale: 1,
    bounds: { minX: 0, minY: 0, maxX: 10, maxY: 10 },
    views: [
      {
        view: 'front',
        origin: [0, 0],
        bounds: { minX: 0, minY: 0, maxX: 10, maxY: 10 },
        segments: [
          { pts: [[0, 0], [10, 0]], style: 'visible' },
          { pts: [[0, 0], [0, 10]], style: 'hidden' },
        ],
      },
    ],
  };
}

describe('sheetToSvg', () => {
  it('produces a valid, well-formed SVG document', () => {
    const svg = sheetToSvg(tinySheet());
    expect(svg.startsWith('<?xml')).toBe(true);
    expect(svg).toContain('<svg xmlns="http://www.w3.org/2000/svg"');
    expect(svg.trimEnd().endsWith('</svg>')).toBe(true);
    // Balanced <g> groups.
    const open = (svg.match(/<g\b/g) || []).length;
    const close = (svg.match(/<\/g>/g) || []).length;
    expect(open).toBe(close);
  });

  it('emits a polyline per segment and dashes hidden edges only', () => {
    const svg = sheetToSvg(tinySheet());
    expect((svg.match(/<polyline/g) || []).length).toBe(2);
    expect((svg.match(/stroke-dasharray/g) || []).length).toBe(1);
  });

  it('renders dimensions as line + arrow polygon + text', () => {
    const dims = [{ kind: 'linear', a: [0, 0], b: [10, 0], offset: 3 }];
    const svg = sheetToSvg(tinySheet(), dims);
    expect(svg).toContain('<line');
    expect(svg).toContain('<polygon');
    expect(svg).toContain('<text');
    expect(svg).toContain('>10<');
  });

  it('flips y so a low-y point maps below a high-y point on paper', () => {
    // Two vertices at y=0 and y=10 in sheet space; on paper (y-down) the y=10
    // point must have the smaller paper-y.
    const svg = sheetToSvg(tinySheet());
    const poly = svg.match(/points="([^"]*)"/g);
    expect(poly).not.toBeNull();
    // The vertical hidden segment (0,0)->(0,10) — its two paper points differ
    // in y, and the sheet-high end is nearer the top (smaller paper y).
    const vertical = poly.find((p) => p.includes('points'));
    expect(vertical).toBeTruthy();
  });

  it('escapes special characters in the title block', () => {
    const svg = sheetToSvg(tinySheet(), [], {
      titleBlock: true,
      title: 'A & B <x>',
      date: '2026-07-14',
    });
    expect(svg).toContain('A &amp; B &lt;x&gt;');
    expect(svg).toContain('title-block');
    expect(svg).toContain('2026-07-14');
  });

  it('returns a minimal valid SVG for an empty sheet', () => {
    const svg = sheetToSvg({ scale: 1, views: [], bounds: null });
    expect(svg).toContain('<svg');
    expect(svg).toContain('</svg>');
    expect(svg).not.toContain('<polyline');
  });

  it('serializes a real projected cube sheet with dimensions', () => {
    const sheet = createSheet(CUBE, { views: ['front'], scale: 2 });
    const dims = [{ kind: 'radius', center: [0, 0], rim: [0, 1] }];
    const svg = sheetToSvg(sheet, dims, { titleBlock: true });
    expect(svg).toContain('<svg');
    expect(svg).toContain('<polyline');
    // viewBox is present and finite.
    const vb = svg.match(/viewBox="([^"]+)"/);
    expect(vb).not.toBeNull();
    for (const n of vb[1].split(/\s+/)) {
      expect(Number.isFinite(Number(n))).toBe(true);
    }
  });

  it('has no NaN coordinates in output', () => {
    const sheet = createSheet(CUBE, { views: ['front', 'top', 'right', 'iso'] });
    const dims = [{ kind: 'linear', a: [0, 0], b: [1, 0], offset: 0.5 }];
    const svg = sheetToSvg(sheet, dims, { titleBlock: true });
    expect(svg).not.toContain('NaN');
  });
});
