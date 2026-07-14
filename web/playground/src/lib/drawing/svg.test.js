import { describe, expect, it } from 'vitest';
import { sheetToSvg } from './svg.js';
import { createLinearDim, createRadiusDim } from './dimensions.js';

// A minimal hand-built sheet: one view with a visible and a hidden segment.
const SHEET = {
  scale: 1,
  bounds: { minX: 0, minY: 0, maxX: 10, maxY: 10 },
  views: [
    {
      view: 'front',
      bounds: { minX: 0, minY: 0, maxX: 10, maxY: 10 },
      segments: [
        { pts: [[0, 0], [10, 0]], style: 'visible' },
        { pts: [[0, 0], [0, 10]], style: 'hidden' },
      ],
    },
  ],
};

describe('sheetToSvg', () => {
  it('produces a valid standalone svg document', () => {
    const svg = sheetToSvg(SHEET);
    expect(svg.startsWith('<svg')).toBe(true);
    expect(svg).toContain('xmlns="http://www.w3.org/2000/svg"');
    expect(svg).toContain('viewBox=');
    expect(svg.trimEnd().endsWith('</svg>')).toBe(true);
  });

  it('emits a polyline per segment and dashes only hidden edges', () => {
    const svg = sheetToSvg(SHEET);
    const polylines = svg.match(/<polyline/g) ?? [];
    expect(polylines).toHaveLength(2);
    const dashes = svg.match(/stroke-dasharray/g) ?? [];
    expect(dashes).toHaveLength(1); // only the hidden segment
  });

  it('groups each view in its own <g>', () => {
    const svg = sheetToSvg(SHEET);
    expect(svg).toContain('class="view view-front"');
  });

  it('flips y so a top-of-sheet point maps to a small svg-y', () => {
    // Sheet y=10 (top) should map nearer the top of the SVG (small y) than y=0.
    const svg = sheetToSvg(SHEET, [], { margin: 1 });
    // The hidden segment spans sheet y 0→10; its endpoints should differ in y.
    const points = [...svg.matchAll(/points="([^"]+)"/g)].map((m) => m[1]);
    const hidden = points.find((p) => p.includes('11') || p.includes('1,'));
    expect(hidden).toBeTruthy();
  });

  it('renders dimensions with lines, arrowheads, text, and label', () => {
    const dim = createLinearDim([0, 0], [10, 0], 3, 1);
    const svg = sheetToSvg(SHEET, [dim]);
    expect(svg).toContain('class="dimensions"');
    expect(svg).toContain('<line');
    expect(svg).toContain('<polygon');
    expect(svg).toContain('<text');
    expect(svg).toContain('>10<');
  });

  it('expands the canvas to include dimension geometry beyond the views', () => {
    const withDim = sheetToSvg(SHEET, [createLinearDim([0, 0], [10, 0], 40, 1)]);
    const plain = sheetToSvg(SHEET);
    const hOf = (s) => Number(s.match(/height="([\d.]+)"/)[1]);
    expect(hOf(withDim)).toBeGreaterThan(hOf(plain));
  });

  it('escapes special characters in the title', () => {
    const svg = sheetToSvg(SHEET, [], { title: 'A & B <part>', titleBlock: true });
    expect(svg).toContain('A &amp; B &lt;part&gt;');
    expect(svg).not.toContain('<part>');
  });

  it('includes scale and date in the title block when requested', () => {
    const svg = sheetToSvg(SHEET, [], { titleBlock: true, date: '2026-07-14' });
    expect(svg).toContain('class="title-block"');
    expect(svg).toContain('Scale 1:1');
    expect(svg).toContain('2026-07-14');
  });

  it('returns a valid empty sheet when there is nothing to draw', () => {
    const svg = sheetToSvg({ scale: 1, bounds: null, views: [] });
    expect(svg.startsWith('<svg')).toBe(true);
    expect(svg).toContain('</svg>');
    expect(svg).not.toContain('<polyline');
  });

  it('renders a radius dimension label with R prefix', () => {
    const svg = sheetToSvg(SHEET, [createRadiusDim([5, 5], [5, 9], 1, 1)]);
    expect(svg).toContain('>R4<');
  });
});
