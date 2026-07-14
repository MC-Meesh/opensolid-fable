import { describe, expect, it } from 'vitest';
import { createSheet, DEFAULT_VIEWS, fitView, viewOrigins } from './sheet.js';

// Unit cube [0,1]^3 (see project.test.js) — a body that draws in every view.
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

describe('viewOrigins', () => {
  const spans = {
    front: { w: 2, h: 2, cx: 0, cy: 0 },
    top: { w: 2, h: 2, cx: 0, cy: 0 },
    right: { w: 2, h: 2, cx: 0, cy: 0 },
    iso: { w: 3, h: 3, cx: 0, cy: 0 },
  };

  it('anchors front at the origin', () => {
    expect(viewOrigins(spans, 1).front).toEqual([0, 0]);
  });

  it('third-angle: top above front, right to the right', () => {
    const o = viewOrigins(spans, 1, 'third', 1);
    expect(o.top[0]).toBe(0); // shared X
    expect(o.top[1]).toBeGreaterThan(0); // above
    expect(o.right[1]).toBe(0); // shared Y
    expect(o.right[0]).toBeGreaterThan(0); // to the right
  });

  it('first-angle flips top below and right to the left', () => {
    const o = viewOrigins(spans, 1, 'first', 1);
    expect(o.top[1]).toBeLessThan(0);
    expect(o.right[0]).toBeLessThan(0);
  });

  it('places iso diagonally clear of the ortho set', () => {
    const o = viewOrigins(spans, 1, 'third', 1);
    expect(o.iso[0]).toBeGreaterThan(0);
    expect(o.iso[1]).toBeGreaterThan(0);
  });

  it('scale widens the gaps between views', () => {
    const near = viewOrigins(spans, 1, 'third');
    const far = viewOrigins(spans, 10, 'third');
    expect(Math.abs(far.right[0])).toBeGreaterThan(Math.abs(near.right[0]));
  });
});

describe('createSheet', () => {
  it('places the default four views from a cube', () => {
    const sheet = createSheet(CUBE);
    expect(sheet.views.map((v) => v.view)).toEqual(DEFAULT_VIEWS);
    expect(sheet.scale).toBe(1);
    expect(sheet.angle).toBe('third');
    expect(sheet.bounds).not.toBeNull();
  });

  it('applies scale to segment coordinates', () => {
    const one = createSheet(CUBE, { views: ['front'], scale: 1 });
    const ten = createSheet(CUBE, { views: ['front'], scale: 10 });
    const spanOf = (s) => {
      const b = s.bounds;
      return Math.hypot(b.maxX - b.minX, b.maxY - b.minY);
    };
    expect(spanOf(ten)).toBeCloseTo(spanOf(one) * 10, 6);
  });

  it('front view is centered on the origin', () => {
    const sheet = createSheet(CUBE, { views: ['front'], scale: 2 });
    const { minX, maxX, minY, maxY } = sheet.views[0].bounds;
    expect((minX + maxX) / 2).toBeCloseTo(0, 6);
    expect((minY + maxY) / 2).toBeCloseTo(0, 6);
  });

  it('omits views that project to nothing (empty mesh)', () => {
    const empty = { positions: new Float32Array(0), indices: new Uint32Array(0) };
    const sheet = createSheet(empty);
    expect(sheet.views).toEqual([]);
    expect(sheet.bounds).toBeNull();
  });

  it('every placed view carries sheet-coordinate segments', () => {
    const sheet = createSheet(CUBE);
    for (const v of sheet.views) {
      expect(v.segments.length).toBeGreaterThan(0);
      for (const seg of v.segments) {
        expect(seg.pts.length).toBeGreaterThanOrEqual(2);
        expect(seg.style).toBe('visible');
      }
      expect(v.bounds).not.toBeNull();
    }
  });

  it('respects a custom view subset', () => {
    const sheet = createSheet(CUBE, { views: ['front', 'right'] });
    expect(sheet.views.map((v) => v.view)).toEqual(['front', 'right']);
  });
});

describe('fitView', () => {
  it('centers on the bounds and scales to fill with margin', () => {
    const view = fitView({ minX: 0, minY: 0, maxX: 100, maxY: 50 }, { w: 400, h: 400 });
    expect(view.cx).toBe(50);
    expect(view.cy).toBe(25);
    // Limiting dimension is width (100 units into 400px), minus 12% margin.
    expect(view.scale).toBeCloseTo((400 / 100) * 0.88, 6);
  });

  it('falls back to a centered default for missing bounds or size', () => {
    expect(fitView(null, { w: 400, h: 400 })).toEqual({ cx: 0, cy: 0, scale: 60 });
    expect(fitView({ minX: 0, minY: 0, maxX: 1, maxY: 1 }, { w: 0, h: 0 })).toEqual({
      cx: 0,
      cy: 0,
      scale: 60,
    });
  });
});
