import { describe, expect, it } from 'vitest';
import {
  addArc,
  addCircle,
  addLine,
  addPoint,
  addRectangle,
  createSketch,
  entityRadius,
} from './model.js';
import {
  extendEntityAt,
  faceBoundaryLoopsUV,
  offsetEntities,
  trimEntityAt,
} from './edit.js';

/** Add a line between two fresh coordinate points; returns the line id. */
function line(s, x1, y1, x2, y2) {
  return addLine(s, addPoint(s, x1, y1), addPoint(s, x2, y2));
}

const lineEnds = (s, id) => {
  const e = s.entities[id];
  return [s.points[e.p1], s.points[e.p2]];
};

describe('offsetEntities', () => {
  it('offsets a single line to a parallel line', () => {
    const s = createSketch();
    const id = line(s, 0, 0, 2, 0);
    const [created] = offsetEntities(s, [id], 1);
    const [a, b] = lineEnds(s, created);
    // Left normal of +x travel is +y.
    expect(a.x).toBeCloseTo(0, 9);
    expect(a.y).toBeCloseTo(1, 9);
    expect(b.x).toBeCloseTo(2, 9);
    expect(b.y).toBeCloseTo(1, 9);
  });

  it('negative distance offsets the other way', () => {
    const s = createSketch();
    const id = line(s, 0, 0, 2, 0);
    const [created] = offsetEntities(s, [id], -0.5);
    const [a] = lineEnds(s, created);
    expect(a.y).toBeCloseTo(-0.5, 9);
  });

  it('joins connected chain corners at the offset intersection', () => {
    // CCW rectangle 0,0 → 2,1; left normal points inward, so +dist shrinks it.
    const s = createSketch();
    const [bottom, right, top, left] = addRectangle(s, 0, 0, 2, 1);
    const created = offsetEntities(s, [bottom, right, top, left], 0.25);
    expect(created).toHaveLength(4);
    const xs = [];
    const ys = [];
    for (const id of created) {
      for (const p of lineEnds(s, id)) {
        xs.push(p.x);
        ys.push(p.y);
      }
    }
    expect(Math.min(...xs)).toBeCloseTo(0.25, 9);
    expect(Math.max(...xs)).toBeCloseTo(1.75, 9);
    expect(Math.min(...ys)).toBeCloseTo(0.25, 9);
    expect(Math.max(...ys)).toBeCloseTo(0.75, 9);
    // Corners are shared: 4 lines, 4 distinct offset points.
    const pointIds = new Set(
      created.flatMap((id) => [s.entities[id].p1, s.entities[id].p2])
    );
    expect(pointIds.size).toBe(4);
  });

  it('offsets a circle concentrically outward', () => {
    const s = createSketch();
    const id = addCircle(s, addPoint(s, 0, 0), 1);
    const [created] = offsetEntities(s, [id], 0.5);
    expect(entityRadius(s, s.entities[created])).toBeCloseTo(1.5, 9);
  });

  it('skips an entity whose offset collapses through the center', () => {
    const s = createSketch();
    const id = addCircle(s, addPoint(s, 0, 0), 1);
    expect(offsetEntities(s, [id], -2)).toEqual([]);
  });
});

describe('trimEntityAt', () => {
  it('trims the clicked end back to the intersection', () => {
    const s = createSketch();
    const h = line(s, -2, 0, 2, 0);
    line(s, 0, -2, 0, 2); // crosses at origin
    expect(trimEntityAt(s, h, 1, 0)).toBe(true);
    // The horizontal line survives only on the left half.
    const survivor = Object.values(s.entities).find(
      (e) => e.type === 'line' && Math.abs(s.points[e.p1].y) < 1e-9 && Math.abs(s.points[e.p2].y) < 1e-9
    );
    const xs = [s.points[survivor.p1].x, s.points[survivor.p2].x].sort((a, b) => a - b);
    expect(xs[0]).toBeCloseTo(-2, 9);
    expect(xs[1]).toBeCloseTo(0, 9);
  });

  it('splits a line when the clicked stretch is interior', () => {
    const s = createSketch();
    const h = line(s, -2, 0, 2, 0);
    line(s, -1, -2, -1, 2); // cut at x=-1
    line(s, 1, -2, 1, 2); // cut at x=1
    trimEntityAt(s, h, 0, 0); // click between the cuts
    const horiz = Object.values(s.entities).filter(
      (e) => e.type === 'line' && Math.abs(s.points[e.p1].y) < 1e-9 && Math.abs(s.points[e.p2].y) < 1e-9
    );
    expect(horiz).toHaveLength(2);
    const spans = horiz
      .map((e) => [s.points[e.p1].x, s.points[e.p2].x].sort((a, b) => a - b).join(','))
      .sort();
    expect(spans).toEqual(['-2,-1', '1,2']);
  });

  it('deletes an entity with no bounding intersection', () => {
    const s = createSketch();
    const solo = line(s, 0, 0, 1, 0);
    expect(trimEntityAt(s, solo, 0.5, 0)).toBe(true);
    expect(s.entities[solo]).toBeUndefined();
  });

  it('trims a full circle into the complementary arc', () => {
    const s = createSketch();
    const circle = addCircle(s, addPoint(s, 0, 0), 1);
    line(s, -2, 0, 2, 0); // two cuts at angle 0 and π
    trimEntityAt(s, circle, 0, 1); // click the top → remove top, keep bottom
    const arc = Object.values(s.entities).find((e) => e.type === 'arc');
    expect(arc).toBeTruthy();
    // Kept arc spans the lower semicircle (a point at y<0 lies on it).
    const c = s.points[arc.center];
    expect(Math.hypot(c.x, c.y)).toBeCloseTo(0, 9);
    expect(entityRadius(s, arc)).toBeCloseTo(1, 9);
  });
});

describe('extendEntityAt', () => {
  it('extends the clicked end to the nearest intersection', () => {
    const s = createSketch();
    const a = line(s, 0, 0, 1, 0);
    line(s, 2, -1, 2, 1); // vertical wall at x=2
    expect(extendEntityAt(s, a, 0.9, 0)).toBe(true);
    const [p1, p2] = lineEnds(s, a);
    const far = p1.x > p2.x ? p1 : p2;
    expect(far.x).toBeCloseTo(2, 9);
    expect(far.y).toBeCloseTo(0, 9);
  });

  it('extends the start end when clicked near the start', () => {
    const s = createSketch();
    const a = line(s, 0, 0, 1, 0);
    line(s, -2, -1, -2, 1); // wall at x=-2
    extendEntityAt(s, a, 0.1, 0);
    const [p1, p2] = lineEnds(s, a);
    const near = p1.x < p2.x ? p1 : p2;
    expect(near.x).toBeCloseTo(-2, 9);
  });

  it('returns false when there is nothing to extend to', () => {
    const s = createSketch();
    const a = line(s, 0, 0, 1, 0);
    expect(extendEntityAt(s, a, 0.9, 0)).toBe(false);
  });
});

describe('faceBoundaryLoopsUV', () => {
  it('projects a quad face boundary into the XY plane', () => {
    // Unit square as two triangles sharing the 0-2 diagonal.
    const positions = [0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0];
    const indices = [0, 1, 2, 0, 2, 3];
    const loops = faceBoundaryLoopsUV(positions, indices, [0, 1], 'XY');
    expect(loops).toHaveLength(1);
    // Four corners after collinear simplification.
    expect(loops[0]).toHaveLength(4);
    const key = loops[0].map(([u, v]) => `${u},${v}`).sort();
    expect(key).toEqual(['0,0', '0,1', '1,0', '1,1']);
  });

  it('collapses collinear mesh points along a straight edge', () => {
    // A 1×1 square whose bottom edge is split by a midpoint vertex (4 tris).
    const positions = [
      0, 0, 0, // 0
      0.5, 0, 0, // 1 midpoint on the bottom edge
      1, 0, 0, // 2
      1, 1, 0, // 3
      0, 1, 0, // 4
    ];
    const indices = [0, 1, 4, 1, 2, 3, 1, 3, 4];
    const loops = faceBoundaryLoopsUV(positions, indices, [0, 1, 2], 'XY');
    expect(loops).toHaveLength(1);
    // The collinear midpoint (0.5,0) is dropped → 4 real corners.
    expect(loops[0]).toHaveLength(4);
  });
});
