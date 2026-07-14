import { describe, expect, it } from 'vitest';
import {
  createSketch,
  addPoint,
  addLine,
  addCircle,
  addArc,
  addConstraint,
  entityRadius,
} from './model.js';
import {
  offsetEntity,
  entityIntersections,
  trimEntityAt,
  extendEntityAt,
  convertEntities,
} from './edit.js';

/** Build a line from raw coordinates; returns the entity id. */
function line(s, x1, y1, x2, y2) {
  return addLine(s, addPoint(s, x1, y1), addPoint(s, x2, y2));
}

const near = (a, b, eps = 1e-6) => Math.abs(a - b) < eps;

describe('offsetEntity', () => {
  it('offsets a line along its left normal', () => {
    const s = createSketch();
    const id = line(s, 0, 0, 10, 0); // direction +x, left normal +y
    const off = offsetEntity(s, id, 2);
    const e = s.entities[off];
    expect(e.type).toBe('line');
    const p1 = s.points[e.p1];
    const p2 = s.points[e.p2];
    expect([p1.x, p1.y]).toEqual([0, 2]);
    expect([p2.x, p2.y]).toEqual([10, 2]);
  });

  it('offsets a line the other way for a negative distance', () => {
    const s = createSketch();
    const id = line(s, 0, 0, 10, 0);
    const off = offsetEntity(s, id, -3);
    const e = s.entities[off];
    expect(s.points[e.p1].y).toBe(-3);
  });

  it('does not touch the original entity', () => {
    const s = createSketch();
    const id = line(s, 0, 0, 10, 0);
    offsetEntity(s, id, 2);
    const orig = s.entities[id];
    expect(s.points[orig.p1].y).toBe(0);
    expect(Object.keys(s.entities)).toHaveLength(2);
  });

  it('grows and shrinks a circle radius', () => {
    const s = createSketch();
    const id = addCircle(s, addPoint(s, 0, 0), 5);
    expect(s.entities[offsetEntity(s, id, 2)].radius).toBe(7);
    expect(s.entities[offsetEntity(s, id, -3)].radius).toBe(2);
  });

  it('rejects a circle offset driven to a non-positive radius', () => {
    const s = createSketch();
    const id = addCircle(s, addPoint(s, 0, 0), 5);
    expect(offsetEntity(s, id, -5)).toBeNull();
    expect(offsetEntity(s, id, -8)).toBeNull();
  });

  it('offsets an arc concentrically', () => {
    const s = createSketch();
    const c = addPoint(s, 0, 0);
    const id = addArc(s, c, addPoint(s, 5, 0), addPoint(s, 0, 5), true);
    const off = offsetEntity(s, id, 1);
    const e = s.entities[off];
    expect(e.type).toBe('arc');
    expect(near(entityRadius(s, e), 6)).toBe(true);
    expect(near(s.points[e.p1].x, 6)).toBe(true);
    expect(near(s.points[e.p2].y, 6)).toBe(true);
  });

  it('returns null for a zero distance or unknown id', () => {
    const s = createSketch();
    const id = line(s, 0, 0, 10, 0);
    expect(offsetEntity(s, id, 0)).toBeNull();
    expect(offsetEntity(s, 'nope', 2)).toBeNull();
  });
});

describe('entityIntersections', () => {
  it('finds where two segments cross', () => {
    const s = createSketch();
    const h = line(s, -5, 0, 5, 0);
    line(s, 0, -5, 0, 5);
    const pts = entityIntersections(s, h);
    expect(pts).toHaveLength(1);
    expect(near(pts[0][0], 0)).toBe(true);
    expect(near(pts[0][1], 0)).toBe(true);
  });

  it('ignores crossings that fall outside a segment span', () => {
    const s = createSketch();
    const h = line(s, -5, 0, 5, 0);
    line(s, 8, -5, 8, 5); // crosses the infinite line at x=8, off the segment
    expect(entityIntersections(s, h)).toHaveLength(0);
  });

  it('finds both crossings of a line through a circle', () => {
    const s = createSketch();
    const h = line(s, -10, 0, 10, 0);
    addCircle(s, addPoint(s, 0, 0), 5);
    const pts = entityIntersections(s, h).map((p) => p[0]).sort((a, b) => a - b);
    expect(pts).toHaveLength(2);
    expect(near(pts[0], -5)).toBe(true);
    expect(near(pts[1], 5)).toBe(true);
  });
});

describe('trimEntityAt (line)', () => {
  it('removes the middle piece between two crossings', () => {
    const s = createSketch();
    const h = line(s, -10, 0, 10, 0);
    line(s, -3, -5, -3, 5);
    line(s, 3, -5, 3, 5);
    const survivors = trimEntityAt(s, h, 0, 0);
    expect(survivors).toHaveLength(2);
    const spans = survivors
      .map((id) => {
        const e = s.entities[id];
        return [s.points[e.p1].x, s.points[e.p2].x].sort((a, b) => a - b);
      })
      .sort((a, b) => a[0] - b[0]);
    expect(spans[0][0]).toBe(-10);
    expect(near(spans[0][1], -3)).toBe(true);
    expect(near(spans[1][0], 3)).toBe(true);
    expect(spans[1][1]).toBe(10);
  });

  it('trims back to one end when the pick is beyond the last crossing', () => {
    const s = createSketch();
    const h = line(s, -10, 0, 10, 0);
    line(s, -3, -5, -3, 5);
    line(s, 3, -5, 3, 5);
    const survivors = trimEntityAt(s, h, -8, 0); // pick left of x=-3
    expect(survivors).toHaveLength(1);
    const e = s.entities[survivors[0]];
    const xs = [s.points[e.p1].x, s.points[e.p2].x].sort((a, b) => a - b);
    expect(near(xs[0], -3)).toBe(true);
    expect(xs[1]).toBe(10);
  });

  it('deletes the whole entity when nothing brackets the pick', () => {
    const s = createSketch();
    const h = line(s, -10, 0, 10, 0);
    const survivors = trimEntityAt(s, h, 0, 0);
    expect(survivors).toHaveLength(0);
    expect(s.entities[h]).toBeUndefined();
  });

  it('drops a length constraint on the trimmed line', () => {
    const s = createSketch();
    const h = line(s, -10, 0, 10, 0);
    line(s, 3, -5, 3, 5);
    addConstraint(s, { type: 'length', line: h, value: 20 });
    trimEntityAt(s, h, -8, 0);
    expect(Object.values(s.constraints).some((c) => c.type === 'length')).toBe(
      false
    );
  });
});

describe('trimEntityAt (circle / arc)', () => {
  it('turns a circle into the complementary arc', () => {
    const s = createSketch();
    const circle = addCircle(s, addPoint(s, 0, 0), 5);
    line(s, 0, -10, 0, 10); // crosses at (0, 5) and (0, -5)
    const survivors = trimEntityAt(s, circle, 5, 0); // pick the right side
    expect(survivors).toHaveLength(1);
    const arc = s.entities[survivors[0]];
    expect(arc.type).toBe('arc');
    expect(s.entities[circle]).toBeUndefined();
    // The kept arc bulges left (negative x), opposite the removed pick.
    const c = s.points[arc.center];
    const mid = midArcPoint(s, arc);
    expect(mid[0]).toBeLessThan(c.x);
  });

  it('needs at least two crossings to trim a circle', () => {
    const s = createSketch();
    const circle = addCircle(s, addPoint(s, 0, 0), 5);
    line(s, 5, -10, 5, 10); // tangent-ish, single crossing region
    const survivors = trimEntityAt(s, circle, -5, 0);
    expect(survivors).toEqual([circle]);
  });

  it('trims the near sub-arc of an arc', () => {
    const s = createSketch();
    const c = addPoint(s, 0, 0);
    // Semicircle from (5,0) ccw to (-5,0) through the top.
    const arc = addArc(s, c, addPoint(s, 5, 0), addPoint(s, -5, 0), true);
    line(s, 0, -10, 0, 10); // crosses the arc at the top (0, 5)
    const survivors = trimEntityAt(s, arc, 4, 3); // pick the right quarter
    expect(survivors).toHaveLength(1);
    const kept = s.entities[survivors[0]];
    // Right quarter removed → kept quarter is on the left.
    const mid = midArcPoint(s, kept);
    expect(mid[0]).toBeLessThan(0);
  });
});

describe('extendEntityAt', () => {
  it('extends a line end to the next crossing', () => {
    const s = createSketch();
    const a = line(s, 0, 0, 5, 0);
    line(s, 10, -5, 10, 5);
    expect(extendEntityAt(s, a, 4, 0)).toBe(true); // pick near the right end
    const e = s.entities[a];
    const far = s.points[e.p2].x === 10 ? e.p2 : e.p1;
    expect(near(s.points[far].x, 10)).toBe(true);
  });

  it('extends the near end backwards', () => {
    const s = createSketch();
    const a = line(s, 0, 0, 5, 0);
    line(s, -3, -5, -3, 5);
    expect(extendEntityAt(s, a, 1, 0)).toBe(true); // pick near the left end
    const e = s.entities[a];
    expect(near(s.points[e.p1].x, -3)).toBe(true);
  });

  it('returns false when there is nothing to meet', () => {
    const s = createSketch();
    const a = line(s, 0, 0, 5, 0);
    expect(extendEntityAt(s, a, 4, 0)).toBe(false);
  });

  it('cannot extend a circle', () => {
    const s = createSketch();
    const c = addCircle(s, addPoint(s, 0, 0), 5);
    expect(extendEntityAt(s, c, 5, 0)).toBe(false);
  });
});

describe('convertEntities', () => {
  it('projects a closed loop into shared-endpoint lines', () => {
    const s = createSketch();
    const ids = convertEntities(s, [
      [
        [0, 0],
        [10, 0],
        [10, 10],
        [0, 10],
        [0, 0],
      ],
    ]);
    expect(ids).toHaveLength(4);
    // Four distinct corner points, each shared by two lines.
    expect(Object.keys(s.points)).toHaveLength(4);
  });

  it('projects an open polyline without closing it', () => {
    const s = createSketch();
    const ids = convertEntities(s, [
      [
        [0, 0],
        [5, 0],
        [5, 5],
      ],
    ]);
    expect(ids).toHaveLength(2);
  });

  it('collapses duplicate vertices and skips degenerate loops', () => {
    const s = createSketch();
    const ids = convertEntities(s, [
      [
        [0, 0],
        [0, 0],
      ],
      [[1, 1]],
    ]);
    expect(ids).toHaveLength(0);
  });
});

/** Midpoint of an arc's swept span, in world coordinates. */
function midArcPoint(s, arc) {
  const c = s.points[arc.center];
  const p1 = s.points[arc.p1];
  const p2 = s.points[arc.p2];
  const r = entityRadius(s, arc);
  const start = Math.atan2(p1.y - c.y, p1.x - c.x);
  const end = Math.atan2(p2.y - c.y, p2.x - c.x);
  let sweep = arc.ccw ? end - start : start - end;
  while (sweep <= 0) sweep += 2 * Math.PI;
  const mid = start + (arc.ccw ? 1 : -1) * (sweep / 2);
  return [c.x + r * Math.cos(mid), c.y + r * Math.sin(mid)];
}
