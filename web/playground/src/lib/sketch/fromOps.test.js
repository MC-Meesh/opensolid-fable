import { describe, expect, it } from 'vitest';
import { bulgeArcCenter, sketchFromOps } from './fromOps.js';
import { extractProfile } from './profile.js';
import { profileToOps } from '../sweep.js';

describe('bulgeArcCenter', () => {
  it('finds the center of a ccw quarter arc', () => {
    // (1,0) -> (0,1) counter-clockwise around the origin: sweep π/2.
    const b = Math.tan(Math.PI / 8);
    const [cx, cy] = bulgeArcCenter(1, 0, 0, 1, b);
    expect(cx).toBeCloseTo(0, 10);
    expect(cy).toBeCloseTo(0, 10);
  });

  it('finds the center of a cw major arc', () => {
    // (1,0) -> (0,1) clockwise the long way around the origin: sweep 3π/2.
    const b = -Math.tan((3 * Math.PI) / 8);
    const [cx, cy] = bulgeArcCenter(1, 0, 0, 1, b);
    expect(cx).toBeCloseTo(0, 10);
    expect(cy).toBeCloseTo(0, 10);
  });
});

describe('sketchFromOps', () => {
  it('rebuilds a script-style square (implicit close) as a closed loop', () => {
    // Scripts close through Profile.close(): the snapshot has no final seg
    // landing back on the start.
    const ops = {
      start: [0, 0],
      segs: [
        { x: 1, y: 0, bulge: 0 },
        { x: 1, y: 1, bulge: 0 },
        { x: 0, y: 1, bulge: 0 },
      ],
    };
    const sketch = sketchFromOps(ops);
    expect(Object.keys(sketch.entities)).toHaveLength(4); // closing line added
    expect(Object.keys(sketch.points)).toHaveLength(4); // corners shared

    const profile = extractProfile(sketch, 'XY');
    expect(profile.closed).toBe(true);
    expect(profile.segments).toHaveLength(4);

    // Round-trip back to ops: same loop, explicit closing segment.
    const ops2 = profileToOps(profile);
    expect(ops2.start).toEqual([0, 0]);
    expect(ops2.segs.map((s) => [s.x, s.y])).toEqual([
      [1, 0],
      [1, 1],
      [0, 1],
      [0, 0],
    ]);
    expect(ops2.segs.every((s) => s.bulge === 0)).toBe(true);
  });

  it('does not add a closing line when the last seg lands on the start', () => {
    const ops = {
      start: [0, 0],
      segs: [
        { x: 1, y: 0, bulge: 0 },
        { x: 1, y: 1, bulge: 0 },
        { x: 0, y: 1, bulge: 0 },
        { x: 0, y: 0, bulge: 0 },
      ],
    };
    const sketch = sketchFromOps(ops);
    expect(Object.keys(sketch.entities)).toHaveLength(4);
    expect(Object.keys(sketch.points)).toHaveLength(4);
    expect(extractProfile(sketch, 'XY').closed).toBe(true);
  });

  it('rebuilds arcs from bulges and round-trips them', () => {
    // Quarter disc: arc (1,0) -> (0,1) around the origin, closed via origin.
    const bulge = Math.tan(Math.PI / 8);
    const ops = {
      start: [1, 0],
      segs: [
        { x: 0, y: 1, bulge },
        { x: 0, y: 0, bulge: 0 },
        { x: 1, y: 0, bulge: 0 },
      ],
    };
    const sketch = sketchFromOps(ops);
    const arcs = Object.values(sketch.entities).filter((e) => e.type === 'arc');
    expect(arcs).toHaveLength(1);
    expect(arcs[0].ccw).toBe(true);
    const center = sketch.points[arcs[0].center];
    expect(center.x).toBeCloseTo(0, 10);
    expect(center.y).toBeCloseTo(0, 10);

    const profile = extractProfile(sketch, 'XY');
    expect(profile.closed).toBe(true);
    const ops2 = profileToOps(profile);
    expect(ops2.start[0]).toBeCloseTo(1, 10);
    expect(ops2.start[1]).toBeCloseTo(0, 10);
    const arcSeg = ops2.segs.find((s) => s.bulge !== 0);
    expect(arcSeg.bulge).toBeCloseTo(bulge, 10);
  });

  it('rebuilds a full circle snapshot (two semicircular arcs)', () => {
    // circleProfile emits two half arcs; via profileToOps both have bulge 1.
    const ops = {
      start: [3, 0],
      segs: [
        { x: 1, y: 0, bulge: 1 },
        { x: 3, y: 0, bulge: 1 },
      ],
    };
    const sketch = sketchFromOps(ops);
    const arcs = Object.values(sketch.entities).filter((e) => e.type === 'arc');
    expect(arcs).toHaveLength(2);
    for (const arc of arcs) {
      const c = sketch.points[arc.center];
      expect(c.x).toBeCloseTo(2, 9);
      expect(c.y).toBeCloseTo(0, 9);
    }
    expect(extractProfile(sketch, 'XY').closed).toBe(true);
  });
});
