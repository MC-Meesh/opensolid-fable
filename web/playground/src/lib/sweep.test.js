import { describe, expect, it } from 'vitest';
import {
  buildSweepShape,
  opsBounds,
  profileToOps,
  sweepPostOps,
  sweepTreeNode,
} from './sweep.js';
import { serializeTree } from './sceneTree.js';

// Minimal stand-ins recording construction, like sceneTree.test.js.
class FakeShape {
  constructor(desc) {
    this.desc = desc;
    this.freed = false;
  }
  free() {
    this.freed = true;
  }
  static extrude(profile, height) {
    return new FakeShape(['extrude', profile.trace.slice(), height]);
  }
  static revolve(profile, angle) {
    return new FakeShape(['revolve', profile.trace.slice(), angle]);
  }
  translate(x, y, z) {
    return new FakeShape(['translate', this.desc, x, y, z]);
  }
  rotate(ax, ay, az, angle) {
    return new FakeShape(['rotate', this.desc, ax, ay, az, angle]);
  }
}

class FakeProfile {
  constructor(x, y) {
    this.trace = [['new', x, y]];
    this.freed = false;
  }
  arcTo(x, y, bulge) {
    this.trace.push(['arcTo', x, y, bulge]);
  }
  close() {
    this.trace.push(['close']);
  }
  free() {
    this.freed = true;
  }
}

const SQUARE = {
  closed: true,
  plane: 'XY',
  segments: [
    { kind: 'line', start: [0, 0], end: [2, 0] },
    { kind: 'line', start: [2, 0], end: [2, 1] },
    { kind: 'line', start: [2, 1], end: [0, 1] },
    { kind: 'line', start: [0, 1], end: [0, 0] },
  ],
};

describe('profileToOps', () => {
  it('converts lines to bulge-0 segments ending back at the start', () => {
    const ops = profileToOps(SQUARE);
    expect(ops).toEqual({
      start: [0, 0],
      segs: [
        { x: 2, y: 0, bulge: 0 },
        { x: 2, y: 1, bulge: 0 },
        { x: 0, y: 1, bulge: 0 },
        { x: 0, y: 0, bulge: 0 },
      ],
    });
  });

  it('converts a CCW semicircular arc to bulge +1', () => {
    const profile = {
      closed: true,
      plane: 'XY',
      segments: [
        {
          kind: 'arc',
          center: [0, 0],
          radius: 1,
          startAngle: 0,
          endAngle: Math.PI,
          ccw: true,
        },
        { kind: 'line', start: [-1, 0], end: [1, 0] },
      ],
    };
    const ops = profileToOps(profile);
    expect(ops.start[0]).toBeCloseTo(1);
    expect(ops.start[1]).toBeCloseTo(0);
    expect(ops.segs[0].x).toBeCloseTo(-1);
    expect(ops.segs[0].bulge).toBeCloseTo(1);
    // Last segment snaps exactly onto the start point.
    expect(ops.segs[1]).toEqual({ x: ops.start[0], y: ops.start[1], bulge: 0 });
  });

  it('flips the bulge sign for clockwise arcs', () => {
    const profile = {
      closed: true,
      plane: 'XY',
      segments: [
        {
          kind: 'arc',
          center: [0, 0],
          radius: 1,
          startAngle: 0,
          endAngle: Math.PI,
          ccw: false,
        },
        { kind: 'line', start: [-1, 0], end: [1, 0] },
      ],
    };
    expect(profileToOps(profile).segs[0].bulge).toBeCloseTo(-1);
  });

  it('converts a full circle (two semicircles) to two bulge-1 arcs', () => {
    const profile = {
      closed: true,
      plane: 'XY',
      segments: [
        {
          kind: 'arc',
          center: [3, 2],
          radius: 1,
          startAngle: 0,
          endAngle: Math.PI,
          ccw: true,
        },
        {
          kind: 'arc',
          center: [3, 2],
          radius: 1,
          startAngle: Math.PI,
          endAngle: 0,
          ccw: true,
        },
      ],
    };
    const ops = profileToOps(profile);
    expect(ops.start[0]).toBeCloseTo(4);
    expect(ops.start[1]).toBeCloseTo(2);
    expect(ops.segs.map((s) => s.bulge)).toEqual([
      expect.closeTo(1, 5),
      expect.closeTo(1, 5),
    ]);
    expect(ops.segs[1].x).toBe(ops.start[0]);
    expect(ops.segs[1].y).toBe(ops.start[1]);
  });

  it('rejects open profiles', () => {
    expect(() => profileToOps({ closed: false, reason: 'open endpoint' })).toThrow(
      /not closed/
    );
  });
});

describe('sweepPostOps', () => {
  it('leaves the native frame alone when it already matches the plane', () => {
    expect(sweepPostOps('XZ', 'extrude', 5)).toEqual([]);
    expect(sweepPostOps('XY', 'revolve', 360)).toEqual([]);
  });

  it('orients extrusions along the plane normal, starting at the plane', () => {
    expect(sweepPostOps('XY', 'extrude', 5)).toEqual([
      { op: 'rotate', args: [1, 0, 0, -Math.PI / 2] },
      { op: 'translate', args: [0, 0, 5] },
    ]);
    expect(sweepPostOps('YZ', 'extrude', 5)).toEqual([
      { op: 'rotate', args: [0, 0, 1, Math.PI / 2] },
      { op: 'translate', args: [5, 0, 0] },
    ]);
  });

  it('orients revolutions around the sketch v axis', () => {
    expect(sweepPostOps('XZ', 'revolve', 360)).toEqual([
      { op: 'rotate', args: [1, 0, 0, Math.PI / 2] },
    ]);
    expect(sweepPostOps('YZ', 'revolve', 360)).toEqual([
      { op: 'rotate', args: [1, 1, 1, (2 * Math.PI) / 3] },
    ]);
  });

  it('rejects unknown planes and kinds', () => {
    expect(() => sweepPostOps('UV', 'extrude', 1)).toThrow(/unknown sketch plane/);
    expect(() => sweepPostOps('XY', 'loft', 1)).toThrow(/unknown sweep kind/);
  });
});

describe('buildSweepShape', () => {
  const ops = profileToOps(SQUARE);

  it('builds, orients, and frees intermediates for an XY extrude', () => {
    const shape = buildSweepShape(FakeShape, FakeProfile, {
      kind: 'extrude',
      plane: 'XY',
      ops,
      value: 2,
    });
    expect(shape.desc[0]).toBe('translate');
    expect(shape.desc.slice(2)).toEqual([0, 0, 2]);
    const rotated = shape.desc[1];
    expect(rotated[0]).toBe('rotate');
    expect(rotated.slice(2)).toEqual([1, 0, 0, -Math.PI / 2]);
    const swept = rotated[1];
    expect(swept[0]).toBe('extrude');
    expect(swept[2]).toBe(2);
    // Profile fed through the builder API and closed.
    expect(swept[1][0]).toEqual(['new', 0, 0]);
    expect(swept[1].at(-1)).toEqual(['close']);
    expect(shape.freed).toBe(false);
  });

  it('passes the revolve angle through unchanged on XY', () => {
    const shape = buildSweepShape(FakeShape, FakeProfile, {
      kind: 'revolve',
      plane: 'XY',
      ops,
      value: 270,
    });
    expect(shape.desc[0]).toBe('revolve');
    expect(shape.desc[2]).toBe(270);
  });
});

describe('sweepTreeNode', () => {
  const ops = profileToOps(SQUARE);

  it('serializes a standalone sweep with its plane orientation', () => {
    const node = sweepTreeNode(null, {
      kind: 'extrude',
      plane: 'XY',
      ops,
      value: 2,
    });
    expect(serializeTree(node)).toBe(
      'const p1 = new Profile(0, 0);\n' +
        'p1.lineTo(2, 0);\n' +
        'p1.lineTo(2, 1);\n' +
        'p1.lineTo(0, 1);\n' +
        'p1.lineTo(0, 0);\n' +
        'p1.close();\n' +
        `return Shape.extrude(p1, 2).rotate(1, 0, 0, ${-Math.PI / 2}).translate(0, 0, 2);\n`
    );
  });

  it('unions the sweep with an existing root', () => {
    const root = { id: 7, op: 'sphere', args: [1], children: [] };
    const node = sweepTreeNode(root, {
      kind: 'revolve',
      plane: 'XY',
      ops,
      value: 360,
    });
    expect(node.op).toBe('union');
    expect(node.children[0]).toBe(root);
    expect(serializeTree(node)).toContain(
      'return Shape.sphere(1).union(Shape.revolve(p1, 360));'
    );
  });

  it('uses negative ids that cannot collide with traced nodes', () => {
    const node = sweepTreeNode({ id: 3, op: 'sphere', args: [1], children: [] }, {
      kind: 'extrude',
      plane: 'XZ',
      ops,
      value: 1,
    });
    const ids = [];
    const walk = (n) => {
      ids.push(n.id);
      n.children.forEach(walk);
    };
    walk(node);
    expect(ids.filter((id) => id < 0)).toHaveLength(ids.length - 1);
    expect(new Set(ids).size).toBe(ids.length);
  });
});

describe('opsBounds', () => {
  it('bounds the profile vertices', () => {
    expect(opsBounds(profileToOps(SQUARE))).toEqual({
      min: [0, 0],
      max: [2, 1],
    });
  });
});
