import { describe, expect, it } from 'vitest';
import {
  buildSweepShape,
  mirrorOpsV,
  nativeSweepOps,
  opsBounds,
  profileToOps,
  sweepPostOps,
  sweepTreeNode,
} from './sweep.js';
import { serializeTree } from './sceneTree.js';
import { planeNormal, planeToWorld } from './sketch/profile.js';

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
      { op: 'rotate', args: [1, 1, -1, (2 * Math.PI) / 3] },
    ]);
  });

  it('orients revolutions around the sketch v axis', () => {
    expect(sweepPostOps('XZ', 'revolve', 360)).toEqual([
      { op: 'rotate', args: [1, 0, 0, -Math.PI / 2] },
    ]);
    expect(sweepPostOps('YZ', 'revolve', 360)).toEqual([
      { op: 'rotate', args: [0, 1, 0, Math.PI / 2] },
    ]);
  });

  it('rejects unknown planes and kinds', () => {
    expect(() => sweepPostOps('UV', 'extrude', 1)).toThrow(/unknown sketch plane/);
    expect(() => sweepPostOps('XY', 'loft', 1)).toThrow(/unknown sweep kind/);
  });
});

// ---- geometric verification: native ops + post-ops == planeToWorld --------

/** Rodrigues rotation of `p` around unit-normalized `[ax, ay, az]`. */
function rotatePoint([x, y, z], [ax, ay, az, angle]) {
  const len = Math.hypot(ax, ay, az);
  const [ux, uy, uz] = [ax / len, ay / len, az / len];
  const cos = Math.cos(angle);
  const sin = Math.sin(angle);
  const dot = ux * x + uy * y + uz * z;
  return [
    x * cos + (uy * z - uz * y) * sin + ux * dot * (1 - cos),
    y * cos + (uz * x - ux * z) * sin + uy * dot * (1 - cos),
    z * cos + (ux * y - uy * x) * sin + uz * dot * (1 - cos),
  ];
}

function applyPostOps(point, postOps) {
  let p = point;
  for (const { op, args } of postOps) {
    if (op === 'rotate') p = rotatePoint(p, args);
    else p = [p[0] + args[0], p[1] + args[1], p[2] + args[2]];
  }
  return p;
}

const closeTo3 = (a, b) =>
  Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]) < 1e-12;

function opsVerts(ops) {
  return [ops.start, ...ops.segs.map((s) => [s.x, s.y])];
}

function signedArea(verts) {
  let area = 0;
  for (let i = 0; i < verts.length; i += 1) {
    const [x1, y1] = verts[i];
    const [x2, y2] = verts[(i + 1) % verts.length];
    area += x1 * y2 - x2 * y1;
  }
  return area / 2;
}

describe('sweep plane mapping (WYSIWYG)', () => {
  const SKETCH_VERTS = [
    [0, 0],
    [2, 0],
    [2, 1],
    [0, 1],
  ];

  it('extrude spans exactly planeToWorld(u, v) .. + height * normal', () => {
    const h = 2;
    for (const plane of ['XY', 'XZ', 'YZ']) {
      const ops = nativeSweepOps(profileToOps({ ...SQUARE, plane }), plane, 'extrude');
      const post = sweepPostOps(plane, 'extrude', h);
      // Every native prism edge, carried through the post-ops.
      const edges = opsVerts(ops).map(([p, q]) => [
        applyPostOps([p, 0, q], post),
        applyPostOps([p, h, q], post),
      ]);
      const n = planeNormal(plane);
      for (const [u, v] of SKETCH_VERTS) {
        const base = planeToWorld(plane, u, v);
        const top = base.map((c, i) => c + h * n[i]);
        // Some edge runs from the drawn point on the plane to +normal * h
        // (in either direction — the cross-section is constant).
        const hit = edges.some(
          ([a, b]) =>
            (closeTo3(a, base) && closeTo3(b, top)) ||
            (closeTo3(a, top) && closeTo3(b, base))
        );
        expect(hit, `${plane} vertex (${u}, ${v})`).toBe(true);
      }
    }
  });

  it('revolve places the profile at planeToWorld and spins around the v axis', () => {
    for (const plane of ['XY', 'XZ', 'YZ']) {
      const ops = nativeSweepOps(profileToOps({ ...SQUARE, plane }), plane, 'revolve');
      const post = sweepPostOps(plane, 'revolve', 360);
      // The native start half-plane (theta = 0) lands on the sketch plane.
      for (const [p, q] of opsVerts(ops)) {
        expect(applyPostOps([p, q, 0], post)).toSatisfy((pt) =>
          closeTo3(pt, planeToWorld(plane, p, q))
        );
      }
      // The native revolve axis (world Y) maps onto the sketch v axis line.
      const axis = applyPostOps([0, 1, 0], post);
      const ev = planeToWorld(plane, 0, 1);
      const cross = Math.hypot(
        axis[1] * ev[2] - axis[2] * ev[1],
        axis[2] * ev[0] - axis[0] * ev[2],
        axis[0] * ev[1] - axis[1] * ev[0]
      );
      expect(cross).toBeCloseTo(0, 12);
    }
  });

  it('mirrorOpsV mirrors v, keeps the anchor, winding, and bulges', () => {
    const circleish = {
      closed: true,
      plane: 'XZ',
      segments: [
        {
          kind: 'arc',
          center: [1, 1],
          radius: 1,
          startAngle: 0,
          endAngle: Math.PI,
          ccw: true,
        },
        { kind: 'line', start: [0, 1], end: [2, 1] },
      ],
    };
    const ops = profileToOps(circleish);
    const mirrored = mirrorOpsV(ops);
    expect(mirrored.start).toEqual([ops.start[0], -ops.start[1]]);
    // Same vertex set, v negated.
    const expectVerts = opsVerts(ops)
      .map(([x, y]) => `${x},${-y}`)
      .sort();
    expect(
      opsVerts(mirrored)
        .map(([x, y]) => `${x},${y}`)
        .sort()
    ).toEqual(expectVerts);
    // The CCW semicircle keeps its +1 bulge.
    expect(Math.max(...mirrored.segs.map((s) => s.bulge))).toBeCloseTo(1);
    // The loop still closes exactly on the start vertex.
    const last = mirrored.segs.at(-1);
    expect([last.x, last.y]).toEqual(mirrored.start);
    // Vertex winding stays counterclockwise (checked on an area-carrying
    // polygon; the arc fixture's vertex polyline is degenerate).
    expect(signedArea(opsVerts(mirrorOpsV(profileToOps(SQUARE))))).toBeGreaterThan(0);
  });

  it('nativeSweepOps mirrors only extrusions on XZ/YZ', () => {
    const ops = profileToOps(SQUARE);
    expect(nativeSweepOps(ops, 'XY', 'extrude')).toBe(ops);
    expect(nativeSweepOps(ops, 'XZ', 'revolve')).toBe(ops);
    expect(nativeSweepOps(ops, 'YZ', 'revolve')).toBe(ops);
    const mirrored = nativeSweepOps(ops, 'XZ', 'extrude');
    expect(mirrored.start[0]).toBeCloseTo(0);
    expect(mirrored.start[1]).toBeCloseTo(0);
    expect(signedArea(opsVerts(mirrored))).toBeGreaterThan(0);
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
