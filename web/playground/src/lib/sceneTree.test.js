import { describe, expect, it } from 'vitest';
import {
  createTracer,
  freeNodes,
  nodeLabel,
  profileSegStatement,
  runTracedScript,
  scriptHeader,
  serializeTree,
} from './sceneTree.js';

// Stand-in for WasmShape covering the full scripting API.
class FakeShape {
  constructor(desc) {
    this.desc = desc;
    this.freed = false;
  }
  free() {
    this.freed = true;
  }
  static sphere(r) {
    return new FakeShape(['sphere', r]);
  }
  static box3(hx, hy, hz) {
    return new FakeShape(['box3', hx, hy, hz]);
  }
  static roundedBox(hx, hy, hz, r) {
    return new FakeShape(['roundedBox', hx, hy, hz, r]);
  }
  static cylinder(r, hh) {
    return new FakeShape(['cylinder', r, hh]);
  }
  static torus(major, minor) {
    return new FakeShape(['torus', major, minor]);
  }
  static capsule(x1, y1, z1, x2, y2, z2, r) {
    return new FakeShape(['capsule', x1, y1, z1, x2, y2, z2, r]);
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
  scale(sx, sy, sz) {
    return new FakeShape(['scale', this.desc, sx, sy, sz]);
  }
  uniformScale(factor) {
    return new FakeShape(['uniformScale', this.desc, factor]);
  }
  distance(x, y, z) {
    return 1.0;
  }
  union(other) {
    return new FakeShape(['union', this.desc, other.desc]);
  }
  intersect(other) {
    return new FakeShape(['intersect', this.desc, other.desc]);
  }
  subtract(other) {
    return new FakeShape(['subtract', this.desc, other.desc]);
  }
  smoothUnion(other, radius) {
    return new FakeShape(['smoothUnion', this.desc, other.desc, radius]);
  }
}

// Stand-in for WasmProfile2D: records calls so tests can assert delegation.
class FakeProfile {
  constructor(x, y) {
    this.trace = [['new', x, y]];
    this.freed = false;
  }
  lineTo(x, y) {
    this.trace.push(['lineTo', x, y]);
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

// Structural comparison that ignores node ids and retained shapes.
function skeleton(node) {
  return {
    op: node.op,
    args: node.args,
    children: node.children.map(skeleton),
  };
}

describe('runTracedScript', () => {
  it('traces a primitive and retains the real shape', () => {
    const { root, nodes } = runTracedScript('return Shape.sphere(2);', FakeShape);
    expect(root.op).toBe('sphere');
    expect(root.args).toEqual([2]);
    expect(root.children).toEqual([]);
    expect(root.shape).toBeInstanceOf(FakeShape);
    expect(root.shape.desc).toEqual(['sphere', 2]);
    expect(nodes).toHaveLength(1);
  });

  it('traces chained transforms and booleans', () => {
    const source = `
      const body = Shape.roundedBox(1.0, 0.55, 0.8, 0.15);
      const bump = Shape.sphere(0.55).translate(0, 0.65, 0);
      const solid = body.smoothUnion(bump, 0.25);
      return solid.subtract(Shape.cylinder(0.28, 2.0));
    `;
    const { root, nodes } = runTracedScript(source, FakeShape);
    expect(skeleton(root)).toEqual({
      op: 'subtract',
      args: [],
      children: [
        {
          op: 'smoothUnion',
          args: [0.25],
          children: [
            { op: 'roundedBox', args: [1.0, 0.55, 0.8, 0.15], children: [] },
            {
              op: 'translate',
              args: [0, 0.65, 0],
              children: [{ op: 'sphere', args: [0.55], children: [] }],
            },
          ],
        },
        { op: 'cylinder', args: [0.28, 2.0], children: [] },
      ],
    });
    expect(nodes).toHaveLength(6);
    // Every intermediate delegated to the real class.
    expect(root.shape.desc[0]).toBe('subtract');
  });

  it('handles loops and reuse — a shared subtree is one node', () => {
    const source = `
      const unit = Shape.sphere(0.5);
      let solid = unit;
      for (let i = 1; i < 3; i++) {
        solid = solid.union(unit.translate(i, 0, 0));
      }
      return solid;
    `;
    const { root } = runTracedScript(source, FakeShape);
    // union(union(unit, t1), t2) — `unit` node is shared by reference.
    const unitA = root.children[0].children[0];
    const unitB = root.children[0].children[1].children[0];
    const unitC = root.children[1].children[0];
    expect(unitA).toBe(unitB);
    expect(unitA).toBe(unitC);
  });

  it('omits an explicit undefined optional arg', () => {
    const { root } = runTracedScript(
      'return Shape.sphere(1).smoothUnion(Shape.sphere(2), undefined);',
      FakeShape
    );
    expect(root.args).toEqual([]);
  });

  it('rejects a non-shape operand with a helpful message', () => {
    expect(() =>
      runTracedScript('return Shape.sphere(1).union(42);', FakeShape)
    ).toThrow(/\.union\(\.\.\.\) expects a Shape/);
  });

  it('frees already-created shapes when the script throws', () => {
    const { TracingShape, nodes } = createTracer(FakeShape);
    expect(() => {
      const build = new Function('Shape', 'Shape.sphere(1); throw new Error("boom");');
      build(TracingShape);
    }).toThrow('boom');
    // Simulate what runTracedScript does on error.
    freeNodes(nodes);
    expect(nodes).toHaveLength(1);

    expect(() =>
      runTracedScript('Shape.sphere(1); throw new Error("boom");', FakeShape)
    ).toThrow('boom');
  });

  it('still requires the script to return a shape', () => {
    expect(() => runTracedScript('return 42;', FakeShape)).toThrow(
      /must return a Shape/
    );
  });
});

describe('nodeLabel', () => {
  it('formats primitives, transforms, and booleans', () => {
    const { root } = runTracedScript(
      `const a = Shape.box3(1.0, 0.55, 0.8).translate(0, 0.65, 0);
       return a.subtract(Shape.sphere(0.3));`,
      FakeShape
    );
    expect(nodeLabel(root)).toBe('Subtract');
    expect(nodeLabel(root.children[0])).toBe('Translate [0, 0.65, 0]');
    expect(nodeLabel(root.children[0].children[0])).toBe('Box3 [1, 0.55, 0.8]');
    expect(nodeLabel(root.children[1])).toBe('Sphere [0.3]');
  });

  it('shows optional args when present', () => {
    const { root } = runTracedScript(
      'return Shape.sphere(1).smoothUnion(Shape.sphere(2), 0.25);',
      FakeShape
    );
    expect(nodeLabel(root)).toBe('Smooth Union [0.25]');
  });

  it('trims float noise', () => {
    const { root } = runTracedScript('return Shape.sphere(0.1 + 0.2);', FakeShape);
    expect(nodeLabel(root)).toBe('Sphere [0.3]');
  });
});

describe('serializeTree', () => {
  it('inlines single-use primitive and transform chains', () => {
    const { root } = runTracedScript(
      'return Shape.sphere(0.55).translate(0, 0.65, 0).rotate(0, 1, 0, 0.5).uniformScale(2);',
      FakeShape
    );
    expect(serializeTree(root)).toBe(
      'return Shape.sphere(0.55).translate(0, 0.65, 0).rotate(0, 1, 0, 0.5).uniformScale(2);\n'
    );
  });

  it('hoists every boolean step into a readable const binding', () => {
    const { root } = runTracedScript(
      `const body = Shape.roundedBox(1, 0.55, 0.8, 0.15);
       const bump = Shape.sphere(0.55).translate(0, 0.65, 0);
       return body.smoothUnion(bump, 0.25).subtract(Shape.cylinder(0.28, 2));`,
      FakeShape
    );
    expect(serializeTree(root)).toBe(
      'const s1 = Shape.roundedBox(1, 0.55, 0.8, 0.15)' +
        '.smoothUnion(Shape.sphere(0.55).translate(0, 0.65, 0), 0.25);\n' +
        'return s1.subtract(Shape.cylinder(0.28, 2));\n'
    );
  });

  it('keeps one statement per feature as GUI unions accumulate', () => {
    const { root } = runTracedScript(
      `return Shape.box3(1, 1, 1)
         .union(Shape.sphere(0.5))
         .union(Shape.torus(0.6, 0.2))
         .union(Shape.cylinder(0.3, 0.6));`,
      FakeShape
    );
    expect(serializeTree(root)).toBe(
      'const s1 = Shape.box3(1, 1, 1).union(Shape.sphere(0.5));\n' +
        'const s2 = s1.union(Shape.torus(0.6, 0.2));\n' +
        'return s2.union(Shape.cylinder(0.3, 0.6));\n'
    );
  });

  it('prepends the given header with a blank separator line', () => {
    const { root } = runTracedScript('return Shape.sphere(1);', FakeShape);
    expect(serializeTree(root, { header: '// API docs\n' })).toBe(
      '// API docs\n\nreturn Shape.sphere(1);\n'
    );
    expect(serializeTree(root, { header: '' })).toBe('return Shape.sphere(1);\n');
  });

  it('hoists shared subtrees into const bindings', () => {
    const { root } = runTracedScript(
      `const unit = Shape.sphere(0.5);
       return unit.union(unit.translate(1, 0, 0));`,
      FakeShape
    );
    expect(serializeTree(root)).toBe(
      'const s1 = Shape.sphere(0.5);\n' +
        'return s1.union(s1.translate(1, 0, 0));\n'
    );
  });

  it('round-trips: re-tracing the serialized script gives an isomorphic tree', () => {
    const source = `
      const unit = Shape.sphere(0.5);
      let solid = Shape.box3(2, 0.2, 2);
      for (let i = 0; i < 3; i++) {
        solid = solid.smoothUnion(unit.translate(i - 1, 0.5, 0), 0.2);
      }
      return solid.subtract(Shape.cylinder(0.1, 3).translate(0, 0, 0.5));
    `;
    const first = runTracedScript(source, FakeShape);
    const script = serializeTree(first.root);
    const second = runTracedScript(script, FakeShape);
    expect(skeleton(second.root)).toEqual(skeleton(first.root));
    // Sharing is preserved, not duplicated: same node count both times.
    expect(second.nodes).toHaveLength(first.nodes.length);
  });
});

describe('scriptHeader', () => {
  it('extracts the leading // comment block, trimming trailing blanks', () => {
    const source = '// line one\n//   line two\n\nconst a = Shape.sphere(1);\n';
    expect(scriptHeader(source)).toBe('// line one\n//   line two\n');
  });

  it('returns "" when the script does not start with a comment', () => {
    expect(scriptHeader('const a = Shape.sphere(1);\n// trailing\n')).toBe('');
    expect(scriptHeader('')).toBe('');
    expect(scriptHeader('\n\n')).toBe('');
  });

  it('spans multi-line /* */ blocks and mixed comment styles', () => {
    const source = '/* multi\n   line */\n// more\nreturn Shape.sphere(1);\n';
    expect(scriptHeader(source)).toBe('/* multi\n   line */\n// more\n');
  });

  it('keeps interior blank lines but stops at the first code line', () => {
    const source = '// a\n\n// b\nreturn Shape.sphere(1); // inline\n// after\n';
    expect(scriptHeader(source)).toBe('// a\n\n// b\n');
  });

  it('round-trips through serializeTree GUI regeneration', () => {
    const { root } = runTracedScript('return Shape.sphere(1);', FakeShape);
    const header = '// the API reference header\n';
    const script = serializeTree(root, { header });
    // A later GUI edit re-extracts the same header from the current script.
    expect(scriptHeader(script)).toBe(header);
    const again = runTracedScript(script, FakeShape);
    expect(serializeTree(again.root, { header: scriptHeader(script) })).toBe(script);
  });
});

describe('freeNodes', () => {
  it('frees every retained shape and clears the references', () => {
    const { root, nodes } = runTracedScript(
      'return Shape.sphere(1).union(Shape.sphere(2));',
      FakeShape
    );
    const shapes = nodes.map((n) => n.shape);
    freeNodes(nodes);
    expect(shapes.every((s) => s.freed)).toBe(true);
    expect(nodes.every((n) => n.shape === null)).toBe(true);
    expect(root.shape).toBeNull();
    // Idempotent.
    expect(() => freeNodes(nodes)).not.toThrow();
  });
});

describe('sweep ops (extrude / revolve)', () => {
  const SQUARE = `
    const p = new Profile(0, 0);
    p.lineTo(2, 0);
    p.lineTo(2, 1);
    p.lineTo(0, 1);
    p.close();
    return Shape.extrude(p, 5);
  `;

  it('traces extrude with a profile snapshot and delegates to the real classes', () => {
    const { root } = runTracedScript(SQUARE, FakeShape, FakeProfile);
    expect(root.op).toBe('extrude');
    expect(root.args).toEqual([5]);
    expect(root.children).toEqual([]);
    expect(root.profile).toEqual({
      start: [0, 0],
      segs: [
        { x: 2, y: 0, bulge: 0 },
        { x: 2, y: 1, bulge: 0 },
        { x: 0, y: 1, bulge: 0 },
      ],
    });
    // Real profile calls flowed through (lineTo delegates as arcTo bulge 0).
    expect(root.shape.desc[1]).toEqual([
      ['new', 0, 0],
      ['arcTo', 2, 0, 0],
      ['arcTo', 2, 1, 0],
      ['arcTo', 0, 1, 0],
      ['close'],
    ]);
  });

  it('snapshots the profile at sweep time — later mutation is invisible', () => {
    const source = `
      const p = new Profile(0, 0);
      p.lineTo(1, 0);
      p.lineTo(1, 1);
      p.close();
      const solid = Shape.extrude(p, 2);
      p.lineTo(9, 9);
      return solid;
    `;
    const { root } = runTracedScript(source, FakeShape, FakeProfile);
    expect(root.profile.segs).toEqual([
      { x: 1, y: 0, bulge: 0 },
      { x: 1, y: 1, bulge: 0 },
    ]);
  });

  it('rejects a non-profile first argument with a helpful message', () => {
    expect(() =>
      runTracedScript('return Shape.extrude(42, 1);', FakeShape, FakeProfile)
    ).toThrow(/Shape\.extrude\(\.\.\.\) expects a Profile/);
  });

  it('frees real profiles once the script finishes', () => {
    const created = [];
    class TrackingProfile extends FakeProfile {
      constructor(x, y) {
        super(x, y);
        created.push(this);
      }
    }
    runTracedScript(SQUARE, FakeShape, TrackingProfile);
    expect(created).toHaveLength(1);
    expect(created[0].freed).toBe(true);
  });

  it('frees real profiles even when the script throws', () => {
    const created = [];
    class TrackingProfile extends FakeProfile {
      constructor(x, y) {
        super(x, y);
        created.push(this);
      }
    }
    expect(() =>
      runTracedScript(
        'const p = new Profile(0, 0); throw new Error("boom");',
        FakeShape,
        TrackingProfile
      )
    ).toThrow('boom');
    expect(created[0].freed).toBe(true);
  });

  it('labels sweep nodes', () => {
    const { root } = runTracedScript(SQUARE, FakeShape, FakeProfile);
    expect(nodeLabel(root)).toBe('Extrude [5]');
  });

  it('serializes a sweep as profile statements plus the sweep call', () => {
    const { root } = runTracedScript(SQUARE, FakeShape, FakeProfile);
    expect(serializeTree(root)).toBe(
      'const p1 = new Profile(0, 0);\n' +
        'p1.lineTo(2, 0);\n' +
        'p1.lineTo(2, 1);\n' +
        'p1.lineTo(0, 1);\n' +
        'p1.close();\n' +
        'return Shape.extrude(p1, 5);\n'
    );
  });

  it('serializes arcs with arcTo and round-trips revolve', () => {
    const source = `
      const p = new Profile(1, 0);
      p.arcTo(3, 0, 1);
      p.arcTo(1, 0, 1);
      p.close();
      return Shape.revolve(p, 360).rotate(1, 0, 0, 1.5707963267948966);
    `;
    const { root } = runTracedScript(source, FakeShape, FakeProfile);
    const script = serializeTree(root);
    expect(script).toContain('p1.arcTo(3, 0, 1);');
    expect(script).toContain('Shape.revolve(p1, 360)');
    const again = runTracedScript(script, FakeShape, FakeProfile);
    expect(serializeTree(again.root)).toBe(script);
    expect(skeleton(again.root)).toEqual(skeleton(root));
  });

  it('hoists a shared sweep node once, profile included', () => {
    const source = `
      const p = new Profile(0, 0);
      p.lineTo(1, 0);
      p.lineTo(1, 1);
      p.close();
      const solid = Shape.extrude(p, 2);
      return solid.union(solid.translate(3, 0, 0));
    `;
    const { root } = runTracedScript(source, FakeShape, FakeProfile);
    const script = serializeTree(root);
    expect(script.match(/new Profile/g)).toHaveLength(1);
    expect(script).toContain('const s1 = Shape.extrude(p1, 2);');
    const again = runTracedScript(script, FakeShape, FakeProfile);
    expect(serializeTree(again.root)).toBe(script);
  });
});

describe('profileSegStatement', () => {
  it('emits lineTo for a bulge-0 segment', () => {
    expect(profileSegStatement('p', { x: 1, y: 2, bulge: 0 })).toBe('p.lineTo(1, 2);');
  });

  it('emits arcTo for a bulged segment', () => {
    expect(profileSegStatement('p', { x: 1, y: 2, bulge: 0.5 })).toBe('p.arcTo(1, 2, 0.5);');
  });

  it('emits ellipseArcTo naming the ellipse geometry', () => {
    const seg = { kind: 'ellipse', x: -1, y: 0, cx: 0, cy: 0, rx: 2, ry: 1, rotation: 0, ccw: true };
    expect(profileSegStatement('p', seg)).toBe('p.ellipseArcTo(-1, 0, 0, 0, 2, 1, 0, true);');
  });

  it('emits cubicTo with control points first', () => {
    const seg = { kind: 'spline', x: 0, y: 1, c1x: 1, c1y: 2, c2x: 3, c2y: 4 };
    expect(profileSegStatement('p', seg)).toBe('p.cubicTo(1, 2, 3, 4, 0, 1);');
  });
});
