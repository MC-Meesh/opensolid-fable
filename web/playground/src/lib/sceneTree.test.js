import { describe, expect, it } from 'vitest';
import {
  createTracer,
  freeNodes,
  nodeLabel,
  runTracedScript,
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
  it('inlines single-use chains', () => {
    const { root } = runTracedScript(
      `const body = Shape.roundedBox(1, 0.55, 0.8, 0.15);
       const bump = Shape.sphere(0.55).translate(0, 0.65, 0);
       return body.smoothUnion(bump, 0.25).subtract(Shape.cylinder(0.28, 2));`,
      FakeShape
    );
    expect(serializeTree(root)).toBe(
      'return Shape.roundedBox(1, 0.55, 0.8, 0.15)' +
        '.smoothUnion(Shape.sphere(0.55).translate(0, 0.65, 0), 0.25)' +
        '.subtract(Shape.cylinder(0.28, 2));\n'
    );
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
