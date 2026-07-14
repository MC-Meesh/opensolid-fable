import { describe, it, expect } from 'vitest';
import { findBlendTarget, buildFilletShape, filletTreeNode } from './fillet.js';
import { serializeTree, runTracedScript } from './sceneTree.js';

// Stand-in shape whose distance() is scripted per node so findBlendTarget can
// be exercised deterministically.
class FakeShape {
  constructor(dist) {
    this.dist = dist;
  }
  distance() {
    return this.dist;
  }
  filletEdge(other, radius, edge) {
    return new FakeShape(0, ['filletEdge', radius, edge.slice()]);
  }
  chamferEdge(other, radius, edge) {
    return new FakeShape(0, ['chamferEdge', radius, edge.slice()]);
  }
}

const leaf = (id, op, dist) => ({ id, op, args: [], children: [], shape: new FakeShape(dist) });
const union = (id, a, b, op = 'union') => ({
  id,
  op,
  args: [],
  children: [a, b],
  shape: new FakeShape(0),
});

describe('findBlendTarget', () => {
  it('finds the union whose two operands both pass through the seed', () => {
    const a = leaf(1, 'box3', 0.001);
    const b = leaf(2, 'box3', -0.002);
    const root = union(3, a, b);
    const hit = findBlendTarget(root, [1, 0, 1]);
    expect(hit).not.toBe(null);
    expect(hit.node.id).toBe(3);
    expect(hit.a.id).toBe(1);
    expect(hit.b.id).toBe(2);
  });

  it('returns null when an operand is far from the seed', () => {
    const root = union(3, leaf(1, 'box3', 0.001), leaf(2, 'box3', 0.5));
    expect(findBlendTarget(root, [9, 9, 9])).toBe(null);
  });

  it('ignores subtract / intersect (not union-family)', () => {
    const root = {
      id: 3,
      op: 'subtract',
      args: [],
      children: [leaf(1, 'box3', 0), leaf(2, 'cyl', 0)],
      shape: new FakeShape(0),
    };
    expect(findBlendTarget(root, [0, 0, 0])).toBe(null);
  });

  it('picks the closest-fitting union when several qualify', () => {
    const inner = union(3, leaf(1, 'a', 0.004), leaf(2, 'b', 0.004));
    const outer = union(6, inner, leaf(5, 'c', 0.001), 'smoothUnion');
    // outer's operands: inner.shape.distance()=0 and leaf5=0.001 → score 0.001
    // inner's operands: 0.004 + 0.004 = 0.008. Outer fits better.
    const hit = findBlendTarget(outer, [0, 0, 0]);
    expect(hit.node.id).toBe(6);
  });
});

describe('buildFilletShape', () => {
  it('calls filletEdge with radius and a materialized polyline', () => {
    const a = new FakeShape(0);
    const b = new FakeShape(0);
    const shape = buildFilletShape(a, b, { mode: 'fillet', radius: 0.2, edge: [0, 0, 0, 1, 0, 0] });
    expect(shape.dist).toBe(0); // returned FakeShape
  });

  it('routes chamfer mode to chamferEdge', () => {
    let calledOp = null;
    const a = {
      filletEdge: () => ((calledOp = 'filletEdge'), new FakeShape(0)),
      chamferEdge: () => ((calledOp = 'chamferEdge'), new FakeShape(0)),
    };
    buildFilletShape(a, new FakeShape(0), { mode: 'chamfer', radius: 0.1, edge: [0, 0, 0] });
    expect(calledOp).toBe('chamferEdge');
  });
});

describe('filletTreeNode', () => {
  it('replaces the target union with a filletEdge over the same operands', () => {
    const a = leaf(1, 'box3', 0);
    const b = leaf(2, 'box3', 0);
    const root = union(3, a, b);
    const out = filletTreeNode(root, {
      targetId: 3,
      mode: 'fillet',
      radius: 0.1,
      edge: [1, -1, 1, 1, 1, 1],
    });
    expect(out.op).toBe('filletEdge');
    expect(out.args).toEqual([0.1, [1, -1, 1, 1, 1, 1]]);
    expect(out.children).toEqual([a, b]); // operands shared by reference
  });

  it('rewrites a nested target and shares untouched subtrees', () => {
    const a = leaf(1, 'a', 0);
    const b = leaf(2, 'b', 0);
    const c = leaf(4, 'c', 0);
    const inner = union(3, a, b);
    const root = union(5, inner, c);
    const out = filletTreeNode(root, { targetId: 3, mode: 'chamfer', radius: 0.05, edge: [0, 0, 0, 1, 0, 0] });
    expect(out.id).not.toBe(5); // ancestor re-cloned with a fresh id
    expect(out.op).toBe('union');
    expect(out.children[1]).toBe(c); // untouched sibling shared
    expect(out.children[0].op).toBe('chamferEdge');
  });

  it('throws when the target id is absent', () => {
    const root = union(3, leaf(1, 'a', 0), leaf(2, 'b', 0));
    expect(() => filletTreeNode(root, { targetId: 99, mode: 'fillet', radius: 0.1, edge: [0, 0, 0] })).toThrow(
      /no blend target/
    );
  });

  it('produces a tree that serializes to a runnable edge-blend script', () => {
    // End-to-end: trace a real union, rewrite it, serialize, re-run.
    class ScriptShape {
      constructor(d) {
        this.desc = d;
      }
      free() {}
      static box3(x, y, z) {
        return new ScriptShape(['box3', x, y, z]);
      }
      translate(x, y, z) {
        return new ScriptShape(['t', this.desc, x, y, z]);
      }
      distance() {
        return 0;
      }
      union(o) {
        return new ScriptShape(['union', this.desc, o.desc]);
      }
      filletEdge(o, r, e) {
        return new ScriptShape(['filletEdge', this.desc, o.desc, r, e.slice()]);
      }
    }
    const src = 'const a = Shape.box3(1,1,1); const b = Shape.box3(1,1,1).translate(1,0,0); return a.union(b);';
    const { root } = runTracedScript(src, ScriptShape, class {});
    const rewritten = filletTreeNode(root, {
      targetId: root.id,
      mode: 'fillet',
      radius: 0.2,
      edge: [1, -1, 1, 1, 1, 1],
    });
    const script = serializeTree(rewritten);
    expect(script).toContain('.filletEdge(');
    expect(script).toContain(', 0.2, [1, -1, 1, 1, 1, 1])');
    // Re-runs without throwing and returns a filletEdge shape.
    const again = runTracedScript(script, ScriptShape, class {});
    expect(again.root.op).toBe('filletEdge');
  });
});
