import { describe, expect, it } from 'vitest';
import {
  findUnionSeparating,
  nearestBody,
  replaceWithBlend,
  resolveEdgeTarget,
} from './edgeFillet.js';

function node(id, op, children = [], shape = null) {
  return { id, op, args: [], children, shape };
}

// A leaf whose SDF is the Euclidean distance to `center` — so nearestBody
// resolves a face origin to the body it sits on.
function body(id, op, center) {
  return node(id, op, [], {
    distance: (x, y, z) => Math.hypot(x - center[0], y - center[1], z - center[2]),
  });
}

describe('findUnionSeparating', () => {
  it('finds the union whose children hold the two bodies', () => {
    const a = node(1, 'box3');
    const b = node(2, 'sphere');
    const root = node(3, 'union', [a, b]);
    expect(findUnionSeparating(root, 1, 2)).toBe(root);
  });

  it('returns null for identical bodies (a primitive-intrinsic edge)', () => {
    const a = node(1, 'box3');
    const root = node(2, 'union', [a, node(3, 'sphere')]);
    expect(findUnionSeparating(root, 1, 1)).toBeNull();
  });

  it('returns null when the bodies meet only across a subtract', () => {
    const a = node(1, 'box3');
    const b = node(2, 'sphere');
    const root = node(3, 'subtract', [a, b]);
    expect(findUnionSeparating(root, 1, 2)).toBeNull();
  });

  it('picks the deepest union that splits the two bodies', () => {
    const a = node(1, 'box3');
    const b = node(2, 'sphere');
    const inner = node(3, 'union', [a, b]);
    const c = node(4, 'cylinder');
    const root = node(5, 'union', [inner, c]);
    // a and b are separated by the inner union, not the outer one.
    expect(findUnionSeparating(root, 1, 2)).toBe(inner);
    // a and c are only separated by the outer union.
    expect(findUnionSeparating(root, 1, 4)).toBe(root);
  });
});

describe('nearestBody', () => {
  it('resolves a point to the closest pickable body', () => {
    const a = body(1, 'box3', [0, 0, 0]);
    const b = body(2, 'sphere', [5, 0, 0]);
    const root = node(3, 'union', [a, b]);
    expect(nearestBody(root, [0.1, 0, 0])).toBe(a);
    expect(nearestBody(root, [4.9, 0, 0])).toBe(b);
  });
});

describe('replaceWithBlend', () => {
  it('rewrites the union into a filletEdge carrying radius and polyline', () => {
    const a = node(1, 'box3');
    const b = node(2, 'sphere');
    const root = node(3, 'union', [a, b], { free() {} });
    const edge = [0, 0, 0, 1, 0, 0];
    const next = replaceWithBlend(root, 3, 'fillet', 0.2, edge);
    expect(next.op).toBe('filletEdge');
    expect(next.args).toEqual([0.2]);
    expect(next.edge).toEqual(edge);
    // The polyline is copied, not aliased.
    expect(next.edge).not.toBe(edge);
    expect(next.shape).toBeNull();
    expect(next.children).toHaveLength(2);
  });

  it('emits chamferEdge in chamfer mode', () => {
    const root = node(3, 'union', [node(1, 'box3'), node(2, 'sphere')]);
    const next = replaceWithBlend(root, 3, 'chamfer', 0.1, [0, 0, 0, 1, 0, 0]);
    expect(next.op).toBe('chamferEdge');
  });

  it('returns the root unchanged when the id is missing', () => {
    const root = node(3, 'union', [node(1, 'box3'), node(2, 'sphere')]);
    expect(replaceWithBlend(root, 999, 'fillet', 0.2, [])).toBe(root);
  });
});

describe('resolveEdgeTarget', () => {
  const pickBetween = (originA, originB) => ({
    regionA: { plane: { origin: originA } },
    regionB: { plane: { origin: originB } },
    flat: [0, 0, 0, 1, 0, 0],
  });

  it('names the union to rewrite for an edge between two bodies', () => {
    const a = body(1, 'box3', [0, 0, 0]);
    const b = body(2, 'sphere', [5, 0, 0]);
    const root = node(3, 'union', [a, b]);
    const result = resolveEdgeTarget(root, pickBetween([0, 0, 0], [5, 0, 0]));
    expect(result.ok).toBe(true);
    expect(result.unionId).toBe(3);
    expect(result.bodyA).toBe(a);
    expect(result.bodyB).toBe(b);
  });

  it('rejects an edge that resolves to a single body', () => {
    const a = body(1, 'box3', [0, 0, 0]);
    const b = body(2, 'sphere', [5, 0, 0]);
    const root = node(3, 'union', [a, b]);
    // Both face origins sit on body a.
    const result = resolveEdgeTarget(root, pickBetween([0, 0, 0], [0.2, 0, 0]));
    expect(result.ok).toBe(false);
    expect(result.reason).toMatch(/single body|two bodies/i);
  });

  it('rejects an edge where the bodies are not unioned', () => {
    const a = body(1, 'box3', [0, 0, 0]);
    const b = body(2, 'sphere', [5, 0, 0]);
    const root = node(3, 'subtract', [a, b]);
    const result = resolveEdgeTarget(root, pickBetween([0, 0, 0], [5, 0, 0]));
    expect(result.ok).toBe(false);
    expect(result.reason).toMatch(/unioned/i);
  });
});
