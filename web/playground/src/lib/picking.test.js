import { describe, expect, it } from 'vitest';
import { pickCandidates, pickNodeAt } from './picking.js';

function node(id, op, children = [], shape = null) {
  return { id, op, args: [], children, shape };
}

function withDist(n, d) {
  n.shape = { distance: () => d };
  return n;
}

describe('pickCandidates', () => {
  it('returns a lone primitive as a candidate', () => {
    const root = node(1, 'sphere');
    expect(pickCandidates(root)).toEqual([root]);
  });

  it('returns a unary chain (translate(sphere)) as one candidate', () => {
    const prim = node(1, 'sphere');
    const root = node(2, 'translate', [prim]);
    expect(pickCandidates(root)).toEqual([root]);
  });

  it('splits at binary ops into leaf candidates', () => {
    const left = node(1, 'sphere');
    const right = node(2, 'box3');
    const root = node(3, 'union', [left, right]);
    const c = pickCandidates(root);
    expect(c).toHaveLength(2);
    expect(c).toContain(left);
    expect(c).toContain(right);
  });

  it('finds unary chains inside binary ops', () => {
    const s = node(1, 'sphere');
    const ts = node(2, 'translate', [s]);
    const b = node(3, 'box3');
    const root = node(4, 'subtract', [ts, b]);
    const c = pickCandidates(root);
    expect(c).toHaveLength(2);
    expect(c).toContain(ts);
    expect(c).toContain(b);
  });

  it('handles nested binary ops', () => {
    const a = node(1, 'sphere');
    const b = node(2, 'box3');
    const c = node(3, 'cylinder');
    const inner = node(4, 'union', [a, b]);
    const root = node(5, 'subtract', [inner, c]);
    const candidates = pickCandidates(root);
    expect(candidates).toHaveLength(3);
    expect(candidates).toContain(a);
    expect(candidates).toContain(b);
    expect(candidates).toContain(c);
  });

  it('deduplicates shared nodes in a DAG', () => {
    const shared = node(1, 'sphere');
    const t = node(2, 'translate', [shared]);
    const root = node(3, 'union', [shared, t]);
    const c = pickCandidates(root);
    expect(c).toHaveLength(2);
    const ids = c.map((n) => n.id);
    expect(ids).toContain(1);
    expect(ids).toContain(2);
  });

  it('handles unary wrapping a binary (translate(union(...)))', () => {
    const a = node(1, 'sphere');
    const b = node(2, 'box3');
    const u = node(3, 'union', [a, b]);
    const root = node(4, 'translate', [u]);
    const c = pickCandidates(root);
    expect(c).toHaveLength(2);
    expect(c).toContain(a);
    expect(c).toContain(b);
  });
});

describe('pickNodeAt', () => {
  it('picks the candidate nearest to the point', () => {
    const a = withDist(node(1, 'sphere'), 2.0);
    const b = withDist(node(2, 'box3'), 0.3);
    const c = withDist(node(3, 'cylinder'), 1.5);
    expect(pickNodeAt([a, b, c], [0, 0, 0])).toBe(b);
  });

  it('uses absolute distance (inside is also near)', () => {
    const outside = withDist(node(1, 'sphere'), 0.5);
    const inside = withDist(node(2, 'box3'), -0.1);
    expect(pickNodeAt([outside, inside], [0, 0, 0])).toBe(inside);
  });

  it('returns null for empty candidates', () => {
    expect(pickNodeAt([], [0, 0, 0])).toBeNull();
  });
});
