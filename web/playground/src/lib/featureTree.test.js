import { describe, expect, it } from 'vitest';
import {
  buildFeatures,
  buildReferenceFeatures,
  pruneTree,
  resolveKeys,
} from './featureTree.js';

// Plain construction-tree nodes (the shape sceneTree's tracer produces),
// with ids in creation order.
function node(id, op, args = [], children = [], extra = {}) {
  return { id, op, args, children, shape: null, ...extra };
}

/** box.translate(...).union(sphere) with creation-ordered ids. */
function sampleTree() {
  const box = node(1, 'box3', [1, 1, 1]);
  const translate = node(2, 'translate', [0, 2, 0], [box]);
  const sphere = node(3, 'sphere', [0.5]);
  const union = node(4, 'union', [], [translate, sphere]);
  return { box, translate, sphere, union };
}

describe('buildFeatures', () => {
  it('returns an empty list without a root', () => {
    expect(buildFeatures(null)).toEqual([]);
  });

  it('lists features chronologically with per-type numbering', () => {
    const { union } = sampleTree();
    const features = buildFeatures(union);
    expect(features.map((f) => f.name)).toEqual([
      'Box1',
      'Translate1',
      'Sphere1',
      'Union1',
    ]);
    expect(features.map((f) => f.id)).toEqual([1, 2, 3, 4]);
    expect(features.map((f) => f.kind)).toEqual([
      'primitive',
      'transform',
      'primitive',
      'boolean',
    ]);
    expect(features[0].key).toBe('box:1');
    expect(features[3].key).toBe('union:1');
  });

  it('numbers repeated types independently', () => {
    const a = node(1, 'box3', [1, 1, 1]);
    const b = node(2, 'box3', [2, 2, 2]);
    const u = node(3, 'union', [], [a, b]);
    const names = buildFeatures(u).map((f) => f.name);
    expect(names).toEqual(['Box1', 'Box2', 'Union1']);
  });

  it('nests a sketch feature under a profile-carrying sweep', () => {
    const profile = { start: [0, 0], segs: [{ x: 1, y: 0, bulge: 0 }] };
    const ext = node(1, 'extrude', [2], [], { profile });
    const features = buildFeatures(ext);
    expect(features.map((f) => f.name)).toEqual(['Extrude1', 'Sketch1']);
    const sketch = features[1];
    expect(sketch.kind).toBe('sketch');
    expect(sketch.depth).toBe(1);
    expect(sketch.parentKey).toBe('extrude:1');
    expect(sketch.node).toBe(ext);
  });

  it('applies user renames by key and keeps the default name', () => {
    const { union } = sampleTree();
    const features = buildFeatures(union, { 'box:1': 'Base plate' });
    expect(features[0].name).toBe('Base plate');
    expect(features[0].defaultName).toBe('Box1');
    expect(features[1].name).toBe('Translate1');
  });

  it('lists a shared (DAG) node once', () => {
    const s = node(1, 'sphere', [1]);
    const t = node(2, 'translate', [1, 0, 0], [s]);
    const u = node(3, 'union', [], [t, s]);
    expect(buildFeatures(u).map((f) => f.id)).toEqual([1, 2, 3]);
  });
});

describe('resolveKeys', () => {
  it('maps feature keys to node ids, sketch keys to the owning sweep', () => {
    const profile = { start: [0, 0], segs: [{ x: 1, y: 0, bulge: 0 }] };
    const ext = node(1, 'extrude', [2], [], { profile });
    const box = node(2, 'box3', [1, 1, 1]);
    const u = node(3, 'union', [], [ext, box]);
    const features = buildFeatures(u);
    expect(resolveKeys(features, ['box:1'])).toEqual(new Set([2]));
    expect(resolveKeys(features, ['sketch:1'])).toEqual(new Set([1]));
    expect(resolveKeys(features, ['nope:9'])).toEqual(new Set());
  });
});

describe('pruneTree', () => {
  it('returns the root unchanged for an empty id set', () => {
    const { union } = sampleTree();
    expect(pruneTree(union, new Set())).toBe(union);
  });

  it('collapses a boolean onto the surviving operand when a leaf is pruned', () => {
    const { union, translate } = sampleTree();
    // Hide the sphere: the union has one operand left, so it disappears.
    expect(pruneTree(union, new Set([3]))).toBe(translate);
  });

  it('bypasses a pruned boolean to its receiver', () => {
    const { union, translate } = sampleTree();
    expect(pruneTree(union, new Set([4]))).toBe(translate);
  });

  it('bypasses a pruned transform to its child', () => {
    const { union, box, sphere } = sampleTree();
    const pruned = pruneTree(union, new Set([2]));
    expect(pruned.op).toBe('union');
    expect(pruned.children).toEqual([box, sphere]);
    expect(pruned.shape).toBeNull();
  });

  it('drops a transform whose only child is pruned', () => {
    const { union, sphere } = sampleTree();
    // Hiding the box leaves translate with nothing to act on.
    expect(pruneTree(union, new Set([1]))).toBe(sphere);
  });

  it('returns null when nothing remains', () => {
    const { union } = sampleTree();
    expect(pruneTree(union, new Set([1, 3]))).toBeNull();
    const lone = node(1, 'sphere', [1]);
    expect(pruneTree(lone, new Set([1]))).toBeNull();
  });

  it('prunes a shared (DAG) node from every parent', () => {
    const s = node(1, 'sphere', [1]);
    const t = node(2, 'translate', [1, 0, 0], [s]);
    const u = node(3, 'union', [], [t, s]);
    expect(pruneTree(u, new Set([1]))).toBeNull();
  });

  it('keeps a sweep profile on rebuilt ancestors', () => {
    const profile = { start: [0, 0], segs: [{ x: 1, y: 0, bulge: 0 }] };
    const ext = node(1, 'extrude', [2], [], { profile });
    const box = node(2, 'box3', [1, 1, 1]);
    const sub = node(3, 'subtract', [], [ext, box]);
    const pruned = pruneTree(sub, new Set([2]));
    expect(pruned).toBe(ext);
    expect(pruned.profile).toBe(profile);
  });
});

describe('buildReferenceFeatures', () => {
  const refGeom = [
    { id: 1, name: 'Plane1', kind: 'plane', entity: { kind: 'plane' } },
    { id: 3, name: 'MyAxis', kind: 'axis', entity: { kind: 'axis' } },
    { id: 4, name: 'CSys1', kind: 'csys', entity: { kind: 'csys' } },
  ];

  it('produces reference-flagged rows keyed by id', () => {
    const rows = buildReferenceFeatures(refGeom);
    expect(rows.map((r) => r.key)).toEqual(['ref:1', 'ref:3', 'ref:4']);
    expect(rows.every((r) => r.reference === true)).toBe(true);
    expect(rows.map((r) => r.refId)).toEqual([1, 3, 4]);
  });

  it('carries the item name and a type label from REFERENCE_META', () => {
    const rows = buildReferenceFeatures(refGeom);
    expect(rows[0]).toMatchObject({ name: 'Plane1', kind: 'plane', type: 'Plane' });
    expect(rows[1].name).toBe('MyAxis');
    expect(rows[2].type).toBe('CSys');
  });

  it('returns an empty list for no reference geometry', () => {
    expect(buildReferenceFeatures()).toEqual([]);
    expect(buildReferenceFeatures([])).toEqual([]);
  });
});
