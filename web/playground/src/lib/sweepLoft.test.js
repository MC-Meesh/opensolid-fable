import { describe, expect, it } from 'vitest';
import { runTracedScript, serializeTree } from './sceneTree.js';
import { addFeatureNode, checkConsistency } from './storeSync.js';
import { defaultLoftNode, defaultSweepNode } from './sweep.js';
import { buildFeatures } from './featureTree.js';

// Stand-ins covering just the sweep/loft scripting surface.
class FakeShape {
  free() {}
  static sweep(profile, path) {
    expect(profile).toBeInstanceOf(FakeProfile);
    expect(path).toBeInstanceOf(FakePath);
    return new FakeShape();
  }
  static loft(bottom, top, height) {
    expect(bottom).toBeInstanceOf(FakeProfile);
    expect(top).toBeInstanceOf(FakeProfile);
    expect(typeof height).toBe('number');
    return new FakeShape();
  }
  union() {
    return new FakeShape();
  }
}
class FakeProfile {
  arcTo() {}
  lineTo() {}
  close() {}
  free() {}
}
class FakePath {
  lineTo() {}
  free() {}
}

const trace = (src) => runTracedScript(src, FakeShape, FakeProfile, FakePath);

describe('sweep scripting', () => {
  const script = `const p = new Profile(-0.25, 0);
p.arcTo(0.25, 0, 1);
p.arcTo(-0.25, 0, 1);
p.close();
const path = new Path(0, 0, 0);
path.lineTo(0, 1, 0);
path.lineTo(1, 1, 0);
return Shape.sweep(p, path);
`;

  it('traces a sweep node carrying profile and path snapshots', () => {
    const { root } = trace(script);
    expect(root.op).toBe('sweep');
    expect(root.args).toEqual([]);
    expect(root.profile.start).toEqual([-0.25, 0]);
    expect(root.path).toEqual([
      [0, 0, 0],
      [0, 1, 0],
      [1, 1, 0],
    ]);
  });

  it('round-trips: serialized tree reparses to the same model', () => {
    const { root } = trace(script);
    const emitted = serializeTree(root);
    expect(emitted).toContain('new Path(0, 0, 0)');
    expect(emitted).toContain('Shape.sweep(');
    expect(checkConsistency(emitted, root).ok).toBe(true);
  });
});

describe('loft scripting', () => {
  const script = `const b = new Profile(-0.5, -0.5);
b.lineTo(0.5, -0.5);
b.lineTo(0.5, 0.5);
b.lineTo(-0.5, 0.5);
b.close();
const t = new Profile(-0.6, 0);
t.arcTo(0.6, 0, 1);
t.arcTo(-0.6, 0, 1);
t.close();
return Shape.loft(b, t, 1);
`;

  it('traces a loft node carrying both profile snapshots and height', () => {
    const { root } = trace(script);
    expect(root.op).toBe('loft');
    expect(root.args).toEqual([1]);
    expect(root.profile.start).toEqual([-0.5, -0.5]);
    expect(root.profile2.start).toEqual([-0.6, 0]);
  });

  it('round-trips: serialized tree reparses to the same model', () => {
    const { root } = trace(script);
    const emitted = serializeTree(root);
    expect(emitted).toContain('Shape.loft(');
    expect(checkConsistency(emitted, root).ok).toBe(true);
  });
});

describe('default feature nodes', () => {
  it('the default Sweep grafts and round-trips consistently', () => {
    const root = addFeatureNode(null, defaultSweepNode());
    const emitted = serializeTree(root);
    expect(checkConsistency(emitted, root).ok).toBe(true);
  });

  it('the default Loft grafts and round-trips consistently', () => {
    const root = addFeatureNode(null, defaultLoftNode());
    const emitted = serializeTree(root);
    expect(checkConsistency(emitted, root).ok).toBe(true);
  });

  it('unions onto an existing feature', () => {
    const first = addFeatureNode(null, defaultSweepNode());
    const combined = addFeatureNode(first, defaultLoftNode());
    expect(combined.op).toBe('union');
    expect(checkConsistency(serializeTree(combined), combined).ok).toBe(true);
  });
});

describe('feature tree', () => {
  it('lists Sweep and Loft with a nested sketch', () => {
    const { root } = trace(`const p = new Profile(-0.25, 0);
p.arcTo(0.25, 0, 1);
p.arcTo(-0.25, 0, 1);
p.close();
const path = new Path(0, 0, 0);
path.lineTo(0, 1, 0);
return Shape.sweep(p, path);
`);
    const features = buildFeatures(root);
    const sweep = features.find((f) => f.type === 'Sweep');
    expect(sweep).toBeTruthy();
    expect(sweep.kind).toBe('sweep');
    // A nested Sketch feature is attached to the profile-bearing sweep node.
    expect(features.some((f) => f.type === 'Sketch' && f.depth === 1)).toBe(true);
  });
});
