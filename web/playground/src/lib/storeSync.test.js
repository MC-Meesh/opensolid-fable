import { describe, expect, it, vi } from 'vitest';
import {
  addPrimitiveNode,
  assertStoreConsistency,
  checkConsistency,
  createStubApi,
  hashTree,
  reparseTree,
} from './storeSync.js';
import { BINARY_OPS, serializeTree } from './sceneTree.js';
import { applyRotate, applyScale, applyTranslate } from './transformEdit.js';
import { OP_SPECS, setBooleanOp, setNodeArg } from './propertyEdit.js';
import { deleteNode } from './deleteNode.js';
import { sweepTreeNode } from './sweep.js';

// Deterministic PRNG (mulberry32) so failures reproduce from the seed.
function mulberry32(seed) {
  let a = seed >>> 0;
  return () => {
    a = (a + 0x6d2b79f5) >>> 0;
    let t = a;
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  };
}

function reachable(root) {
  const out = [];
  const seen = new Set();
  const walk = (node) => {
    if (seen.has(node.id)) return;
    seen.add(node.id);
    out.push(node);
    node.children.forEach(walk);
  };
  walk(root);
  return out;
}

const PRIMS = [
  ['sphere', [0.5]],
  ['box3', [0.5, 0.4, 0.3]],
  ['roundedBox', [0.5, 0.5, 0.5, 0.1]],
  ['cylinder', [0.3, 0.6]],
  ['torus', [0.6, 0.2]],
  ['capsule', [0, 0, 0, 0, 1, 0, 0.2]],
];

function randomPrim(rand) {
  return PRIMS[Math.floor(rand() * PRIMS.length)];
}

function randomSweep(root, rand) {
  const kind = rand() < 0.5 ? 'extrude' : 'revolve';
  return sweepTreeNode(root, {
    kind,
    plane: 'XY',
    value: kind === 'extrude' ? 0.8 : 270,
    ops: {
      start: [0.1, 0.1],
      segs: [
        { x: 1, y: 0.1, bulge: 0 },
        { x: 1, y: 1, bulge: 0.3 },
        { x: 0.1, y: 0.1, bulge: 0 },
      ],
    },
  });
}

// One random GUI edit expressed as a store (tree) mutation. Mirrors the App
// mutation surface: palette add, gizmo transforms, property-panel arg edits,
// boolean op switch, feature delete, sweep apply. Returns the new root (the
// same root when the drawn mutation does not apply to the drawn node).
function randomMutation(root, rand) {
  const nodes = reachable(root);
  const node = nodes[Math.floor(rand() * nodes.length)];
  const num = (lo, hi) => Math.round((lo + rand() * (hi - lo)) * 1000) / 1000;
  switch (Math.floor(rand() * 8)) {
    case 0:
      return addPrimitiveNode(root, ...randomPrim(rand));
    case 1:
      return applyTranslate(root, node.id, [num(-1, 1), num(-1, 1), num(-1, 1)]);
    case 2:
      return applyRotate(root, node.id, [0, 1, 0], num(-3, 3), [0, 0, 0]);
    case 3: {
      const f = num(0.5, 1.5);
      return rand() < 0.5
        ? applyScale(root, node.id, [f, f, f], [0, 0, 0])
        : applyScale(root, node.id, [f, num(0.5, 1.5), num(0.5, 1.5)], [0, 0, 0]);
    }
    case 4: {
      const fields = OP_SPECS[node.op]?.groups.flatMap((g) => g.fields) ?? [];
      if (fields.length === 0) return root;
      const field = fields[Math.floor(rand() * fields.length)];
      const lo = Math.max(field.min, -2);
      const hi = Math.min(field.max, 2);
      const result = setNodeArg(root, node.id, field.arg, num(lo, hi));
      return result.root ?? root;
    }
    case 5: {
      const booleans = nodes.filter((n) => BINARY_OPS.includes(n.op));
      if (booleans.length === 0) return root;
      const target = booleans[Math.floor(rand() * booleans.length)];
      const op = BINARY_OPS[Math.floor(rand() * BINARY_OPS.length)];
      const result = setBooleanOp(root, target.id, op);
      return result.root ?? root;
    }
    case 6: {
      const result = deleteNode(root, node.id);
      return result.root ?? root;
    }
    default:
      return randomSweep(root, rand);
  }
}

describe('reparseTree', () => {
  it('parses a canonical script into a plain-data tree with nothing live', () => {
    const rp = reparseTree('return Shape.sphere(0.5).translate(0, 1, 0);\n');
    expect(rp.root.op).toBe('translate');
    expect(rp.root.children[0].op).toBe('sphere');
    expect(rp.root.shape).toBeNull();
    expect(rp.leaked).toBe(0);
  });

  it('traces and frees shapes a script builds but never returns (no orphans)', () => {
    const rp = reparseTree(
      'const junk = Shape.sphere(2);\nreturn Shape.box3(1, 1, 1);\n'
    );
    expect(rp.leaked).toBe(0);
    expect(rp.nodes).toHaveLength(2); // the orphan is traced (and freed) too
    expect(reachable(rp.root)).toHaveLength(1); // …but is not in the store
  });

  it('supports the full API surface including profiles and sweeps', () => {
    const rp = reparseTree(
      [
        'const p1 = new Profile(0, 0);',
        'p1.lineTo(1, 0);',
        'p1.arcTo(1, 1, 0.3);',
        'p1.lineTo(0, 0);',
        'p1.close();',
        'return Shape.extrude(p1, 0.8).smoothUnion(Shape.torus(0.6, 0.2), 0.1);',
        '',
      ].join('\n')
    );
    expect(rp.root.op).toBe('smoothUnion');
    expect(rp.root.children[0].profile.segs).toHaveLength(3);
    expect(rp.leaked).toBe(0);
  });

  it('throws on broken scripts', () => {
    expect(() => reparseTree('return Shape.nope(1);')).toThrow();
    expect(() => reparseTree('const x = 1;')).toThrow(/return a Shape/);
  });
});

describe('createStubApi leak tracking', () => {
  it('counts unfreed instances', () => {
    const api = createStubApi();
    const a = api.Shape.sphere(1);
    const b = a.translate(0, 1, 0);
    expect(api.live.size).toBe(2);
    a.free();
    b.free();
    expect(api.live.size).toBe(0);
  });
});

describe('addPrimitiveNode', () => {
  it('returns the bare primitive for an empty store', () => {
    const node = addPrimitiveNode(null, 'sphere', [0.5]);
    expect(serializeTree(node)).toBe('return Shape.sphere(0.5);\n');
  });

  it('unions the primitive onto an existing tree', () => {
    const { root } = reparseTree('return Shape.box3(1, 1, 1);\n');
    const next = addPrimitiveNode(root, 'torus', [0.6, 0.2]);
    expect(serializeTree(next)).toBe(
      'return Shape.box3(1, 1, 1).union(Shape.torus(0.6, 0.2));\n'
    );
  });
});

describe('checkConsistency / assertStoreConsistency', () => {
  it('accepts a script and the tree it evaluates to, canonical or not', () => {
    const script = 'const r = 0.25 * 2;\nreturn Shape.sphere(r);\n';
    const { root } = reparseTree(script);
    expect(checkConsistency(script, root).ok).toBe(true);
  });

  it('rejects a tree that drifted from the script', () => {
    const script = 'return Shape.sphere(0.5);\n';
    const { root } = reparseTree(script);
    const drifted = { ...root, args: [0.75] };
    const result = checkConsistency(script, drifted);
    expect(result.ok).toBe(false);
    expect(result.expected).toContain('0.5');
    expect(result.actual).toContain('0.75');
  });

  it('rejects (without throwing) when the script no longer evaluates', () => {
    const { root } = reparseTree('return Shape.sphere(0.5);\n');
    const result = checkConsistency('return Shape.sphere(;\n', root);
    expect(result.ok).toBe(false);
    expect(result.error).toBeTruthy();
  });

  it('assertStoreConsistency console.errors only on divergence', () => {
    const log = { error: vi.fn() };
    const { root } = reparseTree('return Shape.sphere(0.5);\n');
    expect(assertStoreConsistency('return Shape.sphere(0.5);\n', root, log)).toBe(true);
    expect(log.error).not.toHaveBeenCalled();
    expect(
      assertStoreConsistency('return Shape.sphere(0.9);\n', root, log)
    ).toBe(false);
    expect(log.error).toHaveBeenCalledOnce();
  });
});

describe('hashTree', () => {
  it('is stable for equal trees and differs across model changes', () => {
    const a = reparseTree('return Shape.sphere(0.5);\n').root;
    const b = reparseTree('return Shape.sphere(0.5);\n').root;
    const c = reparseTree('return Shape.sphere(0.6);\n').root;
    expect(hashTree(a)).toBe(hashTree(b));
    expect(hashTree(a)).not.toBe(hashTree(c));
  });
});

describe('property-based round-trip', () => {
  it('random mutation sequences: serialize → reparse → serialize is identity', () => {
    for (let seed = 1; seed <= 25; seed += 1) {
      const rand = mulberry32(seed);
      let root = addPrimitiveNode(null, ...randomPrim(rand));
      // Normalize synthetic ids the way the app does: every commit
      // re-evaluates the serialized script.
      root = reparseTree(serializeTree(root)).root;

      for (let step = 0; step < 12; step += 1) {
        root = randomMutation(root, rand);
        const script = serializeTree(root);
        const rp = reparseTree(script);
        expect(rp.leaked, `seed ${seed} step ${step}`).toBe(0);
        // The reparsed store serializes back to the identical script — the
        // two representations describe the same model.
        expect(serializeTree(rp.root), `seed ${seed} step ${step}`).toBe(script);
        expect(hashTree(rp.root)).toBe(hashTree(root));
        root = rp.root;
      }
    }
  });
});

describe('interleaved script and GUI edits', () => {
  // A "user" script edit: valid JS, deliberately non-canonical, sometimes
  // model-changing, sometimes leaving orphan shapes behind.
  function randomScriptEdit(script, rand, step) {
    switch (Math.floor(rand() * 3)) {
      case 0:
        // Hand-written arithmetic and an orphan build the parser can't own.
        return `const orphan${step} = Shape.sphere(0.5 + ${step / 10});\n${script}`;
      case 1:
        // Non-canonical rewrite of the return into a named binding.
        return script.replace(
          /return ([\s\S]+);\n$/,
          `const out${step} = $1;\nreturn out${step};\n`
        );
      default:
        // Model-changing edit: union another primitive onto the result.
        return script.replace(
          /return ([\s\S]+);\n$/,
          `const prev${step} = $1;\nreturn prev${step}.union(Shape.cylinder(0.2, 0.4));\n`
        );
    }
  }

  it('converges with no divergence and no orphaned scene nodes', () => {
    for (let seed = 100; seed < 112; seed += 1) {
      const rand = mulberry32(seed);
      let script = 'return Shape.box3(0.5, 0.4, 0.3);\n';
      let root = reparseTree(script).root;

      for (let step = 0; step < 16; step += 1) {
        if (rand() < 0.5) {
          // GUI lane: mutate the store, regenerate the script view.
          root = randomMutation(root, rand);
          script = serializeTree(root);
        } else {
          // Script lane: the user edits the view; it reparses into the store.
          script = randomScriptEdit(script, rand, step);
        }
        const rp = reparseTree(script);
        expect(rp.leaked, `seed ${seed} step ${step}`).toBe(0);
        root = rp.root;
        // Invariant after every commit: view and store agree.
        const check = checkConsistency(script, root);
        expect(check.ok, `seed ${seed} step ${step}: ${check.error ?? ''}`).toBe(true);
      }

      // Final convergence: one more GUI commit canonicalizes the script and
      // a double round trip is a fixed point.
      script = serializeTree(root);
      const rp = reparseTree(script);
      expect(serializeTree(rp.root)).toBe(script);
    }
  });
});
