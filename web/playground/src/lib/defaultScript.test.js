import { describe, expect, it } from 'vitest';
import { DEFAULT_SCRIPT } from './defaultScript.js';
import {
  BINARY_OPS,
  PRIMITIVE_OPS,
  SWEEP_OPS,
  UNARY_OPS,
  runTracedScript,
  scriptHeader,
} from './sceneTree.js';

// Minimal stand-ins: enough for the default script to run.
class FakeShape {
  free() {}
}
for (const op of [...PRIMITIVE_OPS, ...SWEEP_OPS]) {
  FakeShape[op] = () => new FakeShape();
}
for (const op of [...UNARY_OPS, ...BINARY_OPS]) {
  FakeShape.prototype[op] = () => new FakeShape();
}
class FakeProfile {
  lineTo() {}
  arcTo() {}
  close() {}
  free() {}
}

describe('DEFAULT_SCRIPT', () => {
  it('documents the entire scripting API in its header comment', () => {
    const header = scriptHeader(DEFAULT_SCRIPT);
    const api = [
      ...PRIMITIVE_OPS,
      ...UNARY_OPS,
      ...BINARY_OPS,
      ...SWEEP_OPS,
      'Profile',
      'lineTo',
      'arcTo',
      'close',
    ];
    for (const name of api) {
      expect(header, `header must document "${name}"`).toContain(name);
    }
  });

  it('is a header comment followed by runnable sample code', () => {
    const header = scriptHeader(DEFAULT_SCRIPT);
    expect(header.length).toBeGreaterThan(0);
    // GUI regeneration keeps exactly this block, so it must be the API docs.
    expect(header).toContain('Shape.sphere');
    const { root } = runTracedScript(DEFAULT_SCRIPT, FakeShape, FakeProfile);
    expect(root.op).toBe('subtract');
  });
});
