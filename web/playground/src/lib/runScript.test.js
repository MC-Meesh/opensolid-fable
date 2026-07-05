import { describe, expect, it } from 'vitest';
import { runScript } from './runScript.js';

class FakeShape {
  static sphere(r) {
    const shape = new FakeShape();
    shape.kind = 'sphere';
    shape.r = r;
    return shape;
  }
}

describe('runScript', () => {
  it('returns the shape the script returns', () => {
    const shape = runScript('return Shape.sphere(2);', FakeShape);
    expect(shape).toBeInstanceOf(FakeShape);
    expect(shape.r).toBe(2);
  });

  it('exposes only the Shape and Profile bindings, in strict mode', () => {
    expect(() => runScript('undeclared = 1; return Shape.sphere(1);', FakeShape))
      .toThrow(ReferenceError);
  });

  it('binds Profile when a profile class is given', () => {
    class FakeProfile {
      constructor(x, y) {
        this.x = x;
        this.y = y;
      }
    }
    const shape = runScript(
      'const p = new Profile(1, 2); return Shape.sphere(p.x + p.y);',
      FakeShape,
      FakeProfile
    );
    expect(shape.r).toBe(3);
  });

  it('propagates syntax errors', () => {
    expect(() => runScript('return return;', FakeShape)).toThrow(SyntaxError);
  });

  it('propagates runtime errors from the script body', () => {
    expect(() => runScript('throw new Error("boom");', FakeShape)).toThrow('boom');
  });

  it('rejects scripts that do not return a Shape', () => {
    expect(() => runScript('return 42;', FakeShape)).toThrow(/must return a Shape/);
    expect(() => runScript('Shape.sphere(1);', FakeShape)).toThrow(/must return a Shape/);
  });
});
