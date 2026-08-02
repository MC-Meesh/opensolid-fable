import { describe, it, expect } from 'vitest';
import { wasmErrorMessage } from './wasmError.js';

describe('wasmErrorMessage', () => {
  it('unwraps a structured kernel error to its message', () => {
    const thrown =
      '{"code":"invalid_argument","category":"argument",' +
      '"message":"invalid argument `radius`: must be positive"}';
    expect(wasmErrorMessage(thrown)).toBe('invalid argument `radius`: must be positive');
  });

  it('appends the hint when the kernel provides one', () => {
    const thrown =
      '{"code":"non_manifold_mesh","category":"geometry",' +
      '"message":"non-manifold mesh in sdf_to_brep: 1 pinched edge(s)",' +
      '"hint":"Nudge the feature size instead."}';
    expect(wasmErrorMessage(thrown)).toBe(
      'non-manifold mesh in sdf_to_brep: 1 pinched edge(s)\n\nHint: Nudge the feature size instead.',
    );
  });

  it('passes plain thrown strings through untouched', () => {
    expect(wasmErrorMessage('something broke')).toBe('something broke');
  });

  it('reads .message from real Error objects', () => {
    expect(wasmErrorMessage(new TypeError('x is not a function'))).toBe('x is not a function');
  });

  it('shows near-JSON that fails to parse as-is', () => {
    const mangled = '{"code":"cut off';
    expect(wasmErrorMessage(mangled)).toBe(mangled);
  });
});
