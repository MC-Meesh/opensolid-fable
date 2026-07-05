import { describe, expect, it, vi } from 'vitest';
import { createWasmLoader } from './loader.js';

function okPkg() {
  return {
    default: vi.fn(async () => {}),
    WasmShape: class {},
    WasmProfile2D: class {},
  };
}

describe('createWasmLoader', () => {
  it('starts idle, transitions loading -> ready, and exposes the API classes', async () => {
    const pkg = okPkg();
    const loader = createWasmLoader({ importPkg: async () => pkg });
    expect(loader.getState().status).toBe('idle');

    const seen = [];
    loader.subscribe((s) => seen.push(s.status));
    const api = await loader.ensure();

    expect(seen).toEqual(['loading', 'ready']);
    expect(loader.getState().status).toBe('ready');
    expect(api.WasmShape).toBe(pkg.WasmShape);
    expect(api.WasmProfile2D).toBe(pkg.WasmProfile2D);
  });

  it('is single-flight: concurrent ensure() shares one import + init', async () => {
    const pkg = okPkg();
    const importPkg = vi.fn(async () => pkg);
    const loader = createWasmLoader({ importPkg });

    const [a, b, c] = await Promise.all([loader.ensure(), loader.ensure(), loader.ensure()]);
    await loader.ensure();

    expect(importPkg).toHaveBeenCalledTimes(1);
    expect(pkg.default).toHaveBeenCalledTimes(1);
    expect(a).toBe(b);
    expect(b).toBe(c);
  });

  it('fails with npm run wasm instructions when pkg/ is missing (stub module)', async () => {
    const loader = createWasmLoader({ importPkg: async () => ({ __missingPkg: true }) });
    await expect(loader.ensure()).rejects.toThrow(/npm run wasm/);
    const state = loader.getState();
    expect(state.status).toBe('failed');
    expect(state.error).toContain('pkg/opensolid_wasm.js does not exist');
    expect(state.error).toContain('npm run wasm');
  });

  it('fails with instructions when the bindings module cannot be imported', async () => {
    const loader = createWasmLoader({
      importPkg: async () => {
        throw new Error('404 Not Found');
      },
    });
    await expect(loader.ensure()).rejects.toThrow(/npm run wasm/);
    expect(loader.getState().error).toContain('404 Not Found');
  });

  it('times out a hanging init and reports the probe diagnostic', async () => {
    const loader = createWasmLoader({
      importPkg: async () => ({ default: () => new Promise(() => {}) }),
      timeoutMs: 20,
      probe: async () => 'fetching /pkg/opensolid_wasm_bg.wasm returned HTTP 404',
    });
    await expect(loader.ensure()).rejects.toThrow(/timed out/);
    const state = loader.getState();
    expect(state.status).toBe('failed');
    expect(state.error).toContain('HTTP 404');
    expect(state.error).toContain('npm run wasm');
  });

  it('reports init rejections with the probe diagnostic', async () => {
    const loader = createWasmLoader({
      importPkg: async () => ({
        default: async () => {
          throw new Error('CompileError: bad magic number');
        },
      }),
      probe: async () => 'fetching /pkg/opensolid_wasm_bg.wasm returned HTTP 200',
    });
    await expect(loader.ensure()).rejects.toThrow(/bad magic number/);
    expect(loader.getState().error).toContain('HTTP 200');
  });

  it('retry() after a failure re-attempts and can succeed', async () => {
    let attempts = 0;
    const pkg = okPkg();
    const loader = createWasmLoader({
      importPkg: async () => {
        attempts += 1;
        if (attempts === 1) return { __missingPkg: true };
        return pkg;
      },
    });

    await expect(loader.ensure()).rejects.toThrow();
    expect(loader.getState().status).toBe('failed');

    const api = await loader.retry();
    expect(loader.getState().status).toBe('ready');
    expect(api.WasmShape).toBe(pkg.WasmShape);
    expect(attempts).toBe(2);
  });

  it('retry() while ready is a no-op returning the same API', async () => {
    const pkg = okPkg();
    const importPkg = vi.fn(async () => pkg);
    const loader = createWasmLoader({ importPkg });
    const first = await loader.ensure();
    const second = await loader.retry();
    expect(second).toBe(first);
    expect(importPkg).toHaveBeenCalledTimes(1);
  });
});
