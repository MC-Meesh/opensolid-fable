/**
 * Single-flight loader for the wasm-bindgen pkg. This module is the ONE
 * place that owns the WASM lifecycle; React reads it through WasmContext.
 *
 * State machine: idle -> loading -> ready | failed. `retry()` moves
 * failed -> loading again. There is deliberately no way to be "loading"
 * twice: concurrent ensure() calls share one in-flight promise.
 *
 * Failure is a first-class state with a human-actionable message: a missing
 * pkg/ (not yet generated), a fetch that 404s, and an init that hangs (10s
 * timeout) all produce a reason string naming the URL involved, the HTTP
 * status when one exists, and the `npm run wasm` fix.
 */

export const INIT_TIMEOUT_MS = 10_000;

const PKG_HINT =
  'pkg/ is generated build output, not checked in. From web/playground run:\n' +
  '    npm run wasm\n' +
  'then reload (restart the dev server if it was started without pkg/).';

// Built dynamically so Vite's asset pipeline does not inline a second copy
// of the .wasm binary; the glue module already embeds the real reference.
// Correct in dev (where init failures actually happen); in a production
// build the probe still reports *something* useful (usually a 404 pathname).
function wasmBinaryUrl() {
  const rel = ['..', '..', 'pkg', 'opensolid_wasm_bg.wasm'].join('/');
  return new URL(rel, import.meta.url);
}

async function probeWasmBinary(fetchFn) {
  let url;
  try {
    url = wasmBinaryUrl();
  } catch {
    return 'could not determine the .wasm URL';
  }
  try {
    const res = await fetchFn(url, { method: 'HEAD' });
    const type = res.headers?.get?.('content-type') ?? 'unknown';
    // A dev server's SPA fallback answers missing files with HTTP 200
    // text/html, so report the content-type alongside the status.
    const suffix = type.includes('wasm')
      ? ''
      : ` with content-type ${type} — not a .wasm file, so it likely does not exist on the server`;
    return `fetching ${url.pathname} returned HTTP ${res.status}${suffix}`;
  } catch (err) {
    return `fetching ${url.pathname} failed: ${errorMessage(err)}`;
  }
}

function errorMessage(err) {
  return String(err?.message ?? err);
}

function withTimeout(promise, ms) {
  return new Promise((resolve, reject) => {
    const timer = setTimeout(
      () => reject(new Error(`timed out after ${Math.round(ms / 1000)}s`)),
      ms
    );
    promise.then(
      (value) => {
        clearTimeout(timer);
        resolve(value);
      },
      (err) => {
        clearTimeout(timer);
        reject(err);
      }
    );
  });
}

export function createWasmLoader({
  importPkg = () => import('virtual:opensolid-wasm'),
  timeoutMs = INIT_TIMEOUT_MS,
  probe = probeWasmBinary,
  fetchFn = (...args) => fetch(...args),
} = {}) {
  let state = { status: 'idle', error: null, api: null };
  let inflight = null;
  const listeners = new Set();

  function setState(next) {
    state = next;
    for (const listener of listeners) listener(state);
  }

  async function load() {
    let mod;
    try {
      mod = await importPkg();
    } catch (err) {
      throw new Error(
        `Could not load the WASM bindings module (pkg/opensolid_wasm.js): ` +
          `${errorMessage(err)}\n${PKG_HINT}`
      );
    }
    if (mod.__missingPkg) {
      throw new Error(`pkg/opensolid_wasm.js does not exist.\n${PKG_HINT}`);
    }
    try {
      await withTimeout(mod.default(), timeoutMs);
    } catch (err) {
      const diagnostic = await probe(fetchFn);
      throw new Error(
        `WASM module failed to initialize: ${errorMessage(err)} ` +
          `(diagnostic: ${diagnostic}).\n${PKG_HINT}`
      );
    }
    return {
      WasmShape: mod.WasmShape,
      WasmProfile2D: mod.WasmProfile2D,
      WasmPath3D: mod.WasmPath3D,
    };
  }

  function ensure() {
    if (state.status === 'ready') return Promise.resolve(state.api);
    if (!inflight) {
      setState({ status: 'loading', error: null, api: null });
      inflight = load().then(
        (api) => {
          setState({ status: 'ready', error: null, api });
          return api;
        },
        (err) => {
          inflight = null;
          setState({ status: 'failed', error: errorMessage(err), api: null });
          throw err;
        }
      );
    }
    return inflight;
  }

  function retry() {
    if (state.status !== 'failed') return ensure();
    inflight = null;
    return ensure();
  }

  return {
    getState: () => state,
    subscribe: (listener) => {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
    ensure,
    retry,
  };
}

export const wasmLoader = createWasmLoader();
