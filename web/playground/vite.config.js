import { existsSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';
import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

const projectRoot = path.dirname(fileURLToPath(import.meta.url));
const pkgEntry = path.join(projectRoot, 'pkg', 'opensolid_wasm.js');

// The wasm-bindgen output in pkg/ locates its .wasm file with
// `new URL('opensolid_wasm_bg.wasm', import.meta.url)`, which Vite handles
// natively in both dev and build (emitted as an asset). No wasm plugin needed.
//
// pkg/ is generated (`npm run wasm`) and therefore may be absent. The app
// must never import it statically: a missing static import kills the whole
// module graph and the user sees a blank page or an eternal spinner instead
// of an actionable error. Instead, src/wasm/loader.js dynamically imports
// the virtual id below, and this plugin resolves it:
//   - pkg/ present  -> the real wasm-bindgen entry module
//   - pkg/ missing  -> a stub exporting { __missingPkg: true } so the loader
//                      can render a "run npm run wasm" error screen
// Resolution happens once per dev-server run; if you build pkg/ after
// starting the dev server, restart it (the error screen says so too).
const WASM_VIRTUAL_ID = 'virtual:opensolid-wasm';
const WASM_STUB_ID = `\0${WASM_VIRTUAL_ID}:missing`;

function wasmPkgPlugin() {
  return {
    name: 'opensolid-wasm-pkg',
    resolveId(id) {
      if (id !== WASM_VIRTUAL_ID) return null;
      return existsSync(pkgEntry) ? pkgEntry : WASM_STUB_ID;
    },
    load(id) {
      if (id !== WASM_STUB_ID) return null;
      return 'export const __missingPkg = true;\n';
    },
  };
}

export default defineConfig({
  plugins: [react(), wasmPkgPlugin()],
  base: './',
  build: {
    outDir: 'dist',
    target: 'esnext',
  },
});
