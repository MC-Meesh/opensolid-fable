import { defineConfig } from 'vite';
import react from '@vitejs/plugin-react';

// The wasm-bindgen output in pkg/ locates its .wasm file with
// `new URL('opensolid_wasm_bg.wasm', import.meta.url)`, which Vite handles
// natively in both dev and build (emitted as an asset). No wasm plugin needed.
export default defineConfig({
  plugins: [react()],
  base: './',
  build: {
    outDir: 'dist',
    target: 'esnext',
  },
});
