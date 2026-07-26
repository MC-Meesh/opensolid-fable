// Publish gate. Runs on `npm pack` and `npm publish`.
//
// The whole promise of this package is that `npx opensolid-mcp` works with
// nothing but Node >= 18 — no Rust, no wasm-pack, no cargo. That promise lives
// or dies on two files being inside the tarball, and both fail *silently*:
//
//   1. pkg/ missing entirely — publishing from a tree that was never built.
//   2. pkg/.gitignore present — wasm-pack writes it containing `*`, and npm
//      falls back to a package's .gitignore when there is no .npmignore, so it
//      drops pkg/ out of the tarball even though `files` lists it. The package
//      publishes, installs, and then throws MODULE_NOT_FOUND on first use.
//
// Both are unrecoverable once published (npm versions are immutable), so this
// refuses to build the tarball instead. build-wasm.mjs already removes the
// .gitignore; this is the backstop for a stale pkg/ built before that fix.

import { existsSync, rmSync, statSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const serverDir = resolve(here, '..');
const pkgDir = resolve(serverDir, 'pkg');

function fail(message) {
  console.error(`prepack: ${message}\n`);
  process.exit(1);
}

for (const file of ['opensolid_wasm.js', 'opensolid_wasm_bg.wasm']) {
  const path = resolve(pkgDir, file);
  if (!existsSync(path)) {
    fail(
      `refusing to pack without pkg/${file} — the published package would\n` +
        'install fine and then fail at runtime with MODULE_NOT_FOUND.\n\n' +
        'Build the kernel first:\n\n  cd tools/mcp-server && npm run build\n',
    );
  }
  if (statSync(path).size === 0) {
    fail(`pkg/${file} is empty — rebuild with \`npm run build\`.`);
  }
}

// Belt and braces: strip the ignore file even if pkg/ predates build-wasm.mjs
// learning to remove it.
const nestedIgnore = resolve(pkgDir, '.gitignore');
if (existsSync(nestedIgnore)) {
  rmSync(nestedIgnore, { force: true });
  console.error(
    'prepack: removed a stale pkg/.gitignore (it would have excluded the wasm ' +
      'kernel from the tarball).',
  );
}

console.error('prepack: pkg/ contains the prebuilt wasm kernel — ok to pack.');
