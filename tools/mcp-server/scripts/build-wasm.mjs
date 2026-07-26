// Build the opensolid-wasm crate for the Node runtime into ./pkg.
//
// The MCP server runs playground scripts against the exact same kernel the
// browser playground uses, so agent-authored scripts behave identically in
// both. `pkg/` is generated build output (like the playground's), not checked
// in — run this after any Rust change under crates/.

import { spawnSync } from 'node:child_process';
import { rmSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const serverDir = resolve(here, '..');
const crate = resolve(serverDir, '../../crates/opensolid-wasm');

const args = [
  'build',
  crate,
  '--target',
  'nodejs',
  '--no-typescript',
  '--out-dir',
  resolve(serverDir, 'pkg'),
];

console.error(`wasm-pack ${args.join(' ')}`);
const result = spawnSync('wasm-pack', args, { stdio: 'inherit' });

if (result.error) {
  console.error(
    'Failed to run wasm-pack. Install it with `cargo install wasm-pack` and ' +
      'ensure the wasm target is present: `rustup target add wasm32-unknown-unknown`.',
  );
  process.exit(1);
}

if (result.status === 0) {
  // wasm-pack writes `pkg/.gitignore` containing `*`, and npm consults a
  // package's own .gitignore when there is no .npmignore. package.json lists
  // the two wasm files by exact path, which beats that rule — but the
  // directory form (`files: ["pkg/"]`) does not: measured on npm 11.6.4, the
  // pair emits a tarball with zero pkg files. It installs clean and then
  // throws MODULE_NOT_FOUND on the user's first call, with no warning at pack
  // time. Since that is one innocuous edit away, delete the trap at the source
  // rather than rely on the `files` form staying exactly as it is.
  //
  // pkg/ stays untracked via the repo-root .gitignore entry, which npm packing
  // does not consult (of-2y4.3).
  rmSync(resolve(serverDir, 'pkg', '.gitignore'), { force: true });
}

process.exit(result.status ?? 1);
