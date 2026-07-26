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
  // wasm-pack writes `pkg/.gitignore` containing `*`. That single line is the
  // reason a published tarball used to arrive with no kernel in it: npm has no
  // .npmignore here, so it falls back to the nearest .gitignore — including
  // this nested one — and quietly drops every file `files` asked for under
  // pkg/. Nothing warns; `npm pack` just emits a package where `require`ing
  // the wasm throws MODULE_NOT_FOUND on the user's machine.
  //
  // Delete it at the source. pkg/ stays untracked via the repo-root .gitignore
  // entry, which npm packing does not consult (of-2y4.3).
  rmSync(resolve(serverDir, 'pkg', '.gitignore'), { force: true });
}

process.exit(result.status ?? 1);
