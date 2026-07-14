// Build the opensolid-wasm crate for the Node runtime into ./pkg.
//
// The MCP server runs playground scripts against the exact same kernel the
// browser playground uses, so agent-authored scripts behave identically in
// both. `pkg/` is generated build output (like the playground's), not checked
// in — run this after any Rust change under crates/.

import { spawnSync } from 'node:child_process';
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
process.exit(result.status ?? 1);
