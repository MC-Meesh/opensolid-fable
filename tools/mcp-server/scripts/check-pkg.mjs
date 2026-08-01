// Fail fast with a clear message if the generated wasm package is missing —
// or if it is older than the kernel source it was built from.
//
// `pkg/` is build output (not checked in); `npm run build` regenerates it.
//
// A stale pkg/ is worse than a missing one. The suite runs against a kernel
// that no longer matches HEAD, and the failures read as kernel regressions: a
// pkg/ ten days behind crates/ produced nine `optimize could not build the
// model at any sampled parameter corner` failures in test/optimize.test.js
// that `npm run build` cleared outright (of-koc8). Comparing mtimes catches
// that before a single test runs.

import { existsSync, readdirSync, statSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { basename, dirname, join, relative, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const serverDir = resolve(here, '..');
const pkgDir = resolve(serverDir, 'pkg');
const entry = resolve(pkgDir, 'opensolid_wasm.js');
const wasm = resolve(pkgDir, 'opensolid_wasm_bg.wasm');

if (!existsSync(entry) || !existsSync(wasm)) {
  console.error(
    'Missing tools/mcp-server/pkg — the wasm kernel is not built.\n' +
      'Build it first:\n\n  cd tools/mcp-server && npm run build\n',
  );
  process.exit(1);
}

const repoRoot = resolve(serverDir, '..', '..');
const crates = join(repoRoot, 'crates');

// A copy of this script running outside the worktree — the packed tarball, or
// the temp-dir regression test in test/package.test.js — has no crates/ to
// compare against. Existence is all that can be checked there.
if (!existsSync(crates)) {
  process.exit(0);
}

// Everything wasm-pack compiles from. Files only: a directory's own mtime
// moves whenever an editor drops a swapfile beside real source, and a real
// source edit moves that file's own mtime anyway.
function newestFile(root) {
  let newest = null;
  const stack = [root];
  while (stack.length > 0) {
    const current = stack.pop();
    let info;
    try {
      info = statSync(current);
    } catch {
      continue; // vanished mid-walk, or a broken symlink
    }
    if (info.isDirectory()) {
      // `target/` is build output — always newer than pkg/, never a reason to
      // rebuild. `.git` churns on every command and holds no kernel source.
      const name = basename(current);
      if (name === 'target' || name === '.git') continue;
      for (const child of readdirSync(current)) stack.push(join(current, child));
      continue;
    }
    if (newest === null || info.mtimeMs > newest.mtimeMs) {
      newest = { path: current, mtimeMs: info.mtimeMs };
    }
  }
  return newest;
}

const newest = [
  newestFile(crates),
  newestFile(join(repoRoot, 'Cargo.toml')),
  newestFile(join(repoRoot, 'Cargo.lock')),
].reduce((a, b) => (b !== null && (a === null || b.mtimeMs > a.mtimeMs) ? b : a), null);

const built = statSync(wasm).mtimeMs;

if (newest !== null && newest.mtimeMs > built) {
  const stamp = (ms) => new Date(ms).toISOString();
  const message =
    'pkg/ is older than crates/ — run `npm run build`.\n\n' +
    `  built   ${stamp(built)}  ${relative(repoRoot, wasm)}\n` +
    `  source  ${stamp(newest.mtimeMs)}  ${relative(repoRoot, newest.path)}\n\n` +
    'The suite would run against a kernel that no longer matches the source,\n' +
    'and the failures would read as kernel regressions rather than a stale\n' +
    'build. Rebuild first:\n\n  cd tools/mcp-server && npm run build\n';

  // Escape hatch for a source change that provably cannot affect the wasm (a
  // docs-only edit under crates/, say). It warns rather than passing silently,
  // so a run that used it says so in its own log.
  if (process.env.OPENSOLID_ALLOW_STALE_PKG === '1') {
    console.error(`${message}\nOPENSOLID_ALLOW_STALE_PKG=1 — continuing anyway.\n`);
    process.exit(0);
  }

  console.error(message);
  process.exit(1);
}
