// Preflight check that the generated wasm-bindgen output exists — and is not
// older than the kernel source it was built from.
//
//   node scripts/check-pkg.mjs          exit 1 if pkg/ is missing or stale
//                                       (prebuild)
//   node scripts/check-pkg.mjs --warn   warn but continue (predev — the dev
//                                       server still starts and the browser
//                                       shows an actionable error screen)
//
// A stale pkg/ is worse than a missing one: a missing pkg/ renders an error
// screen, but a stale one runs a kernel of any age and the browser shows a
// visibly wrong render or a missing API with no hint why (of-vw5e, same blind
// spot the mcp server's suite had in of-koc8). Comparing mtimes catches that
// before vite starts.
import { existsSync, readdirSync, statSync } from 'node:fs';
import path from 'node:path';
import { fileURLToPath } from 'node:url';

const projectRoot = path.dirname(path.dirname(fileURLToPath(import.meta.url)));
const pkgDir = path.join(projectRoot, 'pkg');
const pkgEntry = path.join(pkgDir, 'opensolid_wasm.js');
const wasm = path.join(pkgDir, 'opensolid_wasm_bg.wasm');
const warnOnly = process.argv.includes('--warn');

const rule = '='.repeat(72);

if (!existsSync(pkgEntry) || !existsSync(wasm)) {
  console.error(`
${rule}
  Missing generated WASM package: ${path.relative(process.cwd(), pkgEntry)}

  pkg/ is build output, not checked in. Generate it with:

      npm run wasm

  (requires the wasm32-unknown-unknown target and wasm-pack — see README.md)
${rule}
`);
  if (warnOnly) {
    console.error('Continuing anyway; the app will show an error screen until pkg/ exists.\n');
    process.exit(0);
  }
  process.exit(1);
}

const repoRoot = path.resolve(projectRoot, '..', '..');
const crates = path.join(repoRoot, 'crates');

// A copy of this project outside the worktree has no crates/ to compare
// against. Existence is all that can be checked there.
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
      const name = path.basename(current);
      if (name === 'target' || name === '.git') continue;
      for (const child of readdirSync(current)) stack.push(path.join(current, child));
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
  newestFile(path.join(repoRoot, 'Cargo.toml')),
  newestFile(path.join(repoRoot, 'Cargo.lock')),
].reduce((a, b) => (b !== null && (a === null || b.mtimeMs > a.mtimeMs) ? b : a), null);

const built = statSync(wasm).mtimeMs;

if (newest !== null && newest.mtimeMs > built) {
  const stamp = (ms) => new Date(ms).toISOString();
  console.error(`
${rule}
  Stale WASM package: pkg/ is older than crates/ — run \`npm run wasm\`.

    built   ${stamp(built)}  ${path.relative(repoRoot, wasm)}
    source  ${stamp(newest.mtimeMs)}  ${path.relative(repoRoot, newest.path)}

  The app would run a kernel that no longer matches the source: a visibly
  wrong render, or an API missing in the browser. Rebuild with:

      npm run wasm
${rule}
`);

  if (warnOnly) {
    console.error('Continuing anyway; the app will run the stale kernel until pkg/ is rebuilt.\n');
    process.exit(0);
  }

  // Escape hatch for a source change that provably cannot affect the wasm (a
  // docs-only edit under crates/, say). It warns rather than passing silently,
  // so a run that used it says so in its own log.
  if (process.env.OPENSOLID_ALLOW_STALE_PKG === '1') {
    console.error('OPENSOLID_ALLOW_STALE_PKG=1 — continuing anyway.\n');
    process.exit(0);
  }

  process.exit(1);
}
