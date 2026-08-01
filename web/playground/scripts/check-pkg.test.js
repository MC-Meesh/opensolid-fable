// The preflight that decides whether `npm run build` (and, in warn mode,
// `npm run dev`) starts against a real, current wasm kernel.
//
// It guards the playground's version of the stale-pkg blind spot (of-vw5e,
// same shape as the mcp server's of-koc8): a pkg/ built before the current
// crates/ still loads, so the browser runs a kernel of any age and shows a
// visibly wrong render or a missing API with no hint why.
//
// Every case below drives the real script in a synthetic worktree — mtimes set
// by hand, so freshness is decided by the comparison and not by whatever the
// checkout happened to leave behind.

import { describe, expect, test } from 'vitest';
import { spawnSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, rmSync, utimesSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';
import { dirname, join, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const script = readFileSync(resolve(here, 'check-pkg.mjs'), 'utf8');

const SECOND = 1000;
const BASE = Date.UTC(2026, 7, 1, 12, 0, 0) / SECOND; // seconds, as utimesSync wants

function touch(path, contents, atSeconds) {
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, contents);
  utimesSync(path, atSeconds, atSeconds);
}

// A miniature worktree: the script resolves crates/ and Cargo.lock as
// ../../crates and ../../Cargo.lock relative to the project directory, so the
// nesting has to match the real one (web/playground under the repo root).
function worktree({ builtAt, sourceAt, withCrates = true, extra = {} }) {
  const root = mkdtempSync(join(tmpdir(), 'opensolid-checkpkg-'));
  const project = join(root, 'web', 'playground');

  touch(join(project, 'scripts', 'check-pkg.mjs'), script, BASE);
  if (builtAt !== null) {
    touch(join(project, 'pkg', 'opensolid_wasm.js'), 'export default {};\n', builtAt);
    touch(join(project, 'pkg', 'opensolid_wasm_bg.wasm'), '\0asm', builtAt);
  }
  if (withCrates) {
    touch(join(root, 'crates', 'opensolid-kernel', 'src', 'lib.rs'), 'pub fn k() {}\n', sourceAt);
    touch(join(root, 'Cargo.toml'), '[workspace]\n', sourceAt);
    touch(join(root, 'Cargo.lock'), 'version = 4\n', sourceAt);
  }
  for (const [relPath, at] of Object.entries(extra)) {
    touch(join(root, relPath), 'x\n', at);
  }

  return { root, script: join(project, 'scripts', 'check-pkg.mjs') };
}

// spawnSync rather than execFileSync: the passing cases have stderr worth
// asserting on too (warn mode and the escape hatch must announce themselves),
// and execFileSync only hands back stderr when the command fails.
function run(scriptPath, { args = [], env = {} } = {}) {
  const result = spawnSync(process.execPath, [scriptPath, ...args], {
    encoding: 'utf8',
    env: { ...process.env, ...env },
  });
  expect(result.error).toBeUndefined();
  return { status: result.status, stderr: result.stderr, stdout: result.stdout };
}

describe('check-pkg freshness gate', () => {
  test('a pkg/ newer than crates/ passes', () => {
    const { root, script: path } = worktree({ builtAt: BASE + 60, sourceAt: BASE });
    try {
      const { status, stderr } = run(path);
      expect(status, `expected a pass: ${stderr}`).toBe(0);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('a pkg/ older than crates/ fails and names both sides', () => {
    const tenDays = 10 * 24 * 60 * 60;
    const { root, script: path } = worktree({ builtAt: BASE - tenDays, sourceAt: BASE });
    try {
      const { status, stderr } = run(path);
      expect(status, 'a stale pkg/ must not be allowed to build').toBe(1);
      expect(stderr).toMatch(/pkg\/ is older than crates\//);
      expect(stderr).toMatch(/npm run wasm/);
      // Naming the offending file is the whole point — "something is stale"
      // leaves the user staring at a wrong render with no lead to follow.
      expect(stderr).toMatch(/crates\/opensolid-kernel\/src\/lib\.rs/);
      expect(stderr).toMatch(/opensolid_wasm_bg\.wasm/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('--warn continues on a stale pkg/, but says so', () => {
    const { root, script: path } = worktree({ builtAt: BASE - 60, sourceAt: BASE });
    try {
      const { status, stderr } = run(path, { args: ['--warn'] });
      expect(status, `predev must still start the dev server: ${stderr}`).toBe(0);
      expect(stderr).toMatch(/pkg\/ is older than crates\//);
      expect(stderr).toMatch(/Continuing anyway/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('a Cargo.lock newer than pkg/ fails on its own', () => {
    const { root, script: path } = worktree({ builtAt: BASE, sourceAt: BASE - 60 });
    try {
      utimesSync(join(root, 'Cargo.lock'), BASE + 60, BASE + 60);
      const { status, stderr } = run(path);
      expect(status, 'a dependency bump changes the kernel too').toBe(1);
      expect(stderr).toMatch(/Cargo\.lock/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('build output under target/ is not mistaken for source', () => {
    const { root, script: path } = worktree({
      builtAt: BASE,
      sourceAt: BASE - 60,
      // cargo writes these on every build, always after pkg/ was produced.
      extra: { 'crates/opensolid-kernel/target/debug/libkernel.rlib': BASE + 600 },
    });
    try {
      const { status, stderr } = run(path);
      expect(status, `target/ churn must not demand a rebuild: ${stderr}`).toBe(0);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('an equally old pkg/ passes — only strictly newer source is stale', () => {
    const { root, script: path } = worktree({ builtAt: BASE, sourceAt: BASE });
    try {
      const { status, stderr } = run(path);
      expect(status, `same-mtime must not flap: ${stderr}`).toBe(0);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('OPENSOLID_ALLOW_STALE_PKG=1 continues, but says so', () => {
    const { root, script: path } = worktree({ builtAt: BASE - 60, sourceAt: BASE });
    try {
      const { status, stderr } = run(path, { env: { OPENSOLID_ALLOW_STALE_PKG: '1' } });
      expect(status).toBe(0);
      expect(stderr).toMatch(/continuing anyway/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('a missing pkg/ still fails, with the build instruction', () => {
    const { root, script: path } = worktree({ builtAt: null, sourceAt: BASE });
    try {
      const { status, stderr } = run(path);
      expect(status).toBe(1);
      expect(stderr).toMatch(/Missing generated WASM package/);
      expect(stderr).toMatch(/npm run wasm/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('--warn continues on a missing pkg/ — the dev server shows the error screen', () => {
    const { root, script: path } = worktree({ builtAt: null, sourceAt: BASE });
    try {
      const { status, stderr } = run(path, { args: ['--warn'] });
      expect(status, `predev must still start the dev server: ${stderr}`).toBe(0);
      expect(stderr).toMatch(/Missing generated WASM package/);
      expect(stderr).toMatch(/Continuing anyway/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  // Outside a worktree — an exported copy of web/playground — there is no
  // crates/ to compare against, and demanding a rebuild there would be
  // nonsense.
  test('with no crates/ to compare against, existence is enough', () => {
    const { root, script: path } = worktree({
      builtAt: BASE,
      sourceAt: BASE,
      withCrates: false,
    });
    try {
      const { status, stderr } = run(path);
      expect(status, `a crates-less checkout must not fail: ${stderr}`).toBe(0);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });
});
