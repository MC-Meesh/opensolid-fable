// The preflight that decides whether `npm test` is allowed to start.
//
// It guards a specific and expensive failure: a pkg/ built before the current
// crates/ still loads, so the suite runs to completion against the wrong
// kernel and reports what look like optimizer or topology regressions. That
// happened for real (of-koc8) and cost a false-alarm investigation before a
// rebuild cleared all nine failures.
//
// Every case below drives the real script in a synthetic worktree — mtimes set
// by hand, so freshness is decided by the comparison and not by whatever the
// checkout happened to leave behind.

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { mkdirSync, mkdtempSync, readFileSync, rmSync, utimesSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';
import { dirname, join, resolve } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const serverDir = resolve(here, '..');
const script = readFileSync(resolve(serverDir, 'scripts', 'check-pkg.mjs'), 'utf8');

const SECOND = 1000;
const BASE = Date.UTC(2026, 6, 26, 12, 0, 0) / SECOND; // seconds, as utimesSync wants

function touch(path, contents, atSeconds) {
  mkdirSync(dirname(path), { recursive: true });
  writeFileSync(path, contents);
  utimesSync(path, atSeconds, atSeconds);
}

// A miniature worktree: the script resolves crates/ and Cargo.lock as
// ../../crates and ../../Cargo.lock relative to the server directory, so the
// nesting has to match the real one.
function worktree({ builtAt, sourceAt, withCrates = true, extra = {} }) {
  const root = mkdtempSync(join(tmpdir(), 'opensolid-checkpkg-'));
  const server = join(root, 'tools', 'mcp-server');

  touch(join(server, 'scripts', 'check-pkg.mjs'), script, BASE);
  if (builtAt !== null) {
    touch(join(server, 'pkg', 'opensolid_wasm.js'), 'module.exports = {};\n', builtAt);
    touch(join(server, 'pkg', 'opensolid_wasm_bg.wasm'), '\0asm', builtAt);
  }
  if (withCrates) {
    touch(join(root, 'crates', 'opensolid-kernel', 'src', 'lib.rs'), 'pub fn k() {}\n', sourceAt);
    touch(join(root, 'Cargo.toml'), '[workspace]\n', sourceAt);
    touch(join(root, 'Cargo.lock'), 'version = 4\n', sourceAt);
  }
  for (const [relPath, at] of Object.entries(extra)) {
    touch(join(root, relPath), 'x\n', at);
  }

  return { root, script: join(server, 'scripts', 'check-pkg.mjs') };
}

// spawnSync rather than execFileSync: the passing cases have stderr worth
// asserting on too (the escape hatch must announce itself), and execFileSync
// only hands back stderr when the command fails.
function run(scriptPath, env = {}) {
  const result = spawnSync(process.execPath, [scriptPath], {
    encoding: 'utf8',
    env: { ...process.env, ...env },
  });
  assert.equal(result.error, undefined, `could not run check-pkg: ${result.error}`);
  return { status: result.status, stderr: result.stderr, stdout: result.stdout };
}

describe('check-pkg freshness gate', () => {
  test('a pkg/ newer than crates/ passes', () => {
    const { root, script: path } = worktree({ builtAt: BASE + 60, sourceAt: BASE });
    try {
      const { status, stderr } = run(path);
      assert.equal(status, 0, `expected a pass, got ${status}: ${stderr}`);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  // The of-koc8 case: pkg/ from 2026-07-16, crates/ at 2026-07-26 HEAD.
  test('a pkg/ older than crates/ fails and names both sides', () => {
    const tenDays = 10 * 24 * 60 * 60;
    const { root, script: path } = worktree({ builtAt: BASE - tenDays, sourceAt: BASE });
    try {
      const { status, stderr } = run(path);
      assert.equal(status, 1, 'a stale pkg/ must not be allowed to run the suite');
      assert.match(stderr, /pkg\/ is older than crates\//);
      assert.match(stderr, /npm run build/);
      // Naming the offending file is the whole point — "something is stale" is
      // what sent the last investigation looking at the optimizer.
      assert.match(stderr, /crates\/opensolid-kernel\/src\/lib\.rs/);
      assert.match(stderr, /opensolid_wasm_bg\.wasm/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('a Cargo.lock newer than pkg/ fails on its own', () => {
    const { root, script: path } = worktree({ builtAt: BASE, sourceAt: BASE - 60 });
    try {
      utimesSync(join(root, 'Cargo.lock'), BASE + 60, BASE + 60);
      const { status, stderr } = run(path);
      assert.equal(status, 1, 'a dependency bump changes the kernel too');
      assert.match(stderr, /Cargo\.lock/);
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
      assert.equal(status, 0, `target/ churn must not demand a rebuild: ${stderr}`);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('an equally old pkg/ passes — only strictly newer source is stale', () => {
    const { root, script: path } = worktree({ builtAt: BASE, sourceAt: BASE });
    try {
      const { status, stderr } = run(path);
      assert.equal(status, 0, `same-mtime must not flap: ${stderr}`);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('OPENSOLID_ALLOW_STALE_PKG=1 continues, but says so', () => {
    const { root, script: path } = worktree({ builtAt: BASE - 60, sourceAt: BASE });
    try {
      const { status, stderr } = run(path, { OPENSOLID_ALLOW_STALE_PKG: '1' });
      assert.equal(status, 0);
      assert.match(stderr, /continuing anyway/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  test('a missing wasm still fails, with the build instruction', () => {
    const { root, script: path } = worktree({ builtAt: null, sourceAt: BASE });
    try {
      const { status, stderr } = run(path);
      assert.equal(status, 1);
      assert.match(stderr, /wasm kernel is not built/);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });

  // Outside a worktree — an unpacked npm tarball, or the temp-dir copy in
  // package.test.js — there is no crates/ to compare against, and demanding a
  // rebuild there would be nonsense.
  test('with no crates/ to compare against, existence is enough', () => {
    const { root, script: path } = worktree({
      builtAt: BASE,
      sourceAt: BASE,
      withCrates: false,
    });
    try {
      const { status, stderr } = run(path);
      assert.equal(status, 0, `a crates-less checkout must not fail: ${stderr}`);
    } finally {
      rmSync(root, { recursive: true, force: true });
    }
  });
});
