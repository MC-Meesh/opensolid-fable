// Distribution gate (of-2y4.3): does the *tarball* work?
//
// Every other test in this directory runs against the worktree, where pkg/,
// test/, and examples/ all exist. A consumer gets none of that — they get
// whatever `npm pack` decided to include, unpacked into node_modules. The
// failure this guards is specific and silent: a tarball that installs cleanly
// and then throws MODULE_NOT_FOUND the first time an agent calls a tool,
// because the prebuilt wasm kernel was never in it. npm emits no warning for
// that, and a published version cannot be amended.
//
// So these tests pack for real, unpack into a temp dir, and drive the unpacked
// copy over stdio — the same path `npx opensolid-mcp` takes.

import { test, describe, before, after } from 'node:test';
import assert from 'node:assert/strict';
import { execFileSync, spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { mkdtempSync, rmSync, readFileSync, existsSync, writeFileSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';
import { dirname, resolve, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const serverDir = resolve(here, '..');
const manifest = JSON.parse(readFileSync(resolve(serverDir, 'package.json'), 'utf8'));

// npm pack + a release-profile wasm load are slow; one shared unpack for all of it.
const PACK_TIMEOUT_MS = 180_000;

describe('package manifest', () => {
  // `npx opensolid-mcp` resolves a *package* named opensolid-mcp. If the
  // package were still named opensolid-mcp-server, the documented one-liner
  // would install someone else's package or nothing at all.
  test('is named for the command it is invoked as', () => {
    assert.equal(manifest.name, 'opensolid-mcp');
    assert.ok(manifest.bin['opensolid-mcp'], 'bin must expose an opensolid-mcp command');
  });

  // `private: true` makes `npm publish` refuse outright. It was the original
  // reason nothing here was installable.
  test('is publishable', () => {
    assert.notEqual(manifest.private, true);
    assert.ok(manifest.license, 'npm warns and consumers cannot vet an unlicensed package');
    assert.ok(manifest.repository, 'provenance for a package that ships a binary blob');
  });

  test('declares the Node floor the README promises', () => {
    assert.equal(manifest.engines.node, '>=18');
  });

  test('the bin entry exists and is executable as a script', () => {
    const bin = resolve(serverDir, manifest.bin['opensolid-mcp']);
    assert.ok(existsSync(bin), `${bin} is missing`);
    assert.match(
      readFileSync(bin, 'utf8').split('\n')[0],
      /^#!.*\bnode\b/,
      'a bin without a node shebang is not runnable via npx on POSIX',
    );
  });
});

describe('tarball contents', () => {
  // The wasm kernel is the entire reason this package can be installed without
  // a Rust toolchain. Assert on the real pack output, not on the `files` field:
  // the ways pkg/ goes missing (a nested .gitignore, an unbuilt tree) all leave
  // `files` looking perfectly correct.
  let packed;

  before(() => {
    const json = execFileSync('npm', ['pack', '--dry-run', '--json'], {
      cwd: serverDir,
      encoding: 'utf8',
      stdio: ['ignore', 'pipe', 'ignore'],
      timeout: PACK_TIMEOUT_MS,
    });
    packed = JSON.parse(json)[0];
  });

  test('ships the prebuilt wasm kernel', () => {
    const paths = packed.files.map((f) => f.path);
    assert.ok(
      paths.includes('pkg/opensolid_wasm_bg.wasm'),
      `tarball has no wasm binary — \`npx opensolid-mcp\` would need a Rust toolchain. Got: ${paths.join(', ')}`,
    );
    assert.ok(paths.includes('pkg/opensolid_wasm.js'), 'tarball has no wasm-bindgen glue');
    const wasm = packed.files.find((f) => f.path === 'pkg/opensolid_wasm_bg.wasm');
    assert.ok(wasm.size > 100_000, `wasm binary is implausibly small (${wasm.size} B)`);
  });

  // wasm-pack's `nodejs` target emits CommonJS (`exports.MeshData = ...`), but
  // this package is `"type": "module"`, which would make Node parse that .js as
  // ESM and throw `exports is not defined in ES module scope` on the very first
  // require. The nested pkg/package.json — which declares no `type` — is what
  // scopes pkg/ back to CommonJS. It is one line in `files` and the entire
  // package is inert without it, so it gets its own assertion.
  test('ships pkg/package.json, which scopes the wasm glue back to CommonJS', () => {
    const paths = packed.files.map((f) => f.path);
    assert.ok(
      paths.includes('pkg/package.json'),
      'without it the CommonJS wasm glue is parsed as ESM and every tool call throws',
    );
  });

  test('ships every module the server imports at runtime', () => {
    const paths = packed.files.map((f) => f.path);
    for (const mod of ['src/server.js', 'src/tools.js', 'src/kernel.js', 'src/mesh.js', 'src/render.js', 'src/png.js', 'src/optimize.js']) {
      assert.ok(paths.includes(mod), `${mod} missing from the tarball`);
    }
  });

  // The gallery's renders and exports are ~100 MB of regenerable build output
  // and the tests are not a consumer's business. Shipping them would make an
  // `npx` cold start download two orders of magnitude more than it needs to.
  test('does not ship tests, examples, or build outputs', () => {
    const strays = packed.files
      .map((f) => f.path)
      .filter((p) => p.startsWith('test/') || p.startsWith('examples/') || p.startsWith('scripts/'));
    assert.deepEqual(strays, [], `tarball carries development-only files: ${strays.join(', ')}`);
    assert.ok(
      packed.size < 5_000_000,
      `tarball is ${(packed.size / 1e6).toFixed(1)} MB — an npx cold start pays this every time`,
    );
  });
});

describe('the unpacked package runs', () => {
  let dir;
  let unpacked;

  before(() => {
    dir = mkdtempSync(join(tmpdir(), 'opensolid-pack-'));
    const json = execFileSync('npm', ['pack', '--json', '--pack-destination', dir], {
      cwd: serverDir,
      encoding: 'utf8',
      stdio: ['ignore', 'pipe', 'ignore'],
      timeout: PACK_TIMEOUT_MS,
    });
    const tarball = join(dir, JSON.parse(json)[0].filename);
    execFileSync('tar', ['-xzf', tarball, '-C', dir], { timeout: PACK_TIMEOUT_MS });
    unpacked = join(dir, 'package'); // npm tarballs root everything under package/
  });

  after(() => {
    if (dir) rmSync(dir, { recursive: true, force: true });
  });

  // The end-to-end claim: nothing but Node and the tarball. No cargo, no
  // wasm-pack, no worktree — the server boots, loads the wasm kernel, and
  // builds a solid whose volume is right.
  test('answers MCP over stdio and meshes a real solid', { timeout: PACK_TIMEOUT_MS }, async () => {
    const outputDir = join(dir, 'out');
    const child = spawn(process.execPath, [join(unpacked, 'src', 'server.js')], {
      cwd: dir,
      env: { ...process.env, OPENSOLID_MCP_OUTPUT_DIR: outputDir },
      stdio: ['pipe', 'pipe', 'pipe'],
    });

    const pending = new Map();
    const rl = createInterface({ input: child.stdout });
    rl.on('line', (line) => {
      if (!line.trim()) return;
      const msg = JSON.parse(line);
      const resolveFn = pending.get(msg.id);
      if (resolveFn) {
        pending.delete(msg.id);
        resolveFn(msg);
      }
    });

    let nextId = 1;
    const rpc = (method, params) =>
      new Promise((res, rej) => {
        const id = nextId++;
        pending.set(id, res);
        child.on('error', rej);
        child.stdin.write(JSON.stringify({ jsonrpc: '2.0', id, method, params }) + '\n');
      });

    try {
      const init = await rpc('initialize', { protocolVersion: '2024-11-05' });
      assert.equal(init.result.serverInfo.name, 'opensolid-mcp-server');

      const list = await rpc('tools/list', {});
      const names = list.result.tools.map((t) => t.name);
      for (const expected of ['create_model', 'measure', 'validate', 'export', 'get_screenshot']) {
        assert.ok(names.includes(expected), `tools/list omits ${expected}`);
      }

      // A 20x20x20 box: exercises the wasm load, the mesher, and mass
      // properties. 8000 mm^3 exactly; the SDF mesher reads a hair under, so
      // allow 1% — wide enough for the meshing bias, far tighter than the
      // "the kernel did not load at all" failure this is really watching for.
      const called = await rpc('tools/call', {
        name: 'create_model',
        arguments: { script: 'return Shape.box3(10, 10, 10);', name: 'pack-smoke' },
      });
      assert.ok(!called.result.isError, `create_model failed: ${JSON.stringify(called.result)}`);
      const payload = JSON.parse(called.result.content[0].text);
      assert.equal(payload.valid, true);
      assert.ok(
        Math.abs(payload.volume - 8000) / 8000 < 0.01,
        `volume ${payload.volume} is not a 20 mm cube`,
      );
    } finally {
      child.stdin.end();
      child.kill();
    }
  });

  // Regression guard for the exact silent failure described at the top of this
  // file: strip the wasm out of an unpacked copy and the server must die with a
  // message naming what is missing, not with a bare stack trace.
  test('a wasm-less install fails loudly, not mysteriously', () => {
    assert.ok(
      existsSync(join(unpacked, 'pkg', 'opensolid_wasm_bg.wasm')),
      'precondition: the unpacked tarball has the wasm',
    );
    const check = resolve(serverDir, 'scripts', 'check-pkg.mjs');
    const emptyPkg = mkdtempSync(join(tmpdir(), 'opensolid-nowasm-'));
    // check-pkg resolves pkg/ as a sibling of scripts/; mirror that layout.
    const scripts = join(emptyPkg, 'scripts');
    execFileSync('mkdir', ['-p', scripts]);
    writeFileSync(join(scripts, 'check-pkg.mjs'), readFileSync(check, 'utf8'));
    try {
      execFileSync(process.execPath, [join(scripts, 'check-pkg.mjs')], { stdio: 'pipe' });
      assert.fail('check-pkg passed with no pkg/ present');
    } catch (err) {
      assert.equal(err.status, 1);
      assert.match(String(err.stderr), /wasm kernel is not built|Missing/);
    } finally {
      rmSync(emptyPkg, { recursive: true, force: true });
    }
  });
});
