// The doc<->capability gap, held shut by test (of-2y4.4).
//
// The script DSL hands a script whole wasm classes, so *every* member of those
// classes is callable whether or not anyone wrote it down. That is how fourteen
// ops (cone, halfSpace, rib, loft, taper, shell, filletEdge, chamferEdge, the
// two patterns, mirror, distance, normalAt, bounds) ended up shipped and
// undocumented, and how `sweep` ended up documented-ish but *unreachable* —
// nothing bound the `Path` class it needs.
//
// These tests close both directions and keep them closed:
//   - every op the manifest claims actually exists on the bound class;
//   - every member of a bound class appears in the manifest (as an op or as
//     explicitly-listed internal plumbing) — so a new kernel op cannot land
//     hidden;
//   - every binding the manifest names is actually injected into a script — so
//     an op cannot be documented while its argument type is unreachable;
//   - every non-internal op is named in AGENT_GUIDE.md — so the prose cannot
//     silently fall behind again.

import { test, describe } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync, mkdtempSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';
import { dirname, resolve, join } from 'node:path';
import { createTools } from '../src/tools.js';
import { Shape, Profile, Path, OpenPath, runScript } from '../src/kernel.js';
import { SCRIPT_API, SCRIPT_BINDINGS, SERVER_INFO, UNITS, buildManifest } from '../src/capabilities.js';

const here = dirname(fileURLToPath(import.meta.url));
const repoRoot = resolve(here, '../../..');

/** A tool registry writing to a throwaway dir, as in tools.test.js. */
function freshTools() {
  return createTools({ outputDir: mkdtempSync(join(tmpdir(), 'osmcp-cap-')) });
}

// wasm-bindgen plumbing every generated class carries. Not API in any sense.
const WBG_INTERNALS = new Set(['__wrap', '__destroy_into_raw', '__wbg_ptr']);

const CLASSES = {
  Shape: Shape,
  Profile: Profile,
  Path: Path,
  OpenPath: OpenPath,
};

/** Every member a script could reach on a bound class. */
function membersOf(Class) {
  const statics = Object.getOwnPropertyNames(Class)
    .filter((k) => !['length', 'name', 'prototype'].includes(k))
    .filter((k) => !WBG_INTERNALS.has(k));
  const methods = Object.getOwnPropertyNames(Class.prototype).filter((k) => !WBG_INTERNALS.has(k));
  return { statics, methods };
}

/** The names the manifest claims for a binding, split by static vs instance. */
function manifestNames(bindingName) {
  const entry = SCRIPT_API[bindingName];
  const statics = new Set(entry.statics.map((op) => op.name));
  const methods = new Set(entry.methods.map((op) => op.name));
  for (const op of entry.internal) {
    (op.static ? statics : methods).add(op.name);
  }
  return { statics, methods };
}

describe('capability manifest matches the bound classes', () => {
  for (const [bindingName, Class] of Object.entries(CLASSES)) {
    test(`${bindingName}: every documented op exists`, () => {
      const actual = membersOf(Class);
      const claimed = manifestNames(bindingName);
      for (const name of claimed.statics) {
        assert.ok(
          actual.statics.includes(name),
          `manifest claims ${bindingName}.${name} but the class has no such static`,
        );
      }
      for (const name of claimed.methods) {
        assert.ok(
          actual.methods.includes(name),
          `manifest claims ${bindingName}#${name} but the class has no such method`,
        );
      }
    });

    test(`${bindingName}: no member is hidden from the manifest`, () => {
      const actual = membersOf(Class);
      const claimed = manifestNames(bindingName);
      for (const name of actual.statics) {
        assert.ok(
          claimed.statics.has(name),
          `${bindingName}.${name} is callable from a script but missing from ` +
            'SCRIPT_API in src/capabilities.js — document it or list it as internal',
        );
      }
      for (const name of actual.methods) {
        assert.ok(
          claimed.methods.has(name),
          `${bindingName}#${name} is callable from a script but missing from ` +
            'SCRIPT_API in src/capabilities.js — document it or list it as internal',
        );
      }
    });
  }

  test('every op carries a signature, a doc line, and named params', () => {
    for (const [bindingName, entry] of Object.entries(SCRIPT_API)) {
      const ops = bindingName === 'param' ? [entry] : [...entry.statics, ...entry.methods];
      for (const op of ops) {
        assert.ok(op.signature, `${bindingName}.${op.name} has no signature`);
        assert.ok(op.doc, `${bindingName}.${op.name} has no doc`);
        assert.ok(Array.isArray(op.params), `${bindingName}.${op.name} has no params array`);
        for (const p of op.params) {
          assert.ok(p.name && p.type, `${bindingName}.${op.name} has an unnamed/untyped param`);
          assert.ok(
            op.signature.includes(p.name),
            `${bindingName}.${op.name}: param '${p.name}' is absent from the signature`,
          );
        }
      }
    }
  });
});

describe('capability manifest matches the live tool registry', () => {
  test('get_capabilities lists exactly the tools the server serves', () => {
    const t = freshTools();
    const manifest = JSON.parse(t.call('get_capabilities', {}).content[0].text);
    const served = t.definitions.map((d) => d.name).sort();
    assert.deepEqual(
      manifest.tools.map((x) => x.name).sort(),
      served,
      'the manifest tool list drifted from tools/list',
    );
    for (const tool of manifest.tools) {
      assert.ok(tool.description, `${tool.name} has no description`);
      assert.ok(tool.inputSchema, `${tool.name} has no inputSchema`);
    }
    assert.deepEqual(manifest.server, SERVER_INFO);
    assert.deepEqual(manifest.script.bindings, SCRIPT_BINDINGS);
  });

  test('sections narrow the manifest and an unknown one is a clean error', () => {
    const t = freshTools();
    for (const section of ['tools', 'script', 'conventions', 'units']) {
      const out = JSON.parse(t.call('get_capabilities', { section }).content[0].text);
      assert.deepEqual(Object.keys(out), [section]);
    }
    const bad = t.call('get_capabilities', { section: 'everything' });
    assert.equal(bad.isError, true);
    assert.match(bad.content[0].text, /unknown section 'everything'/);
  });

  test('buildManifest carries the tool input schemas verbatim', () => {
    const t = freshTools();
    const manifest = buildManifest(SERVER_INFO, t.definitions);
    const exportTool = manifest.tools.find((x) => x.name === 'export');
    assert.deepEqual(
      exportTool.inputSchema.properties.unit.enum,
      UNITS.map((u) => u.key),
    );
  });
});

describe('every documented binding is actually reachable from a script', () => {
  test('runScript injects every binding the manifest names', () => {
    // An unbound name is a ReferenceError inside the script — exactly the
    // failure that stranded `sweep`, since a script cannot construct a `Path`
    // it has no name for.
    for (const binding of SCRIPT_BINDINGS) {
      const { shape } = runScript(
        `if (typeof ${binding} !== 'function') throw new Error('${binding} is not bound');
         return Shape.sphere(1);`,
      );
      assert.ok(shape, `${binding} is not reachable from a script`);
    }
  });

  test('Shape.sweep is reachable — it needs the Path binding', () => {
    const t = freshTools();
    const out = t.call('create_model', {
      script: `
        const p = new Profile(-2, -2);
        p.lineTo(2, -2); p.lineTo(2, 2); p.lineTo(-2, 2); p.close();
        const path = new Path(0, 0, 0);
        path.lineTo(0, 20, 0);
        path.lineTo(10, 20, 0);
        return Shape.sweep(p, path);`,
    });
    assert.equal(out.isError, undefined, out.content[0].text);
    const model = JSON.parse(out.content[0].text);
    assert.equal(model.valid, true);
    // An L of two 20-and-10-long runs on a 4x4 section: hundreds of units, and
    // decisively more than either leg alone (the mitre joins them).
    assert.ok(model.volume > 400, `swept volume ${model.volume}`);
  });

  test('Shape.rib is reachable — it needs the OpenPath binding', () => {
    const t = freshTools();
    const out = t.call('create_model', {
      script: `
        const base = Shape.box3(20, 1, 20);
        const spine = new OpenPath(-15, 0);
        spine.lineTo(15, 0);
        return base.union(Shape.rib(spine, 2, 10, 'both'));`,
    });
    assert.equal(out.isError, undefined, out.content[0].text);
    const model = JSON.parse(out.content[0].text);
    assert.equal(model.valid, true);
    // The 40x2x40 plate alone is 3200; the rib adds material on top of it.
    assert.ok(model.volume > 3300, `ribbed volume ${model.volume}`);
  });

  test('an unknown rib side is an error that names the alternatives', () => {
    const t = freshTools();
    const out = t.call('create_model', {
      script: `
        const spine = new OpenPath(0, 0);
        spine.lineTo(10, 0);
        return Shape.rib(spine, 2, 10, 'left');`,
    });
    assert.equal(out.isError, true);
    assert.match(out.content[0].text, /both\/first\/second/);
  });
});

describe('the prose docs cover the whole surface', () => {
  const guide = readFileSync(resolve(repoRoot, 'docs/AGENT_GUIDE.md'), 'utf8');

  test('AGENT_GUIDE.md names every non-internal script op', () => {
    // Match the call form — `Shape.cone(` or `.filletEdge(` — not the bare
    // word, so a stray mention in prose ("meshing bounds") cannot pass for
    // documentation of `bounds()`.
    const missing = [];
    for (const [bindingName, entry] of Object.entries(SCRIPT_API)) {
      if (bindingName === 'param') continue;
      for (const op of entry.statics) {
        if (!guide.includes(`${bindingName}.${op.name}(`)) missing.push(`${bindingName}.${op.name}`);
      }
      for (const op of entry.methods) {
        if (op.name === 'constructor') continue;
        if (!guide.includes(`.${op.name}(`)) missing.push(`${bindingName}.${op.name}`);
      }
    }
    assert.deepEqual(
      missing,
      [],
      `undocumented in docs/AGENT_GUIDE.md: ${missing.join(', ')} — these are ` +
        'callable from any script, so an agent that has not read the Rust cannot find them',
    );
  });

  test('AGENT_GUIDE.md names every tool', () => {
    const t = freshTools();
    for (const def of t.definitions) {
      assert.ok(guide.includes(def.name), `docs/AGENT_GUIDE.md does not mention the ${def.name} tool`);
    }
  });
});
