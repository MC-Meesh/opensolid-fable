// Acceptance test for the right-angle bracket (of-2y4.1) — the "real part" gate.
//
// Unlike tools.test.js, this drives the **actual stdio server** over JSON-RPC
// rather than calling the tool handlers in-process, so the transport, the
// framing, and the wasm load are all on the hook too. It is the same part and
// the same script the agent gallery publishes
// (examples/agent-gallery/bracket-right-angle.md).
//
// The spec: 60x40x5 base plate, 40x40x5 vertical plate, triangular gusset,
// 4x M5 (Ø5) mounting holes two per plate, 3 mm fillets on the interior corner
// and the gusset edges.
//
// The assertions that matter are the *volume* ones. A screenshot cannot tell
// you a hole went in sideways, and `valid: true` does not either — only a
// volume checked against a hand-computed number does. See the analytic
// derivation on ANALYTIC_VOLUME below.

import { test, before, after, describe } from 'node:test';
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { createInterface } from 'node:readline';
import { mkdtempSync, rmSync } from 'node:fs';
import { tmpdir } from 'node:os';
import { fileURLToPath } from 'node:url';
import { dirname, resolve, join } from 'node:path';

const here = dirname(fileURLToPath(import.meta.url));
const SERVER = resolve(here, '..', 'src', 'server.js');

// ── Analytic truth ─────────────────────────────────────────────────────────
// L-section area, drawn in (x, z) and swept 40 mm along +Y:
//   base plate            60 x 5                        = 300 mm^2
//   wall above the base    5 x 35                       = 175 mm^2
//   3 mm interior fillet   r^2 - pi*r^2/4               =   1.93 mm^2
//   -> (300 + 175 + 1.9314) * 40                        = 19077.25 mm^3
// gusset: right triangle, 20 mm legs, 5 mm thick        =  1000    mm^3
// four Ø5 holes, each through 5 mm of plate: 4*pi*2.5^2*5 = -392.70 mm^3
// The smoothUnion blend adds a little material at the gusset joints; measured
// against the sharp union it is ~127 mm^3, which is why the tolerance below is
// a band and not a point.
const L_SECTION = (300 + 175 + (9 - (Math.PI * 9) / 4)) * 40;
const GUSSET = 0.5 * 20 * 20 * 5;
const HOLES = 4 * Math.PI * 2.5 ** 2 * 5;
const ANALYTIC_VOLUME = L_SECTION + GUSSET - HOLES; // ~19791.6 mm^3

// The SDF mesher reads slightly under true volume at the default accuracy
// (a plain 60x40x5 slab measures 11968 against a true 12000, -0.26%). 1.5%
// is loose enough for that bias plus the blend, and tight enough to fail if a
// hole goes in on the wrong axis (that is a ~2-4x error, not a percent).
const VOLUME_TOL = 0.015;

const BODY_SCRIPT = `
const B = 0.41421356237309503;          // tan(90°/4): a 90° arc, DXF bulge
const p = new Profile(-30, 0);
p.lineTo(30, 0);
p.lineTo(30, 5);
p.lineTo(-22, 5);
p.arcTo(-25, 8, -B);                    // 3 mm interior corner fillet
p.lineTo(-25, 40);
p.lineTo(-30, 40);
p.close();
const ell = Shape.extrude(p, 40);       // extrude sweeps +Y: the 40 mm width
const t = new Profile(-25, 5);
t.lineTo(-5, 5);
t.lineTo(-25, 25);
t.close();
const gusset = Shape.extrude(t, 5).translate(0, 17.5, 0);
let part = ell.smoothUnion(gusset, 3);  // 3 mm fillets on the gusset edges
`;

// cylinder() is a +Y-axis cylinder, so each hole is rotated onto its drilling
// axis first. Getting this wrong is silent: the part still reports valid:true.
const DRILL_SCRIPT = `
const zHole = Shape.cylinder(2.5, 10).rotate(1, 0, 0, 90);   // -> +Z, base plate
for (const y of [10, 30]) part = part.subtract(zHole.translate(15, y, 0));
const xHole = Shape.cylinder(2.5, 10).rotate(0, 0, 1, 90);   // -> +X, vertical plate
for (const y of [10, 30]) part = part.subtract(xHole.translate(-27.5, y, 32));
`;

// The trailing 360° rotation is a workaround, not modelling: it is the
// identity geometrically, but it perturbs the tracked bounding box, and
// without that perturbation this part meshes open at the default accuracy and
// STEP export declines. Tracked in of-obv.
const BRACKET_SCRIPT = `${BODY_SCRIPT}${DRILL_SCRIPT}\nreturn part.rotate(0, 1, 0, 360);`;
const UNDRILLED_SCRIPT = `${BODY_SCRIPT}\nreturn part.rotate(0, 1, 0, 360);`;

// ── Minimal MCP stdio client ───────────────────────────────────────────────
function connect(outputDir) {
  const child = spawn('node', [SERVER], {
    stdio: ['pipe', 'pipe', 'pipe'],
    env: { ...process.env, OPENSOLID_MCP_OUTPUT_DIR: outputDir },
  });
  child.stderr.resume(); // drain the ready banner; never parse stderr
  const pending = new Map();
  createInterface({ input: child.stdout }).on('line', (line) => {
    if (!line.trim()) return;
    let msg;
    try {
      msg = JSON.parse(line);
    } catch {
      return;
    }
    const p = pending.get(msg.id);
    if (!p) return;
    pending.delete(msg.id);
    if (msg.error) p.reject(new Error(`${msg.error.code}: ${msg.error.message}`));
    else p.resolve(msg.result);
  });
  let nextId = 1;
  const request = (method, params) => {
    const id = nextId++;
    return new Promise((res, rej) => {
      pending.set(id, { resolve: res, reject: rej });
      child.stdin.write(JSON.stringify({ jsonrpc: '2.0', id, method, params }) + '\n');
    });
  };
  return { request, close: () => child.stdin.end() };
}

// Unwrap an MCP tool result into the shape the assertions want.
function unwrap(res) {
  const c = res?.content?.[0];
  if (c?.type === 'image') return { isError: !!res.isError, image: c };
  let json = null;
  try {
    json = JSON.parse(c?.text ?? '');
  } catch {
    /* plain-text error */
  }
  return { isError: !!res.isError, text: c?.text ?? '', json };
}

describe('right-angle bracket acceptance (of-2y4.1)', () => {
  let client;
  let outputDir;
  const call = async (name, args) => unwrap(await client.request('tools/call', { name, arguments: args }));

  before(async () => {
    outputDir = mkdtempSync(join(tmpdir(), 'bracket-acceptance-'));
    client = connect(outputDir);
    const init = await client.request('initialize', {
      protocolVersion: '2024-11-05',
      capabilities: {},
      clientInfo: { name: 'bracket-acceptance', version: '1' },
    });
    assert.equal(init.serverInfo.name, 'opensolid-mcp-server');
  });

  after(() => {
    client?.close();
    if (outputDir) rmSync(outputDir, { recursive: true, force: true });
  });

  test('builds a closed, watertight solid', async () => {
    const r = await call('create_model', { script: BRACKET_SCRIPT, name: 'bracket-right-angle' });
    assert.ok(!r.isError, `create_model failed: ${r.text}`);
    assert.equal(r.json.valid, true, `bracket is not a valid solid: ${JSON.stringify(r.json.issues)}`);
    assert.deepEqual(r.json.issues, []);
    assert.ok(r.json.mesh.triangles > 1000, 'suspiciously coarse mesh');
  });

  test('volume matches the hand-computed section within tolerance', async () => {
    const r = await call('create_model', { script: BRACKET_SCRIPT, name: 'bracket-volume' });
    assert.ok(!r.isError, r.text);
    const err = Math.abs(r.json.volume - ANALYTIC_VOLUME) / ANALYTIC_VOLUME;
    assert.ok(
      err < VOLUME_TOL,
      `volume ${r.json.volume.toFixed(1)} mm^3 is ${(err * 100).toFixed(2)}% off the ` +
        `analytic ${ANALYTIC_VOLUME.toFixed(1)} mm^3 (tolerance ${VOLUME_TOL * 100}%)`,
    );
  });

  // The regression guard for the axis convention. cylinder() is +Y-axis; if a
  // future change makes these rotations wrong (or a reader "fixes" them to the
  // +Z the docs used to claim), the holes stop being through-holes and become
  // channels through the part. That does NOT trip `valid`, and it does not
  // trip a screenshot. It only shows up as removed volume, so assert on it.
  test('the four M5 holes remove the right amount of material', async () => {
    const drilled = await call('create_model', { script: BRACKET_SCRIPT, name: 'drilled' });
    const solid = await call('create_model', { script: UNDRILLED_SCRIPT, name: 'undrilled' });
    assert.ok(!drilled.isError && !solid.isError, 'both models must build');
    const removed = solid.json.volume - drilled.json.volume;
    // Four Ø5 holes through 5 mm of plate = 392.7 mm^3.
    //
    // The band is wide on purpose. This differences two *independently meshed*
    // volumes, and each carries the mesher's ~0.3% bias on a ~20000 mm^3 body
    // — around +/-60 mm^3 of noise on a 392.7 mm^3 signal, so ~15% before
    // anything is wrong. (Measured drift on the reference part: 438.8.) It
    // still separates cleanly from the failure it exists to catch: a hole on
    // the wrong axis bores a channel lengthwise through the part and removes
    // 800-1600 mm^3, several times the true figure.
    const HOLE_TOL = 0.25;
    assert.ok(
      Math.abs(removed - HOLES) / HOLES < HOLE_TOL,
      `holes removed ${removed.toFixed(1)} mm^3, expected ~${HOLES.toFixed(1)} mm^3. ` +
        'A large overshoot means a hole is drilled on the wrong axis.',
    );
  });

  test('renders each named view', async () => {
    const r = await call('create_model', { script: BRACKET_SCRIPT, name: 'bracket-views' });
    assert.ok(!r.isError, r.text);
    for (const view of ['iso', 'top', 'front']) {
      const shot = await call('get_screenshot', { model_id: r.json.model_id, view, width: 320, height: 240 });
      assert.ok(!shot.isError, `${view} render failed: ${shot.text}`);
      assert.equal(shot.image.mimeType, 'image/png');
      // A PNG of an empty frame still encodes; require real bytes.
      assert.ok(shot.image.data.length > 500, `${view} render is suspiciously small`);
    }
  });

  test('exports STEP and STL', async () => {
    const r = await call('create_model', { script: BRACKET_SCRIPT, name: 'bracket-export' });
    assert.ok(!r.isError, r.text);
    for (const format of ['step', 'stl']) {
      const e = await call('export', { model_id: r.json.model_id, format, path: `bracket-acceptance.${format}` });
      assert.ok(!e.isError, `${format} export failed: ${e.text}`);
      assert.ok(e.json.bytes > 1000, `${format} export is suspiciously small`);
    }
  });
});
