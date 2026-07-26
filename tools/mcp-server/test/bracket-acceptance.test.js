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
// (a plain 60x40x5 slab measures 11996 against a true 12000, -0.03%). 1.5%
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

// This script once ended in a no-op 360° rotation: a workaround for a
// bounds-alignment mesher defect (of-obv) without which the part meshed open
// and STEP export declined. The of-obv fix removed the need; the part meshes
// closed as written.
const BRACKET_SCRIPT = `${BODY_SCRIPT}${DRILL_SCRIPT}\nreturn part;`;
const UNDRILLED_SCRIPT = `${BODY_SCRIPT}\nreturn part;`;

// The of-4tu bug itself, kept as a live negative control: the same four bores
// with the rotations omitted, which is what an agent writes if it believes the
// docs' old claim that `cylinder` stands on +Z. This part is a perfectly valid
// closed solid whose holes run lengthwise through the plates. It is the shape
// the correctness oracles have to be able to reject.
const MISDRILLED_SCRIPT = `${BODY_SCRIPT}
const hole = Shape.cylinder(2.5, 10);                        // left on +Y
for (const y of [10, 30]) part = part.subtract(hole.translate(15, y, 0));
for (const y of [10, 30]) part = part.subtract(hole.translate(-27.5, y, 32));
return part;`;

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
    // volumes, and each carries the mesher's per-mesh bias on a ~20000 mm^3
    // body — tens of mm^3 of noise on a 392.7 mm^3 signal. (Measured drift on
    // the reference part: 390.3.) It still separates cleanly from the failure
    // it exists to catch: a hole on the wrong axis bores a channel lengthwise
    // through the part and removes 800-1600 mm^3, several times the true
    // figure.
    const HOLE_TOL = 0.25;
    assert.ok(
      Math.abs(removed - HOLES) / HOLES < HOLE_TOL,
      `holes removed ${removed.toFixed(1)} mm^3, expected ~${HOLES.toFixed(1)} mm^3. ` +
        'A large overshoot means a hole is drilled on the wrong axis.',
    );
  });

  // The oracles from of-2y4.5, on the part that motivated them. The point of
  // these three tests is the pairing: each one must pass on the bracket *and*
  // fail on MISDRILLED_SCRIPT, which is a valid closed solid with the same
  // nominal features on the wrong axes. A check that only passes on the good
  // part would have passed on the bad one too — that is how of-4tu shipped.
  test('inspect_topology finds the four bores on their intended axes', async () => {
    const r = await call('create_model', { script: BRACKET_SCRIPT, name: 'bracket-topology' });
    assert.ok(!r.isError, r.text);
    const topo = await call('inspect_topology', { model_id: r.json.model_id });
    assert.ok(!topo.isError, topo.text);
    const { counts, cylinders } = topo.json;
    assert.equal(counts.shells, 1, 'the bracket is one solid');
    assert.equal(counts.genus, 4, 'four holes through the part');
    assert.equal(counts.throughHoles, 4);
    // Two bores along +Z through the base plate, two along +X through the wall.
    const axisOf = (c) => c.axis.map((v) => Math.round(Math.abs(v))).join('');
    const byAxis = cylinders.map(axisOf).sort();
    assert.deepEqual(byAxis, ['001', '001', '100', '100']);
    for (const c of cylinders) {
      assert.equal(c.kind, 'through-hole');
      assert.ok(Math.abs(c.diameter - 5) < 0.15, `bore Ø${c.diameter}`);
      // Each passes through 5 mm of plate.
      assert.ok(Math.abs(c.depth - 5) < 0.25, `bore depth ${c.depth}`);
    }
  });

  test('assert_model passes on the bracket and rejects the mis-drilled part', async () => {
    // Written the way an agent should: state the intent, including the axes.
    const expect = [
      { type: 'closed_solid' },
      { type: 'shells', value: 1 },
      { type: 'genus', value: 4 },
      { type: 'volume', value: ANALYTIC_VOLUME, relative_tolerance: VOLUME_TOL },
      { type: 'bbox_size', value: [60, 40, 40], tolerance: 0.5 },
      { type: 'through_holes', value: 2, axis: [0, 0, 1], diameter: 5, tolerance: 0.3 },
      { type: 'through_holes', value: 2, axis: [1, 0, 0], diameter: 5, tolerance: 0.3 },
      { type: 'hole_at', at: [15, 10, 2.5], axis: [0, 0, 1], diameter: 5, tolerance: 0.3 },
    ];

    const good = await call('create_model', { script: BRACKET_SCRIPT, name: 'bracket-assert' });
    assert.ok(!good.isError, good.text);
    const pass = await call('assert_model', { model_id: good.json.model_id, expect });
    assert.ok(!pass.isError, pass.text);
    assert.equal(
      pass.json.ok,
      true,
      `the bracket should meet its own spec: ${JSON.stringify(
        pass.json.checks.filter((c) => !c.ok),
        null,
        2,
      )}`,
    );

    const bad = await call('create_model', { script: MISDRILLED_SCRIPT, name: 'bracket-misdrilled' });
    assert.ok(!bad.isError, bad.text);
    // The damning fact this whole exercise turns on: the wrong part is a valid
    // solid. Mesh closure cannot adjudicate a hole's direction.
    assert.equal(bad.json.valid, true, 'the mis-drilled part really does report valid');
    const fail = await call('assert_model', { model_id: bad.json.model_id, expect });
    assert.ok(!fail.isError, fail.text);
    assert.equal(fail.json.ok, false, 'the mis-drilled part must not pass the spec');
    const byType = new Map(fail.json.checks.map((c) => [`${c.type}:${JSON.stringify(c.expected)}`, c]));
    assert.equal(
      byType.get('closed_solid:true').ok,
      true,
      'and it fails on structure, not on closure — closure passes',
    );
    for (const c of fail.json.checks.filter((x) => x.type === 'through_holes')) {
      assert.equal(c.ok, false, `axis assertion should fail: ${c.message}`);
      assert.match(c.message, /axis/);
    }
    assert.equal(byType.get('hole_at:{"at":[15,10,2.5],"axis":[0,0,1],"diameter":5}').ok, false);
  });

  test('diff_models checks the holes removed the volume they should', async () => {
    const solid = await call('create_model', { script: UNDRILLED_SCRIPT, name: 'diff-undrilled' });
    const drilled = await call('create_model', { script: BRACKET_SCRIPT, name: 'diff-drilled' });
    assert.ok(!solid.isError && !drilled.isError, 'both models must build');
    // Same tolerance rationale as the volume-delta test above: two independently
    // meshed ~20000 mm³ bodies differenced for a 392.7 mm³ signal.
    const good = await call('diff_models', {
      model_id_a: solid.json.model_id,
      model_id_b: drilled.json.model_id,
      expect_volume_delta: { value: -HOLES, relative_tolerance: 0.25 },
    });
    assert.ok(!good.isError, good.text);
    assert.equal(good.json.volumeDeltaCheck.ok, true, good.json.volumeDeltaCheck.message);
    // Drilling four through-holes adds exactly four handles to the surface.
    assert.equal(good.json.delta.counts.genus, 4);
    assert.equal(good.json.delta.counts.throughHoles, 4);

    // The mis-drilled part fails the same delta: a lengthwise channel removes
    // several times the material a through-hole does.
    const bad = await call('create_model', { script: MISDRILLED_SCRIPT, name: 'diff-misdrilled' });
    const wrong = await call('diff_models', {
      model_id_a: solid.json.model_id,
      model_id_b: bad.json.model_id,
      expect_volume_delta: { value: -HOLES, relative_tolerance: 0.25 },
    });
    assert.ok(!wrong.isError, wrong.text);
    assert.equal(
      wrong.json.volumeDeltaCheck.ok,
      false,
      `mis-drilled delta ${wrong.json.delta.volume} should not pass as ${-HOLES}`,
    );
    // ...and it severed the part in two, which the shell count says outright.
    assert.equal(wrong.json.b.counts.shells, 2, 'the lengthwise bores cut the plate apart');
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
