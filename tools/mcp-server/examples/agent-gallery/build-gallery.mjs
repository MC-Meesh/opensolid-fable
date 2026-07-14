// Reproducer for the agent gallery. Drives the *real* MCP tool handlers
// (createTools — the same code path the stdio server dispatches) for every
// worked example, then writes screenshots + STEP/STL exports into
// examples/output/ and prints a JSON manifest of the results.
//
// Nothing here is hand-authored geometry data: every number in the gallery
// transcripts is copied from this script's output. Regenerate any time with:
//
//   cd tools/mcp-server && npm run build && node examples/agent-gallery/build-gallery.mjs
//
import { writeFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve } from 'node:path';
import { createTools } from '../../src/tools.js';

const here = dirname(fileURLToPath(import.meta.url));
const outputDir = resolve(here, '..', 'output');
const tools = createTools({ outputDir });

// Each example is a sequence of tool calls. `capture` runs one call, records a
// trimmed view of the result, and returns the parsed payload for chaining.
const manifest = [];

function callTool(name, args) {
  const res = tools.call(name, args);
  if (res.isError) {
    throw new Error(`${name} failed: ${res.content[0].text}`);
  }
  console.error(`  ok  ${name}${args.format ? ' ' + args.format : ''}${args.view ? ' ' + args.view : ''}`);
  return res;
}

function createModel(args) {
  const res = callTool('create_model', args);
  return JSON.parse(res.content[0].text);
}

function measure(model_id, query) {
  const res = callTool('measure', { model_id, query });
  return JSON.parse(res.content[0].text);
}

function validate(model_id) {
  const res = callTool('validate', { model_id });
  return JSON.parse(res.content[0].text);
}

function screenshot(model_id, file, view = 'iso', width = 720, height = 540) {
  const res = callTool('get_screenshot', { model_id, view, width, height });
  const png = Buffer.from(res.content[0].data, 'base64');
  const path = resolve(outputDir, file);
  writeFileSync(path, png);
  return { file, view, bytes: png.length };
}

function exportFile(model_id, format, path) {
  const res = callTool('export', { model_id, format, path });
  return JSON.parse(res.content[0].text);
}

// ─────────────────────────────────────────────────────────────────────────
// 1. Angle bracket with four mounting holes
// ─────────────────────────────────────────────────────────────────────────
{
  const script = `
// 90° angle bracket: a 60×40×4 horizontal flange and a 60×4×40 vertical
// flange along the back edge, with four Ø6 mounting holes in the base.
const base = Shape.box3(30, 20, 2);                        // 60 × 40 × 4
const wall = Shape.box3(30, 2, 20).translate(0, -18, 22);  // 60 × 4 × 40, back edge
let bracket = base.union(wall);
const hole = Shape.cylinder(3, 6);                         // r=3, punches through
for (const x of [-20, 20]) for (const y of [-12, 6]) {
  bracket = bracket.subtract(hole.translate(x, y, 0));
}
return bracket;
`.trim();
  const m = createModel({ script, name: 'angle-bracket' });
  const shots = [screenshot(m.model_id, 'angle-bracket-iso.png', 'iso')];
  const step = exportFile(m.model_id, 'step', 'angle-bracket.step');
  const stl = exportFile(m.model_id, 'stl', 'angle-bracket.stl');
  manifest.push({ example: 'angle-bracket', script, create: m, mass: measure(m.model_id, 'mass'), shots, step, stl });
}

// ─────────────────────────────────────────────────────────────────────────
// 2. Hinge leaf with three knuckles and a pin bore
// ─────────────────────────────────────────────────────────────────────────
{
  const script = `
// One leaf of a butt hinge: a flat plate with three barrel knuckles on the
// pin axis (X) and a pin bore drilled through them. Two of these — one
// mirrored — pin together into a working hinge.
const plate = Shape.box3(30, 15, 0.75).translate(0, -15.75, 0);  // 60 × 30 × 1.5 leaf
// A knuckle is a cylinder whose +Z axis is rotated onto +X, then slid along X.
const knuckle = Shape.cylinder(4, 6).rotate(0, 1, 0, 90);        // r=4, 12 long on X
let leaf = plate;
for (const x of [-24, 0, 24]) leaf = leaf.union(knuckle.translate(x, 0, 0));
const pin = Shape.cylinder(1.6, 40).rotate(0, 1, 0, 90);         // Ø3.2 bore on X
return leaf.subtract(pin);
`.trim();
  const m = createModel({ script, name: 'hinge-leaf' });
  const shots = [screenshot(m.model_id, 'hinge-leaf-iso.png', 'iso')];
  const step = exportFile(m.model_id, 'step', 'hinge-leaf.step');
  const stl = exportFile(m.model_id, 'stl', 'hinge-leaf.stl');
  manifest.push({ example: 'hinge-leaf', script, create: m, mass: measure(m.model_id, 'mass'), shots, step, stl });
}

// ─────────────────────────────────────────────────────────────────────────
// 3. Enclosure — shelled body (open top) + press-fit lid
// ─────────────────────────────────────────────────────────────────────────
{
  const bodyScript = `
// Electronics enclosure: a rounded box hollowed to a 3 mm wall with an open
// top. The shell is outer.subtract(inner) — the inner cavity is raised so it
// breaks through the top face, leaving the box open for the lid.
const outer = Shape.roundedBox(40, 30, 15, 3);                 // 80 × 60 × 30
const cavity = Shape.box3(37, 27, 14).translate(0, 0, 2);      // 3 mm walls, open top
return outer.subtract(cavity);
`.trim();
  const body = createModel({ script: bodyScript, name: 'enclosure-body' });
  const bodyShots = [screenshot(body.model_id, 'enclosure-body-iso.png', 'iso')];
  const bodyStep = exportFile(body.model_id, 'step', 'enclosure-body.step');
  const bodyStl = exportFile(body.model_id, 'stl', 'enclosure-body.stl');

  const lidScript = `
// Matching lid: a top plate with a recessed lip that press-fits into the
// enclosure's open mouth (0.5 mm clearance on each wall).
const cap = Shape.roundedBox(40, 30, 1.5, 3).translate(0, 0, 16.5);  // top plate
const lip = Shape.box3(36.5, 26.5, 2).translate(0, 0, 13);           // insert lip
return cap.union(lip);
`.trim();
  const lid = createModel({ script: lidScript, name: 'enclosure-lid' });
  const lidShots = [screenshot(lid.model_id, 'enclosure-lid-iso.png', 'iso')];
  const lidStep = exportFile(lid.model_id, 'step', 'enclosure-lid.step');
  const lidStl = exportFile(lid.model_id, 'stl', 'enclosure-lid.stl');

  manifest.push({
    example: 'enclosure',
    body: { script: bodyScript, create: body, mass: measure(body.model_id, 'mass'), shots: bodyShots, step: bodyStep, stl: bodyStl },
    lid: { script: lidScript, create: lid, mass: measure(lid.model_id, 'mass'), shots: lidShots, step: lidStep, stl: lidStl },
  });
}

// ─────────────────────────────────────────────────────────────────────────
// 4. Spur-gear-ish toothed disk (circular pattern via a loop)
// ─────────────────────────────────────────────────────────────────────────
{
  const script = `
// A toothed disk: a root disk with N teeth placed on a circular pattern by
// rotating one tooth box around the Z axis, plus a central bore. The pattern
// is just a JS loop — the script vocabulary is a real programming language.
const TEETH = 16, TH = 4, ROOT = 16, BORE = 4;
let gear = Shape.cylinder(ROOT, TH);
const tooth = Shape.box3(3, 2.2, TH).translate(ROOT + 1.5, 0, 0);
for (let i = 0; i < TEETH; i++) {
  gear = gear.union(tooth.rotate(0, 0, 1, (360 * i) / TEETH));
}
return gear.subtract(Shape.cylinder(BORE, TH + 2));           // central bore
`.trim();
  const m = createModel({ script, name: 'gear-disk' });
  const shots = [screenshot(m.model_id, 'gear-disk-top.png', 'top'), screenshot(m.model_id, 'gear-disk-iso.png', 'iso')];
  const step = exportFile(m.model_id, 'step', 'gear-disk.step');
  const stl = exportFile(m.model_id, 'stl', 'gear-disk.stl');
  manifest.push({ example: 'gear-disk', script, create: m, mass: measure(m.model_id, 'mass'), shots, step, stl });
}

// ─────────────────────────────────────────────────────────────────────────
// 5. Bottle — revolve a silhouette, then shell it hollow
// ─────────────────────────────────────────────────────────────────────────
{
  const script = `
// A bottle: revolve a 2D silhouette 360° about the Y axis, then hollow it by
// subtracting an inner revolve that breaks through the top (the shell). The
// arcTo on the shoulder is a rounded fillet from body to neck.
const outer = new Profile(0, 0);
outer.lineTo(18, 0);
outer.lineTo(18, 44);
outer.arcTo(6, 60, 0.55);     // rounded shoulder (fillet), body -> neck
outer.lineTo(6, 74);
outer.lineTo(0, 74);
outer.close();
let bottle = Shape.revolve(outer, 360);

const cavity = new Profile(0, 3);
cavity.lineTo(15, 3);
cavity.lineTo(15, 44);
cavity.arcTo(3, 60, 0.55);
cavity.lineTo(3, 90);         // taller than the neck: opens the mouth
cavity.lineTo(0, 90);
cavity.close();
return bottle.subtract(Shape.revolve(cavity, 360));
`.trim();
  const m = createModel({ script, name: 'bottle' });
  const shots = [screenshot(m.model_id, 'bottle-front.png', 'front'), screenshot(m.model_id, 'bottle-iso.png', 'iso')];
  const step = exportFile(m.model_id, 'step', 'bottle.step');
  const stl = exportFile(m.model_id, 'stl', 'bottle.stl');
  manifest.push({ example: 'bottle', script, create: m, mass: measure(m.model_id, 'mass'), shots, step, stl });
}

writeFileSync(resolve(outputDir, 'manifest.json'), JSON.stringify(manifest, null, 2));
console.log(JSON.stringify(manifest, null, 2));
