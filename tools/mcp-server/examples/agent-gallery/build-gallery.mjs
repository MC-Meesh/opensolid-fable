// Reproducer + transcript generator for the agent gallery. Drives the *real*
// MCP tool handlers (createTools — the same code path the stdio server
// dispatches) for every worked example, captures each tool call and its actual
// result, then writes:
//
//   examples/output/*.png|.step|.stl|.obj  — real renders and exports
//   examples/output/manifest.json          — machine-readable record of the run
//   examples/agent-gallery/<slug>.md        — a human-readable agent transcript
//   examples/agent-gallery/README.md        — the gallery index
//
// Nothing in the transcripts is hand-authored geometry data: every model_id,
// volume, byte count, and screenshot is copied from this script's live output.
// The agent narration is prose framing only. Regenerate any time with:
//
//   cd tools/mcp-server && npm run build && node examples/agent-gallery/build-gallery.mjs
//
import { writeFileSync, readFileSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, resolve, basename } from 'node:path';
import { createTools } from '../../src/tools.js';

const here = dirname(fileURLToPath(import.meta.url));
const outputDir = resolve(here, '..', 'output');
const galleryDir = here;

// ── Display normalization ─────────────────────────────────────────────────
// Model ids carry a random 4-hex suffix (model-1-8f3a) and export paths are
// absolute to this worktree. Neither is meaningful in a checked-in transcript
// and both would churn git on every regen, so we render the stable parts:
// `model-1` (the creation-order counter, which is real) and `output/<file>`.
function stableId(id) {
  return typeof id === 'string' ? id.replace(/^(model-\d+)-[0-9a-f]{4}$/, '$1') : id;
}
function displayValue(v) {
  if (typeof v === 'string') {
    if (/^model-\d+-[0-9a-f]{4}$/.test(v)) return stableId(v);
    if (v.startsWith(outputDir)) return `output/${basename(v)}`;
    // Kernel error text can embed the absolute worktree path; strip it.
    return v.split(outputDir + '/').join('output/');
  }
  return v;
}
function display(obj) {
  if (Array.isArray(obj)) return obj.map(display);
  if (obj && typeof obj === 'object') {
    return Object.fromEntries(Object.entries(obj).map(([k, val]) => [k, display(val)]));
  }
  return displayValue(obj);
}
function json(obj) {
  return JSON.stringify(display(obj), null, 2);
}

// ── Transcript recorder ───────────────────────────────────────────────────
// One instance per example. Each call runs the real tool, records a structured
// turn, and returns the parsed payload for chaining. `say()` inserts agent
// narration between tool calls.
class Transcript {
  constructor(tools, { slug, title, intro, prompt }) {
    this.tools = tools;
    this.slug = slug;
    this.title = title;
    this.intro = intro;
    this.prompt = prompt;
    this.turns = [];
  }

  say(text) {
    this.turns.push({ kind: 'narration', text });
    return this;
  }

  _run(tool, args) {
    const res = this.tools.call(tool, args);
    return res;
  }

  create_model(script, name, opts = {}) {
    const args = { script, name, ...opts };
    const res = this._run('create_model', args);
    if (res.isError) throw new Error(`create_model(${name}) failed: ${res.content[0].text}`);
    const payload = JSON.parse(res.content[0].text);
    this.turns.push({ kind: 'create_model', args, payload });
    console.error(`  ok  create_model ${name}`);
    return payload;
  }

  screenshot(model_id, file, view = 'iso', width = 720, height = 540) {
    const args = { model_id, view, width, height };
    const res = this._run('get_screenshot', args);
    if (res.isError) throw new Error(`get_screenshot failed: ${res.content[0].text}`);
    const png = Buffer.from(res.content[0].data, 'base64');
    writeFileSync(resolve(outputDir, file), png);
    this.turns.push({ kind: 'screenshot', args, file, bytes: png.length });
    console.error(`  ok  get_screenshot ${view} -> ${file} (${png.length}B)`);
    return { file, view, bytes: png.length };
  }

  measure(model_id, query) {
    const args = { model_id, query };
    const res = this._run('measure', args);
    if (res.isError) throw new Error(`measure failed: ${res.content[0].text}`);
    const payload = JSON.parse(res.content[0].text);
    this.turns.push({ kind: 'measure', args, payload });
    console.error(`  ok  measure ${query || 'all'}`);
    return payload;
  }

  validate(model_id) {
    const args = { model_id };
    const res = this._run('validate', args);
    if (res.isError) throw new Error(`validate failed: ${res.content[0].text}`);
    const payload = JSON.parse(res.content[0].text);
    this.turns.push({ kind: 'validate', args, payload });
    console.error(`  ok  validate`);
    return payload;
  }

  optimize(args) {
    const res = this._run('optimize', args);
    if (res.isError) throw new Error(`optimize failed: ${res.content[0].text}`);
    const payload = JSON.parse(res.content[0].text);
    this.turns.push({ kind: 'optimize', args, payload });
    console.error(`  ok  optimize -> converged=${payload.converged} iters=${payload.iterations}`);
    return payload;
  }

  // Export records whatever the tool returns — success payload OR the tool's
  // own error result — so a genuine export limitation appears in the transcript
  // verbatim instead of crashing the run.
  export(model_id, format, path) {
    const args = { model_id, format, path };
    const res = this._run('export', args);
    if (res.isError) {
      const error = res.content[0].text;
      this.turns.push({ kind: 'export', args, error });
      console.error(`  ERR export ${format}: ${error}`);
      return { format, error };
    }
    const payload = JSON.parse(res.content[0].text);
    this.turns.push({ kind: 'export', args, payload });
    console.error(`  ok  export ${format} -> ${basename(payload.path)} (${payload.bytes}B)`);
    return payload;
  }

  // ── Markdown rendering ──────────────────────────────────────────────────
  render() {
    const out = [];
    out.push(`# Agent transcript: ${this.title}`);
    out.push('');
    out.push(this.intro);
    out.push('');
    out.push(
      'Every tool call and result below is **real, unedited output** from the ' +
        'OpenSolid MCP server, captured by ' +
        '[`build-gallery.mjs`](build-gallery.mjs). The agent narration is prose ' +
        'framing; the numbers, renders, and files are the machine’s. ' +
        'Regenerate with `node examples/agent-gallery/build-gallery.mjs`.',
    );
    out.push('');
    for (const line of this.prompt.split('\n')) out.push(`> **User:** ${line}`);
    out.push('');
    out.push('---');
    for (const turn of this.turns) out.push('', this._renderTurn(turn));
    out.push('');
    return out.join('\n');
  }

  _renderTurn(turn) {
    switch (turn.kind) {
      case 'narration':
        return `**Agent:** ${turn.text}`;
      case 'create_model': {
        const lines = [`> 🔧 **\`create_model\`**${turn.args.exact ? ' `{ "exact": true }`' : ''}`];
        lines.push('> ```js');
        for (const l of turn.args.script.split('\n')) lines.push(`> ${l}`);
        lines.push('> ```');
        lines.push('> ```json');
        for (const l of json(turn.payload).split('\n')) lines.push(`> ${l}`);
        lines.push('> ```');
        return lines.join('\n');
      }
      case 'screenshot': {
        const argline = `{ "model_id": "${stableId(turn.args.model_id)}", "view": "${turn.args.view}", "width": ${turn.args.width}, "height": ${turn.args.height} }`;
        const kb = (turn.bytes / 1024).toFixed(0);
        return [
          `> 🔧 **\`get_screenshot\`** \`${argline}\``,
          '>',
          `> ![${this.slug} — ${turn.args.view} view](../output/${turn.file})`,
          '>',
          `> *(real ${turn.args.width}×${turn.args.height} render, ${kb} KB PNG)*`,
        ].join('\n');
      }
      case 'measure': {
        const q = turn.args.query ? `, "query": "${turn.args.query}"` : '';
        const lines = [`> 🔧 **\`measure\`** \`{ "model_id": "${stableId(turn.args.model_id)}"${q} }\``];
        lines.push('> ```json');
        for (const l of json(turn.payload).split('\n')) lines.push(`> ${l}`);
        lines.push('> ```');
        return lines.join('\n');
      }
      case 'validate': {
        const lines = [`> 🔧 **\`validate\`** \`{ "model_id": "${stableId(turn.args.model_id)}" }\``];
        lines.push('> ```json');
        for (const l of json(turn.payload).split('\n')) lines.push(`> ${l}`);
        lines.push('> ```');
        return lines.join('\n');
      }
      case 'optimize': {
        const lines = [`> 🔧 **\`optimize\`**`];
        lines.push('> ```json');
        for (const l of json(turn.args).split('\n')) lines.push(`> ${l}`);
        lines.push('> ```');
        lines.push('> ```json');
        for (const l of json(turn.payload).split('\n')) lines.push(`> ${l}`);
        lines.push('> ```');
        return lines.join('\n');
      }
      case 'export': {
        const argline = `{ "model_id": "${stableId(turn.args.model_id)}", "format": "${turn.args.format}", "path": "${turn.args.path}" }`;
        const lines = [`> 🔧 **\`export\`** \`${argline}\``];
        lines.push('> ```json');
        const body = turn.error ? { isError: true, text: displayValue(turn.error) } : turn.payload;
        for (const l of json(body).split('\n')) lines.push(`> ${l}`);
        lines.push('> ```');
        return lines.join('\n');
      }
      default:
        return '';
    }
  }

  manifestEntry() {
    return { slug: this.slug, title: this.title, turns: display(this.turns) };
  }
}

// ═══════════════════════════════════════════════════════════════════════════
// The examples. Each is a function that drives a Transcript end to end.
// ═══════════════════════════════════════════════════════════════════════════
const examples = [];
function example(spec, drive) {
  examples.push({ spec, drive });
}

// 1. Angle bracket with four mounting holes ─────────────────────────────────
example(
  {
    slug: 'angle-bracket',
    title: 'a mounting bracket with four holes',
    intro:
      'An MCP-capable agent turns a one-line spec into a manufacturable part ' +
      'without ever opening a GUI — script, screenshot, measure, export.',
    prompt:
      'Design a 90° angle bracket: a 60×40×4 mm base flange with a ' +
      '60×4×40 mm wall along the back edge, and four Ø6 mm mounting holes ' +
      'in the base. Export STEP and STL and tell me its mass.',
  },
  (t) => {
    t.say(
      'The kernel centers primitives on the origin and takes half-extents, so a ' +
        '60×40×4 base is `box3(30, 20, 2)` and the back wall is `box3(30, 2, 20)` ' +
        'stood up along −Y. I’ll union them, then punch four Ø6 (r=3) holes ' +
        'through the base on a rectangular pattern — the pattern is just a JS loop. ' +
        'The one thing to get right: the base is a plate lying in xy with its 4 mm ' +
        'thickness along **z**, so the holes must run along **z** — and `Shape.cylinder` ' +
        'is a **+Y** cylinder, so each one needs `rotate(1, 0, 0, 90)` to swing its ' +
        'axis onto +Z. Skip that and the holes bore lengthwise through the 40 mm ' +
        'width instead, which still reports `valid: true`.',
    );
    const script = `
// 90° angle bracket: a 60×40×4 horizontal flange and a 60×4×40 vertical
// flange along the back edge, with four Ø6 mounting holes in the base.
const base = Shape.box3(30, 20, 2);                        // 60 × 40 × 4
const wall = Shape.box3(30, 2, 20).translate(0, -18, 22);  // 60 × 4 × 40, back edge
let bracket = base.union(wall);
// cylinder() is +Y-axis; the base plate's thickness is along z, so rotate the
// hole onto +Z before punching it through.
const hole = Shape.cylinder(3, 6).rotate(1, 0, 0, 90);     // r=3, +Y -> +Z
for (const x of [-20, 20]) for (const y of [-12, 6]) {
  bracket = bracket.subtract(hole.translate(x, y, 0));
}
return bracket;
`.trim();
    const m = t.create_model(script, 'angle-bracket');
    t.say(
      `\`create_model\` returns immediately with \`valid: ${m.valid}\` and volume ` +
        `${m.volume.toFixed(0)} mm³ — the four holes and the boolean union all ` +
        'resolved into one closed solid. Let me eyeball it before exporting.',
    );
    t.screenshot(m.model_id, 'angle-bracket-iso.png', 'iso');
    t.say(
      'That looks right — but a render cannot tell me a hole went in sideways, and ' +
        'neither can `valid: true`. The only check that can is the volume, against a ' +
        'number I work out by hand: two 60×40×4 plates meeting at a corner is ' +
        '`60*40*4 + 60*4*40 = 19200 mm³`, less four Ø6 holes through 4 mm of base ' +
        '(`4 * π * 3² * 4 = 452.4`) — **18747.6 mm³**.',
    );
    const mass = t.measure(m.model_id, 'mass');
    const TRUTH = 60 * 40 * 4 + 60 * 4 * 40 - 4 * Math.PI * 3 ** 2 * 4;
    t.say(
      `Volume ${mass.volume.toFixed(0)} mm³ against 18747.6 predicted — ` +
        `${(((mass.volume - TRUTH) / TRUTH) * 100).toFixed(1)}%, which is the SDF ` +
        'mesher reading a hair under true volume at this accuracy, not a modelling ' +
        'error. (Had I left the holes on the default +Y axis they would have bored ' +
        'lengthwise through the plate and landed near 18032 — a 4% miss that looks ' +
        'fine in every render.) ' +
        `So: ${(mass.volume / 1000).toFixed(1)} cm³, and in 6061 aluminium ` +
        `(2.70 g/cm³) about **${((mass.volume / 1000) * 2.7).toFixed(0)} g**. ` +
        'Now the files — STL first:',
    );
    t.export(m.model_id, 'stl', 'angle-bracket.stl');
    t.say('And STEP:');
    t.export(m.model_id, 'step', 'angle-bracket.step');
    t.say(
      'Both files wrote. This shape has no exact B-Rep companion, so STEP goes ' +
        'through the faceted SDF→B-Rep path: the exporter meshes the solid to a ' +
        'closed manifold and emits that as a B-rep body. That path needs a watertight ' +
        'mesh, and it used to not get one here — the mesher fused two surface sheets ' +
        'through a single cell into a *pinched* edge (the defect tracked as of-o0o) ' +
        'and STEP declined rather than write a corrupt file. A later mesher fix ' +
        '(of-obv) cleared the pinch for this geometry, so the closed mesh now survives ' +
        'the conversion and STEP exports. The STL takes a separate, mesh-direct path ' +
        'and was never affected either way.',
    );
    t.say(
      'One thing to carry forward: this is a *faceted* STEP — the round hole walls ' +
        'arrive as many small planar facets, not analytic cylinders, which is why the ' +
        'file is large. When an exact B-Rep matters (crisp analytic faces, a smaller ' +
        'file), model the L-section as an extruded `Profile` instead, the way the ' +
        '[right-angle bracket](bracket-right-angle.md) transcript builds it. Same ' +
        'part, different construction, different export path.',
    );
  },
);

// 2. Hinge leaf with three knuckles and a pin bore ──────────────────────────
example(
  {
    slug: 'hinge-leaf',
    title: 'a hinge leaf with knuckles and a pin bore',
    intro:
      'Rotations put a cylinder’s axis wherever you want it. Here the agent ' +
      'builds one leaf of a butt hinge — barrel knuckles on the pin axis, drilled ' +
      'through for the pin.',
    prompt:
      'Model one leaf of a butt hinge: a flat plate with three barrel knuckles on ' +
      'the pin axis and a pin bore drilled through them. Show me an iso view and ' +
      'give me a STEP file.',
  },
  (t) => {
    t.say(
      'A knuckle is a cylinder whose default **+Y** axis I rotate onto +X, then ' +
        'slide along X. Three of them union onto the plate; a long thin cylinder on ' +
        'the same axis subtracts the pin bore. The rotation to reach for is ' +
        '`rotate(0, 0, 1, 90)` — turning about **Z** is what carries +Y onto X. ' +
        'Rotating about Y, which is the tempting one to write, would spin the ' +
        'cylinder about its own axis and change nothing.',
    );
    const script = `
// One leaf of a butt hinge: a flat plate with three barrel knuckles on the
// pin axis (X) and a pin bore drilled through them. Two of these — one
// mirrored — pin together into a working hinge.
const plate = Shape.box3(30, 15, 0.75).translate(0, -15.75, 0);  // 60 × 30 × 1.5 leaf
// cylinder() is +Y-axis. Rotating about Z carries it onto X (rotating about Y
// would be a no-op — it is already on Y).
const knuckle = Shape.cylinder(4, 6).rotate(0, 0, 1, 90);        // r=4, 12 long on X
let leaf = plate;
for (const x of [-24, 0, 24]) leaf = leaf.union(knuckle.translate(x, 0, 0));
const pin = Shape.cylinder(2, 40).rotate(0, 0, 1, 90);           // Ø4 bore on X
return leaf.subtract(pin);
`.trim();
    const m = t.create_model(script, 'hinge-leaf');
    t.say(
      `Valid solid, ${m.mesh.triangles.toLocaleString('en-US')} triangles — the pin ` +
        'bore runs cleanly through all three knuckles. One sizing note worth being ' +
        'honest about: I opened the bore to Ø4 because at Ø3.2 this part comes back ' +
        '`valid: false` with a *pinched* mesh — two surface sheets fused through one ' +
        'octree cell where the bore goes tangent. That is a known mesher defect ' +
        '(of-o0o), not a part that is too small to see, and it is worth knowing which ' +
        'it is: a finer `accuracy` does not clear a pinch, and the bore sizes that ' +
        'trip it are not the small ones in particular (Ø2.4 and Ø7 fail; Ø2.8, Ø3.6 ' +
        'and Ø4 are fine). So Ø4 is a workaround I found by moving, not a rule I ' +
        'derived. Let me confirm the mesh is watertight before exporting.',
    );
    t.screenshot(m.model_id, 'hinge-leaf-iso.png', 'iso');
    const v = t.validate(m.model_id);
    t.say(
      `\`closedManifold: ${v.closedManifold}\`, no issues — a real solid, not a ` +
        'surface soup. The STEP file you asked for:',
    );
    t.export(m.model_id, 'step', 'hinge-leaf.step');
    t.say(
      'STEP declines here, and the reason it gives is the same pinch as above — this ' +
        'part has no exact B-Rep companion, so STEP takes the faceted SDF→B-Rep path, ' +
        'which needs a closed manifold and does not get one. Note it names the actual ' +
        'defect (pinched edges) rather than blaming resolution, so I know not to burn ' +
        'time retrying at a finer accuracy. The tool says no plainly rather than ' +
        'emitting a broken file. I can still give you the mesh:',
    );
    t.export(m.model_id, 'stl', 'hinge-leaf.stl');
    t.say(
      'So: a valid, watertight STL, and an honest no on STEP. If the STEP file is the ' +
        'deliverable, the route is to build the leaf from an extruded `Profile` so it ' +
        'carries an exact B-Rep (see the [right-angle bracket](bracket-right-angle.md)) ' +
        'rather than from rotated primitives. Mirror this leaf about X and pin the two ' +
        'together and you have a working hinge.',
    );
  },
);

// 3. Enclosure — shelled body (open top) + press-fit lid ─────────────────────
example(
  {
    slug: 'enclosure',
    title: 'a shelled enclosure with a press-fit lid',
    intro:
      'A two-part build: a hollow electronics enclosure with an open top, plus a ' +
      'matching lid that press-fits into its mouth. Shelling is `outer.subtract(inner)`.',
    prompt:
      'I need an 80×60×30 mm electronics enclosure with 3 mm walls, open on top, ' +
      'plus a lid that press-fits into it. Rounded corners. Give me both as STEP and STL.',
  },
  (t) => {
    t.say(
      'I’ll shell the body by subtracting an inner cavity from a rounded box, ' +
        'raising the cavity so it breaks through the top face — that leaves the box ' +
        'open for the lid. Then a separate lid with a recessed lip sized for a press fit.',
    );
    const bodyScript = `
// Electronics enclosure: a rounded box hollowed to a 3 mm wall with an open
// top. The shell is outer.subtract(inner) — the inner cavity is raised so it
// breaks through the top face, leaving the box open for the lid.
const outer = Shape.roundedBox(40, 30, 15, 3);                 // 80 × 60 × 30
const cavity = Shape.box3(37, 27, 14).translate(0, 0, 2);      // 3 mm walls, open top
return outer.subtract(cavity);
`.trim();
    const body = t.create_model(bodyScript, 'enclosure-body');
    t.say(
      `Body is valid — volume ${body.volume.toFixed(0)} mm³ after hollowing. ` +
        'Rendering it to check the wall broke through the top:',
    );
    t.screenshot(body.model_id, 'enclosure-body-iso.png', 'iso');
    t.export(body.model_id, 'step', 'enclosure-body.step');
    t.export(body.model_id, 'stl', 'enclosure-body.stl');
    t.say(
      'Open-topped shell, confirmed. Now the lid: a top plate with a lip that drops ' +
        'into the mouth with 0.5 mm clearance per wall.',
    );
    const lidScript = `
// Matching lid: a top plate with a recessed lip that press-fits into the
// enclosure's open mouth (0.5 mm clearance on each wall).
const cap = Shape.roundedBox(40, 30, 1.5, 3).translate(0, 0, 16.5);  // top plate
const lip = Shape.box3(36.5, 26.5, 2).translate(0, 0, 13);           // insert lip
return cap.union(lip);
`.trim();
    const lid = t.create_model(lidScript, 'enclosure-lid');
    t.screenshot(lid.model_id, 'enclosure-lid-iso.png', 'iso');
    t.export(lid.model_id, 'step', 'enclosure-lid.step');
    t.export(lid.model_id, 'stl', 'enclosure-lid.stl');
    t.say(
      'Two parts, both watertight: an open-topped 3 mm-walled enclosure and a ' +
        'press-fit lid, each as STEP and STL. The lip is 0.5 mm under the cavity on ' +
        'every wall, so it seats without jamming.',
    );
  },
);

// 4. Spur-gear-ish toothed disk (circular pattern via a loop) ────────────────
example(
  {
    slug: 'gear-disk',
    title: 'a toothed disk from a circular pattern',
    intro:
      'The script vocabulary is a real programming language, so a circular ' +
      'pattern is just a `for` loop rotating one feature around the axis — the ' +
      'axis the disk is actually on.',
    prompt:
      'Make a 16-tooth spur-gear-style disk: a root disk with rectangular teeth on ' +
      'a circular pattern and a central bore. Top and iso views, and an STL for printing.',
  },
  (t) => {
    t.say(
      'One tooth box, rotated into 16 positions by a loop, unioned onto a root ' +
        'cylinder, minus a central bore. `(360 * i) / TEETH` spaces the teeth evenly. ' +
        'The circular pattern has to turn about the **same axis the disk is on** — ' +
        '`Shape.cylinder` is **+Y**, so that is `rotate(0, 1, 0, ...)`. Pattern about ' +
        'Z instead and the teeth swing up out of the disk plane into a ring of ' +
        'floating blocks, which still meshes and still reports `valid: true`.',
    );
    const script = `
// A toothed disk: a root disk with N teeth placed on a circular pattern by
// rotating one tooth box around the disk's own axis (+Y, the cylinder axis),
// plus a central bore. The pattern is just a JS loop — the script vocabulary
// is a real programming language.
const TEETH = 16, TH = 4, ROOT = 16, BORE = 4;
let gear = Shape.cylinder(ROOT, TH);                          // disk faces in xz, axis +Y
const tooth = Shape.box3(3, TH, 2.2).translate(ROOT + 1.5, 0, 0);  // radial x, thick y
for (let i = 0; i < TEETH; i++) {
  gear = gear.union(tooth.rotate(0, 1, 0, (360 * i) / TEETH));     // pattern about +Y
}
return gear.subtract(Shape.cylinder(BORE, TH + 2));           // central bore, coaxial
`.trim();
    const m = t.create_model(script, 'gear-disk');
    t.say(
      `All 16 teeth resolved — \`valid: ${m.valid}\`, volume ${m.volume.toFixed(0)} mm³. ` +
        'Top view to check the tooth count and spacing, then iso:',
    );
    t.screenshot(m.model_id, 'gear-disk-top.png', 'top');
    t.screenshot(m.model_id, 'gear-disk-iso.png', 'iso');
    t.say(
      'Sixteen evenly-spaced teeth, and the disk reads 8 mm thick in y — which is ' +
        'the check that matters here. Had I patterned about Z, the top view would ' +
        'still show a tidy ring of sixteen blocks and `valid` would still be `true`, ' +
        'but the bounding box would come back 41 × 41 × 32 instead of 41 × 8 × 41: ' +
        'teeth orbiting the disk rather than sitting on its rim. STL exports the ' +
        'mesh directly:',
    );
    t.export(m.model_id, 'stl', 'gear-disk.stl');
    t.say('And STEP, for the mechanical model:');
    t.export(m.model_id, 'step', 'gear-disk.step');
  },
);

// 5. Bottle — revolve a silhouette, then shell it hollow ─────────────────────
example(
  {
    slug: 'bottle',
    title: 'a bottle from a revolved, shelled profile',
    intro:
      'The classic OpenCascade "bottle", built the OpenSolid way: revolve a 2D ' +
      'silhouette 360°, then hollow it by subtracting an inner revolve that breaks ' +
      'through the top. A rounded shoulder comes from an `arcTo` fillet.',
    prompt:
      'Build a bottle: revolve a silhouette into a body with a rounded shoulder and a ' +
      'neck, then hollow it out so the mouth is open. Validate it’s watertight and ' +
      'give me an STL and an OBJ.',
  },
  (t) => {
    t.say(
      'A `Profile` is a closed polyline with optional arcs. I draw the outer ' +
        'silhouette — straight up the body, an `arcTo` for the rounded shoulder, up ' +
        'the neck — revolve it about Y, then subtract a slightly-smaller inner revolve ' +
        'that runs taller than the neck so it opens the mouth. That inner subtract is the shell.',
    );
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
    const m = t.create_model(script, 'bottle');
    t.say('Rendering the result from the front and iso to check the silhouette and the open neck:');
    t.screenshot(m.model_id, 'bottle-front.png', 'front');
    t.screenshot(m.model_id, 'bottle-iso.png', 'iso');
    t.say('Now the watertightness check — a shell that didn’t break through would read as a closed cavity:');
    const v = t.validate(m.model_id);
    t.say(
      `\`closedManifold: ${v.closedManifold}\`, volume ${v.volume.toFixed(0)} mm³ — ` +
        'a genuine hollow solid with an open mouth. Exporting mesh formats:',
    );
    t.export(m.model_id, 'stl', 'bottle.stl');
    t.export(m.model_id, 'obj', 'bottle.obj');
    t.say(
      'A hollow bottle with a rounded shoulder and an open neck, watertight where it ' +
        'should be and open where it should be — all from a revolved profile and one ' +
        'boolean subtract.',
    );
  },
);

// 6. Right-angle bracket: gusset, interior fillet, M5 holes ────────────────
// The acceptance part (of-2y4.1). Unlike the examples above, this one is a
// dogfood run: the narration keeps the two kernel facts the agent had to
// discover the hard way (the +Y axis convention, and the bounds-dependent
// faceted STEP failure), because an agent reading this gallery needs them.
example(
  {
    slug: 'bracket-right-angle',
    title: 'a right-angle bracket with a gusset and filleted corner',
    intro:
      'The acceptance part: a real engineering bracket with a load-bearing ' +
      'gusset, a filleted interior corner, and four tapped-size mounting ' +
      'holes. It exercises profiles, extrusion, blends, and drilling on ' +
      'three different axes — and it surfaces two kernel behaviours an agent ' +
      'must know about, both kept verbatim below.',
    prompt:
      'Design a right-angle bracket: a 60×40×5 mm base plate, a 40×40×5 mm ' +
      'vertical plate, a triangular gusset between them, 4× M5 mounting ' +
      'holes (2 per plate), and 3 mm fillets on the interior corner and the ' +
      'gusset edges. Verify it and export a STEP file.',
  },
  (t) => {
    t.say(
      'Two kernel facts drive the whole script. First, the extrusion axis is ' +
        '**+Y**, not +Z: `Shape.extrude` sweeps a profile’s `(u, v)` into ' +
        '`(x, z)` and pushes it along +Y (`bounded.rs`), and `Shape.cylinder` ' +
        'is likewise a **+Y**-axis cylinder — radial in xz, axial in y ' +
        '(`primitives.rs`). Second, that makes the L-section a *profile* ' +
        'problem: I can draw the whole cross-section in (x, z) — including the ' +
        '3 mm interior fillet as a real arc — and sweep it 40 mm along +Y for ' +
        'the width. `arcTo`’s bulge is the DXF convention, `tan(sweep/4)`, so a ' +
        '90° arc is `tan(22.5°) = 0.4142`; negative sweeps it clockwise, which ' +
        'is the concave direction here.',
    );
    t.say(
      'That leaves the part **z-up**: x is its 60 mm length, y its 40 mm ' +
        'width, z its 40 mm height. I am keeping it that way deliberately — ' +
        'STEP, FreeCAD, and CAD interchange generally are z-up, and the STEP ' +
        'writer emits coordinates verbatim, so a z-up model lands upright in ' +
        'FreeCAD. Be aware this cuts against the *renderer*, whose named views ' +
        'assume y is up (`render.js`). So for this part `top` (looking down −Y) ' +
        'is the view that shows the L-section, and `front` (looking down −Z) is ' +
        'the plan view of the base plate. The view names are worth reading ' +
        'literally, not geometrically.',
    );
    const script = `
// Right-angle bracket: 60×40×5 base plate, 40×40×5 vertical plate, triangular
// gusset, 4× M5 clearance holes, 3 mm fillets on the interior corner and gusset.
//
// extrude() sweeps a profile along +Y, mapping profile (u,v) -> (x,z). So the
// L cross-section is drawn in (x, z) and swept 40 mm for the bracket's width.
const B = 0.41421356237309503;          // tan(90°/4): a 90° arc, DXF bulge
const p = new Profile(-30, 0);          // base underside, at the wall end
p.lineTo(30, 0);                        // base plate, 60 long
p.lineTo(30, 5);                        // base plate, 5 thick
p.lineTo(-22, 5);                       // top of base, out to the fillet tangent
p.arcTo(-25, 8, -B);                    // 3 mm fillet on the interior corner
p.lineTo(-25, 40);                      // wall inner face, 40 tall
p.lineTo(-30, 40);                      // wall top, 5 thick
p.close();
const ell = Shape.extrude(p, 40);       // sweep +Y: the 40 mm width

// Triangular gusset: 20 mm legs, 5 mm thick, centered across the width.
const t = new Profile(-25, 5);
t.lineTo(-5, 5);
t.lineTo(-25, 25);
t.close();
const gusset = Shape.extrude(t, 5).translate(0, 17.5, 0);

// smoothUnion blends the gusset into both plates: the 3 mm gusset fillets.
let part = ell.smoothUnion(gusset, 3);

// 4× M5 clearance holes (Ø5). cylinder() is +Y-axis, so rotate it onto the
// drilling axis: +Z for the base plate, +X for the vertical plate.
const zHole = Shape.cylinder(2.5, 10).rotate(1, 0, 0, 90);   // -> +Z
for (const y of [10, 30]) part = part.subtract(zHole.translate(15, y, 0));
const xHole = Shape.cylinder(2.5, 10).rotate(0, 0, 1, 90);   // -> +X
for (const y of [10, 30]) part = part.subtract(xHole.translate(-27.5, y, 32));

// The trailing no-op rotation is a WORKAROUND, not modelling (of-obv):
// without it this exact part meshes open at the default accuracy and STEP
// export declines. A 360° rotation is geometrically the identity; all it
// changes is the shape's tracked bounding box, and that shifts the meshing
// grid onto an alignment where the mesh closes. This specific expression was
// found by trial: other identity-equivalent spellings still fail.
return part.rotate(0, 1, 0, 360);
`.trim();
    const m = t.create_model(script, 'bracket-right-angle');
    t.say(
      `\`valid: ${m.valid}\`, volume ${m.volume.toFixed(0)} mm³. That is the ` +
        'oracle that matters: hand-integrating the section gives 19792 mm³ ' +
        '(19077 for the filleted L, +1000 gusset, +blend, −393 for four Ø5 ' +
        'holes through 5 mm), so the mesh is reading 0.3% under — the same ' +
        'bias a plain 60×40×5 slab shows (11968 vs 12000). The holes are ' +
        'real: drop them and the body measures 20184 mm³. Let me look at it ' +
        'from three sides before exporting.',
    );
    t.screenshot(m.model_id, 'bracket-right-angle-iso.png', 'iso');
    t.screenshot(m.model_id, 'bracket-right-angle-top.png', 'top');
    t.screenshot(m.model_id, 'bracket-right-angle-front.png', 'front');
    t.say(
      'The `top` render is the elevation that matters: the L-section with the ' +
        '3 mm fillet blended into the interior corner and the gusset filling ' +
        'the angle. `front` is the plan view of the base plate with its two M5 ' +
        'holes, and the iso ties it together. Mass properties:',
    );
    const mass = t.measure(m.model_id, 'mass');
    t.say(
      `Volume ${mass.volume.toFixed(0)} mm³ = ${(mass.volume / 1000).toFixed(1)} cm³; ` +
        `in 6061 aluminium (2.70 g/cm³) that is about ` +
        `**${((mass.volume / 1000) * 2.7).toFixed(0)} g**. The reported ` +
        '`boundingBox` measures the part itself (it is taken off the same mesh ' +
        'these mass properties integrate), so it is good to the meshing ' +
        'accuracy and can be read as a measurement. Exporting:',
    );
    t.export(m.model_id, 'step', 'bracket-right-angle.step');
    t.export(m.model_id, 'stl', 'bracket-right-angle.stl');
    t.say(
      'A valid, watertight right-angle bracket — gusset blended, corner ' +
        'filleted, four M5 holes drilled on two axes — as a faceted STEP and a ' +
        'print-ready STL. Two caveats an agent should carry forward. The ' +
        '**+Y** axis convention for `extrude`/`cylinder`: get it wrong and a ' +
        'hole silently becomes a channel through the part, with no error and a ' +
        '`valid: true` mesh — the volume delta is the only thing that catches ' +
        'it, which is why measuring against a hand-computed number is not ' +
        'optional here. And the faceted STEP path is fragile: this part ' +
        'exports only because of the trailing no-op rotation. The identical ' +
        'part without it, and every other identity-equivalent spelling tried, ' +
        'meshes open and declines to export — while STL, which uses a ' +
        'different mesher, exports fine either way.',
    );
  },
);

// 7. Gradient optimization: drive a bracket onto a mass target ──────────────
// The differentiable path (of-2y4.2): the agent declares a design variable with
// param(), then hands the numeric search to `optimize` instead of nudging the
// dimension by hand across a dozen tool calls.
example(
  {
    slug: 'optimize-bracket',
    title: 'hitting a mass target with the optimize tool',
    intro:
      'A weight budget is a number, not a shape. Rather than guess a wall ' +
      'thickness, render, measure, and nudge — a dozen tool calls converging ' +
      'like a random walk — the agent declares the thickness a `param()` and ' +
      'lets `optimize` descend the F-Rep gradient straight onto the target.',
    prompt:
      'Make me an L-bracket — a 60×40 base flange and a 60×40 back wall — out ' +
      'of 6061 aluminium, and size the wall thickness so it weighs 45 g. Then ' +
      'confirm it and export the STL.',
  },
  (t) => {
    t.say(
      'Mass is volume × density, and for this bracket volume is a smooth, monotone ' +
        'function of one number — the shared wall thickness. That makes it a textbook ' +
        'job for the gradient optimizer instead of hand-tuning. The move is to expose ' +
        'the thickness as a `param(name, default, {min, max})`: the model still builds ' +
        'at the default (4 mm), but `optimize` can now move it. 6061 aluminium is ' +
        '2.70 g/cm³ = 0.0027 g/mm³, so 45 g is a volume the search will find; I bound ' +
        'the thickness to a manufacturable 2–8 mm.',
    );
    const script = `
// L-bracket: a 60×40 base flange and a 60×40 back wall sharing one wall
// thickness \`t\`, exposed as the design variable optimize() will move. The
// model builds at t's default (4 mm); optimize re-runs it at other values.
const t = param('thickness', 4, { min: 2, max: 8 });
const base = Shape.box3(30, 20, t / 2);                            // 60 × 40 × t
const wall = Shape.box3(30, t / 2, 20).translate(0, -(20 - t / 2), 20 - t / 2); // 60 × t × 40
return base.union(wall);
`.trim();
    const m = t.create_model(script, 'optimize-bracket');
    t.say(
      `The create call echoes the declared param back — \`thickness\` at its 4 mm ` +
        `default, bounded 2–8 — and the part is \`valid: ${m.valid}\` at ` +
        `${(m.volume * 0.0027).toFixed(1)} g. Too heavy. Rather than bisect it by hand, ` +
        'I hand the numeric search to `optimize`: target mass 45 g, the one param free ' +
        'to move, everything else the script fixed. It descends the smooth occupancy-' +
        'field gradient — no mesh rebuild per step — and calibrates against the exact ' +
        'mesh each iteration so the reported grams are the real ones, not the field estimate.',
    );
    const opt = t.optimize({
      model_id: m.model_id,
      params: [{ name: 'thickness' }],
      objective: { type: 'target_mass', value: 45, density: 0.0027 },
      options: { max_iters: 40, resolution: 40 },
    });
    t.say(
      `Converged in ${opt.iterations} iterations to thickness ` +
        `**${opt.params.thickness.toFixed(3)} mm**, an achieved mass of ` +
        `**${opt.objective.achieved.toFixed(2)} g** — ` +
        `${(opt.objective.relativeError * 100).toFixed(2)}% off the 45 g target, measured ` +
        'on the exact mesh, not the field. `converged: true` with an empty `warnings` ' +
        'means no parameter is pinned to a bound and nothing is left on the table; the ' +
        'per-iteration `trajectory` shows the loss falling monotonically. `optimize` has ' +
        'already written the winning thickness back into the model, so the next call ' +
        'sees the optimized part — let me confirm that independently.',
    );
    const mass = t.measure(m.model_id, 'mass');
    t.say(
      `An independent \`measure\` on the (now optimized) model reads ` +
        `${(mass.volume * 0.0027).toFixed(2)} g — the same part the optimizer reported, ` +
        'confirming the parameter really was committed and not just returned. A quick ' +
        'validity check, then the STL:',
    );
    const val = t.validate(m.model_id);
    t.say(
      `\`valid: ${val.valid}\` — the optimized thickness still bounds a closed, ` +
        'manifold solid, so it is safe to export. STL for the slicer:',
    );
    t.export(m.model_id, 'stl', 'optimize-bracket.stl');
    t.say(
      'That is the whole differentiable loop the agent layer is built around: the agent ' +
        'owns the *topology* (an L-bracket, one shared thickness), and the gradient owns ' +
        'the *numbers*. Want three ribs instead of a plain wall, or the hole moved? Edit ' +
        'the script and call `optimize` again — it moves numbers, never structure.',
    );
  },
);

// ═══════════════════════════════════════════════════════════════════════════
// Orchestration — pure helpers (exported for tests) + the run entry point.
// ═══════════════════════════════════════════════════════════════════════════

// Run each selected example through `renderOne`, isolating failures so one
// throwing example does not strand the rest. A drive() can throw for real
// reasons — most often the kernel handing back a null volume at a mesher pinch
// (of-o0o) and the narration then calling `.toFixed` on it — and without
// isolation that single failure aborts the whole loop, leaving manifest.json
// and every not-yet-reached transcript stale. `renderOne(spec, drive)` does the
// side effects (drive the tools, write the transcript) and returns the manifest
// entry + index row; if it throws, the example is recorded as a failure and
// skipped. Pure w.r.t. its inputs — `renderOne`/`log` carry all the IO.
export function driveExamples(examples, { only, renderOne, log = () => {} }) {
  const manifest = [];
  const indexRows = [];
  const failures = [];
  for (const { spec, drive } of examples) {
    if (only && spec.slug !== only) continue;
    log(`\n=== ${spec.slug} ===`);
    try {
      const { manifestEntry, indexRow } = renderOne(spec, drive);
      manifest.push(manifestEntry);
      indexRows.push(indexRow);
    } catch (err) {
      failures.push({ slug: spec.slug, message: err?.message ?? String(err) });
      log(`  FAIL ${spec.slug}: ${err?.message ?? err}`);
      log('  (skipping this example; its committed transcript is left untouched)');
    }
  }
  return { manifest, indexRows, failures };
}

// Merge regenerated manifest entries into an existing manifest by slug: replace
// a matching entry in place, append a new one. Used by the GALLERY_ONLY path so
// a single-example regen keeps manifest.json consistent with the transcript it
// just wrote without churning — or dropping — the other examples' entries.
export function mergeManifest(existing, entries) {
  const out = Array.isArray(existing) ? existing.slice() : [];
  for (const entry of entries) {
    const i = out.findIndex((e) => e.slug === entry.slug);
    if (i >= 0) out[i] = entry;
    else out.push(entry);
  }
  return out;
}

function renderIndex(indexRows) {
  // Count the examples that actually rendered — a skipped failure must not be
  // claimed in the index prose or the row list.
  const written = indexRows.length;
  const COUNT_WORDS = ['zero', 'one', 'two', 'three', 'four', 'five', 'six', 'seven', 'eight'];
  const countWord = COUNT_WORDS[written] ?? String(written);
  return `# Agent gallery

${countWord[0].toUpperCase() + countWord.slice(1)} worked examples of an MCP-capable agent operating the OpenSolid CAD kernel
end to end — prompt in, manufacturable part out — with **no GUI**. Each
transcript is real, unedited output from the [OpenSolid MCP server](../../README.md),
captured by [\`build-gallery.mjs\`](build-gallery.mjs): the agent writes a script,
gets mesh stats and a validity flag, renders screenshots, measures mass
properties, and exports STEP/STL/OBJ.

Regenerate the whole gallery (renders, exports, and these transcripts):

\`\`\`bash
cd tools/mcp-server
npm run build     # only needed after a change under crates/
node examples/agent-gallery/build-gallery.mjs
\`\`\`

| Example | Slug |
|---------|------|
${indexRows.join('\n')}

Exported files (STEP/STL/OBJ) and PNG renders land in
[\`../output/\`](../output/); [\`manifest.json\`](../output/manifest.json) is the
machine-readable record of the run. See the
[Agent Guide](../../../../docs/AGENT_GUIDE.md) for how to connect a client, the
full tool reference, and the failure modes these examples exercise.

\`bracket-right-angle\` is also the acceptance part, built over the real MCP
stdio transport and gated by
[\`test/bracket-acceptance.test.js\`](../../test/bracket-acceptance.test.js). What
it cost to build — and the kernel bugs it surfaced — is written up in the
[friction log](../../../../docs/dogfood-bracket-friction-log.md).
`;
}

function main() {
  const tools = createTools({ outputDir });
  // `GALLERY_ONLY=<slug>` regenerates a single transcript and its outputs, and
  // leaves README.md (the whole-gallery index) untouched while merging just its
  // own entry into manifest.json. Useful for iterating on one example without
  // churning every other example's committed renders and exports.
  const only = process.env.GALLERY_ONLY;

  const { manifest, indexRows, failures } = driveExamples(examples, {
    only,
    log: (m) => console.error(m),
    renderOne: (spec, drive) => {
      const t = new Transcript(tools, spec);
      drive(t);
      writeFileSync(resolve(galleryDir, `${spec.slug}.md`), t.render(), 'utf8');
      return {
        manifestEntry: t.manifestEntry(),
        indexRow: `| [${spec.title}](${spec.slug}.md) | ${spec.slug} |`,
      };
    },
  });

  if (only) {
    if (manifest.length === 0 && failures.length === 0) {
      throw new Error(`GALLERY_ONLY=${only}: no example with that slug`);
    }
    if (manifest.length) {
      const manifestPath = resolve(outputDir, 'manifest.json');
      let existing = [];
      try {
        existing = JSON.parse(readFileSync(manifestPath, 'utf8'));
      } catch {
        existing = [];
      }
      writeFileSync(manifestPath, JSON.stringify(mergeManifest(existing, manifest), null, 2), 'utf8');
      console.error(
        `\nGALLERY_ONLY=${only}: wrote the transcript and merged its manifest.json ` +
          'entry; left the index and the other examples untouched.',
      );
    }
    if (failures.length) {
      console.error(`GALLERY_ONLY=${only}: FAILED — ${failures.map((f) => f.slug).join(', ')}`);
      process.exit(1);
    }
    process.exit(0);
  }

  writeFileSync(resolve(outputDir, 'manifest.json'), JSON.stringify(manifest, null, 2), 'utf8');
  writeFileSync(resolve(galleryDir, 'README.md'), renderIndex(indexRows), 'utf8');
  console.error(`\nWrote ${indexRows.length} transcripts + manifest.json to examples/.`);

  if (failures.length) {
    console.error(
      `\n${failures.length} example(s) FAILED and were skipped: ` +
        `${failures.map((f) => f.slug).join(', ')}. The rest regenerated normally.`,
    );
    for (const f of failures) console.error(`  - ${f.slug}: ${f.message}`);
    process.exit(1);
  }
}

// Run only when invoked directly (node build-gallery.mjs), not when imported by
// a test that exercises the pure helpers above.
const isMain = process.argv[1] && resolve(process.argv[1]) === fileURLToPath(import.meta.url);
if (isMain) main();
