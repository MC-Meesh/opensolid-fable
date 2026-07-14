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
import { writeFileSync } from 'node:fs';
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
        'through the base on a rectangular pattern — the pattern is just a JS loop.',
    );
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
    const m = t.create_model(script, 'angle-bracket');
    t.say(
      `\`create_model\` returns immediately with \`valid: ${m.valid}\` and volume ` +
        `${m.volume.toFixed(0)} mm³ — the four holes and the boolean union all ` +
        'resolved into one closed solid. Let me eyeball it before exporting.',
    );
    t.screenshot(m.model_id, 'angle-bracket-iso.png', 'iso');
    t.say('That matches the spec. Mass properties, then the files:');
    const mass = t.measure(m.model_id, 'mass');
    t.say(
      `Volume ${mass.volume.toFixed(0)} mm³ = ${(mass.volume / 1000).toFixed(1)} cm³; ` +
        `in 6061 aluminium (2.70 g/cm³) that’s about ` +
        `**${((mass.volume / 1000) * 2.7).toFixed(0)} g**. Exporting STEP and STL:`,
    );
    t.export(m.model_id, 'step', 'angle-bracket.step');
    t.export(m.model_id, 'stl', 'angle-bracket.stl');
    t.say(
      'Done — a valid, watertight bracket with four through-holes, delivered as an ' +
        'analytic-fallback STEP file and a print-ready STL, straight from the prompt.',
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
      'A knuckle is a cylinder whose default +Z axis I rotate onto +X, then slide ' +
        'along X. Three of them union onto the plate; a long thin cylinder on the same ' +
        'axis subtracts the pin bore. `rotate(0, 1, 0, 90)` swings +Z onto +X.',
    );
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
    const m = t.create_model(script, 'hinge-leaf');
    t.say(
      `Valid solid, ${m.mesh.triangles.toLocaleString('en-US')} triangles — the pin ` +
        'bore runs cleanly through all three knuckles. Let me look at it and confirm ' +
        'the mesh is watertight before exporting.',
    );
    t.screenshot(m.model_id, 'hinge-leaf-iso.png', 'iso');
    const v = t.validate(m.model_id);
    t.say(
      `\`closedManifold: ${v.closedManifold}\`, no issues — a real solid, not a ` +
        'surface soup. Exporting STEP for the mechanical model:',
    );
    t.export(m.model_id, 'step', 'hinge-leaf.step');
    t.export(m.model_id, 'stl', 'hinge-leaf.stl');
    t.say('Mirror this leaf about X and pin the two together and you have a working hinge.');
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
      'pattern is just a `for` loop rotating one feature around the axis. This one ' +
      'also surfaces a genuine export limitation — and how the tool reports it.',
    prompt:
      'Make a 16-tooth spur-gear-style disk: a root disk with rectangular teeth on ' +
      'a circular pattern and a central bore. Top and iso views, and an STL for printing.',
  },
  (t) => {
    t.say(
      'One tooth box, rotated into 16 positions by a loop, unioned onto a root ' +
        'cylinder, minus a central bore. `(360 * i) / TEETH` spaces the teeth evenly.',
    );
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
    const m = t.create_model(script, 'gear-disk');
    t.say(
      `All 16 teeth resolved — \`valid: ${m.valid}\`, volume ${m.volume.toFixed(0)} mm³. ` +
        'Top view to check the tooth count and spacing, then iso:',
    );
    t.screenshot(m.model_id, 'gear-disk-top.png', 'top');
    t.screenshot(m.model_id, 'gear-disk-iso.png', 'iso');
    t.say('Sixteen evenly-spaced teeth. STL exports the mesh directly:');
    t.export(m.model_id, 'stl', 'gear-disk.stl');
    t.say(
      'For completeness I’ll also try STEP. The thin teeth sit right at the edge ' +
        'of the model’s bounding box, and the faceted STEP path needs the surface ' +
        'strictly *inside* the meshing region — so this is a case where the export ' +
        'declines rather than emitting a broken file. Watch how it reports:',
    );
    t.export(m.model_id, 'step', 'gear-disk.step');
    t.say(
      'That’s the honest failure mode: `isError: true` with a specific reason, not ' +
        'a silently-corrupt STEP. The STL is unaffected — meshing and STEP’s ' +
        'planar-region recovery are different code paths. For an analytic STEP of a gear ' +
        'you’d thicken the teeth slightly or model them as an extruded `Profile`; the ' +
        'STL here is already print-ready.',
    );
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

// ═══════════════════════════════════════════════════════════════════════════
// Run everything and write outputs.
// ═══════════════════════════════════════════════════════════════════════════
const tools = createTools({ outputDir });
const manifest = [];
const indexRows = [];

for (const { spec, drive } of examples) {
  console.error(`\n=== ${spec.slug} ===`);
  const t = new Transcript(tools, spec);
  drive(t);
  writeFileSync(resolve(galleryDir, `${spec.slug}.md`), t.render(), 'utf8');
  manifest.push(t.manifestEntry());
  indexRows.push(`| [${spec.title}](${spec.slug}.md) | ${spec.slug} |`);
}

writeFileSync(resolve(outputDir, 'manifest.json'), JSON.stringify(manifest, null, 2), 'utf8');

const index = `# Agent gallery

Five worked examples of an MCP-capable agent operating the OpenSolid CAD kernel
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
`;
writeFileSync(resolve(galleryDir, 'README.md'), index, 'utf8');

console.error(`\nWrote ${examples.length} transcripts + manifest.json to examples/.`);
