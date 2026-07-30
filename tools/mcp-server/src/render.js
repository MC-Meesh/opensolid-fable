// Headless software rasterizer: orthographic, z-buffered, flat-shaded render
// of a triangle mesh to an RGBA framebuffer, encoded as PNG. Pure JS, no GPU
// and no headless browser — a screenshot is a few milliseconds and has no
// external dependencies.
//
// Beyond the plain shaded shot this module is the *visual inspection channel*
// (of-2y4.6): a multimodal reader can only judge geometry it can actually see,
// and a whole-model shaded thumbnail hides exactly the class of defect that
// passes every machine check — a hole drilled on the wrong axis, a pocket that
// broke through, a boss that landed 2 mm off. Three levers fix that:
//
//   * framing   — `region`/`target`/`zoom`/`direction` put a named feature in
//                 the middle of the frame at a legible size.
//   * section   — an axis-aligned clip plane with a shaded cap, so interior
//                 geometry (wall thickness, a blind hole's floor) is visible.
//   * edges     — feature + silhouette line-work with hidden-line removal,
//                 which reads like a dimension drawing rather than a blob.
//
// Every setting is an explicit input and nothing here samples a clock, a
// random source, or the iteration order of a hash: the same request against
// the same mesh renders byte-identical PNGs.

import { encodePng } from './png.js';

// Named CAD views: `dir` is the direction the camera looks (into the scene),
// `up` is the world axis that points up on screen. y is up in model space.
const VIEWS = {
  iso: { dir: [-1, -1, -1], up: [0, 1, 0] },
  front: { dir: [0, 0, -1], up: [0, 1, 0] },
  back: { dir: [0, 0, 1], up: [0, 1, 0] },
  right: { dir: [-1, 0, 0], up: [0, 1, 0] },
  left: { dir: [1, 0, 0], up: [0, 1, 0] },
  top: { dir: [0, -1, 0], up: [0, 0, -1] },
  bottom: { dir: [0, 1, 0], up: [0, 0, 1] },
};

export const VIEW_NAMES = Object.keys(VIEWS);

/**
 * Render modes. `shaded` is the flat-lit solid (the original behaviour),
 * `edges` is line-work only on a light ground — the dimension-drawing look —
 * and `shaded_edges` overlays the line-work on the solid.
 */
export const RENDER_MODES = ['shaded', 'shaded_edges', 'edges'];

/** Section-plane orientations, by the world axis the plane cuts across. */
export const SECTION_AXES = ['X', 'Y', 'Z'];

const AXIS_VEC = {
  X: [1, 0, 0],
  Y: [0, 1, 0],
  Z: [0, 0, 1],
};

// Fixed palette and light. These are deliberately constants rather than
// options: two shots of the same part are only comparable if the only thing
// that changed between them is the thing the caller asked to change.
const SHADED_BG = [24, 27, 33]; // dark slate
const PAPER_BG = [247, 247, 245]; // near-white ground for line drawings
const MATERIAL = [122, 162, 208]; // steel blue
const CUT_FACE = [206, 138, 74]; // amber — a cut face must never read as material
const INK = [18, 20, 26]; // line-work
const LIGHT = [-0.4, 0.7, -1.0]; // camera-space, upper-left, toward viewer

/** Fraction of the frame the fitted content occupies (the rest is margin). */
const FIT_MARGIN = 0.9;

/**
 * Depth slack, as a fraction of the scene's depth extent, allowed when testing
 * a line fragment against the surface depth buffer. Edges lie exactly *on* the
 * surface they bound, so a strict test erases them; a bias this small still
 * hides line-work a whole feature deep.
 */
const EDGE_DEPTH_BIAS = 1e-3;

function normalize([x, y, z]) {
  const len = Math.hypot(x, y, z) || 1;
  return [x / len, y / len, z / len];
}

function cross(a, b) {
  return [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];
}

function dot(a, b) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

/** Parse a `[x,y,z]` argument, or return null when it was not supplied. */
function vec3(value, label) {
  if (value === undefined || value === null) return null;
  if (!Array.isArray(value) || value.length !== 3 || !value.every((n) => Number.isFinite(n))) {
    throw new Error(`${label} must be [x, y, z] with three finite numbers`);
  }
  return [Number(value[0]), Number(value[1]), Number(value[2])];
}

/**
 * The screen-up axis for a caller-supplied view direction. Looking straight
 * down or straight up, world +Y is degenerate as an up vector, so fall back to
 * the same Z convention the named `top`/`bottom` views use.
 */
function defaultUpFor(dir) {
  const n = normalize(dir);
  if (Math.abs(n[1]) > 0.999) return n[1] < 0 ? [0, 0, -1] : [0, 0, 1];
  return [0, 1, 0];
}

/**
 * Resolve the camera orientation: an explicit `direction` (with an optional
 * `up`) wins over a named `view`, which falls back to `iso`.
 */
function resolveView(opts) {
  const custom = vec3(opts.direction, 'direction');
  if (custom) {
    if (Math.hypot(custom[0], custom[1], custom[2]) < 1e-9) {
      throw new Error('direction must be a non-zero vector');
    }
    return { dir: custom, up: vec3(opts.up, 'up') || defaultUpFor(custom), name: null };
  }
  if (opts.view !== undefined && opts.view !== null && !VIEWS[opts.view]) {
    throw new Error(`unknown view '${opts.view}'; use one of ${VIEW_NAMES.join(', ')} or pass a direction`);
  }
  const name = opts.view && VIEWS[opts.view] ? opts.view : 'iso';
  return { dir: VIEWS[name].dir, up: VIEWS[name].up, name };
}

/**
 * The unit direction the camera will look, for the same `{view, direction}`
 * arguments {@link renderScene} takes. Exported because silhouette edges are
 * view-dependent and have to be asked of the kernel *before* rendering — this
 * keeps the two answers from drifting apart.
 *
 * @param {{view?:string, direction?:number[]}} opts
 * @returns {number[]} unit [x,y,z]
 */
export function viewDirection(opts = {}) {
  return normalize(resolveView(opts).dir);
}

function resolveMode(mode) {
  if (mode === undefined || mode === null) return 'shaded';
  if (!RENDER_MODES.includes(mode)) {
    throw new Error(`unknown mode '${mode}'; use one of ${RENDER_MODES.join(', ')}`);
  }
  return mode;
}

/**
 * Resolve a section request against the model bounds. The half-space kept is
 * `normal · p + constant >= 0`, which by default keeps the side with the
 * *smaller* coordinate on the section axis (`coord <= offset`); `flip` keeps
 * the other half. This is the same convention (and the same plane algebra) the
 * playground's `lib/sectionView.js` hands to three.js clipping, so a section
 * screenshot and the GUI's section view cut the same way.
 *
 * An omitted `offset` re-seats the plane at the model's centre on that axis —
 * the playground's `defaultSection`/`reseatOffset` behaviour, and the only
 * offset that is guaranteed to actually intersect the part.
 */
function resolveSection(section, bounds) {
  if (section === undefined || section === null) return null;
  if (typeof section !== 'object' || Array.isArray(section)) {
    throw new Error('section must be an object like { axis: "X", offset: 0, flip: false }');
  }
  const axis = String(section.axis === undefined ? 'X' : section.axis).toUpperCase();
  if (!SECTION_AXES.includes(axis)) {
    throw new Error(`section.axis must be one of ${SECTION_AXES.join(', ')}`);
  }
  const i = SECTION_AXES.indexOf(axis);
  const midpoint = (bounds[i] + bounds[i + 3]) / 2;
  if (section.offset !== undefined && section.offset !== null && !Number.isFinite(section.offset)) {
    throw new Error('section.offset must be a finite number');
  }
  const offset =
    section.offset === undefined || section.offset === null ? midpoint : Number(section.offset);
  const flip = Boolean(section.flip);
  const s = flip ? 1 : -1;
  const a = AXIS_VEC[axis];
  return {
    axis,
    offset,
    flip,
    normal: [a[0] * s, a[1] * s, a[2] * s],
    constant: -s * offset,
  };
}

/** The eight corners of a `{min, max}` region, validated. */
function regionCorners(region) {
  if (region === undefined || region === null) return null;
  if (typeof region !== 'object' || Array.isArray(region)) {
    throw new Error('region must be an object like { min: [x,y,z], max: [x,y,z] }');
  }
  const min = vec3(region.min, 'region.min');
  const max = vec3(region.max, 'region.max');
  if (!min || !max) throw new Error('region needs both min and max as [x, y, z]');
  for (let k = 0; k < 3; k++) {
    if (max[k] < min[k]) {
      throw new Error(`region.max[${k}] (${max[k]}) is below region.min[${k}] (${min[k]})`);
    }
  }
  const corners = [];
  for (let i = 0; i < 8; i++) {
    corners.push([i & 1 ? max[0] : min[0], i & 2 ? max[1] : min[1], i & 4 ? max[2] : min[2]]);
  }
  return corners;
}

function resolveZoom(zoom) {
  if (zoom === undefined || zoom === null) return 1;
  if (!Number.isFinite(zoom) || zoom <= 0) {
    throw new Error('zoom must be a positive number (1 fits the frame, 2 is twice as close)');
  }
  return Number(zoom);
}

function clampDim(value, fallback) {
  const n = Number(value);
  if (!Number.isFinite(n)) return fallback;
  return Math.max(16, Math.min(2048, Math.round(n)));
}

function clampLineWidth(value, fallback) {
  const n = Number(value);
  if (!Number.isFinite(n)) return fallback;
  return Math.max(1, Math.min(8, Math.round(n)));
}

function round(n, places = 4) {
  const f = 10 ** places;
  return Math.round(n * f) / f;
}

/**
 * Render a mesh and report the camera that produced it.
 *
 * @param {{positions:Float32Array, indices:Uint32Array}} mesh
 * @param {number[]} bounds [minx,miny,minz,maxx,maxy,maxz]
 * @param {object} [opts]
 * @param {string} [opts.view] named view (default iso); ignored when `direction` is set
 * @param {number[]} [opts.direction] arbitrary view direction [x,y,z]
 * @param {number[]} [opts.up] screen-up vector for a custom direction
 * @param {{min:number[],max:number[]}} [opts.region] world-space box to frame
 * @param {number[]} [opts.target] world point to centre in frame
 * @param {number} [opts.zoom] scale multiplier on the fitted framing
 * @param {string} [opts.mode] shaded | shaded_edges | edges
 * @param {{axis:string,offset?:number,flip?:boolean}} [opts.section] clip plane
 * @param {{feature?:Float32Array, silhouette?:Float32Array}} [opts.edges] world-space
 *   line segments, flat [x0,y0,z0,x1,y1,z1,…]
 * @param {number} [opts.lineWidth] line-work thickness in px
 * @param {number} [opts.width] image width in px (default 800)
 * @param {number} [opts.height] image height in px (default 600)
 * @returns {{png:Buffer, camera:object}}
 */
export function renderScene(mesh, bounds, opts = {}) {
  const width = clampDim(opts.width, 800);
  const height = clampDim(opts.height, 600);
  const mode = resolveMode(opts.mode);
  const drawSurface = mode !== 'edges';
  const drawEdges = mode !== 'shaded';
  const lineWidth = clampLineWidth(opts.lineWidth, mode === 'edges' ? 2 : 1);
  const { dir, up, name: viewName } = resolveView(opts);
  const section = resolveSection(opts.section, bounds);
  const corners = regionCorners(opts.region);
  const zoom = resolveZoom(opts.zoom);
  const target = vec3(opts.target, 'target');

  // Camera basis: forward (look direction), right, true-up.
  const f = normalize(dir);
  let r = cross(f, normalize(up));
  if (Math.hypot(r[0], r[1], r[2]) < 1e-9) {
    r = [1, 0, 0]; // up parallel to view; pick an arbitrary right.
  }
  r = normalize(r);
  const u = normalize(cross(r, f));

  const center = [
    (bounds[0] + bounds[3]) / 2,
    (bounds[1] + bounds[4]) / 2,
    (bounds[2] + bounds[5]) / 2,
  ];

  // World point -> camera space (x right, y up, z depth along f), relative to
  // the model centre so the numbers stay small and well-conditioned.
  const camX = (px, py, pz) =>
    (px - center[0]) * r[0] + (py - center[1]) * r[1] + (pz - center[2]) * r[2];
  const camY = (px, py, pz) =>
    (px - center[0]) * u[0] + (py - center[1]) * u[1] + (pz - center[2]) * u[2];
  const camZ = (px, py, pz) =>
    (px - center[0]) * f[0] + (py - center[1]) * f[1] + (pz - center[2]) * f[2];

  const { positions, indices } = mesh;

  // Project every vertex into camera space.
  const vertexCount = positions.length / 3;
  const cx = new Float64Array(vertexCount);
  const cy = new Float64Array(vertexCount);
  const cz = new Float64Array(vertexCount);
  let minX = Infinity, maxX = -Infinity, minY = Infinity, maxY = -Infinity;
  for (let i = 0; i < vertexCount; i++) {
    const px = positions[i * 3] - center[0];
    const py = positions[i * 3 + 1] - center[1];
    const pz = positions[i * 3 + 2] - center[2];
    const x = px * r[0] + py * r[1] + pz * r[2];
    const y = px * u[0] + py * u[1] + pz * u[2];
    const z = px * f[0] + py * f[1] + pz * f[2];
    cx[i] = x;
    cy[i] = y;
    cz[i] = z;
    if (x < minX) minX = x;
    if (x > maxX) maxX = x;
    if (y < minY) minY = y;
    if (y > maxY) maxY = y;
  }
  if (!Number.isFinite(minX)) {
    minX = maxX = minY = maxY = 0;
  }

  // Framing. Without a `region` the fit is the projected mesh (a tight fit of
  // the whole part, which is what the plain shot has always done); with one it
  // is the requested box, so an agent can frame a feature it located by
  // bounding box without knowing how that box projects.
  let fitMinX = minX, fitMaxX = maxX, fitMinY = minY, fitMaxY = maxY;
  if (corners) {
    fitMinX = fitMinY = Infinity;
    fitMaxX = fitMaxY = -Infinity;
    for (const [px, py, pz] of corners) {
      const x = camX(px, py, pz);
      const y = camY(px, py, pz);
      if (x < fitMinX) fitMinX = x;
      if (x > fitMaxX) fitMaxX = x;
      if (y < fitMinY) fitMinY = y;
      if (y > fitMaxY) fitMaxY = y;
    }
  }

  // Uniform orthographic fit with a margin, preserving aspect.
  const spanX = Math.max(fitMaxX - fitMinX, 1e-9);
  const spanY = Math.max(fitMaxY - fitMinY, 1e-9);
  const scale = Math.min((width * FIT_MARGIN) / spanX, (height * FIT_MARGIN) / spanY) * zoom;
  const midX = target ? camX(target[0], target[1], target[2]) : (fitMinX + fitMaxX) / 2;
  const midY = target ? camY(target[0], target[1], target[2]) : (fitMinY + fitMaxY) / 2;
  const toScreenX = (x) => width / 2 + (x - midX) * scale;
  const toScreenY = (y) => height / 2 - (y - midY) * scale; // flip y for image
  // Screen -> camera-space, for the per-pixel section-cap plane evaluation.
  const fromScreenX = (sx) => midX + (sx - width / 2) / scale;
  const fromScreenY = (sy) => midY - (sy - height / 2) / scale;

  // Framebuffer + depth buffer. Smaller depth (along f) is nearer the camera.
  const bg = mode === 'edges' ? PAPER_BG : SHADED_BG;
  const rgba = Buffer.alloc(width * height * 4);
  for (let p = 0; p < width * height; p++) {
    rgba[p * 4] = bg[0];
    rgba[p * 4 + 1] = bg[1];
    rgba[p * 4 + 2] = bg[2];
    rgba[p * 4 + 3] = 255;
  }
  const depth = new Float64Array(width * height).fill(Infinity);

  const light = normalize(LIGHT);
  const shade = (nrm, base) => {
    // Two-sided flat shading: ambient + diffuse, magnitude only.
    const intensity = Math.min(1, 0.28 + 0.72 * Math.abs(dot(nrm, light)));
    return [
      Math.round(base[0] * intensity),
      Math.round(base[1] * intensity),
      Math.round(base[2] * intensity),
    ];
  };

  // The section plane in camera space: A·x + B·y + C·z + D >= 0 is kept.
  let planeA = 0, planeB = 0, planeC = 0, planeD = 0;
  if (section) {
    const n = section.normal;
    planeA = dot(n, r);
    planeB = dot(n, u);
    planeC = dot(n, f);
    planeD = dot(n, center) + section.constant;
  }
  const sideOf = (x, y, z) => planeA * x + planeB * y + planeC * z + planeD;

  // Cut outline: the plane-crossing chords of the clipped triangles. Line-work
  // modes draw these, so a section reads as a bounded cut face rather than as
  // a shape that happens to stop.
  const cutSegments = [];

  /** Fill a screen-space triangle, writing colour where the depth test wins. */
  function fillTriangle(ax, ay, az, bx, by, bz, gx, gy, gz, col) {
    const area = (bx - ax) * (gy - ay) - (gx - ax) * (by - ay);
    if (Math.abs(area) < 1e-9) return;
    const loX = Math.max(0, Math.floor(Math.min(ax, bx, gx)));
    const hiX = Math.min(width - 1, Math.ceil(Math.max(ax, bx, gx)));
    const loY = Math.max(0, Math.floor(Math.min(ay, by, gy)));
    const hiY = Math.min(height - 1, Math.ceil(Math.max(ay, by, gy)));
    const invArea = 1 / area;
    for (let y = loY; y <= hiY; y++) {
      for (let x = loX; x <= hiX; x++) {
        const sx = x + 0.5;
        const sy = y + 0.5;
        const w0 = ((bx - sx) * (gy - sy) - (gx - sx) * (by - sy)) * invArea;
        const w1 = ((gx - sx) * (ay - sy) - (ax - sx) * (gy - sy)) * invArea;
        const w2 = 1 - w0 - w1;
        if (w0 < 0 || w1 < 0 || w2 < 0) continue;
        const d = w0 * az + w1 * bz + w2 * gz;
        const idx = y * width + x;
        if (d >= depth[idx]) continue;
        depth[idx] = d;
        if (!col) continue;
        const p = idx * 4;
        rgba[p] = col[0];
        rgba[p + 1] = col[1];
        rgba[p + 2] = col[2];
        rgba[p + 3] = 255;
      }
    }
  }

  // Surface pass. In `edges` mode this still runs — it fills the depth buffer,
  // which is what removes hidden line-work — it just writes no colour.
  const scratch = [];
  for (let t = 0; t < indices.length; t += 3) {
    const ia = indices[t];
    const ib = indices[t + 1];
    const ic = indices[t + 2];

    // Flat normal in camera space, from the unclipped triangle so clipping
    // never changes how a face is lit.
    const ux = cx[ib] - cx[ia], uy = cy[ib] - cy[ia], uz = cz[ib] - cz[ia];
    const vx = cx[ic] - cx[ia], vy = cy[ic] - cy[ia], vz = cz[ic] - cz[ia];
    const nrm = normalize([
      uy * vz - uz * vy,
      uz * vx - ux * vz,
      ux * vy - uy * vx,
    ]);
    const col = drawSurface ? shade(nrm, MATERIAL) : null;

    if (!section) {
      fillTriangle(
        toScreenX(cx[ia]), toScreenY(cy[ia]), cz[ia],
        toScreenX(cx[ib]), toScreenY(cy[ib]), cz[ib],
        toScreenX(cx[ic]), toScreenY(cy[ic]), cz[ic],
        col,
      );
      continue;
    }

    // Sutherland–Hodgman against the kept half-space. The polygon is 0, 3 or
    // 4 vertices; a crossing contributes exactly two points, which are the
    // triangle's chord on the cut plane.
    const sa = sideOf(cx[ia], cy[ia], cz[ia]);
    const sb = sideOf(cx[ib], cy[ib], cz[ib]);
    const sg = sideOf(cx[ic], cy[ic], cz[ic]);
    if (sa < 0 && sb < 0 && sg < 0) continue;
    if (sa >= 0 && sb >= 0 && sg >= 0) {
      fillTriangle(
        toScreenX(cx[ia]), toScreenY(cy[ia]), cz[ia],
        toScreenX(cx[ib]), toScreenY(cy[ib]), cz[ib],
        toScreenX(cx[ic]), toScreenY(cy[ic]), cz[ic],
        col,
      );
      continue;
    }
    scratch.length = 0;
    const vi = [ia, ib, ic];
    const vs = [sa, sb, sg];
    let chord0 = null;
    let chord1 = null;
    for (let k = 0; k < 3; k++) {
      const i0 = vi[k];
      const i1 = vi[(k + 1) % 3];
      const s0 = vs[k];
      const s1 = vs[(k + 1) % 3];
      if (s0 >= 0) scratch.push([cx[i0], cy[i0], cz[i0]]);
      if ((s0 >= 0) !== (s1 >= 0)) {
        const w = s0 / (s0 - s1);
        const p = [
          cx[i0] + (cx[i1] - cx[i0]) * w,
          cy[i0] + (cy[i1] - cy[i0]) * w,
          cz[i0] + (cz[i1] - cz[i0]) * w,
        ];
        scratch.push(p);
        if (chord0 === null) chord0 = p;
        else if (chord1 === null) chord1 = p;
      }
    }
    if (chord0 && chord1) cutSegments.push(chord0, chord1);
    for (let k = 1; k + 1 < scratch.length; k++) {
      const p0 = scratch[0];
      const p1 = scratch[k];
      const p2 = scratch[k + 1];
      fillTriangle(
        toScreenX(p0[0]), toScreenY(p0[1]), p0[2],
        toScreenX(p1[0]), toScreenY(p1[1]), p1[2],
        toScreenX(p2[0]), toScreenY(p2[1]), p2[2],
        col,
      );
    }
  }

  if (section) {
    drawSectionCap();
  }

  if (drawEdges) {
    const bias = EDGE_DEPTH_BIAS * Math.max(depthSpan(cz), 1e-9);
    const strokes = [];
    collectWorldEdges(opts.edges && opts.edges.feature, strokes);
    collectWorldEdges(opts.edges && opts.edges.silhouette, strokes);
    for (let i = 0; i + 1 < cutSegments.length; i += 2) {
      strokes.push(cutSegments[i], cutSegments[i + 1]);
    }
    for (let i = 0; i + 1 < strokes.length; i += 2) {
      drawStroke(strokes[i], strokes[i + 1], bias);
    }
  }

  const camera = {
    view: viewName,
    direction: f.map((n) => round(n)),
    up: u.map((n) => round(n)),
    target: [
      round(center[0] + midX * r[0] + midY * u[0]),
      round(center[1] + midX * r[1] + midY * u[1]),
      round(center[2] + midX * r[2] + midY * u[2]),
    ],
    zoom: round(zoom),
    scale: round(scale),
    visibleExtent: [round(width / scale), round(height / scale)],
    width,
    height,
    mode,
    lineWidth: drawEdges ? lineWidth : undefined,
    section: section ? { axis: section.axis, offset: round(section.offset), flip: section.flip } : null,
  };
  if (camera.lineWidth === undefined) delete camera.lineWidth;

  return { png: encodePng(rgba, width, height), camera };

  // ---- helpers that close over the framebuffer ----------------------------

  function depthSpan(zs) {
    let lo = Infinity;
    let hi = -Infinity;
    for (let i = 0; i < zs.length; i++) {
      if (zs[i] < lo) lo = zs[i];
      if (zs[i] > hi) hi = zs[i];
    }
    const span = hi - lo;
    return Number.isFinite(span) ? span : 1e-9;
  }

  function collectWorldEdges(flat, out) {
    if (!flat || flat.length < 6) return;
    for (let i = 0; i + 5 < flat.length; i += 6) {
      const a = [flat[i], flat[i + 1], flat[i + 2]];
      const b = [flat[i + 3], flat[i + 4], flat[i + 5]];
      let p = [camX(a[0], a[1], a[2]), camY(a[0], a[1], a[2]), camZ(a[0], a[1], a[2])];
      let q = [camX(b[0], b[1], b[2]), camY(b[0], b[1], b[2]), camZ(b[0], b[1], b[2])];
      if (section) {
        // A section hides material, and it must hide that material's
        // line-work too: an unclipped feature edge floating in the cut-away
        // half is worse than no edge at all.
        const s0 = sideOf(p[0], p[1], p[2]);
        const s1 = sideOf(q[0], q[1], q[2]);
        if (s0 < 0 && s1 < 0) continue;
        if (s0 < 0 || s1 < 0) {
          const w = s0 / (s0 - s1);
          const mid = [
            p[0] + (q[0] - p[0]) * w,
            p[1] + (q[1] - p[1]) * w,
            p[2] + (q[2] - p[2]) * w,
          ];
          if (s0 < 0) p = mid;
          else q = mid;
        }
      }
      out.push(p, q);
    }
  }

  /**
   * The cut face: the region of the section plane that lies inside the solid.
   *
   * Inside-ness is decided by ray parity rather than by triangulating the
   * cross-section — for a closed manifold, a point is interior exactly when an
   * odd number of surface crossings lie beyond it along the view ray. That is
   * one extra rasterization pass over the *unclipped* mesh, and it handles
   * multiple disjoint loops and holes-within-loops without any loop chaining.
   */
  function drawSectionCap() {
    if (Math.abs(planeC) < 1e-9) return; // plane edge-on: the cap has no area
    const parity = new Uint8Array(width * height);
    const planeDepthAt = (sx, sy) =>
      -(planeD + planeA * fromScreenX(sx) + planeB * fromScreenY(sy)) / planeC;

    for (let t = 0; t < indices.length; t += 3) {
      const ia = indices[t], ib = indices[t + 1], ic = indices[t + 2];
      const ax = toScreenX(cx[ia]), ay = toScreenY(cy[ia]);
      const bx = toScreenX(cx[ib]), by = toScreenY(cy[ib]);
      const gx = toScreenX(cx[ic]), gy = toScreenY(cy[ic]);
      const area = (bx - ax) * (gy - ay) - (gx - ax) * (by - ay);
      if (Math.abs(area) < 1e-9) continue;
      const loX = Math.max(0, Math.floor(Math.min(ax, bx, gx)));
      const hiX = Math.min(width - 1, Math.ceil(Math.max(ax, bx, gx)));
      const loY = Math.max(0, Math.floor(Math.min(ay, by, gy)));
      const hiY = Math.min(height - 1, Math.ceil(Math.max(ay, by, gy)));
      const invArea = 1 / area;
      for (let y = loY; y <= hiY; y++) {
        for (let x = loX; x <= hiX; x++) {
          const sx = x + 0.5;
          const sy = y + 0.5;
          const w0 = ((bx - sx) * (gy - sy) - (gx - sx) * (by - sy)) * invArea;
          const w1 = ((gx - sx) * (ay - sy) - (ax - sx) * (gy - sy)) * invArea;
          const w2 = 1 - w0 - w1;
          if (w0 < 0 || w1 < 0 || w2 < 0) continue;
          const d = w0 * cz[ia] + w1 * cz[ib] + w2 * cz[ic];
          if (d > planeDepthAt(sx, sy)) parity[y * width + x] ^= 1;
        }
      }
    }

    const nrm = normalize([planeA, planeB, planeC]);
    const col = drawSurface ? shade(nrm, CUT_FACE) : null;
    for (let y = 0; y < height; y++) {
      for (let x = 0; x < width; x++) {
        const idx = y * width + x;
        if (!parity[idx]) continue;
        const d = planeDepthAt(x + 0.5, y + 0.5);
        if (d >= depth[idx]) continue;
        depth[idx] = d;
        if (!col) continue;
        const p = idx * 4;
        rgba[p] = col[0];
        rgba[p + 1] = col[1];
        rgba[p + 2] = col[2];
        rgba[p + 3] = 255;
      }
    }
  }

  /** Draw one camera-space segment as depth-tested ink. */
  function drawStroke(a, b, bias) {
    let x0 = toScreenX(a[0]), y0 = toScreenY(a[1]), z0 = a[2];
    let x1 = toScreenX(b[0]), y1 = toScreenY(b[1]), z1 = b[2];
    // Clip to the frame before stepping: a zoomed-in shot can push an endpoint
    // millions of pixels off-screen, and an unclipped DDA would walk all of it.
    const clipped = clipToFrame(x0, y0, x1, y1, width, height);
    if (!clipped) return;
    const [t0, t1] = clipped;
    const dx = x1 - x0, dy = y1 - y0, dz = z1 - z0;
    x0 += dx * t0; y0 += dy * t0; z0 += dz * t0;
    x1 = x0 + dx * (t1 - t0); y1 = y0 + dy * (t1 - t0); z1 = z0 + dz * (t1 - t0);

    const steps = Math.max(1, Math.ceil(Math.max(Math.abs(x1 - x0), Math.abs(y1 - y0))));
    const half = (lineWidth - 1) / 2;
    for (let i = 0; i <= steps; i++) {
      const s = i / steps;
      const px = x0 + (x1 - x0) * s;
      const py = y0 + (y1 - y0) * s;
      const pz = z0 + (z1 - z0) * s;
      const bx = Math.round(px - half);
      const by = Math.round(py - half);
      for (let oy = 0; oy < lineWidth; oy++) {
        const y = by + oy;
        if (y < 0 || y >= height) continue;
        for (let ox = 0; ox < lineWidth; ox++) {
          const x = bx + ox;
          if (x < 0 || x >= width) continue;
          const idx = y * width + x;
          // Hidden-line removal: ink only where the segment is at or in front
          // of the nearest surface. Depth is not written, so crossing edges
          // never occlude each other.
          if (pz > depth[idx] + bias) continue;
          const p = idx * 4;
          rgba[p] = INK[0];
          rgba[p + 1] = INK[1];
          rgba[p + 2] = INK[2];
          rgba[p + 3] = 255;
        }
      }
    }
  }
}

/**
 * Liang–Barsky clip of a screen-space segment to `[0,w] x [0,h]`, returning
 * the surviving parameter interval `[t0, t1]`, or null when it misses.
 */
function clipToFrame(x0, y0, x1, y1, w, h) {
  let t0 = 0;
  let t1 = 1;
  const dx = x1 - x0;
  const dy = y1 - y0;
  const tests = [
    [-dx, x0 - 0],
    [dx, w - x0],
    [-dy, y0 - 0],
    [dy, h - y0],
  ];
  for (const [p, q] of tests) {
    if (p === 0) {
      if (q < 0) return null; // parallel to this edge and outside it
      continue;
    }
    const t = q / p;
    if (p < 0) {
      if (t > t1) return null;
      if (t > t0) t0 = t;
    } else {
      if (t < t0) return null;
      if (t < t1) t1 = t;
    }
  }
  return [t0, t1];
}

/**
 * Render a mesh to a PNG buffer.
 *
 * @param {{positions:Float32Array, indices:Uint32Array}} mesh
 * @param {number[]} bounds [minx,miny,minz,maxx,maxy,maxz]
 * @param {object} [opts] see {@link renderScene}
 * @returns {Buffer} PNG bytes
 */
export function renderPng(mesh, bounds, opts = {}) {
  return renderScene(mesh, bounds, opts).png;
}
