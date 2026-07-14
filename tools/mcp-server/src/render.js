// Headless software rasterizer: orthographic, z-buffered, flat-shaded render
// of a triangle mesh to an RGBA framebuffer, encoded as PNG. Pure JS, no GPU
// and no headless browser — a screenshot is a few milliseconds and has no
// external dependencies.

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

/**
 * Render a mesh to a PNG buffer.
 *
 * @param {{positions:Float32Array, indices:Uint32Array}} mesh
 * @param {number[]} bounds [minx,miny,minz,maxx,maxy,maxz]
 * @param {{view?:string, width?:number, height?:number}} [opts]
 * @returns {Buffer} PNG bytes
 */
export function renderPng(mesh, bounds, opts = {}) {
  const width = clampDim(opts.width, 800);
  const height = clampDim(opts.height, 600);
  const viewName = opts.view && VIEWS[opts.view] ? opts.view : 'iso';
  const view = VIEWS[viewName];

  // Camera basis: forward (look direction), right, true-up.
  const f = normalize(view.dir);
  let r = cross(f, normalize(view.up));
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

  const { positions, indices } = mesh;

  // Project every vertex into camera space (x-right, y-up, depth along f).
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

  // Uniform orthographic fit with a margin, preserving aspect.
  const margin = 0.9;
  const spanX = Math.max(maxX - minX, 1e-9);
  const spanY = Math.max(maxY - minY, 1e-9);
  const scale = Math.min((width * margin) / spanX, (height * margin) / spanY);
  const midX = (minX + maxX) / 2;
  const midY = (minY + maxY) / 2;
  const toScreenX = (x) => width / 2 + (x - midX) * scale;
  const toScreenY = (y) => height / 2 - (y - midY) * scale; // flip y for image

  // Framebuffer + depth buffer. Smaller depth (along f) is nearer the camera.
  const bg = [24, 27, 33]; // dark slate
  const rgba = Buffer.alloc(width * height * 4);
  for (let p = 0; p < width * height; p++) {
    rgba[p * 4] = bg[0];
    rgba[p * 4 + 1] = bg[1];
    rgba[p * 4 + 2] = bg[2];
    rgba[p * 4 + 3] = 255;
  }
  const depth = new Float64Array(width * height).fill(Infinity);

  const base = [122, 162, 208]; // steel blue material
  const light = normalize([-0.4, 0.7, -1.0]); // camera-space, upper-left, toward viewer

  for (let t = 0; t < indices.length; t += 3) {
    const ia = indices[t];
    const ib = indices[t + 1];
    const ic = indices[t + 2];

    const ax = toScreenX(cx[ia]), ay = toScreenY(cy[ia]);
    const bx = toScreenX(cx[ib]), by = toScreenY(cy[ib]);
    const gx = toScreenX(cx[ic]), gy = toScreenY(cy[ic]);

    // Signed area in screen space; skip degenerate triangles.
    const area = (bx - ax) * (gy - ay) - (gx - ax) * (by - ay);
    if (Math.abs(area) < 1e-9) continue;

    // Flat normal in camera space from projected-space geometry.
    const ux = cx[ib] - cx[ia], uy = cy[ib] - cy[ia], uz = cz[ib] - cz[ia];
    const vx = cx[ic] - cx[ia], vy = cy[ic] - cy[ia], vz = cz[ic] - cz[ia];
    const nrm = normalize([
      uy * vz - uz * vy,
      uz * vx - ux * vz,
      ux * vy - uy * vx,
    ]);
    // Two-sided flat shading: ambient + diffuse, magnitude only.
    const intensity = Math.min(1, 0.28 + 0.72 * Math.abs(dot(nrm, light)));

    const col0 = Math.round(base[0] * intensity);
    const col1 = Math.round(base[1] * intensity);
    const col2 = Math.round(base[2] * intensity);

    // Pixel bounding box, clamped to the framebuffer.
    const loX = Math.max(0, Math.floor(Math.min(ax, bx, gx)));
    const hiX = Math.min(width - 1, Math.ceil(Math.max(ax, bx, gx)));
    const loY = Math.max(0, Math.floor(Math.min(ay, by, gy)));
    const hiY = Math.min(height - 1, Math.ceil(Math.max(ay, by, gy)));
    const invArea = 1 / area;

    for (let y = loY; y <= hiY; y++) {
      for (let x = loX; x <= hiX; x++) {
        const sx = x + 0.5;
        const sy = y + 0.5;
        // Barycentric weights via signed sub-areas.
        const w0 = ((bx - sx) * (gy - sy) - (gx - sx) * (by - sy)) * invArea;
        const w1 = ((gx - sx) * (ay - sy) - (ax - sx) * (gy - sy)) * invArea;
        const w2 = 1 - w0 - w1;
        if (w0 < 0 || w1 < 0 || w2 < 0) continue;

        const d = w0 * cz[ia] + w1 * cz[ib] + w2 * cz[ic];
        const idx = y * width + x;
        if (d >= depth[idx]) continue;
        depth[idx] = d;
        const p = idx * 4;
        rgba[p] = col0;
        rgba[p + 1] = col1;
        rgba[p + 2] = col2;
        rgba[p + 3] = 255;
      }
    }
  }

  return encodePng(rgba, width, height);
}

function clampDim(value, fallback) {
  const n = Number(value);
  if (!Number.isFinite(n)) return fallback;
  return Math.max(16, Math.min(2048, Math.round(n)));
}
