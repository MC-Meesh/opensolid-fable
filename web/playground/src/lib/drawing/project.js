// Orthographic view projection for 2D drawings (of-fsl.26.2).
//
// A drawing view is the body's line-work seen along a standard direction,
// flattened to the sheet. This module turns a triangle mesh into 2D line
// segments for a named view (front/top/right/iso …), per DRAWINGS.md §2.2:
//
//   1. Build an orthonormal camera basis {u, v, w} from the view direction
//      (VIEW_DIRECTIONS) plus an up hint — w points toward the camera, u is
//      screen-right, v is screen-up.
//   2. Extract the body's draw-able 3D edges: view-independent *feature*
//      (crease) edges plus per-view *silhouette* (outline) edges (§2.1).
//   3. Project each edge with (x, y, z) -> (p·u, p·v), keeping p·w as the
//      depth key for later occlusion (HLR is deferred — §8, visible only).
//
// Kept free of three.js / React so it is unit-testable in isolation, matching
// views.js and sketchView.js. Edges come from the mesh (positions/indices);
// when the mesher later surfaces a crease-edge buffer through MeshData
// (of-fsl.26.1), `bodyEdges3d` prefers it over the dihedral walk.

import { viewDirection } from '../views.js';

// Up hint per view: the world direction that should point up-screen. Matches
// sketchView.js SKETCH_VIEW_POSES where they overlap (front/right up = +Y,
// top up = -Z so looking straight down never degenerates against the axis).
const VIEW_UP = {
  front: [0, 1, 0],
  back: [0, 1, 0],
  left: [0, 1, 0],
  right: [0, 1, 0],
  top: [0, 0, -1],
  bottom: [0, 0, 1],
  iso: [0, 1, 0],
};

// Default dihedral angle (radians) above which a shared mesh edge counts as a
// crease/feature edge. ~20°: keeps box/prism edges, drops the many shallow
// facet seams of a tessellated smooth surface (those show as silhouettes).
const FEATURE_ANGLE = (20 * Math.PI) / 180;

// Position weld tolerance as a fraction of the mesh bounding diagonal. Dual
// contouring can emit per-triangle-duplicated vertices; welding by quantized
// position recovers edge adjacency regardless of index sharing.
const WELD_FRAC = 1e-5;

function sub(a, b) {
  return [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
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
function normalize(a) {
  const len = Math.hypot(a[0], a[1], a[2]);
  return len > 0 ? [a[0] / len, a[1] / len, a[2] / len] : [0, 0, 0];
}

/**
 * Orthonormal camera basis for a named view: `{ u, v, w }` unit vectors where
 * `w` points from the model toward the camera (the view direction), `u` is
 * screen-right, and `v` is screen-up. Returns `null` for an unknown view.
 *
 * With hint `h` (VIEW_UP), `u = normalize(h × w)` and `v = w × u`, a
 * right-handed screen frame: projecting `(p·u, p·v)` yields x-right / y-up 2D.
 */
export function viewBasis(name) {
  const dir = viewDirection(name);
  if (!dir) return null;
  const w = normalize(dir);
  let h = VIEW_UP[name] ?? [0, 1, 0];
  // Guard against an up hint parallel to the view direction.
  if (Math.abs(dot(h, w)) > 0.999) h = [1, 0, 0];
  const u = normalize(cross(h, w));
  const v = cross(w, u); // already unit: w ⟂ u, both unit
  // Normalize signed zeros (-0 → 0) so bases compare cleanly.
  const clean = (a) => a.map((c) => (c === 0 ? 0 : c));
  return { u: clean(u), v: clean(v), w: clean(w) };
}

/**
 * Project a 3D point into a view basis: `[p·u, p·v, p·w]` where the first two
 * are sheet coordinates and the third is the depth key (larger = nearer the
 * camera along the view direction).
 */
export function projectPoint(basis, p) {
  return [dot(p, basis.u), dot(p, basis.v), dot(p, basis.w)];
}

// Bounding diagonal of a flat positions array, for the weld tolerance.
function positionsDiagonal(positions) {
  if (!positions || positions.length < 3) return 0;
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let k = 0; k < 3; k += 1) {
      const c = positions[i + k];
      if (c < min[k]) min[k] = c;
      if (c > max[k]) max[k] = c;
    }
  }
  return Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]);
}

// Weld vertices by quantized position: returns { rep, faces } where `rep[i]`
// is the canonical vertex id for original vertex i, and `faces` is the list of
// triangles as canonical-id triples with their outward normal.
function weldTopology(positions, indices) {
  const diag = positionsDiagonal(positions);
  const tol = Math.max(diag * WELD_FRAC, 1e-9);
  const map = new Map();
  const rep = new Uint32Array(positions.length / 3);
  let next = 0;
  for (let i = 0; i < positions.length; i += 3) {
    const kx = Math.round(positions[i] / tol);
    const ky = Math.round(positions[i + 1] / tol);
    const kz = Math.round(positions[i + 2] / tol);
    const key = `${kx},${ky},${kz}`;
    let id = map.get(key);
    if (id === undefined) {
      id = next++;
      map.set(key, id);
    }
    rep[i / 3] = id;
  }
  const faces = [];
  for (let t = 0; t < indices.length; t += 3) {
    const i0 = indices[t];
    const i1 = indices[t + 1];
    const i2 = indices[t + 2];
    const p0 = [positions[i0 * 3], positions[i0 * 3 + 1], positions[i0 * 3 + 2]];
    const p1 = [positions[i1 * 3], positions[i1 * 3 + 1], positions[i1 * 3 + 2]];
    const p2 = [positions[i2 * 3], positions[i2 * 3 + 1], positions[i2 * 3 + 2]];
    const normal = normalize(cross(sub(p1, p0), sub(p2, p0)));
    faces.push({ a: rep[i0], b: rep[i1], c: rep[i2], p0, p1, p2, normal });
  }
  return { faces };
}

// edgeKey for a welded vertex pair (order-independent).
function edgeKey(a, b) {
  return a < b ? `${a}|${b}` : `${b}|${a}`;
}

// Map every mesh edge to the faces adjacent to it, plus its endpoint
// positions. Boundary edges keep a single face.
function edgeAdjacency(faces) {
  const edges = new Map();
  const push = (a, b, pa, pb, face) => {
    const key = edgeKey(a, b);
    let e = edges.get(key);
    if (!e) {
      e = { pa, pb, faces: [] };
      edges.set(key, e);
    }
    e.faces.push(face);
  };
  for (const f of faces) {
    push(f.a, f.b, f.p0, f.p1, f);
    push(f.b, f.c, f.p1, f.p2, f);
    push(f.c, f.a, f.p2, f.p0, f);
  }
  return edges;
}

/**
 * View-independent crease/feature edges of a mesh: shared edges whose two
 * adjacent triangle normals differ by more than `angle`, plus every boundary
 * edge (a hole rim reads as an outline). Returns flat 3D segments as
 * `[[ax,ay,az],[bx,by,bz]], …`.
 */
export function meshFeatureEdges(positions, indices, angle = FEATURE_ANGLE) {
  if (!positions?.length || !indices?.length) return [];
  const { faces } = weldTopology(positions, indices);
  const edges = edgeAdjacency(faces);
  const cosThresh = Math.cos(angle);
  const segments = [];
  for (const e of edges.values()) {
    if (e.faces.length === 1) {
      segments.push([e.pa, e.pb]);
    } else if (e.faces.length >= 2) {
      const c = dot(e.faces[0].normal, e.faces[1].normal);
      if (c < cosThresh) segments.push([e.pa, e.pb]);
    }
  }
  return segments;
}

/**
 * Per-view silhouette (outline) edges: shared edges whose two adjacent faces
 * face opposite ways relative to the view direction (`sign(n·w)` differs), the
 * mesh approximation of the true `n·view_dir = 0` locus (DRAWINGS.md §2.1).
 * Boundary edges are always on the outline. `viewDir` is the direction toward
 * the camera (VIEW_DIRECTIONS). Returns flat 3D segments.
 */
export function meshSilhouetteEdges(positions, indices, viewDir) {
  if (!positions?.length || !indices?.length) return [];
  const w = normalize(viewDir);
  const { faces } = weldTopology(positions, indices);
  const edges = edgeAdjacency(faces);
  const segments = [];
  for (const e of edges.values()) {
    if (e.faces.length === 1) {
      segments.push([e.pa, e.pb]);
    } else if (e.faces.length >= 2) {
      const s0 = dot(e.faces[0].normal, w);
      const s1 = dot(e.faces[1].normal, w);
      if (s0 * s1 < 0) segments.push([e.pa, e.pb]);
    }
  }
  return segments;
}

// Read a flat crease-edge buffer ([x0,y0,z0,x1,y1,z1, …], the of-fsl.26.1
// MeshData.feature_edges convention) into 3D segment pairs.
function segmentsFromBuffer(buffer) {
  const out = [];
  for (let i = 0; i + 5 < buffer.length; i += 6) {
    out.push([
      [buffer[i], buffer[i + 1], buffer[i + 2]],
      [buffer[i + 3], buffer[i + 4], buffer[i + 5]],
    ]);
  }
  return out;
}

/**
 * All draw-able 3D edges of a body for a given view: feature (crease) edges —
 * from `mesh.featureEdges` when the mesher supplies it, else the dihedral walk
 * — unioned with the per-view mesh silhouette. Returns flat 3D segments.
 */
export function bodyEdges3d(mesh, viewName) {
  if (!mesh) return [];
  const dir = viewDirection(viewName);
  if (!dir) return [];
  const { positions, indices, featureEdges } = mesh;
  const feature =
    featureEdges && featureEdges.length
      ? segmentsFromBuffer(featureEdges)
      : meshFeatureEdges(positions, indices);
  const silhouette = meshSilhouetteEdges(positions, indices, dir);
  return feature.concat(silhouette);
}

/**
 * Project a body into a named view: `{ view, segments, bounds }` where each
 * segment is `{ pts: [[u,v],[u,v]], style, depth }` in view (sheet-unscaled)
 * coordinates. `style` is always `'visible'` for the MVP (HLR deferred, §8);
 * `depth` is the mean p·w, kept for the later occlusion pass. `bounds` is
 * `{ minX, minY, maxX, maxY }` over all segment points (null when empty).
 */
export function projectView(mesh, viewName) {
  const basis = viewBasis(viewName);
  if (!basis) return { view: viewName, segments: [], bounds: null };
  const edges = bodyEdges3d(mesh, viewName);
  const segments = [];
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const [a, b] of edges) {
    const pa = projectPoint(basis, a);
    const pb = projectPoint(basis, b);
    // Drop degenerate (zero-length) projections — an edge seen end-on.
    if (Math.hypot(pb[0] - pa[0], pb[1] - pa[1]) < 1e-12) continue;
    segments.push({
      pts: [
        [pa[0], pa[1]],
        [pb[0], pb[1]],
      ],
      style: 'visible',
      depth: (pa[2] + pb[2]) / 2,
    });
    minX = Math.min(minX, pa[0], pb[0]);
    minY = Math.min(minY, pa[1], pb[1]);
    maxX = Math.max(maxX, pa[0], pb[0]);
    maxY = Math.max(maxY, pa[1], pb[1]);
  }
  const bounds =
    segments.length > 0 ? { minX, minY, maxX, maxY } : null;
  return { view: viewName, segments, bounds };
}
