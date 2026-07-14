// Measure-tool topology (of-fsl.17): recover pickable *model* entities —
// corner vertices, straight edges, and circular rims — from the displayed
// F-Rep mesh, so the Measure tool can snap a raycast hit to real geometry
// instead of an arbitrary triangle vertex.
//
// The mesh is marching-cubes/adaptive-octree tessellation: flat faces are
// covered in many cell-sized triangles, so a "model edge" is a *crease* (a
// sharp dihedral between two faces) or a *boundary* (an edge with a single
// adjacent triangle). Crease/boundary edges are collected, collinear runs are
// merged into single straight segments, junction vertices (feature-edge
// degree != 2) become corner vertices, and closed feature-edge loops that fit
// a circle become circular rims (hole / cylinder radius).
//
// Kept free of three.js and React so it can be unit-tested on plain
// position/index arrays (same pattern as facePlane.js / faceHighlight.js).

/** Dihedral angle (degrees) above which a shared edge counts as a crease. */
const CREASE_TOL_DEG = 25;
/** Angle (degrees) below which two edge directions merge as collinear. */
const COLLINEAR_TOL_DEG = 4;
/** A fitted circle's radius spread must stay under this fraction of radius. */
const CIRCLE_FIT_TOL = 0.04;
/** Cycles shorter than this can't be told apart from a triangle corner. */
const MIN_CIRCLE_VERTS = 6;

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);

function normalize(a) {
  const n = norm(a);
  return n > 0 ? [a[0] / n, a[1] / n, a[2] / n] : null;
}

function vertex(positions, index) {
  return [positions[3 * index], positions[3 * index + 1], positions[3 * index + 2]];
}

function triVertexIds(indices, tri) {
  return [indices[3 * tri], indices[3 * tri + 1], indices[3 * tri + 2]];
}

function triNormal(positions, indices, tri) {
  const [a, b, c] = triVertexIds(indices, tri).map((i) => vertex(positions, i));
  return normalize(cross(sub(b, a), sub(c, a)));
}

// Numeric undirected edge key: meshes stay far below 2^26 vertices, so the
// pair packs losslessly into one double (mirrors facePlane.js).
const EDGE_KEY_BASE = 2 ** 26;
const edgeKey = (p, q) => (p < q ? p * EDGE_KEY_BASE + q : q * EDGE_KEY_BASE + p);

function meshDiagonal(positions) {
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let k = 0; k < 3; k += 1) {
      const c = positions[i + k];
      if (c < min[k]) min[k] = c;
      if (c > max[k]) max[k] = c;
    }
  }
  return norm(sub(max, min));
}

/**
 * Fit a circle to coplanar points: centroid-centered plane via the covariance
 * cross-products, then the mean radius. Returns `{ center, radius, normal }`
 * when the radial spread stays under `CIRCLE_FIT_TOL` (a real rim), else null.
 */
export function fitCircle(points) {
  const n = points.length;
  if (n < 3) return null;
  const centroid = [0, 0, 0];
  for (const p of points) {
    centroid[0] += p[0];
    centroid[1] += p[1];
    centroid[2] += p[2];
  }
  const center = scale(centroid, 1 / n);

  // Plane normal: sum of consecutive spoke cross-products (Newell-like), which
  // is robust for a ring of points ordered around the loop.
  let normalAcc = [0, 0, 0];
  for (let i = 0; i < n; i += 1) {
    const a = sub(points[i], center);
    const b = sub(points[(i + 1) % n], center);
    normalAcc = add(normalAcc, cross(a, b));
  }
  const normal = normalize(normalAcc);
  if (!normal) return null;

  let meanR = 0;
  const radii = [];
  for (const p of points) {
    const d = sub(p, center);
    // Radius measured in the fitted plane (drop any out-of-plane component).
    const planar = sub(d, scale(normal, dot(d, normal)));
    const r = norm(planar);
    radii.push(r);
    meanR += r;
  }
  meanR /= n;
  if (meanR <= 0) return null;

  let spread = 0;
  for (const r of radii) spread = Math.max(spread, Math.abs(r - meanR));
  if (spread > CIRCLE_FIT_TOL * meanR) return null;

  return { center, radius: meanR, normal };
}

/**
 * Recover the pickable model entities from a triangle mesh.
 *
 * Returns `{ vertices, edges, circles }`:
 * - `vertices`: `{ point }` for each junction (feature-edge degree != 2).
 * - `edges`: `{ a, b, dir, length }` straight segments (collinear feature-edge
 *   runs merged corner-to-corner).
 * - `circles`: `{ center, radius, normal }` for closed feature-edge loops that
 *   fit a circle within tolerance.
 */
export function buildEdgeModel(positions, indices) {
  const triCount = indices.length / 3;
  if (triCount === 0) return { vertices: [], edges: [], circles: [] };

  // 1. Edge -> adjacent triangles, remembering the endpoint vertex ids.
  const byEdge = new Map();
  for (let t = 0; t < triCount; t += 1) {
    const [a, b, c] = triVertexIds(indices, t);
    for (const [p, q] of [[a, b], [b, c], [c, a]]) {
      const key = edgeKey(p, q);
      let rec = byEdge.get(key);
      if (!rec) byEdge.set(key, (rec = { p, q, tris: [] }));
      rec.tris.push(t);
    }
  }

  // 2. Classify each edge as a feature edge (boundary or crease). A crease is
  // a sharp dihedral: the two faces' normals diverge past CREASE_TOL_DEG. The
  // |dot| form is winding-independent (a real solid never folds to ~0°).
  const cosCrease = Math.cos((CREASE_TOL_DEG * Math.PI) / 180);
  // Feature graph as vertex-id -> Set(neighbor vertex-id).
  const adj = new Map();
  const linkVerts = (p, q) => {
    if (!adj.has(p)) adj.set(p, new Set());
    if (!adj.has(q)) adj.set(q, new Set());
    adj.get(p).add(q);
    adj.get(q).add(p);
  };
  for (const { p, q, tris } of byEdge.values()) {
    let feature = false;
    if (tris.length === 1) {
      feature = true; // boundary edge
    } else if (tris.length === 2) {
      const n0 = triNormal(positions, indices, tris[0]);
      const n1 = triNormal(positions, indices, tris[1]);
      if (n0 && n1 && Math.abs(dot(n0, n1)) < cosCrease) feature = true;
    }
    if (feature) linkVerts(p, q);
  }

  if (adj.size === 0) return { vertices: [], edges: [], circles: [] };

  const point = (id) => vertex(positions, id);
  const diag = meshDiagonal(positions) || 1;
  const weldTol = 1e-6 * diag;

  // 3. Circular rims: connected components where every vertex has degree 2
  // form a simple closed loop; fit a circle to the ordered ring.
  const circles = [];
  const consumed = new Set(); // vertex ids used by an accepted circle loop
  const visited = new Set();
  for (const start of adj.keys()) {
    if (visited.has(start)) continue;
    // Gather the component and check the all-degree-2 property.
    const comp = [];
    const stack = [start];
    visited.add(start);
    let allDeg2 = true;
    while (stack.length) {
      const v = stack.pop();
      comp.push(v);
      if (adj.get(v).size !== 2) allDeg2 = false;
      for (const nb of adj.get(v)) {
        if (!visited.has(nb)) {
          visited.add(nb);
          stack.push(nb);
        }
      }
    }
    if (!allDeg2 || comp.length < MIN_CIRCLE_VERTS) continue;
    // Order the ring by walking neighbors from an arbitrary start.
    const ring = [];
    const seen = new Set();
    let cur = start;
    let prev = null;
    while (cur != null && !seen.has(cur)) {
      seen.add(cur);
      ring.push(point(cur));
      let next = null;
      for (const nb of adj.get(cur)) {
        if (nb !== prev) next = nb;
      }
      prev = cur;
      cur = next;
    }
    const circle = fitCircle(ring);
    if (circle) {
      circles.push(circle);
      for (const v of comp) consumed.add(v);
    }
  }

  // 4. Straight edges: trace maximal collinear runs between corner vertices.
  // A vertex is a corner when its feature-edge degree != 2, or its two edges
  // are not collinear (a polygon corner). Circle-consumed vertices are skipped.
  const cosCollinear = Math.cos((COLLINEAR_TOL_DEG * Math.PI) / 180);
  const isCorner = (v) => {
    const nbrs = adj.get(v);
    if (nbrs.size !== 2) return true;
    const [n0, n1] = [...nbrs];
    const d0 = normalize(sub(point(n0), point(v)));
    const d1 = normalize(sub(point(v), point(n1)));
    if (!d0 || !d1) return true;
    return dot(d0, d1) < cosCollinear;
  };

  const cornerSet = new Set();
  for (const v of adj.keys()) {
    if (!consumed.has(v) && isCorner(v)) cornerSet.add(v);
  }

  const edges = [];
  const usedEdge = new Set();
  for (const startCorner of cornerSet) {
    for (const firstStep of adj.get(startCorner)) {
      if (consumed.has(firstStep)) continue;
      if (usedEdge.has(edgeKey(startCorner, firstStep))) continue;
      // Walk collinear degree-2 vertices until the next corner.
      let prev = startCorner;
      let cur = firstStep;
      usedEdge.add(edgeKey(prev, cur));
      while (!cornerSet.has(cur)) {
        let next = null;
        for (const nb of adj.get(cur)) {
          if (nb !== prev) next = nb;
        }
        if (next == null || consumed.has(next)) break;
        usedEdge.add(edgeKey(cur, next));
        prev = cur;
        cur = next;
      }
      const a = point(startCorner);
      const b = point(cur);
      const dir = normalize(sub(b, a));
      const length = norm(sub(b, a));
      if (dir && length > weldTol) edges.push({ a, b, dir, length });
    }
  }

  const vertices = [...cornerSet].map((v) => ({ point: point(v) }));
  return { vertices, edges, circles };
}

/** Squared distance from point `p` to the segment `a`–`b`, plus closest point. */
function closestOnSegment(p, a, b) {
  const ab = sub(b, a);
  const len2 = dot(ab, ab);
  let t = len2 > 0 ? dot(sub(p, a), ab) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const closest = add(a, scale(ab, t));
  return { closest, dist: norm(sub(p, closest)), t };
}

/** Nearest point on a circle's ring to `p`, and its distance. */
function closestOnCircle(p, circle) {
  const { center, radius, normal } = circle;
  const d = sub(p, center);
  const planar = sub(d, scale(normal, dot(d, normal)));
  const dir = normalize(planar) || [1, 0, 0];
  const onRing = add(center, scale(dir, radius));
  return { closest: onRing, dist: norm(sub(p, onRing)) };
}

/**
 * Snap a raycast hit `point` to the nearest model entity within `tol`,
 * preferring a corner vertex, then a straight edge, then a circular rim.
 * Returns the matched entity (with a representative `point`) or null.
 */
export function snapEntity(model, point, tol) {
  if (!model || !point) return null;

  let bestVert = null;
  let bestVertD = tol;
  for (const v of model.vertices) {
    const d = norm(sub(point, v.point));
    if (d < bestVertD) {
      bestVertD = d;
      bestVert = v;
    }
  }
  if (bestVert) return { kind: 'vertex', point: bestVert.point };

  let bestEdge = null;
  let bestEdgeD = tol;
  for (const e of model.edges) {
    const { closest, dist } = closestOnSegment(point, e.a, e.b);
    if (dist < bestEdgeD) {
      bestEdgeD = dist;
      bestEdge = { kind: 'edge', a: e.a, b: e.b, dir: e.dir, length: e.length, point: closest };
    }
  }
  if (bestEdge) return bestEdge;

  let bestCircle = null;
  let bestCircleD = tol;
  for (const c of model.circles) {
    const { closest, dist } = closestOnCircle(point, c);
    if (dist < bestCircleD) {
      bestCircleD = dist;
      bestCircle = {
        kind: 'circle',
        center: c.center,
        radius: c.radius,
        normal: c.normal,
        point: closest,
      };
    }
  }
  if (bestCircle) return bestCircle;

  return null;
}
