// Edge/vertex topology extraction for the Measure tool (of-fsl.17). The
// playground can only pick bodies and planar faces (picking.js, facePlane.js);
// measuring a hole diameter or the distance between two edges needs edge and
// vertex picking too. This module recovers a lightweight edge model from the
// displayed F-Rep mesh so a raycast hit can snap to the nearest model vertex,
// straight edge, or circular rim.
//
// Feature edges are mesh edges that are either an open boundary or a crease
// between two triangles whose normals differ by more than `creaseAngleDeg`.
// Coplanar interior edges (a face's internal triangulation) are ignored. The
// feature edges are traced into polylines: chains between junction/endpoint
// vertices (degree != 2) become straight/curved edges, and untouched all-
// degree-2 components are closed loops, fitted to a circle so cylinders and
// holes report a radius.
//
// Kept free of three.js and React so it can be unit-tested on plain
// position/index arrays (same pattern as picking.js and facePlane.js).

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const dist = (a, b) => Math.hypot(a[0] - b[0], a[1] - b[1], a[2] - b[2]);

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

/** Unit triangle normal, or null for a degenerate triangle. */
function triNormal(positions, indices, tri) {
  const [a, b, c] = triVertexIds(indices, tri).map((i) => vertex(positions, i));
  return normalize(cross(sub(b, a), sub(c, a)));
}

// Undirected edge key: meshes stay far below 2^26 vertices, so a vertex-id
// pair packs losslessly into one double (matches facePlane.js).
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
  return positions.length ? norm(sub(max, min)) : 0;
}

/** Newell's method: area-weighted normal of a (possibly non-planar) polygon. */
function newellNormal(points) {
  const n = [0, 0, 0];
  for (let i = 0; i < points.length; i += 1) {
    const c = points[i];
    const d = points[(i + 1) % points.length];
    n[0] += (c[1] - d[1]) * (c[2] + d[2]);
    n[1] += (c[2] - d[2]) * (c[0] + d[0]);
    n[2] += (c[0] - d[0]) * (c[1] + d[1]);
  }
  return normalize(n);
}

/**
 * Fit a circle to a closed loop of points: center at the centroid, radius the
 * mean distance to it. Returns `{ center, radius, normal }` when the loop is
 * round (radial spread under 5% of the radius) and planar, else null.
 */
export function fitCircle(points) {
  if (points.length < 4) return null;
  const center = [0, 0, 0];
  for (const p of points) {
    center[0] += p[0];
    center[1] += p[1];
    center[2] += p[2];
  }
  center[0] /= points.length;
  center[1] /= points.length;
  center[2] /= points.length;
  let mean = 0;
  for (const p of points) mean += dist(p, center);
  mean /= points.length;
  if (mean <= 0) return null;
  let maxDev = 0;
  for (const p of points) {
    const dev = Math.abs(dist(p, center) - mean);
    if (dev > maxDev) maxDev = dev;
  }
  if (maxDev > 0.05 * mean) return null;
  return { center, radius: mean, normal: newellNormal(points) };
}

/** Largest distance of any interior point from the chord through the ends. */
function maxLineDeviation(points) {
  const a = points[0];
  const b = points[points.length - 1];
  const axis = sub(b, a);
  const len = norm(axis);
  if (len === 0) return Infinity;
  const u = [axis[0] / len, axis[1] / len, axis[2] / len];
  let maxDev = 0;
  for (let i = 1; i < points.length - 1; i += 1) {
    const w = sub(points[i], a);
    const t = dot(w, u);
    const proj = [a[0] + u[0] * t, a[1] + u[1] * t, a[2] + u[2] * t];
    const dev = dist(points[i], proj);
    if (dev > maxDev) maxDev = dev;
  }
  return maxDev;
}

function addAdj(adj, a, b) {
  let set = adj.get(a);
  if (!set) adj.set(a, (set = new Set()));
  set.add(b);
}

/**
 * Recover an edge model from an indexed triangle mesh.
 *
 * Returns `{ diagonal, corners: [{ point }], edges: [edge] }` where each edge
 * is `{ points, length, closed, straight, endpoints: [a, b], circle }` and
 * `circle` is `{ center, radius, normal }` or null.
 */
export function buildEdgeModel(positions, indices, { creaseAngleDeg = 25 } = {}) {
  const triCount = indices.length / 3;
  const normals = new Array(triCount);
  for (let t = 0; t < triCount; t += 1) normals[t] = triNormal(positions, indices, t);

  // Mesh edge -> the triangles that share it.
  const edgeTris = new Map();
  for (let t = 0; t < triCount; t += 1) {
    const [a, b, c] = triVertexIds(indices, t);
    for (const [p, q] of [[a, b], [b, c], [c, a]]) {
      const key = edgeKey(p, q);
      let rec = edgeTris.get(key);
      if (!rec) edgeTris.set(key, (rec = { p: Math.min(p, q), q: Math.max(p, q), tris: [] }));
      rec.tris.push(t);
    }
  }

  const cosCrease = Math.cos((creaseAngleDeg * Math.PI) / 180);
  const adj = new Map();
  const featureEdges = [];
  for (const rec of edgeTris.values()) {
    let feature = false;
    if (rec.tris.length !== 2) {
      feature = true; // open boundary or non-manifold seam
    } else {
      const n0 = normals[rec.tris[0]];
      const n1 = normals[rec.tris[1]];
      if (!n0 || !n1 || dot(n0, n1) < cosCrease) feature = true;
    }
    if (feature) {
      featureEdges.push([rec.p, rec.q]);
      addAdj(adj, rec.p, rec.q);
      addAdj(adj, rec.q, rec.p);
    }
  }

  const degree = (v) => adj.get(v)?.size ?? 0;
  const visited = new Set();

  // Walk a chain from `start` toward `second`, following degree-2 vertices
  // until a junction/endpoint (degree != 2) or a closed loop. Returns the
  // ordered vertex ids; a closed loop ends with its start vertex repeated.
  function walkChain(start, second) {
    const chain = [start];
    let prev = start;
    let cur = second;
    for (;;) {
      const key = edgeKey(prev, cur);
      if (visited.has(key)) break;
      visited.add(key);
      chain.push(cur);
      if (degree(cur) !== 2) break;
      let next = null;
      for (const nb of adj.get(cur)) {
        if (nb !== prev) {
          next = nb;
          break;
        }
      }
      if (next === null) break;
      prev = cur;
      cur = next;
    }
    return chain;
  }

  const polylines = [];
  const corners = [];
  for (const [v, set] of adj) {
    if (set.size !== 2) corners.push(v);
  }
  // Open chains anchored at junction/endpoint vertices.
  for (const c of corners) {
    for (const nb of adj.get(c)) {
      if (visited.has(edgeKey(c, nb))) continue;
      polylines.push({ ids: walkChain(c, nb), closed: false });
    }
  }
  // Remaining untouched feature edges belong to all-degree-2 closed loops.
  for (const [a, b] of featureEdges) {
    if (visited.has(edgeKey(a, b))) continue;
    const ids = walkChain(a, b);
    if (ids.length > 1 && ids[0] === ids[ids.length - 1]) ids.pop();
    polylines.push({ ids, closed: true });
  }

  const diagonal = meshDiagonal(positions);
  const straightTol = Math.max(diagonal * 1e-3, 1e-9);
  const edges = polylines
    .filter((pl) => pl.ids.length >= 2)
    .map((pl) => {
      const points = pl.ids.map((id) => vertex(positions, id));
      let length = 0;
      for (let i = 1; i < points.length; i += 1) length += dist(points[i - 1], points[i]);
      if (pl.closed) length += dist(points[points.length - 1], points[0]);
      const circle = pl.closed ? fitCircle(points) : null;
      const straight = !pl.closed && maxLineDeviation(points) <= straightTol;
      return {
        points,
        length,
        closed: pl.closed,
        straight,
        endpoints: [points[0], points[points.length - 1]],
        circle,
      };
    });

  return {
    diagonal,
    corners: corners.map((id) => ({ point: vertex(positions, id) })),
    edges,
  };
}

/** Closest point on segment [a, b] to p, and the distance to it. */
function pointToSegment(p, a, b) {
  const ab = sub(b, a);
  const len2 = dot(ab, ab);
  let t = len2 > 0 ? dot(sub(p, a), ab) / len2 : 0;
  t = Math.max(0, Math.min(1, t));
  const cp = [a[0] + ab[0] * t, a[1] + ab[1] * t, a[2] + ab[2] * t];
  return { distance: dist(p, cp), point: cp };
}

/** Min distance from p to a polyline, and the closest point on it. */
function pointToPolyline(p, points, closed) {
  let best = { distance: Infinity, point: points[0] };
  const segs = closed ? points.length : points.length - 1;
  for (let i = 0; i < segs; i += 1) {
    const hit = pointToSegment(p, points[i], points[(i + 1) % points.length]);
    if (hit.distance < best.distance) best = hit;
  }
  return best;
}

/**
 * Snap a raycast hit `point` to the nearest model feature within `tol` world
 * units, preferring a corner vertex, then an edge/rim. Returns a measure
 * entity — `{ kind: 'vertex' | 'edge' | 'circle', ... }` — or null when
 * nothing is close enough (the caller falls back to a planar face or the raw
 * surface point).
 */
export function snapEntity(model, point, tol) {
  let bestCorner = null;
  let bestCornerDist = tol;
  for (const c of model.corners) {
    const d = dist(point, c.point);
    if (d < bestCornerDist) {
      bestCornerDist = d;
      bestCorner = c;
    }
  }
  if (bestCorner) return { kind: 'vertex', point: bestCorner.point };

  let bestEdge = null;
  let bestEdgeDist = tol;
  let bestPoint = null;
  for (const e of model.edges) {
    const hit = pointToPolyline(point, e.points, e.closed);
    if (hit.distance < bestEdgeDist) {
      bestEdgeDist = hit.distance;
      bestEdge = e;
      bestPoint = hit.point;
    }
  }
  if (bestEdge) {
    if (bestEdge.circle) {
      return {
        kind: 'circle',
        center: bestEdge.circle.center,
        radius: bestEdge.circle.radius,
        normal: bestEdge.circle.normal,
        length: bestEdge.length,
        points: bestEdge.points,
        point: bestPoint,
      };
    }
    return {
      kind: 'edge',
      a: bestEdge.endpoints[0],
      b: bestEdge.endpoints[1],
      length: bestEdge.length,
      closed: bestEdge.closed,
      points: bestEdge.points,
      point: bestPoint,
    };
  }
  return null;
}
