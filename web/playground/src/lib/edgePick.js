// Feature-edge picking for the edge-selective fillet/chamfer tool (of-rpo).
//
// A CSG feature edge is a *crease* in the displayed F-Rep mesh: an undirected
// mesh edge whose two adjacent triangles meet at a sharp dihedral angle. Facets
// on a curved surface (the many cell-sized triangles marching cubes emits) meet
// almost flat and are rejected; the intersection curve a boolean carves between
// two primitives stays sharp and is kept. Given a viewport click's world hit
// point we find the nearest crease edge and chain the connected crease edges
// through it into a single polyline — the flat `[x0,y0,z0, x1,y1,z1, …]` array
// `Shape.filletEdge` / `Shape.chamferEdge` consume (see
// docs/design/edge-fillet-chamfer.md §2).
//
// Kept free of three.js and React so it can be unit-tested on plain
// position/index arrays (same pattern as facePlane.js and picking.js).

/** Dihedral angle (degrees) above which a shared edge counts as a crease.
 * CSG intersection curves between analytic primitives are near-orthogonal;
 * adjacent facets of a meshed curved surface stay a few degrees apart, so a
 * generous threshold separates the two cleanly. */
const CREASE_ANGLE_DEG = 25;
/** Stop chaining across a corner in the crease curve itself: a continuation is
 * only followed when the direction turns by less than this. */
const MAX_TURN_DEG = 45;

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
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

/** Unit triangle normal, or null if degenerate. */
function triNormal(positions, indices, tri) {
  const [a, b, c] = triVertexIds(indices, tri).map((i) => vertex(positions, i));
  return normalize(cross(sub(b, a), sub(c, a)));
}

// Numeric undirected edge key: meshes stay far below 2^26 vertices, so a vertex
// pair packs losslessly into one double (same trick as facePlane.js).
const EDGE_KEY_BASE = 2 ** 26;
const edgeKey = (p, q) => (p < q ? p * EDGE_KEY_BASE + q : q * EDGE_KEY_BASE + p);

/** Closest point on segment [a, b] to p, and its squared distance to p. */
function closestOnSegment(p, a, b) {
  const ab = sub(b, a);
  const len2 = dot(ab, ab);
  let t = len2 > 0 ? dot(sub(p, a), ab) / len2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const q = [a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2]];
  const d = sub(p, q);
  return { point: q, dist2: dot(d, d) };
}

/**
 * Collect the crease edges of a mesh: undirected vertex pairs whose two
 * adjacent triangles differ in normal by more than `CREASE_ANGLE_DEG` (a
 * boundary edge with a single triangle also counts). Returns `edges` — an
 * array of `{ a, b }` vertex-id pairs — and `vertexEdges`, a Map from each
 * crease vertex id to the indices (into `edges`) of the crease edges touching
 * it. Amortized once per mesh by `createEdgePickIndex`.
 */
export function collectCreaseEdges(positions, indices) {
  const triCount = indices.length / 3;
  const byEdge = new Map();
  for (let t = 0; t < triCount; t += 1) {
    const [a, b, c] = triVertexIds(indices, t);
    for (const [p, q] of [
      [a, b],
      [b, c],
      [c, a],
    ]) {
      const key = edgeKey(p, q);
      let rec = byEdge.get(key);
      if (!rec) byEdge.set(key, (rec = { a: p < q ? p : q, b: p < q ? q : p, tris: [] }));
      rec.tris.push(t);
    }
  }

  const cosTol = Math.cos((CREASE_ANGLE_DEG * Math.PI) / 180);
  const edges = [];
  for (const rec of byEdge.values()) {
    let crease;
    if (rec.tris.length === 1) {
      crease = true; // open boundary — always a feature edge
    } else if (rec.tris.length === 2) {
      const n0 = triNormal(positions, indices, rec.tris[0]);
      const n1 = triNormal(positions, indices, rec.tris[1]);
      crease = !n0 || !n1 || dot(n0, n1) < cosTol;
    } else {
      crease = true; // non-manifold junction — treat as a feature edge
    }
    if (crease) edges.push({ a: rec.a, b: rec.b });
  }

  const vertexEdges = new Map();
  edges.forEach((edge, index) => {
    for (const v of [edge.a, edge.b]) {
      let list = vertexEdges.get(v);
      if (!list) vertexEdges.set(v, (list = []));
      list.push(index);
    }
  });

  return { edges, vertexEdges };
}

/**
 * Walk the crease-edge graph from one endpoint of the seed edge, appending the
 * straightest continuation at each degree-2 vertex and stopping at junctions
 * (a vertex with any number of crease edges other than two), at a corner in the
 * curve (a turn beyond `MAX_TURN_DEG`), or when the walk closes a loop. Returns
 * the ordered vertex-id list *excluding* the seed edge's two endpoints, in
 * traversal order away from `fromVertex`.
 */
function walkChain(positions, edges, vertexEdges, seedIndex, fromVertex, otherVertex) {
  const out = [];
  const visited = new Set([seedIndex]);
  let prev = otherVertex;
  let current = fromVertex;
  let prevDir = normalize(sub(vertex(positions, current), vertex(positions, prev)));
  const cosTurn = Math.cos((MAX_TURN_DEG * Math.PI) / 180);

  for (;;) {
    const incident = vertexEdges.get(current) ?? [];
    if (incident.length !== 2) break; // junction or dead end — stop cleanly
    const nextIndex = incident.find((i) => !visited.has(i));
    if (nextIndex === undefined) break; // closed loop back onto a visited edge
    const edge = edges[nextIndex];
    const nextVertex = edge.a === current ? edge.b : edge.a;
    const dir = normalize(sub(vertex(positions, nextVertex), vertex(positions, current)));
    if (prevDir && dir && dot(prevDir, dir) < cosTurn) break; // sharp corner
    visited.add(nextIndex);
    out.push(nextVertex);
    prev = current;
    current = nextVertex;
    prevDir = dir;
  }
  return out;
}

/**
 * Edge-pick index over one mesh: `pickAt(worldPoint)` returns the feature edge
 * nearest the point as a chained polyline, or `null` when the mesh has no
 * crease edges. Crease topology is built lazily on the first pick and reused.
 *
 * Result shape:
 *   `{ seed: [x,y,z], points: [[x,y,z], …], polyline: [x0,y0,z0, …], segments,
 *      dist }`
 * — `seed` is the closest point on the picked polyline to `worldPoint`,
 * `points` the ordered chain vertices, `polyline` the flat array the WASM
 * `filletEdge`/`chamferEdge` bindings consume, `segments` the edge count, and
 * `dist` the pick distance (for a hover/selection threshold).
 */
export function createEdgePickIndex(positions, indices) {
  let topo = null;

  function pickAt(worldPoint) {
    if (!worldPoint || worldPoint.length < 3) return null;
    topo ??= collectCreaseEdges(positions, indices);
    const { edges, vertexEdges } = topo;
    if (edges.length === 0) return null;

    // Nearest crease edge to the click.
    let seedIndex = -1;
    let bestDist2 = Infinity;
    let seedPoint = null;
    for (let i = 0; i < edges.length; i += 1) {
      const a = vertex(positions, edges[i].a);
      const b = vertex(positions, edges[i].b);
      const { point, dist2 } = closestOnSegment(worldPoint, a, b);
      if (dist2 < bestDist2) {
        bestDist2 = dist2;
        seedIndex = i;
        seedPoint = point;
      }
    }
    if (seedIndex < 0) return null;

    const seed = edges[seedIndex];
    const forward = walkChain(positions, edges, vertexEdges, seedIndex, seed.b, seed.a);
    const backward = walkChain(positions, edges, vertexEdges, seedIndex, seed.a, seed.b);
    // backward runs away from the seed; reverse it so the chain reads start→end.
    const ordered = [...backward.reverse(), seed.a, seed.b, ...forward];

    const points = ordered.map((v) => vertex(positions, v));
    const polyline = [];
    for (const p of points) polyline.push(p[0], p[1], p[2]);

    return {
      seed: seedPoint,
      points,
      polyline,
      segments: points.length - 1,
      dist: Math.sqrt(bestDist2),
    };
  }

  return { pickAt };
}
