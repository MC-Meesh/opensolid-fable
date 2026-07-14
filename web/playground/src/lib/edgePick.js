// Edge picking for click-to-fillet/chamfer (of-rpo).
//
// A boolean edge in the F-Rep model is the crease where two adjacent planar
// face regions meet — the sharp seam a union/subtract leaves between two
// bodies. Given a click that hit a triangle of the displayed mesh, this
// module finds the nearest such crease and recovers the ordered polyline of
// mesh vertices along it. That polyline is exactly what the kernel's
// `filletEdge` / `chamferEdge` bindings take (a flat `[x, y, z, …]` array,
// chained into blend segments; see crates/opensolid-wasm polyline_region).
//
// It leans on the same face-region growth as sketch-on-face
// (createFaceRegionIndex in facePlane.js): the two regions bordering the
// crease are the two operand faces, and their centroids let the caller map
// the edge back to the two operand bodies in the construction tree.
//
// Kept free of three.js / React so it can be unit-tested on plain
// position/index arrays (same pattern as facePlane.js and picking.js).

// Undirected edge key: meshes stay far below 2^26 vertices, so a vertex pair
// packs losslessly into one double (matches facePlane.js).
const EDGE_KEY_BASE = 2 ** 26;

const edgeKey = (p, q) => (p < q ? p * EDGE_KEY_BASE + q : q * EDGE_KEY_BASE + p);

function vertex(positions, index) {
  return [positions[3 * index], positions[3 * index + 1], positions[3 * index + 2]];
}

function triVertexIds(indices, tri) {
  return [indices[3 * tri], indices[3 * tri + 1], indices[3 * tri + 2]];
}

/** Squared distance from point `p` to segment `a`–`b` (all `[x, y, z]`). */
export function pointSegmentDist2(p, a, b) {
  const abx = b[0] - a[0];
  const aby = b[1] - a[1];
  const abz = b[2] - a[2];
  const apx = p[0] - a[0];
  const apy = p[1] - a[1];
  const apz = p[2] - a[2];
  const abLen2 = abx * abx + aby * aby + abz * abz;
  let t = abLen2 > 0 ? (apx * abx + apy * aby + apz * abz) / abLen2 : 0;
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  const cx = a[0] + t * abx - p[0];
  const cy = a[1] + t * aby - p[1];
  const cz = a[2] + t * abz - p[2];
  return cx * cx + cy * cy + cz * cz;
}

/**
 * Map every undirected triangle edge to the triangles that share it:
 * `Map<edgeKey, tri[]>`. A manifold interior edge borders two triangles.
 */
export function buildEdgeTriMap(indices) {
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
      let tris = byEdge.get(key);
      if (!tris) byEdge.set(key, (tris = []));
      tris.push(t);
    }
  }
  return byEdge;
}

/**
 * Order a bag of undirected mesh edges (vertex-id pairs) into a single
 * polyline, restricted to the connected component containing `seedVid`.
 *
 * Walks the crease graph: from an endpoint (degree 1) when the crease is an
 * open arc, or from the seed vertex when it is a closed loop, hopping to an
 * unvisited neighbour each step. Disconnected creases (two bodies touching
 * along separate seams) keep only the seam under the cursor. Returns an array
 * of ordered vertex ids.
 */
export function orderPolyline(edges, seedVid) {
  const adj = new Map();
  const link = (a, b) => {
    let n = adj.get(a);
    if (!n) adj.set(a, (n = new Set()));
    n.add(b);
  };
  for (const [a, b] of edges) {
    link(a, b);
    link(b, a);
  }
  if (adj.size === 0) return [];

  // Connected component containing the seed (fall back to any vertex).
  const start0 = adj.has(seedVid) ? seedVid : adj.keys().next().value;
  const component = new Set([start0]);
  const stack = [start0];
  while (stack.length) {
    const v = stack.pop();
    for (const n of adj.get(v)) {
      if (!component.has(n)) {
        component.add(n);
        stack.push(n);
      }
    }
  }

  // Prefer an endpoint of this component so an open arc walks end-to-end.
  let start = start0;
  for (const v of component) {
    if (adj.get(v).size === 1) {
      start = v;
      break;
    }
  }

  const order = [start];
  const visited = new Set([start]);
  let current = start;
  for (;;) {
    let next = null;
    for (const n of adj.get(current)) {
      if (!visited.has(n)) {
        next = n;
        break;
      }
    }
    if (next === null) break;
    order.push(next);
    visited.add(next);
    current = next;
  }
  return order;
}

/**
 * Find the boolean edge nearest a click and recover its polyline.
 *
 * `regions` is a face-region index (createFaceRegionIndex) over the displayed
 * mesh; `positions`/`indices` are that mesh's buffers; `point` is the world
 * hit point and `faceIndex` the raycast triangle.
 *
 * On success returns
 *   `{ ok: true, points: [[x,y,z], …], flat: [x,y,z, …],
 *      seed: [x,y,z], midpoint: [x,y,z], regionA, regionB }`
 * where `points` is the ordered crease polyline (the kernel input), `regionA`
 * is the clicked planar face and `regionB` the planar face across the crease.
 * On failure returns `{ ok: false, reason }` with a user-facing explanation.
 */
export function pickEdge(regions, positions, indices, point, faceIndex, edgeTriMap = null) {
  if (!regions || faceIndex === null || faceIndex === undefined) {
    return { ok: false, reason: 'click nearer to an edge of the model' };
  }
  const regionA = regions.regionAt(faceIndex);
  if (!regionA.planar) {
    return {
      ok: false,
      reason:
        regionA.reason === 'face is curved'
          ? 'Filleting a curved face is not supported yet — pick a flat face beside the edge'
          : 'Click on a flat face next to the edge to fillet',
    };
  }

  const map = edgeTriMap ?? buildEdgeTriMap(indices);
  const aTris = new Set(regionA.tris);

  // Nearest crease: a boundary edge of region A whose other triangle lies in
  // a different region. That triangle's region is the second operand face.
  let bestDist2 = Infinity;
  let neighborTri = -1;
  for (const tri of regionA.tris) {
    const [a, b, c] = triVertexIds(indices, tri);
    for (const [p, q] of [
      [a, b],
      [b, c],
      [c, a],
    ]) {
      const tris = map.get(edgeKey(p, q));
      if (!tris) continue;
      for (const t of tris) {
        if (aTris.has(t)) continue;
        const d2 = pointSegmentDist2(point, vertex(positions, p), vertex(positions, q));
        if (d2 < bestDist2) {
          bestDist2 = d2;
          neighborTri = t;
        }
      }
    }
  }

  if (neighborTri < 0) {
    return { ok: false, reason: 'no edge borders this face' };
  }

  const regionB = regions.regionAt(neighborTri);
  if (!regionB.planar) {
    return {
      ok: false,
      reason: 'This edge borders a curved face — F-Rep edge fillet needs two flat faces (MVP)',
    };
  }

  // Every mesh edge shared between region A and region B is a crease segment.
  const bTris = new Set(regionB.tris);
  const creaseEdges = [];
  const seenEdge = new Set();
  for (const tri of regionA.tris) {
    const [a, b, c] = triVertexIds(indices, tri);
    for (const [p, q] of [
      [a, b],
      [b, c],
      [c, a],
    ]) {
      const key = edgeKey(p, q);
      if (seenEdge.has(key)) continue;
      const tris = map.get(key);
      if (tris && tris.some((t) => bTris.has(t))) {
        seenEdge.add(key);
        creaseEdges.push([p, q]);
      }
    }
  }

  if (creaseEdges.length === 0) {
    return { ok: false, reason: 'could not trace the edge between these faces' };
  }

  // Seed the ordering from the vertex nearest the click so a multi-loop
  // crease keeps the loop the user aimed at.
  let seedVid = creaseEdges[0][0];
  let seedD2 = Infinity;
  for (const [p, q] of creaseEdges) {
    for (const v of [p, q]) {
      const d2 = pointSegmentDist2(point, vertex(positions, v), vertex(positions, v));
      if (d2 < seedD2) {
        seedD2 = d2;
        seedVid = v;
      }
    }
  }

  const orderedIds = orderPolyline(creaseEdges, seedVid);
  const points = orderedIds.map((v) => vertex(positions, v));
  if (points.length < 2) {
    return { ok: false, reason: 'could not trace the edge between these faces' };
  }

  const flat = [];
  for (const pt of points) flat.push(pt[0], pt[1], pt[2]);
  const mid = points[Math.floor(points.length / 2)];

  return {
    ok: true,
    points,
    flat,
    seed: [point[0], point[1], point[2]],
    midpoint: [mid[0], mid[1], mid[2]],
    regionA,
    regionB,
  };
}
