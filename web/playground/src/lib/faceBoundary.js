/**
 * Boundary-loop extraction and projection for "Convert Entities".
 *
 * A planar face is stored as a set of mesh triangles (`tris`, indices into the
 * displayed F-Rep mesh — the same region facePlane.js grows). Its outline is
 * the set of edges used by exactly one triangle of the region; chaining those
 * boundary edges yields ordered loops, which project into the sketch plane's
 * (u, v) basis to become 2D geometry the sketch can adopt.
 *
 * Kept free of three.js/React so it unit-tests on plain position/index arrays,
 * matching facePlane.js / picking.js.
 */

// Meshes stay well below 2^26 vertices, so an undirected edge packs into one
// double (same trick as facePlane.js adjacency).
const EDGE_KEY_BASE = 2 ** 26;

const edgeKey = (a, b) => (a < b ? a * EDGE_KEY_BASE + b : b * EDGE_KEY_BASE + a);

function vertex(positions, i) {
  return [positions[3 * i], positions[3 * i + 1], positions[3 * i + 2]];
}

/**
 * Ordered boundary loops of the triangle region `tris` (each a list of vertex
 * ids into `positions`). Boundary edges border exactly one region triangle;
 * they chain into closed loops (outer boundary plus any holes). Degenerate or
 * open fragments are skipped.
 */
export function regionBoundaryVertexLoops(indices, tris) {
  const count = new Map(); // edgeKey → times seen within the region
  const region = new Set(tris);
  for (const t of region) {
    const a = indices[3 * t];
    const b = indices[3 * t + 1];
    const c = indices[3 * t + 2];
    for (const [p, q] of [
      [a, b],
      [b, c],
      [c, a],
    ]) {
      const k = edgeKey(p, q);
      count.set(k, (count.get(k) ?? 0) + 1);
    }
  }

  // Undirected adjacency over boundary edges (count === 1).
  const adj = new Map();
  const addAdj = (p, q) => {
    if (!adj.has(p)) adj.set(p, []);
    adj.get(p).push(q);
  };
  for (const t of region) {
    const a = indices[3 * t];
    const b = indices[3 * t + 1];
    const c = indices[3 * t + 2];
    for (const [p, q] of [
      [a, b],
      [b, c],
      [c, a],
    ]) {
      if (count.get(edgeKey(p, q)) === 1) {
        addAdj(p, q);
        addAdj(q, p);
      }
    }
  }

  const used = new Set(); // consumed undirected edges
  const loops = [];
  for (const start of adj.keys()) {
    // Begin a loop from any unused edge leaving `start`.
    let next = (adj.get(start) || []).find((n) => !used.has(edgeKey(start, n)));
    if (next === undefined) continue;
    const loop = [start];
    let prev = start;
    let cur = next;
    used.add(edgeKey(prev, cur));
    while (cur !== start) {
      loop.push(cur);
      const step = (adj.get(cur) || []).find(
        (n) => n !== prev && !used.has(edgeKey(cur, n))
      );
      if (step === undefined) break; // open fragment; abandon
      used.add(edgeKey(cur, step));
      prev = cur;
      cur = step;
    }
    if (cur === start && loop.length >= 3) {
      loop.push(start); // close it
      loops.push(loop);
    }
  }
  return loops;
}

/**
 * Project a loop of 3D vertex ids into the sketch plane's 2D (u, v) frame:
 * `uv = ((P - origin)·u, (P - origin)·v)`. `plane` is `{ origin, u, v }` from
 * facePlane.js.
 */
export function projectLoopToPlane(positions, vertexLoop, plane) {
  const { origin, u, v } = plane;
  return vertexLoop.map((id) => {
    const [x, y, z] = vertex(positions, id);
    const dx = x - origin[0];
    const dy = y - origin[1];
    const dz = z - origin[2];
    return [
      dx * u[0] + dy * u[1] + dz * u[2],
      dx * v[0] + dy * v[1] + dz * v[2],
    ];
  });
}

/**
 * Full convert-entities source: the face region's boundary loops as 2D
 * sketch-plane polylines, ready for `convertEntities`.
 */
export function faceBoundaryLoops(positions, indices, tris, plane) {
  return regionBoundaryVertexLoops(indices, tris).map((loop) =>
    projectLoopToPlane(positions, loop, plane)
  );
}
