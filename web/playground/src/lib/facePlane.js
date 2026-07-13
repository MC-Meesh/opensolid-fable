// Face-plane detection for "sketch on a face" (of-4eh.16) and face
// hover/selection highlighting (of-4eh.18): given a triangle of the
// displayed F-Rep mesh, grow the maximal connected planar region around it
// and derive a sketch plane `{ origin, normal, u, v, extent }` — origin at
// the region centroid, (u, v) a right-handed WYSIWYG basis (u × v = normal),
// extent the region radius for indicator/grid sizing. Results also carry the
// region's triangle indices (`tris`) so callers can paint the face. Curved
// surfaces are rejected so the caller can explain instead of silently
// sketching on a facet.
//
// Kept free of three.js and React so it can be unit-tested on plain
// position/index arrays (same pattern as picking.js).

/** Angular tolerance for admitting a triangle into the seed's region. */
const NORMAL_TOL_DEG = 3;
/** Max normal spread inside an accepted region before it counts as curved. */
const SPREAD_TOL_DEG = 0.8;
/** Plane-offset tolerance as a fraction of the mesh bounding-box diagonal. */
const OFFSET_TOL_FACTOR = 2e-3;
/** Regions smaller than this are curved-surface facets, not real faces:
 * marching cubes emits many cell-sized triangles per planar face. */
const MIN_REGION_TRIS = 3;

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

/** Unnormalized triangle normal (length = 2 * area). */
function triAreaNormal(positions, indices, tri) {
  const [a, b, c] = triVertexIds(indices, tri).map((i) => vertex(positions, i));
  return cross(sub(b, a), sub(c, a));
}

/**
 * Stable in-plane basis for a face normal: `v` is the projection of world Y
 * (screen-up once the camera is normal-to), falling back to -Z for
 * near-horizontal faces; `u = v × n`, so `u × v = n`. Reproduces the named
 * planes' conventions exactly: n=+Z → (X, Y), n=+Y → (X, -Z), n=+X → (-Z, Y).
 */
export function facePlaneBasis(normal) {
  const n = normalize(normal);
  if (!n) throw new Error('face normal is zero');
  const ref = Math.abs(n[1]) < 0.99 ? [0, 1, 0] : [0, 0, -1];
  const d = dot(ref, n);
  const v = normalize([ref[0] - d * n[0], ref[1] - d * n[1], ref[2] - d * n[2]]);
  const u = cross(v, n);
  return { u, v };
}

/**
 * Axis-angle of the rotation whose columns are `(c0, c1, c2)` (orthonormal,
 * det +1), via the quaternion form: `{ axis: [x, y, z], angle }` in radians,
 * or `null` for the identity. Used to serialize a face-plane orientation as
 * a single `rotate(ax, ay, az, angle)` op.
 */
export function axisAngleFromBasis(c0, c1, c2) {
  const [m00, m10, m20] = c0;
  const [m01, m11, m21] = c1;
  const [m02, m12, m22] = c2;
  const trace = m00 + m11 + m22;
  let x;
  let y;
  let z;
  let w;
  if (trace > 0) {
    const s = 2 * Math.sqrt(trace + 1);
    w = s / 4;
    x = (m21 - m12) / s;
    y = (m02 - m20) / s;
    z = (m10 - m01) / s;
  } else if (m00 >= m11 && m00 >= m22) {
    const s = 2 * Math.sqrt(1 + m00 - m11 - m22);
    x = s / 4;
    y = (m01 + m10) / s;
    z = (m02 + m20) / s;
    w = (m21 - m12) / s;
  } else if (m11 >= m22) {
    const s = 2 * Math.sqrt(1 + m11 - m00 - m22);
    x = (m01 + m10) / s;
    y = s / 4;
    z = (m12 + m21) / s;
    w = (m02 - m20) / s;
  } else {
    const s = 2 * Math.sqrt(1 + m22 - m00 - m11);
    x = (m02 + m20) / s;
    y = (m12 + m21) / s;
    z = s / 4;
    w = (m21 - m12) / s;
  }
  const sinHalf = Math.hypot(x, y, z);
  if (sinHalf < 1e-9) return null;
  const angle = 2 * Math.atan2(sinHalf, w);
  return { axis: [x / sinHalf, y / sinHalf, z / sinHalf], angle };
}

// Numeric undirected edge key: meshes stay far below 2^26 vertices, so the
// pair packs losslessly into one double (and Map lookups stay cheap on the
// ~100k-triangle meshes a click has to classify interactively).
const EDGE_KEY_BASE = 2 ** 26;

/** Edge-adjacency over triangles: shared undirected vertex-id pairs. */
function buildAdjacency(indices) {
  const triCount = indices.length / 3;
  const byEdge = new Map();
  for (let t = 0; t < triCount; t += 1) {
    const [a, b, c] = triVertexIds(indices, t);
    for (const [p, q] of [[a, b], [b, c], [c, a]]) {
      const key = p < q ? p * EDGE_KEY_BASE + q : q * EDGE_KEY_BASE + p;
      let tris = byEdge.get(key);
      if (!tris) byEdge.set(key, (tris = []));
      tris.push(t);
    }
  }
  const neighbors = Array.from({ length: triCount }, () => []);
  for (const tris of byEdge.values()) {
    for (const t of tris) {
      for (const other of tris) {
        if (other !== t) neighbors[t].push(other);
      }
    }
  }
  return neighbors;
}

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
 * Grow the connected planar region around seed triangle `faceIndex` and
 * classify it (see `detectFacePlane`). `neighbors` and `offsetTol` are
 * precomputed by the caller so an index over one mesh can amortize them.
 */
function growFaceRegion(positions, indices, faceIndex, neighbors, offsetTol) {
  const seedArea = triAreaNormal(positions, indices, faceIndex);
  const seedNormal = normalize(seedArea);
  if (!seedNormal) {
    return { planar: false, reason: 'degenerate triangle under the cursor', tris: [faceIndex] };
  }
  const seedPoint = vertex(positions, triVertexIds(indices, faceIndex)[0]);
  const cosTol = Math.cos((NORMAL_TOL_DEG * Math.PI) / 180);

  const accepted = new Set([faceIndex]);
  const queue = [faceIndex];
  while (queue.length > 0) {
    const tri = queue.pop();
    for (const next of neighbors[tri]) {
      if (accepted.has(next)) continue;
      const areaN = triAreaNormal(positions, indices, next);
      const n = normalize(areaN);
      if (!n || dot(n, seedNormal) < cosTol) continue;
      const offPlane = triVertexIds(indices, next).some(
        (i) => Math.abs(dot(sub(vertex(positions, i), seedPoint), seedNormal)) > offsetTol
      );
      if (offPlane) continue;
      accepted.add(next);
      queue.push(next);
    }
  }

  const tris = Array.from(accepted);
  if (accepted.size < MIN_REGION_TRIS) {
    return { planar: false, reason: 'face is too small to classify as planar', tris };
  }

  // Area-weighted average normal and centroid over the region.
  const avgNormal = [0, 0, 0];
  const centroid = [0, 0, 0];
  let totalArea = 0;
  for (const tri of accepted) {
    const areaN = triAreaNormal(positions, indices, tri);
    const area = norm(areaN) / 2;
    const verts = triVertexIds(indices, tri).map((i) => vertex(positions, i));
    for (let k = 0; k < 3; k += 1) {
      avgNormal[k] += areaN[k] / 2;
      centroid[k] += ((verts[0][k] + verts[1][k] + verts[2][k]) / 3) * area;
    }
    totalArea += area;
  }
  const normal = normalize(avgNormal);
  if (!normal || totalArea <= 0) {
    return { planar: false, reason: 'face region has no area', tris };
  }
  for (let k = 0; k < 3; k += 1) centroid[k] /= totalArea;

  const cosSpread = Math.cos((SPREAD_TOL_DEG * Math.PI) / 180);
  for (const tri of accepted) {
    const n = normalize(triAreaNormal(positions, indices, tri));
    if (n && dot(n, normal) < cosSpread) {
      return { planar: false, reason: 'face is curved', tris };
    }
  }

  let extent = 0;
  const seen = new Set();
  for (const tri of accepted) {
    for (const i of triVertexIds(indices, tri)) {
      if (seen.has(i)) continue;
      seen.add(i);
      const d = norm(sub(vertex(positions, i), centroid));
      if (d > extent) extent = d;
    }
  }

  const { u, v } = facePlaneBasis(normal);
  return {
    planar: true,
    plane: { origin: centroid, normal, u, v, extent },
    tris,
  };
}

/**
 * Region index over one mesh: `regionAt(faceIndex)` classifies the face
 * containing that triangle (same result shape as `detectFacePlane`).
 * Adjacency and the offset tolerance are built lazily once, and every
 * triangle of a computed region maps to the shared result object — so
 * pointer-move hovers inside one face are O(1) after the first hit, and two
 * seeds on the same face always yield the identical region (no highlight
 * shimmer between neighboring triangles).
 */
export function createFaceRegionIndex(positions, indices) {
  const triCount = indices.length / 3;
  let neighbors = null;
  let offsetTol = null;
  const byTri = new Map();

  function regionAt(faceIndex) {
    if (!Number.isInteger(faceIndex) || faceIndex < 0 || faceIndex >= triCount) {
      return { planar: false, reason: 'no triangle under the cursor', tris: [] };
    }
    const cached = byTri.get(faceIndex);
    if (cached) return cached;
    neighbors ??= buildAdjacency(indices);
    offsetTol ??= OFFSET_TOL_FACTOR * meshDiagonal(positions) + 1e-12;
    const result = growFaceRegion(positions, indices, faceIndex, neighbors, offsetTol);
    for (const tri of result.tris) byTri.set(tri, result);
    return result;
  }

  return { regionAt };
}

/**
 * Classify the mesh face under the clicked triangle `faceIndex` and derive
 * its sketch plane.
 *
 * Grows the connected region of triangles whose normals stay within
 * `NORMAL_TOL_DEG` of the seed's and whose vertices stay within a small
 * offset of the seed's plane, then checks the region reads as planar: enough
 * triangles and a normal spread under `SPREAD_TOL_DEG` (a curved surface
 * fills the admission tolerance; a true face's spread is float noise).
 *
 * Returns `{ planar: true, plane: { origin, normal, u, v, extent }, tris }`
 * or `{ planar: false, reason, tris }` — `tris` is the grown region's
 * triangle indices either way. Repeated queries over one mesh should use
 * `createFaceRegionIndex` instead, which amortizes the adjacency build.
 */
export function detectFacePlane(positions, indices, faceIndex) {
  return createFaceRegionIndex(positions, indices).regionAt(faceIndex);
}
