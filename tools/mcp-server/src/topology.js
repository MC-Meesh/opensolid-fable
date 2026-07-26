// Topology interrogation for agents (of-2y4.5): recover a *countable*
// description of a part — planar faces, circular rims, holes, shells, genus —
// from the mesh it is measured on, and answer "is there a hole through this
// point along this axis?" directly.
//
// Why this is the thing that was missing. The friction log
// (docs/dogfood-bracket-friction-log.md) records a bracket whose four M5 holes
// were bored sideways through the plates: `validate` said `valid: true`, the
// screenshot looked plausible, STL wrote fine, and the only oracle that caught
// it was a volume compared against a number computed by hand. Every tool the
// agent had reported *scalars over the whole part* — a volume, an area, a
// bounding box — and a wrong-axis hole removes very nearly the right amount of
// material, so scalars are close to blind to it. What separates the two parts
// instantly is their *structure*: four Ø5 bores along +Z, not along +Y. So this
// module reports structure.
//
// Three independent computations, in increasing order of how much they can be
// trusted:
//
//   1. `segmentPlanarFaces` — planar faces by seed-plane region growth, with
//      the area they do *not* account for reported rather than hidden.
//   2. `detectRims` / `classifyCylinders` — circular rims from the mesh's
//      feature graph, paired into holes and bosses, with the hole/boss verdict
//      decided by the field rather than by topology.
//   3. `meshTopology` — V − E + F over the welded triangle mesh, giving shell
//      count and genus. Pure combinatorics: no tolerance, no fit, and no drift
//      with meshing accuracy. A plate with four drilled holes has genus 4, and
//      no accuracy setting can make it say otherwise.
//   4. `probeAxis` — the signed distance field sampled along a line. Independent
//      of meshing entirely.
//
// The feature-graph half descends from the playground's `measureTopology.js`
// (of-fsl.17) and the planar-face half from its `facePlane.js` (of-4eh.18),
// both of which had this logic locked inside GUI interactions. What did *not*
// survive the port is that module's straight-edge and corner-vertex tracing:
// snapping a cursor to the nearest entity tolerates a noisy feature graph,
// counting does not. On `box3(10,5,8)` at accuracy 0.1 the dihedral test flags
// 4616 of 25650 mesh edges as creases — the dual-contouring mesher renders a
// sharp edge as a one-cell bevel, so those counts come out as 4592 "model
// edges" and 4400 "corner vertices" on a shape with 12 and 8. A count that
// wrong is worse than no count: the whole complaint in the friction log is
// oracles that answer confidently and wrongly. Circular rims survive because
// they can be *fitted* and the fit either holds or does not.
//
// Everything but `probeAxis`/`classifyCylinders` takes plain arrays, so it is
// unit-testable on hand-written meshes.

/** Dihedral angle (degrees) above which a shared edge counts as a crease. */
const CREASE_TOL_DEG = 25;
/** A fitted circle's inlier residual must stay under this fraction of radius. */
const CIRCLE_FIT_TOL = 0.04;
/** Components smaller than this can't be told apart from a triangle corner. */
const MIN_CIRCLE_VERTS = 8;
/** At least this fraction of a component's vertices must fit the circle. */
const MIN_CIRCLE_INLIER_FRACTION = 0.5;
/** Outlier-trimming rounds before a component is judged not a circle. */
const TRIM_ROUNDS = 8;
/** Each round cuts points beyond this multiple of the median residual. */
const TRIM_MEDIAN_FACTOR = 2.5;
/** Inliers must span at least this much of the circle, so an arc is not a rim. */
const MIN_CIRCLE_COVERAGE_DEG = 300;
/** Two rims pair into one cylindrical feature within this fraction of radius. */
const RIM_PAIR_TOL = 0.08;
/** Rim normals must be this close to parallel (degrees) to pair. */
const RIM_AXIS_TOL_DEG = 10;
/** Samples per axis probe. Features thinner than span/this can slip through. */
const PROBE_SAMPLES = 2048;
/** Bisection steps refining each sign change to a crossing point. */
const PROBE_REFINE = 40;
/** Interior samples along a bore before calling it clear of material. */
const BORE_SAMPLES = 24;
/** Step past each rim, as a fraction of the feature's smaller dimension. */
const END_STEP_FRACTION = 0.2;

// Planar-face recovery, matching web/playground/src/lib/facePlane.js so the two
// agree about what a face is.
/** A triangle joins a face when its normal is within this of the seed's. */
const NORMAL_TOL_DEG = 3;
/** ...and its vertices lie within this fraction of the mesh diagonal of it. */
const OFFSET_TOL_FACTOR = 2e-3;
/**
 * A planar region below this fraction of the total area counts as remainder.
 *
 * Area, not triangle count. `facePlane.js` screens on a minimum triangle count
 * because it classifies the one region under a cursor and a lone triangle there
 * is a mis-click; that test does not transfer to a census. An SDF mesh gives a
 * flat face hundreds of triangles, but the STEP reader's analytic tessellation
 * gives a rectangular face exactly **two** — so a `>= 3` screen silently drops
 * every rectangular face of every imported body (measured: 2 faces reported for
 * a re-imported drilled plate that has 6). Area is the scale-free question, and
 * it rejects the bevel slivers the count screen was reaching for anyway.
 */
const MIN_FACE_AREA_FRACTION = 5e-3;

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

function triArea(positions, indices, tri) {
  const [a, b, c] = triVertexIds(indices, tri).map((i) => vertex(positions, i));
  return 0.5 * norm(cross(sub(b, a), sub(c, a)));
}

// Numeric undirected edge key: meshes stay far below 2^26 vertices, so the
// pair packs losslessly into one double (mirrors the playground's facePlane.js).
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
 * Eigenvector of a symmetric 3×3 matrix (row-major 3×3 array) for its smallest
 * eigenvalue, by cyclic Jacobi rotations.
 *
 * Used to fit a plane to a point set: the smallest-eigenvalue direction of the
 * covariance is the plane normal. Plain Jacobi rather than the analytic cubic
 * because the analytic form loses precision badly on the near-degenerate
 * covariance a ring of coplanar points produces (one eigenvalue near zero).
 */
export function smallestEigenvector(matrix) {
  // Working copy of the matrix, and the accumulated rotation.
  const a = matrix.map((row) => row.slice());
  let v = [
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
  ];
  for (let sweep = 0; sweep < 24; sweep += 1) {
    let off = 0;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) off += a[p][q] * a[p][q];
    if (off < 1e-30) break;
    for (const [p, q] of [[0, 1], [0, 2], [1, 2]]) {
      if (Math.abs(a[p][q]) < 1e-300) continue;
      const theta = (a[q][q] - a[p][p]) / (2 * a[p][q]);
      const t = Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
      const c = 1 / Math.sqrt(t * t + 1);
      const s = t * c;
      // Rotate rows/columns p,q of `a`, and columns p,q of `v`.
      for (let k = 0; k < 3; k += 1) {
        const akp = a[k][p];
        const akq = a[k][q];
        a[k][p] = c * akp - s * akq;
        a[k][q] = s * akp + c * akq;
      }
      for (let k = 0; k < 3; k += 1) {
        const apk = a[p][k];
        const aqk = a[q][k];
        a[p][k] = c * apk - s * aqk;
        a[q][k] = s * apk + c * aqk;
      }
      for (let k = 0; k < 3; k += 1) {
        const vkp = v[k][p];
        const vkq = v[k][q];
        v[k][p] = c * vkp - s * vkq;
        v[k][q] = s * vkp + c * vkq;
      }
    }
  }
  let best = 0;
  for (let i = 1; i < 3; i += 1) {
    if (a[i][i] < a[best][best]) best = i;
  }
  return normalize([v[0][best], v[1][best], v[2][best]]);
}

/**
 * Fit a circle to a 3D point set, tolerating outliers:
 * `{ center, radius, normal, inliers, total, coverageDeg }` or null.
 *
 * Plane by PCA (smallest covariance eigenvector), circle in that plane by the
 * algebraic (Kåsa) least-squares fit, then one trimming pass that drops points
 * whose radial residual exceeds the tolerance and refits on what is left.
 *
 * The trimming is the point. A hole rim recovered from the mesh's feature graph
 * does not arrive as a clean ring: the crease test also fires on the mesher's
 * bevel around the rim, hanging short spurs off it. Measured on a 30×5×20 plate
 * with four Ø5 bores at accuracy 0.2, each rim component came back with 186
 * vertices of which 37 had feature-degree ≠ 2 — so the ordered-ring fit this
 * replaces (which required *every* vertex to have degree 2) found all four rims
 * on a one-hole plate and none at all on the four-hole one. An order-free fit
 * with outlier rejection finds them in both.
 *
 * `coverageDeg` guards the other direction: a fillet's arc fits a circle
 * perfectly well, so the inliers must also wrap most of the way around one
 * before it counts as a rim.
 */
export function fitCircle(points) {
  const total = points.length;
  if (total < 3) return null;

  const centroidOf = (pts) => scale(pts.reduce(add, [0, 0, 0]), 1 / pts.length);

  const planeNormal = (pts) => {
    const c = centroidOf(pts);
    const m = [
      [0, 0, 0],
      [0, 0, 0],
      [0, 0, 0],
    ];
    for (const p of pts) {
      const d = sub(p, c);
      for (let i = 0; i < 3; i += 1) {
        for (let j = 0; j < 3; j += 1) m[i][j] += d[i] * d[j];
      }
    }
    return smallestEigenvector(m);
  };

  // In-plane circle fit: solve the normal equations of |x²+y² + Dx + Ey + F| = 0.
  const fitInPlane = (pts, origin, u, v) => {
    const xy = pts.map((p) => {
      const d = sub(p, origin);
      return [dot(d, u), dot(d, v)];
    });
    let sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, sz = 0, sxz = 0, syz = 0;
    for (const [x, y] of xy) {
      const z = x * x + y * y;
      sx += x;
      sy += y;
      sxx += x * x;
      syy += y * y;
      sxy += x * y;
      sz += z;
      sxz += x * z;
      syz += y * z;
    }
    const n = xy.length;
    // [sxx sxy sx; sxy syy sy; sx sy n] · [D E F]ᵀ = −[sxz syz sz]ᵀ
    const m = [
      [sxx, sxy, sx, -sxz],
      [sxy, syy, sy, -syz],
      [sx, sy, n, -sz],
    ];
    // Gauss-Jordan with partial pivoting.
    for (let col = 0; col < 3; col += 1) {
      let piv = col;
      for (let r = col + 1; r < 3; r += 1) {
        if (Math.abs(m[r][col]) > Math.abs(m[piv][col])) piv = r;
      }
      if (Math.abs(m[piv][col]) < 1e-14) return null;
      [m[col], m[piv]] = [m[piv], m[col]];
      const d = m[col][col];
      for (let k = col; k < 4; k += 1) m[col][k] /= d;
      for (let r = 0; r < 3; r += 1) {
        if (r === col) continue;
        const f = m[r][col];
        for (let k = col; k < 4; k += 1) m[r][k] -= f * m[col][k];
      }
    }
    const [D, E, F] = [m[0][3], m[1][3], m[2][3]];
    const cx = -D / 2;
    const cy = -E / 2;
    const rsq = cx * cx + cy * cy - F;
    if (!(rsq > 0)) return null;
    return { cx, cy, radius: Math.sqrt(rsq), xy };
  };

  const attempt = (pts) => {
    if (pts.length < 3) return null;
    const normal = planeNormal(pts);
    if (!normal) return null;
    const origin = centroidOf(pts);
    // Any orthonormal basis of the plane; the fit is basis-independent.
    const seed = Math.abs(normal[0]) < 0.9 ? [1, 0, 0] : [0, 1, 0];
    const u = normalize(cross(normal, seed));
    if (!u) return null;
    const v = cross(normal, u);
    const fit = fitInPlane(pts, origin, u, v);
    if (!fit) return null;
    const center = add(origin, add(scale(u, fit.cx), scale(v, fit.cy)));
    const residuals = fit.xy.map(([x, y]) =>
      Math.abs(Math.hypot(x - fit.cx, y - fit.cy) - fit.radius),
    );
    const angles = fit.xy.map(([x, y]) => Math.atan2(y - fit.cy, x - fit.cx));
    return { center, radius: fit.radius, normal, residuals, angles };
  };

  // Progressive trimming. A single trim-and-refit is not enough: the *first*
  // plane and circle are fitted over the outliers too, so a rim component
  // carrying a third of its vertices as bevel noise can have its initial
  // estimate pulled far enough off that the strict threshold then keeps the
  // wrong points. Cutting at a multiple of the *median* residual each round
  // removes the worst offenders without committing to a threshold the current
  // estimate has not earned, and re-fitting the plane each time lets the
  // estimate walk back onto the ring. Measured on the gallery bracket at
  // accuracy 0.15, one-shot trimming found 3 of its 8 rims; this finds all 8.
  let fit = attempt(points);
  if (!fit) return null;
  let kept = points;
  for (let round = 0; round < TRIM_ROUNDS; round += 1) {
    const strict = CIRCLE_FIT_TOL * fit.radius;
    if (Math.max(...fit.residuals) <= strict) break;
    const sortedResiduals = [...fit.residuals].sort((a, b) => a - b);
    const median = sortedResiduals[Math.floor(sortedResiduals.length / 2)];
    const cut = Math.max(strict, TRIM_MEDIAN_FACTOR * median);
    const next = kept.filter((_, i) => fit.residuals[i] <= cut);
    if (next.length === kept.length || next.length < MIN_CIRCLE_VERTS) break;
    const refit = attempt(next);
    if (!refit) break;
    kept = next;
    fit = refit;
  }

  if (kept.length < MIN_CIRCLE_VERTS) return null;
  if (kept.length / total < MIN_CIRCLE_INLIER_FRACTION) return null;
  if (Math.max(...fit.residuals) > CIRCLE_FIT_TOL * fit.radius) return null;

  // Angular coverage: the largest gap between consecutive inlier angles says
  // whether they wrap the circle or only span an arc of it.
  const sorted = [...fit.angles].sort((a, b) => a - b);
  let biggestGap = sorted[0] + 2 * Math.PI - sorted[sorted.length - 1];
  for (let i = 1; i < sorted.length; i += 1) {
    biggestGap = Math.max(biggestGap, sorted[i] - sorted[i - 1]);
  }
  const coverageDeg = ((2 * Math.PI - biggestGap) * 180) / Math.PI;
  if (coverageDeg < MIN_CIRCLE_COVERAGE_DEG) return null;

  return {
    center: fit.center,
    radius: fit.radius,
    normal: fit.normal,
    inliers: kept.length,
    total,
    coverageDeg,
  };
}

/**
 * Connected components of the mesh's *feature graph*: the sub-graph of mesh
 * edges that are creases (a dihedral above `CREASE_TOL_DEG`) or boundaries (one
 * adjacent triangle). Returns an array of vertex-id arrays.
 *
 * The `|dot|` dihedral form is winding-independent (a real solid never folds
 * back to ~0°). Ported from web/playground/src/lib/measureTopology.js.
 */
export function featureComponents(positions, indices) {
  const triCount = indices.length / 3;
  if (triCount === 0) return [];

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

  const cosCrease = Math.cos((CREASE_TOL_DEG * Math.PI) / 180);
  const adj = new Map();
  const link = (p, q) => {
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
    if (feature) link(p, q);
  }

  const components = [];
  const visited = new Set();
  for (const start of adj.keys()) {
    if (visited.has(start)) continue;
    const comp = [];
    const stack = [start];
    visited.add(start);
    while (stack.length) {
      const v = stack.pop();
      comp.push(v);
      for (const nb of adj.get(v)) {
        if (!visited.has(nb)) {
          visited.add(nb);
          stack.push(nb);
        }
      }
    }
    components.push(comp);
  }
  return components;
}

/**
 * Circular rims of a meshed part: `{ center, radius, normal, inliers, total,
 * coverageDeg }` per rim. A hole mouth, a cylinder end, a boss top.
 *
 * One `fitCircle` per feature-graph component of at least `MIN_CIRCLE_VERTS`
 * vertices. A component that is not a circle simply does not fit — the plate's
 * own outline, being one large connected mass of creases, is rejected on its
 * residuals rather than needing a special case.
 */
export function detectRims(positions, indices) {
  const rims = [];
  for (const comp of featureComponents(positions, indices)) {
    if (comp.length < MIN_CIRCLE_VERTS) continue;
    const fit = fitCircle(comp.map((v) => vertex(positions, v)));
    if (fit) rims.push(fit);
  }
  return rims;
}

/**
 * Recover the planar faces of a meshed part: `{ faces, totalArea,
 * planarAreaFraction, remainderArea, remainderRegions }`.
 *
 * Each face is `{ normal, offset, area, triangles }` — `offset` being the
 * plane's signed distance from the origin along `normal`, so the face is
 * identified, not merely counted. Faces are sorted by descending area.
 *
 * ### Why a *planar* census rather than a face count
 *
 * A dual-contouring mesh has no sharp edges: where a real box has an edge, the
 * mesher leaves a one-cell bevel of transition triangles whose normals sweep
 * the whole dihedral. So the obvious algorithm — cut the triangle-adjacency
 * graph at every crease, count components — reports 194 faces for a box,
 * because it counts each bevel facet.
 *
 * Growing from a *seed plane* is stable against exactly that: a triangle joins
 * a region only if its own normal is within `NORMAL_TOL_DEG` of the seed's *and*
 * all three of its vertices lie within `offsetTol` of the seed plane. Bevel
 * triangles fail both tests and simply do not join, instead of fragmenting the
 * face they border. This is the playground's `facePlane.js` region growth run
 * over every triangle instead of the one under a cursor.
 *
 * The bevel area has to go somewhere, so it is reported as `remainderArea`
 * rather than absorbed: on a well-formed part it is a percent or two (bevels
 * plus any genuinely curved surface), and a caller comparing face areas against
 * intent needs to know how much of the surface the planar census does not speak
 * for. Face areas come out a percent or so under their analytic value for the
 * same reason — `planarAreaFraction` is that discount, stated rather than left
 * to be discovered.
 *
 * Regions below `MIN_FACE_AREA_FRACTION` of the total go to the remainder: a
 * long bevel strip *is* locally planar, and admitting it would stand dozens of
 * 0.1%-area "faces" next to the six real ones. That area screen is the only
 * one — see the constant for why a triangle-count screen cannot be used here.
 */
export function segmentPlanarFaces(positions, indices) {
  const triCount = indices.length / 3;
  const empty = {
    faces: [],
    totalArea: 0,
    planarAreaFraction: 0,
    remainderArea: 0,
    remainderRegions: 0,
  };
  if (triCount === 0) return empty;

  const byEdge = new Map();
  for (let t = 0; t < triCount; t += 1) {
    const [a, b, c] = triVertexIds(indices, t);
    for (const [p, q] of [[a, b], [b, c], [c, a]]) {
      const key = edgeKey(p, q);
      let rec = byEdge.get(key);
      if (!rec) byEdge.set(key, (rec = []));
      rec.push(t);
    }
  }
  const neighbours = Array.from({ length: triCount }, () => []);
  for (const tris of byEdge.values()) {
    for (const t of tris) {
      for (const other of tris) {
        if (other !== t) neighbours[t].push(other);
      }
    }
  }

  const normals = [];
  const areas = [];
  let totalArea = 0;
  for (let t = 0; t < triCount; t += 1) {
    normals.push(triNormal(positions, indices, t));
    const a = triArea(positions, indices, t);
    areas.push(a);
    totalArea += a;
  }
  if (totalArea <= 0) return empty;

  const cosTol = Math.cos((NORMAL_TOL_DEG * Math.PI) / 180);
  const offsetTol = OFFSET_TOL_FACTOR * (meshDiagonal(positions) || 1) + 1e-12;

  // Seed from the largest triangles first: on any tessellation the interior of
  // a flat face carries bigger triangles than the transition band around it,
  // so this grows the real faces before their fringe can claim their triangles.
  const order = Array.from({ length: triCount }, (_, t) => t).sort((a, b) => areas[b] - areas[a]);

  const assigned = new Uint8Array(triCount);
  const regions = [];
  for (const seed of order) {
    if (assigned[seed] || !normals[seed]) continue;
    const seedNormal = normals[seed];
    const seedPoint = vertex(positions, triVertexIds(indices, seed)[0]);
    const accepted = [seed];
    assigned[seed] = 1;
    const stack = [seed];
    while (stack.length) {
      const tri = stack.pop();
      for (const next of neighbours[tri]) {
        if (assigned[next] || !normals[next]) continue;
        if (dot(normals[next], seedNormal) < cosTol) continue;
        const offPlane = triVertexIds(indices, next).some(
          (i) => Math.abs(dot(sub(vertex(positions, i), seedPoint), seedNormal)) > offsetTol,
        );
        if (offPlane) continue;
        assigned[next] = 1;
        accepted.push(next);
        stack.push(next);
      }
    }
    let area = 0;
    let acc = [0, 0, 0];
    for (const t of accepted) {
      area += areas[t];
      acc = add(acc, scale(normals[t], areas[t]));
    }
    const normal = normalize(acc) || seedNormal;
    regions.push({
      normal,
      offset: dot(normal, seedPoint),
      area,
      triangles: accepted.length,
    });
  }

  const minArea = MIN_FACE_AREA_FRACTION * totalArea;
  const faces = regions.filter((r) => r.area >= minArea).sort((a, b) => b.area - a.area);
  const planarArea = faces.reduce((sum, f) => sum + f.area, 0);
  return {
    faces,
    totalArea,
    planarAreaFraction: planarArea / totalArea,
    remainderArea: totalArea - planarArea,
    remainderRegions: regions.length - faces.length,
  };
}

/**
 * Euler characteristic and genus of the raw triangle mesh:
 * `{ components, vertices, edges, triangles, eulerCharacteristic, genus,
 * closed }`.
 *
 * `genus` is the number of handles in the surface — for a plate, exactly the
 * number of holes drilled through it — from `V − E + F = 2(C − G)` over all
 * components. It is a *topological invariant*: unlike a volume it does not
 * drift with meshing accuracy, so "I expect 4 holes" is an assertion that
 * either holds or does not. It is null when the surface is not closed, where
 * the formula does not apply.
 *
 * Positions are welded by exact coordinate match first. The kernel's meshers
 * already share vertices, but an STL-style buffer with duplicated corners would
 * otherwise report every triangle as its own component.
 */
export function meshTopology(positions, indices) {
  const triCount = indices.length / 3;
  if (triCount === 0) {
    return {
      components: 0,
      vertices: 0,
      edges: 0,
      triangles: 0,
      eulerCharacteristic: 0,
      genus: null,
      closed: false,
    };
  }

  const byKey = new Map();
  const weld = new Int32Array(positions.length / 3);
  for (let v = 0; v < weld.length; v += 1) {
    const key = `${positions[3 * v]},${positions[3 * v + 1]},${positions[3 * v + 2]}`;
    let id = byKey.get(key);
    if (id === undefined) {
      id = byKey.size;
      byKey.set(key, id);
    }
    weld[v] = id;
  }
  const vertexCount = byKey.size;

  const edgeUse = new Map();
  const parent = new Int32Array(vertexCount);
  for (let i = 0; i < vertexCount; i += 1) parent[i] = i;
  const find = (x) => {
    let r = x;
    while (parent[r] !== r) r = parent[r];
    while (parent[x] !== r) {
      const next = parent[x];
      parent[x] = r;
      x = next;
    }
    return r;
  };
  const union = (a, b) => {
    const ra = find(a);
    const rb = find(b);
    if (ra !== rb) parent[rb] = ra;
  };

  for (let t = 0; t < triCount; t += 1) {
    const [a, b, c] = triVertexIds(indices, t).map((i) => weld[i]);
    for (const [p, q] of [[a, b], [b, c], [c, a]]) {
      if (p === q) continue; // degenerate sliver: not an edge of the surface
      const key = edgeKey(p, q);
      edgeUse.set(key, (edgeUse.get(key) || 0) + 1);
      union(p, q);
    }
  }

  const roots = new Set();
  for (let v = 0; v < vertexCount; v += 1) roots.add(find(v));
  const components = roots.size;
  const edgeCount = edgeUse.size;
  let closed = true;
  for (const uses of edgeUse.values()) {
    if (uses !== 2) {
      closed = false;
      break;
    }
  }

  const chi = vertexCount - edgeCount + triCount;
  return {
    components,
    vertices: vertexCount,
    edges: edgeCount,
    triangles: triCount,
    eulerCharacteristic: chi,
    genus: closed ? components - chi / 2 : null,
    closed,
  };
}

/**
 * Group circular rims into cylindrical features: two rims of equal radius whose
 * centres are separated along their shared normal are the two ends of one
 * cylinder.
 *
 * Returns `{ axis, radius, ends, length, center }` per feature. An unpaired rim
 * comes back with a single `end` and a null `length` — a blind pocket's mouth,
 * a chamfer, or a cylinder end the mesher rounded over, all three worth
 * reporting rather than dropping.
 */
export function groupCylinders(rims) {
  const used = new Set();
  const features = [];
  const cosAxis = Math.cos((RIM_AXIS_TOL_DEG * Math.PI) / 180);
  for (let i = 0; i < rims.length; i += 1) {
    if (used.has(i)) continue;
    const a = rims[i];
    let partner = -1;
    let bestOffAxis = Infinity;
    for (let j = i + 1; j < rims.length; j += 1) {
      if (used.has(j)) continue;
      const b = rims[j];
      if (Math.abs(a.radius - b.radius) > RIM_PAIR_TOL * a.radius) continue;
      if (Math.abs(dot(a.normal, b.normal)) < cosAxis) continue;
      const d = sub(b.center, a.center);
      const along = dot(d, a.normal);
      const offAxis = norm(sub(d, scale(a.normal, along)));
      // Coaxial: the lateral offset must be small against the radius, and the
      // two rims must not be the same circle counted twice.
      if (offAxis > RIM_PAIR_TOL * a.radius) continue;
      if (Math.abs(along) <= RIM_PAIR_TOL * a.radius) continue;
      if (offAxis < bestOffAxis) {
        bestOffAxis = offAxis;
        partner = j;
      }
    }
    used.add(i);
    if (partner >= 0) {
      const b = rims[partner];
      used.add(partner);
      features.push({
        axis: normalize(sub(b.center, a.center)) || a.normal,
        radius: (a.radius + b.radius) / 2,
        ends: [a.center, b.center],
        length: norm(sub(b.center, a.center)),
        center: scale(add(a.center, b.center), 0.5),
      });
    } else {
      features.push({
        axis: a.normal,
        radius: a.radius,
        ends: [a.center],
        length: null,
        center: a.center,
      });
    }
  }
  return features;
}

/**
 * Cast a line through `at` along `axis` and report the solid/void spans it
 * crosses, by sampling the shape's signed distance field.
 *
 * Returns `{ axis, at, spans, solidSpans, voidSpans, throughHole, gapLength,
 * materialLength }`, where each span is `{ kind: 'solid'|'void', from, to,
 * length }` in the parameter along the (normalized) axis measured from `at`.
 * Exterior void beyond the part is not a span, so the list starts and ends on
 * material.
 *
 * `throughHole` is true when the line passes solid → void → solid: material,
 * then a gap, then material again. That is what drilling *through* something
 * means, and it is the question `valid`, `measure` and a screenshot could not
 * answer. The gap's length is the void the line crosses, so probing the
 * intended axis of a hole that was bored on the wrong one comes back
 * `throughHole: false` — which is the of-4tu bug, caught in one call.
 *
 * Uniform sampling with bisection refinement rather than sphere tracing: after
 * a smooth blend or an anisotropic scale the field is not an exact Euclidean
 * distance, so a marching step of `|d|` is not guaranteed safe. Uniform
 * sampling is instead *predictably* limited — a feature thinner than
 * `span / PROBE_SAMPLES` can slip between samples, and nothing else can.
 *
 * @param {object} shape a WasmShape (uses `distance` and `bounds`)
 * @param {[number,number,number]} axis direction, need not be normalized
 * @param {[number,number,number]} at a point the line passes through
 */
export function probeAxis(shape, axis, at) {
  const dir = Array.isArray(axis) && axis.length === 3 ? normalize(axis) : null;
  if (!dir || !axis.every((c) => Number.isFinite(c))) {
    throw new Error('axis must be a non-zero finite [x,y,z] direction');
  }
  if (!Array.isArray(at) || at.length !== 3 || !at.every((c) => Number.isFinite(c))) {
    throw new Error('at must be a finite [x,y,z] point');
  }

  const b = shape.bounds();
  const diag = norm(sub([b[3], b[4], b[5]], [b[0], b[1], b[2]])) || 1;
  // Reach past the shape both ways so the line provably starts and ends outside
  // the solid: the tracked box is conservative, and its own diagonal clears any
  // point inside it.
  const reach = diag * 1.1;

  const sampleAt = (t) => {
    const p = add(at, scale(dir, t));
    return shape.distance(p[0], p[1], p[2]);
  };

  const t0 = -reach;
  const t1 = reach;
  const step = (t1 - t0) / PROBE_SAMPLES;

  const crossingBetween = (lo, hi) => {
    let a = lo;
    let c = hi;
    let da = sampleAt(a);
    for (let i = 0; i < PROBE_REFINE; i += 1) {
      const m = 0.5 * (a + c);
      const dm = sampleAt(m);
      if (dm === 0) return m;
      if (dm < 0 === da < 0) {
        a = m;
        da = dm;
      } else {
        c = m;
      }
    }
    return 0.5 * (a + c);
  };

  const crossings = [];
  let prevT = t0;
  let prevD = sampleAt(prevT);
  for (let i = 1; i <= PROBE_SAMPLES; i += 1) {
    const t = t0 + i * step;
    const d = sampleAt(t);
    if (d < 0 !== prevD < 0) {
      crossings.push({ t: crossingBetween(prevT, t), entering: d < 0 });
    }
    prevT = t;
    prevD = d;
  }

  // Spans between consecutive crossings. Starting outside, crossings alternate
  // enter/exit, so an enter→exit pair bounds solid and an exit→enter pair bounds
  // the void between two pieces of material.
  const spans = [];
  for (let i = 0; i + 1 < crossings.length; i += 1) {
    const from = crossings[i].t;
    const to = crossings[i + 1].t;
    spans.push({
      kind: crossings[i].entering ? 'solid' : 'void',
      from,
      to,
      length: to - from,
    });
  }

  const solid = spans.filter((s) => s.kind === 'solid');
  const voids = spans.filter((s) => s.kind === 'void');
  return {
    axis: dir,
    at,
    spans,
    solidSpans: solid.length,
    voidSpans: voids.length,
    throughHole: solid.length >= 2 && voids.length >= 1,
    gapLength: voids.length ? Math.max(...voids.map((s) => s.length)) : null,
    materialLength: solid.reduce((sum, s) => sum + s.length, 0),
  };
}

/**
 * Classify each cylindrical feature by asking the field about it:
 * `{ kind, axis, radius, diameter, center, ends, depth }`, where `kind` is
 * `through-hole`, `pocket`, `cavity`, `boss`, or `rim`.
 *
 * Rim geometry alone cannot tell these apart — the same two coaxial circles
 * bound a drilled hole, a blind pocket, and a protruding pin — so the field
 * decides, from two questions asked strictly about where the feature is:
 *
 * 1. **Is the bore empty?** Sample the axis strictly *between* the two rims.
 *    All samples outside the solid means the cylinder is a void; any inside
 *    means the cylinder is itself material, i.e. a `boss`.
 * 2. **Do both ends open to air?** Sample just past each rim, along the axis.
 *    Air at both is a `through-hole`; material past exactly one means the other
 *    rim is a pocket floor, not a second mouth, so it is a `pocket`; material
 *    past both is an enclosed `cavity`.
 *
 * Question 2 is what a bore-emptiness test alone gets wrong: a blind pocket has
 * *two* fitted rims — its mouth and the circle where its floor meets its wall —
 * with clear space between them, and reads as a through-hole without it.
 *
 * Deliberately not "cast a ray along the whole axis and see whether it exits":
 * a hole through one leg of an L-bracket has the *other* leg further along that
 * axis, and a whole-line test would call it blind. The step past each rim is
 * scaled to the feature so it stays local to it.
 *
 * A single-rim feature (no coaxial partner the fit could find) gets
 * `kind: 'rim'` — a chamfer, or a cylinder end the mesher rounded over — with
 * `depth` the void running inward from it, if any.
 */
export function classifyCylinders(shape, features) {
  const outside = (p) => shape.distance(p[0], p[1], p[2]) > 0;
  return features.map((f) => {
    const common = {
      axis: f.axis,
      radius: f.radius,
      diameter: 2 * f.radius,
      center: f.center,
      ends: f.ends,
    };
    if (f.ends.length === 2) {
      const [p0, p1] = f.ends;
      const span = sub(p1, p0);
      let boreClear = true;
      for (let i = 1; i < BORE_SAMPLES; i += 1) {
        if (!outside(add(p0, scale(span, i / BORE_SAMPLES)))) {
          boreClear = false;
          break;
        }
      }
      if (!boreClear) {
        // Rim to rim along a solid cylinder: the boss's own length.
        return { kind: 'boss', ...common, depth: f.length };
      }
      // A step short enough to stay against the rim it is testing, on both the
      // feature's scales — a shallow wide bore and a deep narrow one.
      const step = END_STEP_FRACTION * Math.min(f.radius, f.length);
      const beyond = [
        outside(add(p0, scale(span, -step / f.length))),
        outside(add(p1, scale(span, step / f.length))),
      ];
      const open = beyond.filter(Boolean).length;
      const kind = open === 2 ? 'through-hole' : open === 1 ? 'pocket' : 'cavity';
      // Rim to rim: the material a hole passes through, or a pocket's depth.
      return { kind, ...common, depth: f.length };
    }
    const probe = probeAxis(shape, f.axis, f.center);
    const voidAtMouth = probe.spans.find((s) => s.kind === 'void' && s.from <= 0 && s.to >= 0);
    return { kind: 'rim', ...common, depth: voidAtMouth ? voidAtMouth.length : null };
  });
}

/**
 * The full mesh-and-field topology census for a shape: `{ counts, mesh,
 * planarFaces, cylinders }`.
 *
 * Every number here comes from the mesh or the field, so it is available for
 * *any* model — including the SDF-only ones (blends, sweeps, shells, offsets)
 * that have no B-Rep to count. Where a model does carry an exact B-Rep, that
 * body's own entity counts are the authoritative answer and arrive via
 * `brepCheck`; this is the census for everything else, and the cross-check for
 * the models that have both.
 *
 * @param {object} shape a WasmShape (uses `distance` and `bounds`)
 * @param {{positions:Float32Array, indices:Uint32Array}} mesh
 */
export function inspect(shape, mesh) {
  const { positions, indices } = mesh;
  const rims = detectRims(positions, indices);
  const planar = segmentPlanarFaces(positions, indices);
  const topology = meshTopology(positions, indices);
  const cylinders = classifyCylinders(shape, groupCylinders(rims));
  return {
    counts: {
      planarFaces: planar.faces.length,
      circularRims: rims.length,
      throughHoles: cylinders.filter((c) => c.kind === 'through-hole').length,
      pockets: cylinders.filter((c) => c.kind === 'pocket').length,
      cavities: cylinders.filter((c) => c.kind === 'cavity').length,
      bosses: cylinders.filter((c) => c.kind === 'boss').length,
      unpairedRims: cylinders.filter((c) => c.kind === 'rim').length,
      shells: topology.components,
      // Handles in the surface. For a plate this *is* how many holes were
      // drilled through it, and unlike a volume it cannot drift with accuracy.
      genus: topology.genus,
    },
    mesh: topology,
    planarFaces: planar,
    cylinders,
  };
}
