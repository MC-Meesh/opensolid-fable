// Measurement combiners for the Measure tool (of-fsl.17): given picked
// entities (vertices, edges, circular rims, planar faces — produced by
// measureTopology.js and facePlane.js), report the numbers SolidWorks'
// Evaluate > Measure surfaces: a single entity's coordinate/length/radius/
// area, and the distance/angle between two entities. Also the whole-body
// bounding-box dimensions, always available while measuring.
//
// Pure and free of three.js/React so it can be unit-tested on plain arrays.

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);
const cross = (a, b) => [
  a[1] * b[2] - a[2] * b[1],
  a[2] * b[0] - a[0] * b[2],
  a[0] * b[1] - a[1] * b[0],
];

function normalize(a) {
  const n = norm(a);
  return n > 0 ? [a[0] / n, a[1] / n, a[2] / n] : null;
}

function vertex(positions, index) {
  return [positions[3 * index], positions[3 * index + 1], positions[3 * index + 2]];
}

const RAD_TO_DEG = 180 / Math.PI;

/** Whole-body axis-aligned bounding box: min/max, per-axis size, diagonal. */
export function boundingBoxDims(positions) {
  if (!positions || positions.length === 0) return null;
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let k = 0; k < 3; k += 1) {
      const c = positions[i + k];
      if (c < min[k]) min[k] = c;
      if (c > max[k]) max[k] = c;
    }
  }
  const size = [max[0] - min[0], max[1] - min[1], max[2] - min[2]];
  return {
    min,
    max,
    size,
    diagonal: norm(size),
    center: [(min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2],
  };
}

/** Total area of the triangles `tris` of an indexed mesh (planar face area). */
export function triListArea(positions, indices, tris) {
  let area = 0;
  for (const t of tris) {
    const a = vertex(positions, indices[3 * t]);
    const b = vertex(positions, indices[3 * t + 1]);
    const c = vertex(positions, indices[3 * t + 2]);
    area += norm(cross(sub(b, a), sub(c, a))) / 2;
  }
  return area;
}

/** A single representative point for an entity (its "location"). */
export function refPoint(entity) {
  switch (entity.kind) {
    case 'vertex':
    case 'point':
      return entity.point;
    case 'circle':
      return entity.center;
    case 'face':
      return entity.origin;
    case 'edge':
      return [
        (entity.a[0] + entity.b[0]) / 2,
        (entity.a[1] + entity.b[1]) / 2,
        (entity.a[2] + entity.b[2]) / 2,
      ];
    default:
      return null;
  }
}

/** The plane `{ origin, normal }` an entity lies in, or null. */
function entityPlane(entity) {
  if (entity.kind === 'face' && entity.normal) return { origin: entity.origin, normal: entity.normal };
  if (entity.kind === 'circle' && entity.normal) return { origin: entity.center, normal: entity.normal };
  return null;
}

/** A direction for an entity: an edge's axis or a face/circle normal, else null. */
function entityDirection(entity) {
  if (entity.kind === 'edge') return normalize(sub(entity.b, entity.a));
  if (entity.kind === 'face' || entity.kind === 'circle') return entity.normal ?? null;
  return null;
}

/** Report a single picked entity's intrinsic measurement. */
export function measureSingle(entity) {
  switch (entity.kind) {
    case 'vertex':
    case 'point':
      return { kind: entity.kind, coord: entity.point };
    case 'edge':
      return { kind: 'edge', length: entity.length, from: entity.a, to: entity.b, closed: entity.closed };
    case 'circle':
      return {
        kind: 'circle',
        radius: entity.radius,
        diameter: entity.radius * 2,
        circumference: 2 * Math.PI * entity.radius,
        center: entity.center,
      };
    case 'face':
      return { kind: 'face', area: entity.area, centroid: entity.origin, normal: entity.normal };
    default:
      return null;
  }
}

/**
 * Report the relationship between two picked entities: the straight-line
 * distance between their reference points and its per-axis components; for
 * two planes (faces/circles) the angle between normals, plus the
 * perpendicular gap when they are parallel; for a point-vs-plane pairing the
 * normal distance to the plane; for two edges the angle between them.
 */
export function measurePair(a, b) {
  const pa = refPoint(a);
  const pb = refPoint(b);
  const delta = sub(pb, pa);
  const result = {
    distance: norm(delta),
    delta: [Math.abs(delta[0]), Math.abs(delta[1]), Math.abs(delta[2])],
  };

  const planeA = entityPlane(a);
  const planeB = entityPlane(b);
  const dirA = entityDirection(a);
  const dirB = entityDirection(b);

  if (planeA && planeB) {
    const c = Math.min(1, Math.max(-1, dot(planeA.normal, planeB.normal)));
    result.angle = Math.acos(c) * RAD_TO_DEG;
    const parallel = Math.abs(Math.abs(c) - 1) < 1e-3;
    if (parallel) {
      result.planeDistance = Math.abs(dot(sub(planeB.origin, planeA.origin), planeA.normal));
    }
  } else if (planeA && !planeB) {
    result.planeDistance = Math.abs(dot(sub(pb, planeA.origin), planeA.normal));
  } else if (planeB && !planeA) {
    result.planeDistance = Math.abs(dot(sub(pa, planeB.origin), planeB.normal));
  } else if (a.kind === 'edge' && b.kind === 'edge' && dirA && dirB) {
    const c = Math.min(1, Math.abs(dot(dirA, dirB)));
    result.angle = Math.acos(c) * RAD_TO_DEG;
  }

  return result;
}
