/**
 * Profile extraction: turn a sketch into a closed, ordered loop of line/arc
 * segments — the 2D profile consumed by extrude/revolve (matching the
 * kernel's `Profile` contract: at least 2 segments, consecutive segments
 * connected, no single full-period segment; a lone circle is emitted as two
 * semicircular arcs).
 *
 * Profile shape:
 *   {
 *     closed: true,
 *     plane: 'XY' | 'XZ' | 'YZ',
 *     segments: [
 *       { kind: 'line', start: [x, y], end: [x, y] },
 *       { kind: 'arc', center: [x, y], radius, startAngle, endAngle, ccw },
 *     ],
 *   }
 * or `{ closed: false, reason }` when no valid loop exists.
 *
 * Segments are ordered counterclockwise (positive area), so mapping through
 * `planeToWorld` yields a winding normal of +Z / +Y / +X respectively.
 */

import { arcSweep, normalizeAngle, sampleArc, signedArea } from './geom.js';
import { entityRadius } from './model.js';

const ARC_SAMPLES = 32;

/** Map sketch-plane (u, v) to world [x, y, z] for a named plane. */
export function planeToWorld(plane, u, v) {
  switch (plane) {
    case 'XY':
      return [u, v, 0];
    case 'XZ':
      return [u, 0, v];
    case 'YZ':
      return [0, u, v];
    default:
      throw new Error(`unknown sketch plane: ${plane}`);
  }
}

/** Unit normal of a named sketch plane. */
export function planeNormal(plane) {
  switch (plane) {
    case 'XY':
      return [0, 0, 1];
    case 'XZ':
      return [0, 1, 0];
    case 'YZ':
      return [1, 0, 0];
    default:
      throw new Error(`unknown sketch plane: ${plane}`);
  }
}

/** Union-find over point ids gluing coincident-constrained points. */
function mergeRoots(sketch) {
  const parent = {};
  const find = (x) => {
    while (parent[x] !== undefined && parent[x] !== x) {
      parent[x] = parent[parent[x]] ?? parent[x];
      x = parent[x];
    }
    return x;
  };
  for (const c of Object.values(sketch.constraints)) {
    if (c.type !== 'coincident') continue;
    const ra = find(c.a);
    const rb = find(c.b);
    if (ra !== rb) parent[ra] = rb;
  }
  return find;
}

function arcGeometry(sketch, entity) {
  const c = sketch.points[entity.center];
  const p1 = sketch.points[entity.p1];
  const p2 = sketch.points[entity.p2];
  const radius = entityRadius(sketch, entity);
  const startAngle = normalizeAngle(Math.atan2(p1.y - c.y, p1.x - c.x));
  const endAngle = normalizeAngle(Math.atan2(p2.y - c.y, p2.x - c.x));
  return { center: [c.x, c.y], radius, startAngle, endAngle };
}

/** Loop polyline for area/orientation checks (arcs sampled). */
function samplePolyline(sketch, ordered) {
  const pts = [];
  for (const { entity, forward } of ordered) {
    if (entity.type === 'line') {
      const p = sketch.points[forward ? entity.p1 : entity.p2];
      pts.push([p.x, p.y]);
    } else {
      const { center, radius, startAngle, endAngle } = arcGeometry(
        sketch,
        entity
      );
      const ccw = forward ? entity.ccw : !entity.ccw;
      const from = forward ? startAngle : endAngle;
      const to = forward ? endAngle : startAngle;
      const sweep = arcSweep(from, to, ccw);
      const samples = sampleArc(
        center[0],
        center[1],
        radius,
        from,
        sweep,
        ccw,
        ARC_SAMPLES
      );
      samples.pop(); // next segment supplies the shared endpoint
      pts.push(...samples);
    }
  }
  return pts;
}

function circleProfile(sketch, circle, plane) {
  const c = sketch.points[circle.center];
  const r = circle.radius;
  if (!(r > 0)) return { closed: false, reason: 'circle has zero radius' };
  return {
    closed: true,
    plane,
    segments: [
      {
        kind: 'arc',
        center: [c.x, c.y],
        radius: r,
        startAngle: 0,
        endAngle: Math.PI,
        ccw: true,
      },
      {
        kind: 'arc',
        center: [c.x, c.y],
        radius: r,
        startAngle: Math.PI,
        endAngle: 0,
        ccw: true,
      },
    ],
  };
}

/**
 * Extract the sketch's single closed profile, or explain why there is none.
 */
export function extractProfile(sketch, plane = 'XY') {
  const entities = Object.values(sketch.entities);
  const circles = entities.filter((e) => e.type === 'circle');
  const chain = entities.filter((e) => e.type === 'line' || e.type === 'arc');

  if (entities.length === 0) {
    return { closed: false, reason: 'empty sketch' };
  }
  if (circles.length > 0) {
    if (circles.length === 1 && chain.length === 0) {
      return circleProfile(sketch, circles[0], plane);
    }
    return {
      closed: false,
      reason: 'a circle must be the only entity in the sketch',
    };
  }
  if (chain.length < 2) {
    return { closed: false, reason: 'a profile needs at least 2 segments' };
  }

  // Endpoint connectivity graph over merged points.
  const find = mergeRoots(sketch);
  const incidence = new Map(); // root -> [{ entity, end: 'p1' | 'p2' }]
  for (const entity of chain) {
    if (find(entity.p1) === find(entity.p2)) {
      return {
        closed: false,
        reason: 'a segment has coincident endpoints',
      };
    }
    for (const end of ['p1', 'p2']) {
      const root = find(entity[end]);
      if (!incidence.has(root)) incidence.set(root, []);
      incidence.get(root).push({ entity, end });
    }
  }
  for (const [, users] of incidence) {
    if (users.length === 1) {
      return { closed: false, reason: 'profile has an open endpoint' };
    }
    if (users.length > 2) {
      return {
        closed: false,
        reason: 'profile branches (more than 2 segments meet at a point)',
      };
    }
  }

  // Trace the loop from the first entity.
  const ordered = [];
  const visited = new Set();
  let current = { entity: chain[0], forward: true };
  for (;;) {
    ordered.push(current);
    visited.add(current.entity.id);
    const exitEnd = current.forward ? 'p2' : 'p1';
    const exitRoot = find(current.entity[exitEnd]);
    const next = incidence
      .get(exitRoot)
      .find((u) => u.entity.id !== current.entity.id);
    if (next.entity.id === chain[0].id) break;
    if (visited.has(next.entity.id)) {
      return { closed: false, reason: 'profile is not a single loop' };
    }
    current = { entity: next.entity, forward: next.end === 'p1' };
  }
  if (ordered.length !== chain.length) {
    return {
      closed: false,
      reason: 'sketch contains more than one loop or stray segments',
    };
  }

  // Verify endpoints actually touch (constraints may be unsolved).
  const gapTol = loopExtent(sketch, chain) * 1e-6 + 1e-9;
  for (let i = 0; i < ordered.length; i++) {
    const cur = ordered[i];
    const nxt = ordered[(i + 1) % ordered.length];
    const a = sketch.points[cur.forward ? cur.entity.p2 : cur.entity.p1];
    const b = sketch.points[nxt.forward ? nxt.entity.p1 : nxt.entity.p2];
    if (Math.hypot(a.x - b.x, a.y - b.y) > gapTol) {
      return {
        closed: false,
        reason: 'segment endpoints are not touching (solve constraints)',
      };
    }
  }

  // Counterclockwise output for a consistent winding normal.
  let loop = ordered;
  const area = signedArea(samplePolyline(sketch, ordered));
  if (Math.abs(area) < gapTol * gapTol) {
    return { closed: false, reason: 'profile encloses no area' };
  }
  if (area < 0) {
    loop = ordered
      .slice()
      .reverse()
      .map(({ entity, forward }) => ({ entity, forward: !forward }));
  }

  const segments = loop.map(({ entity, forward }) => {
    if (entity.type === 'line') {
      const a = sketch.points[forward ? entity.p1 : entity.p2];
      const b = sketch.points[forward ? entity.p2 : entity.p1];
      return { kind: 'line', start: [a.x, a.y], end: [b.x, b.y] };
    }
    const { center, radius, startAngle, endAngle } = arcGeometry(
      sketch,
      entity
    );
    return {
      kind: 'arc',
      center,
      radius,
      startAngle: forward ? startAngle : endAngle,
      endAngle: forward ? endAngle : startAngle,
      ccw: forward ? entity.ccw : !entity.ccw,
    };
  });

  return { closed: true, plane, segments };
}

function loopExtent(sketch, entities) {
  let minX = Infinity;
  let minY = Infinity;
  let maxX = -Infinity;
  let maxY = -Infinity;
  for (const e of entities) {
    for (const pid of [e.p1, e.p2]) {
      const p = sketch.points[pid];
      minX = Math.min(minX, p.x);
      minY = Math.min(minY, p.y);
      maxX = Math.max(maxX, p.x);
      maxY = Math.max(maxY, p.y);
    }
  }
  return Math.hypot(maxX - minX, maxY - minY) || 1;
}

/** Segment start point in 2D (line start or arc start-angle point). */
export function segmentStart2D(segment) {
  if (segment.kind === 'line') return segment.start;
  const [cx, cy] = segment.center;
  return [
    cx + segment.radius * Math.cos(segment.startAngle),
    cy + segment.radius * Math.sin(segment.startAngle),
  ];
}

/** Segment end point in 2D. */
export function segmentEnd2D(segment) {
  if (segment.kind === 'line') return segment.end;
  const [cx, cy] = segment.center;
  return [
    cx + segment.radius * Math.cos(segment.endAngle),
    cy + segment.radius * Math.sin(segment.endAngle),
  ];
}

/**
 * Lift a 2D profile into 3D on its sketch plane: each segment gains
 * `start3`/`end3` (and `center3` for arcs); the profile gains `normal`.
 */
export function profileTo3D(profile) {
  if (!profile.closed) return profile;
  const { plane } = profile;
  return {
    ...profile,
    normal: planeNormal(plane),
    segments: profile.segments.map((seg) => {
      const start = segmentStart2D(seg);
      const end = segmentEnd2D(seg);
      const lifted = {
        ...seg,
        start3: planeToWorld(plane, start[0], start[1]),
        end3: planeToWorld(plane, end[0], end[1]),
      };
      if (seg.kind === 'arc') {
        lifted.center3 = planeToWorld(plane, seg.center[0], seg.center[1]);
      }
      return lifted;
    }),
  };
}
