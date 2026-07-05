// Sketch profile -> sweep solid: convert an extracted 2D profile into the
// `Profile` builder ops consumed by `Shape.extrude` / `Shape.revolve`, and
// orient the swept solid onto the sketch plane.
//
// The kernel sweeps in a fixed frame: extrude maps profile (u, v) to world
// (x, z) and spans y ∈ [0, height]; revolve spins profile (u = radius,
// v = height) around the world Y axis starting from the +X half-plane. The
// per-plane post-ops below rotate (and for extrude translate) that native
// result so the profile lands on the chosen sketch plane, extruded along
// the plane's +normal / revolved around the sketch's v axis.
//
// Kept free of React and WASM imports so it can be unit-tested with
// stand-in Shape/Profile classes (same pattern as sceneTree.js).

import { arcSweep } from './sketch/geom.js';
import { segmentEnd2D, segmentStart2D } from './sketch/profile.js';

/**
 * Convert a closed extracted profile (`extractProfile` output) into profile
 * builder ops: `{ start: [x, y], segs: [{ x, y, bulge }] }`, where `bulge`
 * is the DXF convention `tan(sweep / 4)`, positive counter-clockwise. The
 * last segment ends exactly on `start` so the loop closes cleanly.
 */
export function profileToOps(profile) {
  if (!profile.closed) {
    throw new Error(`profile is not closed: ${profile.reason}`);
  }
  const start = segmentStart2D(profile.segments[0]);
  const segs = profile.segments.map((seg, i) => {
    const last = i === profile.segments.length - 1;
    const [x, y] = last ? start : segmentEnd2D(seg);
    if (seg.kind === 'line') return { x, y, bulge: 0 };
    const sweep = arcSweep(seg.startAngle, seg.endAngle, seg.ccw);
    const signed = seg.ccw ? sweep : -sweep;
    return { x, y, bulge: Math.tan(signed / 4) };
  });
  return { start, segs };
}

/**
 * Ops (applied in order) that carry the native sweep result onto the sketch
 * plane: `[{ op: 'rotate'|'translate', args }]`.
 *
 * Extrude goes along the plane's +normal starting at the plane; revolve
 * spins around the sketch's v axis with the profile's u as radius.
 */
export function sweepPostOps(plane, kind, param) {
  if (kind === 'extrude') {
    switch (plane) {
      case 'XZ':
        // Native frame: profile (u, v) -> (x, z), swept along +Y.
        return [];
      case 'XY':
        return [
          { op: 'rotate', args: [1, 0, 0, -Math.PI / 2] },
          { op: 'translate', args: [0, 0, param] },
        ];
      case 'YZ':
        return [
          { op: 'rotate', args: [0, 0, 1, Math.PI / 2] },
          { op: 'translate', args: [param, 0, 0] },
        ];
      default:
        throw new Error(`unknown sketch plane: ${plane}`);
    }
  }
  if (kind === 'revolve') {
    switch (plane) {
      case 'XY':
        // Native frame: profile (u, v) -> (radius, y), around the Y axis.
        return [];
      case 'XZ':
        return [{ op: 'rotate', args: [1, 0, 0, Math.PI / 2] }];
      case 'YZ':
        // Cyclic axis permutation x->y->z->x (120° about (1,1,1)).
        return [{ op: 'rotate', args: [1, 1, 1, (2 * Math.PI) / 3] }];
      default:
        throw new Error(`unknown sketch plane: ${plane}`);
    }
  }
  throw new Error(`unknown sweep kind: ${kind}`);
}

/**
 * Build the swept, plane-oriented shape for `{ kind, plane, ops, value }`
 * (extrude height or revolve angle in degrees). Frees the profile and every
 * intermediate shape; the caller owns the returned shape.
 */
export function buildSweepShape(ShapeClass, ProfileClass, sweep) {
  const { kind, plane, ops, value } = sweep;
  const profile = new ProfileClass(ops.start[0], ops.start[1]);
  let shape;
  try {
    for (const seg of ops.segs) profile.arcTo(seg.x, seg.y, seg.bulge);
    profile.close();
    shape = ShapeClass[kind](profile, value);
  } finally {
    profile.free?.();
  }
  for (const post of sweepPostOps(plane, kind, value)) {
    const next = shape[post.op](...post.args);
    shape.free?.();
    shape = next;
  }
  return shape;
}

/**
 * Graft a sweep onto an existing construction tree as plain node data (no
 * retained shapes): the sweep node wrapped in its plane post-ops, unioned
 * with `root` when one exists. Synthetic ids are negative so they can't
 * collide with traced ids; the tree is only serialized, then re-evaluated.
 */
export function sweepTreeNode(root, sweep) {
  const { kind, plane, ops, value } = sweep;
  let id = -1;
  let node = {
    id: id--,
    op: kind,
    args: [value],
    children: [],
    profile: { start: [...ops.start], segs: ops.segs.map((s) => ({ ...s })) },
  };
  for (const post of sweepPostOps(plane, kind, value)) {
    node = { id: id--, op: post.op, args: post.args, children: [node] };
  }
  if (!root) return node;
  return { id: id--, op: 'union', args: [], children: [root, node] };
}

/** Axis-aligned bounding box `{ min: [u, v], max: [u, v] }` of profile ops
 * vertices (arc bulges may extend slightly past it; good enough for UI
 * defaults). */
export function opsBounds(ops) {
  const xs = [ops.start[0], ...ops.segs.map((s) => s.x)];
  const ys = [ops.start[1], ...ops.segs.map((s) => s.y)];
  return {
    min: [Math.min(...xs), Math.min(...ys)],
    max: [Math.max(...xs), Math.max(...ys)],
  };
}
