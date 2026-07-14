// Sketch profile -> sweep solid: convert an extracted 2D profile into the
// `Profile` builder ops consumed by `Shape.extrude` / `Shape.revolve`, and
// orient the swept solid onto the sketch plane.
//
// The kernel sweeps in a fixed frame: extrude maps profile (u, v) to world
// (x, z) and spans y ∈ [0, height]; revolve spins profile (u = radius,
// v = height) around the world Y axis starting from the +X half-plane. The
// per-plane post-ops below rotate (and for extrude translate) that native
// result so the profile lands on the chosen sketch plane — at exactly
// `planeToWorld(u, v)` (see lib/sketch/profile.js) — extruded along the
// plane's +normal / revolved around the sketch's v axis.
//
// The XZ and YZ sketch frames map v to -z / u to -z, which the extrude
// post-rotations alone cannot reach (a rotation cannot mirror), so extrude
// on those planes additionally mirrors the profile ops (`nativeSweepOps`).
//
// Kept free of React and WASM imports so it can be unit-tested with
// stand-in Shape/Profile classes (same pattern as sceneTree.js).

import { arcSweep } from './sketch/geom.js';
import {
  isFacePlane,
  planeNormal,
  planeToWorld,
  segmentEnd2D,
  segmentStart2D,
} from './sketch/profile.js';
import { axisAngleFromBasis } from './facePlane.js';

/** Extrude end conditions (SolidWorks parity). `blind` is the default: a
 * signed height on one side of the sketch plane. `symmetric` centers the
 * same height on the plane; `through` fills the whole scene both ways;
 * `toFace` terminates the extrude at a target plane (a picked face). */
export const END_CONDITIONS = ['blind', 'symmetric', 'through', 'toFace'];

/** Extrude modes: `boss` adds material (union with the scene), `cut`
 * removes it (subtract from the scene). */
export const EXTRUDE_MODES = ['boss', 'cut'];

const dot3 = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const scale3 = (a, s) => [a[0] * s, a[1] * s, a[2] * s];
const sub3 = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];

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
 * Mirror profile ops across the u axis (v -> -v) while preserving the
 * counterclockwise traversal the kernel expects: the mirrored loop is walked
 * in reverse (mirror and reversal each negate a bulge, so bulges carry over
 * unchanged). The start vertex stays the loop's anchor.
 */
export function mirrorOpsV(ops) {
  // Loop vertices v0..vn-1 (segs[i] runs v_i -> v_i+1, the last back to v0).
  const verts = [ops.start, ...ops.segs.slice(0, -1).map((s) => [s.x, s.y])];
  const segs = [];
  for (let i = ops.segs.length - 1; i >= 0; i -= 1) {
    const [x, y] = verts[i];
    segs.push({ x, y: -y, bulge: ops.segs[i].bulge });
  }
  return { start: [ops.start[0], -ops.start[1]], segs };
}

/**
 * Profile ops in the kernel's native sweep frame for the given plane and
 * sweep kind. Extrude on XZ/YZ and on face planes needs the mirror (the
 * kernel's native (u, v) -> (x, z) mapping is left-handed relative to the
 * +Y sweep direction, and those targets are reached by pure rotations);
 * revolve feeds (radius, height) directly for every plane.
 */
export function nativeSweepOps(ops, plane, kind) {
  if (kind === 'extrude' && (plane === 'XZ' || plane === 'YZ' || isFacePlane(plane))) {
    return mirrorOpsV(ops);
  }
  return ops;
}

/** Face-plane post-ops: `rotate` from the axis-angle of the given basis
 * columns (skipped for the identity), then `translate` by `offset`. */
function facePostOps(cols, offset) {
  const posts = [];
  const rotation = axisAngleFromBasis(...cols);
  if (rotation) posts.push({ op: 'rotate', args: [...rotation.axis, rotation.angle] });
  if (offset.some((c) => c !== 0)) posts.push({ op: 'translate', args: offset });
  return posts;
}

/**
 * Ops (applied in order) that carry the native sweep result onto the sketch
 * plane: `[{ op: 'rotate'|'translate', args }]`, paired with
 * `nativeSweepOps` for the same plane and kind.
 *
 * Extrude goes along the plane's normal starting at the plane — a negative
 * `param` extrudes the same |param| in the -normal direction (the kernel
 * only sweeps +Y, so the sign lives entirely in the post translate; pair
 * with |param| as the sweep argument). Revolve spins around the sketch's v
 * axis with the profile's u as radius.
 */
export function sweepPostOps(plane, kind, param) {
  if (kind === 'extrude') {
    if (isFacePlane(plane)) {
      // Mirrored ops sit at native (u, 0, -v); the rotation with columns
      // (u, n, -v) carries that to u·e_u + v·e_v and the +Y sweep onto the
      // face normal. det = -e_u · (n × e_v) = e_u · e_u = 1.
      const { origin, normal: n, u, v } = plane;
      const back = Math.min(param, 0);
      return facePostOps(
        [u, n, [-v[0], -v[1], -v[2]]],
        [origin[0] + back * n[0], origin[1] + back * n[1], origin[2] + back * n[2]]
      );
    }
    switch (plane) {
      case 'XZ':
        // Mirrored ops already sit at (x, z) = planeToWorld(u, v); the
        // native sweep along +Y is the plane normal. A reverse extrude
        // just drops the span from y ∈ [0, |param|] to [param, 0].
        return param < 0 ? [{ op: 'translate', args: [0, param, 0] }] : [];
      case 'XY':
        // After the rotation the solid spans z ∈ [-|param|, 0] — already
        // the reverse extrude; the forward one shifts up by the height.
        return [
          { op: 'rotate', args: [1, 0, 0, -Math.PI / 2] },
          ...(param > 0 ? [{ op: 'translate', args: [0, 0, param] }] : []),
        ];
      case 'YZ':
        // 120° about (1, 1, -1): x -> -z, native sweep +Y -> +X (the plane
        // normal), z -> -y; with mirrored ops this lands (u, v) on (y, -z).
        return [
          { op: 'rotate', args: [1, 1, -1, (2 * Math.PI) / 3] },
          ...(param < 0 ? [{ op: 'translate', args: [param, 0, 0] }] : []),
        ];
      default:
        throw new Error(`unknown sketch plane: ${plane}`);
    }
  }
  if (kind === 'revolve') {
    if (isFacePlane(plane)) {
      // Native revolve puts the profile on the +X half-plane around Y; the
      // rotation with columns (u, v, n) carries (radius, height) onto
      // (e_u, e_v) and the axis onto the sketch v axis through the origin.
      const { origin, normal: n, u, v } = plane;
      return facePostOps([u, v, n], [...origin]);
    }
    switch (plane) {
      case 'XY':
        // Native frame: profile (u, v) -> (radius, y), around the Y axis.
        return [];
      case 'XZ':
        // Y axis -> -Z: the sketch's v axis, with the profile on the plane.
        return [{ op: 'rotate', args: [1, 0, 0, -Math.PI / 2] }];
      case 'YZ':
        // Y axis stays put; the +X start half-plane rotates onto -Z (= +u).
        return [{ op: 'rotate', args: [0, 1, 0, Math.PI / 2] }];
      default:
        throw new Error(`unknown sketch plane: ${plane}`);
    }
  }
  throw new Error(`unknown sweep kind: ${kind}`);
}

/** The kernel-facing sweep argument: extrude heights carry their direction
 * in the post-ops (the kernel requires height > 0); revolve angles pass
 * through. */
function sweepArg(kind, value) {
  return kind === 'extrude' ? Math.abs(value) : value;
}

/**
 * Resolve an extrude's end condition into the concrete pieces the shared
 * builders consume:
 *   - `param`: the signed value fed to `sweepPostOps` (its magnitude is the
 *     kernel height, its sign the sweep direction).
 *   - `height`: the kernel extrude height (always `> 0`).
 *   - `draft`: draft angle in degrees (passed to `Shape.extrude`).
 *   - `postExtra`: post-ops appended after `sweepPostOps`, e.g. the centering
 *     translate that turns a one-sided extrude into a symmetric one.
 *   - `clip`: an optional terminating half-space `{ point, normal }` (the
 *     "up to face" cap), intersected with the extrude after orientation.
 *
 * `blind` (the default) keeps the historical behavior exactly. `symmetric`
 * and `through` extrude forward and re-center on the sketch plane; `through`
 * uses `sweep.reach` (the scene span along the normal). `toFace` extrudes
 * toward `sweep.target` (a plane `{ origin, normal }`, typically a picked
 * face) far enough to cross it, then clips at that plane.
 */
export function extrudePlan(sweep) {
  const end = sweep.end ?? 'blind';
  const value = sweep.value;
  const draft = sweep.draft ?? 0;
  const n = planeNormal(sweep.plane);
  const origin = planeToWorld(sweep.plane, 0, 0);

  if (end === 'blind') {
    return { param: value, height: Math.abs(value), draft, postExtra: [], clip: null };
  }
  if (end === 'symmetric' || end === 'through') {
    // Extrude forward, then slide back half the height so the span straddles
    // the sketch plane. `through` spans the whole scene along the normal.
    const height = end === 'through' ? sweep.reach ?? Math.abs(value) : Math.abs(value);
    return {
      param: height,
      height,
      draft,
      postExtra: [{ op: 'translate', args: scale3(n, -0.5 * height) }],
      clip: null,
    };
  }
  if (end === 'toFace') {
    const target = sweep.target;
    if (!target) throw new Error('up-to-face extrude needs a target face');
    const height = sweep.reach ?? Math.abs(value) * 4;
    // Extrude toward whichever side of the sketch plane the target lies on.
    const dir = dot3(sub3(target.origin, origin), n) >= 0 ? 1 : -1;
    // Keep the half-space on the sketch-plane side of the target plane: the
    // kernel's half-space is interior where keepNormal·(p − point) ≤ 0, so we
    // want keepNormal·(origin − target) ≤ 0.
    const tn = target.normal;
    const keep = dot3(sub3(origin, target.origin), tn) <= 0 ? 1 : -1;
    return {
      param: dir * height,
      height,
      draft,
      postExtra: [],
      clip: { point: target.origin, normal: scale3(tn, keep) },
    };
  }
  throw new Error(`unknown end condition: ${end}`);
}

/** Extrude op args: `[height]`, or `[height, draftDegrees]` when drafted. */
function extrudeArgs(height, draft) {
  return draft ? [height, draft] : [height];
}

/**
 * Build the swept, plane-oriented shape for a sweep descriptor. Extrudes
 * honor `mode`/`end`/`draft`/`target`/`reach` (see `extrudePlan`); revolves
 * take a signed angle in degrees. Frees the profile and every intermediate
 * shape; the caller owns the returned shape.
 */
export function buildSweepShape(ShapeClass, ProfileClass, sweep) {
  const { kind, plane, value } = sweep;
  const ops = nativeSweepOps(sweep.ops, plane, kind);
  const plan = kind === 'extrude' ? extrudePlan(sweep) : null;
  const param = plan ? plan.param : value;
  const profile = new ProfileClass(ops.start[0], ops.start[1]);
  let shape;
  try {
    for (const seg of ops.segs) profile.arcTo(seg.x, seg.y, seg.bulge);
    profile.close();
    shape = plan
      ? ShapeClass.extrude(profile, ...extrudeArgs(plan.height, plan.draft))
      : ShapeClass[kind](profile, sweepArg(kind, value));
  } finally {
    profile.free?.();
  }
  const apply = (next) => {
    shape.free?.();
    shape = next;
  };
  for (const post of sweepPostOps(plane, kind, param)) apply(shape[post.op](...post.args));
  if (plan) {
    for (const post of plan.postExtra) apply(shape[post.op](...post.args));
    if (plan.clip) {
      const { point, normal } = plan.clip;
      const half = ShapeClass.halfSpace(...point, ...normal);
      const next = shape.intersect(half);
      half.free?.();
      apply(next);
    }
  }
  return shape;
}

/**
 * Graft a sweep onto an existing construction tree as plain node data (no
 * retained shapes): the sweep node wrapped in its plane post-ops, combined
 * with `root` when one exists — unioned for a boss, subtracted for a cut.
 * Synthetic ids are negative so they can't collide with traced ids; the tree
 * is only serialized, then re-evaluated.
 */
export function sweepTreeNode(root, sweep) {
  const { kind, plane, value } = sweep;
  const ops = nativeSweepOps(sweep.ops, plane, kind);
  const plan = kind === 'extrude' ? extrudePlan(sweep) : null;
  const param = plan ? plan.param : value;
  let id = -1;
  let node = {
    id: id--,
    op: kind,
    args: plan ? extrudeArgs(plan.height, plan.draft) : [sweepArg(kind, value)],
    children: [],
    profile: { start: [...ops.start], segs: ops.segs.map((s) => ({ ...s })) },
  };
  for (const post of sweepPostOps(plane, kind, param)) {
    node = { id: id--, op: post.op, args: post.args, children: [node] };
  }
  if (plan) {
    for (const post of plan.postExtra) {
      node = { id: id--, op: post.op, args: post.args, children: [node] };
    }
    if (plan.clip) {
      const { point, normal } = plan.clip;
      const half = { id: id--, op: 'halfSpace', args: [...point, ...normal], children: [] };
      node = { id: id--, op: 'intersect', args: [], children: [node, half] };
    }
  }
  if (!root) return node;
  const boolOp = kind === 'extrude' && sweep.mode === 'cut' ? 'subtract' : 'union';
  return { id: id--, op: boolOp, args: [], children: [root, node] };
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
