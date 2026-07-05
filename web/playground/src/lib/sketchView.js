// Pure math for the sketch-mode viewport: SolidWorks-style "Normal To"
// camera poses, orthographic frustum sizing, adaptive grid spacing, and the
// px <-> world-unit mapping shared by the 2D sketch overlay and the 3D
// orthographic camera. Kept free of three.js so it can be unit-tested in
// isolation.

import { planeToWorld, worldToPlane } from './sketch/profile.js';

/**
 * Per-plane view poses. `normal` is the direction from the plane toward the
 * camera; `up` is the world-space camera up that gives the conventional CAD
 * orientation (front: Y up; top: looking down -Y with Z running down-screen;
 * right: Y up).
 */
export const SKETCH_VIEW_POSES = {
  XY: { normal: [0, 0, 1], up: [0, 1, 0] }, // front
  XZ: { normal: [0, 1, 0], up: [0, 0, -1] }, // top
  YZ: { normal: [1, 0, 0], up: [0, 1, 0] }, // right
};

/**
 * Camera pose looking orthogonally at a sketch plane (all planes pass through
 * the origin). The current orbit target is projected onto the plane so the
 * view stays centered on the region the user was looking at, and the camera
 * sits `dist` away along the plane normal.
 *
 * Returns { position, target, up } as [x, y, z] triples, or null for an
 * unknown plane.
 */
export function sketchViewPose(plane, target, dist) {
  const spec = SKETCH_VIEW_POSES[plane];
  if (!spec) return null;
  const n = spec.normal;
  // Project the target onto the plane: t - (t . n) n.
  const d = target[0] * n[0] + target[1] * n[1] + target[2] * n[2];
  const projected = [
    target[0] - d * n[0],
    target[1] - d * n[1],
    target[2] - d * n[2],
  ];
  return {
    position: [
      projected[0] + n[0] * dist,
      projected[1] + n[1] * dist,
      projected[2] + n[2] * dist,
    ],
    target: projected,
    up: [...spec.up],
  };
}

/**
 * Half-extents of an orthographic frustum that matches the apparent size of a
 * perspective camera with vertical FOV `fovDeg` at distance `dist` from its
 * target.
 */
export function orthoHalfExtents(dist, fovDeg, aspect) {
  const halfH = dist * Math.tan((fovDeg * Math.PI) / 180 / 2);
  return { halfW: halfH * aspect, halfH };
}

/**
 * Screen pixels per world unit of the sketch orthographic camera: the ortho
 * frustum height is 2 * dist * tan(fov / 2) (matched to the perspective
 * camera's apparent size at the target), mapped onto the viewport height.
 * The factor is uniform in x and y because the frustum width uses the same
 * aspect ratio as the viewport.
 */
export function sketchPxPerUnit(dist, fovDeg, viewportHeightPx) {
  const { halfH } = orthoHalfExtents(dist, fovDeg, 1);
  return viewportHeightPx / (2 * halfH);
}

/** Camera distance that yields `pxPerUnit` — inverse of `sketchPxPerUnit`. */
export function sketchDistForPxPerUnit(pxPerUnit, fovDeg, viewportHeightPx) {
  return viewportHeightPx / (2 * pxPerUnit * Math.tan((fovDeg * Math.PI) / 180 / 2));
}

/**
 * The 2D overlay view `{ cx, cy, scale }` (view center in sketch-plane
 * coordinates, px per world unit) that exactly matches the sketch camera
 * looking at `target` (a point on the plane) from distance `dist`.
 */
export function sketchViewFromCamera(plane, target, dist, fovDeg, viewportHeightPx) {
  const [cx, cy] = worldToPlane(plane, target);
  return { cx, cy, scale: sketchPxPerUnit(dist, fovDeg, viewportHeightPx) };
}

/**
 * Inverse of `sketchViewFromCamera`: the normal-to camera pose (plus its
 * distance) whose world-to-screen transform equals the overlay view.
 */
export function cameraFromSketchView(plane, view, fovDeg, viewportHeightPx) {
  if (!SKETCH_VIEW_POSES[plane]) return null;
  const dist = sketchDistForPxPerUnit(view.scale, fovDeg, viewportHeightPx);
  const target = planeToWorld(plane, view.cx, view.cy);
  return { ...sketchViewPose(plane, target, dist), dist };
}

/**
 * Sketch-plane (u, v) to overlay screen px for view `{ cx, cy, scale }` and
 * viewport `size` `{ w, h }`. Screen y grows downward; v grows up.
 */
export function sketchWorldToScreen(view, size, u, v) {
  return [
    (u - view.cx) * view.scale + size.w / 2,
    size.h / 2 - (v - view.cy) * view.scale,
  ];
}

/** Inverse of `sketchWorldToScreen`. */
export function sketchScreenToWorld(view, size, sx, sy) {
  return {
    x: (sx - size.w / 2) / view.scale + view.cx,
    y: (size.h / 2 - sy) / view.scale + view.cy,
  };
}

/**
 * Adaptive decade grid for the 3D viewport, driven by camera distance.
 * Minor lines fade out continuously as the camera pulls back (so the next
 * decade takes over without popping), and the whole grid fades radially at
 * `fadeDist` from the camera.
 *
 * Returns { minor, major, minorAlpha, fadeDist }.
 */
export function gridLevels(dist) {
  const safe = Math.max(dist, 1e-6);
  // At dist=10 the minor spacing is 0.5 (matching the old GridHelper look);
  // each 10x of distance steps the grid up one decade.
  const lf = Math.log10(safe / 10);
  const level = Math.floor(lf);
  const t = lf - level;
  const minor = 0.5 * 10 ** level;
  return {
    minor,
    major: minor * 10,
    minorAlpha: 1 - t,
    fadeDist: safe * 4,
  };
}

/** Smooth cubic ease-in-out on [0, 1]. */
export function easeInOutCubic(t) {
  const x = Math.min(1, Math.max(0, t));
  return x < 0.5 ? 4 * x * x * x : 1 - (-2 * x + 2) ** 3 / 2;
}

/**
 * Side length for the sketch-plane indicator: several times the scene extent
 * (bounding-sphere center offset + radius) so drawn geometry never outruns
 * it, with a sensible floor for empty scenes.
 */
export function planeIndicatorSize(sceneCenter, sceneRadius) {
  const reach =
    Math.hypot(sceneCenter[0], sceneCenter[1], sceneCenter[2]) +
    Math.max(sceneRadius, 0);
  return Math.max(20, reach * 6);
}
