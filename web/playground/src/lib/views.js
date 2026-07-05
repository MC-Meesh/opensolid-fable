// Standard view orientations, following SolidWorks naming.
//
// World convention (documented, matches three.js and the Shape API): **Y is
// up**, the ground grid lies in the XZ plane, and "front" looks along -Z at
// the XY plane. Cameras always keep world up = +Y; the top/bottom directions
// carry a tiny Z tilt so a look-at from straight above never degenerates
// against that up vector.
//
// Kept free of three.js imports so it can be unit-tested directly.

/** Camera distance per unit of bounding radius when framing a shape. */
export const FIT_DISTANCE_FACTOR = 2.6;

/** Smallest bounding radius we frame against (avoids a zero-size camera). */
export const MIN_FIT_RADIUS = 0.1;

const POLE_TILT = 0.001;

function normalize([x, y, z]) {
  const len = Math.hypot(x, y, z);
  return [x / len, y / len, z / len];
}

// Unit direction from the look-at target toward the camera, per view.
const VIEW_DIRECTIONS = {
  front: [0, 0, 1],
  back: [0, 0, -1],
  left: [-1, 0, 0],
  right: [1, 0, 0],
  top: normalize([0, 1, POLE_TILT]),
  bottom: normalize([0, -1, POLE_TILT]),
  iso: normalize([1, 0.7, 1.2]),
};

/** Names of every standard view, in SolidWorks Ctrl+1..7 order. */
export const VIEW_NAMES = ['front', 'back', 'left', 'right', 'top', 'bottom', 'iso'];

/**
 * Keyboard shortcut map: digit -> view name (SolidWorks Ctrl+1..7 order;
 * plain digits also work since browsers reserve Ctrl/Cmd+digit for tabs).
 */
export const VIEW_SHORTCUTS = {
  1: 'front',
  2: 'back',
  3: 'left',
  4: 'right',
  5: 'top',
  6: 'bottom',
  7: 'iso',
};

/**
 * Unit direction from the target toward the camera for a named view, or
 * `null` for an unknown name.
 */
export function viewDirection(name) {
  const dir = VIEW_DIRECTIONS[name];
  return dir ? [...dir] : null;
}

/**
 * The standard view that looks straight down a world axis: clicking the +X
 * triad arm gives the right view, +Y the top view, +Z the front view.
 */
export function axisView(axis, positive = true) {
  if (axis === 'x') return positive ? 'right' : 'left';
  if (axis === 'y') return positive ? 'top' : 'bottom';
  if (axis === 'z') return positive ? 'front' : 'back';
  return null;
}

/**
 * Camera position and target for a named view framing a bounding sphere.
 * Returns `{ position, target }` or `null` for an unknown view.
 */
export function cameraStateFor(name, center, radius) {
  const dir = viewDirection(name);
  if (!dir) return null;
  const dist = Math.max(radius, MIN_FIT_RADIUS) * FIT_DISTANCE_FACTOR;
  return {
    position: [
      center[0] + dir[0] * dist,
      center[1] + dir[1] * dist,
      center[2] + dir[2] * dist,
    ],
    target: [...center],
  };
}
