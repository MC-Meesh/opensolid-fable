// Section view: geometry for an axis-aligned clipping plane that slices the
// displayed model so its interior can be inspected (SolidWorks "Section
// View"). Display-only — nothing here touches the model or the script.
//
// Kept free of three.js imports so the plane math can be unit-tested directly;
// Viewport3D turns clipPlaneParams() into a THREE.Plane and drives the
// stencil-buffer capping and the drag handle.
//
// World convention (shared with lib/views.js): Y is up, the ground grid lies
// in XZ. A section is one of the three world axes, an offset (signed distance
// along that axis, in model units), and a flip that swaps which half is kept.

/** The three section-plane orientations, by the world axis they cut across. */
export const SECTION_AXES = ['X', 'Y', 'Z'];

const AXIS_VEC = {
  X: [1, 0, 0],
  Y: [0, 1, 0],
  Z: [0, 0, 1],
};

/** Index into a 3-vector for a section axis ('X' -> 0, 'Y' -> 1, 'Z' -> 2). */
export function axisComponent(axis) {
  return SECTION_AXES.indexOf(axis);
}

/**
 * Axis-aligned bounds of a positions array (flat [x,y,z,...]), plus the
 * center point and half-diagonal radius. Returns a unit box at the origin for
 * an empty array so callers always get a usable frame.
 */
export function sectionBounds(positions) {
  if (!positions || positions.length < 3) {
    return { min: [-0.5, -0.5, -0.5], max: [0.5, 0.5, 0.5], center: [0, 0, 0], radius: 0.5 };
  }
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let k = 0; k < 3; k += 1) {
      const c = positions[i + k];
      if (c < min[k]) min[k] = c;
      if (c > max[k]) max[k] = c;
    }
  }
  const center = [(min[0] + max[0]) / 2, (min[1] + max[1]) / 2, (min[2] + max[2]) / 2];
  const radius = Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]) / 2;
  return { min, max, center, radius };
}

/**
 * Slider travel for the offset along the section axis: the model's extent on
 * that axis, padded a hair so the plane can clear the model at either end.
 */
export function offsetRange(bounds, axis) {
  const i = axisComponent(axis);
  const span = bounds.max[i] - bounds.min[i];
  const pad = Math.max(span * 0.05, 1e-3);
  return { min: bounds.min[i] - pad, max: bounds.max[i] + pad };
}

/**
 * The default section for a freshly opened view: cut across X through the
 * model center, keeping the lower half.
 */
export function defaultSection(bounds) {
  return { axis: 'X', offset: bounds.center[0], flip: false };
}

/**
 * Re-seat a section's offset when its axis changes, so switching X -> Y lands
 * the plane back at the model center rather than at a stale coordinate.
 */
export function reseatOffset(section, bounds) {
  return { ...section, offset: bounds.center[axisComponent(section.axis)] };
}

/**
 * THREE.Plane parameters `{ normal, constant }` for a section. three.js
 * clipping keeps the half-space where `normal · p + constant >= 0` and
 * discards the rest, so we choose the normal/constant to keep, by default,
 * the side with the smaller axis coordinate (`coord <= offset`); flip keeps
 * the other half. The plane equation is `s·coord - s·offset >= 0` with
 * `s = -1` (default) or `s = +1` (flipped).
 */
export function clipPlaneParams({ axis, offset, flip }) {
  const a = AXIS_VEC[axis];
  const s = flip ? 1 : -1;
  return {
    normal: [a[0] * s, a[1] * s, a[2] * s],
    constant: -s * offset,
  };
}

/**
 * World position for the drag handle / plane widget: the model center in the
 * two in-plane axes, and the offset along the section axis.
 */
export function handlePosition(bounds, { axis, offset }) {
  const i = axisComponent(axis);
  const p = [...bounds.center];
  p[i] = offset;
  return p;
}
