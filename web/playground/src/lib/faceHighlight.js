// Face hover/selection highlighting (of-4eh.18): the hovered or selected
// planar face is shown by darkening its triangles through a per-vertex color
// attribute on the main mesh — no coplanar overlay geometry, so no
// z-fighting at any zoom or angle. The mesh is expanded to non-indexed
// triangles so a painted region keeps a crisp boundary: with shared
// vertices, colors would interpolate one triangle deep into every
// neighboring face.
//
// Kept free of three.js and React so it can be unit-tested on plain arrays.

/** Unhighlighted vertex color (multiplied against the material color). */
export const BASE_RGB = [1, 1, 1];
/** Hover: a neutral darkening, subtle but unmistakable. */
export const HOVER_RGB = [0.62, 0.66, 0.72];
/** Selected face: stronger, with a cool accent shift so it reads as
 * "selected" rather than "shadowed" and stays distinct from hover. */
export const SELECTED_RGB = [0.35, 0.72, 1.15];

/**
 * Expand an indexed mesh `{ positions, normals, indices }` into non-indexed
 * triangle soup plus an all-`BASE_RGB` color attribute. Triangle order (and
 * therefore raycast `faceIndex` numbering) is preserved.
 */
export function expandToNonIndexed({ positions, normals, indices }) {
  const vertCount = indices.length;
  const pos = new Float32Array(vertCount * 3);
  const nor = new Float32Array(vertCount * 3);
  for (let i = 0; i < vertCount; i += 1) {
    const src = indices[i] * 3;
    const dst = i * 3;
    pos[dst] = positions[src];
    pos[dst + 1] = positions[src + 1];
    pos[dst + 2] = positions[src + 2];
    nor[dst] = normals[src];
    nor[dst + 1] = normals[src + 1];
    nor[dst + 2] = normals[src + 2];
  }
  const colors = new Float32Array(vertCount * 3).fill(1);
  return { positions: pos, normals: nor, colors };
}

/** Write one triangle's 3 vertex colors; false if `tri` is out of range. */
function paintTri(colors, tri, [r, g, b]) {
  const base = tri * 9;
  if (!Number.isInteger(tri) || base < 0 || base + 9 > colors.length) return false;
  for (let v = 0; v < 9; v += 3) {
    colors[base + v] = r;
    colors[base + v + 1] = g;
    colors[base + v + 2] = b;
  }
  return true;
}

/**
 * Repaint the non-indexed `colors` attribute: restore `previousTris` to
 * `BASE_RGB`, then paint each `{ tris, rgb }` region in order (later regions
 * win on overlap, so pass hover before selected). Out-of-range triangles
 * (a stale region against a rebuilt mesh) are skipped. Returns the list of
 * triangles painted, to hand back as `previousTris` next time.
 */
export function paintHighlights(colors, previousTris, regions) {
  for (const tri of previousTris) paintTri(colors, tri, BASE_RGB);
  const painted = [];
  for (const { tris, rgb } of regions) {
    for (const tri of tris) {
      if (paintTri(colors, tri, rgb)) painted.push(tri);
    }
  }
  return painted;
}
