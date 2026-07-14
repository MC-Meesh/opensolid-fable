// Persistent face references (of-fsl.8): the stable-reference half of
// parametric rebuild. A feature placed on a picked face stores a geometric
// reference — orientation + anchor point — instead of only baking the face's
// plane as literal numbers, so the reference can be RE-RESOLVED against the
// rebuilt mesh after an upstream edit (SolidWorks persistent-naming). When no
// matching face survives, the reference is DANGLING and the feature is flagged.
//
// See docs/parametric-rebuild.md for the scheme, the nearest-point heuristic,
// and the honest-MVP limits. Kept free of three.js/React/WASM so it is
// unit-testable on plain region data (same pattern as facePlane.js).

/** Orientation tolerance: a candidate face normal must lie within this angle
 * of the stored normal to be considered the same face. Matches the region
 * grower's admission tolerance in facePlane.js. */
export const NORMAL_TOL_DEG = 3;

/** Anchor tolerance as a fraction of the model's bounding-sphere radius: the
 * best-oriented face must sit within this distance of the stored anchor, or
 * the reference is treated as dangling rather than snapping to a far face. */
export const ANCHOR_TOL_FACTOR = 0.35;

const sub = (a, b) => [a[0] - b[0], a[1] - b[1], a[2] - b[2]];
const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const norm = (a) => Math.hypot(a[0], a[1], a[2]);

/**
 * Build a persistent face reference from a detected face plane and its owning
 * feature key. `plane` is a `detectFacePlane`/region plane
 * (`{ origin, normal, u, v, extent }`); `owner` is the feature key
 * (`extrude:1`) the reference belongs to.
 *
 * Returns `{ owner, normal, anchor, extent }` — a plain, serializable record.
 * The orientation is the face normal; the anchor is the region centroid
 * (`origin`); `extent` scales nothing on its own but travels with the ref for
 * future tolerance tuning.
 */
export function faceRefFromPlane(owner, plane) {
  return {
    owner,
    normal: [...plane.normal],
    anchor: [...plane.origin],
    extent: plane.extent ?? 0,
  };
}

/**
 * Enumerate the distinct planar face regions of a mesh via a region index
 * (`createFaceRegionIndex`). Every triangle is classified once; regions are
 * shared objects, so identity dedupes them. Returns the planar regions only,
 * each `{ planar: true, plane, tris }` as produced by facePlane.js.
 *
 * `triCount` is the mesh's triangle count (indices.length / 3). This is an
 * O(triangles) sweep, amortized by the index's per-triangle memoization; it
 * runs only when a model actually carries face references.
 */
export function planarRegionsOf(regionIndex, triCount) {
  const seen = new Set();
  const regions = [];
  for (let t = 0; t < triCount; t += 1) {
    const region = regionIndex.regionAt(t);
    if (!region?.planar || seen.has(region)) continue;
    seen.add(region);
    regions.push(region);
  }
  return regions;
}

/** Area-weighted-ish centroid of a region: its plane origin, which the grower
 * already computes as the area-weighted centroid. */
function centroidOf(region) {
  return region.plane.origin;
}

/**
 * Re-resolve a face reference against the planar regions of the current mesh.
 *
 * Orientation gates first: only regions whose normal is within
 * `NORMAL_TOL_DEG` of the reference normal are candidates (a coplanar but
 * oppositely-oriented face — a hole's far wall — is rejected outright). Among
 * those, the region whose centroid is nearest the stored anchor wins, provided
 * that distance is within `ANCHOR_TOL_FACTOR * modelRadius`.
 *
 * Returns:
 *   { resolved: true,  plane, anchor, distance }  — a live reference; `plane`
 *       is the matched region's fresh plane and `anchor` its centroid, so the
 *       caller can re-lock the ref onto where the face is now.
 *   { resolved: false, reason }                   — dangling: no face cleared
 *       both gates ('no matching orientation' or 'nearest face too far').
 */
export function resolveFaceRef(ref, regions, modelRadius) {
  const cosTol = Math.cos((NORMAL_TOL_DEG * Math.PI) / 180);
  const refNormal = ref.normal;
  const rn = norm(refNormal);
  const unitRef = rn > 0 ? refNormal.map((c) => c / rn) : refNormal;

  let best = null;
  let bestDist = Infinity;
  let anyOriented = false;
  for (const region of regions) {
    if (!region.planar) continue;
    const n = region.plane.normal;
    if (dot(n, unitRef) < cosTol) continue;
    anyOriented = true;
    const d = norm(sub(centroidOf(region), ref.anchor));
    if (d < bestDist) {
      bestDist = d;
      best = region;
    }
  }

  if (!best) {
    return { resolved: false, reason: 'no matching orientation' };
  }
  const tol = ANCHOR_TOL_FACTOR * (modelRadius > 0 ? modelRadius : 1);
  if (bestDist > tol) {
    return { resolved: false, reason: 'nearest face too far' };
  }
  return {
    resolved: true,
    plane: best.plane,
    anchor: [...centroidOf(best)],
    distance: bestDist,
  };
}

/**
 * Re-resolve every reference in a `Map<featureKey, FaceRef>` against the mesh's
 * planar regions. Returns `{ statuses, refs }`:
 *   statuses — `Map<featureKey, { status: 'ok'|'dangling', reason? }>`
 *   refs     — a NEW `Map<featureKey, FaceRef>` with live refs re-anchored to
 *              the face's current centroid (dangling refs pass through
 *              unchanged so a later edit can bring the face back).
 *
 * `modelRadius` scales the anchor tolerance (half the mesh bounding-box
 * diagonal is the natural choice).
 */
export function resolveRefs(refMap, regions, modelRadius) {
  const statuses = new Map();
  const refs = new Map();
  for (const [key, ref] of refMap) {
    const result = resolveFaceRef(ref, regions, modelRadius);
    if (result.resolved) {
      statuses.set(key, { status: 'ok' });
      refs.set(key, { ...ref, anchor: result.anchor });
    } else {
      statuses.set(key, { status: 'dangling', reason: result.reason });
      refs.set(key, ref);
    }
  }
  return { statuses, refs };
}
