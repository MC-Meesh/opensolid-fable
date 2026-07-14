// Reference-geometry collection store (of-fsl.14): the App-level parallel
// state that holds user-created reference planes / axes / points / coordinate
// systems. Reference geometry is NOT a Shape and does not live in the
// script/traced-tree source of truth, so it is kept as a keyed list beside it
// (the same pattern as featureNames / hiddenKeys).
//
// Kept pure (no React) so the collection logic and the method -> constructor
// dispatch unit-test in isolation. App owns the array in state and routes its
// add/delete/rename handlers through here.
//
// A collection item is `{ id, name, kind, entity }`:
//   id     — stable numeric key (monotonic; never reused after a delete)
//   name   — SolidWorks-style display name ('Plane1', 'Axis2', ...)
//   kind   — 'plane' | 'axis' | 'point' | 'csys'
//   entity — the plain-data geometry from lib/referenceGeometry.js

import {
  angledPlane,
  axisFromPlaneIntersection,
  axisFromPointDirection,
  axisFromTwoPoints,
  csysFromPlane,
  csysFromPointAndAxes,
  defaultReferenceName,
  midPlane,
  offsetPlane,
  pointAtAxisPlane,
  pointFromCoords,
  pointFromMidpoint,
} from './referenceGeometry.js';

// Method dispatch: (kind, method) -> builder(params) returning a plain-data
// entity. Params carry already-resolved bases (plane strings/objects, axis and
// point entities), so the store stays free of id resolution — the panel maps
// its base pickers to concrete geometry before calling build().
const BUILDERS = {
  plane: {
    offset: (p) => offsetPlane(p.base, p.distance),
    angled: (p) => angledPlane(p.base, p.angleDeg, p.hinge),
    mid: (p) => midPlane(p.a, p.b),
  },
  axis: {
    'two-points': (p) => axisFromTwoPoints(p.p1, p.p2),
    'point-direction': (p) => axisFromPointDirection(p.origin, p.direction),
    'plane-intersection': (p) => axisFromPlaneIntersection(p.a, p.b),
  },
  point: {
    coords: (p) => pointFromCoords(p.position),
    midpoint: (p) => pointFromMidpoint(p.p1, p.p2),
    'axis-plane': (p) => pointAtAxisPlane(p.axis, p.plane),
  },
  csys: {
    plane: (p) => csysFromPlane(p.plane),
    'point-axes': (p) => csysFromPointAndAxes(p.origin, p.xDir, p.yHint),
  },
};

/** Every method key a given kind supports (creation-UI menu order). */
export function methodsForKind(kind) {
  return Object.keys(BUILDERS[kind] ?? {});
}

/**
 * Build a reference-geometry entity from a creation spec. Throws (with the
 * constructor's own message) on a degenerate configuration or an unknown
 * kind/method — the panel surfaces the message and keeps the collection
 * unchanged.
 */
export function buildReferenceEntity(kind, method, params) {
  const builder = BUILDERS[kind]?.[method];
  if (!builder) throw new Error(`unknown reference method: ${kind}/${method}`);
  return builder(params);
}

/** Names currently in use — feeds defaultReferenceName for the next ordinal. */
export function referenceNames(list) {
  return list.map((item) => item.name);
}

/** Next free numeric id: one past the current maximum (never reuses ids). */
export function nextReferenceId(list) {
  return list.reduce((max, item) => Math.max(max, item.id), 0) + 1;
}

/**
 * Append a reference entity to the collection, assigning a fresh id and a
 * default SolidWorks-style name (unless `name` overrides it). Returns a new
 * array (the input is untouched).
 */
export function addReference(list, kind, entity, name) {
  return [
    ...list,
    {
      id: nextReferenceId(list),
      name: name || defaultReferenceName(kind, referenceNames(list)),
      kind,
      entity,
    },
  ];
}

/**
 * Build and append in one step — the panel's create action. Throws through
 * from buildReferenceEntity on a degenerate spec, leaving `list` unchanged.
 */
export function createReference(list, kind, method, params, name) {
  return addReference(list, kind, buildReferenceEntity(kind, method, params), name);
}

/** Remove the item with `id`. Returns a new array (unchanged if absent). */
export function deleteReference(list, id) {
  return list.filter((item) => item.id !== id);
}

/**
 * Rename the item with `id`. An empty/blank name reverts to the kind's default
 * name for that slot. Returns a new array (unchanged if the id is absent).
 */
export function renameReference(list, id, name) {
  const trimmed = (name ?? '').trim();
  return list.map((item) => {
    if (item.id !== id) return item;
    const others = list.filter((o) => o.id !== id).map((o) => o.name);
    return {
      ...item,
      name: trimmed || defaultReferenceName(item.kind, others),
    };
  });
}

/** The reference planes only, as {id, name, entity} — for plane pickers. */
export function referencePlanes(list) {
  return list.filter((item) => item.kind === 'plane');
}

/** Look up a collection item's entity by id (or undefined). */
export function referenceEntityById(list, id) {
  return list.find((item) => item.id === id)?.entity;
}
