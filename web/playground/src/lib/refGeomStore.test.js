import { describe, expect, it } from 'vitest';
import {
  addReference,
  buildReferenceEntity,
  createReference,
  deleteReference,
  methodsForKind,
  nextReferenceId,
  referenceEntityById,
  referenceNames,
  referencePlanes,
  renameReference,
} from './refGeomStore.js';

const plane = () => buildReferenceEntity('plane', 'offset', { base: 'XY', distance: 2 });

describe('buildReferenceEntity dispatch', () => {
  it('builds each plane method', () => {
    expect(buildReferenceEntity('plane', 'offset', { base: 'XY', distance: 1 }).kind).toBe('plane');
    expect(buildReferenceEntity('plane', 'angled', { base: 'XY', angleDeg: 30 }).method).toBe('angled');
    expect(
      buildReferenceEntity('plane', 'mid', { a: 'XY', b: buildReferenceEntity('plane', 'offset', { base: 'XY', distance: 4 }) }).origin[2]
    ).toBeCloseTo(2);
  });

  it('builds each axis method', () => {
    expect(buildReferenceEntity('axis', 'two-points', { p1: [0, 0, 0], p2: [1, 0, 0] }).kind).toBe('axis');
    expect(buildReferenceEntity('axis', 'point-direction', { origin: [0, 0, 0], direction: [0, 1, 0] }).method).toBe('point-direction');
    expect(buildReferenceEntity('axis', 'plane-intersection', { a: 'XY', b: 'XZ' }).kind).toBe('axis');
  });

  it('builds each point method', () => {
    expect(buildReferenceEntity('point', 'coords', { position: [1, 2, 3] }).position).toEqual([1, 2, 3]);
    expect(buildReferenceEntity('point', 'midpoint', { p1: [0, 0, 0], p2: [2, 0, 0] }).position[0]).toBeCloseTo(1);
    const axis = buildReferenceEntity('axis', 'point-direction', { origin: [1, 1, -2], direction: [0, 0, 1] });
    expect(buildReferenceEntity('point', 'axis-plane', { axis, plane: 'XY' }).position[2]).toBeCloseTo(0);
  });

  it('builds each csys method', () => {
    expect(buildReferenceEntity('csys', 'plane', { plane: 'XY' }).kind).toBe('csys');
    expect(buildReferenceEntity('csys', 'point-axes', { origin: [0, 0, 0], xDir: [1, 0, 0], yHint: [0, 1, 0] }).method).toBe('point-axes');
  });

  it('throws on an unknown kind/method', () => {
    expect(() => buildReferenceEntity('plane', 'nope', {})).toThrow(/unknown reference method/);
    expect(() => buildReferenceEntity('widget', 'x', {})).toThrow(/unknown reference method/);
  });

  it('propagates a degenerate-spec error', () => {
    expect(() => buildReferenceEntity('axis', 'two-points', { p1: [0, 0, 0], p2: [0, 0, 0] })).toThrow(/distinct/);
  });
});

describe('methodsForKind', () => {
  it('lists the methods per kind', () => {
    expect(methodsForKind('plane')).toEqual(['offset', 'angled', 'mid']);
    expect(methodsForKind('axis')).toContain('plane-intersection');
    expect(methodsForKind('nope')).toEqual([]);
  });
});

describe('collection ops', () => {
  it('nextReferenceId starts at 1 and never reuses', () => {
    expect(nextReferenceId([])).toBe(1);
    const l = addReference([], 'plane', plane());
    expect(nextReferenceId(l)).toBe(2);
    const after = deleteReference(l, 1);
    expect(nextReferenceId(after)).toBe(1); // empty -> back to 1
    const l2 = addReference(l, 'axis', buildReferenceEntity('axis', 'two-points', { p1: [0, 0, 0], p2: [1, 0, 0] }));
    expect(nextReferenceId(deleteReference(l2, 2))).toBe(2); // id 1 remains -> max+1
  });

  it('addReference assigns id, default name, and kind; input untouched', () => {
    const l0 = [];
    const l1 = addReference(l0, 'plane', plane());
    expect(l0).toEqual([]);
    expect(l1[0]).toMatchObject({ id: 1, name: 'Plane1', kind: 'plane' });
    const l2 = addReference(l1, 'plane', plane());
    expect(l2[1].name).toBe('Plane2');
  });

  it('addReference honors a custom name', () => {
    expect(addReference([], 'plane', plane(), 'TopDatum')[0].name).toBe('TopDatum');
  });

  it('createReference builds and appends', () => {
    const l = createReference([], 'point', 'coords', { position: [1, 2, 3] });
    expect(l[0].kind).toBe('point');
    expect(l[0].entity.position).toEqual([1, 2, 3]);
    expect(l[0].name).toBe('Point1');
  });

  it('createReference leaves the list unchanged on a bad spec', () => {
    const l = createReference([], 'plane', 'offset', { base: 'XY', distance: 1 });
    expect(() => createReference(l, 'axis', 'two-points', { p1: [0, 0, 0], p2: [0, 0, 0] })).toThrow();
    expect(l).toHaveLength(1);
  });

  it('deleteReference removes by id', () => {
    const l = addReference(addReference([], 'plane', plane()), 'plane', plane());
    expect(deleteReference(l, 1).map((i) => i.id)).toEqual([2]);
    expect(deleteReference(l, 99)).toHaveLength(2); // absent id: no-op
  });

  it('renameReference sets a name and reverts blank to the default', () => {
    const l = addReference([], 'plane', plane());
    expect(renameReference(l, 1, 'Datum').at(0).name).toBe('Datum');
    expect(renameReference(renameReference(l, 1, 'Datum'), 1, '  ').at(0).name).toBe('Plane1');
  });
});

describe('selectors', () => {
  it('referenceNames lists names', () => {
    const l = addReference(addReference([], 'plane', plane(), 'A'), 'axis', buildReferenceEntity('axis', 'two-points', { p1: [0, 0, 0], p2: [1, 0, 0] }), 'B');
    expect(referenceNames(l)).toEqual(['A', 'B']);
  });

  it('referencePlanes filters to planes only', () => {
    let l = addReference([], 'plane', plane());
    l = addReference(l, 'axis', buildReferenceEntity('axis', 'two-points', { p1: [0, 0, 0], p2: [1, 0, 0] }));
    expect(referencePlanes(l).map((i) => i.kind)).toEqual(['plane']);
  });

  it('referenceEntityById returns the entity or undefined', () => {
    const l = addReference([], 'plane', plane());
    expect(referenceEntityById(l, 1).kind).toBe('plane');
    expect(referenceEntityById(l, 42)).toBeUndefined();
  });
});
