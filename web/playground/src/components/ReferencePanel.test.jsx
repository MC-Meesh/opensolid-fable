/**
 * Server-render smoke tests, matching SweepPanel.test.jsx: catch reference
 * errors and broken JSX in the render path without a browser. The geometry
 * constructors and the collection store are unit-tested in
 * lib/referenceGeometry.test.js and lib/refGeomStore.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import ReferencePanel from './ReferencePanel.jsx';

const noop = () => {};

function render(props = {}) {
  return renderToString(
    <ReferencePanel
      open
      refGeom={[]}
      error={null}
      onCreate={noop}
      onClose={noop}
      {...props}
    />
  );
}

describe('ReferencePanel', () => {
  it('renders nothing when closed', () => {
    expect(render({ open: false })).toBe('');
  });

  it('renders the kind tabs and the default offset-plane form', () => {
    const html = render();
    expect(html).toContain('Reference Geometry');
    for (const kind of ['Plane', 'Axis', 'Point', 'CSys']) {
      expect(html).toContain(kind);
    }
    // Offset plane is the default method: base picker + distance field.
    expect(html).toContain('Base plane');
    expect(html).toContain('Distance');
    expect(html).toContain('Create');
  });

  it('lists reference planes as base-plane options alongside the named planes', () => {
    const refGeom = [
      { id: 1, name: 'Plane1', kind: 'plane', entity: { kind: 'plane' } },
      { id: 2, name: 'Axis1', kind: 'axis', entity: { kind: 'axis' } },
    ];
    const html = render({ refGeom });
    // Named planes are always offered; the reference plane joins them.
    expect(html).toContain('>XY<');
    expect(html).toContain('>Plane1<');
    // A reference axis is not a plane option.
    expect(html).not.toContain('>Axis1<');
  });

  it('surfaces a build error from a degenerate spec', () => {
    const html = render({ error: 'mid-plane needs two parallel planes' });
    expect(html).toContain('ref-error');
    expect(html).toContain('mid-plane needs two parallel planes');
  });
});
