/**
 * Server-render smoke tests, matching SectionPanel.test.jsx: catch reference
 * errors and broken JSX in the render path without a browser. The geometry
 * construction and form dispatch are unit-tested in lib/referenceGeometry.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import ReferencePanel from './ReferencePanel.jsx';

const noop = () => {};

function render(props = {}) {
  return renderToString(
    <ReferencePanel open refGeom={[]} onAdd={noop} onClose={noop} {...props} />
  );
}

describe('ReferencePanel', () => {
  it('renders nothing when closed', () => {
    expect(render({ open: false })).toBe('');
  });

  it('renders the type buttons and the default offset-plane method', () => {
    const html = render();
    expect(html).toContain('Reference Geometry');
    for (const label of ['>Plane<', '>Axis<', '>Point<', '>Coord System<']) {
      expect(html).toContain(label);
    }
    // Default kind is plane → offset method fields shown.
    expect(html).toContain('Base plane');
    expect(html).toContain('Distance');
  });

  it('lists existing reference planes as base options', () => {
    const refGeom = [
      { id: 'r1', kind: 'plane', name: 'Top datum', geom: { normal: [0, 0, 1], origin: [0, 0, 5] } },
      { id: 'r2', kind: 'axis', geom: {} }, // axes are not offered as base planes
    ];
    const html = render({ refGeom });
    expect(html).toContain('Top datum');
    // The three named planes are always present.
    expect(html).toContain('>XY<');
    expect(html).toContain('>YZ<');
  });

  it('marks the active kind as pressed exactly once', () => {
    const html = render();
    expect((html.match(/aria-pressed="true"/g) ?? []).length).toBe(1);
  });
});
