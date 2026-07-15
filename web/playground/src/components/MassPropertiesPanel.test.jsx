/**
 * Server-render smoke tests: catch reference errors and broken JSX in the
 * render path without a browser, and pin the reported values and units the
 * panel puts on screen. The mass math itself is unit-tested in
 * lib/massProps.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import MassPropertiesPanel from './MassPropertiesPanel.jsx';
import { massProperties } from '../lib/massProps.js';

const noop = () => {};

/** A unit-density 10-unit cube, as `measure()` reports it. */
const CUBE_I = (1000 * 10 ** 2) / 6;
const cubeMeasure = {
  volume: 1000,
  surfaceArea: 600,
  centroid: [0, 0, 0],
  inertia: [
    [CUBE_I, 0, 0],
    [0, CUBE_I, 0],
    [0, 0, CUBE_I],
  ],
  boundingBox: { min: [-5, -5, -5], max: [5, 5, 5], size: [10, 10, 10] },
  triangles: 12,
  vertices: 8,
  exact: true,
};

function render(report, { material = 'aluminium-6061', density = '2700', unit = 'mm' } = {}) {
  return renderToString(
    <MassPropertiesPanel
      report={report}
      material={material}
      density={density}
      unit={unit}
      onMaterialChange={noop}
      onDensityChange={noop}
      onClose={noop}
    />
  );
}

const cubeReport = (unit = 'mm', density = 2700) =>
  massProperties({ measure: cubeMeasure, density, unit });

describe('MassPropertiesPanel', () => {
  it('renders nothing when closed', () => {
    expect(render(null)).toBe('');
  });

  it('reports the mass of a 10 mm aluminium cube in grams', () => {
    const html = render(cubeReport());
    expect(html).toContain('2.7 g');
  });

  it('labels geometry in the document unit', () => {
    const html = render(cubeReport('in'), { unit: 'in' });
    expect(html).toContain('in³');
    expect(html).toContain('in²');
    expect(html).not.toContain('mm³');
  });

  it('shows volume, surface area, centre of mass, and the inertia tensor', () => {
    const html = render(cubeReport());
    expect(html).toContain('Mass');
    expect(html).toContain('Volume');
    expect(html).toContain('1000');
    expect(html).toContain('Surface area');
    expect(html).toContain('600');
    expect(html).toContain('Center of mass');
    expect(html).toContain('Moments of inertia');
  });

  it('keeps mass in SI while geometry stays in document units', () => {
    // A kilogram is a kilogram; only the geometry carries the document unit.
    const html = render(cubeReport('in'), { unit: 'in' });
    expect(html).toContain('kg/m³');
  });

  it('offers the material list and shows the current density', () => {
    const html = render(cubeReport(), { material: 'titanium', density: '4510' });
    expect(html).toContain('Titanium');
    expect(html).toContain('Steel AISI 1020');
    expect(html).toContain('value="4510"');
  });

  it('notes when the measurement is exact', () => {
    expect(render(cubeReport())).toContain('Exact');
    const approx = massProperties({
      measure: { ...cubeMeasure, exact: false },
      density: 2700,
      unit: 'mm',
    });
    expect(render(approx)).toContain('Approximate');
  });

  it('shows the kernel error instead of values when the shape has no volume', () => {
    const open = massProperties({
      measure: {
        ...cubeMeasure,
        volume: null,
        centroid: null,
        inertia: null,
        massError: 'mesh is not a closed, consistently oriented manifold',
      },
      density: 2700,
      unit: 'mm',
    });
    const html = render(open);
    expect(html).toContain('manifold');
    expect(html).not.toContain('Moments of inertia');
  });

  it('keeps the material controls usable when the readout errors', () => {
    // The density field must stay editable so a bad density can be corrected.
    const bad = massProperties({ measure: cubeMeasure, density: 0, unit: 'mm' });
    const html = render(bad, { density: '0' });
    expect(html).toContain('positive');
    expect(html).toContain('kg/m³');
    expect(html).toContain('Titanium');
  });

  it('renders a close control', () => {
    expect(render(cubeReport())).toContain('aria-label="Close mass properties"');
  });
});
