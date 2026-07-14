/**
 * Server-render smoke tests for the Measure readout (of-fsl.17): catch
 * reference errors and confirm each entity-count state renders the right
 * numbers (single entity, pair, and the always-on body dimensions).
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import MeasurePanel from './MeasurePanel.jsx';

const noop = () => {};
const bbox = { size: [2, 3, 4], diagonal: Math.sqrt(29), center: [0, 0, 0] };

function render(readout) {
  return renderToString(<MeasurePanel readout={readout} onClear={noop} onClose={noop} />);
}

describe('MeasurePanel', () => {
  it('prompts to pick an entity when nothing is selected', () => {
    const html = render({ bbox, single: null, pair: null, count: 0 });
    expect(html).toContain('Click a vertex, edge, or face');
    expect(html).toContain('Body dimensions');
    expect(html).toContain('2, 3, 4');
  });

  it('reports a single edge length', () => {
    const html = render({
      bbox,
      single: { kind: 'edge', length: 5, from: [0, 0, 0], to: [5, 0, 0], closed: false },
      pair: null,
      count: 1,
    });
    expect(html).toContain('Length');
    expect(html).toContain('5');
    expect(html).toContain('Edge');
  });

  it('reports a circle radius and diameter', () => {
    const html = render({
      bbox,
      single: { kind: 'circle', radius: 3, diameter: 6, circumference: 18.8496, center: [0, 0, 0] },
      pair: null,
      count: 1,
    });
    expect(html).toContain('Radius');
    expect(html).toContain('Diameter');
    expect(html).toContain('6');
  });

  it('reports the distance and angle between two entities', () => {
    const html = render({
      bbox,
      single: null,
      pair: { distance: 10, delta: [10, 0, 0], angle: 90, planeDistance: undefined },
      count: 2,
    });
    expect(html).toContain('Distance');
    expect(html).toContain('Angle');
    expect(html).toContain('90°');
    expect(html).toContain('Clear selection');
  });
});
