/**
 * Server-render smoke tests (matching MeshSettings/MainToolbar): catch
 * reference errors and confirm the readout formats each measurement kind.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import MeasurePanel from './MeasurePanel.jsx';

const noop = () => {};
const bbox = { min: [0, 0, 0], max: [2, 3, 4], size: [2, 3, 4], diagonal: Math.hypot(2, 3, 4) };

function render(overrides = {}) {
  return renderToString(
    <MeasurePanel
      active
      bbox={bbox}
      entities={[]}
      single={null}
      pair={null}
      onClear={noop}
      {...overrides}
    />
  );
}

describe('MeasurePanel', () => {
  it('renders nothing when inactive', () => {
    expect(render({ active: false })).toBe('');
  });

  it('always shows the body bounding-box size', () => {
    const html = render();
    expect(html).toContain('Body size');
    expect(html).toContain('2 × 3 × 4');
  });

  it('prompts for a pick when nothing is selected', () => {
    expect(render()).toContain('Click a vertex');
  });

  it('formats a single vertex coordinate', () => {
    const html = render({
      entities: [{ kind: 'vertex', point: [1, 2, 3] }],
      single: { kind: 'vertex', coord: [1, 2, 3] },
    });
    expect(html).toContain('Coordinate');
    expect(html).toContain('(1, 2, 3)');
  });

  it('formats a circle radius and diameter', () => {
    const html = render({
      entities: [{ kind: 'circle', point: [2, 0, 0], center: [0, 0, 0], radius: 2, normal: [0, 0, 1] }],
      single: { kind: 'circle', radius: 2, diameter: 4, center: [0, 0, 0] },
    });
    expect(html).toContain('Radius');
    expect(html).toContain('Diameter');
  });

  it('formats a pair distance, delta and angle', () => {
    const html = render({
      entities: [
        { kind: 'face', point: [0, 0, 0] },
        { kind: 'face', point: [0, 0, 5] },
      ],
      pair: { distance: 5, delta: [0, 0, 5], angle: 0, gap: 5 },
    });
    expect(html).toContain('Distance');
    expect(html).toContain('Parallel gap');
    expect(html).toContain('0.0°');
  });

  it('shows a Clear button once an entity is picked', () => {
    const html = render({ entities: [{ kind: 'vertex', point: [0, 0, 0] }], single: { kind: 'vertex', coord: [0, 0, 0] } });
    expect(html).toContain('Clear');
  });
});
