/**
 * Server-render smoke tests (matching SweepPanel.test.jsx): catch reference
 * errors and broken JSX in the render path without a browser. The edge/tree
 * logic itself is unit-tested in lib/edgePick.test.js and lib/edgeFillet.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import FilletPanel from './FilletPanel.jsx';

const noop = () => {};

function render(fillet, error = null) {
  return renderToString(
    <FilletPanel
      fillet={fillet}
      error={error}
      onMode={noop}
      onRadius={noop}
      onApply={noop}
      onCancel={noop}
    />
  );
}

describe('FilletPanel', () => {
  it('renders nothing without a pending fillet', () => {
    expect(render(null)).toBe('');
  });

  it('prompts for an edge while armed, with no radius field or Apply', () => {
    const html = render({ armed: true, mode: 'fillet' });
    expect(html).toContain('Click an edge');
    expect(html).toContain('Fillet');
    expect(html).toContain('Chamfer');
    expect(html).not.toContain('Radius');
    expect(html).not.toContain('>Apply<');
    expect(html).toContain('Cancel');
  });

  it('shows the radius control and Apply once an edge is picked', () => {
    const html = render({ armed: false, mode: 'fillet', radius: 0.2, range: 2 });
    expect(html).toContain('Radius');
    expect(html).toContain('Adjust the radius');
    expect(html).toContain('Apply');
  });

  it('labels the control Setback in chamfer mode and checks it', () => {
    const html = render({ armed: false, mode: 'chamfer', radius: 0.1, range: 2 });
    expect(html).toContain('Setback');
    expect(html).not.toContain('Radius');
    expect(html).toMatch(/aria-checked="true"[^>]*>Chamfer/);
  });

  it('surfaces an error and disables Apply', () => {
    const html = render({ armed: false, mode: 'fillet', radius: 0.2, range: 2 }, 'Meshing failed');
    expect(html).toContain('Meshing failed');
    expect(html).toMatch(/<button[^>]*disabled[^>]*>\s*Apply/);
  });
});
