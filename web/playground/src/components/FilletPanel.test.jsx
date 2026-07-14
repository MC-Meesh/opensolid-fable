/**
 * Server-render smoke tests (matching SweepPanel.test.jsx): catch reference
 * errors and broken JSX in the render path without a browser. The edge-pick and
 * tree-rewrite math is unit-tested in lib/edgePick.test.js / lib/fillet.test.js.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import FilletPanel from './FilletPanel.jsx';

const noop = () => {};

describe('FilletPanel', () => {
  it('renders nothing without a pending fillet', () => {
    expect(
      renderToString(
        <FilletPanel fillet={null} error={null} onChange={noop} onField={noop} onApply={noop} onCancel={noop} />
      )
    ).toBe('');
  });

  it('prompts for an edge and disables Apply before one is picked', () => {
    const html = renderToString(
      <FilletPanel
        fillet={{ mode: 'fillet', radius: 0.1, edge: null }}
        error={null}
        onChange={noop}
        onField={noop}
        onApply={noop}
        onCancel={noop}
      />
    );
    expect(html).toContain('Fillet');
    expect(html).toContain('Chamfer');
    expect(html).toContain('Click a feature edge to blend');
    expect(html).toContain('Radius');
    expect(html).toMatch(/<button[^>]*disabled[^>]*>Apply<\/button>/);
  });

  it('shows the selected edge, enables Apply, and reflects chamfer mode', () => {
    const html = renderToString(
      <FilletPanel
        fillet={{ mode: 'chamfer', radius: 0.2, edge: [0, 0, 0, 1, 0, 0], segments: 3 }}
        error={null}
        onChange={noop}
        onField={noop}
        onApply={noop}
        onCancel={noop}
      />
    );
    expect(html).toMatch(/aria-checked="true"[^>]*>Chamfer/);
    expect(html).toContain('Setback'); // chamfer relabels the radius field
    expect(html).toContain('Edge selected (3 segments)');
    expect(html).not.toMatch(/<button[^>]*disabled[^>]*>Apply<\/button>/);
  });

  it('surfaces an error and disables Apply', () => {
    const html = renderToString(
      <FilletPanel
        fillet={{ mode: 'fillet', radius: 0.2, edge: [0, 0, 0, 1, 0, 0], segments: 1 }}
        error="radius exceeds local feature size"
        onChange={noop}
        onField={noop}
        onApply={noop}
        onCancel={noop}
      />
    );
    expect(html).toContain('radius exceeds local feature size');
    expect(html).toContain('Edge selected (1 segment)');
    expect(html).toMatch(/<button[^>]*disabled[^>]*>Apply<\/button>/);
  });
});
