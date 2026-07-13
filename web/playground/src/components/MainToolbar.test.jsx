/**
 * Server-render smoke tests, matching SweepPanel.test.jsx: catch reference
 * errors and check the workflow grouping / disabled-reason tooltips render.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import MainToolbar from './MainToolbar.jsx';

const noop = () => {};

function render(overrides = {}) {
  return renderToString(
    <MainToolbar
      disabled={false}
      sketchOpen={false}
      onSketchToggle={noop}
      canSweep={false}
      sweepDisabledReason="Open a sketch and draw a closed profile first"
      onSweep={noop}
      onView={noop}
      onFit={noop}
      wireframe={false}
      onWireframeChange={noop}
      onDownloadStl={noop}
      onDownloadStep={noop}
      exactBooleans={false}
      onExactBooleansChange={noop}
      {...overrides}
    />
  );
}

describe('MainToolbar', () => {
  it('renders the four workflow groups with labeled buttons', () => {
    const html = render();
    expect(html).toContain('Sketch');
    expect(html).toContain('Features');
    expect(html).toContain('View');
    expect(html).toContain('Export');
    for (const label of ['Extrude', 'Revolve', 'Fit', 'Front', 'Top', 'Right', 'Iso', 'Wireframe', 'STL', 'STEP']) {
      expect(html).toContain(label);
    }
  });

  it('explains faceted export of organic shapes in the STEP tooltip', () => {
    const html = render();
    expect(html).toMatch(/title="[^"]*organic shapes export as faceted geometry/);
  });

  it('keeps the meshing settings in the overflow menu', () => {
    const html = render();
    expect(html).toContain('tool-menu');
    expect(html).toContain('Exact booleans');
  });

  it('disables sweep buttons with a tooltip explaining why', () => {
    const html = render();
    expect(html).toMatch(/<span class="tool-wrap" title="Open a sketch and draw a closed profile first">/);
    expect(html).toMatch(/disabled[^>]*>.*Extrude/);
  });

  it('enables sweeps when a closed profile exists', () => {
    const html = render({ sketchOpen: true, canSweep: true });
    expect(html).not.toMatch(/disabled[^>]*>.*Extrude/);
    expect(html).toContain('Exit Sketch');
  });

  it('marks active toggles (sketch open, wireframe on)', () => {
    const html = render({ sketchOpen: true, wireframe: true });
    const activeCount = (html.match(/main-tool active/g) ?? []).length;
    expect(activeCount).toBe(2);
  });

  it('disables everything with a loading tooltip before WASM is ready', () => {
    const html = render({ disabled: true });
    expect(html).toContain('Still loading the WASM kernel');
  });
});
