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
      section={false}
      onSectionToggle={noop}
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
    for (const label of ['Extrude', 'Revolve', 'Fit', 'Front', 'Top', 'Right', 'Iso', 'Wireframe', 'Section', 'STL', 'STEP']) {
      expect(html).toContain(label);
    }
  });

  it('renders the Measure (Inspect) toggle with its shortcut tooltip', () => {
    const html = render();
    expect(html).toContain('Inspect');
    expect(html).toContain('Measure');
    expect(html).toMatch(/title="[^"]*\(M\)"/);
  });

  it('marks the Measure button active while measuring', () => {
    const html = render({ measureOpen: true });
    // aria-pressed=true renders on the active toggle.
    expect(html).toMatch(/aria-label="Measure"[^>]*aria-pressed="true"|aria-pressed="true"[^>]*aria-label="Measure"/);
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
    expect(html).toMatch(/<button[^>]*disabled[^>]*aria-label="Extrude"/);
  });

  it('enables sweeps when a closed profile exists', () => {
    const html = render({ sketchOpen: true, canSweep: true });
    expect(html).not.toMatch(/<button[^>]*disabled[^>]*aria-label="Extrude"/);
    expect(html).toContain('Exit Sketch');
  });

  it('marks active toggles (sketch open, wireframe on, section on)', () => {
    const html = render({ sketchOpen: true, wireframe: true, section: true });
    const activeCount = (html.match(/main-tool active/g) ?? []).length;
    expect(activeCount).toBe(3);
  });

  it('disables everything with a loading tooltip before WASM is ready', () => {
    const html = render({ disabled: true });
    expect(html).toContain('Still loading the WASM kernel');
  });

  it('renders an Edit group with Undo/Redo, disabled when history is empty', () => {
    const html = render();
    expect(html).toContain('>Edit<');
    expect(html).toMatch(/<button[^>]*disabled[^>]*aria-label="Undo"/);
    expect(html).toMatch(/<button[^>]*disabled[^>]*aria-label="Redo"/);
    expect(html).toContain('Nothing to undo');
    expect(html).toContain('Nothing to redo');
  });

  it('enables Undo/Redo and surfaces the step depth in the tooltip', () => {
    const html = render({ canUndo: true, undoDepth: 3, canRedo: true, redoDepth: 1 });
    expect(html).not.toMatch(/<button[^>]*disabled[^>]*aria-label="Undo"/);
    expect(html).not.toMatch(/<button[^>]*disabled[^>]*aria-label="Redo"/);
    expect(html).toContain('Undo (Ctrl+Z) — 3 steps');
    expect(html).toContain('Redo (Ctrl+Shift+Z) — 1 step');
  });
});
