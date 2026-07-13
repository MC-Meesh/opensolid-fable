/**
 * Server-render smoke tests, matching MainToolbar.test.jsx: catch reference
 * errors and check the meshing controls (including the exact-booleans
 * toggle) render with the right state.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import Toolbar from './Toolbar.jsx';

const noop = () => {};

function render(overrides = {}) {
  return renderToString(
    <Toolbar
      exactBooleans={false}
      onExactBooleansChange={noop}
      onRun={noop}
      onDownloadStl={noop}
      onDownloadStep={noop}
      disabled={false}
      {...overrides}
    />
  );
}

describe('Toolbar', () => {
  it('renders run, STL, STEP, and the exact-booleans toggle', () => {
    const html = render();
    expect(html).toContain('Run');
    expect(html).toContain('Download STL');
    expect(html).toContain('Download STEP');
    expect(html).toContain('Exact booleans');
    expect(html).toMatch(/type="checkbox"(?![^>]*checked)/);
  });

  it('has no accuracy slider: meshing precision is a fixed default', () => {
    const html = render();
    expect(html).not.toContain('Accuracy');
    expect(html).not.toContain('type="range"');
  });

  it('explains faceted export of organic shapes in the STEP tooltip', () => {
    const html = render();
    expect(html).toMatch(/title="[^"]*organic shapes export as faceted geometry/);
  });

  it('checks the exact-booleans box when the mode is on', () => {
    const html = render({ exactBooleans: true });
    expect(html).toMatch(/type="checkbox"[^>]*checked/);
  });

  it('explains the exact pipeline in the toggle tooltip', () => {
    const html = render();
    expect(html).toMatch(/title="[^"]*exact B-Rep pipeline/);
  });

  it('disables all controls before WASM is ready', () => {
    const html = render({ disabled: true });
    const disabledCount = (html.match(/disabled/g) ?? []).length;
    // Run, STL, STEP, exact-booleans checkbox.
    expect(disabledCount).toBe(4);
  });
});
