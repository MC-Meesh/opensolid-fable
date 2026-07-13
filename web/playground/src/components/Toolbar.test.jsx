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
      accuracy={0.01}
      onAccuracyChange={noop}
      onAccuracyCommit={noop}
      exactBooleans={false}
      onExactBooleansChange={noop}
      onRun={noop}
      onDownloadStl={noop}
      disabled={false}
      {...overrides}
    />
  );
}

describe('Toolbar', () => {
  it('renders run, STL, accuracy, and the exact-booleans toggle', () => {
    const html = render();
    expect(html).toContain('Run');
    expect(html).toContain('Download STL');
    expect(html).toContain('Accuracy');
    expect(html).toContain('Exact booleans');
    expect(html).toMatch(/type="checkbox"(?![^>]*checked)/);
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
    // Run, STL, accuracy slider, exact-booleans checkbox.
    expect(disabledCount).toBe(4);
  });
});
