/**
 * Server-render smoke tests, matching MainToolbar.test.jsx: catch reference
 * errors and check the exact-booleans toggle renders with the right state.
 * (Meshing accuracy is fixed — deliberately no slider to test.)
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import MeshSettings from './MeshSettings.jsx';

const noop = () => {};

function render(overrides = {}) {
  return renderToString(
    <MeshSettings
      exactBooleans={false}
      onExactBooleansChange={noop}
      disabled={false}
      {...overrides}
    />
  );
}

describe('MeshSettings', () => {
  it('renders the exact-booleans toggle (and no accuracy slider)', () => {
    const html = render();
    expect(html).toContain('Exact booleans');
    expect(html).toMatch(/type="checkbox"(?![^>]*checked)/);
    expect(html).not.toContain('type="range"');
    expect(html).not.toContain('Accuracy');
  });

  it('checks the exact-booleans box when the mode is on', () => {
    const html = render({ exactBooleans: true });
    expect(html).toMatch(/type="checkbox"[^>]*checked/);
  });

  it('explains the exact pipeline in the toggle tooltip', () => {
    const html = render();
    expect(html).toMatch(/title="[^"]*exact B-Rep pipeline/);
  });

  it('disables the toggle before WASM is ready', () => {
    const html = render({ disabled: true });
    const disabledCount = (html.match(/disabled/g) ?? []).length;
    // Just the exact-booleans checkbox.
    expect(disabledCount).toBe(1);
  });
});
