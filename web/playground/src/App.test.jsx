/**
 * Server-render smoke tests: mount the whole app render path (no effects, so
 * WASM init never runs) to catch reference errors — e.g. a callback used in
 * a hook dependency list before its declaration — and to pin the loading /
 * failed UI states without needing a real pkg/.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import App from './App.jsx';
import { WasmProvider } from './wasm/WasmContext.jsx';

function fakeLoader(state) {
  return {
    getState: () => state,
    subscribe: () => () => {},
    ensure: () => Promise.resolve(state.api),
    retry: () => Promise.resolve(state.api),
  };
}

describe('App', () => {
  it('server-renders without reference errors and shows the loading state', () => {
    const loader = fakeLoader({ status: 'idle', error: null, api: null });
    const html = renderToString(
      <WasmProvider loader={loader}>
        <App />
      </WasmProvider>
    );
    expect(html).toContain('OpenSolid Playground');
    expect(html).toContain('Sketch');
    expect(html).toContain('Features'); // toolbar workflow group
    expect(html).toContain('Loading WASM');
  });

  it('renders the tabbed side panel with Code and Tree panes', () => {
    const loader = fakeLoader({ status: 'idle', error: null, api: null });
    const html = renderToString(
      <WasmProvider loader={loader}>
        <App />
      </WasmProvider>
    );
    expect(html).toContain('sidebar-tabs');
    expect(html).toContain('>Code<');
    expect(html).toContain('>Tree<');
    // Both panes stay mounted so editor/tree state survives tab switches;
    // the inactive one is CSS-hidden.
    expect(html).toContain('sidebar-pane hidden');
    expect(html).toContain('role="separator"'); // draggable splitter
    // The stacked-panels layout is gone.
    expect(html).not.toContain('scene-panel');
    expect(html).not.toContain('class="left"');
  });

  it('renders the actionable error screen (not a spinner) when WASM init failed', () => {
    const loader = fakeLoader({
      status: 'failed',
      error: 'pkg/opensolid_wasm.js does not exist.\nnpm run wasm',
      api: null,
    });
    const html = renderToString(
      <WasmProvider loader={loader}>
        <App />
      </WasmProvider>
    );
    expect(html).toContain('WASM engine failed to load');
    expect(html).toContain('npm run wasm');
    expect(html).toContain('Retry');
    expect(html).not.toContain('Loading WASM');
  });

  it('renders no overlay once WASM is ready', () => {
    const loader = fakeLoader({
      status: 'ready',
      error: null,
      api: { WasmShape: class {}, WasmProfile2D: class {} },
    });
    const html = renderToString(
      <WasmProvider loader={loader}>
        <App />
      </WasmProvider>
    );
    expect(html).not.toContain('Loading WASM');
    expect(html).not.toContain('WASM engine failed to load');
  });
});
