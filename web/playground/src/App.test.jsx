/**
 * Server-render smoke test: mounts the whole app render path (no effects, so
 * WASM init never runs) to catch reference errors — e.g. a callback used in
 * a hook dependency list before its declaration.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import App from './App.jsx';

describe('App', () => {
  it('server-renders without reference errors', () => {
    const html = renderToString(<App />);
    expect(html).toContain('OpenSolid Playground');
    expect(html).toContain('Sketch');
    expect(html).toContain('Loading WASM');
  });
});
