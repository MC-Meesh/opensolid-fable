/**
 * Server-render smoke tests: catch reference errors and broken JSX in the
 * render path without a browser. Projection/layout logic lives in
 * lib/drawing/ and is unit-tested there.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import DrawingCanvas from './DrawingCanvas.jsx';

// Unit cube [0,1]^3 — a body that draws in every view.
const CUBE = {
  positions: new Float32Array([
    0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0,
    0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1,
  ]),
  indices: new Uint32Array([
    0, 3, 2, 0, 2, 1, 4, 5, 6, 4, 6, 7,
    0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2,
    0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5,
  ]),
  key: 1,
};

describe('DrawingCanvas', () => {
  it('renders the overlay with view chips, angle, scale, and status', () => {
    const html = renderToString(
      <DrawingCanvas open mesh={CUBE} onViewChange={() => {}} onExit={() => {}} />
    );
    expect(html).toContain('drawing-overlay');
    expect(html).toContain('drawing-toolbar');
    for (const label of ['Front', 'Top', 'Right', 'Iso']) {
      expect(html).toContain(label);
    }
    expect(html).toContain('Third-angle');
    expect(html).toContain('Finish');
    expect(html).toContain('tool-chip');
    // The cube draws segments in the sheet.
    expect(html).toContain('polyline');
  });

  it('renders hidden when closed', () => {
    const html = renderToString(
      <DrawingCanvas open={false} mesh={CUBE} onViewChange={() => {}} onExit={() => {}} />
    );
    expect(html).toContain('hidden');
  });

  it('shows an empty-state hint when there is no geometry', () => {
    const empty = { positions: new Float32Array(0), indices: new Uint32Array(0), key: 2 };
    const html = renderToString(
      <DrawingCanvas open mesh={empty} onViewChange={() => {}} onExit={() => {}} />
    );
    expect(html).toContain('Nothing to draw');
  });

  it('renders without a mesh (null) without throwing', () => {
    const html = renderToString(
      <DrawingCanvas open mesh={null} onViewChange={() => {}} onExit={() => {}} />
    );
    expect(html).toContain('drawing-overlay');
    expect(html).toContain('Nothing to draw');
  });
});
