/**
 * Server-render smoke tests: catch reference errors and broken JSX in the
 * render path without a browser. Interaction logic lives in lib/sketch/ and
 * is unit-tested there.
 */
import { describe, expect, it } from 'vitest';
import { renderToString } from 'react-dom/server';
import SketchCanvas from './SketchCanvas.jsx';

describe('SketchCanvas', () => {
  it('renders the overlay with toolbar, plane picker, and status', () => {
    const html = renderToString(
      <SketchCanvas
        open
        plane="XY"
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
      />
    );
    expect(html).toContain('sketch-overlay');
    expect(html).toContain('sketch-toolbar');
    for (const label of ['Select', 'Line', 'Rect', 'Circle', 'Arc']) {
      expect(html).toContain(label);
    }
    for (const plane of ['XY', 'XZ', 'YZ']) {
      expect(html).toContain(plane);
    }
    // Empty sketch reports an open profile.
    expect(html).toContain('empty sketch');
  });

  it('renders hidden when closed', () => {
    const html = renderToString(
      <SketchCanvas
        open={false}
        plane="XZ"
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
      />
    );
    expect(html).toContain('hidden');
  });
});
