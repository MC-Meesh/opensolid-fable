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
    for (const label of [
      'Select',
      'Line',
      'Rect',
      'Circle',
      'Arc',
      'Ellipse',
      'Spline',
      'Polygon',
      'Slot',
      'Centerline',
      'Trim',
      'Extend',
      'Dimension',
      'Mirror',
      'Offset',
      'Convert',
      'Extrude',
      'Revolve',
    ]) {
      expect(html).toContain(label);
    }
    for (const plane of ['XY', 'XZ', 'YZ']) {
      expect(html).toContain(plane);
    }
    // Empty sketch reports an open profile.
    expect(html).toContain('empty sketch');
  });

  it('enables Convert only when opened on a face with boundary loops', () => {
    const withoutFace = renderToString(
      <SketchCanvas open plane="XY" onPlaneChange={() => {}} onProfileChange={() => {}} />
    );
    // No face loops → the Convert button is disabled.
    expect(withoutFace).toMatch(/Convert<\/button>/);
    expect(withoutFace).toContain('disabled');
    const withFace = renderToString(
      <SketchCanvas
        open
        plane="XY"
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
        faceLoops={[[[0, 0], [1, 0], [1, 1], [0, 1]]]}
      />
    );
    expect(withFace).toContain('Convert');
  });

  it('renders sketch-mode controls: Finish, undo/redo, and the tool chip', () => {
    const html = renderToString(
      <SketchCanvas
        open
        plane="XY"
        onPlaneChange={() => {}}
        onProfileChange={() => {}}
      />
    );
    expect(html).toContain('Finish');
    expect(html).toContain('Undo (Cmd/Ctrl+Z)');
    expect(html).toContain('Redo (Shift+Cmd/Ctrl+Z)');
    // The active tool is always visible in the status bar.
    expect(html).toContain('tool-chip');
    expect(html).toContain('sketch-hint');
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
