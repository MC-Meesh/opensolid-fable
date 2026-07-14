// Drawing-mode overlay (of-fsl.26.2, DRAWINGS.md §7).
//
// A full-canvas SVG overlay that sits over the 3D viewport — parallel to
// SketchCanvas — showing a 2D orthographic drawing of the current body. It
// projects the mesh into the standard views (project.js), lays them out on a
// sheet (sheet.js), and draws the placed line-work with the same pan/zoom math
// the sketch overlay uses (sketchView.js). MVP: visible edges only (no HLR),
// no dimensions, no export — those land in of-fsl.26.3.
//
// The pan/zoom view `{ cx, cy, scale }` (sheet center + px per sheet unit) is
// owned by the parent, mirroring how App owns the sketch view; this overlay
// pans/zooms it and self-fits the sheet when first opened.

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { createSheet, DEFAULT_VIEWS, fitView } from '../lib/drawing/sheet.js';
import { sketchWorldToScreen } from '../lib/sketchView.js';

const MIN_SCALE = 0.2;
const MAX_SCALE = 20000;
const DEFAULT_VIEW = { cx: 0, cy: 0, scale: 60 };

// Human labels for the view chips, in the standard reading order.
const VIEW_LABELS = {
  front: 'Front',
  top: 'Top',
  right: 'Right',
  iso: 'Iso',
};

export default function DrawingCanvas({ open, mesh, view, onViewChange, onExit }) {
  const svgRef = useRef(null);
  const [size, setSize] = useState({ w: 0, h: 0 });
  const [scale, setScale] = useState(1); // sheet units per model unit
  const [angle, setAngle] = useState('third');
  const [activeViews, setActiveViews] = useState(DEFAULT_VIEWS);

  const v = view ?? DEFAULT_VIEW;
  const viewRef = useRef(v);
  viewRef.current = v;
  const onViewChangeRef = useRef(onViewChange);
  onViewChangeRef.current = onViewChange;

  const sheet = useMemo(
    () => createSheet(mesh, { views: activeViews, scale, angle }),
    [mesh, activeViews, scale, angle]
  );

  // ---- resize --------------------------------------------------------------
  useEffect(() => {
    const el = svgRef.current;
    if (!el) return undefined;
    const observer = new ResizeObserver(() => {
      setSize({ w: el.clientWidth, h: el.clientHeight });
    });
    observer.observe(el);
    setSize({ w: el.clientWidth, h: el.clientHeight });
    return () => observer.disconnect();
  }, []);

  // Self-fit: frame the sheet whenever the overlay opens (once we know the
  // viewport size and have geometry). A ref guards against re-fitting on every
  // pan while open.
  const fittedRef = useRef(false);
  useEffect(() => {
    if (!open) {
      fittedRef.current = false;
      return;
    }
    if (fittedRef.current || size.w === 0 || !sheet.bounds) return;
    fittedRef.current = true;
    onViewChangeRef.current?.(fitView(sheet.bounds, size));
  }, [open, size, sheet]);

  const fitToSheet = useCallback(() => {
    onViewChangeRef.current?.(fitView(sheet.bounds, size));
  }, [sheet, size]);

  // ---- wheel zoom (cursor-anchored) ---------------------------------------
  useEffect(() => {
    const el = svgRef.current;
    if (!el) return undefined;
    const onWheel = (event) => {
      event.preventDefault();
      const rect = el.getBoundingClientRect();
      const sx = event.clientX - rect.left;
      const sy = event.clientY - rect.top;
      const cur = viewRef.current;
      const factor = Math.exp(-event.deltaY * 0.0015);
      const scl = Math.min(MAX_SCALE, Math.max(MIN_SCALE, cur.scale * factor));
      const wx = (sx - el.clientWidth / 2) / cur.scale + cur.cx;
      const wy = (el.clientHeight / 2 - sy) / cur.scale + cur.cy;
      onViewChangeRef.current?.({
        scale: scl,
        cx: wx - (sx - el.clientWidth / 2) / scl,
        cy: wy - (el.clientHeight / 2 - sy) / scl,
      });
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  // ---- pan (drag anywhere) -------------------------------------------------
  const dragRef = useRef(null);
  const onPointerDown = useCallback((event) => {
    if (event.button !== 0 && event.button !== 1) return;
    dragRef.current = {
      startX: event.clientX,
      startY: event.clientY,
      view0: viewRef.current,
    };
    event.target.setPointerCapture?.(event.pointerId);
    event.preventDefault();
  }, []);

  const onPointerMove = useCallback((event) => {
    const drag = dragRef.current;
    if (!drag) return;
    const dx = (event.clientX - drag.startX) / drag.view0.scale;
    const dy = (event.clientY - drag.startY) / drag.view0.scale;
    onViewChangeRef.current?.({
      ...drag.view0,
      cx: drag.view0.cx - dx,
      cy: drag.view0.cy + dy,
    });
  }, []);

  const onPointerUp = useCallback((event) => {
    if (dragRef.current) {
      event.target.releasePointerCapture?.(event.pointerId);
      dragRef.current = null;
    }
  }, []);

  const toggleView = useCallback((name) => {
    setActiveViews((prev) =>
      prev.includes(name)
        ? prev.filter((n) => n !== name)
        : DEFAULT_VIEWS.filter((n) => prev.includes(n) || n === name)
    );
  }, []);

  // Keyboard: Esc leaves drawing mode.
  useEffect(() => {
    if (!open) return undefined;
    const onKey = (event) => {
      if (event.key === 'Escape') onExit?.();
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [open, onExit]);

  const toScreen = useCallback(
    (x, y) => sketchWorldToScreen(v, size, x, y),
    [v, size]
  );

  const empty = sheet.views.length === 0;

  return (
    <div className={`drawing-overlay${open ? '' : ' hidden'}`}>
      <svg
        ref={svgRef}
        className="drawing-svg"
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onContextMenu={(event) => event.preventDefault()}
      >
        {sheet.views.map((placed) => {
          const b = placed.bounds;
          const [lx, ly] = toScreen(b.minX, b.minY); // bottom-left in world → screen
          return (
            <g key={placed.view} className="drawing-view">
              {placed.segments.map((seg, i) => {
                const pts = seg.pts
                  .map(([x, y]) => {
                    const [sx, sy] = toScreen(x, y);
                    return `${sx},${sy}`;
                  })
                  .join(' ');
                return (
                  <polyline
                    key={i}
                    className={`edge ${seg.style}`}
                    points={pts}
                    fill="none"
                  />
                );
              })}
              <text className="view-label" x={lx} y={ly + 16}>
                {VIEW_LABELS[placed.view] ?? placed.view}
              </text>
            </g>
          );
        })}
      </svg>

      {empty && (
        <div className="drawing-empty">
          Nothing to draw — model a body, then open Drawing.
        </div>
      )}

      <div className="drawing-toolbar">
        <div className="group">
          <span className="group-label">Views</span>
          {DEFAULT_VIEWS.map((name) => (
            <button
              key={name}
              className={`tool-btn${activeViews.includes(name) ? ' active' : ''}`}
              onClick={() => toggleView(name)}
              title={`Toggle the ${VIEW_LABELS[name]} view`}
            >
              {VIEW_LABELS[name]}
            </button>
          ))}
        </div>
        <div className="group">
          <span className="group-label">Angle</span>
          <button
            className="tool-btn"
            onClick={() => setAngle((a) => (a === 'third' ? 'first' : 'third'))}
            title="Toggle first-angle / third-angle projection layout"
          >
            {angle === 'third' ? 'Third-angle' : 'First-angle'}
          </button>
        </div>
        <div className="group">
          <span className="group-label">Scale</span>
          <button className="tool-btn" onClick={() => setScale((s) => s / 2)} title="Halve the drawing scale">
            −
          </button>
          <span className="scale-value">{scale}:1</span>
          <button className="tool-btn" onClick={() => setScale((s) => s * 2)} title="Double the drawing scale">
            +
          </button>
        </div>
        <div className="group">
          <button className="tool-btn" onClick={fitToSheet} title="Fit the sheet in the view (F)">
            Fit
          </button>
          <button className="tool-btn primary" onClick={onExit} title="Leave drawing mode (Esc)">
            Finish
          </button>
        </div>
      </div>

      <div className="drawing-status">
        <span className="tool-chip">Drawing</span>
        <span className="drawing-hint">
          Drag to pan · scroll to zoom · visible edges only (HLR &amp; dimensions coming)
        </span>
      </div>
    </div>
  );
}
