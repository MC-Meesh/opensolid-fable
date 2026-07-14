import {
  forwardRef,
  useCallback,
  useEffect,
  useImperativeHandle,
  useMemo,
  useRef,
  useState,
} from 'react';
import {
  addArc,
  addCircle,
  addConstraint,
  addLine,
  addLoop,
  addPoint,
  addPolygon,
  addRectangle,
  addSlot,
  createSketch,
  deleteConstraint,
  deleteEntity,
  deletePoint,
  entityPointIds,
  entityRadius,
  mirrorEntities,
  translatePoints,
  validateConstraint,
} from '../lib/sketch/model.js';
import {
  extendEntityAt,
  offsetEntities,
  trimEntityAt,
} from '../lib/sketch/edit.js';
import { solve } from '../lib/sketch/solver.js';
import {
  axisAlign,
  distToEntity,
  hitTest,
  nearestPoint,
  snapToGrid,
} from '../lib/sketch/snap.js';
import { arcSweep, normalizeAngle } from '../lib/sketch/geom.js';
import {
  extractProfile,
  isFacePlane,
  planeAxisLabels,
  planeLabel,
  profileTo3D,
  segmentEnd2D,
  segmentStart2D,
} from '../lib/sketch/profile.js';
import {
  canRedo,
  canUndo,
  createHistory,
  record,
  redoTo,
  snapshot,
  undoTo,
} from '../lib/sketch/history.js';
import {
  formatAngle,
  formatNumber,
  parseDimension,
} from '../lib/sketch/format.js';
import {
  sketchScreenToWorld,
  sketchWorldToScreen,
} from '../lib/sketchView.js';
import { sketchFromOps } from '../lib/sketch/fromOps.js';
import { opsBounds } from '../lib/sweep.js';
import { DEFAULT_LENGTH_UNIT, withUnit } from '../lib/units.js';

const SNAP_PX = 10;
const HIT_PX = 8;
const DRAG_PX = 4; // click vs drag threshold for press-drag tools
const MIN_SCALE = 2;
const MAX_SCALE = 5000;
// Placeholder until Viewport3D reports the camera-derived view.
const DEFAULT_VIEW = { cx: 0, cy: 0, scale: 60 };

const TOOLS = [
  {
    id: 'select',
    label: 'Select',
    key: 'V',
    hint: 'Drag points or segments to adjust · click a dimension to edit · Del deletes',
  },
  {
    id: 'line',
    label: 'Line',
    key: 'L',
    hint: 'Click to place points · type a number for exact length · click the first point to close · Esc ends',
  },
  {
    id: 'rect',
    label: 'Rect',
    key: 'R',
    hint: 'Drag (or click two corners) · type width, Tab, height, Enter for exact size',
  },
  {
    id: 'circle',
    label: 'Circle',
    key: 'C',
    hint: 'Drag from center (or click center, then edge) · type a number for exact radius',
  },
  {
    id: 'arc',
    label: 'Arc',
    key: 'A',
    hint: 'Click center, start, then end (drag direction sets the sweep)',
  },
  {
    id: 'polygon',
    label: 'Polygon',
    key: 'G',
    hint: 'Drag from center to a vertex · type a number to set the side count',
  },
  {
    id: 'slot',
    label: 'Slot',
    key: 'S',
    hint: 'Click both centerline ends, then drag to set the width',
  },
  {
    id: 'centerline',
    label: 'Centerline',
    key: 'N',
    hint: 'Construction line (excluded from the profile) · click two points · Esc ends',
  },
  {
    id: 'trim',
    label: 'Trim',
    key: 'T',
    hint: 'Click a segment to trim it back to its nearest intersections',
  },
  {
    id: 'extend',
    label: 'Extend',
    key: 'X',
    hint: 'Click near an open end to extend it to the nearest intersection',
  },
  { id: 'pan', label: 'Pan', key: 'P', hint: 'Drag to pan (middle-drag works with any tool)' },
];

const DEFAULT_POLYGON_SIDES = 6;
const MIN_POLYGON_SIDES = 3;

const PLANES = ['XY', 'XZ', 'YZ'];

/** Decade grid: minor step sized to stay >= 8 screen px. */
function gridSteps(scale) {
  const minor = 10 ** Math.ceil(Math.log10(8 / scale));
  return { minor, major: minor * 10 };
}

function wrapToPi(a) {
  while (a > Math.PI) a -= 2 * Math.PI;
  while (a < -Math.PI) a += 2 * Math.PI;
  return a;
}

function arcScreenGeometry(sketch, entity) {
  const c = sketch.points[entity.center];
  const p1 = sketch.points[entity.p1];
  const p2 = sketch.points[entity.p2];
  const r = entityRadius(sketch, entity);
  const start = normalizeAngle(Math.atan2(p1.y - c.y, p1.x - c.x));
  const end = normalizeAngle(Math.atan2(p2.y - c.y, p2.x - c.x));
  const sweep = arcSweep(start, end, entity.ccw);
  return { c, p1, p2, r, start, end, sweep };
}

/** Id of the nearest entity outline within `tol` of (x, y), or null. */
function nearestEntityAt(sketch, x, y, tol) {
  let best = null;
  for (const e of Object.values(sketch.entities)) {
    const d = distToEntity(sketch, e, x, y);
    if (d <= tol && (!best || d < best.d)) best = { id: e.id, d };
  }
  return best ? best.id : null;
}

/** A representative world point for an entity: line midpoint, or curve center. */
function entityAnchor(sketch, id) {
  const e = sketch.entities[id];
  if (!e) return null;
  if (e.type === 'line') {
    const a = sketch.points[e.p1];
    const b = sketch.points[e.p2];
    return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
  }
  return sketch.points[e.center];
}

/**
 * 2D sketch canvas: an SVG overlay in world units (v up), with drawing
 * tools, direct manipulation, dimension entry, undo/redo, and live
 * closed-profile extraction.
 *
 * The pan/zoom view `{ cx, cy, scale }` is owned by the parent (`view` /
 * `onViewChange`): it is initialized from the 3D sketch camera and every
 * change is applied back to that camera, so the overlay's px-per-world-unit
 * mapping is exactly the camera's world-to-screen transform — what you draw
 * over the rendered model is what extrudes (of-4eh.14).
 *
 * Interaction model (Onshape-style):
 *  - Line: click-click chaining with rubber band; clicking the chain's first
 *    point closes the loop. Esc ends the chain.
 *  - Rect/Circle: press-drag-release, or click-move-click — both work.
 *  - While drafting, typing digits opens an exact-dimension entry (Enter
 *    commits, Tab switches width/height for rectangles).
 *  - Select: drag points or whole segments; drag a circle's outline to change
 *    its radius; click a dimension label to edit its value.
 *  - Cmd/Ctrl+Z undo, Shift+Cmd/Ctrl+Z or Ctrl+Y redo.
 *  - Esc walks back one level at a time: entry → draft → selection → select
 *    tool → exit sketch mode.
 *
 * The sketch itself lives in a ref and is mutated in place; `rev` bumps
 * trigger re-render. Keep the component mounted (hidden via CSS) so the
 * sketch survives toggling the overlay.
 */
export default forwardRef(function SketchCanvas(
  {
    open,
    plane,
    view: viewProp,
    onViewChange,
    onPlaneChange,
    onProfileChange,
    onSweep,
    onExit,
    // Feature-tree sketch editing: `{ name }` of the feature being edited.
    // The sweep buttons become a single "Apply" that fires onApplyEdit, and
    // the plane is locked (the original plane is baked into the tree's
    // orientation wrappers, so changing it here would lie).
    editing = null,
    onApplyEdit,
    documentUnit = DEFAULT_LENGTH_UNIT,
    // Boundary loops (sketch u,v) of the face this sketch opened on, if any,
    // for "convert entities". Each loop is an array of [u, v] points.
    faceLoops = null,
  },
  ref
) {
  const sketchRef = useRef(createSketch());
  const historyRef = useRef(createHistory());
  const svgRef = useRef(null);
  const [rev, setRev] = useState(0);
  const [size, setSize] = useState({ w: 0, h: 0 });
  const view = viewProp ?? DEFAULT_VIEW;
  const viewRef = useRef(view);
  viewRef.current = view;
  const onViewChangeRef = useRef(onViewChange);
  onViewChangeRef.current = onViewChange;
  const [tool, setTool] = useState('line');
  const [selection, setSelection] = useState([]);
  const [draft, setDraft] = useState(null);
  const draftRef = useRef(null);
  draftRef.current = draft;
  const [cursor, setCursor] = useState(null); // world pos + snap info
  const [gridOn, setGridOn] = useState(true);
  const [snapOn, setSnapOn] = useState(true);
  const [dimValue, setDimValue] = useState('');
  const [dimEntry, setDimEntry] = useState(null); // typed exact dimension while drafting
  const dimEntryRef = useRef(null);
  dimEntryRef.current = dimEntry;
  const [dimEdit, setDimEdit] = useState(null); // { id, text, wx, wy } editing a dimension
  const [message, setMessage] = useState(null);
  const [polygonSides, setPolygonSides] = useState(DEFAULT_POLYGON_SIDES);
  const polygonSidesRef = useRef(DEFAULT_POLYGON_SIDES);
  polygonSidesRef.current = polygonSides;
  const [offsetDist, setOffsetDist] = useState('');
  const [offsetFlip, setOffsetFlip] = useState(false);
  const dragRef = useRef(null);

  const sketch = sketchRef.current;
  const touch = useCallback(() => setRev((r) => r + 1), []);

  const runSolve = useCallback((pinned) => {
    const result = solve(sketchRef.current, { pinned });
    setMessage(
      result.converged ? null : 'Constraints conflict: solver did not converge'
    );
    return result;
  }, []);

  /** Record a committed mutation for undo. `before` from takeBefore(). */
  const takeBefore = useCallback(() => snapshot(sketchRef.current), []);
  const commitRecord = useCallback(
    (before) => {
      record(historyRef.current, before);
      touch();
    },
    [touch]
  );

  const resetTransient = useCallback(() => {
    setSelection([]);
    setDraft(null);
    setDimEntry(null);
    setDimEdit(null);
    setMessage(null);
    dragRef.current = null;
  }, []);

  const doUndo = useCallback(() => {
    const prev = undoTo(historyRef.current, sketchRef.current);
    if (!prev) return;
    sketchRef.current = prev;
    resetTransient();
    touch();
  }, [resetTransient, touch]);

  const doRedo = useCallback(() => {
    const next = redoTo(historyRef.current, sketchRef.current);
    if (!next) return;
    sketchRef.current = next;
    resetTransient();
    touch();
  }, [resetTransient, touch]);

  // Entering sketch mode: draw-first default for an empty sketch.
  useEffect(() => {
    if (!open) return;
    const empty = Object.keys(sketchRef.current.entities).length === 0;
    setTool(empty ? 'line' : 'select');
    setMessage(null);
  }, [open]);

  // ---- coordinate transforms ----------------------------------------------

  const worldToScreen = useCallback(
    (x, y) => sketchWorldToScreen(view, size, x, y),
    [view, size]
  );

  const screenToWorld = useCallback(
    (sx, sy) => sketchScreenToWorld(view, size, sx, sy),
    [view, size]
  );

  const eventWorld = useCallback(
    (event) => {
      const rect = svgRef.current.getBoundingClientRect();
      return screenToWorld(event.clientX - rect.left, event.clientY - rect.top);
    },
    [screenToWorld]
  );

  // ---- resize / wheel ------------------------------------------------------

  useEffect(() => {
    const el = svgRef.current;
    const observer = new ResizeObserver(() => {
      setSize({ w: el.clientWidth, h: el.clientHeight });
    });
    observer.observe(el);
    return () => observer.disconnect();
  }, []);

  useEffect(() => {
    const el = svgRef.current;
    const onWheel = (event) => {
      event.preventDefault();
      const rect = el.getBoundingClientRect();
      const sx = event.clientX - rect.left;
      const sy = event.clientY - rect.top;
      const v = viewRef.current;
      const factor = Math.exp(-event.deltaY * 0.0015);
      const scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, v.scale * factor));
      // Keep the world point under the cursor stationary.
      const wx = (sx - el.clientWidth / 2) / v.scale + v.cx;
      const wy = (el.clientHeight / 2 - sy) / v.scale + v.cy;
      onViewChangeRef.current?.({
        scale,
        cx: wx - (sx - el.clientWidth / 2) / scale,
        cy: wy - (el.clientHeight / 2 - sy) / scale,
      });
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  // ---- snapping ------------------------------------------------------------

  const { minor: gridMinor, major: gridMajor } = gridSteps(view.scale);

  /**
   * Resolve a click/hover into a snapped location: existing point first,
   * then axis alignment against `axisFrom`, then the grid.
   */
  const resolveSnap = useCallback(
    (world, { axisFrom = null, exclude } = {}) => {
      const snapR = SNAP_PX / view.scale;
      const near = snapOn
        ? nearestPoint(sketchRef.current, world.x, world.y, snapR, exclude)
        : null;
      if (near) return { id: near.id, x: near.x, y: near.y, axis: null };
      let { x, y } = world;
      let axis = null;
      if (axisFrom) {
        ({ x, y, axis } = axisAlign(axisFrom.x, axisFrom.y, x, y));
      }
      if (snapOn && gridOn) {
        const g = snapToGrid(x, y, gridMinor);
        if (axis === 'h') x = g.x;
        else if (axis === 'v') y = g.y;
        else ({ x, y } = g);
      }
      return { id: null, x, y, axis };
    },
    [view.scale, snapOn, gridOn, gridMinor]
  );

  // ---- profile -------------------------------------------------------------

  const profile = useMemo(
    () => extractProfile(sketchRef.current, plane),
    // eslint-disable-next-line react-hooks/exhaustive-deps
    [rev, plane]
  );

  useEffect(() => {
    onProfileChange?.(profile.closed ? profileTo3D(profile) : profile);
  }, [profile, onProfileChange]);

  // ---- draft helpers -------------------------------------------------------

  /** Live position of a draft anchor: snapped point id or fixed coords. */
  const anchorPos = useCallback((anchor) => {
    if (!anchor) return null;
    if (anchor.id) {
      const p = sketchRef.current.points[anchor.id];
      if (p) return { x: p.x, y: p.y };
    }
    return { x: anchor.x, y: anchor.y };
  }, []);

  const cancelDraft = useCallback(() => {
    setDraft(null);
    setDimEntry(null);
  }, []);

  /** Reuse a snapped existing point or create one. */
  const materialize = useCallback((s, anchor) => {
    if (anchor.id && s.points[anchor.id]) return anchor.id;
    return addPoint(s, anchor.x, anchor.y);
  }, []);

  // ---- draw commits --------------------------------------------------------

  /**
   * Commit one line segment from the draft anchor to `snap`. Optionally
   * dimension it (`lengthValue`). Ends the chain when the loop closes.
   */
  const commitLineSegment = useCallback(
    (snap, { lengthValue = null } = {}) => {
      const d = draftRef.current;
      const s = sketchRef.current;
      const startPos = anchorPos(d.start);
      if (snap.id && snap.id === d.start.id) {
        // Clicked the pending point again: finish the chain.
        cancelDraft();
        return;
      }
      if (!snap.id && Math.hypot(snap.x - startPos.x, snap.y - startPos.y) < 1e-9) {
        return;
      }
      const construction = Boolean(d.construction);
      const before = takeBefore();
      const p1 = materialize(s, d.start);
      const closing = Boolean(snap.id) && snap.id === d.chainStartId;
      const p2 = materialize(s, snap);
      if (p1 === p2) return;
      const lineId = addLine(s, p1, p2, { construction });
      if (snap.axis === 'h') {
        addConstraint(s, { type: 'horizontal', line: lineId });
      } else if (snap.axis === 'v') {
        addConstraint(s, { type: 'vertical', line: lineId });
      }
      if (lengthValue) {
        addConstraint(s, { type: 'length', line: lineId, value: lengthValue });
      }
      runSolve();
      commitRecord(before);
      setDimEntry(null);
      if (closing) {
        setDraft(null);
      } else {
        setDraft({
          kind: 'line',
          construction,
          start: { id: p2, x: s.points[p2].x, y: s.points[p2].y },
          chainStartId: d.chainStartId ?? p1,
        });
      }
    },
    [anchorPos, cancelDraft, materialize, takeBefore, runSolve, commitRecord]
  );

  /**
   * Commit a rectangle from the draft corner to (x2, y2). With exact `dims`,
   * width/height dimension constraints are added.
   */
  const commitRect = useCallback(
    (x2, y2, { dims = null } = {}) => {
      const d = draftRef.current;
      const s = sketchRef.current;
      if (Math.abs(x2 - d.x1) < 1e-9 || Math.abs(y2 - d.y1) < 1e-9) return false;
      const before = takeBefore();
      const [bottom, right] = addRectangle(s, d.x1, d.y1, x2, y2);
      if (dims) {
        addConstraint(s, { type: 'length', line: bottom, value: dims.w });
        addConstraint(s, { type: 'length', line: right, value: dims.h });
      }
      runSolve();
      commitRecord(before);
      setDraft(null);
      setDimEntry(null);
      return true;
    },
    [takeBefore, runSolve, commitRecord]
  );

  /** Commit a circle at the draft center. Exact radius adds a dimension. */
  const commitCircle = useCallback(
    (radius, { exact = false } = {}) => {
      const d = draftRef.current;
      const s = sketchRef.current;
      if (!(radius > 1e-9)) return false;
      const before = takeBefore();
      const center = materialize(s, d.center);
      const circleId = addCircle(s, center, radius);
      if (exact) {
        addConstraint(s, { type: 'radius', entity: circleId, value: radius });
      }
      runSolve();
      commitRecord(before);
      setDraft(null);
      setDimEntry(null);
      return true;
    },
    [materialize, takeBefore, runSolve, commitRecord]
  );

  /** Commit the arc draft: end point lands on the circle at `angle`. */
  const commitArc = useCallback(
    (angle) => {
      const d = draftRef.current;
      const s = sketchRef.current;
      const c = anchorPos(d.center);
      const start = anchorPos(d.start);
      const r = Math.hypot(start.x - c.x, start.y - c.y);
      const ex = c.x + r * Math.cos(angle);
      const ey = c.y + r * Math.sin(angle);
      if (Math.hypot(ex - start.x, ey - start.y) < 1e-9) return;
      const before = takeBefore();
      const centerId = materialize(s, d.center);
      const startId = materialize(s, d.start);
      const endId = addPoint(s, ex, ey);
      const ccw = (d.accum ?? 0) >= 0;
      addArc(s, centerId, startId, endId, ccw);
      runSolve();
      commitRecord(before);
      setDraft(null);
    },
    [anchorPos, materialize, takeBefore, runSolve, commitRecord]
  );

  /**
   * Commit a regular polygon at the draft center, sized to the cursor (which
   * fixes both the circumradius and the first-vertex direction).
   */
  const commitPolygon = useCallback(
    (cursorX, cursorY) => {
      const d = draftRef.current;
      const s = sketchRef.current;
      const c = anchorPos(d.center);
      const radius = Math.hypot(cursorX - c.x, cursorY - c.y);
      if (!(radius > 1e-9)) return false;
      const rotation = Math.atan2(cursorY - c.y, cursorX - c.x);
      const before = takeBefore();
      addPolygon(s, c.x, c.y, radius, polygonSidesRef.current, rotation);
      runSolve();
      commitRecord(before);
      setDraft(null);
      setDimEntry(null);
      return true;
    },
    [anchorPos, takeBefore, runSolve, commitRecord]
  );

  /**
   * Commit a straight slot whose centerline runs draft.p1 → draft.p2, with
   * half-width set by the cursor's perpendicular distance from that line.
   */
  const commitSlot = useCallback(
    (cursorX, cursorY) => {
      const d = draftRef.current;
      const s = sketchRef.current;
      const { p1, p2 } = d;
      const dx = p2.x - p1.x;
      const dy = p2.y - p1.y;
      const len = Math.hypot(dx, dy);
      if (!(len > 1e-9)) return false;
      // Perpendicular distance from the cursor to the infinite centerline.
      const width = Math.abs((cursorX - p1.x) * -dy + (cursorY - p1.y) * dx) / len;
      if (!(width > 1e-9)) return false;
      const before = takeBefore();
      addSlot(s, p1.x, p1.y, p2.x, p2.y, width);
      runSolve();
      commitRecord(before);
      setDraft(null);
      return true;
    },
    [takeBefore, runSolve, commitRecord]
  );

  // ---- exact dimension entry (type a number while drafting) ---------------

  const commitDimEntry = useCallback(() => {
    const entry = dimEntryRef.current;
    const d = draftRef.current;
    if (!entry || !d) return;
    if (entry.kind === 'line') {
      const len = parseDimension(entry.text);
      if (!len) {
        setMessage('Enter a positive length');
        return;
      }
      const from = anchorPos(d.start);
      const at = cursor?.snap ?? cursor;
      let dx = (at?.x ?? from.x + 1) - from.x;
      let dy = (at?.y ?? from.y) - from.y;
      const dist = Math.hypot(dx, dy);
      if (dist < 1e-9) {
        dx = 1;
        dy = 0;
      } else {
        dx /= dist;
        dy /= dist;
      }
      commitLineSegment(
        {
          id: null,
          x: from.x + dx * len,
          y: from.y + dy * len,
          axis: cursor?.snap?.axis ?? null,
        },
        { lengthValue: len }
      );
      return;
    }
    if (entry.kind === 'circle') {
      const r = parseDimension(entry.text);
      if (!r) {
        setMessage('Enter a positive radius');
        return;
      }
      commitCircle(r, { exact: true });
      return;
    }
    if (entry.kind === 'rect') {
      const w = parseDimension(entry.w);
      const h = parseDimension(entry.h);
      if (w && !h) {
        setDimEntry({ ...entry, field: 'h' });
        return;
      }
      if (!w || !h) {
        setMessage('Enter positive width and height');
        return;
      }
      const at = cursor?.snap ?? cursor;
      const sx = at && at.x < d.x1 ? -1 : 1;
      const sy = at && at.y < d.y1 ? -1 : 1;
      commitRect(d.x1 + sx * w, d.y1 + sy * h, { dims: { w, h } });
    }
  }, [anchorPos, cursor, commitLineSegment, commitCircle, commitRect]);

  /** Route a printable/edit key into the dimension entry. Returns handled. */
  const dimEntryKey = useCallback(
    (event) => {
      const d = draftRef.current;
      if (!d) return false;
      const kind =
        d.kind === 'line' && d.start
          ? 'line'
          : d.kind === 'circle'
            ? 'circle'
            : d.kind === 'rect'
              ? 'rect'
              : d.kind === 'polygon'
                ? 'polygon'
                : null;
      if (!kind) return false;
      const entry = dimEntryRef.current;
      const key = event.key;
      const isDigit = /^[0-9.]$/.test(key);

      // Polygon: typed digits set the side count (radius comes from the drag).
      if (kind === 'polygon') {
        if (/^[0-9]$/.test(key)) {
          const cur = entry?.kind === 'polygon' ? entry.text : '';
          const text = (cur + key).slice(0, 2);
          setDimEntry({ kind: 'polygon', text });
          const n = parseInt(text, 10);
          if (n >= MIN_POLYGON_SIDES) setPolygonSides(n);
          return true;
        }
        if (key === 'Backspace' && entry?.kind === 'polygon') {
          const text = entry.text.slice(0, -1);
          setDimEntry({ kind: 'polygon', text });
          const n = parseInt(text, 10);
          setPolygonSides(n >= MIN_POLYGON_SIDES ? n : DEFAULT_POLYGON_SIDES);
          return true;
        }
        return false;
      }

      if (isDigit) {
        if (kind === 'rect') {
          const cur = entry?.kind === 'rect' ? entry : { kind: 'rect', w: '', h: '', field: 'w' };
          const field = cur.field;
          if (cur[field].length < 10) {
            setDimEntry({ ...cur, [field]: cur[field] + key });
          }
        } else {
          const cur = entry?.kind === kind ? entry : { kind, text: '' };
          if (cur.text.length < 10) setDimEntry({ ...cur, text: cur.text + key });
        }
        return true;
      }
      if (!entry) return false;
      if (key === 'Backspace') {
        if (entry.kind === 'rect') {
          const field = entry.field;
          setDimEntry({ ...entry, [field]: entry[field].slice(0, -1) });
        } else {
          setDimEntry({ ...entry, text: entry.text.slice(0, -1) });
        }
        return true;
      }
      if (key === 'Tab' && entry.kind === 'rect') {
        setDimEntry({ ...entry, field: entry.field === 'w' ? 'h' : 'w' });
        return true;
      }
      if (key === 'Enter') {
        commitDimEntry();
        return true;
      }
      return false;
    },
    [commitDimEntry]
  );

  // ---- tool click handling ---------------------------------------------

  const handleToolClick = useCallback(
    (world, event) => {
      const s = sketchRef.current;
      switch (tool) {
        case 'select': {
          const tol = HIT_PX / view.scale;
          const hit = hitTest(s, world.x, world.y, tol, tol);
          if (!hit) {
            if (!event.shiftKey) setSelection([]);
            return;
          }
          setSelection((sel) => {
            const present = sel.some(
              (item) => item.kind === hit.kind && item.id === hit.id
            );
            if (event.shiftKey) {
              return present
                ? sel.filter(
                    (item) => !(item.kind === hit.kind && item.id === hit.id)
                  )
                : [...sel, hit];
            }
            return [hit];
          });
          if (hit.kind === 'point') {
            dragRef.current = {
              mode: 'point',
              id: hit.id,
              before: takeBefore(),
              moved: false,
            };
            event.target.setPointerCapture?.(event.pointerId);
          } else if (hit.kind === 'entity') {
            const entity = s.entities[hit.id];
            if (entity.type === 'circle') {
              // Grabbing the outline adjusts the radius directly.
              dragRef.current = {
                mode: 'radius',
                id: hit.id,
                before: takeBefore(),
                moved: false,
              };
            } else {
              dragRef.current = {
                mode: 'entity',
                ids: [...new Set(entityPointIds(entity))],
                last: world,
                before: takeBefore(),
                moved: false,
              };
            }
            event.target.setPointerCapture?.(event.pointerId);
          }
          return;
        }
        case 'line': {
          if (!draft) {
            const snap = resolveSnap(world);
            setDraft({ kind: 'line', start: snap, chainStartId: snap.id ?? null });
            return;
          }
          const startPos = anchorPos(draft.start);
          const snap = resolveSnap(world, { axisFrom: startPos });
          commitLineSegment(snap);
          return;
        }
        case 'rect': {
          if (!draft) {
            const snap = resolveSnap(world);
            event.target.setPointerCapture?.(event.pointerId);
            setDraft({
              kind: 'rect',
              x1: snap.x,
              y1: snap.y,
              pressed: true,
              psx: event.clientX,
              psy: event.clientY,
            });
            return;
          }
          const snap = resolveSnap(world);
          commitRect(snap.x, snap.y);
          return;
        }
        case 'circle': {
          if (!draft) {
            const snap = resolveSnap(world);
            event.target.setPointerCapture?.(event.pointerId);
            setDraft({
              kind: 'circle',
              center: snap,
              pressed: true,
              psx: event.clientX,
              psy: event.clientY,
            });
            return;
          }
          const c = anchorPos(draft.center);
          const snap = resolveSnap(world, {
            exclude: draft.center.id ? new Set([draft.center.id]) : undefined,
          });
          commitCircle(Math.hypot(snap.x - c.x, snap.y - c.y));
          return;
        }
        case 'arc': {
          if (!draft) {
            const snap = resolveSnap(world);
            setDraft({ kind: 'arc', center: snap, stage: 1 });
            return;
          }
          if (draft.stage === 1) {
            const snap = resolveSnap(world);
            const c = anchorPos(draft.center);
            if (Math.hypot(snap.x - c.x, snap.y - c.y) < 1e-9) return;
            const angle = Math.atan2(world.y - c.y, world.x - c.x);
            setDraft({
              ...draft,
              start: snap,
              stage: 2,
              prevAngle: angle,
              accum: 0,
            });
            return;
          }
          const c = anchorPos(draft.center);
          commitArc(Math.atan2(world.y - c.y, world.x - c.x));
          return;
        }
        case 'centerline': {
          if (!draft) {
            const snap = resolveSnap(world);
            setDraft({
              kind: 'line',
              construction: true,
              start: snap,
              chainStartId: snap.id ?? null,
            });
            return;
          }
          const startPos = anchorPos(draft.start);
          const snap = resolveSnap(world, { axisFrom: startPos });
          commitLineSegment(snap);
          return;
        }
        case 'polygon': {
          if (!draft) {
            const snap = resolveSnap(world);
            event.target.setPointerCapture?.(event.pointerId);
            setDraft({
              kind: 'polygon',
              center: snap,
              pressed: true,
              psx: event.clientX,
              psy: event.clientY,
            });
            return;
          }
          const snap = resolveSnap(world, {
            exclude: draft.center.id ? new Set([draft.center.id]) : undefined,
          });
          commitPolygon(snap.x, snap.y);
          return;
        }
        case 'slot': {
          if (!draft) {
            const snap = resolveSnap(world);
            setDraft({ kind: 'slot', p1: { x: snap.x, y: snap.y }, stage: 1 });
            return;
          }
          if (draft.stage === 1) {
            const snap = resolveSnap(world, { axisFrom: draft.p1 });
            if (Math.hypot(snap.x - draft.p1.x, snap.y - draft.p1.y) < 1e-9) return;
            setDraft({ ...draft, p2: { x: snap.x, y: snap.y }, stage: 2 });
            return;
          }
          commitSlot(world.x, world.y);
          return;
        }
        case 'trim':
        case 'extend': {
          const tol = HIT_PX / view.scale;
          const hitId = nearestEntityAt(s, world.x, world.y, tol);
          if (!hitId) return;
          const before = takeBefore();
          const changed =
            tool === 'trim'
              ? trimEntityAt(s, hitId, world.x, world.y)
              : extendEntityAt(s, hitId, world.x, world.y);
          if (changed) {
            setSelection([]);
            runSolve();
            commitRecord(before);
          } else if (tool === 'extend') {
            setMessage('Nothing to extend to — no intersection beyond that end');
          }
          return;
        }
        default:
      }
    },
    [
      tool,
      draft,
      view.scale,
      resolveSnap,
      anchorPos,
      commitLineSegment,
      commitRect,
      commitCircle,
      commitArc,
      commitPolygon,
      commitSlot,
      takeBefore,
      runSolve,
      commitRecord,
    ]
  );

  // ---- pointer events --------------------------------------------------

  const onPointerDown = useCallback(
    (event) => {
      if (event.button === 1 || tool === 'pan') {
        dragRef.current = {
          mode: 'pan',
          startX: event.clientX,
          startY: event.clientY,
          view0: view,
        };
        event.target.setPointerCapture?.(event.pointerId);
        event.preventDefault();
        return;
      }
      if (event.button === 2) {
        cancelDraft();
        return;
      }
      if (event.button === 0) {
        if (dimEdit) {
          setDimEdit(null);
          return;
        }
        handleToolClick(eventWorld(event), event);
      }
    },
    [tool, view, dimEdit, cancelDraft, handleToolClick, eventWorld]
  );

  const onPointerMove = useCallback(
    (event) => {
      const drag = dragRef.current;
      if (drag?.mode === 'pan') {
        const dx = (event.clientX - drag.startX) / drag.view0.scale;
        const dy = (event.clientY - drag.startY) / drag.view0.scale;
        onViewChangeRef.current?.({
          ...drag.view0,
          cx: drag.view0.cx - dx,
          cy: drag.view0.cy + dy,
        });
        return;
      }
      const world = eventWorld(event);
      if (drag?.mode === 'point') {
        const s = sketchRef.current;
        const p = s.points[drag.id];
        if (p) {
          const snap = resolveSnap(world, { exclude: new Set([drag.id]) });
          p.x = snap.x;
          p.y = snap.y;
          drag.moved = true;
          runSolve(new Set([drag.id]));
          touch();
        }
        return;
      }
      if (drag?.mode === 'entity') {
        const s = sketchRef.current;
        const dx = world.x - drag.last.x;
        const dy = world.y - drag.last.y;
        if (dx !== 0 || dy !== 0) {
          translatePoints(s, drag.ids, dx, dy);
          drag.last = world;
          drag.moved = true;
          runSolve(new Set(drag.ids));
          touch();
        }
        return;
      }
      if (drag?.mode === 'radius') {
        const s = sketchRef.current;
        const entity = s.entities[drag.id];
        if (entity) {
          const c = s.points[entity.center];
          const r = Math.hypot(world.x - c.x, world.y - c.y);
          if (r > 1e-9) {
            entity.radius = r;
            drag.moved = true;
            runSolve(new Set([entity.center]));
            touch();
          }
        }
        return;
      }
      // Hover feedback for draw tools.
      let axisFrom = null;
      const d = draftRef.current;
      if (d?.kind === 'line' && d.start) {
        axisFrom = anchorPos(d.start);
      }
      const snap = resolveSnap(world, { axisFrom });
      if (d?.pressed) {
        const dist = Math.hypot(event.clientX - d.psx, event.clientY - d.psy);
        if (dist > DRAG_PX && !d.dragging) setDraft({ ...d, dragging: true });
      }
      if (d?.kind === 'arc' && d.stage === 2) {
        const c = anchorPos(d.center);
        const angle = Math.atan2(world.y - c.y, world.x - c.x);
        const delta = wrapToPi(angle - d.prevAngle);
        setDraft({ ...d, prevAngle: angle, accum: d.accum + delta });
      }
      setCursor({ x: world.x, y: world.y, snap });
    },
    [eventWorld, resolveSnap, anchorPos, runSolve, touch]
  );

  const onPointerUp = useCallback(
    (event) => {
      const drag = dragRef.current;
      if (drag) {
        event.target.releasePointerCapture?.(event.pointerId);
        if (drag.moved && drag.before) commitRecord(drag.before);
        dragRef.current = null;
        return;
      }
      // Press-drag commits for rect/circle.
      const d = draftRef.current;
      if (d?.pressed) {
        let committed = false;
        if (d.dragging) {
          const world = eventWorld(event);
          if (d.kind === 'rect') {
            const snap = resolveSnap(world);
            committed = commitRect(snap.x, snap.y);
          } else if (d.kind === 'circle') {
            const c = anchorPos(d.center);
            const snap = resolveSnap(world, {
              exclude: d.center.id ? new Set([d.center.id]) : undefined,
            });
            committed = commitCircle(Math.hypot(snap.x - c.x, snap.y - c.y));
          } else if (d.kind === 'polygon') {
            const snap = resolveSnap(world, {
              exclude: d.center.id ? new Set([d.center.id]) : undefined,
            });
            committed = commitPolygon(snap.x, snap.y);
          }
        }
        // A plain click (or a degenerate drag): stay in click-move-click mode.
        if (!committed) setDraft({ ...d, pressed: false, dragging: false });
      }
    },
    [
      eventWorld,
      resolveSnap,
      anchorPos,
      commitRect,
      commitCircle,
      commitPolygon,
      commitRecord,
    ]
  );

  // ---- selection-derived state / constraint actions ---------------------

  const selEntities = selection
    .filter((item) => item.kind === 'entity')
    .map((item) => sketch.entities[item.id])
    .filter(Boolean);
  const selLines = selEntities.filter((e) => e.type === 'line');
  const selCurves = selEntities.filter(
    (e) => e.type === 'circle' || e.type === 'arc'
  );
  const selPoints = selection
    .filter((item) => item.kind === 'point')
    .map((item) => sketch.points[item.id])
    .filter(Boolean);

  const applyConstraint = useCallback(
    (constraints) => {
      const s = sketchRef.current;
      for (const c of constraints) {
        const problem = validateConstraint(s, c);
        if (problem) {
          setMessage(`Cannot apply ${c.type}: ${problem}`);
          return;
        }
      }
      const before = takeBefore();
      for (const c of constraints) addConstraint(s, c);
      runSolve();
      commitRecord(before);
    },
    [takeBefore, runSolve, commitRecord]
  );

  /**
   * Mirror the selected entities across the single selected line (its axis),
   * adding reflected copies. The axis line itself is not duplicated.
   */
  const mirrorSelection = useCallback(() => {
    if (selLines.length !== 1) {
      setMessage('Select exactly one line as the mirror axis');
      return;
    }
    const axis = selLines[0];
    const targets = selEntities.filter((e) => e.id !== axis.id);
    if (targets.length === 0) {
      setMessage('Select geometry to mirror alongside the axis line');
      return;
    }
    const s = sketchRef.current;
    const a = s.points[axis.p1];
    const b = s.points[axis.p2];
    const before = takeBefore();
    const created = mirrorEntities(
      s,
      targets.map((e) => e.id),
      a.x,
      a.y,
      b.x,
      b.y
    );
    runSolve();
    commitRecord(before);
    setSelection(created.map((id) => ({ kind: 'entity', id })));
  }, [selLines, selEntities, takeBefore, runSolve, commitRecord]);

  /**
   * Offset the selected entities by the entered distance (flip reverses the
   * side). Connected chains join at their offset corners; the copies replace
   * the selection so they can be dimensioned or swept.
   */
  const offsetSelection = useCallback(() => {
    const dist = parseDimension(offsetDist);
    if (!dist) {
      setMessage('Enter a positive offset distance');
      return;
    }
    if (selEntities.length === 0) {
      setMessage('Select geometry to offset first');
      return;
    }
    const s = sketchRef.current;
    const before = takeBefore();
    const created = offsetEntities(
      s,
      selEntities.map((e) => e.id),
      offsetFlip ? -dist : dist
    );
    if (created.length === 0) {
      setMessage('Offset collapsed — try a smaller distance or flip the side');
      return;
    }
    runSolve();
    commitRecord(before);
    setSelection(created.map((id) => ({ kind: 'entity', id })));
  }, [offsetDist, offsetFlip, selEntities, takeBefore, runSolve, commitRecord]);

  /** Convert the sketch's face boundary loops into editable sketch lines. */
  const convertEntities = useCallback(() => {
    if (!faceLoops || faceLoops.length === 0) {
      setMessage('Open the sketch on a flat face to convert its edges');
      return;
    }
    const s = sketchRef.current;
    const before = takeBefore();
    const created = [];
    for (const loop of faceLoops) created.push(...addLoop(s, loop));
    if (created.length === 0) {
      setMessage('Face boundary produced no geometry');
      return;
    }
    runSolve();
    commitRecord(before);
    setSelection(created.map((id) => ({ kind: 'entity', id })));
  }, [faceLoops, takeBefore, runSolve, commitRecord]);

  const applyDimension = useCallback(() => {
    const value = parseDimension(dimValue);
    if (!value) {
      setMessage('Enter a positive dimension value');
      return;
    }
    const constraints = [
      ...selLines.map((e) => ({ type: 'length', line: e.id, value })),
      ...selCurves.map((e) => ({ type: 'radius', entity: e.id, value })),
    ];
    if (constraints.length === 0) {
      setMessage('Select a line (length) or circle/arc (radius) first');
      return;
    }
    applyConstraint(constraints);
  }, [dimValue, selLines, selCurves, applyConstraint]);

  const deleteSelection = useCallback(() => {
    if (selection.length === 0) return;
    const s = sketchRef.current;
    const before = takeBefore();
    for (const item of selection) {
      if (item.kind === 'entity') deleteEntity(s, item.id);
      else if (item.kind === 'point') deletePoint(s, item.id);
      else if (item.kind === 'constraint') deleteConstraint(s, item.id);
    }
    setSelection([]);
    runSolve();
    commitRecord(before);
  }, [selection, takeBefore, runSolve, commitRecord]);

  const clearSketch = useCallback(() => {
    if (Object.keys(sketchRef.current.points).length > 0) {
      const before = takeBefore();
      sketchRef.current = createSketch();
      commitRecord(before);
    }
    resetTransient();
    touch();
  }, [takeBefore, commitRecord, resetTransient, touch]);

  // Feature-tree "edit sketch": replace the working sketch with one rebuilt
  // from a sweep node's profile snapshot and frame the view on it. History
  // restarts — undo cannot cross back into the previous sketch.
  useImperativeHandle(
    ref,
    () => ({
      loadProfile(ops) {
        sketchRef.current = sketchFromOps(ops);
        historyRef.current = createHistory();
        resetTransient();
        setTool('select');
        const { min, max } = opsBounds(ops);
        const extent = Math.max(max[0] - min[0], max[1] - min[1]) || 1;
        onViewChangeRef.current?.({
          cx: (min[0] + max[0]) / 2,
          cy: (min[1] + max[1]) / 2,
          scale:
            Math.min(
              MAX_SCALE,
              Math.max(
                MIN_SCALE,
                (0.6 * Math.min(size.w || 800, size.h || 600)) / extent
              )
            ) || viewRef.current.scale,
        });
        touch();
      },
    }),
    [resetTransient, touch, size]
  );

  // ---- dimension editing (click a dimension label) ------------------------

  const openDimEdit = useCallback((constraint, wx, wy) => {
    setDimEdit({
      id: constraint.id,
      text: String(constraint.value),
      wx,
      wy,
    });
  }, []);

  const commitDimEdit = useCallback(() => {
    const edit = dimEdit;
    if (!edit) return;
    const s = sketchRef.current;
    const constraint = s.constraints[edit.id];
    const value = parseDimension(edit.text);
    if (!constraint || !value) {
      setMessage('Enter a positive dimension value');
      return;
    }
    const before = takeBefore();
    constraint.value = value;
    runSolve();
    commitRecord(before);
    setDimEdit(null);
  }, [dimEdit, takeBefore, runSolve, commitRecord]);

  // ---- keyboard ----------------------------------------------------------

  const escStep = useCallback(() => {
    if (dimEntryRef.current) {
      setDimEntry(null);
    } else if (dimEdit) {
      setDimEdit(null);
    } else if (draftRef.current) {
      cancelDraft();
    } else if (selection.length > 0) {
      setSelection([]);
    } else if (tool !== 'select') {
      setTool('select');
    } else {
      onExit?.();
    }
  }, [dimEdit, selection, tool, cancelDraft, onExit]);

  useEffect(() => {
    if (!open) return undefined;
    const onKey = (event) => {
      if (event.target.tagName === 'INPUT') return;
      const mod = event.metaKey || event.ctrlKey;

      if (mod && (event.key === 'z' || event.key === 'Z')) {
        event.preventDefault();
        if (event.shiftKey) doRedo();
        else doUndo();
        return;
      }
      if (mod && (event.key === 'y' || event.key === 'Y')) {
        event.preventDefault();
        doRedo();
        return;
      }
      if (mod) return;

      if (event.key === 'Escape') {
        escStep();
        return;
      }

      // Exact-dimension entry while drafting eats digits and edit keys.
      if (dimEntryKey(event)) {
        event.preventDefault();
        return;
      }

      if (event.key === 'Delete' || event.key === 'Backspace') {
        deleteSelection();
        return;
      }
      const shortcut = TOOLS.find(
        (t) => t.key.toLowerCase() === event.key.toLowerCase()
      );
      if (shortcut && !draftRef.current) {
        setTool(shortcut.id);
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [open, escStep, dimEntryKey, deleteSelection, doUndo, doRedo]);

  // Leaving a tool cancels its draft.
  const selectTool = useCallback(
    (id) => {
      cancelDraft();
      setTool(id);
    },
    [cancelDraft]
  );

  // ---- rendering helpers --------------------------------------------------

  const isSelected = useCallback(
    (kind, id) => selection.some((s) => s.kind === kind && s.id === id),
    [selection]
  );

  function renderGrid() {
    if (!gridOn || size.w === 0) return null;
    const lines = [];
    const halfW = size.w / 2 / view.scale;
    const halfH = size.h / 2 / view.scale;
    const x0 = view.cx - halfW;
    const x1 = view.cx + halfW;
    const y0 = view.cy - halfH;
    const y1 = view.cy + halfH;
    for (const [step, cls] of [
      [gridMinor, 'grid-minor'],
      [gridMajor, 'grid-major'],
    ]) {
      for (let x = Math.ceil(x0 / step) * step; x <= x1; x += step) {
        const [sx] = worldToScreen(x, 0);
        lines.push(
          <line key={`${cls}x${x}`} className={cls} x1={sx} y1={0} x2={sx} y2={size.h} />
        );
      }
      for (let y = Math.ceil(y0 / step) * step; y <= y1; y += step) {
        const [, sy] = worldToScreen(0, y);
        lines.push(
          <line key={`${cls}y${y}`} className={cls} x1={0} y1={sy} x2={size.w} y2={sy} />
        );
      }
    }
    // Axes through the origin.
    const [ox, oy] = worldToScreen(0, 0);
    const [uName, vName] = planeAxisLabels(plane);
    lines.push(
      <line key="axis-u" className="axis-u" x1={0} y1={oy} x2={size.w} y2={oy} />,
      <line key="axis-v" className="axis-v" x1={ox} y1={0} x2={ox} y2={size.h} />,
      <text key="axis-u-label" className="axis-label axis-u-label" x={size.w - 14} y={oy - 6}>
        {uName}
      </text>,
      <text key="axis-v-label" className="axis-label axis-v-label" x={ox + 6} y={14}>
        {vName}
      </text>
    );
    return lines;
  }

  /** Filled highlight for the detected closed profile. */
  function renderProfileFill() {
    if (!profile.closed || size.w === 0) return null;
    const segs = profile.segments;
    const [mx, my] = worldToScreen(...segmentStart2D(segs[0]));
    let d = `M ${mx} ${my}`;
    for (const seg of segs) {
      const [ex, ey] = worldToScreen(...segmentEnd2D(seg));
      if (seg.kind === 'line') {
        d += ` L ${ex} ${ey}`;
      } else {
        const rr = seg.radius * view.scale;
        const sweep = arcSweep(
          normalizeAngle(seg.startAngle),
          normalizeAngle(seg.endAngle),
          seg.ccw
        );
        const largeArc = sweep > Math.PI ? 1 : 0;
        // World CCW renders as SVG sweep-flag 0 because screen Y is flipped.
        const sweepFlag = seg.ccw ? 0 : 1;
        d += ` A ${rr} ${rr} 0 ${largeArc} ${sweepFlag} ${ex} ${ey}`;
      }
    }
    return <path className="profile-fill" d={`${d} Z`} />;
  }

  function renderEntity(entity) {
    const cls = `entity${entity.construction ? ' construction' : ''}${
      isSelected('entity', entity.id) ? ' selected' : ''
    }`;
    if (entity.type === 'line') {
      const a = sketch.points[entity.p1];
      const b = sketch.points[entity.p2];
      const [x1, y1] = worldToScreen(a.x, a.y);
      const [x2, y2] = worldToScreen(b.x, b.y);
      return <line key={entity.id} className={cls} x1={x1} y1={y1} x2={x2} y2={y2} />;
    }
    if (entity.type === 'circle') {
      const c = sketch.points[entity.center];
      const [cx, cy] = worldToScreen(c.x, c.y);
      return (
        <circle
          key={entity.id}
          className={cls}
          cx={cx}
          cy={cy}
          r={entity.radius * view.scale}
          fill="none"
        />
      );
    }
    // Arc: world CCW renders as SVG sweep-flag 0 because screen Y is flipped.
    const { p1, p2, r, sweep } = arcScreenGeometry(sketch, entity);
    const [sx, sy] = worldToScreen(p1.x, p1.y);
    const [ex, ey] = worldToScreen(p2.x, p2.y);
    const rr = r * view.scale;
    const largeArc = sweep > Math.PI ? 1 : 0;
    const sweepFlag = entity.ccw ? 0 : 1;
    return (
      <path
        key={entity.id}
        className={cls}
        d={`M ${sx} ${sy} A ${rr} ${rr} 0 ${largeArc} ${sweepFlag} ${ex} ${ey}`}
        fill="none"
      />
    );
  }

  function renderConstraintGlyph(constraint) {
    const s = sketch;
    const editable = constraint.type === 'length' || constraint.type === 'radius';
    const cls = `glyph${editable ? ' dim' : ''}${
      isSelected('constraint', constraint.id) ? ' selected' : ''
    }`;
    const select = (event) => {
      event.stopPropagation();
      setSelection([{ kind: 'constraint', id: constraint.id }]);
    };
    const textAt = (wx, wy, label, onDown = select, key = constraint.id) => {
      const [x, y] = worldToScreen(wx, wy);
      return (
        <text key={key} className={cls} x={x + 6} y={y - 6} onPointerDown={onDown}>
          {label}
        </text>
      );
    };
    switch (constraint.type) {
      case 'horizontal':
      case 'vertical': {
        const line = s.entities[constraint.line];
        if (!line) return null;
        const a = s.points[line.p1];
        const b = s.points[line.p2];
        return textAt(
          (a.x + b.x) / 2,
          (a.y + b.y) / 2,
          constraint.type === 'horizontal' ? 'H' : 'V'
        );
      }
      case 'length': {
        const line = s.entities[constraint.line];
        if (!line) return null;
        const a = s.points[line.p1];
        const b = s.points[line.p2];
        const mx = (a.x + b.x) / 2;
        const my = (a.y + b.y) / 2;
        return textAt(mx, my, withUnit(formatNumber(constraint.value), documentUnit), (event) => {
          event.stopPropagation();
          openDimEdit(constraint, mx, my);
        });
      }
      case 'radius': {
        const entity = s.entities[constraint.entity];
        if (!entity) return null;
        const c = s.points[entity.center];
        const r = entityRadius(s, entity);
        const d = Math.SQRT1_2;
        const wx = c.x + r * d;
        const wy = c.y + r * d;
        return textAt(wx, wy, withUnit(`R${formatNumber(constraint.value)}`, documentUnit), (event) => {
          event.stopPropagation();
          openDimEdit(constraint, wx, wy);
        });
      }
      case 'coincident': {
        const p = s.points[constraint.a];
        if (!p) return null;
        const [x, y] = worldToScreen(p.x, p.y);
        return (
          <circle
            key={constraint.id}
            className={cls}
            cx={x}
            cy={y}
            r={7}
            fill="none"
            onPointerDown={select}
          />
        );
      }
      case 'tangent': {
        const line = s.entities[constraint.line];
        const curve = s.entities[constraint.curve];
        if (!line || !curve) return null;
        const a = s.points[line.p1];
        const b = s.points[line.p2];
        const c = s.points[curve.center];
        const dx = b.x - a.x;
        const dy = b.y - a.y;
        const lenSq = dx * dx + dy * dy || 1;
        const t = ((c.x - a.x) * dx + (c.y - a.y) * dy) / lenSq;
        return textAt(a.x + dx * t, a.y + dy * t, 'T');
      }
      case 'parallel':
      case 'perpendicular':
      case 'collinear':
      case 'equal': {
        // Place the glyph at the midpoint of the first referenced entity.
        const label = {
          parallel: '∥',
          perpendicular: '⊥',
          collinear: 'C',
          equal: '=',
        }[constraint.type];
        const anchor = entityAnchor(s, constraint.a);
        if (!anchor) return null;
        return textAt(anchor.x, anchor.y, label);
      }
      case 'concentric': {
        const ea = s.entities[constraint.a];
        if (!ea) return null;
        const c = s.points[ea.center];
        const [x, y] = worldToScreen(c.x, c.y);
        return (
          <circle
            key={constraint.id}
            className={cls}
            cx={x}
            cy={y}
            r={9}
            fill="none"
            onPointerDown={select}
          />
        );
      }
      case 'midpoint': {
        const p = s.points[constraint.point];
        if (!p) return null;
        return textAt(p.x, p.y, 'M');
      }
      case 'symmetric': {
        const a = s.points[constraint.a];
        const b = s.points[constraint.b];
        if (!a || !b) return null;
        return textAt((a.x + b.x) / 2, (a.y + b.y) / 2, 'S');
      }
      case 'fix': {
        const p = s.points[constraint.point];
        if (!p) return null;
        return textAt(p.x, p.y, '⯐');
      }
      default:
        return null;
    }
  }

  function renderDraft() {
    if (!draft || !cursor) return null;
    const snap = cursor.snap;
    if (draft.kind === 'line' && draft.start) {
      const p = anchorPos(draft.start);
      const [x1, y1] = worldToScreen(p.x, p.y);
      const [x2, y2] = worldToScreen(snap.x, snap.y);
      return (
        <g>
          <line className="draft" x1={x1} y1={y1} x2={x2} y2={y2} />
          <circle className="draft-anchor" cx={x1} cy={y1} r={3.5} />
        </g>
      );
    }
    if (draft.kind === 'rect') {
      const [x1, y1] = worldToScreen(draft.x1, draft.y1);
      const [x2, y2] = worldToScreen(snap.x, snap.y);
      return (
        <rect
          className="draft"
          x={Math.min(x1, x2)}
          y={Math.min(y1, y2)}
          width={Math.abs(x2 - x1)}
          height={Math.abs(y2 - y1)}
          fill="none"
        />
      );
    }
    if (draft.kind === 'circle') {
      const c = anchorPos(draft.center);
      const [cx, cy] = worldToScreen(c.x, c.y);
      // Preview the snapped radius unless the cursor snapped to the center.
      const at = snap.id && snap.id === draft.center.id ? cursor : snap;
      const r = Math.hypot(at.x - c.x, at.y - c.y) * view.scale;
      return <circle className="draft" cx={cx} cy={cy} r={r} fill="none" />;
    }
    if (draft.kind === 'arc') {
      const c = anchorPos(draft.center);
      const [cx, cy] = worldToScreen(c.x, c.y);
      if (draft.stage === 1) {
        const r = Math.hypot(cursor.x - c.x, cursor.y - c.y) * view.scale;
        return <circle className="draft faint" cx={cx} cy={cy} r={r} fill="none" />;
      }
      const start = anchorPos(draft.start);
      const r = Math.hypot(start.x - c.x, start.y - c.y);
      const angle = Math.atan2(cursor.y - c.y, cursor.x - c.x);
      const ex = c.x + r * Math.cos(angle);
      const ey = c.y + r * Math.sin(angle);
      const ccw = (draft.accum ?? 0) >= 0;
      const startAngle = normalizeAngle(
        Math.atan2(start.y - c.y, start.x - c.x)
      );
      const sweep = arcSweep(startAngle, normalizeAngle(angle), ccw);
      const [sx, sy] = worldToScreen(start.x, start.y);
      const [px, py] = worldToScreen(ex, ey);
      const rr = r * view.scale;
      return (
        <g>
          <circle className="draft faint" cx={cx} cy={cy} r={rr} fill="none" />
          <path
            className="draft"
            d={`M ${sx} ${sy} A ${rr} ${rr} 0 ${sweep > Math.PI ? 1 : 0} ${
              ccw ? 0 : 1
            } ${px} ${py}`}
            fill="none"
          />
        </g>
      );
    }
    if (draft.kind === 'polygon') {
      const c = anchorPos(draft.center);
      const at = snap.id && snap.id === draft.center.id ? cursor : snap;
      const radius = Math.hypot(at.x - c.x, at.y - c.y);
      if (!(radius > 1e-9)) return null;
      const rot = Math.atan2(at.y - c.y, at.x - c.x);
      const n = Math.max(MIN_POLYGON_SIDES, Math.round(polygonSides));
      const pts = [];
      for (let i = 0; i < n; i++) {
        const a = rot + (i * 2 * Math.PI) / n;
        pts.push(
          worldToScreen(c.x + radius * Math.cos(a), c.y + radius * Math.sin(a))
        );
      }
      return (
        <polygon
          className="draft"
          points={pts.map(([x, y]) => `${x},${y}`).join(' ')}
          fill="none"
        />
      );
    }
    if (draft.kind === 'slot') {
      const { p1 } = draft;
      const [x1, y1] = worldToScreen(p1.x, p1.y);
      if (draft.stage === 1) {
        const [x2, y2] = worldToScreen(snap.x, snap.y);
        return <line className="draft faint" x1={x1} y1={y1} x2={x2} y2={y2} />;
      }
      const { p2 } = draft;
      const dx = p2.x - p1.x;
      const dy = p2.y - p1.y;
      const len = Math.hypot(dx, dy) || 1;
      const width =
        Math.abs((cursor.x - p1.x) * -dy + (cursor.y - p1.y) * dx) / len;
      const [cx2, cy2] = worldToScreen(p2.x, p2.y);
      // Sample the obround outline (rails + semicircular caps) as one polyline.
      const base = Math.atan2(dy, dx); // centerline direction
      const outline = [];
      const capSamples = 16;
      // Start cap: sweep the far side around p1 from +normal to −normal.
      for (let i = 0; i <= capSamples; i++) {
        const a = base + Math.PI / 2 + (i / capSamples) * Math.PI;
        outline.push([p1.x + width * Math.cos(a), p1.y + width * Math.sin(a)]);
      }
      // End cap: sweep around p2 from −normal back to +normal.
      for (let i = 0; i <= capSamples; i++) {
        const a = base - Math.PI / 2 + (i / capSamples) * Math.PI;
        outline.push([p2.x + width * Math.cos(a), p2.y + width * Math.sin(a)]);
      }
      const pts = outline
        .map(([wx, wy]) => worldToScreen(wx, wy).join(','))
        .join(' ');
      return (
        <g>
          <line className="draft faint" x1={x1} y1={y1} x2={cx2} y2={cy2} />
          <polygon className="draft" points={pts} fill="none" />
        </g>
      );
    }
    return null;
  }

  /** Live measurement (or typed entry) beside the cursor while drafting. */
  function renderDimReadout() {
    if (!draft || !cursor || size.w === 0) return null;
    const snap = cursor.snap;
    let text = null;
    if (dimEntry) {
      if (dimEntry.kind === 'rect') {
        const w = dimEntry.field === 'w';
        text = `W ${dimEntry.w || '…'}${w ? '▏' : ''} × H ${dimEntry.h || '…'}${
          w ? '' : '▏'
        }`;
      } else if (dimEntry.kind === 'circle') {
        text = `R ${dimEntry.text}▏`;
      } else if (dimEntry.kind === 'polygon') {
        text = `${dimEntry.text || polygonSides}▏ sides`;
      } else {
        text = `L ${dimEntry.text}▏`;
      }
    } else if (draft.kind === 'polygon') {
      const c = anchorPos(draft.center);
      const at = snap.id && snap.id === draft.center.id ? cursor : snap;
      const r = Math.hypot(at.x - c.x, at.y - c.y);
      text = `${polygonSides}-gon · R ${formatNumber(r)}`;
    } else if (draft.kind === 'slot' && draft.stage === 2) {
      const { p1, p2 } = draft;
      const dx = p2.x - p1.x;
      const dy = p2.y - p1.y;
      const len = Math.hypot(dx, dy) || 1;
      const width =
        Math.abs((cursor.x - p1.x) * -dy + (cursor.y - p1.y) * dx) / len;
      text = `L ${formatNumber(len)} · W ${formatNumber(2 * width)}`;
    } else if (draft.kind === 'slot') {
      const { p1 } = draft;
      text = `L ${formatNumber(Math.hypot(snap.x - p1.x, snap.y - p1.y))}`;
    } else if (draft.kind === 'line' && draft.start) {
      const p = anchorPos(draft.start);
      const len = Math.hypot(snap.x - p.x, snap.y - p.y);
      if (len > 1e-9) {
        const angle = Math.atan2(snap.y - p.y, snap.x - p.x);
        text = `${formatNumber(len)}  ∠${formatAngle(angle)}`;
      }
    } else if (draft.kind === 'rect') {
      const w = Math.abs(snap.x - draft.x1);
      const h = Math.abs(snap.y - draft.y1);
      if (w > 1e-9 || h > 1e-9) {
        text = `${formatNumber(w)} × ${formatNumber(h)}`;
      }
    } else if (draft.kind === 'circle') {
      const c = anchorPos(draft.center);
      text = `R ${formatNumber(Math.hypot(snap.x - c.x, snap.y - c.y))}`;
    } else if (draft.kind === 'arc' && draft.stage === 2) {
      const start = anchorPos(draft.start);
      const c = anchorPos(draft.center);
      const r = Math.hypot(start.x - c.x, start.y - c.y);
      text = `R ${formatNumber(r)}  ⌒${formatAngle(Math.abs(draft.accum ?? 0))}`;
    } else if (draft.kind === 'arc') {
      const c = anchorPos(draft.center);
      text = `R ${formatNumber(Math.hypot(cursor.x - c.x, cursor.y - c.y))}`;
    }
    if (!text) return null;
    const [sx, sy] = worldToScreen(cursor.x, cursor.y);
    return (
      <div
        className={`dim-readout${dimEntry ? ' entry' : ''}`}
        style={{ left: sx + 16, top: sy + 16 }}
      >
        {text}
      </div>
    );
  }

  function renderDimEdit() {
    if (!dimEdit || size.w === 0) return null;
    const [sx, sy] = worldToScreen(dimEdit.wx, dimEdit.wy);
    return (
      <input
        className="dim-edit"
        style={{ left: sx, top: sy - 26 }}
        autoFocus
        type="number"
        min="0"
        step="any"
        value={dimEdit.text}
        onChange={(e) => setDimEdit({ ...dimEdit, text: e.target.value })}
        onKeyDown={(e) => {
          if (e.key === 'Enter') commitDimEdit();
          else if (e.key === 'Escape') setDimEdit(null);
        }}
        onBlur={() => setDimEdit(null)}
        onPointerDown={(e) => e.stopPropagation()}
      />
    );
  }

  const activeTool = TOOLS.find((t) => t.id === tool);
  const dimTargets = selLines.length + selCurves.length;
  const chainStartPos =
    draft?.kind === 'line' && draft.chainStartId
      ? sketch.points[draft.chainStartId]
      : null;
  const hoverClosesLoop =
    chainStartPos && cursor?.snap?.id === draft.chainStartId;

  return (
    <div className={`sketch-overlay${open ? '' : ' hidden'}`}>
      <svg
        ref={svgRef}
        className={`sketch-svg tool-${tool}`}
        onPointerDown={onPointerDown}
        onPointerMove={onPointerMove}
        onPointerUp={onPointerUp}
        onContextMenu={(event) => event.preventDefault()}
      >
        {renderGrid()}
        {renderProfileFill()}
        {Object.values(sketch.entities).map(renderEntity)}
        {Object.values(sketch.constraints).map(renderConstraintGlyph)}
        {renderDraft()}
        {Object.values(sketch.points).map((p) => {
          const [x, y] = worldToScreen(p.x, p.y);
          const selected = isSelected('point', p.id);
          return (
            <rect
              key={p.id}
              className={`point${selected ? ' selected' : ''}`}
              x={x - 3}
              y={y - 3}
              width={6}
              height={6}
            />
          );
        })}
        {cursor?.snap?.id &&
          sketch.points[cursor.snap.id] &&
          (() => {
            const p = sketch.points[cursor.snap.id];
            const [x, y] = worldToScreen(p.x, p.y);
            return (
              <circle
                className={`snap-indicator${hoverClosesLoop ? ' close-loop' : ''}`}
                cx={x}
                cy={y}
                r={8}
                fill="none"
              />
            );
          })()}
      </svg>

      {renderDimReadout()}
      {renderDimEdit()}

      <div className="sketch-toolbar">
        <div className="group">
          {TOOLS.map((t) => (
            <button
              key={t.id}
              className={`tool-btn${tool === t.id ? ' active' : ''}`}
              onClick={() => selectTool(t.id)}
              title={`${t.hint} (${t.key})`}
            >
              {t.label}
            </button>
          ))}
        </div>
        <div className="group">
          <button
            className="tool-btn"
            disabled={!canUndo(historyRef.current)}
            title="Undo (Cmd/Ctrl+Z)"
            onClick={doUndo}
          >
            ↩
          </button>
          <button
            className="tool-btn"
            disabled={!canRedo(historyRef.current)}
            title="Redo (Shift+Cmd/Ctrl+Z)"
            onClick={doRedo}
          >
            ↪
          </button>
        </div>
        <div className="group">
          <span className="group-label">Plane</span>
          {isFacePlane(plane) && (
            <button
              className="tool-btn active"
              disabled
              title="Sketching on the picked face — pick a named plane to leave it"
            >
              Face
            </button>
          )}
          {PLANES.map((p) => (
            <button
              key={p}
              className={`tool-btn${plane === p ? ' active' : ''}`}
              disabled={Boolean(editing)}
              title={
                editing
                  ? 'The plane is fixed while editing an existing sketch'
                  : undefined
              }
              onClick={() => onPlaneChange(p)}
            >
              {p}
            </button>
          ))}
        </div>
        <div className="group">
          <label>
            <input
              type="checkbox"
              checked={gridOn}
              onChange={(e) => setGridOn(e.target.checked)}
            />
            Grid
          </label>
          <label>
            <input
              type="checkbox"
              checked={snapOn}
              onChange={(e) => setSnapOn(e.target.checked)}
            />
            Snap
          </label>
        </div>
        <div className="group">
          <span className="group-label">Constrain</span>
          <button
            className="tool-btn"
            disabled={selLines.length === 0}
            title="Horizontal"
            onClick={() =>
              applyConstraint(
                selLines.map((e) => ({ type: 'horizontal', line: e.id }))
              )
            }
          >
            H
          </button>
          <button
            className="tool-btn"
            disabled={selLines.length === 0}
            title="Vertical"
            onClick={() =>
              applyConstraint(
                selLines.map((e) => ({ type: 'vertical', line: e.id }))
              )
            }
          >
            V
          </button>
          <button
            className="tool-btn"
            disabled={selPoints.length !== 2}
            title="Coincident (two points)"
            onClick={() =>
              applyConstraint([
                {
                  type: 'coincident',
                  a: selPoints[0].id,
                  b: selPoints[1].id,
                },
              ])
            }
          >
            Coinc
          </button>
          <button
            className="tool-btn"
            disabled={selLines.length !== 1 || selCurves.length !== 1}
            title="Tangent (line + circle/arc)"
            onClick={() =>
              applyConstraint([
                {
                  type: 'tangent',
                  line: selLines[0].id,
                  curve: selCurves[0].id,
                },
              ])
            }
          >
            Tan
          </button>
          <button
            className="tool-btn"
            disabled={selLines.length !== 2}
            title="Parallel (two lines)"
            onClick={() =>
              applyConstraint([
                { type: 'parallel', a: selLines[0].id, b: selLines[1].id },
              ])
            }
          >
            ∥
          </button>
          <button
            className="tool-btn"
            disabled={selLines.length !== 2}
            title="Perpendicular (two lines)"
            onClick={() =>
              applyConstraint([
                { type: 'perpendicular', a: selLines[0].id, b: selLines[1].id },
              ])
            }
          >
            ⊥
          </button>
          <button
            className="tool-btn"
            disabled={selLines.length !== 2}
            title="Collinear (two lines)"
            onClick={() =>
              applyConstraint([
                { type: 'collinear', a: selLines[0].id, b: selLines[1].id },
              ])
            }
          >
            Col
          </button>
          <button
            className="tool-btn"
            disabled={selLines.length !== 2 && selCurves.length !== 2}
            title="Equal (two lines → length, or two circles/arcs → radius)"
            onClick={() =>
              applyConstraint([
                selLines.length === 2
                  ? { type: 'equal', a: selLines[0].id, b: selLines[1].id }
                  : { type: 'equal', a: selCurves[0].id, b: selCurves[1].id },
              ])
            }
          >
            =
          </button>
          <button
            className="tool-btn"
            disabled={selCurves.length !== 2}
            title="Concentric (two circles/arcs)"
            onClick={() =>
              applyConstraint([
                { type: 'concentric', a: selCurves[0].id, b: selCurves[1].id },
              ])
            }
          >
            ◎
          </button>
          <button
            className="tool-btn"
            disabled={selPoints.length !== 1 || selLines.length !== 1}
            title="Midpoint (point + line)"
            onClick={() =>
              applyConstraint([
                {
                  type: 'midpoint',
                  point: selPoints[0].id,
                  line: selLines[0].id,
                },
              ])
            }
          >
            Mid
          </button>
          <button
            className="tool-btn"
            disabled={selPoints.length !== 2 || selLines.length !== 1}
            title="Symmetric (two points about an axis line)"
            onClick={() =>
              applyConstraint([
                {
                  type: 'symmetric',
                  a: selPoints[0].id,
                  b: selPoints[1].id,
                  line: selLines[0].id,
                },
              ])
            }
          >
            Sym
          </button>
          <button
            className="tool-btn"
            disabled={selPoints.length === 0}
            title="Fix (anchor selected points in place)"
            onClick={() =>
              applyConstraint(
                selPoints.map((p) => ({ type: 'fix', point: p.id }))
              )
            }
          >
            Fix
          </button>
          <input
            className="dim-input"
            type="number"
            min="0"
            step="any"
            placeholder="dim"
            value={dimValue}
            onChange={(e) => setDimValue(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Enter') applyDimension();
            }}
          />
          <button
            className="tool-btn"
            disabled={dimTargets === 0}
            title="Set length (line) or radius (circle/arc)"
            onClick={applyDimension}
          >
            Set
          </button>
        </div>
        <div className="group">
          <span className="group-label">Modify</span>
          <button
            className="tool-btn"
            disabled={selLines.length !== 1 || selEntities.length < 2}
            title="Mirror the selected geometry across the selected line"
            onClick={mirrorSelection}
          >
            Mirror
          </button>
          <input
            className="dim-input"
            type="number"
            min="0"
            step="any"
            placeholder="offset"
            value={offsetDist}
            onChange={(e) => setOffsetDist(e.target.value)}
            onKeyDown={(e) => {
              if (e.key === 'Enter') offsetSelection();
            }}
          />
          <button
            className={`tool-btn${offsetFlip ? ' active' : ''}`}
            title="Flip the offset side"
            onClick={() => setOffsetFlip((f) => !f)}
          >
            ⇄
          </button>
          <button
            className="tool-btn"
            disabled={selEntities.length === 0}
            title="Offset the selected geometry by the entered distance"
            onClick={offsetSelection}
          >
            Offset
          </button>
          <button
            className="tool-btn"
            disabled={!faceLoops || faceLoops.length === 0}
            title="Convert the sketched face's boundary edges into sketch geometry"
            onClick={convertEntities}
          >
            Convert
          </button>
        </div>
        <div className="group">
          {editing ? (
            <button
              className={`tool-btn sweep-btn${profile.closed ? ' ready' : ''}`}
              disabled={!profile.closed}
              title={`Replace the profile of ${editing.name} with this sketch`}
              onClick={() => onApplyEdit?.()}
            >
              ✓ Apply to {editing.name}
            </button>
          ) : (
            <>
              <button
                className={`tool-btn sweep-btn${profile.closed ? ' ready' : ''}`}
                disabled={!profile.closed}
                title="Extrude the closed profile along the plane normal"
                onClick={() => onSweep?.('extrude')}
              >
                Extrude
              </button>
              <button
                className={`tool-btn sweep-btn${profile.closed ? ' ready' : ''}`}
                disabled={!profile.closed}
                title="Revolve the closed profile around the sketch's vertical axis"
                onClick={() => onSweep?.('revolve')}
              >
                Revolve
              </button>
            </>
          )}
        </div>
        <div className="group">
          <button className="tool-btn" onClick={clearSketch}>
            Clear
          </button>
          <button
            className="tool-btn finish-btn"
            title="Finish sketch (Esc)"
            onClick={() => onExit?.()}
          >
            ✓ Finish
          </button>
        </div>
      </div>

      <div className="sketch-status">
        <span className="tool-chip">{activeTool?.label}</span>
        <span className={`profile-chip${profile.closed ? ' ok' : ''}`}>
          {profile.closed
            ? `Profile closed · ${profile.segments.length} segment${
                profile.segments.length === 1 ? '' : 's'
              } on ${planeLabel(plane)} — ${
                editing ? `Apply to ${editing.name}` : 'Extrude or Revolve it'
              }`
            : `Open profile: ${profile.reason}`}
        </span>
        {message && <span className="sketch-message">{message}</span>}
        {activeTool && <span className="sketch-hint">{activeTool.hint}</span>}
      </div>
    </div>
  );
});
