import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import {
  addArc,
  addCircle,
  addConstraint,
  addLine,
  addPoint,
  addRectangle,
  createSketch,
  deleteConstraint,
  deleteEntity,
  deletePoint,
  entityPointIds,
  entityRadius,
  validateConstraint,
} from '../lib/sketch/model.js';
import { solve } from '../lib/sketch/solver.js';
import {
  axisAlign,
  hitTest,
  nearestPoint,
  snapToGrid,
} from '../lib/sketch/snap.js';
import { arcSweep, normalizeAngle } from '../lib/sketch/geom.js';
import { extractProfile, profileTo3D } from '../lib/sketch/profile.js';

const SNAP_PX = 10;
const HIT_PX = 8;
const MIN_SCALE = 2;
const MAX_SCALE = 5000;

const TOOLS = [
  { id: 'select', label: 'Select', hint: 'Drag points to move · Del deletes' },
  { id: 'line', label: 'Line', hint: 'Click to chain · click start to close · Esc ends' },
  { id: 'rect', label: 'Rect', hint: 'Click two opposite corners' },
  { id: 'circle', label: 'Circle', hint: 'Click center, then radius' },
  { id: 'arc', label: 'Arc', hint: 'Click center, start, then end (drag direction)' },
  { id: 'pan', label: 'Pan', hint: 'Drag to pan (middle-drag works anywhere)' },
];

const PLANES = ['XY', 'XZ', 'YZ'];
/** Sketch-axis names (u, v) per plane, for the axis labels. */
const PLANE_AXES = { XY: ['X', 'Y'], XZ: ['X', 'Z'], YZ: ['Y', 'Z'] };

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

/** Whether any entity references the point. */
function pointInUse(sketch, pid) {
  return Object.values(sketch.entities).some((e) =>
    entityPointIds(e).includes(pid)
  );
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

/**
 * 2D sketch canvas: an SVG overlay with its own pan/zoom coordinate system
 * (world units, Y up), drawing tools, constraint management, and live
 * closed-profile extraction.
 *
 * The sketch itself lives in a ref and is mutated in place; `rev` bumps
 * trigger re-render. Keep the component mounted (hidden via CSS) so the
 * sketch survives toggling the overlay.
 */
export default function SketchCanvas({ open, plane, onPlaneChange, onProfileChange }) {
  const sketchRef = useRef(createSketch());
  const svgRef = useRef(null);
  const [rev, setRev] = useState(0);
  const [size, setSize] = useState({ w: 0, h: 0 });
  const [view, setView] = useState({ cx: 0, cy: 0, scale: 60 });
  const [tool, setTool] = useState('select');
  const [selection, setSelection] = useState([]);
  const [draft, setDraft] = useState(null);
  const draftRef = useRef(null);
  draftRef.current = draft;
  const [cursor, setCursor] = useState(null); // world pos + snap info
  const [gridOn, setGridOn] = useState(true);
  const [snapOn, setSnapOn] = useState(true);
  const [dimValue, setDimValue] = useState('');
  const [message, setMessage] = useState(null);
  const dragRef = useRef(null); // { mode: 'pan', ... } | { mode: 'point', id }

  const sketch = sketchRef.current;
  const touch = useCallback(() => setRev((r) => r + 1), []);

  const runSolve = useCallback(
    (pinned) => {
      const result = solve(sketchRef.current, { pinned });
      setMessage(
        result.converged ? null : 'Constraints conflict: solver did not converge'
      );
      return result;
    },
    []
  );

  // ---- coordinate transforms ----------------------------------------------

  const worldToScreen = useCallback(
    (x, y) => [
      (x - view.cx) * view.scale + size.w / 2,
      size.h / 2 - (y - view.cy) * view.scale,
    ],
    [view, size]
  );

  const screenToWorld = useCallback(
    (sx, sy) => ({
      x: (sx - size.w / 2) / view.scale + view.cx,
      y: (size.h / 2 - sy) / view.scale + view.cy,
    }),
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
      setView((v) => {
        const factor = Math.exp(-event.deltaY * 0.0015);
        const scale = Math.min(MAX_SCALE, Math.max(MIN_SCALE, v.scale * factor));
        // Keep the world point under the cursor stationary.
        const wx = (sx - el.clientWidth / 2) / v.scale + v.cx;
        const wy = (el.clientHeight / 2 - sy) / v.scale + v.cy;
        return {
          scale,
          cx: wx - (sx - el.clientWidth / 2) / scale,
          cy: wy - (el.clientHeight / 2 - sy) / scale,
        };
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

  const copyProfile = useCallback(() => {
    navigator.clipboard
      ?.writeText(JSON.stringify(profileTo3D(profile), null, 2))
      .then(() => setMessage('Profile JSON copied to clipboard'))
      .catch(() => setMessage('Clipboard unavailable'));
  }, [profile]);

  // ---- draft lifecycle -----------------------------------------------------

  const cancelDraft = useCallback(() => {
    const d = draftRef.current;
    if (d) {
      const s = sketchRef.current;
      for (const pid of d.created ?? []) {
        if (s.points[pid] && !pointInUse(s, pid)) deletePoint(s, pid);
      }
      touch();
    }
    setDraft(null);
  }, [touch]);

  /** Reuse the snapped existing point or create one; track creations. */
  const materializePoint = useCallback((snap, created) => {
    if (snap.id) return snap.id;
    const pid = addPoint(sketchRef.current, snap.x, snap.y);
    created.push(pid);
    return pid;
  }, []);

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
            dragRef.current = { mode: 'point', id: hit.id };
            event.target.setPointerCapture?.(event.pointerId);
          }
          return;
        }
        case 'line': {
          if (!draft) {
            const snap = resolveSnap(world);
            const created = [];
            const pid = materializePoint(snap, created);
            setDraft({ kind: 'line', startId: pid, prev: pid, created });
            return;
          }
          const prevPt = s.points[draft.prev];
          const snap = resolveSnap(world, { axisFrom: prevPt });
          if (snap.id === draft.prev) {
            // Clicked the pending point again (double-click): finish chain.
            cancelDraft();
            return;
          }
          const created = draft.created;
          const pid = materializePoint(snap, created);
          const lineId = addLine(s, draft.prev, pid);
          if (snap.axis === 'h') {
            addConstraint(s, { type: 'horizontal', line: lineId });
          } else if (snap.axis === 'v') {
            addConstraint(s, { type: 'vertical', line: lineId });
          }
          runSolve();
          if (pid === draft.startId) setDraft(null); // loop closed
          else setDraft({ ...draft, prev: pid, created });
          touch();
          return;
        }
        case 'rect': {
          if (!draft) {
            const snap = resolveSnap(world);
            setDraft({ kind: 'rect', x1: snap.x, y1: snap.y });
            return;
          }
          const snap = resolveSnap(world);
          if (
            Math.abs(snap.x - draft.x1) > 1e-9 &&
            Math.abs(snap.y - draft.y1) > 1e-9
          ) {
            addRectangle(s, draft.x1, draft.y1, snap.x, snap.y);
            runSolve();
            setDraft(null);
            touch();
          }
          return;
        }
        case 'circle': {
          if (!draft) {
            const snap = resolveSnap(world);
            const created = [];
            const pid = materializePoint(snap, created);
            setDraft({ kind: 'circle', centerId: pid, created });
            return;
          }
          const c = s.points[draft.centerId];
          const snap = resolveSnap(world, { exclude: new Set([draft.centerId]) });
          const r = Math.hypot(snap.x - c.x, snap.y - c.y);
          if (r > 1e-9) {
            addCircle(s, draft.centerId, r);
            runSolve();
            setDraft(null);
            touch();
          }
          return;
        }
        case 'arc': {
          if (!draft) {
            const snap = resolveSnap(world);
            const created = [];
            const pid = materializePoint(snap, created);
            setDraft({ kind: 'arc', centerId: pid, created, stage: 1 });
            return;
          }
          if (draft.stage === 1) {
            const snap = resolveSnap(world);
            if (snap.id === draft.centerId) return;
            const created = draft.created;
            const pid = materializePoint(snap, created);
            const c = s.points[draft.centerId];
            const p = s.points[pid];
            if (Math.hypot(p.x - c.x, p.y - c.y) < 1e-9) return;
            const angle = Math.atan2(world.y - c.y, world.x - c.x);
            setDraft({
              ...draft,
              startId: pid,
              created,
              stage: 2,
              prevAngle: angle,
              accum: 0,
            });
            return;
          }
          // Stage 2: place the end on the arc's circle at the cursor angle.
          const c = s.points[draft.centerId];
          const start = s.points[draft.startId];
          const r = Math.hypot(start.x - c.x, start.y - c.y);
          const angle = Math.atan2(world.y - c.y, world.x - c.x);
          const ex = c.x + r * Math.cos(angle);
          const ey = c.y + r * Math.sin(angle);
          if (Math.hypot(ex - start.x, ey - start.y) < 1e-9) return;
          const created = draft.created;
          const endId = addPoint(s, ex, ey);
          created.push(endId);
          const ccw = (draft.accum ?? 0) >= 0;
          addArc(s, draft.centerId, draft.startId, endId, ccw);
          runSolve();
          setDraft(null);
          touch();
          return;
        }
        default:
      }
    },
    [tool, draft, view.scale, resolveSnap, materializePoint, cancelDraft, runSolve, touch]
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
      if (event.button === 0) handleToolClick(eventWorld(event), event);
    },
    [tool, view, cancelDraft, handleToolClick, eventWorld]
  );

  const onPointerMove = useCallback(
    (event) => {
      const drag = dragRef.current;
      if (drag?.mode === 'pan') {
        const dx = (event.clientX - drag.startX) / drag.view0.scale;
        const dy = (event.clientY - drag.startY) / drag.view0.scale;
        setView({
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
          runSolve(new Set([drag.id]));
          touch();
        }
        return;
      }
      // Hover feedback for draw tools.
      let axisFrom = null;
      if (draft?.kind === 'line' && draft.prev) {
        axisFrom = sketchRef.current.points[draft.prev];
      }
      const snap = resolveSnap(world, { axisFrom });
      if (draft?.kind === 'arc' && draft.stage === 2) {
        const c = sketchRef.current.points[draft.centerId];
        const angle = Math.atan2(world.y - c.y, world.x - c.x);
        const delta = wrapToPi(angle - draft.prevAngle);
        setDraft({ ...draft, prevAngle: angle, accum: draft.accum + delta });
      }
      setCursor({ x: world.x, y: world.y, snap });
    },
    [draft, eventWorld, resolveSnap, runSolve, touch]
  );

  const onPointerUp = useCallback((event) => {
    if (dragRef.current) {
      event.target.releasePointerCapture?.(event.pointerId);
      dragRef.current = null;
    }
  }, []);

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
      for (const c of constraints) addConstraint(s, c);
      runSolve();
      touch();
    },
    [runSolve, touch]
  );

  const applyDimension = useCallback(() => {
    const value = Number(dimValue);
    if (!(value > 0)) {
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
    const s = sketchRef.current;
    for (const item of selection) {
      if (item.kind === 'entity') deleteEntity(s, item.id);
      else if (item.kind === 'point') deletePoint(s, item.id);
      else if (item.kind === 'constraint') deleteConstraint(s, item.id);
    }
    setSelection([]);
    runSolve();
    touch();
  }, [selection, runSolve, touch]);

  const clearSketch = useCallback(() => {
    sketchRef.current = createSketch();
    setSelection([]);
    setDraft(null);
    setMessage(null);
    touch();
  }, [touch]);

  // ---- keyboard ----------------------------------------------------------

  useEffect(() => {
    if (!open) return undefined;
    const onKey = (event) => {
      if (event.target.tagName === 'INPUT') return;
      if (event.key === 'Escape') {
        cancelDraft();
        setSelection([]);
      } else if (event.key === 'Delete' || event.key === 'Backspace') {
        deleteSelection();
      }
    };
    window.addEventListener('keydown', onKey);
    return () => window.removeEventListener('keydown', onKey);
  }, [open, cancelDraft, deleteSelection]);

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
    const [uName, vName] = PLANE_AXES[plane];
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

  function renderEntity(entity) {
    const cls = `entity${isSelected('entity', entity.id) ? ' selected' : ''}`;
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
    const { c, p1, p2, r, sweep } = arcScreenGeometry(sketch, entity);
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
    const cls = `glyph${isSelected('constraint', constraint.id) ? ' selected' : ''}`;
    const select = (event) => {
      event.stopPropagation();
      setSelection([{ kind: 'constraint', id: constraint.id }]);
    };
    const textAt = (wx, wy, label, key = constraint.id) => {
      const [x, y] = worldToScreen(wx, wy);
      return (
        <text key={key} className={cls} x={x + 6} y={y - 6} onPointerDown={select}>
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
        return textAt((a.x + b.x) / 2, (a.y + b.y) / 2, `${constraint.value}`);
      }
      case 'radius': {
        const entity = s.entities[constraint.entity];
        if (!entity) return null;
        const c = s.points[entity.center];
        const r = entityRadius(s, entity);
        const d = Math.SQRT1_2;
        return textAt(c.x + r * d, c.y + r * d, `R${constraint.value}`);
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
      default:
        return null;
    }
  }

  function renderDraft() {
    if (!draft || !cursor) return null;
    const s = sketch;
    const snap = cursor.snap;
    if (draft.kind === 'line' && draft.prev) {
      const p = s.points[draft.prev];
      const [x1, y1] = worldToScreen(p.x, p.y);
      const [x2, y2] = worldToScreen(snap.x, snap.y);
      return <line className="draft" x1={x1} y1={y1} x2={x2} y2={y2} />;
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
      const c = s.points[draft.centerId];
      const [cx, cy] = worldToScreen(c.x, c.y);
      // Preview the snapped radius unless the cursor snapped to the center.
      const at = snap.id === draft.centerId ? cursor : snap;
      const r = Math.hypot(at.x - c.x, at.y - c.y) * view.scale;
      return <circle className="draft" cx={cx} cy={cy} r={r} fill="none" />;
    }
    if (draft.kind === 'arc') {
      const c = s.points[draft.centerId];
      const [cx, cy] = worldToScreen(c.x, c.y);
      if (draft.stage === 1) {
        const r = Math.hypot(cursor.x - c.x, cursor.y - c.y) * view.scale;
        return <circle className="draft faint" cx={cx} cy={cy} r={r} fill="none" />;
      }
      const start = s.points[draft.startId];
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
    return null;
  }

  const toolHint = TOOLS.find((t) => t.id === tool)?.hint;
  const dimTargets = selLines.length + selCurves.length;

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
              <circle className="snap-indicator" cx={x} cy={y} r={8} fill="none" />
            );
          })()}
      </svg>

      <div className="sketch-toolbar">
        <div className="group">
          {TOOLS.map((t) => (
            <button
              key={t.id}
              className={`tool-btn${tool === t.id ? ' active' : ''}`}
              onClick={() => selectTool(t.id)}
              title={t.hint}
            >
              {t.label}
            </button>
          ))}
        </div>
        <div className="group">
          <span className="group-label">Plane</span>
          {PLANES.map((p) => (
            <button
              key={p}
              className={`tool-btn${plane === p ? ' active' : ''}`}
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
          <button className="tool-btn" onClick={clearSketch}>
            Clear
          </button>
        </div>
      </div>

      <div className="sketch-status">
        <span className={`profile-chip${profile.closed ? ' ok' : ''}`}>
          {profile.closed
            ? `Profile closed · ${profile.segments.length} segment${
                profile.segments.length === 1 ? '' : 's'
              } on ${plane}`
            : `Open profile: ${profile.reason}`}
        </span>
        {profile.closed && (
          <button className="tool-btn" onClick={copyProfile}>
            Copy profile JSON
          </button>
        )}
        {message && <span className="sketch-message">{message}</span>}
        {toolHint && <span className="sketch-hint">{toolHint}</span>}
      </div>
    </div>
  );
}
