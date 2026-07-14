import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useWasm } from './wasm/WasmContext.jsx';
import ErrorBoundary from './components/ErrorBoundary.jsx';
import WasmErrorScreen from './components/WasmErrorScreen.jsx';
import ScriptEditor from './components/ScriptEditor.jsx';
import Viewport3D from './components/Viewport3D.jsx';
import MainToolbar from './components/MainToolbar.jsx';
import StatusBar from './components/StatusBar.jsx';
import FeatureTree from './components/FeatureTree.jsx';
import PropertyPanel from './components/PropertyPanel.jsx';
import SketchCanvas from './components/SketchCanvas.jsx';
import SweepPanel from './components/SweepPanel.jsx';
import SectionPanel from './components/SectionPanel.jsx';
import { DEFAULT_SCRIPT } from './lib/defaultScript.js';
import { freeNodes, nodeLabel, runTracedScript, scriptHeader, serializeTree } from './lib/sceneTree.js';
import { buildBinaryStl } from './lib/stl.js';
import { pickCandidates, pickNodeAt } from './lib/picking.js';
import { applyTranslate, applyRotate, applyScale, pathTo, nodeAt, replaceById } from './lib/transformEdit.js';
import { setNodeArg, setBooleanOp } from './lib/propertyEdit.js';
import { deleteNode } from './lib/deleteNode.js';
import { VIEW_SHORTCUTS } from './lib/views.js';
import { buildFeatures, pruneTree, resolveKeys } from './lib/featureTree.js';
import { PALETTE } from './lib/shapeGraph.js';
import { addPrimitiveNode, assertStoreConsistency } from './lib/storeSync.js';
import { buildSweepShape, opsBounds, profileToOps, sweepTreeNode } from './lib/sweep.js';
import { createFaceRegionIndex } from './lib/facePlane.js';
import { isFacePlane } from './lib/sketch/profile.js';
import { opsHaveCurvedSegs } from './lib/sketch/fromOps.js';
import { faceRefFromPlane, planarRegionsOf, resolveRefs } from './lib/persistentRef.js';
import { computeRebuildState } from './lib/rebuildState.js';
import {
  createHistory,
  commit as recordHistory,
  undo as undoHistory,
  redo as redoHistory,
  depth as historyDepthOf,
} from './lib/history.js';
import { defaultSection, offsetRange, reseatOffset, sectionBounds } from './lib/sectionView.js';

// Adaptive meshing target: maximum chordal deviation from the exact
// surface, in model units. The octree refines near curvature and CSG
// feature edges and stays coarse on flat regions. Fixed at a high-precision
// default (no user knob): the mesher's feature snapping and remeshing keep
// this both crisp and fast, and scripts needing another target can call
// the wasm meshAdaptive(accuracy) API directly.
const MESH_ACCURACY = 0.005;
const EDIT_DEBOUNCE_MS = 400;
const TOAST_MS = 3500;

// Side panel (Code | Tree tabs) resize clamp: wide enough for the editor,
// never so wide the viewport starves at 1280px windows.
const SIDEBAR_MIN = 240;
const SIDEBAR_MAX = 560;
const SIDEBAR_DEFAULT = 340;

function downloadBlob(blob, filename) {
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = filename;
  link.click();
  URL.revokeObjectURL(link.href);
}

function meshShape(shape, accuracy = MESH_ACCURACY) {
  const data = shape.meshAdaptive(accuracy);
  const positions = data.positions;
  const normals = data.normals;
  const indices = data.indices;
  data.free();
  return { positions, normals, indices };
}

function shapePivot(shape) {
  const b = shape.bounds();
  return [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2];
}

// Half the bounding-box diagonal of a positions array — the natural scale for
// the persistent-reference anchor tolerance. Returns 0 for an empty mesh.
function meshRadius(positions) {
  if (!positions || positions.length === 0) return 0;
  const min = [Infinity, Infinity, Infinity];
  const max = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < positions.length; i += 3) {
    for (let k = 0; k < 3; k += 1) {
      const c = positions[i + k];
      if (c < min[k]) min[k] = c;
      if (c > max[k]) max[k] = c;
    }
  }
  return Math.hypot(max[0] - min[0], max[1] - min[1], max[2] - min[2]) / 2;
}

// How far a "through all" extrude must reach to clear the whole scene from
// either side of the sketch plane: twice the scene's bounding diagonal plus
// the profile extent, with a profile-only fallback when the scene is empty.
function sceneReach(root, extent) {
  const b = root?.shape?.bounds?.();
  if (!b || b.length < 6) return extent * 8;
  const diag = Math.hypot(b[3] - b[0], b[4] - b[1], b[5] - b[2]);
  return Math.max(diag, extent) * 2 + extent;
}

export default function App() {
  // The WASM lifecycle lives in one store (src/wasm/loader.js, surfaced via
  // WasmContext) — App only reads status and the bound API classes.
  const { status: wasmStatus, error: wasmError, api: wasm, ready: wasmReady, retry: retryWasm } = useWasm();
  const [error, setError] = useState(null);
  const [exactBooleans, setExactBooleans] = useState(false);
  const [wireframe, setWireframe] = useState(false);
  // Section view (of-fsl.18): a display-only clipping plane { axis, offset,
  // flip }, or null when off. The offset is shared by the panel slider and the
  // viewport drag handle.
  const [section, setSection] = useState(null);
  const [mesh, setMesh] = useState(null);
  const [stats, setStats] = useState(null);
  const [tree, setTree] = useState(null);
  const [selectedNode, setSelectedNode] = useState(null);
  const [selectedMesh, setSelectedMesh] = useState(null);
  const [selectedPivot, setSelectedPivot] = useState(null);
  const [gizmoMode, setGizmoMode] = useState('translate');
  const [sketchOpen, setSketchOpen] = useState(false);
  const [sketchPlane, setSketchPlane] = useState('XY');
  // Shared sketch-mode view (plane coords + px per world unit): initialized
  // from the camera by Viewport3D, panned/zoomed by SketchCanvas, applied
  // back to the camera — one world-to-screen transform for both layers.
  const [sketchView, setSketchView] = useState(null);
  const [sweep, setSweep] = useState(null);
  const [sweepError, setSweepError] = useState(null);
  const [previewMesh, setPreviewMesh] = useState(null);
  // Hovered planar face: `{ meshKey, tris }` — triangle indices into the
  // displayed mesh, shown as an in-place darkening (of-4eh.18). meshKey
  // guards against painting stale triangles onto a rebuilt mesh.
  const [hoverFace, setHoverFace] = useState(null);
  const [profileClosed, setProfileClosed] = useState(false);
  // Face pick for sketch-on-face: the detectFacePlane result of the last
  // click that hit the mesh (planar face plane, or the reason it isn't
  // usable). Cleared on miss clicks and whenever the mesh is rebuilt, and
  // consumed when a sketch opens on it.
  const [pickedFace, setPickedFace] = useState(null);
  const [toast, setToast] = useState(null);
  const profileRef = useRef(null);
  const viewportRef = useRef(null);
  // Face-region index over the displayed mesh, rebuilt lazily per mesh key.
  const faceRegionsRef = useRef({ key: null, index: null });

  // Feature tree (presentation layer over the traced tree): user renames,
  // per-feature visibility/suppression, panel collapse, and the sketch
  // feature currently open for editing. All keyed by feature keys
  // (`type:ordinal`), which are deterministic for a given script.
  const [featureNames, setFeatureNames] = useState({});
  const [hiddenKeys, setHiddenKeys] = useState(() => new Set());
  const [suppressedKeys, setSuppressedKeys] = useState(() => new Set());
  const [editingSketch, setEditingSketch] = useState(null); // { nodeId, name }

  // Persistent face references (of-fsl.8): Map<featureKey, FaceRef> for
  // sweeps placed on a picked face. Held in a ref (survives renders without
  // re-triggering) and re-resolved against the rebuilt mesh after every edit;
  // the derived per-feature rebuild status drives the tree badges.
  const faceRefsRef = useRef(new Map());
  // A face-sketch just applied but whose new feature key isn't known until the
  // tree rebuilds: { plane, priorKeys } consumed by the assignment effect.
  const pendingFaceRefRef = useRef(null);
  const [rebuildState, setRebuildState] = useState(() => new Map());

  // Side panel: one tabbed panel (Code | Tree) with a draggable splitter.
  // Both panes stay mounted (CSS-hidden) — the CodeMirror instance and the
  // tree's expand state must survive tab switches.
  const [sidebarTab, setSidebarTab] = useState('code');
  const [sidebarWidth, setSidebarWidth] = useState(SIDEBAR_DEFAULT);
  const sidebarWidthRef = useRef(SIDEBAR_DEFAULT);
  const sketchCanvasRef = useRef(null);
  // What the viewport shows: the full model, a pruned re-evaluation (some
  // features hidden/suppressed), or nothing (everything hidden). Pruned mode
  // owns its traced nodes and frees them when replaced.
  const displayRef = useRef({ mode: 'full' });

  // Bidirectional sync: the script text is the source of truth; every model
  // edit funnels through commitScript below.
  const scriptRef = useRef(DEFAULT_SCRIPT);
  // Feature-level (session-wide) undo/redo: a linear stack of script
  // snapshots — the script fully serializes the construction tree, so each
  // store commit that changes it pushes one entry (see src/lib/history.js).
  // Sketch-mode's own history (src/lib/sketch/history.js) nests under this:
  // it owns Ctrl+Z while a sketch is open, and applying the sketch lands as
  // one entry here. Depth is mirrored into state to drive the toolbar.
  const historyRef = useRef(createHistory(DEFAULT_SCRIPT));
  const [historyDepth, setHistoryDepth] = useState({ undo: 0, redo: 0 });
  // Store generation: bumped once per model commit. Remesh requests and the
  // pruned-display recompute carry the generation they were issued for, and
  // results from superseded generations are dropped instead of rendered.
  const generationRef = useRef(0);
  const exactBooleansRef = useRef(false);
  const shapeRef = useRef(null);
  const tracedRef = useRef(null);
  const meshRef = useRef(null);
  const meshKeyRef = useRef(0);
  const editorRef = useRef(null);
  const selectedPathRef = useRef(null);
  const editTimerRef = useRef(null);
  const wasmRef = useRef(null);
  wasmRef.current = wasm;

  const clearSelection = useCallback(() => {
    setSelectedNode(null);
    setSelectedMesh(null);
    setSelectedPivot(null);
    selectedPathRef.current = null;
  }, []);

  const remesh = useCallback(({ reframe = false, generation = generationRef.current } = {}) => {
    // Stale-render guard: a request issued for a superseded model generation
    // must never reach the screen.
    if (generation !== generationRef.current) return;
    const display = displayRef.current;
    if (display.mode === 'empty') {
      const key = ++meshKeyRef.current;
      meshRef.current = { positions: new Float32Array(0), indices: new Uint32Array(0), key };
      setMesh({
        positions: new Float32Array(0),
        normals: new Float32Array(0),
        indices: new Uint32Array(0),
        frame: null,
        key,
      });
      setStats(null);
      return;
    }
    const shape = display.mode === 'pruned' ? display.shape : shapeRef.current;
    if (!shape) return;
    setError(null);
    const acc = MESH_ACCURACY;
    const started = performance.now();
    let data;
    try {
      data = shape.meshAdaptive(acc);
    } catch (err) {
      setError(`Meshing failed: ${String(err)}`);
      return;
    }
    const elapsedMs = performance.now() - started;

    const positions = data.positions;
    const normals = data.normals;
    const indices = data.indices;
    data.free();

    if (indices.length === 0) {
      setError(
        'Mesh is empty: the surface never crosses the sampled region. ' +
          'Check the shape is non-degenerate.'
      );
    }

    let frame = null;
    if (reframe) {
      const b = shape.bounds();
      frame = {
        center: [(b[0] + b[3]) / 2, (b[1] + b[4]) / 2, (b[2] + b[5]) / 2],
        radius: Math.max(
          Math.hypot(b[3] - b[0], b[4] - b[1], b[5] - b[2]) / 2,
          0.1
        ),
      };
    }

    const key = ++meshKeyRef.current;
    meshRef.current = { positions, indices, key };
    setMesh({ positions, normals, indices, frame, key });
    setStats({
      triangles: indices.length / 3,
      vertices: positions.length / 3,
      accuracy: acc,
      exact: Boolean(shape.isExact?.()),
      elapsedMs,
    });
  }, []);

  const evaluateScript = useCallback(() => {
    const api = wasmRef.current;
    if (!api) return;
    // Each evaluation is a new model generation; anything still in flight for
    // the previous one is now stale.
    const generation = ++generationRef.current;
    setError(null);
    // Route booleans through the kernel's exact B-Rep pipeline when the
    // toggle is on (optional-chained: older pkg builds lack the export).
    api.WasmShape.setExactBooleans?.(exactBooleansRef.current);
    let traced;
    try {
      traced = runTracedScript(scriptRef.current, api.WasmShape, api.WasmProfile2D);
    } catch (err) {
      setError(String(err?.stack || err));
      return;
    }
    // The displayed mesh is rebuilt, so hovered/picked face triangles are
    // stale (the region index re-keys itself off the new mesh key).
    setHoverFace(null);
    setPickedFace(null);
    if (tracedRef.current) freeNodes(tracedRef.current.nodes);
    tracedRef.current = traced;
    setTree(traced.root);
    shapeRef.current = traced.root.shape;
    // Dev-mode tripwire: the script view must describe exactly the tree that
    // is about to render. Divergence means a mutation bypassed the store.
    if (import.meta.env.DEV) {
      assertStoreConsistency(scriptRef.current, traced.root);
    }
    remesh({ reframe: true, generation });

    if (selectedPathRef.current) {
      const restored = nodeAt(traced.root, selectedPathRef.current);
      if (restored && restored.shape) {
        setSelectedNode(restored);
        try {
          setSelectedMesh(meshShape(restored.shape));
          setSelectedPivot(shapePivot(restored.shape));
        } catch {
          clearSelection();
        }
      } else {
        clearSelection();
      }
    } else {
      clearSelection();
    }
  }, [remesh, clearSelection]);

  // Tree-based GUI edits regenerate the whole script; keep the leading
  // comment block (the API-reference header) so it survives every edit.
  const serializeWithHeader = useCallback(
    (root) => serializeTree(root, { header: scriptHeader(scriptRef.current) }),
    []
  );

  // THE single store commit: every model edit — script keystrokes, palette,
  // gizmo, property panel, feature delete, sketch apply, sweep apply — lands
  // here. It cancels any pending debounced edit, updates the script view
  // (unless the edit came from the editor itself), and re-evaluates into the
  // traced tree, which fans out to remesh + tree/panel updates. Nothing else
  // may write scriptRef or call evaluateScript directly.
  //
  // `selectPath`: undefined keeps the current selection (restored by path
  // after re-evaluation), null clears it, an array selects that path.
  // `record`: push the previous script onto the undo stack (default). Undo
  // and redo replay a snapshot with record=false so they don't re-record.
  const commitScript = useCallback(
    (source, { fromEditor = false, selectPath, record = true } = {}) => {
      clearTimeout(editTimerRef.current);
      if (selectPath !== undefined) selectedPathRef.current = selectPath;
      // Snapshot the pre-commit script for undo. recordHistory no-ops when the
      // script is unchanged, so redundant commits never create a history step.
      if (record && recordHistory(historyRef.current, source)) {
        setHistoryDepth(historyDepthOf(historyRef.current));
      }
      scriptRef.current = source;
      if (!fromEditor) editorRef.current?.setDoc(source);
      evaluateScript();
    },
    [evaluateScript]
  );

  // Feature-level undo/redo: swap the live script for the adjacent history
  // snapshot and replay it through the store (record=false — the snapshot is
  // already in history). Selection is cleared: the tree it referenced may no
  // longer exist after the model changes shape.
  const undo = useCallback(() => {
    const target = undoHistory(historyRef.current);
    if (target === null) return;
    setHistoryDepth(historyDepthOf(historyRef.current));
    commitScript(target, { record: false, selectPath: null });
  }, [commitScript]);

  const redo = useCallback(() => {
    const target = redoHistory(historyRef.current);
    if (target === null) return;
    setHistoryDepth(historyDepthOf(historyRef.current));
    commitScript(target, { record: false, selectPath: null });
  }, [commitScript]);

  // GUI-edit lane: serialize the mutated tree back into a canonical script.
  // The script is a regenerated view — only the header comment survives a
  // tree commit verbatim; statement layout is canonical form.
  const commitTree = useCallback(
    (newRoot, opts = {}) => {
      commitScript(serializeWithHeader(newRoot), { ...opts, fromEditor: false });
    },
    [commitScript, serializeWithHeader]
  );

  // Script -> store: re-parse and re-evaluate, debounced behind keystrokes.
  // A tree commit in the debounce window cancels the timer (the commit
  // regenerated the script, superseding the keystrokes it was tracking).
  const handleScriptChange = useCallback(
    (source) => {
      if (source === scriptRef.current) return;
      scriptRef.current = source;
      clearTimeout(editTimerRef.current);
      editTimerRef.current = setTimeout(() => {
        commitScript(scriptRef.current, { fromEditor: true });
      }, EDIT_DEBOUNCE_MS);
    },
    [commitScript]
  );

  const runNow = useCallback(() => {
    commitScript(scriptRef.current, { fromEditor: true });
  }, [commitScript]);

  useEffect(() => () => clearTimeout(editTimerRef.current), []);

  // Palette add is a store mutation: union the new primitive onto the tree
  // and let the commit regenerate the script from it.
  const handleAddShape = useCallback(
    (ctor, args) => {
      commitTree(addPrimitiveNode(tracedRef.current?.root ?? null, ctor, args));
    },
    [commitTree]
  );

  const selectNode = useCallback(
    (node, { allowRoot = false } = {}) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      // Viewport picks treat the root as "deselect" (isolating the whole
      // model is a no-op); feature-tree clicks pass allowRoot so the final
      // feature can still open its parameters.
      if (!node || (node === root && !allowRoot) || node.id === selectedNode?.id) {
        clearSelection();
        return;
      }
      setSelectedNode(node);
      selectedPathRef.current = pathTo(root, node.id);
      if (node.shape) {
        try {
          setSelectedMesh(meshShape(node.shape));
          setSelectedPivot(shapePivot(node.shape));
        } catch {
          clearSelection();
        }
      }
    },
    [selectedNode, clearSelection]
  );

  // Lazily (re)build the face-region index for the currently displayed mesh.
  const faceRegions = useCallback(() => {
    const displayed = meshRef.current;
    if (!displayed?.indices?.length) return null;
    if (faceRegionsRef.current.key !== displayed.key) {
      faceRegionsRef.current = {
        key: displayed.key,
        index: createFaceRegionIndex(displayed.positions, displayed.indices),
      };
    }
    return faceRegionsRef.current.index;
  }, []);

  const handlePick = useCallback(
    (point, faceIndex) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      const region = point && faceIndex !== null ? faceRegions()?.regionAt(faceIndex) : null;
      // While an "up to face" extrude is pending, a planar-face click picks
      // that face as the terminating plane instead of selecting a body.
      if (sweep?.kind === 'extrude' && sweep.end === 'toFace') {
        if (region?.planar) {
          const { origin, normal } = region.plane;
          setSweep((s) => (s ? { ...s, target: { origin, normal } } : s));
        } else if (point) {
          setToast('Click a flat face to terminate the extrude');
        }
        return;
      }
      if (!point) {
        clearSelection();
        setPickedFace(null);
        return;
      }
      // Remember the clicked mesh face (independent of the body-selection
      // toggle) so "Sketch" can open on it and the viewport can tint it.
      setPickedFace(region ? { ...region, meshKey: meshRef.current.key } : null);
      const candidates = pickCandidates(root);
      const picked = pickNodeAt(candidates, point);
      if (picked) {
        selectNode(picked);
      } else {
        clearSelection();
      }
    },
    [selectNode, clearSelection, faceRegions, sweep]
  );

  // Hover highlight: resolve the pointer's triangle to its planar face
  // region and darken it in place on the main mesh — no overlay geometry,
  // so nothing to z-fight (of-4eh.18). Curved surfaces get no highlight,
  // which keeps hover an honest "you can sketch here" affordance.
  const handleHover = useCallback(
    (point, faceIndex) => {
      const displayed = meshRef.current;
      if (!point || faceIndex === null || !displayed) {
        setHoverFace(null);
        return;
      }
      const region = faceRegions()?.regionAt(faceIndex);
      if (!region?.planar) {
        setHoverFace(null);
        return;
      }
      // Regions are shared objects, so identity says "same face": keep the
      // previous state to skip re-renders while the pointer stays on it.
      setHoverFace((prev) =>
        prev && prev.tris === region.tris && prev.meshKey === displayed.key
          ? prev
          : { meshKey: displayed.key, tris: region.tris }
      );
    },
    [faceRegions]
  );

  // SolidWorks Delete gesture: remove the selected body/branch from the tree
  // and commit through the store.
  const handleDeleteSelected = useCallback(() => {
    const root = tracedRef.current?.root;
    if (!root || !selectedNode) return;
    const result = deleteNode(root, selectedNode.id);
    if (result.error) {
      setError(result.error);
      return;
    }
    clearSelection();
    commitTree(result.root, { selectPath: null });
  }, [selectedNode, clearSelection, commitTree]);

  const handleTransform = useCallback(
    (event) => {
      const root = tracedRef.current?.root;
      if (!root || !selectedNode) return;

      const path = pathTo(root, selectedNode.id);
      let newRoot;

      if (event.mode === 'translate') {
        newRoot = applyTranslate(root, selectedNode.id, event.delta);
      } else if (event.mode === 'rotate') {
        newRoot = applyRotate(root, selectedNode.id, event.axis, event.angle, event.pivot);
      } else if (event.mode === 'scale') {
        newRoot = applyScale(root, selectedNode.id, event.factors, event.pivot);
      } else {
        return;
      }

      commitTree(newRoot, { selectPath: path });
    },
    [selectedNode, commitTree]
  );

  // Property panel edits: mutate the traced tree, then commit through the
  // same store funnel the gizmo uses.
  const applyTreeEdit = useCallback(
    (result, nodeId) => {
      if (result.error) {
        setError(result.error);
        return;
      }
      if (result.root === tracedRef.current?.root) return;
      commitTree(result.root, {
        selectPath: pathTo(result.root, nodeId) ?? selectedPathRef.current,
      });
    },
    [commitTree]
  );

  const handleEditArg = useCallback(
    (nodeId, argIndex, value) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      applyTreeEdit(setNodeArg(root, nodeId, argIndex, value), nodeId);
    },
    [applyTreeEdit]
  );

  const handleChangeOp = useCallback(
    (nodeId, op) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      applyTreeEdit(setBooleanOp(root, nodeId, op), nodeId);
    },
    [applyTreeEdit]
  );

  // ---- feature tree --------------------------------------------------------

  const features = useMemo(() => buildFeatures(tree, featureNames), [tree, featureNames]);

  // Persistent-reference rebuild pass (of-fsl.8). After each rebuild produces a
  // fresh mesh and feature list: (1) attach a pending face ref to its newly
  // created sweep feature, (2) drop refs whose owning feature no longer exists,
  // (3) re-resolve every surviving reference against the current mesh's planar
  // faces, re-anchoring live ones and flagging vanished ones as dangling, and
  // (4) publish the per-feature rebuild status the tree paints as badges.
  useEffect(() => {
    const refs = faceRefsRef.current;

    // (1) Assign a just-applied face-sketch ref to its new sweep feature.
    const pending = pendingFaceRefRef.current;
    if (pending) {
      const created = features.find(
        (f) => f.kind === 'sweep' && !pending.priorKeys.has(f.key)
      );
      if (created) {
        refs.set(created.key, faceRefFromPlane(created.key, pending.plane));
        pendingFaceRefRef.current = null;
      }
    }

    // (2) Prune references orphaned by feature deletion / script edits.
    const liveKeys = new Set(features.map((f) => f.key));
    for (const key of [...refs.keys()]) if (!liveKeys.has(key)) refs.delete(key);

    if (refs.size === 0) {
      setRebuildState((prev) => (prev.size === 0 ? prev : new Map()));
      return;
    }

    // (3) Re-resolve against the displayed mesh's planar faces.
    const displayed = meshRef.current;
    const index = faceRegions();
    let statuses = new Map();
    if (index && displayed?.indices?.length) {
      const regions = planarRegionsOf(index, displayed.indices.length / 3);
      const result = resolveRefs(refs, regions, meshRadius(displayed.positions));
      faceRefsRef.current = result.refs;
      statuses = result.statuses;
    } else {
      // No mesh to resolve against: every reference is dangling for now.
      for (const key of refs.keys()) {
        statuses.set(key, { status: 'dangling', reason: 'model has no faces to resolve against' });
      }
    }

    // (4) Fold into per-feature rebuild state for the tree badges.
    setRebuildState(computeRebuildState(features, statuses));
  }, [mesh, features, faceRegions]);

  // Hide/suppress are view-layer: recompute the displayed mesh from a pruned
  // copy of the tree; the script (source of truth) is untouched. The pruned
  // tree is serialized and re-run because bypassed operations need fresh
  // intermediate shapes.
  useEffect(() => {
    const api = wasmRef.current;
    const root = tracedRef.current?.root;
    if (!api || !root) return;
    // The recompute belongs to the generation whose tree it prunes; a store
    // commit racing this effect supersedes it.
    const generation = generationRef.current;
    const ids = resolveKeys(
      buildFeatures(root),
      [...hiddenKeys, ...suppressedKeys]
    );
    const previous = displayRef.current;
    const freePrevious = () => {
      if (previous.mode === 'pruned') freeNodes(previous.nodes);
    };
    if (ids.size === 0) {
      if (previous.mode === 'full') return;
      freePrevious();
      displayRef.current = { mode: 'full' };
      remesh({ generation });
      return;
    }
    const pruned = pruneTree(root, ids);
    if (pruned === root) return;
    freePrevious();
    if (!pruned) {
      displayRef.current = { mode: 'empty' };
      remesh({ generation });
      return;
    }
    let traced;
    try {
      traced = runTracedScript(serializeTree(pruned), api.WasmShape, api.WasmProfile2D);
    } catch (err) {
      setError(`Recomputing without hidden features failed: ${String(err)}`);
      displayRef.current = { mode: 'full' };
      remesh({ generation });
      return;
    }
    if (generation !== generationRef.current) {
      // Superseded while recomputing: drop the result instead of rendering
      // a scene from a stale tree.
      freeNodes(traced.nodes);
      return;
    }
    displayRef.current = { mode: 'pruned', shape: traced.root.shape, nodes: traced.nodes };
    remesh({ generation });
  }, [tree, hiddenKeys, suppressedKeys, wasmReady, remesh]);

  const handleFeatureRename = useCallback((key, name) => {
    setFeatureNames((prev) => {
      const next = { ...prev };
      if (name) next[key] = name;
      else delete next[key]; // empty rename reverts to the default name
      return next;
    });
  }, []);

  const handleToggleHide = useCallback((key) => {
    setHiddenKeys((prev) => {
      const next = new Set(prev);
      if (next.has(key)) next.delete(key);
      else next.add(key);
      return next;
    });
  }, []);

  const handleToggleSuppress = useCallback((key) => {
    setSuppressedKeys((prev) => {
      const next = new Set(prev);
      if (next.has(key)) next.delete(key);
      else next.add(key);
      return next;
    });
  }, []);

  // Committing a whole-tree edit (delete, sketch replacement): the same
  // store funnel, with no selection carry.
  const commitRoot = useCallback(
    (newRoot) => {
      commitTree(newRoot, { selectPath: null });
    },
    [commitTree]
  );

  const handleFeatureDelete = useCallback(
    (feature) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      const pruned = pruneTree(root, new Set([feature.id]));
      if (!pruned) {
        setError('Cannot delete the last feature.');
        return;
      }
      if (pruned === root) return;
      clearSelection();
      commitRoot(pruned);
    },
    [clearSelection, commitRoot]
  );

  const enterSketchEdit = useCallback(
    (feature) => {
      const node = feature.node;
      if (!node?.profile) return;
      if (opsHaveCurvedSegs(node.profile)) {
        setSweepError(
          'This sketch has ellipse/spline segments and cannot be edited on the canvas yet.'
        );
        return;
      }
      clearSelection();
      setSweep(null);
      setSweepError(null);
      // A leftover face plane from a previous pick would be unrelated to
      // this feature; fall back to a named plane (the profile is stored in
      // the sweep's native frame, so the view plane is presentational).
      setSketchPlane((p) => (isFacePlane(p) ? 'XY' : p));
      sketchCanvasRef.current?.loadProfile(node.profile);
      setEditingSketch({ nodeId: node.id, name: feature.name });
      setSketchOpen(true);
    },
    [clearSelection]
  );

  const handleFeatureSelect = useCallback(
    (feature) => {
      if (feature.kind === 'sketch') {
        enterSketchEdit(feature);
        return;
      }
      selectNode(feature.node, { allowRoot: true });
    },
    [enterSketchEdit, selectNode]
  );

  // Apply an edited sketch back onto its sweep feature: only the profile
  // snapshot is replaced — the sweep parameter and the plane-orientation
  // wrappers around the node stay valid because the profile is expressed in
  // the sweep's native (u, v) frame.
  const handleApplySketchEdit = useCallback(() => {
    const root = tracedRef.current?.root;
    const profile = profileRef.current;
    if (!editingSketch || !root || !profile?.closed) return;
    let ops;
    try {
      ops = profileToOps(profile);
    } catch (err) {
      setError(String(err));
      return;
    }
    const path = pathTo(root, editingSketch.nodeId);
    const target = path !== null ? nodeAt(root, path) : null;
    setEditingSketch(null);
    setSketchOpen(false);
    if (!target) return; // feature vanished (script edited meanwhile)
    const newNode = {
      ...target,
      profile: { start: [...ops.start], segs: ops.segs.map((s) => ({ ...s })) },
      shape: null,
    };
    commitRoot(replaceById(root, editingSketch.nodeId, newNode));
  }, [editingSketch, commitRoot]);

  const downloadStl = useCallback(() => {
    const current = meshRef.current;
    if (!current || current.indices.length === 0) {
      setError('Nothing to export yet: run a script that produces a mesh.');
      return;
    }
    const buffer = buildBinaryStl(current.positions, current.indices);
    downloadBlob(new Blob([buffer], { type: 'model/stl' }), 'opensolid.stl');
  }, []);

  // STEP export serializes the displayed shape itself (not its mesh):
  // exact B-Rep chains emit analytic surfaces, everything else emits a
  // faceted body recovered from the SDF at the current accuracy.
  const downloadStep = useCallback(() => {
    const display = displayRef.current;
    const shape = display.mode === 'pruned' ? display.shape : shapeRef.current;
    if (display.mode === 'empty' || !shape) {
      setError('Nothing to export yet: run a script that produces a shape.');
      return;
    }
    if (typeof shape.exportStep !== 'function') {
      setError('STEP export needs a rebuilt WASM package: run `npm run wasm`.');
      return;
    }
    let text;
    try {
      text = shape.exportStep(MESH_ACCURACY);
    } catch (err) {
      setError(String(err));
      return;
    }
    downloadBlob(new Blob([text], { type: 'application/step' }), 'model.step');
  }, []);

  // ---- extrude / revolve workflow -----------------------------------------

  const handleSweepStart = useCallback(
    (kind) => {
      const profile = profileRef.current;
      if (!profile?.closed) return;
      let ops;
      try {
        ops = profileToOps(profile);
      } catch (err) {
        setError(String(err));
        return;
      }
      const { min, max } = opsBounds(ops);
      const extent = Math.max(max[0] - min[0], max[1] - min[1]) || 1;
      clearSelection();
      setSweepError(null);
      setSketchOpen(false);
      setSweep(
        kind === 'extrude'
          ? {
              kind,
              plane: profile.plane,
              ops,
              value: extent,
              range: extent * 4,
              // SolidWorks-parity controls (see SweepPanel / lib/sweep.js).
              mode: 'boss',
              end: 'blind',
              draft: 0,
              reach: sceneReach(tracedRef.current?.root, extent),
              target: null,
            }
          : { kind, plane: profile.plane, ops, value: 360, range: 360 }
      );
    },
    [clearSelection]
  );

  const handleSweepChange = useCallback((value) => {
    setSweep((current) => (current ? { ...current, value } : current));
  }, []);

  // Update one or more extrude-mode fields (mode/end/draft) on the pending
  // sweep. Switching away from "up to face" drops any captured target.
  const handleSweepField = useCallback((patch) => {
    setSweep((current) => {
      if (!current) return current;
      const next = { ...current, ...patch };
      if (patch.end && patch.end !== 'toFace') next.target = null;
      return next;
    });
  }, []);

  const cancelSweep = useCallback(() => {
    setSweep(null);
    setSweepError(null);
    setSketchOpen(true);
  }, []);

  // Commit the pending sweep: graft it onto the tree (unioned with any
  // existing shape) and commit through the store.
  const applySweep = useCallback(() => {
    if (!sweep) return;
    // A sweep placed on a picked face carries a persistent reference: remember
    // the face plane and the sweep keys that already exist, so the assignment
    // effect can attach the ref to the newly-created sweep feature once the
    // tree rebuilds (its key isn't known until then).
    if (isFacePlane(sweep.plane)) {
      const priorKeys = new Set(features.filter((f) => f.kind === 'sweep').map((f) => f.key));
      pendingFaceRefRef.current = { plane: sweep.plane, priorKeys };
    }
    setSweep(null);
    setSweepError(null);
    commitTree(sweepTreeNode(tracedRef.current?.root ?? null, sweep));
  }, [sweep, commitTree, features]);

  // Live preview: remesh the pending sweep whenever its parameters change.
  useEffect(() => {
    if (!sweep || !wasm) {
      setPreviewMesh(null);
      return;
    }
    let shape = null;
    try {
      shape = buildSweepShape(wasm.WasmShape, wasm.WasmProfile2D, sweep);
      setPreviewMesh(meshShape(shape));
      setSweepError(null);
    } catch (err) {
      setPreviewMesh(null);
      setSweepError(String(err));
    } finally {
      shape?.free?.();
    }
  }, [sweep, wasm]);

  // SolidWorks entry gesture: with a face picked, Sketch opens ON that face
  // (of-4eh.16). A curved face keeps the button honest with a toast instead
  // of silently sketching on a facet.
  const handleSketchToggle = useCallback(() => {
    if (!sketchOpen) {
      if (pickedFace?.planar) {
        setSketchPlane(pickedFace.plane);
        setPickedFace(null);
      } else {
        if (pickedFace) {
          setToast(
            pickedFace.reason === 'face is curved'
              ? 'Sketch on curved faces is not supported yet'
              : `Cannot sketch on this face: ${pickedFace.reason}`
          );
          return;
        }
        // No face picked: never reopen on a stale face plane.
        setSketchPlane((p) => (isFacePlane(p) ? 'XY' : p));
      }
    }
    setSweep(null);
    setSweepError(null);
    setEditingSketch(null);
    setSketchOpen((v) => !v);
  }, [sketchOpen, pickedFace]);

  // Leaving sketch mode without applying abandons a pending feature edit.
  const handleSketchExit = useCallback(() => {
    setEditingSketch(null);
    setSketchOpen(false);
  }, []);

  // Transient toast (non-blocking notice, e.g. "curved face").
  useEffect(() => {
    if (!toast) return undefined;
    const timer = setTimeout(() => setToast(null), TOAST_MS);
    return () => clearTimeout(timer);
  }, [toast]);

  // First successful WASM init commits the current script once.
  const bootedRef = useRef(false);
  useEffect(() => {
    if (!wasmReady || bootedRef.current) return;
    bootedRef.current = true;
    runNow();
  }, [wasmReady, runNow]);

  useEffect(() => {
    function onKeyDown(event) {
      const tag = event.target.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA') return;
      const cm = event.target.closest('.cm-editor');
      if (cm) return;
      // Sketch mode owns the keyboard (tools, dimensions, undo, Esc): its
      // own history nests under the feature-level history handled here.
      if (sketchOpen) return;

      // Feature-level undo/redo. Ctrl/Cmd+Z undoes; Ctrl/Cmd+Shift+Z and
      // Ctrl/Cmd+Y redo (the editor keeps its own undo — this handler already
      // bailed above when focus is in an input or the CodeMirror editor).
      const mod = event.ctrlKey || event.metaKey;
      if (mod && (event.key === 'z' || event.key === 'Z')) {
        event.preventDefault();
        if (event.shiftKey) redo();
        else undo();
        return;
      }
      if (mod && (event.key === 'y' || event.key === 'Y')) {
        event.preventDefault();
        redo();
        return;
      }

      if (event.key === 't' || event.key === 'T') {
        setGizmoMode('translate');
      } else if (event.key === 'r' || event.key === 'R') {
        setGizmoMode('rotate');
      } else if (event.key === 's' || event.key === 'S') {
        setGizmoMode('scale');
      } else if (event.key === 'f' || event.key === 'F' || event.key === ' ') {
        // SolidWorks F / Space: zoom to fit.
        event.preventDefault();
        viewportRef.current?.zoomToFit();
      } else if (!event.ctrlKey && !event.metaKey && !event.altKey && VIEW_SHORTCUTS[event.key]) {
        // 1-7: standard views in SolidWorks Ctrl+1..7 order (plain digits —
        // browsers reserve Ctrl/Cmd+digit for tab switching).
        viewportRef.current?.snapView(VIEW_SHORTCUTS[event.key]);
      } else if (event.key === 'Delete' || event.key === 'Backspace') {
        event.preventDefault();
        handleDeleteSelected();
      } else if (event.key === 'Escape') {
        if (sweep) cancelSweep();
        else clearSelection();
      }
    }
    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [clearSelection, sweep, cancelSweep, sketchOpen, handleDeleteSelected, undo, redo]);

  // Exact booleans rebuild shapes, not just meshes: re-run the script.
  const handleExactBooleansChange = useCallback(
    (enabled) => {
      exactBooleansRef.current = enabled;
      setExactBooleans(enabled);
      runNow();
    },
    [runNow]
  );

  const handleProfileChange = useCallback((profile) => {
    profileRef.current = profile;
    setProfileClosed(Boolean(profile?.closed));
  }, []);

  // ---- section view --------------------------------------------------------
  // Toggle on: seed a default plane through the current model center. Axis
  // changes re-seat the offset back to center; offset/flip are simple edits.
  // Bounds are read from the live mesh so the plane always spans the model.
  const handleSectionToggle = useCallback(() => {
    setSection((cur) => (cur ? null : defaultSection(sectionBounds(meshRef.current?.positions))));
  }, []);

  const handleSectionAxis = useCallback((axis) => {
    setSection((cur) =>
      cur ? reseatOffset({ ...cur, axis }, sectionBounds(meshRef.current?.positions)) : cur
    );
  }, []);

  const handleSectionOffset = useCallback((offset) => {
    setSection((cur) => (cur ? { ...cur, offset } : cur));
  }, []);

  const handleSectionFlip = useCallback((flip) => {
    setSection((cur) => (cur ? { ...cur, flip } : cur));
  }, []);

  const handleSectionClose = useCallback(() => setSection(null), []);

  const sectionRange = useMemo(
    () => (section ? offsetRange(sectionBounds(mesh?.positions), section.axis) : null),
    [section, mesh]
  );

  // Splitter drag: clamp so the editor stays usable and the viewport never
  // starves. Listeners go on the window — the pointer outruns a 5px handle.
  const startSplitterDrag = useCallback((event) => {
    event.preventDefault();
    const startX = event.clientX;
    const startWidth = sidebarWidthRef.current;
    const onMove = (e) => {
      const width = Math.min(
        SIDEBAR_MAX,
        Math.max(SIDEBAR_MIN, startWidth + (e.clientX - startX))
      );
      sidebarWidthRef.current = width;
      setSidebarWidth(width);
    };
    const onUp = () => {
      window.removeEventListener('pointermove', onMove);
      window.removeEventListener('pointerup', onUp);
    };
    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
  }, []);

  return (
    <div className="app">
      <aside className="sidebar" style={{ width: sidebarWidth }}>
        <div className="sidebar-header">
          <span
            className="brand"
            title="Edit the script or the scene — both stay in sync."
          >
            OpenSolid Playground
          </span>
          <button
            className="run-btn"
            onClick={runNow}
            disabled={!wasmReady}
            title="Run the script (Ctrl/Cmd+Enter)"
          >
            Run
          </button>
        </div>
        <div className="sidebar-tabs" role="tablist" aria-label="Side panel">
          <button
            role="tab"
            aria-selected={sidebarTab === 'code'}
            className={sidebarTab === 'code' ? 'active' : ''}
            onClick={() => setSidebarTab('code')}
          >
            Code
          </button>
          <button
            role="tab"
            aria-selected={sidebarTab === 'tree'}
            className={sidebarTab === 'tree' ? 'active' : ''}
            onClick={() => setSidebarTab('tree')}
          >
            Tree
          </button>
        </div>
        <div className={`sidebar-pane${sidebarTab === 'code' ? '' : ' hidden'}`}>
          <ErrorBoundary name="Script editor">
            <ScriptEditor
              ref={editorRef}
              initialDoc={DEFAULT_SCRIPT}
              onChange={handleScriptChange}
              onRun={runNow}
            />
          </ErrorBoundary>
        </div>
        <div className={`sidebar-pane${sidebarTab === 'tree' ? '' : ' hidden'}`}>
          <div className="palette">
            {PALETTE.map((item) => (
              <button
                key={item.ctor}
                className="secondary"
                disabled={!wasmReady}
                title={`Add ${item.label.toLowerCase()} to the scene`}
                onClick={() => handleAddShape(item.ctor, item.args)}
              >
                + {item.label}
              </button>
            ))}
          </div>
          <FeatureTree
            embedded
            features={features}
            selectedId={selectedNode?.id}
            hiddenKeys={hiddenKeys}
            suppressedKeys={suppressedKeys}
            rebuildState={rebuildState}
            disabled={!wasmReady}
            onSelect={handleFeatureSelect}
            onRename={handleFeatureRename}
            onToggleHide={handleToggleHide}
            onToggleSuppress={handleToggleSuppress}
            onDelete={handleFeatureDelete}
          />
        </div>
        {error && <pre className="error">{error}</pre>}
      </aside>
      <div
        className="splitter"
        role="separator"
        aria-orientation="vertical"
        title="Drag to resize the side panel"
        onPointerDown={startSplitterDrag}
      />
      <div className="right">
        <MainToolbar
          disabled={!wasmReady}
          canUndo={historyDepth.undo > 0}
          canRedo={historyDepth.redo > 0}
          undoDepth={historyDepth.undo}
          redoDepth={historyDepth.redo}
          onUndo={undo}
          onRedo={redo}
          sketchOpen={sketchOpen}
          sketchOnFace={Boolean(pickedFace?.planar)}
          onSketchToggle={handleSketchToggle}
          canSweep={sketchOpen && profileClosed && !sweep}
          sweepDisabledReason={
            sketchOpen
              ? 'Close the profile loop in the sketch first'
              : 'Open a sketch and draw a closed profile first'
          }
          onSweep={handleSweepStart}
          onView={(name) => viewportRef.current?.snapView(name)}
          onFit={() => viewportRef.current?.zoomToFit()}
          wireframe={wireframe}
          onWireframeChange={setWireframe}
          section={Boolean(section)}
          onSectionToggle={handleSectionToggle}
          onDownloadStl={downloadStl}
          onDownloadStep={downloadStep}
          exactBooleans={exactBooleans}
          onExactBooleansChange={handleExactBooleansChange}
        />
        <ErrorBoundary name="3D viewport">
          <Viewport3D
            ref={viewportRef}
            mesh={mesh}
            wireframe={wireframe}
            sketchPlane={sketchOpen ? sketchPlane : null}
            sketchView={sketchView}
            onSketchViewChange={setSketchView}
            gizmoMode={gizmoMode}
            selectedMesh={selectedMesh}
            selectedPivot={selectedPivot}
            hoverFaceTris={
              mesh && hoverFace?.meshKey === mesh.key ? hoverFace.tris : null
            }
            selectedFaceTris={
              mesh && pickedFace?.planar && pickedFace.meshKey === mesh.key
                ? pickedFace.tris
                : null
            }
            previewMesh={previewMesh}
            section={section}
            onSectionOffsetChange={handleSectionOffset}
            onPick={handlePick}
            onHover={handleHover}
            onTransform={handleTransform}
          />
        </ErrorBoundary>
        <SweepPanel
          sweep={sweep}
          error={sweepError}
          onChange={handleSweepChange}
          onField={handleSweepField}
          onApply={applySweep}
          onCancel={cancelSweep}
        />
        {section && sectionRange && (
          <SectionPanel
            section={section}
            range={sectionRange}
            onAxisChange={handleSectionAxis}
            onFlip={handleSectionFlip}
            onOffsetChange={handleSectionOffset}
            onClose={handleSectionClose}
          />
        )}
        {selectedNode && (
          <div className="gizmo-bar">
            <button
              className={gizmoMode === 'translate' ? 'gizmo-active' : 'secondary'}
              onClick={() => setGizmoMode('translate')}
              title="Translate (T)"
            >
              Move
            </button>
            <button
              className={gizmoMode === 'rotate' ? 'gizmo-active' : 'secondary'}
              onClick={() => setGizmoMode('rotate')}
              title="Rotate (R)"
            >
              Rotate
            </button>
            <button
              className={gizmoMode === 'scale' ? 'gizmo-active' : 'secondary'}
              onClick={() => setGizmoMode('scale')}
              title="Scale (S)"
            >
              Scale
            </button>
            <span className="gizmo-label">{nodeLabel(selectedNode)}</span>
            <button
              className="secondary danger"
              onClick={handleDeleteSelected}
              title="Delete this body (Delete)"
            >
              Delete
            </button>
            <button className="secondary" onClick={clearSelection} title="Deselect (Esc)">
              Deselect
            </button>
          </div>
        )}
        {selectedNode && !sketchOpen && (
          <PropertyPanel
            node={selectedNode}
            disabled={!wasmReady}
            onEditArg={handleEditArg}
            onChangeOp={handleChangeOp}
          />
        )}
        <ErrorBoundary name="Sketch canvas">
          <SketchCanvas
            ref={sketchCanvasRef}
            open={sketchOpen}
            plane={sketchPlane}
            view={sketchView}
            onViewChange={setSketchView}
            onPlaneChange={setSketchPlane}
            onProfileChange={handleProfileChange}
            onSweep={handleSweepStart}
            onExit={handleSketchExit}
            editing={editingSketch ? { name: editingSketch.name } : null}
            onApplyEdit={handleApplySketchEdit}
          />
        </ErrorBoundary>
        {toast && <div className="toast">{toast}</div>}
        {stats && !sketchOpen && <StatusBar stats={stats} />}
        {(wasmStatus === 'idle' || wasmStatus === 'loading') && (
          <div className="loading">Loading WASM…</div>
        )}
        {wasmStatus === 'failed' && (
          <WasmErrorScreen error={wasmError} onRetry={retryWasm} />
        )}
      </div>
    </div>
  );
}
