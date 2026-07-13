import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useWasm } from './wasm/WasmContext.jsx';
import ErrorBoundary from './components/ErrorBoundary.jsx';
import WasmErrorScreen from './components/WasmErrorScreen.jsx';
import ScriptEditor from './components/ScriptEditor.jsx';
import Viewport3D from './components/Viewport3D.jsx';
import Toolbar from './components/Toolbar.jsx';
import MainToolbar from './components/MainToolbar.jsx';
import StatusBar from './components/StatusBar.jsx';
import ScenePanel from './components/ScenePanel.jsx';
import FeatureTree from './components/FeatureTree.jsx';
import PropertyPanel from './components/PropertyPanel.jsx';
import SketchCanvas from './components/SketchCanvas.jsx';
import SweepPanel from './components/SweepPanel.jsx';
import { DEFAULT_SCRIPT } from './lib/defaultScript.js';
import { freeNodes, nodeLabel, runTracedScript, scriptHeader, serializeTree } from './lib/sceneTree.js';
import { buildBinaryStl } from './lib/stl.js';
import { pickCandidates, pickNodeAt } from './lib/picking.js';
import { applyTranslate, applyRotate, applyScale, pathTo, nodeAt, replaceById } from './lib/transformEdit.js';
import { setNodeArg, setBooleanOp } from './lib/propertyEdit.js';
import { deleteNode } from './lib/deleteNode.js';
import { VIEW_SHORTCUTS } from './lib/views.js';
import { buildFeatures, pruneTree, resolveKeys } from './lib/featureTree.js';
import {
  addShape,
  deleteShape,
  listNodes,
  parseScript,
  updateNumericArg,
} from './lib/shapeGraph.js';
import { buildSweepShape, opsBounds, profileToOps, sweepTreeNode } from './lib/sweep.js';
import { createFaceRegionIndex } from './lib/facePlane.js';
import { isFacePlane } from './lib/sketch/profile.js';

// Adaptive meshing target: maximum chordal deviation from the exact
// surface, in model units. The octree refines near curvature and CSG
// feature edges and stays coarse on flat regions.
const DEFAULT_ACCURACY = 0.01;
const EDIT_DEBOUNCE_MS = 400;
const TOAST_MS = 3500;

function downloadBlob(blob, filename) {
  const link = document.createElement('a');
  link.href = URL.createObjectURL(blob);
  link.download = filename;
  link.click();
  URL.revokeObjectURL(link.href);
}

function meshShape(shape, accuracy) {
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

export default function App() {
  // The WASM lifecycle lives in one store (src/wasm/loader.js, surfaced via
  // WasmContext) — App only reads status and the bound API classes.
  const { status: wasmStatus, error: wasmError, api: wasm, ready: wasmReady, retry: retryWasm } = useWasm();
  const [error, setError] = useState(null);
  const [accuracy, setAccuracy] = useState(DEFAULT_ACCURACY);
  const [exactBooleans, setExactBooleans] = useState(false);
  const [wireframe, setWireframe] = useState(false);
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
  const [featuresCollapsed, setFeaturesCollapsed] = useState(false);
  const [editingSketch, setEditingSketch] = useState(null); // { nodeId, name }
  const sketchCanvasRef = useRef(null);
  // What the viewport shows: the full model, a pruned re-evaluation (some
  // features hidden/suppressed), or nothing (everything hidden). Pruned mode
  // owns its traced nodes and frees them when replaced.
  const displayRef = useRef({ mode: 'full' });

  // Bidirectional sync: the shape operation graph parsed from the script.
  // GUI mutations (palette, parameter edits) rewrite individual statements
  // and push the new script text into the editor, preserving all hand-written
  // code. The graph is re-derived after every change from either side.
  const [graph, setGraph] = useState(() => parseScript(DEFAULT_SCRIPT));

  const scriptRef = useRef(DEFAULT_SCRIPT);
  const graphRef = useRef(graph);
  const accuracyRef = useRef(DEFAULT_ACCURACY);
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

  const remesh = useCallback(({ reframe = false } = {}) => {
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
    const acc = accuracyRef.current;
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
    remesh({ reframe: true });

    if (selectedPathRef.current) {
      const restored = nodeAt(traced.root, selectedPathRef.current);
      if (restored && restored.shape) {
        setSelectedNode(restored);
        try {
          const acc = accuracyRef.current;
          setSelectedMesh(meshShape(restored.shape, acc));
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

  const commitGraph = useCallback(() => {
    const next = parseScript(scriptRef.current);
    graphRef.current = next;
    setGraph(next);
  }, []);

  // Tree-based GUI edits regenerate the whole script; keep the leading
  // comment block (the API-reference header) so it survives every edit.
  const serializeWithHeader = useCallback(
    (root) => serializeTree(root, { header: scriptHeader(scriptRef.current) }),
    []
  );

  // Script -> GUI: re-parse and re-evaluate, debounced behind keystrokes.
  const handleScriptChange = useCallback(
    (source) => {
      if (source === scriptRef.current) return;
      scriptRef.current = source;
      clearTimeout(editTimerRef.current);
      editTimerRef.current = setTimeout(() => {
        commitGraph();
        evaluateScript();
      }, EDIT_DEBOUNCE_MS);
    },
    [commitGraph, evaluateScript]
  );

  const runNow = useCallback(() => {
    clearTimeout(editTimerRef.current);
    commitGraph();
    evaluateScript();
  }, [commitGraph, evaluateScript]);

  useEffect(() => () => clearTimeout(editTimerRef.current), []);

  // GUI -> Script: apply a shapeGraph mutation, push the rewritten script
  // into the editor, and refresh graph + shape immediately.
  const applyMutation = useCallback(
    (result) => {
      if (result.error) {
        setError(result.error);
        return false;
      }
      scriptRef.current = result.source;
      editorRef.current?.setDoc(result.source);
      commitGraph();
      evaluateScript();
      return true;
    },
    [commitGraph, evaluateScript]
  );

  const handleAddShape = useCallback(
    (ctor, args) => {
      const result = addShape(graphRef.current, ctor, args);
      applyMutation(result);
    },
    [applyMutation]
  );

  const handleDeleteShape = useCallback(
    (name) => {
      applyMutation(deleteShape(graphRef.current, name));
    },
    [applyMutation]
  );

  const handleUpdateArg = useCallback(
    (nodeId, linkIndex, argIndex, value) => {
      applyMutation(updateNumericArg(graphRef.current, nodeId, linkIndex, argIndex, value));
    },
    [applyMutation]
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
          const acc = accuracyRef.current;
          setSelectedMesh(meshShape(node.shape, acc));
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
      if (!point) {
        clearSelection();
        setPickedFace(null);
        return;
      }
      // Remember the clicked mesh face (independent of the body-selection
      // toggle) so "Sketch" can open on it and the viewport can tint it.
      const region = faceIndex !== null ? faceRegions()?.regionAt(faceIndex) : null;
      setPickedFace(region ? { ...region, meshKey: meshRef.current.key } : null);
      const candidates = pickCandidates(root);
      const picked = pickNodeAt(candidates, point);
      if (picked) {
        selectNode(picked);
      } else {
        clearSelection();
      }
    },
    [selectNode, clearSelection, faceRegions]
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
  // and push the rewritten script through the usual sync path.
  const handleDeleteSelected = useCallback(() => {
    const root = tracedRef.current?.root;
    if (!root || !selectedNode) return;
    const result = deleteNode(root, selectedNode.id);
    if (result.error) {
      setError(result.error);
      return;
    }
    clearSelection();
    const script = serializeWithHeader(result.root);
    scriptRef.current = script;
    editorRef.current?.setDoc(script);
    commitGraph();
    evaluateScript();
  }, [selectedNode, clearSelection, serializeWithHeader, commitGraph, evaluateScript]);

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

      selectedPathRef.current = path;
      const script = serializeWithHeader(newRoot);
      scriptRef.current = script;
      editorRef.current?.setDoc(script);
      commitGraph();
      evaluateScript();
    },
    [selectedNode, serializeWithHeader, commitGraph, evaluateScript]
  );

  // Property panel edits: mutate the traced tree, then push the serialized
  // script through the same sync path the gizmo uses.
  const applyTreeEdit = useCallback(
    (result, nodeId) => {
      if (result.error) {
        setError(result.error);
        return;
      }
      if (result.root === tracedRef.current?.root) return;
      selectedPathRef.current = pathTo(result.root, nodeId) ?? selectedPathRef.current;
      const script = serializeWithHeader(result.root);
      scriptRef.current = script;
      editorRef.current?.setDoc(script);
      commitGraph();
      evaluateScript();
    },
    [commitGraph, evaluateScript]
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

  // Hide/suppress are view-layer: recompute the displayed mesh from a pruned
  // copy of the tree; the script (source of truth) is untouched. The pruned
  // tree is serialized and re-run because bypassed operations need fresh
  // intermediate shapes.
  useEffect(() => {
    const api = wasmRef.current;
    const root = tracedRef.current?.root;
    if (!api || !root) return;
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
      remesh();
      return;
    }
    const pruned = pruneTree(root, ids);
    if (pruned === root) return;
    freePrevious();
    if (!pruned) {
      displayRef.current = { mode: 'empty' };
      remesh();
      return;
    }
    let traced;
    try {
      traced = runTracedScript(serializeTree(pruned), api.WasmShape, api.WasmProfile2D);
    } catch (err) {
      setError(`Recomputing without hidden features failed: ${String(err)}`);
      displayRef.current = { mode: 'full' };
      remesh();
      return;
    }
    displayRef.current = { mode: 'pruned', shape: traced.root.shape, nodes: traced.nodes };
    remesh();
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

  // Committing a whole-tree edit (delete, sketch replacement): serialize and
  // push through the same sync path the gizmo uses, with no selection carry.
  const commitRoot = useCallback(
    (newRoot) => {
      selectedPathRef.current = null;
      const script = serializeWithHeader(newRoot);
      scriptRef.current = script;
      editorRef.current?.setDoc(script);
      commitGraph();
      evaluateScript();
    },
    [commitGraph, evaluateScript]
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
      text = shape.exportStep(accuracyRef.current);
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
          ? { kind, plane: profile.plane, ops, value: extent, range: extent * 4 }
          : { kind, plane: profile.plane, ops, value: 360, range: 360 }
      );
    },
    [clearSelection]
  );

  const handleSweepChange = useCallback((value) => {
    setSweep((current) => (current ? { ...current, value } : current));
  }, []);

  const cancelSweep = useCallback(() => {
    setSweep(null);
    setSweepError(null);
    setSketchOpen(true);
  }, []);

  // Commit the pending sweep: graft it onto the tree (unioned with any
  // existing shape), then push the serialized script through the same sync
  // path the gizmo uses.
  const applySweep = useCallback(() => {
    if (!sweep) return;
    const script = serializeWithHeader(sweepTreeNode(tracedRef.current?.root ?? null, sweep));
    scriptRef.current = script;
    editorRef.current?.setDoc(script);
    setSweep(null);
    setSweepError(null);
    commitGraph();
    evaluateScript();
  }, [sweep, serializeWithHeader, commitGraph, evaluateScript]);

  // Live preview: remesh the pending sweep whenever its parameters change.
  useEffect(() => {
    if (!sweep || !wasm) {
      setPreviewMesh(null);
      return;
    }
    let shape = null;
    try {
      shape = buildSweepShape(wasm.WasmShape, wasm.WasmProfile2D, sweep);
      setPreviewMesh(meshShape(shape, accuracyRef.current));
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

  // First successful WASM init runs the default script once.
  const bootedRef = useRef(false);
  useEffect(() => {
    if (!wasmReady || bootedRef.current) return;
    bootedRef.current = true;
    evaluateScript();
  }, [wasmReady, evaluateScript]);

  useEffect(() => {
    function onKeyDown(event) {
      const tag = event.target.tagName;
      if (tag === 'INPUT' || tag === 'TEXTAREA') return;
      const cm = event.target.closest('.cm-editor');
      if (cm) return;
      // Sketch mode owns the keyboard (tools, dimensions, undo, Esc).
      if (sketchOpen) return;

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
  }, [clearSelection, sweep, cancelSweep, sketchOpen, handleDeleteSelected]);

  const handleAccuracyChange = useCallback((value) => {
    accuracyRef.current = value;
    setAccuracy(value);
  }, []);

  const handleAccuracyCommit = useCallback(() => {
    // The re-tessellated mesh renumbers triangles; face states are stale.
    setHoverFace(null);
    remesh();
  }, [remesh]);

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

  const graphNodes = useMemo(() => listNodes(graph), [graph]);

  return (
    <div className="app">
      <FeatureTree
        features={features}
        selectedId={selectedNode?.id}
        hiddenKeys={hiddenKeys}
        suppressedKeys={suppressedKeys}
        collapsed={featuresCollapsed}
        disabled={!wasmReady}
        onToggleCollapse={() => setFeaturesCollapsed((v) => !v)}
        onSelect={handleFeatureSelect}
        onRename={handleFeatureRename}
        onToggleHide={handleToggleHide}
        onToggleSuppress={handleToggleSuppress}
        onDelete={handleFeatureDelete}
      />
      <div className="left">
        <header>
          <h1>OpenSolid Playground</h1>
          <p>Edit the script or the scene — both stay in sync.</p>
        </header>
        <ScenePanel
          nodes={graphNodes}
          selected={null}
          onSelect={() => {}}
          onAddShape={handleAddShape}
          onDeleteShape={handleDeleteShape}
          onUpdateArg={handleUpdateArg}
          disabled={!wasmReady}
        />
        <ErrorBoundary name="Script editor">
          <ScriptEditor
            ref={editorRef}
            initialDoc={DEFAULT_SCRIPT}
            onChange={handleScriptChange}
            onRun={runNow}
          />
        </ErrorBoundary>
        {error && <pre className="error">{error}</pre>}
        <Toolbar
          accuracy={accuracy}
          onAccuracyChange={handleAccuracyChange}
          onAccuracyCommit={handleAccuracyCommit}
          exactBooleans={exactBooleans}
          onExactBooleansChange={handleExactBooleansChange}
          onRun={runNow}
          onDownloadStl={downloadStl}
          onDownloadStep={downloadStep}
          disabled={!wasmReady}
        />
      </div>
      <div className="right">
        <MainToolbar
          disabled={!wasmReady}
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
            onPick={handlePick}
            onHover={handleHover}
            onTransform={handleTransform}
          />
        </ErrorBoundary>
        <SweepPanel
          sweep={sweep}
          error={sweepError}
          onChange={handleSweepChange}
          onApply={applySweep}
          onCancel={cancelSweep}
        />
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
