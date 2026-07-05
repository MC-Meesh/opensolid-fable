import { useCallback, useEffect, useRef, useState } from 'react';
import init, { WasmShape } from '../pkg/opensolid_wasm.js';
import ScriptEditor from './components/ScriptEditor.jsx';
import Viewport3D from './components/Viewport3D.jsx';
import Toolbar from './components/Toolbar.jsx';
import StatusBar from './components/StatusBar.jsx';
import SceneTree from './components/SceneTree.jsx';
import { DEFAULT_SCRIPT } from './lib/defaultScript.js';
import { freeNodes, nodeLabel, runTracedScript } from './lib/sceneTree.js';
import { buildBinaryStl } from './lib/stl.js';

// Single WASM instantiation shared across (strict-mode re-)mounts.
let wasmInit = null;
function ensureWasm() {
  wasmInit ??= init();
  return wasmInit;
}

const DEFAULT_RESOLUTION = 64;

export default function App() {
  const [wasmReady, setWasmReady] = useState(false);
  const [error, setError] = useState(null);
  const [resolution, setResolution] = useState(DEFAULT_RESOLUTION);
  const [wireframe, setWireframe] = useState(false);
  const [mesh, setMesh] = useState(null); // { positions, normals, indices, frame, key }
  const [stats, setStats] = useState(null); // { triangles, vertices, resolution, elapsedMs }
  const [tree, setTree] = useState(null); // root node of the construction tree
  const [selected, setSelected] = useState(null); // isolated tree node, or null

  const scriptRef = useRef(DEFAULT_SCRIPT); // live editor contents
  const resolutionRef = useRef(DEFAULT_RESOLUTION); // committed slider value
  const shapeRef = useRef(null); // WasmShape currently shown (full model or isolated node)
  const tracedRef = useRef(null); // { root, nodes } from the last successful run
  const meshRef = useRef(null); // last mesh buffers, for STL export
  const meshKeyRef = useRef(0);

  const remesh = useCallback(({ reframe = false } = {}) => {
    const shape = shapeRef.current;
    if (!shape) return;
    setError(null);
    const res = resolutionRef.current;
    const started = performance.now();
    let data;
    try {
      data = shape.mesh(res);
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

    meshRef.current = { positions, indices };
    setMesh({ positions, normals, indices, frame, key: ++meshKeyRef.current });
    setStats({
      triangles: indices.length / 3,
      vertices: positions.length / 3,
      resolution: res,
      elapsedMs,
    });
  }, []);

  const evaluateScript = useCallback(() => {
    setError(null);
    let traced;
    try {
      traced = runTracedScript(scriptRef.current, WasmShape);
    } catch (err) {
      setError(String(err?.stack || err));
      return;
    }
    // The tree nodes own the intermediate WasmShapes (including the root's).
    if (tracedRef.current) freeNodes(tracedRef.current.nodes);
    tracedRef.current = traced;
    setTree(traced.root);
    setSelected(null);
    shapeRef.current = traced.root.shape;
    remesh({ reframe: true });
  }, [remesh]);

  const selectNode = useCallback(
    (node) => {
      const root = tracedRef.current?.root;
      if (!root) return;
      // Clicking the current selection, the root, or clearing → full model.
      const isolate = node && node !== root && node.id !== selected?.id ? node : null;
      setSelected(isolate);
      shapeRef.current = (isolate ?? root).shape;
      remesh({ reframe: true });
    },
    [remesh, selected]
  );

  const downloadStl = useCallback(() => {
    const current = meshRef.current;
    if (!current || current.indices.length === 0) {
      setError('Nothing to export yet: run a script that produces a mesh.');
      return;
    }
    const buffer = buildBinaryStl(current.positions, current.indices);
    const blob = new Blob([buffer], { type: 'model/stl' });
    const link = document.createElement('a');
    link.href = URL.createObjectURL(blob);
    link.download = 'opensolid.stl';
    link.click();
    URL.revokeObjectURL(link.href);
  }, []);

  // Boot: instantiate WASM, then evaluate the default script once.
  const bootedRef = useRef(false);
  useEffect(() => {
    let cancelled = false;
    ensureWasm()
      .then(() => {
        if (cancelled || bootedRef.current) return;
        bootedRef.current = true;
        setWasmReady(true);
        evaluateScript();
      })
      .catch((err) => {
        if (!cancelled) setError(`Failed to load WASM module: ${String(err)}`);
      });
    return () => {
      cancelled = true;
    };
  }, [evaluateScript]);

  const handleResolutionChange = useCallback((value) => {
    resolutionRef.current = value;
    setResolution(value);
  }, []);

  const handleResolutionCommit = useCallback(() => {
    remesh();
  }, [remesh]);

  const handleScriptChange = useCallback((source) => {
    scriptRef.current = source;
  }, []);

  return (
    <div className="app">
      <div className="left">
        <header>
          <h1>OpenSolid Playground</h1>
          <p>Write a script that returns a Shape, then Run (Ctrl/Cmd+Enter).</p>
        </header>
        <SceneTree root={tree} selectedId={selected?.id} onSelect={selectNode} />
        <ScriptEditor
          initialDoc={DEFAULT_SCRIPT}
          onChange={handleScriptChange}
          onRun={evaluateScript}
        />
        {error && <pre className="error">{error}</pre>}
        <Toolbar
          resolution={resolution}
          onResolutionChange={handleResolutionChange}
          onResolutionCommit={handleResolutionCommit}
          wireframe={wireframe}
          onWireframeChange={setWireframe}
          onRun={evaluateScript}
          onDownloadStl={downloadStl}
          disabled={!wasmReady}
        />
      </div>
      <div className="right">
        <Viewport3D mesh={mesh} wireframe={wireframe} />
        {selected && (
          <div className="isolate-banner">
            <span>
              Isolated: <strong>{nodeLabel(selected)}</strong>
            </span>
            <button className="secondary" onClick={() => selectNode(null)}>
              Show full model
            </button>
          </div>
        )}
        {stats && <StatusBar stats={stats} />}
        {!wasmReady && <div className="loading">Loading WASM…</div>}
      </div>
    </div>
  );
}
