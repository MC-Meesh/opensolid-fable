import { forwardRef, useCallback, useEffect, useImperativeHandle, useRef, useState } from 'react';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { TransformControls } from 'three/addons/controls/TransformControls.js';
import ViewTriad from './ViewTriad.jsx';
import { FIT_DISTANCE_FACTOR, MIN_FIT_RADIUS, viewDirection } from '../lib/views.js';
import {
  cameraFromSketchView,
  easeInOutCubic,
  gridLevels,
  orthoHalfExtents,
  planeIndicatorSize,
  sketchViewFromCamera,
  sketchViewPose,
} from '../lib/sketchView.js';
import { isFacePlane } from '../lib/sketch/profile.js';
import { HOVER_RGB, SELECTED_RGB, expandToNonIndexed, paintHighlights } from '../lib/faceHighlight.js';
import {
  axisComponent,
  clipPlaneParams,
  handlePosition,
  sectionBounds,
} from '../lib/sectionView.js';

// World convention: Y up, ground grid in the XZ plane, front view looks
// along -Z. Standard view directions live in lib/views.js.

function frameCamera(ctx, { center, radius }) {
  const { camera, controls } = ctx;
  // Clip planes are owned by the render loop (adapted to camera distance
  // every frame), so framing only has to place the camera.
  // Mid-sketch reframes (script edited while sketching) must NOT move the
  // camera: the sketch overlay is glued to the camera's world-to-screen
  // transform, so the view is owned by the overlay until the sketch closes.
  if (ctx.activeSketchPlane) return;
  const target = new THREE.Vector3(...center);
  const direction = new THREE.Vector3(...viewDirection('iso'));
  camera.position.copy(target).addScaledVector(direction, radius * FIT_DISTANCE_FACTOR);
  controls.target.copy(target);
}

const SKETCH_PLANES = {
  XY: { rotation: [0, 0, 0], color: 0x4f9cf9 },
  XZ: { rotation: [-Math.PI / 2, 0, 0], color: 0x5fdf8a },
  YZ: { rotation: [0, Math.PI / 2, 0], color: 0xef6f6f },
};

const FACE_PLANE_COLOR = 0xf2a65a;

// Reference-geometry glyph colors (of-fsl.14): a distinct datum palette so
// planes/axes/points read apart from sketch planes; coordinate systems use the
// conventional R/G/B axis triad.
const REFERENCE_COLORS = {
  plane: 0x9a86f5,
  axis: 0xf5b942,
  point: 0xff5d9e,
  csysX: 0xef6f6f,
  csysY: 0x5fdf8a,
  csysZ: 0x4f9cf9,
};

/** A named plane the viewport knows, or a picked face plane; else null. */
function validSketchPlane(plane) {
  if (!plane) return null;
  if (isFacePlane(plane)) return plane;
  return SKETCH_PLANES[plane] ? plane : null;
}

const SKETCH_VIEW_ANIM_MS = 300;

/**
 * Unbounded adaptive ground grid: a camera-tracking quad whose fragment
 * shader draws world-space minor/major lines with fwidth anti-aliasing and a
 * radial distance fade. Spacing uniforms are re-derived from the camera
 * distance every frame (see gridLevels), so the grid never runs out no
 * matter how large the scene gets.
 */
function createInfiniteGrid() {
  const material = new THREE.ShaderMaterial({
    transparent: true,
    depthWrite: false,
    side: THREE.DoubleSide,
    uniforms: {
      uMinor: { value: 0.5 },
      uMajor: { value: 5 },
      uMinorAlpha: { value: 1 },
      uFadeDist: { value: 40 },
      uCamPos: { value: new THREE.Vector3() },
      uMinorColor: { value: new THREE.Color(0x22272f) },
      uMajorColor: { value: new THREE.Color(0x2f3742) },
    },
    vertexShader: /* glsl */ `
      varying vec3 vWorldPos;
      void main() {
        vec4 wp = modelMatrix * vec4(position, 1.0);
        vWorldPos = wp.xyz;
        gl_Position = projectionMatrix * viewMatrix * wp;
      }
    `,
    fragmentShader: /* glsl */ `
      varying vec3 vWorldPos;
      uniform float uMinor;
      uniform float uMajor;
      uniform float uMinorAlpha;
      uniform float uFadeDist;
      uniform vec3 uCamPos;
      uniform vec3 uMinorColor;
      uniform vec3 uMajorColor;

      float gridLine(vec2 coord, float spacing) {
        vec2 g = coord / spacing;
        vec2 d = abs(fract(g - 0.5) - 0.5) / fwidth(g);
        return 1.0 - min(min(d.x, d.y), 1.0);
      }

      void main() {
        vec2 p = vWorldPos.xz;
        // Three decade levels crossfaded by uMinorAlpha so lines promote
        // smoothly from minor to major strength as the camera pulls back.
        float wMinor = 0.45 * uMinorAlpha;
        float wMajor = mix(0.45, 0.8, uMinorAlpha);
        float lineA = gridLine(p, uMinor) * wMinor;
        float lineB = gridLine(p, uMajor) * wMajor;
        float lineC = gridLine(p, uMajor * 10.0) * 0.8;
        float dist = distance(vWorldPos, uCamPos);
        float fade = 1.0 - smoothstep(uFadeDist * 0.3, uFadeDist, dist);
        float alpha = max(max(lineA, lineB), lineC) * fade;
        if (alpha < 0.003) discard;
        vec3 colorB = mix(uMinorColor, uMajorColor, uMinorAlpha);
        vec3 color = (lineC >= lineB && lineC >= lineA) ? uMajorColor
          : (lineB >= lineA ? colorB : uMinorColor);
        gl_FragColor = vec4(color, alpha);
      }
    `,
  });
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(2, 2), material);
  mesh.rotation.x = -Math.PI / 2;
  mesh.renderOrder = -1;
  return mesh;
}

/**
 * Kick off a short camera fly-to processed by the render loop. Orbit input is
 * suspended for the duration; the loop re-enables it and fires onDone.
 */
function startViewAnimation(ctx, pose, onDone = null) {
  const { camera, controls } = ctx;
  controls.enabled = false;
  ctx.cameraAnim = {
    fromPos: camera.position.clone(),
    fromUp: camera.up.clone(),
    fromTarget: controls.target.clone(),
    toPos: new THREE.Vector3(...pose.position),
    toUp: new THREE.Vector3(...pose.up),
    toTarget: new THREE.Vector3(...pose.target),
    start: null,
    duration: SKETCH_VIEW_ANIM_MS,
    onDone,
  };
}

/** Per-frame grid upkeep: recenter under the view, cover it, adapt spacing. */
function updateInfiniteGrid(grid, camera, target) {
  const dist = Math.max(camera.position.distanceTo(target), 1e-3);
  const levels = gridLevels(dist);
  const u = grid.material.uniforms;
  u.uMinor.value = levels.minor;
  u.uMajor.value = levels.major;
  u.uMinorAlpha.value = levels.minorAlpha;
  u.uFadeDist.value = levels.fadeDist;
  u.uCamPos.value.copy(camera.position);
  // The line pattern is anchored in world space, so moving/scaling the quad
  // only changes coverage, never the pattern itself.
  grid.position.set(target.x, 0, target.z);
  grid.scale.setScalar(levels.fadeDist * 2);
}

// Selection ghost: solid accent tint (attached to the gizmo anchor so it
// previews the transform while dragging). Face hover/selection highlighting
// paints the main mesh's vertex colors instead — see lib/faceHighlight.js.
const HIGHLIGHT_MATERIAL = new THREE.MeshStandardMaterial({
  color: 0x4fc3f7,
  metalness: 0.1,
  roughness: 0.4,
  transparent: true,
  opacity: 0.35,
  depthWrite: false,
});

const PREVIEW_MATERIAL = new THREE.MeshStandardMaterial({
  color: 0x7ce38b,
  metalness: 0.1,
  roughness: 0.45,
  transparent: true,
  opacity: 0.55,
  depthWrite: false,
});

// Section view (of-fsl.18): a movable clipping plane whose exposed interior is
// filled with a capped cross-section via the stencil-buffer technique from the
// three.js `webgl_clipping_stencil` example. The section color tints the cap
// and the plane widget; a distinct hue reads as "cut material".
const SECTION_COLOR = 0x6fa8dc;
const WIDGET_COLOR = 0x4f9cf9;

// Rotation of the plane widget so its quad (native normal +Z) sits
// perpendicular to the section axis.
const WIDGET_ROTATION = {
  X: [0, Math.PI / 2, 0],
  Y: [-Math.PI / 2, 0, 0],
  Z: [0, 0, 0],
};

/**
 * Two invisible copies of the model that write the stencil buffer where the
 * clip plane passes through solid: back faces increment, front faces
 * decrement, so the count is non-zero exactly across the cut. The cap quad
 * then fills those pixels. Both meshes share the model's geometry (assigned by
 * the caller and refreshed on every remesh) — they never own it.
 */
function createSectionStencilGroup(plane) {
  const base = new THREE.MeshBasicMaterial();
  base.depthWrite = false;
  base.depthTest = false;
  base.colorWrite = false;
  base.stencilWrite = true;
  base.stencilFunc = THREE.AlwaysStencilFunc;

  const backMat = base.clone();
  backMat.side = THREE.BackSide;
  backMat.clippingPlanes = [plane];
  backMat.stencilFail = THREE.IncrementWrapStencilOp;
  backMat.stencilZFail = THREE.IncrementWrapStencilOp;
  backMat.stencilZPass = THREE.IncrementWrapStencilOp;

  const frontMat = base.clone();
  frontMat.side = THREE.FrontSide;
  frontMat.clippingPlanes = [plane];
  frontMat.stencilFail = THREE.DecrementWrapStencilOp;
  frontMat.stencilZFail = THREE.DecrementWrapStencilOp;
  frontMat.stencilZPass = THREE.DecrementWrapStencilOp;

  const back = new THREE.Mesh(new THREE.BufferGeometry(), backMat);
  const front = new THREE.Mesh(new THREE.BufferGeometry(), frontMat);
  back.renderOrder = 1;
  front.renderOrder = 1;
  const group = new THREE.Group();
  group.add(back, front);
  base.dispose();
  return { group, back, front, backMat, frontMat };
}

/** The filled cross-section quad: drawn only where the stencil count is
 *  non-zero, and it clears the stencil after itself so the next frame starts
 *  clean. Oriented onto the plane each frame by the render loop. */
function createSectionCap() {
  const mat = new THREE.MeshStandardMaterial({
    color: SECTION_COLOR,
    metalness: 0.1,
    roughness: 0.75,
    side: THREE.DoubleSide,
    stencilWrite: true,
    stencilRef: 0,
    stencilFunc: THREE.NotEqualStencilFunc,
    stencilFail: THREE.ReplaceStencilOp,
    stencilZFail: THREE.ReplaceStencilOp,
    stencilZPass: THREE.ReplaceStencilOp,
  });
  const cap = new THREE.Mesh(new THREE.PlaneGeometry(1, 1), mat);
  cap.onAfterRender = (renderer) => renderer.clearStencil();
  cap.renderOrder = 1.1;
  return { cap, mat };
}

/** Faint translucent quad + outline marking the whole section plane, so the
 *  cut reads even where it doesn't intersect solid, and giving the drag handle
 *  a visible surface. A child of the handle anchor, so it moves with it. */
function createSectionWidget() {
  const fillGeometry = new THREE.PlaneGeometry(1, 1);
  const fillMaterial = new THREE.MeshBasicMaterial({
    color: WIDGET_COLOR,
    transparent: true,
    opacity: 0.06,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  const edgeGeometry = new THREE.EdgesGeometry(fillGeometry);
  const edgeMaterial = new THREE.LineBasicMaterial({
    color: WIDGET_COLOR,
    transparent: true,
    opacity: 0.5,
  });
  const group = new THREE.Group();
  group.add(new THREE.Mesh(fillGeometry, fillMaterial));
  group.add(new THREE.LineSegments(edgeGeometry, edgeMaterial));
  return { group, fillGeometry, fillMaterial, edgeGeometry, edgeMaterial };
}

const Viewport3D = forwardRef(function Viewport3D(
  {
    mesh,
    wireframe,
    sketchPlane,
    sketchView,
    onSketchViewChange,
    gizmoMode,
    selectedMesh,
    selectedPivot,
    hoverFaceTris,
    selectedFaceTris,
    previewMesh,
    section,
    referenceGeometry,
    onSectionOffsetChange,
    measureEntities,
    measureHover,
    onPick,
    onHover,
    onTransform,
  },
  ref
) {
  const containerRef = useRef(null);
  const sceneRef = useRef(null);
  const [triadQuat, setTriadQuat] = useState([0, 0, 0, 1]);
  const onSketchViewChangeRef = useRef(onSketchViewChange);
  onSketchViewChangeRef.current = onSketchViewChange;

  // Publish the camera's current (or fly-to destination) world-to-screen
  // mapping as a sketch overlay view, so the overlay can mirror it exactly.
  const reportSketchView = useCallback(() => {
    const ctx = sceneRef.current;
    const height = containerRef.current?.clientHeight;
    if (!ctx?.activeSketchPlane || !height) return;
    const anim = ctx.cameraAnim;
    const pos = anim ? anim.toPos : ctx.camera.position;
    const target = anim ? anim.toTarget : ctx.controls.target;
    const dist = Math.max(pos.distanceTo(target), 1e-3);
    onSketchViewChangeRef.current?.(
      sketchViewFromCamera(
        ctx.activeSketchPlane,
        target.toArray(),
        dist,
        ctx.camera.fov,
        height
      )
    );
  }, []);

  useEffect(() => {
    const container = containerRef.current;

    const renderer = new THREE.WebGLRenderer({ antialias: true });
    renderer.setPixelRatio(window.devicePixelRatio);
    // Section view clips per-material (main mesh + stencil group only), so the
    // cap quad and grid stay unclipped. Local, not global, clipping.
    renderer.localClippingEnabled = true;
    container.appendChild(renderer.domElement);

    const scene = new THREE.Scene();
    scene.background = new THREE.Color(0x14171c);

    const camera = new THREE.PerspectiveCamera(45, 1, 0.01, 1000);
    camera.position.set(3, 2.5, 4);

    // Orthographic twin used while sketching (no perspective distortion of
    // dimensions). OrbitControls always drives the perspective camera; the
    // ortho camera mirrors its pose each frame with a matched frustum.
    const orthoCamera = new THREE.OrthographicCamera(-1, 1, 1, -1, 0.01, 1000);

    // SolidWorks mouse mapping: middle-drag rotates, Shift+middle pans,
    // scroll zooms toward the cursor. Left-drag also rotates so trackpad
    // users aren't stranded; right-drag pans.
    const orbitControls = new OrbitControls(camera, renderer.domElement);
    orbitControls.enableDamping = true;
    orbitControls.zoomToCursor = true;
    orbitControls.mouseButtons = {
      LEFT: THREE.MOUSE.ROTATE,
      MIDDLE: THREE.MOUSE.ROTATE,
      RIGHT: THREE.MOUSE.PAN,
    };

    function onPointerDownCapture(event) {
      if (event.button === 1) {
        orbitControls.mouseButtons.MIDDLE = event.shiftKey
          ? THREE.MOUSE.PAN
          : THREE.MOUSE.ROTATE;
      }
    }
    container.addEventListener('pointerdown', onPointerDownCapture, true);

    scene.add(new THREE.HemisphereLight(0xbfd4ff, 0x3a3226, 0.9));
    const keyLight = new THREE.DirectionalLight(0xffffff, 1.6);
    keyLight.position.set(4, 6, 3);
    scene.add(keyLight);
    const fillLight = new THREE.DirectionalLight(0x88aaff, 0.4);
    fillLight.position.set(-4, -2, -3);
    scene.add(fillLight);

    const grid = createInfiniteGrid();
    scene.add(grid);

    // vertexColors multiplies the face-highlight color attribute into the
    // base color (all-white when nothing is highlighted).
    const material = new THREE.MeshStandardMaterial({
      color: 0x5f9ee8,
      metalness: 0.15,
      roughness: 0.5,
      vertexColors: true,
    });
    const meshObject = new THREE.Mesh(new THREE.BufferGeometry(), material);
    scene.add(meshObject);

    const ghostMesh = new THREE.Mesh(new THREE.BufferGeometry(), HIGHLIGHT_MATERIAL);
    ghostMesh.renderOrder = 1;
    ghostMesh.visible = false;

    const previewObject = new THREE.Mesh(new THREE.BufferGeometry(), PREVIEW_MATERIAL);
    previewObject.renderOrder = 2;
    previewObject.visible = false;
    scene.add(previewObject);

    const anchor = new THREE.Group();
    anchor.add(ghostMesh);
    anchor.visible = false;
    scene.add(anchor);

    const transformControls = new TransformControls(camera, renderer.domElement);
    transformControls.attach(anchor);
    // attach() sets the helper root visible=true; the gizmo the user sees is the
    // helper (getHelper()), not the controls object — which extends Controls, not
    // Object3D, so its own `visible` flag renders nothing. Hide the helper directly
    // so the gizmo doesn't linger at the origin before anything is selected.
    const transformControlsHelper = transformControls.getHelper();
    transformControlsHelper.visible = false;
    transformControls.visible = false;
    transformControls.enabled = false;
    scene.add(transformControlsHelper);

    transformControls.addEventListener('dragging-changed', (event) => {
      orbitControls.enabled = !event.value;
    });

    // ---- Section view: clip plane + capped cross-section + drag handle ------
    const sectionPlane = new THREE.Plane(new THREE.Vector3(1, 0, 0), 0);
    const stencil = createSectionStencilGroup(sectionPlane);
    stencil.group.visible = false;
    scene.add(stencil.group);
    const capParts = createSectionCap();
    capParts.cap.visible = false;
    scene.add(capParts.cap);

    // The handle anchor carries the plane widget and is what TransformControls
    // drags; the offset is read back off its position each frame.
    const sectionAnchor = new THREE.Group();
    sectionAnchor.visible = false;
    const sectionWidget = createSectionWidget();
    sectionAnchor.add(sectionWidget.group);
    scene.add(sectionAnchor);

    const sectionHandle = new TransformControls(camera, renderer.domElement);
    sectionHandle.setMode('translate');
    sectionHandle.setSpace('world');
    sectionHandle.attach(sectionAnchor);
    const sectionHandleHelper = sectionHandle.getHelper();
    sectionHandleHelper.visible = false;
    sectionHandle.enabled = false;
    scene.add(sectionHandleHelper);
    sectionHandle.addEventListener('dragging-changed', (event) => {
      orbitControls.enabled = !event.value;
    });
    sectionHandle.addEventListener('mouseUp', () => {
      const s = sceneRef.current?.section;
      const cb = sceneRef.current?._onSectionOffset;
      if (!s?.active || !cb) return;
      cb(sectionAnchor.position.getComponent(axisComponent(s.axis)));
    });

    const raycaster = new THREE.Raycaster();
    const pointerDown = new THREE.Vector2();
    let pointerDownTime = 0;

    function castAt(clientX, clientY) {
      const rect = container.getBoundingClientRect();
      const ndc = new THREE.Vector2(
        ((clientX - rect.left) / rect.width) * 2 - 1,
        -((clientY - rect.top) / rect.height) * 2 + 1
      );
      raycaster.setFromCamera(ndc, sceneRef.current?.sketchOrtho ? orthoCamera : camera);
      const hits = raycaster.intersectObject(meshObject);
      return hits.length > 0 ? hits[0] : null;
    }

    function onPointerDown(event) {
      const rect = container.getBoundingClientRect();
      pointerDown.set(event.clientX - rect.left, event.clientY - rect.top);
      pointerDownTime = performance.now();
    }

    function onPointerUp(event) {
      if (event.button !== 0) return;
      if (performance.now() - pointerDownTime > 500) return;
      const rect = container.getBoundingClientRect();
      const upX = event.clientX - rect.left;
      const upY = event.clientY - rect.top;
      const dist = Math.hypot(upX - pointerDown.x, upY - pointerDown.y);
      if (dist > 5) return;
      if (transformControls.axis) return;

      const cb = sceneRef.current?._onPick;
      if (!cb) return;
      const hit = castAt(event.clientX, event.clientY);
      // A miss clicks empty space: deselect. The hit triangle index feeds
      // face-plane detection for sketch-on-face.
      cb(hit ? [hit.point.x, hit.point.y, hit.point.z] : null, hit?.faceIndex ?? null);
    }

    // Hover highlight: rAF-throttled raycast while no button is held.
    let hoverPending = false;
    let hoverX = 0;
    let hoverY = 0;

    function onPointerMove(event) {
      if (event.buttons !== 0) return;
      hoverX = event.clientX;
      hoverY = event.clientY;
      if (hoverPending) return;
      hoverPending = true;
      requestAnimationFrame(() => {
        hoverPending = false;
        const cb = sceneRef.current?._onHover;
        if (!cb) return;
        if (transformControls.axis) {
          cb(null);
          return;
        }
        const hit = castAt(hoverX, hoverY);
        cb(hit ? [hit.point.x, hit.point.y, hit.point.z] : null, hit?.faceIndex ?? null);
      });
    }

    function onPointerLeave() {
      sceneRef.current?._onHover?.(null);
    }

    container.addEventListener('pointerdown', onPointerDown);
    container.addEventListener('pointerup', onPointerUp);
    container.addEventListener('pointermove', onPointerMove);
    container.addEventListener('pointerleave', onPointerLeave);

    function resize() {
      const { clientWidth: w, clientHeight: h } = container;
      if (w === 0 || h === 0) return;
      renderer.setSize(w, h);
      camera.aspect = w / h;
      camera.updateProjectionMatrix();
      // The px-per-world-unit factor depends on the viewport height, so a
      // resize while sketching changes the overlay mapping.
      reportSketchView();
    }
    const observer = new ResizeObserver(resize);
    observer.observe(container);
    resize();

    // Mirror the perspective camera's pose with a frustum matched to its
    // apparent size at the orbit target, so dollying still reads as zoom.
    function syncOrthoCamera() {
      const dist = Math.max(camera.position.distanceTo(orbitControls.target), 1e-3);
      const { halfW, halfH } = orthoHalfExtents(dist, camera.fov, camera.aspect);
      orthoCamera.left = -halfW;
      orthoCamera.right = halfW;
      orthoCamera.top = halfH;
      orthoCamera.bottom = -halfH;
      orthoCamera.near = dist / 100;
      orthoCamera.far = dist * 100;
      orthoCamera.position.copy(camera.position);
      orthoCamera.quaternion.copy(camera.quaternion);
      orthoCamera.up.copy(camera.up);
      orthoCamera.updateProjectionMatrix();
    }

    // Keep the orientation triad in sync with the camera.
    const lastQuat = new THREE.Quaternion(0, 0, 0, 1);
    renderer.setAnimationLoop(() => {
      const ctx = sceneRef.current;
      const anim = ctx?.cameraAnim;
      if (anim) {
        const now = performance.now();
        anim.start ??= now;
        const t = easeInOutCubic((now - anim.start) / anim.duration);
        camera.position.lerpVectors(anim.fromPos, anim.toPos, t);
        camera.up.lerpVectors(anim.fromUp, anim.toUp, t).normalize();
        orbitControls.target.lerpVectors(anim.fromTarget, anim.toTarget, t);
        camera.lookAt(orbitControls.target);
        if (now - anim.start >= anim.duration) {
          ctx.cameraAnim = null;
          orbitControls.enabled = true;
          orbitControls.update();
          anim.onDone?.();
        }
      } else {
        orbitControls.update();
      }

      // Adapt clip planes to the view distance so zooming far out (or in)
      // never clips the scene or the grid away.
      const viewDist = Math.max(camera.position.distanceTo(orbitControls.target), 1e-3);
      camera.near = viewDist / 500;
      camera.far = viewDist * 500;
      camera.updateProjectionMatrix();

      // Section view: the handle position (dragged, or set from the panel) is
      // the source of truth for the offset. Re-derive the clip plane from it
      // every frame so dragging updates the cut live, and re-seat the cap onto
      // the plane.
      const sec = ctx?.section;
      if (sec?.active) {
        const offset = sectionAnchor.position.getComponent(axisComponent(sec.axis));
        const { normal, constant } = clipPlaneParams({ axis: sec.axis, offset, flip: sec.flip });
        sectionPlane.normal.set(normal[0], normal[1], normal[2]);
        sectionPlane.constant = constant;
        const cap = capParts.cap;
        sectionPlane.coplanarPoint(cap.position);
        cap.lookAt(
          cap.position.x - sectionPlane.normal.x,
          cap.position.y - sectionPlane.normal.y,
          cap.position.z - sectionPlane.normal.z
        );
      }

      const sketching = Boolean(ctx?.sketchOrtho);
      const activeCamera = sketching ? orthoCamera : camera;
      if (sketching) syncOrthoCamera();
      updateInfiniteGrid(grid, activeCamera, orbitControls.target);
      renderer.render(scene, activeCamera);

      if (camera.quaternion.angleTo(lastQuat) > 1e-4) {
        lastQuat.copy(camera.quaternion);
        const q = camera.quaternion;
        setTriadQuat([q.x, q.y, q.z, q.w]);
      }
    });

    function onKeyDown(event) {
      if (event.shiftKey && transformControls.enabled) {
        transformControls.setTranslationSnap(0.5);
        transformControls.setRotationSnap(THREE.MathUtils.degToRad(15));
        transformControls.setScaleSnap(0.25);
      }
    }

    function onKeyUp(event) {
      if (!event.shiftKey) {
        transformControls.setTranslationSnap(null);
        transformControls.setRotationSnap(null);
        transformControls.setScaleSnap(null);
      }
    }

    window.addEventListener('keydown', onKeyDown);
    window.addEventListener('keyup', onKeyUp);

    sceneRef.current = {
      renderer,
      camera,
      controls: orbitControls,
      material,
      meshObject,
      transformControls,
      transformControlsHelper,
      anchor,
      ghostMesh,
      previewObject,
      sectionPlane,
      sectionStencil: stencil,
      sectionCap: capParts,
      sectionWidget,
      sectionAnchor,
      sectionHandle,
      sectionHandleHelper,
      // Live section descriptor mirrored from the `section` prop; the render
      // loop reads axis/flip and the handle position each frame.
      section: { active: false, axis: 'X', flip: false },
      paintedTris: [],
      cameraAnim: null,
      sketchOrtho: false,
      savedView: null,
      activeSketchPlane: null,
      _onPick: null,
      _onHover: null,
      _onTransform: null,
      _onSectionOffset: null,
    };

    return () => {
      container.removeEventListener('pointerdown', onPointerDownCapture, true);
      container.removeEventListener('pointerdown', onPointerDown);
      container.removeEventListener('pointerup', onPointerUp);
      container.removeEventListener('pointermove', onPointerMove);
      container.removeEventListener('pointerleave', onPointerLeave);
      window.removeEventListener('keydown', onKeyDown);
      window.removeEventListener('keyup', onKeyUp);
      observer.disconnect();
      renderer.setAnimationLoop(null);
      transformControls.detach();
      transformControls.dispose();
      sectionHandle.detach();
      sectionHandle.dispose();
      // Stencil meshes borrow the model geometry; never dispose it here.
      stencil.backMat.dispose();
      stencil.frontMat.dispose();
      capParts.cap.geometry.dispose();
      capParts.mat.dispose();
      sectionWidget.fillGeometry.dispose();
      sectionWidget.fillMaterial.dispose();
      sectionWidget.edgeGeometry.dispose();
      sectionWidget.edgeMaterial.dispose();
      orbitControls.dispose();
      meshObject.geometry.dispose();
      ghostMesh.geometry.dispose();
      previewObject.geometry.dispose();
      material.dispose();
      grid.geometry.dispose();
      grid.material.dispose();
      renderer.dispose();
      renderer.domElement.remove();
      sceneRef.current = null;
    };
  }, [reportSketchView]);

  /** Snap the camera to a standard view, keeping target and distance. */
  const snapView = useCallback((name) => {
    const ctx = sceneRef.current;
    const dir = viewDirection(name);
    if (!ctx || !dir) return;
    const dist = Math.max(ctx.camera.position.distanceTo(ctx.controls.target), 1e-6);
    ctx.camera.position
      .copy(ctx.controls.target)
      .addScaledVector(new THREE.Vector3(...dir), dist);
  }, []);

  /** Frame the current mesh, preserving the view direction. */
  const zoomToFit = useCallback(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    const geometry = ctx.meshObject.geometry;
    if (!geometry.getAttribute('position')?.count) return;
    geometry.computeBoundingSphere();
    const sphere = geometry.boundingSphere;
    if (!sphere || !Number.isFinite(sphere.radius) || sphere.radius <= 0) return;

    const direction = ctx.camera.position.clone().sub(ctx.controls.target);
    if (direction.lengthSq() < 1e-12) direction.set(...viewDirection('iso'));
    direction.normalize();

    const radius = Math.max(sphere.radius, MIN_FIT_RADIUS);
    ctx.controls.target.copy(sphere.center);
    ctx.camera.position
      .copy(sphere.center)
      .addScaledVector(direction, radius * FIT_DISTANCE_FACTOR);
  }, []);

  useImperativeHandle(ref, () => ({ snapView, zoomToFit }), [snapView, zoomToFit]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx || !mesh) return;
    // Non-indexed with a color attribute so face highlighting can paint
    // per-triangle colors without bleeding into adjacent faces. Triangle
    // order matches the source mesh, so raycast faceIndex still addresses
    // the original index buffer.
    const { positions, normals, colors } = expandToNonIndexed(mesh);
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute('normal', new THREE.BufferAttribute(normals, 3));
    geometry.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    ctx.meshObject.geometry.dispose();
    ctx.meshObject.geometry = geometry;
    ctx.paintedTris = [];
    if (mesh.frame) frameCamera(ctx, mesh.frame);
  }, [mesh]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (ctx) ctx.material.wireframe = wireframe;
  }, [wireframe]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    if (previewMesh) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(previewMesh.positions, 3));
      geo.setAttribute('normal', new THREE.BufferAttribute(previewMesh.normals, 3));
      geo.setIndex(new THREE.BufferAttribute(previewMesh.indices, 1));
      ctx.previewObject.geometry.dispose();
      ctx.previewObject.geometry = geo;
      ctx.previewObject.visible = true;
    } else {
      ctx.previewObject.visible = false;
    }
  }, [previewMesh]);

  // Section view: activate/teardown the clip plane and configure its widget,
  // handle, and cap from the `section` prop. Runs after the mesh effect above
  // so the stencil group binds to the freshly loaded geometry; re-runs on
  // every remesh to keep that binding and the widget sizing current.
  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    const {
      material,
      sectionPlane,
      sectionStencil,
      sectionCap,
      sectionWidget,
      sectionAnchor,
      sectionHandle,
      sectionHandleHelper,
    } = ctx;
    const s = ctx.section;

    if (!section) {
      if (s.active) {
        s.active = false;
        material.clippingPlanes = null;
        material.needsUpdate = true;
        sectionStencil.group.visible = false;
        sectionCap.cap.visible = false;
        sectionAnchor.visible = false;
        sectionHandleHelper.visible = false;
        sectionHandle.enabled = false;
      }
      return;
    }

    s.active = true;
    s.axis = section.axis;
    s.flip = section.flip;

    const geometry = ctx.meshObject.geometry;
    const bounds = sectionBounds(geometry.getAttribute('position')?.array);
    const size = Math.max(bounds.radius * 2.5, 1);

    // Widget quad: perpendicular to the section axis, sized to the model.
    sectionWidget.group.rotation.set(...WIDGET_ROTATION[section.axis]);
    sectionWidget.group.scale.setScalar(size);
    sectionCap.cap.scale.setScalar(size);

    // Seat the handle at the plane center; restrict its arrows to the axis.
    const pos = handlePosition(bounds, section);
    sectionAnchor.position.set(pos[0], pos[1], pos[2]);
    sectionHandle.showX = section.axis === 'X';
    sectionHandle.showY = section.axis === 'Y';
    sectionHandle.showZ = section.axis === 'Z';

    // Bind the stencil meshes to the current model geometry (borrowed, not
    // owned) and clip the model itself against the shared plane.
    sectionStencil.back.geometry = geometry;
    sectionStencil.front.geometry = geometry;
    material.clippingPlanes = [sectionPlane];
    material.clipShadows = true;
    material.needsUpdate = true;

    sectionStencil.group.visible = true;
    sectionCap.cap.visible = true;
    sectionAnchor.visible = true;
    sectionHandleHelper.visible = true;
    sectionHandle.enabled = true;
  }, [section, mesh]);

  // Measure-tool overlay (of-fsl.17): depth-test-free markers on picked
  // entities, edge/circle highlights, a dashed link line between the two
  // picks, and a hover snap marker. Everything renders on top of the model so
  // the measurement stays legible at any angle or zoom.
  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return undefined;
    if (!measureEntities?.length && !measureHover) return undefined;

    const MEASURE_COLOR = 0x4fc3f7;
    const HOVER_COLOR = 0xffd479;
    const LINK_COLOR = 0xf2a65a;

    const scene = ctx.meshObject.parent;
    const group = new THREE.Group();
    group.renderOrder = 5;

    const geo = ctx.meshObject.geometry;
    if (!geo.boundingSphere) geo.computeBoundingSphere();
    const sphere = geo.boundingSphere;
    const modelR = sphere && sphere.radius > 0 ? sphere.radius : 1;
    const markerR = Math.max(modelR * 0.018, 1e-3);

    const disposables = [];
    const sphereGeo = new THREE.SphereGeometry(markerR, 16, 12);
    disposables.push(sphereGeo);

    const basicMat = (color, opacity = 1) => {
      const m = new THREE.MeshBasicMaterial({
        color,
        depthTest: false,
        depthWrite: false,
        transparent: true,
        opacity,
      });
      disposables.push(m);
      return m;
    };
    const solidLineMat = (color) => {
      const m = new THREE.LineBasicMaterial({
        color,
        depthTest: false,
        depthWrite: false,
        transparent: true,
      });
      disposables.push(m);
      return m;
    };

    const addMarker = (point, color, scale = 1) => {
      const s = new THREE.Mesh(sphereGeo, basicMat(color));
      s.scale.setScalar(scale);
      s.position.set(point[0], point[1], point[2]);
      s.renderOrder = 6;
      group.add(s);
    };
    const addPolyline = (points, color, dashed = false) => {
      const g = new THREE.BufferGeometry().setFromPoints(
        points.map((p) => new THREE.Vector3(p[0], p[1], p[2]))
      );
      disposables.push(g);
      let material;
      if (dashed) {
        material = new THREE.LineDashedMaterial({
          color,
          depthTest: false,
          depthWrite: false,
          transparent: true,
          dashSize: markerR * 2.5,
          gapSize: markerR * 1.8,
        });
        disposables.push(material);
      } else {
        material = solidLineMat(color);
      }
      const line = new THREE.Line(g, material);
      if (dashed) line.computeLineDistances();
      line.renderOrder = 6;
      group.add(line);
    };
    const addCircle = (center, normal, radius, color) => {
      const c = new THREE.Vector3(center[0], center[1], center[2]);
      const n = new THREE.Vector3(normal[0], normal[1], normal[2]).normalize();
      const ref =
        Math.abs(n.y) < 0.99 ? new THREE.Vector3(0, 1, 0) : new THREE.Vector3(1, 0, 0);
      const u = new THREE.Vector3().crossVectors(ref, n).normalize();
      const v = new THREE.Vector3().crossVectors(n, u).normalize();
      const pts = [];
      const N = 64;
      for (let i = 0; i <= N; i += 1) {
        const a = (2 * Math.PI * i) / N;
        pts.push(
          c
            .clone()
            .addScaledVector(u, radius * Math.cos(a))
            .addScaledVector(v, radius * Math.sin(a))
        );
      }
      const g = new THREE.BufferGeometry().setFromPoints(pts);
      disposables.push(g);
      const line = new THREE.Line(g, solidLineMat(color));
      line.renderOrder = 6;
      group.add(line);
    };

    const drawEntity = (e, color) => {
      if (!e) return;
      if (e.point) addMarker(e.point, color);
      if (e.kind === 'edge') addPolyline([e.a, e.b], color);
      else if (e.kind === 'circle') addCircle(e.center, e.normal, e.radius, color);
    };

    for (const e of measureEntities ?? []) drawEntity(e, MEASURE_COLOR);
    if (measureEntities?.length === 2) {
      const [a, b] = measureEntities;
      if (a.point && b.point) addPolyline([a.point, b.point], LINK_COLOR, true);
    }
    if (measureHover) {
      const hoverColor = HOVER_COLOR;
      if (measureHover.point) addMarker(measureHover.point, hoverColor, 0.85);
      if (measureHover.kind === 'edge') addPolyline([measureHover.a, measureHover.b], hoverColor);
      else if (measureHover.kind === 'circle') {
        addCircle(measureHover.center, measureHover.normal, measureHover.radius, hoverColor);
      }
    }

    scene.add(group);
    return () => {
      scene.remove(group);
      for (const d of disposables) d.dispose();
    };
  }, [measureEntities, measureHover, mesh]);

  // Face hover/selection: darken the region's triangles in the main mesh's
  // color attribute (selected paints after hover, so it wins on overlap).
  // Runs after the mesh effect above, so a rebuilt mesh starts unpainted.
  useEffect(() => {
    const ctx = sceneRef.current;
    const colorAttr = ctx?.meshObject.geometry.getAttribute('color');
    if (!colorAttr) return;
    const regions = [];
    if (hoverFaceTris) regions.push({ tris: hoverFaceTris, rgb: HOVER_RGB });
    if (selectedFaceTris) regions.push({ tris: selectedFaceTris, rgb: SELECTED_RGB });
    ctx.paintedTris = paintHighlights(colorAttr.array, ctx.paintedTris, regions);
    colorAttr.needsUpdate = true;
  }, [mesh, hoverFaceTris, selectedFaceTris]);

  // SolidWorks-style "Normal To": entering sketch mode (or switching planes
  // mid-sketch) flies the camera orthogonal to the sketch plane and switches
  // to orthographic projection; exiting restores the saved perspective view.
  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    const { camera, controls } = ctx;
    ctx.activeSketchPlane = validSketchPlane(sketchPlane);
    if (ctx.activeSketchPlane) {
      if (!ctx.savedView) {
        ctx.savedView = {
          position: camera.position.toArray(),
          target: controls.target.toArray(),
          up: camera.up.toArray(),
        };
      }
      const dist = Math.max(camera.position.distanceTo(controls.target), 1e-3);
      const pose = sketchViewPose(sketchPlane, controls.target.toArray(), dist);
      startViewAnimation(ctx, pose, () => {
        ctx.sketchOrtho = true;
      });
      // Hand the destination pose's world-to-screen mapping to the overlay.
      reportSketchView();
    } else if (ctx.savedView) {
      const saved = ctx.savedView;
      ctx.savedView = null;
      ctx.sketchOrtho = false;
      startViewAnimation(ctx, saved);
    }
  }, [sketchPlane, reportSketchView]);

  // Overlay -> camera: the sketch overlay owns pan/zoom while sketching;
  // apply its view so the rendered model stays exactly under the sketch.
  useEffect(() => {
    const ctx = sceneRef.current;
    const height = containerRef.current?.clientHeight;
    if (!ctx?.activeSketchPlane || !sketchView || !height) return;
    const pose = cameraFromSketchView(
      ctx.activeSketchPlane,
      sketchView,
      ctx.camera.fov,
      height
    );
    if (!pose) return;
    if (ctx.cameraAnim) {
      // Mid fly-in: retarget the animation instead of fighting it.
      ctx.cameraAnim.toPos.set(...pose.position);
      ctx.cameraAnim.toUp.set(...pose.up);
      ctx.cameraAnim.toTarget.set(...pose.target);
    } else {
      ctx.camera.position.set(...pose.position);
      ctx.camera.up.set(...pose.up);
      ctx.controls.target.set(...pose.target);
      ctx.camera.lookAt(ctx.controls.target);
    }
  }, [sketchView]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx || !validSketchPlane(sketchPlane)) return undefined;
    const face = isFacePlane(sketchPlane) ? sketchPlane : null;
    const spec = face ? null : SKETCH_PLANES[sketchPlane];

    const scene = ctx.meshObject.parent;
    const group = new THREE.Group();
    let size;
    if (face) {
      // PlaneGeometry lies in local XY with normal +Z, so mapping the local
      // axes onto the face basis (u, v, n) lands the quad on the face.
      group.quaternion.setFromRotationMatrix(
        new THREE.Matrix4().makeBasis(
          new THREE.Vector3(...face.u),
          new THREE.Vector3(...face.v),
          new THREE.Vector3(...face.normal)
        )
      );
      group.position.set(...face.origin);
      // Sized from the detected face region, with a floor for slivers.
      size = Math.max(face.extent * 2.5, 1);
    } else {
      group.rotation.set(...spec.rotation);

      // Size the indicator from the scene bounds so drawn geometry never
      // outruns it; it is a visual aid, not a physical object.
      const meshGeometry = ctx.meshObject.geometry;
      if (!meshGeometry.boundingSphere) meshGeometry.computeBoundingSphere();
      const sphere = meshGeometry.boundingSphere;
      const hasBounds =
        sphere && Number.isFinite(sphere.radius) && sphere.radius > 0;
      size = planeIndicatorSize(
        hasBounds ? sphere.center.toArray() : [0, 0, 0],
        hasBounds ? sphere.radius : 0
      );
    }
    const color = face ? FACE_PLANE_COLOR : spec.color;

    const fillGeometry = new THREE.PlaneGeometry(size, size);
    const fillMaterial = new THREE.MeshBasicMaterial({
      color,
      transparent: true,
      opacity: 0.12,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    group.add(new THREE.Mesh(fillGeometry, fillMaterial));

    const edgeGeometry = new THREE.EdgesGeometry(fillGeometry);
    const edgeMaterial = new THREE.LineBasicMaterial({
      color,
      transparent: true,
      opacity: 0.6,
    });
    group.add(new THREE.LineSegments(edgeGeometry, edgeMaterial));

    scene.add(group);
    return () => {
      scene.remove(group);
      fillGeometry.dispose();
      fillMaterial.dispose();
      edgeGeometry.dispose();
      edgeMaterial.dispose();
    };
  }, [sketchPlane, mesh]);

  // Reference-geometry glyphs (of-fsl.14): render each datum plane / axis /
  // point / coordinate system as a translucent aid. These are not Shapes and
  // never mesh — purely a visual layer, rebuilt when the collection changes.
  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx || !referenceGeometry || referenceGeometry.length === 0) return undefined;
    const scene = ctx.meshObject.parent;

    // A scene-relative default size for datums that carry no extent of their
    // own (axes, points, coordinate systems).
    const meshGeometry = ctx.meshObject.geometry;
    if (!meshGeometry.boundingSphere) meshGeometry.computeBoundingSphere();
    const sphere = meshGeometry.boundingSphere;
    const baseSize =
      sphere && Number.isFinite(sphere.radius) && sphere.radius > 0 ? sphere.radius * 1.5 : 5;

    const root = new THREE.Group();
    const disposables = [];
    const track = (obj) => {
      disposables.push(obj);
      return obj;
    };

    for (const item of referenceGeometry) {
      const g = item.geom;
      if (!g) continue;
      if (item.kind === 'plane') {
        const size = Math.max((g.extent ?? 0) * 2.5, baseSize);
        const grp = new THREE.Group();
        grp.quaternion.setFromRotationMatrix(
          new THREE.Matrix4().makeBasis(
            new THREE.Vector3(...g.u),
            new THREE.Vector3(...g.v),
            new THREE.Vector3(...g.normal)
          )
        );
        grp.position.set(...g.origin);
        const fill = track(new THREE.PlaneGeometry(size, size));
        const fillMat = track(
          new THREE.MeshBasicMaterial({
            color: REFERENCE_COLORS.plane,
            transparent: true,
            opacity: 0.1,
            side: THREE.DoubleSide,
            depthWrite: false,
          })
        );
        grp.add(new THREE.Mesh(fill, fillMat));
        const edges = track(new THREE.EdgesGeometry(fill));
        const edgeMat = track(
          new THREE.LineBasicMaterial({ color: REFERENCE_COLORS.plane, transparent: true, opacity: 0.6 })
        );
        grp.add(new THREE.LineSegments(edges, edgeMat));
        root.add(grp);
      } else if (item.kind === 'axis') {
        const half = ((g.extent ?? 0) > 0 ? g.extent : baseSize) * 1.2;
        const a = [
          g.origin[0] - g.direction[0] * half,
          g.origin[1] - g.direction[1] * half,
          g.origin[2] - g.direction[2] * half,
        ];
        const b = [
          g.origin[0] + g.direction[0] * half,
          g.origin[1] + g.direction[1] * half,
          g.origin[2] + g.direction[2] * half,
        ];
        const geo = track(new THREE.BufferGeometry().setFromPoints([
          new THREE.Vector3(...a),
          new THREE.Vector3(...b),
        ]));
        const mat = track(new THREE.LineBasicMaterial({ color: REFERENCE_COLORS.axis }));
        root.add(new THREE.Line(geo, mat));
      } else if (item.kind === 'point') {
        const geo = track(new THREE.SphereGeometry(baseSize * 0.04, 12, 8));
        const mat = track(new THREE.MeshBasicMaterial({ color: REFERENCE_COLORS.point }));
        const dot = new THREE.Mesh(geo, mat);
        dot.position.set(...g.position);
        root.add(dot);
      } else if (item.kind === 'csys') {
        const len = baseSize * 0.6;
        for (const [dirKey, color] of [
          ['x', REFERENCE_COLORS.csysX],
          ['y', REFERENCE_COLORS.csysY],
          ['z', REFERENCE_COLORS.csysZ],
        ]) {
          const d = g[dirKey];
          const geo = track(new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(...g.origin),
            new THREE.Vector3(
              g.origin[0] + d[0] * len,
              g.origin[1] + d[1] * len,
              g.origin[2] + d[2] * len
            ),
          ]));
          const mat = track(new THREE.LineBasicMaterial({ color }));
          root.add(new THREE.Line(geo, mat));
        }
      }
    }

    scene.add(root);
    return () => {
      scene.remove(root);
      for (const d of disposables) d.dispose();
    };
  }, [referenceGeometry, mesh]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    if (selectedMesh && selectedPivot) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(selectedMesh.positions, 3));
      geo.setAttribute('normal', new THREE.BufferAttribute(selectedMesh.normals, 3));
      geo.setIndex(new THREE.BufferAttribute(selectedMesh.indices, 1));
      ctx.ghostMesh.geometry.dispose();
      ctx.ghostMesh.geometry = geo;
      ctx.ghostMesh.visible = true;

      const [px, py, pz] = selectedPivot;
      ctx.anchor.position.set(px, py, pz);
      ctx.ghostMesh.position.set(-px, -py, -pz);
      ctx.anchor.quaternion.identity();
      ctx.anchor.scale.set(1, 1, 1);
      ctx.anchor.visible = true;

      ctx.transformControlsHelper.visible = true;
      ctx.transformControls.visible = true;
      ctx.transformControls.enabled = true;
    } else {
      ctx.ghostMesh.visible = false;
      ctx.anchor.visible = false;
      ctx.transformControlsHelper.visible = false;
      ctx.transformControls.visible = false;
      ctx.transformControls.enabled = false;
    }
  }, [selectedMesh, selectedPivot]);

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    if (gizmoMode && ctx.transformControls.enabled) {
      ctx.transformControls.setMode(gizmoMode);
    }
  }, [gizmoMode]);

  const onTransformRef = useRef(onTransform);
  onTransformRef.current = onTransform;
  const onPickRef = useRef(onPick);
  onPickRef.current = onPick;
  const onHoverRef = useRef(onHover);
  onHoverRef.current = onHover;
  const onSectionOffsetChangeRef = useRef(onSectionOffsetChange);
  onSectionOffsetChangeRef.current = onSectionOffsetChange;

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    ctx._onPick = (...args) => onPickRef.current?.(...args);
    ctx._onHover = (...args) => onHoverRef.current?.(...args);
    ctx._onSectionOffset = (...args) => onSectionOffsetChangeRef.current?.(...args);
  });

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;

    function onDragEnd() {
      const cb = onTransformRef.current;
      if (!cb) return;
      const pivot = [ctx.anchor.position.x, ctx.anchor.position.y, ctx.anchor.position.z];
      const mode = ctx.transformControls.mode;

      if (mode === 'translate') {
        const startPivot = sceneRef.current?._startPivot;
        if (startPivot) {
          cb({
            mode: 'translate',
            delta: [pivot[0] - startPivot[0], pivot[1] - startPivot[1], pivot[2] - startPivot[2]],
            pivot: startPivot,
          });
        }
      } else if (mode === 'rotate') {
        const q = ctx.anchor.quaternion;
        const angle = 2 * Math.acos(Math.min(1, Math.abs(q.w)));
        if (angle > 1e-6) {
          const s = Math.sin(angle / 2);
          const axis = [q.x / s, q.y / s, q.z / s];
          cb({ mode: 'rotate', axis, angle, pivot });
        }
      } else if (mode === 'scale') {
        const sc = ctx.anchor.scale;
        if (Math.abs(sc.x - 1) > 1e-6 || Math.abs(sc.y - 1) > 1e-6 || Math.abs(sc.z - 1) > 1e-6) {
          cb({ mode: 'scale', factors: [sc.x, sc.y, sc.z], pivot });
        }
      }

      ctx.anchor.quaternion.identity();
      ctx.anchor.scale.set(1, 1, 1);
    }

    function onDragStart() {
      const pos = ctx.anchor.position;
      sceneRef.current._startPivot = [pos.x, pos.y, pos.z];
    }

    ctx.transformControls.addEventListener('mouseDown', onDragStart);
    ctx.transformControls.addEventListener('mouseUp', onDragEnd);
    return () => {
      ctx.transformControls.removeEventListener('mouseDown', onDragStart);
      ctx.transformControls.removeEventListener('mouseUp', onDragEnd);
    };
  }, []);

  return (
    <div className="viewport">
      <div className="viewport-canvas" ref={containerRef} />
      <ViewTriad quat={triadQuat} onSelectView={snapView} />
    </div>
  );
});

export default Viewport3D;
