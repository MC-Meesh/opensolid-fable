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

// Selection ghost: solid accent tint. Hover ghost: fainter neutral wash so
// the two states read differently at a glance.
const HIGHLIGHT_MATERIAL = new THREE.MeshStandardMaterial({
  color: 0x4fc3f7,
  metalness: 0.1,
  roughness: 0.4,
  transparent: true,
  opacity: 0.35,
  depthWrite: false,
});

const HOVER_MATERIAL = new THREE.MeshStandardMaterial({
  color: 0xdde6f2,
  metalness: 0.05,
  roughness: 0.6,
  transparent: true,
  opacity: 0.16,
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
    hoverMesh,
    previewMesh,
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

    const material = new THREE.MeshStandardMaterial({
      color: 0x5f9ee8,
      metalness: 0.15,
      roughness: 0.5,
    });
    const meshObject = new THREE.Mesh(new THREE.BufferGeometry(), material);
    scene.add(meshObject);

    const hoverObject = new THREE.Mesh(new THREE.BufferGeometry(), HOVER_MATERIAL);
    hoverObject.renderOrder = 1;
    hoverObject.visible = false;
    scene.add(hoverObject);

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
      return hits.length > 0 ? hits[0].point : null;
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
      const p = castAt(event.clientX, event.clientY);
      // A miss clicks empty space: deselect.
      cb(p ? [p.x, p.y, p.z] : null);
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
        const p = castAt(hoverX, hoverY);
        cb(p ? [p.x, p.y, p.z] : null);
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
      hoverObject,
      previewObject,
      cameraAnim: null,
      sketchOrtho: false,
      savedView: null,
      activeSketchPlane: null,
      _onPick: null,
      _onHover: null,
      _onTransform: null,
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
      orbitControls.dispose();
      meshObject.geometry.dispose();
      ghostMesh.geometry.dispose();
      hoverObject.geometry.dispose();
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
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute('position', new THREE.BufferAttribute(mesh.positions, 3));
    geometry.setAttribute('normal', new THREE.BufferAttribute(mesh.normals, 3));
    geometry.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
    ctx.meshObject.geometry.dispose();
    ctx.meshObject.geometry = geometry;
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

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    if (hoverMesh) {
      const geo = new THREE.BufferGeometry();
      geo.setAttribute('position', new THREE.BufferAttribute(hoverMesh.positions, 3));
      geo.setAttribute('normal', new THREE.BufferAttribute(hoverMesh.normals, 3));
      geo.setIndex(new THREE.BufferAttribute(hoverMesh.indices, 1));
      ctx.hoverObject.geometry.dispose();
      ctx.hoverObject.geometry = geo;
      ctx.hoverObject.visible = true;
    } else {
      ctx.hoverObject.visible = false;
    }
  }, [hoverMesh]);

  // SolidWorks-style "Normal To": entering sketch mode (or switching planes
  // mid-sketch) flies the camera orthogonal to the sketch plane and switches
  // to orthographic projection; exiting restores the saved perspective view.
  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    const { camera, controls } = ctx;
    ctx.activeSketchPlane = sketchPlane && SKETCH_PLANES[sketchPlane] ? sketchPlane : null;
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
    if (!ctx || !sketchPlane) return undefined;
    const spec = SKETCH_PLANES[sketchPlane];
    if (!spec) return undefined;

    const scene = ctx.meshObject.parent;
    const group = new THREE.Group();
    group.rotation.set(...spec.rotation);

    // Size the indicator from the scene bounds so drawn geometry never
    // outruns it; it is a visual aid, not a physical object.
    const meshGeometry = ctx.meshObject.geometry;
    if (!meshGeometry.boundingSphere) meshGeometry.computeBoundingSphere();
    const sphere = meshGeometry.boundingSphere;
    const hasBounds =
      sphere && Number.isFinite(sphere.radius) && sphere.radius > 0;
    const size = planeIndicatorSize(
      hasBounds ? sphere.center.toArray() : [0, 0, 0],
      hasBounds ? sphere.radius : 0
    );

    const fillGeometry = new THREE.PlaneGeometry(size, size);
    const fillMaterial = new THREE.MeshBasicMaterial({
      color: spec.color,
      transparent: true,
      opacity: 0.12,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
    group.add(new THREE.Mesh(fillGeometry, fillMaterial));

    const edgeGeometry = new THREE.EdgesGeometry(fillGeometry);
    const edgeMaterial = new THREE.LineBasicMaterial({
      color: spec.color,
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

  useEffect(() => {
    const ctx = sceneRef.current;
    if (!ctx) return;
    ctx._onPick = (...args) => onPickRef.current?.(...args);
    ctx._onHover = (...args) => onHoverRef.current?.(...args);
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
