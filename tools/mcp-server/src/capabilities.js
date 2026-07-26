// The machine-readable capability manifest: every MCP tool and every script
// operation, with signatures, in one JSON payload.
//
// Why this exists (of-2y4.4): the script DSL hands a script the *whole*
// `WasmShape` class, but the prose docs described a subset — so `cone`,
// `halfSpace`, `rib`, `loft`, `taper`, `shell`, `filletEdge`, `chamferEdge`,
// `linearPattern`, `circularPattern`, `mirror`, `distance`, `normalAt` and
// `bounds` were all callable and none of them were written down. A cold agent
// could only discover them by reading Rust. The prose is now fixed too, but
// prose drifts; this inventory is the machine-readable one an agent can pull
// at runtime (`get_capabilities`), and `test/capabilities.test.js` diffs it
// against the actual bound classes in both directions, so an op added to the
// kernel without a manifest entry — or an entry naming an op that no longer
// exists — fails the build rather than becoming the next hidden surface.
//
// Every entry carries a `kind`, so a client can present the surface without
// hand-classifying it:
//   primitive | feature | transform | boolean | blend | pattern | query
//   | builder | internal
// `internal` marks members the server itself drives (meshing, measurement,
// export). They are callable from a script, but a script should return a Shape
// and let the tools do the measuring.

/**
 * Identity this server reports in `initialize` and in the manifest. Kept here
 * so the two can never disagree.
 */
export const SERVER_INFO = { name: 'opensolid-mcp-server', version: '0.1.0' };

/** Script bindings, in the order `runScript` supplies them. */
export const SCRIPT_BINDINGS = ['Shape', 'Profile', 'Path', 'OpenPath', 'param'];

/** Document-unit keys `export` accepts (see docs/units.md). */
export const UNITS = [
  { key: 'mm', label: 'millimetre', step: 'SI_UNIT(.MILLI.,.METRE.)' },
  { key: 'cm', label: 'centimetre', step: 'SI_UNIT(.CENTI.,.METRE.)' },
  { key: 'm', label: 'metre', step: 'SI_UNIT($,.METRE.)' },
  { key: 'in', label: 'inch', step: "CONVERSION_BASED_UNIT('INCH', 25.4 mm)" },
];

/** The conventions that silently produce wrong parts when assumed wrong. */
export const CONVENTIONS = {
  axis:
    'Sweeps run along +Y: `cylinder` is radial in xz and axial in y, `extrude` ' +
    'maps a profile (u,v) to world (x,z) and sweeps +Y, `revolve` turns about Y, ' +
    '`torus` rings in xz. STEP/FreeCAD are z-up and coordinates are written ' +
    'verbatim, so a plate modelled flat in xy needs `.rotate(1,0,0,90)` on its ' +
    'through-holes. Rotating a shape about the axis it already lies on is a no-op, ' +
    'and a hole on the wrong axis still reports valid:true — only `measure` ' +
    'against a hand-computed volume catches it.',
  sizes:
    'Box/cylinder/torus/cone arguments are half-extents and half-heights, and the ' +
    'shape is centred on the origin: box3(hx,hy,hz) is 2hx x 2hy x 2hz.',
  angles: 'Every angle argument is in degrees.',
  units:
    'The kernel is unitless — coordinates are bare numbers. The document unit is ' +
    'metadata: `export` declares it in the STEP header and never rescales geometry.',
  immutability: 'Every op returns a new Shape; nothing mutates in place.',
  errors:
    'Kernel failures come back as MCP results with isError:true and a message; ' +
    'they never throw across the wire. Branch on isError and read the text.',
};

/**
 * `Shape` — the solid. Static constructors build one, methods derive a new one.
 *
 * `exactCompanion: true` means the op can carry an analytic B-Rep alongside the
 * SDF, so `exact: true` models keep crisp edges and export analytic STEP.
 * Everything else is SDF-only and exports a faceted-but-valid B-Rep.
 */
const SHAPE_STATICS = [
  {
    name: 'sphere',
    kind: 'primitive',
    signature: 'Shape.sphere(radius)',
    params: [{ name: 'radius', type: 'number' }],
    exactCompanion: true,
    doc: 'Sphere centred at the origin.',
  },
  {
    name: 'box3',
    kind: 'primitive',
    signature: 'Shape.box3(hx, hy, hz)',
    params: [
      { name: 'hx', type: 'number' },
      { name: 'hy', type: 'number' },
      { name: 'hz', type: 'number' },
    ],
    exactCompanion: true,
    doc: 'Axis-aligned box of half-extents (hx,hy,hz), centred at the origin.',
  },
  {
    name: 'roundedBox',
    kind: 'primitive',
    signature: 'Shape.roundedBox(hx, hy, hz, radius)',
    params: [
      { name: 'hx', type: 'number' },
      { name: 'hy', type: 'number' },
      { name: 'hz', type: 'number' },
      { name: 'radius', type: 'number', doc: 'edge radius, <= the smallest half-extent' },
    ],
    exactCompanion: false,
    doc: 'Box with every edge rounded. Half-extents include the rounding.',
  },
  {
    name: 'cylinder',
    kind: 'primitive',
    signature: 'Shape.cylinder(radius, halfHeight)',
    params: [
      { name: 'radius', type: 'number' },
      { name: 'halfHeight', type: 'number' },
    ],
    exactCompanion: true,
    doc: 'Cylinder along +Y: radial in xz, y in +/-halfHeight.',
  },
  {
    name: 'torus',
    kind: 'primitive',
    signature: 'Shape.torus(majorRadius, minorRadius)',
    params: [
      { name: 'majorRadius', type: 'number' },
      { name: 'minorRadius', type: 'number' },
    ],
    exactCompanion: true,
    doc: 'Torus with its ring in the xz plane, centred at the origin.',
  },
  {
    name: 'cone',
    kind: 'primitive',
    signature: 'Shape.cone(radiusBottom, radiusTop, halfHeight)',
    params: [
      { name: 'radiusBottom', type: 'number', doc: 'radius at y = -halfHeight' },
      { name: 'radiusTop', type: 'number', doc: 'radius at y = +halfHeight' },
      { name: 'halfHeight', type: 'number' },
    ],
    exactCompanion: true,
    doc:
      'Cone or frustum along +Y. Either radius may be zero for a pointed tip ' +
      '(not both) — Shape.cone(r, 0, hh) is the cone, Shape.cone(r1, r2, hh) the frustum.',
  },
  {
    name: 'capsule',
    kind: 'primitive',
    signature: 'Shape.capsule(x1, y1, z1, x2, y2, z2, radius)',
    params: [
      { name: 'x1', type: 'number' },
      { name: 'y1', type: 'number' },
      { name: 'z1', type: 'number' },
      { name: 'x2', type: 'number' },
      { name: 'y2', type: 'number' },
      { name: 'z2', type: 'number' },
      { name: 'radius', type: 'number' },
    ],
    exactCompanion: false,
    doc: 'Sphere swept along the segment from (x1,y1,z1) to (x2,y2,z2).',
  },
  {
    name: 'halfSpace',
    kind: 'primitive',
    signature: 'Shape.halfSpace(px, py, pz, nx, ny, nz)',
    params: [
      { name: 'px', type: 'number' },
      { name: 'py', type: 'number' },
      { name: 'pz', type: 'number' },
      { name: 'nx', type: 'number' },
      { name: 'ny', type: 'number' },
      { name: 'nz', type: 'number' },
    ],
    exactCompanion: false,
    doc:
      'The solid half on the negative side of the plane through (px,py,pz) with ' +
      'outward normal (nx,ny,nz). Unbounded on its own: intersect it with a ' +
      'through-all extrude to get an "up to face" terminator.',
  },
  {
    name: 'extrude',
    kind: 'feature',
    signature: 'Shape.extrude(profile, height, draftDegrees?)',
    params: [
      { name: 'profile', type: 'Profile' },
      { name: 'height', type: 'number', doc: 'swept along +Y from y=0 to y=height' },
      {
        name: 'draftDegrees',
        type: 'number',
        required: false,
        doc: 'positive narrows toward the top cap (mould-release draft); |draft| < ~80',
      },
    ],
    exactCompanion: false,
    doc: 'Closed profile swept along +Y. Profile (x,y) maps to world (x,z).',
  },
  {
    name: 'revolve',
    kind: 'feature',
    signature: 'Shape.revolve(profile, angleDegrees)',
    params: [
      { name: 'profile', type: 'Profile', doc: 'must lie in x >= 0' },
      { name: 'angleDegrees', type: 'number', doc: 'in (0, 360]' },
    ],
    exactCompanion: false,
    doc:
      'Closed profile revolved about the Y axis, sweeping from the +X half-plane ' +
      'toward +Z. Profile (x,y) maps to (radius, y).',
  },
  {
    name: 'sweep',
    kind: 'feature',
    signature: 'Shape.sweep(profile, path)',
    params: [
      { name: 'profile', type: 'Profile' },
      { name: 'path', type: 'Path', doc: 'a 3D polyline; build it with `new Path(x,y,z)`' },
    ],
    exactCompanion: false,
    doc:
      'Closed profile swept along a 3D polyline, twist-free along each segment; ' +
      'joints are mitred by the union of the per-segment prisms. Constant profile, no twist.',
  },
  {
    name: 'loft',
    kind: 'feature',
    signature: 'Shape.loft(bottom, top, height)',
    params: [
      { name: 'bottom', type: 'Profile', doc: 'profile on y = 0' },
      { name: 'top', type: 'Profile', doc: 'profile on y = height' },
      { name: 'height', type: 'number' },
    ],
    exactCompanion: false,
    doc:
      'Blend between two closed profiles on parallel planes by linearly morphing ' +
      'their signed distances along y. Parallel planes only, linear morph.',
  },
  {
    name: 'rib',
    kind: 'feature',
    signature: 'Shape.rib(path, thickness, height, side)',
    params: [
      { name: 'path', type: 'OpenPath', doc: 'an open 2D polyline; `new OpenPath(x,y)`' },
      { name: 'thickness', type: 'number' },
      { name: 'height', type: 'number', doc: 'swept along +Y from y=0 to y=height' },
      {
        name: 'side',
        type: 'string',
        enum: ['both', 'first', 'second'],
        doc:
          '"both" puts thickness/2 each way (exact distance), "first" the full ' +
          'thickness left of the path direction, "second" the full thickness right',
      },
    ],
    exactCompanion: false,
    doc:
      'Open path thickened into a support rib and swept along +Y; path (x,y) maps ' +
      'to world (x,z). Union the result with the parent body yourself.',
  },
];

const SHAPE_METHODS = [
  {
    name: 'translate',
    kind: 'transform',
    signature: 'shape.translate(x, y, z)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
      { name: 'z', type: 'number' },
    ],
    exactCompanion: true,
    doc: 'This shape moved by (x,y,z).',
  },
  {
    name: 'rotate',
    kind: 'transform',
    signature: 'shape.rotate(ax, ay, az, angleDegrees)',
    params: [
      { name: 'ax', type: 'number' },
      { name: 'ay', type: 'number' },
      { name: 'az', type: 'number' },
      { name: 'angleDegrees', type: 'number' },
    ],
    exactCompanion: true,
    doc:
      'Rotated about the origin by angleDegrees around the axis (ax,ay,az) (any ' +
      'non-zero length). Rotating about the axis the shape already lies on is a no-op.',
  },
  {
    name: 'scale',
    kind: 'transform',
    signature: 'shape.scale(sx, sy, sz)',
    params: [
      { name: 'sx', type: 'number' },
      { name: 'sy', type: 'number' },
      { name: 'sz', type: 'number' },
    ],
    exactCompanion: false,
    doc:
      'Per-axis scale about the origin (each factor > 0). Booleans and meshing stay ' +
      'correct, but the field is no longer an exact distance, so later smooth-blend ' +
      'radii are distorted — prefer uniformScale when the factors are equal.',
  },
  {
    name: 'uniformScale',
    kind: 'transform',
    signature: 'shape.uniformScale(factor)',
    params: [{ name: 'factor', type: 'number', doc: '> 0' }],
    exactCompanion: true,
    doc: 'Uniform scale about the origin.',
  },
  {
    name: 'taper',
    kind: 'transform',
    signature: 'shape.taper(px, py, pz, nx, ny, nz, angleDegrees)',
    params: [
      { name: 'px', type: 'number', doc: 'pull direction x' },
      { name: 'py', type: 'number', doc: 'pull direction y' },
      { name: 'pz', type: 'number', doc: 'pull direction z' },
      { name: 'nx', type: 'number', doc: 'neutral-plane point x' },
      { name: 'ny', type: 'number', doc: 'neutral-plane point y' },
      { name: 'nz', type: 'number', doc: 'neutral-plane point z' },
      { name: 'angleDegrees', type: 'number' },
    ],
    exactCompanion: false,
    doc:
      'Mould-release draft about a parting plane: side walls flare toward the pull ' +
      'direction above the neutral plane and pinch below it. Whole-body — this is ' +
      "the F-Rep approximation of a face-selective draft. Note the argument order: " +
      'pull direction first, then the point on the neutral plane.',
  },
  {
    name: 'union',
    kind: 'boolean',
    signature: 'a.union(other)',
    params: [{ name: 'other', type: 'Shape' }],
    exactCompanion: true,
    doc: 'Boolean union.',
  },
  {
    name: 'subtract',
    kind: 'boolean',
    signature: 'a.subtract(other)',
    params: [{ name: 'other', type: 'Shape' }],
    exactCompanion: true,
    doc: 'Boolean subtraction of b from a.',
  },
  {
    name: 'intersect',
    kind: 'boolean',
    signature: 'a.intersect(other)',
    params: [{ name: 'other', type: 'Shape' }],
    exactCompanion: true,
    doc: 'Boolean intersection.',
  },
  {
    name: 'smoothUnion',
    kind: 'blend',
    signature: 'a.smoothUnion(other, radius?)',
    params: [
      { name: 'other', type: 'Shape' },
      {
        name: 'radius',
        type: 'number',
        required: false,
        doc: "omitted picks 10% of the combined bounding box's largest extent",
      },
    ],
    exactCompanion: false,
    doc: 'Union with a smooth (organic) blend of the given radius.',
  },
  {
    name: 'filletEdge',
    kind: 'blend',
    signature: 'a.filletEdge(other, radius, edge)',
    params: [
      { name: 'other', type: 'Shape' },
      { name: 'radius', type: 'number' },
      {
        name: 'edge',
        type: 'number[]',
        doc: 'flat [x0,y0,z0, x1,y1,z1, …] polyline of the picked feature edge',
      },
    ],
    exactCompanion: false,
    doc:
      'Edge-selective fillet: a rounded blend applied only along the selected edge of ' +
      'the union of a and b; every other edge stays sharp. Unlike smoothUnion, which ' +
      'blends the whole intersection curve.',
  },
  {
    name: 'chamferEdge',
    kind: 'blend',
    signature: 'a.chamferEdge(other, setback, edge)',
    params: [
      { name: 'other', type: 'Shape' },
      { name: 'setback', type: 'number' },
      { name: 'edge', type: 'number[]', doc: 'flat [x0,y0,z0, …] polyline' },
    ],
    exactCompanion: false,
    doc: 'Edge-selective chamfer: a planar bevel along the selected edge of the union.',
  },
  {
    name: 'shell',
    kind: 'feature',
    signature: 'shape.shell(thickness)',
    params: [{ name: 'thickness', type: 'number', doc: 'total wall, positive and finite' }],
    exactCompanion: false,
    doc:
      'Hollow into a shell of total wall thickness, centred on the surface ' +
      '(thickness/2 each side) — so the outer extent grows by thickness/2. ' +
      'Closed all round: intersect or subtract to open a face.',
  },
  {
    name: 'linearPattern',
    kind: 'pattern',
    signature: 'shape.linearPattern(dx, dy, dz, count)',
    params: [
      { name: 'dx', type: 'number' },
      { name: 'dy', type: 'number' },
      { name: 'dz', type: 'number' },
      { name: 'count', type: 'number', doc: 'rounded to the nearest integer, >= 1' },
    ],
    exactCompanion: false,
    doc: 'count copies, copy k translated by k*(dx,dy,dz). Copy 0 is the original.',
  },
  {
    name: 'circularPattern',
    kind: 'pattern',
    signature: 'shape.circularPattern(ax, ay, az, cx, cy, cz, count, angleDegrees?)',
    params: [
      { name: 'ax', type: 'number', doc: 'axis direction x' },
      { name: 'ay', type: 'number', doc: 'axis direction y' },
      { name: 'az', type: 'number', doc: 'axis direction z' },
      { name: 'cx', type: 'number', doc: 'axis point x' },
      { name: 'cy', type: 'number', doc: 'axis point y' },
      { name: 'cz', type: 'number', doc: 'axis point z' },
      { name: 'count', type: 'number', doc: 'rounded to the nearest integer, >= 1' },
      { name: 'angleDegrees', type: 'number', required: false, doc: 'total span, default 360' },
    ],
    exactCompanion: false,
    doc:
      'count copies spaced evenly around the axis through (cx,cy,cz); consecutive ' +
      'copies differ by angleDegrees/count. Note the order: axis direction, then axis point.',
  },
  {
    name: 'mirror',
    kind: 'pattern',
    signature: 'shape.mirror(nx, ny, nz, px, py, pz)',
    params: [
      { name: 'nx', type: 'number', doc: 'plane normal x' },
      { name: 'ny', type: 'number', doc: 'plane normal y' },
      { name: 'nz', type: 'number', doc: 'plane normal z' },
      { name: 'px', type: 'number', doc: 'plane point x' },
      { name: 'py', type: 'number', doc: 'plane point y' },
      { name: 'pz', type: 'number', doc: 'plane point z' },
    ],
    exactCompanion: false,
    doc:
      'This shape unioned with its reflection across the plane — a mirrored *copy*, ' +
      'not a reflection in place. Note the order: normal first, then the plane point.',
  },
  {
    name: 'distance',
    kind: 'query',
    signature: 'shape.distance(x, y, z)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
      { name: 'z', type: 'number' },
    ],
    returns: 'number',
    doc:
      'Signed distance to the surface: negative inside, positive outside. After ' +
      'smooth blends or anisotropic scaling it is not an exact Euclidean distance, ' +
      'but the sign and zero set stay correct — so it answers "is this point inside?" ' +
      'and "which of these points is nearer?" in-script, without a round trip.',
  },
  {
    name: 'normalAt',
    kind: 'query',
    signature: 'shape.normalAt(x, y, z)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
      { name: 'z', type: 'number' },
    ],
    returns: 'number[3]',
    doc: 'Outward unit surface normal [nx,ny,nz] at the point (the normalized field gradient).',
  },
  {
    name: 'bounds',
    kind: 'query',
    signature: 'shape.bounds()',
    params: [],
    returns: 'number[6]',
    doc:
      'Tracked bounding box as [minX,minY,minZ, maxX,maxY,maxZ]. Conservative — it ' +
      'encloses the surface and can overstate a blended or repeatedly-rotated part. ' +
      "For the part's real extent use the measured boundingBox from `measure`.",
  },
  {
    name: 'isExact',
    kind: 'query',
    signature: 'shape.isExact()',
    params: [],
    returns: 'boolean',
    doc: 'Whether this shape will serve a validated exact B-Rep tessellation.',
  },
];

// Callable from a script, but the server drives them: a script should return a
// Shape and let create_model/measure/validate/export do the rest. Listed so the
// inventory is total — nothing about the surface is undisclosed.
const SHAPE_INTERNAL = [
  {
    name: 'constructor',
    doc:
      'wasm-bindgen’s default constructor. A Shape has no public constructor — ' +
      'build one with a static (Shape.box3, Shape.extrude, …).',
  },
  { name: 'setExactBooleans', static: true, doc: 'create_model’s `exact` flag sets this.' },
  { name: 'mesh', doc: 'Fixed-resolution mesh; the server meshes adaptively instead.' },
  { name: 'meshAdaptive', doc: 'Adaptive mesh behind every tool’s `accuracy` argument.' },
  { name: 'measure', doc: 'Backs the `measure` tool.' },
  {
    name: 'validate',
    doc: 'Backs the `validate` tool and create_model’s valid/issues; takes (accuracy, deep).',
  },
  {
    name: 'brepCheck',
    doc:
      'The kernel’s B-Rep body validation plus that body’s entity census, folded into ' +
      '`validate` and `inspect_topology`. Reports available:false with a reason when the ' +
      'shape has no exact B-Rep.',
  },
  {
    name: 'meshAgreement',
    doc:
      'Runs every mesher (uniform grid, adaptive SDF, faceted-STEP recovery, exact ' +
      'tessellation) and reports whether they agree the shape is a closed solid.',
  },
  { name: 'exportStep', doc: 'Backs `export` with format "step"; takes (accuracy, unit).' },
  { name: 'silhouetteEdges', doc: 'View-dependent outline edges, used by the renderer.' },
  { name: 'fieldMeasure', doc: 'Field-quadrature measurement used by `optimize`.' },
  { name: 'fieldClearance', doc: 'Keep-out clearance term used by `optimize` constraints.' },
  { name: 'free', doc: 'wasm-bindgen memory release; the runtime handles it.' },
];

const PROFILE_METHODS = [
  {
    name: 'constructor',
    kind: 'builder',
    signature: 'new Profile(x, y)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
    ],
    doc: 'Start a closed profile at (x,y).',
  },
  {
    name: 'lineTo',
    kind: 'builder',
    signature: 'p.lineTo(x, y)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
    ],
    doc: 'Straight segment to (x,y).',
  },
  {
    name: 'arcTo',
    kind: 'builder',
    signature: 'p.arcTo(x, y, bulge)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
      { name: 'bulge', type: 'number', doc: 'tan(theta/4) of the swept angle; 0 is straight' },
    ],
    doc: 'Circular-arc segment to (x,y).',
  },
  {
    name: 'ellipseArcTo',
    kind: 'builder',
    signature: 'p.ellipseArcTo(x, y, cx, cy, rx, ry, rotationDegrees, ccw)',
    params: [
      { name: 'x', type: 'number', doc: 'endpoint x' },
      { name: 'y', type: 'number', doc: 'endpoint y' },
      { name: 'cx', type: 'number', doc: 'ellipse centre x' },
      { name: 'cy', type: 'number', doc: 'ellipse centre y' },
      { name: 'rx', type: 'number', doc: 'semi-axis along the rotated x' },
      { name: 'ry', type: 'number', doc: 'semi-axis along the rotated y' },
      { name: 'rotationDegrees', type: 'number' },
      { name: 'ccw', type: 'boolean', doc: 'true sweeps counter-clockwise, false clockwise' },
    ],
    doc:
      'Elliptical-arc segment. Centre-parameterised, not SVG endpoint form: both the ' +
      'current point and (x,y) must lie on the named ellipse.',
  },
  {
    name: 'cubicTo',
    kind: 'builder',
    signature: 'p.cubicTo(c1x, c1y, c2x, c2y, x, y)',
    params: [
      { name: 'c1x', type: 'number' },
      { name: 'c1y', type: 'number' },
      { name: 'c2x', type: 'number' },
      { name: 'c2y', type: 'number' },
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
    ],
    doc: 'Cubic Bézier segment to (x,y) with the two control points.',
  },
  {
    name: 'close',
    kind: 'builder',
    signature: 'p.close()',
    params: [],
    doc: 'Close the loop back to the start. Required before extrude/revolve/loft.',
  },
];

const OPEN_PATH_METHODS = [
  {
    name: 'constructor',
    kind: 'builder',
    signature: 'new OpenPath(x, y)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
    ],
    doc: 'Start an open 2D polyline at (x,y). Consumed by Shape.rib — never closed.',
  },
  {
    name: 'lineTo',
    kind: 'builder',
    signature: 'p.lineTo(x, y)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
    ],
    doc: 'Straight segment to (x,y).',
  },
  {
    name: 'arcTo',
    kind: 'builder',
    signature: 'p.arcTo(x, y, bulge)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
      { name: 'bulge', type: 'number', doc: 'tan(theta/4); 0 is straight' },
    ],
    doc: 'Circular-arc segment to (x,y).',
  },
  {
    name: 'ellipseArcTo',
    kind: 'builder',
    signature: 'p.ellipseArcTo(x, y, cx, cy, rx, ry, rotationDegrees, ccw)',
    params: [
      { name: 'x', type: 'number', doc: 'endpoint x' },
      { name: 'y', type: 'number', doc: 'endpoint y' },
      { name: 'cx', type: 'number', doc: 'ellipse centre x' },
      { name: 'cy', type: 'number', doc: 'ellipse centre y' },
      { name: 'rx', type: 'number' },
      { name: 'ry', type: 'number' },
      { name: 'rotationDegrees', type: 'number' },
      { name: 'ccw', type: 'boolean', doc: 'true sweeps counter-clockwise' },
    ],
    doc: 'Elliptical-arc segment; centre-parameterised (see Profile.ellipseArcTo).',
  },
  {
    name: 'cubicTo',
    kind: 'builder',
    signature: 'p.cubicTo(c1x, c1y, c2x, c2y, x, y)',
    params: [
      { name: 'c1x', type: 'number' },
      { name: 'c1y', type: 'number' },
      { name: 'c2x', type: 'number' },
      { name: 'c2y', type: 'number' },
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
    ],
    doc: 'Cubic Bézier segment to (x,y).',
  },
];

const PATH_METHODS = [
  {
    name: 'constructor',
    kind: 'builder',
    signature: 'new Path(x, y, z)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
      { name: 'z', type: 'number' },
    ],
    doc: 'Start a 3D polyline at (x,y,z). Consumed by Shape.sweep.',
  },
  {
    name: 'lineTo',
    kind: 'builder',
    signature: 'path.lineTo(x, y, z)',
    params: [
      { name: 'x', type: 'number' },
      { name: 'y', type: 'number' },
      { name: 'z', type: 'number' },
    ],
    doc: 'Straight segment to (x,y,z). Straight segments only — no arcs in 3D yet.',
  },
];

const PARAM_BINDING = {
  name: 'param',
  kind: 'builder',
  signature: "param(name, default, { min, max })",
  params: [
    { name: 'name', type: 'string', doc: 'unique within the script' },
    { name: 'default', type: 'number', doc: 'the value the model builds at' },
    {
      name: 'min',
      type: 'number',
      required: false,
      doc: 'lower bound; min and max are both-or-neither',
    },
    { name: 'max', type: 'number', required: false, doc: 'upper bound' },
  ],
  returns: 'number',
  doc:
    'Declare a design variable and return the value to use. The model builds at the ' +
    'default, and `optimize` may then move the number within its bounds. Bounds are ' +
    'required somewhere — the declaration or the optimize call — before a param can move.',
};

// Every builder class carries wasm-bindgen's memory-release method. The
// runtime handles it; a script never calls it.
const BUILDER_INTERNAL = [
  { name: 'free', doc: 'wasm-bindgen memory release; the runtime handles it.' },
];

/** The script surface, keyed by binding name. */
export const SCRIPT_API = {
  Shape: { statics: SHAPE_STATICS, methods: SHAPE_METHODS, internal: SHAPE_INTERNAL },
  Profile: { statics: [], methods: PROFILE_METHODS, internal: BUILDER_INTERNAL },
  Path: { statics: [], methods: PATH_METHODS, internal: BUILDER_INTERNAL },
  OpenPath: { statics: [], methods: OPEN_PATH_METHODS, internal: BUILDER_INTERNAL },
  param: PARAM_BINDING,
};

/**
 * Assemble the full manifest: MCP tools (their real `inputSchema`s, so the
 * document can never disagree with what the server accepts) plus the script
 * inventory above.
 *
 * @param {{name:string, version:string}} server
 * @param {Array<object>} toolDefinitions the live `tools/list` definitions
 */
export function buildManifest(server, toolDefinitions) {
  return {
    server,
    conventions: CONVENTIONS,
    units: UNITS,
    tools: toolDefinitions.map((d) => ({
      name: d.name,
      description: d.description,
      inputSchema: d.inputSchema,
    })),
    script: {
      contract:
        'create_model takes a JavaScript function body (not a module) that must ' +
        'return a Shape. Strict mode, no imports, no require, no filesystem or ' +
        'network. Bindings in scope: ' +
        SCRIPT_BINDINGS.join(', ') +
        '.',
      bindings: SCRIPT_BINDINGS,
      api: SCRIPT_API,
    },
  };
}
