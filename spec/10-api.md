# 10 — Public API

The Rust API surface, C FFI, Python bindings, and AI-friendly design patterns.

**Python bindings are non-negotiable.** The majority of CAD/CAE users script in Python.
PyO3 bindings are a first-class deliverable — they are not a "future" or "nice-to-have"
layer. Every public Rust API function gets a Python equivalent.

**Timing:** Python bindings ship at component **stabilization** (4 weeks without API
signature changes), not at component creation. This prevents the engineering tax of
maintaining dual-language bindings during exploration-phase work. The overall commitment
is unchanged: before any external release, Python bindings cover all public APIs.

## 1. Design Principles

### 1.1 Inspired by Parasolid's PK API

Parasolid's API has 900+ functions organized by entity class (PK_BODY_*, PK_FACE_*, etc.).
We follow the same entity-centric organization but with Rust idioms:

- Options structs instead of long parameter lists
- Result types instead of error codes
- Rich error enums instead of integer fault codes

### 1.2 AI-Friendly Design

Every API decision is evaluated against: "Can an LLM agent use this correctly?"

- **Deterministic**: Same inputs → same outputs. No randomized algorithms.
- **Discoverable**: Type system guides correct usage. Invalid combinations are unrepresentable.
- **Diagnosable**: Errors explain what failed, why, and what to try instead.
- **Introspectable**: Every entity can be queried for all its properties.
- **Composable**: Small operations combine predictably.

### 1.3 Two API Layers

The kernel exposes two complementary APIs:

1. **Low-level (Kernel methods)**: Direct entity manipulation, full control, for power users
   and internal operations.
2. **High-level (Builder API)**: Fluent, chainable interface for common workflows. This is
   what most users (and AI agents) interact with. See Section 9.

## 2. Kernel API (High-Level)

```rust
use opensolid::prelude::*;

// === Kernel Lifecycle ===

let mut kernel = Kernel::new();
let kernel = Kernel::with_config(KernelConfig {
    tolerance: ToleranceConfig::default(),
    max_history: 100,
    journal_path: None,
});

// === Primitive Creation ===

let block = kernel.make_block(10.0, 20.0, 30.0)?;
let cylinder = kernel.make_cylinder(5.0, 40.0)?;
let sphere = kernel.make_sphere(15.0)?;
let cone = kernel.make_cone(10.0, 5.0, 25.0)?;  // bottom_r, top_r, height
let torus = kernel.make_torus(20.0, 5.0)?;       // major_r, minor_r

// Position primitives
kernel.transform(cylinder, &Transform3::translation(5.0, 10.0, 0.0))?;
kernel.transform(cylinder, &Transform3::rotation(
    UnitVector3::Z, std::f64::consts::FRAC_PI_4
))?;

// === Boolean Operations ===

let result = kernel.unite(block, cylinder)?;
let result = kernel.subtract(block, cylinder)?;
let result = kernel.intersect(block, cylinder)?;

// With options
let result = kernel.boolean(block, cylinder, &BooleanOptions {
    operation: BooleanOp::Subtract,
    tolerance: 1e-6,
    keep_input_bodies: false,
    check_result: true,
    merge_coincident: true,
    simplify: true,
})?;

// === Blending ===

let edges = kernel.body_edges(result);
let target_edges = edges.iter()
    .filter(|e| kernel.edge_length(**e) > 5.0)
    .collect::<Vec<_>>();

kernel.fillet_edges(&target_edges, &FilletOptions {
    radius: 2.0,
    corner_type: CornerType::Setback,
    propagate: true,
    overflow: OverflowBehavior::Fail,
})?;

kernel.chamfer_edges(&[edge_id], &ChamferOptions {
    chamfer_type: ChamferType::Symmetric(1.5),
})?;

// === Sweeping ===

// Extrude a face
let extruded = kernel.extrude(face_id, &ExtrudeOptions {
    direction: Vector3::new(0.0, 0.0, 1.0),
    distance: 15.0,
    draft_angle: Some(3.0_f64.to_radians()),
    cap: true,
})?;

// Revolve
let revolved = kernel.revolve(profile_face, &RevolveOptions {
    axis_origin: Point3::ORIGIN,
    axis_direction: UnitVector3::Y,
    angle: std::f64::consts::TAU,  // Full revolution
    cap: true,
})?;

// Loft between profiles
let lofted = kernel.loft(&LoftOptions {
    sections: vec![profile_a, profile_b, profile_c],
    guides: vec![guide_curve],
    start_condition: LoftEndCondition::Free,
    end_condition: LoftEndCondition::Free,
    closed: false,
    cap: true,
})?;

// === Queries ===

let mass = kernel.mass_properties(body, 7800.0)?;  // Steel density
println!("Volume: {} mm³", mass.volume);
println!("CoM: {:?}", mass.center_of_mass);

let face_count = kernel.body_faces(body).len();
let edge_count = kernel.body_edges(body).len();
let is_valid = kernel.check_body(body).is_ok();

// === I/O ===

// Import STEP
let import_result = kernel.import_step(
    Path::new("part.step"),
    &ImportOptions::default(),
)?;

// Export STEP
kernel.export_step(
    &[body],
    Path::new("output.step"),
    &ExportOptions {
        schema: StepSchema::AP214,
        export_colors: true,
        ..Default::default()
    },
)?;

// Export mesh
let mesh = kernel.tessellate_body(body, &TessellationOptions {
    chord_tolerance: 0.1,
    angle_tolerance: 15.0_f64.to_radians(),
    compute_normals: true,
    ..Default::default()
});
StlExporter::export_binary(&mesh, Path::new("output.stl"))?;

// === Undo/Redo ===

kernel.undo()?;
kernel.redo()?;

// Branching for exploration
let branch = kernel.branch();
// Try something on the branch...
let result_a = branch.fillet_edges(&edges, &big_radius_options);
// Compare with original...
```

## 3. Query API

Every entity is fully introspectable:

```rust
// === Body Queries ===
impl Kernel {
    pub fn body_type(&self, body: EntityId<Body>) -> BodyType;
    pub fn body_faces(&self, body: EntityId<Body>) -> Vec<EntityId<Face>>;
    pub fn body_edges(&self, body: EntityId<Body>) -> Vec<EntityId<Edge>>;
    pub fn body_vertices(&self, body: EntityId<Body>) -> Vec<EntityId<Vertex>>;
    pub fn body_shells(&self, body: EntityId<Body>) -> Vec<EntityId<Shell>>;
    pub fn body_bounding_box(&self, body: EntityId<Body>) -> BoundingBox3;
    pub fn body_precision(&self, body: EntityId<Body>) -> BodyPrecision;
}

// === Face Queries ===
impl Kernel {
    pub fn face_surface(&self, face: EntityId<Face>) -> &Surface;
    pub fn face_surface_type(&self, face: EntityId<Face>) -> SurfaceClassification;
    pub fn face_edges(&self, face: EntityId<Face>) -> Vec<EntityId<Edge>>;
    pub fn face_adjacent_faces(&self, face: EntityId<Face>) -> Vec<EntityId<Face>>;
    pub fn face_area(&self, face: EntityId<Face>) -> f64;
    pub fn face_normal_at(&self, face: EntityId<Face>, u: f64, v: f64) -> UnitVector3;
    pub fn face_point_at(&self, face: EntityId<Face>, u: f64, v: f64) -> Point3;
    pub fn face_contains_point(&self, face: EntityId<Face>, point: &Point3) -> bool;
    pub fn face_bounding_box(&self, face: EntityId<Face>) -> BoundingBox3;
}

// === Edge Queries ===
impl Kernel {
    pub fn edge_curve(&self, edge: EntityId<Edge>) -> &Curve;
    pub fn edge_curve_type(&self, edge: EntityId<Edge>) -> CurveClassification;
    pub fn edge_length(&self, edge: EntityId<Edge>) -> f64;
    pub fn edge_faces(&self, edge: EntityId<Edge>) -> Vec<EntityId<Face>>;
    pub fn edge_vertices(&self, edge: EntityId<Edge>) -> (EntityId<Vertex>, EntityId<Vertex>);
    pub fn edge_midpoint(&self, edge: EntityId<Edge>) -> Point3;
    pub fn edge_tangent_at(&self, edge: EntityId<Edge>, t: f64) -> UnitVector3;
    pub fn edge_tolerance(&self, edge: EntityId<Edge>) -> f64;
    pub fn edge_is_smooth(&self, edge: EntityId<Edge>, angle_threshold: f64) -> bool;
}

// === Vertex Queries ===
impl Kernel {
    pub fn vertex_point(&self, vertex: EntityId<Vertex>) -> Point3;
    pub fn vertex_edges(&self, vertex: EntityId<Vertex>) -> Vec<EntityId<Edge>>;
    pub fn vertex_faces(&self, vertex: EntityId<Vertex>) -> Vec<EntityId<Face>>;
    pub fn vertex_tolerance(&self, vertex: EntityId<Vertex>) -> f64;
}

// === Geometric Queries ===
impl Kernel {
    pub fn closest_point_on_body(&self, point: &Point3, body: EntityId<Body>)
        -> (Point3, f64, EntityId<Face>);
    pub fn point_in_body(&self, point: &Point3, body: EntityId<Body>) -> Containment;
    pub fn min_distance(&self, body_a: EntityId<Body>, body_b: EntityId<Body>) -> f64;
    pub fn interference_check(&self, bodies: &[EntityId<Body>]) -> Vec<Interference>;
}
```

## 4. Error Reporting

Errors are structured for programmatic handling by AI agents:

```rust
/// Top-level kernel error type.
pub enum KernelError {
    Boolean(BooleanError),
    Blend(BlendError),
    Offset(OffsetError),
    Sweep(SweepError),
    DirectModel(DirectModelError),
    Import(ImportError),
    Export(ExportError),
    Topology(TopologyError),
    InvalidEntity { id: String, reason: &'static str },
}

impl KernelError {
    /// Human-readable explanation of what went wrong.
    pub fn explanation(&self) -> String;

    /// Suggested actions to resolve the error.
    pub fn suggestions(&self) -> Vec<String>;

    /// The entities involved in the error.
    pub fn involved_entities(&self) -> Vec<AnyEntityId>;

    /// Severity: can the operation be retried with different parameters?
    pub fn is_recoverable(&self) -> bool;
}
```

Example error output for an AI agent:
```
Error: BlendError::RadiusTooLarge {
    edge: Edge#42,
    max_feasible: 3.7,
    requested: 5.0,
}
Explanation: "Fillet radius 5.0mm exceeds the maximum feasible radius of 3.7mm
             for edge Edge#42. The adjacent faces are too narrow."
Suggestions: [
    "Reduce fillet radius to <= 3.7mm",
    "Try variable-radius fillet tapering to 0 at narrow end",
    "Fillet adjacent edges first, then this edge",
]
```

## 5. C FFI

For integration with C/C++ applications:

```rust
// Auto-generated via cbindgen

#[repr(C)]
pub struct OsKernel { _private: [u8; 0] }

#[repr(C)]
pub struct OsBodyId { index: u32, generation: u32 }

#[no_mangle]
pub extern "C" fn os_kernel_new() -> *mut OsKernel;

#[no_mangle]
pub extern "C" fn os_kernel_free(kernel: *mut OsKernel);

#[no_mangle]
pub extern "C" fn os_make_block(
    kernel: *mut OsKernel,
    x: f64, y: f64, z: f64,
    out_body: *mut OsBodyId,
) -> OsErrorCode;

#[no_mangle]
pub extern "C" fn os_boolean(
    kernel: *mut OsKernel,
    body_a: OsBodyId,
    body_b: OsBodyId,
    operation: OsBooleanOp,
    out_result: *mut OsBodyId,
) -> OsErrorCode;

#[no_mangle]
pub extern "C" fn os_import_step(
    kernel: *mut OsKernel,
    path: *const c_char,
    out_bodies: *mut *mut OsBodyId,
    out_count: *mut u32,
) -> OsErrorCode;

#[repr(C)]
pub enum OsErrorCode {
    Ok = 0,
    InvalidBody = 1,
    BooleanFailed = 2,
    ImportFailed = 3,
    // ...
}
```

## 6. Python Bindings (PyO3)

```python
import opensolid

kernel = opensolid.Kernel()

# Create and combine
block = kernel.make_block(10, 20, 30)
cylinder = kernel.make_cylinder(5, 40)
kernel.transform(cylinder, opensolid.Translation(5, 10, 0))

result = kernel.subtract(block, cylinder)

# Fillet
edges = kernel.body_edges(result)
short_edges = [e for e in edges if kernel.edge_length(e) < 15]
kernel.fillet_edges(short_edges, radius=2.0)

# Query
mass = kernel.mass_properties(result, density=7800)
print(f"Volume: {mass.volume:.1f} mm³")
print(f"Center of mass: {mass.center_of_mass}")

# I/O
kernel.import_step("input.step")
kernel.export_step([result], "output.step", schema="AP214")

# Tessellate
mesh = kernel.tessellate(result, chord_tolerance=0.1)
mesh.export_stl("output.stl")
```

## 7. WASM Target

For browser-based applications:

```rust
// Compiled with wasm-pack, exposed via wasm-bindgen
#[wasm_bindgen]
pub struct WasmKernel {
    inner: Kernel,
}

#[wasm_bindgen]
impl WasmKernel {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self;

    pub fn make_block(&mut self, x: f64, y: f64, z: f64) -> u32; // Returns body ID
    pub fn boolean(&mut self, a: u32, b: u32, op: u32) -> u32;
    pub fn tessellate(&self, body: u32) -> js_sys::Float32Array;
    pub fn import_step(&mut self, data: &[u8]) -> Vec<u32>;
    pub fn export_step(&self, bodies: &[u32]) -> Vec<u8>;
}
```

## 8. Streaming / Progressive API

For AI agents that want incremental feedback:

```rust
/// A long-running operation that reports progress.
pub trait ProgressiveOp {
    type Result;
    type Progress;

    fn start(&mut self) -> impl Stream<Item = Self::Progress>;
    fn cancel(&mut self);
    fn result(self) -> Option<Self::Result>;
}

// Example: boolean with progress
let mut op = kernel.boolean_progressive(body_a, body_b, &options);
let stream = op.start();
while let Some(progress) = stream.next().await {
    match progress {
        BooleanProgress::IntersectingFaces { done, total } => { ... }
        BooleanProgress::ClassifyingFaces { done, total } => { ... }
        BooleanProgress::Reconstructing => { ... }
    }
}
let result = op.result().unwrap();
```

## 9. High-Level Builder API

The low-level Kernel API gives full control but requires managing EntityIds manually.
The Builder API is a fluent, chainable interface for common modeling workflows.
This is the primary API for scripting, AI agents, and tutorials.

```rust
use opensolid::prelude::*;

// Fluent construction
let bracket = Shape::block(100.0, 50.0, 10.0)
    .translate(0.0, 0.0, 5.0)
    .subtract(Shape::cylinder(15.0, 20.0).at(50.0, 25.0, 0.0))
    .fillet_all_edges(2.0)
    .build(&mut kernel)?;

// Named selections for parametric workflows
let plate = Shape::block(200.0, 100.0, 5.0)
    .name("plate")
    .with_named_face("top", FaceSelector::Normal(UnitVector3::Z))
    .subtract(
        Shape::cylinder(10.0, 10.0)
            .pattern_linear(Vector3::X, 4, 40.0)
            .name("bolt_holes")
    )
    .fillet_edges(EdgeSelector::ByAdjacentFace("bolt_holes"), 1.0)
    .build(&mut kernel)?;

// Access named entities after build
let top_face = kernel.find_named("plate.top")?;
let hole_edges = kernel.find_all_named("bolt_holes.*")?;
```

Python equivalent (identical semantics):
```python
bracket = (Shape.block(100, 50, 10)
    .translate(0, 0, 5)
    .subtract(Shape.cylinder(15, 20).at(50, 25, 0))
    .fillet_all_edges(2.0)
    .build(kernel))

plate = (Shape.block(200, 100, 5)
    .name("plate")
    .with_named_face("top", FaceSelector.normal(UnitVector3.Z))
    .subtract(
        Shape.cylinder(10, 10)
            .pattern_linear(Vector3.X, count=4, spacing=40)
            .name("bolt_holes")
    )
    .fillet_edges(EdgeSelector.by_adjacent_face("bolt_holes"), radius=1.0)
    .build(kernel))
```

## 10. Mesh as First-Class Output

Mesh/STL is not a second-class export afterthought. Many downstream workflows
(FEA, 3D printing, game engines, rendering) consume meshes, not B-rep. The kernel
treats mesh output as a primary deliverable.

```rust
/// Triangle mesh with per-vertex normals, UV coords, and face provenance.
pub struct TriangleMesh {
    pub vertices: Vec<Point3>,
    pub normals: Vec<UnitVector3>,
    pub triangles: Vec<[u32; 3]>,
    /// Maps each triangle back to the B-rep face it came from.
    pub face_provenance: Vec<EntityId<Face>>,
    /// Optional UV coordinates (for texture mapping).
    pub uvs: Option<Vec<[f64; 2]>>,
}

impl Kernel {
    /// Tessellate with full control.
    pub fn tessellate_body(
        &self, body: EntityId<Body>, options: &TessellationOptions,
    ) -> Result<TriangleMesh, TessellationError>;

    /// Quick export to STL (binary).
    pub fn export_stl(
        &self, body: EntityId<Body>, path: &Path, options: &StlOptions,
    ) -> Result<(), ExportError>;

    /// Quick export to OBJ (with normals and UVs).
    pub fn export_obj(
        &self, body: EntityId<Body>, path: &Path, options: &ObjOptions,
    ) -> Result<(), ExportError>;

    /// Import mesh and wrap as a sheet body (for hybrid workflows).
    pub fn import_mesh(
        &mut self, mesh: &TriangleMesh,
    ) -> Result<EntityId<Body>, ImportError>;
}
```

Face provenance in the mesh allows downstream tools to map mesh regions back to
B-rep topology (e.g., "these triangles came from the fillet face").
