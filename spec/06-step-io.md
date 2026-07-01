# 06 — STEP Import/Export

ISO 10303 AP203/AP214 reader and writer. This is a hard requirement for interoperability.

## 0. Key Design Principle: Growing Whitelist

STEP import is an **infinite problem**. Every CAD vendor implements their own dialect.
Files from SolidWorks use entity types not documented in public specifications. Files
from CATIA use non-standard extensions. Treating STEP as a binary "done/not done"
feature is a path to failure.

Instead: STEP import supports a **growing entity whitelist**. Unknown entities are
skipped with a warning, never crash the import. The whitelist is expanded based on
corpus failures.

```rust
/// Entity types we can currently interpret.
/// This list grows over time as we encounter new entity types in real files.
pub struct EntityWhitelist {
    supported: HashSet<&'static str>,
}

impl EntityWhitelist {
    /// Check if we can interpret this entity type.
    /// Unknown types are skipped, not crashed on.
    pub fn is_supported(&self, type_name: &str) -> bool;

    /// Count of supported entity types (grows over time).
    pub fn supported_count(&self) -> usize;
}
```

**Pass-rate targets** (tracked in CI against adversarial corpus):
- Month 3: 80% import success on FreeCAD exports
- Month 6: 90% on ABC Dataset (200 files)
- Month 9: 85% on vendor exports (SolidWorks, CATIA, NX, Fusion)
- Month 12: 95% on full corpus (500+ files)
- Month 18: 99% on full corpus

**Part 21 lexing:** Consider using `ruststep` for Part 21 tokenization (parsing the
text encoding) and only building the AP203/AP214 semantic interpretation layer ourselves.
The semantic layer (entity inheritance, complex entities, unit handling) is where our
value is — not in reinventing string splitting.

## 1. Overview

STEP (Standard for the Exchange of Product model data) is the universal CAD interchange
format. We must support:

- **AP203** — Configuration Controlled 3D Design (most common for mechanical parts)
- **AP214** — Automotive Design (superset of AP203 with colors, layers, annotations)
- **Part 21** — Physical file format (the text encoding)

### 1.1 Why Build Our Own Semantic Layer

Existing Rust STEP parsers (`ruststep`) handle Part 21 syntax but not the full AP203/AP214
semantic interpretation. CAD interop requires understanding:
- Entity inheritance hierarchies
- Implicit vs. explicit entity relationships
- Tolerance and unit interpretation
- Assembly structure and transforms
- Complex entity instances

## 2. Part 21 Physical File Format

### 2.1 File Structure

```
ISO-10303-21;
HEADER;
  FILE_DESCRIPTION((''), '2;1');
  FILE_NAME('part.step', '2026-05-16', ('Author'), ('Org'), '', 'OpenSolid', '');
  FILE_SCHEMA(('AUTOMOTIVE_DESIGN'));
ENDSEC;
DATA;
  #1 = CARTESIAN_POINT('', (0.0, 0.0, 0.0));
  #2 = DIRECTION('', (0.0, 0.0, 1.0));
  #3 = AXIS2_PLACEMENT_3D('', #1, #2, #4);
  ...
ENDSEC;
END-ISO-10303-21;
```

### 2.2 Parser Architecture

```rust
/// Two-phase parsing: syntax (Part 21) then semantics (AP203/AP214 interpretation).
pub struct StepReader {
    entities: HashMap<u64, RawEntity>,
    schema: StepSchema,
}

/// Phase 1: Parse Part 21 syntax into raw entities.
pub struct Part21Parser;

impl Part21Parser {
    pub fn parse(input: &str) -> Result<Part21File, ParseError>;
}

pub struct Part21File {
    pub header: Header,
    pub entities: Vec<(u64, RawEntity)>,  // (#id, entity)
}

pub struct Header {
    pub description: Vec<String>,
    pub name: String,
    pub schema: String,               // "AUTOMOTIVE_DESIGN" or "CONFIG_CONTROL_DESIGN"
    pub author: Vec<String>,
    pub organization: Vec<String>,
    pub preprocessor: String,
    pub originating_system: String,
    pub timestamp: String,
}

/// A raw, uninterpreted STEP entity.
pub struct RawEntity {
    pub type_name: String,            // e.g., "CARTESIAN_POINT"
    pub attributes: Vec<StepValue>,
}

/// STEP data types (Part 21 §7).
pub enum StepValue {
    Integer(i64),
    Real(f64),
    String(String),
    Enum(String),                     // .TRUE., .F., .VERTEX_POINT., etc.
    EntityRef(u64),                   // #123
    List(Vec<StepValue>),
    Null,                             // $ (unset)
    Derived,                          // * (derived attribute)
    TypedValue { type_name: String, value: Box<StepValue> },
    Complex(Vec<RawEntity>),          // Complex entity instance
}
```

### 2.3 Entity Resolution

```rust
/// Phase 2: Resolve entity references and build typed structures.
pub struct EntityResolver {
    entities: HashMap<u64, RawEntity>,
    resolved: HashMap<u64, ResolvedEntity>,
}

impl EntityResolver {
    /// Resolve all entity references recursively.
    pub fn resolve_all(&mut self) -> Result<(), ResolveError>;

    /// Get a specific entity by ID, resolving lazily.
    pub fn get<T: FromStep>(&mut self, id: u64) -> Result<T, ResolveError>;
}

/// Trait for types that can be constructed from STEP entities.
pub trait FromStep: Sized {
    fn from_step(entity: &RawEntity, resolver: &mut EntityResolver) -> Result<Self, ResolveError>;
}
```

## 3. AP203/AP214 Entity Hierarchy

### 3.1 Geometry Entities

```
representation_item
├── geometric_representation_item
│   ├── point
│   │   ├── cartesian_point
│   │   ├── point_on_curve
│   │   └── point_on_surface
│   ├── direction
│   ├── vector
│   ├── placement
│   │   ├── axis1_placement
│   │   ├── axis2_placement_2d
│   │   └── axis2_placement_3d
│   ├── curve
│   │   ├── line
│   │   ├── conic
│   │   │   ├── circle
│   │   │   ├── ellipse
│   │   │   ├── hyperbola
│   │   │   └── parabola
│   │   ├── bounded_curve
│   │   │   ├── b_spline_curve
│   │   │   │   ├── b_spline_curve_with_knots
│   │   │   │   ├── uniform_curve
│   │   │   │   ├── quasi_uniform_curve
│   │   │   │   └── bezier_curve
│   │   │   ├── trimmed_curve
│   │   │   ├── composite_curve
│   │   │   │   └── composite_curve_on_surface
│   │   │   └── polyline
│   │   ├── pcurve
│   │   ├── surface_curve
│   │   │   └── intersection_curve
│   │   └── offset_curve_3d
│   ├── surface
│   │   ├── elementary_surface
│   │   │   ├── plane
│   │   │   ├── cylindrical_surface
│   │   │   ├── conical_surface
│   │   │   ├── spherical_surface
│   │   │   └── toroidal_surface
│   │   ├── bounded_surface
│   │   │   ├── b_spline_surface
│   │   │   │   ├── b_spline_surface_with_knots
│   │   │   │   ├── uniform_surface
│   │   │   │   ├── quasi_uniform_surface
│   │   │   │   └── bezier_surface
│   │   │   ├── rectangular_trimmed_surface
│   │   │   └── curve_bounded_surface
│   │   ├── swept_surface
│   │   │   ├── surface_of_linear_extrusion
│   │   │   └── surface_of_revolution
│   │   ├── offset_surface
│   │   └── degenerate_toroidal_surface
│   └── geometric_set
│       └── geometric_curve_set
```

### 3.2 Topology Entities

```
topological_representation_item
├── vertex
│   └── vertex_point
├── edge
│   ├── edge_curve
│   └── oriented_edge
├── face_bound
│   └── face_outer_bound
├── face
│   ├── face_surface
│   │   └── advanced_face
│   └── oriented_face
├── connected_face_set
│   ├── closed_shell
│   └── open_shell
└── loop
    ├── edge_loop
    ├── vertex_loop
    └── poly_loop
```

### 3.3 Shape Representation

```
representation
└── shape_representation
    ├── advanced_brep_shape_representation     (solid bodies)
    ├── manifold_surface_shape_representation  (sheet bodies)
    ├── geometrically_bounded_surface_shape_representation
    └── faceted_brep_shape_representation      (tessellated)

representation_item
└── mapped_item (for assemblies — instances a shape with a transform)

topological_representation_item
├── manifold_solid_brep
├── shell_based_surface_model
├── brep_with_voids
└── faceted_brep
```

### 3.4 Product Structure (Assemblies)

```
product
└── product_definition
    └── product_definition_shape
        └── shape_definition_representation
            → shape_representation

next_assembly_usage_occurrence  (parent-child relationship)
└── product_definition_relationship
```

## 4. STEP Import Pipeline

```rust
pub struct StepImporter;

impl StepImporter {
    /// Import a STEP file into the kernel.
    ///
    /// Pipeline:
    /// 1. Parse Part 21 syntax
    /// 2. Identify schema (AP203 or AP214)
    /// 3. Resolve entity references
    /// 4. Build geometry (curves, surfaces)
    /// 5. Build topology (vertices, edges, faces, shells, bodies)
    /// 6. Validate imported bodies
    /// 7. Heal if necessary (tolerance adjustment, gap closure)
    pub fn import(
        path: &Path,
        kernel: &mut Kernel,
        options: &ImportOptions,
    ) -> Result<ImportResult, ImportError>;
}

pub struct ImportOptions {
    /// Target tolerance for imported geometry.
    pub tolerance: f64,
    /// Healing strategy. Default is HealStrategy::Auto.
    /// Import ALWAYS heals — the question is how aggressively.
    pub heal_strategy: HealStrategy,
    /// Unit override (if file units are ambiguous).
    pub unit_override: Option<LengthUnit>,
    /// Import colors/materials (AP214).
    pub import_colors: bool,
    /// Import assembly structure.
    pub import_assemblies: bool,
    /// Import product names.
    pub import_names: bool,
    /// Maximum tolerance the importer will accept after healing.
    /// Bodies exceeding this are returned with a warning, never rejected.
    pub max_acceptable_tolerance: f64,
}

/// STEP import HEALS, it does NOT reject.
/// Real-world STEP files are messy. Files from CATIA, SolidWorks, Creo all have
/// tolerance issues, missing pcurves, degenerate edges. The importer must handle
/// them gracefully — a rejected file is a lost customer.
pub enum HealStrategy {
    /// Attempt all healing operations automatically. Default.
    Auto,
    /// Minimal healing (only gap closure and tolerance adjustment).
    Minimal,
    /// Report what would be healed but don't modify geometry.
    ReportOnly,
}

pub struct ImportResult {
    pub bodies: Vec<EntityId<Body>>,
    pub assemblies: Vec<Assembly>,
    pub warnings: Vec<ImportWarning>,
    pub stats: ImportStats,
}

pub struct ImportStats {
    pub total_entities: usize,
    pub bodies_imported: usize,
    pub faces_total: usize,
    pub edges_total: usize,
    pub heal_operations: usize,
    pub elapsed: Duration,
}

pub enum ImportWarning {
    ToleranceElevated { body: EntityId<Body>, max_tolerance: f64 },
    UnknownEntity { id: u64, type_name: String },
    DegenerateGeometry { entity_id: u64, description: String },
    HealApplied { body: EntityId<Body>, operation: String },
    UnitAmbiguity { default_assumed: LengthUnit },
}
```

### 4.1 Geometry Construction from STEP Entities

```rust
/// Convert STEP geometry entities to kernel geometry.
pub struct GeometryBuilder;

impl GeometryBuilder {
    pub fn build_point(&self, entity: &CartesianPointEntity) -> Point3;
    pub fn build_direction(&self, entity: &DirectionEntity) -> UnitVector3;
    pub fn build_axis2_placement(&self, entity: &Axis2Placement3dEntity) -> Transform3;

    pub fn build_curve(&self, entity: &CurveEntity, resolver: &mut EntityResolver)
        -> Result<EntityId<Curve>, ImportError>;

    pub fn build_surface(&self, entity: &SurfaceEntity, resolver: &mut EntityResolver)
        -> Result<EntityId<Surface>, ImportError>;

    pub fn build_bspline_curve(&self, entity: &BSplineCurveWithKnotsEntity)
        -> Result<BSplineCurve, ImportError>;

    pub fn build_bspline_surface(&self, entity: &BSplineSurfaceWithKnotsEntity)
        -> Result<BSplineSurface, ImportError>;
}
```

### 4.2 Topology Construction from STEP Entities

```rust
/// Convert STEP topology entities to kernel topology.
pub struct TopologyImporter;

impl TopologyImporter {
    /// Build a solid body from a manifold_solid_brep entity.
    pub fn build_solid(
        &self,
        entity: &ManifoldSolidBrepEntity,
        resolver: &mut EntityResolver,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
    ) -> Result<EntityId<Body>, ImportError>;

    /// Build a face from an advanced_face entity.
    pub fn build_face(
        &self,
        entity: &AdvancedFaceEntity,
        resolver: &mut EntityResolver,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
    ) -> Result<EntityId<Face>, ImportError>;

    /// Build an edge from an edge_curve entity.
    pub fn build_edge(
        &self,
        entity: &EdgeCurveEntity,
        resolver: &mut EntityResolver,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
    ) -> Result<EntityId<Edge>, ImportError>;
}
```

## 5. STEP Export Pipeline

```rust
pub struct StepExporter;

impl StepExporter {
    /// Export bodies to a STEP file.
    ///
    /// Pipeline:
    /// 1. Collect all entities to export
    /// 2. Assign STEP entity IDs (sequential #1, #2, ...)
    /// 3. Write header
    /// 4. Write geometry entities (bottom-up: points, curves, surfaces)
    /// 5. Write topology entities (bottom-up: vertices, edges, faces, shells, solids)
    /// 6. Write product structure (if assemblies)
    /// 7. Write colors/materials (AP214)
    pub fn export(
        bodies: &[EntityId<Body>],
        kernel: &Kernel,
        path: &Path,
        options: &ExportOptions,
    ) -> Result<ExportStats, ExportError>;
}

pub struct ExportOptions {
    /// Schema to write (AP203 or AP214).
    pub schema: StepSchema,
    /// Export colors/materials (requires AP214).
    pub export_colors: bool,
    /// Export assembly structure.
    pub export_assemblies: bool,
    /// Author information for header.
    pub author: String,
    pub organization: String,
    /// Application name for header.
    pub application: String,
}

pub enum StepSchema {
    AP203,
    AP214,
}

pub struct ExportStats {
    pub entities_written: usize,
    pub file_size_bytes: usize,
    pub elapsed: Duration,
}
```

### 5.1 Entity ID Assignment

```rust
/// Manages assignment of sequential STEP entity IDs during export.
pub struct EntityIdAllocator {
    next_id: u64,
    /// Map from kernel entity to STEP entity ID.
    curve_ids: HashMap<EntityId<Curve>, u64>,
    surface_ids: HashMap<EntityId<Surface>, u64>,
    vertex_ids: HashMap<EntityId<Vertex>, u64>,
    edge_ids: HashMap<EntityId<Edge>, u64>,
    face_ids: HashMap<EntityId<Face>, u64>,
    // ... etc
}
```

### 5.2 STEP Writer

```rust
/// Low-level STEP text writer.
pub struct StepWriter {
    output: BufWriter<File>,
    indent: usize,
}

impl StepWriter {
    pub fn write_header(&mut self, header: &Header) -> io::Result<()>;
    pub fn write_entity(&mut self, id: u64, type_name: &str, attrs: &[StepValue]) -> io::Result<()>;
    pub fn begin_data(&mut self) -> io::Result<()>;
    pub fn end_data(&mut self) -> io::Result<()>;
}
```

## 6. Geometry Healing

Imported STEP files frequently have geometry issues:

```rust
pub struct GeometryHealer;

impl GeometryHealer {
    /// Fix tolerance gaps between edges and vertices.
    pub fn fix_gaps(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        max_gap: f64,
    ) -> HealResult;

    /// Fix edges whose curves don't lie on adjacent faces' surfaces.
    pub fn fix_edge_surface_consistency(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        tolerance: f64,
    ) -> HealResult;

    /// Merge duplicate vertices that are within tolerance.
    pub fn merge_close_vertices(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        tolerance: f64,
    ) -> HealResult;

    /// Recompute edge curves from face-face intersection.
    pub fn recompute_edge_curves(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
    ) -> HealResult;

    /// Fix face orientation inconsistencies.
    pub fn fix_orientation(
        body: EntityId<Body>,
        store: &mut TopologyStore,
    ) -> HealResult;

    /// Full healing pipeline (all of the above in order).
    pub fn heal(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        options: &HealOptions,
    ) -> HealResult;
}

pub struct HealResult {
    pub operations_applied: Vec<HealOperation>,
    pub remaining_issues: Vec<GeometryIssue>,
    pub max_tolerance_before: f64,
    pub max_tolerance_after: f64,
}

pub enum HealOperation {
    VertexMerged { v1: EntityId<Vertex>, v2: EntityId<Vertex>, gap: f64 },
    EdgeCurveRecomputed { edge: EntityId<Edge> },
    ToleranceElevated { edge: EntityId<Edge>, new_tolerance: f64 },
    FaceReoriented { face: EntityId<Face> },
    GapClosed { edge: EntityId<Edge>, gap: f64 },
}
```

## 7. Unit Handling

STEP files specify units in the representation context:

```rust
pub enum LengthUnit {
    Meter,
    Millimeter,
    Centimeter,
    Inch,
    Foot,
}

pub enum AngleUnit {
    Radian,
    Degree,
}

pub struct StepUnits {
    pub length: LengthUnit,
    pub angle: AngleUnit,
    pub solid_angle: SolidAngleUnit,
}

impl StepUnits {
    /// Extract units from STEP representation context entities.
    pub fn from_context(
        context: &GlobalUnitAssignedContextEntity,
        resolver: &mut EntityResolver,
    ) -> Result<Self, ImportError>;

    /// Convert a length value from file units to kernel units (meters).
    pub fn to_meters(&self, value: f64) -> f64;
}
```

## 8. Round-Trip Testing

STEP round-trip correctness is validated by:

```rust
#[cfg(test)]
mod step_roundtrip_tests {
    /// Import a STEP file, export it, import again, compare.
    /// Bodies should be geometrically equivalent within tolerance.
    fn roundtrip_test(filename: &str, tolerance: f64);

    /// Verify exported STEP can be imported by OpenCASCADE.
    /// (External validation using OCC command-line tools)
    fn occ_compatibility_test(filename: &str);

    /// Verify our import matches OCC's import (volume, face count, etc.)
    fn import_equivalence_test(filename: &str);
}
```

### 8.1 Test Corpus

Golden STEP files for testing:
- NIST CAX-IF conformance test files
- OpenCASCADE test data (occt-cube.step, etc.)
- ABC Dataset samples (research dataset of ~1M CAD models)
- Hand-crafted edge cases (tangent intersections, high-degree NURBS, assemblies)
- Files exported by major CAD systems (SolidWorks, NX, CATIA, Fusion 360)

## 9. Known Interop Issues

| Issue | Source | Mitigation |
|-------|--------|-----------|
| Tolerance mismatch | Different systems use different precision | Heal on import, tolerance elevation |
| Missing pcurves | Some exporters omit parameter-space curves | Recompute from 3D curves + surface |
| Wrong orientation | Inconsistent face/edge sense flags | Orientation healing pass |
| Degenerate edges | Zero-length edges at singularities | Remove and adjust topology |
| Non-manifold input | Some exporters create invalid topology | Detect and split/heal |
| Unit confusion | Files claiming mm but containing m-scale coords | Heuristic detection (bounding box analysis) |
| Complex entity instances | Multiple inheritance entities | Full EXPRESS type system required |
| Naming conventions | `''` vs. `$` for empty names | Accept both |
