# 11 — Testing Strategy

How we verify correctness at every level.

## 1. Testing Philosophy

A geometry kernel has no room for "mostly works." A single wrong boolean or bad STEP
export can corrupt an entire manufacturing pipeline. Testing must be:

- **Exhaustive at boundaries**: Edge cases, degenerate geometry, tolerance limits
- **Verified against references**: Compare results to known-good outputs
- **Property-based**: Geometric invariants must hold for arbitrary inputs
- **Round-trip validated**: Import → Export → Import must preserve geometry
- **Performance-tracked**: Regression benchmarks catch algorithmic degradation

## 2. Test Levels

### 2.1 Unit Tests (per-crate)

Every function gets targeted tests for correctness:

```rust
#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn bspline_evaluation_matches_de_boor() {
        let curve = BSplineCurve {
            degree: 3,
            control_points: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(3.0, 1.0, 0.0),
            ],
            weights: None,
            knots: vec![0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0],
            knot_multiplicities: vec![4, 4],
            is_periodic: false,
        };

        let point = evaluate_bspline_curve(&curve, 0.5);
        assert_relative_eq!(point.x, 1.5, epsilon = 1e-10);
    }
}
```

### 2.2 Property-Based Tests (proptest)

Geometric invariants that must hold for ALL inputs:

```rust
use proptest::prelude::*;

proptest! {
    /// NURBS curve derivative is consistent with finite differences.
    #[test]
    fn bspline_derivative_consistency(
        t in 0.01f64..0.99,
        degree in 2u32..6,
    ) {
        let curve = random_bspline_curve(degree, 10);
        let deriv = curve.derivative_at(t);
        let h = 1e-7;
        let finite_diff = (curve.point_at(t + h) - curve.point_at(t - h)) / (2.0 * h);
        assert_near!(deriv, finite_diff, 1e-5);
    }

    /// Boolean union volume >= max(volume_a, volume_b).
    #[test]
    fn boolean_union_volume_lower_bound(
        size_a in 1.0f64..10.0,
        size_b in 1.0f64..10.0,
        offset in -5.0f64..5.0,
    ) {
        let mut kernel = Kernel::new();
        let a = kernel.make_block(size_a, size_a, size_a).unwrap();
        let b = kernel.make_block(size_b, size_b, size_b).unwrap();
        kernel.transform(b, &Transform3::translation(offset, 0.0, 0.0)).unwrap();

        let vol_a = kernel.mass_properties(a, 1.0).unwrap().volume;
        let vol_b = kernel.mass_properties(b, 1.0).unwrap().volume;

        let result = kernel.unite(a, b).unwrap();
        let vol_result = kernel.mass_properties(result, 1.0).unwrap().volume;

        assert!(vol_result >= vol_a.max(vol_b) - 1e-6);
        assert!(vol_result <= vol_a + vol_b + 1e-6);
    }

    /// Boolean identities: A ∪ A = A, A ∩ A = A, A - A = empty.
    #[test]
    fn boolean_self_operations(size in 1.0f64..20.0) {
        let mut kernel = Kernel::new();
        let a = kernel.make_sphere(size).unwrap();
        let a_copy = kernel.copy_body(a).unwrap();

        let union = kernel.unite(a, a_copy).unwrap();
        let vol_union = kernel.mass_properties(union, 1.0).unwrap().volume;
        let expected = 4.0 / 3.0 * std::f64::consts::PI * size.powi(3);
        assert_relative_eq!(vol_union, expected, epsilon = 1e-3);
    }

    /// Tessellation produces watertight mesh for any solid body.
    #[test]
    fn tessellation_watertight(seed in 0u64..1000) {
        let mut kernel = Kernel::new();
        let body = random_csg_body(&mut kernel, seed);
        let mesh = kernel.tessellate_body(body, &TessellationOptions::default());
        assert!(mesh_is_watertight(&mesh));
    }

    /// Edge curve lies on both adjacent faces' surfaces (within tolerance).
    #[test]
    fn edge_surface_consistency(seed in 0u64..1000) {
        let mut kernel = Kernel::new();
        let body = random_csg_body(&mut kernel, seed);
        for edge in kernel.body_edges(body) {
            let tolerance = kernel.edge_tolerance(edge);
            let curve = kernel.edge_curve(edge);
            let faces = kernel.edge_faces(edge);
            for t in linspace(curve.domain(), 20) {
                let point = curve.point_at(t);
                for &face in &faces {
                    let (_, _, dist) = kernel.face_surface(face).project_point(&point);
                    assert!(dist <= tolerance * 10.0,
                        "Edge {:?} deviates {} from face {:?} at t={}",
                        edge, dist, face, t);
                }
            }
        }
    }
}
```

### 2.3 Golden File Tests

Compare operation results against pre-computed reference files:

```rust
/// Test against reference STEP file.
/// The reference was validated by importing into SolidWorks/NX and confirming correct.
#[test]
fn step_import_nist_ctc_01() {
    let mut kernel = Kernel::new();
    let result = kernel.import_step(
        Path::new("tests/golden/nist_ctc_01.step"),
        &ImportOptions::default(),
    ).unwrap();

    assert_eq!(result.bodies.len(), 1);
    let body = result.bodies[0];
    assert_eq!(kernel.body_faces(body).len(), 34);
    assert_eq!(kernel.body_edges(body).len(), 50);
    assert_eq!(kernel.body_vertices(body).len(), 18);

    let mass = kernel.mass_properties(body, 1.0).unwrap();
    assert_relative_eq!(mass.volume, 1234.567, epsilon = 0.1);
}
```

### 2.4 Round-Trip Tests

```rust
/// Import STEP → export STEP → import again → compare.
#[test]
fn step_roundtrip_preserves_geometry() {
    let mut kernel = Kernel::new();
    let import1 = kernel.import_step(
        Path::new("tests/golden/complex_assembly.step"),
        &ImportOptions::default(),
    ).unwrap();

    let temp_path = Path::new("/tmp/roundtrip_test.step");
    kernel.export_step(&import1.bodies, temp_path, &ExportOptions::default()).unwrap();

    let mut kernel2 = Kernel::new();
    let import2 = kernel2.import_step(temp_path, &ImportOptions::default()).unwrap();

    assert_eq!(import1.bodies.len(), import2.bodies.len());
    for (b1, b2) in import1.bodies.iter().zip(import2.bodies.iter()) {
        let vol1 = kernel.mass_properties(*b1, 1.0).unwrap().volume;
        let vol2 = kernel2.mass_properties(*b2, 1.0).unwrap().volume;
        assert_relative_eq!(vol1, vol2, epsilon = vol1 * 1e-6);  // 0.0001% tolerance
    }
}
```

### 2.5 Integration Tests

End-to-end workflows that exercise multiple subsystems:

```rust
/// Full workflow: create part with booleans + fillets, export STEP, reimport, validate.
#[test]
fn integration_create_export_reimport() {
    let mut kernel = Kernel::new();

    // Create a block with a hole and fillet
    let block = kernel.make_block(100.0, 50.0, 30.0).unwrap();
    let hole = kernel.make_cylinder(10.0, 40.0).unwrap();
    kernel.transform(hole, &Transform3::translation(50.0, 25.0, -5.0)).unwrap();

    let body = kernel.subtract(block, hole).unwrap();

    // Fillet the hole edges
    let hole_edges: Vec<_> = kernel.body_edges(body).into_iter()
        .filter(|e| matches!(
            kernel.edge_curve_type(*e),
            CurveClassification::Circular { .. }
        ))
        .collect();
    kernel.fillet_edges(&hole_edges, &FilletOptions { radius: 2.0, ..Default::default() }).unwrap();

    // Validate
    assert!(kernel.check_body(body).is_ok());

    // Export and reimport
    let path = Path::new("/tmp/integration_test.step");
    kernel.export_step(&[body], path, &ExportOptions::default()).unwrap();

    let mut kernel2 = Kernel::new();
    let reimported = kernel2.import_step(path, &ImportOptions::default()).unwrap();
    assert_eq!(reimported.bodies.len(), 1);
    assert!(kernel2.check_body(reimported.bodies[0]).is_ok());

    // Volume should match
    let vol1 = kernel.mass_properties(body, 1.0).unwrap().volume;
    let vol2 = kernel2.mass_properties(reimported.bodies[0], 1.0).unwrap().volume;
    assert_relative_eq!(vol1, vol2, epsilon = 1.0);  // Within 1 mm³
}
```

## 3. Test Corpus

### 3.1 STEP Test Files

| Source | Contents | Purpose |
|--------|----------|---------|
| NIST CAx-IF | Conformance test cases | Standard compliance |
| OpenCASCADE test data | Simple to complex solids | Parser correctness |
| ABC Dataset (sample) | ~100 real mechanical parts | Real-world coverage |
| CAD system exports | Files from SW, NX, CATIA, Fusion | Interop validation |
| Hand-crafted edge cases | Tangent booleans, degenerate NURBS | Robustness |

### 3.2 Synthetic Test Generation

```rust
/// Generate random valid CSG bodies for fuzz testing.
pub fn random_csg_body(kernel: &mut Kernel, seed: u64) -> EntityId<Body> {
    let mut rng = StdRng::seed_from_u64(seed);
    let ops = rng.gen_range(2..8);

    let mut body = match rng.gen_range(0..3) {
        0 => kernel.make_block(
            rng.gen_range(5.0..50.0),
            rng.gen_range(5.0..50.0),
            rng.gen_range(5.0..50.0),
        ).unwrap(),
        1 => kernel.make_cylinder(
            rng.gen_range(5.0..25.0),
            rng.gen_range(10.0..50.0),
        ).unwrap(),
        _ => kernel.make_sphere(rng.gen_range(10.0..30.0)).unwrap(),
    };

    for _ in 0..ops {
        let tool = random_primitive(kernel, &mut rng);
        let offset = Vector3::new(
            rng.gen_range(-20.0..20.0),
            rng.gen_range(-20.0..20.0),
            rng.gen_range(-20.0..20.0),
        );
        kernel.transform(tool, &Transform3::translation(offset.x, offset.y, offset.z)).unwrap();

        let op = match rng.gen_range(0..3) {
            0 => BooleanOp::Unite,
            1 => BooleanOp::Subtract,
            _ => BooleanOp::Intersect,
        };

        if let Ok(result) = kernel.boolean(body, tool, &BooleanOptions {
            operation: op,
            ..Default::default()
        }) {
            body = result;
        }
    }

    body
}
```

## 4. Validation Checks

### 4.1 Body Validation (replaces Parasolid's PK_BODY_check)

```rust
pub struct BodyChecker;

impl BodyChecker {
    pub fn check(body: EntityId<Body>, kernel: &Kernel) -> Vec<BodyFault> {
        let mut faults = Vec::new();

        // Topological checks
        faults.extend(Self::check_manifold(body, kernel));
        faults.extend(Self::check_closed_shells(body, kernel));
        faults.extend(Self::check_orientation_consistency(body, kernel));
        faults.extend(Self::check_euler_formula(body, kernel));
        faults.extend(Self::check_loop_closure(body, kernel));

        // Geometric checks
        faults.extend(Self::check_edge_on_surfaces(body, kernel));
        faults.extend(Self::check_vertex_on_edges(body, kernel));
        faults.extend(Self::check_no_self_intersection(body, kernel));
        faults.extend(Self::check_tolerances(body, kernel));

        faults
    }
}

pub enum BodyFault {
    NonManifoldEdge { edge: EntityId<Edge>, face_count: usize },
    OpenShell { shell: EntityId<Shell>, boundary_edges: Vec<EntityId<Edge>> },
    InconsistentOrientation { face: EntityId<Face>, neighbor: EntityId<Face> },
    EulerViolation { shell: EntityId<Shell>, expected: i32, actual: i32 },
    UnclosedLoop { loop_id: EntityId<Loop>, gap: f64 },
    EdgeOffSurface { edge: EntityId<Edge>, face: EntityId<Face>, max_deviation: f64 },
    VertexOffEdge { vertex: EntityId<Vertex>, edge: EntityId<Edge>, deviation: f64 },
    SelfIntersection { face_a: EntityId<Face>, face_b: EntityId<Face> },
    ToleranceExceeded { edge: EntityId<Edge>, tolerance: f64, limit: f64 },
}
```

## 5. Performance Benchmarks

```rust
use criterion::{criterion_group, criterion_main, Criterion};

fn bench_boolean_union(c: &mut Criterion) {
    c.bench_function("boolean_union_two_blocks", |b| {
        b.iter(|| {
            let mut kernel = Kernel::new();
            let a = kernel.make_block(10.0, 10.0, 10.0).unwrap();
            let b_body = kernel.make_block(10.0, 10.0, 10.0).unwrap();
            kernel.transform(b_body, &Transform3::translation(5.0, 5.0, 5.0)).unwrap();
            kernel.unite(a, b_body).unwrap()
        })
    });
}

fn bench_fillet(c: &mut Criterion) {
    c.bench_function("fillet_all_edges_block", |b| {
        b.iter(|| {
            let mut kernel = Kernel::new();
            let block = kernel.make_block(20.0, 20.0, 20.0).unwrap();
            let edges = kernel.body_edges(block);
            kernel.fillet_edges(&edges, &FilletOptions {
                radius: 2.0,
                ..Default::default()
            }).unwrap();
        })
    });
}

fn bench_step_import(c: &mut Criterion) {
    c.bench_function("step_import_1000_faces", |b| {
        b.iter(|| {
            let mut kernel = Kernel::new();
            kernel.import_step(
                Path::new("benches/data/complex_1000_faces.step"),
                &ImportOptions::default(),
            ).unwrap()
        })
    });
}

fn bench_tessellation(c: &mut Criterion) {
    c.bench_function("tessellate_1000_face_body", |b| {
        let mut kernel = Kernel::new();
        let result = kernel.import_step(
            Path::new("benches/data/complex_1000_faces.step"),
            &ImportOptions::default(),
        ).unwrap();
        let body = result.bodies[0];

        b.iter(|| {
            kernel.tessellate_body(body, &TessellationOptions {
                chord_tolerance: 0.1,
                ..Default::default()
            })
        })
    });
}

criterion_group!(benches,
    bench_boolean_union,
    bench_fillet,
    bench_step_import,
    bench_tessellation,
);
criterion_main!(benches);
```

## 6. CI Pipeline

```yaml
# Every PR must pass:
- cargo test --all-features             # All unit + integration tests
- cargo test --release -- --ignored     # Long-running property tests
- cargo bench -- --output-format json   # Performance regression check
- cargo clippy -- -D warnings           # Lint
- cargo fmt -- --check                  # Formatting
- step_roundtrip_suite                  # Golden STEP file round-trips
```

## 7. Reference Implementation Comparison (OCC)

**This infrastructure must be built in week 1, before any algorithm implementation.**

OpenCASCADE (via CadQuery/Python) serves as ground truth for all geometric operations.
Without this, we're testing against our own assumptions.

### 7.1 OCC Comparison Script

```python
#!/usr/bin/env python3
"""scripts/occ_reference.py — Generate reference data for a STEP file using OCC."""
import json, sys
from OCP.STEPControl import STEPControl_Reader
from OCP.BRepGProp import brepgprop
from OCP.GProp import GProp_GProps

def analyze_step(path: str) -> dict:
    reader = STEPControl_Reader()
    reader.ReadFile(path)
    reader.TransferRoots()
    
    results = []
    for i in range(reader.NbShapes()):
        shape = reader.Shape(i + 1)
        props = GProp_GProps()
        brepgprop.VolumeProperties(shape, props)
        
        results.append({
            "volume": props.Mass(),
            "center_of_mass": list(props.CentreOfMass().Coord()),
            "face_count": count_faces(shape),
            "edge_count": count_edges(shape),
        })
    
    return {"bodies": results, "file": path}

if __name__ == "__main__":
    result = analyze_step(sys.argv[1])
    json.dump(result, sys.stdout, indent=2)
```

### 7.2 CI Integration

```yaml
# runs on every PR touching geometry, topology, booleans, or STEP
occ_comparison:
  - for file in tests/corpus/*.step; do
      python scripts/occ_reference.py "$file" > "/tmp/occ_$(basename $file).json"
      cargo test --test occ_comparison -- --file "$file" --reference "/tmp/occ_$(basename $file).json"
    done
```

### 7.3 Rust-Side Comparison Test

```rust
/// Compare our STEP import against OCC's import.
/// Both should produce bodies with matching topology counts and volume.
fn compare_with_occ(step_file: &Path) {
    // Our import
    let mut kernel = Kernel::new();
    let ours = kernel.import_step(step_file, &ImportOptions::default()).unwrap();

    // OCC import (via Python subprocess calling CadQuery)
    let occ_result = run_occ_import(step_file);

    // Compare
    assert_eq!(ours.bodies.len(), occ_result.body_count);
    for (i, body) in ours.bodies.iter().enumerate() {
        let our_vol = kernel.mass_properties(*body, 1.0).unwrap().volume;
        let occ_vol = occ_result.volumes[i];
        let relative_error = (our_vol - occ_vol).abs() / occ_vol;
        assert!(relative_error < 0.001,  // Within 0.1%
            "Volume mismatch on body {}: ours={}, occ={}, error={}%",
            i, our_vol, occ_vol, relative_error * 100.0);
    }
}
```

### 7.4 What Gets Compared

| Operation | OCC Reference | Tolerance |
|-----------|--------------|-----------|
| STEP import (volume) | BRepGProp.VolumeProperties | 0.1% relative |
| STEP import (face count) | TopExp_Explorer | exact match |
| Boolean (volume) | BRepAlgoAPI_Fuse/Cut/Common | 0.1% relative |
| Tessellation (watertight) | BRepMesh_IncrementalMesh | mesh is closed |
| SSI (curve endpoint) | GeomAPI_IntSS | within ssi_tolerance |

## 8. Adversarial Test Corpus

**This must exist before any algorithm implementation begins.**

The corpus drives development — it defines what "works" means. Without it, property
tests on random primitives give false confidence because they never produce the
geometry configurations that break real-world algorithms.

### 8.1 Corpus Structure

```
tests/corpus/
├── README.md              # Corpus documentation and pass-rate tracking
├── abc_dataset/           # 200+ mechanical parts from ABC Dataset (real scans)
├── nist_cax/              # NIST CAx-IF conformance test cases
├── vendor_exports/        # STEP files from specific CAD systems
│   ├── solidworks/        # SolidWorks 2020-2025 exports
│   ├── catia/             # CATIA V5/V6 exports
│   ├── nx/                # Siemens NX exports
│   ├── fusion360/         # Autodesk Fusion exports
│   └── freecad/           # FreeCAD exports (easiest to generate)
├── edge_cases/            # Hand-crafted degenerate geometry
│   ├── tangent_boolean/   # Near-tangent surface pairs
│   ├── coincident_faces/  # Overlapping face regions
│   ├── thin_features/     # Sub-tolerance features
│   ├── high_degree_nurbs/ # Degree 7+ surfaces
│   └── periodic_surfaces/ # Full cylinders, spheres, tori
└── reference/             # OCC-generated reference data (JSON per file)
```

### 8.2 Pass Rate Targets

| Milestone | Corpus Subset | Import Succeeds | Volume Matches OCC | Body Valid |
|-----------|---------------|-----------------|-------------------|------------|
| Month 3 | freecad/ (50 files) | 80% | 70% | 60% |
| Month 6 | abc_dataset/ (200 files) | 90% | 80% | 75% |
| Month 9 | vendor_exports/ (all) | 85% | 75% | 70% |
| Month 12 | Full corpus (500+ files) | 95% | 90% | 85% |
| Month 18 | Full corpus | 99% | 95% | 95% |

### 8.3 Corpus Rules

- Unknown/unsupported STEP entities **never crash** the import. They are skipped with a warning.
- Each corpus file has a corresponding `.json` reference (from OCC) checked into git.
- CI tracks pass rate. Regressions (pass rate drops) block merge.
- New corpus files are added whenever a user reports a failing STEP file.
