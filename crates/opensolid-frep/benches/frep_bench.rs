//! Criterion benchmarks for the F-Rep hot paths: single-point SDF
//! evaluation, gradients, and uniform dual-contouring meshing.
//!
//! Perf context (spec/00-overview.md §6): the spec's initial targets are
//! B-Rep-centric — e.g. NURBS surface eval < 5µs/point, tessellation of a
//! 1000-face body < 1s, trivial boolean < 50ms. F-Rep is our fast path, so
//! the corresponding expectations here are far tighter:
//! - primitive SDF eval: tens of ns
//! - 3-deep CSG tree eval: < 200ns (generic), dyn dispatch adds one vcall
//!   per node on top
//! - `mesh_sdf` at res 64 on a CSG part: well under 1s (it is the F-Rep
//!   analogue of tessellation, and the mesher is the boolean fast path's
//!   output stage)
//!
//! Baselines recorded 2026-07-03 (Apple Silicon M-series, default bench
//! profile), so regressions have a concrete reference; > 10% regressions
//! on these paths block merge per the spec:
//! - eval/sphere            ~ 1.0 ns
//! - eval/csg_tree_generic  ~ 15 ns
//! - eval/csg_tree_dyn      ~ 10 ns
//! - gradient/csg_tree      ~ 15 ns  (analytic grad; was ~101 ns with FD)
//! - mesh/uniform_res32     ~ 0.8 ms (rayon-parallel mesher)
//! - mesh/uniform_res64     ~ 5.2 ms
//!
//! (Re-measure with `cargo bench -p opensolid-frep` and update alongside
//! intentional changes. Note the dyn tree currently measures FASTER than
//! the monomorphized one — an inlining/codegen artifact of this particular
//! tree at this particular query point, not a general rule; treat the two
//! as independent baselines rather than a comparison.)

use criterion::{Criterion, criterion_group, criterion_main};
use opensolid_core::types::{BoundingBox3, Point3};
use opensolid_frep::blend::SmoothUnion;
use opensolid_frep::csg::Subtraction;
use opensolid_frep::eval::gradient;
use opensolid_frep::primitives::{Box3, Cylinder, Sdf, Sphere};
use opensolid_frep::{MeshOptions, Shape, mesh_sdf};
use std::hint::black_box;

/// A representative 3-deep CSG tree: (box ⊔smooth sphere) − cylinder.
/// Mirrors the shape used by the kernel demo example.
fn demo_tree() -> impl Sdf {
    Subtraction {
        a: SmoothUnion {
            a: Box3 {
                center: Point3::origin(),
                half_extents: [0.75, 0.75, 0.75],
            },
            b: Sphere {
                center: Point3::new(0.6, 0.6, 0.6),
                radius: 0.6,
            },
            radius: 0.2,
        },
        b: Cylinder {
            center: Point3::origin(),
            radius: 0.4,
            half_height: 3.0,
        },
    }
}

/// The same tree built through the dyn-dispatch `Shape` API, to keep the
/// cost of runtime composition visible next to the zero-cost path.
fn demo_tree_dyn() -> Shape {
    let cube = Shape::new(Box3 {
        center: Point3::origin(),
        half_extents: [0.75, 0.75, 0.75],
    });
    let ball = Shape::new(Sphere {
        center: Point3::new(0.6, 0.6, 0.6),
        radius: 0.6,
    });
    let hole = Shape::new(Cylinder {
        center: Point3::origin(),
        radius: 0.4,
        half_height: 3.0,
    });
    cube.smooth_union(ball, 0.2).subtract(hole)
}

/// Query point near the surface, where meshing and root-finding spend
/// their time (avoids branch-predictable deep-inside/outside fast cases).
fn query_point() -> Point3 {
    Point3::new(0.71, 0.33, -0.52)
}

fn bench_eval(c: &mut Criterion) {
    let sphere = Sphere {
        center: Point3::origin(),
        radius: 1.0,
    };
    let tree = demo_tree();
    let tree_dyn = demo_tree_dyn();
    let p = query_point();

    let mut group = c.benchmark_group("eval");
    group.bench_function("sphere", |b| {
        b.iter(|| black_box(&sphere).eval(black_box(&p)))
    });
    group.bench_function("csg_tree_generic", |b| {
        b.iter(|| black_box(&tree).eval(black_box(&p)))
    });
    group.bench_function("csg_tree_dyn", |b| {
        b.iter(|| black_box(&tree_dyn).eval(black_box(&p)))
    });
    group.finish();
}

fn bench_gradient(c: &mut Criterion) {
    let tree = demo_tree();
    let p = query_point();

    let mut group = c.benchmark_group("gradient");
    group.bench_function("csg_tree", |b| {
        b.iter(|| gradient(black_box(&tree), black_box(&p)))
    });
    group.finish();
}

fn bench_mesh(c: &mut Criterion) {
    let tree = demo_tree();
    let bounds = BoundingBox3::new(Point3::new(-2.0, -2.0, -2.0), Point3::new(2.0, 2.0, 2.0));

    let mut group = c.benchmark_group("mesh");
    // Meshing is orders of magnitude slower than point eval; keep sample
    // counts low so `cargo bench` stays in CI-friendly territory.
    group.sample_size(20);
    for resolution in [32usize, 64] {
        let opts = MeshOptions { bounds, resolution };
        group.bench_function(format!("uniform_res{resolution}"), |b| {
            b.iter(|| mesh_sdf(black_box(&tree), black_box(&opts)))
        });
    }
    group.finish();
}

criterion_group!(benches, bench_eval, bench_gradient, bench_mesh);
criterion_main!(benches);
