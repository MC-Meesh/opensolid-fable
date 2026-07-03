//! Property-based tests for SDF metric invariants and mesher guarantees.
//!
//! Invariants checked over random primitives, CSG trees, and points:
//! - `Union` (and `SmoothUnion`) evaluates to at most the min of its
//!   components; `Intersection` to at least the max.
//! - Every field built from exact primitives and the metric-preserving
//!   combinators is 1-Lipschitz along random segments. All current
//!   combinators preserve the 1-Lipschitz bound: sharp `Union` /
//!   `Intersection` / `Subtraction` (min/max/negation of 1-Lipschitz
//!   functions), `SmoothUnion` / `SmoothSubtraction` (gradient is a convex
//!   combination of child gradients — see `blend.rs`), rigid `Transformed`
//!   and `UniformScale` (isometries / rescaled distances, see
//!   `transform.rs`). None of the CSG combinators preserve *exactness*:
//!   only the Lipschitz bound survives composition, which is why the
//!   gradient-norm ~ 1 check below is restricted to exact primitives.
//! - For exact primitives, `|grad|` is ~1 near the surface (away from
//!   non-smooth loci).
//! - The dual-contouring mesher emits every vertex within one cell
//!   (diagonal) of the SDF zero set for random spheres and boxes.
//!
//! Case counts are deliberately small to keep the suite CI-friendly (<5s).

use opensolid_core::types::{BoundingBox3, Point3, Vector3};
use opensolid_frep::blend::{SmoothSubtraction, SmoothUnion};
use opensolid_frep::csg::{Intersection, Subtraction, Union};
use opensolid_frep::primitives::{
    Box3, Capsule, Cone, Cylinder, HalfSpace, RoundedBox, Sdf, Sphere, Torus,
};
use opensolid_frep::{MeshOptions, mesh_sdf_indexed};
use proptest::prelude::*;

// ---------------------------------------------------------------------------
// Strategies: primitive descriptions and recursive CSG trees.
//
// Proptest shrinks over plain data, so we generate serializable descriptions
// and build `Box<dyn Sdf>` from them inside each test.
// ---------------------------------------------------------------------------

#[derive(Clone, Debug)]
enum Prim {
    Sphere {
        center: [f64; 3],
        radius: f64,
    },
    Box {
        center: [f64; 3],
        half: [f64; 3],
    },
    Cylinder {
        center: [f64; 3],
        radius: f64,
        half_height: f64,
    },
    Capsule {
        start: [f64; 3],
        end: [f64; 3],
        radius: f64,
    },
    Torus {
        center: [f64; 3],
        major: f64,
        minor: f64,
    },
    Cone {
        center: [f64; 3],
        half_height: f64,
        r_bottom: f64,
        r_top: f64,
    },
    RoundedBox {
        center: [f64; 3],
        half: [f64; 3],
        radius: f64,
    },
    HalfSpace {
        normal: [f64; 3],
        offset: f64,
    },
}

impl Prim {
    fn build(&self) -> Box<dyn Sdf> {
        fn pt(c: &[f64; 3]) -> Point3 {
            Point3::new(c[0], c[1], c[2])
        }
        match self {
            Prim::Sphere { center, radius } => Box::new(Sphere {
                center: pt(center),
                radius: *radius,
            }),
            Prim::Box { center, half } => Box::new(Box3 {
                center: pt(center),
                half_extents: *half,
            }),
            Prim::Cylinder {
                center,
                radius,
                half_height,
            } => Box::new(Cylinder {
                center: pt(center),
                radius: *radius,
                half_height: *half_height,
            }),
            Prim::Capsule { start, end, radius } => Box::new(Capsule {
                start: pt(start),
                end: pt(end),
                radius: *radius,
            }),
            Prim::Torus {
                center,
                major,
                minor,
            } => Box::new(Torus {
                center: pt(center),
                major_radius: *major,
                minor_radius: *minor,
            }),
            Prim::Cone {
                center,
                half_height,
                r_bottom,
                r_top,
            } => Box::new(Cone {
                center: pt(center),
                half_height: *half_height,
                radius_bottom: *r_bottom,
                radius_top: *r_top,
            }),
            Prim::RoundedBox {
                center,
                half,
                radius,
            } => Box::new(RoundedBox {
                center: pt(center),
                half_extents: *half,
                radius: *radius,
            }),
            Prim::HalfSpace { normal, offset } => Box::new(HalfSpace {
                normal: Vector3::new(normal[0], normal[1], normal[2]),
                offset: *offset,
            }),
        }
    }
}

fn coord() -> impl Strategy<Value = f64> {
    -1.5..1.5f64
}

fn center() -> impl Strategy<Value = [f64; 3]> {
    [coord(), coord(), coord()]
}

/// Exact-distance primitives (every primitive in the crate is an exact SDF).
/// `HalfSpace` is excluded from bounded contexts (mesher) but included here.
fn prim() -> impl Strategy<Value = Prim> {
    prop_oneof![
        (center(), 0.2..1.5f64).prop_map(|(center, radius)| Prim::Sphere { center, radius }),
        (center(), [0.2..1.5f64, 0.2..1.5f64, 0.2..1.5f64])
            .prop_map(|(center, half)| Prim::Box { center, half }),
        (center(), 0.2..1.2f64, 0.2..1.2f64).prop_map(|(center, radius, half_height)| {
            Prim::Cylinder {
                center,
                radius,
                half_height,
            }
        }),
        (center(), center(), 0.2..1.0f64)
            .prop_filter("capsule axis must not be degenerate", |(s, e, _)| {
                let d = [e[0] - s[0], e[1] - s[1], e[2] - s[2]];
                (d[0] * d[0] + d[1] * d[1] + d[2] * d[2]).sqrt() > 0.2
            })
            .prop_map(|(start, end, radius)| Prim::Capsule { start, end, radius }),
        (center(), 0.8..2.0f64, 0.1..0.6f64).prop_map(|(center, major, minor)| Prim::Torus {
            center,
            major,
            minor
        }),
        (center(), 0.3..1.2f64, 0.3..1.2f64, 0.0..0.8f64).prop_map(
            |(center, half_height, r_bottom, r_top)| Prim::Cone {
                center,
                half_height,
                r_bottom,
                r_top,
            }
        ),
        (
            center(),
            [0.3..1.2f64, 0.3..1.2f64, 0.3..1.2f64],
            0.0..0.25f64
        )
            .prop_map(|(center, half, radius)| Prim::RoundedBox {
                center,
                half,
                radius
            }),
        (center(), -1.0..1.0f64)
            .prop_filter("half-space normal must not be near zero", |(n, _)| {
                (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt() > 0.3
            })
            .prop_map(|(normal, offset)| Prim::HalfSpace { normal, offset }),
    ]
}

#[derive(Clone, Debug)]
enum Tree {
    Leaf(Prim),
    Union(Box<Tree>, Box<Tree>),
    Intersection(Box<Tree>, Box<Tree>),
    Subtraction(Box<Tree>, Box<Tree>),
    SmoothUnion(Box<Tree>, Box<Tree>, f64),
    SmoothSubtraction(Box<Tree>, Box<Tree>, f64),
}

impl Tree {
    fn build(&self) -> Box<dyn Sdf> {
        match self {
            Tree::Leaf(p) => p.build(),
            Tree::Union(a, b) => Box::new(Union {
                a: a.build(),
                b: b.build(),
            }),
            Tree::Intersection(a, b) => Box::new(Intersection {
                a: a.build(),
                b: b.build(),
            }),
            Tree::Subtraction(a, b) => Box::new(Subtraction {
                a: a.build(),
                b: b.build(),
            }),
            Tree::SmoothUnion(a, b, r) => Box::new(SmoothUnion {
                a: a.build(),
                b: b.build(),
                radius: *r,
            }),
            Tree::SmoothSubtraction(a, b, r) => Box::new(SmoothSubtraction {
                a: a.build(),
                b: b.build(),
                radius: *r,
            }),
        }
    }
}

/// Random CSG trees up to depth 3 over the exact primitives, mixing sharp
/// and smooth combinators.
fn tree() -> impl Strategy<Value = Tree> {
    prim()
        .prop_map(Tree::Leaf)
        .prop_recursive(3, 16, 2, |leaf| {
            prop_oneof![
                (leaf.clone(), leaf.clone())
                    .prop_map(|(a, b)| Tree::Union(Box::new(a), Box::new(b))),
                (leaf.clone(), leaf.clone())
                    .prop_map(|(a, b)| Tree::Intersection(Box::new(a), Box::new(b))),
                (leaf.clone(), leaf.clone())
                    .prop_map(|(a, b)| Tree::Subtraction(Box::new(a), Box::new(b))),
                (leaf.clone(), leaf.clone(), 0.05..0.5f64).prop_map(|(a, b, r)| Tree::SmoothUnion(
                    Box::new(a),
                    Box::new(b),
                    r
                )),
                (leaf.clone(), leaf, 0.05..0.5f64).prop_map(|(a, b, r)| Tree::SmoothSubtraction(
                    Box::new(a),
                    Box::new(b),
                    r
                )),
            ]
        })
}

fn point() -> impl Strategy<Value = Point3> {
    (-4.0..4.0f64, -4.0..4.0f64, -4.0..4.0f64).prop_map(|(x, y, z)| Point3::new(x, y, z))
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/// Newton-project a seed onto the zero set. Exact SDFs converge in a step or
/// two; returns `None` if the gradient vanishes (medial axis) or the
/// iteration fails to land near the surface.
fn project_to_surface(sdf: &dyn Sdf, seed: Point3) -> Option<Point3> {
    let mut p = seed;
    for _ in 0..12 {
        let d = sdf.eval(&p);
        if d.abs() < 1e-9 {
            return Some(p);
        }
        let g = sdf.grad(&p);
        let n2 = g.norm_squared();
        if n2 < 1e-12 {
            return None;
        }
        p -= g * (d / n2);
    }
    (sdf.eval(&p).abs() < 1e-7).then_some(p)
}

/// SDFs are non-smooth on measure-zero loci (edges, branch ties, medial
/// axes); the gradient-norm invariant only holds where the field is
/// differentiable. Detect non-smooth points the same way as
/// `grad_fd_agreement.rs`: forward and backward differences must agree.
fn locally_smooth(sdf: &dyn Sdf, p: &Point3) -> bool {
    let h = 1e-4;
    for axis in 0..3 {
        let mut lo = *p;
        let mut hi = *p;
        lo[axis] -= h;
        hi[axis] += h;
        let f = sdf.eval(p);
        let forward = (sdf.eval(&hi) - f) / h;
        let backward = (f - sdf.eval(&lo)) / h;
        if (forward - backward).abs() > 1e-2 {
            return false;
        }
    }
    true
}

fn cube_bounds(half: f64) -> BoundingBox3 {
    BoundingBox3::new(
        Point3::new(-half, -half, -half),
        Point3::new(half, half, half),
    )
}

// ---------------------------------------------------------------------------
// Properties
// ---------------------------------------------------------------------------

proptest! {
    #![proptest_config(ProptestConfig::with_cases(96))]

    /// Sharp union equals min of its components; smooth union never exceeds
    /// it (the blend only carves *outward*, deepening the field).
    #[test]
    fn union_at_most_min_of_components(a in tree(), b in tree(), p in point(), r in 0.05..0.5f64) {
        let (sa, sb) = (a.build(), b.build());
        let (da, db) = (sa.eval(&p), sb.eval(&p));
        let sharp = Union { a: sa, b: sb };
        prop_assert!(sharp.eval(&p) <= da.min(db) + 1e-12);
        let smooth = SmoothUnion { a: sharp.a, b: sharp.b, radius: r };
        prop_assert!(smooth.eval(&p) <= da.min(db) + 1e-12);
    }

    /// Intersection is at least the max of its components (equal, for the
    /// sharp max-combinator).
    #[test]
    fn intersection_at_least_max_of_components(a in tree(), b in tree(), p in point()) {
        let (sa, sb) = (a.build(), b.build());
        let (da, db) = (sa.eval(&p), sb.eval(&p));
        let inter = Intersection { a: sa, b: sb };
        prop_assert!(inter.eval(&p) >= da.max(db) - 1e-12);
    }

    /// Subtraction can only shrink the solid: the field never drops below
    /// the base operand's.
    #[test]
    fn subtraction_at_least_base(a in tree(), b in tree(), p in point()) {
        let sa = a.build();
        let da = sa.eval(&p);
        let sub = Subtraction { a: sa, b: b.build() };
        prop_assert!(sub.eval(&p) >= da - 1e-12);
    }

    /// Every field built from exact primitives and the metric-preserving
    /// combinators satisfies |f(p) - f(q)| <= |p - q|, checked pairwise
    /// along random segments (stronger than endpoints only).
    #[test]
    fn eval_is_one_lipschitz_along_segments(t in tree(), p in point(), q in point()) {
        let sdf = t.build();
        const STEPS: usize = 8;
        let mut prev_p = p;
        let mut prev_v = sdf.eval(&p);
        for i in 1..=STEPS {
            let s = i as f64 / STEPS as f64;
            let pi = Point3::from(p.coords.lerp(&q.coords, s));
            let vi = sdf.eval(&pi);
            let dist = (pi - prev_p).norm();
            prop_assert!(
                (vi - prev_v).abs() <= dist + 1e-9,
                "Lipschitz violated: |{vi} - {prev_v}| > {dist} at {pi:?}"
            );
            prev_p = pi;
            prev_v = vi;
        }
    }
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(64))]

    /// Exact primitives have |grad| ~ 1 near the surface, away from
    /// non-smooth loci. CSG trees are deliberately excluded: composition
    /// preserves only the Lipschitz bound, not exactness.
    #[test]
    fn gradient_norm_near_one_at_surface(
        pr in prim(),
        seed in point(),
        jitter in [-0.02..0.02f64, -0.02..0.02f64, -0.02..0.02f64],
    ) {
        let sdf = pr.build();
        let Some(surface) = project_to_surface(sdf.as_ref(), seed) else {
            return Ok(()); // seed hit a gradient-free locus (e.g. sphere center)
        };
        let near = surface + Vector3::new(jitter[0], jitter[1], jitter[2]);
        if !locally_smooth(sdf.as_ref(), &near) {
            return Ok(()); // edge / branch tie: any subgradient is legal
        }
        let norm = sdf.grad(&near).norm();
        prop_assert!(
            (norm - 1.0).abs() < 1e-4,
            "gradient norm {norm} at {near:?} for {pr:?}"
        );
    }
}

// ---------------------------------------------------------------------------
// Mesher invariants: every emitted vertex lies within one cell (diagonal)
// of the zero set. The SDFs used are exact, so |eval(v)| is the true
// distance to the surface.
// ---------------------------------------------------------------------------

/// Dual contouring places each vertex inside a cell the surface crosses, so
/// no vertex can be farther from the zero set than a cell diagonal.
fn assert_vertices_within_one_cell(
    sdf: &dyn Sdf,
    half: f64,
    resolution: usize,
) -> Result<(), TestCaseError> {
    let opts = MeshOptions {
        bounds: cube_bounds(half),
        resolution,
    };
    let mesh = mesh_sdf_indexed(sdf, &opts);
    prop_assert!(!mesh.is_empty(), "mesh unexpectedly empty");
    let cell_diagonal = (2.0 * half / resolution as f64) * 3.0f64.sqrt();
    for v in &mesh.positions {
        let d = sdf.eval(v).abs();
        prop_assert!(
            d <= cell_diagonal,
            "vertex {v:?} is {d} from the zero set (> cell diagonal {cell_diagonal})"
        );
    }
    Ok(())
}

proptest! {
    #![proptest_config(ProptestConfig::with_cases(16))]

    #[test]
    fn mesher_vertices_near_zero_set_sphere(
        c in [-0.3..0.3f64, -0.3..0.3f64, -0.3..0.3f64],
        radius in 0.5..1.2f64,
    ) {
        let s = Sphere {
            center: Point3::new(c[0], c[1], c[2]),
            radius,
        };
        // Surface fits strictly inside |x| <= 0.3 + 1.2 = 1.5 < 2.0.
        assert_vertices_within_one_cell(&s, 2.0, 12)?;
    }

    #[test]
    fn mesher_vertices_near_zero_set_box(
        c in [-0.2..0.2f64, -0.2..0.2f64, -0.2..0.2f64],
        half in [0.4..1.2f64, 0.4..1.2f64, 0.4..1.2f64],
    ) {
        let b = Box3 {
            center: Point3::new(c[0], c[1], c[2]),
            half_extents: half,
        };
        // Surface fits strictly inside |x| <= 0.2 + 1.2 = 1.4 < 2.0.
        assert_vertices_within_one_cell(&b, 2.0, 12)?;
    }
}
