//! Two contracts for the differentiable field tower.
//!
//! 1. **No drift.** The tower ([`diff::field`]) restates every primitive and
//!    operator over a generic scalar. That duplication is the design's main
//!    risk: a formula could be fixed in `primitives.rs` and missed here, and
//!    nothing in the type system would notice. So every tower function is
//!    pinned against its `Sdf` counterpart at random points.
//!
//! 2. **Correct derivatives.** Every dual-number parameter gradient is
//!    checked against central finite differences of the *same* field.
//!
//! Both are checked away from kinks: `min`/`max` are non-differentiable where
//! branches tie, and FD straddles the kink there, so the comparison would be
//! meaningless. See `locally_smooth`.

use opensolid_core::types::Point3;
use opensolid_frep::blend::{SmoothSubtraction, SmoothUnion};
use opensolid_frep::csg::{Intersection, Subtraction, Union};
use opensolid_frep::diff::{ParamSdf, Scalar, Vec3, field};
use opensolid_frep::ops::{SdfOpsExt, Shell};
use opensolid_frep::primitives::{
    Box3, Capsule, Cone, Cylinder, HalfSpace, RoundedBox, Sdf, Sphere, Torus,
};

/// Deterministic LCG (Numerical Recipes constants), matching the style of
/// `grad_fd_agreement.rs` — reproducible sampling with no rand dependency.
struct Lcg(u64);

impl Lcg {
    fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((self.0 >> 11) as f64) / ((1u64 << 53) as f64)
    }

    /// A point in the cube [-2, 2]³.
    fn point(&mut self) -> Point3 {
        Point3::new(
            self.next_f64() * 4.0 - 2.0,
            self.next_f64() * 4.0 - 2.0,
            self.next_f64() * 4.0 - 2.0,
        )
    }
}

fn v(p: &Point3) -> Vec3<f64> {
    Vec3::from_point(p)
}

// ------------------------------------------------------- 1. no-drift checks

/// The tower at `f64` must reproduce the `Sdf` impl it mirrors.
fn assert_tower_matches(sdf: &dyn Sdf, tower: impl Fn(&Point3) -> f64, seed: u64, name: &str) {
    let mut rng = Lcg(seed);
    for _ in 0..400 {
        let p = rng.point();
        let (a, b) = (sdf.eval(&p), tower(&p));
        assert!(
            (a - b).abs() < 1e-12,
            "{name} drifted from its Sdf impl at {p:?}: Sdf {a} vs tower {b}"
        );
    }
}

#[test]
fn sphere_tower_matches_sdf() {
    let s = Sphere {
        center: Point3::new(0.3, -0.2, 0.1),
        radius: 0.9,
    };
    assert_tower_matches(
        &s,
        |p| field::sphere(v(p), Vec3::from_point(&s.center), s.radius),
        1,
        "sphere",
    );
}

#[test]
fn box3_tower_matches_sdf() {
    let b = Box3 {
        center: Point3::new(-0.1, 0.4, 0.2),
        half_extents: [0.7, 1.1, 0.5],
    };
    assert_tower_matches(
        &b,
        |p| {
            field::box3(
                v(p),
                Vec3::from_point(&b.center),
                Vec3::new(b.half_extents[0], b.half_extents[1], b.half_extents[2]),
            )
        },
        2,
        "box3",
    );
}

#[test]
fn rounded_box_tower_matches_sdf() {
    let b = RoundedBox {
        center: Point3::new(0.2, 0.0, -0.3),
        half_extents: [0.8, 0.6, 1.0],
        radius: 0.2,
    };
    assert_tower_matches(
        &b,
        |p| {
            field::rounded_box(
                v(p),
                Vec3::from_point(&b.center),
                Vec3::new(b.half_extents[0], b.half_extents[1], b.half_extents[2]),
                b.radius,
            )
        },
        3,
        "rounded_box",
    );
}

#[test]
fn cylinder_tower_matches_sdf() {
    let c = Cylinder {
        center: Point3::new(0.1, 0.2, -0.1),
        radius: 0.7,
        half_height: 0.9,
    };
    assert_tower_matches(
        &c,
        |p| field::cylinder(v(p), Vec3::from_point(&c.center), c.radius, c.half_height),
        4,
        "cylinder",
    );
}

#[test]
fn torus_tower_matches_sdf() {
    let t = Torus {
        center: Point3::new(0.0, 0.1, 0.0),
        major_radius: 0.9,
        minor_radius: 0.3,
    };
    assert_tower_matches(
        &t,
        |p| {
            field::torus(
                v(p),
                Vec3::from_point(&t.center),
                t.major_radius,
                t.minor_radius,
            )
        },
        5,
        "torus",
    );
}

#[test]
fn cone_tower_matches_sdf() {
    let c = Cone {
        center: Point3::new(0.0, 0.0, 0.0),
        half_height: 0.8,
        radius_bottom: 0.9,
        radius_top: 0.3,
    };
    assert_tower_matches(
        &c,
        |p| {
            field::cone(
                v(p),
                Vec3::from_point(&c.center),
                c.half_height,
                c.radius_bottom,
                c.radius_top,
            )
        },
        6,
        "cone",
    );
}

#[test]
fn capsule_tower_matches_sdf() {
    let c = Capsule {
        start: Point3::new(-0.5, -0.3, 0.0),
        end: Point3::new(0.6, 0.4, 0.2),
        radius: 0.35,
    };
    assert_tower_matches(
        &c,
        |p| {
            field::capsule(
                v(p),
                Vec3::from_point(&c.start),
                Vec3::from_point(&c.end),
                c.radius,
            )
        },
        7,
        "capsule",
    );
}

#[test]
fn half_space_tower_matches_sdf() {
    let h = HalfSpace {
        normal: opensolid_core::types::Vector3::new(0.3, 1.0, -0.4),
        offset: 0.25,
    };
    assert_tower_matches(
        &h,
        |p| {
            field::half_space(
                v(p),
                Vec3::new(h.normal.x, h.normal.y, h.normal.z),
                h.offset,
            )
        },
        8,
        "half_space",
    );
}

#[test]
fn sharp_csg_tower_matches_sdf() {
    let a = Sphere {
        center: Point3::new(-0.3, 0.0, 0.0),
        radius: 0.8,
    };
    let b = Box3 {
        center: Point3::new(0.3, 0.0, 0.0),
        half_extents: [0.5, 0.5, 0.5],
    };
    let fa = |p: &Point3| field::sphere(v(p), Vec3::cst(-0.3, 0.0, 0.0), 0.8);
    let fb = |p: &Point3| field::box3(v(p), Vec3::cst(0.3, 0.0, 0.0), Vec3::splat(0.5));

    let u = Union {
        a: Sphere {
            center: a.center,
            radius: a.radius,
        },
        b: Box3 {
            center: b.center,
            half_extents: b.half_extents,
        },
    };
    assert_tower_matches(&u, |p| field::union(fa(p), fb(p)), 9, "union");

    let i = Intersection {
        a: Sphere {
            center: a.center,
            radius: a.radius,
        },
        b: Box3 {
            center: b.center,
            half_extents: b.half_extents,
        },
    };
    assert_tower_matches(
        &i,
        |p| field::intersection(fa(p), fb(p)),
        10,
        "intersection",
    );

    let s = Subtraction {
        a: Sphere {
            center: a.center,
            radius: a.radius,
        },
        b: Box3 {
            center: b.center,
            half_extents: b.half_extents,
        },
    };
    assert_tower_matches(&s, |p| field::subtraction(fa(p), fb(p)), 11, "subtraction");
}

#[test]
fn smooth_csg_tower_matches_sdf() {
    let fa = |p: &Point3| field::sphere(v(p), Vec3::cst(-0.3, 0.0, 0.0), 0.8);
    let fb = |p: &Point3| field::box3(v(p), Vec3::cst(0.3, 0.0, 0.0), Vec3::splat(0.5));
    let mk = || {
        (
            Sphere {
                center: Point3::new(-0.3, 0.0, 0.0),
                radius: 0.8,
            },
            Box3 {
                center: Point3::new(0.3, 0.0, 0.0),
                half_extents: [0.5, 0.5, 0.5],
            },
        )
    };

    let (a, b) = mk();
    let su = SmoothUnion { a, b, radius: 0.4 };
    assert_tower_matches(
        &su,
        |p| field::smooth_union(fa(p), fb(p), 0.4),
        12,
        "smooth_union",
    );

    let (a, b) = mk();
    let ss = SmoothSubtraction { a, b, radius: 0.4 };
    assert_tower_matches(
        &ss,
        |p| field::smooth_subtraction(fa(p), fb(p), 0.4),
        13,
        "smooth_subtraction",
    );
}

#[test]
fn offset_family_tower_matches_sdf() {
    let base = || Sphere {
        center: Point3::origin(),
        radius: 0.8,
    };
    let fs = |p: &Point3| field::sphere(v(p), Vec3::zero(), 0.8);

    let off = base().offset(0.2).expect("valid offset");
    assert_tower_matches(&off, |p| field::offset(fs(p), 0.2), 14, "offset");

    let sh: Shell<Sphere> = base().shell(0.3).expect("valid shell");
    assert_tower_matches(&sh, |p| field::shell(fs(p), 0.3), 15, "shell");

    let r = base().rounded(0.15).expect("valid round");
    assert_tower_matches(&r, |p| field::rounded(fs(p), 0.15), 16, "rounded");
}

// ------------------------------------------- 2. parameter-gradient checks

/// True if no `min`/`max` branch in this shape is close to tying at `p`, so
/// the field is locally smooth and FD is meaningful.
fn locally_smooth<S: ParamSdf<N>, const N: usize>(
    shape: &S,
    p: &Point3,
    params: &[f64; N],
    h: f64,
) -> bool {
    // A kink shows up as FD disagreeing with itself at two step sizes.
    let a = shape.grad_fd(p, params, h);
    let b = shape.grad_fd(p, params, h * 4.0);
    (0..N).all(|i| (a[i] - b[i]).abs() < 1e-4)
}

fn assert_param_grad_matches_fd<S: ParamSdf<N>, const N: usize>(
    shape: &S,
    params: &[f64; N],
    seed: u64,
    name: &str,
) {
    let mut rng = Lcg(seed);
    let mut checked = 0;
    for _ in 0..600 {
        let p = rng.point();
        if !locally_smooth(shape, &p, params, 1e-5) {
            continue; // on a kink — FD is not a valid reference
        }
        let (_, g) = shape.value_and_grad(&p, params);
        let fd = shape.grad_fd(&p, params, 1e-5);
        for i in 0..N {
            assert!(
                (g[i] - fd[i]).abs() < 1e-5,
                "{name} param {i} ({}) at {p:?}: dual {} vs fd {}",
                shape.param_names()[i],
                g[i],
                fd[i]
            );
        }
        checked += 1;
    }
    assert!(
        checked > 100,
        "{name}: only {checked} smooth samples — test is not exercising much"
    );
}

/// Every primitive with all of its dimensions as live parameters.
struct AllPrimitives;

impl ParamSdf<9> for AllPrimitives {
    fn field<T: Scalar>(&self, p: Vec3<T>, q: &[T; 9]) -> T {
        let s = field::sphere(p, Vec3::cst(-1.2, 0.0, 0.0), q[0]);
        let b = field::box3(p, Vec3::cst(1.2, 0.0, 0.0), Vec3::new(q[1], q[2], q[3]));
        let c = field::cylinder(p, Vec3::cst(0.0, 1.2, 0.0), q[4], q[5]);
        let t = field::torus(p, Vec3::cst(0.0, -1.2, 0.0), q[6], q[7]);
        let k = field::capsule(p, Vec3::cst(0.0, 0.0, 1.0), Vec3::cst(0.0, 0.0, 2.0), q[8]);
        field::union(field::union(field::union(s, b), field::union(c, t)), k)
    }

    fn param_names(&self) -> [&'static str; 9] {
        [
            "sphere_r", "box_hx", "box_hy", "box_hz", "cyl_r", "cyl_h", "tor_R", "tor_r", "cap_r",
        ]
    }
}

#[test]
fn primitive_param_grads_match_fd() {
    assert_param_grad_matches_fd(
        &AllPrimitives,
        &[0.8, 0.5, 0.6, 0.7, 0.5, 0.6, 0.7, 0.25, 0.3],
        21,
        "primitives",
    );
}

/// Cone and rounded box, whose fields have the fiddliest closed forms.
struct TrickyPrimitives;

impl ParamSdf<5> for TrickyPrimitives {
    fn field<T: Scalar>(&self, p: Vec3<T>, q: &[T; 5]) -> T {
        let c = field::cone(p, Vec3::cst(-1.0, 0.0, 0.0), q[0], q[1], q[2]);
        let r = field::rounded_box(p, Vec3::cst(1.0, 0.0, 0.0), Vec3::splat(q[3]), q[4]);
        field::union(c, r)
    }

    fn param_names(&self) -> [&'static str; 5] {
        ["cone_h", "cone_r1", "cone_r2", "rbox_h", "rbox_r"]
    }
}

#[test]
fn tricky_primitive_param_grads_match_fd() {
    assert_param_grad_matches_fd(&TrickyPrimitives, &[0.8, 0.9, 0.3, 0.7, 0.2], 22, "tricky");
}

/// Every operator in the tower, stacked, with parameters threaded through
/// each — the composition is where chain-rule mistakes surface.
struct AllOperators;

impl ParamSdf<6> for AllOperators {
    fn field<T: Scalar>(&self, p: Vec3<T>, q: &[T; 6]) -> T {
        let a = field::sphere(p, Vec3::cst(-0.4, 0.0, 0.0), q[0]);
        let b = field::box3(p, Vec3::cst(0.4, 0.0, 0.0), Vec3::splat(q[1]));
        let blended = field::smooth_union(a, b, q[2]);
        let cutter = field::cylinder(p, Vec3::zero(), q[3], T::cst(3.0));
        let cut = field::smooth_subtraction(blended, cutter, q[4]);
        field::offset(cut, q[5])
    }

    fn param_names(&self) -> [&'static str; 6] {
        ["sphere_r", "box_h", "blend_k", "hole_r", "cut_k", "grow"]
    }
}

#[test]
fn operator_stack_param_grads_match_fd() {
    assert_param_grad_matches_fd(
        &AllOperators,
        &[0.7, 0.5, 0.3, 0.25, 0.2, 0.05],
        23,
        "operators",
    );
}

/// Transforms: translation and uniform scale carrying parameters.
struct Transforms;

impl ParamSdf<3> for Transforms {
    fn field<T: Scalar>(&self, p: Vec3<T>, q: &[T; 3]) -> T {
        // Translate by (q0, 0, 0), then a scaled sphere of radius q2.
        let moved = field::translate(p, Vec3::new(q[0], T::zero(), T::zero()));
        field::uniform_scale(moved, q[1], |inner| {
            field::sphere(inner, Vec3::zero(), q[2])
        })
    }

    fn param_names(&self) -> [&'static str; 3] {
        ["shift_x", "scale", "radius"]
    }
}

#[test]
fn transform_param_grads_match_fd() {
    assert_param_grad_matches_fd(&Transforms, &[0.3, 1.4, 0.6], 24, "transforms");
}

/// Sharp CSG has correct gradients *almost* everywhere — but on the seam
/// where branches tie, the parameter derivative is a subgradient and FD
/// straddles the kink. Pin that this is the only place they differ.
#[test]
fn sharp_csg_subgradient_picks_the_winning_branch() {
    struct TwoBalls;
    impl ParamSdf<2> for TwoBalls {
        fn field<T: Scalar>(&self, p: Vec3<T>, q: &[T; 2]) -> T {
            let a = field::sphere(p, Vec3::cst(-1.0, 0.0, 0.0), q[0]);
            let b = field::sphere(p, Vec3::cst(1.0, 0.0, 0.0), q[1]);
            field::union(a, b)
        }
    }
    // Nearer the left ball: only its radius moves the field.
    let (_, g) = TwoBalls.value_and_grad(&Point3::new(-2.0, 0.0, 0.0), &[0.5, 0.5]);
    assert_eq!(g, [-1.0, 0.0]);

    // Nearer the right ball: the sensitivity swaps entirely.
    let (_, g) = TwoBalls.value_and_grad(&Point3::new(2.0, 0.0, 0.0), &[0.5, 0.5]);
    assert_eq!(g, [0.0, -1.0]);
}

/// The smooth blend's whole point: on the seam, where sharp CSG hands back a
/// one-sided subgradient, the blend is differentiable and *both* parameters
/// get a share of the sensitivity.
#[test]
fn smooth_union_gives_both_branches_gradient_on_the_seam() {
    struct TwoBalls;
    impl ParamSdf<2> for TwoBalls {
        fn field<T: Scalar>(&self, p: Vec3<T>, q: &[T; 2]) -> T {
            let a = field::sphere(p, Vec3::cst(-1.0, 0.0, 0.0), q[0]);
            let b = field::sphere(p, Vec3::cst(1.0, 0.0, 0.0), q[1]);
            field::smooth_union(a, b, T::cst(0.5))
        }
    }
    // Equidistant from both balls — exactly the tie that kinks sharp CSG.
    let (_, g) = TwoBalls.value_and_grad(&Point3::new(0.0, 0.0, 0.0), &[0.5, 0.5]);
    assert!(
        g[0] < -0.1 && g[1] < -0.1,
        "both radii must matter on the seam: {g:?}"
    );
    assert!(
        (g[0] - g[1]).abs() < 1e-12,
        "symmetric point must be symmetric: {g:?}"
    );
}
