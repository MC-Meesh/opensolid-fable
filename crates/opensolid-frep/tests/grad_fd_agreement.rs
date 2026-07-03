//! Every analytic `Sdf::grad` override must agree with the central
//! finite-difference default to 1e-6 at random points.

use opensolid_core::types::{Point3, Vector3};
use opensolid_frep::blend::{SmoothSubtraction, SmoothUnion};
use opensolid_frep::csg::{Intersection, Subtraction, Union};
use opensolid_frep::primitives::{Box3, Capsule, Cylinder, HalfSpace, Sdf, Sphere};

/// Forwards `eval` only, so `grad` falls back to the trait's
/// finite-difference default even when the inner type overrides it.
struct FdOnly<'a>(&'a dyn Sdf);

impl Sdf for FdOnly<'_> {
    fn eval(&self, p: &Point3) -> f64 {
        self.0.eval(p)
    }
}

/// Deterministic LCG (Numerical Recipes constants) so the "random"
/// sample set is reproducible without a rand dependency.
struct Lcg(u64);

impl Lcg {
    fn next_f64(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    /// Uniform in [-3, 3]^3.
    fn point(&mut self) -> Point3 {
        Point3::new(
            self.next_f64() * 6.0 - 3.0,
            self.next_f64() * 6.0 - 3.0,
            self.next_f64() * 6.0 - 3.0,
        )
    }
}

const SAMPLES: usize = 200;
const TOL: f64 = 1e-6;

/// SDFs are non-smooth on measure-zero loci (edges, branch ties); central
/// differences straddling such a locus disagree with any one-sided analytic
/// subgradient. Skip a sample only when the FD probe itself shows the field
/// is locally non-smooth (forward and backward differences disagree).
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

fn assert_grad_matches_fd(sdf: &dyn Sdf, seed: u64) {
    let mut rng = Lcg(seed);
    let mut checked = 0;
    for _ in 0..SAMPLES {
        let p = rng.point();
        if !locally_smooth(sdf, &p) {
            continue;
        }
        let analytic = sdf.grad(&p);
        let fd = FdOnly(sdf).grad(&p);
        let diff = (analytic - fd).norm();
        assert!(
            diff < TOL,
            "analytic {analytic:?} vs FD {fd:?} differ by {diff:e} at {p:?}"
        );
        checked += 1;
    }
    // The smoothness filter must not silently discard the whole sample set.
    assert!(
        checked > SAMPLES / 2,
        "only {checked}/{SAMPLES} points checked"
    );
}

fn sphere(x: f64, r: f64) -> Sphere {
    Sphere {
        center: Point3::new(x, 0.0, 0.0),
        radius: r,
    }
}

#[test]
fn sphere_grad_matches_fd() {
    let s = Sphere {
        center: Point3::new(0.3, -0.2, 0.5),
        radius: 1.2,
    };
    assert_grad_matches_fd(&s, 1);
}

#[test]
fn box3_grad_matches_fd() {
    let b = Box3 {
        center: Point3::new(0.1, 0.2, -0.3),
        half_extents: [1.0, 0.6, 1.4],
    };
    assert_grad_matches_fd(&b, 2);
}

#[test]
fn cylinder_grad_matches_fd() {
    let c = Cylinder {
        center: Point3::new(-0.2, 0.1, 0.4),
        radius: 0.8,
        half_height: 1.1,
    };
    assert_grad_matches_fd(&c, 3);
}

#[test]
fn half_space_grad_matches_fd() {
    let h = HalfSpace {
        normal: Vector3::new(1.0, -2.0, 0.5),
        offset: 0.7,
    };
    assert_grad_matches_fd(&h, 4);
}

#[test]
fn capsule_grad_matches_fd() {
    let c = Capsule {
        start: Point3::new(-0.8, -1.0, 0.2),
        end: Point3::new(0.5, 1.2, -0.4),
        radius: 0.6,
    };
    assert_grad_matches_fd(&c, 5);
}

#[test]
fn union_grad_matches_fd() {
    let u = Union {
        a: sphere(-0.7, 1.0),
        b: sphere(0.7, 1.2),
    };
    assert_grad_matches_fd(&u, 6);
}

#[test]
fn intersection_grad_matches_fd() {
    let i = Intersection {
        a: sphere(-0.4, 1.5),
        b: Box3 {
            center: Point3::origin(),
            half_extents: [1.2, 0.9, 1.0],
        },
    };
    assert_grad_matches_fd(&i, 7);
}

#[test]
fn subtraction_grad_matches_fd() {
    let s = Subtraction {
        a: sphere(0.0, 1.8),
        b: Cylinder {
            center: Point3::origin(),
            radius: 0.5,
            half_height: 2.5,
        },
    };
    assert_grad_matches_fd(&s, 8);
}

#[test]
fn smooth_union_grad_matches_fd() {
    let su = SmoothUnion {
        a: sphere(-0.6, 1.0),
        b: sphere(0.6, 0.9),
        radius: 0.4,
    };
    assert_grad_matches_fd(&su, 9);
}

#[test]
fn smooth_subtraction_grad_matches_fd() {
    let ss = SmoothSubtraction {
        a: sphere(0.0, 1.5),
        b: sphere(0.8, 0.7),
        radius: 0.3,
    };
    assert_grad_matches_fd(&ss, 10);
}

#[test]
fn nested_tree_grad_matches_fd() {
    let tree = SmoothUnion {
        a: Subtraction {
            a: Union {
                a: sphere(-0.5, 1.0),
                b: Box3 {
                    center: Point3::origin(),
                    half_extents: [0.8, 0.8, 0.8],
                },
            },
            b: Cylinder {
                center: Point3::origin(),
                radius: 0.3,
                half_height: 2.0,
            },
        },
        b: Capsule {
            start: Point3::new(1.0, -1.0, 0.0),
            end: Point3::new(1.5, 1.0, 0.5),
            radius: 0.4,
        },
        radius: 0.25,
    };
    assert_grad_matches_fd(&tree, 11);
}
