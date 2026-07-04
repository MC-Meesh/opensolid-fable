//! Shared helpers for `eval_interval` containment tests.

use crate::primitives::Sdf;
use opensolid_core::types::{BoundingBox3, Point3};

/// Deterministic 64-bit LCG (Knuth's MMIX constants) so tests need no
/// external `rand` dependency and never flake.
pub struct Lcg(pub u64);

impl Lcg {
    /// Uniform in `[0, 1)`.
    pub fn unit(&mut self) -> f64 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn in_range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }
}

/// A random non-empty box with corners in `[-extent, extent]^3`.
pub fn random_box(rng: &mut Lcg, extent: f64) -> BoundingBox3 {
    let axis = |rng: &mut Lcg| {
        let a = rng.in_range(-extent, extent);
        let b = rng.in_range(-extent, extent);
        (a.min(b), a.max(b))
    };
    let (x0, x1) = axis(rng);
    let (y0, y1) = axis(rng);
    let (z0, z1) = axis(rng);
    BoundingBox3::new(Point3::new(x0, y0, z0), Point3::new(x1, y1, z1))
}

/// Corners, center, and `n` random interior points of `b`.
pub fn sample_points(b: &BoundingBox3, rng: &mut Lcg, n: usize) -> Vec<Point3> {
    let mut pts = Vec::with_capacity(9 + n);
    for i in 0..8 {
        pts.push(Point3::new(
            if i & 1 == 0 { b.min.x } else { b.max.x },
            if i & 2 == 0 { b.min.y } else { b.max.y },
            if i & 4 == 0 { b.min.z } else { b.max.z },
        ));
    }
    pts.push(b.center());
    for _ in 0..n {
        pts.push(Point3::new(
            rng.in_range(b.min.x, b.max.x),
            rng.in_range(b.min.y, b.max.y),
            rng.in_range(b.min.z, b.max.z),
        ));
    }
    pts
}

/// The fundamental soundness property of `eval_interval`: for many random
/// boxes at mixed scales, the field value at every sampled point of the box
/// lies inside the box's interval. The `1e-9` slack absorbs the documented
/// 1-ulp under-coverage of round-to-nearest interval endpoints.
pub fn assert_interval_containment(sdf: &dyn Sdf, seed: u64) {
    let mut rng = Lcg(seed);
    for round in 0..200 {
        // Mix large boxes (spanning the shape) with small ones (deep
        // inside/outside), the two regimes octree refinement produces.
        let extent = if round % 3 == 0 { 4.0 } else { 0.5 };
        let b = random_box(&mut rng, extent);
        let i = sdf.eval_interval(&b);
        assert!(
            i.lo <= i.hi,
            "invalid interval [{}, {}] for box {b:?}",
            i.lo,
            i.hi
        );
        for p in sample_points(&b, &mut rng, 20) {
            let d = sdf.eval(&p);
            assert!(
                i.lo - 1e-9 <= d && d <= i.hi + 1e-9,
                "eval({p:?}) = {d} outside eval_interval({b:?}) = [{}, {}]",
                i.lo,
                i.hi,
            );
        }
    }
}
