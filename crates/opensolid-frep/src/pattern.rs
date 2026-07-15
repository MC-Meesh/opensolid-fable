//! Patterns and mirror: repeat or reflect a shape through domain mapping.
//!
//! Each operator here is a *union of transformed copies* of one inner field,
//! realized by mapping the query point into every copy's local frame and
//! taking the `min`. Because the maps are isometries (translation, rotation,
//! reflection) they preserve Euclidean distance, so a copy contributes an
//! exact signed distance and the `min` stays 1-Lipschitz — the same robust
//! bound the CSG combinators keep ([`crate::csg`]).
//!
//! - [`Mirror`] is the cheap two-eval domain map: `min(f(p), f(reflect(p)))`,
//!   the shape unioned with its reflection across a plane. The result is
//!   exactly symmetric about the plane.
//! - [`LinearPattern`] and [`CircularPattern`] are count-limited unions: they
//!   evaluate the inner field once per instance. A pure modulo/fold domain
//!   trick would be `O(1)` but only stays a correct *bound* when copies don't
//!   overlap, so we take the explicit union — correct for any spacing — and
//!   let empty-space interval pruning keep meshing cheap.
//!
//! All four `Sdf` methods are provided so the operators compose with meshing:
//! `eval_interval` maps the query box into each copy's frame (exact for
//! translation, conservative corner-AABB for rotation/reflection) and mins,
//! and `branches` recurses into every copy within `tol` of winning so sharp
//! edges between overlapping copies stay crisp.

use crate::primitives::Sdf;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::interval::Interval;
use opensolid_core::types::{BoundingBox3, Point3, Transform3, Vector3};

/// AABB of a box's eight corners after mapping each corner by `f`. Used to
/// bound the inverse image of a query box under a rotation or reflection: the
/// true image is a rotated/reflected box, and its corner-AABB is a
/// conservative superset.
fn mapped_box_aabb(b: &BoundingBox3, f: impl Fn(Point3) -> Point3) -> BoundingBox3 {
    let corners = (0..8).map(|i| {
        f(Point3::new(
            if i & 1 == 0 { b.min.x } else { b.max.x },
            if i & 2 == 0 { b.min.y } else { b.max.y },
            if i & 4 == 0 { b.min.z } else { b.max.z },
        ))
    });
    BoundingBox3::from_points(corners)
}

/// `count` copies of `sdf`, copy `k` translated by `k * step`.
///
/// `eval(p) = min over k in 0..count of sdf(p - k·step)` — the union of the
/// copies. Exact and 1-Lipschitz whenever the inner field is.
pub struct LinearPattern<S> {
    pub sdf: S,
    step: Vector3,
    count: usize,
}

impl<S> LinearPattern<S> {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `count == 0` or `step` is non-finite.
    pub fn new(sdf: S, step: Vector3, count: usize) -> CoreResult<Self> {
        if count == 0 {
            return Err(CoreError::InvalidArgument {
                argument: "count",
                reason: "must be at least 1".to_string(),
            });
        }
        if !step.iter().all(|c| c.is_finite()) {
            return Err(CoreError::InvalidArgument {
                argument: "step",
                reason: format!("must be finite, got ({}, {}, {})", step.x, step.y, step.z),
            });
        }
        Ok(Self { sdf, step, count })
    }

    /// Query point mapped into copy `k`'s local frame (`p - k·step`).
    fn local(&self, p: &Point3, k: usize) -> Point3 {
        p - self.step * k as f64
    }
}

impl<S: Sdf> Sdf for LinearPattern<S> {
    fn eval(&self, p: &Point3) -> f64 {
        (0..self.count)
            .map(|k| self.sdf.eval(&self.local(p, k)))
            .fold(f64::INFINITY, f64::min)
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        // Translation leaves gradients unrotated; return the winner's.
        let mut best = f64::INFINITY;
        let mut g = Vector3::zeros();
        for k in 0..self.count {
            let q = self.local(p, k);
            let d = self.sdf.eval(&q);
            if d < best {
                best = d;
                g = self.sdf.grad(&q);
            }
        }
        g
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Translating an AABB stays an AABB, so each copy's bound is exact.
        (0..self.count)
            .map(|k| {
                let shift = self.step * k as f64;
                let shifted = BoundingBox3::new(b.min - shift, b.max - shift);
                self.sdf.eval_interval(&shifted)
            })
            .reduce(|a, c| a.min(&c))
            .expect("count >= 1")
    }

    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let locals: Vec<Point3> = (0..self.count).map(|k| self.local(p, k)).collect();
        let min_val = locals
            .iter()
            .map(|q| self.sdf.eval(q))
            .fold(f64::INFINITY, f64::min);
        // Values and gradients pass through translation unchanged, so recurse
        // directly into every copy within `tol` of the winning min.
        for q in &locals {
            if self.sdf.eval(q) <= min_val + tol {
                self.sdf.branches(q, tol, out);
            }
        }
    }
}

/// `count` copies of `sdf` rotated about the axis line through `center` with
/// direction `axis`, copy `k` turned by `k * angle` radians.
///
/// `eval(p) = min over k of sdf(Rₖ⁻¹ · p)` where `Rₖ` is the rotation about
/// the axis — the union of the rotated copies, exact and 1-Lipschitz.
pub struct CircularPattern<S> {
    pub sdf: S,
    center: Point3,
    axis: Vector3,
    angle: f64,
    count: usize,
}

impl<S> CircularPattern<S> {
    /// `axis` is normalized; `angle` is the per-copy increment in radians.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `count == 0`, `axis` has non-finite
    /// or (near) zero length, or `angle` is non-finite.
    pub fn new(
        sdf: S,
        center: Point3,
        axis: Vector3,
        angle: f64,
        count: usize,
    ) -> CoreResult<Self> {
        if count == 0 {
            return Err(CoreError::InvalidArgument {
                argument: "count",
                reason: "must be at least 1".to_string(),
            });
        }
        if !angle.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "angle",
                reason: format!("must be finite, got {angle}"),
            });
        }
        let norm = axis.norm();
        if !norm.is_finite() || norm < 1e-12 {
            return Err(CoreError::InvalidArgument {
                argument: "axis",
                reason: format!(
                    "must be a finite non-zero direction, got ({}, {}, {})",
                    axis.x, axis.y, axis.z
                ),
            });
        }
        Ok(Self {
            sdf,
            center,
            axis: axis / norm,
            angle,
            count,
        })
    }

    /// The forward rotation isometry (about the origin) for copy `k`.
    fn rotation(&self, k: usize) -> Transform3 {
        Transform3::rotation(self.axis * (k as f64 * self.angle))
    }

    /// Map `p` into copy `k`'s local frame: undo the rotation about `center`.
    fn local(&self, p: &Point3, rot: &Transform3) -> Point3 {
        self.center + rot.inverse_transform_vector(&(p - self.center))
    }
}

impl<S: Sdf> Sdf for CircularPattern<S> {
    fn eval(&self, p: &Point3) -> f64 {
        (0..self.count)
            .map(|k| {
                let rot = self.rotation(k);
                self.sdf.eval(&self.local(p, &rot))
            })
            .fold(f64::INFINITY, f64::min)
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        // Chain rule through the rotation: ∇(g ∘ Rₖ⁻¹)(p) = Rₖ · ∇g(Rₖ⁻¹p).
        let mut best = f64::INFINITY;
        let mut g = Vector3::zeros();
        for k in 0..self.count {
            let rot = self.rotation(k);
            let q = self.local(p, &rot);
            let d = self.sdf.eval(&q);
            if d < best {
                best = d;
                g = rot.transform_vector(&self.sdf.grad(&q));
            }
        }
        g
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        // Each copy's inverse-image box is rotated; bound by its corner-AABB.
        (0..self.count)
            .map(|k| {
                let rot = self.rotation(k);
                let box_k = mapped_box_aabb(b, |c| self.local(&c, &rot));
                self.sdf.eval_interval(&box_k)
            })
            .reduce(|a, c| a.min(&c))
            .expect("count >= 1")
    }

    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let rots: Vec<Transform3> = (0..self.count).map(|k| self.rotation(k)).collect();
        let locals: Vec<Point3> = rots.iter().map(|rot| self.local(p, rot)).collect();
        let min_val = locals
            .iter()
            .map(|q| self.sdf.eval(q))
            .fold(f64::INFINITY, f64::min);
        for (rot, q) in rots.iter().zip(&locals) {
            if self.sdf.eval(q) <= min_val + tol {
                let start = out.len();
                self.sdf.branches(q, tol, out);
                // Value is preserved by the isometry; rotate the gradients.
                for branch in &mut out[start..] {
                    branch.1 = rot.transform_vector(&branch.1);
                }
            }
        }
    }
}

/// `sdf` unioned with its reflection across the plane through `point` with
/// unit `normal`: `eval(p) = min(sdf(p), sdf(reflect(p)))`. The result is
/// exactly symmetric about the plane.
pub struct Mirror<S> {
    pub sdf: S,
    point: Point3,
    normal: Vector3,
}

impl<S> Mirror<S> {
    /// `normal` is normalized.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `normal` has non-finite or (near)
    /// zero length.
    pub fn new(sdf: S, point: Point3, normal: Vector3) -> CoreResult<Self> {
        let norm = normal.norm();
        if !norm.is_finite() || norm < 1e-12 {
            return Err(CoreError::InvalidArgument {
                argument: "normal",
                reason: format!(
                    "must be a finite non-zero direction, got ({}, {}, {})",
                    normal.x, normal.y, normal.z
                ),
            });
        }
        Ok(Self {
            sdf,
            point,
            normal: normal / norm,
        })
    }

    /// Reflect a point across the mirror plane.
    fn reflect_point(&self, p: &Point3) -> Point3 {
        let signed = (p - self.point).dot(&self.normal);
        p - self.normal * (2.0 * signed)
    }

    /// Reflect a direction across the mirror plane (the plane's linear part,
    /// which is symmetric and its own inverse).
    fn reflect_vector(&self, v: &Vector3) -> Vector3 {
        v - self.normal * (2.0 * v.dot(&self.normal))
    }
}

impl<S: Sdf> Sdf for Mirror<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(p).min(self.sdf.eval(&self.reflect_point(p)))
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        let here = self.sdf.eval(p);
        let mirror_p = self.reflect_point(p);
        let there = self.sdf.eval(&mirror_p);
        if here <= there {
            self.sdf.grad(p)
        } else {
            // ∇(g ∘ reflect)(p) = Reflectᵀ ∇g = Reflect ∇g (reflection is
            // symmetric), so reflect the inner gradient back.
            self.reflect_vector(&self.sdf.grad(&mirror_p))
        }
    }

    fn eval_interval(&self, b: &BoundingBox3) -> Interval {
        let here = self.sdf.eval_interval(b);
        let reflected = mapped_box_aabb(b, |c| self.reflect_point(&c));
        here.min(&self.sdf.eval_interval(&reflected))
    }

    fn branches(&self, p: &Point3, tol: f64, out: &mut Vec<(f64, Vector3)>) {
        let here = self.sdf.eval(p);
        let mirror_p = self.reflect_point(p);
        let there = self.sdf.eval(&mirror_p);
        let min_val = here.min(there);
        if here <= min_val + tol {
            self.sdf.branches(p, tol, out);
        }
        if there <= min_val + tol {
            let start = out.len();
            self.sdf.branches(&mirror_p, tol, out);
            // Value is preserved by the reflection; reflect the gradients.
            for branch in &mut out[start..] {
                branch.1 = self.reflect_vector(&branch.1);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eval::gradient;
    use crate::primitives::{Box3, Sphere};
    use std::f64::consts::{FRAC_PI_2, PI};

    fn unit_sphere_at(x: f64, y: f64, z: f64) -> Sphere {
        Sphere {
            center: Point3::new(x, y, z),
            radius: 1.0,
        }
    }

    fn assert_unit_gradient(sdf: &dyn Sdf, p: &Point3) {
        let g = gradient(sdf, p).norm();
        assert!((g - 1.0).abs() < 1e-4, "gradient norm {g} at {p:?}");
    }

    // ---- LinearPattern ----

    #[test]
    fn linear_pattern_places_each_copy() {
        // Unit sphere at origin, three copies stepped by (3,0,0): surfaces at
        // x = 0, 3, 6 (radius 1), centers inside.
        let p = LinearPattern::new(
            Sphere {
                center: Point3::origin(),
                radius: 1.0,
            },
            Vector3::new(3.0, 0.0, 0.0),
            3,
        )
        .expect("valid pattern");
        for cx in [0.0, 3.0, 6.0] {
            assert!(
                p.eval(&Point3::new(cx, 0.0, 0.0)) < 0.0,
                "center {cx} outside"
            );
            assert!(
                (p.eval(&Point3::new(cx + 1.0, 0.0, 0.0))).abs() < 1e-12,
                "surface at {cx}+1"
            );
        }
        // Between copies (x = 1.5) and past the last copy is outside.
        assert!(p.eval(&Point3::new(1.5, 0.0, 0.0)) > 0.0);
        assert!(p.eval(&Point3::new(9.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn linear_pattern_count_one_is_the_original() {
        let base = unit_sphere_at(0.5, -0.2, 0.3);
        let p = LinearPattern::new(base, Vector3::new(2.0, 1.0, 0.0), 1).expect("valid");
        for q in [
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 0.8, 0.3),
        ] {
            assert_eq!(p.eval(&q), unit_sphere_at(0.5, -0.2, 0.3).eval(&q));
        }
    }

    #[test]
    fn linear_pattern_gradient_is_a_unit_normal() {
        let p = LinearPattern::new(
            Sphere {
                center: Point3::origin(),
                radius: 1.0,
            },
            Vector3::new(3.0, 0.0, 0.0),
            3,
        )
        .expect("valid");
        // On the second copy's surface the gradient points radially out of it.
        let g = p.grad(&Point3::new(4.0, 0.0, 0.0));
        assert!((g - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-9, "got {g:?}");
        assert_unit_gradient(&p, &Point3::new(3.0, 1.0, 0.0));
    }

    #[test]
    fn linear_pattern_rejects_bad_args() {
        let s = || unit_sphere_at(0.0, 0.0, 0.0);
        assert!(LinearPattern::new(s(), Vector3::new(1.0, 0.0, 0.0), 0).is_err());
        assert!(LinearPattern::new(s(), Vector3::new(f64::NAN, 0.0, 0.0), 3).is_err());
    }

    #[test]
    fn linear_pattern_interval_containment() {
        let p = LinearPattern::new(
            Box3 {
                center: Point3::origin(),
                half_extents: [0.6, 0.4, 0.5],
            },
            Vector3::new(2.0, 0.3, 0.0),
            4,
        )
        .expect("valid");
        crate::test_util::assert_interval_containment(&p, 61);
    }

    // ---- CircularPattern ----

    #[test]
    fn circular_pattern_places_each_copy() {
        // Sphere offset to x = 2, four copies a quarter-turn apart about the
        // y axis through the origin: centers at (±2,0,0) and (0,0,±2).
        let p = CircularPattern::new(
            unit_sphere_at(2.0, 0.0, 0.0),
            Point3::origin(),
            Vector3::new(0.0, 1.0, 0.0),
            FRAC_PI_2,
            4,
        )
        .expect("valid");
        for c in [
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(-2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(0.0, 0.0, -2.0),
        ] {
            assert!(p.eval(&c) < 0.0, "center {c:?} should be inside");
        }
        // A gap between copies (45°, radius 2) is empty.
        let g = Point3::new(2.0_f64.sqrt(), 0.0, 2.0_f64.sqrt());
        assert!(p.eval(&g) > 0.0, "gap {g:?} should be outside");
    }

    #[test]
    fn circular_pattern_about_offset_center() {
        // Sphere at origin, center of rotation at (5,0,0), half-turn, 2 copies:
        // copy 0 at origin, copy 1 reflected through the center to (10,0,0).
        let p = CircularPattern::new(
            Sphere {
                center: Point3::origin(),
                radius: 1.0,
            },
            Point3::new(5.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            PI,
            2,
        )
        .expect("valid");
        assert!(p.eval(&Point3::origin()) < 0.0);
        assert!(p.eval(&Point3::new(10.0, 0.0, 0.0)) < 0.0);
        assert!((p.eval(&Point3::new(11.0, 0.0, 0.0))).abs() < 1e-12);
    }

    #[test]
    fn circular_pattern_gradient_is_a_unit_normal() {
        let p = CircularPattern::new(
            unit_sphere_at(2.0, 0.0, 0.0),
            Point3::origin(),
            Vector3::new(0.0, 1.0, 0.0),
            FRAC_PI_2,
            4,
        )
        .expect("valid");
        // Surface of the copy at (0,0,2), on its +z side.
        let g = p.grad(&Point3::new(0.0, 0.0, 3.0));
        assert!((g - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-9, "got {g:?}");
        assert_unit_gradient(&p, &Point3::new(0.0, 0.5, 3.0));
    }

    #[test]
    fn circular_pattern_rejects_bad_args() {
        let s = || unit_sphere_at(2.0, 0.0, 0.0);
        let c = Point3::origin();
        let ax = Vector3::new(0.0, 1.0, 0.0);
        assert!(CircularPattern::new(s(), c, ax, FRAC_PI_2, 0).is_err());
        assert!(CircularPattern::new(s(), c, Vector3::zeros(), FRAC_PI_2, 4).is_err());
        assert!(CircularPattern::new(s(), c, ax, f64::INFINITY, 4).is_err());
    }

    #[test]
    fn circular_pattern_interval_containment() {
        let p = CircularPattern::new(
            Box3 {
                center: Point3::new(1.5, 0.0, 0.0),
                half_extents: [0.5, 0.4, 0.3],
            },
            Point3::origin(),
            Vector3::new(0.0, 1.0, 0.0),
            FRAC_PI_2,
            4,
        )
        .expect("valid");
        crate::test_util::assert_interval_containment(&p, 62);
    }

    // ---- Mirror ----

    #[test]
    fn mirror_is_symmetric_about_the_plane() {
        // Sphere offset to +x mirrored across the x = 0 plane (normal +x):
        // the field must satisfy f(x,y,z) == f(-x,y,z) everywhere.
        let m = Mirror::new(
            unit_sphere_at(2.0, 0.0, 0.0),
            Point3::origin(),
            Vector3::new(1.0, 0.0, 0.0),
        )
        .expect("valid");
        for q in [
            Point3::new(2.0, 0.3, -0.4),
            Point3::new(0.7, -1.1, 0.9),
            Point3::new(3.5, 0.0, 0.0),
            Point3::new(0.0, 2.0, 1.0),
        ] {
            let mirrored = Point3::new(-q.x, q.y, q.z);
            assert!(
                (m.eval(&q) - m.eval(&mirrored)).abs() < 1e-12,
                "asymmetry at {q:?}"
            );
        }
        // Both the original (x=2) and its reflection (x=-2) are solid.
        assert!(m.eval(&Point3::new(2.0, 0.0, 0.0)) < 0.0);
        assert!(m.eval(&Point3::new(-2.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn mirror_keeps_the_whole_original() {
        // A shape straddling the plane keeps its entire original body plus the
        // reflected copy (this is a union, not a fold of one half).
        let m = Mirror::new(
            Sphere {
                center: Point3::new(0.5, 0.0, 0.0),
                radius: 1.0,
            },
            Point3::origin(),
            Vector3::new(1.0, 0.0, 0.0),
        )
        .expect("valid");
        // Original reaches x = 1.5; reflection reaches x = -1.5.
        assert!((m.eval(&Point3::new(1.5, 0.0, 0.0))).abs() < 1e-12);
        assert!((m.eval(&Point3::new(-1.5, 0.0, 0.0))).abs() < 1e-12);
        assert!(m.eval(&Point3::origin()) < 0.0);
    }

    #[test]
    fn mirror_across_oblique_plane() {
        // Plane through origin with normal (1,1,0)/√2 swaps x and y (reflection
        // across the line y = -x in plan). A sphere at (2,0,0) reflects to
        // (0,-2,0).
        let m = Mirror::new(
            unit_sphere_at(2.0, 0.0, 0.0),
            Point3::origin(),
            Vector3::new(1.0, 1.0, 0.0),
        )
        .expect("valid");
        assert!(m.eval(&Point3::new(2.0, 0.0, 0.0)) < 0.0);
        assert!(m.eval(&Point3::new(0.0, -2.0, 0.0)) < 0.0);
        assert!((m.eval(&Point3::new(0.0, -3.0, 0.0))).abs() < 1e-12);
    }

    #[test]
    fn mirror_gradient_is_a_unit_normal_on_both_sides() {
        let m = Mirror::new(
            unit_sphere_at(2.0, 0.0, 0.0),
            Point3::origin(),
            Vector3::new(1.0, 0.0, 0.0),
        )
        .expect("valid");
        // Original side.
        let g = m.grad(&Point3::new(3.0, 0.0, 0.0));
        assert!(
            (g - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-9,
            "orig {g:?}"
        );
        // Reflected side: outward normal points in -x.
        let g = m.grad(&Point3::new(-3.0, 0.0, 0.0));
        assert!(
            (g - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-9,
            "refl {g:?}"
        );
        assert_unit_gradient(&m, &Point3::new(-2.0, 1.0, 0.0));
    }

    #[test]
    fn mirror_offset_plane() {
        // Plane at x = 1: a sphere at origin reflects to a sphere at (2,0,0).
        let m = Mirror::new(
            Sphere {
                center: Point3::origin(),
                radius: 0.5,
            },
            Point3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
        )
        .expect("valid");
        assert!(m.eval(&Point3::origin()) < 0.0);
        assert!(m.eval(&Point3::new(2.0, 0.0, 0.0)) < 0.0);
        assert!(m.eval(&Point3::new(1.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn mirror_rejects_bad_normal() {
        let s = || unit_sphere_at(2.0, 0.0, 0.0);
        assert!(Mirror::new(s(), Point3::origin(), Vector3::zeros()).is_err());
        assert!(Mirror::new(s(), Point3::origin(), Vector3::new(f64::NAN, 1.0, 0.0)).is_err());
    }

    #[test]
    fn mirror_interval_containment() {
        let m = Mirror::new(
            Box3 {
                center: Point3::new(1.2, 0.3, 0.0),
                half_extents: [0.5, 0.4, 0.6],
            },
            Point3::origin(),
            Vector3::new(1.0, 0.2, 0.0),
        )
        .expect("valid");
        crate::test_util::assert_interval_containment(&m, 63);
    }
}
