//! Tolerance model: what "close enough" means in the kernel.
//!
//! Per `spec/08-tolerances.md`: geometry is exact math, topology is a
//! connectivity graph, and floating point means the two never agree
//! perfectly. Every comparison in the kernel goes through a
//! [`ToleranceContext`] so the meaning of equality is explicit and
//! configurable, never an ad-hoc `1e-9` scattered through call sites.
//!
//! All comparisons are NaN-safe: any comparison involving NaN is `false`.

use crate::types::{Point3, Vector3};

/// System resolution: distances smaller than this are considered zero.
/// This is the precision floor of the kernel; tolerance values below it
/// are clamped up to it in every comparison.
pub const SYSTEM_RESOLUTION: f64 = 1e-10;

/// Angular resolution floor for direction comparisons, in radians.
pub const ANGULAR_RESOLUTION: f64 = 1e-12;

/// Relative tolerance factor for large-magnitude comparisons.
///
/// f64 spacing grows with magnitude (~1.2e-7 absolute at 1e9), so a purely
/// absolute tolerance silently becomes unsatisfiable at large coordinates.
/// The effective linear tolerance is `max(linear, magnitude * REL_TOLERANCE_FACTOR)`
/// — roughly "a few thousand ULPs" at any scale.
pub const REL_TOLERANCE_FACTOR: f64 = 1e-12;

/// Tolerances used for approximate comparisons throughout the kernel.
///
/// The default context matches the spec's consistency targets: 1e-6 model
/// units linear (10x the boolean SSI target of 1e-7), 1e-8 radians angular
/// (well above [`ANGULAR_RESOLUTION`], tight enough that surfaces meeting
/// at that angle are visually and functionally coincident), and 1e-9
/// parametric (callers are expected to compare parameters scaled to a
/// roughly unit-sized domain).
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ToleranceContext {
    /// Linear tolerance in model units.
    pub linear: f64,
    /// Angular tolerance in radians.
    pub angular: f64,
    /// Parametric tolerance, relative to a unit-scale parameter domain.
    pub parametric: f64,
}

impl Default for ToleranceContext {
    fn default() -> Self {
        Self {
            linear: 1e-6,
            angular: 1e-8,
            parametric: 1e-9,
        }
    }
}

impl ToleranceContext {
    /// Effective linear tolerance for values of the given magnitude:
    /// absolute below magnitude 1, relative above (see [`REL_TOLERANCE_FACTOR`]),
    /// never below [`SYSTEM_RESOLUTION`].
    fn effective_linear(&self, magnitude: f64) -> f64 {
        self.linear
            .max(magnitude * REL_TOLERANCE_FACTOR)
            .max(SYSTEM_RESOLUTION)
    }

    /// Angular tolerance clamped to the [`ANGULAR_RESOLUTION`] floor.
    fn effective_angular(&self) -> f64 {
        self.angular.max(ANGULAR_RESOLUTION)
    }

    /// Are two scalars equal within tolerance (absolute + relative hybrid)?
    pub fn approx_eq(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.effective_linear(a.abs().max(b.abs()))
    }

    /// Is a scalar zero within the linear tolerance?
    pub fn approx_zero(&self, x: f64) -> bool {
        x.abs() <= self.effective_linear(0.0)
    }

    /// Are two parameters equal within the parametric tolerance (absolute)?
    ///
    /// Callers comparing parameters on a large or tiny domain should scale
    /// values to a roughly unit domain first.
    pub fn params_approx_eq(&self, a: f64, b: f64) -> bool {
        (a - b).abs() <= self.parametric.max(f64::EPSILON)
    }

    /// Are two points coincident within tolerance?
    pub fn points_approx_eq(&self, a: &Point3, b: &Point3) -> bool {
        let magnitude = a.coords.norm().max(b.coords.norm());
        (a - b).norm() <= self.effective_linear(magnitude)
    }

    /// Are two vectors equal (component displacement) within tolerance?
    pub fn vectors_approx_eq(&self, a: &Vector3, b: &Vector3) -> bool {
        let magnitude = a.norm().max(b.norm());
        (a - b).norm() <= self.effective_linear(magnitude)
    }

    /// Is a vector zero within the linear tolerance?
    pub fn vector_approx_zero(&self, v: &Vector3) -> bool {
        v.norm() <= self.effective_linear(0.0)
    }

    /// Do two vectors point in the same direction within the angular
    /// tolerance? Inputs need not be normalized; zero (or NaN) vectors
    /// have no direction and compare unequal to everything.
    ///
    /// Implemented without `acos`: for an angle t between vectors,
    /// `|a x b| = |a||b| sin(t)`, and sin is well-conditioned near zero
    /// where acos-of-dot loses all precision. For tolerances past 90
    /// degrees the cross test is no longer monotonic, so the dot test
    /// (well-conditioned there) takes over.
    pub fn directions_approx_eq(&self, a: &Vector3, b: &Vector3) -> bool {
        let tol = self.effective_angular();
        let scale = a.norm() * b.norm();
        if scale <= 0.0 || scale.is_nan() {
            return false;
        }
        let dot = a.dot(b);
        if tol <= std::f64::consts::FRAC_PI_2 {
            dot >= 0.0 && a.cross(b).norm() <= tol.sin() * scale
        } else {
            dot >= tol.cos() * scale
        }
    }

    /// Are two vectors parallel (same or opposite direction) within the
    /// angular tolerance?
    pub fn vectors_parallel(&self, a: &Vector3, b: &Vector3) -> bool {
        let neg_b = -b;
        self.directions_approx_eq(a, b) || self.directions_approx_eq(a, &neg_b)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Rotation3;

    fn ctx() -> ToleranceContext {
        ToleranceContext::default()
    }

    #[test]
    fn scalar_boundaries_around_tolerance() {
        let t = ctx();
        assert!(t.approx_eq(1.0, 1.0));
        assert!(t.approx_eq(1.0, 1.0 + 0.9e-6));
        assert!(!t.approx_eq(1.0, 1.0 + 1.1e-6));
        assert!(t.approx_zero(0.9e-6));
        assert!(!t.approx_zero(1.1e-6));
        assert!(t.approx_zero(-0.9e-6));
    }

    #[test]
    fn large_coordinates_use_relative_tolerance() {
        let t = ctx();
        // At 1e9, effective tolerance is 1e9 * 1e-12 = 1e-3.
        assert!(t.approx_eq(1e9, 1e9 + 0.9e-3));
        assert!(!t.approx_eq(1e9, 1e9 + 1.1e-3));
        // The same absolute difference near the origin is NOT equal.
        assert!(!t.approx_eq(0.0, 0.9e-3));
    }

    #[test]
    fn tolerance_floor_is_system_resolution() {
        let t = ToleranceContext {
            linear: 0.0,
            angular: 0.0,
            parametric: 0.0,
        };
        // Even with a zero linear tolerance, SYSTEM_RESOLUTION applies.
        assert!(t.approx_eq(0.0, 0.9e-10));
        assert!(!t.approx_eq(0.0, 1.1e-10));
        // Angular floor likewise: identical directions still compare equal.
        let x = Vector3::new(1.0, 0.0, 0.0);
        assert!(t.directions_approx_eq(&x, &x));
    }

    #[test]
    fn nan_never_compares_equal() {
        let t = ctx();
        assert!(!t.approx_eq(f64::NAN, 1.0));
        assert!(!t.approx_eq(f64::NAN, f64::NAN));
        assert!(!t.approx_zero(f64::NAN));
        let p = Point3::new(f64::NAN, 0.0, 0.0);
        assert!(!t.points_approx_eq(&p, &p));
        let v = Vector3::new(f64::NAN, 0.0, 0.0);
        assert!(!t.directions_approx_eq(&v, &v));
    }

    #[test]
    fn points_and_vectors_hybrid_behavior() {
        let t = ctx();
        let a = Point3::new(1.0, 2.0, 3.0);
        let b = Point3::new(1.0, 2.0, 3.0 + 0.9e-6);
        assert!(t.points_approx_eq(&a, &b));
        // Same sub-micron offset far from the origin: relative kicks in.
        let far_a = Point3::new(1e9, 0.0, 0.0);
        let far_b = Point3::new(1e9 + 0.9e-3, 0.0, 0.0);
        assert!(t.points_approx_eq(&far_a, &far_b));
        let near_a = Point3::new(0.0, 0.0, 0.0);
        let near_b = Point3::new(0.9e-3, 0.0, 0.0);
        assert!(!t.points_approx_eq(&near_a, &near_b));

        assert!(t.vectors_approx_eq(
            &Vector3::new(1.0, 0.0, 0.0),
            &Vector3::new(1.0, 0.9e-6, 0.0)
        ));
        assert!(t.vector_approx_zero(&Vector3::new(0.5e-6, 0.5e-6, 0.0)));
        assert!(!t.vector_approx_zero(&Vector3::new(2e-6, 0.0, 0.0)));
    }

    #[test]
    fn unit_vector_angles_without_acos() {
        let t = ctx(); // angular = 1e-8 rad
        let x = Vector3::new(1.0, 0.0, 0.0);
        let rot_small = Rotation3::from_axis_angle(&Vector3::z_axis(), 0.5e-8);
        let rot_large = Rotation3::from_axis_angle(&Vector3::z_axis(), 2.0e-8);
        assert!(t.directions_approx_eq(&x, &(rot_small * x)));
        assert!(!t.directions_approx_eq(&x, &(rot_large * x)));
        // Identical vectors always pass despite rounding in cross/dot.
        let odd = Vector3::new(0.1234, -5.678, 9.1011).normalize();
        assert!(t.directions_approx_eq(&odd, &odd));
    }

    #[test]
    fn direction_comparison_ignores_length() {
        let t = ctx();
        let a = Vector3::new(2.0, 0.0, 0.0);
        let b = Vector3::new(500.0, 0.0, 0.0);
        assert!(t.directions_approx_eq(&a, &b));
        // But vector (displacement) equality does care about length.
        assert!(!t.vectors_approx_eq(&a, &b));
    }

    #[test]
    fn parallel_and_antiparallel() {
        let t = ctx();
        let x = Vector3::new(1.0, 0.0, 0.0);
        let neg_x = Vector3::new(-3.0, 0.0, 0.0);
        assert!(!t.directions_approx_eq(&x, &neg_x));
        assert!(t.vectors_parallel(&x, &neg_x));
        assert!(t.vectors_parallel(&x, &x));
        let y = Vector3::new(0.0, 1.0, 0.0);
        assert!(!t.vectors_parallel(&x, &y));
    }

    #[test]
    fn zero_vectors_have_no_direction() {
        let t = ctx();
        let zero = Vector3::zeros();
        let x = Vector3::new(1.0, 0.0, 0.0);
        assert!(!t.directions_approx_eq(&zero, &x));
        assert!(!t.directions_approx_eq(&zero, &zero));
        assert!(!t.vectors_parallel(&zero, &x));
    }

    #[test]
    fn wide_angular_tolerance_uses_dot_path() {
        let t = ToleranceContext {
            angular: 3.0, // > pi/2: exercises the cos fallback
            ..ToleranceContext::default()
        };
        let x = Vector3::new(1.0, 0.0, 0.0);
        let mostly_back = Rotation3::from_axis_angle(&Vector3::z_axis(), 2.9) * x;
        let nearly_opposite = Rotation3::from_axis_angle(&Vector3::z_axis(), 3.1) * x;
        assert!(t.directions_approx_eq(&x, &mostly_back));
        assert!(!t.directions_approx_eq(&x, &nearly_opposite));
    }

    #[test]
    fn params_compare_absolutely() {
        let t = ctx();
        assert!(t.params_approx_eq(0.5, 0.5 + 0.9e-9));
        assert!(!t.params_approx_eq(0.5, 0.5 + 1.1e-9));
    }
}
