use crate::primitives::Sdf;
use opensolid_core::types::{Point3, Transform3, Vector3};

/// An SDF placed by a rigid transform (rotation + translation).
///
/// Evaluates the inner SDF at the inverse-transformed query point. Rigid
/// isometries preserve Euclidean distance, so the field stays an exact
/// signed distance.
pub struct Transformed<S> {
    pub sdf: S,
    inverse: Transform3,
}

impl<S> Transformed<S> {
    pub fn new(sdf: S, transform: Transform3) -> Self {
        Self {
            sdf,
            inverse: transform.inverse(),
        }
    }
}

impl<S: Sdf> Sdf for Transformed<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(&(self.inverse * p))
    }
}

/// An SDF scaled uniformly about the origin by `factor > 0`:
/// `eval(p) = factor * inner.eval(p / factor)`.
///
/// Uniform scaling multiplies every Euclidean distance by the same factor,
/// so rescaling the inner value keeps the field an exact distance.
///
/// Non-uniform scale is deliberately excluded: it stretches space by a
/// direction-dependent amount (spheres become ellipsoids), so no single
/// correction factor can restore the inner value to a distance — `|∇f|`
/// drifts away from 1 and everything that relies on the metric property
/// (blend radii, meshing step bounds, offsets) silently breaks. An
/// approximate anisotropic scale would need its own re-normalizing
/// operator, not this wrapper.
pub struct UniformScale<S> {
    pub sdf: S,
    factor: f64,
}

impl<S> UniformScale<S> {
    /// Panics if `factor` is not strictly positive.
    pub fn new(sdf: S, factor: f64) -> Self {
        assert!(factor > 0.0, "scale factor must be positive, got {factor}");
        Self { sdf, factor }
    }
}

impl<S: Sdf> Sdf for UniformScale<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(&Point3::from(p.coords / self.factor)) * self.factor
    }
}

/// Chainable constructors for the transform wrappers. Each call wraps the
/// receiver, so transforms apply to the shape in the order they are chained:
/// `sdf.rotated(r).translated(t)` rotates first, then translates.
pub trait SdfTransformExt: Sdf + Sized {
    /// Apply an arbitrary rigid transform.
    fn transformed(self, transform: Transform3) -> Transformed<Self> {
        Transformed::new(self, transform)
    }

    /// Move by `offset`.
    fn translated(self, offset: Vector3) -> Transformed<Self> {
        self.transformed(Transform3::translation(offset.x, offset.y, offset.z))
    }

    /// Rotate about the origin. `axis_angle`'s direction is the rotation
    /// axis and its norm the angle in radians.
    fn rotated(self, axis_angle: Vector3) -> Transformed<Self> {
        self.transformed(Transform3::rotation(axis_angle))
    }

    /// Scale uniformly about the origin (`factor > 0`).
    fn scaled(self, factor: f64) -> UniformScale<Self> {
        UniformScale::new(self, factor)
    }
}

impl<S: Sdf + Sized> SdfTransformExt for S {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::eval::gradient;
    use crate::primitives::{Box3, Sphere};
    use std::f64::consts::FRAC_PI_2;

    fn unit_sphere() -> Sphere {
        Sphere {
            center: Point3::origin(),
            radius: 1.0,
        }
    }

    fn assert_unit_gradient(sdf: &dyn Sdf, p: &Point3) {
        let g = gradient(sdf, p).norm();
        assert!((g - 1.0).abs() < 1e-4, "gradient norm {g} at {p:?}");
    }

    #[test]
    fn translated_sphere_surface() {
        let s = unit_sphere().translated(Vector3::new(2.0, 0.0, 0.0));
        assert!(s.eval(&Point3::new(3.0, 0.0, 0.0)).abs() < 1e-12);
        assert!((s.eval(&Point3::new(2.0, 0.0, 0.0)) + 1.0).abs() < 1e-12);
        assert!((s.eval(&Point3::origin()) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn rotated_box_surface() {
        // Box reaching to x = ±2; after +90° about z it reaches to y = ±2.
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [2.0, 1.0, 1.0],
        }
        .rotated(Vector3::new(0.0, 0.0, FRAC_PI_2));
        assert!(b.eval(&Point3::new(0.0, 2.0, 0.0)).abs() < 1e-12);
        assert!((b.eval(&Point3::new(2.0, 0.0, 0.0)) - 1.0).abs() < 1e-12);
        assert!(b.eval(&Point3::origin()) < 0.0);
    }

    #[test]
    fn transformed_preserves_unit_gradient() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 0.5, 0.25],
        }
        .rotated(Vector3::new(0.3, -0.2, 0.9))
        .translated(Vector3::new(1.0, 2.0, -0.5));
        assert_unit_gradient(&b, &Point3::new(2.5, 2.1, -0.3));
        assert_unit_gradient(&b, &Point3::new(0.9, 1.8, -0.6));
    }

    #[test]
    fn composed_transforms_match_single_isometry() {
        let rot = Transform3::rotation(Vector3::new(0.0, 0.0, FRAC_PI_2));
        let tr = Transform3::translation(1.0, 2.0, 3.0);
        let box3 = || Box3 {
            center: Point3::origin(),
            half_extents: [2.0, 1.0, 0.5],
        };
        // Chained rotate-then-translate must equal the single isometry tr * rot.
        let chained = box3()
            .rotated(Vector3::new(0.0, 0.0, FRAC_PI_2))
            .translated(Vector3::new(1.0, 2.0, 3.0));
        let single = box3().transformed(tr * rot);
        for p in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 4.0, 3.0),
            Point3::new(-2.5, 1.3, 4.7),
        ] {
            assert!((chained.eval(&p) - single.eval(&p)).abs() < 1e-12);
        }
    }

    #[test]
    fn scaled_sphere_distance_is_exact_everywhere() {
        // Unit sphere scaled by 2 is a radius-2 sphere; the field must be
        // the exact distance |p| - 2, inside and out.
        let s = unit_sphere().scaled(2.0);
        for p in [
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, -3.0, 4.0),
        ] {
            let expected = p.coords.norm() - 2.0;
            assert!((s.eval(&p) - expected).abs() < 1e-12);
        }
    }

    #[test]
    fn scale_is_about_the_origin() {
        // Sphere translated to (1,0,0) then scaled by 2 lands at (2,0,0)
        // with radius 2.
        let s = unit_sphere()
            .translated(Vector3::new(1.0, 0.0, 0.0))
            .scaled(2.0);
        assert!(s.eval(&Point3::new(4.0, 0.0, 0.0)).abs() < 1e-12);
        assert!(s.eval(&Point3::origin()).abs() < 1e-12);
        assert!((s.eval(&Point3::new(2.0, 0.0, 0.0)) + 2.0).abs() < 1e-12);
    }

    #[test]
    fn scaled_box_preserves_unit_gradient() {
        let b = Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 0.5, 0.25],
        }
        .scaled(3.0);
        assert_unit_gradient(&b, &Point3::new(3.2, 0.4, 0.1));
        assert_unit_gradient(&b, &Point3::new(0.5, 0.3, 0.2));
    }

    #[test]
    fn shrinking_scale_works() {
        let s = unit_sphere().scaled(0.25);
        assert!(s.eval(&Point3::new(0.25, 0.0, 0.0)).abs() < 1e-12);
        assert!((s.eval(&Point3::new(1.25, 0.0, 0.0)) - 1.0).abs() < 1e-12);
    }

    #[test]
    #[should_panic(expected = "scale factor must be positive")]
    fn zero_scale_panics() {
        UniformScale::new(unit_sphere(), 0.0);
    }

    #[test]
    #[should_panic(expected = "scale factor must be positive")]
    fn negative_scale_panics() {
        UniformScale::new(unit_sphere(), -1.0);
    }
}
