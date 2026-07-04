//! Offset, shell, and rounding operators on signed distance fields.
//!
//! # Metric caveats
//!
//! All three operators reinterpret the field's level sets, so their geometric
//! meaning depends on the field actually measuring distance (`|∇f| = 1`):
//!
//! - **Offset of an exact SDF, outward (`d > 0`)**: exact. The result is the
//!   Minkowski dilation by a ball of radius `d`; convex edges and corners
//!   come out rounded (sphere-swept), and the shifted field is again an
//!   exact SDF.
//! - **Offset of an exact SDF, inward (`d < 0`)**: the level set is the
//!   correct eroded solid, but the shifted field is no longer exact
//!   everywhere — near convex features of the eroded solid it underestimates
//!   the true distance. The solid vanishes if `|d|` exceeds the inradius.
//! - **Offset of a non-exact field** (the interior of `max`/`min` CSG near
//!   edges, smooth blends, or anything else where `|∇f| ≠ 1`): the surface
//!   moves by `d / |∇f|` locally, not by `d`. The offset is metrically wrong
//!   exactly where the input field is inexact.
//! - **Shell** inherits the same caveat: the wall is only `thickness` wide
//!   where the input field is exact.
//! - **Rounding** is morphological opening — shrink by `r`, then grow by
//!   `r`, which replaces convex edges with radius-`r` fillets. It *cannot*
//!   be expressed as a pair of composed field offsets: field offsets are
//!   level-set shifts and compose additively, so `(f + r) - r ≡ f`
//!   identically and the pair collapses to the sharp original. The inward
//!   half of the pair must instead be baked into the geometry: build the
//!   core solid already inset by `r` (e.g. a box with half-extents reduced
//!   by `r`) and apply [`Rounded`] — the outward half of the offset pair —
//!   to it.

use crate::primitives::Sdf;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::types::{Point3, Vector3};

/// The solid offset by `distance`: `eval(p) = f(p) - distance`.
///
/// Positive `distance` grows the solid (Minkowski dilation by a ball when
/// `f` is exact), negative shrinks it (erosion). See the
/// [module docs](self) for what happens when `f` is not an exact SDF.
pub struct Offset<S> {
    pub sdf: S,
    distance: f64,
}

impl<S> Offset<S> {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `distance` is not finite.
    pub fn new(sdf: S, distance: f64) -> CoreResult<Self> {
        if !distance.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "distance",
                reason: format!("must be finite, got {distance}"),
            });
        }
        Ok(Self { sdf, distance })
    }
}

impl<S: Sdf> Sdf for Offset<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(p) - self.distance
    }

    // A constant shift leaves the gradient untouched.
    fn grad(&self, p: &Point3) -> Vector3 {
        self.sdf.grad(p)
    }
}

/// A hollow shell of total wall width `thickness`, centered on the surface
/// of the inner solid: `eval(p) = |f(p)| - thickness / 2`.
///
/// The interior of the original solid (deeper than `thickness / 2`) lies
/// *outside* the shell solid. Where `f` is exact the wall extends
/// `thickness / 2` to each side of the original surface; see the
/// [module docs](self) for the non-exact caveat.
pub struct Shell<S> {
    pub sdf: S,
    thickness: f64,
}

impl<S> Shell<S> {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `thickness` is not positive and
    /// finite.
    pub fn new(sdf: S, thickness: f64) -> CoreResult<Self> {
        if thickness <= 0.0 || !thickness.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "thickness",
                reason: format!("must be positive and finite, got {thickness}"),
            });
        }
        Ok(Self { sdf, thickness })
    }
}

impl<S: Sdf> Sdf for Shell<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(p).abs() - self.thickness / 2.0
    }

    // d|f| = sign(f) df. On the original surface (f = 0) the field is
    // non-smooth; returning the inner gradient there is a valid subgradient
    // per the `Sdf::grad` contract.
    fn grad(&self, p: &Point3) -> Vector3 {
        let g = self.sdf.grad(p);
        if self.sdf.eval(p) >= 0.0 { g } else { -g }
    }
}

/// The outward half of a rounding offset pair: `eval(p) = f(p) - radius`.
///
/// Grows the core solid by `radius` and, because dilation by a ball is
/// sphere-sweeping, replaces its convex edges and corners with radius-`r`
/// fillets. To round a shape while preserving its nominal dimensions, apply
/// this to a core built inset by `radius` (a rounded `2×2×2` box is a
/// `1.6×1.6×1.6` box with `radius = 0.2`). The inward half of the pair must
/// live in the core's geometry — composing two field offsets collapses to
/// the sharp original; see the [module docs](self).
pub struct Rounded<S> {
    pub sdf: S,
    radius: f64,
}

impl<S> Rounded<S> {
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    pub fn new(sdf: S, radius: f64) -> CoreResult<Self> {
        if radius <= 0.0 || !radius.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "radius",
                reason: format!("must be positive and finite, got {radius}"),
            });
        }
        Ok(Self { sdf, radius })
    }
}

impl<S: Sdf> Sdf for Rounded<S> {
    fn eval(&self, p: &Point3) -> f64 {
        self.sdf.eval(p) - self.radius
    }

    fn grad(&self, p: &Point3) -> Vector3 {
        self.sdf.grad(p)
    }
}

/// Chainable constructors for the offset-family operators, mirroring
/// [`SdfTransformExt`](crate::transform::SdfTransformExt).
pub trait SdfOpsExt: Sdf + Sized {
    /// Offset the solid by `distance` (positive grows, negative shrinks).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `distance` is not finite.
    fn offset(self, distance: f64) -> CoreResult<Offset<Self>> {
        Offset::new(self, distance)
    }

    /// Hollow the solid into a shell of total wall width `thickness`.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `thickness` is not positive and
    /// finite.
    fn shell(self, thickness: f64) -> CoreResult<Shell<Self>> {
        Shell::new(self, thickness)
    }

    /// Grow by `radius`, rounding convex edges and corners.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    fn rounded(self, radius: f64) -> CoreResult<Rounded<Self>> {
        Rounded::new(self, radius)
    }
}

impl<S: Sdf + Sized> SdfOpsExt for S {}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::mesh::{MeshOptions, mesh_sdf_indexed};
    use crate::primitives::{Box3, Sphere};
    use opensolid_core::types::BoundingBox3;

    fn unit_sphere() -> Sphere {
        Sphere {
            center: Point3::origin(),
            radius: 1.0,
        }
    }

    fn unit_box() -> Box3 {
        Box3 {
            center: Point3::origin(),
            half_extents: [1.0, 1.0, 1.0],
        }
    }

    fn sample_points() -> Vec<Point3> {
        vec![
            Point3::origin(),
            Point3::new(0.5, 0.0, 0.0),
            Point3::new(1.25, 0.0, 0.0),
            Point3::new(-1.2, 0.7, 0.3),
            Point3::new(0.0, 1.5, -0.5),
            Point3::new(3.0, -2.0, 1.0),
        ]
    }

    #[test]
    fn offset_sphere_is_bigger_sphere() {
        // A sphere offset outward by 0.25 must have the exact field of a
        // radius-1.25 sphere, inside and out.
        let grown = unit_sphere().offset(0.25).expect("valid offset");
        let reference = Sphere {
            center: Point3::origin(),
            radius: 1.25,
        };
        for p in sample_points() {
            assert!(
                (grown.eval(&p) - reference.eval(&p)).abs() < 1e-12,
                "at {p:?}"
            );
        }
    }

    #[test]
    fn negative_offset_shrinks_sphere() {
        let shrunk = unit_sphere().offset(-0.25).expect("valid offset");
        let reference = Sphere {
            center: Point3::origin(),
            radius: 0.75,
        };
        for p in sample_points() {
            assert!(
                (shrunk.eval(&p) - reference.eval(&p)).abs() < 1e-12,
                "at {p:?}"
            );
        }
    }

    #[test]
    fn offset_preserves_gradient() {
        let grown = unit_sphere().offset(0.4).expect("valid offset");
        for p in [Point3::new(0.3, -0.2, 0.9), Point3::new(2.0, 1.0, -1.0)] {
            let g = grown.grad(&p);
            let inner = unit_sphere().grad(&p);
            assert!((g - inner).norm() < 1e-12, "at {p:?}");
        }
    }

    /// The documented caveat: field offsets compose additively, so an inward
    /// then outward offset pair reproduces the sharp original instead of the
    /// morphological opening.
    #[test]
    fn offset_pair_collapses_to_original() {
        let pair = unit_box()
            .offset(-0.2)
            .expect("valid offset")
            .offset(0.2)
            .expect("valid offset");
        for p in sample_points() {
            assert!(
                (pair.eval(&p) - unit_box().eval(&p)).abs() < 1e-12,
                "at {p:?}"
            );
        }
    }

    #[test]
    fn offset_rejects_nonfinite_distance() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let err = match unit_sphere().offset(bad) {
                Ok(_) => panic!("distance {bad}: expected rejection"),
                Err(e) => e,
            };
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "distance",
                        ..
                    }
                ),
                "distance {bad}: got {err}"
            );
        }
    }

    #[test]
    fn shell_of_box_is_hollow() {
        let shell = unit_box().shell(0.3).expect("valid shell");
        // The center of the original box is deep inside the cavity, hence
        // OUTSIDE the shell solid, at distance 1 - 0.15 from the inner wall.
        let center = shell.eval(&Point3::origin());
        assert!(
            center > 0.0,
            "center must be outside the shell, got {center}"
        );
        assert!((center - 0.85).abs() < 1e-12);
        // A point on the original surface is at the middle of the wall.
        assert!((shell.eval(&Point3::new(1.0, 0.0, 0.0)) + 0.15).abs() < 1e-12);
        // Points just past either wall face are outside.
        assert!(shell.eval(&Point3::new(1.2, 0.0, 0.0)) > 0.0);
        assert!(shell.eval(&Point3::new(0.8, 0.0, 0.0)) > 0.0);
        // Points within the wall are inside.
        assert!(shell.eval(&Point3::new(0.95, 0.0, 0.0)) < 0.0);
        assert!(shell.eval(&Point3::new(1.05, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn shell_gradient_points_away_from_wall_center() {
        let shell = unit_sphere().shell(0.2).expect("valid shell");
        // Outside the original surface the shell gradient matches the inner
        // field's; inside it is flipped (moving inward exits the wall).
        let outside = Point3::new(1.5, 0.0, 0.0);
        assert!((shell.grad(&outside) - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
        let inside = Point3::new(0.5, 0.0, 0.0);
        assert!((shell.grad(&inside) - Vector3::new(-1.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn shell_rejects_nonpositive_thickness() {
        for bad in [0.0, -0.5, f64::NAN, f64::INFINITY] {
            let err = match unit_box().shell(bad) {
                Ok(_) => panic!("thickness {bad}: expected rejection"),
                Err(e) => e,
            };
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "thickness",
                        ..
                    }
                ),
                "thickness {bad}: got {err}"
            );
        }
    }

    #[test]
    fn shell_of_box_meshes_to_closed_manifold() {
        // The shell has two surfaces (outer skin and inner cavity); the
        // existing dual-contouring mesher must produce a closed manifold
        // covering both.
        let shell = unit_box().shell(0.3).expect("valid shell");
        let opts = MeshOptions {
            bounds: BoundingBox3 {
                min: Point3::new(-1.4, -1.4, -1.4),
                max: Point3::new(1.4, 1.4, 1.4),
            },
            resolution: 32,
        };
        let mesh = mesh_sdf_indexed(&shell, &opts);
        assert!(!mesh.is_empty());
        assert!(mesh.is_closed_manifold());
        let cell = 2.8 / 32.0;
        for p in &mesh.positions {
            assert!(shell.eval(p).abs() < cell, "vertex {p:?} off the surface");
        }
        // Both walls must be present: vertices near x = 1.15 (outer) and
        // x = 0.85 (inner) on the +x axis side.
        let has_outer = mesh.positions.iter().any(|p| p.x > 1.05);
        let has_inner = mesh
            .positions
            .iter()
            .any(|p| p.x.abs() < 0.95 && p.y.abs() < 0.5 && p.z.abs() < 0.5);
        assert!(has_outer, "no outer wall vertices");
        assert!(has_inner, "no inner cavity vertices");
    }

    #[test]
    fn rounded_inset_box_preserves_extents_and_rounds_corners() {
        // Core inset by the radius: rounding restores the nominal ±1 faces.
        let core = Box3 {
            center: Point3::origin(),
            half_extents: [0.8, 0.8, 0.8],
        };
        let rounded = core.rounded(0.2).expect("valid radius");
        // Face centers still sit at ±1.
        assert!(rounded.eval(&Point3::new(1.0, 0.0, 0.0)).abs() < 1e-12);
        assert!(rounded.eval(&Point3::new(0.0, -1.0, 0.0)).abs() < 1e-12);
        // The sharp corner (1,1,1) has been cut away...
        let sharp_corner = Point3::new(1.0, 1.0, 1.0);
        assert!(unit_box().eval(&sharp_corner).abs() < 1e-12);
        assert!(rounded.eval(&sharp_corner) > 0.05);
        // ...and the surface now passes through the sphere-swept corner at
        // core corner + radius along the diagonal.
        let s = 0.8 + 0.2 / 3.0_f64.sqrt();
        assert!(rounded.eval(&Point3::new(s, s, s)).abs() < 1e-12);
    }

    #[test]
    fn rounded_rejects_nonpositive_radius() {
        for bad in [0.0, -0.1, f64::NAN, f64::INFINITY] {
            let err = match unit_box().rounded(bad) {
                Ok(_) => panic!("radius {bad}: expected rejection"),
                Err(e) => e,
            };
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "radius",
                        ..
                    }
                ),
                "radius {bad}: got {err}"
            );
        }
    }

    #[test]
    fn operators_compose_through_shape() {
        // The dyn-friendly Shape handle can carry the new operators.
        let shell = crate::Shape::new(unit_box().shell(0.3).expect("valid shell"));
        assert!(shell.eval(&Point3::origin()) > 0.0);
        assert!(shell.eval(&Point3::new(1.0, 0.0, 0.0)) < 0.0);
    }
}
