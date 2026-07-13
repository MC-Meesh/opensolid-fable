//! Parametric 3D surfaces: analytic primitives and the evaluation trait.
//!
//! Parameterization conventions (matching spec/02-geometry.md and the frame
//! rules of [`crate::curve`]):
//! - `Plane`: `point(u, v) = origin + u*e_u + v*e_v` where `(e_u, e_v)` is
//!   the deterministic [`plane_basis`] of `normal`. Unbounded, aperiodic.
//! - `Cylinder`: `u` is the angle in radians about `axis` (counterclockwise
//!   by the right-hand rule, period 2π), `v` is the signed axial distance
//!   from `origin`. Radial direction at `u = 0` is `plane_basis(axis).0`.
//! - `Cone`: like `Cylinder`, but the radius varies linearly with `v`:
//!   `rho(v) = radius + v*tan(half_angle)`, so the surface widens along
//!   `+axis`. The apex lies at `v = -radius/tan(half_angle)`; evaluation is
//!   defined on both nappes (`rho < 0` mirrors through the apex) but the
//!   apex itself is a parameterization singularity: `du = 0`, no normal.
//! - `Sphere`: `u` is longitude (period 2π), `v` is latitude in
//!   `[-π/2, π/2]`; `point = center + r*(cos v * radial(u) + sin v * axis)`.
//!   The poles (`v = ±π/2`) are singular (`du = 0`), but the geometric
//!   normal is still well-defined by continuity as `±axis`, so
//!   [`SurfaceEval::normal`] returns it there.
//! - `Torus`: `u` is the angle about `axis` (period 2π), `v` the angle
//!   around the tube (period 2π); `point = center +
//!   (R + r*cos v)*radial(u) + r*sin v*axis`. Constructors require
//!   `R > r > 0` (no spindle/horn tori yet), so the map is regular
//!   everywhere.
//!
//! For every variant the normal is the normalized `du × dv` (right-handed
//! with the parameterization); for the closed primitives this points
//! outward. Where `du × dv` degenerates and no limit normal exists (cone
//! apex), [`SurfaceEval::normal`] returns `None`.

use crate::curve::{TWO_PI, plane_basis};
use crate::topology::SYSTEM_RESOLUTION;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::types::{BoundingBox3, Point3, Vector3};

/// Evaluation interface for parametric surfaces.
pub trait SurfaceEval {
    /// Position on the surface at parameters `(u, v)`.
    fn point(&self, u: f64, v: f64) -> Point3;

    /// Partial derivative with respect to `u` (not necessarily unit).
    fn du(&self, u: f64, v: f64) -> Vector3;

    /// Partial derivative with respect to `v` (not necessarily unit).
    fn dv(&self, u: f64, v: f64) -> Vector3;

    /// Unit normal at `(u, v)`, oriented along `du × dv` (outward for the
    /// closed primitives). `None` where the parameterization is degenerate
    /// and no limit normal exists (e.g. cone apex). Surfaces whose
    /// singularities still have a well-defined geometric normal (sphere
    /// poles) return it.
    fn normal(&self, u: f64, v: f64) -> Option<Vector3>;

    /// Parameter interval `(u_min, u_max)`. Unbounded directions return
    /// infinite endpoints.
    fn domain_u(&self) -> (f64, f64);

    /// Parameter interval `(v_min, v_max)`.
    fn domain_v(&self) -> (f64, f64);

    /// Whether evaluation repeats in `u` with period [`SurfaceEval::period_u`].
    fn is_periodic_u(&self) -> bool;

    /// Whether evaluation repeats in `v` with period [`SurfaceEval::period_v`].
    fn is_periodic_v(&self) -> bool;

    /// Period in `u` for u-periodic surfaces, `None` otherwise.
    fn period_u(&self) -> Option<f64> {
        None
    }

    /// Period in `v` for v-periodic surfaces, `None` otherwise.
    fn period_v(&self) -> Option<f64> {
        None
    }

    /// Is `(u, v)` a parameterization singularity (`du × dv = 0`)?
    fn is_singular(&self, u: f64, v: f64) -> bool;
}

/// Analytic 3D surface primitives.
///
/// Named `Surface3` (paralleling [`crate::curve::Curve3`]) to avoid clashing
/// with the [`crate::topology::Surface`] placeholder marker, which a later
/// issue replaces with references to this type.
#[derive(Debug, Clone, PartialEq)]
pub enum Surface3 {
    /// Infinite plane through `origin` with unit `normal`.
    Plane { origin: Point3, normal: Vector3 },
    /// Infinite circular cylinder of `radius` about the line through
    /// `origin` with unit direction `axis`.
    Cylinder {
        origin: Point3,
        axis: Vector3,
        radius: f64,
    },
    /// Circular cone about the line through `origin` with unit `axis`:
    /// radius `radius` at `v = 0`, widening along `+axis` with half-angle
    /// `half_angle` in `(0, π/2)`.
    Cone {
        origin: Point3,
        axis: Vector3,
        half_angle: f64,
        radius: f64,
    },
    /// Sphere of `radius` about `center`; unit `axis` points at the north
    /// pole (`v = π/2`).
    Sphere {
        center: Point3,
        axis: Vector3,
        radius: f64,
    },
    /// Torus about the line through `center` with unit `axis`:
    /// `major_radius` from the axis to the tube center, `minor_radius` of
    /// the tube, with `major_radius > minor_radius > 0`.
    Torus {
        center: Point3,
        axis: Vector3,
        major_radius: f64,
        minor_radius: f64,
    },
}

/// Normalize `axis`, rejecting zero or non-finite length with `context`.
fn unit_axis(axis: Vector3, context: &'static str) -> CoreResult<Vector3> {
    let norm = axis.norm();
    if norm == 0.0 || !norm.is_finite() {
        return Err(CoreError::Degenerate {
            context,
            reason: format!("axis must have non-zero finite length, got {axis}"),
        });
    }
    Ok(axis / norm)
}

/// Reject a radius-like argument that is not positive and finite.
fn positive_radius(name: &'static str, value: f64) -> CoreResult<f64> {
    if value <= 0.0 || !value.is_finite() {
        return Err(CoreError::InvalidArgument {
            argument: name,
            reason: format!("must be positive and finite, got {value}"),
        });
    }
    Ok(value)
}

impl Surface3 {
    /// Plane through `origin` normal to `normal` (normalized here).
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `normal` has zero or non-finite length.
    pub fn plane(origin: Point3, normal: Vector3) -> CoreResult<Self> {
        Ok(Surface3::Plane {
            origin,
            normal: unit_axis(normal, "Surface3::plane")?,
        })
    }

    /// Cylinder of `radius` about the axis through `origin` along `axis`
    /// (normalized here).
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `axis` has zero or non-finite length;
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    pub fn cylinder(origin: Point3, axis: Vector3, radius: f64) -> CoreResult<Self> {
        Ok(Surface3::Cylinder {
            origin,
            axis: unit_axis(axis, "Surface3::cylinder")?,
            radius: positive_radius("radius", radius)?,
        })
    }

    /// Cone with `radius` at `origin`, widening along `axis` (normalized
    /// here) with `half_angle`.
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `axis` has zero or non-finite length;
    /// [`CoreError::InvalidArgument`] if `half_angle` is outside `(0, π/2)`
    /// or `radius` is negative or non-finite (zero places the apex at
    /// `v = 0`).
    pub fn cone(origin: Point3, axis: Vector3, half_angle: f64, radius: f64) -> CoreResult<Self> {
        if !(half_angle > 0.0 && half_angle < std::f64::consts::FRAC_PI_2) {
            return Err(CoreError::InvalidArgument {
                argument: "half_angle",
                reason: format!("must be in (0, PI/2), got {half_angle}"),
            });
        }
        if radius < 0.0 || !radius.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "radius",
                reason: format!("must be non-negative and finite, got {radius}"),
            });
        }
        Ok(Surface3::Cone {
            origin,
            axis: unit_axis(axis, "Surface3::cone")?,
            half_angle,
            radius,
        })
    }

    /// Sphere of `radius` about `center` with pole direction `axis`
    /// (normalized here).
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `axis` has zero or non-finite length;
    /// [`CoreError::InvalidArgument`] if `radius` is not positive and finite.
    pub fn sphere(center: Point3, axis: Vector3, radius: f64) -> CoreResult<Self> {
        Ok(Surface3::Sphere {
            center,
            axis: unit_axis(axis, "Surface3::sphere")?,
            radius: positive_radius("radius", radius)?,
        })
    }

    /// Torus about `center` with `axis` (normalized here),
    /// `major_radius > minor_radius > 0`.
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if `axis` has zero or non-finite length;
    /// [`CoreError::InvalidArgument`] if either radius is not positive and
    /// finite, or `major_radius <= minor_radius` (spindle/horn tori are not
    /// supported).
    pub fn torus(
        center: Point3,
        axis: Vector3,
        major_radius: f64,
        minor_radius: f64,
    ) -> CoreResult<Self> {
        positive_radius("major_radius", major_radius)?;
        positive_radius("minor_radius", minor_radius)?;
        if major_radius <= minor_radius {
            return Err(CoreError::InvalidArgument {
                argument: "major_radius",
                reason: format!(
                    "must exceed minor_radius ({major_radius} <= {minor_radius}); \
                     spindle/horn tori are not supported"
                ),
            });
        }
        Ok(Surface3::Torus {
            center,
            axis: unit_axis(axis, "Surface3::torus")?,
            major_radius,
            minor_radius,
        })
    }

    /// Exact axis-aligned bounding box of the full surface, for the
    /// bounded primitives (sphere, torus); `None` for the unbounded ones
    /// (plane, cylinder, cone).
    ///
    /// The torus box comes from the tube-center circle's box dilated by
    /// the tube radius: the circle's half-extent along world axis `i` is
    /// `R·√(1 − axisᵢ²)`, and the Minkowski sum with the radius-`r` tube
    /// ball adds exactly `r` per axis.
    pub fn bounding_box(&self) -> Option<BoundingBox3> {
        match self {
            Surface3::Sphere { center, radius, .. } => {
                let r = Vector3::new(*radius, *radius, *radius);
                Some(BoundingBox3::new(center - r, center + r))
            }
            Surface3::Torus {
                center,
                axis,
                major_radius,
                minor_radius,
            } => {
                let circle_half = |a: f64| major_radius * (1.0 - a * a).max(0.0).sqrt();
                let h = Vector3::new(
                    circle_half(axis.x) + minor_radius,
                    circle_half(axis.y) + minor_radius,
                    circle_half(axis.z) + minor_radius,
                );
                Some(BoundingBox3::new(center - h, center + h))
            }
            Surface3::Plane { .. } | Surface3::Cylinder { .. } | Surface3::Cone { .. } => None,
        }
    }

    /// The unit axis defining this surface's frame.
    fn frame_axis(&self) -> &Vector3 {
        match self {
            Surface3::Plane { normal, .. } => normal,
            Surface3::Cylinder { axis, .. }
            | Surface3::Cone { axis, .. }
            | Surface3::Sphere { axis, .. }
            | Surface3::Torus { axis, .. } => axis,
        }
    }

    /// Radial unit direction at angle `u` about the frame axis, plus its
    /// derivative with respect to `u` (the tangential direction).
    fn radial_frame(&self, u: f64) -> (Vector3, Vector3) {
        let (e_u, e_v) = plane_basis(self.frame_axis());
        let radial = e_u * u.cos() + e_v * u.sin();
        let tangential = e_v * u.cos() - e_u * u.sin();
        (radial, tangential)
    }

    /// Cone radius at axial position `v`: `radius + v*tan(half_angle)`.
    /// Zero exactly at the apex.
    fn cone_rho(half_angle: f64, radius: f64, v: f64) -> f64 {
        radius + v * half_angle.tan()
    }
}

impl SurfaceEval for Surface3 {
    fn point(&self, u: f64, v: f64) -> Point3 {
        match self {
            Surface3::Plane { origin, normal } => {
                let (e_u, e_v) = plane_basis(normal);
                origin + e_u * u + e_v * v
            }
            Surface3::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let (radial, _) = self.radial_frame(u);
                origin + radial * *radius + axis * v
            }
            Surface3::Cone {
                origin,
                axis,
                half_angle,
                radius,
            } => {
                let (radial, _) = self.radial_frame(u);
                origin + radial * Self::cone_rho(*half_angle, *radius, v) + axis * v
            }
            Surface3::Sphere {
                center,
                axis,
                radius,
            } => {
                let (radial, _) = self.radial_frame(u);
                center + (radial * v.cos() + axis * v.sin()) * *radius
            }
            Surface3::Torus {
                center,
                axis,
                major_radius,
                minor_radius,
            } => {
                let (radial, _) = self.radial_frame(u);
                center
                    + radial * (major_radius + minor_radius * v.cos())
                    + axis * (minor_radius * v.sin())
            }
        }
    }

    fn du(&self, u: f64, v: f64) -> Vector3 {
        match self {
            Surface3::Plane { normal, .. } => plane_basis(normal).0,
            Surface3::Cylinder { radius, .. } => {
                let (_, tangential) = self.radial_frame(u);
                tangential * *radius
            }
            Surface3::Cone {
                half_angle, radius, ..
            } => {
                let (_, tangential) = self.radial_frame(u);
                tangential * Self::cone_rho(*half_angle, *radius, v)
            }
            Surface3::Sphere { radius, .. } => {
                let (_, tangential) = self.radial_frame(u);
                tangential * (radius * v.cos())
            }
            Surface3::Torus {
                major_radius,
                minor_radius,
                ..
            } => {
                let (_, tangential) = self.radial_frame(u);
                tangential * (major_radius + minor_radius * v.cos())
            }
        }
    }

    fn dv(&self, u: f64, v: f64) -> Vector3 {
        match self {
            Surface3::Plane { normal, .. } => plane_basis(normal).1,
            Surface3::Cylinder { axis, .. } => *axis,
            Surface3::Cone {
                axis, half_angle, ..
            } => {
                let (radial, _) = self.radial_frame(u);
                radial * half_angle.tan() + axis
            }
            Surface3::Sphere { axis, radius, .. } => {
                let (radial, _) = self.radial_frame(u);
                (axis * v.cos() - radial * v.sin()) * *radius
            }
            Surface3::Torus {
                axis, minor_radius, ..
            } => {
                let (radial, _) = self.radial_frame(u);
                (axis * v.cos() - radial * v.sin()) * *minor_radius
            }
        }
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
        match self {
            Surface3::Plane { normal, .. } => Some(*normal),
            Surface3::Cylinder { .. } => Some(self.radial_frame(u).0),
            Surface3::Cone { .. } => {
                // du × dv = rho * (radial - tan(a)*axis): degenerate at the
                // apex, flips direction with the sign of rho (other nappe).
                if self.is_singular(u, v) {
                    return None;
                }
                let cross = self.du(u, v).cross(&self.dv(u, v));
                Some(cross / cross.norm())
            }
            Surface3::Sphere { center, radius, .. } => {
                // Well-defined everywhere including the poles: the outward
                // radial direction, which normalized du × dv converges to.
                Some((self.point(u, v) - center) / *radius)
            }
            Surface3::Torus { axis, .. } => {
                let (radial, _) = self.radial_frame(u);
                Some(radial * v.cos() + axis * v.sin())
            }
        }
    }

    fn domain_u(&self) -> (f64, f64) {
        match self {
            Surface3::Plane { .. } => (f64::NEG_INFINITY, f64::INFINITY),
            _ => (0.0, TWO_PI),
        }
    }

    fn domain_v(&self) -> (f64, f64) {
        match self {
            Surface3::Plane { .. } | Surface3::Cylinder { .. } | Surface3::Cone { .. } => {
                (f64::NEG_INFINITY, f64::INFINITY)
            }
            Surface3::Sphere { .. } => (-std::f64::consts::FRAC_PI_2, std::f64::consts::FRAC_PI_2),
            Surface3::Torus { .. } => (0.0, TWO_PI),
        }
    }

    fn is_periodic_u(&self) -> bool {
        !matches!(self, Surface3::Plane { .. })
    }

    fn is_periodic_v(&self) -> bool {
        matches!(self, Surface3::Torus { .. })
    }

    fn period_u(&self) -> Option<f64> {
        if self.is_periodic_u() {
            Some(TWO_PI)
        } else {
            None
        }
    }

    fn period_v(&self) -> Option<f64> {
        if self.is_periodic_v() {
            Some(TWO_PI)
        } else {
            None
        }
    }

    fn is_singular(&self, _u: f64, v: f64) -> bool {
        match self {
            Surface3::Plane { .. } | Surface3::Cylinder { .. } | Surface3::Torus { .. } => false,
            // Apex: the u-circle collapses to a point.
            Surface3::Cone {
                half_angle, radius, ..
            } => Self::cone_rho(*half_angle, *radius, v).abs() <= SYSTEM_RESOLUTION,
            // Poles: the longitude circle collapses (du = 0 where cos v = 0).
            Surface3::Sphere { .. } => v.cos().abs() <= SYSTEM_RESOLUTION,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::{FRAC_PI_2, FRAC_PI_4, PI};

    const EPS: f64 = 1e-10;

    fn assert_point_eq(a: &Point3, b: &Point3) {
        assert!(
            (a - b).norm() < EPS,
            "points differ: {a:?} vs {b:?} (dist {})",
            (a - b).norm()
        );
    }

    fn assert_vec_eq(a: &Vector3, b: &Vector3) {
        assert!(
            (a - b).norm() < EPS,
            "vectors differ: {a:?} vs {b:?} (dist {})",
            (a - b).norm()
        );
    }

    /// Central finite differences must match the analytic partials, and the
    /// normal (where defined) must be unit, perpendicular to both partials,
    /// and aligned with du × dv.
    fn check_partials_numerically(s: &Surface3, u: f64, v: f64) {
        let h = 1e-6;
        let fd_du = (s.point(u + h, v) - s.point(u - h, v)) / (2.0 * h);
        let fd_dv = (s.point(u, v + h) - s.point(u, v - h)) / (2.0 * h);
        let du = s.du(u, v);
        let dv = s.dv(u, v);
        assert!(
            (fd_du - du).norm() < 1e-5,
            "du mismatch at ({u},{v}): analytic {du:?} vs fd {fd_du:?}"
        );
        assert!(
            (fd_dv - dv).norm() < 1e-5,
            "dv mismatch at ({u},{v}): analytic {dv:?} vs fd {fd_dv:?}"
        );
        if let Some(n) = s.normal(u, v) {
            assert!((n.norm() - 1.0).abs() < EPS, "normal not unit at ({u},{v})");
            assert!(n.dot(&du).abs() < 1e-8, "normal not ⟂ du at ({u},{v})");
            assert!(n.dot(&dv).abs() < 1e-8, "normal not ⟂ dv at ({u},{v})");
            let cross = du.cross(&dv);
            if cross.norm() > 1e-9 {
                assert!(n.dot(&cross) > 0.0, "normal opposes du × dv at ({u},{v})");
            }
        }
    }

    const SAMPLE_PARAMS: [(f64, f64); 6] = [
        (0.0, 0.0),
        (0.7, -1.3),
        (FRAC_PI_2, 0.4),
        (PI, 1.0),
        (4.0, -0.6),
        (5.9, 0.2),
    ];

    #[test]
    fn plane_points_and_normal() {
        // normal = +Z: plane_basis gives e_u = X, e_v = Y.
        let p = Surface3::plane(Point3::new(1.0, 2.0, 3.0), Vector3::new(0.0, 0.0, 4.0))
            .expect("valid surface");
        assert_point_eq(&p.point(0.0, 0.0), &Point3::new(1.0, 2.0, 3.0));
        assert_point_eq(&p.point(2.0, -1.0), &Point3::new(3.0, 1.0, 3.0));
        assert_vec_eq(&p.normal(7.0, -3.0).unwrap(), &Vector3::z());
        assert_vec_eq(&p.du(0.0, 0.0), &Vector3::x());
        assert_vec_eq(&p.dv(0.0, 0.0), &Vector3::y());
        for (u, v) in SAMPLE_PARAMS {
            check_partials_numerically(&p, u, v);
            assert!(!p.is_singular(u, v));
        }
    }

    #[test]
    fn plane_domain_and_periodicity() {
        let p = Surface3::plane(Point3::origin(), Vector3::z()).expect("valid surface");
        assert!(p.domain_u().0.is_infinite() && p.domain_u().1.is_infinite());
        assert!(p.domain_v().0.is_infinite() && p.domain_v().1.is_infinite());
        assert!(!p.is_periodic_u() && !p.is_periodic_v());
        assert_eq!(p.period_u(), None);
        assert_eq!(p.period_v(), None);
    }

    #[test]
    fn cylinder_analytic_points_and_normals() {
        // axis = +Z: radial(0) = X, radial(π/2) = Y.
        let c = Surface3::cylinder(Point3::origin(), Vector3::z(), 2.0).expect("valid surface");
        assert_point_eq(&c.point(0.0, 0.0), &Point3::new(2.0, 0.0, 0.0));
        assert_point_eq(&c.point(FRAC_PI_2, 3.0), &Point3::new(0.0, 2.0, 3.0));
        assert_point_eq(&c.point(PI, -1.0), &Point3::new(-2.0, 0.0, -1.0));
        assert_vec_eq(&c.normal(0.0, 5.0).unwrap(), &Vector3::x());
        assert_vec_eq(&c.normal(FRAC_PI_2, -2.0).unwrap(), &Vector3::y());
        for (u, v) in SAMPLE_PARAMS {
            check_partials_numerically(&c, u, v);
            assert!(!c.is_singular(u, v));
        }
    }

    #[test]
    fn cylinder_periodicity_wraps() {
        let c = Surface3::cylinder(
            Point3::new(1.0, -1.0, 0.5),
            Vector3::new(1.0, 1.0, 0.0),
            1.5,
        )
        .expect("valid surface");
        assert!(c.is_periodic_u());
        assert!(!c.is_periodic_v());
        assert_eq!(c.period_u(), Some(TWO_PI));
        assert_point_eq(&c.point(0.3, 2.0), &c.point(0.3 + TWO_PI, 2.0));
        assert_vec_eq(&c.du(0.3, 2.0), &c.du(0.3 + TWO_PI, 2.0));
        assert_eq!(c.domain_u(), (0.0, TWO_PI));
    }

    #[test]
    fn cone_points_normals_and_apex() {
        // half_angle = π/4 (tan = 1), radius 1 at v = 0: apex at v = -1.
        let k =
            Surface3::cone(Point3::origin(), Vector3::z(), FRAC_PI_4, 1.0).expect("valid surface");
        assert_point_eq(&k.point(0.0, 0.0), &Point3::new(1.0, 0.0, 0.0));
        assert_point_eq(&k.point(0.0, 1.0), &Point3::new(2.0, 0.0, 1.0));
        // Apex reached exactly.
        assert_point_eq(&k.point(2.3, -1.0), &Point3::new(0.0, 0.0, -1.0));
        assert!(k.is_singular(0.0, -1.0));
        assert!(k.normal(0.0, -1.0).is_none());
        assert!(!k.is_singular(0.0, 0.0));
        // Outward normal tilts away from +axis: n = (radial - axis)/√2.
        let inv_sqrt2 = 1.0 / 2.0f64.sqrt();
        assert_vec_eq(
            &k.normal(0.0, 0.0).unwrap(),
            &Vector3::new(inv_sqrt2, 0.0, -inv_sqrt2),
        );
        for (u, v) in SAMPLE_PARAMS {
            check_partials_numerically(&k, u, v);
        }
    }

    #[test]
    fn cone_second_nappe_evaluates_with_flipped_normal() {
        let k =
            Surface3::cone(Point3::origin(), Vector3::z(), FRAC_PI_4, 1.0).expect("valid surface");
        // v = -2 is past the apex: rho = -1, the mirror cone.
        let p = k.point(0.0, -2.0);
        assert_point_eq(&p, &Point3::new(-1.0, 0.0, -2.0));
        // Normal still normalized, still consistent with du × dv.
        check_partials_numerically(&k, 0.0, -2.0);
    }

    #[test]
    fn sphere_points_poles_and_normals() {
        let s =
            Surface3::sphere(Point3::new(0.0, 0.0, 1.0), Vector3::z(), 2.0).expect("valid surface");
        assert_point_eq(&s.point(0.0, 0.0), &Point3::new(2.0, 0.0, 1.0));
        // Poles: point = center ± r*axis, singular, but normal = ±axis.
        assert_point_eq(&s.point(1.234, FRAC_PI_2), &Point3::new(0.0, 0.0, 3.0));
        assert_point_eq(&s.point(4.567, -FRAC_PI_2), &Point3::new(0.0, 0.0, -1.0));
        assert!(s.is_singular(0.0, FRAC_PI_2));
        assert!(s.is_singular(3.0, -FRAC_PI_2));
        assert!(!s.is_singular(0.0, 0.0));
        assert_vec_eq(&s.normal(1.234, FRAC_PI_2).unwrap(), &Vector3::z());
        assert_vec_eq(&s.normal(4.567, -FRAC_PI_2).unwrap(), &-Vector3::z());
        assert_vec_eq(&s.normal(0.0, 0.0).unwrap(), &Vector3::x());
        // Away from the poles the partials behave analytically.
        for (u, v) in [(0.0, 0.0), (1.0, 0.7), (PI, -1.2), (5.0, 1.4)] {
            check_partials_numerically(&s, u, v);
        }
    }

    #[test]
    fn sphere_periodicity_and_domain() {
        let s = Surface3::sphere(Point3::origin(), Vector3::new(0.0, 1.0, 1.0), 1.0)
            .expect("valid surface");
        assert!(s.is_periodic_u());
        assert!(!s.is_periodic_v());
        assert_eq!(s.period_u(), Some(TWO_PI));
        assert_eq!(s.domain_v(), (-FRAC_PI_2, FRAC_PI_2));
        assert_point_eq(&s.point(0.4, 0.3), &s.point(0.4 + TWO_PI, 0.3));
        // Every point lies at distance r from the center.
        for (u, v) in SAMPLE_PARAMS {
            let r = s.point(u, v) - Point3::origin();
            assert!((r.norm() - 1.0).abs() < EPS, "off sphere at ({u},{v})");
        }
    }

    #[test]
    fn torus_analytic_points_and_implicit_equation() {
        // axis = +Z, R = 3, r = 1: implicit (√(x²+y²) − R)² + z² = r².
        let t = Surface3::torus(Point3::origin(), Vector3::z(), 3.0, 1.0).expect("valid surface");
        assert_point_eq(&t.point(0.0, 0.0), &Point3::new(4.0, 0.0, 0.0));
        assert_point_eq(&t.point(0.0, PI), &Point3::new(2.0, 0.0, 0.0));
        assert_point_eq(&t.point(0.0, FRAC_PI_2), &Point3::new(3.0, 0.0, 1.0));
        assert_point_eq(&t.point(FRAC_PI_2, 0.0), &Point3::new(0.0, 4.0, 0.0));
        for (u, v) in SAMPLE_PARAMS {
            let p = t.point(u, v);
            let ring = (p.x * p.x + p.y * p.y).sqrt() - 3.0;
            assert!(
                (ring * ring + p.z * p.z - 1.0).abs() < EPS,
                "off torus at ({u},{v})"
            );
            check_partials_numerically(&t, u, v);
            assert!(!t.is_singular(u, v));
        }
    }

    #[test]
    fn torus_normals_point_outward_from_tube() {
        let t = Surface3::torus(Point3::origin(), Vector3::z(), 3.0, 1.0).expect("valid surface");
        // Outer equator: normal = +radial; inner equator: −radial; top: +axis.
        assert_vec_eq(&t.normal(0.0, 0.0).unwrap(), &Vector3::x());
        assert_vec_eq(&t.normal(0.0, PI).unwrap(), &-Vector3::x());
        assert_vec_eq(&t.normal(0.0, FRAC_PI_2).unwrap(), &Vector3::z());
    }

    #[test]
    fn torus_doubly_periodic() {
        let t = Surface3::torus(
            Point3::new(1.0, 0.0, 0.0),
            Vector3::new(1.0, 2.0, 2.0),
            5.0,
            0.5,
        )
        .expect("valid surface");
        assert!(t.is_periodic_u() && t.is_periodic_v());
        assert_eq!(t.period_u(), Some(TWO_PI));
        assert_eq!(t.period_v(), Some(TWO_PI));
        assert_eq!(t.domain_u(), (0.0, TWO_PI));
        assert_eq!(t.domain_v(), (0.0, TWO_PI));
        assert_point_eq(&t.point(0.2, 0.9), &t.point(0.2 + TWO_PI, 0.9));
        assert_point_eq(&t.point(0.2, 0.9), &t.point(0.2, 0.9 + TWO_PI));
    }

    #[test]
    fn constructors_normalize_axes() {
        let c = Surface3::cylinder(Point3::origin(), Vector3::new(0.0, 0.0, 9.0), 1.0)
            .expect("valid surface");
        match &c {
            Surface3::Cylinder { axis, .. } => assert_vec_eq(axis, &Vector3::z()),
            _ => unreachable!(),
        }
        // v is axial distance because the axis is unit length.
        assert_point_eq(&c.point(0.0, 2.0), &Point3::new(1.0, 0.0, 2.0));
    }

    #[test]
    fn plane_rejects_zero_normal() {
        let err = Surface3::plane(Point3::origin(), Vector3::zeros()).unwrap_err();
        assert!(matches!(err, CoreError::Degenerate { .. }), "got {err}");
        let msg = err.to_string();
        assert!(msg.contains("Surface3::plane"), "missing context: {msg}");
        assert!(msg.contains("non-zero"), "missing constraint: {msg}");
    }

    #[test]
    fn constructors_reject_zero_axis() {
        for (name, err) in [
            (
                "cylinder",
                Surface3::cylinder(Point3::origin(), Vector3::zeros(), 1.0).unwrap_err(),
            ),
            (
                "cone",
                Surface3::cone(Point3::origin(), Vector3::zeros(), FRAC_PI_4, 1.0).unwrap_err(),
            ),
            (
                "sphere",
                Surface3::sphere(Point3::origin(), Vector3::zeros(), 1.0).unwrap_err(),
            ),
            (
                "torus",
                Surface3::torus(Point3::origin(), Vector3::zeros(), 3.0, 1.0).unwrap_err(),
            ),
        ] {
            assert!(
                matches!(err, CoreError::Degenerate { .. }),
                "{name}: got {err}"
            );
            assert!(err.to_string().contains(name), "{name}: unhelpful: {err}");
        }
    }

    #[test]
    fn cylinder_rejects_nonpositive_radius() {
        for bad in [0.0, -2.0, f64::NAN, f64::INFINITY] {
            let err = Surface3::cylinder(Point3::origin(), Vector3::z(), bad).unwrap_err();
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
    fn sphere_rejects_nonpositive_radius() {
        let err = Surface3::sphere(Point3::origin(), Vector3::z(), -1.0).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "radius",
                    ..
                }
            ),
            "got {err}"
        );
    }

    #[test]
    fn cone_rejects_bad_half_angle() {
        for bad in [0.0, -0.1, FRAC_PI_2, f64::NAN] {
            let err = Surface3::cone(Point3::origin(), Vector3::z(), bad, 1.0).unwrap_err();
            assert!(
                matches!(
                    err,
                    CoreError::InvalidArgument {
                        argument: "half_angle",
                        ..
                    }
                ),
                "half_angle {bad}: got {err}"
            );
            assert!(err.to_string().contains("PI/2"), "unhelpful: {err}");
        }
    }

    #[test]
    fn cone_rejects_negative_radius() {
        let err = Surface3::cone(Point3::origin(), Vector3::z(), FRAC_PI_4, -1.0).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "radius",
                    ..
                }
            ),
            "got {err}"
        );
        // Zero radius is allowed: apex at v = 0.
        Surface3::cone(Point3::origin(), Vector3::z(), FRAC_PI_4, 0.0).expect("apex cone valid");
    }

    #[test]
    fn torus_rejects_spindle_and_bad_radii() {
        let err = Surface3::torus(Point3::origin(), Vector3::z(), 1.0, 1.0).unwrap_err();
        assert!(
            matches!(
                err,
                CoreError::InvalidArgument {
                    argument: "major_radius",
                    ..
                }
            ),
            "got {err}"
        );
        assert!(
            err.to_string().contains("minor_radius"),
            "missing constraint: {err}"
        );
        for (major, minor, argument) in [
            (0.0, 1.0, "major_radius"),
            (3.0, -1.0, "minor_radius"),
            (f64::NAN, 1.0, "major_radius"),
        ] {
            let err = Surface3::torus(Point3::origin(), Vector3::z(), major, minor).unwrap_err();
            assert!(
                matches!(&err, CoreError::InvalidArgument { argument: a, .. } if *a == argument),
                "R={major} r={minor}: got {err}"
            );
        }
    }

    #[test]
    fn bounding_box_none_for_unbounded_surfaces() {
        let plane = Surface3::plane(Point3::origin(), Vector3::z()).expect("valid surface");
        let cyl = Surface3::cylinder(Point3::origin(), Vector3::z(), 1.0).expect("valid surface");
        let cone =
            Surface3::cone(Point3::origin(), Vector3::z(), FRAC_PI_4, 1.0).expect("valid surface");
        assert!(plane.bounding_box().is_none());
        assert!(cyl.bounding_box().is_none());
        assert!(cone.bounding_box().is_none());
    }

    #[test]
    fn sphere_bounding_box_is_exact() {
        let s = Surface3::sphere(
            Point3::new(1.0, -2.0, 3.0),
            Vector3::new(1.0, 1.0, 0.0),
            2.5,
        )
        .expect("valid surface");
        let bb = s.bounding_box().expect("sphere is bounded");
        assert_point_eq(&bb.min, &Point3::new(-1.5, -4.5, 0.5));
        assert_point_eq(&bb.max, &Point3::new(3.5, 0.5, 5.5));
    }

    #[test]
    fn torus_bounding_box_axis_aligned_is_exact() {
        // Axis = Z: box is ±(R + r) laterally, ±r axially.
        let t = Surface3::torus(Point3::new(1.0, 2.0, 3.0), Vector3::z(), 3.0, 0.5)
            .expect("valid surface");
        let bb = t.bounding_box().expect("torus is bounded");
        assert_point_eq(&bb.min, &Point3::new(-2.5, -1.5, 2.5));
        assert_point_eq(&bb.max, &Point3::new(4.5, 5.5, 3.5));
    }

    /// For a tilted torus the box must contain every surface point (dense
    /// sample) and be tight: some sample must come within sampling error
    /// of each of the six box faces.
    #[test]
    fn torus_bounding_box_tilted_contains_samples_and_is_tight() {
        let t = Surface3::torus(
            Point3::new(-1.0, 0.5, 2.0),
            Vector3::new(1.0, 2.0, -1.0),
            2.0,
            0.7,
        )
        .expect("valid surface");
        let bb = t.bounding_box().expect("torus is bounded");
        let n = 200;
        let mut sampled = BoundingBox3::EMPTY;
        for i in 0..n {
            for j in 0..n {
                let u = TWO_PI * i as f64 / n as f64;
                let v = TWO_PI * j as f64 / n as f64;
                let p = t.point(u, v);
                assert!(
                    p.x >= bb.min.x - EPS
                        && p.y >= bb.min.y - EPS
                        && p.z >= bb.min.z - EPS
                        && p.x <= bb.max.x + EPS
                        && p.y <= bb.max.y + EPS
                        && p.z <= bb.max.z + EPS,
                    "point {p:?} at ({u},{v}) escapes box {bb:?}"
                );
                sampled = sampled.union(&BoundingBox3::from_points([p]));
            }
        }
        // Sampling a 200×200 grid reaches within O((2π/200)²·R) of each
        // extreme; 1e-2 is comfortably above that error and far below r.
        let slack = 1e-2;
        assert!(
            (sampled.min - bb.min).norm() < slack && (sampled.max - bb.max).norm() < slack,
            "box not tight: exact {bb:?} vs sampled {sampled:?}"
        );
    }
}
