//! Exact intersection curves between analytic surface primitives.
//!
//! [`intersect`] returns the full intersection set of two [`Surface3`]
//! primitives as closed-form [`Curve3`] geometry, with every curve
//! classified as transversal (the surfaces cross) or tangential (they
//! touch without crossing). Degenerate contacts that are a single point
//! (a plane tangent to a sphere, a plane through a cone apex) are reported
//! as [`SurfaceIntersection::TangentPoint`], and identical surfaces as
//! [`SurfaceIntersection::Coincident`].
//!
//! Coverage is the analytic MVP: plane against every primitive where the
//! result is expressible with the current [`Curve3`] variants (lines,
//! circles, ellipses), plus the equal-radius cylinder-cylinder special
//! cases (parallel axes → line pair, intersecting axes → ellipse pair).
//! Configurations whose intersection needs conics or quartics we cannot
//! represent yet (plane-cone parabolas/hyperbolas, oblique plane-torus,
//! skew or unequal-radius cylinder pairs, quadric-quadric in general)
//! return [`CoreError::NotImplemented`] — never a misleading `Empty`.
//!
//! All classification comparisons (parallelism, tangency, coincidence) go
//! through the caller's [`ToleranceContext`], per the kernel tolerance
//! model.

use crate::curve::Curve3;
use crate::surface::Surface3;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::tolerance::{ANGULAR_RESOLUTION, ToleranceContext};
use opensolid_core::types::Point3;

/// How two surfaces meet along an intersection curve.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum IntersectionKind {
    /// The surfaces cross each other along the curve.
    Transversal,
    /// The surfaces touch along the curve without crossing (their normals
    /// are parallel there).
    Tangential,
}

/// One intersection curve with its crossing classification.
#[derive(Debug, Clone, PartialEq)]
pub struct IntersectionCurve {
    pub curve: Curve3,
    pub kind: IntersectionKind,
}

/// The complete intersection set of two surfaces.
#[derive(Debug, Clone, PartialEq)]
pub enum SurfaceIntersection {
    /// The surfaces do not meet.
    Empty,
    /// The surfaces touch at exactly one point (always a tangential /
    /// degenerate contact — a transversal intersection is never a point).
    TangentPoint(Point3),
    /// The surfaces meet along one or more curves.
    Curves(Vec<IntersectionCurve>),
    /// The surfaces are the same geometric locus.
    Coincident,
}

impl SurfaceIntersection {
    fn transversal(curves: Vec<Curve3>) -> Self {
        SurfaceIntersection::Curves(
            curves
                .into_iter()
                .map(|curve| IntersectionCurve {
                    curve,
                    kind: IntersectionKind::Transversal,
                })
                .collect(),
        )
    }

    fn tangential(curve: Curve3) -> Self {
        SurfaceIntersection::Curves(vec![IntersectionCurve {
            curve,
            kind: IntersectionKind::Tangential,
        }])
    }
}

/// Exact intersection of two analytic surfaces.
///
/// The result is symmetric in the arguments up to curve orientation.
///
/// # Errors
/// [`CoreError::NotImplemented`] for surface pairs or configurations whose
/// intersection cannot be expressed with the current [`Curve3`] variants
/// (see the module docs for the supported set).
pub fn intersect(
    a: &Surface3,
    b: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    use Surface3::*;
    match (a, b) {
        (Plane { .. }, Plane { .. }) => Ok(plane_plane(a, b, tol)),
        (Plane { .. }, Sphere { .. }) => Ok(plane_sphere(a, b, tol)),
        (Sphere { .. }, Plane { .. }) => Ok(plane_sphere(b, a, tol)),
        (Plane { .. }, Cylinder { .. }) => Ok(plane_cylinder(a, b, tol)),
        (Cylinder { .. }, Plane { .. }) => Ok(plane_cylinder(b, a, tol)),
        (Plane { .. }, Cone { .. }) => plane_cone(a, b, tol),
        (Cone { .. }, Plane { .. }) => plane_cone(b, a, tol),
        (Plane { .. }, Torus { .. }) => plane_torus(a, b, tol),
        (Torus { .. }, Plane { .. }) => plane_torus(b, a, tol),
        (Cylinder { .. }, Cylinder { .. }) => cylinder_cylinder(a, b, tol),
        _ => Err(CoreError::NotImplemented {
            feature: "analytic SSI for this surface pair (only plane-X and \
                      cylinder-cylinder cases are implemented)",
        }),
    }
}

/// Effective angular tolerance in radians (mirrors the private clamp in
/// [`ToleranceContext`]). Used for scalar sine/cosine-of-angle tests that
/// have no dedicated helper.
fn angular_tol(tol: &ToleranceContext) -> f64 {
    tol.angular.max(ANGULAR_RESOLUTION)
}

fn plane_plane(a: &Surface3, b: &Surface3, tol: &ToleranceContext) -> SurfaceIntersection {
    let (
        Surface3::Plane {
            origin: o1,
            normal: n1,
        },
        Surface3::Plane {
            origin: o2,
            normal: n2,
        },
    ) = (a, b)
    else {
        unreachable!("dispatched on Plane/Plane")
    };

    if tol.vectors_parallel(n1, n2) {
        return if tol.approx_zero(n1.dot(&(o2 - o1))) {
            SurfaceIntersection::Coincident
        } else {
            SurfaceIntersection::Empty
        };
    }

    // Direction along both planes; a point on both found by sliding from
    // `o1` within plane 1 (along u, the in-plane component of n2) until
    // plane 2 is satisfied.
    let dir = n1.cross(n2).normalize();
    let u = n2 - n1 * n1.dot(n2);
    let t = n2.dot(&(o2 - o1)) / n2.dot(&u);
    let origin = o1 + u * t;
    SurfaceIntersection::transversal(vec![Curve3::Line { origin, dir }])
}

fn plane_sphere(
    plane: &Surface3,
    sphere: &Surface3,
    tol: &ToleranceContext,
) -> SurfaceIntersection {
    let (Surface3::Plane { origin, normal }, &Surface3::Sphere { center, radius, .. }) =
        (plane, sphere)
    else {
        unreachable!("dispatched on Plane/Sphere")
    };

    // Signed distance from the sphere center to the plane, and its foot.
    let h = normal.dot(&(center - origin));
    let foot = center - normal * h;
    if tol.approx_eq(h.abs(), radius) {
        SurfaceIntersection::TangentPoint(foot)
    } else if h.abs() > radius {
        SurfaceIntersection::Empty
    } else {
        SurfaceIntersection::transversal(vec![Curve3::Circle {
            center: foot,
            axis: *normal,
            radius: (radius * radius - h * h).sqrt(),
        }])
    }
}

fn plane_cylinder(
    plane: &Surface3,
    cylinder: &Surface3,
    tol: &ToleranceContext,
) -> SurfaceIntersection {
    let (
        Surface3::Plane {
            origin: po,
            normal: n,
        },
        &Surface3::Cylinder {
            origin: co,
            axis,
            radius,
        },
    ) = (plane, cylinder)
    else {
        unreachable!("dispatched on Plane/Cylinder")
    };

    let c = n.dot(&axis);

    if tol.vectors_parallel(n, &axis) {
        // Plane perpendicular to the axis: a circle where the axis pierces
        // the plane.
        let t = n.dot(&(po - co)) / c;
        return SurfaceIntersection::transversal(vec![Curve3::Circle {
            center: co + axis * t,
            axis,
            radius,
        }]);
    }

    if c.abs() <= angular_tol(tol) {
        // Plane parallel to the axis: zero, one (tangent), or two lines.
        let d = n.dot(&(co - po));
        let foot = co - n * d;
        if tol.approx_eq(d.abs(), radius) {
            return SurfaceIntersection::tangential(Curve3::Line {
                origin: foot,
                dir: axis,
            });
        }
        if d.abs() > radius {
            return SurfaceIntersection::Empty;
        }
        let w = axis.cross(n); // unit: axis ⟂ n here
        let half_chord = (radius * radius - d * d).sqrt();
        return SurfaceIntersection::transversal(vec![
            Curve3::Line {
                origin: foot + w * half_chord,
                dir: axis,
            },
            Curve3::Line {
                origin: foot - w * half_chord,
                dir: axis,
            },
        ]);
    }

    // Oblique plane: an ellipse centered where the axis pierces the plane,
    // major axis along the in-plane projection of the cylinder axis,
    // stretched by 1/|cos| of the axis-normal angle.
    let t = n.dot(&(po - co)) / c;
    let center = co + axis * t;
    let major_dir = (axis - n * c).normalize();
    SurfaceIntersection::transversal(vec![Curve3::Ellipse {
        center,
        axis: *n,
        major_dir,
        major_radius: radius / c.abs(),
        minor_radius: radius,
    }])
}

fn plane_cone(
    plane: &Surface3,
    cone: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    let (
        Surface3::Plane {
            origin: po,
            normal: n,
        },
        &Surface3::Cone {
            origin: co,
            axis,
            half_angle,
            radius,
        },
    ) = (plane, cone)
    else {
        unreachable!("dispatched on Plane/Cone")
    };

    let apex = co - axis * (radius / half_angle.tan());

    if tol.vectors_parallel(n, &axis) {
        // Plane perpendicular to the axis: a circle of the local cone
        // radius, or the apex alone when the plane passes through it.
        let c = n.dot(&axis);
        let v = n.dot(&(po - co)) / c;
        let rho = radius + v * half_angle.tan();
        let center = co + axis * v;
        return Ok(if tol.approx_zero(rho) {
            SurfaceIntersection::TangentPoint(center)
        } else {
            SurfaceIntersection::transversal(vec![Curve3::Circle {
                center,
                axis,
                radius: rho.abs(),
            }])
        });
    }

    // Orient the normal to make an acute angle with the axis; the plane is
    // unchanged. cos_n = cos(angle between n and axis).
    let n = if n.dot(&axis) < 0.0 { -n } else { *n };
    let cos_n = n.dot(&axis);
    let sin_n = (1.0 - cos_n * cos_n).sqrt();

    // The section is an ellipse iff the plane is steeper to the axis than
    // the generators: (angle of n to axis) + half_angle < π/2. At equality
    // the section is a parabola, beyond it a hyperbola — not representable
    // as Curve3 yet.
    let ellipse_margin = cos_n * half_angle.cos() - sin_n * half_angle.sin();
    if ellipse_margin <= angular_tol(tol) {
        return Err(CoreError::NotImplemented {
            feature: "plane-cone sections other than circles and ellipses \
                      (parabola, hyperbola, generator lines)",
        });
    }

    let d = n.dot(&(apex - po));
    if tol.approx_zero(d) {
        // A steep plane through the apex meets the cone only there.
        return Ok(SurfaceIntersection::TangentPoint(apex));
    }

    // Extreme generators lie in the span of the axis and the normal. Their
    // hits on the plane are the major-axis endpoints.
    let u = (n - axis * cos_n) / sin_n;
    let (sin_a, cos_a) = half_angle.sin_cos();
    let g_plus = axis * cos_a + u * sin_a;
    let g_minus = axis * cos_a - u * sin_a;
    let t_plus = -d / n.dot(&g_plus);
    let t_minus = -d / n.dot(&g_minus);
    let end_plus = apex + g_plus * t_plus;
    let end_minus = apex + g_minus * t_minus;

    let center = na_center(&end_plus, &end_minus);
    let span = end_plus - end_minus;
    let major_radius = span.norm() / 2.0;
    let major_dir = span / span.norm();

    // Semi-minor from the cone quadric F(x) = (axis·(x-q))² − cos²α|x-q|²:
    // along w (in-plane, conjugate to the major axis) the linear term
    // vanishes at the center, so F(center + s·w) = 0 gives s² directly.
    let w = n.cross(&major_dir);
    let m = center - apex;
    let f_center = axis.dot(&m).powi(2) - cos_a * cos_a * m.norm_squared();
    let f_w = axis.dot(&w).powi(2) - cos_a * cos_a;
    let minor_radius = (-f_center / f_w).sqrt();

    Ok(SurfaceIntersection::transversal(vec![Curve3::Ellipse {
        center,
        axis: n,
        major_dir,
        major_radius,
        minor_radius,
    }]))
}

fn plane_torus(
    plane: &Surface3,
    torus: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    let (
        Surface3::Plane {
            origin: po,
            normal: n,
        },
        &Surface3::Torus {
            center,
            axis,
            major_radius,
            minor_radius,
        },
    ) = (plane, torus)
    else {
        unreachable!("dispatched on Plane/Torus")
    };

    if tol.vectors_parallel(n, &axis) {
        // Plane perpendicular to the axis at height h from the center:
        // ring radii ρ with (ρ − R)² + h² = r².
        let h = axis.dot(&(po - center));
        let ring_center = center + axis * h;
        return Ok(if tol.approx_eq(h.abs(), minor_radius) {
            SurfaceIntersection::tangential(Curve3::Circle {
                center: ring_center,
                axis,
                radius: major_radius,
            })
        } else if h.abs() > minor_radius {
            SurfaceIntersection::Empty
        } else {
            let half_width = (minor_radius * minor_radius - h * h).sqrt();
            SurfaceIntersection::transversal(vec![
                Curve3::Circle {
                    center: ring_center,
                    axis,
                    radius: major_radius + half_width,
                },
                Curve3::Circle {
                    center: ring_center,
                    axis,
                    radius: major_radius - half_width,
                },
            ])
        });
    }

    if n.dot(&axis).abs() <= angular_tol(tol) && tol.approx_zero(n.dot(&(center - po))) {
        // Plane through the axis: the two tube cross-section circles.
        let w = axis.cross(n); // unit: axis ⟂ n here
        return Ok(SurfaceIntersection::transversal(vec![
            Curve3::Circle {
                center: center + w * major_radius,
                axis: *n,
                radius: minor_radius,
            },
            Curve3::Circle {
                center: center - w * major_radius,
                axis: *n,
                radius: minor_radius,
            },
        ]));
    }

    Err(CoreError::NotImplemented {
        feature: "plane-torus sections other than axis-perpendicular and \
                  axis-containing planes (general sections are quartics)",
    })
}

fn cylinder_cylinder(
    a: &Surface3,
    b: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    let (
        &Surface3::Cylinder {
            origin: o1,
            axis: a1,
            radius: r1,
        },
        &Surface3::Cylinder {
            origin: o2,
            axis: a2,
            radius: r2,
        },
    ) = (a, b)
    else {
        unreachable!("dispatched on Cylinder/Cylinder")
    };

    if !tol.approx_eq(r1, r2) {
        return Err(CoreError::NotImplemented {
            feature: "cylinder-cylinder intersection with unequal radii",
        });
    }
    let radius = r1;

    if tol.vectors_parallel(&a1, &a2) {
        // Parallel axes separated by d: zero, one (tangent), or two lines.
        let offset = o2 - o1;
        let perp = offset - a1 * a1.dot(&offset);
        if tol.vector_approx_zero(&perp) {
            return Ok(SurfaceIntersection::Coincident);
        }
        let d = perp.norm();
        let mid = o1 + perp * 0.5;
        if tol.approx_eq(d, 2.0 * radius) {
            return Ok(SurfaceIntersection::tangential(Curve3::Line {
                origin: mid,
                dir: a1,
            }));
        }
        if d > 2.0 * radius {
            return Ok(SurfaceIntersection::Empty);
        }
        let w = a1.cross(&(perp / d)); // unit: a1 ⟂ perp
        let half_chord = (radius * radius - d * d / 4.0).sqrt();
        return Ok(SurfaceIntersection::transversal(vec![
            Curve3::Line {
                origin: mid + w * half_chord,
                dir: a1,
            },
            Curve3::Line {
                origin: mid - w * half_chord,
                dir: a1,
            },
        ]));
    }

    // Non-parallel axes must intersect (be coplanar) for the equal-radius
    // ellipse-pair degeneracy; skew axes give an irreducible quartic.
    let cross = a1.cross(&a2);
    let gap = (o2 - o1).dot(&cross) / cross.norm();
    if !tol.approx_zero(gap) {
        return Err(CoreError::NotImplemented {
            feature: "cylinder-cylinder intersection with skew axes",
        });
    }

    // Point where the axes cross.
    let t1 = (o2 - o1).cross(&a2).dot(&cross) / cross.norm_squared();
    let hub = o1 + a1 * t1;

    // Equal-radius cylinders with intersecting axes degenerate into two
    // ellipses, one in each plane bisecting the axes. Both have semi-minor
    // `radius` along the common perpendicular; the semi-major along the
    // bisector grows with 1/sin of the half-angle it makes with each axis.
    let bis_sum = (a1 + a2).normalize(); // bisector of the axes
    let bis_diff = (a1 - a2).normalize(); // bisector of a1 and -a2
    // cos(θ/2) and sin(θ/2) for the angle θ between the axes.
    let cos_half = bis_sum.dot(&a1);
    let sin_half = bis_diff.dot(&a1);

    Ok(SurfaceIntersection::transversal(vec![
        Curve3::Ellipse {
            center: hub,
            axis: bis_diff,
            major_dir: bis_sum,
            major_radius: radius / sin_half,
            minor_radius: radius,
        },
        Curve3::Ellipse {
            center: hub,
            axis: bis_sum,
            major_dir: bis_diff,
            major_radius: radius / cos_half,
            minor_radius: radius,
        },
    ]))
}

/// Midpoint of two points (avoids pulling nalgebra into scope here).
fn na_center(a: &Point3, b: &Point3) -> Point3 {
    Point3::from((a.coords + b.coords) / 2.0)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::CurveEval;
    use opensolid_core::types::Vector3;
    use std::f64::consts::{FRAC_PI_6, PI, SQRT_2};

    const EPS: f64 = 1e-9;

    fn tol() -> ToleranceContext {
        ToleranceContext::default()
    }

    /// Geometric residual of `p` against the surface locus: ~0 iff on it.
    fn residual(s: &Surface3, p: &Point3) -> f64 {
        match s {
            Surface3::Plane { origin, normal } => normal.dot(&(p - origin)).abs(),
            &Surface3::Sphere { center, radius, .. } => ((p - center).norm() - radius).abs(),
            &Surface3::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let d = p - origin;
                ((d - axis * axis.dot(&d)).norm() - radius).abs()
            }
            &Surface3::Cone {
                origin,
                axis,
                half_angle,
                radius,
            } => {
                let d = p - origin;
                let v = axis.dot(&d);
                let rho = (d - axis * v).norm();
                (rho - (radius + v * half_angle.tan()).abs()).abs()
            }
            &Surface3::Torus {
                center,
                axis,
                major_radius,
                minor_radius,
            } => {
                let d = p - center;
                let h = axis.dot(&d);
                let rho = (d - axis * h).norm();
                ((rho - major_radius).hypot(h) - minor_radius).abs()
            }
        }
    }

    /// Sample the curve and assert every sample lies on both surfaces.
    fn assert_curve_on_both(curve: &Curve3, a: &Surface3, b: &Surface3) {
        let params: Vec<f64> = match curve {
            Curve3::Line { .. } => (-4..=4).map(|i| f64::from(i) * 0.75).collect(),
            _ => (0..16).map(|i| f64::from(i) * PI / 8.0).collect(),
        };
        for t in params {
            let p = curve.point(t);
            let (ra, rb) = (residual(a, &p), residual(b, &p));
            assert!(ra < EPS, "curve point {p:?} off first surface by {ra:e}");
            assert!(rb < EPS, "curve point {p:?} off second surface by {rb:e}");
        }
    }

    /// Unwrap the `Curves` case, checking count and sampling every curve
    /// against both surfaces.
    fn expect_curves(a: &Surface3, b: &Surface3, count: usize) -> Vec<IntersectionCurve> {
        let result = intersect(a, b, &tol()).expect("supported configuration");
        let SurfaceIntersection::Curves(curves) = result else {
            panic!("expected Curves, got {result:?}");
        };
        assert_eq!(curves.len(), count, "curve count");
        for ic in &curves {
            assert_curve_on_both(&ic.curve, a, b);
        }
        curves
    }

    fn plane_z(z: f64) -> Surface3 {
        Surface3::Plane {
            origin: Point3::new(0.0, 0.0, z),
            normal: Vector3::z(),
        }
    }

    fn plane_x(x: f64) -> Surface3 {
        Surface3::Plane {
            origin: Point3::new(x, 0.0, 0.0),
            normal: Vector3::x(),
        }
    }

    fn cylinder_z(radius: f64) -> Surface3 {
        Surface3::Cylinder {
            origin: Point3::origin(),
            axis: Vector3::z(),
            radius,
        }
    }

    fn unit_sphere_at_origin(radius: f64) -> Surface3 {
        Surface3::Sphere {
            center: Point3::origin(),
            axis: Vector3::z(),
            radius,
        }
    }

    // ── plane-plane ────────────────────────────────────────────────────

    #[test]
    fn plane_plane_transversal_line() {
        let a = plane_z(1.0);
        let b = plane_x(2.0);
        let curves = expect_curves(&a, &b, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
        let Curve3::Line { origin, dir } = &curves[0].curve else {
            panic!("expected a line, got {:?}", curves[0].curve);
        };
        assert!(dir.cross(&Vector3::y()).norm() < EPS, "dir should be ±Y");
        assert!((origin.x - 2.0).abs() < EPS && (origin.z - 1.0).abs() < EPS);
    }

    #[test]
    fn plane_plane_generic_oblique() {
        let a = Surface3::Plane {
            origin: Point3::new(1.0, -2.0, 0.5),
            normal: Vector3::new(1.0, 2.0, -1.0).normalize(),
        };
        let b = Surface3::Plane {
            origin: Point3::new(-3.0, 0.7, 2.0),
            normal: Vector3::new(0.3, -1.0, 4.0).normalize(),
        };
        let curves = expect_curves(&a, &b, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
    }

    #[test]
    fn plane_plane_parallel_empty() {
        let result = intersect(&plane_z(0.0), &plane_z(1.0), &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
    }

    #[test]
    fn plane_plane_coincident() {
        // Same locus: origin shifted in-plane, normal flipped.
        let a = plane_z(1.0);
        let b = Surface3::Plane {
            origin: Point3::new(5.0, -3.0, 1.0),
            normal: -Vector3::z(),
        };
        let result = intersect(&a, &b, &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Coincident);
    }

    // ── plane-sphere ───────────────────────────────────────────────────

    #[test]
    fn plane_sphere_circle() {
        let plane = plane_z(1.0);
        let sphere = unit_sphere_at_origin(2.0);
        let curves = expect_curves(&plane, &sphere, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
        let Curve3::Circle {
            center,
            axis,
            radius,
        } = &curves[0].curve
        else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!((center - Point3::new(0.0, 0.0, 1.0)).norm() < EPS);
        assert!(axis.cross(&Vector3::z()).norm() < EPS);
        assert!((radius - 3.0_f64.sqrt()).abs() < EPS);
    }

    #[test]
    fn plane_sphere_tangent_is_point_not_curve() {
        let plane = plane_z(2.0);
        let sphere = unit_sphere_at_origin(2.0);
        let expected = SurfaceIntersection::TangentPoint(Point3::new(0.0, 0.0, 2.0));
        assert_eq!(intersect(&plane, &sphere, &tol()).unwrap(), expected);
        // Swapped argument order dispatches identically.
        assert_eq!(intersect(&sphere, &plane, &tol()).unwrap(), expected);
    }

    #[test]
    fn plane_sphere_empty() {
        let result = intersect(&plane_z(3.0), &unit_sphere_at_origin(2.0), &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
    }

    // ── plane-cylinder ─────────────────────────────────────────────────

    #[test]
    fn plane_cylinder_perpendicular_circle() {
        let plane = plane_z(5.0);
        let cyl = cylinder_z(1.5);
        let curves = expect_curves(&plane, &cyl, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
        let Curve3::Circle { center, radius, .. } = &curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!((center - Point3::new(0.0, 0.0, 5.0)).norm() < EPS);
        assert!((radius - 1.5).abs() < EPS);
    }

    #[test]
    fn plane_cylinder_parallel_two_lines() {
        let plane = plane_x(0.5);
        let cyl = cylinder_z(1.0);
        let curves = expect_curves(&plane, &cyl, 2);
        for ic in &curves {
            assert_eq!(ic.kind, IntersectionKind::Transversal);
            let Curve3::Line { dir, .. } = &ic.curve else {
                panic!("expected lines, got {:?}", ic.curve);
            };
            assert!(
                dir.cross(&Vector3::z()).norm() < EPS,
                "lines run along the axis"
            );
        }
    }

    #[test]
    fn plane_cylinder_parallel_tangent_line() {
        let curves = expect_curves(&plane_x(1.0), &cylinder_z(1.0), 1);
        assert_eq!(curves[0].kind, IntersectionKind::Tangential);
        let Curve3::Line { origin, .. } = &curves[0].curve else {
            panic!("expected a line, got {:?}", curves[0].curve);
        };
        assert!((origin.x - 1.0).abs() < EPS && origin.y.abs() < EPS);
    }

    #[test]
    fn plane_cylinder_parallel_empty() {
        let result = intersect(&plane_x(2.0), &cylinder_z(1.0), &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
    }

    #[test]
    fn plane_cylinder_oblique_ellipse() {
        // Plane at 45° to the axis: major radius r·√2, minor r.
        let plane = Surface3::Plane {
            origin: Point3::origin(),
            normal: Vector3::new(0.0, 1.0, 1.0).normalize(),
        };
        let cyl = cylinder_z(1.0);
        let curves = expect_curves(&plane, &cyl, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
        let Curve3::Ellipse {
            center,
            major_radius,
            minor_radius,
            ..
        } = &curves[0].curve
        else {
            panic!("expected an ellipse, got {:?}", curves[0].curve);
        };
        assert!(center.coords.norm() < EPS);
        assert!((major_radius - SQRT_2).abs() < EPS);
        assert!((minor_radius - 1.0).abs() < EPS);
    }

    // ── plane-cone ─────────────────────────────────────────────────────

    /// Cone about +Z, half-angle 30°, radius 1 at the origin plane; apex at
    /// (0, 0, −√3).
    fn cone_30deg() -> Surface3 {
        Surface3::Cone {
            origin: Point3::origin(),
            axis: Vector3::z(),
            half_angle: FRAC_PI_6,
            radius: 1.0,
        }
    }

    #[test]
    fn plane_cone_perpendicular_circle() {
        let curves = expect_curves(&plane_z(2.0), &cone_30deg(), 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
        let Curve3::Circle { center, radius, .. } = &curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!((center - Point3::new(0.0, 0.0, 2.0)).norm() < EPS);
        assert!((radius - (1.0 + 2.0 * FRAC_PI_6.tan())).abs() < EPS);
    }

    #[test]
    fn plane_cone_perpendicular_through_apex() {
        let apex_z = -3.0_f64.sqrt();
        let result = intersect(&plane_z(apex_z), &cone_30deg(), &tol()).unwrap();
        let SurfaceIntersection::TangentPoint(p) = result else {
            panic!("expected the apex as a tangent point, got {result:?}");
        };
        assert!((p - Point3::new(0.0, 0.0, apex_z)).norm() < EPS);
    }

    #[test]
    fn plane_cone_oblique_ellipse() {
        // Normal 20° off the axis: 20° + 30° < 90°, so an ellipse.
        let tilt = 20.0_f64.to_radians();
        let plane = Surface3::Plane {
            origin: Point3::new(0.0, 0.0, 1.0),
            normal: Vector3::new(tilt.sin(), 0.0, tilt.cos()),
        };
        let curves = expect_curves(&plane, &cone_30deg(), 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
        assert!(matches!(curves[0].curve, Curve3::Ellipse { .. }));
    }

    #[test]
    fn plane_cone_steep_plane_through_apex() {
        let tilt = 20.0_f64.to_radians();
        let apex = Point3::new(0.0, 0.0, -(3.0_f64.sqrt()));
        let plane = Surface3::Plane {
            origin: apex,
            normal: Vector3::new(tilt.sin(), 0.0, tilt.cos()),
        };
        let result = intersect(&plane, &cone_30deg(), &tol()).unwrap();
        let SurfaceIntersection::TangentPoint(p) = result else {
            panic!("expected the apex as a tangent point, got {result:?}");
        };
        assert!((p - apex).norm() < EPS);
    }

    #[test]
    fn plane_cone_axis_parallel_plane_not_implemented() {
        // Normal ⟂ axis gives a hyperbola (or generator pair) — out of MVP.
        let result = intersect(&plane_x(0.5), &cone_30deg(), &tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    // ── plane-torus ────────────────────────────────────────────────────

    fn torus_z(major: f64, minor: f64) -> Surface3 {
        Surface3::Torus {
            center: Point3::origin(),
            axis: Vector3::z(),
            major_radius: major,
            minor_radius: minor,
        }
    }

    #[test]
    fn plane_torus_perpendicular_two_circles() {
        let curves = expect_curves(&plane_z(0.5), &torus_z(3.0, 1.0), 2);
        let mut radii: Vec<f64> = curves
            .iter()
            .map(|ic| {
                assert_eq!(ic.kind, IntersectionKind::Transversal);
                let Curve3::Circle { radius, .. } = ic.curve else {
                    panic!("expected circles, got {:?}", ic.curve);
                };
                radius
            })
            .collect();
        radii.sort_by(f64::total_cmp);
        let half_width = 0.75_f64.sqrt();
        assert!((radii[0] - (3.0 - half_width)).abs() < EPS);
        assert!((radii[1] - (3.0 + half_width)).abs() < EPS);
    }

    #[test]
    fn plane_torus_perpendicular_tangent_circle() {
        let curves = expect_curves(&plane_z(1.0), &torus_z(3.0, 1.0), 1);
        assert_eq!(curves[0].kind, IntersectionKind::Tangential);
        let Curve3::Circle { radius, .. } = curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!((radius - 3.0).abs() < EPS);
    }

    #[test]
    fn plane_torus_perpendicular_empty() {
        let result = intersect(&plane_z(1.5), &torus_z(3.0, 1.0), &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
    }

    #[test]
    fn plane_torus_axial_plane_two_tube_circles() {
        let curves = expect_curves(&plane_x(0.0), &torus_z(3.0, 1.0), 2);
        let mut centers_y: Vec<f64> = curves
            .iter()
            .map(|ic| {
                assert_eq!(ic.kind, IntersectionKind::Transversal);
                let Curve3::Circle { center, radius, .. } = ic.curve else {
                    panic!("expected circles, got {:?}", ic.curve);
                };
                assert!((radius - 1.0).abs() < EPS);
                center.y
            })
            .collect();
        centers_y.sort_by(f64::total_cmp);
        assert!((centers_y[0] + 3.0).abs() < EPS);
        assert!((centers_y[1] - 3.0).abs() < EPS);
    }

    #[test]
    fn plane_torus_oblique_not_implemented() {
        let plane = Surface3::Plane {
            origin: Point3::origin(),
            normal: Vector3::new(0.0, 1.0, 1.0).normalize(),
        };
        let result = intersect(&plane, &torus_z(3.0, 1.0), &tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    // ── cylinder-cylinder ──────────────────────────────────────────────

    fn cylinder(origin: Point3, axis: Vector3, radius: f64) -> Surface3 {
        Surface3::Cylinder {
            origin,
            axis: axis.normalize(),
            radius,
        }
    }

    #[test]
    fn cylinders_parallel_two_lines() {
        let a = cylinder_z(1.0);
        let b = cylinder(Point3::new(1.0, 0.0, 0.0), Vector3::z(), 1.0);
        let curves = expect_curves(&a, &b, 2);
        for ic in &curves {
            assert_eq!(ic.kind, IntersectionKind::Transversal);
            assert!(matches!(ic.curve, Curve3::Line { .. }));
        }
    }

    #[test]
    fn cylinders_parallel_tangent_line() {
        let a = cylinder_z(1.0);
        let b = cylinder(Point3::new(2.0, 0.0, 0.0), Vector3::z(), 1.0);
        let curves = expect_curves(&a, &b, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Tangential);
        let Curve3::Line { origin, .. } = &curves[0].curve else {
            panic!("expected a line, got {:?}", curves[0].curve);
        };
        assert!((origin.x - 1.0).abs() < EPS && origin.y.abs() < EPS);
    }

    #[test]
    fn cylinders_parallel_empty() {
        let a = cylinder_z(1.0);
        let b = cylinder(Point3::new(3.0, 0.0, 0.0), Vector3::z(), 1.0);
        assert_eq!(
            intersect(&a, &b, &tol()).unwrap(),
            SurfaceIntersection::Empty
        );
    }

    #[test]
    fn cylinders_coincident() {
        let a = cylinder_z(1.0);
        // Same axis line, origin slid along it.
        let b = cylinder(Point3::new(0.0, 0.0, 7.0), Vector3::z(), 1.0);
        assert_eq!(
            intersect(&a, &b, &tol()).unwrap(),
            SurfaceIntersection::Coincident
        );
    }

    #[test]
    fn cylinders_perpendicular_equal_radius_ellipse_pair() {
        // The classic Steinmetz degeneracy: both ellipses have major r·√2.
        let a = cylinder_z(1.0);
        let b = cylinder(Point3::origin(), Vector3::x(), 1.0);
        let curves = expect_curves(&a, &b, 2);
        for ic in &curves {
            assert_eq!(ic.kind, IntersectionKind::Transversal);
            let Curve3::Ellipse {
                center,
                major_radius,
                minor_radius,
                ..
            } = &ic.curve
            else {
                panic!("expected ellipses, got {:?}", ic.curve);
            };
            assert!(center.coords.norm() < EPS);
            assert!((major_radius - SQRT_2).abs() < EPS);
            assert!((minor_radius - 1.0).abs() < EPS);
        }
    }

    #[test]
    fn cylinders_oblique_equal_radius_ellipse_pair() {
        // Axes at 40° crossing off the origin.
        let hub = Point3::new(2.0, 1.0, -0.5);
        let theta = 40.0_f64.to_radians();
        let a = cylinder(hub, Vector3::z(), 0.8);
        let b = cylinder(hub, Vector3::new(theta.sin(), 0.0, theta.cos()), 0.8);
        let curves = expect_curves(&a, &b, 2);
        for ic in &curves {
            assert_eq!(ic.kind, IntersectionKind::Transversal);
            let Curve3::Ellipse { center, .. } = &ic.curve else {
                panic!("expected ellipses, got {:?}", ic.curve);
            };
            assert!((center - hub).norm() < EPS, "ellipses centered at the hub");
        }
    }

    #[test]
    fn cylinders_skew_not_implemented() {
        let a = cylinder_z(1.0);
        let b = cylinder(Point3::new(0.0, 5.0, 0.0), Vector3::x(), 1.0);
        let result = intersect(&a, &b, &tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    #[test]
    fn cylinders_unequal_radii_not_implemented() {
        let a = cylinder_z(1.0);
        let b = cylinder(Point3::origin(), Vector3::x(), 0.5);
        let result = intersect(&a, &b, &tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    // ── dispatch ───────────────────────────────────────────────────────

    #[test]
    fn unsupported_pair_not_implemented() {
        let a = unit_sphere_at_origin(1.0);
        let b = unit_sphere_at_origin(2.0);
        let result = intersect(&a, &b, &tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    #[test]
    fn dispatch_is_symmetric_for_swapped_arguments() {
        let plane = plane_z(5.0);
        let cyl = cylinder_z(1.5);
        assert_eq!(
            intersect(&plane, &cyl, &tol()).unwrap(),
            intersect(&cyl, &plane, &tol()).unwrap()
        );
    }
}
