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
//! circles, ellipses), the equal-radius cylinder-cylinder special
//! cases (parallel axes → line pair, intersecting axes → ellipse pair),
//! sphere-sphere in full, and every *coaxial* sphere/torus arrangement —
//! cylinder-sphere with the center on the axis, sphere-torus with the
//! center on the torus axis, coaxial cylinder-torus and torus-torus — via
//! their shared meridian profile (circles of latitude about the common
//! axis). Coaxial cone-cone joins them: two cones sharing an axis line meet
//! in a single latitude circle where their profile rays cross (or a shared
//! apex point, or coincide). Configurations whose intersection needs curves
//! we cannot represent yet (plane-cone parabolas/hyperbolas, non-coaxial
//! cone-cone quartics, oblique plane-torus, skew or unequal-radius cylinder
//! pairs, off-axis sphere/torus quartics) return
//! [`CoreError::NotImplemented`] — never a misleading `Empty`.
//! The sphere/torus pairs' general positions march instead: see
//! [`super::intersect_marched`].
//!
//! All classification comparisons (parallelism, tangency, coincidence) go
//! through the caller's [`ToleranceContext`], per the kernel tolerance
//! model.

use crate::curve::Curve3;
use crate::surface::Surface3;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::tolerance::{ANGULAR_RESOLUTION, ToleranceContext};
use opensolid_core::types::{Point3, Vector3};

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
        (Sphere { .. }, Sphere { .. }) => Ok(sphere_sphere(a, b, tol)),
        (Cylinder { .. }, Sphere { .. }) => cylinder_sphere(a, b, tol),
        (Sphere { .. }, Cylinder { .. }) => cylinder_sphere(b, a, tol),
        (Sphere { .. }, Torus { .. }) => sphere_torus(a, b, tol),
        (Torus { .. }, Sphere { .. }) => sphere_torus(b, a, tol),
        (Cylinder { .. }, Torus { .. }) => cylinder_torus(a, b, tol),
        (Torus { .. }, Cylinder { .. }) => cylinder_torus(b, a, tol),
        (Torus { .. }, Torus { .. }) => torus_torus(a, b, tol),
        (Cone { .. }, Cone { .. }) => cone_cone(a, b, tol),
        _ => Err(CoreError::NotImplemented {
            feature: "analytic SSI for cone pairs other than plane-cone \
                      and coaxial cone-cone",
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
    // as Curve3, so this stays `NotImplemented`; the boolean pipeline marches
    // these unbounded sections instead (`ssi::intersect_marched_bounded`).
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
                  axis-containing planes (general sections are quartics; \
                  use ssi::intersect_marched)",
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

/// A point of the meridian half-plane shared by coaxial surfaces of
/// revolution: `rho` is the distance from the axis, `h` the height along
/// it (both from a common reference origin on the axis).
#[derive(Debug, Clone, Copy)]
struct Profile {
    rho: f64,
    h: f64,
}

/// Sweep the intersection of two meridian profile circles about the common
/// axis (through `origin`, unit direction `axis`).
///
/// Coaxial surfaces of revolution meet where their profiles meet, and each
/// profile crossing sweeps to a full circle of latitude. A profile
/// tangency means the surface normals (the profile normals, rotated about
/// the axis) agree along the swept circle — a tangential curve — or a
/// single tangent point when the contact lies on the axis itself (sphere
/// profiles). Identical profile circles are the coincident surface pair.
///
/// A profile crossing mirrored to `rho < 0` is the same 3D locus as its
/// `rho > 0` sibling (the half-plane sweeps through the full plane), so
/// only positive-`rho` crossings emit circles.
fn coaxial_profiles(
    origin: &Point3,
    axis: &Vector3,
    c1: Profile,
    r1: f64,
    c2: Profile,
    r2: f64,
    tol: &ToleranceContext,
) -> SurfaceIntersection {
    let (drho, dh) = (c2.rho - c1.rho, c2.h - c1.h);
    let d = drho.hypot(dh);
    if tol.approx_zero(d) {
        return if tol.approx_eq(r1, r2) {
            SurfaceIntersection::Coincident
        } else {
            SurfaceIntersection::Empty
        };
    }
    // Unit direction between the profile centers and its in-plane normal;
    // `a` is the (signed) distance from c1 to the radical line along `e`.
    let e = (drho / d, dh / d);
    let n = (-e.1, e.0);
    let a = (d * d + r1 * r1 - r2 * r2) / (2.0 * d);

    if tol.approx_eq(d, r1 + r2) || tol.approx_eq(d, (r1 - r2).abs()) {
        let p = Profile {
            rho: c1.rho + e.0 * a,
            h: c1.h + e.1 * a,
        };
        return if tol.approx_zero(p.rho) {
            SurfaceIntersection::TangentPoint(origin + axis * p.h)
        } else {
            SurfaceIntersection::tangential(Curve3::Circle {
                center: origin + axis * p.h,
                axis: *axis,
                radius: p.rho,
            })
        };
    }
    if d > r1 + r2 || d < (r1 - r2).abs() {
        return SurfaceIntersection::Empty;
    }

    let half = (r1 * r1 - a * a).sqrt();
    let circles = [1.0, -1.0]
        .into_iter()
        .filter_map(|s| {
            let rho = c1.rho + e.0 * a + n.0 * half * s;
            let h = c1.h + e.1 * a + n.1 * half * s;
            (rho > 0.0).then_some(Curve3::Circle {
                center: origin + axis * h,
                axis: *axis,
                radius: rho,
            })
        })
        .collect();
    SurfaceIntersection::transversal(circles)
}

fn sphere_sphere(a: &Surface3, b: &Surface3, tol: &ToleranceContext) -> SurfaceIntersection {
    let (
        &Surface3::Sphere {
            center: c1,
            radius: r1,
            ..
        },
        &Surface3::Sphere {
            center: c2,
            radius: r2,
            ..
        },
    ) = (a, b)
    else {
        unreachable!("dispatched on Sphere/Sphere")
    };

    let offset = c2 - c1;
    let d = offset.norm();
    if tol.approx_zero(d) {
        // Concentric: the same locus or nested with no contact.
        return if tol.approx_eq(r1, r2) {
            SurfaceIntersection::Coincident
        } else {
            SurfaceIntersection::Empty
        };
    }
    // Both profiles sit on the center line: crossings mirror to a single
    // circle, tangencies land on the axis as tangent points.
    coaxial_profiles(
        &c1,
        &(offset / d),
        Profile { rho: 0.0, h: 0.0 },
        r1,
        Profile { rho: 0.0, h: d },
        r2,
        tol,
    )
}

fn cylinder_sphere(
    cylinder: &Surface3,
    sphere: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    let (
        &Surface3::Cylinder {
            origin,
            axis,
            radius: r,
        },
        &Surface3::Sphere {
            center,
            radius: big_r,
            ..
        },
    ) = (cylinder, sphere)
    else {
        unreachable!("dispatched on Cylinder/Sphere")
    };

    let m = center - origin;
    let foot = origin + axis * axis.dot(&m);
    let perp = center - foot;
    let d = perp.norm();

    if tol.approx_zero(d) {
        // Center on the axis: latitude circles of the cylinder.
        return Ok(if tol.approx_eq(big_r, r) {
            // Inscribed sphere: touches along its equator.
            SurfaceIntersection::tangential(Curve3::Circle {
                center: foot,
                axis,
                radius: r,
            })
        } else if big_r < r {
            SurfaceIntersection::Empty
        } else {
            let h = (big_r * big_r - r * r).sqrt();
            SurfaceIntersection::transversal(vec![
                Curve3::Circle {
                    center: foot + axis * h,
                    axis,
                    radius: r,
                },
                Curve3::Circle {
                    center: foot - axis * h,
                    axis,
                    radius: r,
                },
            ])
        });
    }

    // Off-axis center: the closest wall point to the center (the only
    // possible isolated contact) sits at radius r toward it.
    let near = foot + (perp / d) * r;
    if tol.approx_eq(d, r + big_r) {
        // Sphere touches the near wall from outside.
        return Ok(SurfaceIntersection::TangentPoint(near));
    }
    if d > r + big_r {
        return Ok(SurfaceIntersection::Empty);
    }
    if r >= big_r {
        if tol.approx_eq(d, r - big_r) {
            // Sphere touches the near wall from inside the cylinder.
            return Ok(SurfaceIntersection::TangentPoint(near));
        }
        if d < r - big_r {
            // Sphere floats inside the cylinder without contact.
            return Ok(SurfaceIntersection::Empty);
        }
    } else if tol.approx_eq(d, big_r - r) {
        // The far wall is tangent from inside the sphere while the near
        // wall is crossed transversally (Viviani-type singular quartic):
        // the curve has a self-intersection at the tangent point.
        return Err(CoreError::NotImplemented {
            feature: "cylinder-sphere intersection with far-wall tangential \
                      contact (Viviani-type singular quartic)",
        });
    }
    Err(CoreError::NotImplemented {
        feature: "cylinder-sphere general quartic intersection (no conic \
                  closed form; use ssi::intersect_marched)",
    })
}

fn sphere_torus(
    sphere: &Surface3,
    torus: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    let (
        &Surface3::Sphere {
            center: sc,
            radius: sr,
            ..
        },
        &Surface3::Torus {
            center,
            axis,
            major_radius,
            minor_radius,
        },
    ) = (sphere, torus)
    else {
        unreachable!("dispatched on Sphere/Torus")
    };

    let m = sc - center;
    let h = axis.dot(&m);
    if !tol.vector_approx_zero(&(m - axis * h)) {
        return Err(CoreError::NotImplemented {
            feature: "sphere-torus intersection with the center off the torus \
                      axis (general quartic; use ssi::intersect_marched)",
        });
    }
    // Shared meridian: the torus tube circle against the sphere profile.
    Ok(coaxial_profiles(
        &center,
        &axis,
        Profile {
            rho: major_radius,
            h: 0.0,
        },
        minor_radius,
        Profile { rho: 0.0, h },
        sr,
        tol,
    ))
}

fn cylinder_torus(
    cylinder: &Surface3,
    torus: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    let (
        &Surface3::Cylinder {
            origin,
            axis: cyl_axis,
            radius: r_c,
        },
        &Surface3::Torus {
            center,
            axis,
            major_radius,
            minor_radius,
        },
    ) = (cylinder, torus)
    else {
        unreachable!("dispatched on Cylinder/Torus")
    };

    let m = center - origin;
    let coaxial = tol.vectors_parallel(&cyl_axis, &axis)
        && tol.vector_approx_zero(&(m - cyl_axis * cyl_axis.dot(&m)));
    if !coaxial {
        return Err(CoreError::NotImplemented {
            feature: "cylinder-torus intersection with non-coaxial axes \
                      (degree-8 curve in general; use ssi::intersect_marched)",
        });
    }

    // Shared meridian: the vertical line rho = r_c against the tube circle.
    let gap = r_c - major_radius;
    Ok(if tol.approx_eq(gap.abs(), minor_radius) {
        // r_c = R ± r: tangent along the outer/inner equator.
        SurfaceIntersection::tangential(Curve3::Circle {
            center,
            axis,
            radius: r_c,
        })
    } else if gap.abs() > minor_radius {
        SurfaceIntersection::Empty
    } else {
        let h = (minor_radius * minor_radius - gap * gap).sqrt();
        SurfaceIntersection::transversal(vec![
            Curve3::Circle {
                center: center + axis * h,
                axis,
                radius: r_c,
            },
            Curve3::Circle {
                center: center - axis * h,
                axis,
                radius: r_c,
            },
        ])
    })
}

fn torus_torus(
    a: &Surface3,
    b: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    let (
        &Surface3::Torus {
            center: c1,
            axis: a1,
            major_radius: big_r1,
            minor_radius: r1,
        },
        &Surface3::Torus {
            center: c2,
            axis: a2,
            major_radius: big_r2,
            minor_radius: r2,
        },
    ) = (a, b)
    else {
        unreachable!("dispatched on Torus/Torus")
    };

    let m = c2 - c1;
    let z0 = a1.dot(&m);
    let coaxial = tol.vectors_parallel(&a1, &a2) && tol.vector_approx_zero(&(m - a1 * z0));
    if !coaxial {
        return Err(CoreError::NotImplemented {
            feature: "torus-torus intersection with non-coaxial axes \
                      (degree-8 curve in general; use ssi::intersect_marched)",
        });
    }

    // Shared meridian: tube circle against tube circle. A torus is
    // symmetric under flipping its axis, so an antiparallel a2 changes
    // nothing about the profile.
    Ok(coaxial_profiles(
        &c1,
        &a1,
        Profile {
            rho: big_r1,
            h: 0.0,
        },
        r1,
        Profile { rho: big_r2, h: z0 },
        r2,
        tol,
    ))
}

/// Exact intersection of two cones sharing a common axis line.
///
/// In the meridian half-plane (distance `rho` from the axis, height `h`
/// along it from cone 1's apex) each single-nappe cone is a ray from its
/// apex, widening at its half-angle: `rho1(h) = tan α1 · h` for `h ≥ 0`, and
/// `rho2(h) = tan α2 · dir2 · (h − z2)` valid on `dir2·(h − z2) ≥ 0`, where
/// `dir2 = ±1` is cone 2's widening sense along the common axis and `z2` its
/// apex height. Two distinct rays cross at most once, so a coaxial pair meets
/// in at most a single circle of latitude — never the two circles a full
/// double-cone quadric would give, since [`ray_surface_hits`](crate::boolean)
/// fixes the physical cone as the single nappe. The crossing is always
/// transversal (equal profile slopes are the parallel/coincident case, not a
/// tangency); a crossing that lands on the axis is the cones' shared apex, a
/// degenerate [`SurfaceIntersection::TangentPoint`].
///
/// Non-coaxial cone pairs (skew or parallel-but-offset axes) are quartics
/// with no [`Curve3`] closed form and return [`CoreError::NotImplemented`];
/// the boolean pipeline marches those (`ssi::intersect_marched_bounded`).
fn cone_cone(
    a: &Surface3,
    b: &Surface3,
    tol: &ToleranceContext,
) -> CoreResult<SurfaceIntersection> {
    let (
        &Surface3::Cone {
            origin: o1,
            axis: a1,
            half_angle: alpha1,
            radius: r1,
        },
        &Surface3::Cone {
            origin: o2,
            axis: a2,
            half_angle: alpha2,
            radius: r2,
        },
    ) = (a, b)
    else {
        unreachable!("dispatched on Cone/Cone")
    };

    let apex1 = o1 - a1 * (r1 / alpha1.tan());
    let apex2 = o2 - a2 * (r2 / alpha2.tan());

    // Coaxial requires collinear axes: parallel directions and cone 2's apex
    // on cone 1's axis line. Everything else is the general quartic.
    let axis = a1;
    let m = apex2 - apex1;
    let z2 = axis.dot(&m);
    if !tol.vectors_parallel(&a1, &a2) || !tol.vector_approx_zero(&(m - axis * z2)) {
        return Err(CoreError::NotImplemented {
            feature: "non-coaxial cone-cone intersection (general quartic; \
                      use ssi::intersect_marched_bounded)",
        });
    }

    let (t1, t2) = (alpha1.tan(), alpha2.tan());
    let dir2 = if axis.dot(&a2) >= 0.0 { 1.0 } else { -1.0 };

    // Solve rho1(h) = rho2(h):  h·(t1 − t2·dir2) = −t2·dir2·z2.
    let denom = t1 - t2 * dir2;
    if tol.approx_zero(denom) {
        // Equal half-angle and widening sense: parallel profile rays. They
        // coincide only when the apexes do, otherwise never meet.
        return Ok(if tol.approx_zero(z2) {
            SurfaceIntersection::Coincident
        } else {
            SurfaceIntersection::Empty
        });
    }
    let h = -t2 * dir2 * z2 / denom;
    let rho = t1 * h;

    // The crossing must sit on both physical (single-nappe) rays.
    if h < -tol.linear || dir2 * (h - z2) < -tol.linear {
        return Ok(SurfaceIntersection::Empty);
    }

    let center = apex1 + axis * h;
    Ok(if tol.approx_zero(rho) {
        // On the axis: the cones touch only at their shared apex.
        SurfaceIntersection::TangentPoint(center)
    } else {
        SurfaceIntersection::transversal(vec![Curve3::Circle {
            center,
            axis,
            radius: rho,
        }])
    })
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

    // ── cone-cone (coaxial) ─────────────────────────────────────────────

    /// `Surface3::Cone` from apex-agnostic fields, half-angle from a slope.
    fn cone(origin: Point3, axis: Vector3, tan_half: f64, radius: f64) -> Surface3 {
        Surface3::Cone {
            origin,
            axis: axis.normalize(),
            half_angle: tan_half.atan(),
            radius,
        }
    }

    #[test]
    fn cone_cone_opposed_single_circle() {
        // A: r=2 at z=0, apex at z=3 (widens toward −z, slope 2/3).
        // B: apex at z=1, r=2 at z=4 (widens toward +z, slope 2/3).
        // Walls cross at z=2, ρ=2/3 — one transversal latitude circle.
        let a = cone(Point3::new(0.0, 0.0, 0.0), -Vector3::z(), 2.0 / 3.0, 2.0);
        let b = cone(Point3::new(0.0, 0.0, 1.0), Vector3::z(), 2.0 / 3.0, 0.0);
        let curves = expect_curves(&a, &b, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
        let Curve3::Circle { center, radius, .. } = &curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!((center - Point3::new(0.0, 0.0, 2.0)).norm() < EPS);
        assert!((radius - 2.0 / 3.0).abs() < EPS);
    }

    #[test]
    fn cone_cone_coincident() {
        // Two frames on the same infinite cone (same apex/axis/half-angle).
        let a = cone(Point3::new(0.0, 0.0, 0.0), -Vector3::z(), 2.0 / 3.0, 2.0);
        let b = cone(
            Point3::new(0.0, 0.0, 1.0),
            -Vector3::z(),
            2.0 / 3.0,
            4.0 / 3.0,
        );
        assert_eq!(
            intersect(&a, &b, &tol()).unwrap(),
            SurfaceIntersection::Coincident
        );
    }

    #[test]
    fn cone_cone_shared_apex_tangent_point() {
        // Nested cones sharing an apex and axis, different half-angles: they
        // touch only at the apex.
        let a = cone(Point3::origin(), Vector3::z(), FRAC_PI_6.tan(), 0.0);
        let b = cone(Point3::origin(), Vector3::z(), 0.5, 0.0);
        let SurfaceIntersection::TangentPoint(p) = intersect(&a, &b, &tol()).unwrap() else {
            panic!("expected the shared apex as a tangent point");
        };
        assert!((p - Point3::origin()).norm() < EPS);
    }

    #[test]
    fn cone_cone_parallel_walls_empty() {
        // Equal half-angle and widening sense, apexes offset along the axis:
        // parallel profile rays that never meet.
        let a = cone(Point3::new(0.0, 0.0, 0.0), Vector3::z(), 0.5, 0.0);
        let b = cone(Point3::new(0.0, 0.0, 1.0), Vector3::z(), 0.5, 0.0);
        assert_eq!(
            intersect(&a, &b, &tol()).unwrap(),
            SurfaceIntersection::Empty
        );
    }

    #[test]
    fn cone_cone_apexes_pointing_apart_empty() {
        // Nearby apexes widening away from each other: the wall crossing lands
        // on neither physical (single) nappe.
        let a = cone(Point3::new(0.0, 0.0, 0.0), Vector3::z(), 0.5, 0.0);
        let b = cone(Point3::new(0.0, 0.0, -1.0), -Vector3::z(), 0.5, 0.0);
        assert_eq!(
            intersect(&a, &b, &tol()).unwrap(),
            SurfaceIntersection::Empty
        );
    }

    #[test]
    fn cone_cone_non_coaxial_not_implemented() {
        // Skew axes: the general quartic, marched by the boolean pipeline.
        let a = cone(Point3::origin(), Vector3::z(), 0.5, 1.0);
        let b = cone(Point3::new(0.0, 0.0, 1.0), Vector3::x(), 0.5, 1.0);
        assert!(matches!(
            intersect(&a, &b, &tol()),
            Err(CoreError::NotImplemented { .. })
        ));
    }

    #[test]
    fn cone_cone_parallel_offset_axes_not_implemented() {
        // Parallel axes but distinct axis lines — still the general quartic.
        let a = cone(Point3::origin(), Vector3::z(), 0.5, 1.0);
        let b = cone(Point3::new(3.0, 0.0, 0.0), Vector3::z(), 0.5, 1.0);
        assert!(matches!(
            intersect(&a, &b, &tol()),
            Err(CoreError::NotImplemented { .. })
        ));
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

    // ── sphere-sphere ──────────────────────────────────────────────────

    fn sphere_at(center: Point3, radius: f64) -> Surface3 {
        Surface3::Sphere {
            center,
            axis: Vector3::z(),
            radius,
        }
    }

    #[test]
    fn spheres_transversal_circle() {
        let a = unit_sphere_at_origin(2.0);
        let b = sphere_at(Point3::new(3.0, 0.0, 0.0), 2.0);
        let curves = expect_curves(&a, &b, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
        let Curve3::Circle {
            center,
            axis,
            radius,
        } = &curves[0].curve
        else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!((center - Point3::new(1.5, 0.0, 0.0)).norm() < EPS);
        assert!(axis.cross(&Vector3::x()).norm() < EPS);
        assert!((radius - 1.75_f64.sqrt()).abs() < EPS);
    }

    #[test]
    fn spheres_generic_offset_circle() {
        let a = sphere_at(Point3::new(1.0, -2.0, 0.5), 2.5);
        let b = sphere_at(Point3::new(2.0, 0.0, -1.0), 1.8);
        let curves = expect_curves(&a, &b, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Transversal);
    }

    #[test]
    fn spheres_external_tangent_point() {
        let a = unit_sphere_at_origin(2.0);
        let b = sphere_at(Point3::new(5.0, 0.0, 0.0), 3.0);
        let expected = SurfaceIntersection::TangentPoint(Point3::new(2.0, 0.0, 0.0));
        assert_eq!(intersect(&a, &b, &tol()).unwrap(), expected);
        assert_eq!(intersect(&b, &a, &tol()).unwrap(), expected);
    }

    #[test]
    fn spheres_internal_tangent_point() {
        let a = unit_sphere_at_origin(3.0);
        let b = sphere_at(Point3::new(2.0, 0.0, 0.0), 1.0);
        let result = intersect(&a, &b, &tol()).unwrap();
        let SurfaceIntersection::TangentPoint(p) = result else {
            panic!("expected a tangent point, got {result:?}");
        };
        assert!((p - Point3::new(3.0, 0.0, 0.0)).norm() < EPS);
    }

    #[test]
    fn spheres_disjoint_and_nested_empty() {
        let a = unit_sphere_at_origin(2.0);
        let apart = sphere_at(Point3::new(10.0, 0.0, 0.0), 3.0);
        assert_eq!(
            intersect(&a, &apart, &tol()).unwrap(),
            SurfaceIntersection::Empty
        );
        let nested = sphere_at(Point3::new(0.5, 0.0, 0.0), 0.5);
        assert_eq!(
            intersect(&a, &nested, &tol()).unwrap(),
            SurfaceIntersection::Empty
        );
        let concentric = unit_sphere_at_origin(1.0);
        assert_eq!(
            intersect(&a, &concentric, &tol()).unwrap(),
            SurfaceIntersection::Empty
        );
    }

    #[test]
    fn spheres_coincident() {
        let a = unit_sphere_at_origin(2.0);
        // Same locus regardless of the pole axis.
        let b = Surface3::Sphere {
            center: Point3::origin(),
            axis: Vector3::x(),
            radius: 2.0,
        };
        assert_eq!(
            intersect(&a, &b, &tol()).unwrap(),
            SurfaceIntersection::Coincident
        );
    }

    // ── cylinder-sphere ────────────────────────────────────────────────

    #[test]
    fn cylinder_sphere_coaxial_two_circles() {
        let cyl = cylinder_z(1.0);
        let sph = unit_sphere_at_origin(2.0);
        let curves = expect_curves(&cyl, &sph, 2);
        let h = 3.0_f64.sqrt();
        let mut heights: Vec<f64> = curves
            .iter()
            .map(|ic| {
                assert_eq!(ic.kind, IntersectionKind::Transversal);
                let Curve3::Circle { center, radius, .. } = ic.curve else {
                    panic!("expected circles, got {:?}", ic.curve);
                };
                assert!((radius - 1.0).abs() < EPS);
                center.z
            })
            .collect();
        heights.sort_by(f64::total_cmp);
        assert!((heights[0] + h).abs() < EPS && (heights[1] - h).abs() < EPS);
    }

    #[test]
    fn cylinder_sphere_inscribed_tangent_circle() {
        let curves = expect_curves(&cylinder_z(1.5), &unit_sphere_at_origin(1.5), 1);
        assert_eq!(curves[0].kind, IntersectionKind::Tangential);
        let Curve3::Circle { center, radius, .. } = curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!(center.coords.norm() < EPS);
        assert!((radius - 1.5).abs() < EPS);
    }

    #[test]
    fn cylinder_sphere_small_coaxial_empty() {
        let result = intersect(&cylinder_z(1.0), &unit_sphere_at_origin(0.5), &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
    }

    #[test]
    fn cylinder_sphere_external_tangent_point() {
        let cyl = cylinder_z(1.0);
        let sph = sphere_at(Point3::new(3.0, 0.0, 0.0), 2.0);
        let expected = SurfaceIntersection::TangentPoint(Point3::new(1.0, 0.0, 0.0));
        assert_eq!(intersect(&cyl, &sph, &tol()).unwrap(), expected);
        assert_eq!(intersect(&sph, &cyl, &tol()).unwrap(), expected);
    }

    #[test]
    fn cylinder_sphere_internal_tangent_point() {
        // Sphere inside the wide cylinder, touching the near wall.
        let cyl = cylinder_z(3.0);
        let sph = sphere_at(Point3::new(2.0, 0.0, 0.0), 1.0);
        let result = intersect(&cyl, &sph, &tol()).unwrap();
        let SurfaceIntersection::TangentPoint(p) = result else {
            panic!("expected a tangent point, got {result:?}");
        };
        assert!((p - Point3::new(3.0, 0.0, 0.0)).norm() < EPS);
    }

    #[test]
    fn cylinder_sphere_far_wall_tangency_not_implemented() {
        // Viviani configuration: d = R − r, the sphere grazes the far wall
        // while crossing the near one — singular quartic.
        let cyl = cylinder_z(1.0);
        let sph = sphere_at(Point3::new(1.0, 0.0, 0.0), 2.0);
        let result = intersect(&cyl, &sph, &tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    #[test]
    fn cylinder_sphere_general_quartic_not_implemented() {
        let cyl = cylinder_z(1.0);
        let sph = sphere_at(Point3::new(1.2, 0.0, 0.0), 0.8);
        let result = intersect(&cyl, &sph, &tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    #[test]
    fn cylinder_sphere_offset_disjoint_empty() {
        let result = intersect(
            &cylinder_z(1.0),
            &sphere_at(Point3::new(5.0, 0.0, 0.0), 1.0),
            &tol(),
        )
        .unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
    }

    // ── sphere-torus ───────────────────────────────────────────────────

    #[test]
    fn sphere_torus_concentric_two_circles() {
        let sph = unit_sphere_at_origin(3.0);
        let tor = torus_z(3.0, 1.0);
        let curves = expect_curves(&sph, &tor, 2);
        // Meridian: tube circle (3, 0) r 1 against sphere profile radius 3
        // from the center → crossings at rho = 17/6, |h| = √35/6.
        for ic in &curves {
            assert_eq!(ic.kind, IntersectionKind::Transversal);
            let Curve3::Circle { center, radius, .. } = ic.curve else {
                panic!("expected circles, got {:?}", ic.curve);
            };
            assert!((radius - 17.0 / 6.0).abs() < EPS);
            assert!((center.z.abs() - 35.0_f64.sqrt() / 6.0).abs() < EPS);
        }
    }

    #[test]
    fn sphere_torus_outer_equator_tangent_circle() {
        let curves = expect_curves(&unit_sphere_at_origin(4.0), &torus_z(3.0, 1.0), 1);
        assert_eq!(curves[0].kind, IntersectionKind::Tangential);
        let Curve3::Circle { center, radius, .. } = curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!(center.coords.norm() < EPS);
        assert!((radius - 4.0).abs() < EPS);
    }

    #[test]
    fn sphere_torus_inner_equator_tangent_circle() {
        let curves = expect_curves(&unit_sphere_at_origin(2.0), &torus_z(3.0, 1.0), 1);
        assert_eq!(curves[0].kind, IntersectionKind::Tangential);
        let Curve3::Circle { radius, .. } = curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!((radius - 2.0).abs() < EPS);
    }

    #[test]
    fn sphere_torus_on_axis_above_center() {
        // Center lifted along the axis: two circles of unequal radius.
        let sph = sphere_at(Point3::new(0.0, 0.0, 1.0), 2.5);
        let curves = expect_curves(&sph, &torus_z(3.0, 1.0), 2);
        for ic in &curves {
            assert_eq!(ic.kind, IntersectionKind::Transversal);
        }
    }

    #[test]
    fn sphere_torus_small_concentric_empty() {
        let result = intersect(&unit_sphere_at_origin(1.0), &torus_z(3.0, 1.0), &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
    }

    #[test]
    fn sphere_torus_off_axis_not_implemented() {
        let sph = sphere_at(Point3::new(0.5, 0.0, 0.0), 2.0);
        let result = intersect(&sph, &torus_z(3.0, 1.0), &tol());
        assert!(matches!(result, Err(CoreError::NotImplemented { .. })));
    }

    // ── cylinder-torus ─────────────────────────────────────────────────

    #[test]
    fn cylinder_torus_coaxial_through_tube_center() {
        let curves = expect_curves(&cylinder_z(3.0), &torus_z(3.0, 1.0), 2);
        let mut heights: Vec<f64> = curves
            .iter()
            .map(|ic| {
                assert_eq!(ic.kind, IntersectionKind::Transversal);
                let Curve3::Circle { center, radius, .. } = ic.curve else {
                    panic!("expected circles, got {:?}", ic.curve);
                };
                assert!((radius - 3.0).abs() < EPS);
                center.z
            })
            .collect();
        heights.sort_by(f64::total_cmp);
        assert!((heights[0] + 1.0).abs() < EPS && (heights[1] - 1.0).abs() < EPS);
    }

    #[test]
    fn cylinder_torus_coaxial_generic_two_circles() {
        let curves = expect_curves(&cylinder_z(3.5), &torus_z(3.0, 1.0), 2);
        let h = 0.75_f64.sqrt();
        for ic in &curves {
            let Curve3::Circle { center, radius, .. } = ic.curve else {
                panic!("expected circles, got {:?}", ic.curve);
            };
            assert!((radius - 3.5).abs() < EPS);
            assert!((center.z.abs() - h).abs() < EPS);
        }
    }

    #[test]
    fn cylinder_torus_equator_tangent_circles() {
        for (r_c, expected_radius) in [(4.0, 4.0), (2.0, 2.0)] {
            let curves = expect_curves(&cylinder_z(r_c), &torus_z(3.0, 1.0), 1);
            assert_eq!(curves[0].kind, IntersectionKind::Tangential);
            let Curve3::Circle { center, radius, .. } = curves[0].curve else {
                panic!("expected a circle, got {:?}", curves[0].curve);
            };
            assert!(center.coords.norm() < EPS);
            assert!((radius - expected_radius).abs() < EPS);
        }
    }

    #[test]
    fn cylinder_torus_coaxial_empty() {
        let result = intersect(&cylinder_z(4.5), &torus_z(3.0, 1.0), &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
        let result = intersect(&cylinder_z(1.5), &torus_z(3.0, 1.0), &tol()).unwrap();
        assert_eq!(result, SurfaceIntersection::Empty);
    }

    #[test]
    fn cylinder_torus_non_coaxial_not_implemented() {
        let offset = cylinder(Point3::new(0.5, 0.0, 0.0), Vector3::z(), 3.0);
        assert!(matches!(
            intersect(&offset, &torus_z(3.0, 1.0), &tol()),
            Err(CoreError::NotImplemented { .. })
        ));
        let tilted = cylinder(Point3::origin(), Vector3::x(), 3.0);
        assert!(matches!(
            intersect(&tilted, &torus_z(3.0, 1.0), &tol()),
            Err(CoreError::NotImplemented { .. })
        ));
    }

    // ── torus-torus ────────────────────────────────────────────────────

    fn torus_at(center: Point3, axis: Vector3, major: f64, minor: f64) -> Surface3 {
        Surface3::Torus {
            center,
            axis: axis.normalize(),
            major_radius: major,
            minor_radius: minor,
        }
    }

    #[test]
    fn tori_coaxial_stacked_two_circles() {
        let a = torus_z(3.0, 1.0);
        let b = torus_at(Point3::new(0.0, 0.0, 1.0), Vector3::z(), 3.0, 1.0);
        let curves = expect_curves(&a, &b, 2);
        // Tube circles (3, 0) and (3, 1), both r = 1: crossings at
        // h = 1/2, rho = 3 ± √3/2.
        let mut radii: Vec<f64> = curves
            .iter()
            .map(|ic| {
                assert_eq!(ic.kind, IntersectionKind::Transversal);
                let Curve3::Circle { center, radius, .. } = ic.curve else {
                    panic!("expected circles, got {:?}", ic.curve);
                };
                assert!((center.z - 0.5).abs() < EPS);
                radius
            })
            .collect();
        radii.sort_by(f64::total_cmp);
        let half = 0.75_f64.sqrt();
        assert!((radii[0] - (3.0 - half)).abs() < EPS);
        assert!((radii[1] - (3.0 + half)).abs() < EPS);
    }

    #[test]
    fn tori_stacked_kissing_tangent_circle() {
        let a = torus_z(3.0, 1.0);
        let b = torus_at(Point3::new(0.0, 0.0, 2.0), Vector3::z(), 3.0, 1.0);
        let curves = expect_curves(&a, &b, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Tangential);
        let Curve3::Circle { center, radius, .. } = curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!((center - Point3::new(0.0, 0.0, 1.0)).norm() < EPS);
        assert!((radius - 3.0).abs() < EPS);
    }

    #[test]
    fn tori_concentric_kissing_tubes_tangent_circle() {
        // Same plane, majors 2 and 4, tubes radius 1: they kiss at rho 3.
        let a = torus_z(2.0, 1.0);
        let b = torus_z(4.0, 1.0);
        let curves = expect_curves(&a, &b, 1);
        assert_eq!(curves[0].kind, IntersectionKind::Tangential);
        let Curve3::Circle { center, radius, .. } = curves[0].curve else {
            panic!("expected a circle, got {:?}", curves[0].curve);
        };
        assert!(center.coords.norm() < EPS);
        assert!((radius - 3.0).abs() < EPS);
    }

    #[test]
    fn tori_coaxial_far_apart_empty() {
        let a = torus_z(3.0, 1.0);
        let b = torus_at(Point3::new(0.0, 0.0, 3.0), Vector3::z(), 3.0, 1.0);
        assert_eq!(
            intersect(&a, &b, &tol()).unwrap(),
            SurfaceIntersection::Empty
        );
        let nested = torus_z(3.0, 0.25);
        assert_eq!(
            intersect(&torus_z(3.0, 1.0), &nested, &tol()).unwrap(),
            SurfaceIntersection::Empty
        );
    }

    #[test]
    fn tori_coincident_with_flipped_axis() {
        let a = torus_z(3.0, 1.0);
        let b = torus_at(Point3::origin(), -Vector3::z(), 3.0, 1.0);
        assert_eq!(
            intersect(&a, &b, &tol()).unwrap(),
            SurfaceIntersection::Coincident
        );
    }

    #[test]
    fn tori_non_coaxial_not_implemented() {
        let a = torus_z(3.0, 1.0);
        let offset = torus_at(Point3::new(0.4, 0.0, 0.0), Vector3::z(), 3.0, 0.8);
        assert!(matches!(
            intersect(&a, &offset, &tol()),
            Err(CoreError::NotImplemented { .. })
        ));
        let tilted = torus_at(Point3::origin(), Vector3::x(), 3.0, 1.0);
        assert!(matches!(
            intersect(&a, &tilted, &tol()),
            Err(CoreError::NotImplemented { .. })
        ));
    }

    // ── dispatch ───────────────────────────────────────────────────────

    #[test]
    fn unsupported_pair_not_implemented() {
        let a = cone_30deg();
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
