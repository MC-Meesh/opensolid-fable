//! The differentiable field tower: every primitive and operator, written
//! once over a generic [`Scalar`].
//!
//! Each function here mirrors the closed form of its counterpart in
//! [`primitives`](crate::primitives), [`csg`](crate::csg),
//! [`blend`](crate::blend) and [`ops`](crate::ops) exactly — the same
//! algebra, with `f64` replaced by `T: Scalar`. Instantiated at `f64` they
//! reproduce the existing fields (enforced by
//! `tests/param_grad_fd.rs::field_tower_agrees_with_sdf_impls`); instantiated
//! at [`Dual<N>`](super::Dual) they additionally return derivatives with
//! respect to the design parameters.
//!
//! Anything that is a *constant* of the model (a fixed centre, a sample
//! point) can be lifted with [`Vec3::cst`]; anything that is a *parameter*
//! (a radius to optimise) is passed as a seeded dual and its derivative
//! comes out the other end.
//!
//! # Coverage
//!
//! Primitives: sphere, box, rounded box, cylinder, torus, cone, capsule,
//! half-space. Operators: sharp union/intersection/subtraction, smooth
//! union/subtraction, offset/shell/rounded, translate/uniform scale.
//! Profiles, sweeps and patterns are not in the tower yet — see
//! `docs/design/DIFFERENTIABLE.md` §7.

use super::scalar::Scalar;
use super::vec::Vec3;

// ---------------------------------------------------------------- primitives

/// Sphere of `radius` about `center`. Mirrors [`Sphere`](crate::primitives::Sphere).
pub fn sphere<T: Scalar>(p: Vec3<T>, center: Vec3<T>, radius: T) -> T {
    (p - center).norm() - radius
}

/// Box SDF as a function of the per-axis offsets `dᵢ = |pᵢ - cᵢ| - hᵢ`.
/// Mirrors `primitives::box_distance`.
fn box_distance<T: Scalar>(d: Vec3<T>) -> T {
    let outside = d.relu().norm();
    let inside = d.max_component().min(T::zero());
    outside + inside
}

/// Axis-aligned box with `half_extents` about `center`.
/// Mirrors [`Box3`](crate::primitives::Box3).
pub fn box3<T: Scalar>(p: Vec3<T>, center: Vec3<T>, half_extents: Vec3<T>) -> T {
    box_distance((p - center).abs() - half_extents)
}

/// Box with edges rounded by `radius`.
/// Mirrors [`RoundedBox`](crate::primitives::RoundedBox).
pub fn rounded_box<T: Scalar>(p: Vec3<T>, center: Vec3<T>, half_extents: Vec3<T>, radius: T) -> T {
    let inner = half_extents - Vec3::splat(radius);
    box_distance((p - center).abs() - inner) - radius
}

/// Cylinder SDF from its radial and axial offsets.
/// Mirrors `primitives::cylinder_distance`.
fn cylinder_distance<T: Scalar>(radial: T, axial: T) -> T {
    let (ro, ao) = (radial.relu(), axial.relu());
    let outside = (ro.square() + ao.square()).sqrt();
    let inside = radial.max(axial).min(T::zero());
    outside + inside
}

/// Y-axis cylinder of `radius` and `half_height` about `center`.
/// Mirrors [`Cylinder`](crate::primitives::Cylinder).
pub fn cylinder<T: Scalar>(p: Vec3<T>, center: Vec3<T>, radius: T, half_height: T) -> T {
    let q = p - center;
    let radial = (q.x.square() + q.z.square()).sqrt() - radius;
    let axial = q.y.abs() - half_height;
    cylinder_distance(radial, axial)
}

/// Torus in the xz-plane. Mirrors [`Torus`](crate::primitives::Torus).
pub fn torus<T: Scalar>(p: Vec3<T>, center: Vec3<T>, major_radius: T, minor_radius: T) -> T {
    let q = p - center;
    let ring = (q.x.square() + q.z.square()).sqrt() - major_radius;
    (ring.square() + q.y.square()).sqrt() - minor_radius
}

/// Capped cone along y. Mirrors [`Cone`](crate::primitives::Cone).
pub fn cone<T: Scalar>(
    p: Vec3<T>,
    center: Vec3<T>,
    half_height: T,
    radius_bottom: T,
    radius_top: T,
) -> T {
    let (h, r1, r2) = (half_height, radius_bottom, radius_top);
    let q = p - center;
    let qx = (q.x.square() + q.z.square()).sqrt();
    let qy = q.y;
    // Branch on the *value*: which cap is nearer is a discrete choice, and
    // the field is continuous across it.
    let cap_radius = if qy.val() < 0.0 { r1 } else { r2 };
    let ca = (qx - qx.min(cap_radius), qy.abs() - h);
    let k1 = (r2, h);
    let k2 = (r2 - r1, h * T::cst(2.0));
    let t = (((k1.0 - qx) * k2.0 + (k1.1 - qy) * k2.1) / (k2.0.square() + k2.1.square()))
        .clamp(0.0, 1.0);
    let cb = (qx - k1.0 + k2.0 * t, qy - k1.1 + k2.1 * t);
    let sign = if cb.0.val() < 0.0 && ca.1.val() < 0.0 {
        T::cst(-1.0)
    } else {
        T::one()
    };
    let d2 = (ca.0.square() + ca.1.square()).min(cb.0.square() + cb.1.square());
    sign * d2.sqrt()
}

/// Capsule: a segment `start`→`end` dilated by `radius`.
/// Mirrors [`Capsule`](crate::primitives::Capsule).
pub fn capsule<T: Scalar>(p: Vec3<T>, start: Vec3<T>, end: Vec3<T>, radius: T) -> T {
    let pa = p - start;
    let ba = end - start;
    let t = (pa.dot(ba) / ba.dot(ba)).clamp(0.0, 1.0);
    (pa - ba.scale(t)).norm() - radius
}

/// Half-space `{p : n·p <= offset}`.
/// Mirrors [`HalfSpace`](crate::primitives::HalfSpace).
pub fn half_space<T: Scalar>(p: Vec3<T>, normal: Vec3<T>, offset: T) -> T {
    (normal.dot(p) - offset) / normal.norm()
}

// ----------------------------------------------------------------- sharp CSG

/// Sharp union: `min(a, b)`. Mirrors [`Union`](crate::csg::Union).
pub fn union<T: Scalar>(a: T, b: T) -> T {
    a.min(b)
}

/// Sharp intersection: `max(a, b)`. Mirrors
/// [`Intersection`](crate::csg::Intersection).
pub fn intersection<T: Scalar>(a: T, b: T) -> T {
    a.max(b)
}

/// Sharp subtraction: `max(a, -b)`. Mirrors
/// [`Subtraction`](crate::csg::Subtraction).
pub fn subtraction<T: Scalar>(a: T, b: T) -> T {
    a.max(-b)
}

// ---------------------------------------------------------------- smooth CSG

/// Polynomial smooth union. Mirrors [`SmoothUnion`](crate::blend::SmoothUnion).
///
/// Unlike [`union`], this is C¹ in the blend band, so the parameter gradient
/// is continuous there — the reason `radius` doubles as a *smoothing
/// temperature* for optimisation. See `docs/design/DIFFERENTIABLE.md` §4.
pub fn smooth_union<T: Scalar>(a: T, b: T, radius: T) -> T {
    let h = (T::cst(0.5) + T::cst(0.5) * (b - a) / radius).clamp(0.0, 1.0);
    b * (T::one() - h) + a * h - radius * h * (T::one() - h)
}

/// Polynomial smooth subtraction. Mirrors
/// [`SmoothSubtraction`](crate::blend::SmoothSubtraction).
pub fn smooth_subtraction<T: Scalar>(a: T, b: T, radius: T) -> T {
    // `h` uses the raw cutter distance so it clamps to 0 far from the
    // cutter (returning `a`), not the negated one — as in `blend.rs`.
    let h = (T::cst(0.5) - T::cst(0.5) * (a + b) / radius).clamp(0.0, 1.0);
    a * (T::one() - h) - b * h + radius * h * (T::one() - h)
}

// ------------------------------------------------------------------ operators

/// Grow (positive) or shrink (negative) by `distance`.
/// Mirrors [`Offset`](crate::ops::Offset).
pub fn offset<T: Scalar>(d: T, distance: T) -> T {
    d - distance
}

/// Hollow to a wall of `thickness`. Mirrors [`Shell`](crate::ops::Shell).
pub fn shell<T: Scalar>(d: T, thickness: T) -> T {
    d.abs() - thickness * T::cst(0.5)
}

/// Round convex edges by `radius`. Mirrors [`Rounded`](crate::ops::Rounded).
pub fn rounded<T: Scalar>(d: T, radius: T) -> T {
    d - radius
}

// ----------------------------------------------------------------- transforms

/// Translate a field: evaluate the child at `p - offset`.
///
/// Returns the *pulled-back point*; compose it with a primitive. Translation
/// is an isometry, so the field stays a true distance field.
pub fn translate<T: Scalar>(p: Vec3<T>, offset: Vec3<T>) -> Vec3<T> {
    p - offset
}

/// Uniform scale by `factor` about the origin, applied to a child field.
///
/// Mirrors [`UniformScale`](crate::transform::UniformScale): the point is
/// pulled back and the resulting distance rescaled, which keeps the metric
/// property. `eval_child` receives the pulled-back point.
pub fn uniform_scale<T: Scalar>(p: Vec3<T>, factor: T, eval_child: impl Fn(Vec3<T>) -> T) -> T {
    eval_child(p.scale(T::one() / factor)) * factor
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::diff::Dual;

    /// The tower at `f64` must reproduce the existing `Sdf` impls; the
    /// cross-check against every impl lives in `tests/param_grad_fd.rs`.
    #[test]
    fn sphere_matches_closed_form() {
        let d = sphere(Vec3::cst(2.0, 0.0, 0.0), Vec3::<f64>::zero(), 1.0);
        assert!((d - 1.0).abs() < 1e-12);
    }

    #[test]
    fn sphere_radius_derivative_is_minus_one() {
        // d/dr (|p - c| - r) = -1, everywhere.
        let r = Dual::<1>::seed(1.0, 0);
        let d = sphere(Vec3::from_point(&[2.0, 0.0, 0.0].into()), Vec3::zero(), r);
        assert!((d.val() - 1.0).abs() < 1e-12);
        assert!((d.grad()[0] - (-1.0)).abs() < 1e-12);
    }

    #[test]
    fn box_inside_is_negative_outside_positive() {
        let c = Vec3::<f64>::zero();
        let h = Vec3::<f64>::splat(1.0);
        assert!(box3(Vec3::cst(0.0, 0.0, 0.0), c, h) < 0.0);
        assert!(box3(Vec3::cst(3.0, 0.0, 0.0), c, h) > 0.0);
        // Face distance is exact.
        assert!((box3(Vec3::cst(2.0, 0.0, 0.0), c, h) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn union_takes_the_nearer_field() {
        assert_eq!(union(1.0, -2.0), -2.0);
        assert_eq!(intersection(1.0, -2.0), 1.0);
        assert_eq!(subtraction(-3.0, -2.0), 2.0);
    }

    #[test]
    fn smooth_union_is_at_most_sharp_union() {
        // The polynomial blend only ever pulls the surface outward (down).
        for i in 0..20 {
            let a = -1.0 + 0.1 * i as f64;
            let (b, r) = (0.3, 0.5);
            let s = smooth_union(a, b, r);
            assert!(s <= union(a, b) + 1e-12);
            assert!(s >= union(a, b) - 0.25 * r - 1e-12);
        }
    }

    #[test]
    fn smooth_union_degenerates_to_sharp_far_apart() {
        // |a - b| >= radius → h clamps and the blend returns a child exactly.
        assert!((smooth_union(0.0, 5.0, 0.5) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn shell_is_symmetric_about_the_surface() {
        assert!((shell(0.4, 1.0) - shell(-0.4, 1.0)).abs() < 1e-12);
    }

    #[test]
    fn uniform_scale_doubles_a_sphere() {
        // Scaling a unit sphere by 2 puts the surface at radius 2.
        let d = uniform_scale(Vec3::cst(2.0, 0.0, 0.0), 2.0, |q| {
            sphere(q, Vec3::<f64>::zero(), 1.0)
        });
        assert!(d.abs() < 1e-12);
    }

    #[test]
    fn translate_pulls_the_point_back() {
        let q = translate(Vec3::cst(1.0, 2.0, 3.0), Vec3::<f64>::cst(1.0, 1.0, 1.0));
        assert_eq!(q.val(), [0.0, 1.0, 2.0]);
    }
}
