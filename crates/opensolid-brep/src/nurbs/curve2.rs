//! NURBS curves in a surface's 2D parameter space — the freeform
//! counterpart of [`NurbsCurve`](crate::nurbs::NurbsCurve), for trim
//! geometry that is genuinely a NURBS in `(u, v)` (of-50u).
//!
//! The de Boor machinery is [`KnotVector`]'s, shared with the 3D curve:
//! only the coordinate type differs. Rational curves are handled in
//! homogeneous coordinates — each control point `P_i` with weight `w_i`
//! maps to `(w_i·P_i, w_i)` in 3D — and evaluation projects back by
//! dividing through the weight component.
//!
//! Evaluation outside the knot domain clamps the parameter to the domain
//! (clamped curves do not extrapolate), matching the 3D convention.

use super::curve::{KnotVector, NurbsError, binomial};
use nalgebra::Vector3 as Homogeneous;
use opensolid_core::types::{Point2, Vector2};

/// Non-uniform rational B-spline curve in `(u, v)` parameter space.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsCurve2 {
    control_points: Vec<Point2>,
    weights: Vec<f64>,
    knots: KnotVector,
}

impl NurbsCurve2 {
    /// Rational curve from weighted control points. Validation mirrors
    /// [`NurbsCurve::new`](crate::nurbs::NurbsCurve::new): counts must
    /// match the knot vector, weights must be finite and positive, and
    /// control coordinates finite — a 2D `CARTESIAN_POINT` comes out of an
    /// untrusted STEP file exactly as a 3D one does.
    pub fn new(
        control_points: Vec<Point2>,
        weights: Vec<f64>,
        knots: KnotVector,
    ) -> Result<Self, NurbsError> {
        let expected = knots.control_count();
        if control_points.len() != expected {
            return Err(NurbsError::ControlCountMismatch {
                control_points: control_points.len(),
                expected,
            });
        }
        if weights.len() != control_points.len() {
            return Err(NurbsError::WeightCountMismatch {
                weights: weights.len(),
                control_points: control_points.len(),
            });
        }
        if let Some(index) = weights.iter().position(|&w| !(w.is_finite() && w > 0.0)) {
            return Err(NurbsError::NonPositiveWeight { index });
        }
        if let Some(index) = control_points
            .iter()
            .position(|p| !(p.x.is_finite() && p.y.is_finite()))
        {
            return Err(NurbsError::NonFiniteControlPoint { index });
        }
        Ok(Self {
            control_points,
            weights,
            knots,
        })
    }

    /// Non-rational (all weights 1) B-spline curve.
    pub fn bspline(control_points: Vec<Point2>, knots: KnotVector) -> Result<Self, NurbsError> {
        let weights = vec![1.0; control_points.len()];
        Self::new(control_points, weights, knots)
    }

    pub fn control_points(&self) -> &[Point2] {
        &self.control_points
    }

    pub fn weights(&self) -> &[f64] {
        &self.weights
    }

    pub fn knot_vector(&self) -> &KnotVector {
        &self.knots
    }

    pub fn degree(&self) -> usize {
        self.knots.degree()
    }

    /// Parameter domain `(knots[degree], knots[len - degree - 1])`.
    pub fn domain(&self) -> (f64, f64) {
        self.knots.domain()
    }

    /// Homogeneous control point `(w·P, w)` at `index`.
    fn homogeneous(&self, index: usize) -> Homogeneous<f64> {
        let p = &self.control_points[index];
        let w = self.weights[index];
        Homogeneous::new(w * p.x, w * p.y, w)
    }

    /// Parameter-space position at `t` (clamped to the domain).
    pub fn point(&self, t: f64) -> Point2 {
        let (t0, t1) = self.knots.domain();
        let u = t.clamp(t0, t1);
        let p = self.degree();
        let span = self.knots.find_span(u);
        let basis = self.knots.basis_funs(span, u);
        let mut sum = Homogeneous::zeros();
        for (j, &value) in basis.iter().enumerate() {
            sum += self.homogeneous(span - p + j) * value;
        }
        Point2::new(sum.x / sum.z, sum.y / sum.z)
    }

    /// First derivative with respect to `t`.
    pub fn derivative(&self, t: f64) -> Vector2 {
        self.derivatives(t, 1)[1]
    }

    /// Derivatives of the curve with respect to `t`, orders `0..=order`
    /// (`result[0]` is the position as a vector from the origin). Rational
    /// derivatives via the quotient rule on the homogeneous curve (Piegl &
    /// Tiller A4.2), exactly as the 3D curve computes them.
    pub fn derivatives(&self, t: f64, order: usize) -> Vec<Vector2> {
        let (t0, t1) = self.knots.domain();
        let u = t.clamp(t0, t1);
        let p = self.degree();
        let span = self.knots.find_span(u);
        let basis_ders = self.knots.ders_basis_funs(span, u, order);

        let mut homo: Vec<Homogeneous<f64>> = Vec::with_capacity(order + 1);
        for row in basis_ders.iter() {
            let mut sum = Homogeneous::zeros();
            for (j, &value) in row.iter().enumerate() {
                sum += self.homogeneous(span - p + j) * value;
            }
            homo.push(sum);
        }

        // C^(k) = (A^(k) - Σ_{i=1..k} C(k,i)·w^(i)·C^(k-i)) / w.
        let mut ders: Vec<Vector2> = Vec::with_capacity(order + 1);
        for (k, a) in homo.iter().enumerate() {
            let mut v = a.xy();
            for i in 1..=k {
                v -= binomial(k, i) * homo[i].z * ders[k - i];
            }
            ders.push(v / homo[0].z);
        }
        ders
    }

    /// The same locus traced in the opposite direction, over the same
    /// parameter domain: control points and weights reversed, knot `k`
    /// reflected to `t0 + t1 − k`. Mirrors
    /// [`NurbsCurve::reversed`](crate::nurbs::NurbsCurve::reversed), and
    /// like it satisfies `reversed().point(t) == point(t0 + t1 − t)`.
    pub fn reversed(&self) -> NurbsCurve2 {
        let (t0, t1) = self.knots.domain();
        let sum = t0 + t1;
        let knots: Vec<f64> = self.knots.knots().iter().rev().map(|&k| sum - k).collect();
        let degree = self.degree();
        NurbsCurve2 {
            control_points: self.control_points.iter().rev().copied().collect(),
            weights: self.weights.iter().rev().copied().collect(),
            knots: KnotVector::new(degree, knots)
                .expect("reflecting a valid knot vector keeps it valid"),
        }
    }

    /// A copy translated by `shift` in parameter space. Exact for the same
    /// reason [`NurbsCurve::map_control_points`] is exact under affine maps:
    /// evaluation is a weighted average whose basis weights sum to one.
    ///
    /// [`NurbsCurve::map_control_points`]: crate::nurbs::NurbsCurve::map_control_points
    pub fn translated(&self, shift: Vector2) -> NurbsCurve2 {
        NurbsCurve2 {
            control_points: self.control_points.iter().map(|p| p + shift).collect(),
            weights: self.weights.clone(),
            knots: self.knots.clone(),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_1_SQRT_2;

    const TIGHT: f64 = 1e-12;

    /// Exact unit circle in `(u, v)`: rational quadratic, nine control
    /// points over four 90° arcs (Piegl & Tiller §7.5).
    fn unit_circle() -> NurbsCurve2 {
        let pts = vec![
            Point2::new(1.0, 0.0),
            Point2::new(1.0, 1.0),
            Point2::new(0.0, 1.0),
            Point2::new(-1.0, 1.0),
            Point2::new(-1.0, 0.0),
            Point2::new(-1.0, -1.0),
            Point2::new(0.0, -1.0),
            Point2::new(1.0, -1.0),
            Point2::new(1.0, 0.0),
        ];
        let s = FRAC_1_SQRT_2;
        let weights = vec![1.0, s, 1.0, s, 1.0, s, 1.0, s, 1.0];
        let knots = KnotVector::new(
            2,
            vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ],
        )
        .unwrap();
        NurbsCurve2::new(pts, weights, knots).unwrap()
    }

    fn generic_cubic() -> NurbsCurve2 {
        let pts = vec![
            Point2::new(0.0, 0.0),
            Point2::new(1.0, 2.0),
            Point2::new(3.0, 2.5),
            Point2::new(4.0, 0.0),
            Point2::new(5.0, -1.5),
            Point2::new(7.0, 1.0),
        ];
        let weights = vec![1.0, 0.5, 2.0, 1.5, 0.8, 1.0];
        let knots =
            KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 0.4, 0.7, 1.0, 1.0, 1.0, 1.0]).unwrap();
        NurbsCurve2::new(pts, weights, knots).unwrap()
    }

    #[test]
    fn rational_circle_is_exact() {
        let circle = unit_circle();
        for i in 0..=200 {
            let t = i as f64 / 200.0;
            let p = circle.point(t);
            assert!(
                (p.coords.norm() - 1.0).abs() < TIGHT,
                "radius drift {} at t={t}",
                (p.coords.norm() - 1.0).abs()
            );
        }
        let quarters = [
            (0.0, Point2::new(1.0, 0.0)),
            (0.25, Point2::new(0.0, 1.0)),
            (0.5, Point2::new(-1.0, 0.0)),
            (0.75, Point2::new(0.0, -1.0)),
            (1.0, Point2::new(1.0, 0.0)),
        ];
        for (t, expected) in quarters {
            assert!((circle.point(t) - expected).norm() < TIGHT, "at t={t}");
        }
    }

    #[test]
    fn derivatives_match_finite_differences() {
        for curve in [unit_circle(), generic_cubic()] {
            for t in [0.1, 0.3, 0.55, 0.9] {
                let h = 1e-6;
                let fd = (curve.point(t + h) - curve.point(t - h)) / (2.0 * h);
                let d = curve.derivative(t);
                assert!(
                    (fd - d).norm() < 1e-5,
                    "derivative mismatch at t={t}: {d:?} vs fd {fd:?}"
                );
            }
        }
    }

    #[test]
    fn evaluation_clamps_outside_the_domain() {
        let curve = generic_cubic();
        assert!((curve.point(-2.0) - curve.point(0.0)).norm() < TIGHT);
        assert!((curve.point(5.0) - curve.point(1.0)).norm() < TIGHT);
        assert!((curve.point(0.0) - curve.control_points()[0]).norm() < TIGHT);
        assert!((curve.point(1.0) - curve.control_points()[5]).norm() < TIGHT);
    }

    #[test]
    fn reversed_traces_the_same_locus_backwards() {
        let curve = generic_cubic();
        let back = curve.reversed();
        let (t0, t1) = curve.domain();
        assert_eq!(back.domain(), (t0, t1));
        for i in 0..=50 {
            let t = t0 + (t1 - t0) * i as f64 / 50.0;
            let mirrored = t0 + t1 - t;
            assert!(
                (back.point(t) - curve.point(mirrored)).norm() < TIGHT,
                "reversed point mismatch at t={t}"
            );
            assert!(
                (back.derivative(t) + curve.derivative(mirrored)).norm() < 1e-9,
                "reversed tangent must oppose the original at t={t}"
            );
        }
    }

    #[test]
    fn translated_moves_the_whole_locus() {
        let curve = generic_cubic();
        let shift = Vector2::new(2.5, -1.0);
        let moved = curve.translated(shift);
        assert_eq!(moved.weights(), curve.weights());
        assert_eq!(moved.knot_vector(), curve.knot_vector());
        for i in 0..=50 {
            let t = i as f64 / 50.0;
            assert!(
                (moved.point(t) - (curve.point(t) + shift)).norm() < TIGHT,
                "translation must commute with evaluation at t={t}"
            );
        }
    }

    #[test]
    fn constructor_validation_mirrors_the_3d_curve() {
        let kv = KnotVector::clamped_uniform(1, 3).unwrap();
        assert_eq!(
            NurbsCurve2::bspline(vec![Point2::origin(); 2], kv.clone()),
            Err(NurbsError::ControlCountMismatch {
                control_points: 2,
                expected: 3
            })
        );
        assert_eq!(
            NurbsCurve2::new(vec![Point2::origin(); 3], vec![1.0; 2], kv.clone()),
            Err(NurbsError::WeightCountMismatch {
                weights: 2,
                control_points: 3
            })
        );
        assert_eq!(
            NurbsCurve2::new(vec![Point2::origin(); 3], vec![1.0, -1.0, 1.0], kv.clone()),
            Err(NurbsError::NonPositiveWeight { index: 1 })
        );
        assert_eq!(
            NurbsCurve2::new(
                vec![Point2::origin(); 3],
                vec![1.0, f64::NAN, 1.0],
                kv.clone()
            ),
            Err(NurbsError::NonPositiveWeight { index: 1 })
        );
        let mut pts = vec![Point2::origin(); 3];
        pts[2] = Point2::new(f64::INFINITY, 0.0);
        assert_eq!(
            NurbsCurve2::bspline(pts, kv),
            Err(NurbsError::NonFiniteControlPoint { index: 2 })
        );
    }
}
