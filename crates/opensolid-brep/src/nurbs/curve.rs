//! NURBS curves: knot vectors, rational de Boor evaluation, derivatives,
//! and Boehm knot insertion.
//!
//! Algorithms follow Piegl & Tiller, *The NURBS Book* (2nd ed.): FindSpan
//! (A2.1), BasisFuns (A2.2), DersBasisFuns (A2.3), rational derivatives
//! (A4.2), and single knot insertion (A5.1). Rational curves are handled in
//! homogeneous coordinates: each control point `P_i` with weight `w_i` maps
//! to `(w_i·P_i, w_i)` in 4D, and evaluation projects back by dividing
//! through the weight component.
//!
//! Evaluation outside the knot domain clamps the parameter to the domain
//! (clamped curves do not extrapolate).

use crate::curve::CurveEval;
use nalgebra::Vector4;
use opensolid_core::types::{Point3, Vector3};
use thiserror::Error;

/// Errors from NURBS construction and editing.
#[derive(Debug, Error, PartialEq)]
pub enum NurbsError {
    #[error("knot vector needs at least {expected} knots for degree {degree}, got {got}")]
    NotEnoughKnots {
        degree: usize,
        expected: usize,
        got: usize,
    },
    #[error("knot vector is decreasing at index {index}")]
    DecreasingKnots { index: usize },
    #[error("knot vector has an empty domain (start knot equals end knot)")]
    DegenerateDomain,
    #[error("{control_points} control points given, knot vector expects {expected}")]
    ControlCountMismatch {
        control_points: usize,
        expected: usize,
    },
    #[error("{weights} weights given for {control_points} control points")]
    WeightCountMismatch {
        weights: usize,
        control_points: usize,
    },
    #[error("weight at index {index} must be positive")]
    NonPositiveWeight { index: usize },
    #[error("knot {knot} lies outside the open domain ({start}, {end})")]
    KnotOutOfDomain { knot: f64, start: f64, end: f64 },
    #[error("inserting knot {knot} would raise its multiplicity above degree {degree}")]
    MultiplicityExceedsDegree { knot: f64, degree: usize },
}

/// Validated knot vector of a fixed degree.
///
/// Invariants (enforced at construction): non-decreasing, at least
/// `2 * (degree + 1)` knots (so at least `degree + 1` control points), and a
/// non-empty domain `knots[degree] < knots[len - degree - 1]`.
#[derive(Debug, Clone, PartialEq)]
pub struct KnotVector {
    degree: usize,
    knots: Vec<f64>,
}

impl KnotVector {
    /// Validate and wrap a knot vector for curves of `degree`.
    pub fn new(degree: usize, knots: Vec<f64>) -> Result<Self, NurbsError> {
        let expected = 2 * (degree + 1);
        if knots.len() < expected {
            return Err(NurbsError::NotEnoughKnots {
                degree,
                expected,
                got: knots.len(),
            });
        }
        for i in 1..knots.len() {
            if knots[i] < knots[i - 1] {
                return Err(NurbsError::DecreasingKnots { index: i });
            }
        }
        if knots[degree] >= knots[knots.len() - degree - 1] {
            return Err(NurbsError::DegenerateDomain);
        }
        Ok(Self { degree, knots })
    }

    /// Clamped uniform knot vector on `[0, 1]` for `control_count` control
    /// points: end knots repeated `degree + 1` times, interior knots evenly
    /// spaced.
    pub fn clamped_uniform(degree: usize, control_count: usize) -> Result<Self, NurbsError> {
        let expected = 2 * (degree + 1);
        if control_count < degree + 1 {
            return Err(NurbsError::NotEnoughKnots {
                degree,
                expected,
                got: control_count + degree + 1,
            });
        }
        let interior = control_count - degree - 1;
        let mut knots = vec![0.0; degree + 1];
        for i in 1..=interior {
            knots.push(i as f64 / (interior + 1) as f64);
        }
        knots.extend(std::iter::repeat_n(1.0, degree + 1));
        Self::new(degree, knots)
    }

    pub fn degree(&self) -> usize {
        self.degree
    }

    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    /// Number of control points this knot vector pairs with
    /// (`len - degree - 1`).
    pub fn control_count(&self) -> usize {
        self.knots.len() - self.degree - 1
    }

    /// Parameter domain `(knots[degree], knots[len - degree - 1])`.
    pub fn domain(&self) -> (f64, f64) {
        (
            self.knots[self.degree],
            self.knots[self.knots.len() - self.degree - 1],
        )
    }

    /// Multiplicity of `u` in the knot vector (exact float comparison; use
    /// values taken from `knots()` or previously inserted).
    pub fn multiplicity(&self, u: f64) -> usize {
        self.knots.iter().filter(|&&k| k == u).count()
    }

    /// Index of the knot span containing `u` (FindSpan, A2.1): the unique
    /// `i` with `knots[i] <= u < knots[i + 1]`, except at the domain end
    /// where the last non-empty span is returned. `u` is assumed within the
    /// domain.
    pub fn find_span(&self, u: f64) -> usize {
        let p = self.degree;
        let n = self.control_count() - 1;
        if u >= self.knots[n + 1] {
            return n;
        }
        if u <= self.knots[p] {
            return p;
        }
        let (mut lo, mut hi) = (p, n + 1);
        let mut mid = (lo + hi) / 2;
        while u < self.knots[mid] || u >= self.knots[mid + 1] {
            if u < self.knots[mid] {
                hi = mid;
            } else {
                lo = mid;
            }
            mid = (lo + hi) / 2;
        }
        mid
    }

    /// Non-zero basis functions `N_{span-p..=span, p}(u)` (BasisFuns, A2.2).
    fn basis_funs(&self, span: usize, u: f64) -> Vec<f64> {
        let p = self.degree;
        let mut n = vec![0.0; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        n[0] = 1.0;
        for j in 1..=p {
            left[j] = u - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - u;
            let mut saved = 0.0;
            for r in 0..j {
                let temp = n[r] / (right[r + 1] + left[j - r]);
                n[r] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            n[j] = saved;
        }
        n
    }

    /// Basis functions and their derivatives up to `order`
    /// (DersBasisFuns, A2.3). Returns `ders[k][j]` = k-th derivative of
    /// `N_{span-p+j, p}` at `u`; rows with `k > degree` are zero.
    fn ders_basis_funs(&self, span: usize, u: f64, order: usize) -> Vec<Vec<f64>> {
        let p = self.degree;
        let max_k = order.min(p);

        // ndu[j][r] for r < j holds knot differences; ndu[r][j] basis values.
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];
        ndu[0][0] = 1.0;
        for j in 1..=p {
            left[j] = u - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - u;
            let mut saved = 0.0;
            for r in 0..j {
                ndu[j][r] = right[r + 1] + left[j - r];
                let temp = ndu[r][j - 1] / ndu[j][r];
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }

        let mut ders = vec![vec![0.0; p + 1]; order + 1];
        for (j, value) in ders[0].iter_mut().enumerate() {
            *value = ndu[j][p];
        }

        let mut a = [vec![0.0; p + 1], vec![0.0; p + 1]];
        for r in 0..=p {
            let (mut s1, mut s2) = (0usize, 1usize);
            a[0].fill(0.0);
            a[1].fill(0.0);
            a[0][0] = 1.0;
            // `k` simultaneously indexes `ders`, offsets `rk`/`pk`, and
            // addresses `a[s2][k]`; iterator form would obscure the A2.3
            // transcription.
            #[allow(clippy::needless_range_loop)]
            for k in 1..=max_k {
                let mut d = 0.0;
                let rk = r as isize - k as isize;
                let pk = (p - k) as isize;
                if r >= k {
                    a[s2][0] = a[s1][0] / ndu[(pk + 1) as usize][rk as usize];
                    d = a[s2][0] * ndu[rk as usize][pk as usize];
                }
                let j1 = if rk >= -1 { 1 } else { (-rk) as usize };
                let j2 = if r as isize - 1 <= pk { k - 1 } else { p - r };
                for j in j1..=j2 {
                    a[s2][j] = (a[s1][j] - a[s1][j - 1])
                        / ndu[(pk + 1) as usize][(rk + j as isize) as usize];
                    d += a[s2][j] * ndu[(rk + j as isize) as usize][pk as usize];
                }
                if r as isize <= pk {
                    a[s2][k] = -a[s1][k - 1] / ndu[(pk + 1) as usize][r];
                    d += a[s2][k] * ndu[r][pk as usize];
                }
                ders[k][r] = d;
                std::mem::swap(&mut s1, &mut s2);
            }
        }

        // Multiply row k by p! / (p - k)!.
        let mut factor = p as f64;
        for (k, row) in ders.iter_mut().enumerate().take(max_k + 1).skip(1) {
            for value in row.iter_mut() {
                *value *= factor;
            }
            factor *= (p - k) as f64;
        }
        ders
    }
}

/// Non-uniform rational B-spline curve in 3D.
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsCurve {
    control_points: Vec<Point3>,
    weights: Vec<f64>,
    knots: KnotVector,
}

impl NurbsCurve {
    /// Rational curve from weighted control points.
    pub fn new(
        control_points: Vec<Point3>,
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
        if let Some(index) = weights.iter().position(|&w| w <= 0.0) {
            return Err(NurbsError::NonPositiveWeight { index });
        }
        Ok(Self {
            control_points,
            weights,
            knots,
        })
    }

    /// Non-rational (all weights 1) B-spline curve.
    pub fn bspline(control_points: Vec<Point3>, knots: KnotVector) -> Result<Self, NurbsError> {
        let weights = vec![1.0; control_points.len()];
        Self::new(control_points, weights, knots)
    }

    pub fn control_points(&self) -> &[Point3] {
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

    /// Homogeneous control point `(w·P, w)` at `index`.
    fn homogeneous(&self, index: usize) -> Vector4<f64> {
        let p = &self.control_points[index];
        let w = self.weights[index];
        Vector4::new(w * p.x, w * p.y, w * p.z, w)
    }

    /// Derivatives of the curve with respect to `t`, orders `0..=order`
    /// (`result[0]` is the position as a vector from the origin). Rational
    /// derivatives via the quotient rule on the homogeneous curve (A4.2).
    /// `t` is clamped to the domain.
    pub fn derivatives(&self, t: f64, order: usize) -> Vec<Vector3> {
        let (t0, t1) = self.knots.domain();
        let u = t.clamp(t0, t1);
        let p = self.degree();
        let span = self.knots.find_span(u);
        let basis_ders = self.knots.ders_basis_funs(span, u, order);

        // Homogeneous curve derivatives A^(k) = (vec, w) parts.
        let mut homo: Vec<Vector4<f64>> = Vec::with_capacity(order + 1);
        for row in basis_ders.iter() {
            let mut sum = Vector4::zeros();
            for (j, &value) in row.iter().enumerate() {
                sum += self.homogeneous(span - p + j) * value;
            }
            homo.push(sum);
        }

        // C^(k) = (A^(k) - Σ_{i=1..k} C(k,i)·w^(i)·C^(k-i)) / w.
        let mut ders: Vec<Vector3> = Vec::with_capacity(order + 1);
        for (k, a) in homo.iter().enumerate() {
            let mut v = a.xyz();
            for i in 1..=k {
                v -= binomial(k, i) * homo[i].w * ders[k - i];
            }
            ders.push(v / homo[0].w);
        }
        ders
    }

    /// Insert `u` once into the knot vector (Boehm's algorithm, A5.1),
    /// returning a curve with one more control point that traces the same
    /// locus. `u` must lie strictly inside the domain and its resulting
    /// multiplicity must not exceed the degree.
    pub fn insert_knot(&self, u: f64) -> Result<NurbsCurve, NurbsError> {
        let p = self.degree();
        let (t0, t1) = self.knots.domain();
        if !(u > t0 && u < t1) {
            return Err(NurbsError::KnotOutOfDomain {
                knot: u,
                start: t0,
                end: t1,
            });
        }
        let s = self.knots.multiplicity(u);
        if s >= p {
            return Err(NurbsError::MultiplicityExceedsDegree { knot: u, degree: p });
        }
        let k = self.knots.find_span(u);
        let knots = self.knots.knots();
        let n = self.control_points.len() - 1;

        let mut new_homo: Vec<Vector4<f64>> = Vec::with_capacity(n + 2);
        for i in 0..=(k - p) {
            new_homo.push(self.homogeneous(i));
        }
        for i in (k - p + 1)..=(k - s) {
            let alpha = (u - knots[i]) / (knots[i + p] - knots[i]);
            new_homo.push(self.homogeneous(i) * alpha + self.homogeneous(i - 1) * (1.0 - alpha));
        }
        for i in (k - s + 1)..=(n + 1) {
            new_homo.push(self.homogeneous(i - 1));
        }

        let mut new_knots = knots.to_vec();
        new_knots.insert(k + 1, u);

        let mut control_points = Vec::with_capacity(new_homo.len());
        let mut weights = Vec::with_capacity(new_homo.len());
        for h in &new_homo {
            let w = h.w;
            control_points.push(Point3::new(h.x / w, h.y / w, h.z / w));
            weights.push(w);
        }
        NurbsCurve::new(control_points, weights, KnotVector::new(p, new_knots)?)
    }
}

impl CurveEval for NurbsCurve {
    fn point(&self, t: f64) -> Point3 {
        let (t0, t1) = self.knots.domain();
        let u = t.clamp(t0, t1);
        let p = self.degree();
        let span = self.knots.find_span(u);
        let basis = self.knots.basis_funs(span, u);
        let mut sum = Vector4::zeros();
        for (j, &value) in basis.iter().enumerate() {
            sum += self.homogeneous(span - p + j) * value;
        }
        Point3::new(sum.x / sum.w, sum.y / sum.w, sum.z / sum.w)
    }

    fn derivative(&self, t: f64) -> Vector3 {
        self.derivatives(t, 1)[1]
    }

    fn second_derivative(&self, t: f64) -> Vector3 {
        self.derivatives(t, 2)[2]
    }

    fn domain(&self) -> (f64, f64) {
        self.knots.domain()
    }

    fn is_closed(&self) -> bool {
        let (t0, t1) = self.knots.domain();
        (self.point(t0) - self.point(t1)).norm() < 1e-9
    }

    fn is_periodic(&self) -> bool {
        // Clamped representation: geometrically closed curves are still
        // evaluated over a single pass of the domain.
        false
    }
}

fn binomial(n: usize, k: usize) -> f64 {
    let k = k.min(n - k);
    let mut result = 1.0;
    for i in 0..k {
        result = result * (n - i) as f64 / (i + 1) as f64;
    }
    result
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::curve::Curve3;
    use std::f64::consts::FRAC_1_SQRT_2;

    const TIGHT: f64 = 1e-12;

    /// Exact unit circle in the XY plane: rational quadratic, nine control
    /// points over four 90° arcs (Piegl & Tiller §7.5).
    fn unit_circle() -> NurbsCurve {
        let pts = vec![
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(-1.0, 1.0, 0.0),
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(0.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
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
        NurbsCurve::new(pts, weights, knots).unwrap()
    }

    /// Generic rational cubic used for derivative and insertion tests.
    fn generic_rational_cubic() -> NurbsCurve {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.5),
            Point3::new(3.0, 2.5, -1.0),
            Point3::new(4.0, 0.0, 2.0),
            Point3::new(5.0, -1.5, 1.0),
            Point3::new(7.0, 1.0, 0.0),
        ];
        let weights = vec![1.0, 0.5, 2.0, 1.5, 0.8, 1.0];
        let knots =
            KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 0.4, 0.7, 1.0, 1.0, 1.0, 1.0]).unwrap();
        NurbsCurve::new(pts, weights, knots).unwrap()
    }

    fn sample_params(count: usize) -> impl Iterator<Item = f64> {
        (0..=count).map(move |i| i as f64 / count as f64)
    }

    // --- KnotVector ---

    #[test]
    fn knot_vector_validation() {
        assert_eq!(
            KnotVector::new(2, vec![0.0, 0.0, 1.0, 1.0]),
            Err(NurbsError::NotEnoughKnots {
                degree: 2,
                expected: 6,
                got: 4
            })
        );
        assert_eq!(
            KnotVector::new(1, vec![0.0, 0.0, 0.5, 0.4, 1.0, 1.0]),
            Err(NurbsError::DecreasingKnots { index: 3 })
        );
        assert_eq!(
            KnotVector::new(1, vec![0.0, 0.0, 0.0, 0.0]),
            Err(NurbsError::DegenerateDomain)
        );
        let kv = KnotVector::new(2, vec![0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0]).unwrap();
        assert_eq!(kv.control_count(), 4);
        assert_eq!(kv.domain(), (0.0, 1.0));
        assert_eq!(kv.multiplicity(0.5), 1);
        assert_eq!(kv.multiplicity(0.0), 3);
    }

    #[test]
    fn clamped_uniform_knots() {
        let kv = KnotVector::clamped_uniform(3, 6).unwrap();
        assert_eq!(
            kv.knots(),
            &[0.0, 0.0, 0.0, 0.0, 1.0 / 3.0, 2.0 / 3.0, 1.0, 1.0, 1.0, 1.0]
        );
        assert_eq!(kv.control_count(), 6);
        assert!(KnotVector::clamped_uniform(3, 3).is_err());
    }

    #[test]
    fn find_span_brackets_parameter() {
        let kv = KnotVector::new(2, vec![0.0, 0.0, 0.0, 0.3, 0.3, 0.7, 1.0, 1.0, 1.0]).unwrap();
        for u in [0.0, 0.1, 0.3, 0.5, 0.7, 0.9, 1.0] {
            let span = kv.find_span(u);
            let knots = kv.knots();
            assert!(knots[span] <= u, "span floor violated at u={u}");
            if u < 1.0 {
                assert!(u < knots[span + 1], "span ceiling violated at u={u}");
            } else {
                // Domain end maps into the last non-empty span.
                assert_eq!(span, kv.control_count() - 1);
            }
        }
    }

    // --- Degree-1 polyline sanity ---

    #[test]
    fn degree_one_polyline_interpolates() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        ];
        let curve = NurbsCurve::bspline(pts, KnotVector::clamped_uniform(1, 3).unwrap()).unwrap();
        // Domain [0,1], vertex at t=0.5.
        assert!((curve.point(0.0) - Point3::new(0.0, 0.0, 0.0)).norm() < TIGHT);
        assert!((curve.point(0.25) - Point3::new(0.5, 0.0, 0.0)).norm() < TIGHT);
        assert!((curve.point(0.5) - Point3::new(1.0, 0.0, 0.0)).norm() < TIGHT);
        assert!((curve.point(0.75) - Point3::new(1.0, 1.0, 0.0)).norm() < TIGHT);
        assert!((curve.point(1.0) - Point3::new(1.0, 2.0, 0.0)).norm() < TIGHT);
        // Each half of the domain covers one segment: dC/dt = ΔP / Δt.
        assert!((curve.derivative(0.25) - Vector3::new(2.0, 0.0, 0.0)).norm() < TIGHT);
        assert!((curve.derivative(0.75) - Vector3::new(0.0, 4.0, 0.0)).norm() < TIGHT);
        assert!(!curve.is_closed());
        assert!(!curve.is_periodic());
    }

    // --- NURBS circle vs analytic circle ---

    #[test]
    fn nurbs_circle_is_exact() {
        let circle = unit_circle();
        for t in sample_params(200) {
            let p = circle.point(t);
            assert!(
                (p.coords.norm() - 1.0).abs() < TIGHT,
                "radius drift {} at t={t}",
                (p.coords.norm() - 1.0).abs()
            );
            assert!(p.z.abs() < TIGHT, "off plane at t={t}");
        }
    }

    #[test]
    fn nurbs_circle_matches_analytic_circle() {
        let nurbs = unit_circle();
        let analytic = Curve3::circle(Point3::origin(), Vector3::z(), 1.0);
        for t in sample_params(100) {
            let p = nurbs.point(t);
            // Recover the angle and compare against the analytic evaluation.
            let theta = p.y.atan2(p.x);
            let q = analytic.point(theta);
            assert!(
                (p - q).norm() < TIGHT,
                "mismatch at t={t}: nurbs {p:?} vs analytic {q:?}"
            );
        }
    }

    #[test]
    fn nurbs_circle_quarter_points() {
        let circle = unit_circle();
        let quarters = [
            (0.0, Point3::new(1.0, 0.0, 0.0)),
            (0.25, Point3::new(0.0, 1.0, 0.0)),
            (0.5, Point3::new(-1.0, 0.0, 0.0)),
            (0.75, Point3::new(0.0, -1.0, 0.0)),
            (1.0, Point3::new(1.0, 0.0, 0.0)),
        ];
        for (t, expected) in quarters {
            assert!((circle.point(t) - expected).norm() < TIGHT, "at t={t}");
        }
    }

    #[test]
    fn nurbs_circle_tangents_and_closure() {
        let circle = unit_circle();
        for t in [0.05, 0.2, 0.4, 0.6, 0.85] {
            let radial = circle.point(t).coords;
            let tangent = circle.derivative(t);
            assert!(
                radial.dot(&tangent).abs() < 1e-10,
                "tangent not perpendicular to radius at t={t}"
            );
            assert!(tangent.norm() > 0.0);
            // Counterclockwise: (r × dr).z > 0 in the XY plane.
            assert!(radial.cross(&tangent).z > 0.0);
        }
        assert!(circle.is_closed());
        assert!(!circle.is_periodic());
    }

    // --- Derivatives against finite differences ---

    fn check_derivatives_numerically(curve: &NurbsCurve, t: f64) {
        let h = 1e-6;
        let ders = curve.derivatives(t, 2);
        let fd1 = (curve.point(t + h) - curve.point(t - h)) / (2.0 * h);
        assert!(
            (fd1 - ders[1]).norm() < 1e-5,
            "first derivative mismatch at t={t}: {:?} vs fd {:?}",
            ders[1],
            fd1
        );
        let fd2 = (curve.derivative(t + h) - curve.derivative(t - h)) / (2.0 * h);
        assert!(
            (fd2 - ders[2]).norm() < 1e-4,
            "second derivative mismatch at t={t}: {:?} vs fd {:?}",
            ders[2],
            fd2
        );
    }

    #[test]
    fn rational_derivatives_match_finite_differences() {
        // Parameters chosen away from interior knots: central differences
        // lose an order of accuracy where the third derivative jumps.
        let curve = generic_rational_cubic();
        for t in [0.1, 0.25, 0.35, 0.55, 0.65, 0.9] {
            check_derivatives_numerically(&curve, t);
        }
        let circle = unit_circle();
        for t in [0.1, 0.3, 0.6, 0.9] {
            check_derivatives_numerically(&circle, t);
        }
    }

    #[test]
    fn second_derivative_continuous_at_simple_knot() {
        // A cubic with a simple interior knot is C² there: the second
        // derivative evaluated at the knot must agree with one-sided limits.
        let curve = generic_rational_cubic();
        let at = curve.derivatives(0.4, 2)[2];
        let e = 1e-9;
        let left = curve.derivatives(0.4 - e, 2)[2];
        let right = curve.derivatives(0.4 + e, 2)[2];
        assert!((at - left).norm() < 1e-5, "left limit {left:?} vs {at:?}");
        assert!(
            (at - right).norm() < 1e-5,
            "right limit {right:?} vs {at:?}"
        );
    }

    #[test]
    fn derivatives_order_zero_is_position() {
        let curve = generic_rational_cubic();
        let ders = curve.derivatives(0.37, 0);
        assert_eq!(ders.len(), 1);
        assert!((Point3::from(ders[0]) - curve.point(0.37)).norm() < TIGHT);
    }

    #[test]
    fn clamped_endpoint_behavior() {
        let curve = generic_rational_cubic();
        let pts = curve.control_points();
        assert!((curve.point(0.0) - pts[0]).norm() < TIGHT);
        assert!((curve.point(1.0) - pts[pts.len() - 1]).norm() < TIGHT);
        // Out-of-domain parameters clamp instead of extrapolating.
        assert!((curve.point(-3.0) - curve.point(0.0)).norm() < TIGHT);
        assert!((curve.point(9.0) - curve.point(1.0)).norm() < TIGHT);
        // Clamped endpoint tangent points along the first control leg.
        let tangent = curve.derivative(0.0).normalize();
        let leg = (pts[1] - pts[0]).normalize();
        assert!((tangent - leg).norm() < 1e-10);
    }

    // --- Constructor validation ---

    #[test]
    fn curve_constructor_validation() {
        let kv = KnotVector::clamped_uniform(2, 4).unwrap();
        let pts3 = vec![Point3::origin(); 3];
        assert_eq!(
            NurbsCurve::bspline(pts3, kv.clone()),
            Err(NurbsError::ControlCountMismatch {
                control_points: 3,
                expected: 4
            })
        );
        let pts4 = vec![Point3::origin(); 4];
        assert_eq!(
            NurbsCurve::new(pts4.clone(), vec![1.0; 3], kv.clone()),
            Err(NurbsError::WeightCountMismatch {
                weights: 3,
                control_points: 4
            })
        );
        assert_eq!(
            NurbsCurve::new(pts4, vec![1.0, -0.5, 1.0, 1.0], kv),
            Err(NurbsError::NonPositiveWeight { index: 1 })
        );
    }

    // --- Knot insertion ---

    #[test]
    fn knot_insertion_preserves_curve() {
        let curve = generic_rational_cubic();
        let refined = curve.insert_knot(0.3).unwrap();
        assert_eq!(
            refined.control_points().len(),
            curve.control_points().len() + 1
        );
        assert_eq!(
            refined.knot_vector().knots().len(),
            curve.knot_vector().knots().len() + 1
        );
        assert_eq!(refined.knot_vector().multiplicity(0.3), 1);
        for t in sample_params(100) {
            assert!(
                (curve.point(t) - refined.point(t)).norm() < TIGHT,
                "insertion changed curve at t={t}"
            );
            assert!(
                (curve.derivative(t) - refined.derivative(t)).norm() < 1e-9,
                "insertion changed derivative at t={t}"
            );
        }
    }

    #[test]
    fn knot_insertion_preserves_rational_circle() {
        let circle = unit_circle();
        let refined = circle.insert_knot(0.1).unwrap().insert_knot(0.6).unwrap();
        for t in sample_params(100) {
            assert!(
                (circle.point(t) - refined.point(t)).norm() < TIGHT,
                "insertion changed circle at t={t}"
            );
        }
    }

    #[test]
    fn knot_insertion_at_existing_knot_raises_multiplicity() {
        let curve = generic_rational_cubic();
        let once = curve.insert_knot(0.4).unwrap();
        assert_eq!(once.knot_vector().multiplicity(0.4), 2);
        let twice = once.insert_knot(0.4).unwrap();
        assert_eq!(twice.knot_vector().multiplicity(0.4), 3);
        for t in sample_params(100) {
            assert!((curve.point(t) - twice.point(t)).norm() < TIGHT);
        }
        // Multiplicity now equals the degree; one more insertion must fail.
        assert_eq!(
            twice.insert_knot(0.4),
            Err(NurbsError::MultiplicityExceedsDegree {
                knot: 0.4,
                degree: 3
            })
        );
    }

    #[test]
    fn knot_insertion_rejects_out_of_domain() {
        let curve = generic_rational_cubic();
        for u in [-0.5, 0.0, 1.0, 1.5] {
            assert_eq!(
                curve.insert_knot(u),
                Err(NurbsError::KnotOutOfDomain {
                    knot: u,
                    start: 0.0,
                    end: 1.0
                })
            );
        }
    }

    // --- binomial helper ---

    #[test]
    fn binomial_coefficients() {
        assert_eq!(binomial(0, 0), 1.0);
        assert_eq!(binomial(2, 1), 2.0);
        assert_eq!(binomial(4, 2), 6.0);
        assert_eq!(binomial(5, 0), 1.0);
        assert_eq!(binomial(5, 5), 1.0);
    }
}
