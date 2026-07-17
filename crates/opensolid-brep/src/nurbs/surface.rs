//! NURBS surfaces: tensor-product rational evaluation and derivatives.
//!
//! A [`NurbsSurface`] is the tensor product of two [`KnotVector`]s over a
//! rectangular grid of weighted control points. Evaluation follows Piegl &
//! Tiller, *The NURBS Book* (2nd ed.): surface point (A3.5/A4.3) and
//! rational derivatives via the two-parameter quotient rule (Eq. 4.20,
//! A4.4). Basis functions are shared with the curve module
//! ([`KnotVector::basis_funs`] / [`KnotVector::ders_basis_funs`]).
//!
//! Rational surfaces are handled in homogeneous coordinates: control point
//! `P_ij` with weight `w_ij` maps to `(w_ij·P_ij, w_ij)` in 4D and
//! evaluation projects back through the weight component.
//!
//! Evaluation outside the knot domains clamps both parameters (clamped
//! surfaces do not extrapolate).

use crate::nurbs::curve::{KnotVector, NurbsError, binomial};
use crate::surface::SurfaceEval;
use nalgebra::Vector4;
use opensolid_core::SYSTEM_RESOLUTION;
use opensolid_core::types::{BoundingBox3, Point3, Vector3};

/// Non-uniform rational B-spline surface in 3D.
///
/// Control points form a grid indexed `[i][j]` where `i` runs along `u`
/// (rows, paired with the u knot vector) and `j` along `v` (columns, paired
/// with the v knot vector).
#[derive(Debug, Clone, PartialEq)]
pub struct NurbsSurface {
    /// Row-major grid: entry `(i, j)` lives at `i * count_v + j`.
    control_points: Vec<Point3>,
    weights: Vec<f64>,
    knots_u: KnotVector,
    knots_v: KnotVector,
}

impl NurbsSurface {
    /// Rational surface from a grid of weighted control points.
    /// `control_points[i][j]` pairs with `weights[i][j]`; the grid must be
    /// rectangular with `knots_u.control_count()` rows of
    /// `knots_v.control_count()` points, and all weights positive.
    pub fn new(
        control_points: Vec<Vec<Point3>>,
        weights: Vec<Vec<f64>>,
        knots_u: KnotVector,
        knots_v: KnotVector,
    ) -> Result<Self, NurbsError> {
        let rows = knots_u.control_count();
        let cols = knots_v.control_count();
        if control_points.len() != rows {
            return Err(NurbsError::GridRowCountMismatch {
                got: control_points.len(),
                expected: rows,
            });
        }
        for (row, points) in control_points.iter().enumerate() {
            if points.len() != cols {
                return Err(NurbsError::GridColumnCountMismatch {
                    row,
                    got: points.len(),
                    expected: cols,
                });
            }
        }
        if weights.len() != rows {
            return Err(NurbsError::WeightGridShapeMismatch { row: weights.len() });
        }
        for (row, row_weights) in weights.iter().enumerate() {
            if row_weights.len() != cols {
                return Err(NurbsError::WeightGridShapeMismatch { row });
            }
        }
        let flat_weights: Vec<f64> = weights.into_iter().flatten().collect();
        if let Some(index) = flat_weights.iter().position(|&w| w <= 0.0) {
            return Err(NurbsError::NonPositiveWeight { index });
        }
        Ok(Self {
            control_points: control_points.into_iter().flatten().collect(),
            weights: flat_weights,
            knots_u,
            knots_v,
        })
    }

    /// Non-rational (all weights 1) B-spline surface.
    pub fn bspline(
        control_points: Vec<Vec<Point3>>,
        knots_u: KnotVector,
        knots_v: KnotVector,
    ) -> Result<Self, NurbsError> {
        let weights = control_points
            .iter()
            .map(|row| vec![1.0; row.len()])
            .collect();
        Self::new(control_points, weights, knots_u, knots_v)
    }

    /// Control point at grid position `(i, j)`.
    pub fn control_point(&self, i: usize, j: usize) -> Point3 {
        self.control_points[i * self.knots_v.control_count() + j]
    }

    /// Weight at grid position `(i, j)`.
    pub fn weight(&self, i: usize, j: usize) -> f64 {
        self.weights[i * self.knots_v.control_count() + j]
    }

    pub fn knot_vector_u(&self) -> &KnotVector {
        &self.knots_u
    }

    pub fn knot_vector_v(&self) -> &KnotVector {
        &self.knots_v
    }

    pub fn degree_u(&self) -> usize {
        self.knots_u.degree()
    }

    pub fn degree_v(&self) -> usize {
        self.knots_v.degree()
    }

    /// Number of control-point rows (`u` direction) and columns (`v`).
    pub fn grid_size(&self) -> (usize, usize) {
        (self.knots_u.control_count(), self.knots_v.control_count())
    }

    /// Map every control point through `f`, leaving weights and knots
    /// untouched.
    ///
    /// For an **affine** `f` this transforms the surface exactly:
    /// evaluation is a weighted average of control points whose basis
    /// weights sum to one, and affine maps commute with such averages. It
    /// is *not* valid for a projective or otherwise non-affine `f`, which
    /// would have to move the weights too.
    pub fn map_control_points(&mut self, f: impl Fn(Point3) -> Point3) {
        for p in &mut self.control_points {
            *p = f(*p);
        }
    }

    /// Axis-aligned box of the control hull.
    ///
    /// By the convex hull property of (rational, positive-weight) NURBS the
    /// patch lies inside the convex hull of its control points, so this box
    /// contains the whole surface. It is a *bound*, not the tight box of
    /// the geometry, which is exactly what broad-phase culling needs: never
    /// too tight. `None` only for an empty grid, which the constructors
    /// reject.
    pub fn control_hull_box(&self) -> Option<BoundingBox3> {
        let mut points = self.control_points.iter();
        let first = *points.next()?;
        let (mut lo, mut hi) = (first, first);
        for p in points {
            lo = Point3::new(lo.x.min(p.x), lo.y.min(p.y), lo.z.min(p.z));
            hi = Point3::new(hi.x.max(p.x), hi.y.max(p.y), hi.z.max(p.z));
        }
        Some(BoundingBox3::new(lo, hi))
    }

    /// Whether any point of the patch is a parameterization singularity — a
    /// collapsed control row/column (the classic lofted-to-a-point tip),
    /// where `|S_u × S_v|` vanishes and no limit normal exists.
    ///
    /// Tested at the four domain corners and the midpoint of each domain
    /// edge, which is where a collapsed boundary row shows up. The chart
    /// rejects such patches (they have no pole machinery analogue), so this
    /// is a conservative gate, not a geometric classification.
    pub fn has_degenerate_edge(&self) -> bool {
        let (u0, u1) = self.knots_u.domain();
        let (v0, v1) = self.knots_v.domain();
        let (um, vm) = (0.5 * (u0 + u1), 0.5 * (v0 + v1));
        [
            (u0, v0),
            (u0, vm),
            (u0, v1),
            (um, v0),
            (um, v1),
            (u1, v0),
            (u1, vm),
            (u1, v1),
        ]
        .into_iter()
        .any(|(u, v)| self.is_singular(u, v))
    }

    /// Homogeneous control point `(w·P, w)` at grid position `(i, j)`.
    fn homogeneous(&self, i: usize, j: usize) -> Vector4<f64> {
        let index = i * self.knots_v.control_count() + j;
        let p = &self.control_points[index];
        let w = self.weights[index];
        Vector4::new(w * p.x, w * p.y, w * p.z, w)
    }

    /// Clamp `(u, v)` to the knot domains.
    fn clamp_params(&self, u: f64, v: f64) -> (f64, f64) {
        let (u0, u1) = self.knots_u.domain();
        let (v0, v1) = self.knots_v.domain();
        (u.clamp(u0, u1), v.clamp(v0, v1))
    }

    /// All partial derivatives `∂^{k+l} S / ∂u^k ∂v^l` for
    /// `k, l ∈ 0..=order` (`result[k][l]`; `result[0][0]` is the position as
    /// a vector from the origin). Rational derivatives via the tensor
    /// quotient rule on the homogeneous surface (Eq. 4.20, A4.4). `(u, v)`
    /// are clamped to the domain.
    pub fn derivatives(&self, u: f64, v: f64, order: usize) -> Vec<Vec<Vector3>> {
        let (u, v) = self.clamp_params(u, v);
        let p = self.knots_u.degree();
        let q = self.knots_v.degree();
        let span_u = self.knots_u.find_span(u);
        let span_v = self.knots_v.find_span(v);
        let ders_u = self.knots_u.ders_basis_funs(span_u, u, order);
        let ders_v = self.knots_v.ders_basis_funs(span_v, v, order);

        // Homogeneous surface derivatives A^(k,l) = (vec, w) parts.
        let mut homo = vec![vec![Vector4::zeros(); order + 1]; order + 1];
        for (k, row_u) in ders_u.iter().enumerate() {
            for (l, row_v) in ders_v.iter().enumerate() {
                let mut sum = Vector4::zeros();
                for (i, &nu) in row_u.iter().enumerate() {
                    for (j, &nv) in row_v.iter().enumerate() {
                        sum += self.homogeneous(span_u - p + i, span_v - q + j) * (nu * nv);
                    }
                }
                homo[k][l] = sum;
            }
        }

        // S^(k,l) = (A^(k,l) - Σ_i C(k,i)·w^(i,0)·S^(k-i,l)
        //                    - Σ_j C(l,j)·w^(0,j)·S^(k,l-j)
        //                    - Σ_i Σ_j C(k,i)·C(l,j)·w^(i,j)·S^(k-i,l-j)) / w
        // with sums over i in 1..=k, j in 1..=l.
        let w = homo[0][0].w;
        let mut ders = vec![vec![Vector3::zeros(); order + 1]; order + 1];
        for k in 0..=order {
            for l in 0..=order {
                let mut value = homo[k][l].xyz();
                for i in 1..=k {
                    value -= binomial(k, i) * homo[i][0].w * ders[k - i][l];
                }
                for j in 1..=l {
                    value -= binomial(l, j) * homo[0][j].w * ders[k][l - j];
                }
                for i in 1..=k {
                    let bk = binomial(k, i);
                    for j in 1..=l {
                        value -= bk * binomial(l, j) * homo[i][j].w * ders[k - i][l - j];
                    }
                }
                ders[k][l] = value / w;
            }
        }
        ders
    }
}

impl SurfaceEval for NurbsSurface {
    fn point(&self, u: f64, v: f64) -> Point3 {
        let (u, v) = self.clamp_params(u, v);
        let p = self.knots_u.degree();
        let q = self.knots_v.degree();
        let span_u = self.knots_u.find_span(u);
        let span_v = self.knots_v.find_span(v);
        let basis_u = self.knots_u.basis_funs(span_u, u);
        let basis_v = self.knots_v.basis_funs(span_v, v);
        let mut sum = Vector4::zeros();
        for (i, &nu) in basis_u.iter().enumerate() {
            for (j, &nv) in basis_v.iter().enumerate() {
                sum += self.homogeneous(span_u - p + i, span_v - q + j) * (nu * nv);
            }
        }
        Point3::new(sum.x / sum.w, sum.y / sum.w, sum.z / sum.w)
    }

    fn du(&self, u: f64, v: f64) -> Vector3 {
        self.derivatives(u, v, 1)[1][0]
    }

    fn dv(&self, u: f64, v: f64) -> Vector3 {
        self.derivatives(u, v, 1)[0][1]
    }

    fn normal(&self, u: f64, v: f64) -> Option<Vector3> {
        let ders = self.derivatives(u, v, 1);
        let cross = ders[1][0].cross(&ders[0][1]);
        let norm = cross.norm();
        if norm <= SYSTEM_RESOLUTION * ders[1][0].norm() * ders[0][1].norm() {
            // Collapsed edge or parallel partials: no limit normal in
            // general (unlike sphere poles, nothing constrains the
            // neighborhood here).
            return None;
        }
        Some(cross / norm)
    }

    fn domain_u(&self) -> (f64, f64) {
        self.knots_u.domain()
    }

    fn domain_v(&self) -> (f64, f64) {
        self.knots_v.domain()
    }

    fn is_periodic_u(&self) -> bool {
        // Clamped representation: geometrically closed surfaces are still
        // evaluated over a single pass of the domain.
        false
    }

    fn is_periodic_v(&self) -> bool {
        false
    }

    fn is_singular(&self, u: f64, v: f64) -> bool {
        let ders = self.derivatives(u, v, 1);
        let cross = ders[1][0].cross(&ders[0][1]);
        cross.norm() <= SYSTEM_RESOLUTION * ders[1][0].norm() * ders[0][1].norm()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::nurbs::curve::NurbsCurve;
    use crate::surface::Surface3;
    use std::f64::consts::FRAC_1_SQRT_2;

    const TIGHT: f64 = 1e-12;
    /// Tolerance required by the acceptance criteria for the cylinder patch.
    const CYL_TOL: f64 = 1e-10;

    fn sample_params(count: usize) -> impl Iterator<Item = f64> {
        (0..=count).map(move |i| i as f64 / count as f64)
    }

    /// Full-circle control polygon of the exact rational quadratic unit
    /// circle (Piegl & Tiller §7.5), scaled by `radius`, at height `z`.
    fn circle_row(radius: f64, z: f64) -> Vec<Point3> {
        [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.0, 1.0),
            (-1.0, 1.0),
            (-1.0, 0.0),
            (-1.0, -1.0),
            (0.0, -1.0),
            (1.0, -1.0),
            (1.0, 0.0),
        ]
        .iter()
        .map(|&(x, y)| Point3::new(radius * x, radius * y, z))
        .collect()
    }

    fn circle_knots() -> KnotVector {
        KnotVector::new(
            2,
            vec![
                0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
            ],
        )
        .unwrap()
    }

    fn circle_weights() -> Vec<f64> {
        let s = FRAC_1_SQRT_2;
        vec![1.0, s, 1.0, s, 1.0, s, 1.0, s, 1.0]
    }

    /// Exact NURBS cylinder of radius 1 about the z-axis, `v ∈ [0, 1]`
    /// mapping to `z ∈ [0, 2]`: rational quadratic circle in `u`, linear
    /// ruling in `v`. Control grid is 9 (u) × 2 (v).
    const CYL_HEIGHT: f64 = 2.0;

    fn cylinder_patch() -> NurbsSurface {
        let control_points: Vec<Vec<Point3>> = circle_row(1.0, 0.0)
            .into_iter()
            .zip(circle_row(1.0, CYL_HEIGHT))
            .map(|(bottom, top)| vec![bottom, top])
            .collect();
        let weights = circle_weights().iter().map(|&w| vec![w, w]).collect();
        let knots_v = KnotVector::clamped_uniform(1, 2).unwrap();
        NurbsSurface::new(control_points, weights, circle_knots(), knots_v).unwrap()
    }

    /// Generic rational biquadratic patch with varied weights and
    /// non-planar control points, for derivative cross-checks.
    fn generic_rational_patch() -> NurbsSurface {
        let control_points = vec![
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.4),
                Point3::new(0.0, 2.0, 0.1),
                Point3::new(0.0, 3.0, -0.3),
            ],
            vec![
                Point3::new(1.0, 0.0, 0.8),
                Point3::new(1.0, 1.0, 1.6),
                Point3::new(1.0, 2.0, 1.1),
                Point3::new(1.2, 3.0, 0.5),
            ],
            vec![
                Point3::new(2.0, 0.0, 0.3),
                Point3::new(2.0, 1.0, 0.9),
                Point3::new(2.2, 2.0, 1.4),
                Point3::new(2.0, 3.0, 0.0),
            ],
            vec![
                Point3::new(3.0, 0.0, -0.2),
                Point3::new(3.0, 1.2, 0.2),
                Point3::new(3.0, 2.0, 0.6),
                Point3::new(3.0, 3.0, 0.4),
            ],
        ];
        let weights = vec![
            vec![1.0, 0.7, 1.3, 1.0],
            vec![0.9, 2.0, 0.6, 1.1],
            vec![1.4, 0.8, 1.7, 0.9],
            vec![1.0, 1.2, 0.75, 1.0],
        ];
        let knots_u = KnotVector::clamped_uniform(2, 4).unwrap();
        let knots_v = KnotVector::clamped_uniform(2, 4).unwrap();
        NurbsSurface::new(control_points, weights, knots_u, knots_v).unwrap()
    }

    fn bilinear_patch() -> NurbsSurface {
        // Non-planar quad: bilinear interpolation of the four corners.
        let control_points = vec![
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 1.0, 1.0)],
            vec![Point3::new(2.0, 0.0, 0.0), Point3::new(2.0, 1.0, 3.0)],
        ];
        let knots = KnotVector::clamped_uniform(1, 2).unwrap();
        NurbsSurface::bspline(control_points, knots.clone(), knots).unwrap()
    }

    // --- Cylinder patch vs analytic cylinder ---

    #[test]
    fn cylinder_patch_matches_analytic_cylinder() {
        let surface = cylinder_patch();
        let cylinder = Surface3::cylinder(Point3::origin(), Vector3::z(), 1.0)
            .expect("unit cylinder construction is valid");
        for u in sample_params(50) {
            for v in sample_params(10) {
                let p = surface.point(u, v);
                // Exact rational circle: unit distance from the axis, height
                // linear in v.
                assert!(
                    ((p.x * p.x + p.y * p.y).sqrt() - 1.0).abs() < CYL_TOL,
                    "radius drift at (u={u}, v={v})"
                );
                assert!(
                    (p.z - CYL_HEIGHT * v).abs() < CYL_TOL,
                    "height drift at (u={u}, v={v})"
                );
                // Same physical point on the analytic cylinder.
                let theta = p.y.atan2(p.x);
                let q = cylinder.point(theta, p.z);
                assert!(
                    (p - q).norm() < CYL_TOL,
                    "mismatch at (u={u}, v={v}): nurbs {p:?} vs analytic {q:?}"
                );
                // Outward radial normal, matching the analytic surface.
                let n = surface.normal(u, v).expect("cylinder is regular");
                let expected = cylinder.normal(theta, p.z).unwrap();
                assert!(
                    (n - expected).norm() < CYL_TOL,
                    "normal mismatch at (u={u}, v={v}): {n:?} vs {expected:?}"
                );
            }
        }
    }

    #[test]
    fn cylinder_patch_boundary_matches_nurbs_circle() {
        // The v=0 isocurve must reproduce the NURBS circle built from the
        // same row of control points: shared basis code, same locus.
        let surface = cylinder_patch();
        let circle =
            NurbsCurve::new(circle_row(1.0, 0.0), circle_weights(), circle_knots()).unwrap();
        use crate::curve::CurveEval;
        for u in sample_params(100) {
            assert!(
                (surface.point(u, 0.0) - circle.point(u)).norm() < TIGHT,
                "boundary isocurve deviates at u={u}"
            );
        }
    }

    // --- Bilinear patch ---

    #[test]
    fn bilinear_patch_interpolates_corners() {
        let patch = bilinear_patch();
        assert!((patch.point(0.0, 0.0) - patch.control_point(0, 0)).norm() < TIGHT);
        assert!((patch.point(1.0, 0.0) - patch.control_point(1, 0)).norm() < TIGHT);
        assert!((patch.point(0.0, 1.0) - patch.control_point(0, 1)).norm() < TIGHT);
        assert!((patch.point(1.0, 1.0) - patch.control_point(1, 1)).norm() < TIGHT);
    }

    #[test]
    fn bilinear_patch_degenerates_to_bilinear_interpolation() {
        let patch = bilinear_patch();
        let p00 = patch.control_point(0, 0).coords;
        let p10 = patch.control_point(1, 0).coords;
        let p01 = patch.control_point(0, 1).coords;
        let p11 = patch.control_point(1, 1).coords;
        for u in sample_params(10) {
            for v in sample_params(10) {
                let expected = p00 * (1.0 - u) * (1.0 - v)
                    + p10 * u * (1.0 - v)
                    + p01 * (1.0 - u) * v
                    + p11 * u * v;
                assert!(
                    (patch.point(u, v).coords - expected).norm() < TIGHT,
                    "bilinear mismatch at (u={u}, v={v})"
                );
                // Exact analytic partials of the bilinear map.
                let du_expected = (p10 - p00) * (1.0 - v) + (p11 - p01) * v;
                let dv_expected = (p01 - p00) * (1.0 - u) + (p11 - p10) * u;
                assert!((patch.du(u, v) - du_expected).norm() < TIGHT);
                assert!((patch.dv(u, v) - dv_expected).norm() < TIGHT);
            }
        }
    }

    #[test]
    fn planar_bilinear_patch_has_constant_normal() {
        let control_points = vec![
            vec![Point3::new(0.0, 0.0, 1.0), Point3::new(0.0, 2.0, 1.0)],
            vec![Point3::new(3.0, 0.0, 1.0), Point3::new(3.0, 2.0, 1.0)],
        ];
        let knots = KnotVector::clamped_uniform(1, 2).unwrap();
        let patch = NurbsSurface::bspline(control_points, knots.clone(), knots).unwrap();
        for u in sample_params(5) {
            for v in sample_params(5) {
                let n = patch.normal(u, v).expect("planar patch is regular");
                assert!((n - Vector3::z()).norm() < TIGHT, "at (u={u}, v={v})");
            }
        }
    }

    // --- Derivatives against finite differences ---

    #[test]
    fn partials_match_finite_differences() {
        let patch = generic_rational_patch();
        let h = 1e-6;
        // Parameters away from the knot lines at u = v = 0.5, where the
        // biquadratic is only C¹ and central differences of the first
        // partials straddle a second-derivative jump.
        for &u in &[0.1, 0.25, 0.4, 0.75, 0.9] {
            for &v in &[0.15, 0.4, 0.55, 0.8] {
                let ders = patch.derivatives(u, v, 2);
                let fd_du = (patch.point(u + h, v) - patch.point(u - h, v)) / (2.0 * h);
                let fd_dv = (patch.point(u, v + h) - patch.point(u, v - h)) / (2.0 * h);
                assert!(
                    (fd_du - ders[1][0]).norm() < 1e-5,
                    "du mismatch at (u={u}, v={v}): {:?} vs fd {:?}",
                    ders[1][0],
                    fd_du
                );
                assert!(
                    (fd_dv - ders[0][1]).norm() < 1e-5,
                    "dv mismatch at (u={u}, v={v}): {:?} vs fd {:?}",
                    ders[0][1],
                    fd_dv
                );
                // Second and mixed partials from central differences of the
                // first partials.
                let fd_duu = (patch.du(u + h, v) - patch.du(u - h, v)) / (2.0 * h);
                let fd_dvv = (patch.dv(u, v + h) - patch.dv(u, v - h)) / (2.0 * h);
                let fd_duv = (patch.du(u, v + h) - patch.du(u, v - h)) / (2.0 * h);
                assert!(
                    (fd_duu - ders[2][0]).norm() < 1e-4,
                    "d²/du² mismatch at (u={u}, v={v})"
                );
                assert!(
                    (fd_dvv - ders[0][2]).norm() < 1e-4,
                    "d²/dv² mismatch at (u={u}, v={v})"
                );
                assert!(
                    (fd_duv - ders[1][1]).norm() < 1e-4,
                    "d²/dudv mismatch at (u={u}, v={v})"
                );
            }
        }
    }

    #[test]
    fn rational_cylinder_partials_match_finite_differences() {
        let surface = cylinder_patch();
        let h = 1e-6;
        // Away from the double knots at 0.25, 0.5, 0.75.
        for &u in &[0.1, 0.35, 0.6, 0.9] {
            for &v in &[0.2, 0.5, 0.8] {
                let fd_du = (surface.point(u + h, v) - surface.point(u - h, v)) / (2.0 * h);
                let fd_dv = (surface.point(u, v + h) - surface.point(u, v - h)) / (2.0 * h);
                assert!((fd_du - surface.du(u, v)).norm() < 1e-5);
                assert!((fd_dv - surface.dv(u, v)).norm() < 1e-5);
            }
        }
    }

    #[test]
    fn derivatives_order_zero_is_position() {
        let patch = generic_rational_patch();
        let ders = patch.derivatives(0.3, 0.7, 0);
        assert_eq!(ders.len(), 1);
        assert_eq!(ders[0].len(), 1);
        assert!((Point3::from(ders[0][0]) - patch.point(0.3, 0.7)).norm() < TIGHT);
    }

    // --- Domain, clamping, trait plumbing ---

    #[test]
    fn domain_and_clamping() {
        let patch = generic_rational_patch();
        assert_eq!(patch.domain_u(), (0.0, 1.0));
        assert_eq!(patch.domain_v(), (0.0, 1.0));
        assert!(!patch.is_periodic_u());
        assert!(!patch.is_periodic_v());
        assert_eq!(patch.period_u(), None);
        assert_eq!(patch.period_v(), None);
        // Out-of-domain parameters clamp instead of extrapolating.
        assert!((patch.point(-2.0, -3.0) - patch.point(0.0, 0.0)).norm() < TIGHT);
        assert!((patch.point(5.0, 0.5) - patch.point(1.0, 0.5)).norm() < TIGHT);
        // Clamped corner interpolates the corner control point.
        assert!((patch.point(0.0, 0.0) - patch.control_point(0, 0)).norm() < TIGHT);
        assert!((patch.point(1.0, 1.0) - patch.control_point(3, 3)).norm() < TIGHT);
    }

    #[test]
    fn degrees_and_accessors() {
        let patch = cylinder_patch();
        assert_eq!(patch.degree_u(), 2);
        assert_eq!(patch.degree_v(), 1);
        assert_eq!(patch.knot_vector_u().control_count(), 9);
        assert_eq!(patch.knot_vector_v().control_count(), 2);
        assert_eq!(patch.weight(1, 0), FRAC_1_SQRT_2);
        assert_eq!(
            patch.control_point(4, 1),
            Point3::new(-1.0, 0.0, CYL_HEIGHT)
        );
    }

    // --- Degenerate (collapsed) edges ---

    #[test]
    fn collapsed_edge_is_singular() {
        // Triangle patch: the v=0 edge collapses to a single point, so du
        // vanishes along it and there is no normal.
        let apex = Point3::new(0.0, 0.0, 0.0);
        let control_points = vec![
            vec![apex, Point3::new(0.0, 1.0, 0.0)],
            vec![apex, Point3::new(1.0, 1.0, 0.0)],
        ];
        let knots = KnotVector::clamped_uniform(1, 2).unwrap();
        let patch = NurbsSurface::bspline(control_points, knots.clone(), knots).unwrap();
        assert!(patch.is_singular(0.5, 0.0));
        assert_eq!(patch.normal(0.5, 0.0), None);
        // Away from the collapsed edge the patch is regular.
        assert!(!patch.is_singular(0.5, 0.5));
        assert!(patch.normal(0.5, 0.5).is_some());
    }

    // --- Constructor validation ---

    #[test]
    fn surface_constructor_validation() {
        let knots_u = KnotVector::clamped_uniform(1, 3).unwrap();
        let knots_v = KnotVector::clamped_uniform(1, 2).unwrap();
        let row = |y: f64| vec![Point3::new(0.0, y, 0.0), Point3::new(1.0, y, 0.0)];

        // Wrong number of rows for the u knot vector.
        assert_eq!(
            NurbsSurface::bspline(vec![row(0.0), row(1.0)], knots_u.clone(), knots_v.clone()),
            Err(NurbsError::GridRowCountMismatch {
                got: 2,
                expected: 3
            })
        );
        // Ragged row.
        assert_eq!(
            NurbsSurface::bspline(
                vec![row(0.0), vec![Point3::origin()], row(2.0)],
                knots_u.clone(),
                knots_v.clone()
            ),
            Err(NurbsError::GridColumnCountMismatch {
                row: 1,
                got: 1,
                expected: 2
            })
        );
        let grid = vec![row(0.0), row(1.0), row(2.0)];
        // Weight grid with the wrong number of rows.
        assert_eq!(
            NurbsSurface::new(
                grid.clone(),
                vec![vec![1.0, 1.0]; 2],
                knots_u.clone(),
                knots_v.clone()
            ),
            Err(NurbsError::WeightGridShapeMismatch { row: 2 })
        );
        // Weight row with the wrong length.
        assert_eq!(
            NurbsSurface::new(
                grid.clone(),
                vec![vec![1.0, 1.0], vec![1.0], vec![1.0, 1.0]],
                knots_u.clone(),
                knots_v.clone()
            ),
            Err(NurbsError::WeightGridShapeMismatch { row: 1 })
        );
        // Non-positive weight (flat index 3 = row 1, col 1).
        assert_eq!(
            NurbsSurface::new(
                grid,
                vec![vec![1.0, 1.0], vec![1.0, -2.0], vec![1.0, 1.0]],
                knots_u,
                knots_v
            ),
            Err(NurbsError::NonPositiveWeight { index: 3 })
        );
    }
}
