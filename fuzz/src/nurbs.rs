//! Target 3: NURBS construction and evaluation — knot vectors, rational
//! curves, tensor-product surfaces.
//!
//! Everything the STEP reader imports exactly (`B_SPLINE_CURVE_WITH_KNOTS`,
//! `B_SPLINE_SURFACE_WITH_KNOTS`) lands in these types, straight from an
//! untrusted file, so the constructors are a validation boundary in the same
//! sense the parser is. Downstream — tessellation, projection, SSI marching —
//! assumes evaluation returns finite points.
//!
//! # Generating inputs a fuzzer can actually use
//!
//! Random floats never form a valid knot vector (they must be non-decreasing
//! and long enough for the degree), so purely random bytes would test the
//! validator's rejection path and nothing else. This harness generates knot
//! vectors three ways:
//!
//! * [`KnotSpec::Clamped`] — valid by construction: `degree + 1` repeats at
//!   each end and interior knots built by accumulating non-negative
//!   increments, so repeated increments of zero raise interior multiplicity
//!   (the discontinuity case) on purpose.
//! * [`KnotSpec::Uniform`] — whatever `KnotVector::clamped_uniform` produces.
//! * [`KnotSpec::Raw`] — arbitrary floats straight into `KnotVector::new`,
//!   which is what exercises the validator itself.
//!
//! # Post-conditions
//!
//! *Unconditional* (any input, however extreme):
//!
//! * **No panic, no hang.** Construction either returns `Err` or an object
//!   whose whole evaluation surface — `point`, `derivative`, `derivatives`,
//!   `normal`, `insert_knot`, `reversed`, `control_hull_box` — returns.
//! * **`KnotVector::new` enforces what it documents.** An accepted vector is
//!   non-decreasing, long enough for its degree, and has a non-empty domain.
//!   This is the assertion that catches a validator hole: a NaN knot, for
//!   instance, passes a naive `<` monotonicity scan because every comparison
//!   against NaN is false.
//!
//! *Conditional on a well-formed spec* (finite bounded control points, finite
//! positive weights, finite knots with interior multiplicity at most the
//! degree — see [`WellFormed`]) the harness additionally asserts the
//! mathematical identities that make a NURBS implementation correct:
//!
//! * **Finiteness.** Evaluation and derivatives are finite everywhere in the
//!   domain.
//! * **Convex hull.** A rational curve/surface with positive weights lies in
//!   the convex hull of its control points, so every evaluated point lies
//!   inside `control_hull_box`. This is the single strongest cheap oracle
//!   available for spline evaluation: a wrong span index, a mis-summed basis
//!   or a dropped weight almost always escapes the hull.
//! * **Endpoint interpolation.** A clamped curve starts at its first control
//!   point and ends at its last.
//! * **Domain clamping.** `point` clamps out-of-domain parameters, so
//!   evaluating below/above the domain equals evaluating at the endpoint.
//! * **Knot insertion preserves the locus.** Boehm's algorithm must not move
//!   the curve — the textbook invariant, and the one that catches an
//!   off-by-one in the alpha blend.
//! * **Reversal preserves the locus.** `reversed().point(t0 + t1 - t)` is
//!   `point(t)`.

use arbitrary::{Arbitrary, Unstructured};
use opensolid_brep::{CurveEval, KnotVector, NurbsCurve, NurbsError, NurbsSurface, SurfaceEval};
use opensolid_core::types::Point3;

/// Cap on degree. High degrees cost O(p^2) per evaluation and add nothing:
/// AP203 exports rarely exceed degree 7.
const MAX_DEGREE: usize = 7;
/// Cap on control points per direction, for the same reason.
const MAX_CONTROL: usize = 24;
/// Parameters sampled per curve/surface.
const SAMPLES: usize = 9;

/// Fuzz entry point: see the [module docs](self) for the contract.
pub fn fuzz_nurbs_eval(data: &[u8]) {
    let u = Unstructured::new(data);
    let Ok(case) = Case::arbitrary_take_rest(u) else {
        return;
    };
    run_case(&case);
}

/// One decoded input.
#[derive(Debug, Arbitrary)]
pub enum Case {
    Knots(KnotSpec),
    Curve(CurveSpec),
    Surface(SurfaceSpec),
}

/// How to build a knot vector.
#[derive(Debug, Arbitrary)]
pub enum KnotSpec {
    /// Clamped and valid by construction; `steps` are the interior knot
    /// increments (a zero step repeats the previous knot).
    Clamped { degree: u8, steps: Vec<u8> },
    /// `KnotVector::clamped_uniform`.
    Uniform { degree: u8, control_count: u8 },
    /// Raw floats into the validator.
    Raw { degree: u8, knots: Vec<Coord> },
}

#[derive(Debug, Arbitrary)]
pub struct CurveSpec {
    knots: KnotSpec,
    /// Control point coordinates, cycled to whatever length the knot vector
    /// demands (a spec whose lengths never line up would only ever exercise
    /// `ControlCountMismatch`).
    coords: Vec<Coord>,
    weights: Vec<Weight>,
    /// Parameters to evaluate at, beyond the systematic domain sweep.
    probes: Vec<Coord>,
    /// Knot value to insert, as a fraction of the domain.
    insert_at: u8,
}

#[derive(Debug, Arbitrary)]
pub struct SurfaceSpec {
    knots_u: KnotSpec,
    knots_v: KnotSpec,
    coords: Vec<Coord>,
    weights: Vec<Weight>,
    probes: Vec<(Coord, Coord)>,
}

/// A coordinate or parameter value. Tame values are bounded and finite, which
/// is what the geometric oracles need; the rest exist to prove nothing
/// crashes on them.
#[derive(Debug, Clone, Copy, Arbitrary)]
pub enum Coord {
    /// Bounded and finite: `i16 / 32`, i.e. `[-1024, 1024)` in 1/32 steps.
    Tame(i16),
    Huge(bool),
    Tiny(bool),
    Nan,
    Infinite(bool),
}

impl Coord {
    fn value(self) -> f64 {
        match self {
            Coord::Tame(n) => f64::from(n) / 32.0,
            Coord::Huge(neg) => {
                if neg {
                    -1e300
                } else {
                    1e300
                }
            }
            Coord::Tiny(neg) => {
                if neg {
                    -f64::MIN_POSITIVE
                } else {
                    f64::MIN_POSITIVE
                }
            }
            Coord::Nan => f64::NAN,
            Coord::Infinite(neg) => {
                if neg {
                    f64::NEG_INFINITY
                } else {
                    f64::INFINITY
                }
            }
        }
    }

    /// Whether this value keeps the geometric oracles meaningful.
    fn is_tame(self) -> bool {
        matches!(self, Coord::Tame(_))
    }
}

/// A control point weight. Construction requires strictly positive weights;
/// the invalid variants check that the constructor says so instead of
/// producing a curve that divides by zero.
#[derive(Debug, Clone, Copy, Arbitrary)]
pub enum Weight {
    /// `1 + n/256`, in `[1, 256]` — the range real rational geometry uses.
    Positive(u16),
    /// Denormal-adjacent but still positive.
    Tiny,
    Zero,
    Negative,
    Nan,
    Infinite,
}

impl Weight {
    fn value(self) -> f64 {
        match self {
            Weight::Positive(n) => 1.0 + f64::from(n) / 256.0,
            Weight::Tiny => 1e-300,
            Weight::Zero => 0.0,
            Weight::Negative => -1.0,
            Weight::Nan => f64::NAN,
            Weight::Infinite => f64::INFINITY,
        }
    }

    fn is_tame(self) -> bool {
        matches!(self, Weight::Positive(_))
    }
}

/// Whether a constructed object satisfies the preconditions the mathematical
/// oracles assume. Anything outside this is held only to "does not crash".
struct WellFormed;

impl WellFormed {
    /// A knot vector whose knots are finite, whose interior multiplicity does
    /// not exceed the degree, and whose end multiplicities do not exceed
    /// `degree + 1`. Cox–de Boor is only defined under these conditions;
    /// `KnotVector::new` deliberately accepts a wider set (a raw AP203 knot
    /// list is not required to be clamped), so the harness screens here
    /// rather than asserting the wider set is rejected.
    fn knots(kv: &KnotVector) -> bool {
        let knots = kv.knots();
        let p = kv.degree();
        if !knots.iter().all(|k| k.is_finite()) {
            return false;
        }
        let (t0, t1) = kv.domain();
        if !(t1 - t0).is_finite() || t1 <= t0 {
            return false;
        }
        let mut i = 0;
        while i < knots.len() {
            let mut j = i;
            while j < knots.len() && knots[j] == knots[i] {
                j += 1;
            }
            let multiplicity = j - i;
            let at_end = knots[i] <= t0 || knots[i] >= t1;
            let limit = if at_end { p + 1 } else { p };
            if multiplicity > limit {
                return false;
            }
            i = j;
        }
        true
    }
}

fn run_case(case: &Case) {
    match case {
        Case::Knots(spec) => {
            let _ = build_knots(spec);
        }
        Case::Curve(spec) => run_curve(spec),
        Case::Surface(spec) => run_surface(spec),
    }
}

/// Build a knot vector and assert the invariants `KnotVector` documents for
/// anything it accepts.
fn build_knots(spec: &KnotSpec) -> Option<KnotVector> {
    let result = match spec {
        KnotSpec::Clamped { degree, steps } => {
            let p = usize::from(*degree) % (MAX_DEGREE + 1);
            let interior = steps.len().min(MAX_CONTROL);
            let mut knots = vec![0.0f64; p + 1];
            let mut t = 0.0f64;
            for step in steps.iter().take(interior) {
                t += f64::from(*step) / 64.0;
                knots.push(t);
            }
            let end = t + 1.0;
            knots.extend(std::iter::repeat_n(end, p + 1));
            KnotVector::new(p, knots)
        }
        KnotSpec::Uniform {
            degree,
            control_count,
        } => {
            let p = usize::from(*degree) % (MAX_DEGREE + 1);
            let count = usize::from(*control_count) % (MAX_CONTROL + 1);
            KnotVector::clamped_uniform(p, count)
        }
        KnotSpec::Raw { degree, knots } => {
            let p = usize::from(*degree) % (MAX_DEGREE + 1);
            let values: Vec<f64> = knots
                .iter()
                .take(2 * (MAX_DEGREE + 1) + MAX_CONTROL)
                .map(|c| c.value())
                .collect();
            KnotVector::new(p, values)
        }
    };

    let kv = match result {
        Ok(kv) => kv,
        Err(_) => return None,
    };

    // Documented invariants of an accepted knot vector.
    let knots = kv.knots();
    assert!(
        knots.len() >= 2 * (kv.degree() + 1),
        "KnotVector::new accepted {} knots for degree {}",
        knots.len(),
        kv.degree()
    );
    for i in 1..knots.len() {
        assert!(
            knots[i] >= knots[i - 1],
            "KnotVector::new accepted a decreasing knot vector at index {i}: {:?} then {:?}",
            knots[i - 1],
            knots[i]
        );
    }
    let (t0, t1) = kv.domain();
    assert!(
        t0 < t1,
        "KnotVector::new accepted an empty domain ({t0}, {t1})"
    );
    assert_eq!(
        kv.control_count(),
        knots.len() - kv.degree() - 1,
        "control_count disagrees with the knot count"
    );

    // `find_span` must always land on a usable span, including at and beyond
    // the domain ends. An out-of-range span is an immediate out-of-bounds
    // index in `basis_funs`.
    for t in span_probes(t0, t1) {
        let span = kv.find_span(t);
        assert!(
            span >= kv.degree() && span < kv.control_count(),
            "find_span({t}) returned {span}, outside [{}, {})",
            kv.degree(),
            kv.control_count()
        );
    }

    Some(kv)
}

/// Parameters that hit every interesting position relative to a domain.
fn span_probes(t0: f64, t1: f64) -> Vec<f64> {
    let mut probes = vec![t0, t1, 0.5 * (t0 + t1)];
    for i in 0..SAMPLES {
        probes.push(t0 + (t1 - t0) * (i as f64) / ((SAMPLES - 1) as f64));
    }
    probes
}

/// Cycle `values` to exactly `len` entries, or `None` when there are none.
fn cycled<T: Copy>(values: &[T], len: usize) -> Option<Vec<T>> {
    if values.is_empty() {
        return None;
    }
    Some((0..len).map(|i| values[i % values.len()]).collect())
}

fn run_curve(spec: &CurveSpec) {
    let Some(kv) = build_knots(&spec.knots) else {
        return;
    };
    let count = kv.control_count();
    if count == 0 || count > MAX_CONTROL {
        return;
    }
    let Some(coords) = cycled(&spec.coords, count * 3) else {
        return;
    };
    let Some(weights) = cycled(&spec.weights, count) else {
        return;
    };

    let points: Vec<Point3> = coords
        .chunks_exact(3)
        .map(|c| Point3::new(c[0].value(), c[1].value(), c[2].value()))
        .collect();
    let weight_values: Vec<f64> = weights.iter().map(|w| w.value()).collect();

    match NurbsCurve::new(points.clone(), weight_values.clone(), kv.clone()) {
        Err(NurbsError::NonPositiveWeight { index }) => {
            // The rejection must name a weight that really is not finite and
            // positive — a constructor that rejects valid geometry is as much
            // a bug as one that accepts invalid geometry.
            let weight = weight_values[index];
            assert!(
                !(weight.is_finite() && weight > 0.0),
                "rejected finite positive weight {weight} at index {index}"
            );
        }
        // Any other rejection is a shape/count mismatch: nothing to check
        // beyond the constructor having said no instead of panicking.
        Err(_) => {}
        Ok(curve) => {
            let tame = WellFormed::knots(&kv)
                && coords.iter().all(|c| c.is_tame())
                && weights.iter().all(|w| w.is_tame());
            exercise_curve(&curve, spec, tame);
        }
    }
}

fn exercise_curve(curve: &NurbsCurve, spec: &CurveSpec, tame: bool) {
    let (t0, t1) = curve.domain();
    let hull = curve
        .control_hull_box()
        .expect("a curve has control points");

    let mut probes = span_probes(t0, t1);
    // Out-of-domain and pathological parameters: `point` documents clamping.
    probes.extend([
        t0 - 1.0,
        t1 + 1.0,
        f64::NAN,
        f64::INFINITY,
        f64::NEG_INFINITY,
    ]);
    probes.extend(spec.probes.iter().take(8).map(|c| c.value()));

    for &t in &probes {
        let p = curve.point(t);
        let _ = curve.derivative(t);
        let _ = curve.second_derivative(t);
        for order in 0..=3 {
            let _ = curve.derivatives(t, order);
        }
        if !tame || !t.is_finite() {
            continue;
        }

        assert!(
            p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
            "curve.point({t}) is non-finite: {p:?}"
        );
        assert!(
            in_box(&hull, p),
            "curve.point({t}) = {p:?} escapes the control hull box {hull:?}"
        );
        for order in 0..=2 {
            for (k, d) in curve.derivatives(t, order).iter().enumerate() {
                assert!(
                    d.x.is_finite() && d.y.is_finite() && d.z.is_finite(),
                    "curve derivative {k} at {t} is non-finite: {d:?}"
                );
            }
        }

        // Clamping: below/above the domain evaluates as the endpoint.
        let clamped = curve.point(t.clamp(t0, t1));
        assert!(
            near(p, clamped, hull_scale(&hull)),
            "point({t}) = {p:?} does not match the clamped evaluation {clamped:?}"
        );
    }

    if !tame {
        // Still exercise the editing paths for their own no-panic contract.
        let _ = curve.insert_knot(0.5 * (t0 + t1));
        let _ = curve.reversed();
        return;
    }

    let scale = hull_scale(&hull);
    let cps = curve.control_points();

    // Endpoint interpolation holds for a clamped knot vector, which is the
    // only case where the first/last basis function is 1 at the domain end.
    if is_clamped(curve.knot_vector()) {
        assert!(
            near(curve.point(t0), cps[0], scale),
            "clamped curve starts at {:?}, not at its first control point {:?}",
            curve.point(t0),
            cps[0]
        );
        assert!(
            near(curve.point(t1), cps[cps.len() - 1], scale),
            "clamped curve ends at {:?}, not at its last control point {:?}",
            curve.point(t1),
            cps[cps.len() - 1]
        );
    }

    // Reversal traces the same locus backwards.
    let reversed = curve.reversed();
    let (r0, r1) = reversed.domain();
    for &t in &span_probes(t0, t1) {
        let mirrored = r0 + (r1 - r0) * ((t1 - t) / (t1 - t0));
        assert!(
            near(curve.point(t), reversed.point(mirrored), scale),
            "reversed curve does not trace the same locus at {t}: {:?} vs {:?}",
            curve.point(t),
            reversed.point(mirrored)
        );
    }

    // Knot insertion is locus-preserving by construction (Boehm, A5.1).
    let fraction = f64::from(spec.insert_at) / 256.0;
    let u = t0 + (t1 - t0) * fraction;
    if u > t0 && u < t1 {
        if let Ok(refined) = curve.insert_knot(u) {
            assert_eq!(
                refined.control_points().len(),
                cps.len() + 1,
                "knot insertion did not add exactly one control point"
            );
            for &t in &span_probes(t0, t1) {
                assert!(
                    near(curve.point(t), refined.point(t), scale),
                    "knot insertion at {u} moved the curve at {t}: {:?} vs {:?}",
                    curve.point(t),
                    refined.point(t)
                );
            }
        }
    }
}

fn run_surface(spec: &SurfaceSpec) {
    let (Some(ku), Some(kv)) = (build_knots(&spec.knots_u), build_knots(&spec.knots_v)) else {
        return;
    };
    let (rows, cols) = (ku.control_count(), kv.control_count());
    if rows == 0 || cols == 0 || rows > MAX_CONTROL || cols > MAX_CONTROL {
        return;
    }
    let Some(coords) = cycled(&spec.coords, rows * cols * 3) else {
        return;
    };
    let Some(weights) = cycled(&spec.weights, rows * cols) else {
        return;
    };

    let grid: Vec<Vec<Point3>> = (0..rows)
        .map(|i| {
            (0..cols)
                .map(|j| {
                    let base = (i * cols + j) * 3;
                    Point3::new(
                        coords[base].value(),
                        coords[base + 1].value(),
                        coords[base + 2].value(),
                    )
                })
                .collect()
        })
        .collect();
    let weight_grid: Vec<Vec<f64>> = (0..rows)
        .map(|i| (0..cols).map(|j| weights[i * cols + j].value()).collect())
        .collect();

    let Ok(surface) = NurbsSurface::new(grid, weight_grid, ku.clone(), kv.clone()) else {
        return;
    };

    let tame = WellFormed::knots(&ku)
        && WellFormed::knots(&kv)
        && coords.iter().all(|c| c.is_tame())
        && weights.iter().all(|w| w.is_tame());
    exercise_surface(&surface, spec, tame);
}

fn exercise_surface(surface: &NurbsSurface, spec: &SurfaceSpec, tame: bool) {
    let (u0, u1) = surface.domain_u();
    let (v0, v1) = surface.domain_v();
    let hull = surface
        .control_hull_box()
        .expect("a surface has control points");

    let _ = surface.has_degenerate_edge();
    assert_eq!(
        surface.grid_size(),
        (
            surface.knot_vector_u().control_count(),
            surface.knot_vector_v().control_count()
        ),
        "grid_size disagrees with the knot vectors"
    );

    let mut probes: Vec<(f64, f64)> = Vec::new();
    for &u in &span_probes(u0, u1) {
        for &v in &span_probes(v0, v1) {
            probes.push((u, v));
        }
    }
    probes.extend([
        (u0 - 1.0, v0 - 1.0),
        (u1 + 1.0, v1 + 1.0),
        (f64::NAN, 0.5 * (v0 + v1)),
        (0.5 * (u0 + u1), f64::INFINITY),
    ]);
    probes.extend(
        spec.probes
            .iter()
            .take(8)
            .map(|(a, b)| (a.value(), b.value())),
    );

    for &(u, v) in &probes {
        let p = surface.point(u, v);
        let _ = surface.du(u, v);
        let _ = surface.dv(u, v);
        let _ = surface.normal(u, v);
        for order in 0..=2 {
            let _ = surface.derivatives(u, v, order);
        }
        if !tame || !u.is_finite() || !v.is_finite() {
            continue;
        }
        assert!(
            p.x.is_finite() && p.y.is_finite() && p.z.is_finite(),
            "surface.point({u}, {v}) is non-finite: {p:?}"
        );
        assert!(
            in_box(&hull, p),
            "surface.point({u}, {v}) = {p:?} escapes the control hull box {hull:?}"
        );
        if let Some(n) = surface.normal(u, v) {
            assert!(
                (n.norm() - 1.0).abs() < 1e-6,
                "surface.normal({u}, {v}) is not a unit vector: {n:?} (norm {})",
                n.norm()
            );
        }
    }

    if !tame {
        return;
    }
    // Corner interpolation, when both directions are clamped.
    if is_clamped(surface.knot_vector_u()) && is_clamped(surface.knot_vector_v()) {
        let (rows, cols) = surface.grid_size();
        let scale = hull_scale(&hull);
        for (u, v, i, j) in [
            (u0, v0, 0, 0),
            (u0, v1, 0, cols - 1),
            (u1, v0, rows - 1, 0),
            (u1, v1, rows - 1, cols - 1),
        ] {
            let corner = surface.control_point(i, j);
            assert!(
                near(surface.point(u, v), corner, scale),
                "clamped surface corner ({u}, {v}) is {:?}, not control point ({i}, {j}) = {corner:?}",
                surface.point(u, v)
            );
        }
    }
}

/// Whether the end knots are repeated `degree + 1` times (a clamped, or
/// "open", knot vector — the only case with endpoint interpolation).
fn is_clamped(kv: &KnotVector) -> bool {
    let knots = kv.knots();
    let p = kv.degree();
    knots[..=p].iter().all(|&k| k == knots[0])
        && knots[knots.len() - p - 1..]
            .iter()
            .all(|&k| k == knots[knots.len() - 1])
}

/// Absolute tolerance derived from the size of the control hull: spline
/// evaluation error scales with the geometry, not with 1.0.
fn hull_scale(hull: &opensolid_core::types::BoundingBox3) -> f64 {
    let extent = (hull.max.x - hull.min.x)
        .max(hull.max.y - hull.min.y)
        .max(hull.max.z - hull.min.z);
    1e-9 * extent.max(1.0)
}

fn near(a: Point3, b: Point3, tolerance: f64) -> bool {
    (a.x - b.x).abs() <= tolerance
        && (a.y - b.y).abs() <= tolerance
        && (a.z - b.z).abs() <= tolerance
}

/// Hull containment, with the same scale-relative slack: the convex hull
/// property is exact in real arithmetic, so any slack here is for the
/// accumulated rounding of the basis-function sum alone.
fn in_box(hull: &opensolid_core::types::BoundingBox3, p: Point3) -> bool {
    let slack = hull_scale(hull) * 1e3;
    p.x >= hull.min.x - slack
        && p.x <= hull.max.x + slack
        && p.y >= hull.min.y - slack
        && p.y <= hull.max.y + slack
        && p.z >= hull.min.z - slack
        && p.z <= hull.max.z + slack
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn clamped_uniform_cubic_curve_satisfies_every_oracle() {
        let spec = CurveSpec {
            knots: KnotSpec::Uniform {
                degree: 3,
                control_count: 8,
            },
            coords: vec![
                Coord::Tame(0),
                Coord::Tame(32),
                Coord::Tame(-32),
                Coord::Tame(64),
                Coord::Tame(96),
                Coord::Tame(-64),
                Coord::Tame(128),
            ],
            weights: vec![Weight::Positive(0), Weight::Positive(256)],
            probes: vec![Coord::Tame(16), Coord::Tame(-16)],
            insert_at: 128,
        };
        run_curve(&spec);
    }

    #[test]
    fn clamped_uniform_bicubic_surface_satisfies_every_oracle() {
        let spec = SurfaceSpec {
            knots_u: KnotSpec::Uniform {
                degree: 3,
                control_count: 5,
            },
            knots_v: KnotSpec::Uniform {
                degree: 2,
                control_count: 4,
            },
            coords: vec![
                Coord::Tame(0),
                Coord::Tame(48),
                Coord::Tame(-48),
                Coord::Tame(96),
                Coord::Tame(12),
            ],
            weights: vec![Weight::Positive(64)],
            probes: vec![(Coord::Tame(8), Coord::Tame(-8))],
        };
        run_surface(&spec);
    }

    /// The knot-vector validator's own contract, on the shapes that break
    /// naive monotonicity scans.
    #[test]
    fn knot_validator_rejects_or_upholds_its_invariants() {
        let cases: Vec<Vec<f64>> = vec![
            vec![0.0, 0.0, 1.0, 1.0],
            vec![0.0, 0.0, 0.5, 1.0, 1.0],
            vec![f64::NAN, 0.0, 1.0, 1.0],
            vec![0.0, f64::NAN, f64::NAN, 1.0],
            vec![0.0, 0.0, f64::NAN, 1.0],
            vec![f64::NEG_INFINITY, 0.0, 1.0, f64::INFINITY],
            vec![0.0, 0.0, 0.0, 0.0],
            vec![1.0, 0.0, 1.0, 2.0],
        ];
        for knots in cases {
            for degree in 0..=2u8 {
                let spec = KnotSpec::Raw {
                    degree,
                    knots: knots
                        .iter()
                        .map(|&k| {
                            if k.is_nan() {
                                Coord::Nan
                            } else if k.is_infinite() {
                                Coord::Infinite(k < 0.0)
                            } else {
                                Coord::Tame((k * 32.0) as i16)
                            }
                        })
                        .collect(),
                };
                let _ = build_knots(&spec);
            }
        }
    }

    /// Weights the constructor must reject rather than divide by.
    #[test]
    fn invalid_weights_are_rejected() {
        let kv = KnotVector::clamped_uniform(2, 4).unwrap();
        let points = vec![Point3::new(0.0, 0.0, 0.0); 4];
        for weight in [
            Weight::Zero,
            Weight::Negative,
            Weight::Nan,
            Weight::Infinite,
        ] {
            let weights = vec![weight.value(); 4];
            let result = NurbsCurve::new(points.clone(), weights, kv.clone());
            assert!(
                matches!(result, Err(NurbsError::NonPositiveWeight { .. })),
                "constructor accepted weight {:?}",
                weight.value()
            );
        }
    }

    /// Byte-driven decoding must never panic, whatever the bytes are.
    #[test]
    fn arbitrary_decoding_is_total() {
        let mut seed = 0x9e37_79b9_7f4a_7c15u64;
        for _ in 0..4000 {
            let mut bytes = [0u8; 128];
            for byte in bytes.iter_mut() {
                seed = seed
                    .wrapping_mul(6364136223846793005)
                    .wrapping_add(1442695040888963407);
                *byte = (seed >> 33) as u8;
            }
            fuzz_nurbs_eval(&bytes);
        }
    }
}
