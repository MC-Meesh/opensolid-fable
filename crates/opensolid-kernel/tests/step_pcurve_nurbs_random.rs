//! Adversarial campaign for the of-50u freeform trim machinery (of-pr3r).
//!
//! of-50u gave the kernel `Curve2::Nurbs` — freeform trim geometry in a
//! surface's `(u, v)` space — and wired it through four surfaces this
//! campaign stresses independently of the original author's tests:
//!
//! 1. **Randomized authored trims.** A block whose top face is split along
//!    a randomized freeform curve, authored into STEP with the *exact* 2D
//!    preimage as each half's `PCURVE` (the split patches are affine in
//!    `(u, v)`, so the preimage of a rational B-spline is the rational
//!    B-spline of its control preimages, weight for weight). The reader's
//!    transplant gate (`transplant_authored_pcurves`) must adopt every one
//!    of them, the adopted curve must hold the parameterization invariant
//!    `surface.point(pcurve(t)) == curve.point(t)` at parameters nobody
//!    sampled while building it, and `brep_mass_properties` — which
//!    integrates through the stored pcurves — must still measure the block.
//! 2. **Corrupted authored trims.** The same file with one 2D control point
//!    perturbed by a log-uniform magnitude spanning twelve decades. The
//!    property is one-sided: whatever the gate adopts must survive the
//!    geometric check, and anything perturbed well past the gate's
//!    allowance must not be adopted at all. The failure mode this hunts is
//!    a gate/check disagreement — a candidate adopted under one allowance
//!    and rejected under the other.
//! 3. **Degenerate authored pcurves.** Constant curves, hostile weights,
//!    mismatched counts, decreasing knots, off-domain knots, huge
//!    coordinates. Collection is documented best-effort ("anything that
//!    fails to resolve simply contributes no candidate"), so every one of
//!    these must degrade to the derived fit without a panic and without an
//!    error diagnostic on an otherwise clean body.
//! 4. **Roundtrip.** Written back out, the adopted trims take the 2D
//!    B-spline emission path (of-50u's writer arm); re-imported, they must
//!    reconstitute a body with the same counts, a clean geometric check,
//!    and the same volume. The cylinder double-roundtrip covers the other
//!    new writer arm — the clockwise rim circle as a rational quadratic.
//! 5. **Seam crossings.** `fit_pcurve` on a *closed* freeform surface
//!    (a NURBS tube), fitting a curve that crosses the patch's join at a
//!    random phase — the case where samples unwrap past the knot rectangle
//!    and `recenter_on_knot_rectangle` decides what the fitted
//!    `Curve2::Nurbs` claims. And `NurbsCurve2::reversed` under knot
//!    vectors whose floating-point reflection collapses adjacent knots —
//!    reachable from a hostile STEP file through
//!    `collect_authored_pcurves` on a `same_sense = .F.` freeform edge.
//!
//! Protocol as `boolean_stress.rs` / `step_heal_random.rs`: deterministic
//! seeded [`Rng`], a repro string on every failure, failures become `bd`
//! beads and the case is `#[ignore]`d referencing the bead rather than
//! softened. `OPENSOLID_CAMPAIGN_SEED=<hex>` remixes every suite seed.
//!
//! Bugs filed from this suite (first run, 2026-08-01):
//! - of-yic4: `NurbsCurve{,2}::reversed` panics when the floating-point
//!   knot reflection `t0 + t1 − k` collapses two distinct knots (the
//!   reflected vector fails `KnotVector::new` behind an `expect`).
//!   Reachable from a crafted STEP file through a `same_sense = .F.`
//!   freeform edge — `collect_authored_pcurves` reverses the authored 2D
//!   trim (of-50u) and `trim_curve` the authored 3D basis — so the reader
//!   aborts instead of diagnosing. See
//!   [`reversing_a_hostile_knot_vector_must_not_panic`] and
//!   [`hostile_authored_pcurve_knots_on_a_reversed_edge_must_not_panic_the_reader`].
//! - of-xsr7: `fit_pcurve` on a *closed* NURBS patch, fitting a curve that
//!   crosses the patch's join mid-span, interpolates unwrapped samples
//!   that straddle the knot rectangle; the patch clamps outside its
//!   domain, so the fitted `Curve2::Nurbs` misses the invariant at its own
//!   fit parameters by up to a diameter (3.2 measured at radius 1.66). See
//!   [`seam_crossing_trim_on_a_closed_freeform_tube_holds_the_invariant`].

use std::fmt::Write as _;

use opensolid_kernel::brep::{
    Body, Curve2, Curve2Eval, Curve3, CurveEval, GeometryStore, KnotVector, NurbsCurve,
    NurbsCurve2, NurbsSurface, SeamSide, Surface3, SurfaceEval, TopologyStore, attach_body_pcurves,
    fit_pcurve, primitives,
};
use opensolid_kernel::brep_mass_properties;
use opensolid_kernel::core::EntityId;
use opensolid_kernel::core::types::{Point2, Point3};
use opensolid_kernel::io::step::read::{SolidOutcome, StepImport, StepReadOptions, read_step};
use opensolid_kernel::io::step::write::{StepWriteOptions, write_step};

// ---------------------------------------------------------------------
// Deterministic RNG (splitmix64), identical to `boolean_stress.rs`.
// ---------------------------------------------------------------------

/// Campaign remix (of-5rim): `OPENSOLID_CAMPAIGN_SEED=<hex>` XORs every
/// suite seed so the same properties walk fresh configurations each run.
/// Unset (CI, plain `cargo test`), the suite is byte-for-byte deterministic.
fn campaign_seed() -> u64 {
    static MIX: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    *MIX.get_or_init(|| match std::env::var("OPENSOLID_CAMPAIGN_SEED") {
        Ok(raw) => {
            let hex = raw.trim();
            let hex = hex.strip_prefix("0x").unwrap_or(hex);
            u64::from_str_radix(&hex.replace('_', ""), 16)
                .unwrap_or_else(|_| panic!("OPENSOLID_CAMPAIGN_SEED must be hex, got {raw:?}"))
        }
        Err(_) => 0,
    })
}

struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        Rng(seed ^ campaign_seed())
    }

    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9E37_79B9_7F4A_7C15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58_476D_1CE4_E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D0_49BB_1331_11EB);
        z ^ (z >> 31)
    }

    fn unit(&mut self) -> f64 {
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }

    fn range(&mut self, lo: f64, hi: f64) -> f64 {
        lo + (hi - lo) * self.unit()
    }

    fn pick(&mut self, n: usize) -> usize {
        (self.next_u64() % n as u64) as usize
    }
}

// ---------------------------------------------------------------------
// STEP text helpers
// ---------------------------------------------------------------------

/// Part 21 real spelling of an `f64`, exactly as the kernel's own writer
/// spells one (shortest round-trip form, decimal point guaranteed).
fn fr(x: f64) -> String {
    assert!(x.is_finite(), "STEP reals must be finite, got {x}");
    let s = format!("{x:?}");
    match s.split_once(['e', 'E']) {
        Some((mantissa, exponent)) => {
            if mantissa.contains('.') {
                format!("{mantissa}E{exponent}")
            } else {
                format!("{mantissa}.0E{exponent}")
            }
        }
        None => s,
    }
}

/// Wrap DATA-section body text in a minimal Part 21 envelope.
fn wrap(data: &str) -> String {
    format!(
        "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\n\
         DATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
    )
}

fn import(src: &str) -> (TopologyStore, GeometryStore, StepImport) {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step(src, &mut store, &mut geo, &StepReadOptions::default())
        .expect("campaign fixtures must be syntactically valid Part 21");
    (store, geo, report)
}

fn brep_body(report: &StepImport, repro: &str) -> EntityId<Body> {
    assert_eq!(report.solids.len(), 1, "{repro}: expected one solid");
    match &report.solids[0].outcome {
        SolidOutcome::BRep(body) => *body,
        other => panic!(
            "{repro}: expected an exact B-Rep import, got {other:?}; diagnostics: {:?}",
            report.diagnostics
        ),
    }
}

fn no_error_diagnostics(report: &StepImport, repro: &str) {
    assert!(
        !report.has_errors(),
        "{repro}: unexpected error diagnostics: {:?}",
        report.diagnostics
    );
}

/// How many authored trims the reader's transplant gate adopted, read off
/// the Info diagnostic `finish_exact_body` emits. Zero when the diagnostic
/// is absent.
fn transplanted_count(report: &StepImport) -> usize {
    report
        .diagnostics
        .iter()
        .find(|d| d.message.contains("adopted verbatim"))
        .and_then(|d| d.message.split_whitespace().next()?.parse().ok())
        .unwrap_or(0)
}

// ---------------------------------------------------------------------
// Randomized split-top fixture
// ---------------------------------------------------------------------
//
// A 2×2×2 block (corners at ±1) whose top face is split from the top A
// corner (−1,−1,1) to the top C corner (1,1,1) along a freeform curve,
// each half an ADVANCED_FACE on its own copy of the bilinear top patch.
// The patch rows ((A, D), (B, C)) put `u` along A→B and `v` along A→D, so
// `uv = ((x+1)/2, (y+1)/2)` — affine — and the authored 2D trim with
// control points `q_i = ((x_i+1)/2, (y_i+1)/2)` and the 3D curve's own
// weights satisfies `S(q(t)) == c(t)` identically.

/// The randomized freeform split curve, in 2D `(u, v)` control points.
/// The 3D control points are the affine images `(2u−1, 2v−1, 1)`.
struct SplitSpec {
    degree: usize,
    /// Control points in `(u, v)`, from `(0, 0)` to `(1, 1)`, with strictly
    /// increasing diagonal stations (which keeps the curve simple: its
    /// diagonal component's control coefficients increase, so by variation
    /// diminishing the curve crosses each diagonal level at most once).
    pts2: Vec<(f64, f64)>,
    /// `None` for a plain B-spline, `Some` for a rational curve (shared by
    /// the 2D and 3D spellings — an affine map preserves weights).
    weights: Option<Vec<f64>>,
    /// Distinct knot values and their multiplicities (clamped ends).
    knot_values: Vec<f64>,
    knot_mults: Vec<usize>,
}

impl SplitSpec {
    fn random(rng: &mut Rng) -> SplitSpec {
        let degree = 2 + rng.pick(2);
        let extra = rng.pick(3); // interior knots, each multiplicity 1
        let n = degree + 1 + extra;

        // Strictly increasing diagonal stations for the interior points.
        let interior = n - 2;
        let mut pts2 = vec![(0.0, 0.0)];
        for i in 0..interior {
            let s = 0.08 + 0.84 * (i as f64 + rng.unit()) / interior as f64;
            let half = (s.min(1.0 - s) - 0.05).max(0.01);
            let w = rng.range(-half, half);
            pts2.push((s - w, s + w));
        }
        pts2.push((1.0, 1.0));

        let weights = (rng.pick(2) == 1).then(|| (0..n).map(|_| rng.range(0.4, 2.5)).collect());

        let mut knot_values = vec![0.0];
        let mut knot_mults = vec![degree + 1];
        for i in 0..extra {
            // Stratified so the interior values stay distinct.
            knot_values.push(0.2 + 0.6 * (i as f64 + rng.unit()) / extra as f64);
            knot_mults.push(1);
        }
        knot_values.push(1.0);
        knot_mults.push(degree + 1);
        SplitSpec {
            degree,
            pts2,
            weights,
            knot_values,
            knot_mults,
        }
    }

    fn point3(&self, i: usize) -> (f64, f64, f64) {
        let (u, v) = self.pts2[i];
        (2.0 * u - 1.0, 2.0 * v - 1.0, 1.0)
    }
}

/// One B-spline curve record: the plain entity when `weights` is `None`,
/// the `RATIONAL_B_SPLINE_CURVE` complex instance otherwise (the same
/// spelling the kernel's writer emits and its reader accepts).
fn curve_record(
    id: u64,
    degree: usize,
    cp_refs: &[u64],
    knot_values: &[f64],
    knot_mults: &[usize],
    weights: Option<&[f64]>,
) -> String {
    let refs = cp_refs
        .iter()
        .map(|r| format!("#{r}"))
        .collect::<Vec<_>>()
        .join(", ");
    let knots = knot_values
        .iter()
        .map(|&k| fr(k))
        .collect::<Vec<_>>()
        .join(", ");
    let mults = knot_mults
        .iter()
        .map(|m| m.to_string())
        .collect::<Vec<_>>()
        .join(", ");
    match weights {
        None => format!(
            "#{id} = B_SPLINE_CURVE_WITH_KNOTS('', {degree}, ({refs}), .UNSPECIFIED., .F., .F., \
             ({mults}), ({knots}), .UNSPECIFIED.);\n"
        ),
        Some(w) => {
            let w = w.iter().map(|&x| fr(x)).collect::<Vec<_>>().join(", ");
            format!(
                "#{id} = ( BOUNDED_CURVE() B_SPLINE_CURVE({degree}, ({refs}), .UNSPECIFIED., \
                 .F., .U.) B_SPLINE_CURVE_WITH_KNOTS(({mults}), ({knots}), .UNSPECIFIED.) \
                 CURVE() GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(({w})) \
                 REPRESENTATION_ITEM('') )\n;\n"
            )
        }
    }
}

/// What the fixture authors as the 2D trim geometry, against the always-exact
/// 3D curve.
enum TrimAuthoring {
    /// The exact affine preimage: the transplant gate's happy path.
    Exact,
    /// The exact preimage with control point `index` displaced by `delta`.
    Perturbed { index: usize, delta: (f64, f64) },
    /// Every 2D control point the same point — a valid, zero-extent curve.
    Constant,
    /// A weight of the given value on one control point.
    Weight(f64),
    /// Control coordinates of ±1e300.
    HugeCoords,
    /// One control point fewer than the knot vector requires.
    CountMismatch,
    /// Knot values out of order.
    DecreasingKnots,
    /// A valid curve parameterized over `[5, 6]` instead of the edge's
    /// `[0, 1]` — evaluation clamps, so it traces a constant.
    OffDomainKnots,
    /// Interior knots whose floating-point reflection under
    /// `t0 + t1 − k` collapses — combined with a `same_sense = .F.` edge,
    /// this drives `collect_authored_pcurves` through
    /// `NurbsCurve2::reversed` on a vector whose reflection is invalid.
    HostileReflectKnots,
}

/// The full randomized split-top STEP file. `authoring` decides the 2D
/// geometry inside the `PCURVE`s; the 3D curve is always the exact one
/// (authored in reverse, with `same_sense = .F.`, for
/// [`TrimAuthoring::HostileReflectKnots`]).
fn split_top_block_step(spec: &SplitSpec, authoring: &TrimAuthoring) -> String {
    let mut b = String::new();
    let corners = [
        (-1.0, -1.0, -1.0),
        (1.0, -1.0, -1.0),
        (1.0, 1.0, -1.0),
        (-1.0, 1.0, -1.0),
        (-1.0, -1.0, 1.0),
        (1.0, -1.0, 1.0),
        (1.0, 1.0, 1.0),
        (-1.0, 1.0, 1.0),
    ];
    for (i, &(x, y, z)) in corners.iter().enumerate() {
        writeln!(
            b,
            "#{} = CARTESIAN_POINT('', ({}, {}, {}));",
            i + 1,
            fr(x),
            fr(y),
            fr(z)
        )
        .unwrap();
        writeln!(b, "#{} = VERTEX_POINT('', #{});", i + 11, i + 1).unwrap();
    }
    const EDGES: [(usize, usize); 12] = [
        (0, 1),
        (1, 2),
        (2, 3),
        (3, 0),
        (4, 5),
        (5, 6),
        (6, 7),
        (7, 4),
        (0, 4),
        (1, 5),
        (2, 6),
        (3, 7),
    ];
    for (e, &(i, j)) in EDGES.iter().enumerate() {
        let eb = 20 + 4 * e;
        let (dx, dy, dz) = (
            corners[j].0 - corners[i].0,
            corners[j].1 - corners[i].1,
            corners[j].2 - corners[i].2,
        );
        writeln!(
            b,
            "#{eb} = DIRECTION('', ({}, {}, {}));",
            fr(dx),
            fr(dy),
            fr(dz)
        )
        .unwrap();
        writeln!(b, "#{} = VECTOR('', #{eb}, 1.);", eb + 1).unwrap();
        writeln!(b, "#{} = LINE('', #{}, #{});", eb + 2, i + 1, eb + 1).unwrap();
        writeln!(
            b,
            "#{} = EDGE_CURVE('', #{}, #{}, #{}, .T.);",
            eb + 3,
            i + 11,
            j + 11,
            eb + 2
        )
        .unwrap();
    }
    let face_specs: [([usize; 4], (f64, f64, f64)); 5] = [
        ([0, 3, 2, 1], (0.0, 0.0, -1.0)),
        ([0, 1, 5, 4], (0.0, -1.0, 0.0)),
        ([1, 2, 6, 5], (1.0, 0.0, 0.0)),
        ([2, 3, 7, 6], (0.0, 1.0, 0.0)),
        ([3, 0, 4, 7], (-1.0, 0.0, 0.0)),
    ];
    let mut shell_faces = Vec::new();
    for (f, &(cycle, (nx, ny, nz))) in face_specs.iter().enumerate() {
        let fb = 100 + 10 * f;
        writeln!(
            b,
            "#{fb} = DIRECTION('', ({}, {}, {}));",
            fr(nx),
            fr(ny),
            fr(nz)
        )
        .unwrap();
        writeln!(
            b,
            "#{} = AXIS2_PLACEMENT_3D('', #{}, #{fb}, $);",
            fb + 1,
            cycle[0] + 1
        )
        .unwrap();
        writeln!(b, "#{} = PLANE('', #{});", fb + 2, fb + 1).unwrap();
        for k in 0..4 {
            let (from, to) = (cycle[k], cycle[(k + 1) % 4]);
            let (idx, &(a, _)) = EDGES
                .iter()
                .enumerate()
                .find(|&(_, &(a, c))| (a, c) == (from, to) || (a, c) == (to, from))
                .expect("face cycles only use listed edges");
            let orientation = if a == from { ".T." } else { ".F." };
            writeln!(
                b,
                "#{} = ORIENTED_EDGE('', *, *, #{}, {orientation});",
                fb + 3 + k,
                23 + 4 * idx
            )
            .unwrap();
        }
        writeln!(
            b,
            "#{} = EDGE_LOOP('', (#{}, #{}, #{}, #{}));",
            fb + 7,
            fb + 3,
            fb + 4,
            fb + 5,
            fb + 6
        )
        .unwrap();
        writeln!(b, "#{} = FACE_OUTER_BOUND('', #{}, .T.);", fb + 8, fb + 7).unwrap();
        writeln!(
            b,
            "#{} = ADVANCED_FACE('', (#{}), #{}, .T.);",
            fb + 9,
            fb + 8,
            fb + 2
        )
        .unwrap();
        shell_faces.push(format!("#{}", fb + 9));
    }

    // The 3D splitting curve. Endpoints are the existing top corner points
    // #5 (A) and #7 (C); interior control points are #260+.
    let n = spec.pts2.len();
    let reversed_3d = matches!(authoring, TrimAuthoring::HostileReflectKnots);
    let mut cp3_refs = Vec::with_capacity(n);
    for i in 0..n {
        if i == 0 {
            cp3_refs.push(5);
        } else if i == n - 1 {
            cp3_refs.push(7);
        } else {
            let (x, y, z) = spec.point3(i);
            let id = 260 + i as u64;
            writeln!(
                b,
                "#{id} = CARTESIAN_POINT('', ({}, {}, {}));",
                fr(x),
                fr(y),
                fr(z)
            )
            .unwrap();
            cp3_refs.push(id);
        }
    }
    // For the same_sense = .F. edge the 3D curve is authored running C→A;
    // the symmetric clamped knot vector reflects onto itself, so reversing
    // the control points (and weights) is the whole reversal.
    let (cp3_refs, weights3) = if reversed_3d {
        let mut r = cp3_refs.clone();
        r.reverse();
        let w = spec.weights.clone().map(|mut w| {
            w.reverse();
            w
        });
        (r, w)
    } else {
        (cp3_refs.clone(), spec.weights.clone())
    };
    b.push_str(&curve_record(
        201,
        spec.degree,
        &cp3_refs,
        &spec.knot_values,
        &spec.knot_mults,
        weights3.as_deref(),
    ));

    // The two copies of the bilinear top patch.
    for id in [210u64, 211] {
        writeln!(
            b,
            "#{id} = B_SPLINE_SURFACE_WITH_KNOTS('', 1, 1, ((#5, #8), (#6, #7)), .UNSPECIFIED., \
             .F., .F., .F., (2, 2), (2, 2), (0., 1.), (0., 1.), .UNSPECIFIED.);"
        )
        .unwrap();
    }

    // The authored 2D trim geometry.
    let mut pts2 = spec.pts2.clone();
    let mut knot_values = spec.knot_values.clone();
    let mut knot_mults = spec.knot_mults.clone();
    let mut weights2 = spec.weights.clone();
    let mut drop_last_cp = false;
    match authoring {
        TrimAuthoring::Exact => {}
        TrimAuthoring::Perturbed { index, delta } => {
            pts2[*index].0 += delta.0;
            pts2[*index].1 += delta.1;
        }
        TrimAuthoring::Constant => {
            for p in pts2.iter_mut() {
                *p = (0.5, 0.5);
            }
        }
        TrimAuthoring::Weight(w) => {
            let mut ws = weights2.unwrap_or_else(|| vec![1.0; n]);
            ws[n / 2] = *w;
            weights2 = Some(ws);
        }
        TrimAuthoring::HugeCoords => {
            for (i, p) in pts2.iter_mut().enumerate() {
                *p = if i % 2 == 0 {
                    (1e300, -1e300)
                } else {
                    (-1e300, 1e300)
                };
            }
        }
        TrimAuthoring::CountMismatch => drop_last_cp = true,
        TrimAuthoring::DecreasingKnots => {
            knot_values.reverse();
        }
        TrimAuthoring::OffDomainKnots => {
            for k in knot_values.iter_mut() {
                *k += 5.0;
            }
        }
        TrimAuthoring::HostileReflectKnots => {
            // Degree 1, four control points, knots [0, 0, 1, 1+ε, 1e17,
            // 1e17]: valid (two distinct interior knots, multiplicity 1
            // each), but `t0 + t1 = 1e17` reflects both interior knots onto
            // the same f64 — the reflected vector is not a valid knot
            // vector at degree 1.
            pts2 = vec![(0.0, 0.0), (0.3, 0.4), (0.7, 0.6), (1.0, 1.0)];
            knot_values = vec![0.0, 1.0, 1.0 + f64::EPSILON, 1e17];
            knot_mults = vec![2, 1, 1, 2];
            weights2 = None;
        }
    }
    let mut cp2_refs = Vec::new();
    for (i, &(u, v)) in pts2.iter().enumerate() {
        let id = 280 + i as u64;
        writeln!(b, "#{id} = CARTESIAN_POINT('', ({}, {}));", fr(u), fr(v)).unwrap();
        cp2_refs.push(id);
    }
    if drop_last_cp {
        cp2_refs.pop();
    }
    let degree_2d = if matches!(authoring, TrimAuthoring::HostileReflectKnots) {
        1
    } else {
        spec.degree
    };
    b.push_str(&curve_record(
        223,
        degree_2d,
        &cp2_refs,
        &knot_values,
        &knot_mults,
        weights2.as_deref(),
    ));

    let sense = if reversed_3d { ".F." } else { ".T." };
    writeln!(
        b,
        "#224 = ( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() \
         REPRESENTATION_CONTEXT('2D SPACE','') )\n;\n\
         #225 = DEFINITIONAL_REPRESENTATION('', (#223), #224);\n\
         #226 = PCURVE('', #210, #225);\n\
         #227 = PCURVE('', #211, #225);\n\
         #228 = SURFACE_CURVE('', #201, (#226, #227), .CURVE_3D.);\n\
         #229 = EDGE_CURVE('', #15, #17, #228, {sense});"
    )
    .unwrap();

    // Half 1: A → C (curved) → D → A on #210; half 2: A → B → C → A
    // (curved, reversed) on #211. Top-ring edges: A→B #39, B→C #43,
    // C→D #47, D→A #51.
    b.push_str(
        "#230 = ORIENTED_EDGE('', *, *, #229, .T.);\n\
         #231 = ORIENTED_EDGE('', *, *, #47, .T.);\n\
         #232 = ORIENTED_EDGE('', *, *, #51, .T.);\n\
         #233 = EDGE_LOOP('', (#230, #231, #232));\n\
         #234 = FACE_OUTER_BOUND('', #233, .T.);\n\
         #235 = ADVANCED_FACE('', (#234), #210, .T.);\n\
         #240 = ORIENTED_EDGE('', *, *, #39, .T.);\n\
         #241 = ORIENTED_EDGE('', *, *, #43, .T.);\n\
         #242 = ORIENTED_EDGE('', *, *, #229, .F.);\n\
         #243 = EDGE_LOOP('', (#240, #241, #242));\n\
         #244 = FACE_OUTER_BOUND('', #243, .T.);\n\
         #245 = ADVANCED_FACE('', (#244), #211, .T.);\n",
    );
    shell_faces.push("#235".to_string());
    shell_faces.push("#245".to_string());
    writeln!(b, "#250 = CLOSED_SHELL('', ({}));", shell_faces.join(", ")).unwrap();
    writeln!(b, "#251 = MANIFOLD_SOLID_BREP('split top', #250);").unwrap();
    wrap(&b)
}

// ---------------------------------------------------------------------
// Shared assertions
// ---------------------------------------------------------------------

/// The split edge — the body's only freeform edge — and its trim range.
fn split_edge(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
) -> (EntityId<opensolid_kernel::brep::Edge>, Curve3, f64, f64) {
    for face in store.faces_of_body(body) {
        for edge_id in store.edges_of_face(face) {
            let edge = store.edge(edge_id).expect("live edge");
            let curve = geo
                .curve(edge.curve.expect("edge curve"))
                .expect("live curve");
            if matches!(curve, Curve3::Nurbs(_)) {
                return (edge_id, curve.clone(), edge.t_start, edge.t_end);
            }
        }
    }
    panic!("the split-top fixture always has exactly one freeform edge");
}

/// Max deviation of `surface.point(pcurve(t))` from `curve.point(t)` over
/// `samples` evenly spaced parameters — the module invariant, measured at
/// parameters nobody used to build the pcurve.
fn max_invariant_deviation(
    surface: &Surface3,
    curve: &Curve3,
    pcurve: &Curve2,
    t_start: f64,
    t_end: f64,
    samples: usize,
) -> f64 {
    let mut worst = 0.0f64;
    for i in 0..samples {
        let t = t_start + (t_end - t_start) * (i as f64) / ((samples - 1) as f64);
        let uv = pcurve.point(t);
        let deviation = (surface.point(uv.x, uv.y) - curve.point(t)).norm();
        if !deviation.is_finite() {
            return f64::INFINITY;
        }
        worst = worst.max(deviation);
    }
    worst
}

/// Every fin riding the split edge, with its face's surface and its pcurve.
fn split_edge_fin_pcurves(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    edge_id: EntityId<opensolid_kernel::brep::Edge>,
) -> Vec<(Surface3, Option<Curve2>)> {
    let mut out = Vec::new();
    for face in store.faces_of_body(body) {
        let surface = store
            .faces
            .get(face)
            .and_then(|f| f.surface)
            .and_then(|id| geo.surface(id))
            .expect("every fixture face has a surface")
            .clone();
        for loop_id in store.loops_of_face(face) {
            for &fin in store.fins_of_loop(loop_id) {
                if store.fin_edge(fin) == edge_id {
                    let pcurve = store
                        .fin(fin)
                        .expect("live fin")
                        .pcurve
                        .and_then(|id| geo.pcurve(id))
                        .cloned();
                    out.push((surface.clone(), pcurve));
                }
            }
        }
    }
    out
}

fn assert_clean(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>, repro: &str) {
    let failures = store.check_with_geometry(geo, body);
    assert!(
        failures.is_empty(),
        "{repro}: body must pass the geometric check: {failures:?}"
    );
}

fn assert_volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>, repro: &str) {
    let mp = brep_mass_properties(store, geo, body)
        .unwrap_or_else(|e| panic!("{repro}: the split block must measure: {e}"));
    assert!(
        (mp.volume - 8.0).abs() < 1e-9 * 8.0,
        "{repro}: volume {} is not 8 — the stored pcurves misintegrate",
        mp.volume
    );
}

// ---------------------------------------------------------------------
// 1. Randomized exact authored trims: adopt, hold the invariant, measure
// ---------------------------------------------------------------------

#[test]
fn random_exact_authored_trims_transplant_and_hold_the_invariant() {
    for case in 0..24u64 {
        let mut rng = Rng::new(0x50D0 + case);
        let spec = SplitSpec::random(&mut rng);
        let repro = format!(
            "seed 0x{:04X} (cargo test --test step_pcurve_nurbs_random \
             random_exact_authored_trims), degree {}, {} CPs, rational {}",
            0x50D0 + case,
            spec.degree,
            spec.pts2.len(),
            spec.weights.is_some()
        );
        let text = split_top_block_step(&spec, &TrimAuthoring::Exact);
        let (store, geo, report) = import(&text);
        no_error_diagnostics(&report, &repro);
        let body = brep_body(&report, &repro);
        assert_clean(&store, &geo, body, &repro);

        assert_eq!(
            transplanted_count(&report),
            2,
            "{repro}: both halves author the exact preimage, so both fins \
             must adopt it; diagnostics: {:?}",
            report.diagnostics
        );

        let (edge_id, curve, t0, t1) = split_edge(&store, &geo, body);
        let fins = split_edge_fin_pcurves(&store, &geo, body, edge_id);
        assert_eq!(fins.len(), 2, "{repro}: the split edge has two fins");
        for (surface, pcurve) in &fins {
            let pcurve = pcurve.as_ref().expect("transplanted fins carry a pcurve");
            let Curve2::Nurbs { fit_params, .. } = pcurve else {
                panic!("{repro}: adopted trim must be Curve2::Nurbs, got {pcurve:?}");
            };
            assert!(
                fit_params.is_empty(),
                "{repro}: a transplanted trim claims exactness (no fit params)"
            );
            // The invariant at 129 parameters nobody sampled during
            // adoption (the gate used 33).
            let worst = max_invariant_deviation(surface, &curve, pcurve, t0, t1, 129);
            assert!(
                worst < 1e-10,
                "{repro}: adopted trim deviates {worst:e} from its edge"
            );
        }
        assert_volume(&store, &geo, body, &repro);
    }
}

// ---------------------------------------------------------------------
// 2. Roundtrip: the adopted trims survive write → read
// ---------------------------------------------------------------------

#[test]
fn random_authored_trims_survive_a_write_read_roundtrip() {
    for case in 0..12u64 {
        let mut rng = Rng::new(0x50D1 + case);
        let spec = SplitSpec::random(&mut rng);
        let repro = format!(
            "seed 0x{:04X} (cargo test --test step_pcurve_nurbs_random \
             random_authored_trims_survive), degree {}, {} CPs, rational {}",
            0x50D1 + case,
            spec.degree,
            spec.pts2.len(),
            spec.weights.is_some()
        );
        let text = split_top_block_step(&spec, &TrimAuthoring::Exact);
        let (store, geo, report) = import(&text);
        no_error_diagnostics(&report, &repro);
        let body = brep_body(&report, &repro);

        let emitted = write_step(&store, &geo, &[body], &StepWriteOptions::default())
            .unwrap_or_else(|e| panic!("{repro}: body must serialize: {e}"));
        assert!(
            emitted.contains("PCURVE"),
            "{repro}: the roundtrip must carry trim geometry"
        );

        let (store2, geo2, report2) = import(&emitted);
        no_error_diagnostics(&report2, &repro);
        let body2 = brep_body(&report2, &repro);
        assert_clean(&store2, &geo2, body2, &repro);

        let a = store.euler_counts(body);
        let b = store2.euler_counts(body2);
        assert_eq!(
            (a.vertices, a.edges, a.faces),
            (b.vertices, b.edges, b.faces),
            "{repro}: euler counts must survive the roundtrip"
        );

        // The re-imported split edge must hold the invariant through
        // whatever pcurve the second read attached (transplant or refit).
        let (edge_id2, curve2, t0, t1) = split_edge(&store2, &geo2, body2);
        for (surface, pcurve) in split_edge_fin_pcurves(&store2, &geo2, body2, edge_id2) {
            let pcurve = pcurve.expect("roundtripped fins carry a pcurve");
            // A fitted pcurve only claims its samples, so hold the
            // roundtrip to a fit-scale allowance rather than exactness.
            let worst = max_invariant_deviation(&surface, &curve2, &pcurve, t0, t1, 129);
            assert!(
                worst < 1e-6,
                "{repro}: roundtripped trim deviates {worst:e} from its edge"
            );
        }
        assert_volume(&store2, &geo2, body2, &repro);
    }
}

// ---------------------------------------------------------------------
// 3. Corrupted authored trims: never adopted broken
// ---------------------------------------------------------------------

#[test]
fn corrupted_authored_trims_are_never_adopted_broken() {
    for case in 0..32u64 {
        let mut rng = Rng::new(0x50D2 + case);
        let spec = SplitSpec::random(&mut rng);
        // Log-uniform magnitude across twelve decades, straddling the
        // transplant gate's allowance from either side.
        let magnitude = 10f64.powf(rng.range(-12.0, -0.7));
        let angle = rng.range(0.0, std::f64::consts::TAU);
        let index = 1 + rng.pick(spec.pts2.len() - 2);
        let delta = (magnitude * angle.cos(), magnitude * angle.sin());
        let repro = format!(
            "seed 0x{:04X} (cargo test --test step_pcurve_nurbs_random \
             corrupted_authored_trims), degree {}, rational {}, CP {} displaced by {:e}",
            0x50D2 + case,
            spec.degree,
            spec.weights.is_some(),
            index,
            magnitude
        );
        let text = split_top_block_step(&spec, &TrimAuthoring::Perturbed { index, delta });
        let (store, geo, report) = import(&text);
        no_error_diagnostics(&report, &repro);
        let body = brep_body(&report, &repro);

        // The two-sided property. Adopted or refit, the body must pass the
        // geometric check — the gate mirrors the check's allowance, so
        // "adopted but failing check" is the defect this hunts. And a
        // corruption far past any allowance must not be adopted at all.
        assert_clean(&store, &geo, body, &repro);
        let adopted = transplanted_count(&report);
        if magnitude > 1e-5 {
            assert_eq!(
                adopted, 0,
                "{repro}: a trim displaced {magnitude:e} is not the edge's \
                 trim and must not be adopted"
            );
        }
        if adopted > 0 {
            let (edge_id, curve, t0, t1) = split_edge(&store, &geo, body);
            for (surface, pcurve) in split_edge_fin_pcurves(&store, &geo, body, edge_id) {
                let pcurve = pcurve.expect("fins carry a pcurve");
                if let Curve2::Nurbs { fit_params, .. } = &pcurve
                    && fit_params.is_empty()
                {
                    let worst = max_invariant_deviation(&surface, &curve, &pcurve, t0, t1, 129);
                    assert!(
                        worst < 1e-6,
                        "{repro}: an adopted trim claims exactness but deviates {worst:e}"
                    );
                }
            }
        }
        assert_volume(&store, &geo, body, &repro);
    }
}

// ---------------------------------------------------------------------
// 4. Degenerate authored pcurves: best-effort means no panic, no damage
// ---------------------------------------------------------------------

#[test]
fn degenerate_authored_pcurves_degrade_without_panic() {
    let cases: Vec<(&str, TrimAuthoring)> = vec![
        ("constant 2D curve", TrimAuthoring::Constant),
        ("negative weight", TrimAuthoring::Weight(-1.0)),
        ("zero weight", TrimAuthoring::Weight(0.0)),
        ("huge weight", TrimAuthoring::Weight(1e12)),
        ("huge coordinates", TrimAuthoring::HugeCoords),
        ("control count mismatch", TrimAuthoring::CountMismatch),
        ("decreasing knots", TrimAuthoring::DecreasingKnots),
        ("off-domain knots", TrimAuthoring::OffDomainKnots),
    ];
    for (label, authoring) in &cases {
        let mut rng = Rng::new(0x50D4);
        let spec = SplitSpec::random(&mut rng);
        let repro = format!(
            "case {label:?} (cargo test --test step_pcurve_nurbs_random \
             degenerate_authored_pcurves)"
        );
        let text = split_top_block_step(&spec, authoring);
        let (store, geo, report) = import(&text);
        // The 3D geometry is intact, so the body must import exactly; the
        // authored 2D geometry contributes no candidate (or a candidate
        // the gate rejects) and the derived fit stands in.
        let body = brep_body(&report, &repro);
        assert_clean(&store, &geo, body, &repro);
        assert_eq!(
            transplanted_count(&report),
            0,
            "{repro}: degenerate authored geometry must never be adopted"
        );
        assert_volume(&store, &geo, body, &repro);
    }
}

// ---------------------------------------------------------------------
// 5. Hostile knot reflection: `reversed` on a reversed-sense edge
// ---------------------------------------------------------------------

/// Knot vectors valid at degree 1 whose floating-point reflection
/// `t0 + t1 − k` collapses the two distinct interior knots onto one f64
/// (or onto the domain end). `NurbsCurve2::reversed` — and its 3D twin —
/// rebuilds the reflected vector through `KnotVector::new` behind an
/// `expect`, so a collapse is an abort, not an error.
///
/// First run: panicked `DegenerateEndSpan { index: 3, knot: 1e17 }`.
#[test]
#[ignore = "of-yic4: NurbsCurve2::reversed panics on a collapsing knot reflection"]
fn reversing_a_hostile_knot_vector_must_not_panic() {
    let knots = KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0 + f64::EPSILON, 1e17, 1e17])
        .expect("distinct interior knots at multiplicity 1 are valid");
    let curve = NurbsCurve2::bspline(
        vec![
            Point2::new(0.0, 0.0),
            Point2::new(0.3, 0.4),
            Point2::new(0.7, 0.6),
            Point2::new(1.0, 1.0),
        ],
        knots,
    )
    .expect("valid curve");
    // repro: cargo test --test step_pcurve_nurbs_random reversing_a_hostile -- --ignored
    let reversed = curve.reversed();
    assert_eq!(
        reversed.domain(),
        curve.domain(),
        "reversal preserves the parameter domain"
    );
}

/// The same collapse driven from a STEP file: a `same_sense = .F.`
/// freeform edge makes `collect_authored_pcurves` reverse the authored 2D
/// trim before the transplant gate ever sees it, so a crafted file aborts
/// the reader.
///
/// First run: the import call panicked inside `NurbsCurve2::reversed`
/// (`curve2.rs:165`) before any diagnostic was produced.
#[test]
#[ignore = "of-yic4: hostile authored pcurve knots panic the STEP reader"]
fn hostile_authored_pcurve_knots_on_a_reversed_edge_must_not_panic_the_reader() {
    let mut rng = Rng::new(0x50D5);
    let spec = SplitSpec::random(&mut rng);
    // repro: cargo test --test step_pcurve_nurbs_random hostile_authored -- --ignored
    let repro = "hostile reflect knots (cargo test --test step_pcurve_nurbs_random \
                 hostile_authored_pcurve_knots)";
    let text = split_top_block_step(&spec, &TrimAuthoring::HostileReflectKnots);
    let (store, geo, report) = import(&text);
    let body = brep_body(&report, repro);
    assert_clean(&store, &geo, body, repro);
    assert_eq!(
        transplanted_count(&report),
        0,
        "{repro}: nonsense authored knots must never be adopted"
    );
}

// ---------------------------------------------------------------------
// 6. Seam crossings on a closed freeform surface
// ---------------------------------------------------------------------

/// The exact rational-quadratic unit-circle control layout (Piegl & Tiller
/// §7.5): nine points over four 90° arcs, weights alternating 1, √2⁄2.
fn circle_controls(radius: f64, phase: f64, z: f64) -> (Vec<Point3>, Vec<f64>) {
    let s = std::f64::consts::FRAC_1_SQRT_2;
    let mut points = Vec::with_capacity(9);
    let mut weights = Vec::with_capacity(9);
    for i in 0..9 {
        let angle = phase + i as f64 * std::f64::consts::FRAC_PI_4;
        let reach = if i % 2 == 0 {
            1.0
        } else {
            std::f64::consts::SQRT_2
        };
        points.push(Point3::new(
            radius * reach * angle.cos(),
            radius * reach * angle.sin(),
            z,
        ));
        weights.push(if i % 2 == 0 { 1.0 } else { s });
    }
    (points, weights)
}

fn quarter_knots() -> KnotVector {
    KnotVector::new(
        2,
        vec![
            0.0, 0.0, 0.0, 0.25, 0.25, 0.5, 0.5, 0.75, 0.75, 1.0, 1.0, 1.0,
        ],
    )
    .expect("quarter-arc knots")
}

/// A closed NURBS tube: the exact circle swept linearly in `v` from `z = 0`
/// to `z = height`. `closure_u` reports the patch closed, which is what
/// makes its join a branch cut for `fit_pcurve`.
fn closed_tube(radius: f64, height: f64) -> NurbsSurface {
    let (ring, ring_weights) = circle_controls(radius, 0.0, 0.0);
    let grid: Vec<Vec<Point3>> = ring
        .iter()
        .map(|p| vec![*p, Point3::new(p.x, p.y, height)])
        .collect();
    let weights: Vec<Vec<f64>> = ring_weights.iter().map(|&w| vec![w, w]).collect();
    let knots_v = KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).expect("linear knots");
    NurbsSurface::new(grid, weights, quarter_knots(), knots_v).expect("closed tube")
}

/// A trim that crosses the closed patch's join mid-span: `fit_pcurve` must
/// still produce a pcurve that holds the invariant at the parameters it
/// claims. The samples unwrap past the knot rectangle and
/// `recenter_on_knot_rectangle` recentres the straddling run; a clamped
/// patch evaluates a parameter outside its rectangle to the rectangle's
/// edge, so any overhang the recentre leaves is arc-length error the fit
/// silently claims as exact.
///
/// First run: every seed failed, worst deviation 3.229 at radius 1.662
/// (seed 0x50D6) — about 1.9 radii of arc claimed exactly and traced
/// wrongly.
#[test]
#[ignore = "of-xsr7: seam-crossing fit on a closed NURBS patch breaks the invariant"]
fn seam_crossing_trim_on_a_closed_freeform_tube_holds_the_invariant() {
    for case in 0..12u64 {
        let mut rng = Rng::new(0x50D6 + case);
        let radius = rng.range(0.5, 3.0);
        let height = rng.range(1.0, 4.0);
        let phase = rng.range(0.15, 0.85) * std::f64::consts::TAU;
        let z = rng.range(0.2, 0.8) * height;
        let repro = format!(
            "seed 0x{:04X} (cargo test --test step_pcurve_nurbs_random \
             seam_crossing_trim -- --ignored), r {radius:.3}, h {height:.3}, \
             phase {phase:.3}, z {z:.3}",
            0x50D6 + case
        );

        let surface = Surface3::nurbs(closed_tube(radius, height));
        let (points, weights) = circle_controls(radius, phase, z);
        let curve = Curve3::nurbs(
            NurbsCurve::new(points, weights, quarter_knots()).expect("rotated circle"),
        );

        let pcurve = fit_pcurve(&surface, &curve, 0.0, 1.0, SeamSide::Low)
            .unwrap_or_else(|e| panic!("{repro}: fit must succeed: {e}"));
        // The pairing has no closed-form inverse, so the freeform fit is
        // the expected variant; a polyline would mean the interpolation
        // solve failed.
        assert!(
            matches!(pcurve, Curve2::Nurbs { .. }),
            "{repro}: expected a fitted Curve2::Nurbs, got {pcurve:?}"
        );
        let worst = max_invariant_deviation(&surface, &curve, &pcurve, 0.0, 1.0, 129);
        assert!(
            worst < 1e-6 * radius,
            "{repro}: fitted seam-crossing trim deviates {worst:e} \
             (radius {radius:.3}) from the curve it claims to trace"
        );
    }
}

/// A curve that collapses to a point in parameter space must be refused
/// with a structured error, never fitted or panicked on.
#[test]
fn a_stationary_curve_on_the_tube_is_refused_cleanly() {
    let surface = Surface3::nurbs(closed_tube(1.0, 2.0));
    let p = Point3::new(1.0, 0.0, 1.0);
    let stationary = Curve3::nurbs(
        NurbsCurve::bspline(
            vec![p, p],
            KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).expect("linear knots"),
        )
        .expect("constant curve"),
    );
    assert!(
        fit_pcurve(&surface, &stationary, 0.0, 1.0, SeamSide::Low).is_err(),
        "a pcurve with no extent bounds nothing and must be refused"
    );
}

// ---------------------------------------------------------------------
// 7. Extreme weights: evaluation stays finite
// ---------------------------------------------------------------------

/// Weight ratios of 1e8 between adjacent control points push the
/// homogeneous coordinates far apart; evaluation and differentiation must
/// stay finite and the derivative must agree with finite differences.
#[test]
fn extreme_weight_ratios_evaluate_finitely() {
    for case in 0..8u64 {
        let mut rng = Rng::new(0x50D7 + case);
        let n = 5;
        let points: Vec<Point2> = (0..n)
            .map(|i| Point2::new(i as f64 + rng.unit(), rng.range(-1.0, 1.0)))
            .collect();
        let weights: Vec<f64> = (0..n).map(|_| 10f64.powf(rng.range(-4.0, 4.0))).collect();
        let repro = format!(
            "seed 0x{:04X} (cargo test --test step_pcurve_nurbs_random \
             extreme_weight_ratios), weights {weights:?}",
            0x50D7 + case
        );
        let knots = KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0])
            .expect("cubic knots");
        let curve = NurbsCurve2::new(points, weights, knots).expect("extreme weights are legal");
        for i in 0..=64 {
            let t = i as f64 / 64.0;
            let p = curve.point(t);
            let d = curve.derivative(t);
            assert!(
                p.x.is_finite() && p.y.is_finite() && d.x.is_finite() && d.y.is_finite(),
                "{repro}: non-finite evaluation at t={t}"
            );
        }
        for t in [0.1, 0.35, 0.6, 0.9] {
            let h = 1e-7;
            let fd = (curve.point(t + h) - curve.point(t - h)) / (2.0 * h);
            let d = curve.derivative(t);
            let scale = d.norm().max(1.0);
            assert!(
                (fd - d).norm() < 1e-3 * scale,
                "{repro}: derivative {d:?} disagrees with finite difference {fd:?} at t={t}"
            );
        }
    }
}

// ---------------------------------------------------------------------
// 8. Cylinder double roundtrip: the clockwise rim's rational spelling
// ---------------------------------------------------------------------

/// One cylinder cap's rim circle runs clockwise in `(u, v)`, which of-50u
/// spells as a rational quadratic 2D B-spline. Two full write → read trips
/// exercise emit → parse → re-derive → emit again; the volume and the
/// geometric check must be stable through both.
#[test]
fn cylinder_double_roundtrip_is_stable_through_the_clockwise_rim() {
    for case in 0..6u64 {
        let mut rng = Rng::new(0x50D8 + case);
        let radius = rng.range(0.5, 5.0);
        let height = rng.range(0.5, 8.0);
        let repro = format!(
            "seed 0x{:04X} (cargo test --test step_pcurve_nurbs_random \
             cylinder_double_roundtrip), r {radius:.6}, h {height:.6}",
            0x50D8 + case
        );
        let expected = std::f64::consts::PI * radius * radius * height;

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body =
            primitives::cylinder(&mut store, &mut geo, radius, height).expect("valid cylinder");
        // A fresh primitive has no trim geometry; attach it so the writer
        // has pcurves to spell — one of which is the clockwise rim.
        attach_body_pcurves(&mut store, &mut geo, body);

        let mut text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
            .unwrap_or_else(|e| panic!("{repro}: cylinder must serialize: {e}"));
        assert!(
            text.contains("RATIONAL_B_SPLINE_CURVE"),
            "{repro}: the clockwise rim must take the rational 2D spelling:\n{text}"
        );
        for trip in 0..2 {
            let (store2, geo2, report) = import(&text);
            no_error_diagnostics(&report, &repro);
            let body2 = brep_body(&report, &repro);
            assert_clean(&store2, &geo2, body2, &repro);
            let mp = brep_mass_properties(&store2, &geo2, body2)
                .unwrap_or_else(|e| panic!("{repro}: trip {trip}: must measure: {e}"));
            assert!(
                (mp.volume - expected).abs() < 1e-9 * expected,
                "{repro}: trip {trip}: volume {} is not {expected}",
                mp.volume
            );
            text = write_step(&store2, &geo2, &[body2], &StepWriteOptions::default())
                .unwrap_or_else(|e| panic!("{repro}: trip {trip}: must re-serialize: {e}"));
        }
    }
}
