//! Adversarial pair for of-0een (of-abi3): stress the exact-path sphere
//! mass-properties fix — `collapsed_row_detour` in `brep_massprops` — across
//! the whole envelope the reader's exact path actually keeps, not just the
//! three survey hits the fix pinned.
//!
//! The geometry of the attack surface: a stray `d` of the `SPHERICAL_SURFACE`
//! anchor decomposes at the seam (the u = 0 meridian, in the x-z plane) into
//! an in-plane part, which moves every seam sample off the surface by
//! ~`|d_inplane|` (first order), and a perpendicular part `d_y`, which moves
//! them off by only `d_y²/2r` (second order). The reader keeps the exact path
//! while the off-surface component stays inside `MAX_ALLOWED_TOLERANCE`
//! (0.01), so a perpendicular stray as large as `√(0.02·r)` is *kept exact* —
//! while the pole-vertex projection lands `~d_y/r` radians off the collapsed
//! pole row. `collapsed_row_detour` only reaches `GAP_TOL_REL · extent`
//! (≈ 0.07 rad for a full sphere), so radii below ~4 leave a kept-exact,
//! refused-to-measure gap — the exact of-0een silent-degrade shape, one
//! tolerance ring further out.
//!
//! Failures found here are filed as their own beads; each failing case is
//! pinned `#[ignore]` on its bead rather than softened.

use opensolid_kernel::MassProperties;
use opensolid_kernel::brep::{Body, GeometryStore, TopologyStore, primitives};
use opensolid_kernel::brep_mass_properties;
use opensolid_kernel::core::EntityId;
use opensolid_kernel::io::step::read::{SolidOutcome, StepReadOptions, read_step};
use opensolid_kernel::io::step::write::{StepWriteOptions, write_step};
use std::f64::consts::PI;

/// The data-section line defining `#id`.
fn record_line(text: &str, id: u64) -> &str {
    let assigned = format!("#{id}=");
    let spaced = format!("#{id} ");
    text.lines()
        .find(|l| l.starts_with(&assigned) || l.starts_with(&spaced))
        .unwrap_or_else(|| panic!("no record #{id}"))
}

/// The first `#id` referenced after the `=`.
fn first_ref(line: &str) -> u64 {
    let (_, body) = line.split_once('=').expect("a record");
    let hash = body.find('#').expect("a reference");
    body[hash + 1..]
        .chars()
        .take_while(|c| c.is_ascii_digit())
        .collect::<String>()
        .parse()
        .expect("a numeric id")
}

/// Add `delta` to the sphere's anchor: the `CARTESIAN_POINT` behind the
/// `SPHERICAL_SURFACE`'s `AXIS2_PLACEMENT_3D`. The edges keep their
/// vertices; only the surface strays under them.
fn displace_sphere_anchor(text: &str, delta: [f64; 3]) -> String {
    let surface = text
        .lines()
        .find(|l| l.contains("= SPHERICAL_SURFACE("))
        .expect("a writer sphere carries one SPHERICAL_SURFACE");
    let placement = first_ref(surface);
    let point_id = first_ref(record_line(text, placement));
    let target = record_line(text, point_id).to_string();
    assert!(
        target.contains("CARTESIAN_POINT"),
        "#{point_id} is not a CARTESIAN_POINT: {target}"
    );
    let open = target.rfind('(').expect("coordinate tuple");
    let close = target[open..].find(')').expect("closed tuple") + open;
    let coords: Vec<f64> = target[open + 1..close]
        .split(',')
        .map(|c| c.trim().parse().expect("numeric coordinate"))
        .collect();
    assert_eq!(coords.len(), 3, "3D point");
    let moved = format!(
        "({:.9},{:.9},{:.9})",
        coords[0] + delta[0],
        coords[1] + delta[1],
        coords[2] + delta[2]
    );
    let replaced = format!("{}{}{}", &target[..open], moved, &target[close + 1..]);
    text.replace(&target, &replaced)
}

/// What the import did with a strayed writer sphere.
enum Strayed {
    /// Kept on the exact path; the imported body and its stores.
    Exact(Box<TopologyStore>, GeometryStore, EntityId<Body>),
    /// The reader degraded away from the exact path (its right, not a bug).
    Degraded(String),
}

/// Round-trip a writer sphere with its `SPHERICAL_SURFACE` anchor displaced
/// by `amplitude · dir`, vertices left in place.
fn import_strayed_sphere(radius: f64, amplitude: f64, dir: [f64; 3]) -> Strayed {
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = primitives::sphere(&mut store, &mut geo, radius).expect("valid radius");
    let text =
        write_step(&store, &geo, &[body], &StepWriteOptions::default()).expect("writer sphere");
    let delta = [dir[0] * amplitude, dir[1] * amplitude, dir[2] * amplitude];
    let strayed = displace_sphere_anchor(&text, delta);

    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step(&strayed, &mut store, &mut geo, &StepReadOptions::default())
        .expect("valid Part 21");
    assert_eq!(report.solids.len(), 1, "r = {radius}: one solid");
    match &report.solids[0].outcome {
        SolidOutcome::BRep(imported) => Strayed::Exact(Box::new(store), geo, *imported),
        other => Strayed::Degraded(format!("{other:?}")),
    }
}

/// Measure, panicking with a repro string on refusal.
fn must_measure(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    what: &str,
) -> MassProperties {
    brep_mass_properties(store, geo, body)
        .unwrap_or_else(|e| panic!("{what}: kept-exact import must be measurable: {e:?}"))
}

fn assert_rel(actual: f64, expected: f64, tol: f64, what: &str) {
    let drift = (actual - expected).abs() / expected.abs().max(f64::MIN_POSITIVE);
    assert!(
        drift <= tol,
        "{what}: {actual:.9} vs expected {expected:.9} (drift {drift:.3e} > {tol:.0e})"
    );
}

/// The merged fix's own cases (the of-0een survey hits), held to the full
/// contract this time: volume and area as merged, plus centroid — which must
/// sit at the *strayed center* (the body is bounded by the strayed surface;
/// the untouched vertices carry no mass) — and the inertia diagonal of a
/// solid ball about its centroid.
#[test]
fn survey_hit_spheres_measure_centroid_and_inertia_too() {
    let cases: [(f64, f64, [f64; 3]); 3] = [
        (7.9204, 0.0457, [0.1869, -0.9784, -0.0880]),
        (4.5420, 0.0241, [0.1266, 0.9230, 0.3634]),
        (8.6605, 0.0451, [-0.0915, -0.9804, -0.1745]),
    ];
    for (radius, amplitude, dir) in cases {
        let Strayed::Exact(store, geo, body) = import_strayed_sphere(radius, amplitude, dir) else {
            panic!("r = {radius}: the merged fix's own case must stay exact");
        };
        let mp = must_measure(&store, &geo, body, &format!("r = {radius}"));
        let volume = 4.0 / 3.0 * PI * radius.powi(3);
        assert_rel(mp.volume, volume, 1e-2, &format!("r = {radius}: volume"));
        assert_rel(
            mp.surface_area,
            4.0 * PI * radius * radius,
            1e-2,
            &format!("r = {radius}: area"),
        );
        let center = [dir[0] * amplitude, dir[1] * amplitude, dir[2] * amplitude];
        for (axis, (got, want)) in ["x", "y", "z"].iter().zip([
            (mp.centroid.x, center[0]),
            (mp.centroid.y, center[1]),
            (mp.centroid.z, center[2]),
        ]) {
            assert!(
                (got - want).abs() <= 1e-2 * radius,
                "r = {radius}: centroid.{axis} {got:.6} vs strayed center {want:.6}"
            );
        }
        let ball = 2.0 / 5.0 * mp.volume * radius * radius;
        for (axis, moment) in ["xx", "yy", "zz"].iter().zip([
            mp.inertia[(0, 0)],
            mp.inertia[(1, 1)],
            mp.inertia[(2, 2)],
        ]) {
            assert_rel(moment, ball, 2e-2, &format!("r = {radius}: inertia {axis}"));
        }
    }
}

/// A stray along the pole axis moves the pole vertices radially: their
/// projections stay *on* the collapsed row and the seam's off-surface
/// component is first-order (capped at 0.01), so the classic
/// `is_collapsed_bridge` path must keep working underneath the new detour.
#[test]
fn axis_stray_sphere_still_measures() {
    for (radius, amplitude) in [(5.0, 0.009), (1.5, 0.009), (9.3, 0.0095)] {
        match import_strayed_sphere(radius, amplitude, [0.0, 0.0, 1.0]) {
            Strayed::Exact(store, geo, body) => {
                let mp = must_measure(&store, &geo, body, &format!("axis stray r = {radius}"));
                assert_rel(
                    mp.volume,
                    4.0 / 3.0 * PI * radius.powi(3),
                    1e-2,
                    &format!("axis stray r = {radius}: volume"),
                );
            }
            Strayed::Degraded(outcome) => {
                panic!("axis stray r = {radius}, 0.009 < cap must stay exact, got {outcome}")
            }
        }
    }
}

/// The tolerance-envelope attack: a stray perpendicular to the seam plane is
/// second-order along the whole seam (`d²/2r`), so the reader keeps the
/// exact path for strays up to `√(0.02·r)` — while the pole projections land
/// `d/r` radians off the collapsed row. For r < ~4 that offset clears
/// `GAP_TOL_REL · extent` and the detour gives up. Whatever the kernel
/// decides to do with these, it must not certify-then-refuse: if the import
/// stays exact, it must measure.
///
/// Found failing and filed as of-whcw: r = 1 and r = 2 import exact and then
/// refuse with `OpenParameterLoop { gap: 2π }`; r = 3 (offset 0.075 rad)
/// squeaks through only because the strayed pcurves' u-excursion inflates
/// `extent`. Pinned here, not softened.
#[test]
#[ignore = "of-whcw: kept-exact stray past the collapsed_row_detour reach still refuses"]
fn perpendicular_stray_near_cap_small_radius_must_measure_if_kept() {
    // (radius, amplitude): amplitude ≈ 0.92·√(0.02·r), safely inside the
    // keep-exact cap; pole offset amplitude/r ≈ 0.13, 0.092, 0.077 rad —
    // all past the ~0.070 detour reach.
    let mut failures = Vec::new();
    for (radius, amplitude) in [(1.0, 0.13), (2.0, 0.185), (3.0, 0.225)] {
        match import_strayed_sphere(radius, amplitude, [0.0, 1.0, 0.0]) {
            Strayed::Exact(store, geo, body) => match brep_mass_properties(&store, &geo, body) {
                Err(e) => failures.push(format!(
                    "refused: r = {radius}, d = {amplitude}, dir (0,1,0): {e:?}"
                )),
                Ok(mp) => assert_rel(
                    mp.volume,
                    4.0 / 3.0 * PI * radius.powi(3),
                    1e-2,
                    &format!("perpendicular stray r = {radius}: volume"),
                ),
            },
            // Degrading away from the exact path is a legitimate answer —
            // the of-0een contract is only "exact implies measurable".
            Strayed::Degraded(outcome) => {
                eprintln!("r = {radius}, d = {amplitude}: degraded ({outcome})");
            }
        }
    }
    assert!(
        failures.is_empty(),
        "{} kept-exact spheres refused to measure:\n{}",
        failures.len(),
        failures.join("\n")
    );
}

/// Same attack just *inside* the detour's reach: pole offset ≈ 0.05 rad.
/// These must stay exact (stray well under the cap) and must measure.
#[test]
fn perpendicular_stray_within_detour_reach_measures() {
    for radius in [4.0_f64, 6.0, 9.0] {
        let amplitude = 0.04 * radius; // pole offset 0.04 < 0.070 reach
        let residual = amplitude * amplitude / (2.0 * radius);
        assert!(residual < 0.0085, "case stays clear of the keep-exact cap");
        match import_strayed_sphere(radius, amplitude, [0.0, 1.0, 0.0]) {
            Strayed::Exact(store, geo, body) => {
                let mp = must_measure(
                    &store,
                    &geo,
                    body,
                    &format!("in-reach stray r = {radius}, d = {amplitude}"),
                );
                assert_rel(
                    mp.volume,
                    4.0 / 3.0 * PI * radius.powi(3),
                    1e-2,
                    &format!("in-reach stray r = {radius}: volume"),
                );
            }
            Strayed::Degraded(outcome) => panic!(
                "r = {radius}, stray {amplitude} (residual {residual:.4}) is well inside \
                 the exact-path cap, got {outcome}"
            ),
        }
    }
}

/// Seeded sweep of the whole kept-exact envelope: random radii, random
/// directions, amplitudes pushed toward whichever cap binds (first-order
/// in-plane 0.01, or second-order perpendicular `√(0.02·r)`). Every case the
/// reader keeps exact must measure with a sane volume. Deterministic; failures
/// print the exact `(radius, amplitude, dir)` triple for replay.
#[test]
fn kept_exact_envelope_sweep_always_measures() {
    // SplitMix64: deterministic, seedable, no deps. Seed spells "abi3" as
    // best hex can: this file's bead.
    let mut state: u64 = 0xab13_5eed_0000_0001;
    let mut next = move || {
        state = state.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = state;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        (z ^ (z >> 31)) as f64 / u64::MAX as f64
    };

    let mut kept = 0usize;
    let mut degraded = 0usize;
    let mut failures = Vec::new();
    for _ in 0..120 {
        let radius = 0.6 + next() * 9.0;
        // Direction: bias toward the seam-perpendicular axis half the time to
        // walk the second-order edge of the envelope.
        let bias = next() < 0.5;
        let (x, y, z) = if bias {
            (0.05 * (next() - 0.5), 1.0, 0.05 * (next() - 0.5))
        } else {
            (next() - 0.5, next() - 0.5, next() - 0.5)
        };
        let norm = (x * x + y * y + z * z).sqrt().max(f64::MIN_POSITIVE);
        let dir = [x / norm, y / norm, z / norm];
        // Off-surface first-order term comes from the in-plane (x, z) part;
        // second-order from the rest. Solve the binding cap approximately and
        // stay at 85% of it.
        let inplane = (dir[0] * dir[0] + dir[2] * dir[2]).sqrt();
        let cap = if inplane > 1e-3 {
            (0.01 / inplane).min((0.02 * radius).sqrt())
        } else {
            (0.02 * radius).sqrt()
        };
        // Known-bug exclusion (of-whcw): a pole-vertex projection more than
        // ~0.07 rad off the collapsed row is past `collapsed_row_detour`'s
        // reach and refuses even though the import kept the exact path. The
        // sweep stays inside 0.055 rad so it nets *new* regressions; the
        // outside is pinned in
        // `perpendicular_stray_near_cap_small_radius_must_measure_if_kept`.
        let tangential = (dir[0] * dir[0] + dir[1] * dir[1]).sqrt();
        let reach_cap = if tangential > 1e-3 {
            0.055 * radius / tangential
        } else {
            f64::INFINITY
        };
        let amplitude = 0.85 * cap.min(reach_cap) * next();

        match import_strayed_sphere(radius, amplitude, dir) {
            Strayed::Degraded(_) => degraded += 1,
            Strayed::Exact(store, geo, body) => {
                kept += 1;
                match brep_mass_properties(&store, &geo, body) {
                    Err(e) => failures.push(format!(
                        "refused: r={radius:.4}, d={amplitude:.4}, dir=({:.4},{:.4},{:.4}): {e:?}",
                        dir[0], dir[1], dir[2]
                    )),
                    Ok(mp) => {
                        let expected = 4.0 / 3.0 * PI * radius.powi(3);
                        let drift = (mp.volume - expected).abs() / expected;
                        if drift > 1e-2 {
                            failures.push(format!(
                                "mismeasured: r={radius:.4}, d={amplitude:.4}, \
                                 dir=({:.4},{:.4},{:.4}): volume {:.6} vs {expected:.6} \
                                 (drift {drift:.2e})",
                                dir[0], dir[1], dir[2], mp.volume
                            ));
                        }
                    }
                }
            }
        }
    }
    assert!(
        kept >= 20,
        "sweep must actually exercise the exact path (kept {kept}, degraded {degraded})"
    );
    assert!(
        failures.is_empty(),
        "{} kept-exact spheres failed the of-0een contract:\n{}",
        failures.len(),
        failures.join("\n")
    );
}
