//! Regression pins for of-0een: a writer sphere whose `SPHERICAL_SURFACE`
//! has strayed under its vertices by less than the raised-tolerance cap is
//! kept on the exact path — and the resulting B-Rep must then be
//! *measurable*.
//!
//! The failure this pins: the reader kept the body (the stray's off-surface
//! component sits inside `MAX_ALLOWED_TOLERANCE`, `record_edge_tolerances`
//! raised the edge tolerance) and returned `SolidOutcome::BRep`, but the
//! seam pcurves refit against the strayed surface end where the pole
//! vertices *project* — a hair off the collapsed pole row — and
//! `brep_mass_properties` refused the loop as open by exactly 2π. An import
//! that reports exact but cannot be measured is a silent degrade; the loop
//! genuinely closes through the pole, and now does
//! (`collapsed_row_detour` in `brep_massprops`).
//!
//! Cases are the concrete survey hits recorded on the bead, replayed
//! deterministically. The randomized campaign lives in
//! `step_mesh_fallback_random.rs` (of-q7yz).

use opensolid_kernel::brep::{GeometryStore, TopologyStore, primitives};
use opensolid_kernel::brep_mass_properties;
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

/// The survey hits recorded on of-0een: `(radius, amplitude, direction)`.
/// Each keeps the sphere on the exact path — the off-surface component of
/// the stray sits inside the tolerance cap — and each used to refuse with
/// `OpenParameterLoop { gap: 2π }`.
const CASES: [(f64, f64, [f64; 3]); 3] = [
    (7.9204, 0.0457, [0.1869, -0.9784, -0.0880]),
    (4.5420, 0.0241, [0.1266, 0.9230, 0.3634]),
    (8.6605, 0.0451, [-0.0915, -0.9804, -0.1745]),
];

#[test]
fn an_exactly_kept_strayed_sphere_measures_its_volume() {
    for (radius, amplitude, dir) in CASES {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::sphere(&mut store, &mut geo, radius).expect("valid radius");
        let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
            .expect("writer sphere");
        let delta = [dir[0] * amplitude, dir[1] * amplitude, dir[2] * amplitude];
        let strayed = displace_sphere_anchor(&text, delta);

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let report = read_step(&strayed, &mut store, &mut geo, &StepReadOptions::default())
            .expect("valid Part 21");
        assert_eq!(report.solids.len(), 1, "r = {radius}: one solid");
        let SolidOutcome::BRep(imported) = &report.solids[0].outcome else {
            panic!(
                "r = {radius}: a stray inside the tolerance cap must stay on the \
                 exact path, got {:?}. Diagnostics:\n{}",
                report.solids[0].outcome,
                report
                    .diagnostics
                    .iter()
                    .map(|d| format!("  {:?}: {}", d.severity, d.message))
                    .collect::<Vec<_>>()
                    .join("\n"),
            );
        };

        let measured = brep_mass_properties(&store, &geo, *imported)
            .unwrap_or_else(|e| panic!("r = {radius}: an exact import must be measurable: {e:?}"))
            .volume;
        let expected = 4.0 / 3.0 * PI * radius * radius * radius;
        let drift = (measured - expected).abs() / expected;
        assert!(
            drift <= 0.01,
            "r = {radius}: volume {measured:.6} vs {expected:.6} (drift {drift:.2e})"
        );
    }
}
