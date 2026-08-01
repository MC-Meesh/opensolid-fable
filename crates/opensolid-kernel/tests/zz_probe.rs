//! Temporary probe (of-8ulj) — delete before commit.

use opensolid_kernel::brep::{Body, GeometryStore, TopologyStore};
use opensolid_kernel::core::EntityId;
use opensolid_kernel::io::step::read::{SolidOutcome, StepReadOptions, read_step_bytes};

#[test]
fn probe_bspline_patch_prism() {
    let path = format!(
        "{}/tests/data/step/occ/nurbs/bspline_patch_prism.stp",
        env!("CARGO_MANIFEST_DIR")
    );
    let bytes = std::fs::read(&path).unwrap();
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report =
        read_step_bytes(&bytes, &mut store, &mut geo, &StepReadOptions::default()).unwrap();
    println!("diagnostics: {:#?}", report.diagnostics);
    let breps: Vec<EntityId<Body>> = report
        .solids
        .iter()
        .filter_map(|s| match &s.outcome {
            SolidOutcome::BRep(b) => Some(*b),
            _ => None,
        })
        .collect();
    println!("brep count {}", breps.len());
    for &body in &breps {
        let failures = store.check_geometry(&geo, body);
        println!("check_geometry: {} failures", failures.len());
        for f in failures.iter().take(20) {
            println!("  {f:?}");
        }
        println!("check: {:?}", store.check(body));
    }
}
