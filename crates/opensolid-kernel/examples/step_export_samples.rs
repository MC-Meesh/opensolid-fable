//! Write a set of sample STEP exports plus a `manifest.json` of analytic
//! expected volumes, for the external validator (of-3qy.10).
//!
//! The samples mirror the round-trip suite in `tests/step_corpus.rs`:
//! primitives and boolean outputs whose volumes are known in closed form. An
//! external CAD system (headless FreeCAD / OCC — see `tools/step-validator/`)
//! imports each file, checks the shape is a valid closed solid, and compares
//! its exactly-computed volume against `expected_volume`.
//!
//! ```bash
//! cargo run --release --example step_export_samples -- /tmp/opensolid-step-samples
//! ```

use std::f64::consts::PI;
use std::path::Path;

use opensolid_kernel::brep::boolean::{intersect, subtract, unite};
use opensolid_kernel::brep::{GeometryStore, TopologyStore, primitives, translate_body};
use opensolid_kernel::core::tolerance::ToleranceContext;
use opensolid_kernel::core::types::Vector3;
use opensolid_kernel::io::step::write::{StepWriteOptions, write_step};

struct Sample {
    name: &'static str,
    /// Analytic volume in mm³.
    expected_volume: f64,
    text: String,
}

fn main() {
    let out = std::env::args().nth(1).unwrap_or_else(|| {
        eprintln!("usage: step_export_samples <output-dir>");
        std::process::exit(2);
    });
    let out = Path::new(&out);
    std::fs::create_dir_all(out).expect("create output dir");

    let tol = ToleranceContext::default();
    let opts = StepWriteOptions::default();
    let mut samples: Vec<Sample> = Vec::new();

    // ── Primitives ────────────────────────────────────────────────────────
    {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 20.0, 30.0, 40.0).expect("block");
        samples.push(Sample {
            name: "block",
            expected_volume: 20.0 * 30.0 * 40.0,
            text: write_step(&store, &geo, &[body], &opts).expect("write block"),
        });
    }
    {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::cylinder(&mut store, &mut geo, 15.0, 40.0).expect("cylinder");
        samples.push(Sample {
            name: "cylinder",
            expected_volume: PI * 15.0 * 15.0 * 40.0,
            text: write_step(&store, &geo, &[body], &opts).expect("write cylinder"),
        });
    }
    {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::sphere(&mut store, &mut geo, 20.0).expect("sphere");
        samples.push(Sample {
            name: "sphere",
            expected_volume: 4.0 / 3.0 * PI * 20.0f64.powi(3),
            text: write_step(&store, &geo, &[body], &opts).expect("write sphere"),
        });
    }
    {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::torus(&mut store, &mut geo, 30.0, 10.0).expect("torus");
        samples.push(Sample {
            name: "torus",
            expected_volume: 2.0 * PI * PI * 30.0 * 10.0 * 10.0,
            text: write_step(&store, &geo, &[body], &opts).expect("write torus"),
        });
    }
    {
        // Placement must survive export, not just shape.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = primitives::block(&mut store, &mut geo, 20.0, 30.0, 40.0).expect("block");
        translate_body(
            &mut store,
            &mut geo,
            body,
            Vector3::new(107.5, -33.25, 9.125),
        )
        .expect("translate");
        samples.push(Sample {
            name: "block-translated",
            expected_volume: 20.0 * 30.0 * 40.0,
            text: write_step(&store, &geo, &[body], &opts).expect("write translated block"),
        });
    }

    // ── Boolean outputs ───────────────────────────────────────────────────
    {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let a = primitives::block(&mut store, &mut geo, 20.0, 20.0, 20.0).expect("block a");
        let b = primitives::block(&mut store, &mut geo, 20.0, 20.0, 20.0).expect("block b");
        translate_body(&mut store, &mut geo, b, Vector3::new(10.0, 10.0, 10.0))
            .expect("translate b");
        let union = unite(&store, &geo, a, b, &tol).expect("unite");
        samples.push(Sample {
            name: "union-blocks",
            expected_volume: 8000.0 + 8000.0 - 1000.0,
            text: write_step(&union.store, &union.geo, &[union.body], &opts).expect("write union"),
        });
        let inter = intersect(&store, &geo, a, b, &tol).expect("intersect");
        samples.push(Sample {
            name: "intersect-blocks",
            expected_volume: 1000.0,
            text: write_step(&inter.store, &inter.geo, &[inter.body], &opts)
                .expect("write intersection"),
        });
        let diff = subtract(&store, &geo, a, b, &tol).expect("subtract");
        samples.push(Sample {
            name: "subtract-blocks",
            expected_volume: 7000.0,
            text: write_step(&diff.store, &diff.geo, &[diff.body], &opts)
                .expect("write subtraction"),
        });
    }
    {
        // Through-hole: ring loops (FACE_BOUND vs FACE_OUTER_BOUND) on export.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let a = primitives::block(&mut store, &mut geo, 40.0, 40.0, 20.0).expect("block");
        let b = primitives::cylinder(&mut store, &mut geo, 8.0, 40.0).expect("cylinder");
        let out_bool = subtract(&store, &geo, a, b, &tol).expect("subtract");
        samples.push(Sample {
            name: "block-through-hole",
            expected_volume: 40.0 * 40.0 * 20.0 - PI * 8.0 * 8.0 * 20.0,
            text: write_step(&out_bool.store, &out_bool.geo, &[out_bool.body], &opts)
                .expect("write through-hole"),
        });
    }
    {
        // Partial-wrap cylindrical band: quarter-cylinder notch on an edge.
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let a = primitives::block(&mut store, &mut geo, 20.0, 20.0, 20.0).expect("block");
        let c = primitives::cylinder(&mut store, &mut geo, 4.0, 30.0).expect("cylinder");
        translate_body(&mut store, &mut geo, c, Vector3::new(10.0, 10.0, 0.0)).expect("translate");
        let out_bool = subtract(&store, &geo, a, c, &tol).expect("subtract");
        samples.push(Sample {
            name: "edge-notch",
            expected_volume: 8000.0 - PI * 4.0 * 4.0 / 4.0 * 20.0,
            text: write_step(&out_bool.store, &out_bool.geo, &[out_bool.body], &opts)
                .expect("write edge notch"),
        });
    }

    // ── Write files + manifest ────────────────────────────────────────────
    let mut manifest = String::from("[\n");
    for (i, s) in samples.iter().enumerate() {
        let file = format!("{}.step", s.name);
        std::fs::write(out.join(&file), &s.text).expect("write sample");
        manifest.push_str(&format!(
            "  {{\"file\": \"{file}\", \"expected_volume\": {}}}{}\n",
            s.expected_volume,
            if i + 1 < samples.len() { "," } else { "" }
        ));
        println!("wrote {file} ({} bytes)", s.text.len());
    }
    manifest.push_str("]\n");
    std::fs::write(out.join("manifest.json"), manifest).expect("write manifest");
    println!(
        "wrote manifest.json ({} samples) to {}",
        samples.len(),
        out.display()
    );
}
