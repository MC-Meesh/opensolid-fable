//! Import every STEP file given on the command line (or every `.stp`/`.step`
//! under a directory argument) and print one line per file: outcome counts,
//! diagnostics summary, healing operations applied, and a final pass-rate
//! figure.
//!
//! This is the local half of the external-validator loop (of-3qy.10): point it
//! at `tests/data/step/` or at a directory of FreeCAD/OCC exports to measure
//! the import pass rate the spec tracks (spec/06-step-io.md §Pass-rate
//! targets).
//!
//! ```bash
//! cargo run --release --example step_import_report -- crates/opensolid-kernel/tests/data/step
//! ```

use std::path::{Path, PathBuf};

use opensolid_kernel::brep::{GeometryStore, TopologyStore};
use opensolid_kernel::io::step::read::{Severity, SolidOutcome, StepReadOptions, read_step_bytes};

fn collect(path: &Path, out: &mut Vec<PathBuf>) {
    if path.is_dir() {
        let mut entries: Vec<_> = std::fs::read_dir(path)
            .unwrap_or_else(|e| panic!("cannot read dir {}: {e}", path.display()))
            .map(|e| e.expect("dir entry").path())
            .collect();
        entries.sort();
        for entry in entries {
            collect(&entry, out);
        }
    } else if path
        .extension()
        .and_then(|e| e.to_str())
        .is_some_and(|e| e.eq_ignore_ascii_case("stp") || e.eq_ignore_ascii_case("step"))
    {
        out.push(path.to_path_buf());
    }
}

fn main() {
    let mut args: Vec<String> = std::env::args().skip(1).collect();
    // --min-rate <pct>: exit non-zero when the pass rate lands below the
    // floor, so a CI job can gate on the metric once coverage supports it.
    let mut min_rate = 0.0f64;
    if let Some(pos) = args.iter().position(|a| a == "--min-rate") {
        let value = args
            .get(pos + 1)
            .and_then(|v| v.parse::<f64>().ok())
            .unwrap_or_else(|| {
                eprintln!("--min-rate needs a percentage, e.g. --min-rate 80");
                std::process::exit(2);
            });
        min_rate = value;
        args.drain(pos..=pos + 1);
    }
    if args.is_empty() {
        eprintln!("usage: step_import_report [--min-rate <pct>] <file-or-dir> [...]");
        std::process::exit(2);
    }
    let mut files = Vec::new();
    for arg in &args {
        collect(Path::new(arg), &mut files);
    }
    if files.is_empty() {
        eprintln!("no .stp/.step files found under {args:?}");
        std::process::exit(2);
    }

    let mut passed = 0usize;
    let mut healed_files = 0usize;
    let mut heal_operations = 0usize;
    for file in &files {
        let bytes = match std::fs::read(file) {
            Ok(b) => b,
            Err(e) => {
                println!("READ-ERR   {}: {e}", file.display());
                continue;
            }
        };
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let name = file.file_name().unwrap_or_default().to_string_lossy();
        match read_step_bytes(&bytes, &mut store, &mut geo, &StepReadOptions::default()) {
            Err(e) => println!("PARSE-ERR  {name}: {e}"),
            Ok(report) => {
                let (mut brep, mut mesh, mut failed) = (0usize, 0usize, 0usize);
                for solid in &report.solids {
                    match &solid.outcome {
                        SolidOutcome::BRep(_) => brep += 1,
                        SolidOutcome::Mesh { .. } => mesh += 1,
                        SolidOutcome::Failed => failed += 1,
                    }
                }
                let errors = report
                    .diagnostics
                    .iter()
                    .filter(|d| d.severity >= Severity::Error)
                    .count();
                let warnings = report
                    .diagnostics
                    .iter()
                    .filter(|d| d.severity == Severity::Warning)
                    .count();
                // "Pass" for the spec's pass-rate metric: at least one solid
                // and every solid imported (exactly or as a mesh fallback).
                let ok = !report.solids.is_empty() && failed == 0;
                if ok {
                    passed += 1;
                }
                // Repairs the healer applied (of-3qy.12). A file with a
                // non-zero count imported only because healing fixed it, or
                // would have degraded further without it.
                if report.heal_operations > 0 {
                    healed_files += 1;
                    heal_operations += report.heal_operations;
                }
                println!(
                    "{}  {name}: {} solid(s) — {brep} exact, {mesh} mesh, {failed} failed; \
                     {errors} error(s), {warnings} warning(s), {} heal op(s)",
                    if ok { "PASS      " } else { "FAIL      " },
                    report.solids.len(),
                    report.heal_operations,
                );
                if !ok {
                    for d in report.diagnostics.iter().take(4) {
                        println!("             [{:?}] {}", d.severity, d.message);
                    }
                }
            }
        }
    }
    let rate = 100.0 * passed as f64 / files.len() as f64;
    println!("\npass rate: {passed}/{} ({rate:.0}%)", files.len());
    println!("healing: {heal_operations} operation(s) across {healed_files} file(s)");
    if rate < min_rate {
        eprintln!("pass rate {rate:.0}% is below the --min-rate floor of {min_rate:.0}%");
        std::process::exit(1);
    }
}
