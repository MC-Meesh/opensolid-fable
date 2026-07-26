//! Stable-toolchain regression gate over every fuzz corpus.
//!
//! `cargo fuzz` needs nightly and sanitizer flags; this test needs neither. It
//! replays each target's committed seeds, its local working corpus, and — most
//! importantly — every crash artifact ever committed under `fuzz/artifacts/`,
//! through the same harness code the fuzzer runs.
//!
//! That is what makes fuzzing pay off on a project whose default CI is stable
//! Rust: the fuzzer finds a crasher once, the artifact is committed, and from
//! then on `cargo test` re-checks the fix on every push, forever, at the cost
//! of a few milliseconds.
//!
//! Run it with:
//!
//! ```sh
//! cargo test --manifest-path fuzz/Cargo.toml --no-default-features
//! ```
//!
//! `--no-default-features` drops `libfuzzer-sys`, so the `fuzz_targets/*.rs`
//! binaries (which need nightly) are not built.

use std::path::{Path, PathBuf};

/// Directories searched for inputs, relative to the fuzz package root.
///
/// * `seeds/` is committed: hand-written adversarial cases plus starting
///   points harvested from the real AP203 corpus.
/// * `corpus/` is libFuzzer's own working corpus — gitignored, present only
///   on a machine that has run the fuzzer.
/// * `artifacts/` is where libFuzzer writes a reproducer when a target
///   crashes; committing those is what turns a fuzz finding into a
///   permanent regression test.
const INPUT_DIRS: &[&str] = &["seeds", "corpus", "artifacts"];

fn package_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
}

/// Every regular file directly inside `dir`, sorted for a stable run order.
/// Missing directories are not an error — a fresh checkout has no `corpus/`.
fn inputs_in(dir: &Path) -> Vec<PathBuf> {
    let Ok(entries) = std::fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut files: Vec<PathBuf> = entries
        .filter_map(Result::ok)
        .map(|e| e.path())
        .filter(|p| p.is_file())
        .filter(|p| p.file_name().is_some_and(|n| n != ".gitkeep"))
        .collect();
    files.sort();
    files
}

/// Replay every input found for `target`, returning how many ran.
fn replay(target: &str, run: fn(&[u8])) -> usize {
    let root = package_root();
    let mut count = 0;
    for dir in INPUT_DIRS {
        for path in inputs_in(&root.join(dir).join(target)) {
            let data = std::fs::read(&path)
                .unwrap_or_else(|e| panic!("cannot read {}: {e}", path.display()));
            // A panic here names the offending file, which is the whole point:
            // the failure message is the reproducer path.
            println!("replaying {}", path.display());
            run(&data);
            count += 1;
        }
    }
    count
}

#[test]
fn step_parse_corpus() {
    let run = opensolid_fuzz::target("step_parse").unwrap();
    let count = replay("step_parse", run);
    assert!(
        count > 0,
        "no inputs found for step_parse; the committed seed corpus is missing"
    );
}

#[test]
fn topology_check_corpus() {
    let run = opensolid_fuzz::target("topology_check").unwrap();
    let count = replay("topology_check", run);
    assert!(
        count > 0,
        "no inputs found for topology_check; the committed seed corpus is missing"
    );
}

#[test]
fn nurbs_eval_corpus() {
    let run = opensolid_fuzz::target("nurbs_eval").unwrap();
    let count = replay("nurbs_eval", run);
    assert!(
        count > 0,
        "no inputs found for nurbs_eval; the committed seed corpus is missing"
    );
}

/// The kernel's own STEP test data, wherever it lives under `tests/data/step`.
fn kernel_step_data() -> PathBuf {
    package_root()
        .join("..")
        .join("crates")
        .join("opensolid-kernel")
        .join("tests")
        .join("data")
        .join("step")
}

/// Every `.stp` under `dir`, at any depth, sorted.
///
/// Deliberately recursive rather than a hardcoded list of subdirectories: the
/// corpus grows (of-ipt.16 added `occ/{blend,coincident,nurbs,periodic,
/// tangent,thin}`), and a new edge-case file should come under the fuzz
/// harness the moment someone adds it, without anyone remembering to update
/// this test.
fn step_files_under(dir: &Path) -> Vec<PathBuf> {
    let mut files = Vec::new();
    let mut stack = vec![dir.to_path_buf()];
    while let Some(current) = stack.pop() {
        let Ok(entries) = std::fs::read_dir(&current) else {
            continue;
        };
        for entry in entries.filter_map(Result::ok) {
            let path = entry.path();
            if path.is_dir() {
                stack.push(path);
            } else if path
                .extension()
                .is_some_and(|e| e.eq_ignore_ascii_case("stp"))
            {
                files.push(path);
            }
        }
    }
    files.sort();
    files
}

/// The real AP203 files the kernel already ships as test data are the best
/// possible seeds for the STEP target: they exercise entity types no synthetic
/// seed would think to include. They are too large to duplicate into
/// `fuzz/seeds/`, so the fuzzer is pointed at them in place (see
/// `fuzz/README.md`) and the replay test reads them from where they live.
#[test]
fn step_parse_survives_the_kernel_test_corpus() {
    let data_dir = kernel_step_data();
    let run = opensolid_fuzz::target("step_parse").unwrap();

    let files = step_files_under(&data_dir);
    assert!(
        !files.is_empty(),
        "no .stp files under {}",
        data_dir.display()
    );

    for path in files {
        let data = std::fs::read(&path).expect("readable test data");
        println!("replaying {}", path.display());
        run(&data);
    }
}

/// Truncating a real file at a byte boundary is the cheapest source of
/// almost-valid input there is, and it is exactly what a corrupted download or
/// an interrupted export produces. Every prefix of a real AP203 file must be
/// rejected or parsed, never crash.
#[test]
fn step_parse_survives_truncated_real_files() {
    let path = kernel_step_data().join("sg1-c5-214.stp");
    let data = std::fs::read(&path).expect("readable test data");
    let run = opensolid_fuzz::target("step_parse").unwrap();

    // 200 evenly spaced prefixes: full coverage of a 23 KiB file would be
    // 23,000 imports, which is a fuzzing session, not a unit test.
    for i in 0..200 {
        let end = data.len() * i / 200;
        run(&data[..end]);
    }
}
