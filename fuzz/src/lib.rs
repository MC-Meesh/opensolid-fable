//! Fuzz harnesses for the OpenSolid kernel.
//!
//! The bodies of the [`cargo-fuzz`] targets live here rather than in
//! `fuzz_targets/*.rs`, for two reasons:
//!
//! 1. `fuzz_targets/*.rs` can only be built by a nightly toolchain with the
//!    sanitizer flags libFuzzer needs. This library builds on stable, so
//!    `tests/corpus_replay.rs` can re-run every committed seed and every
//!    committed crash artifact as an ordinary `cargo test` — the fuzzer finds
//!    a bug once, the replay test keeps it fixed forever.
//! 2. Harness logic that is worth writing (structured input decoding, the
//!    post-conditions each target asserts) is worth unit-testing.
//!
//! # The contract each target checks
//!
//! Every entry point below is a *total function of arbitrary bytes*. None of
//! them may panic, abort, hang, or grow memory without bound, no matter what
//! the input is. Malformed input must come back as an `Err`/diagnostic, never
//! as a crash — this is the "no-panic, no-hang" contract that motivated the
//! issue (of-1dd was a real stack overflow in the STEP parser at 500-deep
//! aggregate nesting, found by a hand-written adversarial file; a fuzzer finds
//! that class mechanically).
//!
//! Beyond "does not crash", the harnesses assert *semantic* post-conditions
//! where a cheap and unambiguous one exists — see the per-module docs. Those
//! are what turn a crash-fuzzer into a correctness fuzzer.
//!
//! [`cargo-fuzz`]: https://rust-fuzzing.github.io/book/cargo-fuzz.html

pub mod nurbs;
pub mod step;
pub mod topology;

pub use nurbs::fuzz_nurbs_eval;
pub use step::fuzz_step_parse;
pub use topology::fuzz_topology_check;

/// A harness entry point: replays one fuzzer input, asserting that target's
/// contract.
pub type FuzzTarget = fn(&[u8]);

/// Every fuzz target's name, paired with the harness entry point that replays
/// one input for it.
///
/// Keeping the mapping in one place is what lets `tests/corpus_replay.rs` walk
/// `seeds/`, `corpus/` and `artifacts/` generically: adding a target here (and
/// a `fuzz_targets/<name>.rs` wrapper) is all it takes to bring its corpus
/// under the stable-toolchain regression gate.
pub const TARGETS: &[(&str, FuzzTarget)] = &[
    ("step_parse", fuzz_step_parse),
    ("topology_check", fuzz_topology_check),
    ("nurbs_eval", fuzz_nurbs_eval),
];

/// Look up a harness entry point by target name.
pub fn target(name: &str) -> Option<FuzzTarget> {
    TARGETS
        .iter()
        .find(|(n, _)| *n == name)
        .map(|&(_, run)| run)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The degenerate inputs every target sees within the first second of a
    /// run. Cheap to check here, and it keeps a target from regressing into
    /// "panics on empty input" between fuzz sessions.
    #[test]
    fn every_target_survives_trivial_input() {
        let trivial: &[&[u8]] = &[
            b"",
            b"\0",
            b"\xff",
            &[0u8; 64],
            &[0xffu8; 64],
            b"ISO-10303-21;",
        ];
        for &(name, run) in TARGETS {
            for input in trivial {
                run(input);
                let _ = name;
            }
        }
    }

    #[test]
    fn target_lookup_matches_the_table() {
        assert!(target("step_parse").is_some());
        assert!(target("topology_check").is_some());
        assert!(target("nurbs_eval").is_some());
        assert!(target("no_such_target").is_none());
        assert_eq!(TARGETS.len(), 3);
    }
}
