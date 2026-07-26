#![no_main]
//! Thin libFuzzer wrapper. The harness lives in `opensolid_fuzz::topology_check` so it
//! can also be replayed on stable Rust by `tests/corpus_replay.rs`.

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    opensolid_fuzz::fuzz_topology_check(data);
});
