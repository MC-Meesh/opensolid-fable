#![no_main]
//! Thin libFuzzer wrapper. The harness lives in `opensolid_fuzz::step_parse` so it
//! can also be replayed on stable Rust by `tests/corpus_replay.rs`.

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    opensolid_fuzz::fuzz_step_parse(data);
});
