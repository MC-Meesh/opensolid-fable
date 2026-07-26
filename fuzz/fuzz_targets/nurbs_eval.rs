#![no_main]
//! Thin libFuzzer wrapper. The harness lives in `opensolid_fuzz::nurbs_eval` so it
//! can also be replayed on stable Rust by `tests/corpus_replay.rs`.

use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: &[u8]| {
    opensolid_fuzz::fuzz_nurbs_eval(data);
});
