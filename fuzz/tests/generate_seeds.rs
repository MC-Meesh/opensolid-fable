//! Regenerates the committed seed corpora under `fuzz/seeds/`.
//!
//! Ignored by default — it writes into the source tree. Run it deliberately:
//!
//! ```sh
//! cargo test --manifest-path fuzz/Cargo.toml --no-default-features \
//!     --test generate_seeds -- --ignored --nocapture
//! ```
//!
//! Why a test and not a script: seeds for the `Arbitrary`-driven targets are
//! *byte strings interpreted by a decoder*, so a useful one cannot be written
//! by hand — it has to be produced by running the decoder and keeping the
//! bytes that decoded into something interesting. Doing that here keeps the
//! generator next to the decoder it depends on, deterministic (a fixed LCG, no
//! system randomness), and re-runnable when the input types change.
//!
//! The output is committed. Regenerating it is a deliberate act, and the diff
//! should be reviewed like any other.

use std::path::PathBuf;

/// Deterministic byte source. A fixed seed means regenerating produces byte
/// identical output, so an unrelated change never shows up as corpus churn.
struct Lcg(u64);

impl Lcg {
    fn next_u8(&mut self) -> u8 {
        self.0 = self
            .0
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        (self.0 >> 33) as u8
    }

    fn bytes(&mut self, len: usize) -> Vec<u8> {
        (0..len).map(|_| self.next_u8()).collect()
    }
}

fn seeds_dir(target: &str) -> PathBuf {
    let dir = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("seeds")
        .join(target);
    std::fs::create_dir_all(&dir).expect("create seeds dir");
    dir
}

/// Emit `count` byte strings per length, keeping only those the target's
/// decoder actually consumes into a program (the rest would be dead weight in
/// the corpus).
fn emit_random_seeds(target: &str, lengths: &[usize], per_length: usize, seed: u64) {
    let dir = seeds_dir(target);
    let run = opensolid_fuzz::target(target).expect("known target");
    let mut lcg = Lcg(seed);
    let mut written = 0usize;

    for &len in lengths {
        for _ in 0..per_length {
            let bytes = lcg.bytes(len);
            // Replaying here does double duty: it proves the seed is safe to
            // commit, and it is the only way to know the decoder accepted it.
            run(&bytes);
            let path = dir.join(format!("{written:03}-len{len}.bin"));
            std::fs::write(&path, &bytes).expect("write seed");
            written += 1;
        }
    }
    println!("wrote {written} seeds to {}", dir.display());
}

#[test]
#[ignore = "writes into the source tree; run deliberately"]
fn generate_topology_check_seeds() {
    // Short inputs build small graphs; long ones build graphs big enough for
    // the shell/face/loop cross-checks to have something to disagree about.
    emit_random_seeds(
        "topology_check",
        &[8, 24, 64, 160, 512],
        4,
        0x2545_f491_4f6c_dd1d,
    );
}

#[test]
#[ignore = "writes into the source tree; run deliberately"]
fn generate_nurbs_eval_seeds() {
    emit_random_seeds(
        "nurbs_eval",
        &[8, 24, 64, 160, 512],
        4,
        0x9e37_79b9_7f4a_7c15,
    );
}

/// The one seed that cannot be random: a complete, importable AP203 solid.
///
/// It comes out of the kernel's own writer rather than being pasted in, so it
/// stays valid as the writer evolves, and it gives the fuzzer a starting point
/// deep inside the mapper — the region a random byte string reaches roughly
/// never.
#[test]
#[ignore = "writes into the source tree; run deliberately"]
fn generate_step_solid_seeds() {
    use opensolid_brep::{GeometryStore, TopologyStore, primitives};
    use opensolid_kernel::io::step::{self, StepWriteOptions};

    let dir = seeds_dir("step_parse");
    let run = opensolid_fuzz::target("step_parse").expect("known target");

    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let block = primitives::block(&mut store, &mut geo, 10.0, 6.0, 4.0).expect("block");
    let cylinder = primitives::cylinder(&mut store, &mut geo, 2.0, 8.0).expect("cylinder");
    let sphere = primitives::sphere(&mut store, &mut geo, 3.0).expect("sphere");

    for (name, bodies) in [
        ("11-block-solid.stp", vec![block]),
        ("12-cylinder-solid.stp", vec![cylinder]),
        ("13-sphere-solid.stp", vec![sphere]),
        ("14-multi-solid.stp", vec![block, cylinder, sphere]),
    ] {
        let text = step::write_step(&store, &geo, &bodies, &StepWriteOptions::default())
            .expect("write_step");
        run(text.as_bytes());
        std::fs::write(dir.join(name), text.as_bytes()).expect("write seed");
        println!("wrote {name}");
    }
}
