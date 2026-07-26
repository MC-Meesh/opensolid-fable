# Fuzzing OpenSolid

Coverage-guided fuzzing for the three places where untrusted input meets the
kernel: the STEP Part 21 front end, topology validation, and NURBS evaluation.

The motivating bug is `of-1dd` — a stack overflow in the STEP parser at 500-deep
aggregate nesting, found by a hand-written adversarial file. Hand-written
adversarial files find one bug each. A fuzzer finds the class.

## Quick start

```sh
rustup toolchain install nightly
cargo +nightly install cargo-fuzz --locked

tools/fuzz/run.sh step_parse 300      # fuzz for five minutes
tools/fuzz/run.sh topology_check 300
tools/fuzz/run.sh nurbs_eval 300
```

`tools/fuzz/run.sh` is the single place that knows the corpus layout and the
resource limits, so a local run and a CI run are the same command.

Without a nightly toolchain you can still run everything the fuzzer has already
found, on stable:

```sh
cargo test --manifest-path fuzz/Cargo.toml --no-default-features
```

## The targets

| Target | Entry point | Contract |
| --- | --- | --- |
| `step_parse` | `step::parse_bytes`, `step::read_step_bytes`, `step::write_step` | no panic/hang on any bytes; `parse` ≡ `parse_bytes`; an exact import passes `check`; what imports exactly, exports and re-parses |
| `topology_check` | `TopologyStore::check` | never panics on a corrupted graph; deterministic; a removed body reports `StaleBody`; corruption is never silent |
| `nurbs_eval` | `KnotVector::new`, `NurbsCurve`, `NurbsSurface` | accepted knot vectors really are finite, ordered and non-empty; evaluation stays in the control hull, interpolates clamped endpoints, and survives knot insertion and reversal unchanged |

Each target's module doc (`src/step.rs`, `src/topology.rs`, `src/nurbs.rs`)
states the full post-condition list and why each one is a defensible oracle.

Most of these go well past "does not crash". A crash-only fuzzer would never
have flagged a knot vector that evaluates to `NaN` — it does not crash, it
quietly returns garbage. The convex-hull and locus-preservation oracles are
what turn this into a correctness suite.

## How it is laid out, and why

```
fuzz/
├── src/            harness bodies — plain stable Rust
├── fuzz_targets/   five-line libFuzzer wrappers (nightly only)
├── seeds/          committed starting corpus
├── corpus/         libFuzzer's working corpus (gitignored)
├── artifacts/      crash reproducers (COMMITTED — see below)
└── tests/          corpus replay + seed generation
```

**The harness logic is in `src/`, not in `fuzz_targets/`.** `fuzz_targets/*.rs`
can only be built by nightly with sanitizer flags; `src/` builds on stable. That
split is what lets `tests/corpus_replay.rs` re-run every seed and every crash
artifact as an ordinary `cargo test`, in the project's normal stable CI. The
fuzzer finds a bug once; the replay test keeps it fixed forever.

**`artifacts/` is deliberately not gitignored.** When libFuzzer drops a
reproducer there, commit it. From that moment the stable CI job re-checks the
fix on every push, in milliseconds. This is the whole payoff loop:

```
nightly fuzz job finds a crasher
  → commit fuzz/artifacts/<target>/crash-<hash> alongside the fix
    → stable `cargo test` replays it on every push, forever
```

**The fuzz package is its own workspace.** `libfuzzer-sys` and `arbitrary` are
fuzzing-only dependencies and must not enter the kernel workspace's lockfile
(CLAUDE.md: "keep dependencies minimal"). The root manifest lists `fuzz` under
`workspace.exclude`; `fuzz/Cargo.toml` opens an empty `[workspace]` of its own.

**libFuzzer is an optional feature.** The `[[bin]]` targets declare
`required-features = ["libfuzzer"]`, so `--no-default-features` skips them
entirely and the harness compiles on stable like any other crate. It is on by
default, so plain `cargo fuzz run` needs no extra flags.

## Seeds

`seeds/step_parse/` holds hand-written adversarial STEP files — every Part 21
value kind, comments and multi-line tokens, aggregate nesting at the parser's
limit, multiple `DATA` sections, dangling and forward references, escape
directives, a file truncated mid-record, NUL bytes — plus complete importable
solids emitted by the kernel's own writer.

The real AP203 corpus under `crates/opensolid-kernel/tests/data/step` is by far
the best seed material available (it exercises entity types no synthetic seed
would think of), but at up to 1 MiB per file it is too large to duplicate here.
`tools/fuzz/run.sh` points libFuzzer at it in place, as a read-only corpus.
libFuzzer does not recurse, so the script enumerates the corpus subdirectories
with `find` rather than listing them — a newly added edge-case file is seeded
automatically, and `tests/corpus_replay.rs` walks the same tree recursively for
the same reason.

`seeds/topology_check/` and `seeds/nurbs_eval/` are byte strings interpreted by
an `Arbitrary` decoder, so they cannot sensibly be written by hand. They are
generated deterministically:

```sh
cargo test --manifest-path fuzz/Cargo.toml --no-default-features \
    --test generate_seeds -- --ignored --nocapture
```

That regenerates `seeds/` byte for byte from a fixed LCG. Re-run it after
changing an input type; review the diff like any other.

## CI

* **`ci.yml`** (stable, every push/PR) — formats and clippies the harness, then
  runs the corpus replay. Cheap, and it is where past findings stay fixed.
* **`fuzz.yml` smoke** (nightly toolchain, every push/PR) — 60 seconds per
  target. Catches "the target stopped building" and shallow regressions.
* **`fuzz.yml` long** (nightly cron 04:23 UTC, or manual dispatch) — 20 minutes
  per target, resuming from the cached corpus so each night starts where the
  last one stopped. Crashes upload as workflow artifacts.

## Findings so far

| Found | Bug | Fix |
| --- | --- | --- |
| `nurbs_eval`, first run | `KnotVector::new` accepted `NaN` and infinite knots. Every comparison against `NaN` is false, so a `NaN` satisfied both the monotonicity scan and the empty-domain test, then poisoned `domain()`, `find_span` and evaluation to `NaN` with nothing reporting why. The STEP reader feeds it knot lists straight from file. | `NurbsError::NonFiniteKnot`, checked first |
| `nurbs_eval`, first run | `NurbsCurve::new` / `NurbsSurface::new` accepted `NaN` and `+inf` weights: `w <= 0.0` is false for both. They divide the homogeneous coordinates into `NaN` at evaluation. | weights must be finite *and* positive |

## Adding a target

1. Write the harness body in `src/<name>.rs`, with a module doc that states the
   post-conditions and why each is defensible.
2. Add it to `TARGETS` in `src/lib.rs` — that alone brings it under the
   corpus-replay gate.
3. Add the five-line `fuzz_targets/<name>.rs` wrapper and its `[[bin]]` entry.
4. Add it to the `matrix.target` lists in `.github/workflows/fuzz.yml` and to
   the validation list in `tools/fuzz/run.sh`.
5. Commit seeds under `seeds/<name>/`.
