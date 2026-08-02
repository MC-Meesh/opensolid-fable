# Randomized correctness campaign

The July program's bead supply (epic `of-ipt`) came from a loop that probed
every landed fix with randomized stress runs and auto-filed what broke. The
runner lived outside the repo and did not survive the machine hop; this
directory is its minimal, in-repo reconstruction (`of-5rim`).

## What a cycle does

1. Draws a fresh 64-bit seed and exports it as `OPENSOLID_CAMPAIGN_SEED`.
2. Runs the randomized suites against it:
   `boolean_stress`, `sweep_random` (opensolid-brep), `ops_randomized`,
   `property_invariants` (opensolid-frep), `convert_roundtrip_random`,
   `step_heal_random` (opensolid-kernel). Each suite's `Rng::new` XORs the
   variable into its hardcoded seeds, so the same analytic properties walk
   fresh configurations. Unset, every suite is byte-for-byte deterministic —
   CI and plain `cargo test` are unaffected. (`property_invariants` is
   proptest-based and draws its own fresh entropy instead.)
3. Classifies failures. A case the suite itself rejects ("choose a different
   seed" — a generator miss, not a kernel bug) is reported but never filed.
4. Reconciles against the open board: a failure whose `<suite>::<test>` key
   or bare test name appears in an open bead's description is counted as
   known, not re-filed.
5. Files each genuinely new failure as a `bd` bug bead (P2) carrying the
   seed, the exact repro command, and the tail of the panic output.

## Running it

```sh
tools/campaign/run.sh              # full cycle, DRY-RUN filing (default)
tools/campaign/run.sh --file       # full cycle, really file beads
tools/campaign/run.sh --seed 0xDECAF --suites sweep_random   # targeted
```

A JSON report of every run (per-suite pass/fail/seconds, filed and known
beads, total test-seconds — the epic's speed metric) lands in
`~/.local/state/opensolid-campaign/`.

Reproducing a filed failure is one command, quoted in the bead:

```sh
OPENSOLID_CAMPAIGN_SEED=0x<seed> cargo test -p <crate> --test <suite> <test>
```

## Wiring (pending Mayor approval)

Proposed mechanism — cron on this box, every 6 hours, from a clone that
tracks `main`:

```cron
0 */6 * * * cd $HOME/gt/opensolid_fable && git pull --ff-only && tools/campaign/run.sh --file >> $HOME/.local/state/opensolid-campaign/cron.log 2>&1
```

`run.sh` holds a `flock`, so an overlapping firing exits immediately; the
reconciliation step makes repeated runs idempotent (no duplicate beads).
Alternative, if cron is unwanted: a witness patrol step invoking the same
command. Do not install either without approval on `of-5rim`.
