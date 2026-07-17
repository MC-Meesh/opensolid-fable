# OpenSolid

Rust-based open CAD kernel using a hybrid F-Rep (implicit/SDF) + B-Rep (NURBS/spline) architecture.

## Vision

Open alternative to Parasolid. Better than CadQuery/OCC (which are bloated and limited).
The key insight: F-Rep gives trivially robust booleans and organic blending; B-Rep gives
precision for engineering surfaces. Combine both in one kernel.

## Architecture

```
opensolid/
├── crates/
│   ├── opensolid-core/    # Points, vectors, transforms, arena allocator
│   ├── opensolid-frep/    # F-Rep: SDF primitives, CSG (min/max), smooth blending
│   ├── opensolid-brep/    # B-Rep: NURBS, topology graph, tolerant modeling
│   └── opensolid-kernel/  # Unified: meshing, implicit↔boundary conversion, session
├── research/              # Prior research (read-only reference)
└── spec/                  # Spec docs from v1 attempt (reference, not gospel)
```

## Build & Test

```bash
cargo build
cargo test
cargo clippy -- -D warnings
```

## Troubleshooting: `cargo test` hangs before any test runs

The signature is a *first exec* wedge: freshly compiled binaries hang at
`_dyld_start` using 0% CPU, while already-validated binaries like `git`/`cargo`
run fine. Note the asymmetry — `cargo check` and `cargo build` keep working
throughout, because `rustc` and `cargo` have already been exec'd before. Only
the brand-new test binary blocks. That asymmetry is most of why this reads as a
Rust or toolchain problem. It is not.

Diagnose it with these two checks, in order. Both are cheap, neither needs root,
and both are positive evidence rather than inference:

```bash
# 1. Sample the hung process. Wedged output is unambiguous: every sample is in
#    dyld and never reaches main. Physical footprint stays ~96K, State S, 0% CPU.
sample <pid> 2 -mayDie
#      1626 Thread_xxx: Main Thread
#        1626 _dyld_start  (in dyld) + 0

# 2. Trivial-binary control — proves it is machine-wide, not your code.
printf 'fn main(){println!("hi");}' > /tmp/w.rs && rustc -o /tmp/w /tmp/w.rs && /tmp/w
```

If a one-line hello-world hangs too, the problem is not the project, and there is
nothing to fix in your change.

One known cause is the host's `amfid` daemon being idle-reaped by jetsam. `amfid`
validates code signatures on a binary's *first* exec, so while it is dead every
fresh binary blocks in dyld — see `of-zis` for the kernel-log evidence. But
`pgrep -q amfid` is **not** a sufficient check and must not be your first move:
an instance on 2026-07-16 (`of-kra`) ran 50+ minutes with `amfid` alive the whole
time and every fresh binary still hanging. A live `amfid` does not rule this out.

It has always resolved on its own without a reboot, but do not count on a
particular window — `launchd` respawns `amfid` on demand, sometimes in ~20-30
minutes, sometimes considerably longer (the `of-kra` instance never cleared
within the session). If `amfid` is in fact dead, an operator (not an agent — SIP
blocks unprivileged attempts) can force it back immediately:

```bash
sudo launchctl kickstart -k system/com.apple.MobileFileIntegrity
```

Do **not** escalate this as a machine fault, and do not ask for a reboot. This
signature has been misdiagnosed three times (as a wedged `syspolicyd`, and as the
Claude Code Bash sandbox blocking exec) and cost hours of blocked gates. The Bash
sandbox is *off* unless explicitly enabled and is not the cause; adding a sandbox
allowlist is a no-op here. Because the wedge self-heals, whatever you changed last
will look like the fix — get the sample and the hello-world control before
concluding anything.

## Research

See `research/` for landscape analysis and `spec/` for the v1 spec. The v1 spec assumed
pure B-Rep — the hybrid F-Rep+B-Rep approach is a departure. Use the spec for Parasolid
functional mapping, tolerance philosophy, and performance targets. Ignore the crate
structure (it's different now).

## Rules

- Every function must have tests. No untested code merges.
- `cargo clippy -- -D warnings` must pass.
- Keep dependencies minimal (nalgebra, thiserror, rayon — that's it).
- F-Rep booleans are the fast path. B-Rep is for precision when needed.

## Merge-completion protocol (all polecats — added after 5 dropped-merge incidents)

`gt done` can silently fail to create an MR (known gt bug, tracked in HQ as hq-iheg;
a post-merge sync can also repoint your local branch ref to a foreign tip).
Before going idle after `gt done`:

1. Verify your work is TRACKED (an MR bead that already CLOSED means it merged — success,
   not a drop; `--status=open` alone false-alarms whenever the refinery is fast):
   `bd list --all | grep -i 'merge: <your-issue-id>'` — an MR bead must exist, open OR
   closed (NOTE: MR beads are type=task labeled gt:merge-request, titled 'Merge: <issue>';
   `--type=merge-request` matches nothing). If closed, optionally confirm reachability once the train lands:
   `git fetch && git log origin/main --grep=<issue-id> --oneline` (allow queue latency).
2. Only if NO MR bead was ever created: push your branch to origin explicitly and re-run
   `gt done` (or `gt mq submit`), then re-check step 1.
3. Trust the GitHub-side ref over your local branch ref — if your local branch suddenly
   points at someone else's commit, your content is still safe on `origin/<your-branch>`.
