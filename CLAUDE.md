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

If a freshly compiled binary hangs at `_dyld_start` using 0% CPU — while
already-built binaries like `git`/`cargo` still run fine — the host's `amfid`
daemon has been idle-reaped by jetsam. `amfid` validates code signatures on a
binary's *first* exec, so while it is dead every fresh `cargo test` binary blocks
in dyld. Previously-run binaries are already validated and are unaffected, which
makes this look like a Rust or toolchain problem. It is not.

Check it first:

```bash
pgrep -q amfid || echo "amfid is dead — this is the hang, not your code"
```

It resolves on its own: `launchd` respawns `amfid` on demand, typically within
~20-30 minutes, and the hang clears with no reboot. To recover immediately, an
operator (not an agent — SIP blocks unprivileged attempts) can run:

```bash
sudo launchctl kickstart -k system/com.apple.MobileFileIntegrity
```

Do **not** escalate this as a machine fault, and do not ask for a reboot. This
signature has been misdiagnosed three times (as a wedged `syspolicyd`, and as the
Claude Code Bash sandbox blocking exec) and cost hours of blocked gates. The Bash
sandbox is *off* unless explicitly enabled and is not the cause; adding a sandbox
allowlist is a no-op here. Because the wedge self-heals, whatever you changed last
will look like the fix — verify `amfid` before concluding anything. See `of-zis`
for the kernel-log evidence.

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

1. Verify your MR bead exists: `bd list --type=merge-request --status=open` must show your branch.
2. If it doesn't, push your branch to origin explicitly and re-run `gt done` (or `gt mq submit`).
3. Trust the GitHub-side ref over your local branch ref — if your local branch suddenly
   points at someone else's commit, your content is still safe on `origin/<your-branch>`.
