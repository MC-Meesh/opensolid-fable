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
