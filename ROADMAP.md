# OpenSolid Roadmap

Living document. The source of truth for task state is the **beads tracker**
(`bd ready`, `bd show <id>`); this file maps the plan so a human can see the
shape of the work at a glance. Adapted from `spec/12-implementation-phases.md`
to the hybrid F-Rep + B-Rep architecture and the actual 4-crate layout
(the spec's crate structure predates the hybrid pivot — see `CLAUDE.md`).

## Where we are

- ✅ Scaffold: 4-crate workspace builds, clippy-clean, all tests green
- ✅ `opensolid-core`: Point3/Vector3/Transform3, BoundingBox3, arena allocator
- ✅ `opensolid-frep`: sphere/box/cylinder SDFs, CSG (min/max), smooth blending,
  finite-difference gradients, **uniform-grid dual contouring mesher**
  (`mesh_sdf` — watertight, manifold-tested)
- ⬜ `opensolid-brep`: stub — geometry and topology are the current frontier
- ⬜ `opensolid-kernel`: re-exports only — conversion layer not started

## Epic index (bead IDs)

| Bead | Epic | Depends on | Why it matters |
|------|------|-----------|----------------|
| `of-mw3` | Core foundations: interval math, tolerance, errors, arena CoW | — | Everything builds on this |
| `of-5fl` | F-Rep engine: primitives, operators, adaptive octree meshing | core interval | The robust-boolean fast path |
| `of-uui` | B-Rep geometry: curves, surfaces, NURBS | — | Precision engineering surfaces |
| `of-0uu` | B-Rep topology: store, Euler ops, validation, primitive solids | — | The structural pillar |
| `of-2cz` | Mesh & interchange: TriangleMesh, STL/OBJ, mass props, tessellation | topology, geometry | First demoable outputs |
| `of-pb7` | SSI & B-Rep booleans | geometry, topology | The hardest classical component |
| `of-0oz` | **Hybrid conversion: F-Rep ↔ B-Rep bridge** | meshing, tessellation, BVH | **The differentiator** — booleans that never fail |
| `of-bcz` | Kernel API & session: builder, undo, sweep | arena CoW, scene graph | What users actually touch |
| `of-69a` | Quality infra: CI, criterion benches, property tests | — | Continuous, runs alongside everything |

Each epic's bead carries the full task list (`bd show <epic-id>` → children).
Dependencies are wired in beads, so `bd ready` always shows exactly what is
unblocked — no coordination needed to pick correct work.

## Build order (dependency reality, not phases)

```
core foundations ──┬─▶ F-Rep engine ───────────┬─▶ hybrid conversion ─▶ hybrid booleans
                   ├─▶ B-Rep geometry ─▶ SSI ──┼─▶ B-Rep booleans ─▶ blend/offset (later)
                   ├─▶ B-Rep topology ─────────┤
                   └─▶ mesh & interchange ─────┘
quality infra (CI/bench/proptest) ── continuous
kernel API & session ── as soon as its inputs exist
```

Two ideas from the spec we keep:

1. **Architecture correct from day one, algorithms hardened iteratively.**
   Tolerant-modeling fields (edge tolerance, pcurves) exist in the topology
   structs from the first commit. SSI/booleans ship transversal-only MVPs
   first and get planned hardening passes — that is not tech debt, it is how
   the math has to be learned.
2. **F-Rep is the fast path.** B-Rep booleans are the hardest thing in any
   kernel; our hybrid escape hatch (`of-0oz`) converts to SDF, does the
   trivially-robust CSG, and meshes back. Booleans in OpenSolid should never
   hard-fail.

## Validation gates (every task)

- Every function tested; `cargo test` green
- `cargo clippy -- -D warnings` clean
- Meshes: watertight/manifold checks where applicable
- Runtime deps stay minimal: `nalgebra`, `thiserror`, `rayon` only
  (`criterion` + `proptest` approved as **dev**-dependencies)
- No panics in public APIs — structured errors (`thiserror`)

## Long poles to watch

- `of-pb7.4` B-Rep boolean pipeline (clash → SSI → classify → reconstruct)
- `of-0oz.3` hybrid boolean fast path (needs both reps + conversion)
- `of-5fl.6` adaptive octree dual contouring with QEF sharp features
