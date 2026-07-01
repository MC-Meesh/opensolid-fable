# OpenSolid Specification

AI-native B-rep geometry kernel in Rust, modeled after Parasolid's architecture.

## Document Structure

| Document | Scope |
|----------|-------|
| [00-overview.md](00-overview.md) | System architecture, design principles, module map |
| [01-data-model.md](01-data-model.md) | Topology + geometry type hierarchy, entity relationships |
| [02-geometry.md](02-geometry.md) | Curves, surfaces, NURBS evaluation, intersection algorithms |
| [03-topology.md](03-topology.md) | Body/shell/face/loop/edge/vertex, half-edge structure, validation |
| [04-booleans.md](04-booleans.md) | Union/subtract/intersect, face classification, topology reconstruction |
| [05-operations.md](05-operations.md) | Blending, offsetting, sweeping, lofting, direct modeling |
| [06-step-io.md](06-step-io.md) | STEP AP203/AP214 import/export, Part 21 parser |
| [07-tessellation.md](07-tessellation.md) | Faceting, mesh generation, adaptive refinement |
| [08-tolerances.md](08-tolerances.md) | Precision model, tolerant modeling, healing |
| [09-session.md](09-session.md) | Memory management, undo/redo, journaling, partitions |
| [10-api.md](10-api.md) | Public Rust API surface, FFI/C API, Python bindings |
| [11-testing.md](11-testing.md) | Test strategy, golden files, STEP conformance, property tests |
| [12-implementation-phases.md](12-implementation-phases.md) | Build order, milestones, what ships when |

## Design Principles

1. **Parasolid-modeled**: Follow Parasolid's proven architecture (B-rep topology + NURBS geometry + tolerant modeling). Don't reinvent what 35 years of production use has validated.
2. **Rust-native**: Ownership semantics for topology graphs. No GC. Thread-safe by construction.
3. **AI-first API**: Rich error messages, introspection, deterministic operations, branching/undo for agent exploration.
4. **STEP as the lingua franca**: Correct AP203/AP214 import/export is a hard requirement, not a nice-to-have.
5. **Test-driven from day one**: Every operation has property-based tests + golden file validation against reference geometry.

## Scope

This kernel aims to implement the full Parasolid functional equivalent:
- All analytic + NURBS geometry types
- Complete boolean operations (unite, subtract, intersect) with tolerant modeling
- Blending (constant/variable radius fillets, chamfers)
- Offsetting and shelling
- Sweeping (extrude, revolve, path sweep, loft)
- Direct modeling (move/offset/replace face)
- STEP AP203/AP214 import and export
- Tessellation for visualization
- Mass properties (volume, area, center of mass, moments of inertia)
- Body validation and healing

## Non-Goals (Initially)

- GUI / visualization application
- CAM / toolpath generation
- FEA meshing (beyond basic tessellation)
- Sheet metal specific operations
- Proprietary format support (IGES, SAT, X_T — except for testing)
