# 12 — Build Order and Dependencies

How to build the kernel correctly from the start — no phases, no sprints, no tech debt.

## Philosophy

This is not a phased roadmap. Phases create tech debt by design — you build something
"good enough for phase 1" and then rewrite it later. With token-based agent execution,
we build everything correctly from day one. The only constraint is **dependency order**:
you can't implement booleans without SSI, and you can't implement SSI without NURBS
evaluation.

The dependency graph determines what can be built in parallel and what blocks on what.
Everything else is built to production quality immediately.

## Dependency Graph

```
                    ┌─────────────────────────────────────────────────────────┐
                    │                                                         │
                    ▼                                                         │
┌──────────┐   ┌──────────┐   ┌──────────────┐   ┌──────────┐   ┌─────────┐│
│   Math   │──▶│ Geometry │──▶│     SSI      │──▶│ Booleans │──▶│  Blend  ││
│          │   │ (curves, │   │ (surface-    │   │          │   │ (fillet,││
│ Point3   │   │ surfaces,│   │  surface     │   │ Unite    │   │ chamfer)││
│ Vector3  │   │ NURBS    │   │  intersect)  │   │ Subtract │   │         ││
│ Transform│   │ eval)    │   │              │   │ Intersect│   │         ││
│ Interval │   │          │   │              │   │          │   │         ││
└──────────┘   └──────────┘   └──────────────┘   └──────────┘   └─────────┘│
     │              │                                   │              │     │
     │              │                                   │              │     │
     ▼              ▼                                   ▼              ▼     │
┌──────────┐   ┌──────────┐                      ┌──────────┐   ┌─────────┐│
│  Core    │   │ Topology │                      │  Offset  │   │  Sweep  ││
│          │   │          │                      │  Shell   │   │ Extrude ││
│ Arena    │   │ Body     │──────────────────────│          │   │ Revolve ││
│ EntityId │   │ Shell    │                      │          │   │ Loft    ││
│ Attribs  │   │ Face     │                      └──────────┘   └─────────┘│
│          │   │ Loop     │                                                 │
└──────────┘   │ Fin/Edge │   ┌──────────────┐   ┌──────────┐              │
               │ Vertex   │──▶│   STEP I/O   │   │  Tessell │              │
               │ Prims    │   │              │   │          │              │
               └──────────┘   │ Part 21      │   │ CDT      │              │
                              │ AP203/AP214  │   │ Adaptive │              │
                              │ Import       │   │ STL/OBJ  │              │
                              │ Export       │   │          │              │
                              │ Healing      │   │          │              │
                              └──────────────┘   └──────────┘              │
                                                                           │
               ┌──────────────────────────────────────────────────────────┘
               │
               ▼
          ┌──────────┐   ┌──────────┐   ┌──────────┐
          │  Direct  │   │  Session │   │  Mass    │
          │ Modeling │   │          │   │  Props   │
          │          │   │ Undo     │   │          │
          │ Move     │   │ Branch   │   │ Volume   │
          │ Replace  │   │ Journal  │   │ Area     │
          │ Delete   │   │ Snapshot │   │ Inertia  │
          └──────────┘   └──────────┘   └──────────┘
```

## Hard Dependencies (Must Be Built Before)

| Component | Requires |
|-----------|----------|
| Geometry (curves/surfaces) | Math, Core |
| Topology | Math, Core |
| Spatial (BVH) | Math, Topology |
| STEP Import | Geometry, Topology |
| STEP Export | Geometry, Topology |
| Tessellation | Geometry, Topology |
| SSI | Geometry |
| Booleans | SSI, Topology |
| Blend (fillet/chamfer) | SSI, Booleans, Topology |
| Offset/Shell | SSI, Topology |
| Sweep (extrude/revolve/loft) | Geometry, Topology |
| Direct Modeling | Booleans, Topology |
| Mass Properties | Geometry, Topology |
| Session (undo/branch) | Core, Topology |
| Healing | Geometry, Topology, SSI |
| Python Bindings | API (each public function gets a binding) |
| Persistent Naming | Core, Topology |

## Parallelizable Work

These groups have no dependencies on each other and can be built simultaneously:

**Group A** (foundation — must come first):
- Math, Core

**Group B** (after A, all independent of each other):
- Geometry (NURBS evaluation, all curve/surface types)
- Topology (data structures, Euler ops, validation, primitives)
- Persistent naming system (part of Core/Topology, built from the start)

**Group C** (after B, independent of each other):
- STEP Import/Export (needs geometry + topology)
- Tessellation + mesh export (needs geometry + topology)
- Sweep operations (needs geometry + topology)
- Mass properties (needs geometry + topology)
- SSI algorithms (needs geometry)
- Spatial indexing / BVH (needs topology)
- Python bindings scaffolding (PyO3 project setup, ships with Group C APIs)

**Group D** (after SSI):
- Booleans (needs SSI + topology)
- Offset/Shell (needs SSI + topology)

**Group E** (after booleans):
- Blending (needs booleans)
- Direct modeling (needs booleans)

**Group F** (after any mutating operations exist):
- Session/undo/branch (needs operations to undo)
- Builder API (high-level fluent interface, wraps all underlying operations)

**Continuous** (runs alongside every group):
- Benchmark suite: every performance-critical path gets a criterion bench in its PR
- OCC comparison: every new algorithm gets validated against OCC output
- Corpus pass-rate tracking: CI monitors import/boolean/tessellation pass rates

**Python bindings ship at component STABILIZATION, not creation.**
A component is "stable" when its public Rust API has had no signature changes for 4 weeks.
This prevents the 30% engineering tax of maintaining dual-language bindings during
exploration-phase work where APIs change daily. The bindings-crew focuses on stable
components and works backward from the oldest-stable to newest-stable.

## What "Build Correctly From the Start" Means

### Every component ships with:
- Full test suite (property-based + golden files)
- STEP round-trip validation (where applicable)
- Performance meeting targets from spec/00-overview.md
- Production error handling (rich error enums, no panics)
- Documentation (rustdoc on all public items)
- Python binding

### Architecture is correct from day one:
- Tolerant modeling data structures from day one (pcurve on Fin, tolerance on Edge)
- All geometry TYPES from day one (not "planes only, add NURBS later")
- SP-curve fields in topology from the start
- Thread safety baked in (not retrofitted)
- CoW snapshot support in arenas from the start

### Iterative hardening is expected for ALGORITHMS:

The architecture and data model must be right from day one. But complex algorithms
(SSI, boolean face classification, tangent intersection handling) are hardened
iteratively. This is NOT "phases" or "tech debt" — it's the mathematical reality
that you cannot handle tangent intersections correctly without first having
transversal intersection working to learn from.

Each component has a "minimum viable" and "hardened" level:

| Component | Minimum Viable | Hardened |
|-----------|---------------|----------|
| SSI | Plane-plane, plane-analytic (exact). NURBS transversal marching. | Tangent detection, branch points, coincident regions, self-intersection. |
| Booleans | Transversal face classification. No coincident faces. | Tangent/coplanar faces, touching edges, identity operations. |
| Fillet | Constant-radius on analytic faces. | Variable-radius, NURBS faces, overflow, partial failure recovery. |
| STEP Import | AP203 basic topology (points, curves, faces, shells). | Full AP214, colors, assemblies, all entity types, aggressive healing. |
| Tessellation | Basic adaptive CDT. | Anisotropic meshing, gap stitching, periodic surface handling. |

"Minimum viable" unblocks downstream components. "Hardened" is required for
production use. Both are tracked separately in PROGRESS.md.

A hardening pass is NOT tech debt. It's the planned second iteration with knowledge
gained from integration testing.

### The dependency graph is not a timeline:
- Agents work on everything in a dependency group simultaneously
- When SSI is ready, booleans start immediately — no "phase gate" review
- If an upstream component needs revision based on downstream learnings, revise immediately

### Before crews start: compile the skeleton

One person writes the actual `Cargo.toml` files for all crates, defining:
- Workspace structure and inter-crate dependencies
- Shared type interfaces (trait definitions, EntityId, key enums)
- `unimplemented!()` bodies for all trait methods

This skeleton MUST compile. It proves the dependency graph has no cycles, validates
crate boundaries, and gives every crew a concrete interface to code against. Without
this, crews will immediately discover circular type dependencies that block work.

```
# Must exist and compile before multi-crew work begins:
crates/opensolid-core/src/lib.rs       # EntityId, Arena, ArenaSnapshot
crates/opensolid-math/src/lib.rs       # Point3, Vector3, Transform3, Interval
crates/opensolid-curves/src/lib.rs     # Curve enum, CurveEval trait
crates/opensolid-surfaces/src/lib.rs   # Surface enum, SurfaceEval trait
crates/opensolid-topology/src/lib.rs   # TopologyStore, Body, Face, Edge structs
# etc. — all crates with type skeletons, no implementations
```

## Crew Structure

| Crew | Scope | Dependencies |
|------|-------|-------------|
| **math-crew** | Math + Core foundations, persistent naming | None |
| **geometry-crew** | All curves, surfaces, NURBS eval, intersection algorithms, SSI | Math |
| **topology-crew** | Topology graph, Euler ops, primitives, validation, BVH/spatial | Core |
| **step-crew** | Part 21 parser, AP203/AP214 import (heal-first), export, healing | Geometry + Topology |
| **boolean-crew** | Full boolean pipeline (clash → SSI → classify → reconstruct) | SSI + Topology |
| **operations-crew** | Blend, offset, sweep, direct modeling, patterns, builder API | Booleans + Topology |
| **infra-crew** | Tessellation, mesh export, mass properties, session/undo | Topology + Geometry |
| **bindings-crew** | Python (PyO3), C FFI (cbindgen) — ships at component stabilization (4 weeks API-stable) | After stabilization |
| **test-crew** | Golden files, property tests, OCC comparison, CI, benchmarks | Continuous |

## Validation Gates

A component is NOT done until:

1. **All tests pass** — unit, property-based, and integration
2. **STEP round-trip** — geometry survives export → reimport (for relevant components)
3. **Body check passes** — any body produced by the component passes full validation
4. **Performance targets met** — benchmarks from spec/00-overview.md (regressions > 10% block)
5. **Error messages are actionable** — every failure path produces a structured error with suggestions
6. **Review checklist complete** — see REVIEW.md
7. **OCC comparison passes** — for SSI/boolean/import work, results match OCC within tolerance
8. **Corpus pass rate stable** — no regressions in adversarial test corpus pass rate
9. **Spec updated** — relevant spec files reflect the implementation
10. **PROGRESS.md updated** — milestone/status reflects completed work

## What Gets Built First (Practical Order)

The dependency graph means some things genuinely cannot start until others finish.
Practically, the build order is:

1. **Math + Core** (must exist for anything else to compile)
2. **Geometry + Topology** (in parallel — these are the two pillars)
3. **STEP Import** (first external validation point — proves geometry + topology work against real data)
4. **SSI + Tessellation + Sweep + Mass Props** (in parallel — all depend only on geometry + topology)
5. **STEP Export** (round-trip validation becomes possible)
6. **Booleans** (the hardest single component — benefits from all prior work)
7. **Blend + Offset + Direct Modeling** (extend booleans)
8. **Session/Undo + Bindings** (polish layer)

This is not "phases" — it's the mathematical reality that you can't intersect surfaces
before you can evaluate them. Each numbered item starts the moment its dependencies are done.
