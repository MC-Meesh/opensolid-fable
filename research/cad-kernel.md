---
title: AI-Native CAD Kernel
created: 2026-05-07
updated: 2026-05-07
status: research
started: 2026-05-07
tags: [project, cad, gpu, ai, geometry-kernel, rust, yc]
---

# AI-Native CAD Kernel

Build a modern, AI-friendly B-rep geometry kernel in Rust. The only production-grade open-source kernel (OpenCASCADE) is 30 years of legacy C++ with an atrocious API. Only 3-4 B-rep kernels exist worldwide. The thesis: LLM-driven design and manufacturing automation need a kernel built for programmatic, agent-driven workflows, not GUIs.

**Origin**: Discussed with James on 2026-05-07. Sequence: Mortise v1 first, then this. Both agree this is a massive, high-upside project. James's take: mesh-to-CAD as the initial product (easier to sell, more unsolved, more fun than replacing OCC wholesale). GPU-first kernel would be very cool but may not be the right core bet (see GPU section). YC Fall batch is the target if something credible exists by application time.

---

# 1. Why Are There So Few Kernels?

Only 3-4 production-grade B-rep kernels exist: Parasolid (Siemens), ACIS (Spatial/Dassault), CGM (Dassault), and arguably OpenCASCADE. This is not an accident. The barriers are structural and compounding:

**Boolean operations have no general solution.** Union, intersection, and subtraction of arbitrary B-rep solids require face-face intersection, curve trimming, vertex creation, and topological sewing. There is no uniform algorithm that handles every geometry combination. Each operation needs custom implementations for different topology/geometry pairings, and the number of edge cases is effectively infinite. Parasolid has accumulated thousands of these over 35+ years.

**Floating-point tolerance poisons everything.** B-rep maintains dual bookkeeping: geometric data (exact curves/surfaces) and topological data (connectivity graphs). Because floating-point arithmetic means "equal" is never exact, kernels define tolerances (typically 10^-6 meters). When tolerances disagree between operations, or when files move between systems, geometry and topology drift out of sync. Healing this is itself a research problem.

**Topology changes are combinatorially explosive.** During booleans or fillets, both geometry and the connectivity graph change. For every possible topology change there is a different code path. Robust kernels have implemented hundreds or thousands of variants over decades. A new kernel hits "works on simple cases, fails on real parts" almost immediately.

**The chicken-and-egg problem.** Nobody adopts a kernel that fails on real-world parts. But you cannot discover and fix failure modes without real-world adoption. The commercial kernels solved this by being embedded in products (Unigraphics, CATIA) that forced them through millions of user interactions over decades.

**Fillets and shells are harder than booleans.** These "simple" operations require offset surface computation, self-intersection detection, and topology reconstruction in degenerate configurations. Most new kernel projects stall here.

**STEP/IGES interop is its own multi-year project.** The STEP standard (ISO 10303) is enormous. Correct import/export requires handling ambiguous interpretations, tolerance mismatches, and format quirks accumulated over 30 years.

---

# 2. Existing Kernel Landscape

## 2.1 Commercial Kernels

### Parasolid (Siemens, 1988)

The gold standard. 200+ ISVs, 350+ applications. Used by SolidWorks, NX, Onshape, Shapr3D.

- C/C++, B-rep (NURBS + analytic surfaces)
- Full operations: booleans, fillets, chamfers, shells, sweeps, lofts, direct editing, sheet metal
- **Convergent Modeling** (~2017): mesh and B-rep coexist in a single model. Boolean operations work between mesh bodies and precise bodies. This is the leading hybrid capability in any kernel.
- File format: X_T/X_B (openly documented, backward compatible to v1.0), STEP, IGES, STL
- Proprietary OEM licensing (royalty-based, no public pricing, no source access)
- 35+ years of edge-case coverage. The moat is not the algorithms but the accumulated fixes.

### ACIS (Spatial/Dassault, 1989)

More modular/extensible than Parasolid via C++ class hierarchy. ~400 customers.

- Named for founders: Alan, Charles, Ian's System
- ISVs can subclass and extend kernel objects (more flexible, more complex)
- Used by BricsCAD, SpaceClaim (Ansys), TurboCAD, Cimatron
- Owned by Dassault (direct competitor to most licensees), creating ongoing tension
- SAT format no longer openly published since ~2000
- Boolean robustness historically considered slightly behind Parasolid

### CGM (Dassault, late 1990s)

Built from scratch for CATIA V5. Unique advantage: native B-rep compatibility with CATIA/3DEXPERIENCE.

- Best-in-class Class-A surfacing (automotive/aerospace exterior surfaces)
- Not thread-safe (uses multiprocessing instead of multithreading)
- ~1/3 of Spatial's 400 customers use CGM
- Desktop SolidWorks uses Parasolid; 3DEXPERIENCE SolidWorks uses CGM (tension in Dassault ecosystem)

### ShapeManager (Autodesk, 2001)

Forked from ACIS 7.0 for $6.4M when Dassault acquired Spatial.

- Internal only, not licensed to third parties
- Powers Inventor, AutoCAD 3D, Fusion 360 (partially)
- Diverged from ACIS over 20+ years

### Licensing Creates Opportunity

Parasolid is owned by Siemens (competitor to licensees). ACIS/CGM are owned by Dassault (competitor to licensees). There is genuine demand for a high-quality, independently governed kernel.

## 2.2 Open-Source Kernels

### OpenCASCADE (OCCT, 1990s, open-sourced 1999)

The only production-grade open-source B-rep kernel. Period. Everything else is experimental.

- Origin: Matra Datavision (France), built for Euclid Quantum CAD. Now owned by Open Cascade SAS (subsidiary of Capgemini).
- C++ (~1M+ lines), LGPL-2.1
- Full NURBS/B-spline infrastructure, non-manifold modeling, shape healing toolkit
- STEP (AP203/AP214), IGES, BREP, STL, OBJ
- Users: FreeCAD, CadQuery, Build123d, KiCad (3D viewer), Gmsh, Salome-Meca

**What it does well:**
- Comprehensive NURBS/spline infrastructure (homogeneous BSpline formulation for all curves/surfaces)
- Surface interpolation and approximation algorithms
- Non-manifold/mixed-dimensional modeling (rare capability)
- Intersection and projection algorithms that work for most cases
- Shape healing for repairing imported geometry

**What everyone complains about:**
1. **API is atrocious.** Cryptic class names (BRepBuilderAPI_MakeEdge, TopoDS_Shape, gp_Pnt). No modern C++ idioms. Until v7.0, required a custom meta-language (CDL) and build tool (WOK).
2. **Documentation is poor and deteriorating.** Examples are sparse, many don't work.
3. **Boolean operations fail on edge cases.** Tolerance gaps, near-tangent intersections, sliver faces. "Fuzzy booleans" help but are a band-aid.
4. **Build system and installation are painful.** Complex CMake, platform-specific issues.
5. **No C API.** Bindings to other languages are painful. opencascade-rs and OCP (Python) are community-maintained.
6. **Visualization is weak.** FreeCAD chose Coin3D over OCCT's viewer for performance.
7. **STEP/IGES import has quirks.** Position/rotation errors, missing surfaces, tolerance mismatches.

**Honest assessment:** Capable for 80% of use cases. The last 20% (complex booleans on real parts, reliable STEP import, industrial fillets/shells) is where it falls short vs. Parasolid. Its API is a tax on every line of code.

### libfive (Matt Keeter, 2015)

Implicit/F-rep kernel. Objects defined as scalar fields (boundary = 0, interior < 0, exterior > 0).

- CSG is trivially robust: union = min(a,b), intersection = max(a,b)
- Blends and warps are simple function compositions
- Excellent meshing algorithm (watertight, manifold, hierarchical, feature-preserving)
- MPL-2.0
- **Cannot output B-rep/STEP.** Mesh output only. Unsuitable for traditional CAD/manufacturing workflows.

### ImplicitCAD (~2012, Haskell)

Same implicit paradigm. Rounded/bevelled CSG is a first-class operation.

- AGPL-3.0, small community, beta quality
- OpenSCAD-compatible input syntax
- No STEP/IGES support (mesh output only)

### CadQuery / Build123d (Python wrappers over OCCT)

**CadQuery** (~2015, Apache 2.0): Fluent API wrapping OCCT. `cq.Workplane("XY").box(10,10,10).edges().fillet(1)`. Lossless STEP export (the killer feature vs. OpenSCAD).

**Build123d** (~2022, Apache 2.0): Rewrite with context-manager-based API. Three styles: Algebra, Builder, Direct. Becoming the preferred Python CAD library for LLM integration.

**What they fix:** Sane Python API, feature-relative positioning, browser support via WASM-compiled OCP.
**What they don't fix:** Boolean robustness is still OCCT's. STEP import quirks persist. Cryptic error messages from OCCT's C++ layer.

## 2.3 Comparison Matrix

| Kernel | Type | B-rep | NURBS | STEP | Booleans | Fillets | License | Maturity |
|--------|------|-------|-------|------|----------|---------|---------|----------|
| Parasolid | Commercial | Yes | Yes | Yes | Excellent | Excellent | Proprietary | 35yr |
| ACIS | Commercial | Yes | Yes | Yes | Very good | Very good | Proprietary | 35yr |
| CGM | Commercial | Yes | Yes | Yes | Very good | Excellent | Proprietary | 25yr |
| OCCT | Open source | Yes | Yes | Yes (quirky) | Fair-Good | Fair-Good | LGPL-2.1 | 30yr |
| libfive | Open source | No | No | No | Trivial (implicit) | Blends only | MPL-2.0 | 9yr |
| CadQuery | Open source | Yes (OCCT) | Yes (OCCT) | Yes (OCCT) | OCCT's | OCCT's | Apache 2.0 | 9yr |
| Build123d | Open source | Yes (OCCT) | Yes (OCCT) | Yes (OCCT) | OCCT's | OCCT's | Apache 2.0 | 3yr |

---

# 3. Emerging Rust-Based Kernels

Three Rust kernels are in various stages of development. None is production-ready.

## 3.1 Fornjot (Hanno Braun, 2021)

The most ambitious attempt at a clean-sheet B-rep kernel in Rust.

- **Creator**: Hanno Braun, self-employed developer in Germany. Full-time on Fornjot since early 2022 via GitHub Sponsors (~34 sponsors).
- **Architecture**: Pure B-rep. Key design decision: topology and geometry are separated into distinct layers, allowing alternative geometry representations to be tested in parallel.
- **Stats**: ~30-40K LOC Rust, 2,521 stars, 44 contributors (overwhelmingly Hanno), ~19,822 commits, 0BSD license (more permissive than MIT).
- **Rendering**: wgpu (WebGPU abstraction), WebAssembly first-class target.

**What works:**
- Sketch (2D polygon), Sweep (straight path), Split, Holes, Transform
- Export: 3MF, STL, OBJ (mesh-based only)

**What doesn't work yet:**
- **Boolean operations** (the top priority, described as "very incomplete")
- NURBS/B-spline/Bezier curves and surfaces
- Fillets, chamfers
- Circular/helical/arbitrary-path sweeps, lofts
- STEP import/export
- Assemblies, constraint solving

**Status**: Experimental. The mainline code paused in favor of an experimental `new/` module (a deliberate architectural reset). After 5 years and ~20K commits, basic booleans still don't work. Feature wishlist deprecated Oct 2024. Funding is precarious.

**Assessment**: Architecturally thoughtful (topology/geometry separation is genuinely good), honest about limitations, 0BSD license removes all friction. But the pace of progress vs. the scope of the problem is concerning. Not a viable foundation for a product today.

## 3.2 OpenGeometry (aka-blackboots, 2024)

A browser-native CAD kernel focused on AEC/BIM.

- **Creator**: Vishwajeet Mane (solo developer), 112 commits since Aug 2024
- **Architecture**: Half-edge B-rep data model, compiled to WebAssembly, Three.js TypeScript wrapper
- **Stats**: ~18K LOC Rust, 310 stars, 1 contributor, MPL-2.0 license
- **Downstream**: OpenPlans (floorplan design app)

**What works:**
- Extrusion, sweep along path, 2D offset
- Booleans (union/intersection/subtraction) via external `boolmesh` crate (mesh-based, not analytical)
- BRep editor: push/pull, split, cut, move with automatic rollback on validation failure
- Export: STL, STEP (triangulated mesh only, not true NURBS B-rep), IFC, PDF
- Transactional BrepBuilder with invariant validation

**What doesn't work:**
- No NURBS, B-splines, Bezier curves (all geometry is tessellated)
- No fillets, chamfers, lofts, shells, patterns
- No file import at all
- STEP export writes triangulated mesh, not true B-rep
- ~23 tests total

**Notable**: Unusually thorough AI agent documentation (AGENTS.md, Claude skills, knowledge docs). Clearly designed for AI-assisted development.

**Assessment**: A focused web-native kernel for AEC/BIM and simple parametric modeling. Think "enough kernel for a floorplan editor" rather than "FreeCAD replacement." Adding NURBS would require rearchitecting the geometry model, not just adding a module. Forking is mechanically easy but strategically risky for anything requiring geometric depth.

## 3.3 Truck (ricosjp) + CADmium

- **Truck**: B-rep kernel in Rust. Handles NURBS surfaces, STEP I/O, boolean operations. Used by CADmium. MIT/Apache. Very early stage.
- **CADmium** (Matt Ferraro): Browser-based CAD using Truck kernel + Rust + WebAssembly. Local-first architecture. Proof-of-concept quality.

These are the most interesting of the Rust kernels because Truck actually handles NURBS, but neither is battle-tested.

---

# 4. Zoo/KittyCAD: The Primary Competitor

Zoo (formerly KittyCAD) is executing on almost exactly the thesis of "GPU-native, AI-first CAD kernel." They have built the full stack from geometry engine to AI agent. Threat level: HIGH.

## Geometry Engine

- In-house B-rep kernel running on GPU (Vulkan, primarily NVIDIA)
- **Cloud-only.** Not a downloadable library. Client sends modeling commands via WebSocket, engine returns video frames via WebRTC + NVIDIA hardware encoder.
- Minimal primitive set: fillets, chamfers, etc. represented as B-splines with specific constraints (rather than special-case surface types). The boolean engine only needs to handle B-spline/B-spline intersection.
- SSI reformulated as parallelizable root-finding on GPU (rather than traditional branching code paths)
- Claims team of <30 engineers closed "decades of feature gap" in under two years

## KCL (KittyCAD Language)

- Functional programming language for parametric CAD
- Variables not mutable, no loops, referential transparency
- Plain-English units (mm, inches, can mix)
- Text-based = diffable, version-controllable, LLM-readable
- Point-and-click UI available; code is optional

## Zookeeper (AI Agent, shipped Jan 2026)

- Conversational CAD agent with access to geometry engine
- Writes, executes, and debugs KCL
- Analyzes 3D models via multi-view snapshots
- Computes mass, volume, surface area, center of mass
- Produces transparent, inspectable feature trees
- Available via Zoo-MCP server for integration with external AI agents

## Text-to-CAD (ML-ephant)

- Proprietary ML model trained on Zoo's datasets
- Generates editable, parametric B-rep models from text prompts

## Strategic Assessment

**Strengths**: Vertical integration (kernel to language to AI to app). GPU-native from day one. API-first design with real developer ecosystem. KCL's text-based representation is a structural advantage for AI.

**Weaknesses**:
- Cloud-only engine (no offline/on-prem). All geometry operations require network round-trips.
- Proprietary engine (not open source)
- Feature coverage vs. Parasolid/ACIS is still a question mark
- The "<30 engineers in 2 years" claim may not hold as they hit the long tail of edge cases

## Differentiation Opportunities vs. Zoo

1. **Local/offline execution** (Zoo is cloud-only)
2. **Open-source kernel** (Zoo's engine is proprietary)
3. **Native Rust library** (not a cloud API with network latency)
4. **Better error reporting** designed for AI agents from the start
5. **Persistent/branchable model state** for AI exploration (try option A and B, compare, choose)
6. **Python ecosystem integration** (CadQuery/Build123d compatibility layer)
7. **Manufacturing constraint awareness** built into the kernel

---

# 5. GPU-Accelerated Geometry: Honest Assessment

## Where GPU Genuinely Helps (10-100x speedup realistic)

| Operation | Why GPU Helps | Evidence |
|-----------|--------------|----------|
| NURBS surface evaluation | Pure math on independent UV points | >40x faster (UC Berkeley) |
| Surface-surface intersection | Subdivision + root-finding on independent patches | >50x vs ACIS (IEEE TVCG 2013) |
| Tessellation | Hardware tessellation units since DX11 | Mature, shipping on every GPU |
| Ray casting / point-in-solid | Embarrassingly parallel per-ray | Billions of rays/sec on RTX hardware |
| Minimum distance / clearance | Per-point-pair computation | >100x faster (NVIDIA GTC 2010) |
| Convex hull / Delaunay | Well-studied parallel algorithms | CudaHull: 30-40x vs Qhull |
| Boolean ops on meshes | Per-surfel in/out classification (LDI/LDNI) | Fast but approximate (mesh-only) |

## Where GPU Does Not Help (<5x, possibly slower)

| Operation | Why GPU Struggles |
|-----------|------------------|
| B-rep boolean: topology reconstruction | Sequential, conditional branching, thousands of special cases |
| Fillet/chamfer computation | Offset surface + self-intersection detection, degenerate cases |
| Shell operations | Offset + topology change |
| Feature recognition | Graph analysis on B-rep topology |
| STEP/IGES parsing | String parsing, schema interpretation |
| Constraint solving (2D sketcher) | Small sparse systems, branching |

## The Honest Framing

"GPU-native CAD kernel" is a great fundraising sentence, but the core B-rep operations (booleans, fillets, topology manipulation) are not well-suited for GPU parallelism. They're branchy, have data dependencies, and involve complex control flow. GPUs excel at SIMD workloads; core B-rep operations are the opposite.

Research on parallel polygon booleans (2D case, simpler than 3D) found synchronization overhead limits speedup to 3-4x (PolygonTailor, MDPI 2025). The 3D B-rep case is worse.

**Zoo's approach**: Use GPU for the numerically heavy stages (intersection, evaluation, tessellation) and CPU for the topologically complex stages (reconstruction, fillet logic). They also kept a minimal primitive set (B-splines only) so the boolean engine handles fewer cases.

**The right framing**: "Modern CAD kernel with GPU acceleration where it matters" rather than "GPU-native kernel." The actual differentiator should be the AI-native API and clean Rust codebase, not GPU parallelism.

## NVIDIA's Actual Work in This Space

- **Dyndrite ACE**: "World's first fully GPU-native geometry kernel." NVIDIA-backed, CUDA/Thrust, hybrid kernel (mesh + B-rep + voxel + tet mesh). **Focused on additive manufacturing** (lattice generation, support structures, slicing, toolpaths), not general-purpose mechanical CAD.
- **Omniverse**: Visualization/simulation platform on OpenUSD. Consumes CAD output, does not produce it. Not a geometry engine.
- **OptiX**: Programmable ray tracing engine. Building block for GPU spatial queries but not itself a geometry engine.

---

# 6. AI + CAD Intersection

## 6.1 What Makes a CAD API "AI-Friendly"

Based on research and what works/fails in current LLM-CAD systems:

**Deterministic operations.** Same inputs must produce identical outputs. No randomized algorithms. Functional programming (KCL, CadQuery) achieves this via referential transparency.

**Composable primitives.** Small, orthogonal operations that combine predictably. `sketch -> extrude -> fillet -> pattern` rather than monolithic commands. Each operation should have a clear contract: what it takes, what it returns, what can fail.

**Good error messages.** This is where OCCT fails catastrophically. Errors like "BRep_API: command not done" give no useful information. An AI-friendly kernel should report: what operation failed, which geometry was involved, why it failed (self-intersection? zero-thickness? tolerance issue?), and what to try instead.

**Introspection.** Programmatically query: how many faces? what are the edge lengths? what is the volume/mass/center-of-mass? Topology traversal: "give me all edges adjacent to this face."

**Undo/rollback and branching.** AI agents explore, make mistakes, backtrack. Persistent data structures (immutable snapshots at each operation) would enable branching exploration (try option A and option B, compare, choose). No current kernel supports this.

**Streaming/incremental evaluation.** Show partial results as operations execute so AI can abort early if heading in the wrong direction.

**Serialization to text.** The entire model state should be representable as human-readable text (KCL, CadQuery code, etc.).

## 6.2 Code-CAD as the AI Bridge

Code-CAD tools (CadQuery, Build123d, OpenSCAD, KCL) represent 3D geometry as programs. This matters because text is what LLMs understand.

- **CadQuery**: Python wrapping OCCT. Fluent API. 3.6k GitHub stars.
- **Build123d**: Rewrite of CadQuery. Becoming the preferred target for LLM-CAD research.
- **KCL** (Zoo): Functional language designed for CAD. Referential transparency.
- **OpenSCAD**: Custom functional language with non-standard semantics. Large community but its non-standard language is a liability for LLMs.

All Python code-CAD tools wrap OCCT, inheriting its quirks, error messages, and failure modes. The opportunity is a kernel that doesn't have those quirks.

## 6.3 LLM-Driven CAD Research (2024-2026)

The field is moving fast. Key results:

| System | Year/Venue | What It Does | Key Result |
|--------|-----------|--------------|------------|
| **CAD-Recode** | ICCV 2025 | LLM generates CadQuery code from point clouds | 10x lower Chamfer distance than prior art |
| **CADFit** | 2026 | Hybrid neural+optimization mesh-to-CAD | Richest operator set (extrusions, revolutions, fillets, chamfers) |
| **BrepGen** | SIGGRAPH 2024 | Diffusion model directly generating B-rep | First to generate free-form and doubly-curved surfaces |
| **CADSmith** | arXiv 2026 | Multi-agent pipeline (Planner/Coder/Executor/Validator) | 100% execution rate, median IoU 0.9629 |
| **CADFusion** | ICML 2025 | Fine-tuned LLaMA-3-8B for text-to-CAD | Two-stage: sequence learning + visual feedback |
| **Text-to-CadQuery** | arXiv 2025 | Fine-tuned LLMs on 170K CadQuery annotations | 69.3% exact match |
| **Zero-to-CAD** | arXiv 2026 | Synthetic CadQuery code generation at scale | Addresses data scarcity |
| **Autodesk Neural CAD** | AU 2025 | Foundation models for geometry in Fusion | Claims 80-90% automation of routine design. Not yet public. |

**Key trends:**
1. LLMs as code generators (generating CadQuery/KCL rather than learned sequence representations) is the most promising direction
2. Closed-loop refinement (generate, execute, verify, fix) outperforms single-pass generation
3. Hybrid approaches (neural proposals + geometric optimization) produce the best results
4. Data scarcity remains the bottleneck; synthetic data generation is one path forward
5. Foundation models (Autodesk, Zoo) may commoditize text-to-CAD in 2026-2027

**The CADSmith architecture** (Planner -> Coder -> Executor -> Validator with inner/outer loops) is likely the right pattern for AI-driven CAD. This is exactly the kind of pipeline that benefits from a kernel with good error messages and programmatic verification.

---

# 7. Mesh-to-CAD: The Wedge Product

## Why This Is the Right Starting Point

1. **Easier to sell** than "replace your kernel." Manufacturers have a concrete pain point (manual scan-to-CAD reconstruction costs $2K-20K+ per part).
2. **More unsolved** than general CAD operations. No automated end-to-end pipeline exists.
3. **Natural entry to kernel development.** Building mesh-to-CAD forces you to build the B-rep data structures, surface fitting, topology construction, and STEP export that form the foundation of a kernel.
4. **Market pull.** 3D scanning market ~$5B (2024), growing ~11% CAGR. The software portion (scan-to-CAD) is $500M-1B, dominated by Hexagon/Geomagic.

## The Problem

Converting triangle meshes/point clouds into parametric B-rep CAD models. The gaps:

- **Surface fitting is underdetermined.** Infinitely many NURBS surfaces can interpolate a point cloud. Must infer surface type, parameters, and trim boundaries.
- **Trim curves are the root of evil.** Reconstructing trim curves requires surface-surface intersection (numerically fragile) in dual parameter spaces, with topological consistency.
- **Topology recovery is combinatorial.** Determining which surfaces are adjacent and how they connect is a search problem.
- **Design intent is ambiguous.** The same geometry can be produced by many construction sequences.

## Commercial Landscape

| Tool | Strengths | Limitations | Pricing |
|------|-----------|-------------|---------|
| **Geomagic Design X** (Hexagon) | Market leader. Auto mesh segmentation, analytic surface fitting, live transfer to SolidWorks/NX | Still heavily manual for complex parts. No parametric feature tree output. | ~$1,900-$2,090+ |
| **Siemens NX RE** | Convergent Modeling (mesh + solid coexist without conversion). Booleans between mesh and solid bodies. | Expensive ($10K+/seat/yr). Pragmatic rather than solving fundamental reconstruction. | Enterprise |
| **SpaceClaim** (Ansys) | Fast surface-to-mesh fitting, direct modeling | "Dumb solid" output (no parametric history) | Enterprise |
| **SolidWorks ScanTo3D** | Familiar environment for SW users | Very limited without third-party plugins (Xtract3D, Geomagic for SW) | Included w/ SW |

**Universal limitation**: none produce a true parametric feature tree. Output is solid bodies or surface quilts, not the ordered sequence of sketch-extrude-fillet operations a designer would create.

## Research Frontier

**CAD-Recode (ICCV 2025)**: LLM generates executable CadQuery code from point clouds. 10x lower Chamfer distance than prior art. The most promising direction: treating reconstruction as a code generation problem leverages LLM pre-training.

**CADFit (2026)**: Hybrid optimization-based framework. Recovers construction sequences from meshes including extrusions, revolutions, fillets, and chamfers. Richest operator set of any research system.

**BrepGen (SIGGRAPH 2024, Autodesk Research)**: First diffusion model generating free-form B-reps directly. Novel structured latent geometry tree.

**ComplexGen (SIGGRAPH 2022)**: B-rep reconstruction via chain complex generation. Sparse CNN + tri-path Transformer. Global optimization for structural validity.

**Key insight**: The best results combine neural proposals with geometric optimization. Pure learning struggles with geometric precision; pure optimization struggles with combinatorial complexity.

## What "Good Enough" Looks Like

**For mechanical parts (prismatic features) -- the low-hanging fruit:**
- Automatic detection of planes, cylinders, cones, spheres, tori (95%+ accuracy)
- Automatic detection of fillets/chamfers and their radii
- Automatic detection of holes (through, blind, counterbored, countersunk)
- Reconstruction of a parametric feature tree (not just surfaces)
- Dimensional accuracy within 0.05mm
- Output as STEP file with editable features
- Total human intervention: <30 minutes for a part that currently takes 8+ hours

**Current state**: primitive detection is largely solved (RANSAC). Everything else remains hard. CADFit and CAD-Recode show the most promise but only on relatively simple shapes.

## The Pragmatic Initial Product

1. Mechanical parts only (prismatic, analytic surfaces)
2. Clean scan data (lab-quality, not field scans)
3. Semi-automatic workflow (software does 80%, user makes key decisions)
4. Output as editable B-rep solid (not necessarily full feature tree initially)
5. Accuracy within 0.1mm

This would capture aerospace/energy MRO where parts are mostly prismatic but volumes are high.

---

# 8. Strategic Analysis

## Where to Start

**Option A: Fork/Extend an Existing Rust Kernel**

| Kernel | Pros | Cons |
|--------|------|------|
| **Truck** | Has NURBS, STEP I/O, booleans. Most complete Rust kernel. | Very early, untested at scale. Small community. |
| **Fornjot** | Thoughtful architecture (topology/geometry separation). 0BSD license. Active commits. | Can't do booleans after 5 years. No NURBS. Slow progress. |
| **OpenGeometry** | Small, clean, browser-native. AI agent docs. | No NURBS (fundamental limitation). Solo developer. ~23 tests. |

**Assessment**: Truck is the most viable starting point among Rust kernels because it has NURBS. But all are risky foundations for a product.

**Option B: Wrap OCCT and Build Up**

Use OCCT as the B-rep backend (via CadQuery/OCP bindings), build the AI-friendly API and mesh-to-CAD pipeline on top. Progressively replace OCCT internals with custom Rust implementations where it matters most.

**Pros**: Immediate access to 30 years of B-rep operations, STEP support, NURBS. CadQuery/Build123d ecosystem. Can ship a product faster.
**Cons**: Inherit OCCT's warts. C++ dependency complicates the Rust story. Hard to claim "clean-sheet kernel" while wrapping OCCT.

**Option C: Clean-Sheet Kernel**

Start from scratch in Rust. Follow Zoo's approach: minimal primitive set (B-splines only), reformulate SSI as parallelizable root-finding.

**Pros**: No legacy baggage. Full architectural control. Best long-term story.
**Cons**: Years before feature parity even with basic OCCT. Zoo has a 2+ year head start.

**Recommendation: Option B as the bridge to Option C.** Build the product (mesh-to-CAD) on OCCT to prove PMF. Simultaneously develop core Rust B-rep data structures. Swap out OCCT subsystems one at a time as the Rust implementations mature.

## The YC Angle

James's insight: YC is actively hunting for physical product / manufacturing / hardware infra deals. An AI-native CAD company fits that thesis.

**One-sentence pitch**: "We're building the CAD infrastructure that lets AI agents design physical products, starting with the hardest unsolved conversion problem in engineering software."

**What to show in 4 weeks:**
1. Take a 3D scan / STL mesh
2. Run it through a pipeline that extracts B-rep features (planes, cylinders, fillets)
3. Output a parametric CAD model (STEP file) that you can edit
4. Show an LLM driving the whole thing via a clean API
5. Record demo video

**What to emphasize:**
- "Only 3 B-rep kernels exist. The only open-source one is from 1993. We're building modern CAD infrastructure for AI agents."
- Mortise as proof you ship
- Mesh-to-CAD as the wedge ("reverse engineering costs manufacturers $X billion/year in manual CAD reconstruction. We automate it.")
- 2-3 LOIs from manufacturing companies or hardware startups

**The long game**: The kernel. But YC likes wedge-then-platform stories.

## Competitive Positioning

| Competitor | Their Angle | Our Differentiation |
|-----------|-------------|-------------------|
| **Zoo/KittyCAD** | Cloud-only GPU engine + KCL + Zookeeper agent | Local/offline, open-source kernel, Python ecosystem, mesh-to-CAD |
| **Geomagic** (Hexagon) | Mature scan-to-CAD with manual workflow | AI-automated, 10x faster, no expert required |
| **Autodesk Neural CAD** | Foundation models inside Fusion | Open, not locked to one vendor's ecosystem |
| **OCCT wrappers** (CadQuery/Build123d) | Python API over legacy C++ | Native Rust kernel, better errors, GPU-accelerated where it matters |

## The Hybrid B-rep + Implicit Approach

Potentially novel: a kernel that handles B-rep for precision, implicit/F-rep for organic/generative geometry, and mesh for scan data, all in one model. Parasolid's convergent modeling (B-rep + mesh) is the closest existing capability. Adding implicit would be genuinely new.

---

# 9. Key References

## CAD Kernel Landscape
- [Parasolid - Siemens](https://plm.sw.siemens.com/en-US/plm-components/parasolid/)
- [ACIS - Wikipedia](https://en.wikipedia.org/wiki/ACIS)
- [Spatial, ACIS, CGM - Engineering.com](https://www.engineering.com/spatial-acis-cgm-and-the-future-of-geometric-modeling-kernels/)
- [Kernel Wars - Demystifying PLM](https://demystifyingplm.ghost.io/kernel-wars/)
- [Open Cascade Technology - Wikipedia](https://en.wikipedia.org/wiki/Open_Cascade_Technology)
- [B-rep: What it is and why it's a problem - Shapr3D](https://www.shapr3d.com/content-library/what-is-b-rep)
- [Boolean operations survey (CAD Journal, 2026)](https://www.sciencedirect.com/science/article/abs/pii/S0010448526000515)

## Rust Kernels
- [Fornjot](https://github.com/hannobraun/fornjot) / [fornjot.app](https://www.fornjot.app/)
- [OpenGeometry](https://github.com/OpenGeometry-io/OpenGeometry) / [opengeometry.io](https://opengeometry.io)
- [Truck](https://github.com/ricosjp/truck)
- [CADmium](https://mattferraro.dev/posts/cadmium)

## Zoo/KittyCAD
- [Zoo CAD Engine Overview](https://zoo.dev/research/zoo-cad-engine-overview)
- [KCL Language](https://zoo.dev/research/introducing-kcl)
- [Zookeeper Agent](https://zoo.dev/research/zookeeper)
- [Text-to-CAD](https://zoo.dev/research/introducing-text-to-cad)
- [Modeling App (open source)](https://github.com/KittyCAD/modeling-app)

## GPU Computational Geometry
- [GPU-based Boolean Ops on Triangulated Solids (Eurographics)](https://diglib.eg.org/items/c90c0c74-92c9-4921-af1c-4846f835ae1c)
- [Affine Arithmetic SSI with GPU Acceleration (IEEE TVCG 2013)](https://dl.acm.org/doi/10.1109/TVCG.2013.237)
- [Direct NURBS Evaluation on GPU (UC Berkeley)](https://mcmains.me.berkeley.edu/pubs/SPM07KrishnamurthyKhardMcMains.pdf)
- [Parallel GPU Algorithms for Mechanical CAD (UC Berkeley thesis)](https://escholarship.org/uc/item/59n1g12w)
- [Dyndrite ACE - NVIDIA blog](https://developer.nvidia.com/blog/dyndrite-unveils-first-gpu-accelerated-geometry-kernel-to-tackle-data-explosion-in-additive-manufacturing/)
- [Hybrid Boolean Operations (ACM TOG 2025)](https://dl.acm.org/doi/abs/10.1145/3730908)

## AI + CAD Research
- [CAD-Recode (ICCV 2025)](https://github.com/filaPro/cad-recode)
- [CADFit (2026)](https://arxiv.org/abs/2603.26512) -- verify URL
- [BrepGen (SIGGRAPH 2024)](https://github.com/samxuxiang/BrepGen)
- [CADSmith (arXiv 2026)](https://arxiv.org/abs/2603.26512)
- [CADFusion (ICML 2025)](https://github.com/microsoft/CADFusion)
- [Text-to-CadQuery (arXiv 2025)](https://arxiv.org/abs/2505.06507)
- [Zero-to-CAD (arXiv 2026)](https://arxiv.org/abs/2604.24479)
- [Autodesk Neural CAD (AU 2025)](https://www.research.autodesk.com/blog/reimagine-cad-new-at-tech-au-2025/)

## Mesh-to-CAD
- Schnabel et al., "Efficient RANSAC for Point-Cloud Shape Detection" (2007)
- Wu et al., "DeepCAD" (ICCV 2021) / [GitHub](https://github.com/rundiwu/DeepCAD)
- Guo et al., "ComplexGen" (SIGGRAPH 2022) / [GitHub](https://github.com/guohaoxiang/ComplexGen)
- Xu et al., "BrepGen" (SIGGRAPH 2024) / [GitHub](https://github.com/samxuxiang/BrepGen)
- Rukhovich et al., "CAD-Recode" (ICCV 2025) / [GitHub](https://github.com/filaPro/cad-recode)
- [Point2CAD (CVPR 2024)](https://github.com/prs-eth/point2cad)

## Code-CAD Tools
- [CadQuery](https://github.com/CadQuery/cadquery) / [Build123d](https://github.com/gumyr/build123d)
- [OpenSCAD](https://openscad.org) / [JSCAD](https://jscad.app/)
- [cad-agent (build123d + MCP)](https://github.com/Svetlana-DAO-LLC/cad-agent)
- [build123d-mcp](https://glama.ai/mcp/servers/pzfreo/build123d-mcp)
