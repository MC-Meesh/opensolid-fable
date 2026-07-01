# OpenSolid Kernel Spec — Simulated Postmortem

## 15 Personas: Honest Assessment

---

### 1. CAD Application Developer (Building a Desktop Parametric Modeler)

**Who they are:** Senior developer at a 10-person startup building a Solidworks competitor. Needs a kernel they can ship commercially without Parasolid licensing fees ($50k+/seat/year). Has 15 years of C++ geometry experience, now writing Rust.

**What works well:**
- Rust's memory safety eliminates an entire class of kernel crashes they've fought for years
- Deterministic operations mean their undo/redo system actually works
- Branching model maps perfectly to parametric design history
- Strong typing catches topology errors at compile time rather than runtime segfaults

**What does NOT work / frustrates them:**
- **Fillet/chamfer is the #1 feature customers ask for and it's the hardest B-rep operation to get right.** If the spec doesn't nail edge-blend topology and handle degenerate cases (fillets larger than adjacent faces, fillet-on-fillet chains), the kernel is unusable for real products. This operation alone has consumed person-decades at Siemens.
- **No draft angle support.** Every injection-molded part needs draft. This is table-stakes for mechanical CAD.
- **Shell operation** (hollowing a solid with uniform wall thickness) — if missing or unreliable, users can't model enclosures, which is 40% of consumer product design.
- **Parametric constraint solver** — the kernel probably punts on this, but without 2D sketch constraints (tangent, coincident, equal-length, fully-constrained detection), no one can build a usable modeler on top.
- **History rebuild performance** — if rebuilding a 200-feature tree takes more than 2 seconds, users will leave. Incremental evaluation is essential but architecturally hard.
- **Naming persistence** — when a fillet changes the face count, how do downstream features find their references? TNaming is an unsolved nightmare in OCCT. If OpenSolid doesn't have a persistent naming scheme from day one, every app built on it will have the "lost reference" bug.

**What they'd wish was different:**
- Ship fillet/chamfer that handles 95% of cases before claiming production-ready
- Provide a face/edge naming persistence API as a first-class concept, not an afterthought
- Include a sketch constraint solver or at least a clean interface point for one

---

### 2. Manufacturing Engineer (STEP Interop with Real Parts)

**Who they are:** 20-year veteran at a Tier 1 automotive supplier. Receives STEP files from OEMs daily. Needs to import them, inspect geometry, detect manufacturing features, and export toolpath-ready models. Currently uses Parasolid via NX.

**What works well:**
- If STEP AP214/AP242 import actually works reliably, that alone justifies evaluation
- Deterministic operations mean batch processing of 500 parts gives consistent results
- Rust performance for large assemblies (50k+ bodies) could beat their current OCCT-based tools

**What does NOT work / frustrates them:**
- **STEP import fidelity is everything and it will be broken.** Real-world STEP files from Catia/NX/Creo contain: trimmed NURBS with knot tolerances outside spec, degenerate faces (zero-area triangular patches on sphere poles), self-intersecting trim curves at seam edges, mixed units within a single file, and non-manifold sheet bodies that "shouldn't work but do." If OpenSolid rejects 5% of files that Parasolid reads, it's unusable.
- **Healing operations are mandatory.** Gaps between faces (up to 0.01mm is common in automotive STEP), short edges, sliver faces — the kernel needs a `heal()` operation that closes gaps, merges near-coincident vertices, and removes degenerate topology. OCCT's `ShapeFix` is terrible but necessary.
- **PMI/GD&T preservation** — STEP AP242 carries tolerancing data. If import drops it, the model is worthless for manufacturing.
- **No IGES support** — legacy parts from the 90s still circulate in IGES. "We don't support IGES" means they can't migrate.
- **Surface-surface intersection robustness** — NURBS/NURBS intersection is where every kernel has bugs. Tangent intersections, near-tangent cases, and high-degree surfaces will produce wrong topology. This takes years to harden.
- **Assembly structure** — STEP files have assembly trees with transforms. If OpenSolid only handles single bodies, they can't process real files.

**What they'd wish was different:**
- A "tolerance-aware" import mode that heals instead of rejecting
- An explicit quality report on import (list of issues found and fixed vs. unfixable)
- Assembly support as a first-class concept, not "just use transforms manually"

---

### 3. AI/ML Researcher (Training Models on CAD Data)

**Who they are:** PhD student at Stanford working on generative CAD. Wants to train a transformer that outputs B-rep construction sequences. Needs to convert ABC dataset (1M+ STEP files) into structured training data, and then reconstruct geometry from model predictions.

**What works well:**
- Deterministic operations are perfect for training — same input always gives same output
- Branching model maps to beam search / tree-of-thought generation
- Good error messages mean training data validation is easier
- Rust performance for batch-processing millions of files

**What does NOT work / frustrates them:**
- **No Python bindings.** Their entire ML pipeline is PyTorch/JAX. Calling Rust via FFI or subprocess is painful. They need `pip install opensolid` with numpy-compatible array interfaces for point clouds, meshes, and B-rep feature extraction.
- **B-rep serialization format** — they need a compact, diffable, tokenizable representation of B-rep topology + geometry. STEP is too verbose and ambiguous. They want something like: `[face_type, surface_params, loop_edges, edge_curves...]` as flat tensors.
- **No sketch-and-extrude decomposition** — they want to go from B-rep → construction sequence (sketch, extrude, fillet, chamfer, boolean). This is the inverse problem (feature recognition) and it's unsolved in general, but even heuristic decomposition would be valuable.
- **Evaluation of generated geometry** — they need: is this B-rep valid? Is it watertight? What's the Hausdorff distance to a target shape? What's the Chamfer distance between point samples? These metrics need to be fast (evaluating millions of candidates during training).
- **Partial construction** — their model generates operations one at a time. If operation 7 of 12 fails, they need to know why (self-intersection? degenerate face?) and ideally get the partial result to continue from.
- **No batch API** — processing 1M files means they need `process_batch(files: Vec<Path>) -> Vec<Result<Features>>` not individual file handles. Parallelism should be internal.
- **Topology as a graph** — they want face-adjacency graphs, edge-loops as ordered lists, vertex-edge incidence — all as integer arrays for GNN training. If the API only exposes iterator-based traversal, extracting graph structure is O(n^2) boilerplate.

**What they'd wish was different:**
- Python bindings with PyO3, available on PyPI
- A "training data extraction" module: B-rep → graph tensors in one call
- A validity checker that returns structured diagnostics, not just pass/fail
- Support for partial/invalid geometry (real ML outputs will be slightly wrong)

---

### 4. Hardware Startup (Designing Physical Products)

**Who they are:** 3-person team building a smart home device. CTO is a software engineer who hates Solidworks but needs to design an enclosure, get it injection-molded, and iterate quickly. Currently using Fusion 360 but want programmatic control.

**What works well:**
- Programmatic API means they can parameterize their enclosure (wall thickness, button cutout positions, logo emboss depth) as variables
- Deterministic means CI can rebuild geometry and catch regressions
- No licensing fees (Fusion 360 free tier keeps removing features)
- Version control on geometry (branching model)

**What does NOT work / frustrates them:**
- **No visualization.** They need to SEE their part. If the kernel has no viewer, they're exporting STL and opening MeshLab in a separate workflow. They need at minimum a web-based viewport or integration with a viewer crate.
- **Learning curve is massive.** They're not geometry experts. They want `box(10, 20, 30).fillet(edges=top, radius=2).shell(thickness=1.5).export("part.step")` — a high-level fluent API. If they have to understand B-rep topology, half-edge data structures, and NURBS knot vectors to make a box with rounded corners, they'll go back to Fusion.
- **No standard parts library.** They need M3 screw bosses, snap-fit clips, heat-set insert holes. Every hardware startup models these from scratch — a parametric parts library would be transformative.
- **DFM feedback** — they don't know manufacturing constraints. Minimum wall thickness for ABS injection molding? Minimum draft angle? If the kernel could flag "this 0.3mm wall will fail in molding," that's worth more than any geometric feature.
- **Threads/fasteners** — modeling ISO metric threads is a nightmare in every CAD tool. Cosmetic threads that export correctly to STEP for the mold shop are essential.

**What they'd wish was different:**
- A high-level "maker" API that wraps the low-level B-rep kernel
- Built-in visualization (even just tessellate-to-WebGL)
- A DFM linting module that flags common manufacturing issues

---

### 5. Open Source Contributor (Wants to Help Build the Kernel)

**Who they are:** Senior Rust developer, 5 years experience, contributor to several major crates. Excited about OpenSolid's mission. Has read the spec and wants to implement something. Has no computational geometry background.

**What works well:**
- Rust ecosystem (cargo, docs.rs, CI) makes onboarding easy
- Strong typing means the compiler catches topology errors
- If the code is well-structured with clear module boundaries, they can contribute to one subsystem

**What does NOT work / frustrates them:**
- **Computational geometry is a minefield for newcomers.** They try to implement line-arc intersection and immediately hit: floating point tolerance, degenerate cases (tangent intersection = 1 point not 2), and parametric vs. Cartesian ambiguity. Without extensive documentation of the geometric reasoning behind tolerance decisions, every PR will be wrong.
- **No test oracle.** How do they know their implementation is correct? They need a comparison against a known-good kernel (OCCT) for validation. Without reference implementations or exhaustive test suites with known-correct outputs, contributors will introduce subtle bugs that only manifest on complex models months later.
- **Tolerance philosophy is unclear.** Is this a fixed-epsilon kernel (like ACIS with SPAresabs)? Adaptive tolerance? If every contributor makes different tolerance assumptions, the kernel will be inconsistent. This needs to be documented as rigorously as a programming language spec.
- **Boolean operations are contribution-repellent.** The hardest 20% of the kernel (Booleans, fillets, NURBS intersection) requires deep domain expertise. Contributors will cluster on the easy parts (primitives, transforms, export) leaving the critical path understaffed.
- **Architecture documentation** — without a clear "here's how a Boolean operation flows through the system" diagram, contributors can't orient. OCCT's lack of architecture docs is why it has so few outside contributors despite being open source.

**What they'd wish was different:**
- A "contributor guide" that explains tolerance philosophy, geometric decisions, and common pitfalls
- Extensive property-based tests (quickcheck) that contributors can run to validate their code
- Clear labeling of "good first issues" that don't require geometry PhD
- A reference test suite with STEP files and expected outputs

---

### 6. CadQuery/Build123d User (Considering Switching from OCCT)

**Who they are:** Mechanical engineer who codes Python on the side. Uses Build123d for parametric parts, frustrated by OCCT crashes, terrible error messages, and the fact that their kernel is a 20-year-old C++ library wrapped in SWIG. Wants something modern.

**What works well:**
- Good error messages are literally the #1 complaint about OCCT — this alone could drive adoption
- Deterministic operations (OCCT has race conditions in parallel tessellation!)
- No more "BRepAlgoAPI_Fuse failed with unknown error" crashes
- Rust safety means no more segfaults from dangling TopoDS_Shape references

**What does NOT work / frustrates them:**
- **They need Python bindings NOW.** The entire CadQuery/Build123d ecosystem is Python. If OpenSolid is Rust-only, they literally cannot use it regardless of quality. PyO3 bindings are non-negotiable for this community.
- **API compatibility** — they have thousands of lines of CadQuery scripts. They won't rewrite from scratch. They need either a compatibility layer or a clear migration guide with equivalent operations.
- **OCCT has 30 years of edge-case handling.** Their existing parts use: lofts through 5+ profiles with guide curves, sweep along helical path (threads), thicken surface to solid, offset surface, Boolean operations on thin-walled bodies, and filleting chains of edges with variable radius. If ANY of these regress compared to OCCT, they won't switch.
- **CadQuery's Selector API** — they select faces/edges by position, orientation, and adjacency (`faces(">Z")`, `edges("|X")`). If OpenSolid doesn't have an equivalent selection mechanism, basic workflows become verbose.
- **Jupyter/CQ-Editor integration** — they view models inline in notebooks. If OpenSolid can't tessellate to a format their viewer understands, the development workflow is broken.

**What they'd wish was different:**
- Python-first experience (not "Rust with Python afterthought")
- A CadQuery compatibility adapter or automated migration tool
- Feature parity benchmarks: "these N operations from the CadQuery test suite work in OpenSolid"
- OCP (OCCT Python wrapper) is the current bottleneck — if OpenSolid's Python bindings are cleaner, that's the selling point

---

### 7. Enterprise Evaluator (Comparing Against Parasolid/ACIS Licensing)

**Who they are:** VP of Engineering at a 500-person PLM company. Pays $2M/year to Siemens for Parasolid licenses. Board wants to reduce costs. Evaluating whether an open-source kernel could replace Parasolid in their product within 3 years.

**What works well:**
- Zero licensing cost is obviously compelling ($2M/year savings)
- Rust's safety story is good for their security-conscious customers
- Deterministic operations reduce their support burden (inconsistent results = support tickets)
- Open source means no vendor lock-in, no sudden license changes (Unity-style)

**What does NOT work / frustrates them:**
- **No certification or warranty.** If OpenSolid produces a bad STEP file that causes a CNC machine to cut wrong, who's liable? Parasolid has decades of validation and a company backing it. Open source has... a GitHub issue.
- **Completeness gap is massive.** Parasolid has ~3000 API functions. OpenSolid probably has 50-100. Their product uses 3D curve-on-surface, exact offsets, mid-surface extraction, minimum distance, clash detection on assemblies, and mass properties. If even 10% of required operations are missing, they cannot ship.
- **No large-scale validation.** Parasolid processes millions of models daily across all licensees. OpenSolid has been tested on... dozens?
- **Migration risk** — if OpenSolid stalls (maintainer burnout), they've invested millions in a dead end.
- **No support SLA.** When their customer finds a bug at 2am before a product launch, they call Siemens. With open source, they file an issue and wait.
- **Regulatory compliance** — aerospace/medical customers need traceability.

**What they'd wish was different:**
- A commercial entity offering support contracts and SLAs
- Published validation results against standard test suites (NIST CAD benchmarks)
- A formal tolerance specification with mathematical guarantees
- Governance model that ensures long-term viability

---

### 8. 3D Printing / Additive Manufacturing User

**Who they are:** Runs a print farm (20 FDM + 2 SLA). Receives customer models (STL, STEP, 3MF), needs to validate, repair, orient, add supports, slice.

**What works well:**
- B-rep means exact geometry before tessellation (adaptive for different print resolutions)
- Rust performance for processing hundreds of models per day

**What does NOT work / frustrates them:**
- **STL import/repair is their daily reality.** 80% of customer files are STL. If OpenSolid can't import mesh or do mesh-to-B-rep, it's useless.
- **3MF support** — modern additive exchange format. STEP is for subtractive.
- **Lattice structures** — lightweight infill, gyroid patterns. Not traditional B-rep.
- **Slicing** — plane-body intersection producing 2D contours. Needs to be FAST (thousands of slices per model) and robust.
- **Model repair** — self-intersections, non-manifold edges, inverted faces. A kernel that rejects invalid input is useless; they need one that fixes it.

**What they'd wish was different:**
- First-class mesh support alongside B-rep
- 3MF import/export
- A "repair" mode that accepts broken geometry and produces valid output
- Fast slicing operation (plane intersection → 2D contours)

---

### 9. Simulation/FEA Engineer (Needs Mesh Quality)

**Who they are:** FEA analyst at a turbomachinery company. Gets STEP models of turbine blades, needs hex-dominant meshes for CFD.

**What works well:**
- Exact B-rep means precise tessellation control
- Deterministic tessellation means reproducible mesh studies

**What does NOT work / frustrates them:**
- **Defeaturing API** — fillets, logos, tiny holes destroy mesh quality. Need: remove fillets below R<1mm, fill holes below diameter 5mm.
- **Mid-surface extraction** — thin-walled structures should be shell elements, not solids.
- **Virtual topology** — merge small faces, split faces at arbitrary curves without modifying geometry.
- **Curvature-adaptive tessellation** — not uniform triangles. Fine in high-curvature, coarse on flats.
- **Quad meshing** — triangles are easy. Industry needs quads for structural FEA.

**What they'd wish was different:**
- A defeaturing module
- Curvature-aware tessellation with sizing function input
- Mid-surface extraction
- Virtual topology editing

---

### 10. Solo Developer Building a Hobby CAD App

**Who they are:** Weekend Rust programmer. Wants to build a CAD app for woodworking projects.

**What works well:**
- `cargo add opensolid` and they're running
- Good error messages help debugging
- Can read the implementation to learn

**What does NOT work / frustrates them:**
- **Documentation gap** between "hello cube" and real operations
- **No visualization crate** — need to SEE geometry
- **Error handling overwhelm** — every operation returns Result with 15 variants
- **No examples that look like real parts**
- **Compile times** — if 3 minutes to compile, edit-run-view cycle is killed

**What they'd wish was different:**
- A `opensolid-viewer` crate
- Tutorial-style docs building real objects
- Fast incremental compilation

---

### 11. Mesh-to-CAD Reconstruction Researcher

**Who they are:** Computer vision researcher. Has LiDAR point clouds, needs parametric B-rep for digital twins.

**What works well:**
- If the kernel can construct B-rep from detected primitives, that's their output format
- Rust performance for dense point clouds

**What does NOT work / frustrates them:**
- **No "construct face from point samples" operation**
- **No fitting operations** — `Surface::fit(points, degree, tolerance)` is fundamental to reverse engineering
- **Tolerance-aware construction** — input data has noise (±2mm), need soft constraints
- **No incremental construction** — need to add faces one at a time, not construct atomically
- **No regularization** — detected features should snap to standard values

**What they'd wish was different:**
- Surface fitting from point data
- Incremental body construction (add faces, heal gaps)
- Regularization / constraint-based snapping

---

### 12. Automated QC/Inspection Company

**Who they are:** Metrology startup. Scan parts, compare to CAD nominal, flag deviations. 1 part/second throughput.

**What works well:**
- Exact B-rep for point-to-surface distance (not point-to-mesh)
- Rust performance for real-time comparison

**What does NOT work / frustrates them:**
- **Point-to-surface closest distance must be FAST** — millions of queries/second, needs BVH
- **No PMI/GD&T integration** — AP242 metadata stripped on import
- **Batch performance** — STEP parsing alone can't exceed budget for 3600 parts/hour
- **Surface normal queries** — must be O(1) per query

**What they'd wish was different:**
- Spatial indexing for closest-point queries (BVH)
- GD&T/PMI preservation
- Sub-100ms STEP import for typical parts

---

### 13. Zoo/KittyCAD Competitor Analyst

**Who they are:** Product strategist at Zoo watching OpenSolid as potential threat/acquisition/contribution target.

**What concerns them:**
- **Governance risk** — bus-factor-1 project
- **Performance benchmarks** — can't compare without published numbers
- **Patent concerns** — B-rep algorithms are heavily patented
- **Feature timeline uncertainty**

**What they'd wish was different:**
- Clear governance model (foundation, funding)
- Published performance benchmarks
- Patent/IP analysis
- Modular architecture for using subsystems independently

---

### 14. Game Engine Developer Wanting CSG

**Who they are:** Graphics programmer building destructible environments. Needs real-time CSG.

**What does NOT work:**
- **Speed** — CAD Booleans are 1000x too slow for real-time gameplay (need < 100μs)
- **They don't need NURBS** — polyhedral Boolean mode that skips SSI entirely
- **Memory allocation** — game engines have frame budgets, can't tolerate system allocator
- **No LOD support** — need multiple tessellation levels
- **Thread safety** — must work in job systems

**What they'd wish was different:**
- A "fast-path" polyhedral CSG mode
- Custom allocator support
- Sub-millisecond performance target for simple operations
- All types Send+Sync

---

### 15. Robotics Engineer Doing Collision Detection

**Who they are:** Robotics software engineer. Needs collision-free path planning against B-rep obstacles.

**What does NOT work:**
- **Collision detection is a different performance regime** — need yes/no in < 50μs, not full Boolean
- **No swept volume operation**
- **No BVH maintained on bodies** for broad-phase
- **No continuous collision detection**
- **Real-time constraints** — 1kHz control loop, never block > 500μs

**What they'd wish was different:**
- "Collision query" module separate from modeling
- BVH built into bodies, updated on transform
- Minimum distance with gradient
- Worst-case time bounds on all operations

---

## Critical Revisions Needed

### 1. Python Bindings Are Non-Negotiable for Adoption
The largest potential user communities (ML researchers, CadQuery/Build123d users, hardware startups) all live in Python. A Rust-only kernel will be admired but not adopted.
**Action:** PyO3 bindings as first-class deliverable. Publish to PyPI. Numpy-compatible arrays.

### 2. Define Tolerance Philosophy as a Formal Specification
Every persona touching real-world data needs to understand tolerance propagation. Contributors can't implement correctly without it.
**Action:** Formal tolerance spec: global resolution, per-operation behavior, gap-healing thresholds, composition rules.

### 3. STEP Import Must Heal, Not Reject
Real STEP files are broken. A kernel that rejects them is unusable.
**Action:** Tolerance-aware import with healing. Diagnostic report. Accept 99%+ of files Parasolid accepts.

### 4. Ship Fillet/Chamfer That Handles Production Cases
11/15 personas need fillet. It's the #1 operation after Boolean.
**Action:** Prioritize fillet as #1 feature after basic Booleans. Handle degenerate cases. Publish 100+ test cases.

### 5. Provide Spatial Indexing and Fast Point/Distance Queries
Inspection, robotics, and simulation all need millions of closest-point queries/second.
**Action:** BVH built into Body type. Expose `closest_point()` and `min_distance()` as O(log n) operations.

### 6. Add a High-Level "Builder" API Above the B-Rep Layer
Most users want `box().fillet().shell().export()`, not half-edge topology manipulation.
**Action:** `opensolid-builder` layer with fluent chaining, defaults, geometric selectors.

### 7. Mesh/STL as First-Class Citizens
3D printing, game dev, and reconstruction all work primarily with meshes. Pure B-rep misses major markets.
**Action:** STL/OBJ import, mesh repair, mesh Boolean, mesh-to-B-rep for primitives.

### 8. Publish Performance Benchmarks and Worst-Case Bounds
Game devs need < 100μs, robotics < 500μs, inspection 1M queries/sec. Without benchmarks, can't evaluate.
**Action:** Published benchmarks. Document worst-case complexity. Consider bounded-time mode.

### 9. Persistent Face/Edge Naming Must Be Day-One Architecture
Every CAD app developer needs features to survive topology changes. Retrofitting is impossible.
**Action:** Persistent naming scheme from day one. Stable identifiers from construction history. Old→new mapping on topology change.

### 10. Establish Governance and Commercial Support Path
Enterprise users won't adopt without sustainable governance, support SLAs, and patent indemnification.
**Action:** Foundation or commercial entity. Governance model. IP analysis. Support tiers.
