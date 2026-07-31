//! Body validation: [`TopologyStore::check`] (`spec/11-testing.md` §4.1,
//! the OpenSolid answer to Parasolid's `PK_BODY_check`).
//!
//! Unlike the Euler-operator invariant checks in [`crate::euler`] — which
//! panic, because an operator corrupting the store is a kernel bug — `check`
//! is a diagnostic for topology of *unknown* provenance (imported, sewn,
//! hand-built, or suspected-corrupt bodies). It never panics, walks as much
//! of the body as it can reach, and reports every problem it finds as a
//! structured [`CheckFailure`], not a bool.
//!
//! Checks performed, in traversal order:
//!
//! - **Referential integrity / orphans**: every containment reference
//!   (body → shell → face → loop → fin → edge → vertex) resolves to a live
//!   entity; children point back at their parents; fins are registered on
//!   their edges and edges on their endpoint vertices; every entry in a
//!   vertex's edge list resolves and actually ends at that vertex; no empty
//!   shells, loop-less faces, or fin-less non-vertex loops. A
//!   [`BodyType::Solid`](crate::BodyType::Solid) body must own at least one
//!   shell — an "empty solid" is explicitly illegal, not vacuously valid.
//! - **Loop bookkeeping**: a face's outer loop is not flagged
//!   [`LoopType::Inner`](crate::LoopType::Inner), no `inner_loops` entry is
//!   flagged [`LoopType::Outer`](crate::LoopType::Outer), and no loop is
//!   listed twice on one face (a duplicate would double-count the `R` term
//!   of the Euler–Poincaré formula).
//! - **Loop connectivity**: `next`/`prev` links agree with each loop's fin
//!   order, and each fin's end vertex is the next fin's start vertex.
//! - **Closure and manifoldness**: every edge of a shell that must be
//!   closed — flagged `is_closed`, or any shell of a
//!   [`BodyType::Solid`](crate::BodyType::Solid) body — has exactly two
//!   fins; no edge anywhere has more than two. The producer-supplied
//!   `is_closed` flag is not trusted: a solid shell flagged open fails
//!   outright, and a flagged-open shell whose every edge is two-fin is
//!   reported as inconsistent regardless of body type. Shells of one body
//!   must also be disjoint: an edge or vertex used by two different shells
//!   is non-manifold between them and reported directly, not left for the
//!   Euler formula to notice.
//! - **Orientation consistency**: the two fins of a manifold edge traverse
//!   it in opposite directions. This is the topological form of "adjacent
//!   faces are consistently oriented"; [`FaceSense`](crate::FaceSense)
//!   relates face normals to *surface* normals and needs the geometry layer.
//! - **Tolerance sanity**: every edge/vertex tolerance is finite, at least
//!   [`SYSTEM_RESOLUTION`], and at most [`MAX_ALLOWED_TOLERANCE`]; vertex
//!   points are finite.
//! - **Euler–Poincaré formula** `V - E + F - R = 2(S - H)`: checked only
//!   when no *structural* failure was found *and* all shells are closed —
//!   the formula applies to closed surfaces, and on a structurally broken
//!   graph the counts are meaningless noise. Non-structural failures
//!   (tolerance sanity, non-finite points — see
//!   [`CheckFailure::is_structural`]) do **not** suppress it: a body with
//!   one loose tolerance *and* a wrong genus reports both. For solid bodies
//!   the bypass is never silent: a solid with a non-closed shell has
//!   already failed the closure checks above, so the formula is only ever
//!   skipped on a body that reports at least one other failure.
//!
//! # Geometric checks
//!
//! [`TopologyStore::check`] reads only the topology graph, because that is
//! all a [`TopologyStore`] holds: the curves, surfaces and trim curves live
//! in a separate [`GeometryStore`], and plenty of callers have a body whose
//! geometry slots are still empty. The geometric half of the spec's body
//! validation is therefore a second entry point,
//! [`TopologyStore::check_geometry`], which takes the geometry store as
//! well; [`TopologyStore::check_with_geometry`] runs both.
//!
//! What it checks (`spec/08-tolerances.md` §7.1, `spec/11-testing.md` §4.1):
//!
//! - **Edge on surface** (invariant 1): every point of an edge's curve lies
//!   within the edge's tolerance of the surface of every face that edge
//!   bounds. This is also the tolerance-coherence check for edges — a
//!   tolerance is a *claim* about how far the curve may stray, and the
//!   claim is measured, not believed.
//! - **Vertex on edge** (invariant 2): a vertex's point lies within the
//!   vertex's tolerance of the endpoint of each adjacent edge's curve, at
//!   the parameter (`t_start`/`t_end`) that endpoint is named by.
//! - **Pcurve fidelity**: a fin's 2D trim curve tracks its edge's 3D curve,
//!   `surface.point(pcurve(t)) == curve.point(t)` — the parameterization
//!   invariant [`crate::pcurve`] is built around, and the thing that goes
//!   wrong when a pcurve lands on the wrong branch of a periodic surface or
//!   is left attached across a repair that rewired its fin.
//! - **Face sense against loop winding**: a face's outer loop runs
//!   counterclockwise in its surface's `(u, v)` space exactly when the
//!   face's [`FaceSense`] is `Positive`, since the surface normal is
//!   `du × dv`. Read from the fins' pcurves, so it applies to bodies that
//!   carry trim geometry.
//! - **Edge parameter range**: `[t_start, t_end]` is finite and increasing,
//!   the precondition every consumer of an edge's curve assumes.
//!
//! # Self-intersection
//!
//! Face-face clash detection is a different shape of computation — a
//! pairwise search over faces rather than a walk down the containment tree
//! — so it is a third entry point,
//! [`TopologyStore::check_self_intersection`], which
//! [`TopologyStore::check_with_geometry`] also runs.
//!
//! It reports a [`CheckFailure::SelfIntersection`] for each pair of faces
//! that meet somewhere other than along topology they share. Faces are
//! pruned against conservative bounding boxes and then intersected through
//! [`crate::ssi`], and each sample of the resulting curves is classified
//! against both faces' trimmed regions; adjacency is excused by measuring
//! the contact against the shared edge or vertex that explains it, so two
//! faces that share an edge *and* cross elsewhere are still caught. See
//! that method for the sampling limits.

use crate::boolean::{
    Chart, CoverEmbedder, CoverPoint, FaceRegionPoly, clip_line_to_box, geometric_snap,
    is_bounded_marched,
};
use crate::curve::{Curve3, CurveEval};
use crate::euler::EulerCounts;
use crate::geometry::GeometryStore;
use crate::pcurve::{Curve2, Curve2Eval};
use crate::project::{CurveProject, SurfaceProject};
use crate::ssi::{self, SurfaceIntersection};
use crate::surface::{Surface3, SurfaceEval};
use crate::topology::{
    Body, BodyType, Edge, Face, FaceSense, Fin, FinSense, Loop, LoopType, SYSTEM_RESOLUTION, Shell,
    ShellOrientation, TopologyStore, Vertex,
};
use opensolid_core::EntityId;
use opensolid_core::tolerance::ToleranceContext;
use opensolid_core::types::{BoundingBox3, Point2, Point3, Vector2};
use thiserror::Error;

/// Maximum allowed tolerance on any entity, from the spec's default
/// `ToleranceConfig::max_allowed_tolerance` (`spec/08-tolerances.md` §3.3).
pub const MAX_ALLOWED_TOLERANCE: f64 = 0.01;

/// Untyped reference to a topological entity, for failure reporting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EntityRef {
    Body(EntityId<Body>),
    Shell(EntityId<Shell>),
    Face(EntityId<Face>),
    Loop(EntityId<Loop>),
    Fin(EntityId<Fin>),
    Edge(EntityId<Edge>),
    Vertex(EntityId<Vertex>),
}

/// One specific defect found by [`TopologyStore::check`].
#[derive(Debug, Clone, PartialEq, Error)]
pub enum CheckFailure {
    /// The body id itself does not resolve.
    #[error("body {0:?} is stale (not in the store)")]
    StaleBody(EntityId<Body>),

    /// A containment or link reference resolves to a removed entity.
    #[error("{from:?} references stale entity {to:?}")]
    StaleReference { from: EntityRef, to: EntityRef },

    /// A child entity's back-pointer names a different parent than the one
    /// listing it.
    #[error("{child:?} does not point back at its parent {expected_parent:?}")]
    BackPointerMismatch {
        child: EntityRef,
        expected_parent: EntityRef,
    },

    /// A shell with no faces (orphan container).
    #[error("shell {0:?} has no faces")]
    EmptyShell(EntityId<Shell>),

    /// A [`BodyType::Solid`](crate::BodyType::Solid) body with no shells.
    /// Every other check on such a body passes vacuously (and the Euler
    /// formula degenerates to `0 = 0`), so the emptiness itself must fail:
    /// a solid is a bounded volume and needs at least one bounding shell.
    #[error("solid body {0:?} has no shells")]
    SolidWithoutShells(EntityId<Body>),

    /// A face with no outer loop.
    #[error("face {0:?} has no outer loop")]
    FaceWithoutOuterLoop(EntityId<Face>),

    /// A loop with neither fins nor a degenerate-loop vertex.
    #[error("loop {0:?} has neither fins nor a vertex")]
    EmptyLoop(EntityId<Loop>),

    /// A loop with both fins and a degenerate-loop vertex.
    #[error("loop {0:?} has both fins and a degenerate-loop vertex")]
    VertexLoopWithFins(EntityId<Loop>),

    /// A face's outer loop flagged [`LoopType::Inner`](crate::LoopType::Inner).
    #[error("face {face:?} outer loop {loop_id:?} is flagged LoopType::Inner")]
    OuterLoopFlaggedInner {
        face: EntityId<Face>,
        loop_id: EntityId<Loop>,
    },

    /// An `inner_loops` entry flagged [`LoopType::Outer`](crate::LoopType::Outer).
    #[error("face {face:?} inner loop {loop_id:?} is flagged LoopType::Outer")]
    InnerLoopFlaggedOuter {
        face: EntityId<Face>,
        loop_id: EntityId<Loop>,
    },

    /// The same loop listed more than once on one face (as both outer and
    /// inner, or twice among the inner loops). Double-counts the `R` term
    /// of the Euler–Poincaré formula.
    #[error("face {face:?} lists loop {loop_id:?} more than once")]
    DuplicateLoopOnFace {
        face: EntityId<Face>,
        loop_id: EntityId<Loop>,
    },

    /// A fin whose `next`/`prev` links disagree with its loop's fin order.
    #[error("fin {fin:?} next/prev links disagree with loop {loop_id:?} order")]
    FinLinkBroken {
        loop_id: EntityId<Loop>,
        fin: EntityId<Fin>,
    },

    /// A fin whose end vertex is not the next fin's start vertex.
    #[error("loop {loop_id:?} is not vertex-continuous at fin {fin:?}")]
    LoopNotVertexContinuous {
        loop_id: EntityId<Loop>,
        fin: EntityId<Fin>,
    },

    /// A fin that is not in its edge's fin list.
    #[error("fin {fin:?} is not registered on its edge {edge:?}")]
    FinMissingFromEdge {
        fin: EntityId<Fin>,
        edge: EntityId<Edge>,
    },

    /// An edge whose fin list contains a fin that runs along a different edge.
    #[error("edge {edge:?} lists fin {fin:?} which does not use it")]
    ForeignFinOnEdge {
        edge: EntityId<Edge>,
        fin: EntityId<Fin>,
    },

    /// An edge that is not in its endpoint vertex's edge list.
    #[error("edge {edge:?} is not registered on its vertex {vertex:?}")]
    EdgeMissingFromVertex {
        edge: EntityId<Edge>,
        vertex: EntityId<Vertex>,
    },

    /// A vertex whose edge list contains an edge that does not end at it
    /// (mirror of [`ForeignFinOnEdge`](CheckFailure::ForeignFinOnEdge)).
    #[error("vertex {vertex:?} lists edge {edge:?} which does not end at it")]
    ForeignEdgeOnVertex {
        vertex: EntityId<Vertex>,
        edge: EntityId<Edge>,
    },

    /// A single-fin (boundary) edge inside a shell that must be closed —
    /// one flagged `is_closed`, or any shell of a
    /// [`BodyType::Solid`](crate::BodyType::Solid) body.
    #[error("shell {shell:?} must be closed but has open (single-fin) edge {edge:?}")]
    OpenEdgeInClosedShell {
        shell: EntityId<Shell>,
        edge: EntityId<Edge>,
    },

    /// A shell of a [`BodyType::Solid`](crate::BodyType::Solid) body flagged
    /// `is_closed = false`: solids must be bounded by closed shells,
    /// whatever the producer-supplied flag claims.
    #[error("solid body {body:?} has shell {shell:?} flagged open (is_closed = false)")]
    OpenShellInSolid {
        body: EntityId<Body>,
        shell: EntityId<Shell>,
    },

    /// A shell flagged `is_closed = false` whose every edge has two fins:
    /// the structure is closed, so the flag is inconsistent with it.
    #[error("shell {0:?} is flagged open but every edge has two fins (structurally closed)")]
    ShellFlaggedOpenButClosed(EntityId<Shell>),

    /// An edge with more than two fins.
    #[error("edge {edge:?} is non-manifold ({fins} fins)")]
    NonManifoldEdge { edge: EntityId<Edge>, fins: usize },

    /// An edge used by two different shells of the same body: the shells
    /// touch along it, which is non-manifold between shells.
    #[error("edge {edge:?} is shared by shells {shell_a:?} and {shell_b:?}")]
    EdgeSharedBetweenShells {
        edge: EntityId<Edge>,
        shell_a: EntityId<Shell>,
        shell_b: EntityId<Shell>,
    },

    /// A vertex used by two different shells of the same body: the shells
    /// touch at it, which is non-manifold between shells.
    #[error("vertex {vertex:?} is shared by shells {shell_a:?} and {shell_b:?}")]
    VertexSharedBetweenShells {
        vertex: EntityId<Vertex>,
        shell_a: EntityId<Shell>,
        shell_b: EntityId<Shell>,
    },

    /// A two-fin edge whose fins are not mated to each other.
    #[error("edge {edge:?}: its two fins are not mated to each other")]
    UnmatedFins { edge: EntityId<Edge> },

    /// A fin whose mate does not point back at it.
    #[error("fin {fin:?} mate link is not mutual (mate {mate:?})")]
    MateNotMutual {
        fin: EntityId<Fin>,
        mate: EntityId<Fin>,
    },

    /// A fin mated to a fin on a different edge.
    #[error("fin {fin:?} and its mate {mate:?} are on different edges")]
    MateOnDifferentEdge {
        fin: EntityId<Fin>,
        mate: EntityId<Fin>,
    },

    /// The two faces sharing an edge traverse it in the same direction:
    /// their orientations disagree.
    #[error(
        "faces {face_a:?} and {face_b:?} are inconsistently oriented across \
         edge {edge:?} (mated fins traverse it in the same direction)"
    )]
    InconsistentOrientation {
        edge: EntityId<Edge>,
        face_a: EntityId<Face>,
        face_b: EntityId<Face>,
    },

    /// `V - E + F - R = 2(S - H)` does not hold for the body.
    #[error("Euler-Poincaré formula violated for body {body:?}: {counts:?}")]
    EulerViolation {
        body: EntityId<Body>,
        counts: EulerCounts,
    },

    /// A tolerance that is NaN, infinite, negative, or below the system
    /// resolution floor.
    #[error("{entity:?} has invalid tolerance {tolerance}")]
    InvalidTolerance { entity: EntityRef, tolerance: f64 },

    /// A tolerance above [`MAX_ALLOWED_TOLERANCE`].
    #[error("{entity:?} tolerance {tolerance} exceeds limit {limit}")]
    ToleranceExceeded {
        entity: EntityRef,
        tolerance: f64,
        limit: f64,
    },

    /// A vertex whose point has a NaN or infinite coordinate.
    #[error("vertex {0:?} has a non-finite point")]
    NonFinitePoint(EntityId<Vertex>),

    /// An edge whose curve strays further from an adjacent face's surface
    /// than the edge's tolerance permits (`spec/08-tolerances.md` §7.1
    /// invariant 1). `allowed` is the edge's tolerance plus round-off slack.
    #[error(
        "edge {edge:?} leaves the surface of face {face:?} by {max_deviation} \
         (allowed {allowed})"
    )]
    EdgeOffSurface {
        edge: EntityId<Edge>,
        face: EntityId<Face>,
        max_deviation: f64,
        allowed: f64,
    },

    /// A vertex whose point is further from the endpoint of an adjacent
    /// edge's curve than the vertex's tolerance permits
    /// (`spec/08-tolerances.md` §7.1 invariant 2).
    #[error(
        "vertex {vertex:?} is {deviation} from the endpoint of edge {edge:?} \
         (allowed {allowed})"
    )]
    VertexOffEdge {
        vertex: EntityId<Vertex>,
        edge: EntityId<Edge>,
        deviation: f64,
        allowed: f64,
    },

    /// A fin whose pcurve does not track its edge's 3D curve: evaluating the
    /// face's surface at the pcurve lands somewhere other than the curve
    /// does at the same parameter (see [`crate::pcurve`]).
    #[error(
        "fin {fin:?} pcurve departs from edge {edge:?} by {max_deviation} \
         (allowed {allowed})"
    )]
    PcurveDeviation {
        fin: EntityId<Fin>,
        edge: EntityId<Edge>,
        max_deviation: f64,
        allowed: f64,
    },

    /// A face whose outer loop winds the wrong way in its surface's `(u, v)`
    /// space for the face's [`FaceSense`]: the loop encloses its region with
    /// the surface normal pointing into the material, not out of it.
    #[error(
        "face {face:?} is flagged {sense:?} but its outer loop winds the other \
         way in parameter space (twice-signed-area {twice_signed_area})"
    )]
    FaceSenseContradictsLoop {
        face: EntityId<Face>,
        sense: FaceSense,
        twice_signed_area: f64,
    },

    /// An edge whose curve parameter range is not a finite increasing
    /// interval, which every consumer of the edge's geometry assumes.
    #[error("edge {edge:?} has parameter range [{t_start}, {t_end}]")]
    InvalidEdgeRange {
        edge: EntityId<Edge>,
        t_start: f64,
        t_end: f64,
    },

    /// Two faces of one body whose trimmed surfaces meet somewhere other
    /// than along the topology they share — the body passes through itself
    /// (`spec/11-testing.md` §4.1 `check_no_self_intersection`). `at` is one
    /// point of the clash, for locating it; a pair is reported once however
    /// large the overlap.
    #[error("faces {face_a:?} and {face_b:?} intersect at {at:?}")]
    SelfIntersection {
        face_a: EntityId<Face>,
        face_b: EntityId<Face>,
        at: Point3,
    },
}

impl CheckFailure {
    /// Whether this failure means the topology *graph* itself is suspect.
    ///
    /// Structural failures make the Euler–Poincaré counts meaningless, so
    /// [`TopologyStore::check`] skips the formula when any is present.
    /// Non-structural failures (bad tolerances, non-finite points, and every
    /// geometric failure from [`TopologyStore::check_geometry`]) leave the
    /// connectivity intact and do not suppress it — a body can have a
    /// perfectly sound graph and geometry that misses it.
    pub fn is_structural(&self) -> bool {
        !matches!(
            self,
            CheckFailure::InvalidTolerance { .. }
                | CheckFailure::ToleranceExceeded { .. }
                | CheckFailure::NonFinitePoint(_)
                | CheckFailure::EdgeOffSurface { .. }
                | CheckFailure::VertexOffEdge { .. }
                | CheckFailure::PcurveDeviation { .. }
                | CheckFailure::FaceSenseContradictsLoop { .. }
                | CheckFailure::InvalidEdgeRange { .. }
                | CheckFailure::SelfIntersection { .. }
        )
    }
}

/// Push `id` if not already present (order-preserving dedup; entity counts
/// per body are small enough that linear scans match the rest of the crate).
fn push_unique<T>(list: &mut Vec<EntityId<T>>, id: EntityId<T>) {
    if !list.contains(&id) {
        list.push(id);
    }
}

impl TopologyStore {
    /// Validate `body`, returning every defect found (empty means valid).
    ///
    /// Safe to call on arbitrarily corrupted topology: stale references are
    /// reported as failures and the affected sub-checks are skipped rather
    /// than panicking. See the [module docs](self) for the full list of
    /// checks and the conditions under which the Euler–Poincaré formula is
    /// evaluated.
    pub fn check(&self, body: EntityId<Body>) -> Vec<CheckFailure> {
        let mut failures = Vec::new();
        let Some(b) = self.bodies.get(body) else {
            return vec![CheckFailure::StaleBody(body)];
        };

        if b.body_type == BodyType::Solid && b.shells.is_empty() {
            failures.push(CheckFailure::SolidWithoutShells(body));
        }

        // Entities reachable from the body, deduplicated, for the
        // edge/vertex-level passes below.
        let mut edges: Vec<EntityId<Edge>> = Vec::new();
        let mut vertices: Vec<EntityId<Vertex>> = Vec::new();
        // First shell seen using each edge/vertex, to catch a second shell
        // touching it (non-manifold between shells).
        let mut edge_owner: Vec<(EntityId<Edge>, EntityId<Shell>)> = Vec::new();
        let mut vertex_owner: Vec<(EntityId<Vertex>, EntityId<Shell>)> = Vec::new();

        for &shell_id in &b.shells {
            let Some(shell) = self.shells.get(shell_id) else {
                failures.push(CheckFailure::StaleReference {
                    from: EntityRef::Body(body),
                    to: EntityRef::Shell(shell_id),
                });
                continue;
            };
            if shell.body != body {
                failures.push(CheckFailure::BackPointerMismatch {
                    child: EntityRef::Shell(shell_id),
                    expected_parent: EntityRef::Body(body),
                });
            }
            if shell.faces.is_empty() {
                failures.push(CheckFailure::EmptyShell(shell_id));
            }

            let mut shell_edges: Vec<EntityId<Edge>> = Vec::new();
            let mut shell_vertices: Vec<EntityId<Vertex>> = Vec::new();
            for &face_id in &shell.faces {
                self.check_face(
                    shell_id,
                    face_id,
                    &mut failures,
                    &mut shell_edges,
                    &mut shell_vertices,
                );
            }

            // Closure. The is_closed flag is producer-supplied and not
            // trusted: solids must be bounded by closed shells whatever the
            // flag says, and the flag is cross-checked against the structure
            // in both directions.
            if b.body_type == BodyType::Solid && !shell.is_closed {
                failures.push(CheckFailure::OpenShellInSolid {
                    body,
                    shell: shell_id,
                });
            }
            if shell.is_closed || b.body_type == BodyType::Solid {
                for &edge_id in &shell_edges {
                    if let Some(edge) = self.edges.get(edge_id) {
                        if edge.fins.len() == 1 {
                            failures.push(CheckFailure::OpenEdgeInClosedShell {
                                shell: shell_id,
                                edge: edge_id,
                            });
                        }
                    }
                }
            }
            if !shell.is_closed
                && !shell_edges.is_empty()
                && shell_edges
                    .iter()
                    .all(|&e| self.edges.get(e).is_some_and(|edge| edge.fins.len() == 2))
            {
                failures.push(CheckFailure::ShellFlaggedOpenButClosed(shell_id));
            }
            // Shells of one body must be disjoint: a second shell using an
            // edge or vertex already claimed by another shell touches it,
            // which is non-manifold between shells.
            for &edge_id in &shell_edges {
                if let Some(edge) = self.edges.get(edge_id) {
                    push_unique(&mut shell_vertices, edge.start_vertex);
                    push_unique(&mut shell_vertices, edge.end_vertex);
                }
                match edge_owner.iter().find(|&&(e, _)| e == edge_id) {
                    Some(&(_, owner)) => failures.push(CheckFailure::EdgeSharedBetweenShells {
                        edge: edge_id,
                        shell_a: owner,
                        shell_b: shell_id,
                    }),
                    None => edge_owner.push((edge_id, shell_id)),
                }
            }
            for &vertex_id in &shell_vertices {
                match vertex_owner.iter().find(|&&(v, _)| v == vertex_id) {
                    Some(&(_, owner)) => failures.push(CheckFailure::VertexSharedBetweenShells {
                        vertex: vertex_id,
                        shell_a: owner,
                        shell_b: shell_id,
                    }),
                    None => vertex_owner.push((vertex_id, shell_id)),
                }
            }

            for edge_id in shell_edges {
                push_unique(&mut edges, edge_id);
            }
            for vertex_id in shell_vertices {
                push_unique(&mut vertices, vertex_id);
            }
        }

        for &edge_id in &edges {
            self.check_edge(edge_id, &mut failures, &mut vertices);
        }
        for &vertex_id in &vertices {
            if let Some(vertex) = self.vertices.get(vertex_id) {
                if !vertex.point.coords.iter().all(|c| c.is_finite()) {
                    failures.push(CheckFailure::NonFinitePoint(vertex_id));
                }
                check_tolerance(
                    &mut failures,
                    EntityRef::Vertex(vertex_id),
                    vertex.tolerance,
                );
                // Mirror of EdgeMissingFromVertex: every entry in the
                // vertex's edge list must resolve and end at the vertex.
                for &edge_id in &vertex.edges {
                    match self.edges.get(edge_id) {
                        None => failures.push(CheckFailure::StaleReference {
                            from: EntityRef::Vertex(vertex_id),
                            to: EntityRef::Edge(edge_id),
                        }),
                        Some(edge)
                            if edge.start_vertex != vertex_id && edge.end_vertex != vertex_id =>
                        {
                            failures.push(CheckFailure::ForeignEdgeOnVertex {
                                vertex: vertex_id,
                                edge: edge_id,
                            });
                        }
                        Some(_) => {}
                    }
                }
            }
        }

        // The Euler–Poincaré formula only applies to closed surfaces, and
        // on a broken graph the counts are meaningless — so it is checked
        // last, only when no structural failure was found and all shells
        // are closed. Non-structural failures (tolerances, non-finite
        // points) leave the graph intact and must not mask a genus/closure
        // defect. Skipping it is never silent for solids: a solid with a
        // non-closed shell already failed the closure checks above.
        let all_closed = b
            .shells
            .iter()
            .all(|&s| self.shells.get(s).is_some_and(|shell| shell.is_closed));
        if !failures.iter().any(CheckFailure::is_structural) && all_closed {
            let counts = self.euler_counts(body);
            if !counts.euler_poincare_holds() {
                failures.push(CheckFailure::EulerViolation { body, counts });
            }
        }

        failures
    }

    /// Face-level checks: back-pointers, loop presence and bookkeeping
    /// (loop-type flags, duplicates), loop connectivity, fin links and
    /// mates. Collects reachable edges and degenerate-loop vertices.
    fn check_face(
        &self,
        shell_id: EntityId<Shell>,
        face_id: EntityId<Face>,
        failures: &mut Vec<CheckFailure>,
        shell_edges: &mut Vec<EntityId<Edge>>,
        vertices: &mut Vec<EntityId<Vertex>>,
    ) {
        let Some(face) = self.faces.get(face_id) else {
            failures.push(CheckFailure::StaleReference {
                from: EntityRef::Shell(shell_id),
                to: EntityRef::Face(face_id),
            });
            return;
        };
        if face.shell != shell_id {
            failures.push(CheckFailure::BackPointerMismatch {
                child: EntityRef::Face(face_id),
                expected_parent: EntityRef::Shell(shell_id),
            });
        }
        if face.outer_loop.is_none() {
            failures.push(CheckFailure::FaceWithoutOuterLoop(face_id));
        }

        let mut seen_loops: Vec<EntityId<Loop>> = Vec::new();
        for (is_outer, loop_id) in face
            .outer_loop
            .into_iter()
            .map(|l| (true, l))
            .chain(face.inner_loops.iter().map(|&l| (false, l)))
        {
            // A loop listed twice would double-count the Euler R term (and
            // double-report every defect inside it), so it is reported once
            // and not walked again.
            if seen_loops.contains(&loop_id) {
                failures.push(CheckFailure::DuplicateLoopOnFace {
                    face: face_id,
                    loop_id,
                });
                continue;
            }
            seen_loops.push(loop_id);
            let Some(lp) = self.loops.get(loop_id) else {
                failures.push(CheckFailure::StaleReference {
                    from: EntityRef::Face(face_id),
                    to: EntityRef::Loop(loop_id),
                });
                continue;
            };
            if is_outer && lp.loop_type == LoopType::Inner {
                failures.push(CheckFailure::OuterLoopFlaggedInner {
                    face: face_id,
                    loop_id,
                });
            }
            if !is_outer && lp.loop_type == LoopType::Outer {
                failures.push(CheckFailure::InnerLoopFlaggedOuter {
                    face: face_id,
                    loop_id,
                });
            }
            if lp.face != face_id {
                failures.push(CheckFailure::BackPointerMismatch {
                    child: EntityRef::Loop(loop_id),
                    expected_parent: EntityRef::Face(face_id),
                });
            }
            match (lp.fins.is_empty(), lp.vertex) {
                (true, None) => failures.push(CheckFailure::EmptyLoop(loop_id)),
                (false, Some(_)) => failures.push(CheckFailure::VertexLoopWithFins(loop_id)),
                (true, Some(v)) => {
                    if self.vertices.get(v).is_some() {
                        push_unique(vertices, v);
                    } else {
                        failures.push(CheckFailure::StaleReference {
                            from: EntityRef::Loop(loop_id),
                            to: EntityRef::Vertex(v),
                        });
                    }
                }
                (false, None) => {}
            }

            let n = lp.fins.len();
            for (i, &fin_id) in lp.fins.iter().enumerate() {
                let Some(fin) = self.fins.get(fin_id) else {
                    failures.push(CheckFailure::StaleReference {
                        from: EntityRef::Loop(loop_id),
                        to: EntityRef::Fin(fin_id),
                    });
                    continue;
                };
                if fin.loop_ref != loop_id {
                    failures.push(CheckFailure::BackPointerMismatch {
                        child: EntityRef::Fin(fin_id),
                        expected_parent: EntityRef::Loop(loop_id),
                    });
                }
                if fin.next != Some(lp.fins[(i + 1) % n])
                    || fin.prev != Some(lp.fins[(i + n - 1) % n])
                {
                    failures.push(CheckFailure::FinLinkBroken {
                        loop_id,
                        fin: fin_id,
                    });
                }
                // Vertex continuity: skipped when either endpoint is
                // unresolvable (the stale reference is reported instead).
                if let (Some(end), Some(start)) = (
                    self.fin_vertex_defensive(fin_id, false),
                    self.fin_vertex_defensive(lp.fins[(i + 1) % n], true),
                ) {
                    if end != start {
                        failures.push(CheckFailure::LoopNotVertexContinuous {
                            loop_id,
                            fin: fin_id,
                        });
                    }
                }
                if let Some(mate_id) = fin.mate {
                    match self.fins.get(mate_id) {
                        None => failures.push(CheckFailure::StaleReference {
                            from: EntityRef::Fin(fin_id),
                            to: EntityRef::Fin(mate_id),
                        }),
                        Some(mate) => {
                            if mate.mate != Some(fin_id) {
                                failures.push(CheckFailure::MateNotMutual {
                                    fin: fin_id,
                                    mate: mate_id,
                                });
                            }
                            if mate.edge != fin.edge {
                                failures.push(CheckFailure::MateOnDifferentEdge {
                                    fin: fin_id,
                                    mate: mate_id,
                                });
                            }
                        }
                    }
                }
                match self.edges.get(fin.edge) {
                    None => failures.push(CheckFailure::StaleReference {
                        from: EntityRef::Fin(fin_id),
                        to: EntityRef::Edge(fin.edge),
                    }),
                    Some(edge) => {
                        if !edge.fins.contains(&fin_id) {
                            failures.push(CheckFailure::FinMissingFromEdge {
                                fin: fin_id,
                                edge: fin.edge,
                            });
                        }
                        push_unique(shell_edges, fin.edge);
                    }
                }
            }
        }
    }

    /// Edge-level checks: manifoldness, mate pairing, orientation across the
    /// edge, fin-list and vertex-list registration, tolerance sanity.
    fn check_edge(
        &self,
        edge_id: EntityId<Edge>,
        failures: &mut Vec<CheckFailure>,
        vertices: &mut Vec<EntityId<Vertex>>,
    ) {
        // Reachable edges were resolved during the face pass.
        let Some(edge) = self.edges.get(edge_id) else {
            return;
        };

        match edge.fins.len() {
            // 1-fin edges are legal on open shells; closed-shell closure is
            // checked per shell in `check`.
            0 | 1 => {}
            2 => {
                let (a_id, b_id) = (edge.fins[0], edge.fins[1]);
                if let (Some(a), Some(b)) = (self.fins.get(a_id), self.fins.get(b_id)) {
                    if a.mate != Some(b_id) || b.mate != Some(a_id) {
                        failures.push(CheckFailure::UnmatedFins { edge: edge_id });
                    }
                    // Opposite traversal directions = consistent orientation
                    // of the two adjacent faces.
                    if a.sense == b.sense {
                        if let (Some(face_a), Some(face_b)) =
                            (self.fin_face_defensive(a_id), self.fin_face_defensive(b_id))
                        {
                            failures.push(CheckFailure::InconsistentOrientation {
                                edge: edge_id,
                                face_a,
                                face_b,
                            });
                        }
                    }
                }
            }
            n => failures.push(CheckFailure::NonManifoldEdge {
                edge: edge_id,
                fins: n,
            }),
        }

        for &fin_id in &edge.fins {
            match self.fins.get(fin_id) {
                None => failures.push(CheckFailure::StaleReference {
                    from: EntityRef::Edge(edge_id),
                    to: EntityRef::Fin(fin_id),
                }),
                Some(fin) if fin.edge != edge_id => {
                    failures.push(CheckFailure::ForeignFinOnEdge {
                        edge: edge_id,
                        fin: fin_id,
                    });
                }
                Some(_) => {}
            }
        }

        for vertex_id in [edge.start_vertex, edge.end_vertex] {
            match self.vertices.get(vertex_id) {
                None => failures.push(CheckFailure::StaleReference {
                    from: EntityRef::Edge(edge_id),
                    to: EntityRef::Vertex(vertex_id),
                }),
                Some(vertex) => {
                    if !vertex.edges.contains(&edge_id) {
                        failures.push(CheckFailure::EdgeMissingFromVertex {
                            edge: edge_id,
                            vertex: vertex_id,
                        });
                    }
                    push_unique(vertices, vertex_id);
                }
            }
        }

        check_tolerance(failures, EntityRef::Edge(edge_id), edge.tolerance);
    }

    /// Start (`want_start`) or end vertex of a fin, or `None` if the fin or
    /// its edge is stale (non-panicking counterpart of
    /// [`TopologyStore::fin_start_vertex`] / [`fin_end_vertex`](TopologyStore::fin_end_vertex)).
    fn fin_vertex_defensive(
        &self,
        fin_id: EntityId<Fin>,
        want_start: bool,
    ) -> Option<EntityId<Vertex>> {
        let fin = self.fins.get(fin_id)?;
        let edge = self.edges.get(fin.edge)?;
        Some(if (fin.sense == FinSense::Forward) == want_start {
            edge.start_vertex
        } else {
            edge.end_vertex
        })
    }

    /// Face a fin bounds, or `None` if any link on the way is stale.
    fn fin_face_defensive(&self, fin_id: EntityId<Fin>) -> Option<EntityId<Face>> {
        let fin = self.fins.get(fin_id)?;
        Some(self.loops.get(fin.loop_ref)?.face)
    }

    // ------------------------------------------------------------------
    // Geometric validation
    // ------------------------------------------------------------------

    /// Validate `body`'s *geometry* against its topology, returning every
    /// defect found (empty means valid). See the [module docs](self) for the
    /// list of checks.
    ///
    /// The geometric counterpart of [`TopologyStore::check`], which reads
    /// only the topology graph. The two are separate entry points because
    /// geometry is optional: [`Edge::curve`], [`Face::surface`] and
    /// [`Fin::pcurve`] are all `Option`, and an entity whose slot is empty
    /// is simply not measured here — a body under construction is not
    /// thereby invalid. [`TopologyStore::check_with_geometry`] runs both.
    ///
    /// Like `check`, this never panics: it walks as much of the body as it
    /// can reach and skips (rather than reports) sub-checks blocked by a
    /// stale reference, which `check` reports.
    pub fn check_geometry(&self, geo: &GeometryStore, body: EntityId<Body>) -> Vec<CheckFailure> {
        let mut failures = Vec::new();
        let Some(b) = self.bodies.get(body) else {
            return vec![CheckFailure::StaleBody(body)];
        };

        let mut edges: Vec<EntityId<Edge>> = Vec::new();
        // (edge, face) pairs already measured. A seam edge is used twice by
        // the one face, against the one surface, and must not be reported
        // twice for it.
        let mut measured: Vec<(EntityId<Edge>, EntityId<Face>)> = Vec::new();

        for &shell_id in &b.shells {
            let Some(shell) = self.shells.get(shell_id) else {
                continue;
            };
            for &face_id in &shell.faces {
                let Some(face) = self.faces.get(face_id) else {
                    continue;
                };
                let surface = face.surface.and_then(|id| geo.surface(id));
                for loop_id in face
                    .outer_loop
                    .into_iter()
                    .chain(face.inner_loops.iter().copied())
                {
                    let Some(lp) = self.loops.get(loop_id) else {
                        continue;
                    };
                    for &fin_id in &lp.fins {
                        let Some(fin) = self.fins.get(fin_id) else {
                            continue;
                        };
                        push_unique(&mut edges, fin.edge);
                        let Some(edge) = self.edges.get(fin.edge) else {
                            continue;
                        };
                        let Some(curve) = edge.curve.and_then(|id| geo.curve(id)) else {
                            continue;
                        };
                        // A broken range makes every sample below meaningless;
                        // it is reported once in the per-edge pass.
                        if !edge_range_is_sane(edge) {
                            continue;
                        }
                        let Some(surface) = surface else {
                            continue;
                        };

                        if !measured.contains(&(fin.edge, face_id)) {
                            measured.push((fin.edge, face_id));
                            let deviation =
                                curve_surface_deviation(surface, curve, edge.t_start, edge.t_end);
                            if let Some((max_deviation, allowed)) =
                                deviation.exceeding(edge.tolerance)
                            {
                                failures.push(CheckFailure::EdgeOffSurface {
                                    edge: fin.edge,
                                    face: face_id,
                                    max_deviation,
                                    allowed,
                                });
                            }
                        }

                        if let Some(pcurve) = fin.pcurve.and_then(|id| geo.pcurve(id)) {
                            let deviation =
                                pcurve_deviation(surface, curve, pcurve, edge.t_start, edge.t_end);
                            if let Some((max_deviation, allowed)) =
                                deviation.exceeding(edge.tolerance)
                            {
                                failures.push(CheckFailure::PcurveDeviation {
                                    fin: fin_id,
                                    edge: fin.edge,
                                    max_deviation,
                                    allowed,
                                });
                            }
                        }
                    }
                }

                // The winding is only meaningful in a surface's parameter
                // space, so a face without one has nothing to read it from.
                if let (Some(surface), Some(outer)) = (surface, face.outer_loop) {
                    if let Some(twice_signed_area) = self.loop_winding(geo, surface, outer) {
                        let wound_ccw = twice_signed_area > 0.0;
                        if wound_ccw != (face.sense == FaceSense::Positive) {
                            failures.push(CheckFailure::FaceSenseContradictsLoop {
                                face: face_id,
                                sense: face.sense,
                                twice_signed_area,
                            });
                        }
                    }
                }
            }
        }

        for &edge_id in &edges {
            let Some(edge) = self.edges.get(edge_id) else {
                continue;
            };
            if !edge_range_is_sane(edge) {
                failures.push(CheckFailure::InvalidEdgeRange {
                    edge: edge_id,
                    t_start: edge.t_start,
                    t_end: edge.t_end,
                });
                continue;
            }
            let Some(curve) = edge.curve.and_then(|id| geo.curve(id)) else {
                continue;
            };
            for (vertex_id, t) in [
                (edge.start_vertex, edge.t_start),
                (edge.end_vertex, edge.t_end),
            ] {
                let Some(vertex) = self.vertices.get(vertex_id) else {
                    continue;
                };
                let end = curve.point(t);
                if !is_finite(&end) || !is_finite(&vertex.point) {
                    continue;
                }
                let mut deviation = Deviation::new();
                deviation.record((vertex.point - end).norm(), &end);
                if let Some((gap, allowed)) = deviation.exceeding(vertex.tolerance) {
                    failures.push(CheckFailure::VertexOffEdge {
                        vertex: vertex_id,
                        edge: edge_id,
                        deviation: gap,
                        allowed,
                    });
                }
            }
        }

        failures
    }

    /// Validate `body` topologically *and* geometrically: the concatenation
    /// of [`TopologyStore::check`], [`TopologyStore::check_geometry`] and
    /// [`TopologyStore::check_self_intersection`], topology first.
    ///
    /// The passes are independent — a geometric defect never suppresses a
    /// topological one or vice versa — so the combined list is exactly what
    /// running all three gives.
    pub fn check_with_geometry(
        &self,
        geo: &GeometryStore,
        body: EntityId<Body>,
    ) -> Vec<CheckFailure> {
        let mut failures = self.check(body);
        // A stale body is reported once, by `check`; re-reporting it from
        // the geometric passes would say the same thing twice.
        if failures == [CheckFailure::StaleBody(body)] {
            return failures;
        }
        failures.extend(self.check_geometry(geo, body));
        failures.extend(self.check_self_intersection(geo, body));
        failures
    }

    /// Find the face pairs of `body` that actually meet in space without
    /// sharing the topology that would make meeting legal — the body
    /// passing through itself (`spec/11-testing.md` §4.1
    /// `check_no_self_intersection`).
    ///
    /// A third entry point rather than part of [`TopologyStore::check_geometry`]
    /// because it is a different *shape* of computation: the other geometric
    /// checks walk the body once and measure each entity against its own
    /// neighbours, while this one is a pairwise search over faces, and is
    /// priced accordingly. [`TopologyStore::check_with_geometry`] runs it.
    ///
    /// # What counts as a clash
    ///
    /// In a valid body two faces may touch only along topology they *share*:
    /// a common edge or a common vertex. Every other point that lies on both
    /// faces' trimmed regions is a defect — interiors crossing, an edge
    /// stabbing through a face, or two faces laid over each other. So a
    /// point is reported when it lies on both faces (inside the trim, or
    /// within tolerance of its boundary) and is further than that same
    /// tolerance band from every edge and vertex the two faces have in
    /// common. Adjacency is thereby excused by *measurement* against the
    /// shared entity's own geometry rather than by skipping adjacent pairs
    /// outright, so two faces that share an edge and *also* cross elsewhere
    /// are still caught.
    ///
    /// # Method and its limits
    ///
    /// Broad phase: a conservative bounding box per face — its rim samples
    /// plus a parameter-space grid over the interior, dilated by the largest
    /// sagitta that grid leaves unresolved, so a curved face's bulge is
    /// inside its own box. Pairs whose boxes are disjoint are dropped
    /// without touching geometry.
    ///
    /// Narrow phase: the surfaces' intersection from [`crate::ssi`] — exact
    /// where a closed form exists, marched otherwise — sampled along its
    /// curves and each sample classified against both faces' trims. This is
    /// a *sampled* search, so it is sound (everything reported really is a
    /// clash, up to the tolerance band above) but not complete: an overlap
    /// far narrower than the sample spacing can slip between samples, and a
    /// surface pair whose intersection the SSI layer cannot compute at all
    /// is skipped rather than guessed at. A face whose geometry is missing,
    /// whose parameter chart cannot be built, or whose loops do not embed is
    /// likewise skipped — the same "measure what is there" policy
    /// [`TopologyStore::check_geometry`] follows.
    ///
    /// Faces are paired across the whole body, not within each shell: an
    /// inner void shell touching its outer shell is as invalid as a shell
    /// crossing itself.
    pub fn check_self_intersection(
        &self,
        geo: &GeometryStore,
        body: EntityId<Body>,
    ) -> Vec<CheckFailure> {
        let Some(b) = self.bodies.get(body) else {
            return vec![CheckFailure::StaleBody(body)];
        };
        // The checks take no tolerance argument, and the entity tolerances
        // they do read are per-edge claims about geometry, not the modelling
        // context. The default context is what the rest of the crate builds
        // bodies with (`crate::tessellate`, `crate::boolean` callers).
        let tol = ToleranceContext::default();

        let mut patches: Vec<FacePatch> = Vec::new();
        for &shell_id in &b.shells {
            let Some(shell) = self.shells.get(shell_id) else {
                continue;
            };
            for &face_id in &shell.faces {
                let Some(face) = self.faces.get(face_id) else {
                    continue;
                };
                // The cover embedder needs the loops' winding in the chart,
                // which is CCW exactly when the face's outward side follows
                // the surface normal (as `crate::boolean` derives it).
                let outward_along_normal = (face.sense == FaceSense::Positive)
                    == (shell.orientation == ShellOrientation::Outward);
                if let Some(patch) = self.face_patch(geo, &tol, face_id, outward_along_normal) {
                    patches.push(patch);
                }
            }
        }

        let mut failures = Vec::new();
        for (i, a) in patches.iter().enumerate() {
            for b in &patches[i + 1..] {
                let overlap = a.bbox.intersection(&b.bbox);
                if overlap.is_empty() {
                    continue;
                }
                if let Some(at) = clash_point(a, b, &overlap, &tol) {
                    failures.push(CheckFailure::SelfIntersection {
                        face_a: a.face,
                        face_b: b.face,
                        at,
                    });
                }
            }
        }
        failures
    }

    /// The self-intersection pass's working view of one face: its trimmed
    /// region in parameter space, a conservative box around it in space, and
    /// the boundary entities that excuse a contact.
    ///
    /// `None` for a face this check cannot measure — missing geometry, a
    /// surface with no invertible chart, a broken edge range, or loops that
    /// do not embed in the chart's cover. Skipping is deliberate: every one
    /// of those is either reported by another check or is a face the kernel
    /// declines to model, and a guessed region would classify clashes
    /// against a boundary that is not the face's.
    fn face_patch(
        &self,
        geo: &GeometryStore,
        tol: &ToleranceContext,
        face_id: EntityId<Face>,
        outward_along_normal: bool,
    ) -> Option<FacePatch> {
        let face = self.faces.get(face_id)?;
        let surface = geo.surface(face.surface?)?.clone();
        let chart = Chart::build(&surface, tol).ok()?;

        let mut edges: Vec<BoundaryEdge> = Vec::new();
        let mut vertices: Vec<(EntityId<Vertex>, Point3)> = Vec::new();
        let mut tolerance = SYSTEM_RESOLUTION;
        let mut claim = |t: f64| {
            if t.is_finite() && t > tolerance {
                tolerance = t.min(MAX_ALLOWED_TOLERANCE);
            }
        };

        let mut loops: Vec<Vec<CoverPoint>> = Vec::new();
        for loop_id in face
            .outer_loop
            .into_iter()
            .chain(face.inner_loops.iter().copied())
        {
            let lp = self.loops.get(loop_id)?;
            let mut walk: Vec<Point3> = Vec::new();
            for &fin_id in &lp.fins {
                let fin = self.fins.get(fin_id)?;
                let edge = self.edges.get(fin.edge)?;
                if !edge_range_is_sane(edge) {
                    return None;
                }
                let curve = edge.curve.and_then(|id| geo.curve(id))?;
                if !edges.iter().any(|e| e.id == fin.edge) {
                    claim(edge.tolerance);
                    edges.push(BoundaryEdge {
                        id: fin.edge,
                        curve: curve.clone(),
                        t_start: edge.t_start,
                        t_end: edge.t_end,
                    });
                }
                for vertex_id in [edge.start_vertex, edge.end_vertex] {
                    let Some(vertex) = self.vertices.get(vertex_id) else {
                        continue;
                    };
                    if !vertices.iter().any(|&(id, _)| id == vertex_id) {
                        claim(vertex.tolerance);
                        vertices.push((vertex_id, vertex.point));
                    }
                }
                append_fin_samples(
                    &mut walk,
                    curve,
                    edge.t_start,
                    edge.t_end,
                    fin.sense == FinSense::Forward,
                );
            }
            if walk.len() < 3 {
                return None;
            }
            let mut embedder = CoverEmbedder::new(&chart, outward_along_normal);
            let mut cover: Vec<CoverPoint> = Vec::with_capacity(walk.len());
            for p in walk {
                if !is_finite(&p) {
                    return None;
                }
                embedder.push(p, &mut cover).ok()?;
            }
            if cover.len() < 3 {
                return None;
            }
            loops.push(cover);
        }
        if loops.is_empty() {
            return None;
        }

        let snap = geometric_snap(loops.iter().flatten().map(|&(_, p)| p));
        let poly = FaceRegionPoly { chart, loops };
        let (uv_min, uv_max) = uv_bounds(&poly)?;
        let bbox = face_bbox(&surface, &poly, uv_min, uv_max)?;
        Some(FacePatch {
            face: face_id,
            surface,
            poly,
            bbox,
            uv_min,
            uv_max,
            snap,
            edges,
            vertices,
            tolerance,
        })
    }

    /// Twice the signed area a loop encloses in its face's parameter space,
    /// walked in fin order — positive counterclockwise, which is the sense
    /// that puts the surface normal (`du × dv`) out of the enclosed region.
    ///
    /// `None` whenever the winding is not readable, in which case nothing is
    /// concluded from it:
    ///
    /// - a fin without a pcurve. Trim geometry is the only place a loop's
    ///   parameter-space path is recorded, and most kernel-built bodies
    ///   carry none (only the STEP reader attaches them today), which is not
    ///   a defect.
    /// - a path that does not close up once branch shifts are taken out
    ///   (below): the polygon is then not the loop's boundary and its area
    ///   means nothing.
    /// - a path enclosing essentially nothing relative to its own extent: a
    ///   sphere face bounded only by its seam meridian runs up one branch
    ///   and back down the other, and the sign of that near-zero area is
    ///   noise. Which branch each of the two runs takes is arbitrary there —
    ///   the two lifts below disagree — so there is nothing to read.
    ///
    /// On a periodic surface the path is lifted to the universal cover
    /// before its area is taken: each fin's pcurve picks its own branch, so
    /// a cylinder wall's boundary arrives as four runs that jump a whole
    /// period between them. Continuing each run from the previous one's end
    /// unrolls those jumps and recovers the rectangle the wall really is.
    ///
    /// The sign answers "which way is this loop wound"; the magnitude answers
    /// "how much does it enclose", which is how a reader tells a face's outer
    /// bound from its holes when the source file does not say.
    pub fn loop_winding(
        &self,
        geo: &GeometryStore,
        surface: &Surface3,
        loop_id: EntityId<Loop>,
    ) -> Option<f64> {
        let lp = self.loops.get(loop_id)?;
        if lp.fins.is_empty() {
            return None;
        }
        // One sampled run of parameter-space points per fin, in fin order.
        let mut runs: Vec<Vec<Point2>> = Vec::with_capacity(lp.fins.len());
        for &fin_id in &lp.fins {
            let fin = self.fins.get(fin_id)?;
            let pcurve = geo.pcurve(fin.pcurve?)?;
            let edge = self.edges.get(fin.edge)?;
            if !edge_range_is_sane(edge) {
                return None;
            }
            let forward = fin.sense == FinSense::Forward;
            let run: Vec<Point2> = edge_samples(edge.t_start, edge.t_end)
                .map(|t| {
                    // A Reversed fin traverses its edge end → start, so its
                    // pcurve is walked backwards over the same range.
                    let t = if forward {
                        t
                    } else {
                        edge.t_start + edge.t_end - t
                    };
                    pcurve.point(t)
                })
                .collect();
            if !run.iter().all(|p| p.coords.iter().all(|c| c.is_finite())) {
                return None;
            }
            runs.push(run);
        }

        // Lift to the universal cover: shift each run by whole periods so it
        // continues from where the previous one ended.
        let (period_u, period_v) = (surface.period_u(), surface.period_v());
        let mut offset = Vector2::zeros();
        let mut previous_end: Option<Point2> = None;
        for run in &mut runs {
            if let Some(previous) = previous_end {
                offset += branch_shift(previous - (run[0] + offset), period_u, period_v);
            }
            for p in run.iter_mut() {
                *p += offset;
            }
            previous_end = Some(*run.last()?);
        }

        let polygon: Vec<Point2> = runs.iter().flatten().copied().collect();
        let (mut min, mut max) = (polygon[0], polygon[0]);
        for p in &polygon {
            min = Point2::new(min.x.min(p.x), min.y.min(p.y));
            max = Point2::new(max.x.max(p.x), max.y.max(p.y));
        }
        let extent = (max.x - min.x).max(max.y - min.y);
        if extent <= 0.0 || !extent.is_finite() {
            return None;
        }

        // Continuity: each run must now start where the previous one ended,
        // the last one included — the lift closes a periodic path only if it
        // was one to begin with.
        for i in 0..runs.len() {
            let end = *runs[i].last()?;
            let start = *runs[(i + 1) % runs.len()].first()?;
            if (start - end).norm() > WINDING_CONTINUITY_REL * extent {
                return None;
            }
        }

        let mut twice_area = 0.0;
        for i in 0..polygon.len() {
            let (a, b) = (polygon[i], polygon[(i + 1) % polygon.len()]);
            twice_area += a.x * b.y - b.x * a.y;
        }
        (twice_area.abs() > WINDING_AREA_REL * extent * extent).then_some(twice_area)
    }
}

/// Tolerance sanity: finite, at least the resolution floor, at most the
/// system-wide cap.
fn check_tolerance(failures: &mut Vec<CheckFailure>, entity: EntityRef, tolerance: f64) {
    if !tolerance.is_finite() || tolerance < SYSTEM_RESOLUTION {
        failures.push(CheckFailure::InvalidTolerance { entity, tolerance });
    } else if tolerance > MAX_ALLOWED_TOLERANCE {
        failures.push(CheckFailure::ToleranceExceeded {
            entity,
            tolerance,
            limit: MAX_ALLOWED_TOLERANCE,
        });
    }
}

// --------------------------------------------------------------------------
// Geometric measurement
// --------------------------------------------------------------------------

/// Samples taken along an edge by the geometric checks. One more than a
/// power of two, so both endpoints and the midpoint are hit exactly.
const GEOMETRY_SAMPLES: usize = 17;

/// Round-off slack, relative to coordinate magnitude, added to a declared
/// tolerance before a measured deviation counts as a violation.
///
/// Deviations are measured by evaluating and projecting points, whose
/// absolute error scales with how far from the origin they sit, so the slack
/// scales with it too (floored at magnitude 1, giving an absolute 1e-11 near
/// the origin). Without it a body a kilometre from the origin reports
/// floating-point noise as a precision defect. It sits far above the ~1e-16
/// relative error the arithmetic carries and far below any tolerance a
/// producer would set deliberately, so it neither cries wolf nor hides one.
const PROJECTION_SLACK_REL: f64 = 1e-11;

/// How much of its own extent a loop must enclose in parameter space before
/// its winding is taken as evidence of a face's orientation.
const WINDING_AREA_REL: f64 = 1e-6;

/// How closely, relative to the loop's parameter-space extent, consecutive
/// fins' pcurves must meet for the loop to count as one closed path.
const WINDING_CONTINUITY_REL: f64 = 1e-6;

/// The whole-period displacement bringing `gap` closest to zero, in each
/// periodic parameter direction. Non-periodic directions never shift: there
/// is no other representative of the same point to move to.
fn branch_shift(gap: Vector2, period_u: Option<f64>, period_v: Option<f64>) -> Vector2 {
    let along = |gap: f64, period: Option<f64>| match period {
        Some(period) if period > 0.0 && period.is_finite() => (gap / period).round() * period,
        _ => 0.0,
    };
    Vector2::new(along(gap.x, period_u), along(gap.y, period_v))
}

/// Whether every coordinate of `p` is finite.
fn is_finite(p: &Point3) -> bool {
    p.coords.iter().all(|c| c.is_finite())
}

/// An edge's parameter range is usable exactly when it is finite and
/// increasing. NaN-safe: only a strictly increasing finite range passes.
fn edge_range_is_sane(edge: &Edge) -> bool {
    edge.t_start.is_finite()
        && edge.t_end.partial_cmp(&edge.t_start) == Some(std::cmp::Ordering::Greater)
}

/// The [`GEOMETRY_SAMPLES`] parameters spanning `[t_start, t_end]`, both
/// endpoints included.
fn edge_samples(t_start: f64, t_end: f64) -> impl Iterator<Item = f64> {
    (0..GEOMETRY_SAMPLES)
        .map(move |i| t_start + (t_end - t_start) * (i as f64) / ((GEOMETRY_SAMPLES - 1) as f64))
}

/// The largest gap found over a run of samples, together with the coordinate
/// magnitude the samples sat at — which is what sets the round-off slack the
/// gap is judged against.
#[derive(Debug, Clone, Copy)]
struct Deviation {
    max: f64,
    scale: f64,
}

impl Deviation {
    fn new() -> Self {
        Deviation {
            max: 0.0,
            scale: 1.0,
        }
    }

    /// Record a gap of `gap` measured at `at`.
    fn record(&mut self, gap: f64, at: &Point3) {
        // `>` is NaN-safe in the direction that matters: a NaN gap is not
        // recorded rather than poisoning the maximum.
        if gap > self.max {
            self.max = gap;
        }
        self.scale = self
            .scale
            .max(at.coords.iter().fold(0.0f64, |acc, &c| acc.max(c.abs())));
    }

    /// `Some((max, allowed))` when the largest gap exceeds what `tolerance`
    /// permits, `None` when it is within it.
    fn exceeding(&self, tolerance: f64) -> Option<(f64, f64)> {
        // A tolerance that is itself nonsense is reported by `check`; here it
        // falls back to the resolution floor rather than compounding into a
        // second, misleading failure (an infinite tolerance permitting
        // everything, a negative one permitting nothing).
        let claimed = if tolerance.is_finite() && tolerance > SYSTEM_RESOLUTION {
            tolerance
        } else {
            SYSTEM_RESOLUTION
        };
        let allowed = claimed + PROJECTION_SLACK_REL * self.scale;
        (self.max > allowed).then_some((self.max, allowed))
    }
}

/// Largest distance from `curve`, over `[t_start, t_end]`, to `surface`.
///
/// Each sample is projected with [`SurfaceProject::project_point`], never
/// with the seeded variant. A seed picks a *branch*, and the question here
/// is not which branch the curve is nearest — it is how far the curve is
/// from the surface at all, which is the global minimum. Seeding from the
/// previous sample is faster and correct for walking a known curve, but on a
/// surface that closes on itself (a rational-quadratic full circle extruded
/// into a ruled patch, all over the STEP corpus) the seeded iteration can
/// settle on a stationary point that is not the nearest one and report a
/// curve lying exactly on its surface as a whole diameter off it.
///
/// Samples whose projection did not converge are skipped: a projection
/// returns a point that genuinely is on the surface, so its distance is an
/// *upper* bound on the true one, and a stalled iteration can only
/// over-report.
fn curve_surface_deviation(
    surface: &Surface3,
    curve: &Curve3,
    t_start: f64,
    t_end: f64,
) -> Deviation {
    let mut deviation = Deviation::new();
    for t in edge_samples(t_start, t_end) {
        let p = curve.point(t);
        if !is_finite(&p) {
            continue;
        }
        let projection = surface.project_point(&p);
        if projection.converged {
            deviation.record(projection.distance, &p);
        }
    }
    deviation
}

/// Largest gap between `surface.point(pcurve(t))` and `curve.point(t)` — the
/// parameterization invariant [`crate::pcurve`] is built around.
fn pcurve_deviation(
    surface: &Surface3,
    curve: &Curve3,
    pcurve: &Curve2,
    t_start: f64,
    t_end: f64,
) -> Deviation {
    let mut deviation = Deviation::new();
    for t in pcurve_check_params(pcurve, t_start, t_end) {
        let uv = pcurve.point(t);
        if !uv.coords.iter().all(|c| c.is_finite()) {
            continue;
        }
        let (on_surface, on_curve) = (surface.point(uv.x, uv.y), curve.point(t));
        if !is_finite(&on_surface) || !is_finite(&on_curve) {
            continue;
        }
        deviation.record((on_surface - on_curve).norm(), &on_curve);
    }
    deviation
}

/// Parameters at which a pcurve is held to the invariant.
///
/// [`Curve2::Line`] and [`Curve2::Circle`] are exact fits and
/// [`Curve2::Projected`] is an exact inverse, so all three are sampled
/// evenly across the edge's range — for `Projected` that is not a courtesy
/// but the strongest check available, since it is the only variant that
/// claims the invariant at parameters nobody sampled while building it. A
/// [`Curve2::Polyline`] only
/// claims to lie on the surface *at its own vertices*: between them it is a
/// chord, and the error there is bounded by the sample spacing — the
/// documented, deliberate approximation of freeform trim (see
/// [`crate::pcurve`]), not a defect. Its vertices are where a pcurve on the
/// wrong branch, running backwards, or fitted against a different edge shows
/// up anyway.
///
/// A polyline whose parameters miss the edge's range is not that edge's
/// pcurve at all, so there the range itself is sampled and the mismatch
/// measured rather than quietly skipped.
fn pcurve_check_params(pcurve: &Curve2, t_start: f64, t_end: f64) -> Vec<f64> {
    if let Curve2::Polyline { params, .. } = pcurve {
        let inside: Vec<f64> = params
            .iter()
            .copied()
            .filter(|&t| t >= t_start && t <= t_end)
            .collect();
        if inside.len() >= 2 {
            return inside;
        }
    }
    edge_samples(t_start, t_end).collect()
}

// --------------------------------------------------------------------------
// Self-intersection
// --------------------------------------------------------------------------

/// Samples taken along one intersection curve. The clash search is a
/// sampled one (see [`TopologyStore::check_self_intersection`]), so this is
/// the resolution below which an overlap can hide: a fine enough comb for
/// any clash a producer would call a defect, and still cheap against the
/// closed-form evaluation of a single curve.
const CLASH_CURVE_SAMPLES: usize = 128;

/// Samples per parameter direction of the grid that fills in a face's
/// bounding box between its rim samples.
const FACE_GRID: usize = 12;

/// Samples per parameter direction over a face's parameter rectangle when
/// its surface is *coincident* with another face's: there is no
/// intersection curve to walk, so the overlap is hunted over the region
/// itself. Denser than [`FACE_GRID`], which only has to bound a box.
const COINCIDENT_GRID: usize = 24;

/// Segments a straight boundary edge is sampled into for the parameter-space
/// cover. Straight in space is not straight in a curved surface's parameter
/// space, so even a line gets several.
const BOUNDARY_LINE_SEGMENTS: usize = 8;

/// Segments a full revolution of a curved boundary edge is sampled into
/// (matching the density `crate::boolean` builds its own face regions at).
const BOUNDARY_CIRCLE_SEGMENTS: usize = 96;

/// How far past the two faces' box overlap the intersection curves are
/// searched, relative to the pair's joint extent.
///
/// Not an accuracy knob — a slack that keeps a *degenerate* overlap (two
/// planar faces meeting along an edge overlap in a zero-width slab)
/// searchable at all, since clipping a line to a zero-width slab is a
/// coin flip on the last bit. It must stay well below the tolerance band a
/// contact is judged against, or a point admitted only by the padding could
/// read as too far from the shared edge that explains it.
const SEARCH_PAD_REL: f64 = 1e-9;

/// The self-intersection pass's working view of one face.
struct FacePatch {
    face: EntityId<Face>,
    surface: Surface3,
    /// The trimmed region in the surface's parameter cover.
    poly: FaceRegionPoly,
    /// Conservative bounds of the face in space (see [`face_bbox`]).
    bbox: BoundingBox3,
    uv_min: (f64, f64),
    uv_max: (f64, f64),
    /// Sub-tolerance nudge for the seam-robust containment test.
    snap: f64,
    edges: Vec<BoundaryEdge>,
    vertices: Vec<(EntityId<Vertex>, Point3)>,
    /// The widest tolerance anything on this face's boundary claims — the
    /// band within which a contact is measured rather than believed.
    tolerance: f64,
}

impl FacePatch {
    /// Does this face reach `p`? Inside the trimmed region, or within
    /// `slack` of its boundary — the second arm is what catches a face
    /// *stabbed* by another face's edge, where the contact locus lies on
    /// one face's boundary and the strict even-odd test is a coin flip.
    fn covers(&self, p: &Point3, slack: f64) -> bool {
        if let Ok(uv) = self.poly.chart.param(p, None) {
            if self.poly.contains_for_clip(uv, self.snap) {
                return true;
            }
        }
        self.boundary_distance(p) <= slack
    }

    /// Distance from `p` to the nearest point of this face's boundary.
    fn boundary_distance(&self, p: &Point3) -> f64 {
        self.edges
            .iter()
            .map(|edge| edge.distance_to(p))
            .fold(f64::INFINITY, f64::min)
    }

    /// Points of the surface over this face's parameter rectangle, row
    /// major at `n` samples per direction. `None` if any sample is
    /// non-finite, which makes every box or overlap built on it meaningless.
    fn grid(&self, n: usize) -> Option<Vec<Point3>> {
        let mut out = Vec::with_capacity(n * n);
        for i in 0..n {
            for j in 0..n {
                let p = self.surface.point(
                    lerp(self.uv_min.0, self.uv_max.0, i, n),
                    lerp(self.uv_min.1, self.uv_max.1, j, n),
                );
                if !is_finite(&p) {
                    return None;
                }
                out.push(p);
            }
        }
        Some(out)
    }
}

/// One edge on a face's boundary, with the range the edge trims it to.
struct BoundaryEdge {
    id: EntityId<Edge>,
    curve: Curve3,
    t_start: f64,
    t_end: f64,
}

impl BoundaryEdge {
    /// Distance from `p` to the *trimmed* edge.
    ///
    /// Measured against the curve itself, never against the polyline the
    /// cover is built from: a 96-segment circle's chords sit up to
    /// `r·1e-3` inside it, which would leave a point exactly on a shared
    /// circular edge looking a thousand tolerances away from it and turn
    /// every cylinder rim into a reported clash.
    ///
    /// The endpoints join the projected parameter as candidates, so a
    /// projection that lands outside the trim (or on a periodic curve's far
    /// side) still measures to the nearest point of the edge that exists.
    fn distance_to(&self, p: &Point3) -> f64 {
        let mut t = self.curve.project_point(p).t;
        if let Some(period) = self.curve.period() {
            if period > 0.0 && period.is_finite() {
                t = self.t_start + (t - self.t_start).rem_euclid(period);
            }
        }
        [t.clamp(self.t_start, self.t_end), self.t_start, self.t_end]
            .into_iter()
            .map(|t| self.curve.point(t))
            .filter(is_finite)
            .map(|q| (q - p).norm())
            .fold(f64::INFINITY, f64::min)
    }
}

/// Sample `i` of `n` across `[lo, hi]`, both ends included.
fn lerp(lo: f64, hi: f64, i: usize, n: usize) -> f64 {
    if n <= 1 {
        return lo;
    }
    lo + (hi - lo) * (i as f64) / ((n - 1) as f64)
}

/// Append one fin's boundary samples to its loop's walk, in traversal
/// order. The final point is dropped: the next fin starts there, and the
/// walk closes implicitly onto its first point.
fn append_fin_samples(
    out: &mut Vec<Point3>,
    curve: &Curve3,
    t_start: f64,
    t_end: f64,
    forward: bool,
) {
    let n = boundary_segments(curve, t_start, t_end);
    let mut points: Vec<Point3> = (0..=n)
        .map(|i| curve.point(lerp(t_start, t_end, i, n + 1)))
        .collect();
    if !forward {
        points.reverse();
    }
    points.pop();
    out.append(&mut points);
}

/// How finely one boundary edge is sampled: a straight edge by a fixed
/// count, an angular one by its sweep, a freeform one by the knot spans it
/// covers (within a span it is a single polynomial piece).
fn boundary_segments(curve: &Curve3, t_start: f64, t_end: f64) -> usize {
    match curve {
        Curve3::Line { .. } => BOUNDARY_LINE_SEGMENTS,
        Curve3::Polyline { .. } => {
            // Parameterized by vertex index: one segment per vertex.
            (((t_end - t_start).ceil()).max(0.0) as usize).max(BOUNDARY_LINE_SEGMENTS)
        }
        Curve3::Nurbs(nurbs) => {
            let (lo, hi) = (t_start.min(t_end), t_start.max(t_end));
            let interior = nurbs
                .knot_vector()
                .knots()
                .windows(2)
                .filter(|w| w[1] > w[0] && w[1] > lo && w[1] < hi)
                .count();
            (interior + 1) * (BOUNDARY_CIRCLE_SEGMENTS / 4)
        }
        _ => {
            let sweep = (t_end - t_start).abs() / std::f64::consts::TAU;
            ((BOUNDARY_CIRCLE_SEGMENTS as f64 * sweep).ceil() as usize).max(BOUNDARY_LINE_SEGMENTS)
        }
    }
}

/// Bounds of a face's cover in parameter space; `None` if it is not finite,
/// in which case nothing downstream can be sampled over it.
fn uv_bounds(poly: &FaceRegionPoly) -> Option<((f64, f64), (f64, f64))> {
    let (mut lo, mut hi) = (
        (f64::INFINITY, f64::INFINITY),
        (f64::NEG_INFINITY, f64::NEG_INFINITY),
    );
    for &((u, v), _) in poly.loops.iter().flatten() {
        if !u.is_finite() || !v.is_finite() {
            return None;
        }
        lo = (lo.0.min(u), lo.1.min(v));
        hi = (hi.0.max(u), hi.1.max(v));
    }
    (lo.0 <= hi.0 && lo.1 <= hi.1).then_some((lo, hi))
}

/// A box containing the whole face, rim *and* interior.
///
/// The rim alone is not enough: a sphere zone straddling the equator bulges
/// a quarter of its radius past the box its two latitude circles span. The
/// face lies entirely over its cover's parameter rectangle, so the surface
/// is sampled on a grid across that rectangle and the box taken over
/// everything. What a grid cannot see is how far the surface bows *between*
/// samples, so the box is dilated by twice the largest such bow found — the
/// distance from each cell edge's midpoint on the surface to the chord
/// between its endpoints. That is zero for a plane, real for a sphere, and
/// scales down as the square of the cell size, so the doubling is ample
/// slack rather than a guess.
fn face_bbox(
    patch_surface: &Surface3,
    poly: &FaceRegionPoly,
    uv_min: (f64, f64),
    uv_max: (f64, f64),
) -> Option<BoundingBox3> {
    let mut grid = Vec::with_capacity(FACE_GRID * FACE_GRID);
    for i in 0..FACE_GRID {
        for j in 0..FACE_GRID {
            let uv = (
                lerp(uv_min.0, uv_max.0, i, FACE_GRID),
                lerp(uv_min.1, uv_max.1, j, FACE_GRID),
            );
            let p = patch_surface.point(uv.0, uv.1);
            if !is_finite(&p) {
                return None;
            }
            grid.push((uv, p));
        }
    }

    let mut sag: f64 = 0.0;
    let at = |i: usize, j: usize| grid[i * FACE_GRID + j];
    for i in 0..FACE_GRID {
        for j in 0..FACE_GRID {
            for (di, dj) in [(1, 0), (0, 1)] {
                let (i2, j2) = (i + di, j + dj);
                if i2 >= FACE_GRID || j2 >= FACE_GRID {
                    continue;
                }
                let ((u1, v1), p1) = at(i, j);
                let ((u2, v2), p2) = at(i2, j2);
                let mid = patch_surface.point(0.5 * (u1 + u2), 0.5 * (v1 + v2));
                if !is_finite(&mid) {
                    return None;
                }
                sag = sag.max((mid - (p1 + (p2 - p1) * 0.5)).norm());
            }
        }
    }

    let bbox = BoundingBox3::from_points(
        poly.loops
            .iter()
            .flatten()
            .map(|&(_, p)| p)
            .chain(grid.iter().map(|&(_, p)| p)),
    );
    (!bbox.is_empty()).then(|| bbox.dilate(2.0 * sag))
}

/// One point where two faces clash, if they do.
fn clash_point(
    a: &FacePatch,
    b: &FacePatch,
    overlap: &BoundingBox3,
    tol: &ToleranceContext,
) -> Option<Point3> {
    let joint = a.bbox.union(&b.bbox);
    let pad = (SEARCH_PAD_REL * joint.extents().norm()).max(SYSTEM_RESOLUTION);
    let roi = overlap.dilate(pad);
    intersection_samples(a, b, &roi, tol)?
        .into_iter()
        .find(|p| is_clash(a, b, p, tol))
}

/// Is `p` — a point on both surfaces — a defect rather than the two faces
/// meeting along topology they share?
fn is_clash(a: &FacePatch, b: &FacePatch, p: &Point3, tol: &ToleranceContext) -> bool {
    if !is_finite(p) {
        return false;
    }
    let scale = p.coords.iter().fold(1.0f64, |m, &c| m.max(c.abs()));
    let slack = a.tolerance.max(b.tolerance).max(tol.linear) + PROJECTION_SLACK_REL * scale;
    if !a.covers(p, slack) || !b.covers(p, slack) {
        return false;
    }
    // Twice the band `covers` admits: a point let in only by sitting `slack`
    // outside a face is at most that far past the shared edge that explains
    // it, and excusing it has to be at least as generous as admitting it was.
    shared_boundary_distance(a, b, p) > 2.0 * slack
}

/// Distance from `p` to the nearest entity the two faces *share* —
/// infinite when they share none, so nothing is excused.
fn shared_boundary_distance(a: &FacePatch, b: &FacePatch, p: &Point3) -> f64 {
    let edges = a
        .edges
        .iter()
        .filter(|e| b.edges.iter().any(|o| o.id == e.id))
        .map(|e| e.distance_to(p));
    let vertices = a
        .vertices
        .iter()
        .filter(|&&(id, _)| b.vertices.iter().any(|&(o, _)| o == id))
        .map(|&(_, point)| (point - p).norm());
    edges.chain(vertices).fold(f64::INFINITY, f64::min)
}

/// Points on both surfaces, to be classified against the two trims.
///
/// `None` — as distinct from an empty vector — when the surface pair's
/// intersection cannot be computed at all, so the pair is skipped rather
/// than declared clean.
fn intersection_samples(
    a: &FacePatch,
    b: &FacePatch,
    roi: &BoundingBox3,
    tol: &ToleranceContext,
) -> Option<Vec<Point3>> {
    match ssi::intersect(&a.surface, &b.surface, tol) {
        Ok(SurfaceIntersection::Empty) => Some(Vec::new()),
        Ok(SurfaceIntersection::TangentPoint(p)) => Some(vec![p]),
        Ok(SurfaceIntersection::Curves(curves)) => Some(
            curves
                .iter()
                .flat_map(|c| sample_intersection_curve(&c.curve, roi))
                .collect(),
        ),
        // The same surface twice: there is no intersection *curve* to walk —
        // the whole of each face's region is on the other's surface — so the
        // regions themselves are the search space. Both are swept, not just
        // one: a small face sitting well inside a large one is caught by its
        // own rim and grid, where the large face's grid could step over it.
        Ok(SurfaceIntersection::Coincident) => {
            let mut samples: Vec<Point3> = Vec::new();
            for patch in [a, b] {
                samples.extend(patch.poly.loops.iter().flatten().map(|&(_, p)| p));
                samples.extend(patch.grid(COINCIDENT_GRID)?);
            }
            Some(samples)
        }
        Err(_) => marched_samples(a, b, roi, tol),
    }
}

/// The numerically traced route, for the pairs with no closed form.
fn marched_samples(
    a: &FacePatch,
    b: &FacePatch,
    roi: &BoundingBox3,
    tol: &ToleranceContext,
) -> Option<Vec<Point3>> {
    let radius = 0.5 * roi.extents().norm();
    if !radius.is_finite() {
        return None;
    }
    // The region of interest as the sphere through the overlap box's
    // corners: the marched-bounded route clips its unbounded partners to it,
    // and anything outside it is outside one of the two faces anyway.
    let bounds = (roi.center(), radius.max(tol.linear));
    let curves = if is_bounded_marched(&a.surface, &b.surface) {
        ssi::intersect_marched_bounded(&a.surface, &b.surface, bounds, tol)
    } else {
        ssi::intersect_marched(&a.surface, &b.surface, tol)
    };
    Some(curves.ok()?.into_iter().flat_map(|c| c.points).collect())
}

/// Points along one intersection curve, over the part of it inside `roi`.
///
/// An unbounded curve — the line two planes meet in — is clipped to the
/// region first; there is no other bound on where to look, and outside the
/// two faces' shared box neither face is present.
fn sample_intersection_curve(curve: &Curve3, roi: &BoundingBox3) -> Vec<Point3> {
    let (mut t0, mut t1) = curve.domain();
    if !t0.is_finite() || !t1.is_finite() {
        let Curve3::Line { origin, dir } = curve else {
            return Vec::new();
        };
        let Some((clipped_start, clipped_end)) = clip_line_to_box(origin, dir, roi) else {
            return Vec::new();
        };
        (t0, t1) = (clipped_start, clipped_end);
    }
    // NaN-safe: only a genuinely increasing range is sampled.
    if t1.partial_cmp(&t0) != Some(std::cmp::Ordering::Greater) {
        return Vec::new();
    }
    (0..=CLASH_CURVE_SAMPLES)
        .map(|i| curve.point(lerp(t0, t1, i, CLASH_CURVE_SAMPLES + 1)))
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topology::{BodyType, FaceSense, LoopType, ShellOrientation};
    use opensolid_core::Point3;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    /// A cube built purely from Euler operators (the known-good baseline).
    fn build_cube() -> (TopologyStore, EntityId<Body>, EntityId<Shell>) {
        let mut store = TopologyStore::new();
        let (body, v0, f_bottom, shell) = store.mvfs(p(0.0, 0.0, 0.0));
        let (_e, v1) = store.mev(v0, f_bottom, p(1.0, 0.0, 0.0)).unwrap();
        let (_e, v2) = store.mev(v1, f_bottom, p(1.0, 1.0, 0.0)).unwrap();
        let (_e, v3) = store.mev(v2, f_bottom, p(0.0, 1.0, 0.0)).unwrap();
        let (_e, f_top) = store.mef(v3, v0, f_bottom).unwrap();
        let (_e, v4) = store.mev(v0, f_top, p(0.0, 0.0, 1.0)).unwrap();
        let (_e, v5) = store.mev(v1, f_top, p(1.0, 0.0, 1.0)).unwrap();
        let (_e, v6) = store.mev(v2, f_top, p(1.0, 1.0, 1.0)).unwrap();
        let (_e, v7) = store.mev(v3, f_top, p(0.0, 1.0, 1.0)).unwrap();
        store.mef(v4, v7, f_top).unwrap();
        store.mef(v7, v6, f_top).unwrap();
        store.mef(v6, v5, f_top).unwrap();
        store.mef(v5, v4, f_top).unwrap();
        (store, body, shell)
    }

    /// A single triangular sheet face. `is_closed` sets the shell's flag.
    fn build_triangle_sheet(
        body_type: BodyType,
        is_closed: bool,
    ) -> (
        TopologyStore,
        EntityId<Body>,
        EntityId<Shell>,
        [EntityId<Edge>; 3],
    ) {
        let mut store = TopologyStore::new();
        let body = store.create_body(body_type);
        let shell = store.create_shell(body, is_closed, ShellOrientation::Outward);
        let face = store.create_face(shell, FaceSense::Positive);
        let v: Vec<_> = [p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0)]
            .iter()
            .map(|&pt| store.create_vertex(pt, SYSTEM_RESOLUTION))
            .collect();
        let edges = [
            store.create_edge(v[0], v[1], SYSTEM_RESOLUTION),
            store.create_edge(v[1], v[2], SYSTEM_RESOLUTION),
            store.create_edge(v[2], v[0], SYSTEM_RESOLUTION),
        ];
        store.create_loop(
            face,
            LoopType::Outer,
            &edges.map(|e| (e, FinSense::Forward)),
        );
        (store, body, shell, edges)
    }

    #[test]
    fn euler_built_cube_passes() {
        let (store, body, _shell) = build_cube();
        assert_eq!(store.check(body), Vec::new());
    }

    #[test]
    fn minimal_mvfs_body_passes() {
        let mut store = TopologyStore::new();
        let (body, ..) = store.mvfs(p(0.0, 0.0, 0.0));
        assert_eq!(store.check(body), Vec::new());
    }

    #[test]
    fn open_sheet_passes() {
        let (store, body, _shell, _edges) = build_triangle_sheet(BodyType::Sheet, false);
        assert_eq!(store.check(body), Vec::new());
    }

    #[test]
    fn closed_flag_on_open_sheet_reports_every_boundary_edge() {
        let (store, body, shell, edges) = build_triangle_sheet(BodyType::Sheet, true);
        let failures = store.check(body);
        assert_eq!(failures.len(), 3);
        for edge in edges {
            assert!(
                failures.contains(&CheckFailure::OpenEdgeInClosedShell { shell, edge }),
                "missing open-edge failure for {edge:?} in {failures:?}"
            );
        }
    }

    #[test]
    fn solid_with_flagged_open_sheet_fails() {
        // The lying producer: an open sheet on a Solid body whose shell
        // honestly reports is_closed = false. Previously passed with zero
        // failures because every closure check trusted the flag.
        let (store, body, shell, edges) = build_triangle_sheet(BodyType::Solid, false);
        let failures = store.check(body);
        assert!(
            failures.contains(&CheckFailure::OpenShellInSolid { body, shell }),
            "expected OpenShellInSolid in {failures:?}"
        );
        for edge in edges {
            assert!(
                failures.contains(&CheckFailure::OpenEdgeInClosedShell { shell, edge }),
                "missing open-edge failure for {edge:?} in {failures:?}"
            );
        }
        assert_eq!(failures.len(), 4);
    }

    #[test]
    fn solid_shell_flagged_open_but_structurally_closed() {
        // A watertight cube whose shell flag lies the other way: the flag
        // itself fails for a solid, and the flag/structure mismatch is
        // reported. Euler stays skipped (failures are non-empty), but not
        // silently.
        let (mut store, body, shell) = build_cube();
        store.shells.get_mut(shell).unwrap().is_closed = false;

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::OpenShellInSolid { body, shell }));
        assert!(failures.contains(&CheckFailure::ShellFlaggedOpenButClosed(shell)));
        assert_eq!(failures.len(), 2);
    }

    #[test]
    fn flag_structure_mismatch_reported_independent_of_body_type() {
        // Same watertight topology on a Sheet body: no solid-closure
        // requirement, but the flag still contradicts the structure.
        let (mut store, body, shell) = build_cube();
        store.bodies.get_mut(body).unwrap().body_type = BodyType::Sheet;
        store.shells.get_mut(shell).unwrap().is_closed = false;

        let failures = store.check(body);
        assert_eq!(
            failures,
            vec![CheckFailure::ShellFlaggedOpenButClosed(shell)]
        );
    }

    #[test]
    fn stale_body_reported() {
        let (mut store, body, _shell) = build_cube();
        store.bodies.remove(body);
        assert_eq!(store.check(body), vec![CheckFailure::StaleBody(body)]);
    }

    #[test]
    fn missing_fin_on_edge_detected() {
        let (mut store, body, shell) = build_cube();
        let (edge, _) = store.edges.iter().next().expect("cube has edges");
        let dropped = store
            .edges
            .get_mut(edge)
            .unwrap()
            .fins
            .pop()
            .expect("manifold edge has 2 fins");

        let failures = store.check(body);
        assert!(
            failures.contains(&CheckFailure::FinMissingFromEdge { fin: dropped, edge }),
            "expected FinMissingFromEdge in {failures:?}"
        );
        // The de-registered fin also leaves the edge single-finned in a
        // closed shell.
        assert!(failures.contains(&CheckFailure::OpenEdgeInClosedShell { shell, edge }));
    }

    #[test]
    fn stale_fin_detected_from_loop_and_edge() {
        let (mut store, body, _shell) = build_cube();
        let (edge, e) = store.edges.iter().next().expect("cube has edges");
        let victim = e.fins[0];
        let loop_id = store.fin(victim).unwrap().loop_ref;
        store.fins.remove(victim);

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::StaleReference {
            from: EntityRef::Loop(loop_id),
            to: EntityRef::Fin(victim),
        }));
        assert!(failures.contains(&CheckFailure::StaleReference {
            from: EntityRef::Edge(edge),
            to: EntityRef::Fin(victim),
        }));
    }

    #[test]
    fn reversed_face_reports_inconsistent_orientation_on_each_edge() {
        let (mut store, body, shell) = build_cube();
        let face = store.faces_of_shell(shell)[0];
        let loop_id = store.face(face).unwrap().outer_loop.unwrap();

        // Coherently reverse the loop: reversed fin order, flipped senses,
        // relinked next/prev. The loop stays closed and vertex-continuous,
        // but the face now disagrees with all four neighbors.
        let fins: Vec<_> = store
            .loop_(loop_id)
            .unwrap()
            .fins
            .iter()
            .rev()
            .copied()
            .collect();
        let n = fins.len();
        for (i, &fin_id) in fins.iter().enumerate() {
            let fin = store.fins.get_mut(fin_id).unwrap();
            fin.sense = fin.sense.opposite();
            fin.next = Some(fins[(i + 1) % n]);
            fin.prev = Some(fins[(i + n - 1) % n]);
        }
        store.loops.get_mut(loop_id).unwrap().fins = fins;

        let failures = store.check(body);
        assert_eq!(
            failures.len(),
            4,
            "one per edge of the flipped quad: {failures:?}"
        );
        for failure in &failures {
            match failure {
                CheckFailure::InconsistentOrientation { face_a, face_b, .. } => {
                    assert!(
                        *face_a == face || *face_b == face,
                        "flipped face must be implicated: {failure:?}"
                    );
                }
                other => panic!("expected only InconsistentOrientation, got {other:?}"),
            }
        }
    }

    #[test]
    fn single_flipped_fin_breaks_loop_continuity_and_orientation() {
        let (mut store, body, shell) = build_cube();
        let face = store.faces_of_shell(shell)[0];
        let loop_id = store.face(face).unwrap().outer_loop.unwrap();
        let fin = store.fins_of_loop(loop_id)[0];
        let edge = store.fin_edge(fin);
        let prev = store.fin_prev(fin);

        let f = store.fins.get_mut(fin).unwrap();
        f.sense = f.sense.opposite();

        let failures = store.check(body);
        // The loop breaks on both sides of the flipped fin...
        assert!(failures.contains(&CheckFailure::LoopNotVertexContinuous { loop_id, fin }));
        assert!(failures.contains(&CheckFailure::LoopNotVertexContinuous { loop_id, fin: prev }));
        // ...and its edge's mated fins now run the same direction.
        assert!(
            failures.iter().any(
                |f| matches!(f, CheckFailure::InconsistentOrientation { edge: e, .. } if *e == edge)
            ),
            "expected InconsistentOrientation on {edge:?} in {failures:?}"
        );
    }

    #[test]
    fn broken_next_link_detected() {
        let (mut store, body, shell) = build_cube();
        let face = store.faces_of_shell(shell)[0];
        let loop_id = store.face(face).unwrap().outer_loop.unwrap();
        let fin = store.fins_of_loop(loop_id)[1];
        store.fins.get_mut(fin).unwrap().next = None;

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::FinLinkBroken { loop_id, fin }));
    }

    #[test]
    fn non_manifold_edge_detected() {
        let (mut store, body, shell) = build_cube();
        // Graft a triangular flap onto an existing cube edge: that edge now
        // has three fins.
        let (edge, e) = store.edges.iter().next().expect("cube has edges");
        let (v0, v1) = (e.start_vertex, e.end_vertex);
        let w = store.create_vertex(p(5.0, 5.0, 5.0), SYSTEM_RESOLUTION);
        let e1w = store.create_edge(v1, w, SYSTEM_RESOLUTION);
        let ew0 = store.create_edge(w, v0, SYSTEM_RESOLUTION);
        let flap = store.create_face(shell, FaceSense::Positive);
        store.create_loop(
            flap,
            LoopType::Outer,
            &[
                (edge, FinSense::Forward),
                (e1w, FinSense::Forward),
                (ew0, FinSense::Forward),
            ],
        );

        let failures = store.check(body);
        assert!(
            failures.contains(&CheckFailure::NonManifoldEdge { edge, fins: 3 }),
            "expected NonManifoldEdge in {failures:?}"
        );
        // The flap's free edges are open edges of the closed shell.
        for open in [e1w, ew0] {
            assert!(failures.contains(&CheckFailure::OpenEdgeInClosedShell { shell, edge: open }));
        }
    }

    #[test]
    fn unmated_and_mismated_fins_detected() {
        let (mut store, body, _shell) = build_cube();
        let mut edge_iter = store.edges.iter();
        let (edge_a, ea) = edge_iter.next().expect("cube has edges");
        let (_edge_b, eb) = edge_iter.next().expect("cube has 12 edges");
        let (fin_a0, fin_a1) = (ea.fins[0], ea.fins[1]);
        let foreign = eb.fins[0];

        // Un-mate one edge's fins entirely.
        store.fins.get_mut(fin_a0).unwrap().mate = None;
        store.fins.get_mut(fin_a1).unwrap().mate = None;
        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::UnmatedFins { edge: edge_a }));

        // Re-mate one of them to a fin of a different edge.
        store.fins.get_mut(fin_a0).unwrap().mate = Some(foreign);
        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::MateNotMutual {
            fin: fin_a0,
            mate: foreign
        }));
        assert!(failures.contains(&CheckFailure::MateOnDifferentEdge {
            fin: fin_a0,
            mate: foreign
        }));
    }

    #[test]
    fn tolerance_sanity_detected() {
        let (mut store, body, _shell) = build_cube();
        let (bad_edge, _) = store.edges.iter().next().unwrap();
        let vertex_ids: Vec<_> = store.vertices.iter().map(|(id, _)| id).collect();
        let (bad_vertex, nan_vertex) = (vertex_ids[0], vertex_ids[1]);

        store.edges.get_mut(bad_edge).unwrap().tolerance = 0.02;
        store.vertices.get_mut(bad_vertex).unwrap().tolerance = -1.0;
        store.vertices.get_mut(nan_vertex).unwrap().point = p(f64::NAN, 0.0, 0.0);

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::ToleranceExceeded {
            entity: EntityRef::Edge(bad_edge),
            tolerance: 0.02,
            limit: MAX_ALLOWED_TOLERANCE,
        }));
        assert!(failures.contains(&CheckFailure::InvalidTolerance {
            entity: EntityRef::Vertex(bad_vertex),
            tolerance: -1.0,
        }));
        assert!(failures.contains(&CheckFailure::NonFinitePoint(nan_vertex)));

        // NaN tolerances are also invalid (asserted via matches!: NaN
        // breaks PartialEq comparison).
        store.edges.get_mut(bad_edge).unwrap().tolerance = f64::NAN;
        let failures = store.check(body);
        assert!(failures.iter().any(|f| matches!(
            f,
            CheckFailure::InvalidTolerance { entity: EntityRef::Edge(e), tolerance } if *e == bad_edge && tolerance.is_nan()
        )));
    }

    #[test]
    fn orphan_entities_detected() {
        let mut store = TopologyStore::new();
        let body = store.create_body(BodyType::General);
        let empty_shell = store.create_shell(body, false, ShellOrientation::Outward);
        let shell = store.create_shell(body, false, ShellOrientation::Outward);
        let bare_face = store.create_face(shell, FaceSense::Positive);
        let looped_face = store.create_face(shell, FaceSense::Positive);
        let empty_loop = store.loops.insert(Loop {
            face: looped_face,
            fins: Vec::new(),
            loop_type: LoopType::Outer,
            vertex: None,
        });
        store.faces.get_mut(looped_face).unwrap().outer_loop = Some(empty_loop);

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::EmptyShell(empty_shell)));
        assert!(failures.contains(&CheckFailure::FaceWithoutOuterLoop(bare_face)));
        assert!(failures.contains(&CheckFailure::EmptyLoop(empty_loop)));
    }

    #[test]
    fn vertex_loop_with_fins_detected() {
        let mut store = TopologyStore::new();
        let (body, v0, face, _shell) = store.mvfs(p(0.0, 0.0, 0.0));
        store.mev(v0, face, p(1.0, 0.0, 0.0)).unwrap();
        let loop_id = store.face(face).unwrap().outer_loop.unwrap();
        // A real two-fin loop that also claims to be a degenerate vertex loop.
        store.loops.get_mut(loop_id).unwrap().vertex = Some(v0);

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::VertexLoopWithFins(loop_id)));
    }

    #[test]
    fn loop_back_pointer_mismatch_detected() {
        let (mut store, body, shell) = build_cube();
        let faces = store.faces_of_shell(shell).to_vec();
        let (face_a, face_b) = (faces[0], faces[1]);
        let loop_id = store.face(face_a).unwrap().outer_loop.unwrap();
        store.loops.get_mut(loop_id).unwrap().face = face_b;

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::BackPointerMismatch {
            child: EntityRef::Loop(loop_id),
            expected_parent: EntityRef::Face(face_a),
        }));
    }

    #[test]
    fn genus_corruption_violates_euler_formula() {
        let (mut store, body, shell) = build_cube();
        store.shells.get_mut(shell).unwrap().genus = 1;

        let failures = store.check(body);
        assert_eq!(failures.len(), 1);
        match &failures[0] {
            CheckFailure::EulerViolation { body: b, counts } => {
                assert_eq!(*b, body);
                assert_eq!(counts.genus, 1);
                assert!(!counts.euler_poincare_holds());
            }
            other => panic!("expected EulerViolation, got {other:?}"),
        }
    }

    #[test]
    fn edge_missing_from_vertex_detected() {
        let (mut store, body, _shell) = build_cube();
        let (edge, e) = store.edges.iter().next().unwrap();
        let vertex = e.start_vertex;
        store
            .vertices
            .get_mut(vertex)
            .unwrap()
            .edges
            .retain(|&x| x != edge);

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::EdgeMissingFromVertex { edge, vertex }));
    }

    #[test]
    fn foreign_fin_on_edge_detected() {
        let (mut store, body, _shell) = build_cube();
        let edge_info: Vec<_> = store
            .edges
            .iter()
            .take(2)
            .map(|(id, e)| (id, e.fins[0]))
            .collect();
        let edge_a = edge_info[0].0;
        let foreign = edge_info[1].1;
        store.edges.get_mut(edge_a).unwrap().fins.push(foreign);

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::ForeignFinOnEdge {
            edge: edge_a,
            fin: foreign
        }));
        // Three registered fins also read as non-manifold.
        assert!(failures.contains(&CheckFailure::NonManifoldEdge {
            edge: edge_a,
            fins: 3
        }));
    }

    #[test]
    fn outer_loop_flagged_inner_detected() {
        let (mut store, body, shell, _edges) = build_triangle_sheet(BodyType::Sheet, false);
        let face = store.faces_of_shell(shell)[0];
        let loop_id = store.face(face).unwrap().outer_loop.unwrap();
        store.loops.get_mut(loop_id).unwrap().loop_type = LoopType::Inner;

        assert_eq!(
            store.check(body),
            vec![CheckFailure::OuterLoopFlaggedInner { face, loop_id }]
        );
    }

    #[test]
    fn inner_loop_flagged_outer_detected() {
        let (mut store, body, shell, edges) = build_triangle_sheet(BodyType::Sheet, false);
        let face = store.faces_of_shell(shell)[0];
        // A degenerate vertex loop at an existing vertex, mis-flagged Outer,
        // listed among the inner loops.
        let v = store.edges.get(edges[0]).unwrap().start_vertex;
        let loop_id = store.loops.insert(Loop {
            face,
            fins: Vec::new(),
            loop_type: LoopType::Outer,
            vertex: Some(v),
        });
        store.faces.get_mut(face).unwrap().inner_loops.push(loop_id);

        assert_eq!(
            store.check(body),
            vec![CheckFailure::InnerLoopFlaggedOuter { face, loop_id }]
        );
    }

    #[test]
    fn duplicate_loop_on_face_detected() {
        // The same loop as both the outer loop and an inner loop: reported
        // once, and the duplicate walk (with its double-reported defects and
        // double-counted Euler R) is skipped.
        let (mut store, body, shell) = build_cube();
        let face = store.faces_of_shell(shell)[0];
        let loop_id = store.face(face).unwrap().outer_loop.unwrap();
        store.faces.get_mut(face).unwrap().inner_loops.push(loop_id);

        assert_eq!(
            store.check(body),
            vec![CheckFailure::DuplicateLoopOnFace { face, loop_id }]
        );
    }

    #[test]
    fn solid_without_shells_detected() {
        let mut store = TopologyStore::new();
        let solid = store.create_body(BodyType::Solid);
        assert_eq!(
            store.check(solid),
            vec![CheckFailure::SolidWithoutShells(solid)]
        );

        // Only solids claim a bounding shell; an empty General body is fine.
        let general = store.create_body(BodyType::General);
        assert_eq!(store.check(general), Vec::new());
    }

    #[test]
    fn foreign_edge_on_vertex_detected() {
        let (mut store, body, _shell) = build_cube();
        let (edge, e) = store.edges.iter().next().unwrap();
        let (v0, v1) = (e.start_vertex, e.end_vertex);
        // An edge that touches neither endpoint of `edge`.
        let foreign = store
            .edges
            .iter()
            .find(|(_, f)| {
                f.start_vertex != v0
                    && f.start_vertex != v1
                    && f.end_vertex != v0
                    && f.end_vertex != v1
            })
            .map(|(id, _)| id)
            .expect("cube has a disjoint edge");
        store.vertices.get_mut(v0).unwrap().edges.push(foreign);

        let failures = store.check(body);
        assert!(
            failures.contains(&CheckFailure::ForeignEdgeOnVertex {
                vertex: v0,
                edge: foreign
            }),
            "expected ForeignEdgeOnVertex in {failures:?}"
        );
        // `edge` itself is still registered both ways: no mirror failure.
        assert!(!failures.contains(&CheckFailure::EdgeMissingFromVertex { edge, vertex: v0 }));
    }

    #[test]
    fn stale_edge_in_vertex_list_detected() {
        let (mut store, body, _shell) = build_cube();
        let (edge, e) = store.edges.iter().next().unwrap();
        let (v0, v1) = (e.start_vertex, e.end_vertex);
        // create_edge registers itself on both vertices; removing the edge
        // leaves both lists holding a stale id.
        let doomed = store.create_edge(v0, v1, SYSTEM_RESOLUTION);
        store.edges.remove(doomed);

        let failures = store.check(body);
        for vertex in [v0, v1] {
            assert!(
                failures.contains(&CheckFailure::StaleReference {
                    from: EntityRef::Vertex(vertex),
                    to: EntityRef::Edge(doomed),
                }),
                "expected stale-edge report on {vertex:?} in {failures:?}"
            );
        }
        let _ = edge;
    }

    #[test]
    fn tolerance_failure_does_not_mask_euler_violation() {
        // A loose tolerance is not a structural failure: the genus
        // corruption must still be reported alongside it.
        let (mut store, body, shell) = build_cube();
        store.shells.get_mut(shell).unwrap().genus = 1;
        let (bad_edge, _) = store.edges.iter().next().unwrap();
        store.edges.get_mut(bad_edge).unwrap().tolerance = 0.02;

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::ToleranceExceeded {
            entity: EntityRef::Edge(bad_edge),
            tolerance: 0.02,
            limit: MAX_ALLOWED_TOLERANCE,
        }));
        assert!(
            failures
                .iter()
                .any(|f| matches!(f, CheckFailure::EulerViolation { body: b, .. } if *b == body)),
            "expected EulerViolation alongside the tolerance failure: {failures:?}"
        );
        assert_eq!(failures.len(), 2);
    }

    #[test]
    fn structural_failure_still_suppresses_euler() {
        // Genus corruption plus a structural defect (a dropped fin): the
        // counts are meaningless, so only the structural failures report.
        let (mut store, body, shell) = build_cube();
        store.shells.get_mut(shell).unwrap().genus = 1;
        let (edge, _) = store.edges.iter().next().unwrap();
        store.edges.get_mut(edge).unwrap().fins.pop();

        let failures = store.check(body);
        assert!(
            !failures
                .iter()
                .any(|f| matches!(f, CheckFailure::EulerViolation { .. })),
            "Euler must stay suppressed on a broken graph: {failures:?}"
        );
    }

    #[test]
    fn shared_vertex_between_shells_detected() {
        // Two shells of one body touching at a single vertex: directly
        // reported, not left for the Euler formula.
        let (mut store, body, shell_a) = build_cube();
        store.bodies.get_mut(body).unwrap().body_type = BodyType::General;
        let (edge, _) = store.edges.iter().next().unwrap();
        let shared = store.edges.get(edge).unwrap().start_vertex;

        let shell_b = store.create_shell(body, false, ShellOrientation::Outward);
        let face = store.create_face(shell_b, FaceSense::Positive);
        let w1 = store.create_vertex(p(5.0, 0.0, 0.0), SYSTEM_RESOLUTION);
        let w2 = store.create_vertex(p(5.0, 5.0, 0.0), SYSTEM_RESOLUTION);
        let tri = [
            store.create_edge(shared, w1, SYSTEM_RESOLUTION),
            store.create_edge(w1, w2, SYSTEM_RESOLUTION),
            store.create_edge(w2, shared, SYSTEM_RESOLUTION),
        ];
        store.create_loop(face, LoopType::Outer, &tri.map(|e| (e, FinSense::Forward)));

        assert_eq!(
            store.check(body),
            vec![CheckFailure::VertexSharedBetweenShells {
                vertex: shared,
                shell_a,
                shell_b,
            }]
        );
    }

    #[test]
    fn shared_edge_between_shells_detected() {
        // A second shell reusing a cube edge: the shared edge, its two
        // shared endpoints, and the resulting third fin all report.
        let (mut store, body, shell_a) = build_cube();
        store.bodies.get_mut(body).unwrap().body_type = BodyType::General;
        let (edge, e) = store.edges.iter().next().unwrap();
        let (v0, v1) = (e.start_vertex, e.end_vertex);

        let shell_b = store.create_shell(body, false, ShellOrientation::Outward);
        let face = store.create_face(shell_b, FaceSense::Positive);
        let w = store.create_vertex(p(5.0, 5.0, 5.0), SYSTEM_RESOLUTION);
        let e1w = store.create_edge(v1, w, SYSTEM_RESOLUTION);
        let ew0 = store.create_edge(w, v0, SYSTEM_RESOLUTION);
        store.create_loop(
            face,
            LoopType::Outer,
            &[
                (edge, FinSense::Forward),
                (e1w, FinSense::Forward),
                (ew0, FinSense::Forward),
            ],
        );

        let failures = store.check(body);
        assert!(failures.contains(&CheckFailure::EdgeSharedBetweenShells {
            edge,
            shell_a,
            shell_b,
        }));
        for vertex in [v0, v1] {
            assert!(
                failures.contains(&CheckFailure::VertexSharedBetweenShells {
                    vertex,
                    shell_a,
                    shell_b,
                }),
                "expected shared-vertex report for {vertex:?} in {failures:?}"
            );
        }
        assert!(failures.contains(&CheckFailure::NonManifoldEdge { edge, fins: 3 }));
        assert_eq!(failures.len(), 4);
    }

    // ------------------------------------------------------------------
    // Geometric checks
    // ------------------------------------------------------------------

    mod geometry {
        use super::*;
        use crate::curve::Curve3;
        use crate::pcurve::{Curve2, attach_body_pcurves};
        use crate::primitives;
        use crate::surface::Surface3;
        use opensolid_core::Vector3;
        use opensolid_core::types::{Point2, Vector2};

        /// Every primitive, with and without trim geometry attached. The
        /// builders promise exact geometry, so nothing here has any slack to
        /// spend: this is the baseline the defect tests perturb.
        pub(super) fn primitive_bodies()
        -> Vec<(&'static str, TopologyStore, GeometryStore, EntityId<Body>)> {
            let mut out = Vec::new();
            for (name, build) in [
                (
                    "block",
                    (|s: &mut TopologyStore, g: &mut GeometryStore| {
                        primitives::block(s, g, 2.0, 3.0, 4.0)
                    }) as fn(&mut TopologyStore, &mut GeometryStore) -> _,
                ),
                ("cylinder", |s, g| primitives::cylinder(s, g, 1.5, 4.0)),
                ("sphere", |s, g| primitives::sphere(s, g, 2.0)),
                ("torus", |s, g| primitives::torus(s, g, 3.0, 1.0)),
                ("cone", |s, g| primitives::cone(s, g, 2.0, 0.0, 3.0)),
                ("frustum", |s, g| primitives::cone(s, g, 2.0, 1.0, 3.0)),
            ] {
                let (mut store, mut geo) = (TopologyStore::new(), GeometryStore::new());
                let body = build(&mut store, &mut geo).expect("valid primitive");
                out.push((name, store, geo, body));
            }
            out
        }

        /// A block with trim geometry on every fin — the fixture the pcurve
        /// and face-sense checks need, since only bodies carrying pcurves
        /// have a readable parameter-space winding.
        fn block_with_pcurves() -> (TopologyStore, GeometryStore, EntityId<Body>) {
            let (mut store, mut geo) = (TopologyStore::new(), GeometryStore::new());
            let body = primitives::block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("valid block");
            let attached = attach_body_pcurves(&mut store, &mut geo, body);
            assert_eq!(attached, 24, "a block has 24 fins, all of them fittable");
            (store, geo, body)
        }

        fn first_face(store: &TopologyStore, body: EntityId<Body>) -> EntityId<Face> {
            store.faces_of_body(body)[0]
        }

        #[test]
        fn primitives_pass_the_geometric_check() {
            for (name, store, geo, body) in primitive_bodies() {
                assert_eq!(
                    store.check_geometry(&geo, body),
                    Vec::new(),
                    "{name} must pass the geometric check"
                );
                assert_eq!(
                    store.check_with_geometry(&geo, body),
                    Vec::new(),
                    "{name} must pass both checks"
                );
            }
        }

        /// The same primitives once trim geometry is derived for them: the
        /// pcurve and face-sense checks now have something to read, and must
        /// still find nothing.
        #[test]
        fn primitives_with_pcurves_pass_the_geometric_check() {
            for (name, mut store, mut geo, body) in primitive_bodies() {
                let attached = attach_body_pcurves(&mut store, &mut geo, body);
                assert!(attached > 0, "{name} must get some trim geometry");
                assert_eq!(
                    store.check_geometry(&geo, body),
                    Vec::new(),
                    "{name} with pcurves must pass the geometric check"
                );
            }
        }

        /// Coverage of the face-sense check across the primitives, and that
        /// it covers the *right* faces: flipping every sense flag on a body
        /// must be caught on exactly those faces whose loop has a readable
        /// winding, and that has to be nearly all of them or the check is
        /// only nominally there.
        ///
        /// The two it cannot read fail for one reason: an outer loop that is
        /// a seam run twice, with a parameterization singularity in between.
        /// The sphere's face is bounded only by its pole-to-pole meridian
        /// and the apex-capped cone's wall only by its apex-to-rim
        /// generator; either encloses nothing in parameter space, and the
        /// two ways to lift it disagree about which branch each run takes.
        #[test]
        fn face_sense_check_covers_the_primitives_it_can_read() {
            let readable: Vec<(&str, usize, usize)> = primitive_bodies()
                .into_iter()
                .map(|(name, mut store, mut geo, body)| {
                    attach_body_pcurves(&mut store, &mut geo, body);
                    let faces = store.faces_of_body(body);
                    for &face in &faces {
                        let face = store.faces.get_mut(face).expect("live face");
                        face.sense = match face.sense {
                            FaceSense::Positive => FaceSense::Negative,
                            FaceSense::Negative => FaceSense::Positive,
                        };
                    }
                    let caught = store
                        .check_geometry(&geo, body)
                        .iter()
                        .filter(|f| matches!(f, CheckFailure::FaceSenseContradictsLoop { .. }))
                        .count();
                    (name, faces.len(), caught)
                })
                .collect();
            assert_eq!(
                readable,
                vec![
                    ("block", 6, 6),
                    ("cylinder", 3, 3),
                    ("sphere", 1, 0),
                    ("torus", 1, 1),
                    ("cone", 2, 1),
                    ("frustum", 3, 3),
                ]
            );
        }

        /// The periodic lift, on the face that needs it: a cylinder wall's
        /// four boundary runs each pick their own branch of `u`, and only
        /// after they are unrolled does the loop read as the full
        /// `[0, 2π] × [0, height]` rectangle its face really is.
        #[test]
        fn a_seam_crossing_loop_winds_over_the_whole_period() {
            let (mut store, mut geo) = (TopologyStore::new(), GeometryStore::new());
            let (radius, height) = (1.5, 4.0);
            let body = primitives::cylinder(&mut store, &mut geo, radius, height).expect("valid");
            attach_body_pcurves(&mut store, &mut geo, body);
            let wall = store
                .faces_of_body(body)
                .into_iter()
                .find(|&f| {
                    let face = store.faces.get(f).expect("live face");
                    matches!(
                        geo.surface(face.surface.expect("surface")),
                        Some(Surface3::Cylinder { .. })
                    )
                })
                .expect("a cylinder has a wall");
            let face = store.faces.get(wall).expect("live face");
            let surface = geo.surface(face.surface.expect("surface")).expect("live");
            let winding = store
                .loop_winding(&geo, surface, face.outer_loop.expect("outer loop"))
                .expect("the wall's winding is readable");
            let expected = 2.0 * std::f64::consts::TAU * height;
            assert!(
                (winding - expected).abs() < 1e-9,
                "expected twice the parameter rectangle {expected}, got {winding}"
            );
        }

        /// A body whose geometry slots are empty is under construction, not
        /// invalid: the Euler-built cube carries no curves or surfaces at all
        /// and has nothing to measure.
        #[test]
        fn geometry_free_body_has_nothing_to_report() {
            let (store, body, _shell) = build_cube();
            let geo = GeometryStore::new();
            assert_eq!(store.check_geometry(&geo, body), Vec::new());
        }

        #[test]
        fn stale_body_reported_once_by_the_combined_check() {
            let (mut store, geo, body) = block_with_pcurves();
            store.bodies.remove(body);
            assert_eq!(
                store.check_with_geometry(&geo, body),
                vec![CheckFailure::StaleBody(body)]
            );
        }

        /// Sliding a face's plane off its own boundary puts all four of that
        /// face's edges off its surface — and only that face's, since each
        /// edge still lies on the neighbour it shares.
        #[test]
        fn edge_off_surface_detected() {
            let (mut store, mut geo, body) = block_with_pcurves();
            let face_id = first_face(&store, body);
            let face = store.faces.get(face_id).expect("live face");
            let old = geo
                .surface(face.surface.expect("block faces carry surfaces"))
                .expect("live surface")
                .clone();
            let Surface3::Plane { origin, normal } = old else {
                panic!("a block's faces are planar");
            };
            let moved = geo
                .add_surface(Surface3::plane(origin + normal * 0.1, normal).expect("valid plane"));
            store.faces.get_mut(face_id).expect("live face").surface = Some(moved);

            let failures = store.check_geometry(&geo, body);
            let off: Vec<_> = failures
                .iter()
                .filter_map(|f| match f {
                    CheckFailure::EdgeOffSurface {
                        edge,
                        face,
                        max_deviation,
                        ..
                    } => Some((*edge, *face, *max_deviation)),
                    _ => None,
                })
                .collect();
            assert_eq!(off.len(), 4, "one per edge of the moved face: {failures:?}");
            for (_, face, deviation) in &off {
                assert_eq!(*face, face_id);
                assert!(
                    (deviation - 0.1).abs() < 1e-9,
                    "the measured gap is the offset, got {deviation}"
                );
            }
        }

        /// The declared tolerance is what the deviation is judged against:
        /// the same displaced surface passes once every edge on it claims a
        /// tolerance that covers the gap.
        #[test]
        fn edge_tolerance_that_covers_the_gap_is_accepted() {
            let (mut store, mut geo, body) = block_with_pcurves();
            let face_id = first_face(&store, body);
            let face = store.faces.get(face_id).expect("live face");
            let Surface3::Plane { origin, normal } = geo
                .surface(face.surface.expect("surface"))
                .expect("live surface")
                .clone()
            else {
                panic!("a block's faces are planar");
            };
            // Well inside MAX_ALLOWED_TOLERANCE, so `check` stays quiet too.
            let gap = 1e-3;
            let moved =
                geo.add_surface(Surface3::plane(origin + normal * gap, normal).expect("plane"));
            store.faces.get_mut(face_id).expect("live face").surface = Some(moved);
            for edge in store.edges_of_face(face_id) {
                store.edges.get_mut(edge).expect("live edge").tolerance = gap * 2.0;
            }

            let failures = store.check_geometry(&geo, body);
            assert!(
                !failures
                    .iter()
                    .any(|f| matches!(f, CheckFailure::EdgeOffSurface { .. })),
                "a tolerance covering the gap admits it: {failures:?}"
            );
        }

        #[test]
        fn vertex_off_edge_detected() {
            let (mut store, geo, body) = block_with_pcurves();
            let vertex_id = store.vertices_of_face(first_face(&store, body))[0];
            let vertex = store.vertices.get_mut(vertex_id).expect("live vertex");
            vertex.point += Vector3::new(0.05, 0.0, 0.0);

            let failures = store.check_geometry(&geo, body);
            let off: Vec<_> = failures
                .iter()
                .filter_map(|f| match f {
                    CheckFailure::VertexOffEdge {
                        vertex, deviation, ..
                    } => Some((*vertex, *deviation)),
                    _ => None,
                })
                .collect();
            // Three edges meet at a block corner, and the vertex has moved
            // off the endpoint of every one of them.
            assert_eq!(off.len(), 3, "{failures:?}");
            for (vertex, deviation) in &off {
                assert_eq!(*vertex, vertex_id);
                assert!((deviation - 0.05).abs() < 1e-9, "got {deviation}");
            }
        }

        /// A moved vertex that declares a tolerance covering the move is a
        /// tolerant vertex, not a broken one — that is what the tolerance is
        /// for (`spec/08-tolerances.md` §7.1 invariant 2).
        #[test]
        fn vertex_tolerance_that_covers_the_move_is_accepted() {
            let (mut store, geo, body) = block_with_pcurves();
            let vertex_id = store.vertices_of_face(first_face(&store, body))[0];
            let vertex = store.vertices.get_mut(vertex_id).expect("live vertex");
            vertex.point += Vector3::new(1e-3, 0.0, 0.0);
            vertex.tolerance = 2e-3;

            assert!(
                !store
                    .check_geometry(&geo, body)
                    .iter()
                    .any(|f| matches!(f, CheckFailure::VertexOffEdge { .. }))
            );
        }

        /// A pcurve pointing somewhere else in parameter space no longer
        /// tracks its edge, however well-formed it is in itself.
        #[test]
        fn pcurve_deviation_detected() {
            let (mut store, mut geo, body) = block_with_pcurves();
            let face_id = first_face(&store, body);
            let loop_id = store.loops_of_face(face_id)[0];
            let fin_id = store.fins_of_loop(loop_id)[0];
            let edge_id = store.fin_edge(fin_id);
            let wrong = geo.add_pcurve(
                Curve2::line(Point2::new(10.0, 10.0), Vector2::x()).expect("valid pcurve"),
            );
            store.fins.get_mut(fin_id).expect("live fin").pcurve = Some(wrong);

            let failures = store.check_geometry(&geo, body);
            assert!(
                failures.iter().any(|f| matches!(
                    f,
                    CheckFailure::PcurveDeviation { fin, edge, .. }
                        if *fin == fin_id && *edge == edge_id
                )),
                "{failures:?}"
            );
        }

        /// A pcurve that traces the right path backwards satisfies neither
        /// the parameterization invariant nor the loop's winding.
        #[test]
        fn reversed_pcurve_detected() {
            let (mut store, mut geo, body) = block_with_pcurves();
            let face_id = first_face(&store, body);
            let loop_id = store.loops_of_face(face_id)[0];
            let fin_id = store.fins_of_loop(loop_id)[0];
            let edge_id = store.fin_edge(fin_id);
            let edge = store.edges.get(edge_id).expect("live edge");
            let (t_start, t_end) = (edge.t_start, edge.t_end);
            let original = geo
                .pcurve(
                    store
                        .fins
                        .get(fin_id)
                        .expect("live fin")
                        .pcurve
                        .expect("pcurve"),
                )
                .expect("live pcurve")
                .clone();
            // Same geometric path, run the other way over the same range.
            let reversed = Curve2::polyline(
                vec![t_start, t_end],
                vec![original.point(t_end), original.point(t_start)],
            )
            .expect("valid pcurve");
            let reversed = geo.add_pcurve(reversed);
            store.fins.get_mut(fin_id).expect("live fin").pcurve = Some(reversed);

            let failures = store.check_geometry(&geo, body);
            assert!(
                failures.iter().any(
                    |f| matches!(f, CheckFailure::PcurveDeviation { fin, .. } if *fin == fin_id)
                ),
                "{failures:?}"
            );
        }

        /// Flipping a face's sense flag without touching its geometry makes
        /// the flag disagree with the direction its own boundary runs.
        #[test]
        fn face_sense_contradicting_the_loop_winding_detected() {
            let (mut store, geo, body) = block_with_pcurves();
            let face_id = first_face(&store, body);
            store.faces.get_mut(face_id).expect("live face").sense = FaceSense::Negative;

            let failures = store.check_geometry(&geo, body);
            assert_eq!(
                failures
                    .iter()
                    .filter(|f| matches!(
                        f,
                        CheckFailure::FaceSenseContradictsLoop { face, sense, .. }
                            if *face == face_id && *sense == FaceSense::Negative
                    ))
                    .count(),
                1,
                "{failures:?}"
            );
            assert_eq!(failures.len(), 1, "nothing else moved: {failures:?}");
        }

        /// A face genuinely built the other way round — sense flag *and*
        /// boundary direction — is consistent, and reads as such.
        #[test]
        fn negative_sense_with_a_clockwise_loop_is_consistent() {
            let (mut store, mut geo) = (TopologyStore::new(), GeometryStore::new());
            let body = store.create_body(BodyType::Sheet);
            let shell = store.create_shell(body, false, ShellOrientation::Outward);
            // A unit square in the z = 0 plane, whose surface normal is -Z:
            // walking the square counterclockwise seen from +Z therefore
            // walks it *clockwise* in the surface's own (u, v) frame.
            let face = store.create_face(shell, FaceSense::Negative);
            let plane =
                geo.add_surface(Surface3::plane(p(0.0, 0.0, 0.0), -Vector3::z()).expect("plane"));
            store.faces.get_mut(face).expect("live face").surface = Some(plane);
            let corners = [
                p(0.0, 0.0, 0.0),
                p(1.0, 0.0, 0.0),
                p(1.0, 1.0, 0.0),
                p(0.0, 1.0, 0.0),
            ];
            let v: Vec<_> = corners
                .iter()
                .map(|&pt| store.create_vertex(pt, SYSTEM_RESOLUTION))
                .collect();
            let mut directed = Vec::new();
            for i in 0..4 {
                let (a, b) = (corners[i], corners[(i + 1) % 4]);
                let curve = geo.add_curve(Curve3::line(a, b - a).expect("valid line"));
                let edge = store.create_edge_with_curve(
                    v[i],
                    v[(i + 1) % 4],
                    SYSTEM_RESOLUTION,
                    curve,
                    0.0,
                    (b - a).norm(),
                );
                directed.push((edge, FinSense::Forward));
            }
            store.create_loop(face, LoopType::Outer, &directed);
            assert_eq!(attach_body_pcurves(&mut store, &mut geo, body), 4);

            assert_eq!(store.check_geometry(&geo, body), Vec::new());

            // ...and the same body with the flag alone flipped does not.
            store.faces.get_mut(face).expect("live face").sense = FaceSense::Positive;
            assert!(
                store
                    .check_geometry(&geo, body)
                    .iter()
                    .any(|f| matches!(f, CheckFailure::FaceSenseContradictsLoop { .. }))
            );
        }

        /// A fin without trim geometry leaves its loop's winding unreadable,
        /// and nothing is concluded from the fins that do have it.
        #[test]
        fn face_sense_is_not_judged_without_full_trim_geometry() {
            let (mut store, geo, body) = block_with_pcurves();
            let face_id = first_face(&store, body);
            let fin = store.fins_of_loop(store.loops_of_face(face_id)[0])[0];
            store.fins.get_mut(fin).expect("live fin").pcurve = None;
            store.faces.get_mut(face_id).expect("live face").sense = FaceSense::Negative;

            assert_eq!(store.check_geometry(&geo, body), Vec::new());
        }

        #[test]
        fn invalid_edge_range_detected() {
            let (mut store, geo, body) = block_with_pcurves();
            let edge_id = store.edges_of_face(first_face(&store, body))[0];
            let edge = store.edges.get_mut(edge_id).expect("live edge");
            let t_start = edge.t_start;
            edge.t_end = t_start;

            let failures = store.check_geometry(&geo, body);
            assert!(
                failures.contains(&CheckFailure::InvalidEdgeRange {
                    edge: edge_id,
                    t_start,
                    t_end: t_start,
                }),
                "{failures:?}"
            );
            // A range that cannot be sampled suppresses the measurements
            // that would sample it, rather than reporting garbage from them.
            assert!(
                !failures.iter().any(|f| matches!(
                    f,
                    CheckFailure::EdgeOffSurface { edge, .. } if *edge == edge_id
                )),
                "{failures:?}"
            );
        }

        /// The two passes are independent: a body with both a topological
        /// and a geometric defect reports both.
        #[test]
        fn combined_check_reports_topology_and_geometry_together() {
            let (mut store, geo, body) = block_with_pcurves();
            let face_id = first_face(&store, body);
            // Topological: a tolerance below the resolution floor.
            let edge_id = store.edges_of_face(face_id)[0];
            store.edges.get_mut(edge_id).expect("live edge").tolerance = 0.0;
            // Geometric: a sense flag disagreeing with the boundary.
            store.faces.get_mut(face_id).expect("live face").sense = FaceSense::Negative;

            let failures = store.check_with_geometry(&geo, body);
            assert!(failures.iter().any(|f| matches!(
                f,
                CheckFailure::InvalidTolerance {
                    entity: EntityRef::Edge(e),
                    ..
                } if *e == edge_id
            )));
            assert!(
                failures
                    .iter()
                    .any(|f| matches!(f, CheckFailure::FaceSenseContradictsLoop { .. }))
            );
        }

        /// Every geometric failure leaves the connectivity intact, so none of
        /// them suppresses the Euler–Poincaré formula.
        #[test]
        fn geometric_failures_are_not_structural() {
            let (store, _geo, body) = block_with_pcurves();
            let face = first_face(&store, body);
            let edge = store.edges_of_face(face)[0];
            let vertex = store.vertices_of_face(face)[0];
            let fin = store.fins_of_loop(store.loops_of_face(face)[0])[0];
            for failure in [
                CheckFailure::EdgeOffSurface {
                    edge,
                    face,
                    max_deviation: 1.0,
                    allowed: 0.0,
                },
                CheckFailure::VertexOffEdge {
                    vertex,
                    edge,
                    deviation: 1.0,
                    allowed: 0.0,
                },
                CheckFailure::PcurveDeviation {
                    fin,
                    edge,
                    max_deviation: 1.0,
                    allowed: 0.0,
                },
                CheckFailure::FaceSenseContradictsLoop {
                    face,
                    sense: FaceSense::Positive,
                    twice_signed_area: -1.0,
                },
                CheckFailure::InvalidEdgeRange {
                    edge,
                    t_start: 1.0,
                    t_end: 0.0,
                },
                CheckFailure::SelfIntersection {
                    face_a: face,
                    face_b: face,
                    at: Point3::origin(),
                },
            ] {
                assert!(!failure.is_structural(), "{failure:?}");
            }
        }

        /// A `Curve2::Polyline` is only held to the invariant at its own
        /// vertices — between them it is a chord, a documented approximation
        /// of freeform trim rather than a defect.
        #[test]
        fn polyline_pcurve_is_checked_at_its_own_vertices() {
            let pcurve =
                Curve2::polyline(vec![0.0, 0.5, 1.0], vec![Point2::origin(); 3]).expect("polyline");
            assert_eq!(pcurve_check_params(&pcurve, 0.0, 1.0), vec![0.0, 0.5, 1.0]);
            // ...unless its parameters do not reach the edge's range at all,
            // where the mismatch is the thing worth measuring.
            assert_eq!(
                pcurve_check_params(&pcurve, 10.0, 11.0),
                edge_samples(10.0, 11.0).collect::<Vec<_>>()
            );
        }
    }

    mod self_intersection {
        use super::*;
        use crate::curve::Curve3;
        use crate::primitives;
        use crate::surface::Surface3;

        /// An empty sheet body to hang hand-built faces off.
        fn sheet() -> (
            TopologyStore,
            GeometryStore,
            EntityId<Body>,
            EntityId<Shell>,
        ) {
            let (mut store, geo) = (TopologyStore::new(), GeometryStore::new());
            let body = store.create_body(BodyType::Sheet);
            let shell = store.create_shell(body, false, ShellOrientation::Outward);
            (store, geo, body, shell)
        }

        fn vertex(store: &mut TopologyStore, at: Point3) -> EntityId<Vertex> {
            store.create_vertex(at, SYSTEM_RESOLUTION)
        }

        /// A straight edge from `from` to `to`, parameterized by arc length.
        fn segment(
            store: &mut TopologyStore,
            geo: &mut GeometryStore,
            (v0, from): (EntityId<Vertex>, Point3),
            (v1, to): (EntityId<Vertex>, Point3),
        ) -> EntityId<Edge> {
            let span = to - from;
            let curve = geo.add_curve(Curve3::line(from, span).expect("a non-degenerate segment"));
            store.create_edge_with_curve(v0, v1, SYSTEM_RESOLUTION, curve, 0.0, span.norm())
        }

        /// A planar triangular face on `shell`, bounded by the given directed
        /// edges. `pts` are the corners in traversal order, which fixes the
        /// plane and its normal: the loop then runs counterclockwise seen
        /// from the normal side, as a `Positive` face on an outward shell
        /// must.
        fn triangle(
            store: &mut TopologyStore,
            geo: &mut GeometryStore,
            shell: EntityId<Shell>,
            pts: [Point3; 3],
            fins: [(EntityId<Edge>, FinSense); 3],
        ) -> EntityId<Face> {
            let normal = (pts[1] - pts[0]).cross(&(pts[2] - pts[0]));
            let surface =
                geo.add_surface(Surface3::plane(pts[0], normal).expect("a real triangle"));
            let face = store.create_face(shell, FaceSense::Positive);
            store.faces.get_mut(face).expect("live face").surface = Some(surface);
            store.create_loop(face, LoopType::Outer, &fins);
            face
        }

        /// A triangle with fresh vertices and edges of its own — sharing
        /// nothing with anything already on the shell.
        fn free_triangle(
            store: &mut TopologyStore,
            geo: &mut GeometryStore,
            shell: EntityId<Shell>,
            pts: [Point3; 3],
        ) -> EntityId<Face> {
            let v: Vec<_> = pts.iter().map(|&pt| vertex(store, pt)).collect();
            let fins = [0, 1, 2].map(|i| {
                let j = (i + 1) % 3;
                (
                    segment(store, geo, (v[i], pts[i]), (v[j], pts[j])),
                    FinSense::Forward,
                )
            });
            triangle(store, geo, shell, pts, fins)
        }

        fn clashes(failures: &[CheckFailure]) -> Vec<(EntityId<Face>, EntityId<Face>)> {
            failures
                .iter()
                .filter_map(|f| match f {
                    CheckFailure::SelfIntersection { face_a, face_b, .. } => {
                        Some((*face_a, *face_b))
                    }
                    _ => None,
                })
                .collect()
        }

        /// The baseline the whole check rests on: every primitive is built
        /// from faces that meet each other constantly — a cylinder's wall
        /// touches both its caps along their whole rims — and not one of
        /// those contacts is a defect, because every one of them is a shared
        /// edge.
        #[test]
        fn primitives_have_no_self_intersections() {
            for (name, store, geo, body) in super::geometry::primitive_bodies() {
                assert_eq!(
                    store.check_self_intersection(&geo, body),
                    Vec::new(),
                    "{name} must have no self-intersections"
                );
            }
        }

        #[test]
        fn faces_that_cross_are_reported() {
            let (mut store, mut geo, body, shell) = sheet();
            let flat = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0), p(0.0, 4.0, 0.0)],
            );
            // Vertical, and passing clean through the flat one's interior.
            let blade = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(1.0, 1.0, -1.0), p(3.0, 1.0, -1.0), p(1.0, 1.0, 2.0)],
            );

            assert_eq!(
                clashes(&store.check_self_intersection(&geo, body)),
                vec![(flat, blade)]
            );
        }

        #[test]
        fn faces_that_stay_apart_are_not_reported() {
            let (mut store, mut geo, body, shell) = sheet();
            free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0)],
            );
            free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 5.0), p(1.0, 0.0, 5.0), p(0.0, 1.0, 5.0)],
            );
            assert_eq!(store.check_self_intersection(&geo, body), Vec::new());
        }

        /// The distinction the whole check turns on: two faces meeting along
        /// an edge they *share* is what a body is made of, and the same two
        /// surfaces meeting anywhere else is a defect. Here the shared edge
        /// is the only contact.
        #[test]
        fn faces_meeting_along_a_shared_edge_are_not_reported() {
            let (mut store, mut geo, body, shell) = sheet();
            let (a, b, c) = (p(0.0, 0.0, 0.0), p(2.0, 0.0, 0.0), p(0.0, 2.0, 0.0));
            let up = p(0.0, 0.0, 2.0);
            let (va, vb, vc, vup) = (
                vertex(&mut store, a),
                vertex(&mut store, b),
                vertex(&mut store, c),
                vertex(&mut store, up),
            );
            // The hinge, used forward by one face and reversed by the other.
            let hinge = segment(&mut store, &mut geo, (va, a), (vb, b));
            let bc = segment(&mut store, &mut geo, (vb, b), (vc, c));
            let ca = segment(&mut store, &mut geo, (vc, c), (va, a));
            let a_up = segment(&mut store, &mut geo, (va, a), (vup, up));
            let up_b = segment(&mut store, &mut geo, (vup, up), (vb, b));
            let flat = triangle(
                &mut store,
                &mut geo,
                shell,
                [a, b, c],
                [
                    (hinge, FinSense::Forward),
                    (bc, FinSense::Forward),
                    (ca, FinSense::Forward),
                ],
            );
            let folded = triangle(
                &mut store,
                &mut geo,
                shell,
                [b, a, up],
                [
                    (hinge, FinSense::Reversed),
                    (a_up, FinSense::Forward),
                    (up_b, FinSense::Forward),
                ],
            );

            let failures = store.check_self_intersection(&geo, body);
            assert_eq!(failures, Vec::new(), "{flat:?}/{folded:?}: {failures:?}");
        }

        /// A face's edge lying *in* another face, sharing nothing with it:
        /// the contact locus is on one face's boundary, where a strict
        /// interior test is a coin flip, and it is still a defect — the two
        /// faces touch along topology neither of them owns.
        #[test]
        fn an_edge_lying_in_another_face_is_reported() {
            let (mut store, mut geo, body, shell) = sheet();
            let flat = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0), p(0.0, 4.0, 0.0)],
            );
            // Stands *on* the flat face: its bottom edge is inside it.
            let fin = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(1.0, 1.0, 0.0), p(2.0, 1.0, 0.0), p(1.0, 1.0, 1.0)],
            );

            assert_eq!(
                clashes(&store.check_self_intersection(&geo, body)),
                vec![(flat, fin)]
            );
        }

        /// Two faces on the *same* surface: there is no intersection curve to
        /// walk, so the overlap is hunted over the region itself.
        #[test]
        fn overlapping_coplanar_faces_are_reported() {
            let (mut store, mut geo, body, shell) = sheet();
            let lower = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0), p(0.0, 4.0, 0.0)],
            );
            let overlapping = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(1.0, 1.0, 0.0), p(5.0, 1.0, 0.0), p(1.0, 5.0, 0.0)],
            );

            assert_eq!(
                clashes(&store.check_self_intersection(&geo, body)),
                vec![(lower, overlapping)]
            );
        }

        /// A coplanar face wholly *inside* another: the containing face's
        /// grid could step clean over it, so both regions are swept.
        #[test]
        fn a_coplanar_face_buried_inside_another_is_reported() {
            let (mut store, mut geo, body, shell) = sheet();
            let large = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 0.0), p(40.0, 0.0, 0.0), p(0.0, 40.0, 0.0)],
            );
            let buried = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(5.0, 5.0, 0.0), p(5.4, 5.0, 0.0), p(5.0, 5.4, 0.0)],
            );

            assert_eq!(
                clashes(&store.check_self_intersection(&geo, body)),
                vec![(large, buried)]
            );
        }

        /// Coplanar but disjoint: the same `Coincident` surface pair, with
        /// nothing to report. The regions, not the surfaces, decide.
        #[test]
        fn coplanar_faces_that_do_not_overlap_are_not_reported() {
            let (mut store, mut geo, body, shell) = sheet();
            free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0)],
            );
            free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(5.0, 5.0, 0.0), p(6.0, 5.0, 0.0), p(5.0, 6.0, 0.0)],
            );
            assert_eq!(store.check_self_intersection(&geo, body), Vec::new());
        }

        /// A curved narrow phase, on a periodic chart: a blade driven into a
        /// cylinder (centred on the origin, so its caps sit at `z = ±1`)
        /// enters through the wall and leaves through the top cap, missing
        /// the bottom entirely. The wall contact is an ellipse that has to
        /// be classified in the cylinder's wrapped parameter cover.
        #[test]
        fn a_blade_through_a_cylinder_is_reported() {
            let (mut store, mut geo) = (TopologyStore::new(), GeometryStore::new());
            let body = primitives::cylinder(&mut store, &mut geo, 1.0, 2.0).expect("a cylinder");
            let shell = store.shells_of_body(body)[0];
            let blade = free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(-2.0, 0.0, 0.5), p(2.0, 0.0, 0.5), p(-2.0, 0.0, 1.5)],
            );

            let reported = clashes(&store.check_self_intersection(&geo, body));
            assert!(
                reported.iter().all(|&(_, b)| b == blade),
                "every clash is against the blade: {reported:?}"
            );
            let hit: Vec<Surface3> = reported
                .iter()
                .map(|&(face, _)| {
                    let surface = store
                        .face(face)
                        .expect("live face")
                        .surface
                        .expect("surface");
                    geo.surface(surface).expect("live surface").clone()
                })
                .collect();
            assert_eq!(hit.len(), 2, "the wall and the top cap: {hit:?}");
            assert!(
                hit.iter().any(|s| matches!(s, Surface3::Cylinder { .. })),
                "the wall is pierced: {hit:?}"
            );
            assert!(
                hit.iter()
                    .any(|s| matches!(s, Surface3::Plane { origin, .. } if origin.z > 0.0)),
                "the top cap is pierced: {hit:?}"
            );
        }

        /// The bounding-box prune has to be conservative about *curvature*,
        /// not just about rims: a sphere's face bulges a long way past the
        /// box its boundary spans, and a prune that believed the rim would
        /// drop the pair before the narrow phase ever saw it.
        #[test]
        fn a_face_box_covers_the_bulge_between_its_rims() {
            let (mut store, mut geo) = (TopologyStore::new(), GeometryStore::new());
            let radius = 2.0;
            let body = primitives::sphere(&mut store, &mut geo, radius).expect("a sphere");
            let face = store.faces_of_body(body)[0];
            let tol = ToleranceContext::default();
            let patch = store
                .face_patch(&geo, &tol, face, true)
                .expect("a sphere face is measurable");

            // The seam meridian the face is bounded by spans x >= 0 only,
            // yet the sphere itself reaches -radius.
            assert!(
                patch.bbox.min.x <= -radius && patch.bbox.max.x >= radius,
                "the box must contain the whole sphere, got {:?}",
                patch.bbox
            );
        }

        /// Geometry the check cannot read is skipped, not guessed at: the
        /// Euler-built cube has no surfaces or curves at all.
        #[test]
        fn a_body_without_geometry_has_nothing_to_pair() {
            let (store, body, _shell) = build_cube();
            assert_eq!(
                store.check_self_intersection(&GeometryStore::new(), body),
                Vec::new()
            );
        }

        #[test]
        fn stale_body_is_reported() {
            let (mut store, mut geo, body, shell) = sheet();
            free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 0.0), p(1.0, 0.0, 0.0), p(0.0, 1.0, 0.0)],
            );
            store.bodies.remove(body);
            assert_eq!(
                store.check_self_intersection(&geo, body),
                vec![CheckFailure::StaleBody(body)]
            );
        }

        /// A clash reaches the combined entry point too — the pass is wired
        /// in, not merely available.
        #[test]
        fn the_combined_check_reports_a_clash() {
            let (mut store, mut geo, body, shell) = sheet();
            free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(0.0, 0.0, 0.0), p(4.0, 0.0, 0.0), p(0.0, 4.0, 0.0)],
            );
            free_triangle(
                &mut store,
                &mut geo,
                shell,
                [p(1.0, 1.0, -1.0), p(3.0, 1.0, -1.0), p(1.0, 1.0, 2.0)],
            );
            assert!(
                store
                    .check_with_geometry(&geo, body)
                    .iter()
                    .any(|f| matches!(f, CheckFailure::SelfIntersection { .. })),
                "the combined check runs the self-intersection pass"
            );
        }
    }
}
