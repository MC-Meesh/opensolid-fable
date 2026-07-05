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
//!   their edges and edges on their endpoint vertices; no empty shells,
//!   loop-less faces, or fin-less non-vertex loops.
//! - **Loop connectivity**: `next`/`prev` links agree with each loop's fin
//!   order, and each fin's end vertex is the next fin's start vertex.
//! - **Closure and manifoldness**: every edge of a shell that must be
//!   closed — flagged `is_closed`, or any shell of a
//!   [`BodyType::Solid`](crate::BodyType::Solid) body — has exactly two
//!   fins; no edge anywhere has more than two. The producer-supplied
//!   `is_closed` flag is not trusted: a solid shell flagged open fails
//!   outright, and a flagged-open shell whose every edge is two-fin is
//!   reported as inconsistent regardless of body type.
//! - **Orientation consistency**: the two fins of a manifold edge traverse
//!   it in opposite directions. This is the topological form of "adjacent
//!   faces are consistently oriented"; [`FaceSense`](crate::FaceSense)
//!   relates face normals to *surface* normals and needs the geometry layer.
//! - **Tolerance sanity**: every edge/vertex tolerance is finite, at least
//!   [`SYSTEM_RESOLUTION`], and at most [`MAX_ALLOWED_TOLERANCE`]; vertex
//!   points are finite.
//! - **Euler–Poincaré formula** `V - E + F - R = 2(S - H)`: checked only
//!   when every other check passed *and* all shells are closed — the formula
//!   applies to closed surfaces, and on a structurally broken graph the
//!   counts are meaningless noise. For solid bodies this bypass is never
//!   silent: a solid with a non-closed shell has already failed the closure
//!   checks above, so the formula is only ever skipped on a body that
//!   reports at least one other failure.
//!
//! Geometric checks from the spec (edge-on-surface, vertex-on-edge,
//! self-intersection) are deferred until faces and edges carry real
//! geometry.

use crate::euler::EulerCounts;
use crate::topology::{
    Body, BodyType, Edge, Face, Fin, FinSense, Loop, SYSTEM_RESOLUTION, Shell, TopologyStore,
    Vertex,
};
use opensolid_core::EntityId;
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

    /// A face with no outer loop.
    #[error("face {0:?} has no outer loop")]
    FaceWithoutOuterLoop(EntityId<Face>),

    /// A loop with neither fins nor a degenerate-loop vertex.
    #[error("loop {0:?} has neither fins nor a vertex")]
    EmptyLoop(EntityId<Loop>),

    /// A loop with both fins and a degenerate-loop vertex.
    #[error("loop {0:?} has both fins and a degenerate-loop vertex")]
    VertexLoopWithFins(EntityId<Loop>),

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

        // Entities reachable from the body, deduplicated, for the
        // edge/vertex-level passes below.
        let mut edges: Vec<EntityId<Edge>> = Vec::new();
        let mut vertices: Vec<EntityId<Vertex>> = Vec::new();

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
            for &face_id in &shell.faces {
                self.check_face(
                    shell_id,
                    face_id,
                    &mut failures,
                    &mut shell_edges,
                    &mut vertices,
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
            for edge_id in shell_edges {
                push_unique(&mut edges, edge_id);
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
            }
        }

        // The Euler–Poincaré formula only applies to closed surfaces, and
        // on a broken graph the counts are meaningless — so it is checked
        // last, only when everything else passed and all shells are closed.
        // Skipping it is never silent for solids: a solid with a non-closed
        // shell already failed the closure checks above.
        let all_closed = b
            .shells
            .iter()
            .all(|&s| self.shells.get(s).is_some_and(|shell| shell.is_closed));
        if failures.is_empty() && all_closed {
            let counts = self.euler_counts(body);
            if !counts.euler_poincare_holds() {
                failures.push(CheckFailure::EulerViolation { body, counts });
            }
        }

        failures
    }

    /// Face-level checks: back-pointers, loop presence, loop connectivity,
    /// fin links and mates. Collects reachable edges and degenerate-loop
    /// vertices.
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

        for loop_id in face
            .outer_loop
            .into_iter()
            .chain(face.inner_loops.iter().copied())
        {
            let Some(lp) = self.loops.get(loop_id) else {
                failures.push(CheckFailure::StaleReference {
                    from: EntityRef::Face(face_id),
                    to: EntityRef::Loop(loop_id),
                });
                continue;
            };
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
}
