//! Euler operators: the primitive topology-modifying operations
//! (`spec/03-topology.md` §5).
//!
//! Every operator preserves the Euler-Poincaré formula
//!
//! ```text
//! V - E + F - R = 2(S - H)
//! ```
//!
//! where `V`/`E`/`F` are vertex/edge/face counts, `R` is the number of rings
//! (loops beyond each face's first — inner loops), `S` the number of shells,
//! and `H` the total genus (through-holes) summed over shells. In debug
//! builds every operator re-validates the affected body after mutating:
//! the formula above plus the half-edge invariants (loop cycles are closed
//! and vertex-continuous, `next`/`prev` are inverses, mates are mutual and
//! share an edge). A violation is a kernel bug, so it panics rather than
//! returning an error.
//!
//! Operator inventory and its effect on the formula terms:
//!
//! | Op      | Effect                                 | Δ(V,E,F,R,S,H)        |
//! |---------|----------------------------------------|-----------------------|
//! | `mvfs`  | seed body: vertex + face + shell       | (+1, 0, +1, 0, +1, 0) |
//! | `mev`   | spur edge to a new vertex              | (+1, +1, 0, 0, 0, 0)  |
//! | `mef`   | split a face with a new edge           | (0, +1, +1, 0, 0, 0)  |
//! | `kemr`  | kill edge, split its loop into a ring  | (0, -1, 0, +1, 0, 0)  |
//! | `kfmrh` | kill face, its loop becomes a ring; adds a handle (same shell) or merges two shells | (0, 0, -1, +1, 0, +1) or (0, 0, -1, +1, -1, 0) |
//!
//! Invalid applications (stale ids, vertices not on the face, edges whose
//! fins span two loops, ...) are rejected with [`EulerError`] before any
//! mutation happens: an `Err` return leaves the store untouched.

use crate::topology::{
    Body, BodyType, Edge, Face, FaceSense, Fin, FinSense, Loop, LoopType, SYSTEM_RESOLUTION, Shell,
    ShellOrientation, TopologyStore, Vertex,
};
use opensolid_core::{EntityId, Point3};
use std::collections::HashSet;
use thiserror::Error;

/// Errors from invalid Euler-operator applications.
///
/// Any `Err` is returned before the store is mutated, so a failed operator
/// leaves the topology exactly as it was.
#[derive(Debug, Error, PartialEq)]
pub enum EulerError {
    #[error("stale vertex id {0:?}")]
    StaleVertex(EntityId<Vertex>),
    #[error("stale face id {0:?}")]
    StaleFace(EntityId<Face>),
    #[error("stale edge id {0:?}")]
    StaleEdge(EntityId<Edge>),
    #[error("vertex {vertex:?} does not lie on face {face:?}")]
    VertexNotOnFace {
        vertex: EntityId<Vertex>,
        face: EntityId<Face>,
    },
    #[error("mef requires two distinct vertices, got {0:?} twice")]
    MefSameVertex(EntityId<Vertex>),
    #[error("mef vertices {vertex_a:?} and {vertex_b:?} lie on different loops of face {face:?}")]
    MefVerticesInDifferentLoops {
        vertex_a: EntityId<Vertex>,
        vertex_b: EntityId<Vertex>,
        face: EntityId<Face>,
    },
    #[error("kemr edge {edge:?} has {fins} fin(s), needs exactly 2")]
    KemrWrongFinCount { edge: EntityId<Edge>, fins: usize },
    #[error("kemr edge {0:?} has its fins on two different loops (not a loop-splitting edge)")]
    KemrFinsInDifferentLoops(EntityId<Edge>),
    #[error("kemr on edge {0:?} would leave an empty loop (use a kill-edge-vertex op instead)")]
    KemrDegenerate(EntityId<Edge>),
    #[error("kfmrh target and killed face are the same face {0:?}")]
    KfmrhSameFace(EntityId<Face>),
    #[error("kfmrh face {face:?} still carries {rings} ring(s); kill those first")]
    KfmrhFaceHasRings { face: EntityId<Face>, rings: usize },
    #[error("kfmrh face {0:?} has no outer loop")]
    KfmrhFaceWithoutLoop(EntityId<Face>),
    #[error("kfmrh faces {target:?} and {killed:?} belong to different bodies")]
    KfmrhDifferentBodies {
        target: EntityId<Face>,
        killed: EntityId<Face>,
    },
    #[error("topology invariant violated: {0}")]
    InvariantViolation(String),
}

/// Entity counts of one body, the terms of the Euler-Poincaré formula.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct EulerCounts {
    pub vertices: usize,
    pub edges: usize,
    pub faces: usize,
    pub loops: usize,
    /// Rings: loops beyond one per face (`loops - faces`).
    pub rings: usize,
    pub shells: usize,
    /// Total genus (through-holes) summed over the body's shells.
    pub genus: usize,
}

impl EulerCounts {
    /// Whether `V - E + F - R = 2(S - H)` holds for these counts.
    pub fn euler_poincare_holds(&self) -> bool {
        let lhs = self.vertices as i64 - self.edges as i64 + self.faces as i64 - self.rings as i64;
        let rhs = 2 * (self.shells as i64 - self.genus as i64);
        lhs == rhs
    }
}

/// Where `mev` attaches the new spur edge on the face.
enum MevAttach {
    /// The face's loop is a degenerate vertex loop at the given vertex; the
    /// spur upgrades it to a real two-fin loop.
    VertexLoop(EntityId<Loop>),
    /// Insert the spur before the fin at `index` of the loop's fin cycle
    /// (that fin starts at the attachment vertex).
    BeforeFin(EntityId<Loop>, usize),
}

impl TopologyStore {
    // ------------------------------------------------------------------
    // Operators
    // ------------------------------------------------------------------

    /// Make Vertex Face Shell: seed a new body with one vertex, one shell,
    /// and one face whose outer loop is a degenerate vertex loop.
    ///
    /// The starting point of every Euler-operator construction sequence.
    pub fn mvfs(
        &mut self,
        point: Point3,
    ) -> (
        EntityId<Body>,
        EntityId<Vertex>,
        EntityId<Face>,
        EntityId<Shell>,
    ) {
        let body = self.create_body(BodyType::Solid);
        let shell = self.create_shell(body, true, ShellOrientation::Outward);
        let face = self.create_face(shell, FaceSense::Positive);
        let vertex = self.create_vertex(point, SYSTEM_RESOLUTION);
        let loop_id = self.loops.insert(Loop {
            face,
            fins: Vec::new(),
            loop_type: LoopType::Vertex,
            vertex: Some(vertex),
        });
        self.faces.get_mut(face).expect("just created").outer_loop = Some(loop_id);

        self.debug_check_invariants(body);
        (body, vertex, face, shell)
    }

    /// Make Edge Vertex: attach a spur edge from `vertex` (which must lie on
    /// `face`) to a new vertex at `point`.
    ///
    /// The new edge runs `vertex → new vertex` and appears twice in the
    /// loop (out and back), so the loop stays closed. If `vertex` occurs
    /// more than once on the face, the spur attaches before the first fin
    /// (in loop order) that starts at it.
    pub fn mev(
        &mut self,
        vertex: EntityId<Vertex>,
        face: EntityId<Face>,
        point: Point3,
    ) -> Result<(EntityId<Edge>, EntityId<Vertex>), EulerError> {
        if self.vertices.get(vertex).is_none() {
            return Err(EulerError::StaleVertex(vertex));
        }
        if self.faces.get(face).is_none() {
            return Err(EulerError::StaleFace(face));
        }

        let attach = self
            .find_mev_attachment(face, vertex)
            .ok_or(EulerError::VertexNotOnFace { vertex, face })?;

        let new_vertex = self.create_vertex(point, SYSTEM_RESOLUTION);
        let edge = self.create_edge(vertex, new_vertex, SYSTEM_RESOLUTION);

        let loop_id = match attach {
            MevAttach::VertexLoop(l) | MevAttach::BeforeFin(l, _) => l,
        };
        let fin_out = self.insert_fin(edge, loop_id, FinSense::Forward);
        let fin_back = self.insert_fin(edge, loop_id, FinSense::Reversed);
        self.register_fins_on_edge(edge, fin_out, fin_back);

        match attach {
            MevAttach::VertexLoop(l) => {
                let is_outer = self.faces.get(face).expect("checked").outer_loop == Some(l);
                let lp = self.loops.get_mut(l).expect("loop of live face");
                lp.fins = vec![fin_out, fin_back];
                lp.vertex = None;
                lp.loop_type = if is_outer {
                    LoopType::Outer
                } else {
                    LoopType::Inner
                };
            }
            MevAttach::BeforeFin(l, index) => {
                let lp = self.loops.get_mut(l).expect("loop of live face");
                lp.fins.insert(index, fin_back);
                lp.fins.insert(index, fin_out);
            }
        }
        self.relink_loop(loop_id);

        self.debug_check_invariants(self.body_of_face(face));
        Ok((edge, new_vertex))
    }

    /// Make Edge Face: split `face` by a new edge from `vertex_a` to
    /// `vertex_b`, which must both lie on the same loop of the face.
    ///
    /// The loop's chain from `vertex_a` to `vertex_b` moves to a new face
    /// (returned); the chain from `vertex_b` back to `vertex_a` stays on
    /// `face`. If a vertex occurs more than once on the loop, its first
    /// occurrence (in loop order) is used.
    pub fn mef(
        &mut self,
        vertex_a: EntityId<Vertex>,
        vertex_b: EntityId<Vertex>,
        face: EntityId<Face>,
    ) -> Result<(EntityId<Edge>, EntityId<Face>), EulerError> {
        if self.vertices.get(vertex_a).is_none() {
            return Err(EulerError::StaleVertex(vertex_a));
        }
        if self.vertices.get(vertex_b).is_none() {
            return Err(EulerError::StaleVertex(vertex_b));
        }
        if self.faces.get(face).is_none() {
            return Err(EulerError::StaleFace(face));
        }
        if vertex_a == vertex_b {
            return Err(EulerError::MefSameVertex(vertex_a));
        }

        // Locate both vertices; they must sit on the same loop.
        let loc_a = self.find_fin_starting_at(face, vertex_a);
        let loc_b = self.find_fin_starting_at(face, vertex_b);
        let ((loop_id, ia), (loop_b, ib)) = match (loc_a, loc_b) {
            (Some(a), Some(b)) => (a, b),
            (None, _) => {
                return Err(EulerError::VertexNotOnFace {
                    vertex: vertex_a,
                    face,
                });
            }
            (_, None) => {
                return Err(EulerError::VertexNotOnFace {
                    vertex: vertex_b,
                    face,
                });
            }
        };
        if loop_id != loop_b {
            return Err(EulerError::MefVerticesInDifferentLoops {
                vertex_a,
                vertex_b,
                face,
            });
        }

        let edge = self.create_edge(vertex_a, vertex_b, SYSTEM_RESOLUTION);

        // Split the fin cycle at the two vertices. Rotated to start at
        // vertex_a's fin, the first k fins walk a → b and move to the new
        // face; the rest walk b → a and stay.
        let fins = self.loops.get(loop_id).expect("located above").fins.clone();
        let n = fins.len();
        let k = (ib + n - ia) % n;
        let rotated = |offset: usize| fins[(ia + offset) % n];
        let moved: Vec<EntityId<Fin>> = (0..k).map(rotated).collect();
        let kept: Vec<EntityId<Fin>> = (k..n).map(rotated).collect();

        let shell = self.faces.get(face).expect("checked").shell;
        let sense = self.faces.get(face).expect("checked").sense;
        let new_face = self.create_face(shell, sense);
        let new_loop = self.loops.insert(Loop {
            face: new_face,
            fins: Vec::new(),
            loop_type: LoopType::Outer,
            vertex: None,
        });
        self.faces
            .get_mut(new_face)
            .expect("just created")
            .outer_loop = Some(new_loop);

        // New edge's fins: forward (a → b) closes the kept chain, reversed
        // (b → a) closes the moved chain.
        let fin_kept = self.insert_fin(edge, loop_id, FinSense::Forward);
        let fin_moved = self.insert_fin(edge, new_loop, FinSense::Reversed);
        self.register_fins_on_edge(edge, fin_kept, fin_moved);

        for &f in &moved {
            self.fins.get_mut(f).expect("live fin").loop_ref = new_loop;
        }
        let mut new_fins = moved;
        new_fins.push(fin_moved);
        self.loops.get_mut(new_loop).expect("just created").fins = new_fins;
        let mut old_fins = kept;
        old_fins.push(fin_kept);
        self.loops.get_mut(loop_id).expect("located above").fins = old_fins;
        self.relink_loop(new_loop);
        self.relink_loop(loop_id);

        self.debug_check_invariants(self.body_of_face(face));
        Ok((edge, new_face))
    }

    /// Kill Edge Make Ring: delete an edge whose two fins lie on the *same*
    /// loop, splitting that loop in two; the second piece becomes a ring
    /// (inner loop) of the same face.
    ///
    /// The ring is the chain of fins that follows the edge's first fin
    /// (`edge.fins[0]`) in loop order; the other chain keeps the original
    /// loop's identity. Both chains must be non-empty (killing a spur edge
    /// is a kill-edge-vertex operation, not KEMR).
    pub fn kemr(&mut self, edge: EntityId<Edge>) -> Result<EntityId<Loop>, EulerError> {
        let e = self.edges.get(edge).ok_or(EulerError::StaleEdge(edge))?;
        if e.fins.len() != 2 {
            return Err(EulerError::KemrWrongFinCount {
                edge,
                fins: e.fins.len(),
            });
        }
        let (fin_1, fin_2) = (e.fins[0], e.fins[1]);
        let (start_vertex, end_vertex) = (e.start_vertex, e.end_vertex);
        let loop_id = self.fin_loop(fin_1);
        if self.fin_loop(fin_2) != loop_id {
            return Err(EulerError::KemrFinsInDifferentLoops(edge));
        }

        let fins = &self.loops.get(loop_id).expect("live loop").fins;
        let n = fins.len();
        let i1 = fins.iter().position(|&f| f == fin_1).expect("fin in loop");
        let i2 = fins.iter().position(|&f| f == fin_2).expect("fin in loop");
        // The two chains strictly between the edge's fins. Both are closed
        // (the killed fins share both endpoints).
        let ring_fins: Vec<EntityId<Fin>> = (1..(i2 + n - i1) % n)
            .map(|off| fins[(i1 + off) % n])
            .collect();
        let kept_fins: Vec<EntityId<Fin>> = (1..(i1 + n - i2) % n)
            .map(|off| fins[(i2 + off) % n])
            .collect();
        if ring_fins.is_empty() || kept_fins.is_empty() {
            return Err(EulerError::KemrDegenerate(edge));
        }

        let face = self.loops.get(loop_id).expect("live loop").face;
        let ring = self.loops.insert(Loop {
            face,
            fins: ring_fins.clone(),
            loop_type: LoopType::Inner,
            vertex: None,
        });
        for &f in &ring_fins {
            self.fins.get_mut(f).expect("live fin").loop_ref = ring;
        }
        self.loops.get_mut(loop_id).expect("live loop").fins = kept_fins;
        self.relink_loop(ring);
        self.relink_loop(loop_id);
        self.faces
            .get_mut(face)
            .expect("live face")
            .inner_loops
            .push(ring);

        // Delete the edge and its fins; unregister from endpoint vertices.
        self.fins.remove(fin_1);
        self.fins.remove(fin_2);
        self.edges.remove(edge);
        for v in [start_vertex, end_vertex] {
            self.vertices
                .get_mut(v)
                .expect("live vertex")
                .edges
                .retain(|&e| e != edge);
        }

        self.debug_check_invariants(self.body_of_face(face));
        Ok(ring)
    }

    /// Kill Face Make Ring Hole: delete `killed` and re-home its outer loop
    /// as a ring (inner loop) of `target`.
    ///
    /// If both faces are on the same shell this adds a handle (the shell's
    /// genus increments — think of connecting two faces of a box through its
    /// interior, yielding a torus). If they are on different shells of the
    /// same body, the two shells merge instead.
    ///
    /// The killed face must carry no rings of its own. Takes two faces where
    /// `spec/03-topology.md` sketches one: the ring has to land on an
    /// explicit target face.
    pub fn kfmrh(
        &mut self,
        target: EntityId<Face>,
        killed: EntityId<Face>,
    ) -> Result<EntityId<Loop>, EulerError> {
        if self.faces.get(target).is_none() {
            return Err(EulerError::StaleFace(target));
        }
        let killed_face = self
            .faces
            .get(killed)
            .ok_or(EulerError::StaleFace(killed))?;
        if target == killed {
            return Err(EulerError::KfmrhSameFace(target));
        }
        if !killed_face.inner_loops.is_empty() {
            return Err(EulerError::KfmrhFaceHasRings {
                face: killed,
                rings: killed_face.inner_loops.len(),
            });
        }
        let ring = killed_face
            .outer_loop
            .ok_or(EulerError::KfmrhFaceWithoutLoop(killed))?;
        let killed_shell = killed_face.shell;
        let target_shell = self.faces.get(target).expect("checked").shell;
        let body = self.shells.get(target_shell).expect("live shell").body;
        if self.shells.get(killed_shell).expect("live shell").body != body {
            return Err(EulerError::KfmrhDifferentBodies { target, killed });
        }

        let lp = self.loops.get_mut(ring).expect("live loop");
        lp.face = target;
        lp.loop_type = LoopType::Inner;
        self.faces
            .get_mut(target)
            .expect("checked")
            .inner_loops
            .push(ring);

        self.shells
            .get_mut(killed_shell)
            .expect("live shell")
            .faces
            .retain(|&f| f != killed);
        self.faces.remove(killed);

        if killed_shell == target_shell {
            self.shells.get_mut(target_shell).expect("live shell").genus += 1;
        } else {
            let moved =
                std::mem::take(&mut self.shells.get_mut(killed_shell).expect("live shell").faces);
            for &f in &moved {
                self.faces.get_mut(f).expect("live face").shell = target_shell;
            }
            self.shells
                .get_mut(target_shell)
                .expect("live shell")
                .faces
                .extend(moved);
            self.bodies
                .get_mut(body)
                .expect("live body")
                .shells
                .retain(|&s| s != killed_shell);
            self.shells.remove(killed_shell);
        }

        self.debug_check_invariants(body);
        Ok(ring)
    }

    // ------------------------------------------------------------------
    // Invariant checking
    // ------------------------------------------------------------------

    /// Count the Euler-Poincaré terms of a body by traversal.
    ///
    /// Vertices are those reachable from the body's loops (edge endpoints
    /// plus degenerate vertex-loop vertices). Panics if `body` is stale.
    pub fn euler_counts(&self, body: EntityId<Body>) -> EulerCounts {
        let mut edges: HashSet<EntityId<Edge>> = HashSet::new();
        let mut vertices: HashSet<EntityId<Vertex>> = HashSet::new();
        let mut faces = 0;
        let mut loops = 0;
        let mut genus = 0;

        let shell_ids = self.shells_of_body(body).to_vec();
        for &shell_id in &shell_ids {
            genus += self.shells.get(shell_id).expect("live shell").genus as usize;
            for &face_id in self.faces_of_shell(shell_id) {
                faces += 1;
                for loop_id in self.loops_of_face(face_id) {
                    loops += 1;
                    let lp = self.loops.get(loop_id).expect("live loop");
                    if let Some(v) = lp.vertex {
                        vertices.insert(v);
                    }
                    for &fin_id in &lp.fins {
                        let edge_id = self.fin_edge(fin_id);
                        let e = self.edges.get(edge_id).expect("live edge");
                        edges.insert(edge_id);
                        vertices.insert(e.start_vertex);
                        vertices.insert(e.end_vertex);
                    }
                }
            }
        }

        EulerCounts {
            vertices: vertices.len(),
            edges: edges.len(),
            faces,
            loops,
            rings: loops - faces,
            shells: shell_ids.len(),
            genus,
        }
    }

    /// Validate a body: half-edge invariants plus the Euler-Poincaré
    /// formula. Panics if `body` is stale.
    ///
    /// Checked per loop: a loop is either a degenerate vertex loop (no fins,
    /// `vertex` set) or a real loop (fins, no `vertex`); every fin points
    /// back at its loop; `next`/`prev` follow the loop's fin order; each
    /// fin's end vertex is the next fin's start vertex; mates are mutual and
    /// share the fin's edge; every fin is registered on its edge.
    pub fn check_body_invariants(&self, body: EntityId<Body>) -> Result<(), EulerError> {
        let fail = |msg: String| Err(EulerError::InvariantViolation(msg));

        for &shell_id in self.shells_of_body(body) {
            for &face_id in self.faces_of_shell(shell_id) {
                for loop_id in self.loops_of_face(face_id) {
                    let lp = self.loops.get(loop_id).expect("live loop");
                    if lp.face != face_id {
                        return fail(format!("{loop_id:?} does not point back at {face_id:?}"));
                    }
                    match (lp.fins.is_empty(), lp.vertex) {
                        (true, None) => {
                            return fail(format!("{loop_id:?} has no fins and no vertex"));
                        }
                        (false, Some(_)) => {
                            return fail(format!("{loop_id:?} has both fins and a vertex"));
                        }
                        (true, Some(v)) => {
                            if self.vertices.get(v).is_none() {
                                return fail(format!("{loop_id:?} references stale vertex"));
                            }
                        }
                        (false, None) => {}
                    }

                    let n = lp.fins.len();
                    for (i, &fin_id) in lp.fins.iter().enumerate() {
                        let fin = match self.fins.get(fin_id) {
                            Some(f) => f,
                            None => return fail(format!("{loop_id:?} references stale fin")),
                        };
                        if fin.loop_ref != loop_id {
                            return fail(format!("{fin_id:?} does not point back at {loop_id:?}"));
                        }
                        if fin.next != Some(lp.fins[(i + 1) % n]) {
                            return fail(format!("{fin_id:?} next link disagrees with loop order"));
                        }
                        if fin.prev != Some(lp.fins[(i + n - 1) % n]) {
                            return fail(format!("{fin_id:?} prev link disagrees with loop order"));
                        }
                        if self.fin_end_vertex(fin_id)
                            != self.fin_start_vertex(lp.fins[(i + 1) % n])
                        {
                            return fail(format!(
                                "loop {loop_id:?} is not vertex-continuous at {fin_id:?}"
                            ));
                        }
                        if let Some(mate_id) = fin.mate {
                            let mate = match self.fins.get(mate_id) {
                                Some(m) => m,
                                None => return fail(format!("{fin_id:?} has a stale mate")),
                            };
                            if mate.mate != Some(fin_id) {
                                return fail(format!("{fin_id:?} mate link is not mutual"));
                            }
                            if mate.edge != fin.edge {
                                return fail(format!(
                                    "{fin_id:?} and its mate are on different edges"
                                ));
                            }
                        }
                        match self.edges.get(fin.edge) {
                            None => return fail(format!("{fin_id:?} references stale edge")),
                            Some(e) if !e.fins.contains(&fin_id) => {
                                return fail(format!(
                                    "{fin_id:?} is not registered on its edge {:?}",
                                    fin.edge
                                ));
                            }
                            Some(_) => {}
                        }
                    }
                }
            }
        }

        let counts = self.euler_counts(body);
        if !counts.euler_poincare_holds() {
            return fail(format!("Euler-Poincaré formula violated: {counts:?}"));
        }
        Ok(())
    }

    /// Debug-build invariant check run after every Euler operator. A failure
    /// here is a kernel bug (the operator broke the topology), so it panics.
    fn debug_check_invariants(&self, body: EntityId<Body>) {
        if cfg!(debug_assertions) {
            if let Err(e) = self.check_body_invariants(body) {
                panic!("Euler operator broke a topology invariant: {e}");
            }
        }
    }

    // ------------------------------------------------------------------
    // Internal helpers
    // ------------------------------------------------------------------

    /// Find where a spur edge at `vertex` attaches on `face`: either the
    /// face's degenerate vertex loop at that vertex, or the first fin (in
    /// loop order, outer loop first) starting at it.
    fn find_mev_attachment(
        &self,
        face: EntityId<Face>,
        vertex: EntityId<Vertex>,
    ) -> Option<MevAttach> {
        for loop_id in self.loops_of_face(face) {
            let lp = self.loops.get(loop_id).expect("live loop");
            if lp.vertex == Some(vertex) {
                return Some(MevAttach::VertexLoop(loop_id));
            }
            for (i, &fin_id) in lp.fins.iter().enumerate() {
                if self.fin_start_vertex(fin_id) == vertex {
                    return Some(MevAttach::BeforeFin(loop_id, i));
                }
            }
        }
        None
    }

    /// First fin on `face` (loops in `loops_of_face` order) starting at
    /// `vertex`, as `(loop, index within the loop's fins)`.
    fn find_fin_starting_at(
        &self,
        face: EntityId<Face>,
        vertex: EntityId<Vertex>,
    ) -> Option<(EntityId<Loop>, usize)> {
        for loop_id in self.loops_of_face(face) {
            for (i, &fin_id) in self.fins_of_loop(loop_id).iter().enumerate() {
                if self.fin_start_vertex(fin_id) == vertex {
                    return Some((loop_id, i));
                }
            }
        }
        None
    }

    /// Insert an unlinked fin (next/prev/mate patched later).
    fn insert_fin(
        &mut self,
        edge: EntityId<Edge>,
        loop_ref: EntityId<Loop>,
        sense: FinSense,
    ) -> EntityId<Fin> {
        self.fins.insert(Fin {
            edge,
            loop_ref,
            sense,
            next: None,
            prev: None,
            mate: None,
            pcurve: None,
        })
    }

    /// Register a fresh fin pair on its edge and mate the two fins.
    fn register_fins_on_edge(
        &mut self,
        edge: EntityId<Edge>,
        fin_a: EntityId<Fin>,
        fin_b: EntityId<Fin>,
    ) {
        let e = self.edges.get_mut(edge).expect("live edge");
        e.fins.push(fin_a);
        e.fins.push(fin_b);
        self.fins.get_mut(fin_a).expect("just inserted").mate = Some(fin_b);
        self.fins.get_mut(fin_b).expect("just inserted").mate = Some(fin_a);
    }

    /// Rebuild `next`/`prev` links of a loop from its fin order.
    fn relink_loop(&mut self, loop_id: EntityId<Loop>) {
        let fins = self.loops.get(loop_id).expect("live loop").fins.clone();
        let n = fins.len();
        for (i, &fin_id) in fins.iter().enumerate() {
            let fin = self.fins.get_mut(fin_id).expect("live fin");
            fin.next = Some(fins[(i + 1) % n]);
            fin.prev = Some(fins[(i + n - 1) % n]);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn p(x: f64, y: f64, z: f64) -> Point3 {
        Point3::new(x, y, z)
    }

    /// Assert the formula holds and the body validates, returning counts.
    fn checked_counts(store: &TopologyStore, body: EntityId<Body>) -> EulerCounts {
        store
            .check_body_invariants(body)
            .expect("body must validate");
        let counts = store.euler_counts(body);
        assert!(
            counts.euler_poincare_holds(),
            "formula violated: {counts:?}"
        );
        counts
    }

    /// mvfs + 3×mev + mef: a square lamina (two faces glued along a square).
    /// Returns (store, body, vertices v0..v3, bottom face, top face).
    #[allow(clippy::type_complexity)]
    fn build_square_lamina() -> (
        TopologyStore,
        EntityId<Body>,
        [EntityId<Vertex>; 4],
        EntityId<Face>,
        EntityId<Face>,
    ) {
        let mut store = TopologyStore::new();
        let (body, v0, f_bottom, _shell) = store.mvfs(p(0.0, 0.0, 0.0));
        let (_e01, v1) = store.mev(v0, f_bottom, p(1.0, 0.0, 0.0)).unwrap();
        let (_e12, v2) = store.mev(v1, f_bottom, p(1.0, 1.0, 0.0)).unwrap();
        let (_e23, v3) = store.mev(v2, f_bottom, p(0.0, 1.0, 0.0)).unwrap();
        let (_e30, f_top) = store.mef(v3, v0, f_bottom).unwrap();
        (store, body, [v0, v1, v2, v3], f_bottom, f_top)
    }

    #[test]
    fn mvfs_seeds_minimal_body() {
        let mut store = TopologyStore::new();
        let (body, vertex, face, shell) = store.mvfs(p(0.0, 0.0, 0.0));

        let counts = checked_counts(&store, body);
        assert_eq!(
            counts,
            EulerCounts {
                vertices: 1,
                edges: 0,
                faces: 1,
                loops: 1,
                rings: 0,
                shells: 1,
                genus: 0,
            }
        );

        let outer = store.face(face).unwrap().outer_loop.expect("outer loop");
        let lp = store.loop_(outer).unwrap();
        assert_eq!(lp.loop_type, LoopType::Vertex);
        assert_eq!(lp.vertex, Some(vertex));
        assert!(lp.fins.is_empty());
        assert_eq!(store.shell(shell).unwrap().genus, 0);
    }

    #[test]
    fn mev_upgrades_vertex_loop_to_spur() {
        let mut store = TopologyStore::new();
        let (body, v0, face, _shell) = store.mvfs(p(0.0, 0.0, 0.0));
        let (edge, v1) = store.mev(v0, face, p(1.0, 0.0, 0.0)).unwrap();

        let counts = checked_counts(&store, body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (2, 1, 1));

        // The vertex loop became a real two-fin loop: out and back.
        let outer = store.face(face).unwrap().outer_loop.unwrap();
        let lp = store.loop_(outer).unwrap();
        assert_eq!(lp.loop_type, LoopType::Outer);
        assert_eq!(lp.vertex, None);
        assert_eq!(lp.fins.len(), 2);
        let (out, back) = (lp.fins[0], lp.fins[1]);
        assert_eq!(store.fin_start_vertex(out), v0);
        assert_eq!(store.fin_end_vertex(out), v1);
        assert_eq!(store.fin_mate(out), Some(back));
        assert_eq!(store.fin_edge(out), edge);
    }

    #[test]
    fn cube_built_purely_from_euler_ops() {
        let mut store = TopologyStore::new();

        // Seed and bottom square: mvfs, 3×mev, mef.
        let (body, v0, f_bottom, shell) = store.mvfs(p(0.0, 0.0, 0.0));
        let expect = |store: &TopologyStore, v: usize, e: usize, f: usize| {
            let c = checked_counts(store, body);
            assert_eq!((c.vertices, c.edges, c.faces), (v, e, f));
        };
        expect(&store, 1, 0, 1);

        let (_e, v1) = store.mev(v0, f_bottom, p(1.0, 0.0, 0.0)).unwrap();
        expect(&store, 2, 1, 1);
        let (_e, v2) = store.mev(v1, f_bottom, p(1.0, 1.0, 0.0)).unwrap();
        expect(&store, 3, 2, 1);
        let (_e, v3) = store.mev(v2, f_bottom, p(0.0, 1.0, 0.0)).unwrap();
        expect(&store, 4, 3, 1);
        let (_e, f_top) = store.mef(v3, v0, f_bottom).unwrap();
        expect(&store, 4, 4, 2);

        // Four corner posts on the top face.
        let (_e, v4) = store.mev(v0, f_top, p(0.0, 0.0, 1.0)).unwrap();
        let (_e, v5) = store.mev(v1, f_top, p(1.0, 0.0, 1.0)).unwrap();
        let (_e, v6) = store.mev(v2, f_top, p(1.0, 1.0, 1.0)).unwrap();
        let (_e, v7) = store.mev(v3, f_top, p(0.0, 1.0, 1.0)).unwrap();
        expect(&store, 8, 8, 2);

        // Close the four sides; what remains of f_top is the top face.
        store.mef(v4, v7, f_top).unwrap();
        expect(&store, 8, 9, 3);
        store.mef(v7, v6, f_top).unwrap();
        expect(&store, 8, 10, 4);
        store.mef(v6, v5, f_top).unwrap();
        expect(&store, 8, 11, 5);
        store.mef(v5, v4, f_top).unwrap();
        expect(&store, 8, 12, 6);

        // Final shape: a genus-0 closed cube.
        let counts = store.euler_counts(body);
        assert_eq!(
            counts,
            EulerCounts {
                vertices: 8,
                edges: 12,
                faces: 6,
                loops: 6,
                rings: 0,
                shells: 1,
                genus: 0,
            }
        );

        // Every face is a quad, every edge manifold with opposed mates,
        // every vertex trivalent.
        for &face in store.faces_of_shell(shell) {
            let outer = store.face(face).unwrap().outer_loop.unwrap();
            assert_eq!(store.fins_of_loop(outer).len(), 4);
        }
        for (edge_id, edge) in store.edges.iter() {
            assert_eq!(edge.fins.len(), 2, "{edge_id:?} must be manifold");
            let (a, b) = (edge.fins[0], edge.fins[1]);
            assert_eq!(store.fin_mate(a), Some(b));
            assert_ne!(store.fin_face(a), store.fin_face(b));
            let (fa, fb) = (store.fin(a).unwrap(), store.fin(b).unwrap());
            assert_eq!(fa.sense, fb.sense.opposite());
        }
        for (_, vertex) in store.vertices.iter() {
            assert_eq!(vertex.edges.len(), 3);
        }
    }

    #[test]
    fn kemr_splits_bridge_into_ring() {
        // Square lamina, then grow a triangle inside one face off a bridge
        // edge from v0; killing the bridge turns the triangle's chain and
        // the square boundary into separate loops.
        let (mut store, body, [v0, ..], f_bottom, _f_top) = build_square_lamina();

        let (bridge, vc) = store.mev(v0, f_bottom, p(0.4, 0.4, 0.0)).unwrap();
        let (_e, vd) = store.mev(vc, f_bottom, p(0.6, 0.4, 0.0)).unwrap();
        let (_e, ve) = store.mev(vd, f_bottom, p(0.5, 0.6, 0.0)).unwrap();
        let (_e, _f_tri) = store.mef(ve, vc, f_bottom).unwrap();
        let before = checked_counts(&store, body);
        assert_eq!((before.vertices, before.edges, before.faces), (7, 8, 3));
        assert_eq!(before.rings, 0);

        let ring = store.kemr(bridge).unwrap();

        let after = checked_counts(&store, body);
        assert_eq!((after.vertices, after.edges, after.faces), (7, 7, 3));
        assert_eq!(after.rings, 1);
        assert_eq!(after.loops, 4);

        let lp = store.loop_(ring).unwrap();
        assert_eq!(lp.loop_type, LoopType::Inner);
        let ring_face = lp.face;
        assert!(store.face(ring_face).unwrap().inner_loops.contains(&ring));
        // The bridge edge and its fins are gone.
        assert!(store.edge(bridge).is_none());
        assert!(!store.edges_of_vertex(v0).contains(&bridge));
        assert!(!store.edges_of_vertex(vc).contains(&bridge));
    }

    #[test]
    fn kfmrh_same_shell_adds_handle() {
        // Square lamina: kill one of the two faces, its boundary becomes a
        // ring of the other face on the same shell → genus 1.
        let (mut store, body, _v, f_bottom, f_top) = build_square_lamina();

        let ring = store.kfmrh(f_bottom, f_top).unwrap();

        let counts = checked_counts(&store, body);
        assert_eq!(
            counts,
            EulerCounts {
                vertices: 4,
                edges: 4,
                faces: 1,
                loops: 2,
                rings: 1,
                shells: 1,
                genus: 1,
            }
        );
        assert!(store.face(f_top).is_none());
        assert_eq!(store.loop_(ring).unwrap().face, f_bottom);
        assert_eq!(store.loop_(ring).unwrap().loop_type, LoopType::Inner);
        let shell = store.face(f_bottom).unwrap().shell;
        assert_eq!(store.shell(shell).unwrap().genus, 1);
    }

    #[test]
    fn kfmrh_across_shells_merges_them() {
        // A lamina body plus a second shell holding a lone vertex-loop face
        // (a second mvfs-style seed, grafted into the same body by hand
        // since mvfs always creates its own body).
        let (mut store, body, _v, f_bottom, _f_top) = build_square_lamina();
        let shell2 = store.create_shell(body, true, ShellOrientation::Outward);
        let face2 = store.create_face(shell2, FaceSense::Positive);
        let lone = store.create_vertex(p(5.0, 5.0, 5.0), SYSTEM_RESOLUTION);
        let vloop = store.loops.insert(Loop {
            face: face2,
            fins: Vec::new(),
            loop_type: LoopType::Vertex,
            vertex: Some(lone),
        });
        store.faces.get_mut(face2).unwrap().outer_loop = Some(vloop);
        let before = checked_counts(&store, body);
        assert_eq!((before.shells, before.faces, before.genus), (2, 3, 0));

        let ring = store.kfmrh(f_bottom, face2).unwrap();

        let after = checked_counts(&store, body);
        assert_eq!(
            after,
            EulerCounts {
                vertices: 5,
                edges: 4,
                faces: 2,
                loops: 3,
                rings: 1,
                shells: 1,
                genus: 0,
            }
        );
        assert!(store.shell(shell2).is_none(), "empty shell is deleted");
        assert!(store.face(face2).is_none());
        assert_eq!(store.loop_(ring).unwrap().face, f_bottom);
        assert_eq!(store.loop_(ring).unwrap().vertex, Some(lone));
    }

    #[test]
    fn mev_rejects_stale_and_detached_ids() {
        let mut store = TopologyStore::new();
        let (_body, v0, face, _shell) = store.mvfs(p(0.0, 0.0, 0.0));

        let stale_v = store.create_vertex(p(9.0, 9.0, 9.0), SYSTEM_RESOLUTION);
        store.vertices.remove(stale_v);
        assert_eq!(
            store.mev(stale_v, face, p(1.0, 0.0, 0.0)),
            Err(EulerError::StaleVertex(stale_v))
        );

        // Live vertex that is not on the face.
        let detached = store.create_vertex(p(9.0, 9.0, 9.0), SYSTEM_RESOLUTION);
        assert_eq!(
            store.mev(detached, face, p(1.0, 0.0, 0.0)),
            Err(EulerError::VertexNotOnFace {
                vertex: detached,
                face
            })
        );

        // The failed calls must not have mutated the body.
        assert_eq!(store.euler_counts(store.body_of_face(face)).edges, 0);
        let _ = v0;
    }

    #[test]
    fn mef_rejects_invalid_vertex_pairs() {
        let (mut store, body, [v0, v1, ..], f_bottom, f_top) = build_square_lamina();

        assert_eq!(
            store.mef(v0, v0, f_bottom),
            Err(EulerError::MefSameVertex(v0))
        );

        // A vertex on a *different* loop of a different face: grow a spur in
        // f_bottom, ring it off, then try to mef between the ring and the
        // outer loop's vertices... simplest cross-loop case: a vertex that
        // is not on the face at all.
        let detached = store.create_vertex(p(9.0, 9.0, 9.0), SYSTEM_RESOLUTION);
        assert_eq!(
            store.mef(v0, detached, f_bottom),
            Err(EulerError::VertexNotOnFace {
                vertex: detached,
                face: f_bottom
            })
        );

        // Vertices on the same face but different loops: put a ring in
        // f_bottom via mev + mev + mef + kemr, then connect across.
        let (bridge, vc) = store.mev(v0, f_bottom, p(0.4, 0.4, 0.0)).unwrap();
        let (_e, vd) = store.mev(vc, f_bottom, p(0.6, 0.4, 0.0)).unwrap();
        let (_e, ve) = store.mev(vd, f_bottom, p(0.5, 0.6, 0.0)).unwrap();
        store.mef(ve, vc, f_bottom).unwrap();
        let ring = store.kemr(bridge).unwrap();
        let ring_face = store.loop_(ring).unwrap().face;
        // One loop of ring_face contains v0's chain, the other the ring;
        // find a vertex from each loop and mef them: rejected.
        let outer = store.face(ring_face).unwrap().outer_loop.unwrap();
        let va = store.fin_start_vertex(store.fins_of_loop(outer)[0]);
        let vb = store.fin_start_vertex(store.fins_of_loop(ring)[0]);
        assert_eq!(
            store.mef(va, vb, ring_face),
            Err(EulerError::MefVerticesInDifferentLoops {
                vertex_a: va,
                vertex_b: vb,
                face: ring_face
            })
        );

        checked_counts(&store, body);
        let _ = (v1, f_top);
    }

    #[test]
    fn kemr_rejects_manifold_spur_and_stale_edges() {
        let (mut store, _body, [v0, ..], f_bottom, _f_top) = build_square_lamina();

        // A lamina edge has its two fins on two different loops.
        let manifold_edge = store.edges_of_face(f_bottom)[0];
        assert_eq!(
            store.kemr(manifold_edge),
            Err(EulerError::KemrFinsInDifferentLoops(manifold_edge))
        );

        // A spur's fins are adjacent: killing it would leave an empty loop.
        let (spur, _vc) = store.mev(v0, f_bottom, p(0.5, 0.5, 0.0)).unwrap();
        assert_eq!(store.kemr(spur), Err(EulerError::KemrDegenerate(spur)));

        // An edge with a single fin (or none) is not a KEMR candidate.
        let (va, vb) = (
            store.create_vertex(p(7.0, 0.0, 0.0), SYSTEM_RESOLUTION),
            store.create_vertex(p(8.0, 0.0, 0.0), SYSTEM_RESOLUTION),
        );
        let free_edge = store.create_edge(va, vb, SYSTEM_RESOLUTION);
        assert_eq!(
            store.kemr(free_edge),
            Err(EulerError::KemrWrongFinCount {
                edge: free_edge,
                fins: 0
            })
        );

        let stale = free_edge;
        store.edges.remove(free_edge);
        assert_eq!(store.kemr(stale), Err(EulerError::StaleEdge(stale)));
    }

    #[test]
    fn kfmrh_rejects_invalid_faces() {
        let (mut store, body, [v0, ..], f_bottom, f_top) = build_square_lamina();

        assert_eq!(
            store.kfmrh(f_bottom, f_bottom),
            Err(EulerError::KfmrhSameFace(f_bottom))
        );

        // Killed face may not carry rings: give f_bottom one.
        let (bridge, vc) = store.mev(v0, f_bottom, p(0.4, 0.4, 0.0)).unwrap();
        let (_e, vd) = store.mev(vc, f_bottom, p(0.6, 0.4, 0.0)).unwrap();
        let (_e, ve) = store.mev(vd, f_bottom, p(0.5, 0.6, 0.0)).unwrap();
        let (_e, _f) = store.mef(ve, vc, f_bottom).unwrap();
        let ring = store.kemr(bridge).unwrap();
        let ringed_face = store.loop_(ring).unwrap().face;
        assert_eq!(
            store.kfmrh(f_top, ringed_face),
            Err(EulerError::KfmrhFaceHasRings {
                face: ringed_face,
                rings: 1
            })
        );

        // Faces of different bodies cannot be connected.
        let (_body2, _v, other_face, _shell2) = store.mvfs(p(9.0, 9.0, 9.0));
        assert_eq!(
            store.kfmrh(f_top, other_face),
            Err(EulerError::KfmrhDifferentBodies {
                target: f_top,
                killed: other_face
            })
        );

        checked_counts(&store, body);
    }

    #[test]
    fn torus_from_kfmrh_satisfies_genus_one_formula() {
        // Build a cube, then punch a conceptual handle by killing the top
        // face into the bottom face: V-E+F-R = 2(S-H) with H = 1.
        let mut store = TopologyStore::new();
        let (body, v0, f_bottom, _shell) = store.mvfs(p(0.0, 0.0, 0.0));
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

        store.kfmrh(f_bottom, f_top).unwrap();

        let counts = checked_counts(&store, body);
        assert_eq!(
            counts,
            EulerCounts {
                vertices: 8,
                edges: 12,
                faces: 5,
                loops: 6,
                rings: 1,
                shells: 1,
                genus: 1,
            }
        );
    }

    #[test]
    fn failed_ops_leave_store_untouched() {
        let (mut store, body, [v0, ..], f_bottom, _f_top) = build_square_lamina();
        let before = store.euler_counts(body);

        let detached = store.create_vertex(p(9.0, 9.0, 9.0), SYSTEM_RESOLUTION);
        assert!(store.mev(detached, f_bottom, p(1.0, 2.0, 0.0)).is_err());
        assert!(store.mef(v0, detached, f_bottom).is_err());
        assert!(store.kemr(store.edges_of_face(f_bottom)[0]).is_err());
        assert!(store.kfmrh(f_bottom, f_bottom).is_err());

        assert_eq!(store.euler_counts(body), before);
        checked_counts(&store, body);
    }
}
