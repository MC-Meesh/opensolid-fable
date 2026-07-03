//! B-Rep topology store: the connectivity graph of a boundary representation.
//!
//! Containment hierarchy (per `spec/01-data-model.md` and `spec/03-topology.md`):
//!
//! ```text
//! Body > Shell > Face > Loop > Fin (half-edge) > Edge > Vertex
//! ```
//!
//! All entities live in typed [`Arena`]s inside [`TopologyStore`] and reference
//! each other through generation-checked [`EntityId`]s, so stale ids are
//! detected rather than silently aliased.
//!
//! Divergences from the spec text, deliberate for this first cut:
//!
//! - Entities do not carry a redundant `id` self-field; the arena key is the
//!   canonical identity.
//! - Links the spec models with a NULL sentinel (`Fin::mate`, unset loop
//!   references) are `Option` here — core `EntityId` has no null value.
//! - `Edge::fins` is a `Vec` rather than `[EntityId<Fin>; 2]` so free edges
//!   (1 fin) and non-manifold edges (>2 fins) are representable.
//! - `Region`, attribute sets, and body transforms are deferred to later
//!   issues, as are Euler operators and validation.
//!
//! Tolerant-modeling fields exist from day one (`spec/08-tolerances.md`):
//! every [`Edge`] and [`Vertex`] carries a `tolerance`, and every [`Fin`] has
//! an optional `pcurve` slot for an SP-curve in the owning face's parameter
//! space.
//!
//! Creation methods panic on stale parent ids (a stale id passed to
//! construction is a caller bug, not a recoverable condition). Navigation
//! methods likewise panic on stale ids — a dangling reference inside the graph
//! means the topology is corrupt. Direct lookups (`body()`, `edge()`, ...)
//! return `Option` so callers can test id validity.

use opensolid_core::{Arena, EntityId, Point3};

/// System resolution: distances smaller than this are considered zero.
/// This is the precision floor of the kernel (`spec/08-tolerances.md`).
pub const SYSTEM_RESOLUTION: f64 = 1e-10;

/// Entities with tolerance above this multiple of [`SYSTEM_RESOLUTION`] are
/// considered "tolerant" (carrying a real precision gap).
const TOLERANT_THRESHOLD: f64 = SYSTEM_RESOLUTION * 10.0;

/// Placeholder for the parametric curve geometry type.
///
/// The geometry layer does not exist yet; this marker only serves as the type
/// parameter of `EntityId<Curve>` slots (`Edge::curve`, `Fin::pcurve`) so the
/// topology carries geometry references from day one. A later issue replaces
/// it with the real curve representation.
#[derive(Debug, Clone)]
pub struct Curve;

/// Placeholder for the parametric surface geometry type. See [`Curve`].
#[derive(Debug, Clone)]
pub struct Surface;

/// The kind of a [`Body`], constraining its topology.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum BodyType {
    /// Closed volume (manifold, oriented, closed shells).
    Solid,
    /// Open surfaces (non-closed shells, sheet bodies).
    Sheet,
    /// Curves only (no faces).
    Wire,
    /// Mixed dimensionality (Parasolid's "general body").
    General,
    /// Minimum body — single vertex, no edges or faces.
    Minimum,
}

/// Whether a shell's face normals point out of or into the material.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ShellOrientation {
    /// Face normals point outward from material.
    Outward,
    /// Face normals point inward (inner void shell).
    Inward,
}

/// Whether a face's normal agrees with its surface normal.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaceSense {
    /// Face normal = surface normal.
    Positive,
    /// Face normal = -surface normal.
    Negative,
}

/// The role of a [`Loop`] on its face.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LoopType {
    /// Standard loop bounding a face region.
    Outer,
    /// Inner loop (hole in the face).
    Inner,
    /// Degenerate loop (single vertex, e.g. cone apex).
    Vertex,
    /// Loop at a singularity (e.g. sphere pole).
    Singular,
}

/// Direction of a [`Fin`] relative to its edge's natural direction.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FinSense {
    /// Fin traverses the edge start → end.
    Forward,
    /// Fin traverses the edge end → start.
    Reversed,
}

impl FinSense {
    /// The opposite sense.
    pub fn opposite(self) -> Self {
        match self {
            FinSense::Forward => FinSense::Reversed,
            FinSense::Reversed => FinSense::Forward,
        }
    }
}

/// Top-level container: a connected model occupying space.
#[derive(Debug, Clone)]
pub struct Body {
    pub body_type: BodyType,
    pub shells: Vec<EntityId<Shell>>,
}

/// A connected set of faces forming a boundary.
#[derive(Debug, Clone)]
pub struct Shell {
    pub body: EntityId<Body>,
    pub faces: Vec<EntityId<Face>>,
    /// true = watertight (encloses volume).
    pub is_closed: bool,
    pub orientation: ShellOrientation,
}

/// A bounded region on a surface: one outer loop plus zero or more holes.
#[derive(Debug, Clone)]
pub struct Face {
    pub shell: EntityId<Shell>,
    /// Underlying surface (geometry layer pending, hence `Option`).
    pub surface: Option<EntityId<Surface>>,
    pub sense: FaceSense,
    /// Set when the outer loop is created via [`TopologyStore::create_loop`].
    pub outer_loop: Option<EntityId<Loop>>,
    pub inner_loops: Vec<EntityId<Loop>>,
}

/// An ordered, closed cycle of fins bounding a face region.
#[derive(Debug, Clone)]
pub struct Loop {
    pub face: EntityId<Face>,
    /// Fins in traversal order (each fin's end vertex is the next fin's start).
    pub fins: Vec<EntityId<Fin>>,
    pub loop_type: LoopType,
}

/// Half-edge: one face's directed use of an edge.
#[derive(Debug, Clone)]
pub struct Fin {
    pub edge: EntityId<Edge>,
    pub loop_ref: EntityId<Loop>,
    pub sense: FinSense,
    /// Next fin in the loop. Always `Some` once created via
    /// [`TopologyStore::create_loop`]; `Option` only because links are
    /// patched after all fins of the loop are allocated.
    pub next: Option<EntityId<Fin>>,
    /// Previous fin in the loop. Same invariant as `next`.
    pub prev: Option<EntityId<Fin>>,
    /// Opposite fin on the other face sharing this edge.
    /// `None` for boundary/free edges (single-fin edges).
    pub mate: Option<EntityId<Fin>>,
    /// 2D curve in the owning face's parameter space (SP-curve for tolerant
    /// edges, `spec/08-tolerances.md`). Geometry layer pending.
    pub pcurve: Option<EntityId<Curve>>,
}

/// A bounded curve segment between two vertices.
#[derive(Debug, Clone)]
pub struct Edge {
    /// Underlying 3D curve (geometry layer pending, hence `Option`).
    pub curve: Option<EntityId<Curve>>,
    pub start_vertex: EntityId<Vertex>,
    pub end_vertex: EntityId<Vertex>,
    /// Curve parameter at the start vertex.
    pub t_start: f64,
    /// Curve parameter at the end vertex.
    pub t_end: f64,
    /// Tolerant modeling: max distance between this edge's curve and the true
    /// intersection of its adjacent faces' surfaces.
    pub tolerance: f64,
    /// All fins using this edge: 2 for manifold, 1 for free/boundary edges,
    /// >2 for non-manifold edges.
    pub fins: Vec<EntityId<Fin>>,
}

impl Edge {
    /// Whether this edge carries a tolerance meaningfully above system resolution.
    pub fn is_tolerant(&self) -> bool {
        self.tolerance > TOLERANT_THRESHOLD
    }
}

/// A point in 3D space, shared between edges.
#[derive(Debug, Clone)]
pub struct Vertex {
    pub point: Point3,
    /// Tolerant modeling: max distance between this vertex's point and the
    /// endpoint of any adjacent edge's curve.
    pub tolerance: f64,
    /// All edges meeting at this vertex.
    pub edges: Vec<EntityId<Edge>>,
}

impl Vertex {
    /// Whether this vertex carries a tolerance meaningfully above system resolution.
    pub fn is_tolerant(&self) -> bool {
        self.tolerance > TOLERANT_THRESHOLD
    }
}

/// Centralized store for all topological entities, one typed arena per kind.
#[derive(Default)]
pub struct TopologyStore {
    pub bodies: Arena<Body>,
    pub shells: Arena<Shell>,
    pub faces: Arena<Face>,
    pub loops: Arena<Loop>,
    pub fins: Arena<Fin>,
    pub edges: Arena<Edge>,
    pub vertices: Arena<Vertex>,
}

impl TopologyStore {
    pub fn new() -> Self {
        Self::default()
    }

    // ------------------------------------------------------------------
    // Creation
    // ------------------------------------------------------------------

    /// Create an empty body.
    pub fn create_body(&mut self, body_type: BodyType) -> EntityId<Body> {
        self.bodies.insert(Body {
            body_type,
            shells: Vec::new(),
        })
    }

    /// Create a shell in `body` and register it on the body.
    ///
    /// Panics if `body` is stale.
    pub fn create_shell(
        &mut self,
        body: EntityId<Body>,
        is_closed: bool,
        orientation: ShellOrientation,
    ) -> EntityId<Shell> {
        let shell = self.shells.insert(Shell {
            body,
            faces: Vec::new(),
            is_closed,
            orientation,
        });
        self.bodies
            .get_mut(body)
            .expect("create_shell: stale Body id")
            .shells
            .push(shell);
        shell
    }

    /// Create a face in `shell` (no loops yet) and register it on the shell.
    ///
    /// Panics if `shell` is stale.
    pub fn create_face(&mut self, shell: EntityId<Shell>, sense: FaceSense) -> EntityId<Face> {
        let face = self.faces.insert(Face {
            shell,
            surface: None,
            sense,
            outer_loop: None,
            inner_loops: Vec::new(),
        });
        self.shells
            .get_mut(shell)
            .expect("create_face: stale Shell id")
            .faces
            .push(face);
        face
    }

    /// Create a vertex at `point` with the given tolerance
    /// (use [`SYSTEM_RESOLUTION`] for a precise vertex).
    pub fn create_vertex(&mut self, point: Point3, tolerance: f64) -> EntityId<Vertex> {
        self.vertices.insert(Vertex {
            point,
            tolerance,
            edges: Vec::new(),
        })
    }

    /// Create an edge between two vertices with the given tolerance
    /// (use [`SYSTEM_RESOLUTION`] for a precise edge) and register it on both
    /// vertices. The curve slot starts empty with a unit parameter range.
    ///
    /// Panics if either vertex id is stale.
    pub fn create_edge(
        &mut self,
        start_vertex: EntityId<Vertex>,
        end_vertex: EntityId<Vertex>,
        tolerance: f64,
    ) -> EntityId<Edge> {
        let edge = self.edges.insert(Edge {
            curve: None,
            start_vertex,
            end_vertex,
            t_start: 0.0,
            t_end: 1.0,
            tolerance,
            fins: Vec::new(),
        });
        self.vertices
            .get_mut(start_vertex)
            .expect("create_edge: stale start Vertex id")
            .edges
            .push(edge);
        self.vertices
            .get_mut(end_vertex)
            .expect("create_edge: stale end Vertex id")
            .edges
            .push(edge);
        edge
    }

    /// Create a loop on `face` from an ordered sequence of directed edges.
    ///
    /// One fin is created per `(edge, sense)` entry. Fins are linked in a
    /// cycle (`next`/`prev`), registered on their edges, and mated with the
    /// edge's existing fin when this makes the edge two-sided. The loop is
    /// registered on the face: `LoopType::Outer` fills `outer_loop`
    /// (panicking if already set), any other type appends to `inner_loops`.
    ///
    /// The store does not verify that consecutive edges share vertices;
    /// invariant checking arrives with the Euler-operator layer.
    ///
    /// Panics if `face` or any edge id is stale.
    pub fn create_loop(
        &mut self,
        face: EntityId<Face>,
        loop_type: LoopType,
        edges: &[(EntityId<Edge>, FinSense)],
    ) -> EntityId<Loop> {
        assert!(self.faces.get(face).is_some(), "create_loop: stale Face id");

        let loop_id = self.loops.insert(Loop {
            face,
            fins: Vec::new(),
            loop_type,
        });

        let fin_ids: Vec<EntityId<Fin>> = edges
            .iter()
            .map(|&(edge, sense)| {
                assert!(self.edges.get(edge).is_some(), "create_loop: stale Edge id");
                self.fins.insert(Fin {
                    edge,
                    loop_ref: loop_id,
                    sense,
                    next: None,
                    prev: None,
                    mate: None,
                    pcurve: None,
                })
            })
            .collect();

        let n = fin_ids.len();
        for (i, &fin_id) in fin_ids.iter().enumerate() {
            let fin = self.fins.get_mut(fin_id).expect("just inserted");
            fin.next = Some(fin_ids[(i + 1) % n]);
            fin.prev = Some(fin_ids[(i + n - 1) % n]);
        }

        for &fin_id in &fin_ids {
            let edge_id = self.fins.get(fin_id).expect("just inserted").edge;
            let edge = self.edges.get_mut(edge_id).expect("checked above");
            edge.fins.push(fin_id);
            if edge.fins.len() == 2 {
                let (a, b) = (edge.fins[0], edge.fins[1]);
                self.fins.get_mut(a).expect("registered fin").mate = Some(b);
                self.fins.get_mut(b).expect("registered fin").mate = Some(a);
            }
        }

        self.loops.get_mut(loop_id).expect("just inserted").fins = fin_ids;

        let face_ref = self.faces.get_mut(face).expect("checked above");
        if loop_type == LoopType::Outer {
            assert!(
                face_ref.outer_loop.is_none(),
                "create_loop: face already has an outer loop"
            );
            face_ref.outer_loop = Some(loop_id);
        } else {
            face_ref.inner_loops.push(loop_id);
        }

        loop_id
    }

    // ------------------------------------------------------------------
    // Direct lookup (Option-returning: usable as id-validity checks)
    // ------------------------------------------------------------------

    pub fn body(&self, id: EntityId<Body>) -> Option<&Body> {
        self.bodies.get(id)
    }

    pub fn shell(&self, id: EntityId<Shell>) -> Option<&Shell> {
        self.shells.get(id)
    }

    pub fn face(&self, id: EntityId<Face>) -> Option<&Face> {
        self.faces.get(id)
    }

    pub fn loop_(&self, id: EntityId<Loop>) -> Option<&Loop> {
        self.loops.get(id)
    }

    pub fn fin(&self, id: EntityId<Fin>) -> Option<&Fin> {
        self.fins.get(id)
    }

    pub fn edge(&self, id: EntityId<Edge>) -> Option<&Edge> {
        self.edges.get(id)
    }

    pub fn vertex(&self, id: EntityId<Vertex>) -> Option<&Vertex> {
        self.vertices.get(id)
    }

    // ------------------------------------------------------------------
    // Downward navigation (parent → children)
    // ------------------------------------------------------------------

    /// Shells of a body. Panics if `body` is stale.
    pub fn shells_of_body(&self, body: EntityId<Body>) -> &[EntityId<Shell>] {
        &self.bodies.get(body).expect("stale Body id").shells
    }

    /// All faces of a body, across all its shells. Panics if `body` is stale.
    pub fn faces_of_body(&self, body: EntityId<Body>) -> Vec<EntityId<Face>> {
        self.shells_of_body(body)
            .iter()
            .flat_map(|&shell| self.faces_of_shell(shell).iter().copied())
            .collect()
    }

    /// Faces of a shell. Panics if `shell` is stale.
    pub fn faces_of_shell(&self, shell: EntityId<Shell>) -> &[EntityId<Face>] {
        &self.shells.get(shell).expect("stale Shell id").faces
    }

    /// Loops of a face, outer loop first. Panics if `face` is stale.
    pub fn loops_of_face(&self, face: EntityId<Face>) -> Vec<EntityId<Loop>> {
        let face = self.faces.get(face).expect("stale Face id");
        face.outer_loop
            .into_iter()
            .chain(face.inner_loops.iter().copied())
            .collect()
    }

    /// Fins of a loop in traversal order. Panics if `loop_id` is stale.
    pub fn fins_of_loop(&self, loop_id: EntityId<Loop>) -> &[EntityId<Fin>] {
        &self.loops.get(loop_id).expect("stale Loop id").fins
    }

    /// Edges bounding a face (outer loop then holes, deduplicated).
    /// Panics if `face` is stale.
    pub fn edges_of_face(&self, face: EntityId<Face>) -> Vec<EntityId<Edge>> {
        let mut edges = Vec::new();
        for loop_id in self.loops_of_face(face) {
            for &fin_id in self.fins_of_loop(loop_id) {
                let edge = self.fin_edge(fin_id);
                if !edges.contains(&edge) {
                    edges.push(edge);
                }
            }
        }
        edges
    }

    /// Ordered start vertices of a face's outer loop. Empty if the face has
    /// no outer loop yet. Panics if `face` is stale.
    pub fn vertices_of_face(&self, face: EntityId<Face>) -> Vec<EntityId<Vertex>> {
        let face = self.faces.get(face).expect("stale Face id");
        match face.outer_loop {
            Some(loop_id) => self
                .fins_of_loop(loop_id)
                .iter()
                .map(|&fin| self.fin_start_vertex(fin))
                .collect(),
            None => Vec::new(),
        }
    }

    // ------------------------------------------------------------------
    // Fin (half-edge) navigation
    // ------------------------------------------------------------------

    /// Next fin in the loop. Panics if `fin` is stale.
    pub fn fin_next(&self, fin: EntityId<Fin>) -> EntityId<Fin> {
        self.fins
            .get(fin)
            .expect("stale Fin id")
            .next
            .expect("fin links are initialized by create_loop")
    }

    /// Previous fin in the loop. Panics if `fin` is stale.
    pub fn fin_prev(&self, fin: EntityId<Fin>) -> EntityId<Fin> {
        self.fins
            .get(fin)
            .expect("stale Fin id")
            .prev
            .expect("fin links are initialized by create_loop")
    }

    /// Mate fin (same edge, other face); `None` on a boundary/free edge.
    /// Panics if `fin` is stale.
    pub fn fin_mate(&self, fin: EntityId<Fin>) -> Option<EntityId<Fin>> {
        self.fins.get(fin).expect("stale Fin id").mate
    }

    /// The edge this fin runs along. Panics if `fin` is stale.
    pub fn fin_edge(&self, fin: EntityId<Fin>) -> EntityId<Edge> {
        self.fins.get(fin).expect("stale Fin id").edge
    }

    /// The loop this fin belongs to. Panics if `fin` is stale.
    pub fn fin_loop(&self, fin: EntityId<Fin>) -> EntityId<Loop> {
        self.fins.get(fin).expect("stale Fin id").loop_ref
    }

    /// The face this fin bounds. Panics if `fin` is stale.
    pub fn fin_face(&self, fin: EntityId<Fin>) -> EntityId<Face> {
        let loop_id = self.fin_loop(fin);
        self.loops.get(loop_id).expect("stale Loop id").face
    }

    /// Start vertex of this fin, respecting its sense. Panics if `fin` is stale.
    pub fn fin_start_vertex(&self, fin: EntityId<Fin>) -> EntityId<Vertex> {
        let fin = self.fins.get(fin).expect("stale Fin id");
        let edge = self.edges.get(fin.edge).expect("stale Edge id");
        match fin.sense {
            FinSense::Forward => edge.start_vertex,
            FinSense::Reversed => edge.end_vertex,
        }
    }

    /// End vertex of this fin, respecting its sense. Panics if `fin` is stale.
    pub fn fin_end_vertex(&self, fin: EntityId<Fin>) -> EntityId<Vertex> {
        let fin = self.fins.get(fin).expect("stale Fin id");
        let edge = self.edges.get(fin.edge).expect("stale Edge id");
        match fin.sense {
            FinSense::Forward => edge.end_vertex,
            FinSense::Reversed => edge.start_vertex,
        }
    }

    // ------------------------------------------------------------------
    // Upward / adjacency navigation
    // ------------------------------------------------------------------

    /// All fins using an edge (2 manifold, 1 boundary, >2 non-manifold).
    /// Panics if `edge` is stale.
    pub fn fins_of_edge(&self, edge: EntityId<Edge>) -> &[EntityId<Fin>] {
        &self.edges.get(edge).expect("stale Edge id").fins
    }

    /// Faces adjacent to an edge, deduplicated (typically 2 for manifold).
    /// Panics if `edge` is stale.
    pub fn faces_of_edge(&self, edge: EntityId<Edge>) -> Vec<EntityId<Face>> {
        let mut faces = Vec::new();
        for &fin in self.fins_of_edge(edge) {
            let face = self.fin_face(fin);
            if !faces.contains(&face) {
                faces.push(face);
            }
        }
        faces
    }

    /// All edges meeting at a vertex. Panics if `vertex` is stale.
    pub fn edges_of_vertex(&self, vertex: EntityId<Vertex>) -> &[EntityId<Edge>] {
        &self.vertices.get(vertex).expect("stale Vertex id").edges
    }

    /// The body containing a face. Panics if `face` is stale.
    pub fn body_of_face(&self, face: EntityId<Face>) -> EntityId<Body> {
        let shell = self.faces.get(face).expect("stale Face id").shell;
        self.shells.get(shell).expect("stale Shell id").body
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tetrahedron built by hand: 4 vertices, 6 edges, 4 faces, 4 loops,
    /// 12 fins, 1 closed shell, 1 solid body.
    struct Tet {
        store: TopologyStore,
        body: EntityId<Body>,
        shell: EntityId<Shell>,
        vertices: [EntityId<Vertex>; 4],
        edges: [(usize, usize, EntityId<Edge>); 6],
        faces: [EntityId<Face>; 4],
    }

    fn build_tetrahedron() -> Tet {
        let mut store = TopologyStore::new();

        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        let points = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Point3::new(0.0, 0.0, 1.0),
        ];
        let vertices = points.map(|p| store.create_vertex(p, SYSTEM_RESOLUTION));

        // Undirected edges keyed by (low, high) vertex index.
        let edge_pairs = [(0, 1), (0, 2), (0, 3), (1, 2), (1, 3), (2, 3)];
        let edges = edge_pairs.map(|(a, b)| {
            (
                a,
                b,
                store.create_edge(vertices[a], vertices[b], SYSTEM_RESOLUTION),
            )
        });

        let directed_edge = |from: usize, to: usize| -> (EntityId<Edge>, FinSense) {
            let (lo, hi) = if from < to { (from, to) } else { (to, from) };
            let &(_, _, id) = edges
                .iter()
                .find(|&&(a, b, _)| (a, b) == (lo, hi))
                .expect("edge exists");
            let sense = if from < to {
                FinSense::Forward
            } else {
                FinSense::Reversed
            };
            (id, sense)
        };

        // Outward-oriented (CCW from outside) vertex cycles.
        let face_cycles: [[usize; 3]; 4] = [
            [0, 2, 1], // bottom, normal -z
            [0, 1, 3], // side, normal -y
            [1, 2, 3], // slanted, normal (1,1,1)
            [2, 0, 3], // side, normal -x
        ];
        let faces = face_cycles.map(|cycle| {
            let face = store.create_face(shell, FaceSense::Positive);
            let loop_edges: Vec<_> = (0..3)
                .map(|i| directed_edge(cycle[i], cycle[(i + 1) % 3]))
                .collect();
            store.create_loop(face, LoopType::Outer, &loop_edges);
            face
        });

        Tet {
            store,
            body,
            shell,
            vertices,
            edges,
            faces,
        }
    }

    #[test]
    fn tetrahedron_entity_counts() {
        let tet = build_tetrahedron();
        let s = &tet.store;
        assert_eq!(s.bodies.len(), 1);
        assert_eq!(s.shells.len(), 1);
        assert_eq!(s.faces.len(), 4);
        assert_eq!(s.loops.len(), 4);
        assert_eq!(s.fins.len(), 12);
        assert_eq!(s.edges.len(), 6);
        assert_eq!(s.vertices.len(), 4);
        // Euler formula for a genus-0 closed shell: V - E + F = 2.
        let euler = s.vertices.len() as i64 - s.edges.len() as i64 + s.faces.len() as i64;
        assert_eq!(euler, 2);
    }

    #[test]
    fn downward_navigation_body_to_fins() {
        let tet = build_tetrahedron();
        let s = &tet.store;

        assert_eq!(s.shells_of_body(tet.body), &[tet.shell]);

        let faces = s.faces_of_body(tet.body);
        assert_eq!(faces.len(), 4);
        assert_eq!(faces, tet.faces.to_vec());

        for &face in &tet.faces {
            let loops = s.loops_of_face(face);
            assert_eq!(loops.len(), 1, "each tet face has exactly its outer loop");
            assert_eq!(s.face(face).unwrap().outer_loop, Some(loops[0]));
            assert_eq!(s.loop_(loops[0]).unwrap().loop_type, LoopType::Outer);

            let fins = s.fins_of_loop(loops[0]);
            assert_eq!(fins.len(), 3, "triangular face has 3 fins");
            assert_eq!(s.edges_of_face(face).len(), 3);
        }
    }

    #[test]
    fn loop_fin_cycle_is_closed() {
        let tet = build_tetrahedron();
        let s = &tet.store;

        for &face in &tet.faces {
            let loop_id = s.loops_of_face(face)[0];
            let fins = s.fins_of_loop(loop_id).to_vec();

            // next-links walk the cycle and return to the start.
            let mut fin = fins[0];
            for _ in 0..3 {
                assert_eq!(s.fin_face(fin), face);
                assert_eq!(s.fin_loop(fin), loop_id);
                // Each fin's end vertex is the next fin's start vertex.
                assert_eq!(
                    s.fin_end_vertex(fin),
                    s.fin_start_vertex(s.fin_next(fin)),
                    "loop must be vertex-continuous"
                );
                // prev is the inverse of next.
                assert_eq!(s.fin_prev(s.fin_next(fin)), fin);
                fin = s.fin_next(fin);
            }
            assert_eq!(fin, fins[0], "3 steps around a triangle return home");
        }
    }

    #[test]
    fn edge_mates_are_mutual_and_opposed() {
        let tet = build_tetrahedron();
        let s = &tet.store;

        for &(_, _, edge) in &tet.edges {
            let fins = s.fins_of_edge(edge);
            assert_eq!(fins.len(), 2, "manifold closed shell: 2 fins per edge");

            let (a, b) = (fins[0], fins[1]);
            assert_eq!(s.fin_mate(a), Some(b));
            assert_eq!(s.fin_mate(b), Some(a));
            // Consistent outward orientation → mates traverse the edge in
            // opposite directions.
            let (fa, fb) = (s.fin(a).unwrap(), s.fin(b).unwrap());
            assert_eq!(fa.sense, fb.sense.opposite());
            // Mates bound two distinct faces.
            assert_ne!(s.fin_face(a), s.fin_face(b));
            assert_eq!(s.faces_of_edge(edge).len(), 2);
        }
    }

    #[test]
    fn upward_navigation() {
        let tet = build_tetrahedron();
        let s = &tet.store;

        for &face in &tet.faces {
            assert_eq!(s.body_of_face(face), tet.body);
        }
        for (i, &vertex) in tet.vertices.iter().enumerate() {
            let edges = s.edges_of_vertex(vertex);
            assert_eq!(edges.len(), 3, "vertex {i}: 3 edges meet at a tet corner");
        }
    }

    #[test]
    fn face_vertices_follow_construction_cycle() {
        let tet = build_tetrahedron();
        let s = &tet.store;
        let expected = [[0, 2, 1], [0, 1, 3], [1, 2, 3], [2, 0, 3]];
        for (face, cycle) in tet.faces.iter().zip(expected) {
            let got = s.vertices_of_face(*face);
            let want: Vec<_> = cycle.iter().map(|&i| tet.vertices[i]).collect();
            assert_eq!(got, want);
        }
    }

    #[test]
    fn id_stability_across_removal() {
        let mut tet = build_tetrahedron();

        // Add an unconnected scratch vertex, then remove it.
        let scratch = tet
            .store
            .create_vertex(Point3::new(9.0, 9.0, 9.0), SYSTEM_RESOLUTION);
        assert!(tet.store.vertex(scratch).is_some());
        tet.store.vertices.remove(scratch);
        assert!(
            tet.store.vertex(scratch).is_none(),
            "removed id must not resolve"
        );

        // A new vertex may reuse the slot, but the stale id stays invalid.
        let replacement = tet
            .store
            .create_vertex(Point3::new(8.0, 8.0, 8.0), SYSTEM_RESOLUTION);
        assert_ne!(replacement, scratch);
        assert!(tet.store.vertex(scratch).is_none());
        assert!(tet.store.vertex(replacement).is_some());

        // Untouched entities are unaffected.
        for &v in &tet.vertices {
            assert!(tet.store.vertex(v).is_some());
        }
        assert_eq!(tet.store.faces_of_body(tet.body).len(), 4);
    }

    #[test]
    fn tolerance_fields() {
        let mut store = TopologyStore::new();
        let a = store.create_vertex(Point3::new(0.0, 0.0, 0.0), SYSTEM_RESOLUTION);
        let b = store.create_vertex(Point3::new(1.0, 0.0, 0.0), 1e-4);

        assert!(!store.vertex(a).unwrap().is_tolerant());
        assert!(store.vertex(b).unwrap().is_tolerant());

        let precise = store.create_edge(a, b, SYSTEM_RESOLUTION);
        let tolerant = store.create_edge(a, b, 1e-5);
        assert!(!store.edge(precise).unwrap().is_tolerant());
        assert!(store.edge(tolerant).unwrap().is_tolerant());
    }

    #[test]
    fn pcurve_and_curve_slots_accept_geometry_ids() {
        let mut tet = build_tetrahedron();

        // Stand-in geometry arena; the real geometry layer replaces this.
        let mut curves: Arena<Curve> = Arena::new();
        let pcurve_id = curves.insert(Curve);
        let curve_id = curves.insert(Curve);

        let loop_id = tet.store.loops_of_face(tet.faces[0])[0];
        let fin_id = tet.store.fins_of_loop(loop_id)[0];
        tet.store.fins.get_mut(fin_id).unwrap().pcurve = Some(pcurve_id);
        assert_eq!(tet.store.fin(fin_id).unwrap().pcurve, Some(pcurve_id));

        let edge_id = tet.store.fin_edge(fin_id);
        tet.store.edges.get_mut(edge_id).unwrap().curve = Some(curve_id);
        assert_eq!(tet.store.edge(edge_id).unwrap().curve, Some(curve_id));
    }

    #[test]
    fn boundary_edge_has_single_fin_and_no_mate() {
        let mut store = TopologyStore::new();
        let body = store.create_body(BodyType::Sheet);
        let shell = store.create_shell(body, false, ShellOrientation::Outward);
        let face = store.create_face(shell, FaceSense::Positive);

        let v: Vec<_> = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ]
        .iter()
        .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
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

        // A lone sheet face: every edge is a boundary edge with one fin.
        for edge in edges {
            let fins = store.fins_of_edge(edge);
            assert_eq!(fins.len(), 1);
            assert_eq!(store.fin_mate(fins[0]), None);
            assert_eq!(store.faces_of_edge(edge), vec![face]);
        }
    }

    #[test]
    fn inner_loop_registers_as_hole() {
        let mut store = TopologyStore::new();
        let body = store.create_body(BodyType::Sheet);
        let shell = store.create_shell(body, false, ShellOrientation::Outward);
        let face = store.create_face(shell, FaceSense::Positive);

        let make_triangle = |store: &mut TopologyStore, offset: f64| {
            let v: Vec<_> = [
                Point3::new(offset, 0.0, 0.0),
                Point3::new(offset + 1.0, 0.0, 0.0),
                Point3::new(offset, 1.0, 0.0),
            ]
            .iter()
            .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
            .collect();
            [
                (
                    store.create_edge(v[0], v[1], SYSTEM_RESOLUTION),
                    FinSense::Forward,
                ),
                (
                    store.create_edge(v[1], v[2], SYSTEM_RESOLUTION),
                    FinSense::Forward,
                ),
                (
                    store.create_edge(v[2], v[0], SYSTEM_RESOLUTION),
                    FinSense::Forward,
                ),
            ]
        };

        let outer_edges = make_triangle(&mut store, 0.0);
        let hole_edges = make_triangle(&mut store, 10.0);
        let outer = store.create_loop(face, LoopType::Outer, &outer_edges);
        let hole = store.create_loop(face, LoopType::Inner, &hole_edges);

        let f = store.face(face).unwrap();
        assert_eq!(f.outer_loop, Some(outer));
        assert_eq!(f.inner_loops, vec![hole]);
        assert_eq!(store.loops_of_face(face), vec![outer, hole]);
        assert_eq!(store.loop_(hole).unwrap().loop_type, LoopType::Inner);
    }
}
