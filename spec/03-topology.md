# 03 — Topology

The connectivity graph: bodies, shells, faces, loops, edges, vertices, and their relationships.

## 1. The Topology Store

All topological entities live in a centralized store with typed arenas:

```rust
pub struct TopologyStore {
    pub bodies: Arena<Body>,
    pub regions: Arena<Region>,
    pub shells: Arena<Shell>,
    pub faces: Arena<Face>,
    pub loops: Arena<Loop>,
    pub fins: Arena<Fin>,
    pub edges: Arena<Edge>,
    pub vertices: Arena<Vertex>,
}
```

### 1.1 Arena Allocator

```rust
pub struct Arena<T> {
    entries: Vec<Option<ArenaEntry<T>>>,
    generations: Vec<u32>,
    free_list: Vec<u32>,
    count: usize,
}

struct ArenaEntry<T> {
    value: T,
}

impl<T> Arena<T> {
    pub fn insert(&mut self, value: T) -> EntityId<T>;
    pub fn remove(&mut self, id: EntityId<T>) -> Option<T>;
    pub fn get(&self, id: EntityId<T>) -> Option<&T>;
    pub fn get_mut(&mut self, id: EntityId<T>) -> Option<&mut T>;
    pub fn iter(&self) -> impl Iterator<Item = (EntityId<T>, &T)>;
    pub fn len(&self) -> usize;
}
```

### 1.2 Entity References

```rust
/// Typed, generation-checked reference to an entity.
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct EntityId<T> {
    index: u32,
    generation: u32,
    _phantom: PhantomData<T>,
}

impl<T> EntityId<T> {
    pub fn is_null(&self) -> bool { self.index == u32::MAX }
    pub const NULL: Self = Self { index: u32::MAX, generation: 0, _phantom: PhantomData };
}
```

## 2. Traversal Operations

Efficient traversal of the topology graph is essential for all operations.

### 2.1 Upward Traversal (child → parent)

```rust
impl TopologyStore {
    /// Get the body containing a face.
    pub fn face_body(&self, face: EntityId<Face>) -> EntityId<Body>;

    /// Get the shell containing a face.
    pub fn face_shell(&self, face: EntityId<Face>) -> EntityId<Shell>;

    /// Get all faces adjacent to an edge (typically 2 for manifold).
    pub fn edge_faces(&self, edge: EntityId<Edge>) -> Vec<EntityId<Face>>;

    /// Get all edges meeting at a vertex.
    pub fn vertex_edges(&self, vertex: EntityId<Vertex>) -> Vec<EntityId<Edge>>;

    /// Get all faces meeting at a vertex.
    pub fn vertex_faces(&self, vertex: EntityId<Vertex>) -> Vec<EntityId<Face>>;
}
```

### 2.2 Downward Traversal (parent → child)

```rust
impl TopologyStore {
    /// Get all faces of a body.
    pub fn body_faces(&self, body: EntityId<Body>) -> Vec<EntityId<Face>>;

    /// Get all edges of a body.
    pub fn body_edges(&self, body: EntityId<Body>) -> Vec<EntityId<Edge>>;

    /// Get all vertices of a body.
    pub fn body_vertices(&self, body: EntityId<Body>) -> Vec<EntityId<Vertex>>;

    /// Get all edges bounding a face.
    pub fn face_edges(&self, face: EntityId<Face>) -> Vec<EntityId<Edge>>;

    /// Get the ordered vertices of a face's outer loop.
    pub fn face_vertices(&self, face: EntityId<Face>) -> Vec<EntityId<Vertex>>;
}
```

### 2.3 Adjacency Traversal (sibling)

```rust
impl TopologyStore {
    /// Get the face on the other side of an edge.
    pub fn adjacent_face(&self, face: EntityId<Face>, edge: EntityId<Edge>) -> Option<EntityId<Face>>;

    /// Get all faces adjacent to a face (sharing an edge).
    pub fn adjacent_faces(&self, face: EntityId<Face>) -> Vec<EntityId<Face>>;

    /// Get all edges adjacent to an edge (sharing a vertex).
    pub fn adjacent_edges(&self, edge: EntityId<Edge>) -> Vec<EntityId<Edge>>;

    /// Walk around a vertex: alternating face-edge-face-edge cycle.
    pub fn vertex_ring(&self, vertex: EntityId<Vertex>) -> Vec<VertexRingEntry>;
}

pub enum VertexRingEntry {
    Face(EntityId<Face>),
    Edge(EntityId<Edge>),
}
```

## 3. Topology Construction (Builder API)

Bodies are constructed bottom-up or via the operations layer. The builder API
provides low-level construction with invariant checking:

```rust
pub struct TopologyBuilder<'a> {
    store: &'a mut TopologyStore,
    geometry: &'a mut GeometryStore,
}

impl<'a> TopologyBuilder<'a> {
    /// Create a vertex at a point.
    pub fn make_vertex(&mut self, point: Point3) -> EntityId<Vertex>;

    /// Create an edge between two vertices on a curve.
    pub fn make_edge(
        &mut self,
        curve: EntityId<Curve>,
        start: EntityId<Vertex>,
        end: EntityId<Vertex>,
        t_start: f64,
        t_end: f64,
    ) -> EntityId<Edge>;

    /// Create a loop from an ordered sequence of edges.
    /// Automatically creates fins and sets up next/prev/mate links.
    pub fn make_loop(
        &mut self,
        edges: &[(EntityId<Edge>, bool)],  // (edge, reversed?)
        face: EntityId<Face>,
        loop_type: LoopType,
    ) -> EntityId<Loop>;

    /// Create a face on a surface with loops.
    pub fn make_face(
        &mut self,
        surface: EntityId<Surface>,
        sense: FaceSense,
        shell: EntityId<Shell>,
    ) -> EntityId<Face>;

    /// Create a shell containing faces.
    pub fn make_shell(
        &mut self,
        body: EntityId<Body>,
        is_closed: bool,
    ) -> EntityId<Shell>;

    /// Create a body.
    pub fn make_body(&mut self, body_type: BodyType) -> EntityId<Body>;

    /// Validate the constructed topology.
    pub fn validate(&self) -> Result<(), Vec<TopologyError>>;
}
```

## 4. Primitive Body Construction

High-level constructors for common solid primitives:

```rust
pub struct Primitives;

impl Primitives {
    /// Create a rectangular block (cuboid).
    pub fn block(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        x_size: f64,
        y_size: f64,
        z_size: f64,
    ) -> EntityId<Body>;

    /// Create a cylinder.
    pub fn cylinder(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        radius: f64,
        height: f64,
    ) -> EntityId<Body>;

    /// Create a cone (truncated if top_radius > 0).
    pub fn cone(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        bottom_radius: f64,
        top_radius: f64,
        height: f64,
    ) -> EntityId<Body>;

    /// Create a sphere.
    pub fn sphere(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        radius: f64,
    ) -> EntityId<Body>;

    /// Create a torus.
    pub fn torus(
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        major_radius: f64,
        minor_radius: f64,
    ) -> EntityId<Body>;
}
```

## 5. Euler Operations

The fundamental topology-modifying operations that preserve the Euler-Poincaré formula:
V - E + F - (L - F) - 2(S - G) = 0

These are the primitive building blocks from which all topology changes are composed:

```rust
pub trait EulerOps {
    /// Make Vertex Face Shell (create initial body with one vertex, one face, one shell).
    fn mvfs(&mut self) -> (EntityId<Body>, EntityId<Vertex>, EntityId<Face>, EntityId<Shell>);

    /// Make Edge Vertex (split a vertex into two by inserting an edge).
    fn mev(
        &mut self,
        vertex: EntityId<Vertex>,
        face: EntityId<Face>,
        point: Point3,
    ) -> (EntityId<Edge>, EntityId<Vertex>);

    /// Make Edge Face (split a face into two by inserting an edge between two vertices).
    fn mef(
        &mut self,
        vertex_a: EntityId<Vertex>,
        vertex_b: EntityId<Vertex>,
        face: EntityId<Face>,
    ) -> (EntityId<Edge>, EntityId<Face>);

    /// Kill Edge Make Ring (remove an edge, creating a new loop/ring).
    fn kemr(
        &mut self,
        edge: EntityId<Edge>,
    ) -> EntityId<Loop>;

    /// Kill Face Make Ring Hole (remove a face, merging two shells).
    fn kfmrh(
        &mut self,
        face: EntityId<Face>,
    ) -> EntityId<Loop>;

    /// Make Shell (create a new empty shell in a body).
    fn ms(&mut self, body: EntityId<Body>) -> EntityId<Shell>;
}
```

## 6. Topology Queries

```rust
impl TopologyStore {
    /// Count entities of each type in a body.
    pub fn body_stats(&self, body: EntityId<Body>) -> BodyStats;

    /// Compute the Euler characteristic of a shell.
    pub fn euler_characteristic(&self, shell: EntityId<Shell>) -> i32;

    /// Determine body type from topology.
    pub fn classify_body(&self, body: EntityId<Body>) -> BodyType;

    /// Find shared edges between two faces.
    pub fn shared_edges(
        &self,
        face_a: EntityId<Face>,
        face_b: EntityId<Face>,
    ) -> Vec<EntityId<Edge>>;

    /// Find shared vertices between two edges.
    pub fn shared_vertices(
        &self,
        edge_a: EntityId<Edge>,
        edge_b: EntityId<Edge>,
    ) -> Vec<EntityId<Vertex>>;

    /// Find all connected components (separate shells).
    pub fn connected_components(&self, body: EntityId<Body>) -> Vec<Vec<EntityId<Face>>>;

    /// Check if a shell is manifold (every edge has exactly 2 faces).
    pub fn is_manifold(&self, shell: EntityId<Shell>) -> bool;

    /// Find boundary edges (edges with only 1 adjacent face — sheet body boundary).
    pub fn boundary_edges(&self, shell: EntityId<Shell>) -> Vec<EntityId<Edge>>;
}

pub struct BodyStats {
    pub vertices: usize,
    pub edges: usize,
    pub faces: usize,
    pub loops: usize,
    pub shells: usize,
    pub regions: usize,
}
```

## 7. Topology Modification Utilities

```rust
impl TopologyStore {
    /// Merge two vertices that are within tolerance.
    pub fn merge_vertices(
        &mut self,
        v1: EntityId<Vertex>,
        v2: EntityId<Vertex>,
    ) -> Result<EntityId<Vertex>, TopologyError>;

    /// Split an edge at a parameter value (inserting a new vertex).
    pub fn split_edge(
        &mut self,
        edge: EntityId<Edge>,
        t: f64,
    ) -> (EntityId<Edge>, EntityId<Edge>, EntityId<Vertex>);

    /// Merge two edges that share a vertex (removing the vertex).
    pub fn merge_edges(
        &mut self,
        edge_a: EntityId<Edge>,
        edge_b: EntityId<Edge>,
    ) -> Result<EntityId<Edge>, TopologyError>;

    /// Reverse the orientation of a face.
    pub fn reverse_face(&mut self, face: EntityId<Face>);

    /// Reverse the orientation of all faces in a shell.
    pub fn reverse_shell(&mut self, shell: EntityId<Shell>);

    /// Sew two sheet bodies together along matching edges.
    pub fn sew(
        &mut self,
        body_a: EntityId<Body>,
        body_b: EntityId<Body>,
        tolerance: f64,
    ) -> Result<EntityId<Body>, TopologyError>;
}
```

## 8. Half-Edge (Fin) Navigation

The fin structure enables efficient O(1) navigation around faces:

```rust
impl TopologyStore {
    /// Get the next fin in the loop (counter-clockwise on outer loop).
    pub fn fin_next(&self, fin: EntityId<Fin>) -> EntityId<Fin>;

    /// Get the previous fin in the loop.
    pub fn fin_prev(&self, fin: EntityId<Fin>) -> EntityId<Fin>;

    /// Get the mate fin (same edge, other face).
    pub fn fin_mate(&self, fin: EntityId<Fin>) -> EntityId<Fin>;

    /// Get the face that this fin bounds.
    pub fn fin_face(&self, fin: EntityId<Fin>) -> EntityId<Face>;

    /// Get the edge this fin is on.
    pub fn fin_edge(&self, fin: EntityId<Fin>) -> EntityId<Edge>;

    /// Get the start vertex of this fin (respecting fin sense).
    pub fn fin_start_vertex(&self, fin: EntityId<Fin>) -> EntityId<Vertex>;

    /// Get the end vertex of this fin (respecting fin sense).
    pub fn fin_end_vertex(&self, fin: EntityId<Fin>) -> EntityId<Vertex>;

    /// Walk around a face: iterate all fins in order.
    pub fn face_fin_iter(&self, face: EntityId<Face>) -> FaceFinIterator;

    /// Orbit around a vertex: all fins emanating from it.
    pub fn vertex_fin_iter(&self, vertex: EntityId<Vertex>) -> VertexFinIterator;
}
```

## 9. Non-Manifold Topology

For sheet bodies and general bodies, the topology relaxes manifold constraints:

- Edges may have 1 adjacent face (boundary edge) or >2 (non-manifold edge)
- Vertices may be non-manifold (multiple edge fans meeting at one point)

```rust
pub struct NonManifoldEdge {
    pub edge: EntityId<Edge>,
    pub fins: Vec<EntityId<Fin>>,      // More than 2 fins
}

impl TopologyStore {
    /// Check if an edge is non-manifold (>2 adjacent faces).
    pub fn is_non_manifold_edge(&self, edge: EntityId<Edge>) -> bool;

    /// Check if a vertex is non-manifold.
    pub fn is_non_manifold_vertex(&self, vertex: EntityId<Vertex>) -> bool;

    /// Get all non-manifold edges in a body.
    pub fn non_manifold_edges(&self, body: EntityId<Body>) -> Vec<NonManifoldEdge>;
}
```
