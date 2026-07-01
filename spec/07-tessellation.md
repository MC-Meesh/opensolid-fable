# 07 — Tessellation

Converting exact B-rep geometry into triangle meshes for visualization and export.

## 1. Overview

Tessellation (faceting) converts smooth B-rep surfaces into discrete triangle meshes.
Used for:
- Visualization / rendering
- STL/OBJ export for 3D printing
- Approximate distance/collision queries
- Point-in-solid testing (ray casting against mesh)

The challenge: balance fidelity (mesh approximates surface within tolerance) against
performance (fewer triangles = faster rendering).

## 2. Requirements

- **Watertight**: No gaps between adjacent face meshes along shared edges
- **Manifold**: Valid triangle mesh topology (no self-intersections, no T-junctions)
- **Adaptive**: More triangles in curved regions, fewer on flat surfaces
- **Parametric control**: User specifies maximum chordal deviation, maximum edge length,
  minimum edge length, and/or target triangle count
- **Consistent**: Deterministic output for same input (important for testing)

## 3. Algorithm

### 3.1 Per-Face Tessellation

Each face is tessellated independently, then stitched at shared edges.

```rust
pub struct TessellationOptions {
    /// Maximum distance from mesh to actual surface (chordal tolerance).
    pub chord_tolerance: f64,
    /// Maximum angle between adjacent triangle normals (angular tolerance, radians).
    pub angle_tolerance: f64,
    /// Maximum edge length in the mesh.
    pub max_edge_length: Option<f64>,
    /// Minimum edge length (avoids over-refinement at singularities).
    pub min_edge_length: Option<f64>,
    /// Compute vertex normals (for smooth shading).
    pub compute_normals: bool,
    /// Compute UV coordinates on each vertex.
    pub compute_uvs: bool,
}

pub struct TriangleMesh {
    pub vertices: Vec<Point3>,
    pub triangles: Vec<[u32; 3]>,      // Indices into vertices
    pub normals: Option<Vec<Vector3>>,  // Per-vertex normals
    pub uvs: Option<Vec<(f64, f64)>>,  // Per-vertex UV coordinates
}

pub struct FaceMesh {
    pub face: EntityId<Face>,
    pub mesh: TriangleMesh,
    pub edge_vertex_map: HashMap<EntityId<Edge>, Vec<u32>>, // Edge → vertex indices on that edge
}

impl Kernel {
    /// Tessellate a single face.
    pub fn tessellate_face(
        &self,
        face: EntityId<Face>,
        options: &TessellationOptions,
    ) -> FaceMesh;

    /// Tessellate an entire body (all faces, stitched at edges).
    pub fn tessellate_body(
        &self,
        body: EntityId<Body>,
        options: &TessellationOptions,
    ) -> BodyMesh;

    /// Tessellate multiple bodies in parallel.
    pub fn tessellate_bodies(
        &self,
        bodies: &[EntityId<Body>],
        options: &TessellationOptions,
    ) -> Vec<BodyMesh>;
}

pub struct BodyMesh {
    pub body: EntityId<Body>,
    pub mesh: TriangleMesh,
    pub face_ranges: Vec<(EntityId<Face>, std::ops::Range<usize>)>, // Face → triangle range
}
```

### 3.2 Edge Discretization

Edges must be discretized first, and adjacent faces must agree on edge vertices
(otherwise gaps appear).

```rust
/// Discretize an edge curve into a polyline.
///
/// Algorithm:
/// 1. Evaluate curve at endpoints
/// 2. Adaptively subdivide: insert midpoint if chord deviation > tolerance
/// 3. Also subdivide if angle between tangents > angle tolerance
/// 4. Respect min/max edge length constraints
pub fn discretize_edge(
    edge: &Edge,
    curve: &Curve,
    options: &TessellationOptions,
) -> Vec<(f64, Point3)>;  // (parameter, point) pairs
```

### 3.3 Face Interior Triangulation

After edge discretization, the face interior is triangulated:

```
1. MAP TO PARAMETER SPACE
   - Project face boundary (loop vertices) into (u,v) parameter space
   - This gives a 2D polygon with holes (inner loops)

2. CONSTRAINED DELAUNAY TRIANGULATION (CDT)
   - Triangulate the 2D polygon using CDT
   - Edge constraints ensure boundary edges appear in the triangulation
   - Inner loops create holes in the triangulation

3. ADAPTIVE REFINEMENT
   - For each triangle: evaluate surface at centroid
   - Check if triangle satisfies chord tolerance and angle tolerance
   - If not: insert Steiner point (midpoint of longest edge or centroid)
   - Re-triangulate locally
   - Repeat until all triangles satisfy tolerances

4. MAP BACK TO 3D
   - Evaluate surface at each (u,v) vertex to get 3D positions
   - Compute normals at each vertex (for smooth shading)
```

### 3.4 Stitching

```rust
/// Stitch face meshes along shared edges to ensure watertight mesh.
///
/// Each shared edge has been discretized identically (same points).
/// Stitching simply merges corresponding vertex indices across faces.
pub fn stitch_face_meshes(
    face_meshes: &[FaceMesh],
    topology: &TopologyStore,
) -> TriangleMesh;
```

## 4. Special Cases

### 4.1 Degenerate Faces

Faces with singularities (sphere poles, cone apex) need special treatment:
- The singularity maps to a single vertex in the mesh
- Triangles adjacent to the singularity are degenerate in parameter space
- Use fan triangulation around the singular vertex

### 4.2 Periodic Faces

Faces on periodic surfaces (full cylinder, full sphere) where the parameter domain wraps:
- Must handle the seam correctly (don't create a gap at u=0/u=2π)
- Duplicate seam vertices in UV space but share them in 3D space

### 4.3 High-Curvature Regions

Near sharp features (small fillet radii), tessellation must be dense:
- Curvature-based refinement naturally handles this
- But may produce extremely small triangles — enforce minimum edge length

## 5. Output Formats

```rust
pub struct StlExporter;

impl StlExporter {
    /// Export a body mesh as ASCII STL.
    pub fn export_ascii(mesh: &BodyMesh, path: &Path) -> io::Result<()>;
    /// Export a body mesh as binary STL.
    pub fn export_binary(mesh: &BodyMesh, path: &Path) -> io::Result<()>;
}

pub struct ObjExporter;

impl ObjExporter {
    /// Export as Wavefront OBJ (with normals and UVs).
    pub fn export(mesh: &BodyMesh, path: &Path) -> io::Result<()>;
}

pub struct GltfExporter;

impl GltfExporter {
    /// Export as glTF 2.0 (for web/3D viewers).
    pub fn export(mesh: &BodyMesh, path: &Path) -> io::Result<()>;
}
```

## 6. Performance Considerations

- **Parallelism**: Each face can be tessellated independently → use rayon for parallel face tessellation
- **Caching**: Cache edge discretizations (each edge is shared by 2 faces)
- **Progressive**: Could support level-of-detail (coarse mesh first, refine on demand)
- **Memory**: For large models, stream triangles to disk rather than accumulating in memory
