# Parasolid Architecture Reference

Comprehensive reference scraped from Parasolid's public documentation (V13, V35),
the X_T Format Reference (April 2008), and supplementary sources.

This document is the raw research that informs the OpenSolid spec. Implementers
should consult this for Parasolid-specific details when the spec references
"modeled after Parasolid."

---

## Sources

- [Parasolid V13.0 Functional Description](http://www.q-solid.com/Parasolid_Docs/fd_index.html)
- [Parasolid V35 Functional Description](http://www.q-solid.com/Parasolid_Docs_V35/)
- [Parasolid XT Format Reference PDF (13thmonkey.org)](http://www.13thmonkey.org/documentation/CAD/Parasolid-XT-format-reference.pdf)
- [Parasolid XT Format Reference (silo.tips)](https://silo.tips/download/parasolid-xt-format-reference-april-2008)
- [Hand-Decoded XT File Example (Okino)](https://www.okino.com/solutions/sphere_parasolid_file_example_explained.htm)
- [Parasolid Format Overview (CAD Exchanger)](https://cadexchanger.com/blog/3d-formats-overview-parasolid/)
- [Parasolid Convergent Modeling (Siemens blog)](https://blogs.sw.siemens.com/plm-components/parasolid-with-convergent-modeling/)
- [Parasolid v33.0 Release Highlights](https://blogs.sw.siemens.com/plm-components/parasolid-v33-0-release-highlights/)
- [Rhino to Parasolid Entity Mapping (ProtoTech)](https://prototechsolutions.com/cad-notes/rhino-to-parasolid-cad-data-translation-the-entity-map-bible/)
- [Parasolid Primer (ProtoTech)](https://prototechsolutions.com/cad-notes/a-15-minute-primer-to-parasolid-geometry-modeler/)
- [Session and Local Precision (V35)](http://www.q-solid.com/Parasolid_Docs_V35/chapters/fd_chap.017.html)
- [Edge Blending Overview (V35)](http://www.q-solid.com/Parasolid_Docs_V35/chapters/fd_chap.075.html)
- [Overview of Convergent Modeling (V35)](http://www.q-solid.com/Parasolid_Docs_V35/chapters/fd_chap.083.html)
- [Facet Model Structure (V35)](http://www.q-solid.com/Parasolid_Docs_V35/chapters/fd_chap.084.html)
- [Parasolid Frustrum GitHub (deGravity)](https://github.com/deGravity/parasolid_frustrum)
- [Tolerant Modeling Paper (Jackson, 1995)](https://dl.acm.org/doi/pdf/10.1145/218013.218067)
- [HOOPS Exchange Parasolid Reader](https://docs.techsoft3d.com/hoops/exchange/start/format/parasolid_reader.html)
- [Parasolid Wikipedia](https://en.wikipedia.org/wiki/Parasolid)

---

## 1. Topology Hierarchy (Exact Parasolid Model)

### Containment Order

```
BODY (PK_CLASS_body)
  └── REGION (PK_CLASS_region) — open connected subset of 3D space
        └── SHELL (PK_CLASS_shell) — connected collection of oriented faces/edges
              └── FACE (PK_CLASS_face) — bounded subset of a surface
                    └── LOOP (PK_CLASS_loop) — connected component of face boundary
                          └── FIN (PK_CLASS_fin) — oriented use of edge by loop
                                └── EDGE (PK_CLASS_edge) — bounded piece of curve
                                      └── VERTEX (PK_CLASS_vertex) — point in space
```

### Key Relationships

- A **body** always has one infinite void region
- **Regions** are either solid or void; they do not overlap except at boundaries
- **Shells** are the boundaries of regions; a shell is a connected collection of oriented faces
- **Faces** reference a geometric surface; their boundary is defined by loops
- **Loops** consist of fins (half-edges) that reference edges
- **Fins** represent the oriented use of an edge by a loop; each edge has exactly two fins (one per adjacent face). When an edge is tolerant, each fin references an SP-curve
- **Edges** reference a geometric curve bounded by vertices
- **Vertices** reference a geometric point (PK_POINT)

### Entity Tags

- Every entity in a session has a unique integer **tag** for identification
- Tags are NOT persistent across sessions (transmit/receive)
- For persistent identification, use `PK_ENTITY_ask_identifier`
- Tag persistence rules: if a face shrinks, its tag persists; if split, one resulting face inherits the original tag

---

## 2. Body Types

| Type | Description |
|------|-------------|
| **Solid body** | Manifold body with at least one finite solid region; fully enclosed volume |
| **Sheet body** | Zero-thickness surface (one or more faces with no enclosed volume) |
| **Wire body** | One-dimensional body consisting only of edges and vertices |
| **Minimum body** | Degenerate body with single vertex (point body) |
| **General body** | Non-manifold, mixed-dimension, cellular, or disconnected; any combination in a single valid body |
| **Compound body** | Collection of related bodies sharing physical aspects (e.g., different representations of same part) |

### Manifold vs. General Bodies

- **Manifold bodies**: Traditional B-rep; every edge shared by exactly 2 faces; well-defined inside/outside
- **General bodies**: Allow non-manifold topology (edge shared by 3+ faces), mixed dimensions (wire edges attached to solid), cellular decomposition (multiple adjacent solid regions), and disconnected components

---

## 3. Geometry Types — Complete Enumeration

### 3.1 Curve Types (PK_CLASS_curve subclasses)

| Type | PK Class | Description | Parameterization |
|------|----------|-------------|-----------------|
| Line | PK_CLASS_line | Infinite straight line | P(t) = origin + t × direction |
| Circle | PK_CLASS_circle | Full circle | P(t) = center + r×cos(t)×ref + r×sin(t)×(axis×ref) |
| Ellipse | PK_CLASS_ellipse | Full ellipse | P(t) = center + a×cos(t)×major + b×sin(t)×minor |
| B-curve (NURBS) | PK_CLASS_bcurve | Non-uniform rational B-spline curve | De Boor evaluation |
| Intersection curve | (procedural) | Branch of surface/surface intersection | Computed from adjacent face intersection |
| SP-curve | (procedural) | Surface-Parameter curve; lives in face's UV space | 2D B-spline in parameter space |
| PE-curve | PK_CLASS_pecurve | Parametrically Evaluated (foreign/procedural) | Application-defined |
| Polyline | PK_CLASS_pline | Connected chain of linear segments | Piecewise linear |
| Offset curve | — | Curve offset from base at constant distance | base(t) + d×normal(t) |
| Trimmed curve | — | Subset of parent curve | Reparameterized portion |
| Spun curve | — | Generated by rotation | Axis spin |

### 3.2 Surface Types (PK_CLASS_surf subclasses)

| Type | PK Class | Description | Parameterization |
|------|----------|-------------|-----------------|
| Plane | PK_CLASS_plane | Infinite flat surface | P(u,v) = origin + u×u_axis + v×v_axis |
| Cylinder | PK_CLASS_cyl | Infinite circular cylinder | P(u,v) = origin + v×axis + r×(cos(u)×ref + sin(u)×(axis×ref)) |
| Cone | PK_CLASS_cone | Infinite circular cone | Like cylinder but radius varies linearly with v |
| Sphere | PK_CLASS_sphere | Complete sphere | P(u,v) = center + r×(cos(v)×cos(u)×ref + cos(v)×sin(u)×cross + sin(v)×axis) |
| Torus | PK_CLASS_torus | Complete torus | Standard toroidal parameterization |
| B-surface (NURBS) | PK_CLASS_bsurf | Non-uniform rational B-spline surface | Tensor-product De Boor |
| Blend surface | PK_CLASS_blendsf | Rolling-ball blend surface | Procedural (from blend operation) |
| Offset surface | PK_CLASS_offset | Surface offset from base by distance | base(u,v) + d×normal(u,v) |
| Swept surface | PK_CLASS_swept | Profile curve swept linearly | Profile(u) + v×direction |
| Spun surface | PK_CLASS_spun | Profile curve spun about axis | R(u,v) = Z(u) + (C(u)-Z(u))cos(v) + A×(C(u)-Z(u))sin(v) |
| Foreign surface | PK_CLASS_fsurf | Application-defined procedural surface | Application callback |
| PE-surface | — | Parametrically Evaluated (foreign) | Application-defined |
| Mesh | PK_CLASS_mesh | Triangulated facet geometry (convergent modeling) | Discrete |

### 3.3 Important Notes

- Analytic geometry (plane, cylinder, cone, sphere, torus) is defined implicitly and is infinite/closed
- B-geometry (bcurve, bsurf) is defined over a finite parameter region
- Procedural geometry (intersection curves, blend surfaces) requires evaluation rather than having explicit NURBS form
- SP-curves (Surface Parameter curves) are 2D B-splines living in a face's UV parameter space — used for tolerant edge representation
- When converting intersection curves to explicit form, the result is typically a B-spline approximation

---

## 4. PK API — Function Organization

The PK API has 900+ functions organized by entity class prefix:

### 4.1 Session & System

```
PK_SESSION_start              — Initialize kernel session
PK_SESSION_stop               — Terminate session
PK_SESSION_register           — Register frustrum callbacks
PK_SESSION_set_precision      — Set session precision
PK_SESSION_ask_precision      — Query session precision
PK_SESSION_comment            — Add comment to journal
PK_MEMORY_alloc               — Allocate memory
PK_MEMORY_free                — Free memory
```

### 4.2 Body Operations

```
PK_BODY_create_solid_block    — Create cuboid solid
PK_BODY_create_solid_cyl      — Create cylindrical solid
PK_BODY_create_solid_cone     — Create conical solid
PK_BODY_create_solid_sphere   — Create spherical solid
PK_BODY_create_solid_torus    — Create toroidal solid
PK_BODY_boolean_2             — Boolean operation (unite/subtract/intersect)
PK_BODY_fix_blends            — Apply blend attributes as geometry
PK_BODY_offset_2              — Offset all faces
PK_BODY_hollow                — Create hollow shell
PK_BODY_extrude               — Linear extrusion
PK_BODY_spin                  — Revolution
PK_BODY_sweep                 — Path sweep
PK_BODY_loft                  — Loft between sections
PK_BODY_section               — Section with plane/sheet
PK_BODY_imprint_body          — Imprint edges between bodies
PK_BODY_check                 — Validate body
PK_BODY_transform             — Apply transformation
PK_BODY_copy                  — Duplicate body
PK_BODY_delete                — Delete body
```

### 4.3 Face Operations

```
PK_FACE_boolean_2             — Local boolean on specific faces
PK_FACE_offset_2              — Offset specific faces
PK_FACE_change                — Generic face editing (reposition, transform, replace)
PK_FACE_move                  — Move/translate faces with automatic reblending
PK_FACE_transform             — Apply transformation to faces
PK_FACE_replace_surfs_3       — Replace surface geometry
PK_FACE_imprint_curves_2      — Imprint curves onto faces (split faces)
PK_FACE_imprint_faces         — Imprint between specific faces
PK_FACE_delete                — Delete faces and heal
PK_FACE_ask_surf              — Get face's surface
PK_FACE_ask_loops             — Get face's loops
PK_FACE_ask_orient            — Get face orientation relative to surface
```

### 4.4 Edge Operations

```
PK_EDGE_set_blend             — Set blend attributes on edge
PK_EDGE_set_blend_chamfer     — Set chamfer blend
PK_EDGE_check_blends          — Diagnostic for blend failures
PK_EDGE_set_precision         — Set tolerant edge
PK_EDGE_ask_curve             — Get edge's curve
PK_EDGE_ask_vertices          — Get edge's start/end vertices
PK_EDGE_ask_faces             — Get adjacent faces
PK_EDGE_ask_fins              — Get edge's fins
```

### 4.5 Topology Queries

```
PK_TOPOL_eval_mass            — Mass properties (volume, area, centroid, inertia)
PK_TOPOL_render_facet         — Generate triangle mesh
PK_TOPOL_render_line          — Generate wireframe lines
PK_TOPOL_find_body            — Find containing body
PK_TOPOL_ask_class            — Get entity class
PK_TOPOL_ask_attribs          — Get attributes
PK_TOPOL_ask_owner            — Get owning entity
```

### 4.6 Geometry Queries

```
PK_CURVE_eval                 — Evaluate point on curve
PK_CURVE_eval_with_derivs     — Evaluate with derivatives
PK_CURVE_ask_interval         — Get parameter domain
PK_SURF_eval                  — Evaluate point on surface
PK_SURF_eval_with_derivs      — Evaluate with partial derivatives
PK_SURF_ask_uvbox             — Get UV domain
PK_GEOM_ask_class             — Get geometry type
PK_GEOM_range_vector          — Project point onto geometry
```

### 4.7 Transmit/Receive (I/O)

```
PK_PART_transmit              — Save bodies to file (X_T or X_B)
PK_PART_receive               — Load bodies from file
PK_PARTITION_transmit         — Save partition
PK_PARTITION_receive          — Load partition
```

### 4.8 Partition & Rollback

```
PK_PARTITION_create           — Create new partition
PK_PARTITION_delete           — Delete partition
PK_PMARK_create               — Create rollback mark
PK_SESSION_set_mark           — Create session-wide mark
PK_PARTITION_return_to_mark   — Roll back partition
PK_SESSION_return_to_mark     — Roll back entire session
PK_DELTA_register_callbacks   — Register delta frustrum
```

---

## 5. Boolean Operations — Detailed Behavior

### PK_BODY_boolean_2

The primary boolean function. Behavior:

- **Target body** is modified in-place (receives the result)
- **Tool body** is consumed (deleted after operation)
- Options structure controls:
  - Operation type: unite, subtract, intersect
  - Whether to merge coincident faces
  - Whether to simplify result topology
  - Tolerant modeling behavior
  - Region control (for general bodies)

### Handling of Special Cases

1. **Coincident faces**: When target and tool share a face:
   - Unite: keep one, discard duplicate
   - Subtract: faces cancel (both removed)
   - Intersect: keep one

2. **Tangent intersections**: Surfaces touch but don't cross:
   - Creates "knife-edge" topology
   - May produce zero-volume regions

3. **Tolerant inputs**: Bodies with elevated edge/vertex tolerances:
   - Boolean proceeds with wider tolerance bands
   - Result inherits maximum tolerance from inputs
   - May increase tolerances at new intersection edges

4. **General bodies**: Non-manifold results:
   - Region control allows fine selection of which regions to keep
   - Cellular results possible (multiple adjacent solid regions)

---

## 6. Blending — Detailed Behavior

### PK_EDGE_set_blend + PK_BODY_fix_blends

Parasolid's blending is a two-phase process:
1. **Set blend attributes** on edges (radius, type, overflows)
2. **Fix blends** to actually incorporate geometry

### Cross-Section Types

| Type | Description |
|------|-------------|
| Circular | Constant radius rolling ball (most common) |
| Conic (ρ=0-0.5) | Elliptical cross-section |
| Conic (ρ=0.5) | Parabolic cross-section |
| Conic (ρ=0.5-1.0) | Hyperbolic cross-section |
| Curvature-continuous | G2 blend (tangent + curvature match at boundaries) |
| Chamfer | Linear (flat) — NOT tangent to blend walls |

### Radius Types

- **Constant**: Same radius along entire edge chain
- **Variable**: Linear variation between start and end values along edge chain

### Overflow Strategies

When a blend is too large for available geometry:

| Strategy | Behavior |
|----------|----------|
| `ov_smooth` | Blend rolls smoothly over the edge/vertex |
| `ov_cliff` | Blend creates a cliff (sharp step) at overflow |
| `ov_cliff_end` | Cliff at the end of the blend |
| `ov_notch` | Creates a notch (cut) at overflow |

### Corner Treatment

Where 3+ blended edges meet at a vertex:
- Parasolid automatically computes setback distance
- Inserts a smooth corner patch connecting adjacent blend surfaces
- Multiple corner types: sphere, smooth setback, mitered

### Face-Face Blending

Blending between non-adjacent faces (faces that don't share an edge):
- Requires specifying holdlines (constraint curves) on each face
- Conic cross-section between holdlines
- Used for complex surfacing (automotive body panels)

---

## 7. Tolerant Modeling — Complete Detail

### Session Precision

- **Default**: 1e-8 meters (linear); 1e-11 radians (angular)
- **Size box**: 1000 × 1000 × 1000 meters centered at origin
- All geometry stored in meters internally
- Session precision set once at start; affects all operations

### Local Precision (Tolerant Edges)

Set via `PK_EDGE_set_precision`:

- Assigns a tolerance value to an edge
- **Edge tolerance** = radius of a "tube" around the edge's curve
- The edge's actual geometry may deviate from the true surface-surface intersection by up to this tolerance
- **Vertex tolerance** = radius of a "sphere" around the vertex's point
- Vertex precision must be ≥ the largest precision of any incident edge

### When an Edge Becomes Tolerant

1. The 3-space curve is deleted (unless nominal geometry is enabled)
2. Two **SP-curves** are created — one attached to each of the edge's fins
3. Each SP-curve lives in the UV parameter space of its respective face
4. SP-curves must lie on the underlying surface of the corresponding face
5. The SP-curves define where the edge "really is" on each adjacent face

### SP-Curve Details

- SP-curves are 2D B-spline curves in the face's (u,v) parameter space
- They replace the 3D edge curve for tolerant edges
- The 3D position of a point on the edge = surface.evaluate(sp_curve.point_at(t))
- The gap between the two SP-curves (evaluated on their respective surfaces) is bounded by the edge tolerance

### Nominal Geometry

- Optional: a tolerant edge may retain a "nominal curve" alongside SP-curves
- The nominal curve is the 3D edge curve that was deleted when tolerance was set
- It must lie within the tolerance tube
- Purpose: faster approximate queries, visualization
- Generally geometrically simpler than the SP-curves

### Healing (Tolerance Reduction)

To remove local precision (make a tolerant edge precise again):
1. Delete the SP-curves
2. Compute the actual surface-surface intersection of the two adjacent faces
3. If the intersection curve lies within the original tolerance: success, edge is now precise
4. If not: healing fails, edge remains tolerant

---

## 8. X_T Format — Complete Specification

### File Structure

```
[Preamble: 2 lines × 80 chars, all ASCII printing characters]
**PART1; MC=...; APPL=...; FORMAT=text; DATE=...;
**PART2; SCH=SCH_MMMMMMM_SSSSS; USFLD_SIZE=n;
**PART3; [frustrum-specific data]
**END_OF_HEADER**
T<version_length> : TRANSMIT FILE created by modeller version <MMMMMMM> SCH_<schema>
[Entity data...]
[Terminator: type=1, index=0]
```

### Header Fields

- `MC=`: Machine code (platform identifier)
- `APPL=`: Application name
- `FORMAT=text` or `FORMAT=binary`
- `DATE=`: Creation date
- `SCH=SCH_MMMMMMM_SSSSS`: Schema version (modeler version + schema number)
- `USFLD_SIZE=n`: User field size

### Entity Encoding

Each entity is encoded as: `[node_type] [index] [field_values...]`

- **node_type**: Integer identifying entity class (see type code table)
- **index**: Unique integer ID within file (used as pointer target)
- **field_values**: Space-separated values per schema definition
- **Pointer fields**: Integer index values (0 = null pointer)
- **Variable-length nodes**: Include length field between type and index

### Data Types in Schema

| Code | Type | Size |
|------|------|------|
| `d` | int32 | 4 bytes |
| `n` | int16 | 2 bytes |
| `u` | unsigned byte | 1 byte |
| `l` | logical (boolean) | 1 byte |
| `c` | character | 1 byte |
| `f` | double (float64) | 8 bytes |
| `p` | pointer-index | 4 bytes |
| `v` | vector (3 doubles) | 24 bytes |
| `b` | box (6 doubles: min+max) | 48 bytes |
| `i` | interval (2 doubles) | 16 bytes |
| `h` | hvec (4 doubles: homogeneous) | 32 bytes |
| `w` | unicode string | variable |

### X_T Type Codes — Complete Table

#### Topology Entities

| Entity | Type Code |
|--------|-----------|
| TERMINATOR | 1 |
| ASSEMBLY | 2 |
| INSTANCE | 3 |
| WORLD | 12 |
| LIST | 13 |
| FACE | 14 |
| FIN (half-edge) | 15 |
| REGION | 16 |
| LOOP / EDGE | 17 |
| VERTEX | 24 |
| SHELL | 50 |
| BODY | 70 |

#### Geometry — Curves

| Entity | Type Code |
|--------|-----------|
| CIRCLE | 19 |
| POINT | 29 |
| LINE | 31 |
| ELLIPSE | 34 |
| B_CURVE | 35 |
| INTERSECTION_CURVE | 44 |
| TRIMMED_CURVE | 48 |
| PE_CURVE | 50 |
| SP_CURVE | 52 |

#### Geometry — Surfaces

| Entity | Type Code |
|--------|-----------|
| PLANE | 31 |
| CYLINDER | 55 |
| CONE | 57 |
| SPHERE | 59 |
| TORUS | 60 |
| BLENDED_EDGE | 62 |
| BLEND_BOUND | 64 |
| OFFSET_SURF | 65 |
| B_SURFACE | 67 |
| SWEPT_SURF | 73 |
| SPUN_SURF | 74 |
| PE_SURF | 76 |

#### Infrastructure / Metadata

| Entity | Type Code |
|--------|-----------|
| POINTER_LIST_BLOCK | 74 |
| TRANSFORM | 78 |
| ATT_DEF_ID | 79 |
| ATTRIB_DEF | 80 |
| ATTRIBUTE | 81 |
| INT_VALUES | 82 |
| REAL_VALUES | 83 |
| CHAR_VALUES | 84 |

#### NURBS Data Entities

| Entity | Type Code | Purpose |
|--------|-----------|---------|
| KNOT_MULT | — | Knot multiplicities |
| KNOT_SET | — | Knot values |
| BSPLINE_VERTICES | — | Control point coordinates |

### NURBS Encoding Detail

B_CURVE contains:
- Degree
- Vertex count
- Vertex dimension (3 for non-rational, 4 for rational/NURBS)
- Periodicity flags
- Reference to KNOT_MULT, KNOT_SET entities
- Reference to BSPLINE_VERTICES entity (coordinate arrays)
- Rational flag (1 = NURBS with weights as 4th coordinate)

B_SURFACE adds:
- U degree, V degree
- U vertex count, V vertex count
- U/V knot references (KNOT_MULT × 2, KNOT_SET × 2)
- Control point grid reference

### Procedural Geometry Storage

- Intersection curves and blend surfaces are stored as procedural definitions, NOT as NURBS
- Converting procedural geometry to explicit B-spline form during X_T reading is "notoriously technical and difficult"
- This is one reason STEP is preferred for interchange — STEP requires explicit geometry

### X_B (Binary) vs. X_T (Text)

- X_B: same logical structure but binary-encoded (faster read/write, smaller files)
- X_B supports embedded mesh data (convergent modeling)
- X_T does NOT store mesh data — requires separate M_T file
- Newer versions can embed schema definitions within files for forward compatibility

---

## 9. Attribute System

### Attribute Definitions (PK_ATTDEF)

Attributes are defined via `PK_ATTDEF_create` with:

- **Name**: Distinguishes the definition across transmit/receive
- **Attribute class** (1-6): Controls behavior during modeling operations:

| Class | Behavior |
|-------|----------|
| 1 | Independent of physical size/position |
| 2 | Dependent on entity size, not position |
| 3 | May vary with position/orientation |
| 4 | Transforms with owner, otherwise independent of size/shape |
| 5 | Transforms with owner unless owner changed in other ways |
| 6 | Like class 1 but supports multiple values (list) |

### Owner Classes

Which entity types can own a given attribute:
- body, face, edge, vertex, shell, loop, region, fin, group, transform, surface, curve, assembly, instance

### System Attributes

| Name | Purpose |
|------|---------|
| SDL/TYSA_COLOUR | RGB color (3 floats 0-1) |
| SDL/TYSA_NAME | Display name string |
| SDL/TYSA_REFLECTIVITY | Surface reflectivity |
| SDL/TYSA_TRANSLUCENCY | Surface translucency |

---

## 10. Frustrum (Application Interface Layer)

The host application must supply these callback functions:

### Memory Functions

| Function | Purpose |
|----------|---------|
| FMALLO | Allocate memory (map to malloc) |
| FMFREE | Free memory (map to free) |

### File I/O Functions

| Function | Purpose |
|----------|---------|
| FFOPRD | Open file for reading |
| FFOPWR | Open file for writing |
| FFCLOS | Close file |
| FFREAD | Read from file |
| FFWRIT | Write to file |

### Graphical Output Functions (Optional)

GO functions receive tessellated data from `PK_TOPOL_render_*`:
- Receive triangles, edges, points for rendering
- Application implements the actual drawing

### Delta Functions (Optional)

For partition rollback:
- Receive change records
- Enable undo/redo of individual partitions

Registration via `PK_SESSION_register` before session start.

---

## 11. Convergent Modeling (Mesh + B-rep)

### Architecture

- Introduced in Parasolid v26+; matured through v28-v35
- Facet data represented as **facet B-rep geometry** alongside classic B-rep
- A body can contain ALL facet geometry, ALL classic geometry, or ANY combination
- **Rubber faces**: Faces with mesh geometry attached; Parasolid operations treat mesh as if it were a classic surface
- **Rubber edges**: Edges with polyline geometry attached; treated as classic curves
- Only triangular facets supported (mesh must be triangulated)

### Mesh Data Structure

- Mesh is a subclass of PK_CLASS_surf (type: PK_CLASS_mesh)
- Created via `PK_MESH_create_from_facets`
- Body creation: `PK_MESH_make_bodies`
- Meshes attached to faces; topology added for operation optimization
- Split mesh into faces via `PK_FACE_imprint_curves_2`

### Operations on Mixed Bodies (v33.0+)

| Operation | Support Level |
|-----------|--------------|
| Booleans | Full (classic ↔ facet, facet ↔ facet) |
| Blending | Constant radius, notch, smooth overflow |
| Offsetting | Dependent offsets on mixed geometry |
| Sectioning | Any combination of classic and facet |
| Face operations | Transform, replace, patch, delete blend chains |
| Imprinting | Mesh with all classic surface types |
| Projection | Mixed bodies/faces |
| Sewing | Mixed face collections |
| Tapering | Faces on mixed models |

### Storage Considerations

- Binary X_B supports embedded mesh data
- Text X_T does NOT store mesh — requires separate M_T file for mesh data
- When converting X_T ↔ X_B, mesh data may be lost if only X_T is preserved

---

## 12. Sweep Operations — Detailed API

### PK_BODY_extrude

Linear extrusion of a profile along a direction vector.

Parameters:
- Profile (face, wire, or curve set)
- Direction vector
- Distance
- Optional draft angle (taper)
- Cap option (true = solid, false = sheet)

### PK_BODY_spin

Revolution of a profile about an axis.

Parameters:
- Profile
- Axis origin + direction
- Angle (partial or full revolution)
- Progression: minimum → wire → sheet → solid → general (depends on profile type)

### PK_BODY_sweep

Move body along a path, leaving lateral entities.

### PK_BODY_loft

Interpolate between multiple cross-section profiles.

Parameters:
- Ordered list of section profiles
- Optional guide curves (constrain surface shape between sections)
- End conditions:
  - No clamp (natural)
  - Planar clamp (tangent to plane at end)
  - Vector clamp (tangent to specified direction)
- Degenerate profiles supported (point sections for tapered ends)

---

## 13. Offset and Hollowing — Detailed API

### PK_BODY_offset_2

Offset all faces of a body by a constant distance.

Behavior:
- Positive distance: outward (body grows)
- Negative distance: inward (body shrinks)
- If body shrinks to nothing: error
- Adjacent faces that no longer meet: insert fillet/extension
- Offset surfaces that self-intersect: trim automatically

### PK_FACE_offset_2

Offset specific faces only:
- Selected faces move along their normals
- Adjacent faces extend/trim to maintain connectivity
- "Step faces" created along smooth boundaries where selected/unselected faces meet

### PK_BODY_hollow

Create thin-walled shell from solid:
- Specify wall thickness
- Specify faces to remove (creating openings)
- Inner shell + outer shell connected at open faces with side walls
- Per-face thickness overrides possible

---

## 14. Direct Modeling Operations

### PK_FACE_change

The generic face editing operation. In a single call, can:
- Reposition a face (translate/rotate)
- Replace underlying surface
- Transform face geometry

Parasolid automatically handles:
- Extending/trimming adjacent faces
- Reblending previously blended edges
- Inserting transition faces where needed

### PK_FACE_move

Specifically for translating faces:
- Moves selected faces by a vector
- Adjacent unselected faces automatically adjust
- Previously blended edges can be automatically re-blended

### Taper/Draft

Add draft angle to faces:
- Specify draft plane and angle
- Face surface is replaced with tapered equivalent
- Adjacent faces extended to match
- "Double-sided tapering" tapers from both sides of a neutral plane

---

## 15. Checking and Validation (PK_BODY_check)

### Check Levels

| Level | What It Checks |
|-------|---------------|
| No geometry check | Topology only (connectivity, closure) |
| Basic | Topology + basic geometry (curves on surfaces) |
| Lazy (self-intersection) | Above + self-intersection detection |
| Full | All checks including expensive geometric validation |

### What PK_BODY_check Validates

1. **Topological integrity**: All fin/edge/vertex references valid
2. **Manifold property**: Every edge shared by exactly 2 faces (for solid bodies)
3. **Shell closure**: Solid body shells are watertight
4. **Orientation consistency**: Face normals consistently point outward
5. **Euler-Poincaré formula**: V - E + F - (L - F) - 2(S - G) = 0
6. **Edge on surfaces**: Edge curve lies on both adjacent face surfaces (within tolerance)
7. **Vertex on edges**: Vertex point lies on all adjacent edge curves (within tolerance)
8. **No self-intersection**: No face intersects itself or another face in the same shell
9. **Tolerance bounds**: All edge/vertex tolerances within system limits

### Fault Information

When a check fails, Parasolid returns:
- Fault code (specific failure type)
- Fault edges/faces (which entities are problematic)
- Fault location (3D point where the problem occurs)
- Severity (warning vs. error)

### BodyShop Toolkit

Separate license for advanced repair:
- Automated healing of imported geometry
- Gap closure
- Face stitching
- Tolerance reduction
- Degeneracy removal

---

## 16. Session Precision Constants

| Constant | Value | Purpose |
|----------|-------|---------|
| Linear precision | 1e-8 meters | Minimum meaningful distance |
| Angular precision | 1e-11 radians | Minimum meaningful angle |
| Size box extent | ±500 meters | Maximum model extent from origin |
| Max tolerance | ~0.01 meters | Practical maximum edge tolerance |
| Typical SSI tolerance | ~1e-6 meters | Intersection curve fitting error |
| Typical import tolerance | ~1e-4 meters | STEP/IGES import gap |

---

## 17. Mass Properties (PK_TOPOL_eval_mass)

Computes for a solid body:
- **Volume** (cubic meters)
- **Surface area** (square meters)
- **Center of mass** (point)
- **Moments of inertia** (3×3 tensor, about origin or center of mass)
- **Principal moments** (eigenvalues of inertia tensor)
- **Principal axes** (eigenvectors of inertia tensor)

Algorithm: Surface integral using the divergence theorem. Exact for analytic surfaces,
numerical integration for NURBS. Accuracy depends on tessellation density for numerical
integration.

---

## 18. Tessellation (PK_TOPOL_render_facet)

### Options

- **Chord tolerance**: Maximum distance from facet to true surface
- **Angular tolerance**: Maximum angle between adjacent facet normals
- **Maximum facet edge length**: Upper bound on triangle size
- **Facet matching**: Require adjacent faces to share vertices at common edges (watertight)
- **Visibility classification**: Only tessellate visible faces

### Output

Via GO (Graphical Output) frustrum callbacks:
- Triangle vertices (3D points)
- Triangle normals (per-vertex for smooth shading)
- Face association (which kernel face each triangle belongs to)
- Edge polylines (for wireframe rendering)

---

## 19. Assembly Model

### Structure

```
ASSEMBLY
  └── INSTANCE (references a body or sub-assembly + transform)
       └── Body or sub-ASSEMBLY
```

### Key Properties

- Assemblies contain instances, not bodies directly
- Instances reference a body + transformation matrix
- Same body can be instanced multiple times (shared geometry, different positions)
- Instances cannot exist outside assemblies
- Assembly structure is independent of body structure

### Operations on Assemblies

- Add/remove instances
- Transform instances
- Query which instance a body belongs to
- Explode assembly into individual bodies
- Boolean instancing (efficient repeated boolean operations using assembly structure)
