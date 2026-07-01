# 13 — AI/LLM-Optimized Python API Design

## 1. Problem Statement

CadQuery is the current best Python CAD API for LLM code generation, but it has
structural problems that make AI-driven CAD unreliable:

| Pain Point | Severity | Impact on LLMs |
|---|---|---|
| Implicit workplane state | HIGH | LLM must simulate invisible state machine to predict behavior |
| Selector string DSL (`">Z"`, `"\|Y and >X"`) | HIGH | Bespoke grammar LLMs memorize imperfectly; `>Z` vs `>>Z` confusion |
| Opaque kernel errors (`StdFail_NotDone`) | HIGH | Zero self-correction signal; LLM cannot diagnose or fix |
| Type confusion (Workplane ≠ Shape ≠ Solid) | MEDIUM | `.faces()` returns Workplane, not faces; breaks LLM assumptions |
| Inconsistent parameter conventions | MEDIUM | `.circle(radius)` vs `.hole(diameter)` |
| Silent empty selections | MEDIUM | Selector matches nothing → no error, just no geometry |
| Workplane stack navigation (`.end(n)`) | MEDIUM | Wrong pop count silently changes target |

OpenSolid's current Python bindings already avoid the worst of these (no implicit
state, no string selectors, no method chaining), but the API is low-level and
procedural. An LLM generating a complex part must compose many disconnected
function calls with manual entity tracking.

## 2. Design Principles

### 2.1 Zero Hidden State

Every operation takes explicit inputs and returns explicit outputs. No operation
depends on invisible context accumulated from prior calls. An LLM can understand
any single line in isolation.

### 2.2 Error Messages That Teach

Every error message includes: (a) what went wrong, (b) the most likely cause,
(c) a concrete code fix. The LLM can parse the error and retry without
human intervention.

### 2.3 Type-Safe Selectors

Replace string-based selection (`">Z"`) with composable Python objects that have
autocomplete, type checking, and clear semantics.

### 2.4 Explicit Units and Conventions

All dimensions use a single convention (radius, not sometimes-diameter). All
angles are in degrees in the user-facing API. All coordinates are absolute unless
the function name says `relative`.

### 2.5 Composable but Not Chained

Operations return new objects rather than mutating state. Composition is via
variable assignment, not method chains. This makes intermediate results
inspectable and debuggable.

## 3. API Design

### 3.1 Part Builder

The central abstraction. Replaces CadQuery's `Workplane` with an explicit,
immutable value that represents a solid body.

```python
import opensolid as os

# Every operation returns a NEW Part — no mutation
block = os.Part.box(width=20, depth=15, height=10)  # centered at origin
cylinder = os.Part.cylinder(radius=5, height=20)
sphere = os.Part.sphere(radius=8)
cone = os.Part.cone(base_radius=5, top_radius=2, height=10)

# Primitives accept keyword-only args after the first — prevents ordering mistakes
# WRONG: os.Part.box(20, 15, 10, center=(0,0,0))  ← allowed but discouraged
# RIGHT: os.Part.box(width=20, depth=15, height=10)
```

### 3.2 Boolean Operations

```python
# Explicit, readable
result = block.subtract(cylinder)
result = block.union(cylinder)
result = block.intersect(cylinder)

# Multiple operations
result = block.subtract(hole1, hole2, hole3)  # variadic

# With positioning
tool = os.Part.cylinder(radius=3, height=20).move(x=5, y=5)
result = block.subtract(tool)
```

### 3.3 Positioning and Transforms

```python
# Explicit named parameters — never positional for transforms
part = block.move(x=10)                    # translate along X
part = block.move(x=10, y=5, z=-2)        # translate in 3D
part = block.rotate(axis="Z", angle=45)    # degrees, always
part = block.rotate(axis=(1,1,0), angle=30, center=(5,0,0))
part = block.mirror(plane="XZ")           # mirror across XZ plane
part = block.scale(factor=2)              # uniform scale
part = block.scale(x=2, y=1, z=0.5)      # non-uniform scale
```

### 3.4 Type-Safe Face/Edge Selection

Replace CadQuery's string selectors with composable Python objects.

```python
# Selector objects — composable, type-checked, self-documenting
top_face    = part.faces.top         # face with highest Z centroid
bottom_face = part.faces.bottom      # face with lowest Z centroid
front_face  = part.faces.front       # face with lowest Y centroid (toward viewer)
back_face   = part.faces.back        # face with highest Y centroid
left_face   = part.faces.left        # face with lowest X centroid
right_face  = part.faces.right       # face with highest X centroid

# By normal direction — explicit, no string parsing
z_up_faces  = part.faces.with_normal(z=1)   # faces whose normal points +Z
x_neg_faces = part.faces.with_normal(x=-1)  # faces whose normal points -X

# By index (for when you know the geometry)
face = part.faces[2]

# By proximity — finds the face closest to a point
face = part.faces.nearest(point=(5, 5, 10))

# Compound selectors — Python logic, not a DSL
vertical_edges = [e for e in part.edges if e.is_parallel_to(axis="Z")]
top_edges = part.faces.top.edges   # all edges of the top face

# Edge selectors
long_edges = [e for e in part.edges if e.length > 10]
top_edges  = part.edges.on_face(part.faces.top)
```

### 3.5 Feature Operations

```python
# Holes — always by radius, always fully specified
result = part.hole(
    face=part.faces.top,        # which face to drill into
    center=(5, 3),              # 2D position on that face
    radius=2.5,
    depth=8,                    # blind hole; omit for through-all
)

# Counterbore
result = part.counterbore_hole(
    face=part.faces.top,
    center=(5, 3),
    hole_radius=2.5,
    hole_depth=15,              # through-all if omitted
    bore_radius=5,
    bore_depth=3,
)

# Fillet — explicit edge selection
result = part.fillet(edges=part.edges.on_face(part.faces.top), radius=1.0)

# Chamfer
result = part.chamfer(edges=part.faces.top.edges, distance=0.5)

# Shell — remove faces to create hollow body
result = part.shell(
    remove_faces=[part.faces.top],
    thickness=2.0,
)

# Pattern — rectangular array
holes = os.Part.cylinder(radius=2, height=20)
tools = holes.pattern_rect(
    x_count=3, x_spacing=10,
    y_count=2, y_spacing=8,
    center=True,
)
result = part.subtract(*tools)

# Pattern — circular array
bolt_hole = os.Part.cylinder(radius=3, height=5)
bolt_holes = bolt_hole.pattern_circular(
    count=6,
    radius=25,
    center=(0, 0),
    start_angle=0,
)
result = part.subtract(*bolt_holes)
```

### 3.6 Sketch + Extrude

For profiles that aren't simple primitives. The sketch is a 2D boundary, not a
stateful workplane.

```python
# L-bracket profile
profile = os.Sketch() \
    .line_to(40, 0) \
    .line_to(40, 5) \
    .line_to(5, 5) \
    .line_to(5, 30) \
    .line_to(0, 30) \
    .close()                    # auto-closes back to start

bracket = profile.extrude(height=10)

# Arc in a sketch
profile = os.Sketch(start=(0, 0)) \
    .line_to(20, 0) \
    .arc_to(end=(20, 20), radius=10) \
    .line_to(0, 20) \
    .close()

# Extrude on a face (not on an implicit workplane)
boss_profile = os.Sketch.circle(radius=5)
result = part.extrude_on(
    face=part.faces.top,
    sketch=boss_profile,
    center=(10, 5),
    height=8,
)
```

### 3.7 Sweep and Revolve

```python
# Revolve a profile around an axis
pipe_profile = os.Sketch.circle(radius=2, center=(10, 0))
torus = pipe_profile.revolve(axis="Y", angle=360)

# Sweep along a path
cross_section = os.Sketch.circle(radius=1.5)
path = os.Path() \
    .line_to(0, 0, 20) \
    .arc_to(end=(10, 0, 30), radius=15) \
    .line_to(10, 0, 50)
swept = cross_section.sweep(path)
```

### 3.8 Error Messages with Fix Suggestions

```python
# CURRENT (CadQuery-style):
#   StdFail_NotDone: BRep_API: command not done

# PROPOSED:
#   FilletError: Fillet radius 5.0 is too large for edge E3 (length=4.2).
#   The fillet radius must be less than half the shortest adjacent edge.
#   Fix: reduce radius to at most 2.1, or select different edges.
#
#   HoleError: Hole depth 25.0 exceeds body thickness 10.0 along the
#   drill direction at center=(5, 3).
#   Fix: reduce depth to at most 10.0, or omit depth for a through-hole.
#
#   SelectionError: part.faces.with_normal(z=1) matched 0 faces.
#   The part has 6 faces with normals: +X, -X, +Y, -Y, +Z(2 faces), -Z.
#   Did you mean: part.faces.top (selects the single highest +Z face)?
#
#   BooleanError: Subtract produced an empty body — the tool completely
#   contains the target. Tool bounding box: [0,0,0]-[50,50,50],
#   target bounding box: [5,5,5]-[15,15,15].
#   Fix: swap the operands, or use a smaller tool.
```

### 3.9 Introspection for Self-Correction

```python
# Every Part carries queryable metadata
part = os.Part.box(width=20, depth=15, height=10)

part.bounds          # BoundingBox3(min=(-10,-7.5,-5), max=(10,7.5,5))
part.face_count      # 6
part.edge_count      # 12
part.vertex_count    # 8
part.volume          # 3000.0
part.center_of_mass  # Point3(0, 0, 0)

# Face introspection
face = part.faces.top
face.normal          # Vector3(0, 0, 1)
face.center          # Point3(0, 0, 5)
face.area            # 300.0
face.edge_count      # 4

# Edge introspection
edge = part.edges[0]
edge.length          # 20.0
edge.midpoint        # Point3(...)
edge.start_point     # Point3(...)
edge.end_point       # Point3(...)
edge.is_linear       # True
```

### 3.10 Export

```python
# Export — simple, explicit
part.export("bracket.step")           # format inferred from extension
part.export("bracket.stl")
part.export("bracket.gltf")
part.export("bracket.stl", binary=True)

# Mesh control
part.export("bracket.stl", mesh_resolution="fine")   # preset
part.export("bracket.stl", chord_tolerance=0.005)     # explicit
```

## 4. Example Scripts: OpenSolid vs CadQuery

### 4.1 L-Bracket with Mounting Holes

**CadQuery:**
```python
import cadquery as cq

result = (
    cq.Workplane("XY")
    .box(40, 30, 5)
    .faces(">Z")
    .workplane()
    .center(-15, 0)
    .rect(10, 30)
    .extrude(25)
    .faces(">Z")
    .workplane()
    .center(5, 8)
    .hole(4)
    .faces(">Z")
    .workplane()
    .center(5, -8)
    .hole(4)
    .faces(">X")
    .workplane()
    .center(0, 10)
    .hole(5)
)
```

Problems: implicit workplane state shifts 5 times. The `.center()` calls are
cumulative — each shifts relative to the last, not absolute. `.hole(4)` uses
diameter, not radius. An LLM frequently gets the center offsets wrong.

**OpenSolid:**
```python
import opensolid as os

# Base plate
base = os.Part.box(width=40, depth=30, height=5)

# Vertical wall
wall = os.Part.box(width=10, depth=30, height=25).move(x=-15, z=15)
bracket = base.union(wall)

# Mounting holes on top of wall
bracket = bracket.hole(face=bracket.faces.top, center=(5, 8), radius=2, depth=None)
bracket = bracket.hole(face=bracket.faces.top, center=(5, -8), radius=2, depth=None)

# Side hole
bracket = bracket.hole(face=bracket.faces.right, center=(0, 10), radius=2.5, depth=None)

bracket.export("bracket.step")
```

Every line is self-contained. No accumulated state. An LLM can modify any
single hole without understanding the full chain.

### 4.2 Spur Gear (Simplified)

**CadQuery:**
```python
import cadquery as cq
import math

n_teeth = 20
module = 2.0
pitch_r = n_teeth * module / 2
outer_r = pitch_r + module
root_r = pitch_r - 1.25 * module
tooth_angle = 360 / n_teeth

pts = []
for i in range(n_teeth):
    a0 = math.radians(i * tooth_angle)
    a1 = math.radians(i * tooth_angle + tooth_angle * 0.3)
    a2 = math.radians(i * tooth_angle + tooth_angle * 0.5)
    a3 = math.radians(i * tooth_angle + tooth_angle * 0.7)
    pts.extend([
        (root_r * math.cos(a0), root_r * math.sin(a0)),
        (outer_r * math.cos(a1), outer_r * math.sin(a1)),
        (outer_r * math.cos(a2), outer_r * math.sin(a2)),
        (root_r * math.cos(a3), root_r * math.sin(a3)),
    ])

result = (
    cq.Workplane("XY")
    .polyline(pts)
    .close()
    .extrude(10)
    .faces(">Z")
    .workplane()
    .hole(10)
)
```

**OpenSolid:**
```python
import opensolid as os
import math

n_teeth = 20
module = 2.0
pitch_r = n_teeth * module / 2
outer_r = pitch_r + module
root_r = pitch_r - 1.25 * module
tooth_angle = 360 / n_teeth

# Build gear tooth profile
sketch = os.Sketch()
for i in range(n_teeth):
    a0 = math.radians(i * tooth_angle)
    a1 = math.radians(i * tooth_angle + tooth_angle * 0.3)
    a2 = math.radians(i * tooth_angle + tooth_angle * 0.5)
    a3 = math.radians(i * tooth_angle + tooth_angle * 0.7)
    sketch.line_to(root_r * math.cos(a0), root_r * math.sin(a0))
    sketch.line_to(outer_r * math.cos(a1), outer_r * math.sin(a1))
    sketch.line_to(outer_r * math.cos(a2), outer_r * math.sin(a2))
    sketch.line_to(root_r * math.cos(a3), root_r * math.sin(a3))
sketch.close()

gear = sketch.extrude(height=10)

# Center bore
gear = gear.hole(face=gear.faces.top, center=(0, 0), radius=5, depth=None)

gear.export("gear.step")
```

Similar structure, but OpenSolid's hole uses `radius=5` (not `diameter=10`).
The sketch is an explicit object, not hidden inside a Workplane chain.

### 4.3 Pipe Fitting (Tee Joint)

**CadQuery:**
```python
import cadquery as cq

od, wall = 25, 2
main_len, branch_len = 80, 40

main_pipe = (
    cq.Workplane("XY")
    .circle(od / 2).extrude(main_len)
    .faces(">Z").workplane()
    .circle(od / 2 - wall).cutThruAll()
)

branch = (
    cq.Workplane("XZ")
    .workplane(offset=main_len / 2)
    .circle(od / 2).extrude(branch_len)
    .faces(">Y").workplane()
    .circle(od / 2 - wall).cutThruAll()
)

# Boolean union — but which object is the Workplane attached to?
result = main_pipe.union(branch.val())  # .val() needed — type confusion
```

Problems: `.cutThruAll()` behavior depends on invisible workplane orientation.
`.val()` is needed to extract the Shape from a Workplane for boolean — a common
LLM mistake.

**OpenSolid:**
```python
import opensolid as os

od, wall = 25, 2
main_len, branch_len = 80, 40

# Main tube
main_outer = os.Part.cylinder(radius=od/2, height=main_len)
main_inner = os.Part.cylinder(radius=od/2 - wall, height=main_len)
main_pipe = main_outer.subtract(main_inner)

# Branch tube — positioned explicitly
branch_outer = os.Part.cylinder(radius=od/2, height=branch_len) \
    .rotate(axis="X", angle=90) \
    .move(z=main_len/2)
branch_inner = os.Part.cylinder(radius=od/2 - wall, height=branch_len) \
    .rotate(axis="X", angle=90) \
    .move(z=main_len/2)
branch_pipe = branch_outer.subtract(branch_inner)

# Union — both are Part objects, no type confusion
tee = main_pipe.union(branch_pipe)

tee.export("tee_fitting.step")
```

### 4.4 Electronics Housing with Standoffs

**CadQuery:**
```python
import cadquery as cq

result = (
    cq.Workplane("XY")
    .box(60, 40, 25)
    .faces(">Z")
    .shell(-2)                    # negative = inward — not obvious
    .faces("<Z[1]")               # second from bottom? internal floor? unclear
    .workplane()
    .pushPoints([(20, 10), (20, -10), (-20, 10), (-20, -10)])
    .circle(3).extrude(8)         # standoffs — but height relative to what?
    .faces(">Z").workplane()      # which face is "top" now? original or standoffs?
    .pushPoints([(20, 10), (20, -10), (-20, 10), (-20, -10)])
    .hole(2.5)                    # screw holes — diameter, not radius
)
```

Problems: `.shell(-2)` sign convention is non-obvious. After shelling, `<Z[1]`
selector is ambiguous. After extruding standoffs, `>Z` might select a standoff
top instead of the housing top.

**OpenSolid:**
```python
import opensolid as os

# Outer shell
housing = os.Part.box(width=60, depth=40, height=25)
housing = housing.shell(remove_faces=[housing.faces.top], thickness=2)

# Standoff positions
standoff_positions = [(20, 10), (20, -10), (-20, 10), (-20, -10)]

# Add standoffs on interior floor
floor = housing.faces.bottom  # unambiguous — lowest Z face
for pos in standoff_positions:
    standoff = os.Part.cylinder(radius=3, height=8).move(x=pos[0], y=pos[1], z=-10.5)
    housing = housing.union(standoff)

# Screw holes through standoff tops
for pos in standoff_positions:
    housing = housing.hole(
        face=housing.faces.nearest(point=(pos[0], pos[1], -2.5)),
        center=(0, 0),   # centered on the face
        radius=1.25,
        depth=8,
    )

housing.export("housing.step")
```

### 4.5 Swept Profile: C-Channel Beam

**CadQuery:**
```python
import cadquery as cq

result = (
    cq.Workplane("XZ")
    .moveTo(0, 0)
    .lineTo(0, 30)
    .lineTo(15, 30)
    .lineTo(15, 27)
    .lineTo(3, 27)
    .lineTo(3, 3)
    .lineTo(15, 3)
    .lineTo(15, 0)
    .close()
    .extrude(200)
)
```

**OpenSolid:**
```python
import opensolid as os

# C-channel cross section
profile = os.Sketch() \
    .line_to(0, 30) \
    .line_to(15, 30) \
    .line_to(15, 27) \
    .line_to(3, 27) \
    .line_to(3, 3) \
    .line_to(15, 3) \
    .line_to(15, 0) \
    .close()

beam = profile.extrude(height=200)

beam.export("c_channel.step")
```

For simple extrusions the two APIs are comparable. The difference emerges when
you need to add features to the result — CadQuery requires navigating workplane
state, while OpenSolid uses explicit face selection.

## 5. Selector Comparison

| CadQuery | OpenSolid | Notes |
|---|---|---|
| `.faces(">Z")` | `.faces.top` | Property, not string parse |
| `.faces("<Z")` | `.faces.bottom` | |
| `.faces(">Z[-2]")` | `.faces.sorted_by_z[-2]` | Explicit sort, Python indexing |
| `.faces(">>Z")` | `.faces.sorted_by_centroid_z` | Different from normal-based |
| `.faces("\|Z")` | `.faces.with_normal(z=1)` or `.faces.with_normal(z=-1)` | Explicit direction |
| `.edges("\|Z")` | `[e for e in part.edges if e.is_parallel_to("Z")]` | Python, not DSL |
| `.edges("\|Z and >Y")` | `part.faces.back.edges.parallel_to("Z")` | Compose via Python |
| `.faces("%Plane")` | `[f for f in part.faces if f.is_planar]` | Property check |

## 6. Error Recovery Protocol

When an LLM encounters an error, the proposed error format enables a
parse-retry-fix loop:

```
1. LLM generates code
2. Code raises: "FilletError: radius 5.0 too large for edge E3 (length 4.2).
   Fix: reduce radius to at most 2.1"
3. LLM parses structured error, adjusts radius to 2.0
4. Retries successfully
```

Each error class provides:
- `error.kind` — machine-readable enum (`"fillet_too_large"`, `"empty_selection"`, etc.)
- `error.message` — human-readable explanation
- `error.suggestion` — concrete fix as a code snippet
- `error.context` — dict of relevant values (radius, edge_length, face_count, etc.)

## 7. Comparison Summary

| Aspect | CadQuery | OpenSolid (proposed) |
|---|---|---|
| State model | Implicit workplane stack | Explicit immutable values |
| Selectors | String DSL (`">Z"`, `"\|Y"`) | Python properties + methods |
| Error messages | `StdFail_NotDone` | Structured with fix suggestions |
| Radius/diameter | Mixed (`.hole` = diameter) | Always radius |
| Angles | Degrees (usually) | Always degrees |
| Method chaining | Required (fluent API) | Optional (variable assignment) |
| Type safety | Workplane wraps everything | Part, Face, Edge are distinct types |
| Self-correction | Impossible from errors | Designed for parse-retry loops |
| Introspection | `.val()`, `.vals()` awkward | `.face_count`, `.volume`, `.bounds` |

## 8. Implementation Notes (for future work)

The builder API is a **layer on top of** the existing low-level bindings, not a
replacement. It wraps `TopologyStore` operations behind `Part` objects that
carry their `EntityId<Body>` internally.

Key implementation decisions:
- `Part` is a thin wrapper around `EntityId<Body>` + `SharedStore`
- Face/Edge selectors are lazy — they query the store when accessed
- Error messages are generated by wrapping kernel operations with geometry-aware
  context (bounding boxes, edge lengths, face normals)
- `Sketch` uses the existing `opensolid_sketch` solver
- `export()` dispatches to `opensolid_export` based on file extension

Estimated implementation: ~2000 lines of Python-side wrapper code in `lib.rs`,
plus ~500 lines of error formatting helpers.
