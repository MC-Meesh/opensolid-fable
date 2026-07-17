//! STEP AP203 writer: kernel B-Rep → Part 21 (ISO-10303-21).
//!
//! [`write_step`] serializes solid bodies from a [`TopologyStore`] /
//! [`GeometryStore`] pair into a `CONFIG_CONTROL_DESIGN` (AP203) physical
//! file — the exact reverse of [`read_step`](super::read::read_step)'s
//! mapping:
//!
//! - **Geometry**: [`Surface3`] → `plane`, `cylindrical_surface`,
//!   `conical_surface`, `spherical_surface`, `toroidal_surface`;
//!   [`Curve3`] → `line`, `circle`, `ellipse` (each positioned by
//!   `axis2_placement_3d` built from `cartesian_point` / `direction`);
//!   [`Curve3::Polyline`] (marched SSI edges) → degree-1
//!   `b_spline_curve_with_knots` with knots at the vertex indices.
//! - **Topology**: `Vertex` / `Edge` / `Loop` / `Face` / `Shell` / `Body` →
//!   `vertex_point`, `edge_curve`, `oriented_edge`, `edge_loop`,
//!   `face_outer_bound` / `face_bound`, `advanced_face`, `closed_shell`,
//!   `manifold_solid_brep`. Shared vertices, edges, curves and surfaces are
//!   emitted once and referenced, so seams and mated fins survive the
//!   round trip.
//!
//! Every `edge_curve` is written with `same_sense = .T.` and its curve in
//! the edge's own orientation; STEP trims edges by their vertices, so the
//! reader re-derives the parameter range from the emitted vertex points
//! (closed edges keep their full period, arcs their sweep).
//!
//! Around the solids the writer emits the minimal AP203 product skeleton —
//! `APPLICATION_CONTEXT` through `PRODUCT_DEFINITION_SHAPE`, an
//! `ADVANCED_BREP_SHAPE_REPRESENTATION` collecting every solid, and a
//! `SHAPE_DEFINITION_REPRESENTATION` tying the two together — plus a
//! geometric representation context declaring SI units (configurable
//! length unit, radians, steradians) and a `1.0E-7` length uncertainty.
//! Units live in the DATA section context as ISO 10303 requires; the
//! HEADER carries the `CONFIG_CONTROL_DESIGN` schema declaration.
//! Coordinates are written verbatim in the declared unit.
//!
//! Reals are formatted with Rust's shortest round-trip representation,
//! adjusted to Part 21 grammar (mandatory decimal point, upper-case `E`),
//! so every coordinate re-reads to the identical `f64`.
//!
//! # What is supported
//!
//! Solid bodies ([`BodyType::Solid`]) with exactly one closed shell, whose
//! faces all carry analytic surfaces and whose edges all carry analytic
//! curves. Anything else — sheet/wire bodies, voids (multiple shells),
//! vertex loops, missing geometry — fails with [`StepWriteError`] rather
//! than emitting an unreadable file. NURBS geometry cannot occur: the
//! geometry store's [`Curve3`] / [`Surface3`] enums have no NURBS variants
//! yet (see the [reader docs](super::read) for the import-side mirror of
//! this limitation).
//!
//! # External tool compatibility
//!
//! The emitted structure (AP203 product skeleton, SI unit context with
//! uncertainty, `ADVANCED_BREP_SHAPE_REPRESENTATION`) is the shape that
//! OpenCascade's `STEPControl_Reader` — and therefore FreeCAD — expects
//! from a minimal AP203 exporter, and mirrors files those tools emit
//! themselves. Tests here verify the round trip through this crate's own
//! reader only (no network, no external binaries); opening the output in
//! FreeCAD ≥ 0.20 or any OCC-based viewer is expected to work for every
//! body this writer accepts.
//!
//! # Example
//!
//! ```
//! use opensolid_kernel::brep::primitives::block;
//! use opensolid_kernel::brep::{GeometryStore, TopologyStore};
//! use opensolid_kernel::io::step::read::{SolidOutcome, StepReadOptions, read_step};
//! use opensolid_kernel::io::step::write::{StepWriteOptions, write_step};
//!
//! let mut store = TopologyStore::new();
//! let mut geo = GeometryStore::new();
//! let body = block(&mut store, &mut geo, 2.0, 3.0, 4.0).unwrap();
//!
//! let text = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap();
//!
//! // The acceptance gate: our own reader re-imports the emitted file exactly.
//! let mut store2 = TopologyStore::new();
//! let mut geo2 = GeometryStore::new();
//! let import = read_step(&text, &mut store2, &mut geo2, &StepReadOptions::default()).unwrap();
//! match &import.solids[0].outcome {
//!     SolidOutcome::BRep(body2) => assert!(store2.check(*body2).is_empty()),
//!     other => panic!("expected exact B-Rep re-import, got {other:?}"),
//! }
//! ```

use std::collections::HashMap;
use std::fmt::Write as _;

use opensolid_brep::curve::plane_basis;
use opensolid_brep::{
    Body, BodyType, Curve3, Edge, FaceSense, FinSense, GeometryStore, Loop, Surface3,
    TopologyStore, Vertex,
};
use opensolid_core::{EntityId, Point3, Vector3};

/// Length unit declared in the emitted representation context.
/// Coordinates are written verbatim, interpreted in this unit.
///
/// The metric units emit a direct `SI_UNIT`; [`LengthUnit::Inch`] emits a
/// `CONVERSION_BASED_UNIT` defined as `25.4` millimetres, the STEP-standard
/// way to declare a non-SI length so importers resolve it to the right scale.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum LengthUnit {
    /// `SI_UNIT(.MILLI., .METRE.)` — the conventional CAD exchange unit.
    #[default]
    Millimetre,
    /// `SI_UNIT(.CENTI., .METRE.)`.
    Centimetre,
    /// `SI_UNIT($, .METRE.)`.
    Metre,
    /// `CONVERSION_BASED_UNIT('INCH', …)` — 25.4 mm.
    Inch,
}

/// Options for [`write_step`].
#[derive(Debug, Clone)]
pub struct StepWriteOptions {
    /// Product id/name for the AP203 product skeleton and `FILE_NAME`.
    pub product_name: String,
    /// Length unit declared in the representation context.
    pub length_unit: LengthUnit,
}

impl Default for StepWriteOptions {
    fn default() -> Self {
        Self {
            product_name: "OpenSolid part".to_string(),
            length_unit: LengthUnit::default(),
        }
    }
}

/// Why a body could not be serialized. No partial file is ever returned.
#[derive(Debug, Clone, PartialEq, Eq, thiserror::Error)]
pub enum StepWriteError {
    /// A body id does not resolve in the given [`TopologyStore`].
    #[error("stale body id: not present in this TopologyStore")]
    StaleBody,
    /// Valid kernel data the AP203 writer cannot express (yet).
    #[error("unsupported for STEP export: {0}")]
    Unsupported(String),
    /// The body's stores are inconsistent (dangling reference, missing
    /// geometry, malformed parameter range).
    #[error("invalid body: {0}")]
    Invalid(String),
}

type WriteResult<T> = Result<T, StepWriteError>;

fn unsupported(what: impl Into<String>) -> StepWriteError {
    StepWriteError::Unsupported(what.into())
}

fn invalid(what: impl Into<String>) -> StepWriteError {
    StepWriteError::Invalid(what.into())
}

/// Serialize `bodies` to a complete STEP AP203 Part 21 file.
///
/// Every body must be a [`BodyType::Solid`] with exactly one closed shell
/// and fully attached analytic geometry; each becomes one
/// `MANIFOLD_SOLID_BREP`, all collected under a single product and shape
/// representation. An empty `bodies` slice yields a valid file with no
/// solids.
///
/// # Errors
/// [`StepWriteError`] if any body is stale, non-solid, multi-shell, or
/// references missing/unsupported geometry. The stores are never mutated.
pub fn write_step(
    store: &TopologyStore,
    geo: &GeometryStore,
    bodies: &[EntityId<Body>],
    options: &StepWriteOptions,
) -> WriteResult<String> {
    let mut emitter = Emitter {
        store,
        geo,
        data: String::new(),
        next_id: 1,
        vertices: HashMap::new(),
        edges: HashMap::new(),
        curves: HashMap::new(),
        surfaces: HashMap::new(),
    };
    let name = string_literal(&options.product_name);

    // AP203 product skeleton.
    let app = emitter.emit(
        "APPLICATION_CONTEXT('configuration controlled 3d designs of mechanical parts and assemblies')",
    );
    emitter.emit(format!(
        "APPLICATION_PROTOCOL_DEFINITION('international standard','config_control_design',1994,#{app})"
    ));
    let mechanical = emitter.emit(format!("MECHANICAL_CONTEXT('',#{app},'mechanical')"));
    let product = emitter.emit(format!("PRODUCT({name},{name},'',(#{mechanical}))"));
    emitter.emit(format!(
        "PRODUCT_RELATED_PRODUCT_CATEGORY('part','',(#{product}))"
    ));
    let design = emitter.emit(format!("DESIGN_CONTEXT('',#{app},'design')"));
    let formation = emitter.emit(format!(
        "PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE('','',#{product},.NOT_KNOWN.)"
    ));
    let definition = emitter.emit(format!(
        "PRODUCT_DEFINITION('design','',#{formation},#{design})"
    ));
    let shape = emitter.emit(format!("PRODUCT_DEFINITION_SHAPE('','',#{definition})"));

    // Units and the geometric representation context. Metric units emit a
    // direct SI_UNIT; inch emits a CONVERSION_BASED_UNIT defined as 25.4 mm
    // so importers resolve it to the correct scale.
    let length_unit = match options.length_unit {
        LengthUnit::Millimetre => {
            emitter.emit("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )")
        }
        LengthUnit::Centimetre => {
            emitter.emit("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.) )")
        }
        LengthUnit::Metre => emitter.emit("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) )"),
        LengthUnit::Inch => {
            // Base millimetre unit the conversion is expressed against, plus
            // the length dimensional signature (exponent 1 on length).
            let mm = emitter.emit("( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) )");
            let dims = emitter.emit("DIMENSIONAL_EXPONENTS(1.0,0.0,0.0,0.0,0.0,0.0,0.0)");
            let measure = emitter.emit(format!(
                "LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(25.4),#{mm})"
            ));
            emitter.emit(format!(
                "( CONVERSION_BASED_UNIT('INCH',#{measure}) LENGTH_UNIT() NAMED_UNIT(#{dims}) )"
            ))
        }
    };
    let angle_unit = emitter.emit("( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) )");
    let solid_angle_unit =
        emitter.emit("( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() )");
    let uncertainty = emitter.emit(format!(
        "UNCERTAINTY_MEASURE_WITH_UNIT(LENGTH_MEASURE(1.0E-7),#{length_unit},\
         'distance_accuracy_value','confusion accuracy')"
    ));
    let context = emitter.emit(format!(
        "( GEOMETRIC_REPRESENTATION_CONTEXT(3) \
         GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT((#{uncertainty})) \
         GLOBAL_UNIT_ASSIGNED_CONTEXT((#{length_unit},#{angle_unit},#{solid_angle_unit})) \
         REPRESENTATION_CONTEXT('Context #1','3D Context') )"
    ));

    // The solids themselves.
    let mut solid_ids = Vec::with_capacity(bodies.len());
    for &body in bodies {
        solid_ids.push(emitter.emit_body(body, &options.product_name)?);
    }

    // One shape representation collecting every solid, anchored at the
    // world placement, tied back to the product definition.
    let world = emitter.emit_axis2(Point3::origin(), Vector3::z(), Vector3::x());
    let mut items = format!("#{world}");
    for id in &solid_ids {
        write!(items, ",#{id}").expect("write to String");
    }
    let representation = emitter.emit(format!(
        "ADVANCED_BREP_SHAPE_REPRESENTATION('',({items}),#{context})"
    ));
    emitter.emit(format!(
        "SHAPE_DEFINITION_REPRESENTATION(#{shape},#{representation})"
    ));

    let mut out = String::with_capacity(emitter.data.len() + 512);
    out.push_str("ISO-10303-21;\nHEADER;\n");
    out.push_str("FILE_DESCRIPTION(('OpenSolid B-Rep export'),'2;1');\n");
    let _ = writeln!(
        out,
        "FILE_NAME({name},'',(''),(''),'OpenSolid','OpenSolid','');"
    );
    out.push_str("FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\nDATA;\n");
    out.push_str(&emitter.data);
    out.push_str("ENDSEC;\nEND-ISO-10303-21;\n");
    Ok(out)
}

/// Format a Part 21 string literal: quoted, with `'` doubled. Other Part 21
/// control directives are not generated (names are written verbatim).
fn string_literal(s: &str) -> String {
    let mut out = String::with_capacity(s.len() + 2);
    out.push('\'');
    for c in s.chars() {
        if c == '\'' {
            out.push_str("''");
        } else {
            out.push(c);
        }
    }
    out.push('\'');
    out
}

/// Format a real to Part 21 grammar: shortest representation that parses
/// back to the identical `f64`, with the mandatory decimal point and an
/// upper-case exponent marker.
fn fmt_real(x: f64) -> String {
    debug_assert!(x.is_finite(), "STEP reals must be finite, got {x}");
    // `{:?}` is Rust's shortest round-trip form, e.g. "1.0", "-2.5e-7",
    // "6.283185307179586", "1e300".
    let s = format!("{x:?}");
    match s.split_once(['e', 'E']) {
        Some((mantissa, exponent)) => {
            if mantissa.contains('.') {
                format!("{mantissa}E{exponent}")
            } else {
                format!("{mantissa}.0E{exponent}")
            }
        }
        None => s, // `{:?}` always prints a '.' when there is no exponent
    }
}

fn fmt_triple(x: f64, y: f64, z: f64) -> String {
    format!("({},{},{})", fmt_real(x), fmt_real(y), fmt_real(z))
}

fn step_bool(b: bool) -> &'static str {
    if b { ".T." } else { ".F." }
}

struct Emitter<'a> {
    store: &'a TopologyStore,
    geo: &'a GeometryStore,
    /// Accumulated DATA-section records.
    data: String,
    next_id: u64,
    /// Kernel entity → emitted instance name, so shared topology and
    /// geometry serialize once.
    vertices: HashMap<EntityId<Vertex>, u64>,
    edges: HashMap<EntityId<Edge>, u64>,
    curves: HashMap<EntityId<Curve3>, u64>,
    surfaces: HashMap<EntityId<Surface3>, u64>,
}

impl Emitter<'_> {
    /// Append `#id = record;` and return the assigned instance name.
    fn emit(&mut self, record: impl AsRef<str>) -> u64 {
        let id = self.next_id;
        self.next_id += 1;
        let _ = writeln!(self.data, "#{id} = {};", record.as_ref());
        id
    }

    fn emit_point(&mut self, p: Point3) -> u64 {
        self.emit(format!("CARTESIAN_POINT('',{})", fmt_triple(p.x, p.y, p.z)))
    }

    fn emit_direction(&mut self, v: Vector3) -> u64 {
        self.emit(format!("DIRECTION('',{})", fmt_triple(v.x, v.y, v.z)))
    }

    /// `AXIS2_PLACEMENT_3D` at `location` with `axis` (z) and `ref_dir` (x).
    fn emit_axis2(&mut self, location: Point3, axis: Vector3, ref_dir: Vector3) -> u64 {
        let point = self.emit_point(location);
        let axis = self.emit_direction(axis);
        let ref_dir = self.emit_direction(ref_dir);
        self.emit(format!(
            "AXIS2_PLACEMENT_3D('',#{point},#{axis},#{ref_dir})"
        ))
    }

    fn emit_body(&mut self, body: EntityId<Body>, name: &str) -> WriteResult<u64> {
        let body_rec = self
            .store
            .bodies
            .get(body)
            .ok_or(StepWriteError::StaleBody)?;
        if body_rec.body_type != BodyType::Solid {
            return Err(unsupported(format!(
                "{:?} bodies (only Solid maps to MANIFOLD_SOLID_BREP)",
                body_rec.body_type
            )));
        }
        let &[shell] = body_rec.shells.as_slice() else {
            return Err(unsupported(format!(
                "bodies with {} shells (voids need BREP_WITH_VOIDS, not emitted yet)",
                body_rec.shells.len()
            )));
        };
        let shell_rec = self
            .store
            .shells
            .get(shell)
            .ok_or_else(|| invalid("body references a stale shell"))?;
        if !shell_rec.is_closed {
            return Err(unsupported("open shells (CLOSED_SHELL requires closure)"));
        }
        if shell_rec.faces.is_empty() {
            return Err(invalid("shell has no faces"));
        }

        let mut face_ids = Vec::with_capacity(shell_rec.faces.len());
        for &face in &shell_rec.faces {
            face_ids.push(self.emit_face(face)?);
        }
        let faces = ref_list(&face_ids);
        let shell_id = self.emit(format!("CLOSED_SHELL('',{faces})"));
        Ok(self.emit(format!(
            "MANIFOLD_SOLID_BREP({},#{shell_id})",
            string_literal(name)
        )))
    }

    fn emit_face(&mut self, face: EntityId<opensolid_brep::Face>) -> WriteResult<u64> {
        let face_rec = self
            .store
            .faces
            .get(face)
            .ok_or_else(|| invalid("shell references a stale face"))?;
        let surface_id = face_rec
            .surface
            .ok_or_else(|| invalid("face has no surface attached"))?;
        let surface = self.emit_surface(surface_id)?;

        let outer = face_rec
            .outer_loop
            .ok_or_else(|| invalid("face has no outer loop"))?;
        let mut bounds = Vec::with_capacity(1 + face_rec.inner_loops.len());
        let outer_loop = self.emit_loop(outer)?;
        bounds.push(self.emit(format!("FACE_OUTER_BOUND('',#{outer_loop},.T.)")));
        for &inner in &face_rec.inner_loops {
            let inner_loop = self.emit_loop(inner)?;
            bounds.push(self.emit(format!("FACE_BOUND('',#{inner_loop},.T.)")));
        }

        let same_sense = step_bool(face_rec.sense == FaceSense::Positive);
        Ok(self.emit(format!(
            "ADVANCED_FACE('',{},#{surface},{same_sense})",
            ref_list(&bounds)
        )))
    }

    fn emit_loop(&mut self, loop_id: EntityId<Loop>) -> WriteResult<u64> {
        let loop_rec = self
            .store
            .loops
            .get(loop_id)
            .ok_or_else(|| invalid("face references a stale loop"))?;
        if loop_rec.fins.is_empty() {
            return Err(unsupported(
                "degenerate vertex loops (VERTEX_LOOP is not readable back yet)",
            ));
        }
        let mut oriented = Vec::with_capacity(loop_rec.fins.len());
        for &fin in &loop_rec.fins {
            let fin_rec = self
                .store
                .fins
                .get(fin)
                .ok_or_else(|| invalid("loop references a stale fin"))?;
            let edge = self.emit_edge(fin_rec.edge)?;
            let orientation = step_bool(fin_rec.sense == FinSense::Forward);
            oriented.push(self.emit(format!("ORIENTED_EDGE('',*,*,#{edge},{orientation})")));
        }
        Ok(self.emit(format!("EDGE_LOOP('',{})", ref_list(&oriented))))
    }

    fn emit_edge(&mut self, edge: EntityId<Edge>) -> WriteResult<u64> {
        if let Some(&id) = self.edges.get(&edge) {
            return Ok(id);
        }
        let edge_rec = self
            .store
            .edges
            .get(edge)
            .ok_or_else(|| invalid("fin references a stale edge"))?;
        let curve_id = edge_rec
            .curve
            .ok_or_else(|| invalid("edge has no curve attached"))?;
        // NaN-safe: anything but a strictly increasing range is rejected.
        if edge_rec.t_start.partial_cmp(&edge_rec.t_end) != Some(std::cmp::Ordering::Less) {
            return Err(invalid(format!(
                "edge parameter range [{}, {}] is not increasing",
                edge_rec.t_start, edge_rec.t_end
            )));
        }
        let curve = self.emit_curve(curve_id)?;
        let start = self.emit_vertex(edge_rec.start_vertex)?;
        let end = self.emit_vertex(edge_rec.end_vertex)?;
        // The curve is written in the edge's own orientation, so the edge
        // always agrees with its geometry; the reader re-derives the
        // parameter range from the vertex points.
        let id = self.emit(format!("EDGE_CURVE('',#{start},#{end},#{curve},.T.)"));
        self.edges.insert(edge, id);
        Ok(id)
    }

    fn emit_vertex(&mut self, vertex: EntityId<Vertex>) -> WriteResult<u64> {
        if let Some(&id) = self.vertices.get(&vertex) {
            return Ok(id);
        }
        let vertex_rec = self
            .store
            .vertices
            .get(vertex)
            .ok_or_else(|| invalid("edge references a stale vertex"))?;
        let point = self.emit_point(vertex_rec.point);
        let id = self.emit(format!("VERTEX_POINT('',#{point})"));
        self.vertices.insert(vertex, id);
        Ok(id)
    }

    fn emit_curve(&mut self, curve: EntityId<Curve3>) -> WriteResult<u64> {
        if let Some(&id) = self.curves.get(&curve) {
            return Ok(id);
        }
        let curve_rec = self
            .geo
            .curve(curve)
            .ok_or_else(|| invalid("edge references a stale curve"))?;
        let id = match *curve_rec {
            Curve3::Line { origin, dir } => {
                let point = self.emit_point(origin);
                let direction = self.emit_direction(dir);
                // Unit magnitude keeps the line parameterized by arc
                // length, matching the kernel's convention.
                let vector = self.emit(format!("VECTOR('',#{direction},1.0)"));
                self.emit(format!("LINE('',#{point},#{vector})"))
            }
            Curve3::Circle {
                center,
                axis,
                radius,
            } => {
                // The kernel's angular reference (t = 0) is plane_basis's
                // u axis; emit it as ref_direction so external tools see
                // the same parameterization the kernel uses.
                let placement = self.emit_axis2(center, axis, plane_basis(&axis).0);
                self.emit(format!("CIRCLE('',#{placement},{})", fmt_real(radius)))
            }
            Curve3::Ellipse {
                center,
                axis,
                major_dir,
                major_radius,
                minor_radius,
            } => {
                // semi_axis_1 lies along ref_direction; emitting the major
                // direction first keeps semi_1 >= semi_2, which the reader
                // maps back without swapping.
                let placement = self.emit_axis2(center, axis, major_dir);
                self.emit(format!(
                    "ELLIPSE('',#{placement},{},{})",
                    fmt_real(major_radius),
                    fmt_real(minor_radius)
                ))
            }
            Curve3::Polyline { ref points, .. } => {
                // Degree-1 B-spline through the vertices with knots at the
                // integers: identical parameterization to the kernel's
                // vertex-index convention (marched SSI polylines).
                let cps: Vec<u64> = points.iter().map(|p| self.emit_point(*p)).collect();
                let n = points.len();
                let mut mults = vec![1i64; n];
                mults[0] = 2;
                mults[n - 1] = 2;
                let knots: Vec<String> = (0..n).map(|k| fmt_real(k as f64)).collect();
                self.emit(format!(
                    "B_SPLINE_CURVE_WITH_KNOTS('',1,{},.POLYLINE_FORM.,.F.,.F.,({}),({}),.UNSPECIFIED.)",
                    ref_list(&cps),
                    mults
                        .iter()
                        .map(|m| m.to_string())
                        .collect::<Vec<_>>()
                        .join(","),
                    knots.join(",")
                ))
            }
        };
        self.curves.insert(curve, id);
        Ok(id)
    }

    fn emit_surface(&mut self, surface: EntityId<Surface3>) -> WriteResult<u64> {
        if let Some(&id) = self.surfaces.get(&surface) {
            return Ok(id);
        }
        let surface_rec = self
            .geo
            .surface(surface)
            .ok_or_else(|| invalid("face references a stale surface"))?;
        let id = match *surface_rec {
            Surface3::Plane { origin, normal } => {
                let placement = self.emit_axis2(origin, normal, plane_basis(&normal).0);
                self.emit(format!("PLANE('',#{placement})"))
            }
            Surface3::Cylinder {
                origin,
                axis,
                radius,
            } => {
                let placement = self.emit_axis2(origin, axis, plane_basis(&axis).0);
                self.emit(format!(
                    "CYLINDRICAL_SURFACE('',#{placement},{})",
                    fmt_real(radius)
                ))
            }
            Surface3::Cone {
                origin,
                axis,
                half_angle,
                radius,
            } => {
                let placement = self.emit_axis2(origin, axis, plane_basis(&axis).0);
                self.emit(format!(
                    "CONICAL_SURFACE('',#{placement},{},{})",
                    fmt_real(radius),
                    fmt_real(half_angle)
                ))
            }
            Surface3::Sphere {
                center,
                axis,
                radius,
            } => {
                let placement = self.emit_axis2(center, axis, plane_basis(&axis).0);
                self.emit(format!(
                    "SPHERICAL_SURFACE('',#{placement},{})",
                    fmt_real(radius)
                ))
            }
            Surface3::Torus {
                center,
                axis,
                major_radius,
                minor_radius,
            } => {
                let placement = self.emit_axis2(center, axis, plane_basis(&axis).0);
                self.emit(format!(
                    "TOROIDAL_SURFACE('',#{placement},{},{})",
                    fmt_real(major_radius),
                    fmt_real(minor_radius)
                ))
            }
            // AP203 spells this B_SPLINE_SURFACE_WITH_KNOTS, which needs
            // the control grid, both knot vectors with multiplicities, and
            // the rational weight grid emitted; that is the of-37i STEP
            // round-trip phase, not this one.
            Surface3::Nurbs(_) => {
                return Err(StepWriteError::Unsupported(
                    "NURBS surface (B_SPLINE_SURFACE_WITH_KNOTS emission is a later of-37i phase)"
                        .into(),
                ));
            }
        };
        self.surfaces.insert(surface, id);
        Ok(id)
    }
}

/// `(#a,#b,…)` over instance names.
fn ref_list(ids: &[u64]) -> String {
    let mut out = String::from("(");
    for (i, id) in ids.iter().enumerate() {
        if i > 0 {
            out.push(',');
        }
        let _ = write!(out, "#{id}");
    }
    out.push(')');
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::TAU;

    use opensolid_brep::primitives::{block, cylinder, sphere, torus};
    use opensolid_brep::{
        FinSense, LoopType, SYSTEM_RESOLUTION, ShellOrientation, TessellationOptions,
        tessellate_body,
    };

    use crate::io::step::read::{SolidOutcome, StepReadOptions, read_step};
    use crate::massprops::mass_properties;

    fn volume(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) -> f64 {
        let mesh = tessellate_body(store, geo, body, &TessellationOptions::default())
            .expect("body must tessellate");
        mass_properties(&mesh)
            .expect("tessellation must be a closed manifold")
            .volume
    }

    /// Re-import an emitted file, requiring every solid to come back as an
    /// exact B-Rep with no error diagnostics.
    fn reimport(text: &str) -> (TopologyStore, GeometryStore, Vec<EntityId<Body>>) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let import = read_step(text, &mut store, &mut geo, &StepReadOptions::default())
            .expect("emitted file must be syntactically valid");
        assert!(
            !import.has_errors(),
            "reader reported errors: {:?}",
            import.diagnostics
        );
        let bodies = import
            .solids
            .iter()
            .map(|solid| match &solid.outcome {
                SolidOutcome::BRep(body) => *body,
                other => panic!(
                    "expected exact B-Rep re-import, got {other:?}; diagnostics: {:?}",
                    import.diagnostics
                ),
            })
            .collect();
        (store, geo, bodies)
    }

    fn assert_counts_equal(
        store: &TopologyStore,
        body: EntityId<Body>,
        store2: &TopologyStore,
        body2: EntityId<Body>,
    ) {
        let a = store.euler_counts(body);
        let b = store2.euler_counts(body2);
        assert_eq!(a.vertices, b.vertices, "vertex count");
        assert_eq!(a.edges, b.edges, "edge count");
        assert_eq!(a.faces, b.faces, "face count");
        assert_eq!(a.loops, b.loops, "loop count");
        assert_eq!(a.rings, b.rings, "ring count");
        assert_eq!(a.shells, b.shells, "shell count");
        assert_eq!(a.genus, b.genus, "genus");
    }

    /// The acceptance gate: write, re-read through our own reader, and
    /// require identical topology counts and volume within 1e-9 relative.
    fn assert_round_trip(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) {
        assert!(
            store.check(body).is_empty(),
            "original body must pass check: {:?}",
            store.check(body)
        );
        let text = write_step(store, geo, &[body], &StepWriteOptions::default())
            .expect("body must serialize");
        let (store2, geo2, bodies) = reimport(&text);
        assert_eq!(bodies.len(), 1, "one MANIFOLD_SOLID_BREP expected");
        let body2 = bodies[0];
        assert!(
            store2.check(body2).is_empty(),
            "re-imported body must pass check: {:?}",
            store2.check(body2)
        );
        assert_counts_equal(store, body, &store2, body2);

        let v1 = volume(store, geo, body);
        let v2 = volume(&store2, &geo2, body2);
        assert!(v1 > 0.0, "original volume must be positive, got {v1}");
        let drift = (v1 - v2).abs() / v1.max(1.0);
        assert!(
            drift <= 1e-9,
            "volume drift {drift:e} exceeds 1e-9 (original {v1}, re-imported {v2})"
        );
    }

    // ------------------------------------------------------------------
    // Round trips
    // ------------------------------------------------------------------

    #[test]
    fn round_trip_block() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 2.0, 3.0, 4.0).expect("block");
        assert_round_trip(&store, &geo, body);
    }

    #[test]
    fn round_trip_cylinder() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = cylinder(&mut store, &mut geo, 1.5, 4.0).expect("cylinder");
        assert_round_trip(&store, &geo, body);
    }

    #[test]
    fn round_trip_sphere() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = sphere(&mut store, &mut geo, 2.0).expect("sphere");
        assert_round_trip(&store, &geo, body);
    }

    #[test]
    fn round_trip_torus() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = torus(&mut store, &mut geo, 3.0, 1.0).expect("torus");
        assert_round_trip(&store, &geo, body);
    }

    /// Truncated cone: exercises `CONICAL_SURFACE` and a slanted line seam.
    /// Bottom radius 1 at z = 0, top radius 1.5 at z = 1 (half-angle
    /// atan(0.5)), mirroring the cylinder primitive's layout.
    fn frustum(store: &mut TopologyStore, geo: &mut GeometryStore) -> EntityId<Body> {
        let (r0, r1, h) = (1.0, 1.5, 1.0);
        let axis = Vector3::z();
        let bottom_center = Point3::new(0.0, 0.0, 0.0);
        let top_center = Point3::new(0.0, 0.0, h);

        let bottom_circle = Curve3::circle(bottom_center, axis, r0).expect("circle");
        let top_circle = Curve3::circle(top_center, axis, r1).expect("circle");
        let seam =
            Curve3::line(Point3::new(r0, 0.0, 0.0), Vector3::new(r1 - r0, 0.0, h)).expect("line");
        let seam_len = ((r1 - r0) * (r1 - r0) + h * h).sqrt();
        let half_angle = ((r1 - r0) / h).atan();
        let bottom_plane = Surface3::plane(bottom_center, -axis).expect("plane");
        let top_plane = Surface3::plane(top_center, axis).expect("plane");
        let wall = Surface3::cone(bottom_center, axis, half_angle, r0).expect("cone");

        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        let v_bottom = store.create_vertex(Point3::new(r0, 0.0, 0.0), SYSTEM_RESOLUTION);
        let v_top = store.create_vertex(Point3::new(r1, 0.0, h), SYSTEM_RESOLUTION);

        let e_bottom = {
            let curve = geo.add_curve(bottom_circle);
            store.create_edge_with_curve(v_bottom, v_bottom, SYSTEM_RESOLUTION, curve, 0.0, TAU)
        };
        let e_top = {
            let curve = geo.add_curve(top_circle);
            store.create_edge_with_curve(v_top, v_top, SYSTEM_RESOLUTION, curve, 0.0, TAU)
        };
        let e_seam = {
            let curve = geo.add_curve(seam);
            store.create_edge_with_curve(v_bottom, v_top, SYSTEM_RESOLUTION, curve, 0.0, seam_len)
        };

        let f_bottom = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_bottom).expect("just created").surface =
            Some(geo.add_surface(bottom_plane));
        store.create_loop(f_bottom, LoopType::Outer, &[(e_bottom, FinSense::Reversed)]);

        let f_top = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_top).expect("just created").surface =
            Some(geo.add_surface(top_plane));
        store.create_loop(f_top, LoopType::Outer, &[(e_top, FinSense::Forward)]);

        let f_wall = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_wall).expect("just created").surface = Some(geo.add_surface(wall));
        store.create_loop(
            f_wall,
            LoopType::Outer,
            &[
                (e_bottom, FinSense::Forward),
                (e_seam, FinSense::Forward),
                (e_top, FinSense::Reversed),
                (e_seam, FinSense::Reversed),
            ],
        );

        body
    }

    #[test]
    fn round_trip_frustum_with_conical_surface() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = frustum(&mut store, &mut geo);
        assert_round_trip(&store, &geo, body);
    }

    /// Tube (hollow cylinder): the top and bottom annuli carry inner loops,
    /// exercising `FACE_BOUND` emission and genus-1 recovery.
    fn tube(store: &mut TopologyStore, geo: &mut GeometryStore) -> EntityId<Body> {
        let (r_out, r_in, h) = (2.0, 1.0, 1.0);
        let axis = Vector3::z();
        let bottom_center = Point3::new(0.0, 0.0, 0.0);
        let top_center = Point3::new(0.0, 0.0, h);

        let c_ob = geo.add_curve(Curve3::circle(bottom_center, axis, r_out).expect("circle"));
        let c_ot = geo.add_curve(Curve3::circle(top_center, axis, r_out).expect("circle"));
        let c_ib = geo.add_curve(Curve3::circle(bottom_center, axis, r_in).expect("circle"));
        let c_it = geo.add_curve(Curve3::circle(top_center, axis, r_in).expect("circle"));
        let s_out = geo.add_curve(Curve3::line(Point3::new(r_out, 0.0, 0.0), axis).expect("line"));
        let s_in = geo.add_curve(Curve3::line(Point3::new(r_in, 0.0, 0.0), axis).expect("line"));

        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);
        store.shells.get_mut(shell).expect("just created").genus = 1;

        let v_ob = store.create_vertex(Point3::new(r_out, 0.0, 0.0), SYSTEM_RESOLUTION);
        let v_ot = store.create_vertex(Point3::new(r_out, 0.0, h), SYSTEM_RESOLUTION);
        let v_ib = store.create_vertex(Point3::new(r_in, 0.0, 0.0), SYSTEM_RESOLUTION);
        let v_it = store.create_vertex(Point3::new(r_in, 0.0, h), SYSTEM_RESOLUTION);

        let e_ob = store.create_edge_with_curve(v_ob, v_ob, SYSTEM_RESOLUTION, c_ob, 0.0, TAU);
        let e_ot = store.create_edge_with_curve(v_ot, v_ot, SYSTEM_RESOLUTION, c_ot, 0.0, TAU);
        let e_ib = store.create_edge_with_curve(v_ib, v_ib, SYSTEM_RESOLUTION, c_ib, 0.0, TAU);
        let e_it = store.create_edge_with_curve(v_it, v_it, SYSTEM_RESOLUTION, c_it, 0.0, TAU);
        let e_os = store.create_edge_with_curve(v_ob, v_ot, SYSTEM_RESOLUTION, s_out, 0.0, h);
        let e_is = store.create_edge_with_curve(v_ib, v_it, SYSTEM_RESOLUTION, s_in, 0.0, h);

        let f_outer = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_outer).expect("just created").surface = Some(
            geo.add_surface(Surface3::cylinder(bottom_center, axis, r_out).expect("cylinder")),
        );
        store.create_loop(
            f_outer,
            LoopType::Outer,
            &[
                (e_ob, FinSense::Forward),
                (e_os, FinSense::Forward),
                (e_ot, FinSense::Reversed),
                (e_os, FinSense::Reversed),
            ],
        );

        // Material lies outside the inner cylinder, so the face normal
        // points at the axis: sense Negative, traversal mirrored.
        let f_inner = store.create_face(shell, FaceSense::Negative);
        store.faces.get_mut(f_inner).expect("just created").surface =
            Some(geo.add_surface(Surface3::cylinder(bottom_center, axis, r_in).expect("cylinder")));
        store.create_loop(
            f_inner,
            LoopType::Outer,
            &[
                (e_ib, FinSense::Reversed),
                (e_is, FinSense::Forward),
                (e_it, FinSense::Forward),
                (e_is, FinSense::Reversed),
            ],
        );

        let f_top = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_top).expect("just created").surface =
            Some(geo.add_surface(Surface3::plane(top_center, axis).expect("plane")));
        store.create_loop(f_top, LoopType::Outer, &[(e_ot, FinSense::Forward)]);
        store.create_loop(f_top, LoopType::Inner, &[(e_it, FinSense::Reversed)]);

        let f_bottom = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(f_bottom).expect("just created").surface =
            Some(geo.add_surface(Surface3::plane(bottom_center, -axis).expect("plane")));
        store.create_loop(f_bottom, LoopType::Outer, &[(e_ob, FinSense::Reversed)]);
        store.create_loop(f_bottom, LoopType::Inner, &[(e_ib, FinSense::Forward)]);

        body
    }

    #[test]
    fn round_trip_tube_with_inner_loops() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = tube(&mut store, &mut geo);
        let counts = store.euler_counts(body);
        assert_eq!(counts.rings, 2, "tube construction must carry two rings");
        assert!(
            store.check(body).is_empty(),
            "tube must pass check: {:?}",
            store.check(body)
        );

        // The volume half of the gate cannot run here: tessellate_body does
        // not yet triangulate planar faces with holes (NotImplemented). The
        // topology gate still applies, and geometry equality is asserted
        // directly on the re-imported stores instead.
        let text = write_step(&store, &geo, &[body], &StepWriteOptions::default())
            .expect("tube must serialize");
        let (store2, geo2, bodies) = reimport(&text);
        assert_eq!(bodies.len(), 1);
        let body2 = bodies[0];
        assert!(
            store2.check(body2).is_empty(),
            "re-imported tube must pass check: {:?}",
            store2.check(body2)
        );
        assert_counts_equal(&store, body, &store2, body2);

        // Reals are written in shortest round-trip form, so every surface
        // and curve re-imports bit-identical.
        let originals: Vec<_> = geo.surfaces.iter().map(|(_, s)| s.clone()).collect();
        let reimported: Vec<_> = geo2.surfaces.iter().map(|(_, s)| s.clone()).collect();
        for surface in &originals {
            assert!(
                reimported.contains(surface),
                "surface {surface:?} lost in round trip"
            );
        }
    }

    #[test]
    fn round_trip_two_solids_in_one_file() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let b1 = block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("block");
        let b2 = sphere(&mut store, &mut geo, 0.5).expect("sphere");

        let text = write_step(&store, &geo, &[b1, b2], &StepWriteOptions::default())
            .expect("two bodies must serialize");
        let (store2, geo2, bodies) = reimport(&text);
        assert_eq!(bodies.len(), 2, "one MANIFOLD_SOLID_BREP per body");
        // Solids re-import in emission order.
        assert_counts_equal(&store, b1, &store2, bodies[0]);
        assert_counts_equal(&store, b2, &store2, bodies[1]);
        for (original, reimported) in [(b1, bodies[0]), (b2, bodies[1])] {
            let v1 = volume(&store, &geo, original);
            let v2 = volume(&store2, &geo2, reimported);
            assert!((v1 - v2).abs() / v1.max(1.0) <= 1e-9);
        }
    }

    // ------------------------------------------------------------------
    // File structure
    // ------------------------------------------------------------------

    #[test]
    fn emits_ap203_envelope_and_skeleton() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("block");
        let text = write_step(&store, &geo, &[body], &StepWriteOptions::default()).expect("write");

        assert!(text.starts_with("ISO-10303-21;\nHEADER;\n"));
        assert!(text.ends_with("ENDSEC;\nEND-ISO-10303-21;\n"));
        assert!(text.contains("FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));"));
        for keyword in [
            "APPLICATION_CONTEXT",
            "APPLICATION_PROTOCOL_DEFINITION",
            "PRODUCT_DEFINITION_SHAPE",
            "SI_UNIT(.MILLI.,.METRE.)",
            "UNCERTAINTY_MEASURE_WITH_UNIT",
            "GEOMETRIC_REPRESENTATION_CONTEXT",
            "ADVANCED_BREP_SHAPE_REPRESENTATION",
            "SHAPE_DEFINITION_REPRESENTATION",
        ] {
            assert!(text.contains(keyword), "missing {keyword}");
        }

        // The envelope and every record must parse through our own parser.
        let file = crate::io::step::parse(&text).expect("emitted file parses");
        assert!(file.header.get("FILE_SCHEMA").is_some());
        assert!(!file.is_empty());
    }

    #[test]
    fn metre_unit_option_changes_si_unit() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("block");
        let options = StepWriteOptions {
            length_unit: LengthUnit::Metre,
            ..StepWriteOptions::default()
        };
        let text = write_step(&store, &geo, &[body], &options).expect("write");
        assert!(text.contains("SI_UNIT($,.METRE.)"));
        // The reader honours the declared unit (of-83h): a metre file's
        // coordinates come back ×1000, in the kernel's millimetres.
        let (store2, geo2, bodies) = reimport(&text);
        assert_eq!(bodies.len(), 1);
        let v = volume(&store2, &geo2, bodies[0]);
        assert!(
            (v - 1.0e9).abs() / 1.0e9 <= 1e-9,
            "1 m³ block must re-import as 1e9 mm³, got {v}"
        );
    }

    #[test]
    fn centimetre_unit_option_emits_centi_si_unit() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("block");
        let options = StepWriteOptions {
            length_unit: LengthUnit::Centimetre,
            ..StepWriteOptions::default()
        };
        let text = write_step(&store, &geo, &[body], &options).expect("write");
        assert!(text.contains("SI_UNIT(.CENTI.,.METRE.)"));
        // The reader honours the declared unit: a centimetre file's
        // coordinates come back ×10, in the kernel's millimetres.
        let (store2, geo2, bodies) = reimport(&text);
        assert_eq!(bodies.len(), 1);
        let v = volume(&store2, &geo2, bodies[0]);
        assert!(
            (v - 1000.0).abs() / 1000.0 <= 1e-9,
            "1 cm³ block must re-import as 1000 mm³, got {v}"
        );
    }

    #[test]
    fn inch_unit_option_emits_conversion_based_unit() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("block");
        let options = StepWriteOptions {
            length_unit: LengthUnit::Inch,
            ..StepWriteOptions::default()
        };
        let text = write_step(&store, &geo, &[body], &options).expect("write");
        // Non-SI: an inch is declared as 25.4 mm through a conversion unit.
        assert!(text.contains("CONVERSION_BASED_UNIT('INCH'"));
        assert!(text.contains("LENGTH_MEASURE(25.4)"));
        // The emitted file must still parse through our own parser.
        let file = crate::io::step::parse(&text).expect("emitted inch file parses");
        assert!(!file.is_empty());
        // The reader honours the declared unit: an inch file's coordinates
        // come back ×25.4, in the kernel's millimetres.
        let (store2, geo2, bodies) = reimport(&text);
        assert_eq!(bodies.len(), 1);
        let v = volume(&store2, &geo2, bodies[0]);
        let expected = 25.4_f64.powi(3);
        assert!(
            (v - expected).abs() / expected <= 1e-9,
            "1 in³ block must re-import as 25.4³ mm³, got {v}"
        );
    }

    #[test]
    fn product_name_with_apostrophe_survives() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("block");
        let options = StepWriteOptions {
            product_name: "Chase's bracket".to_string(),
            ..StepWriteOptions::default()
        };
        let text = write_step(&store, &geo, &[body], &options).expect("write");
        assert!(
            text.contains("'Chase''s bracket'"),
            "apostrophe must double"
        );

        let mut store2 = TopologyStore::new();
        let mut geo2 = GeometryStore::new();
        let import =
            read_step(&text, &mut store2, &mut geo2, &StepReadOptions::default()).expect("parse");
        assert_eq!(import.solids[0].name, "Chase's bracket");
    }

    #[test]
    fn empty_body_list_is_a_valid_file_with_no_solids() {
        let store = TopologyStore::new();
        let geo = GeometryStore::new();
        let text = write_step(&store, &geo, &[], &StepWriteOptions::default()).expect("write");
        let file = crate::io::step::parse(&text).expect("parses");
        assert!(!file.is_empty(), "skeleton entities are still present");
        let mut store2 = TopologyStore::new();
        let mut geo2 = GeometryStore::new();
        let import =
            read_step(&text, &mut store2, &mut geo2, &StepReadOptions::default()).expect("read");
        assert!(import.solids.is_empty());
    }

    // ------------------------------------------------------------------
    // Formatting
    // ------------------------------------------------------------------

    #[test]
    fn fmt_real_meets_part21_grammar_and_round_trips() {
        let cases = [
            0.0,
            1.0,
            -1.0,
            0.5,
            0.1,
            -2.5e-7,
            std::f64::consts::PI,
            TAU,
            1e300,
            1e-10,
            -12345.678901234567,
            f64::MIN_POSITIVE,
        ];
        for &x in &cases {
            let s = fmt_real(x);
            assert!(s.contains('.'), "{s} lacks the mandatory decimal point");
            assert!(!s.contains('e'), "{s} has a lower-case exponent marker");
            assert_eq!(
                s.parse::<f64>().expect("parses as f64"),
                x,
                "{s} must round-trip to the identical f64"
            );
        }
        assert_eq!(fmt_real(1.0), "1.0");
        assert_eq!(fmt_real(1e300), "1.0E300");
        assert_eq!(fmt_real(-2.5e-7), "-2.5E-7");
    }

    #[test]
    fn string_literal_escapes_apostrophes() {
        assert_eq!(string_literal(""), "''");
        assert_eq!(string_literal("plain"), "'plain'");
        assert_eq!(string_literal("it's"), "'it''s'");
        assert_eq!(string_literal("''"), "''''''");
    }

    #[test]
    fn ellipse_curve_emits_major_axis_as_ref_direction() {
        let store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let curve_id = geo.add_curve(
            Curve3::ellipse(
                Point3::new(1.0, 2.0, 3.0),
                Vector3::z(),
                Vector3::x(),
                4.0,
                2.5,
            )
            .expect("ellipse"),
        );
        let mut emitter = Emitter {
            store: &store,
            geo: &geo,
            data: String::new(),
            next_id: 1,
            vertices: HashMap::new(),
            edges: HashMap::new(),
            curves: HashMap::new(),
            surfaces: HashMap::new(),
        };
        emitter.emit_curve(curve_id).expect("ellipse serializes");
        // Placement is #4 (point #1, axis #2, ref #3); ref_direction is the
        // major axis and semi-axes come out major-first, so the reader maps
        // it back without swapping.
        assert!(
            emitter.data.contains("ELLIPSE('',#4,4.0,2.5)"),
            "semi-axes must be emitted major-first:\n{}",
            emitter.data
        );
        assert!(emitter.data.contains("DIRECTION('',(1.0,0.0,0.0))"));
    }

    // ------------------------------------------------------------------
    // Errors
    // ------------------------------------------------------------------

    #[test]
    fn rejects_non_solid_bodies() {
        let mut store = TopologyStore::new();
        let geo = GeometryStore::new();
        let body = store.create_body(BodyType::Sheet);
        let err = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap_err();
        assert!(matches!(err, StepWriteError::Unsupported(_)), "{err}");
    }

    #[test]
    fn rejects_multi_shell_bodies() {
        let mut store = TopologyStore::new();
        let geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        store.create_shell(body, true, ShellOrientation::Outward);
        store.create_shell(body, true, ShellOrientation::Inward);
        let err = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap_err();
        assert!(matches!(err, StepWriteError::Unsupported(_)), "{err}");
    }

    #[test]
    fn rejects_open_shells() {
        let mut store = TopologyStore::new();
        let geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        store.create_shell(body, false, ShellOrientation::Outward);
        let err = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap_err();
        assert!(matches!(err, StepWriteError::Unsupported(_)), "{err}");
    }

    #[test]
    fn rejects_stale_body_ids() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("block");
        store.bodies.remove(body);
        let err = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap_err();
        assert_eq!(err, StepWriteError::StaleBody);
    }

    #[test]
    fn rejects_face_without_surface() {
        let mut store = TopologyStore::new();
        let geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);
        store.create_face(shell, FaceSense::Positive);
        let err = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap_err();
        assert!(
            matches!(&err, StepWriteError::Invalid(what) if what.contains("no surface")),
            "{err}"
        );
    }

    #[test]
    fn rejects_edge_without_curve() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = block(&mut store, &mut geo, 1.0, 1.0, 1.0).expect("block");
        // Reach any edge through the topology graph and detach its curve.
        let shell = store.bodies.get(body).expect("body").shells[0];
        let face = store.shells.get(shell).expect("shell").faces[0];
        let outer = store
            .faces
            .get(face)
            .expect("face")
            .outer_loop
            .expect("loop");
        let fin = store.loops.get(outer).expect("loop").fins[0];
        let edge = store.fins.get(fin).expect("fin").edge;
        store.edges.get_mut(edge).expect("edge").curve = None;

        let err = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap_err();
        assert!(
            matches!(&err, StepWriteError::Invalid(what) if what.contains("no curve")),
            "{err}"
        );
    }

    #[test]
    fn rejects_vertex_loops() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);
        let face = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face).expect("face").surface = Some(
            geo.add_surface(Surface3::sphere(Point3::origin(), Vector3::z(), 1.0).expect("sphere")),
        );
        let vertex = store.create_vertex(Point3::new(0.0, 0.0, 1.0), SYSTEM_RESOLUTION);
        let vertex_loop = store.loops.insert(Loop {
            face,
            fins: Vec::new(),
            loop_type: LoopType::Vertex,
            vertex: Some(vertex),
        });
        store.faces.get_mut(face).expect("face").outer_loop = Some(vertex_loop);

        let err = write_step(&store, &geo, &[body], &StepWriteOptions::default()).unwrap_err();
        assert!(matches!(err, StepWriteError::Unsupported(_)), "{err}");
    }
}
