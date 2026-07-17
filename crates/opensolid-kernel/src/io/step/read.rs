//! STEP AP203 semantic mapper: parsed entity graph → kernel B-Rep.
//!
//! [`read_step`] walks every `MANIFOLD_SOLID_BREP` in a parsed
//! [`StepFile`] and rebuilds it through the kernel's public construction
//! APIs ([`TopologyStore`] / [`GeometryStore`]), exactly when possible:
//!
//! - **Geometry**: `cartesian_point`, `direction`, `vector`,
//!   `axis2_placement_3d`; `plane`, `cylindrical_surface`,
//!   `conical_surface`, `spherical_surface`, `toroidal_surface` →
//!   [`Surface3`]; `line`, `circle`, `ellipse` → [`Curve3`].
//! - **Topology**: `vertex_point`, `edge_curve`, `oriented_edge`,
//!   `edge_loop`, `face_bound` / `face_outer_bound`, `advanced_face`,
//!   `closed_shell`, `manifold_solid_brep` → `Vertex` / `Edge` / `Loop` /
//!   `Face` / `Shell` / `Body`.
//!
//! STEP trims edges by their vertices, not by curve parameters, so the
//! mapper recovers each edge's parameter range by inverse-projecting the
//! vertex points onto the mapped curve, and re-orients the curve when
//! `edge_curve.same_sense` is false so every edge satisfies
//! `t_start < t_end`. Every mapped body is validated with
//! [`TopologyStore::check`].
//!
//! # Mesh fallback
//!
//! `b_spline_curve_with_knots` / `b_spline_surface_with_knots` parse into
//! [`NurbsCurve`] / [`NurbsSurface`] for evaluation, but the geometry
//! store's [`Curve3`] / [`Surface3`] enums have no NURBS variants yet, so
//! exact NURBS import is unsupported. Solids containing NURBS (or any
//! other unmappable entity), and solids whose mapped topology fails
//! `check`, fall back to a **tessellated import**: each face is
//! triangulated straight from the STEP graph (planar faces ear-clipped
//! from their boundary polylines, quadrics gridded over their parameter
//! rectangle, NURBS patches gridded over their knot domain), welded, and
//! wrapped as a [`MeshSdf`] — an F-Rep field ready for CSG. Faces of one
//! solid share each edge's discretization, so junctions weld watertight.
//! Anything the fallback cannot handle either fails the solid
//! ([`SolidOutcome::Failed`]) with per-entity [`Diagnostic`]s explaining
//! why. A failed or fallen-back solid leaves no partial entities in the
//! stores.
//!
//! # Units
//!
//! The declared length unit is honoured: the reader resolves the
//! `LENGTH_UNIT` of every `GLOBAL_UNIT_ASSIGNED_CONTEXT` (`SI_UNIT`
//! prefix/name, or a `CONVERSION_BASED_UNIT` such as inch) and scales all
//! coordinates, vector magnitudes, and radii into the kernel convention,
//! **millimetres**, on import. The applied factor is exposed as
//! [`StepImport::length_scale`]. Files with no interpretable length unit
//! import verbatim (scale 1); an uninterpretable or conflicting
//! declaration emits a [`Severity::Warning`] diagnostic. The declared
//! plane-angle unit is honoured the same way: the reader resolves the
//! `PLANE_ANGLE_UNIT` (`SI_UNIT` radian, or a `CONVERSION_BASED_UNIT` such
//! as degree) and scales all angle measures (e.g. conical-surface
//! semi-angles) into radians, the kernel convention. The applied factor is
//! exposed as [`StepImport::angle_scale`]. Files with no interpretable angle
//! unit import angles verbatim (scale 1).
//!
//! # Example
//!
//! ```
//! use opensolid_kernel::brep::{GeometryStore, TopologyStore};
//! use opensolid_kernel::io::step::read::{SolidOutcome, StepReadOptions, read_step};
//!
//! // A sphere of radius 2: one spherical face closed by a seam meridian.
//! let src = "\
//! ISO-10303-21;
//! HEADER;
//! FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));
//! ENDSEC;
//! DATA;
//! #1 = CARTESIAN_POINT('', (0., 0., 0.));
//! #2 = CARTESIAN_POINT('', (0., 0., -2.));
//! #3 = CARTESIAN_POINT('', (0., 0., 2.));
//! #4 = DIRECTION('', (0., 0., 1.));
//! #5 = DIRECTION('', (0., -1., 0.));
//! #6 = DIRECTION('', (1., 0., 0.));
//! #7 = VERTEX_POINT('', #2);
//! #8 = VERTEX_POINT('', #3);
//! #9 = AXIS2_PLACEMENT_3D('', #1, #4, #6);
//! #10 = AXIS2_PLACEMENT_3D('', #1, #5, #6);
//! #11 = CIRCLE('', #10, 2.);
//! #12 = SPHERICAL_SURFACE('', #9, 2.);
//! #13 = EDGE_CURVE('', #7, #8, #11, .T.);
//! #14 = ORIENTED_EDGE('', *, *, #13, .T.);
//! #15 = ORIENTED_EDGE('', *, *, #13, .F.);
//! #16 = EDGE_LOOP('', (#14, #15));
//! #17 = FACE_OUTER_BOUND('', #16, .T.);
//! #18 = ADVANCED_FACE('', (#17), #12, .T.);
//! #19 = CLOSED_SHELL('', (#18));
//! #20 = MANIFOLD_SOLID_BREP('ball', #19);
//! ENDSEC;
//! END-ISO-10303-21;
//! ";
//! let mut store = TopologyStore::new();
//! let mut geo = GeometryStore::new();
//! let import = read_step(src, &mut store, &mut geo, &StepReadOptions::default()).unwrap();
//! match &import.solids[0].outcome {
//!     SolidOutcome::BRep(body) => assert!(store.check(*body).is_empty()),
//!     other => panic!("expected an exact B-Rep import, got {other:?}"),
//! }
//! ```

use std::collections::HashMap;

use opensolid_brep::curve::plane_basis;
use opensolid_brep::triangulate::{ear_clip_rings, signed_area2};
use opensolid_brep::{
    Body, BodyType, Curve3, CurveEval, Edge, Face, FaceSense, Fin, FinSense, GeometryStore,
    KnotVector, Loop, LoopType, NurbsCurve, NurbsError, NurbsSurface, SYSTEM_RESOLUTION, Shell,
    ShellOrientation, Surface3, SurfaceEval, SurfaceProject, TessellationOptions, TopologyStore,
    Vertex,
};
use opensolid_core::error::CoreError;
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::{EntityId, Point3, Vector3};

use super::{Instance, SimpleRecord, StepError, StepFile, Value};
use crate::convert::MeshSdf;

const TAU: f64 = std::f64::consts::TAU;

/// Relative tolerance for verifying that mapped curves interpolate their
/// edge's vertex points. STEP files are written with finite decimal
/// precision, so this is far looser than [`SYSTEM_RESOLUTION`].
const TRIM_TOL_REL: f64 = 1e-6;

/// Options for [`read_step`].
#[derive(Debug, Clone, Default)]
pub struct StepReadOptions {
    /// Fidelity of the mesh-fallback tessellation.
    pub tessellation: TessellationOptions,
}

/// How serious a [`Diagnostic`] is.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum Severity {
    /// Context on a decision the importer made (e.g. trimming ignored).
    Info,
    /// Valid STEP the kernel cannot represent exactly; import degrades
    /// (typically to the mesh fallback) but continues.
    Warning,
    /// Malformed data or an unrecoverable failure for the affected solid.
    Error,
}

/// One per-entity finding from the import.
#[derive(Debug, Clone)]
pub struct Diagnostic {
    /// The STEP instance name (`#id`) the finding is about, when known.
    pub entity: Option<u64>,
    pub severity: Severity,
    pub message: String,
}

/// What one `MANIFOLD_SOLID_BREP` imported as.
#[derive(Debug)]
pub enum SolidOutcome {
    /// Exact B-Rep import: the body (and its geometry) lives in the stores
    /// passed to [`read_step`] and passed [`TopologyStore::check`].
    BRep(EntityId<Body>),
    /// Tessellated fallback: a closed manifold mesh wrapped as an SDF.
    /// Nothing was added to the stores.
    Mesh {
        /// The welded fallback tessellation.
        mesh: TriangleMesh,
        /// The mesh as a signed distance field, ready for F-Rep CSG.
        sdf: Box<MeshSdf>,
    },
    /// Neither path succeeded; see the report's [`Diagnostic`]s.
    Failed,
}

/// One imported `MANIFOLD_SOLID_BREP`.
#[derive(Debug)]
pub struct ImportedSolid {
    /// STEP instance name (`#id`) of the `MANIFOLD_SOLID_BREP`.
    pub step_id: u64,
    /// The entity's name attribute (often empty).
    pub name: String,
    pub outcome: SolidOutcome,
}

/// Result of importing a STEP file: one entry per solid, plus every
/// per-entity diagnostic gathered along the way.
#[derive(Debug)]
pub struct StepImport {
    pub solids: Vec<ImportedSolid>,
    pub diagnostics: Vec<Diagnostic>,
    /// Millimetres per file length unit, resolved from the file's
    /// `GLOBAL_UNIT_ASSIGNED_CONTEXT` (1.0 when no length unit is declared
    /// or it cannot be interpreted). All imported geometry has already
    /// been multiplied by this factor — coordinates in the stores (and
    /// fallback meshes) are always millimetres.
    pub length_scale: f64,
    /// Radians per file plane-angle unit, resolved from the file's
    /// `GLOBAL_UNIT_ASSIGNED_CONTEXT` (1.0 when no angle unit is declared or
    /// it cannot be interpreted). All imported angle measures (e.g. conical
    /// surface semi-angles) have already been multiplied by this factor —
    /// angles in the stores are always radians.
    pub angle_scale: f64,
}

impl StepImport {
    /// Whether any diagnostic is [`Severity::Error`].
    pub fn has_errors(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error)
    }
}

/// Parse STEP Part 21 source and map every `MANIFOLD_SOLID_BREP` into the
/// given stores (see the [module docs](self) for exact-vs-fallback rules).
///
/// # Errors
/// [`StepError`] if the file is not syntactically valid Part 21. Semantic
/// problems never fail the call; they are reported per solid through
/// [`StepImport::diagnostics`] and each solid's [`SolidOutcome`].
pub fn read_step(
    source: &str,
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    options: &StepReadOptions,
) -> Result<StepImport, StepError> {
    let file = super::parse(source)?;
    Ok(map_file(&file, store, geo, options))
}

/// [`read_step`] over raw bytes (STEP files are ASCII/Latin-1).
///
/// # Errors
/// As [`read_step`].
pub fn read_step_bytes(
    source: &[u8],
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    options: &StepReadOptions,
) -> Result<StepImport, StepError> {
    let file = super::parse_bytes(source)?;
    Ok(map_file(&file, store, geo, options))
}

// ---------------------------------------------------------------------
// Internal error type
// ---------------------------------------------------------------------

/// Why mapping (or fallback-meshing) an entity failed.
#[derive(Debug)]
enum MapError {
    /// Valid STEP the kernel cannot represent exactly.
    Unsupported { entity: u64, what: String },
    /// Malformed or unresolvable data.
    Invalid { entity: u64, what: String },
}

type MapResult<T> = Result<T, MapError>;

impl MapError {
    fn diagnostic(&self) -> Diagnostic {
        match self {
            MapError::Unsupported { entity, what } => Diagnostic {
                entity: Some(*entity),
                severity: Severity::Warning,
                message: format!("unsupported: {what}"),
            },
            MapError::Invalid { entity, what } => Diagnostic {
                entity: Some(*entity),
                severity: Severity::Error,
                message: what.clone(),
            },
        }
    }
}

fn invalid(entity: u64, what: impl Into<String>) -> MapError {
    MapError::Invalid {
        entity,
        what: what.into(),
    }
}

fn unsupported(entity: u64, what: impl Into<String>) -> MapError {
    MapError::Unsupported {
        entity,
        what: what.into(),
    }
}

/// A geometry constructor rejected the mapped parameters.
fn geometry_error(entity: u64, error: &CoreError) -> MapError {
    invalid(entity, format!("invalid geometry: {error}"))
}

fn nurbs_error(entity: u64, error: &NurbsError) -> MapError {
    invalid(entity, format!("invalid B-spline data: {error}"))
}

// ---------------------------------------------------------------------
// Attribute and instance access
// ---------------------------------------------------------------------

fn attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<&Value> {
    rec.attributes.get(index).ok_or_else(|| {
        invalid(
            entity,
            format!(
                "{} has {} attributes, expected at least {}",
                rec.type_name,
                rec.attributes.len(),
                index + 1
            ),
        )
    })
}

/// Numeric coercion: STEP writers sometimes emit `0` where a real is
/// expected, and measures arrive wrapped (`LENGTH_MEASURE(1.0)`).
fn as_number(value: &Value) -> Option<f64> {
    match value {
        Value::Real(x) => Some(*x),
        Value::Integer(n) => Some(*n as f64),
        Value::Typed { value, .. } => as_number(value),
        _ => None,
    }
}

fn real_attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<f64> {
    let value = attr(rec, index, entity)?;
    as_number(value).ok_or_else(|| {
        invalid(
            entity,
            format!("{} attribute {index} is not a number", rec.type_name),
        )
    })
}

fn int_attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<i64> {
    attr(rec, index, entity)?.as_integer().ok_or_else(|| {
        invalid(
            entity,
            format!("{} attribute {index} is not an integer", rec.type_name),
        )
    })
}

fn ref_attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<u64> {
    attr(rec, index, entity)?.as_ref_id().ok_or_else(|| {
        invalid(
            entity,
            format!(
                "{} attribute {index} is not an instance reference",
                rec.type_name
            ),
        )
    })
}

fn bool_attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<bool> {
    match attr(rec, index, entity)?.as_enum() {
        Some("T") | Some("TRUE") => Ok(true),
        Some("F") | Some("FALSE") => Ok(false),
        _ => Err(invalid(
            entity,
            format!(
                "{} attribute {index} is not a .T./.F. boolean",
                rec.type_name
            ),
        )),
    }
}

fn list_attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<&[Value]> {
    attr(rec, index, entity)?.as_list().ok_or_else(|| {
        invalid(
            entity,
            format!("{} attribute {index} is not a list", rec.type_name),
        )
    })
}

fn ref_list(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<Vec<u64>> {
    list_attr(rec, index, entity)?
        .iter()
        .map(|v| {
            v.as_ref_id().ok_or_else(|| {
                invalid(
                    entity,
                    format!(
                        "{} attribute {index} contains a non-reference item",
                        rec.type_name
                    ),
                )
            })
        })
        .collect()
}

fn real_list(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<Vec<f64>> {
    list_attr(rec, index, entity)?
        .iter()
        .map(|v| {
            as_number(v).ok_or_else(|| {
                invalid(
                    entity,
                    format!(
                        "{} attribute {index} contains a non-numeric item",
                        rec.type_name
                    ),
                )
            })
        })
        .collect()
}

fn int_list(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<Vec<i64>> {
    list_attr(rec, index, entity)?
        .iter()
        .map(|v| {
            v.as_integer().ok_or_else(|| {
                invalid(
                    entity,
                    format!(
                        "{} attribute {index} contains a non-integer item",
                        rec.type_name
                    ),
                )
            })
        })
        .collect()
}

/// The name attribute (index 0) of a record, or `""` when absent/unset.
fn name_attr(rec: &SimpleRecord) -> String {
    rec.attributes
        .first()
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string()
}

fn instance(file: &StepFile, id: u64, referrer: u64) -> MapResult<&Instance> {
    file.get(id)
        .ok_or_else(|| invalid(referrer, format!("dangling reference #{id}")))
}

/// Human-readable type name(s) of an instance, for messages.
fn type_names(inst: &Instance) -> String {
    match inst.entity.as_complex() {
        Some(parts) => parts
            .iter()
            .map(|r| r.type_name.as_str())
            .collect::<Vec<_>>()
            .join("+"),
        None => inst
            .entity
            .as_simple()
            .expect("entity is simple or complex")
            .type_name
            .clone(),
    }
}

/// The partial record of type `type_name` bound to instance `id`
/// (searching complex instances' parts).
fn typed_record<'f>(
    file: &'f StepFile,
    id: u64,
    type_name: &str,
    referrer: u64,
) -> MapResult<&'f SimpleRecord> {
    let inst = instance(file, id, referrer)?;
    inst.entity.part(type_name).ok_or_else(|| {
        invalid(
            id,
            format!("expected {type_name}, found {}", type_names(inst)),
        )
    })
}

// ---------------------------------------------------------------------
// Unit resolution
// ---------------------------------------------------------------------

/// Multiplier for an ISO 10303-41 `si_prefix` enumeration name.
fn si_prefix_factor(prefix: &str) -> Option<f64> {
    Some(match prefix {
        "EXA" => 1e18,
        "PETA" => 1e15,
        "TERA" => 1e12,
        "GIGA" => 1e9,
        "MEGA" => 1e6,
        "KILO" => 1e3,
        "HECTO" => 1e2,
        "DECA" => 1e1,
        "DECI" => 1e-1,
        "CENTI" => 1e-2,
        "MILLI" => 1e-3,
        "MICRO" => 1e-6,
        "NANO" => 1e-9,
        "PICO" => 1e-12,
        "FEMTO" => 1e-15,
        "ATTO" => 1e-18,
        _ => return None,
    })
}

/// Millimetres per one of length unit `#unit_id`: an `SI_UNIT` is a prefix
/// of the metre, a `CONVERSION_BASED_UNIT` (e.g. inch) chains through its
/// `(LENGTH_)MEASURE_WITH_UNIT` into another length unit, followed at most
/// `depth` links deep (guards reference cycles). `None` when the
/// declaration cannot be interpreted.
fn length_unit_in_mm(file: &StepFile, unit_id: u64, depth: u32) -> Option<f64> {
    if depth == 0 {
        return None;
    }
    let inst = file.get(unit_id)?;
    if let Some(si) = inst.entity.part("SI_UNIT") {
        let prefix = match si.attributes.first()? {
            Value::Unset => 1.0,
            Value::Enum(name) => si_prefix_factor(name)?,
            _ => return None,
        };
        (si.attributes.get(1)?.as_enum()? == "METRE").then_some(prefix * 1000.0)
    } else if let Some(cbu) = inst.entity.part("CONVERSION_BASED_UNIT") {
        // CONVERSION_BASED_UNIT(name, conversion_factor): the factor is a
        // measure-with-unit whose value counts another length unit.
        let measure = file.get(cbu.attributes.get(1)?.as_ref_id()?)?;
        let rec = measure
            .entity
            .part("LENGTH_MEASURE_WITH_UNIT")
            .or_else(|| measure.entity.part("MEASURE_WITH_UNIT"))?;
        let value = as_number(rec.attributes.first()?)?;
        if !(value.is_finite() && value > 0.0) {
            return None;
        }
        let base = length_unit_in_mm(file, rec.attributes.get(1)?.as_ref_id()?, depth - 1)?;
        Some(value * base)
    } else {
        None
    }
}

/// Resolve the file's declared length unit to a coordinate scale factor
/// (millimetres per file unit — the kernel convention is millimetres).
/// Files declaring no length unit import verbatim (scale 1). An
/// uninterpretable declaration warns and is skipped; declarations that
/// disagree across contexts warn and the first interpretable one wins.
fn resolve_length_scale(file: &StepFile, diagnostics: &mut Vec<Diagnostic>) -> f64 {
    let mut resolved: Option<(u64, f64)> = None;
    for inst in &file.data {
        let Some(ctx) = inst.entity.part("GLOBAL_UNIT_ASSIGNED_CONTEXT") else {
            continue;
        };
        let Some(units) = ctx.attributes.first().and_then(Value::as_list) else {
            continue;
        };
        for unit in units {
            let Some(unit_id) = unit.as_ref_id() else {
                continue;
            };
            let is_length = file
                .get(unit_id)
                .is_some_and(|u| u.entity.part("LENGTH_UNIT").is_some());
            if !is_length {
                continue;
            }
            match (length_unit_in_mm(file, unit_id, 4), resolved) {
                (Some(scale), None) => resolved = Some((unit_id, scale)),
                (Some(scale), Some((first_id, first))) => {
                    if (scale - first).abs() > first.abs() * 1e-9 {
                        diagnostics.push(Diagnostic {
                            entity: Some(unit_id),
                            severity: Severity::Warning,
                            message: format!(
                                "conflicting length units: #{first_id} is {first} mm but \
                                 #{unit_id} is {scale} mm; using #{first_id}"
                            ),
                        });
                    }
                }
                (None, _) => diagnostics.push(Diagnostic {
                    entity: Some(unit_id),
                    severity: Severity::Warning,
                    message: "cannot interpret declared LENGTH_UNIT; coordinates import verbatim"
                        .to_string(),
                }),
            }
        }
    }
    let Some((unit_id, scale)) = resolved else {
        return 1.0;
    };
    if scale != 1.0 {
        diagnostics.push(Diagnostic {
            entity: Some(unit_id),
            severity: Severity::Info,
            message: format!(
                "declared length unit is {scale} mm; coordinates scaled into millimetres"
            ),
        });
    }
    scale
}

/// Radians per one of plane-angle unit `#unit_id`: an `SI_UNIT` is a
/// (possibly prefixed) radian, a `CONVERSION_BASED_UNIT` (e.g. degree)
/// chains through its `(PLANE_ANGLE_)MEASURE_WITH_UNIT` into another angle
/// unit, followed at most `depth` links deep (guards reference cycles).
/// `None` when the declaration cannot be interpreted.
fn angle_unit_in_rad(file: &StepFile, unit_id: u64, depth: u32) -> Option<f64> {
    if depth == 0 {
        return None;
    }
    let inst = file.get(unit_id)?;
    if let Some(si) = inst.entity.part("SI_UNIT") {
        let prefix = match si.attributes.first()? {
            Value::Unset => 1.0,
            Value::Enum(name) => si_prefix_factor(name)?,
            _ => return None,
        };
        (si.attributes.get(1)?.as_enum()? == "RADIAN").then_some(prefix)
    } else if let Some(cbu) = inst.entity.part("CONVERSION_BASED_UNIT") {
        // CONVERSION_BASED_UNIT(name, conversion_factor): the factor is a
        // measure-with-unit whose value counts another angle unit
        // (e.g. DEGREE = 0.017453… rad).
        let measure = file.get(cbu.attributes.get(1)?.as_ref_id()?)?;
        let rec = measure
            .entity
            .part("PLANE_ANGLE_MEASURE_WITH_UNIT")
            .or_else(|| measure.entity.part("MEASURE_WITH_UNIT"))?;
        let value = as_number(rec.attributes.first()?)?;
        if !(value.is_finite() && value > 0.0) {
            return None;
        }
        let base = angle_unit_in_rad(file, rec.attributes.get(1)?.as_ref_id()?, depth - 1)?;
        Some(value * base)
    } else {
        None
    }
}

/// Resolve the file's declared plane-angle unit to a scale factor (radians
/// per file angle unit — the kernel convention is radians). Files declaring
/// no angle unit import verbatim (scale 1). An uninterpretable declaration
/// warns and is skipped; declarations that disagree across contexts warn and
/// the first interpretable one wins. Mirrors [`resolve_length_scale`].
fn resolve_angle_scale(file: &StepFile, diagnostics: &mut Vec<Diagnostic>) -> f64 {
    let mut resolved: Option<(u64, f64)> = None;
    for inst in &file.data {
        let Some(ctx) = inst.entity.part("GLOBAL_UNIT_ASSIGNED_CONTEXT") else {
            continue;
        };
        let Some(units) = ctx.attributes.first().and_then(Value::as_list) else {
            continue;
        };
        for unit in units {
            let Some(unit_id) = unit.as_ref_id() else {
                continue;
            };
            let is_angle = file
                .get(unit_id)
                .is_some_and(|u| u.entity.part("PLANE_ANGLE_UNIT").is_some());
            if !is_angle {
                continue;
            }
            match (angle_unit_in_rad(file, unit_id, 4), resolved) {
                (Some(scale), None) => resolved = Some((unit_id, scale)),
                (Some(scale), Some((first_id, first))) => {
                    if (scale - first).abs() > first.abs() * 1e-9 {
                        diagnostics.push(Diagnostic {
                            entity: Some(unit_id),
                            severity: Severity::Warning,
                            message: format!(
                                "conflicting plane-angle units: #{first_id} is {first} rad but \
                                 #{unit_id} is {scale} rad; using #{first_id}"
                            ),
                        });
                    }
                }
                (None, _) => diagnostics.push(Diagnostic {
                    entity: Some(unit_id),
                    severity: Severity::Warning,
                    message: "cannot interpret declared PLANE_ANGLE_UNIT; angles import verbatim"
                        .to_string(),
                }),
            }
        }
    }
    let Some((unit_id, scale)) = resolved else {
        return 1.0;
    };
    if scale != 1.0 {
        diagnostics.push(Diagnostic {
            entity: Some(unit_id),
            severity: Severity::Info,
            message: format!(
                "declared plane-angle unit is {scale} rad; angles scaled into radians"
            ),
        });
    }
    scale
}

// ---------------------------------------------------------------------
// Geometry resolvers
// ---------------------------------------------------------------------

fn triple(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<[f64; 3]> {
    let items = list_attr(rec, index, entity)?;
    if items.len() != 3 {
        return Err(invalid(
            entity,
            format!(
                "{} expects 3 coordinates, found {}",
                rec.type_name,
                items.len()
            ),
        ));
    }
    let mut out = [0.0; 3];
    for (slot, item) in out.iter_mut().zip(items) {
        *slot = as_number(item)
            .ok_or_else(|| invalid(entity, format!("{}: non-numeric coordinate", rec.type_name)))?;
    }
    Ok(out)
}

/// `scale` is the file's length-unit factor (mm per file unit); it
/// multiplies every length-valued quantity so imported geometry is always
/// millimetres.
fn resolve_point(file: &StepFile, id: u64, referrer: u64, scale: f64) -> MapResult<Point3> {
    let rec = typed_record(file, id, "CARTESIAN_POINT", referrer)?;
    let [x, y, z] = triple(rec, 1, id)?;
    Ok(Point3::new(x * scale, y * scale, z * scale))
}

fn resolve_direction(file: &StepFile, id: u64, referrer: u64) -> MapResult<Vector3> {
    let rec = typed_record(file, id, "DIRECTION", referrer)?;
    let [x, y, z] = triple(rec, 1, id)?;
    Ok(Vector3::new(x, y, z))
}

/// `VECTOR(name, orientation, magnitude)` → direction scaled by magnitude
/// (a length measure, so the unit scale applies).
fn resolve_vector(file: &StepFile, id: u64, referrer: u64, scale: f64) -> MapResult<Vector3> {
    let rec = typed_record(file, id, "VECTOR", referrer)?;
    let dir = resolve_direction(file, ref_attr(rec, 1, id)?, id)?;
    let magnitude = real_attr(rec, 2, id)?;
    Ok(dir * (magnitude * scale))
}

/// A resolved `AXIS2_PLACEMENT_3D`: location plus its z axis and optional
/// x reference direction (defaults per ISO 10303-42: axis → +Z).
struct Placement {
    location: Point3,
    axis: Vector3,
    ref_dir: Option<Vector3>,
}

fn resolve_axis2(file: &StepFile, id: u64, referrer: u64, scale: f64) -> MapResult<Placement> {
    let rec = typed_record(file, id, "AXIS2_PLACEMENT_3D", referrer)?;
    let location = resolve_point(file, ref_attr(rec, 1, id)?, id, scale)?;
    let axis = match attr(rec, 2, id)? {
        Value::Unset => Vector3::z(),
        Value::Ref(dir) => resolve_direction(file, *dir, id)?,
        _ => {
            return Err(invalid(
                id,
                "AXIS2_PLACEMENT_3D axis is neither $ nor a reference",
            ));
        }
    };
    let ref_dir = match attr(rec, 3, id)? {
        Value::Unset => None,
        Value::Ref(dir) => Some(resolve_direction(file, *dir, id)?),
        _ => {
            return Err(invalid(
                id,
                "AXIS2_PLACEMENT_3D ref_direction is neither $ nor a reference",
            ));
        }
    };
    Ok(Placement {
        location,
        axis,
        ref_dir,
    })
}

/// A face surface as mapped from STEP: exact, or NURBS (evaluation only —
/// the geometry store cannot hold it, so it forces the mesh fallback).
enum RawSurface {
    Analytic(Surface3),
    Nurbs(Box<NurbsSurface>),
}

fn resolve_surface(
    file: &StepFile,
    id: u64,
    referrer: u64,
    scale: f64,
    angle_scale: f64,
) -> MapResult<RawSurface> {
    let inst = instance(file, id, referrer)?;
    let Some(rec) = inst.as_simple() else {
        return Err(unsupported(
            id,
            format!("complex surface instance ({})", type_names(inst)),
        ));
    };
    let placement = |index: usize| -> MapResult<Placement> {
        resolve_axis2(file, ref_attr(rec, index, id)?, id, scale)
    };
    match rec.type_name.as_str() {
        "PLANE" => {
            let p = placement(1)?;
            Ok(RawSurface::Analytic(
                Surface3::plane(p.location, p.axis).map_err(|e| geometry_error(id, &e))?,
            ))
        }
        "CYLINDRICAL_SURFACE" => {
            let p = placement(1)?;
            let radius = real_attr(rec, 2, id)? * scale;
            Ok(RawSurface::Analytic(
                Surface3::cylinder(p.location, p.axis, radius)
                    .map_err(|e| geometry_error(id, &e))?,
            ))
        }
        "CONICAL_SURFACE" => {
            let p = placement(1)?;
            let radius = real_attr(rec, 2, id)? * scale;
            // semi_angle is a plane-angle measure: scale into radians (the
            // length scale never applies).
            let semi_angle = real_attr(rec, 3, id)? * angle_scale;
            Ok(RawSurface::Analytic(
                Surface3::cone(p.location, p.axis, semi_angle, radius)
                    .map_err(|e| geometry_error(id, &e))?,
            ))
        }
        "SPHERICAL_SURFACE" => {
            let p = placement(1)?;
            let radius = real_attr(rec, 2, id)? * scale;
            Ok(RawSurface::Analytic(
                Surface3::sphere(p.location, p.axis, radius).map_err(|e| geometry_error(id, &e))?,
            ))
        }
        "TOROIDAL_SURFACE" => {
            let p = placement(1)?;
            let major = real_attr(rec, 2, id)? * scale;
            let minor = real_attr(rec, 3, id)? * scale;
            Ok(RawSurface::Analytic(
                Surface3::torus(p.location, p.axis, major, minor)
                    .map_err(|e| geometry_error(id, &e))?,
            ))
        }
        "B_SPLINE_SURFACE_WITH_KNOTS" => Ok(RawSurface::Nurbs(Box::new(resolve_bspline_surface(
            file, rec, id, scale,
        )?))),
        other => Err(unsupported(id, format!("surface type {other}"))),
    }
}

/// An edge curve as mapped from STEP (same split as [`RawSurface`]).
enum RawCurve {
    Analytic(Curve3),
    Nurbs(Box<NurbsCurve>),
}

fn resolve_curve(file: &StepFile, id: u64, referrer: u64, scale: f64) -> MapResult<RawCurve> {
    let inst = instance(file, id, referrer)?;
    let Some(rec) = inst.as_simple() else {
        return Err(unsupported(
            id,
            format!("complex curve instance ({})", type_names(inst)),
        ));
    };
    match rec.type_name.as_str() {
        "LINE" => {
            let origin = resolve_point(file, ref_attr(rec, 1, id)?, id, scale)?;
            let dir = resolve_vector(file, ref_attr(rec, 2, id)?, id, scale)?;
            Ok(RawCurve::Analytic(
                Curve3::line(origin, dir).map_err(|e| geometry_error(id, &e))?,
            ))
        }
        "CIRCLE" => {
            let p = resolve_axis2(file, ref_attr(rec, 1, id)?, id, scale)?;
            let radius = real_attr(rec, 2, id)? * scale;
            Ok(RawCurve::Analytic(
                Curve3::circle(p.location, p.axis, radius).map_err(|e| geometry_error(id, &e))?,
            ))
        }
        "ELLIPSE" => {
            let p = resolve_axis2(file, ref_attr(rec, 1, id)?, id, scale)?;
            let semi_1 = real_attr(rec, 2, id)? * scale;
            let semi_2 = real_attr(rec, 3, id)? * scale;
            let axis_norm = p.axis.norm();
            if axis_norm == 0.0 || !axis_norm.is_finite() {
                return Err(invalid(id, "ELLIPSE placement axis is degenerate"));
            }
            let unit_axis = p.axis / axis_norm;
            let x_dir = p.ref_dir.unwrap_or_else(|| plane_basis(&unit_axis).0);
            // STEP's semi_axis_1 lies along ref_direction but need not be the
            // larger one; Curve3 requires major >= minor, so rotate the major
            // direction a quarter turn when the axes arrive swapped.
            let (major_dir, major, minor) = if semi_1 >= semi_2 {
                (x_dir, semi_1, semi_2)
            } else {
                (unit_axis.cross(&x_dir), semi_2, semi_1)
            };
            Ok(RawCurve::Analytic(
                Curve3::ellipse(p.location, p.axis, major_dir, major, minor)
                    .map_err(|e| geometry_error(id, &e))?,
            ))
        }
        "B_SPLINE_CURVE_WITH_KNOTS" => Ok(RawCurve::Nurbs(Box::new(resolve_bspline_curve(
            file, rec, id, scale,
        )?))),
        other => Err(unsupported(id, format!("curve type {other}"))),
    }
}

/// Expand STEP's `(knots, multiplicities)` pair into a flat knot sequence.
fn expand_knots(knots: &[f64], multiplicities: &[i64], entity: u64) -> MapResult<Vec<f64>> {
    if knots.len() != multiplicities.len() {
        return Err(invalid(
            entity,
            format!(
                "knot list ({}) and multiplicity list ({}) lengths differ",
                knots.len(),
                multiplicities.len()
            ),
        ));
    }
    let mut flat = Vec::new();
    for (&knot, &mult) in knots.iter().zip(multiplicities) {
        if mult < 1 {
            return Err(invalid(entity, format!("knot multiplicity {mult} < 1")));
        }
        flat.extend(std::iter::repeat_n(knot, mult as usize));
    }
    Ok(flat)
}

fn knot_vector(
    degree: i64,
    knots: &[f64],
    multiplicities: &[i64],
    entity: u64,
) -> MapResult<KnotVector> {
    if degree < 1 {
        return Err(invalid(entity, format!("B-spline degree {degree} < 1")));
    }
    let flat = expand_knots(knots, multiplicities, entity)?;
    KnotVector::new(degree as usize, flat).map_err(|e| nurbs_error(entity, &e))
}

/// `B_SPLINE_CURVE_WITH_KNOTS(name, degree, control_points, form, closed,
/// self_intersect, multiplicities, knots, knot_spec)`.
fn resolve_bspline_curve(
    file: &StepFile,
    rec: &SimpleRecord,
    id: u64,
    scale: f64,
) -> MapResult<NurbsCurve> {
    let degree = int_attr(rec, 1, id)?;
    let control_points = ref_list(rec, 2, id)?
        .into_iter()
        .map(|p| resolve_point(file, p, id, scale))
        .collect::<MapResult<Vec<_>>>()?;
    let multiplicities = int_list(rec, 6, id)?;
    let knots = real_list(rec, 7, id)?;
    let kv = knot_vector(degree, &knots, &multiplicities, id)?;
    NurbsCurve::bspline(control_points, kv).map_err(|e| nurbs_error(id, &e))
}

/// `B_SPLINE_SURFACE_WITH_KNOTS(name, u_degree, v_degree, control_grid,
/// form, u_closed, v_closed, self_intersect, u_mults, v_mults, u_knots,
/// v_knots, knot_spec)`. The control grid's outer index runs over `u`.
fn resolve_bspline_surface(
    file: &StepFile,
    rec: &SimpleRecord,
    id: u64,
    scale: f64,
) -> MapResult<NurbsSurface> {
    let u_degree = int_attr(rec, 1, id)?;
    let v_degree = int_attr(rec, 2, id)?;
    let rows = list_attr(rec, 3, id)?;
    let mut grid = Vec::with_capacity(rows.len());
    for row in rows {
        let cells = row
            .as_list()
            .ok_or_else(|| invalid(id, "control grid row is not a list"))?;
        let points = cells
            .iter()
            .map(|cell| {
                let point = cell
                    .as_ref_id()
                    .ok_or_else(|| invalid(id, "control grid cell is not a reference"))?;
                resolve_point(file, point, id, scale)
            })
            .collect::<MapResult<Vec<_>>>()?;
        grid.push(points);
    }
    let u_mults = int_list(rec, 8, id)?;
    let v_mults = int_list(rec, 9, id)?;
    let u_knots = real_list(rec, 10, id)?;
    let v_knots = real_list(rec, 11, id)?;
    let kv_u = knot_vector(u_degree, &u_knots, &u_mults, id)?;
    let kv_v = knot_vector(v_degree, &v_knots, &v_mults, id)?;
    NurbsSurface::bspline(grid, kv_u, kv_v).map_err(|e| nurbs_error(id, &e))
}

// ---------------------------------------------------------------------
// Edge trimming
// ---------------------------------------------------------------------

/// An analytic edge curve oriented start → end with `t_start < t_end`.
#[derive(Debug)]
struct TrimmedCurve {
    curve: Curve3,
    t_start: f64,
    t_end: f64,
}

/// Reverse an analytic curve's parameterization (each edge gets its own
/// curve instance, so flipping in place is safe). The variants stay
/// well-formed: units stay unit, orthogonality is preserved.
fn reverse_curve(curve: &Curve3) -> Curve3 {
    match curve {
        Curve3::Line { origin, dir } => Curve3::Line {
            origin: *origin,
            dir: -dir,
        },
        Curve3::Circle {
            center,
            axis,
            radius,
        } => Curve3::Circle {
            center: *center,
            axis: -axis,
            radius: *radius,
        },
        Curve3::Ellipse {
            center,
            axis,
            major_dir,
            major_radius,
            minor_radius,
        } => Curve3::Ellipse {
            center: *center,
            axis: -axis,
            major_dir: *major_dir,
            major_radius: *major_radius,
            minor_radius: *minor_radius,
        },
        // Not produced by the reader (B-splines parse as NURBS), but the
        // reversal is well-defined: walk the vertices backwards.
        Curve3::Polyline { points, closed } => Curve3::Polyline {
            points: points.iter().rev().copied().collect(),
            closed: *closed,
        },
    }
}

/// Angle parameter of `p` on a conic, in the curve's own frame, wrapped to
/// `[0, 2π)`. `None` for lines.
fn conic_angle(curve: &Curve3, p: &Point3) -> Option<f64> {
    match curve {
        Curve3::Line { .. } | Curve3::Polyline { .. } => None,
        Curve3::Circle { center, axis, .. } => {
            let (u, v) = plane_basis(axis);
            let r = p - center;
            Some(r.dot(&v).atan2(r.dot(&u)).rem_euclid(TAU))
        }
        Curve3::Ellipse {
            center,
            axis,
            major_dir,
            major_radius,
            minor_radius,
        } => {
            let minor_dir = axis.cross(major_dir);
            let r = p - center;
            let x = r.dot(major_dir) / major_radius;
            let y = r.dot(&minor_dir) / minor_radius;
            Some(y.atan2(x).rem_euclid(TAU))
        }
    }
}

/// Trim an analytic curve to an edge's vertices: orient it along the edge
/// (STEP `same_sense` false means the edge opposes the curve direction)
/// and recover the parameter range, always with `t_start < t_end`.
fn trim_curve(
    curve: &Curve3,
    same_sense: bool,
    start: Point3,
    end: Point3,
    closed: bool,
    entity: u64,
) -> MapResult<TrimmedCurve> {
    let oriented = if same_sense {
        curve.clone()
    } else {
        reverse_curve(curve)
    };
    let (t_start, t_end) = match &oriented {
        Curve3::Line { origin, dir } => {
            if closed {
                return Err(invalid(entity, "a line cannot carry a closed edge"));
            }
            let t0 = (start - origin).dot(dir);
            let t1 = (end - origin).dot(dir);
            // NaN-safe: only a strictly positive advance is acceptable.
            if (t1 - t0).partial_cmp(&SYSTEM_RESOLUTION) != Some(std::cmp::Ordering::Greater) {
                return Err(invalid(
                    entity,
                    "edge endpoints do not advance along its line (check same_sense)",
                ));
            }
            (t0, t1)
        }
        conic => {
            let t0 = conic_angle(conic, &start).expect("conic");
            if closed {
                (t0, t0 + TAU)
            } else {
                let sweep = (conic_angle(conic, &end).expect("conic") - t0).rem_euclid(TAU);
                if sweep <= 1e-9 {
                    return Err(invalid(entity, "conic edge sweeps zero angle"));
                }
                (t0, t0 + sweep)
            }
        }
    };
    let trimmed = TrimmedCurve {
        curve: oriented,
        t_start,
        t_end,
    };
    verify_trim(&trimmed, start, end, entity)?;
    Ok(trimmed)
}

/// Verify the trimmed curve interpolates the edge's vertex points; catches
/// vertices off their curve (wrong radius, off-plane, bad `same_sense`).
fn verify_trim(trimmed: &TrimmedCurve, start: Point3, end: Point3, entity: u64) -> MapResult<()> {
    let scale = start.coords.norm().max(end.coords.norm());
    let tol = TRIM_TOL_REL * (1.0 + scale);
    let at_start = trimmed.curve.point(trimmed.t_start);
    let at_end = trimmed.curve.point(trimmed.t_end);
    if (at_start - start).norm() > tol || (at_end - end).norm() > tol {
        return Err(invalid(
            entity,
            "edge geometry does not pass through the edge's vertex points",
        ));
    }
    Ok(())
}

// ---------------------------------------------------------------------
// Exact B-Rep mapping
// ---------------------------------------------------------------------

/// Everything created in the stores for one solid, for rollback when the
/// solid falls back or fails.
#[derive(Default)]
struct Created {
    body: Option<EntityId<Body>>,
    shell: Option<EntityId<Shell>>,
    faces: Vec<EntityId<Face>>,
    loops: Vec<EntityId<Loop>>,
    fins: Vec<EntityId<Fin>>,
    edges: Vec<EntityId<Edge>>,
    vertices: Vec<EntityId<Vertex>>,
    curves: Vec<EntityId<Curve3>>,
    surfaces: Vec<EntityId<Surface3>>,
}

/// Remove every entity in `created`, leaving the stores as they were
/// before the solid was mapped.
fn rollback(store: &mut TopologyStore, geo: &mut GeometryStore, created: &Created) {
    for &id in &created.fins {
        store.fins.remove(id);
    }
    for &id in &created.loops {
        store.loops.remove(id);
    }
    for &id in &created.faces {
        store.faces.remove(id);
    }
    for &id in &created.edges {
        store.edges.remove(id);
    }
    for &id in &created.vertices {
        store.vertices.remove(id);
    }
    if let Some(id) = created.shell {
        store.shells.remove(id);
    }
    if let Some(id) = created.body {
        store.bodies.remove(id);
    }
    for &id in &created.curves {
        geo.curves.remove(id);
    }
    for &id in &created.surfaces {
        geo.surfaces.remove(id);
    }
}

struct SolidBuilder<'a> {
    file: &'a StepFile,
    store: &'a mut TopologyStore,
    geo: &'a mut GeometryStore,
    /// Length-unit factor (mm per file unit) applied to all geometry.
    scale: f64,
    /// Plane-angle factor (rad per file angle unit) applied to angle measures.
    angle_scale: f64,
    created: Created,
    /// `VERTEX_POINT` #id → mapped vertex (shared between edges).
    vertices: HashMap<u64, EntityId<Vertex>>,
    /// `EDGE_CURVE` #id → mapped edge (shared between faces, so mated
    /// fins arise naturally when both faces reference the same edge).
    edges: HashMap<u64, EntityId<Edge>>,
}

impl SolidBuilder<'_> {
    fn build(&mut self, msb_id: u64, shell_ref: u64) -> MapResult<EntityId<Body>> {
        let shell_rec = typed_record(self.file, shell_ref, "CLOSED_SHELL", msb_id)?;
        let face_refs = ref_list(shell_rec, 1, shell_ref)?;
        if face_refs.is_empty() {
            return Err(invalid(shell_ref, "CLOSED_SHELL has no faces"));
        }

        let body = self.store.create_body(BodyType::Solid);
        self.created.body = Some(body);
        let shell = self
            .store
            .create_shell(body, true, ShellOrientation::Outward);
        self.created.shell = Some(shell);

        for face_ref in face_refs {
            self.map_face(shell, face_ref, shell_ref)?;
        }

        // STEP carries no genus; recover it from the Euler-Poincaré formula
        // so `check` validates the imported topology's own consistency
        // (an odd or negative implied genus still fails the formula).
        let counts = self.store.euler_counts(body);
        let euler = counts.vertices as i64 - counts.edges as i64 + counts.faces as i64
            - counts.rings as i64;
        let genus_x2 = 2 * counts.shells as i64 - euler;
        if genus_x2 >= 0 && genus_x2 % 2 == 0 {
            self.store
                .shells
                .get_mut(shell)
                .expect("just created")
                .genus = (genus_x2 / 2) as u32;
        }
        Ok(body)
    }

    fn map_face(&mut self, shell: EntityId<Shell>, face_ref: u64, referrer: u64) -> MapResult<()> {
        let rec = typed_record(self.file, face_ref, "ADVANCED_FACE", referrer)?;
        let bounds = ref_list(rec, 1, face_ref)?;
        let surface_ref = ref_attr(rec, 2, face_ref)?;
        let same_sense = bool_attr(rec, 3, face_ref)?;

        // Surface first: an unmappable surface (NURBS) is the more
        // fundamental finding than any bound-level problem.
        let surface = match resolve_surface(
            self.file,
            surface_ref,
            face_ref,
            self.scale,
            self.angle_scale,
        )? {
            RawSurface::Analytic(surface) => surface,
            RawSurface::Nurbs(_) => {
                return Err(unsupported(
                    surface_ref,
                    "exact NURBS surface import (geometry store has no NURBS variant); \
                     falling back to tessellation",
                ));
            }
        };
        if bounds.is_empty() {
            return Err(invalid(face_ref, "ADVANCED_FACE has no bounds"));
        }

        // At most one FACE_OUTER_BOUND; without one, the first bound plays
        // the outer role (AP203 permits plain FACE_BOUNDs only).
        let mut outer_index = None;
        for (i, &bound_ref) in bounds.iter().enumerate() {
            let inst = instance(self.file, bound_ref, face_ref)?;
            if inst.entity.part("FACE_OUTER_BOUND").is_some() {
                if outer_index.is_some() {
                    return Err(invalid(face_ref, "face has multiple FACE_OUTER_BOUNDs"));
                }
                outer_index = Some(i);
            }
        }
        let outer_index = outer_index.unwrap_or(0);

        let sense = if same_sense {
            FaceSense::Positive
        } else {
            FaceSense::Negative
        };
        let face = self.store.create_face(shell, sense);
        self.created.faces.push(face);
        let surface_id = self.geo.add_surface(surface);
        self.created.surfaces.push(surface_id);
        self.store
            .faces
            .get_mut(face)
            .expect("just created")
            .surface = Some(surface_id);

        for (i, &bound_ref) in bounds.iter().enumerate() {
            let (loop_ref, orientation) = self.resolve_bound(bound_ref, face_ref)?;
            let mut loop_edges = self.map_loop(loop_ref, bound_ref)?;
            if !orientation {
                loop_edges.reverse();
                for (_, sense) in &mut loop_edges {
                    *sense = sense.opposite();
                }
            }
            let loop_type = if i == outer_index {
                LoopType::Outer
            } else {
                LoopType::Inner
            };
            let loop_id = self.store.create_loop(face, loop_type, &loop_edges);
            self.created.loops.push(loop_id);
            let fins = self
                .store
                .loop_(loop_id)
                .expect("just created")
                .fins
                .clone();
            self.created.fins.extend(fins);
        }
        Ok(())
    }

    /// `FACE_BOUND` / `FACE_OUTER_BOUND` → (loop reference, orientation).
    fn resolve_bound(&self, bound_ref: u64, referrer: u64) -> MapResult<(u64, bool)> {
        let inst = instance(self.file, bound_ref, referrer)?;
        let rec = inst
            .entity
            .part("FACE_OUTER_BOUND")
            .or_else(|| inst.entity.part("FACE_BOUND"))
            .ok_or_else(|| {
                invalid(
                    bound_ref,
                    format!("expected FACE_BOUND, found {}", type_names(inst)),
                )
            })?;
        Ok((ref_attr(rec, 1, bound_ref)?, bool_attr(rec, 2, bound_ref)?))
    }

    fn map_loop(
        &mut self,
        loop_ref: u64,
        referrer: u64,
    ) -> MapResult<Vec<(EntityId<Edge>, FinSense)>> {
        let inst = instance(self.file, loop_ref, referrer)?;
        if inst.entity.part("VERTEX_LOOP").is_some() {
            return Err(unsupported(loop_ref, "VERTEX_LOOP bounds"));
        }
        let rec = typed_record(self.file, loop_ref, "EDGE_LOOP", referrer)?;
        let oriented_edges = ref_list(rec, 1, loop_ref)?;
        if oriented_edges.is_empty() {
            return Err(invalid(loop_ref, "EDGE_LOOP has no edges"));
        }
        let mut edges = Vec::with_capacity(oriented_edges.len());
        for oe_ref in oriented_edges {
            let oe = typed_record(self.file, oe_ref, "ORIENTED_EDGE", loop_ref)?;
            let edge_ref = ref_attr(oe, 3, oe_ref)?;
            let orientation = bool_attr(oe, 4, oe_ref)?;
            let edge = self.map_edge(edge_ref, oe_ref)?;
            let sense = if orientation {
                FinSense::Forward
            } else {
                FinSense::Reversed
            };
            edges.push((edge, sense));
        }
        Ok(edges)
    }

    fn map_edge(&mut self, edge_ref: u64, referrer: u64) -> MapResult<EntityId<Edge>> {
        if let Some(&edge) = self.edges.get(&edge_ref) {
            return Ok(edge);
        }
        let rec = typed_record(self.file, edge_ref, "EDGE_CURVE", referrer)?;
        let start_ref = ref_attr(rec, 1, edge_ref)?;
        let end_ref = ref_attr(rec, 2, edge_ref)?;
        let geometry_ref = ref_attr(rec, 3, edge_ref)?;
        let same_sense = bool_attr(rec, 4, edge_ref)?;
        let closed = start_ref == end_ref;

        let v_start = self.map_vertex(start_ref, edge_ref)?;
        let v_end = self.map_vertex(end_ref, edge_ref)?;
        let start = self.store.vertex(v_start).expect("just created").point;
        let end = self.store.vertex(v_end).expect("just created").point;

        let curve = match resolve_curve(self.file, geometry_ref, edge_ref, self.scale)? {
            RawCurve::Analytic(curve) => curve,
            RawCurve::Nurbs(_) => {
                return Err(unsupported(
                    geometry_ref,
                    "exact NURBS curve import (geometry store has no NURBS variant); \
                     falling back to tessellation",
                ));
            }
        };
        let trimmed = trim_curve(&curve, same_sense, start, end, closed, edge_ref)?;

        let curve_id = self.geo.add_curve(trimmed.curve);
        self.created.curves.push(curve_id);
        let edge = self.store.create_edge_with_curve(
            v_start,
            v_end,
            SYSTEM_RESOLUTION,
            curve_id,
            trimmed.t_start,
            trimmed.t_end,
        );
        self.created.edges.push(edge);
        self.edges.insert(edge_ref, edge);
        Ok(edge)
    }

    fn map_vertex(&mut self, vertex_ref: u64, referrer: u64) -> MapResult<EntityId<Vertex>> {
        if let Some(&vertex) = self.vertices.get(&vertex_ref) {
            return Ok(vertex);
        }
        let rec = typed_record(self.file, vertex_ref, "VERTEX_POINT", referrer)?;
        let point = resolve_point(
            self.file,
            ref_attr(rec, 1, vertex_ref)?,
            vertex_ref,
            self.scale,
        )?;
        let vertex = self.store.create_vertex(point, SYSTEM_RESOLUTION);
        self.created.vertices.push(vertex);
        self.vertices.insert(vertex_ref, vertex);
        Ok(vertex)
    }
}

// ---------------------------------------------------------------------
// Mesh fallback
// ---------------------------------------------------------------------

/// Segment count for sweeping an angular range at the configured step
/// (at least 3, so full circles always produce a real polygon).
fn angular_segments(sweep: f64, options: &TessellationOptions) -> usize {
    ((sweep.abs() / options.angular_step).ceil() as usize).max(3)
}

/// Minimum grid resolution per parameter direction of a NURBS patch.
const NURBS_MIN_SEGMENTS: usize = 8;

/// Grid segments for one NURBS parameter direction: enough to resolve
/// every span of the control polygon.
fn nurbs_segments(control_count: usize) -> usize {
    (4 * control_count.saturating_sub(1)).max(NURBS_MIN_SEGMENTS)
}

struct FallbackMesher<'a> {
    file: &'a StepFile,
    options: &'a TessellationOptions,
    /// Length-unit factor (mm per file unit) applied to all geometry.
    scale: f64,
    /// Plane-angle factor (rad per file angle unit) applied to angle measures.
    angle_scale: f64,
    diagnostics: &'a mut Vec<Diagnostic>,
    /// `EDGE_CURVE` #id → its polyline from start vertex to end vertex.
    /// Shared between adjacent faces so junctions weld watertight.
    polylines: HashMap<u64, Vec<Point3>>,
}

impl FallbackMesher<'_> {
    /// Tessellate one solid straight from the STEP graph. `None` (with
    /// diagnostics) when any face fails or the welded result is not a
    /// closed manifold.
    fn mesh_solid(&mut self, msb_id: u64, shell_ref: u64) -> Option<TriangleMesh> {
        let shell_rec = match typed_record(self.file, shell_ref, "CLOSED_SHELL", msb_id) {
            Ok(rec) => rec,
            Err(e) => {
                self.diagnostics.push(e.diagnostic());
                return None;
            }
        };
        let face_refs = match ref_list(shell_rec, 1, shell_ref) {
            Ok(refs) => refs,
            Err(e) => {
                self.diagnostics.push(e.diagnostic());
                return None;
            }
        };

        let mut mesh = TriangleMesh::new();
        let mut ok = true;
        for face_ref in face_refs {
            if let Err(e) = self.mesh_face(&mut mesh, face_ref, shell_ref) {
                self.diagnostics.push(e.diagnostic());
                ok = false;
            }
        }
        if !ok {
            return None;
        }

        let epsilon = mesh
            .bounding_box()
            .map(|b| (b.max - b.min).norm() * 1e-7)
            .unwrap_or(0.0);
        let welded = mesh.weld(epsilon);
        if !welded.is_closed_manifold() {
            self.diagnostics.push(Diagnostic {
                entity: Some(msb_id),
                severity: Severity::Error,
                message: "fallback tessellation is not a closed manifold".to_string(),
            });
            return None;
        }
        Some(welded)
    }

    fn mesh_face(
        &mut self,
        mesh: &mut TriangleMesh,
        face_ref: u64,
        referrer: u64,
    ) -> MapResult<()> {
        let rec = typed_record(self.file, face_ref, "ADVANCED_FACE", referrer)?;
        let bounds = ref_list(rec, 1, face_ref)?;
        let surface_ref = ref_attr(rec, 2, face_ref)?;
        let same_sense = bool_attr(rec, 3, face_ref)?;

        match resolve_surface(
            self.file,
            surface_ref,
            face_ref,
            self.scale,
            self.angle_scale,
        )? {
            RawSurface::Analytic(surface @ Surface3::Plane { .. }) => {
                self.mesh_planar_face(mesh, face_ref, &bounds, &surface, same_sense)
            }
            RawSurface::Analytic(surface) => {
                self.mesh_quadric_face(mesh, face_ref, &bounds, &surface, same_sense)
            }
            RawSurface::Nurbs(surface) => {
                self.diagnostics.push(Diagnostic {
                    entity: Some(face_ref),
                    severity: Severity::Info,
                    message: "NURBS face tessellated over its full parameter domain \
                              (trimming bounds ignored)"
                        .to_string(),
                });
                mesh_nurbs_face(mesh, &surface, same_sense);
                Ok(())
            }
        }
    }

    /// Ear-clip a planar face's boundary polygon, bridging in any hole
    /// bounds (of-fc8: every drilled plate has them).
    fn mesh_planar_face(
        &mut self,
        mesh: &mut TriangleMesh,
        face_ref: u64,
        bounds: &[u64],
        surface: &Surface3,
        same_sense: bool,
    ) -> MapResult<()> {
        let mut rings_3d = Vec::with_capacity(bounds.len());
        for &bound_ref in bounds {
            rings_3d.push(self.bound_polygon(bound_ref, face_ref)?);
        }
        if rings_3d.iter().all(|r| r.len() < 3) {
            return Err(invalid(
                face_ref,
                "face boundary samples to fewer than 3 points",
            ));
        }
        let Surface3::Plane { normal, .. } = surface else {
            unreachable!("caller dispatched on Plane");
        };
        // The face normal (outward, for a closed shell) is the surface
        // normal exactly when same_sense holds. Projecting onto a basis
        // with e_u × e_v = n makes ear_clip's counterclockwise triples
        // face along +n — outward.
        let n = if same_sense { *normal } else { -normal };
        let (e_u, e_v) = plane_basis(&n);
        let origin = rings_3d[0][0];
        let project = |p: &Point3| {
            let d = p - origin;
            (d.dot(&e_u), d.dot(&e_v))
        };
        let mut rings: Vec<Vec<(f64, f64)>> = rings_3d
            .iter()
            .map(|ring| ring.iter().map(project).collect())
            .collect();
        // `ear_clip_rings` wants the outer loop first. FACE_OUTER_BOUND is
        // supposed to say which that is, but exporters mislabel it and the
        // attribute is optional in FACE_BOUND-only files; the widest ring is
        // the outer one either way. Swap by area rather than trusting the tag.
        let outer = (0..rings.len())
            .max_by(|&a, &b| {
                signed_area2(&rings[a])
                    .abs()
                    .total_cmp(&signed_area2(&rings[b]).abs())
            })
            .expect("bounds is non-empty");
        rings.swap(0, outer);
        rings_3d.swap(0, outer);

        let base = mesh.positions.len();
        for point in rings_3d.iter().flatten() {
            mesh.positions.push(*point);
            mesh.normals.push(n);
        }
        let tris = ear_clip_rings(&rings).ok_or_else(|| {
            invalid(
                face_ref,
                "face has a hole bound that cannot be bridged to its outer bound",
            )
        })?;
        for [a, b, c] in tris {
            mesh.indices.push([base + a, base + b, base + c]);
        }
        Ok(())
    }

    /// Grid a quadric face over its parameter rectangle: full `u` period
    /// anchored at the boundary, `v` range recovered from the boundary
    /// (like the B-Rep tessellator's MVP, trimmed quadrics are assumed to
    /// cover the full angular range).
    fn mesh_quadric_face(
        &mut self,
        mesh: &mut TriangleMesh,
        face_ref: u64,
        bounds: &[u64],
        surface: &Surface3,
        same_sense: bool,
    ) -> MapResult<()> {
        let mut boundary = Vec::new();
        for &bound_ref in bounds {
            boundary.extend(self.bound_polygon(bound_ref, face_ref)?);
        }

        // The u anchor comes from the first non-singular boundary sample,
        // so grid columns land on the same 3D points as adjacent faces'
        // boundary polylines and weld watertight.
        let mut u_anchor = 0.0;
        let mut v_lo = f64::INFINITY;
        let mut v_hi = f64::NEG_INFINITY;
        let mut anchored = false;
        for p in &boundary {
            let projected = surface.project_point(p);
            if !anchored && !surface.is_singular(projected.u, projected.v) {
                u_anchor = projected.u;
                anchored = true;
            }
            v_lo = v_lo.min(projected.v);
            v_hi = v_hi.max(projected.v);
        }

        let (v_lo, v_hi, wrap_v, n_v) = match surface {
            Surface3::Cylinder { .. } | Surface3::Cone { .. } => {
                if !(v_lo.is_finite() && v_hi.is_finite() && v_hi > v_lo) {
                    return Err(invalid(
                        face_ref,
                        "face boundary does not span a v range on its surface",
                    ));
                }
                (v_lo, v_hi, false, 1)
            }
            Surface3::Sphere { .. } => {
                let (lo, hi) = surface.domain_v();
                (lo, hi, false, angular_segments(hi - lo, self.options))
            }
            Surface3::Torus { .. } => {
                let period = surface.period_v().expect("torus is v-periodic");
                (0.0, period, true, angular_segments(period, self.options))
            }
            Surface3::Plane { .. } => unreachable!("caller dispatched planes elsewhere"),
            // `RawSurface::Nurbs` never reaches the analytic path.
            Surface3::Nurbs(_) => unreachable!("caller dispatched NURBS elsewhere"),
        };
        grid_quadric(
            mesh,
            surface,
            GridSpec {
                u_anchor,
                v_lo,
                v_hi,
                wrap_v,
                n_v,
                outward: same_sense,
            },
            self.options,
        );
        Ok(())
    }

    /// A face bound as one closed 3D polygon (no repeated closing point),
    /// oriented per the bound's and oriented-edges' senses.
    fn bound_polygon(&mut self, bound_ref: u64, referrer: u64) -> MapResult<Vec<Point3>> {
        let inst = instance(self.file, bound_ref, referrer)?;
        let rec = inst
            .entity
            .part("FACE_OUTER_BOUND")
            .or_else(|| inst.entity.part("FACE_BOUND"))
            .ok_or_else(|| {
                invalid(
                    bound_ref,
                    format!("expected FACE_BOUND, found {}", type_names(inst)),
                )
            })?;
        let loop_ref = ref_attr(rec, 1, bound_ref)?;
        let bound_orientation = bool_attr(rec, 2, bound_ref)?;

        let loop_rec = typed_record(self.file, loop_ref, "EDGE_LOOP", bound_ref)?;
        let mut polygon = Vec::new();
        for oe_ref in ref_list(loop_rec, 1, loop_ref)? {
            let oe = typed_record(self.file, oe_ref, "ORIENTED_EDGE", loop_ref)?;
            let edge_ref = ref_attr(oe, 3, oe_ref)?;
            let orientation = bool_attr(oe, 4, oe_ref)?;
            let polyline = self.polyline(edge_ref, oe_ref)?;
            // Each fin contributes its open run; the next fin supplies the
            // shared junction point.
            if orientation {
                polygon.extend(&polyline[..polyline.len() - 1]);
            } else {
                polygon.extend(polyline[1..].iter().rev());
            }
        }
        if !bound_orientation {
            polygon.reverse();
        }
        Ok(polygon)
    }

    /// Discretize an `EDGE_CURVE` once, start vertex → end vertex, with
    /// exact vertex endpoints (so faces sharing the edge weld exactly).
    fn polyline(&mut self, edge_ref: u64, referrer: u64) -> MapResult<Vec<Point3>> {
        if let Some(points) = self.polylines.get(&edge_ref) {
            return Ok(points.clone());
        }
        let rec = typed_record(self.file, edge_ref, "EDGE_CURVE", referrer)?;
        let start_ref = ref_attr(rec, 1, edge_ref)?;
        let end_ref = ref_attr(rec, 2, edge_ref)?;
        let geometry_ref = ref_attr(rec, 3, edge_ref)?;
        let same_sense = bool_attr(rec, 4, edge_ref)?;
        let closed = start_ref == end_ref;

        let vertex_point = |vertex_ref: u64| -> MapResult<Point3> {
            let vrec = typed_record(self.file, vertex_ref, "VERTEX_POINT", edge_ref)?;
            resolve_point(
                self.file,
                ref_attr(vrec, 1, vertex_ref)?,
                vertex_ref,
                self.scale,
            )
        };
        let start = vertex_point(start_ref)?;
        let end = vertex_point(end_ref)?;

        let points = match resolve_curve(self.file, geometry_ref, edge_ref, self.scale)? {
            RawCurve::Analytic(curve) => {
                let trimmed = trim_curve(&curve, same_sense, start, end, closed, edge_ref)?;
                let segments = match trimmed.curve {
                    Curve3::Line { .. } => 1,
                    _ => angular_segments(trimmed.t_end - trimmed.t_start, self.options),
                };
                let mut points = Vec::with_capacity(segments + 1);
                points.push(start);
                for k in 1..segments {
                    let t = trimmed.t_start
                        + (trimmed.t_end - trimmed.t_start) * k as f64 / segments as f64;
                    points.push(trimmed.curve.point(t));
                }
                points.push(end);
                points
            }
            RawCurve::Nurbs(curve) => {
                let (t0, t1) = curve.knot_vector().domain();
                let segments = nurbs_segments(curve.knot_vector().control_count());
                let mut points: Vec<Point3> = (0..=segments)
                    .map(|k| curve.point(t0 + (t1 - t0) * k as f64 / segments as f64))
                    .collect();
                if !same_sense {
                    points.reverse();
                }
                let scale = start.coords.norm().max(end.coords.norm());
                let tol = TRIM_TOL_REL * (1.0 + scale);
                if (points[0] - start).norm() > tol || (points[points.len() - 1] - end).norm() > tol
                {
                    self.diagnostics.push(Diagnostic {
                        entity: Some(edge_ref),
                        severity: Severity::Warning,
                        message: "B-spline edge curve endpoints do not match the edge's \
                                  vertex points; snapping"
                            .to_string(),
                    });
                }
                points[0] = start;
                let last = points.len() - 1;
                points[last] = end;
                points
            }
        };
        self.polylines.insert(edge_ref, points.clone());
        Ok(points)
    }
}

/// Parameters for [`grid_quadric`].
struct GridSpec {
    u_anchor: f64,
    v_lo: f64,
    v_hi: f64,
    wrap_v: bool,
    n_v: usize,
    /// Whether the face normal follows the surface normal (`du × dv`);
    /// false flips windings and normals (STEP `same_sense = .F.`).
    outward: bool,
}

/// Tessellate a quadric face over its parameter rectangle: `u` over the
/// full period from `u_anchor` (wrapped by index), `v` over `[v_lo, v_hi]`
/// with `n_v` segments (wrapped if `wrap_v`). Singular rows (sphere poles,
/// cone apex) collapse to a single vertex.
fn grid_quadric(
    mesh: &mut TriangleMesh,
    surface: &Surface3,
    spec: GridSpec,
    options: &TessellationOptions,
) {
    let period = surface.period_u().expect("quadric surfaces are u-periodic");
    let n_u = angular_segments(period, options);
    let row_count = if spec.wrap_v { spec.n_v } else { spec.n_v + 1 };
    let flip = if spec.outward { 1.0 } else { -1.0 };

    let mut rows: Vec<Vec<usize>> = Vec::with_capacity(row_count);
    for j in 0..row_count {
        let v = if !spec.wrap_v && j == spec.n_v {
            spec.v_hi // exact endpoint, no accumulation error
        } else {
            spec.v_lo + (spec.v_hi - spec.v_lo) * j as f64 / spec.n_v as f64
        };
        let singular = surface.is_singular(spec.u_anchor, v);
        let columns = if singular { 1 } else { n_u };
        let mut row = Vec::with_capacity(columns);
        for i in 0..columns {
            let u = spec.u_anchor + period * i as f64 / n_u as f64;
            row.push(mesh.positions.len());
            mesh.positions.push(surface.point(u, v));
            let normal = surface.normal(u, v).unwrap_or_else(|| {
                // No limit normal (cone apex): nudge v toward the interior
                // for a usable shading normal.
                let mid = (spec.v_lo + spec.v_hi) / 2.0;
                surface
                    .normal(u, v + (mid - v) * 1e-6)
                    .unwrap_or_else(Vector3::zeros)
            });
            mesh.normals.push(normal * flip);
        }
        rows.push(row);
    }

    let at = |j: usize, i: usize| -> usize {
        let row = &rows[j % row_count];
        row[i % row.len()]
    };
    for j in 0..spec.n_v {
        for i in 0..n_u {
            // Quad corners in (u, v): a --u--> b, then +v to c/d. The
            // [a, b, c] winding follows du × dv, the surface normal.
            let (a, b) = (at(j, i), at(j, i + 1));
            let (d, c) = (at(j + 1, i), at(j + 1, i + 1));
            let quads = if spec.outward {
                [[a, b, c], [a, c, d]]
            } else {
                [[a, c, b], [a, d, c]]
            };
            for tri in quads {
                if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                    mesh.indices.push(tri);
                }
            }
        }
    }
}

/// Grid a NURBS patch over its full knot domain (trimming ignored — the
/// fallback treats every patch as untrimmed).
fn mesh_nurbs_face(mesh: &mut TriangleMesh, surface: &NurbsSurface, outward: bool) {
    let (u0, u1) = surface.knot_vector_u().domain();
    let (v0, v1) = surface.knot_vector_v().domain();
    let n_u = nurbs_segments(surface.knot_vector_u().control_count());
    let n_v = nurbs_segments(surface.knot_vector_v().control_count());
    let flip = if outward { 1.0 } else { -1.0 };

    let base = mesh.positions.len();
    for j in 0..=n_v {
        let v = v0 + (v1 - v0) * j as f64 / n_v as f64;
        for i in 0..=n_u {
            let u = u0 + (u1 - u0) * i as f64 / n_u as f64;
            mesh.positions.push(surface.point(u, v));
            let normal = surface.normal(u, v).unwrap_or_else(Vector3::zeros);
            mesh.normals.push(normal * flip);
        }
    }
    let at = |j: usize, i: usize| base + j * (n_u + 1) + i;
    for j in 0..n_v {
        for i in 0..n_u {
            let (a, b) = (at(j, i), at(j, i + 1));
            let (d, c) = (at(j + 1, i), at(j + 1, i + 1));
            let quads = if outward {
                [[a, b, c], [a, c, d]]
            } else {
                [[a, c, b], [a, d, c]]
            };
            for tri in quads {
                mesh.indices.push(tri);
            }
        }
    }
}

// ---------------------------------------------------------------------
// Top-level orchestration
// ---------------------------------------------------------------------

fn map_file(
    file: &StepFile,
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    options: &StepReadOptions,
) -> StepImport {
    let mut diagnostics = Vec::new();
    let length_scale = resolve_length_scale(file, &mut diagnostics);
    let angle_scale = resolve_angle_scale(file, &mut diagnostics);
    let mut solids = Vec::new();
    for inst in &file.data {
        let Some(rec) = inst.entity.part("MANIFOLD_SOLID_BREP") else {
            continue;
        };
        let name = name_attr(rec);
        let outcome = map_solid(
            file,
            store,
            geo,
            options,
            length_scale,
            angle_scale,
            inst,
            &mut diagnostics,
        );
        solids.push(ImportedSolid {
            step_id: inst.id,
            name,
            outcome,
        });
    }
    if solids.is_empty() {
        diagnostics.push(Diagnostic {
            entity: None,
            severity: Severity::Warning,
            message: "no MANIFOLD_SOLID_BREP instances in the file".to_string(),
        });
    }
    StepImport {
        solids,
        diagnostics,
        length_scale,
        angle_scale,
    }
}

#[allow(clippy::too_many_arguments)]
fn map_solid(
    file: &StepFile,
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    options: &StepReadOptions,
    scale: f64,
    angle_scale: f64,
    inst: &Instance,
    diagnostics: &mut Vec<Diagnostic>,
) -> SolidOutcome {
    let msb_id = inst.id;
    let rec = inst
        .entity
        .part("MANIFOLD_SOLID_BREP")
        .expect("caller selected MANIFOLD_SOLID_BREP instances");
    let shell_ref = match ref_attr(rec, 1, msb_id) {
        Ok(shell_ref) => shell_ref,
        Err(e) => {
            diagnostics.push(e.diagnostic());
            return SolidOutcome::Failed;
        }
    };

    // Exact path first.
    let mut builder = SolidBuilder {
        file,
        store,
        geo,
        scale,
        angle_scale,
        created: Created::default(),
        vertices: HashMap::new(),
        edges: HashMap::new(),
    };
    let built = builder.build(msb_id, shell_ref);
    let created = builder.created;
    match built {
        Ok(body) => {
            let failures = store.check(body);
            if failures.is_empty() {
                return SolidOutcome::BRep(body);
            }
            for failure in &failures {
                diagnostics.push(Diagnostic {
                    entity: Some(msb_id),
                    severity: Severity::Warning,
                    message: format!("mapped body failed validation: {failure}"),
                });
            }
            rollback(store, geo, &created);
        }
        Err(e) => {
            diagnostics.push(e.diagnostic());
            rollback(store, geo, &created);
        }
    }

    // Mesh fallback.
    diagnostics.push(Diagnostic {
        entity: Some(msb_id),
        severity: Severity::Info,
        message: "falling back to tessellated import".to_string(),
    });
    let mut mesher = FallbackMesher {
        file,
        options: &options.tessellation,
        scale,
        angle_scale,
        diagnostics,
        polylines: HashMap::new(),
    };
    match mesher.mesh_solid(msb_id, shell_ref) {
        Some(mesh) => match MeshSdf::new(&mesh) {
            Ok(sdf) => SolidOutcome::Mesh {
                mesh,
                sdf: Box::new(sdf),
            },
            Err(e) => {
                diagnostics.push(Diagnostic {
                    entity: Some(msb_id),
                    severity: Severity::Error,
                    message: format!("fallback mesh rejected as an SDF: {e}"),
                });
                SolidOutcome::Failed
            }
        },
        None => SolidOutcome::Failed,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_brep::tessellate_body;
    use opensolid_frep::primitives::Sdf;
    use std::f64::consts::{FRAC_PI_2, PI};
    use std::fmt::Write as _;

    // ---- fixture builders ----

    /// Wrap DATA-section body text in a minimal Part 21 envelope.
    fn wrap(data: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\n\
             DATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    fn import(src: &str) -> (TopologyStore, GeometryStore, StepImport) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let result = read_step(src, &mut store, &mut geo, &StepReadOptions::default())
            .expect("fixture parses");
        (store, geo, result)
    }

    fn brep_body(outcome: &SolidOutcome) -> EntityId<Body> {
        match outcome {
            SolidOutcome::BRep(body) => *body,
            other => panic!("expected an exact B-Rep import, got {other:?}"),
        }
    }

    fn no_error_diagnostics(report: &StepImport) {
        assert!(
            !report.has_errors(),
            "unexpected error diagnostics: {:?}",
            report.diagnostics
        );
    }

    /// Signed volume via the divergence theorem: positive iff triangles
    /// wind outward consistently.
    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    /// Every mapped edge must run forward (`t_start < t_end`) and its
    /// curve must interpolate its vertex points at the trim parameters.
    fn assert_edges_interpolate(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) {
        for face in store.faces_of_body(body) {
            for edge_id in store.edges_of_face(face) {
                let edge = store.edge(edge_id).unwrap();
                assert!(edge.t_start < edge.t_end, "{edge_id:?}: reversed trim");
                let curve = geo.curve(edge.curve.expect("edge has a curve")).unwrap();
                let start = store.vertex(edge.start_vertex).unwrap().point;
                let end = store.vertex(edge.end_vertex).unwrap().point;
                assert!((curve.point(edge.t_start) - start).norm() < 1e-9);
                assert!((curve.point(edge.t_end) - end).norm() < 1e-9);
            }
        }
    }

    /// AP203 block: 8 vertex points, 12 line edges, 6 planar faces.
    fn block_step(x: f64, y: f64, z: f64) -> String {
        let (hx, hy, hz) = (x / 2.0, y / 2.0, z / 2.0);
        let corners = [
            (-hx, -hy, -hz),
            (hx, -hy, -hz),
            (hx, hy, -hz),
            (-hx, hy, -hz),
            (-hx, -hy, hz),
            (hx, -hy, hz),
            (hx, hy, hz),
            (-hx, hy, hz),
        ];
        const EDGE_PAIRS: [(usize, usize); 12] = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),
        ];
        // Vertex cycles counterclockwise viewed from outside + the outward
        // normal each implies (same tables as opensolid_brep::primitives).
        let face_specs: [([usize; 4], (f64, f64, f64)); 6] = [
            ([0, 3, 2, 1], (0.0, 0.0, -1.0)),
            ([4, 5, 6, 7], (0.0, 0.0, 1.0)),
            ([0, 1, 5, 4], (0.0, -1.0, 0.0)),
            ([1, 2, 6, 5], (1.0, 0.0, 0.0)),
            ([2, 3, 7, 6], (0.0, 1.0, 0.0)),
            ([3, 0, 4, 7], (-1.0, 0.0, 0.0)),
        ];

        let mut b = String::new();
        for (i, &(px, py, pz)) in corners.iter().enumerate() {
            writeln!(
                b,
                "#{} = CARTESIAN_POINT('', ({px:.6}, {py:.6}, {pz:.6}));",
                i + 1
            )
            .unwrap();
        }
        for i in 0..8 {
            writeln!(b, "#{} = VERTEX_POINT('', #{});", 9 + i, i + 1).unwrap();
        }
        for (e, &(a, c)) in EDGE_PAIRS.iter().enumerate() {
            let base = 17 + 4 * e;
            let (dx, dy, dz) = (
                corners[c].0 - corners[a].0,
                corners[c].1 - corners[a].1,
                corners[c].2 - corners[a].2,
            );
            writeln!(b, "#{base} = DIRECTION('', ({dx:.6}, {dy:.6}, {dz:.6}));").unwrap();
            writeln!(b, "#{} = VECTOR('', #{base}, 1.);", base + 1).unwrap();
            writeln!(b, "#{} = LINE('', #{}, #{});", base + 2, a + 1, base + 1).unwrap();
            writeln!(
                b,
                "#{} = EDGE_CURVE('', #{}, #{}, #{}, .T.);",
                base + 3,
                9 + a,
                9 + c,
                base + 2
            )
            .unwrap();
        }
        for (f, &(cycle, (nx, ny, nz))) in face_specs.iter().enumerate() {
            let base = 65 + 10 * f;
            writeln!(b, "#{base} = DIRECTION('', ({nx:.6}, {ny:.6}, {nz:.6}));").unwrap();
            writeln!(
                b,
                "#{} = AXIS2_PLACEMENT_3D('', #{}, #{base}, $);",
                base + 1,
                cycle[0] + 1
            )
            .unwrap();
            writeln!(b, "#{} = PLANE('', #{});", base + 2, base + 1).unwrap();
            for k in 0..4 {
                let (from, to) = (cycle[k], cycle[(k + 1) % 4]);
                let (idx, &(a, _)) = EDGE_PAIRS
                    .iter()
                    .enumerate()
                    .find(|&(_, &(a, c))| (a, c) == (from, to) || (a, c) == (to, from))
                    .expect("face cycles only use listed edges");
                let orientation = if a == from { ".T." } else { ".F." };
                writeln!(
                    b,
                    "#{} = ORIENTED_EDGE('', *, *, #{}, {orientation});",
                    base + 3 + k,
                    17 + 4 * idx + 3
                )
                .unwrap();
            }
            writeln!(
                b,
                "#{} = EDGE_LOOP('', (#{}, #{}, #{}, #{}));",
                base + 7,
                base + 3,
                base + 4,
                base + 5,
                base + 6
            )
            .unwrap();
            writeln!(
                b,
                "#{} = FACE_OUTER_BOUND('', #{}, .T.);",
                base + 8,
                base + 7
            )
            .unwrap();
            writeln!(
                b,
                "#{} = ADVANCED_FACE('', (#{}), #{}, .T.);",
                base + 9,
                base + 8,
                base + 2
            )
            .unwrap();
        }
        writeln!(
            b,
            "#125 = CLOSED_SHELL('', (#74, #84, #94, #104, #114, #124));"
        )
        .unwrap();
        writeln!(b, "#126 = MANIFOLD_SOLID_BREP('block', #125);").unwrap();
        wrap(&b)
    }

    /// AP203 cylinder: two circular caps plus a seam-closed wall.
    fn cylinder_step(r: f64, h: f64) -> String {
        let hz = h / 2.0;
        wrap(&format!(
            "\
#1 = CARTESIAN_POINT('', (0., 0., {lo:.6}));
#2 = CARTESIAN_POINT('', (0., 0., {hi:.6}));
#3 = CARTESIAN_POINT('', ({r:.6}, 0., {lo:.6}));
#4 = CARTESIAN_POINT('', ({r:.6}, 0., {hi:.6}));
#5 = DIRECTION('', (0., 0., 1.));
#6 = DIRECTION('', (1., 0., 0.));
#7 = DIRECTION('', (0., 0., -1.));
#8 = VERTEX_POINT('', #3);
#9 = VERTEX_POINT('', #4);
#10 = AXIS2_PLACEMENT_3D('', #1, #5, #6);
#11 = AXIS2_PLACEMENT_3D('', #2, #5, #6);
#12 = AXIS2_PLACEMENT_3D('', #1, #7, #6);
#13 = CIRCLE('', #10, {r:.6});
#14 = CIRCLE('', #11, {r:.6});
#15 = VECTOR('', #5, 1.);
#16 = LINE('', #3, #15);
#17 = EDGE_CURVE('', #8, #8, #13, .T.);
#18 = EDGE_CURVE('', #9, #9, #14, .T.);
#19 = EDGE_CURVE('', #8, #9, #16, .T.);
#20 = PLANE('', #12);
#21 = PLANE('', #11);
#22 = CYLINDRICAL_SURFACE('', #10, {r:.6});
#23 = ORIENTED_EDGE('', *, *, #17, .F.);
#24 = EDGE_LOOP('', (#23));
#25 = FACE_OUTER_BOUND('', #24, .T.);
#26 = ADVANCED_FACE('', (#25), #20, .T.);
#27 = ORIENTED_EDGE('', *, *, #18, .T.);
#28 = EDGE_LOOP('', (#27));
#29 = FACE_OUTER_BOUND('', #28, .T.);
#30 = ADVANCED_FACE('', (#29), #21, .T.);
#31 = ORIENTED_EDGE('', *, *, #17, .T.);
#32 = ORIENTED_EDGE('', *, *, #19, .T.);
#33 = ORIENTED_EDGE('', *, *, #18, .F.);
#34 = ORIENTED_EDGE('', *, *, #19, .F.);
#35 = EDGE_LOOP('', (#31, #32, #33, #34));
#36 = FACE_OUTER_BOUND('', #35, .T.);
#37 = ADVANCED_FACE('', (#36), #22, .T.);
#38 = CLOSED_SHELL('', (#26, #30, #37));
#39 = MANIFOLD_SOLID_BREP('cyl', #38);",
            lo = -hz,
            hi = hz,
        ))
    }

    /// AP203 sphere DATA body with instance names starting at `base`:
    /// one spherical face closed by a pole-to-pole seam meridian.
    fn sphere_step_at(base: u64, r: f64) -> String {
        let id = |k: u64| base + k;
        format!(
            "\
#{p0} = CARTESIAN_POINT('', (0., 0., 0.));
#{p1} = CARTESIAN_POINT('', (0., 0., {nr:.6}));
#{p2} = CARTESIAN_POINT('', (0., 0., {r:.6}));
#{d0} = DIRECTION('', (0., 0., 1.));
#{d1} = DIRECTION('', (0., -1., 0.));
#{d2} = DIRECTION('', (1., 0., 0.));
#{v0} = VERTEX_POINT('', #{p1});
#{v1} = VERTEX_POINT('', #{p2});
#{a0} = AXIS2_PLACEMENT_3D('', #{p0}, #{d0}, #{d2});
#{a1} = AXIS2_PLACEMENT_3D('', #{p0}, #{d1}, #{d2});
#{c} = CIRCLE('', #{a1}, {r:.6});
#{s} = SPHERICAL_SURFACE('', #{a0}, {r:.6});
#{e} = EDGE_CURVE('', #{v0}, #{v1}, #{c}, .T.);
#{o0} = ORIENTED_EDGE('', *, *, #{e}, .T.);
#{o1} = ORIENTED_EDGE('', *, *, #{e}, .F.);
#{l} = EDGE_LOOP('', (#{o0}, #{o1}));
#{fb} = FACE_OUTER_BOUND('', #{l}, .T.);
#{f} = ADVANCED_FACE('', (#{fb}), #{s}, .T.);
#{sh} = CLOSED_SHELL('', (#{f}));
#{m} = MANIFOLD_SOLID_BREP('ball', #{sh});",
            nr = -r,
            p0 = id(0),
            p1 = id(1),
            p2 = id(2),
            d0 = id(3),
            d1 = id(4),
            d2 = id(5),
            v0 = id(6),
            v1 = id(7),
            a0 = id(8),
            a1 = id(9),
            c = id(10),
            s = id(11),
            e = id(12),
            o0 = id(13),
            o1 = id(14),
            l = id(15),
            fb = id(16),
            f = id(17),
            sh = id(18),
            m = id(19),
        )
    }

    /// AP203 torus: one toroidal face closed by major and minor seam
    /// circles meeting at a single vertex on the outer equator.
    fn torus_step(major: f64, minor: f64) -> String {
        wrap(&format!(
            "\
#1 = CARTESIAN_POINT('', (0., 0., 0.));
#2 = CARTESIAN_POINT('', ({major:.6}, 0., 0.));
#3 = CARTESIAN_POINT('', ({outer:.6}, 0., 0.));
#4 = DIRECTION('', (0., 0., 1.));
#5 = DIRECTION('', (0., -1., 0.));
#6 = DIRECTION('', (1., 0., 0.));
#7 = VERTEX_POINT('', #3);
#8 = AXIS2_PLACEMENT_3D('', #1, #4, #6);
#9 = AXIS2_PLACEMENT_3D('', #2, #5, #6);
#10 = CIRCLE('', #8, {outer:.6});
#11 = CIRCLE('', #9, {minor:.6});
#12 = TOROIDAL_SURFACE('', #8, {major:.6}, {minor:.6});
#13 = EDGE_CURVE('', #7, #7, #10, .T.);
#14 = EDGE_CURVE('', #7, #7, #11, .T.);
#15 = ORIENTED_EDGE('', *, *, #13, .T.);
#16 = ORIENTED_EDGE('', *, *, #14, .T.);
#17 = ORIENTED_EDGE('', *, *, #13, .F.);
#18 = ORIENTED_EDGE('', *, *, #14, .F.);
#19 = EDGE_LOOP('', (#15, #16, #17, #18));
#20 = FACE_OUTER_BOUND('', #19, .T.);
#21 = ADVANCED_FACE('', (#20), #12, .T.);
#22 = CLOSED_SHELL('', (#21));
#23 = MANIFOLD_SOLID_BREP('donut', #22);",
            outer = major + minor,
        ))
    }

    /// A block whose six faces are degree-1 B-spline patches: exact NURBS
    /// import is unsupported, so this must take the mesh fallback.
    fn nurbs_block_step(x: f64, y: f64, z: f64) -> String {
        let (hx, hy, hz) = (x / 2.0, y / 2.0, z / 2.0);
        let corners = [
            (-hx, -hy, -hz),
            (hx, -hy, -hz),
            (hx, hy, -hz),
            (-hx, hy, -hz),
            (-hx, -hy, hz),
            (hx, -hy, hz),
            (hx, hy, hz),
            (-hx, hy, hz),
        ];
        // Same outward cycles as `block_step`; control rows [[a, d], [b, c]]
        // make du x dv the outward normal.
        let cycles: [[usize; 4]; 6] = [
            [0, 3, 2, 1],
            [4, 5, 6, 7],
            [0, 1, 5, 4],
            [1, 2, 6, 5],
            [2, 3, 7, 6],
            [3, 0, 4, 7],
        ];
        let mut b = String::new();
        let mut face_ids = Vec::new();
        for (f, cycle) in cycles.iter().enumerate() {
            let base = 1 + 6 * f;
            for (k, &corner) in cycle.iter().enumerate() {
                let (px, py, pz) = corners[corner];
                writeln!(
                    b,
                    "#{} = CARTESIAN_POINT('', ({px:.6}, {py:.6}, {pz:.6}));",
                    base + k
                )
                .unwrap();
            }
            let (pa, pb, pc, pd) = (base, base + 1, base + 2, base + 3);
            writeln!(
                b,
                "#{} = B_SPLINE_SURFACE_WITH_KNOTS('', 1, 1, ((#{pa}, #{pd}), (#{pb}, #{pc})), \
                 .UNSPECIFIED., .F., .F., .F., (2, 2), (2, 2), (0., 1.), (0., 1.), .UNSPECIFIED.);",
                base + 4
            )
            .unwrap();
            writeln!(
                b,
                "#{} = ADVANCED_FACE('', (), #{}, .T.);",
                base + 5,
                base + 4
            )
            .unwrap();
            face_ids.push(format!("#{}", base + 5));
        }
        writeln!(b, "#100 = CLOSED_SHELL('', ({}));", face_ids.join(", ")).unwrap();
        writeln!(b, "#101 = MANIFOLD_SOLID_BREP('nurbs block', #100);").unwrap();
        wrap(&b)
    }

    // ---- exact import: hand-authored block and cylinder ----

    #[test]
    fn block_imports_as_exact_brep() {
        let (store, geo, report) = import(&block_step(2.0, 3.0, 4.0));
        no_error_diagnostics(&report);
        assert_eq!(report.solids.len(), 1);
        assert_eq!(report.solids[0].name, "block");
        let body = brep_body(&report.solids[0].outcome);

        assert!(store.check(body).is_empty());
        let counts = store.euler_counts(body);
        assert_eq!(
            (counts.vertices, counts.edges, counts.faces, counts.loops),
            (8, 12, 6, 6)
        );
        assert_eq!(counts.genus, 0);
        for face in store.faces_of_body(body) {
            let surface_id = store.face(face).unwrap().surface.expect("surface attached");
            assert!(matches!(
                geo.surface(surface_id).unwrap(),
                Surface3::Plane { .. }
            ));
        }
        assert_edges_interpolate(&store, &geo, body);

        // The mapped body round-trips through the B-Rep tessellator as a
        // closed manifold with the exact volume.
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        assert!((signed_volume(&mesh) - 24.0).abs() < 1e-9);
    }

    #[test]
    fn cylinder_imports_as_exact_brep() {
        let (store, geo, report) = import(&cylinder_step(1.5, 5.0));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);

        assert!(store.check(body).is_empty());
        let counts = store.euler_counts(body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (2, 3, 3));
        let mut kinds: Vec<&str> = store
            .faces_of_body(body)
            .iter()
            .map(|&f| {
                match geo
                    .surface(store.face(f).unwrap().surface.unwrap())
                    .unwrap()
                {
                    Surface3::Plane { .. } => "plane",
                    Surface3::Cylinder { .. } => "cylinder",
                    _ => "other",
                }
            })
            .collect();
        kinds.sort_unstable();
        assert_eq!(kinds, vec!["cylinder", "plane", "plane"]);
        assert_edges_interpolate(&store, &geo, body);

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        let exact = PI * 1.5 * 1.5 * 5.0;
        assert!(
            (signed_volume(&mesh) - exact).abs() / exact < 0.02,
            "volume {} vs {exact}",
            signed_volume(&mesh)
        );
    }

    #[test]
    fn sphere_imports_as_exact_brep() {
        let (store, geo, report) = import(&wrap(&sphere_step_at(1, 2.0)));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);

        assert!(store.check(body).is_empty());
        let counts = store.euler_counts(body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (2, 1, 1));
        let face = store.faces_of_body(body)[0];
        assert!(matches!(
            geo.surface(store.face(face).unwrap().surface.unwrap())
                .unwrap(),
            Surface3::Sphere { radius, .. } if (radius - 2.0).abs() < 1e-12
        ));
        assert_edges_interpolate(&store, &geo, body);

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        let exact = 4.0 / 3.0 * PI * 8.0;
        assert!((signed_volume(&mesh) - exact).abs() / exact < 0.05);
    }

    #[test]
    fn torus_imports_with_recovered_genus() {
        let (store, geo, report) = import(&torus_step(3.0, 1.0));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);

        assert!(store.check(body).is_empty());
        let counts = store.euler_counts(body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (1, 2, 1));
        // STEP carries no genus: it must be recovered from the Euler formula.
        assert_eq!(counts.genus, 1);
        assert_edges_interpolate(&store, &geo, body);

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        let exact = 2.0 * PI * PI * 3.0;
        assert!((signed_volume(&mesh) - exact).abs() / exact < 0.05);
    }

    #[test]
    fn two_solids_import_independently() {
        let data = format!("{}\n{}", sphere_step_at(1, 2.0), sphere_step_at(101, 1.0));
        let (store, _geo, report) = import(&wrap(&data));
        no_error_diagnostics(&report);
        assert_eq!(report.solids.len(), 2);
        let a = brep_body(&report.solids[0].outcome);
        let b = brep_body(&report.solids[1].outcome);
        assert_ne!(a, b);
        assert_eq!(store.bodies.len(), 2);
        assert!(store.check(a).is_empty());
        assert!(store.check(b).is_empty());
    }

    // ---- length units (of-83h) ----

    /// Wrap a DATA body in an envelope that declares a length unit through
    /// a `GLOBAL_UNIT_ASSIGNED_CONTEXT`. `units` must define instance
    /// `#900` as the length unit (support entities may use `#904`–`#919`).
    fn wrap_with_units(data: &str, units: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\n\
             DATA;\n{data}\n{units}\n\
             #901 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
             #902 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n\
             #903 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) \
             GLOBAL_UNIT_ASSIGNED_CONTEXT((#900,#901,#902)) \
             REPRESENTATION_CONTEXT('Context #1','3D Context') );\n\
             ENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    /// Import a radius-2 sphere declared in the given length unit and
    /// return (import report, sphere surface radius, seam vertex points).
    fn import_unit_sphere(units: &str) -> (StepImport, f64, Vec<Point3>) {
        let (store, geo, report) = import(&wrap_with_units(&sphere_step_at(1, 2.0), units));
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty());
        assert_edges_interpolate(&store, &geo, body);
        let face = store.faces_of_body(body)[0];
        let Surface3::Sphere { radius, .. } = geo
            .surface(store.face(face).unwrap().surface.unwrap())
            .unwrap()
        else {
            panic!("expected a sphere surface");
        };
        let mut vertices = Vec::new();
        for edge_id in store.edges_of_face(face) {
            let edge = store.edge(edge_id).unwrap();
            vertices.push(store.vertex(edge.start_vertex).unwrap().point);
            vertices.push(store.vertex(edge.end_vertex).unwrap().point);
        }
        (report, *radius, vertices)
    }

    #[test]
    fn metre_length_unit_scales_geometry_into_millimetres() {
        let (report, radius, vertices) =
            import_unit_sphere("#900 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) );");
        no_error_diagnostics(&report);
        assert_eq!(report.length_scale, 1000.0);
        assert!((radius - 2000.0).abs() < 1e-9, "radius {radius}");
        for v in &vertices {
            assert!(
                (v.coords.norm() - 2000.0).abs() < 1e-9,
                "pole vertex {v} not scaled"
            );
        }
    }

    #[test]
    fn si_prefix_scales_geometry() {
        let (report, radius, _) =
            import_unit_sphere("#900 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.) );");
        no_error_diagnostics(&report);
        assert_eq!(report.length_scale, 10.0);
        assert!((radius - 20.0).abs() < 1e-12);
    }

    #[test]
    fn conversion_based_inch_unit_scales_geometry() {
        // CATIA-style inch: 2.54 of a centimetre unit that is itself not
        // listed in the unit context (only reachable through the measure).
        let (report, radius, _) = import_unit_sphere(
            "#900 = (CONVERSION_BASED_UNIT('INCH',#905) LENGTH_UNIT() NAMED_UNIT(#904));\n\
             #904 = DIMENSIONAL_EXPONENTS(1.0,0.0,0.0,0.0,0.0,0.0,0.0);\n\
             #905 = LENGTH_MEASURE_WITH_UNIT(LENGTH_MEASURE(2.54),#906);\n\
             #906 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.CENTI.,.METRE.) );",
        );
        no_error_diagnostics(&report);
        assert!((report.length_scale - 25.4).abs() < 1e-12);
        assert!((radius - 50.8).abs() < 1e-9);
    }

    #[test]
    fn millimetre_length_unit_imports_verbatim_and_silent() {
        let (report, radius, _) =
            import_unit_sphere("#900 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );");
        assert_eq!(report.length_scale, 1.0);
        assert!((radius - 2.0).abs() < 1e-12);
        assert!(
            report.diagnostics.is_empty(),
            "mm files must import without unit chatter: {:?}",
            report.diagnostics
        );
    }

    #[test]
    fn no_unit_context_imports_verbatim() {
        let (_, _, report) = import(&wrap(&sphere_step_at(1, 2.0)));
        assert_eq!(report.length_scale, 1.0);
    }

    #[test]
    fn uninterpretable_length_unit_warns_and_imports_verbatim() {
        let (report, radius, _) =
            import_unit_sphere("#900 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.FURLONG.) );");
        assert_eq!(report.length_scale, 1.0);
        assert!((radius - 2.0).abs() < 1e-12, "must import verbatim");
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.severity == Severity::Warning
                    && d.message.contains("cannot interpret declared LENGTH_UNIT")),
            "expected an uninterpretable-unit warning: {:?}",
            report.diagnostics
        );
    }

    #[test]
    fn conflicting_length_units_warn_and_first_wins() {
        // A second context declaring millimetres appears before the
        // metre-declaring #903 context, so millimetres (scale 1) wins.
        let (report, radius, _) = import_unit_sphere(
            "#906 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
             #907 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) \
             GLOBAL_UNIT_ASSIGNED_CONTEXT((#906)) \
             REPRESENTATION_CONTEXT('Context #2','3D Context') );\n\
             #900 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) );",
        );
        assert_eq!(report.length_scale, 1.0);
        assert!((radius - 2.0).abs() < 1e-12);
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.severity == Severity::Warning
                    && d.message.contains("conflicting length units")),
            "expected a conflicting-units warning: {:?}",
            report.diagnostics
        );
    }

    #[test]
    fn metre_unit_scales_mesh_fallback_too() {
        // A NURBS block forces the tessellated fallback; its mesh must be
        // scaled the same way as exact imports.
        let data_mm = nurbs_block_step(2.0, 3.0, 4.0);
        let with_metre = wrap_with_units(
            &data_mm
                [data_mm.find("DATA;\n").unwrap() + 6..data_mm.find("\nENDSEC;\nEND-ISO").unwrap()],
            "#900 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT($,.METRE.) );",
        );
        let (_, _, report) = import(&with_metre);
        assert_eq!(report.length_scale, 1000.0);
        let SolidOutcome::Mesh { mesh, .. } = &report.solids[0].outcome else {
            panic!(
                "expected mesh fallback, got {:?}; diagnostics: {:?}",
                report.solids[0].outcome, report.diagnostics
            );
        };
        let volume = signed_volume(mesh);
        assert!(
            (volume - 24.0e9).abs() / 24.0e9 < 1e-9,
            "fallback volume {volume} not scaled into mm"
        );
    }

    // ---- plane-angle units (of-ed1) ----

    /// Wrap a DATA body in an envelope declaring millimetres for length and
    /// the given plane-angle unit through a `GLOBAL_UNIT_ASSIGNED_CONTEXT`.
    /// `angle_units` must define instance `#901` as the plane-angle unit
    /// (support entities may use `#909`–`#919`).
    fn wrap_with_angle_units(data: &str, angle_units: &str) -> String {
        format!(
            "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\n\
             DATA;\n{data}\n\
             #900 = ( LENGTH_UNIT() NAMED_UNIT(*) SI_UNIT(.MILLI.,.METRE.) );\n\
             {angle_units}\n\
             #902 = ( NAMED_UNIT(*) SI_UNIT($,.STERADIAN.) SOLID_ANGLE_UNIT() );\n\
             #903 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) \
             GLOBAL_UNIT_ASSIGNED_CONTEXT((#900,#901,#902)) \
             REPRESENTATION_CONTEXT('Context #1','3D Context') );\n\
             ENDSEC;\nEND-ISO-10303-21;\n"
        )
    }

    /// A CATIA-style degree plane-angle unit: a `CONVERSION_BASED_UNIT`
    /// counting 0.01745… of the SI radian (`#901`, base radian at `#911`,
    /// which is reachable only through the measure — not the context list).
    const DEGREE_UNIT: &str = "\
        #901 = (CONVERSION_BASED_UNIT('DEGREE',#910) NAMED_UNIT(#909) PLANE_ANGLE_UNIT());\n\
        #909 = DIMENSIONAL_EXPONENTS(0.,0.,0.,0.,0.,0.,0.);\n\
        #910 = PLANE_ANGLE_MEASURE_WITH_UNIT(PLANE_ANGLE_MEASURE(0.017453292519943295),#911);\n\
        #911 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );";

    /// Resolve the plane-angle scale of an envelope carrying `angle_units`
    /// (no geometry needed — `resolve_angle_scale` reads only the context).
    fn angle_scale_of(angle_units: &str) -> (f64, Vec<Diagnostic>) {
        let src = wrap_with_angle_units("", angle_units);
        let file = super::super::parse(&src).expect("fixture parses");
        let mut diags = Vec::new();
        let scale = resolve_angle_scale(&file, &mut diags);
        (scale, diags)
    }

    #[test]
    fn degree_plane_angle_unit_resolves_to_radians_per_degree() {
        let (scale, diags) = angle_scale_of(DEGREE_UNIT);
        assert!(
            (scale - PI / 180.0).abs() < 1e-12,
            "degree scale {scale} != pi/180"
        );
        assert!(
            diags
                .iter()
                .any(|d| d.severity == Severity::Info
                    && d.message.contains("declared plane-angle unit")),
            "expected an info diagnostic: {diags:?}"
        );
    }

    #[test]
    fn radian_plane_angle_unit_imports_verbatim_and_silent() {
        let (scale, diags) =
            angle_scale_of("#901 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );");
        assert_eq!(scale, 1.0);
        assert!(
            diags.is_empty(),
            "radian files must import without angle chatter: {diags:?}"
        );
    }

    #[test]
    fn no_angle_unit_imports_verbatim() {
        // A context with only a length unit leaves the angle scale at 1.0.
        let (_, _, report) = import(&wrap(&sphere_step_at(1, 2.0)));
        assert_eq!(report.angle_scale, 1.0);
    }

    #[test]
    fn uninterpretable_plane_angle_unit_warns_and_imports_verbatim() {
        let (scale, diags) =
            angle_scale_of("#901 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.GRADIAN.) );");
        assert_eq!(scale, 1.0);
        assert!(
            diags.iter().any(|d| d.severity == Severity::Warning
                && d.message
                    .contains("cannot interpret declared PLANE_ANGLE_UNIT")),
            "expected an uninterpretable-unit warning: {diags:?}"
        );
    }

    #[test]
    fn conflicting_plane_angle_units_warn_and_first_wins() {
        // A second context declaring radians appears before the
        // degree-declaring #903 context, so radians (scale 1) wins.
        let (scale, diags) = angle_scale_of(&format!(
            "#921 = ( NAMED_UNIT(*) PLANE_ANGLE_UNIT() SI_UNIT($,.RADIAN.) );\n\
             #922 = ( GEOMETRIC_REPRESENTATION_CONTEXT(3) \
             GLOBAL_UNIT_ASSIGNED_CONTEXT((#921)) \
             REPRESENTATION_CONTEXT('Context #2','3D Context') );\n\
             {DEGREE_UNIT}"
        ));
        assert_eq!(scale, 1.0);
        assert!(
            diags.iter().any(|d| d.severity == Severity::Warning
                && d.message.contains("conflicting plane-angle units")),
            "expected a conflicting-units warning: {diags:?}"
        );
    }

    #[test]
    fn degree_unit_scales_cone_semi_angle_into_radians() {
        // A CATIA degree file: the 45° semi-angle must import as pi/4 rad,
        // not the 45 rad a verbatim read would produce.
        let file = super::super::parse(&wrap_with_angle_units(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));\n\
             #2 = DIRECTION('', (0., 0., 1.));\n\
             #3 = DIRECTION('', (1., 0., 0.));\n\
             #4 = AXIS2_PLACEMENT_3D('', #1, #2, #3);\n\
             #7 = CONICAL_SURFACE('', #4, 2.0, 45.0);",
            DEGREE_UNIT,
        ))
        .expect("fixture parses");
        let mut diags = Vec::new();
        let angle_scale = resolve_angle_scale(&file, &mut diags);
        let scale = resolve_length_scale(&file, &mut diags);
        let RawSurface::Analytic(Surface3::Cone {
            half_angle, radius, ..
        }) = resolve_surface(&file, 7, 0, scale, angle_scale).unwrap()
        else {
            panic!("expected a cone");
        };
        assert!(
            (half_angle - PI / 4.0).abs() < 1e-12,
            "45 deg semi-angle imported as {half_angle} rad, expected pi/4"
        );
        assert!((radius - 2.0).abs() < 1e-12, "radius must be unaffected");
    }

    #[test]
    fn degree_unit_exposed_on_report_end_to_end() {
        // read_step populates StepImport::angle_scale from the file context.
        let src = wrap_with_angle_units(&sphere_step_at(1, 2.0), DEGREE_UNIT);
        let (_, _, report) = import(&src);
        assert!(
            (report.angle_scale - PI / 180.0).abs() < 1e-12,
            "report.angle_scale {} != pi/180",
            report.angle_scale
        );
    }

    // ---- entity-coverage: geometry resolvers ----

    fn parse_fixture(data: &str) -> StepFile {
        super::super::parse(&wrap(data)).expect("fixture parses")
    }

    #[test]
    fn resolves_surface_entities() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (1., 2., 3.));
             #2 = DIRECTION('', (0., 0., 1.));
             #3 = DIRECTION('', (1., 0., 0.));
             #4 = AXIS2_PLACEMENT_3D('', #1, #2, #3);
             #5 = SPHERICAL_SURFACE('', #4, 2.5);
             #6 = TOROIDAL_SURFACE('', #4, 3.0, 0.5);
             #7 = CONICAL_SURFACE('', #4, 2.0, 0.5);
             #8 = CYLINDRICAL_SURFACE('', #4, 1.25);
             #9 = PLANE('', #4);",
        );
        let center = Point3::new(1.0, 2.0, 3.0);
        match resolve_surface(&file, 5, 0, 1.0, 1.0).unwrap() {
            RawSurface::Analytic(Surface3::Sphere {
                center: c, radius, ..
            }) => {
                assert!((c - center).norm() < 1e-12);
                assert!((radius - 2.5).abs() < 1e-12);
            }
            _ => panic!("expected a sphere"),
        }
        match resolve_surface(&file, 6, 0, 1.0, 1.0).unwrap() {
            RawSurface::Analytic(Surface3::Torus {
                major_radius,
                minor_radius,
                ..
            }) => {
                assert!((major_radius - 3.0).abs() < 1e-12);
                assert!((minor_radius - 0.5).abs() < 1e-12);
            }
            _ => panic!("expected a torus"),
        }
        match resolve_surface(&file, 7, 0, 1.0, 1.0).unwrap() {
            RawSurface::Analytic(Surface3::Cone {
                half_angle, radius, ..
            }) => {
                assert!((half_angle - 0.5).abs() < 1e-12);
                assert!((radius - 2.0).abs() < 1e-12);
            }
            _ => panic!("expected a cone"),
        }
        assert!(matches!(
            resolve_surface(&file, 8, 0, 1.0, 1.0).unwrap(),
            RawSurface::Analytic(Surface3::Cylinder { .. })
        ));
        assert!(matches!(
            resolve_surface(&file, 9, 0, 1.0, 1.0).unwrap(),
            RawSurface::Analytic(Surface3::Plane { .. })
        ));
    }

    #[test]
    fn axis2_placement_defaults_to_z_axis() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = AXIS2_PLACEMENT_3D('', #1, $, $);",
        );
        let placement = resolve_axis2(&file, 2, 0, 1.0).unwrap();
        assert!((placement.axis - Vector3::z()).norm() < 1e-12);
        assert!(placement.ref_dir.is_none());
    }

    #[test]
    fn resolves_ellipse_swapping_semi_axes() {
        // semi_axis_1 (along ref_direction x) is the SMALLER one: the mapper
        // must rotate the major direction so Curve3's major >= minor holds.
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = DIRECTION('', (0., 0., 1.));
             #3 = DIRECTION('', (1., 0., 0.));
             #4 = AXIS2_PLACEMENT_3D('', #1, #2, #3);
             #5 = ELLIPSE('', #4, 1.0, 2.0);",
        );
        match resolve_curve(&file, 5, 0, 1.0).unwrap() {
            RawCurve::Analytic(
                curve @ Curve3::Ellipse {
                    major_radius,
                    minor_radius,
                    major_dir,
                    ..
                },
            ) => {
                assert!((major_radius - 2.0).abs() < 1e-12);
                assert!((minor_radius - 1.0).abs() < 1e-12);
                assert!((major_dir - Vector3::y()).norm() < 1e-12);
                // The semi-major vertex lies along +y.
                assert!((curve.point(0.0) - Point3::new(0.0, 2.0, 0.0)).norm() < 1e-12);
            }
            _ => panic!("expected an ellipse"),
        }
    }

    #[test]
    fn resolves_bspline_curve_with_expanded_knots() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (1., 1., 0.));
             #3 = CARTESIAN_POINT('', (2., 0., 0.));
             #4 = B_SPLINE_CURVE_WITH_KNOTS('', 2, (#1, #2, #3), .UNSPECIFIED., .F., .F., \
                  (3, 3), (0., 1.), .UNSPECIFIED.);",
        );
        let RawCurve::Nurbs(curve) = resolve_curve(&file, 4, 0, 1.0).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(curve.degree(), 2);
        let (t0, t1) = curve.knot_vector().domain();
        assert!((curve.point(t0) - Point3::new(0.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((curve.point(t1) - Point3::new(2.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn resolves_bspline_surface_grid() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (0., 1., 0.));
             #3 = CARTESIAN_POINT('', (1., 0., 0.));
             #4 = CARTESIAN_POINT('', (1., 1., 1.));
             #5 = B_SPLINE_SURFACE_WITH_KNOTS('', 1, 1, ((#1, #2), (#3, #4)), .UNSPECIFIED., \
                  .F., .F., .F., (2, 2), (2, 2), (0., 1.), (0., 1.), .UNSPECIFIED.);",
        );
        let RawSurface::Nurbs(surface) = resolve_surface(&file, 5, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS surface");
        };
        assert_eq!(surface.degree_u(), 1);
        assert_eq!(surface.degree_v(), 1);
        assert!((surface.point(0.0, 0.0) - Point3::new(0.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((surface.point(1.0, 1.0) - Point3::new(1.0, 1.0, 1.0)).norm() < 1e-12);
    }

    // ---- entity-coverage: edge trimming ----

    #[test]
    fn trims_quarter_arc_by_vertices() {
        let circle = Curve3::circle(Point3::origin(), Vector3::z(), 1.0).unwrap();
        let trimmed = trim_curve(
            &circle,
            true,
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            false,
            1,
        )
        .unwrap();
        assert!(trimmed.t_start.abs() < 1e-12);
        assert!((trimmed.t_end - FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn same_sense_false_takes_the_complement_arc() {
        // The edge runs against the circle: from (1,0,0) the long way
        // around (through -y) to (0,1,0), a 3pi/2 sweep on the reversed curve.
        let circle = Curve3::circle(Point3::origin(), Vector3::z(), 1.0).unwrap();
        let trimmed = trim_curve(
            &circle,
            false,
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            false,
            1,
        )
        .unwrap();
        assert!(trimmed.t_start < trimmed.t_end, "normalized trim direction");
        assert!((trimmed.t_end - trimmed.t_start - 3.0 * FRAC_PI_2).abs() < 1e-12);
        // Midpoint of the traversal is on the -y side.
        let mid = trimmed.curve.point((trimmed.t_start + trimmed.t_end) / 2.0);
        assert!(mid.y < 0.0, "complement arc passes through -y, got {mid}");
    }

    #[test]
    fn same_sense_false_reverses_a_line() {
        // The line points -x but the edge runs +x from the origin.
        let line = Curve3::line(Point3::new(1.0, 0.0, 0.0), -Vector3::x()).unwrap();
        let trimmed = trim_curve(
            &line,
            false,
            Point3::origin(),
            Point3::new(1.0, 0.0, 0.0),
            false,
            1,
        )
        .unwrap();
        assert!(trimmed.t_start < trimmed.t_end);
        assert!((trimmed.curve.point(trimmed.t_start) - Point3::origin()).norm() < 1e-12);
    }

    #[test]
    fn closed_edge_spans_the_full_circle() {
        let circle = Curve3::circle(Point3::origin(), Vector3::z(), 2.0).unwrap();
        let vertex = Point3::new(0.0, 2.0, 0.0);
        let trimmed = trim_curve(&circle, true, vertex, vertex, true, 1).unwrap();
        assert!((trimmed.t_end - trimmed.t_start - TAU).abs() < 1e-12);
        assert!((trimmed.curve.point(trimmed.t_start) - vertex).norm() < 1e-12);
    }

    #[test]
    fn trim_rejects_vertices_off_the_curve() {
        let circle = Curve3::circle(Point3::origin(), Vector3::z(), 1.0).unwrap();
        let err = trim_curve(
            &circle,
            true,
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            false,
            7,
        )
        .unwrap_err();
        assert!(
            matches!(err, MapError::Invalid { entity: 7, .. }),
            "{err:?}"
        );
    }

    #[test]
    fn trim_rejects_closed_edge_on_a_line() {
        let line = Curve3::line(Point3::origin(), Vector3::x()).unwrap();
        let err = trim_curve(&line, true, Point3::origin(), Point3::origin(), true, 7).unwrap_err();
        assert!(matches!(err, MapError::Invalid { .. }));
    }

    // ---- mesh fallback ----

    #[test]
    fn nurbs_block_falls_back_to_watertight_mesh() {
        let (store, geo, report) = import(&nurbs_block_step(2.0, 2.0, 2.0));
        assert_eq!(report.solids.len(), 1);
        let SolidOutcome::Mesh { mesh, sdf } = &report.solids[0].outcome else {
            panic!(
                "expected the mesh fallback, got {:?}",
                report.solids[0].outcome
            );
        };

        assert!(mesh.is_closed_manifold());
        assert!(
            (signed_volume(mesh) - 8.0).abs() < 1e-9,
            "bilinear patches mesh exactly"
        );
        assert!(sdf.eval(&Point3::origin()) < 0.0, "center is inside");
        let outside = sdf.eval(&Point3::new(5.0, 0.0, 0.0));
        assert!(
            (outside - 4.0).abs() < 1e-6,
            "outside distance, got {outside}"
        );

        // The reason for the fallback is reported per entity.
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.severity == Severity::Warning && d.message.contains("NURBS")),
            "expected a NURBS warning: {:?}",
            report.diagnostics
        );
        // The failed exact attempt left nothing behind.
        assert!(store.bodies.is_empty());
        assert!(store.faces.is_empty());
        assert!(store.vertices.is_empty());
        assert!(geo.surfaces.is_empty());
        assert!(geo.curves.is_empty());
    }

    #[test]
    fn open_shell_fails_with_diagnostics_and_rolls_back() {
        // The block file with one face dropped from its shell: the mapped
        // body fails check() (open edges in a solid shell) and the 5-face
        // fallback tessellation cannot close either.
        let src = block_step(2.0, 2.0, 2.0).replace(
            "(#74, #84, #94, #104, #114, #124)",
            "(#74, #84, #94, #104, #114)",
        );
        let (store, geo, report) = import(&src);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.message.contains("failed validation")),
            "check failures reported: {:?}",
            report.diagnostics
        );
        assert!(report.has_errors());
        assert!(store.bodies.is_empty(), "rolled back");
        assert!(store.edges.is_empty(), "rolled back");
        assert!(geo.surfaces.is_empty(), "rolled back");
    }

    #[test]
    fn dangling_reference_fails_the_solid() {
        let (store, _geo, report) = import(&wrap("#1 = MANIFOLD_SOLID_BREP('broken', #2);"));
        assert_eq!(report.solids.len(), 1);
        assert!(matches!(report.solids[0].outcome, SolidOutcome::Failed));
        assert!(report.has_errors());
        assert!(store.bodies.is_empty());
    }

    #[test]
    fn file_without_solids_warns() {
        let (_store, _geo, report) = import(&wrap("#1 = CARTESIAN_POINT('', (0., 0., 0.));"));
        assert!(report.solids.is_empty());
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.severity == Severity::Warning
                    && d.message.contains("MANIFOLD_SOLID_BREP"))
        );
    }
}
