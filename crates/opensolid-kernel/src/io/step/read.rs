//! STEP AP203 semantic mapper: parsed entity graph → kernel B-Rep.
//!
//! [`read_step`] walks every `MANIFOLD_SOLID_BREP` in a parsed
//! [`StepFile`] and rebuilds it through the kernel's public construction
//! APIs ([`TopologyStore`] / [`GeometryStore`]), exactly when possible:
//!
//! - **Geometry**: `cartesian_point`, `direction`, `vector`,
//!   `axis1_placement`, `axis2_placement_3d`; `plane`,
//!   `cylindrical_surface`, `conical_surface`, `spherical_surface`,
//!   `toroidal_surface` → [`Surface3`]; `line`, `circle`, `ellipse` →
//!   [`Curve3`]. Freeform geometry is exact too (of-3qy.8):
//!   `b_spline_surface` → [`Surface3::Nurbs`] and `b_spline_curve` →
//!   [`Curve3::Nurbs`], in every AP203 spelling — the explicit
//!   `*_with_knots` form, the `rational_b_spline_*` complex instance that
//!   carries weights, and the implicit-knot subtypes `quasi_uniform_*`,
//!   `uniform_*` and `bezier_*`, whose knot vectors are derived from the
//!   degree and control-point count. A `trimmed_curve` is transparent
//!   (edges re-trim by their vertices), and `surface_of_linear_extrusion` /
//!   `surface_of_revolution` reduce to the exact quadric where one exists
//!   (line → plane/cylinder/cone, axis-coplanar circle → sphere/torus,
//!   circle along its normal → cylinder; NURBS bases extrude to an exact
//!   ruled patch, sized to the face that carries it — the entity itself is
//!   unbounded along the sweep).
//! - **Topology**: `vertex_point`, `edge_curve`, `oriented_edge`,
//!   `edge_loop`, `face_bound` / `face_outer_bound`, `advanced_face`,
//!   `closed_shell`, `manifold_solid_brep` → `Vertex` / `Edge` / `Loop` /
//!   `Face` / `Shell` / `Body`. A `vertex_loop` — the bound at a
//!   parameterization singularity, a cone apex or sphere pole — maps to the
//!   degenerate loop that carries a vertex and no fins
//!   ([`Loop::vertex`](opensolid_brep::Loop::vertex)); having no extent, it
//!   never takes the outer role on a face that also has a real loop, however
//!   the file tags it. A `brep_with_voids` maps its
//!   `oriented_closed_shell` cavities to inner shells
//!   ([`ShellOrientation::Inward`](opensolid_brep::ShellOrientation)),
//!   reversing faces whose orientation flag negates the authored winding.
//! - **Trim geometry**: `surface_curve` / `seam_curve` /
//!   `intersection_curve` edges map through their `curve_3d`, and every fin
//!   of an exactly imported body gets a 2D pcurve in its face's parameter
//!   space ([`Fin::pcurve`](opensolid_brep::Fin::pcurve)). The pcurve is
//!   derived, not transplanted — see [`validate_associated_geometry`] for why
//!   the authored `pcurve` geometry does not transfer, and
//!   [`StepReadOptions::pcurves`] to turn the derivation off.
//!
//! STEP trims edges by their vertices, not by curve parameters, so the
//! mapper recovers each edge's parameter range by inverse-projecting the
//! vertex points onto the mapped curve, and re-orients the curve when
//! `edge_curve.same_sense` is false so every edge satisfies
//! `t_start < t_end`. A conic whose two vertices land on the same point
//! sweeps a full period rather than nothing, whether the file spelled that
//! with one vertex or two coincident ones. Where a writer has parked a face's
//! seam on an `edge_curve` other faces also use — what a tangent boolean
//! produces — the seam is moved onto an edge of its own
//! ([`split_overshared_seams`](SolidBuilder::split_overshared_seams)) so no
//! edge carries more than two fins. Every mapped body is validated with
//! [`TopologyStore::check`].
//!
//! # Tolerances
//!
//! What an imported entity's tolerance says is where that entity actually
//! is, measured — never [`SYSTEM_RESOLUTION`] by default (of-bb6). A STEP
//! file rounds a curve, the vertices trimming it and the surfaces of the
//! faces it bounds into decimal text independently, and plenty of producers
//! author geometry that never met exactly to begin with, so the gaps are
//! real and the reader's job is to record them rather than to claim they are
//! not there. A vertex carries the residual [`trim_curve`] accepted when it
//! trimmed each adjacent edge; an edge carries how far its curve strays from
//! the surfaces it bounds, measured by [`record_edge_tolerances`] once the
//! body is whole. Where the measurement exceeds [`MAX_ALLOWED_TOLERANCE`]
//! there is no tolerance the kernel could honour, and the solid degrades to
//! the tessellated import below instead of claiming one.
//!
//! # Mesh fallback
//!
//! B-splines no longer take this path (of-3qy.8) — they map exactly, as
//! above. What is still unsupported is `parabola` / `hyperbola` edges (the
//! geometry store has no conic variant), `composite_curve` edges, and
//! swept surfaces with neither a quadric reduction nor a NURBS form.
//! Solids containing any of those (or any other unmappable entity), and
//! solids whose mapped topology fails `check`, fall back to a
//! **tessellated import**: each face is triangulated straight from the
//! STEP graph (planar faces ear-clipped from their boundary polylines,
//! quadrics gridded over their parameter rectangle, NURBS patches gridded
//! over their knot domain, swept
//! surfaces gridded from their sampled basis curve), welded, and
//! wrapped as a [`MeshSdf`] — an F-Rep field ready for CSG. Faces of one
//! solid share each edge's discretization, so junctions weld watertight.
//! Anything the fallback cannot handle either fails the solid
//! ([`SolidOutcome::Failed`]) with per-entity [`Diagnostic`]s explaining
//! why. A failed or fallen-back solid leaves no partial entities in the
//! stores.
//!
//! # Assemblies
//!
//! The solid list above is *per authored part*. Where a file also carries
//! product structure — `NEXT_ASSEMBLY_USAGE_OCCURRENCE` /
//! `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` placements, or `MAPPED_ITEM`
//! instancing — the [`product`](super::product) submodule resolves it into
//! [`StepImport::instances`]: one [`PlacedSolid`] per *occurrence*, with
//! the rigid transform composed down the assembly tree. Geometry is never
//! duplicated for an instance, so a bolt used ten times is one imported
//! body and ten transforms. A file with no product structure yields one
//! identity-placed occurrence per solid, which is the same picture a
//! caller ignoring assemblies already had.
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
    Body, BodyType, Curve2, Curve3, CurveEval, CurveProject, Edge, Face, FaceSense, Fin, FinSense,
    GeometryStore, KnotVector, Loop, LoopType, MAX_ALLOWED_TOLERANCE, NurbsCurve, NurbsCurve2,
    NurbsError, NurbsSurface, SYSTEM_RESOLUTION, Shell, ShellOrientation, Surface3, SurfaceEval,
    SurfaceProject, TessellationOptions, TopologyStore, Vertex, attach_body_pcurves,
};
use opensolid_core::error::CoreError;
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::Point2;
use opensolid_core::{EntityId, Point3, Vector3};

use super::heal::{GeometryHealer, HealOptions, HealStrategy, reconcile_face_senses};
use super::product::{PlacedSolid, resolve_instances};
use super::{EntityRecord, Instance, SimpleRecord, StepError, StepFile, Value};
use crate::convert::MeshSdf;

const TAU: f64 = std::f64::consts::TAU;

/// Relative tolerance for verifying that mapped curves interpolate their
/// edge's vertex points. STEP files are written with finite decimal
/// precision, so this is far looser than [`SYSTEM_RESOLUTION`].
const TRIM_TOL_REL: f64 = 1e-6;

/// Options for [`read_step`].
#[derive(Debug, Clone)]
pub struct StepReadOptions {
    /// Fidelity of the mesh-fallback tessellation.
    pub tessellation: TessellationOptions,
    /// How aggressively a mapped body that fails validation is repaired
    /// before the reader gives up on the exact path (see [`heal`]). The
    /// default heals automatically: import heals, it does not reject.
    pub heal: HealOptions,
    /// Whether to derive 2D trim geometry for every fin of an exactly
    /// imported body ([`Fin::pcurve`](opensolid_brep::Fin::pcurve)).
    ///
    /// On by default: a face's boundary is only well defined in its
    /// surface's parameter space, and a seam edge in particular cannot be
    /// told apart from a stray duplicate without it. Turn it off to skip the
    /// per-fin projection when an import only needs 3D geometry.
    pub pcurves: bool,
}

impl Default for StepReadOptions {
    fn default() -> Self {
        Self {
            tessellation: TessellationOptions::default(),
            heal: HealOptions::default(),
            pcurves: true,
        }
    }
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
    /// One entry per *placed occurrence* of a solid, resolved from the
    /// file's product structure (`NEXT_ASSEMBLY_USAGE_OCCURRENCE` /
    /// `MAPPED_ITEM` — see [`product`](super::product)). A single-part
    /// file yields exactly one identity-placed entry per solid; an
    /// assembly yields one per instance, so a part used twice appears
    /// twice with the same [`PlacedSolid::solid`] index and different
    /// transforms. The geometry in the stores stays in part-local
    /// coordinates: an instance is *(part, transform)*, never a copy.
    pub instances: Vec<PlacedSolid>,
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
    /// Total repairs the [healer](super::heal) applied across every solid
    /// (`spec/06-step-io.md`'s `ImportStats::heal_operations`). Each is also
    /// reported individually as a [`Severity::Info`] diagnostic.
    pub heal_operations: usize,
}

impl StepImport {
    /// Whether any diagnostic is [`Severity::Error`].
    pub fn has_errors(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error)
    }

    /// Whether the file carries real assembly structure: some solid is
    /// placed away from the origin, or placed more than once. A flat
    /// single-part file is not an assembly however many solids it holds.
    pub fn is_assembly(&self) -> bool {
        self.instances.len() > self.solids.len() || self.instances.iter().any(|i| !i.is_identity())
    }

    /// The occurrences of one solid, by its index in
    /// [`solids`](Self::solids).
    pub fn instances_of(&self, solid: usize) -> impl Iterator<Item = &PlacedSolid> {
        self.instances.iter().filter(move |i| i.solid == solid)
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
pub(super) enum MapError {
    /// Valid STEP the kernel cannot represent exactly.
    Unsupported { entity: u64, what: String },
    /// Malformed or unresolvable data.
    Invalid { entity: u64, what: String },
}

pub(super) type MapResult<T> = Result<T, MapError>;

impl MapError {
    pub(super) fn diagnostic(&self) -> Diagnostic {
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

pub(super) fn invalid(entity: u64, what: impl Into<String>) -> MapError {
    MapError::Invalid {
        entity,
        what: what.into(),
    }
}

pub(super) fn unsupported(entity: u64, what: impl Into<String>) -> MapError {
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

pub(super) fn attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<&Value> {
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

pub(super) fn ref_attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<u64> {
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

pub(super) fn list_attr(rec: &SimpleRecord, index: usize, entity: u64) -> MapResult<&[Value]> {
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
pub(super) fn name_attr(rec: &SimpleRecord) -> String {
    rec.attributes
        .first()
        .and_then(Value::as_str)
        .unwrap_or("")
        .to_string()
}

pub(super) fn instance(file: &StepFile, id: u64, referrer: u64) -> MapResult<&Instance> {
    file.get(id)
        .ok_or_else(|| invalid(referrer, format!("dangling reference #{id}")))
}

/// Human-readable type name(s) of an instance, for messages.
pub(super) fn type_names(inst: &Instance) -> String {
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
pub(super) fn typed_record<'f>(
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
pub(super) fn resolve_point(
    file: &StepFile,
    id: u64,
    referrer: u64,
    scale: f64,
) -> MapResult<Point3> {
    let rec = typed_record(file, id, "CARTESIAN_POINT", referrer)?;
    let [x, y, z] = triple(rec, 1, id)?;
    Ok(Point3::new(x * scale, y * scale, z * scale))
}

pub(super) fn resolve_direction(file: &StepFile, id: u64, referrer: u64) -> MapResult<Vector3> {
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
pub(super) struct Placement {
    pub(super) location: Point3,
    pub(super) axis: Vector3,
    pub(super) ref_dir: Option<Vector3>,
}

pub(super) fn resolve_axis2(
    file: &StepFile,
    id: u64,
    referrer: u64,
    scale: f64,
) -> MapResult<Placement> {
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
    /// `SURFACE_OF_LINEAR_EXTRUSION` of a NURBS basis: an exact ruled patch
    /// once the face says how far along `dir` it reaches (the STEP surface
    /// is unbounded there, so only the face's own bounds can size it — see
    /// [`extrude_nurbs_curve`]).
    ExtrudedNurbs {
        curve: Box<NurbsCurve>,
        dir: Vector3,
    },
    /// `SURFACE_OF_LINEAR_EXTRUSION` with no analytic/NURBS reduction:
    /// tessellation-only (the basis polyline swept along `dir`).
    Extruded {
        basis: Box<RawCurve>,
        dir: Vector3,
    },
    /// `SURFACE_OF_REVOLUTION` with no analytic reduction:
    /// tessellation-only (the basis polyline revolved about the axis).
    Revolved {
        basis: Box<RawCurve>,
        origin: Point3,
        axis: Vector3,
    },
}

fn resolve_surface(
    file: &StepFile,
    id: u64,
    referrer: u64,
    scale: f64,
    angle_scale: f64,
) -> MapResult<RawSurface> {
    let inst = instance(file, id, referrer)?;
    // As for curves: one path for every B-spline spelling, including the
    // `RATIONAL_B_SPLINE_SURFACE` complex instance.
    if let Some(parts) = BSplineParts::surface(&inst.entity) {
        return Ok(RawSurface::Nurbs(Box::new(resolve_bspline_surface(
            file, &parts, id, scale,
        )?)));
    }
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
        // `SURFACE_OF_LINEAR_EXTRUSION(name, swept_curve, extrusion_axis)`.
        // A line extrudes to a plane and a circle along its own normal to a
        // cylinder — the forms real exporters emit for prismatic walls —
        // giving an exact import; a NURBS basis extrudes to an exact ruled
        // NURBS patch, built per face by [`extruded_nurbs_surface`] because
        // the extent is the face's to state and not the entity's; anything
        // else keeps the swept form for tessellation.
        "SURFACE_OF_LINEAR_EXTRUSION" => {
            let basis = resolve_curve(file, ref_attr(rec, 1, id)?, id, scale, angle_scale)?;
            let dir = resolve_vector(file, ref_attr(rec, 2, id)?, id, scale)?;
            let dir_norm = dir.norm();
            if dir_norm < 1e-12 || !dir_norm.is_finite() {
                return Err(invalid(
                    id,
                    "SURFACE_OF_LINEAR_EXTRUSION axis is degenerate",
                ));
            }
            if let Some(reduced) = reduce_extrusion(basis.analytic_basis(), &dir, id)? {
                return Ok(RawSurface::Analytic(reduced));
            }
            if let RawCurve::Nurbs(curve) = basis {
                return Ok(RawSurface::ExtrudedNurbs { curve, dir });
            }
            Ok(RawSurface::Extruded {
                basis: Box::new(basis),
                dir,
            })
        }
        // `SURFACE_OF_REVOLUTION(name, swept_curve, axis_position)`. Lines
        // revolve to planes/cylinders/cones and axis-coplanar circles to
        // spheres/tori — exporters routinely spell quadrics this way — all
        // reduced to exact analytic imports; other bases keep the swept
        // form for tessellation.
        "SURFACE_OF_REVOLUTION" => {
            let basis = resolve_curve(file, ref_attr(rec, 1, id)?, id, scale, angle_scale)?;
            let (origin, axis_raw) = resolve_axis1(file, ref_attr(rec, 2, id)?, id, scale)?;
            let axis_norm = axis_raw.norm();
            if axis_norm < 1e-12 || !axis_norm.is_finite() {
                return Err(invalid(id, "SURFACE_OF_REVOLUTION axis is degenerate"));
            }
            let axis = axis_raw / axis_norm;
            if let Some(reduced) = reduce_revolution(basis.analytic_basis(), origin, &axis, id)? {
                return Ok(RawSurface::Analytic(reduced));
            }
            Ok(RawSurface::Revolved {
                basis: Box::new(basis),
                origin,
                axis,
            })
        }
        other => Err(unsupported(id, format!("surface type {other}"))),
    }
}

/// Reduction acceptance tolerance for swept-surface geometry: directions
/// and relative positions within this of an exact quadric configuration
/// map to the quadric. Far looser than machine epsilon (STEP files carry
/// finite decimal text) but tight enough that a genuinely skew sweep never
/// silently becomes the wrong exact surface.
const SWEEP_REDUCE_TOL: f64 = 1e-9;

/// The exact quadric a linear extrusion collapses to, if any: a plane
/// (line basis) or a circular cylinder (circle basis extruded along its
/// own normal).
fn reduce_extrusion(
    basis: Option<&Curve3>,
    dir: &Vector3,
    entity: u64,
) -> MapResult<Option<Surface3>> {
    let unit = dir / dir.norm();
    match basis {
        Some(Curve3::Line { origin, dir: d }) => {
            let normal = d.cross(&unit);
            if normal.norm() < SWEEP_REDUCE_TOL {
                return Err(invalid(
                    entity,
                    "SURFACE_OF_LINEAR_EXTRUSION sweeps a line along itself",
                ));
            }
            Ok(Some(
                Surface3::plane(*origin, normal).map_err(|e| geometry_error(entity, &e))?,
            ))
        }
        Some(Curve3::Circle {
            center,
            axis,
            radius,
        }) if axis.dot(&unit).abs() > 1.0 - SWEEP_REDUCE_TOL => Ok(Some(
            Surface3::cylinder(*center, *axis, *radius).map_err(|e| geometry_error(entity, &e))?,
        )),
        _ => Ok(None),
    }
}

/// The exact quadric a revolution collapses to, if any.
///
/// - Line ⊥ axis → plane (every line point keeps its height).
/// - Line ∥ axis → cylinder of the line-to-axis distance.
/// - Line meeting the axis obliquely → cone with its apex at the
///   intersection, opening toward the side the line's anchor point lies on.
/// - Circle whose plane contains the axis → sphere (center on the axis) or
///   torus (center off it, when the tube clears the axis).
fn reduce_revolution(
    basis: Option<&Curve3>,
    origin: Point3,
    axis: &Vector3,
    entity: u64,
) -> MapResult<Option<Surface3>> {
    match basis {
        Some(Curve3::Line { origin: c0, dir: d }) => {
            let b = d.dot(axis);
            let w = c0 - origin;
            if b.abs() > 1.0 - SWEEP_REDUCE_TOL {
                // Parallel: cylinder, unless the line lies on the axis.
                let radial = w - axis * w.dot(axis);
                if radial.norm() < SWEEP_REDUCE_TOL * (1.0 + w.norm()) {
                    return Err(invalid(
                        entity,
                        "SURFACE_OF_REVOLUTION revolves a line lying on its axis",
                    ));
                }
                return Ok(Some(
                    Surface3::cylinder(origin, *axis, radial.norm())
                        .map_err(|e| geometry_error(entity, &e))?,
                ));
            }
            if b.abs() < SWEEP_REDUCE_TOL {
                // Perpendicular: all line points share one height → plane.
                return Ok(Some(
                    Surface3::plane(*c0, *axis).map_err(|e| geometry_error(entity, &e))?,
                ));
            }
            // Oblique: a cone exactly when the line meets the axis
            // (coplanar); a skew line sweeps a hyperboloid (no reduction).
            let n = d.cross(axis);
            if w.dot(&n).abs() > SWEEP_REDUCE_TOL * (1.0 + w.norm()) * n.norm() {
                return Ok(None);
            }
            let t_apex = (b * w.dot(axis) - w.dot(d)) / (1.0 - b * b);
            let apex = c0 + d * t_apex;
            let half_angle = b.abs().clamp(0.0, 1.0).acos();
            // Open the nappe toward the line's anchor point; if the anchor
            // IS the apex, along the line direction's axial component.
            let side = (c0 - apex).dot(axis);
            let toward = if side.abs() > SWEEP_REDUCE_TOL * (1.0 + apex.coords.norm()) {
                side
            } else {
                b
            };
            let cone_axis = if toward >= 0.0 { *axis } else { -*axis };
            Ok(Some(
                Surface3::cone(apex, cone_axis, half_angle, 0.0)
                    .map_err(|e| geometry_error(entity, &e))?,
            ))
        }
        Some(Curve3::Circle {
            center,
            axis: circle_axis,
            radius,
        }) => {
            let w = center - origin;
            let scale = 1.0 + w.norm() + radius;
            // The circle's plane must contain the axis: normals
            // perpendicular, and the axis passing through the plane.
            if circle_axis.dot(axis).abs() > SWEEP_REDUCE_TOL
                || w.dot(circle_axis).abs() > SWEEP_REDUCE_TOL * scale
            {
                return Ok(None);
            }
            let radial = w - axis * w.dot(axis);
            let major = radial.norm();
            if major < SWEEP_REDUCE_TOL * scale {
                // Meridian circle centered on the axis → sphere.
                return Ok(Some(
                    Surface3::sphere(*center, *axis, *radius)
                        .map_err(|e| geometry_error(entity, &e))?,
                ));
            }
            if major > radius + SWEEP_REDUCE_TOL * scale {
                let tube_center = origin + axis * w.dot(axis);
                return Ok(Some(
                    Surface3::torus(tube_center, *axis, major, *radius)
                        .map_err(|e| geometry_error(entity, &e))?,
                ));
            }
            // Tube crossing the axis: a self-intersecting (lemon/apple)
            // torus the kernel cannot hold — tessellate instead.
            Ok(None)
        }
        _ => Ok(None),
    }
}

/// Extrude a NURBS curve into the exact ruled patch spanning `span` — the
/// two signed distances along the *unit* extrusion direction the rows sit
/// at, measured from the curve. `v = 0` is the `span.0` row and `v = 1` the
/// `span.1` row; degree 1 in `v`, the curve's weights repeated (a translate
/// preserves rationality).
///
/// The span cannot come from the STEP entity. `SURFACE_OF_LINEAR_EXTRUSION`
/// is unbounded in the sweep direction and its `VECTOR` magnitude is
/// routinely 1 whatever the part measures, so only the faces built on the
/// surface say how far — and which way — it has to reach (of-8ulj). See
/// [`extruded_nurbs_surface`].
fn extrude_nurbs_curve(
    curve: &NurbsCurve,
    unit: &Vector3,
    span: (f64, f64),
    entity: u64,
) -> MapResult<NurbsSurface> {
    let knots_v =
        KnotVector::new(1, vec![0.0, 0.0, 1.0, 1.0]).map_err(|e| nurbs_error(entity, &e))?;
    let (lo, hi) = span;
    // Control grid outer index runs over u — the curve direction — so the
    // surface normal du × dv matches curve-tangent × extrusion, the STEP
    // convention for surface_of_linear_extrusion.
    let grid: Vec<Vec<Point3>> = curve
        .control_points()
        .iter()
        .map(|p| vec![p + unit * lo, p + unit * hi])
        .collect();
    let weights: Vec<Vec<f64>> = curve.weights().iter().map(|&w| vec![w, w]).collect();
    NurbsSurface::new(grid, weights, curve.knot_vector().clone(), knots_v)
        .map_err(|e| nurbs_error(entity, &e))
}

/// Slack added to a measured extrusion span, relative to the span itself:
/// the patch is meant to *contain* its face, not to end exactly on it, so a
/// boundary edge never lands on the knot-domain edge where a projection has
/// to clamp.
///
/// Small on purpose. The tessellator reads a face as an untrimmed patch —
/// and grids it directly, rather than deferring to the CDT pass — only while
/// its boundary sits within `1e-6` of the knot domain's border
/// (`nurbs_lattice`), and the usual profile (perpendicular to the sweep) is
/// measured exactly, so this margin is all that stands between such a face
/// and the fast path.
const EXTRUSION_SPAN_MARGIN: f64 = 1e-9;

/// The ruled patch one `ADVANCED_FACE` needs from its
/// `SURFACE_OF_LINEAR_EXTRUSION` of a NURBS curve: the curve swept far
/// enough along `dir` — in *both* directions — to cover the face's own
/// bounds (of-8ulj).
///
/// The extent has to be measured because the entity does not carry one: the
/// surface is unbounded along the sweep, and the `VECTOR`'s magnitude and
/// sign say nothing about the faces built on it (`bspline_patch_prism.stp`
/// spells a magnitude-1 `+z` sweep for faces reaching 20 mm along `−z`).
fn extruded_nurbs_surface(
    file: &StepFile,
    bounds: &[u64],
    face_ref: u64,
    scale: f64,
    angle_scale: f64,
    curve: &NurbsCurve,
    dir: &Vector3,
) -> MapResult<NurbsSurface> {
    let unit = dir / dir.norm();
    let span = extrusion_span(file, bounds, face_ref, scale, angle_scale, curve, &unit)?;
    extrude_nurbs_curve(curve, &unit, span, face_ref)
}

/// How far along `unit` a face's bounds reach, as signed distances from the
/// swept curve — the `(lo, hi)` [`extrude_nurbs_curve`] sweeps between.
///
/// Every point of the surface is `C(u) + unit·t`, so the `t` to cover is
/// what the bounding curves' points measure *minus* the swept curve's own
/// displacement along `unit` at the matching `u`. That displacement is not
/// known per point without inverting the curve, so its full range over the
/// control hull is subtracted instead: the result always contains the true
/// span and overshoots by at most the profile's own extent along the sweep
/// — zero for the usual perpendicular profile, and harmless otherwise
/// (a patch may reach past its face; it may never fall short of it).
fn extrusion_span(
    file: &StepFile,
    bounds: &[u64],
    face_ref: u64,
    scale: f64,
    angle_scale: f64,
    curve: &NurbsCurve,
    unit: &Vector3,
) -> MapResult<(f64, f64)> {
    if bounds.is_empty() {
        return Err(invalid(face_ref, "ADVANCED_FACE has no bounds"));
    }
    let origin = curve.control_points()[0];
    let (profile_lo, profile_hi) = extent_of(curve.control_points(), &origin, unit);

    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for &bound_ref in bounds {
        let (b_lo, b_hi) =
            bound_extent(file, bound_ref, face_ref, scale, angle_scale, &origin, unit)?;
        lo = lo.min(b_lo);
        hi = hi.max(b_hi);
    }
    let (lo, hi) = (lo - profile_hi, hi - profile_lo);

    // NaN-safe: only a strictly positive span is a patch at all.
    if (hi - lo).partial_cmp(&1e-12) != Some(std::cmp::Ordering::Greater) {
        return Err(invalid(
            face_ref,
            "face boundary does not span the extrusion direction",
        ));
    }
    let margin = (hi - lo) * EXTRUSION_SPAN_MARGIN;
    Ok((lo - margin, hi + margin))
}

/// The `(lo, hi)` of `(p - origin)·unit` over every point one `FACE_BOUND`
/// can reach, without discretizing it (the exact path has no polylines).
fn bound_extent(
    file: &StepFile,
    bound_ref: u64,
    referrer: u64,
    scale: f64,
    angle_scale: f64,
    origin: &Point3,
    unit: &Vector3,
) -> MapResult<(f64, f64)> {
    let inst = instance(file, bound_ref, referrer)?;
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

    let loop_inst = instance(file, loop_ref, bound_ref)?;
    // A VERTEX_LOOP is one point — degenerate as a bound, but it still sits
    // on the surface and so still has to be covered.
    if let Some(vertex_loop) = loop_inst.entity.part("VERTEX_LOOP") {
        let point = vertex_point(file, ref_attr(vertex_loop, 1, loop_ref)?, loop_ref, scale)?;
        return Ok(extent_of(&[point], origin, unit));
    }

    let loop_rec = typed_record(file, loop_ref, "EDGE_LOOP", bound_ref)?;
    let mut lo = f64::INFINITY;
    let mut hi = f64::NEG_INFINITY;
    for oe_ref in ref_list(loop_rec, 1, loop_ref)? {
        let oe = typed_record(file, oe_ref, "ORIENTED_EDGE", loop_ref)?;
        let edge_ref = ref_attr(oe, 3, oe_ref)?;
        let edge = typed_record(file, edge_ref, "EDGE_CURVE", oe_ref)?;
        let start = vertex_point(file, ref_attr(edge, 1, edge_ref)?, edge_ref, scale)?;
        let end = vertex_point(file, ref_attr(edge, 2, edge_ref)?, edge_ref, scale)?;
        let geometry = resolve_curve(
            file,
            ref_attr(edge, 3, edge_ref)?,
            edge_ref,
            scale,
            angle_scale,
        )?;
        let (e_lo, e_hi) = curve_extent(&geometry, [start, end], origin, unit);
        lo = lo.min(e_lo);
        hi = hi.max(e_hi);
    }
    if lo > hi {
        return Err(invalid(loop_ref, "EDGE_LOOP has no edges"));
    }
    Ok((lo, hi))
}

/// A `VERTEX_POINT`'s location.
fn vertex_point(file: &StepFile, vertex_ref: u64, referrer: u64, scale: f64) -> MapResult<Point3> {
    let rec = typed_record(file, vertex_ref, "VERTEX_POINT", referrer)?;
    resolve_point(file, ref_attr(rec, 1, vertex_ref)?, vertex_ref, scale)
}

/// The `(lo, hi)` of `(p - origin)·unit` over `points`; `(+inf, -inf)` when
/// empty, so unions compose.
fn extent_of(points: &[Point3], origin: &Point3, unit: &Vector3) -> (f64, f64) {
    points
        .iter()
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), p| {
            let d = (p - origin).dot(unit);
            (lo.min(d), hi.max(d))
        })
}

/// A **containing** bound on `(p - origin)·unit` over an edge's curve,
/// trimmed by its vertices `ends`.
///
/// Never tighter than the truth, and cheap: a segment is its endpoints, a
/// conic section its whole conic (an arc's own trim would be tighter), a
/// NURBS curve its control hull — which contains the curve because
/// [`NurbsCurve`] admits only positive weights. A parabola/hyperbola has no
/// bound without its trim, so its ends stand in — those never reach the
/// exact path anyway, and the mesh path sizes nothing.
fn curve_extent(raw: &RawCurve, ends: [Point3; 2], origin: &Point3, unit: &Vector3) -> (f64, f64) {
    let around = |center: &Point3, reach: f64| {
        let d = (center - origin).dot(unit);
        (d - reach, d + reach)
    };
    match raw {
        RawCurve::Analytic(Curve3::Line { .. }) => extent_of(&ends, origin, unit),
        RawCurve::Analytic(Curve3::Polyline { points, .. }) => extent_of(points, origin, unit),
        RawCurve::Analytic(Curve3::Circle {
            center,
            axis,
            radius,
        }) => {
            // The circle's plane meets `unit` in a line; the reach along it
            // is the radius foreshortened by the axis tilt.
            let tilt = axis.dot(unit).clamp(-1.0, 1.0);
            around(center, radius * (1.0 - tilt * tilt).max(0.0).sqrt())
        }
        RawCurve::Analytic(Curve3::Ellipse {
            center,
            axis,
            major_dir,
            major_radius,
            minor_radius,
        }) => {
            let minor_dir = axis.cross(major_dir);
            let (a, b) = (
                major_radius * major_dir.dot(unit),
                minor_radius * minor_dir.dot(unit),
            );
            around(center, (a * a + b * b).sqrt())
        }
        RawCurve::Analytic(Curve3::Nurbs(curve)) => extent_of(curve.control_points(), origin, unit),
        RawCurve::Nurbs(curve) => extent_of(curve.control_points(), origin, unit),
        RawCurve::Conic(_) => extent_of(&ends, origin, unit),
        RawCurve::Trimmed { basis, .. } | RawCurve::OnSurface { basis } => {
            curve_extent(basis, ends, origin, unit)
        }
        RawCurve::Composite(segments) => segments.iter().fold(
            (f64::INFINITY, f64::NEG_INFINITY),
            |(lo, hi), (segment, _)| {
                let (s_lo, s_hi) = curve_extent(segment, ends, origin, unit);
                (lo.min(s_lo), hi.max(s_hi))
            },
        ),
    }
}

/// A resolved `AXIS1_PLACEMENT`: location plus its axis direction
/// (defaulting to +Z per ISO 10303-42 when unset).
fn resolve_axis1(
    file: &StepFile,
    id: u64,
    referrer: u64,
    scale: f64,
) -> MapResult<(Point3, Vector3)> {
    let rec = typed_record(file, id, "AXIS1_PLACEMENT", referrer)?;
    let location = resolve_point(file, ref_attr(rec, 1, id)?, id, scale)?;
    let axis = match attr(rec, 2, id)? {
        Value::Unset => Vector3::z(),
        Value::Ref(dir) => resolve_direction(file, *dir, id)?,
        _ => {
            return Err(invalid(
                id,
                "AXIS1_PLACEMENT axis is neither $ nor a reference",
            ));
        }
    };
    Ok((location, axis))
}

/// A conic the geometry store has no [`Curve3`] variant for (parabola,
/// hyperbola). Carries the exact STEP parameterization plus a closed-form
/// parameter inverse, so the mesh fallback can trim it by edge vertices;
/// the exact B-Rep path rejects it (degrading the solid to tessellation).
#[derive(Clone)]
enum ConicCurve {
    /// `PARABOLA`: `p(t) = location + focal·t²·x_dir + 2·focal·t·y_dir`
    /// (ISO 10303-42), the axis of symmetry along `x_dir`.
    Parabola {
        location: Point3,
        x_dir: Vector3,
        y_dir: Vector3,
        focal: f64,
    },
    /// `HYPERBOLA`: `p(t) = location + a·cosh(t)·x_dir + b·sinh(t)·y_dir`
    /// (ISO 10303-42), one branch opening along `+x_dir`.
    Hyperbola {
        location: Point3,
        x_dir: Vector3,
        y_dir: Vector3,
        a: f64,
        b: f64,
    },
}

impl ConicCurve {
    fn point(&self, t: f64) -> Point3 {
        match self {
            ConicCurve::Parabola {
                location,
                x_dir,
                y_dir,
                focal,
            } => location + x_dir * (focal * t * t) + y_dir * (2.0 * focal * t),
            ConicCurve::Hyperbola {
                location,
                x_dir,
                y_dir,
                a,
                b,
            } => location + x_dir * (a * t.cosh()) + y_dir * (b * t.sinh()),
        }
    }

    /// Parameter of `p`, assuming it lies on the curve: both conics are
    /// strictly monotonic in their `y_dir` coordinate, so the inverse is
    /// closed-form (no projection iteration needed).
    fn param_of(&self, p: &Point3) -> f64 {
        match self {
            ConicCurve::Parabola {
                location,
                y_dir,
                focal,
                ..
            } => (p - location).dot(y_dir) / (2.0 * focal),
            ConicCurve::Hyperbola {
                location, y_dir, b, ..
            } => ((p - location).dot(y_dir) / b).asinh(),
        }
    }

    /// The same point set traversed the other way (`t → -t` relabeling):
    /// both parameterizations are odd in `y_dir`, so flipping it reverses.
    fn reversed(&self) -> ConicCurve {
        let mut flipped = self.clone();
        match &mut flipped {
            ConicCurve::Parabola { y_dir, .. } | ConicCurve::Hyperbola { y_dir, .. } => {
                *y_dir = -*y_dir;
            }
        }
        flipped
    }
}

/// One parameter-or-point trim bound of a `TRIMMED_CURVE` (STEP allows
/// either or both; cartesian points are preferred as unit-scale-proof).
#[derive(Clone, Copy, Default)]
struct RawTrim {
    point: Option<Point3>,
    param: Option<f64>,
}

/// An edge curve as mapped from STEP (same split as [`RawSurface`]).
enum RawCurve {
    Analytic(Curve3),
    Nurbs(Box<NurbsCurve>),
    /// Parabola/hyperbola: mesh fallback only (no `Curve3` variant).
    Conic(ConicCurve),
    /// `TRIMMED_CURVE`: a basis plus its trim bounds. Edge geometry re-trims
    /// by vertices, so the bounds only matter for bases inside
    /// `COMPOSITE_CURVE` segments (where no vertices exist).
    Trimmed {
        basis: Box<RawCurve>,
        trims: [RawTrim; 2],
        sense: bool,
    },
    /// `COMPOSITE_CURVE`: chained `(parent, same_sense)` segments; mesh
    /// fallback only (sampled into a polyline).
    Composite(Vec<(RawCurve, bool)>),
    /// `SURFACE_CURVE` / `SEAM_CURVE` / `INTERSECTION_CURVE`: a 3D basis
    /// curve that also names the surfaces it lies on, optionally with a
    /// `PCURVE` per surface giving the 2D trim geometry.
    ///
    /// Transparent to the exact path, like [`RawCurve::Trimmed`]: the edge
    /// takes the 3D basis. The associated geometry is validated
    /// ([`validate_associated_geometry`]) but not carried — fin pcurves are
    /// re-derived in the kernel's own parameterization
    /// ([`attach_body_pcurves`]), and the seam an authored
    /// `SEAM_CURVE` marks is equally visible in the topology, as an edge its
    /// face uses twice.
    OnSurface {
        basis: Box<RawCurve>,
    },
}

impl RawCurve {
    /// The curve under any `TRIMMED_CURVE` or `SURFACE_CURVE` wrapping.
    /// Vertex and face bounds re-trim and the surface association says
    /// nothing about the locus, so both wrappers are transparent to every
    /// consumer here.
    fn basis(&self) -> &RawCurve {
        match self {
            RawCurve::Trimmed { basis, .. } | RawCurve::OnSurface { basis } => basis.basis(),
            other => other,
        }
    }

    /// The analytic basis — what the swept-surface reductions care about
    /// (a NURBS basis never collapses to a quadric).
    fn analytic_basis(&self) -> Option<&Curve3> {
        match self.basis() {
            RawCurve::Analytic(curve) => Some(curve),
            _ => None,
        }
    }

    /// The exact [`Curve3`] the geometry store can hold, analytic or NURBS.
    /// `None` for the forms that still have no store representation
    /// (conics, composites) and therefore force the mesh fallback.
    fn exact_curve(&self) -> Option<Curve3> {
        match self.basis() {
            RawCurve::Analytic(curve) => Some(curve.clone()),
            RawCurve::Nurbs(nurbs) => Some(Curve3::nurbs((**nurbs).clone())),
            _ => None,
        }
    }
}

/// One bound of a `TRIMMED_CURVE`: a `(cartesian_point | parameter_value)`
/// select, possibly listing both representations.
fn resolve_trim(
    file: &StepFile,
    value: &Value,
    entity: u64,
    scale: f64,
    param_scale: f64,
) -> MapResult<RawTrim> {
    let mut trim = RawTrim::default();
    let items = match value {
        Value::List(items) => items.as_slice(),
        single => std::slice::from_ref(single),
    };
    for item in items {
        match item {
            Value::Ref(point_ref) => {
                trim.point = Some(resolve_point(file, *point_ref, entity, scale)?);
            }
            Value::Typed { type_name, value } if type_name == "PARAMETER_VALUE" => {
                let t = as_number(value)
                    .ok_or_else(|| invalid(entity, "non-numeric PARAMETER_VALUE trim"))?;
                trim.param = Some(t * param_scale);
            }
            _ => {}
        }
    }
    if trim.point.is_none() && trim.param.is_none() {
        return Err(invalid(
            entity,
            "TRIMMED_CURVE trim carries neither a cartesian point nor a parameter value",
        ));
    }
    Ok(trim)
}

/// Validate one item of a `SURFACE_CURVE`'s associated geometry.
///
/// The item is a `pcurve_or_surface` select: either the surface itself, or a
/// `PCURVE(name, basis_surface, reference_to_curve)` naming that surface
/// alongside the 2D curve tracing the same locus in its parameter space.
/// Either way it must resolve, and a `PCURVE` must actually carry a
/// definitional representation — a file that says an edge has trim geometry
/// and then does not supply it is malformed, and importing it as if it were
/// fine would hide the defect.
///
/// The 2D geometry inside the `DEFINITIONAL_REPRESENTATION` is validated as
/// present but is deliberately not transplanted into the kernel. Two things
/// stand in the way, and both make re-deriving it the honest answer:
///
/// - STEP parameterizes the pcurve in its *basis curve's* parameter, while
///   the kernel re-parameterizes every curve it imports into its own
///   convention (arc length for lines, angle for conics — see
///   [`trim_curve`]). The two do not line up, and a fin's pcurve must line
///   up with its edge exactly (see [`opensolid_brep::pcurve`]).
/// - The authored `(u, v)` values are stated in the STEP surface's frame,
///   whose `ref_direction` the kernel's [`Surface3`] does not keep: a
///   cylinder derives its own angular origin from its axis. Authored `u = 0`
///   is therefore not kernel `u = 0`.
///
/// The one thing projection cannot recover — that a `SEAM_CURVE`'s edge is a
/// parameterization seam, so its two fins need opposite branches — the
/// topology states just as plainly: a seam is an edge its face uses twice.
/// [`attach_body_pcurves`] reads it from there, which also covers the files
/// that author a seam without a `SEAM_CURVE`.
fn validate_associated_geometry(file: &StepFile, id: u64, referrer: u64) -> MapResult<()> {
    let inst = instance(file, id, referrer)?;
    let Some(rec) = inst.as_simple() else {
        // A complex instance here is a surface (surfaces are the entities
        // that get combined), never a PCURVE.
        return Ok(());
    };
    if rec.type_name != "PCURVE" {
        return Ok(());
    }
    // Attribute 1 is the basis surface: checked to resolve, then dropped
    // with the rest of the 2D geometry.
    ref_attr(rec, 1, id)?;
    let definitional = ref_attr(rec, 2, id)?;
    let rep = typed_record(file, definitional, "DEFINITIONAL_REPRESENTATION", id)?;
    if ref_list(rep, 1, definitional)?.is_empty() {
        return Err(invalid(
            definitional,
            "DEFINITIONAL_REPRESENTATION of a PCURVE has no items",
        ));
    }
    Ok(())
}

fn resolve_curve(
    file: &StepFile,
    id: u64,
    referrer: u64,
    scale: f64,
    angle_scale: f64,
) -> MapResult<RawCurve> {
    let inst = instance(file, id, referrer)?;
    // Every B-spline spelling — simple `B_SPLINE_CURVE_WITH_KNOTS`, an
    // implicit-knot subtype, or the complex instance that carries rational
    // weights — resolves through one path.
    if let Some(parts) = BSplineParts::curve(&inst.entity) {
        return Ok(RawCurve::Nurbs(Box::new(resolve_bspline_curve(
            file, &parts, id, scale,
        )?)));
    }
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
        // `PARABOLA(name, position, focal_dist)` — see [`ConicCurve`].
        "PARABOLA" => {
            let p = resolve_axis2(file, ref_attr(rec, 1, id)?, id, scale)?;
            let focal = real_attr(rec, 2, id)? * scale;
            if !(focal > 0.0 && focal.is_finite()) {
                return Err(invalid(id, format!("PARABOLA focal distance {focal} <= 0")));
            }
            let (x_dir, y_dir) = conic_frame(&p, id)?;
            Ok(RawCurve::Conic(ConicCurve::Parabola {
                location: p.location,
                x_dir,
                y_dir,
                focal,
            }))
        }
        // `HYPERBOLA(name, position, semi_axis, semi_imag_axis)`.
        "HYPERBOLA" => {
            let p = resolve_axis2(file, ref_attr(rec, 1, id)?, id, scale)?;
            let a = real_attr(rec, 2, id)? * scale;
            let b = real_attr(rec, 3, id)? * scale;
            if !(a > 0.0 && a.is_finite() && b > 0.0 && b.is_finite()) {
                return Err(invalid(id, format!("HYPERBOLA semi-axes ({a}, {b}) <= 0")));
            }
            let (x_dir, y_dir) = conic_frame(&p, id)?;
            Ok(RawCurve::Conic(ConicCurve::Hyperbola {
                location: p.location,
                x_dir,
                y_dir,
                a,
                b,
            }))
        }
        // `TRIMMED_CURVE(name, basis_curve, trim_1, trim_2, sense_agreement,
        // master_representation)`. The basis resolves recursively; trims are
        // kept for bases that have no vertices to re-trim by (composite
        // segments). Parameter trims on circles/ellipses are plane-angle
        // measures and scale into radians; conic (parabola/hyperbola) and
        // B-spline parameters are unit-free.
        "TRIMMED_CURVE" => {
            let basis = resolve_curve(file, ref_attr(rec, 1, id)?, id, scale, angle_scale)?;
            let param_scale = match &basis {
                RawCurve::Analytic(Curve3::Circle { .. } | Curve3::Ellipse { .. }) => angle_scale,
                _ => 1.0,
            };
            let trims = [
                resolve_trim(file, attr(rec, 2, id)?, id, scale, param_scale)?,
                resolve_trim(file, attr(rec, 3, id)?, id, scale, param_scale)?,
            ];
            let sense = bool_attr(rec, 4, id)?;
            Ok(RawCurve::Trimmed {
                basis: Box::new(basis),
                trims,
                sense,
            })
        }
        // `COMPOSITE_CURVE(name, segments, self_intersect)`; each segment is
        // a `COMPOSITE_CURVE_SEGMENT(transition, same_sense, parent_curve)`.
        "COMPOSITE_CURVE" => {
            let mut segments = Vec::new();
            for seg_ref in ref_list(rec, 1, id)? {
                let seg = typed_record(file, seg_ref, "COMPOSITE_CURVE_SEGMENT", id)?;
                let same_sense = bool_attr(seg, 1, seg_ref)?;
                let parent = resolve_curve(
                    file,
                    ref_attr(seg, 2, seg_ref)?,
                    seg_ref,
                    scale,
                    angle_scale,
                )?;
                segments.push((parent, same_sense));
            }
            if segments.is_empty() {
                return Err(invalid(id, "COMPOSITE_CURVE has no segments"));
            }
            Ok(RawCurve::Composite(segments))
        }
        // `SURFACE_CURVE(name, curve_3d, associated_geometry,
        // master_representation)`, and its two subtypes: `SEAM_CURVE`
        // (associated geometry is two pcurves on the *same* surface, which
        // is what marks the edge as a parameterization seam) and
        // `INTERSECTION_CURVE`. Every one of them is a 3D curve that also
        // knows which surfaces it lies on, so the edge takes `curve_3d` and
        // the associated geometry only tells the fins where they sit in
        // parameter space.
        "SURFACE_CURVE" | "SEAM_CURVE" | "INTERSECTION_CURVE" => {
            let basis = resolve_curve(file, ref_attr(rec, 1, id)?, id, scale, angle_scale)?;
            for item in list_attr(rec, 2, id)? {
                let Some(item_ref) = item.as_ref_id() else {
                    return Err(invalid(
                        id,
                        format!("{} associated geometry is not a reference", rec.type_name),
                    ));
                };
                validate_associated_geometry(file, item_ref, id)?;
            }
            Ok(RawCurve::OnSurface {
                basis: Box::new(basis),
            })
        }
        other => Err(unsupported(id, format!("curve type {other}"))),
    }
}

/// Orthonormal in-plane frame of a conic's placement: `x_dir` from the
/// placement's ref_direction (Gram-Schmidt against the axis, defaulting to
/// [`plane_basis`]), `y_dir = axis × x_dir`.
pub(super) fn conic_frame(p: &Placement, entity: u64) -> MapResult<(Vector3, Vector3)> {
    let axis_norm = p.axis.norm();
    if axis_norm == 0.0 || !axis_norm.is_finite() {
        return Err(invalid(entity, "conic placement axis is degenerate"));
    }
    let unit_axis = p.axis / axis_norm;
    let x_raw = p.ref_dir.unwrap_or_else(|| plane_basis(&unit_axis).0);
    let x_planar = x_raw - unit_axis * x_raw.dot(&unit_axis);
    let x_norm = x_planar.norm();
    if x_norm < 1e-12 {
        return Err(invalid(
            entity,
            "conic placement ref_direction is parallel to its axis",
        ));
    }
    let x_dir = x_planar / x_norm;
    Ok((x_dir, unit_axis.cross(&x_dir)))
}

/// A B-spline subtype that derives its knots from the degree and the
/// control-point count instead of listing them (ISO 10303-42 §4.4.42–44).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum ImplicitKnots {
    /// `quasi_uniform_*`: clamped ends, interior knots one apart.
    QuasiUniform,
    /// `uniform_*`: every multiplicity 1, running `-degree ..= count`.
    Uniform,
    /// `bezier_*`: a single span over `[0, 1]`.
    Bezier,
}

/// Where a B-spline record keeps its pieces.
///
/// AP203 spells a B-spline either as one simple record — `B_SPLINE_CURVE_-
/// WITH_KNOTS(name, degree, control_points, …, multiplicities, knots, spec)`
/// — or as a complex instance whose parts split the same data across the
/// supertype chain:
///
/// ```text
/// ( BOUNDED_CURVE() B_SPLINE_CURVE(degree, control_points, form, closed,
///   self_intersect) B_SPLINE_CURVE_WITH_KNOTS(mults, knots, spec) CURVE()
///   GEOMETRIC_REPRESENTATION_ITEM() RATIONAL_B_SPLINE_CURVE(weights)
///   REPRESENTATION_ITEM(name) )
/// ```
///
/// Partial records carry only their own declared attributes, so every index
/// shifts by one against the simple form — there is no inherited `name` in
/// front, the complex form having moved it to `REPRESENTATION_ITEM`.
/// `base_offset` absorbs that shift so one resolver serves both spellings.
/// The `RATIONAL_` part is optional: exporters occasionally spell a
/// non-rational curve this way, and unit weights are then the right
/// reading. Surfaces follow the same pattern with `_SURFACE` names, a
/// `u`/`v` degree pair, and grid-shaped control points and weights.
struct BSplineParts<'a> {
    /// Record carrying the degree(s) and the control points.
    base: &'a SimpleRecord,
    /// Index of the first degree attribute in `base`.
    base_offset: usize,
    /// Record carrying the multiplicity and knot lists, plus the index of
    /// the first multiplicity list — absent for the implicit-knot subtypes.
    knots: Option<(&'a SimpleRecord, usize)>,
    /// The implicit-knot subtype, when `knots` is absent.
    implicit: Option<ImplicitKnots>,
    /// The `RATIONAL_*` weights record. Only complex instances have one: a
    /// rational B-spline is always multiply inherited.
    weights: Option<&'a SimpleRecord>,
}

/// The curve subtypes whose knots are implicit, in the order they are
/// probed. Mirrored by [`SURFACE_IMPLICIT`].
const CURVE_IMPLICIT: [(&str, ImplicitKnots); 3] = [
    ("QUASI_UNIFORM_CURVE", ImplicitKnots::QuasiUniform),
    ("UNIFORM_CURVE", ImplicitKnots::Uniform),
    ("BEZIER_CURVE", ImplicitKnots::Bezier),
];

const SURFACE_IMPLICIT: [(&str, ImplicitKnots); 3] = [
    ("QUASI_UNIFORM_SURFACE", ImplicitKnots::QuasiUniform),
    ("UNIFORM_SURFACE", ImplicitKnots::Uniform),
    ("BEZIER_SURFACE", ImplicitKnots::Bezier),
];

impl<'a> BSplineParts<'a> {
    /// Locate a curve's pieces, or `None` if `entity` is not a B-spline
    /// curve in any of its spellings.
    fn curve(entity: &'a EntityRecord) -> Option<Self> {
        Self::find(
            entity,
            "B_SPLINE_CURVE",
            "B_SPLINE_CURVE_WITH_KNOTS",
            "RATIONAL_B_SPLINE_CURVE",
            &CURVE_IMPLICIT,
            6,
        )
    }

    /// Locate a surface's pieces, or `None` if `entity` is not a B-spline
    /// surface in any of its spellings.
    fn surface(entity: &'a EntityRecord) -> Option<Self> {
        Self::find(
            entity,
            "B_SPLINE_SURFACE",
            "B_SPLINE_SURFACE_WITH_KNOTS",
            "RATIONAL_B_SPLINE_SURFACE",
            &SURFACE_IMPLICIT,
            8,
        )
    }

    /// `simple_knot_index` is where the multiplicity lists start in the
    /// single-record spelling (after `name`, the degrees, the control
    /// points and the form/closed/self-intersect flags).
    fn find(
        entity: &'a EntityRecord,
        base_type: &str,
        with_knots_type: &str,
        rational_type: &str,
        implicit_types: &[(&str, ImplicitKnots)],
        simple_knot_index: usize,
    ) -> Option<Self> {
        match entity {
            EntityRecord::Simple(rec) => {
                let implicit = implicit_types
                    .iter()
                    .find(|(name, _)| *name == rec.type_name)
                    .map(|&(_, form)| form);
                if implicit.is_none() && rec.type_name != with_knots_type {
                    return None;
                }
                Some(Self {
                    base: rec,
                    base_offset: 1,
                    knots: implicit.is_none().then_some((rec, simple_knot_index)),
                    implicit,
                    weights: None,
                })
            }
            EntityRecord::Complex(_) => {
                let base = entity.part(base_type)?;
                let knots = entity.part(with_knots_type).map(|rec| (rec, 0));
                let implicit = implicit_types
                    .iter()
                    .find(|(name, _)| entity.part(name).is_some())
                    .map(|&(_, form)| form);
                // A complex instance that inherits `B_SPLINE_*` but states
                // its knots neither way is not one we can parameterize.
                if knots.is_none() && implicit.is_none() {
                    return None;
                }
                Some(Self {
                    base,
                    base_offset: 0,
                    knots,
                    implicit,
                    weights: entity.part(rational_type),
                })
            }
        }
    }
}

/// Derive the knot vector of an implicit-knot B-spline subtype
/// (ISO 10303-42 §4.4.42–44) from its degree and control-point count.
fn implicit_knot_vector(
    form: ImplicitKnots,
    degree: i64,
    count: usize,
    entity: u64,
) -> MapResult<KnotVector> {
    if degree < 1 {
        return Err(invalid(entity, format!("B-spline degree {degree} < 1")));
    }
    let degree = degree as usize;
    if count < degree + 1 {
        return Err(invalid(
            entity,
            format!("{count} control points cannot carry a degree-{degree} B-spline"),
        ));
    }
    let knots: Vec<f64> = match form {
        ImplicitKnots::Bezier => {
            if count != degree + 1 {
                return Err(invalid(
                    entity,
                    format!(
                        "Bezier form has {count} control points, expected {}",
                        degree + 1
                    ),
                ));
            }
            let mut knots = vec![0.0; degree + 1];
            knots.extend(std::iter::repeat_n(1.0, degree + 1));
            knots
        }
        // Clamped, with one knot per interior span boundary: the domain is
        // `[0, count - degree]`, one unit per span.
        ImplicitKnots::QuasiUniform => {
            let spans = count - degree;
            let mut knots = vec![0.0; degree + 1];
            knots.extend((1..spans).map(|i| i as f64));
            knots.extend(std::iter::repeat_n(spans as f64, degree + 1));
            knots
        }
        // Unclamped: the first `degree` knots sit below the domain, so the
        // curve starts short of its first control point.
        ImplicitKnots::Uniform => (0..count + degree + 1)
            .map(|i| i as f64 - degree as f64)
            .collect(),
    };
    KnotVector::new(degree, knots).map_err(|e| nurbs_error(entity, &e))
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

/// `B_SPLINE_CURVE(degree, control_points, form, closed, self_intersect)`
/// plus, per [`BSplineParts`], the knots (explicit or derived) and the
/// optional rational weights.
fn resolve_bspline_curve(
    file: &StepFile,
    parts: &BSplineParts,
    id: u64,
    scale: f64,
) -> MapResult<NurbsCurve> {
    let base = parts.base;
    let offset = parts.base_offset;
    let degree = int_attr(base, offset, id)?;
    let control_points = ref_list(base, offset + 1, id)?
        .into_iter()
        .map(|p| resolve_point(file, p, id, scale))
        .collect::<MapResult<Vec<_>>>()?;
    let kv = match parts.knots {
        Some((rec, first)) => {
            let multiplicities = int_list(rec, first, id)?;
            let knots = real_list(rec, first + 1, id)?;
            knot_vector(degree, &knots, &multiplicities, id)?
        }
        None => implicit_knot_vector(
            parts.implicit.expect("knots are explicit or implicit"),
            degree,
            control_points.len(),
            id,
        )?,
    };
    match parts.weights {
        Some(rec) => NurbsCurve::new(control_points, real_list(rec, 0, id)?, kv),
        None => NurbsCurve::bspline(control_points, kv),
    }
    .map_err(|e| nurbs_error(id, &e))
}

/// A 2D `CARTESIAN_POINT` — the coordinates inside a `PCURVE`'s
/// definitional representation. Surface parameters carry no length unit,
/// so nothing scales.
fn resolve_point_2d(file: &StepFile, id: u64, referrer: u64) -> MapResult<Point2> {
    let rec = typed_record(file, id, "CARTESIAN_POINT", referrer)?;
    let items = list_attr(rec, 1, id)?;
    if items.len() != 2 {
        return Err(invalid(
            id,
            format!(
                "a parameter-space CARTESIAN_POINT expects 2 coordinates, found {}",
                items.len()
            ),
        ));
    }
    let mut out = [0.0; 2];
    for (slot, item) in out.iter_mut().zip(items) {
        *slot = as_number(item).ok_or_else(|| invalid(id, "non-numeric coordinate"))?;
    }
    Ok(Point2::new(out[0], out[1]))
}

/// The 2D B-spline inside a `PCURVE`'s definitional representation, mapped
/// verbatim — the same spellings [`resolve_bspline_curve`] accepts, with 2D
/// control points and no unit scale.
fn resolve_bspline_curve_2d(file: &StepFile, id: u64, referrer: u64) -> MapResult<NurbsCurve2> {
    let inst = instance(file, id, referrer)?;
    let Some(parts) = BSplineParts::curve(&inst.entity) else {
        return Err(unsupported(id, "authored pcurve geometry is not a B-spline"));
    };
    let base = parts.base;
    let offset = parts.base_offset;
    let degree = int_attr(base, offset, id)?;
    let control_points = ref_list(base, offset + 1, id)?
        .into_iter()
        .map(|p| resolve_point_2d(file, p, id))
        .collect::<MapResult<Vec<_>>>()?;
    let kv = match parts.knots {
        Some((rec, first)) => {
            let multiplicities = int_list(rec, first, id)?;
            let knots = real_list(rec, first + 1, id)?;
            knot_vector(degree, &knots, &multiplicities, id)?
        }
        None => implicit_knot_vector(
            parts.implicit.expect("knots are explicit or implicit"),
            degree,
            control_points.len(),
            id,
        )?,
    };
    match parts.weights {
        Some(rec) => NurbsCurve2::new(control_points, real_list(rec, 0, id)?, kv),
        None => NurbsCurve2::bspline(control_points, kv),
    }
    .map_err(|e| nurbs_error(id, &e))
}

/// A rectangular list-of-lists of `cartesian_point` references at `index`,
/// resolved row by row. The outer index runs over `u`.
fn resolve_control_grid(
    file: &StepFile,
    rec: &SimpleRecord,
    index: usize,
    id: u64,
    scale: f64,
) -> MapResult<Vec<Vec<Point3>>> {
    let rows = list_attr(rec, index, id)?;
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
    Ok(grid)
}

/// A rectangular list-of-lists of reals at `index` (a rational patch's
/// weight grid, shaped like its control grid).
fn resolve_weight_grid(rec: &SimpleRecord, index: usize, id: u64) -> MapResult<Vec<Vec<f64>>> {
    list_attr(rec, index, id)?
        .iter()
        .map(|row| {
            let cells = row
                .as_list()
                .ok_or_else(|| invalid(id, "weight grid row is not a list"))?;
            cells
                .iter()
                .map(|cell| {
                    as_number(cell).ok_or_else(|| invalid(id, "weight grid cell is not a number"))
                })
                .collect::<MapResult<Vec<_>>>()
        })
        .collect()
}

/// `B_SPLINE_SURFACE(u_degree, v_degree, control_grid, form, u_closed,
/// v_closed, self_intersect)` plus, per [`BSplineParts`], the knots
/// (explicit or derived) and the optional rational weight grid. The control
/// grid's outer index runs over `u`.
fn resolve_bspline_surface(
    file: &StepFile,
    parts: &BSplineParts,
    id: u64,
    scale: f64,
) -> MapResult<NurbsSurface> {
    let base = parts.base;
    let offset = parts.base_offset;
    let u_degree = int_attr(base, offset, id)?;
    let v_degree = int_attr(base, offset + 1, id)?;
    let grid = resolve_control_grid(file, base, offset + 2, id, scale)?;
    let (kv_u, kv_v) = match parts.knots {
        Some((rec, first)) => {
            let u_mults = int_list(rec, first, id)?;
            let v_mults = int_list(rec, first + 1, id)?;
            let u_knots = real_list(rec, first + 2, id)?;
            let v_knots = real_list(rec, first + 3, id)?;
            (
                knot_vector(u_degree, &u_knots, &u_mults, id)?,
                knot_vector(v_degree, &v_knots, &v_mults, id)?,
            )
        }
        None => {
            let form = parts.implicit.expect("knots are explicit or implicit");
            let cols = grid.first().map_or(0, Vec::len);
            (
                implicit_knot_vector(form, u_degree, grid.len(), id)?,
                implicit_knot_vector(form, v_degree, cols, id)?,
            )
        }
    };
    match parts.weights {
        Some(rec) => NurbsSurface::new(grid, resolve_weight_grid(rec, 0, id)?, kv_u, kv_v),
        None => NurbsSurface::bspline(grid, kv_u, kv_v),
    }
    .map_err(|e| nurbs_error(id, &e))
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
    /// How far the curve's start point actually lands from the start
    /// vertex's point, and likewise for the end. Non-zero for any file whose
    /// vertices and curves were rounded independently, which is most of
    /// them; [`verify_trim`] bounds it and the vertex then carries it.
    start_residual: f64,
    end_residual: f64,
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
        // Not produced by the reader (marched SSI geometry only), but the
        // reversal is well-defined: walk the vertices backwards.
        Curve3::Polyline { points, closed } => Curve3::Polyline {
            points: points.iter().rev().copied().collect(),
            closed: *closed,
        },
        // Control points, weights and knots all reflect about the domain
        // midpoint, so the locus and the domain are both preserved.
        Curve3::Nurbs(nurbs) => Curve3::nurbs(nurbs.reversed()),
    }
}

/// Angle parameter of `p` on a conic, in the curve's own frame, wrapped to
/// `[0, 2π)`. `None` for every variant not parameterized by an angle.
fn conic_angle(curve: &Curve3, p: &Point3) -> Option<f64> {
    match curve {
        Curve3::Line { .. } | Curve3::Polyline { .. } | Curve3::Nurbs(_) => None,
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

/// How far apart two of an edge's points may be and still count as the
/// same point. Grows with distance from the origin, because the decimals
/// STEP writes lose absolute precision as coordinates grow.
fn trim_tol(a: Point3, b: Point3) -> f64 {
    TRIM_TOL_REL * (1.0 + a.coords.norm().max(b.coords.norm()))
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
        // A freeform curve has no closed form to invert, so the vertices
        // are located by projection; `verify_trim` below then confirms the
        // recovered parameters really do land on them, which is what
        // catches a vertex that is off the curve entirely. A closed edge
        // takes the whole knot interval — a clamped B-spline traces its
        // locus exactly once, so there is no period to add.
        Curve3::Nurbs(nurbs) => {
            if closed {
                nurbs.knot_vector().domain()
            } else {
                let t0 = nurbs.project_point(&start).t;
                let t1 = nurbs.project_point(&end).t;
                if (t1 - t0).partial_cmp(&SYSTEM_RESOLUTION) != Some(std::cmp::Ordering::Greater) {
                    return Err(invalid(
                        entity,
                        "edge endpoints do not advance along its B-spline (check same_sense)",
                    ));
                }
                (t0, t1)
            }
        }
        conic => {
            // Only the angle-parameterized variants are left. `Polyline`
            // reaches here only if some future caller hands one in — the
            // reader never builds one — so refuse rather than panic on
            // input of unknown provenance.
            let Some(t0) = conic_angle(conic, &start) else {
                return Err(invalid(
                    entity,
                    "edge geometry has no angular parameterization to trim by",
                ));
            };
            // A full circle has two spellings in STEP. The tidy one names a
            // single vertex twice, which is what `closed` reports. The other
            // names two distinct `VERTEX_POINT` entities that happen to sit
            // on the same point — what OCC writes when the circle is tangent
            // to a neighbouring face, because the tangency is a real corner
            // of the model and the seam has to land on it. Both spellings
            // mean the same thing: the edge sweeps a whole period. Reading
            // the second one literally gives a zero sweep, and an edge that
            // swept nothing would not be an edge at all.
            if closed || (end - start).norm() <= trim_tol(start, end) {
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
    let mut trimmed = TrimmedCurve {
        curve: oriented,
        t_start,
        t_end,
        start_residual: 0.0,
        end_residual: 0.0,
    };
    let (start_residual, end_residual) = verify_trim(&trimmed, start, end, entity)?;
    trimmed.start_residual = start_residual;
    trimmed.end_residual = end_residual;
    Ok(trimmed)
}

/// Verify the trimmed curve interpolates the edge's vertex points; catches
/// vertices off their curve (wrong radius, off-plane, bad `same_sense`).
/// Returns how far it misses each of them by, which the caller records as
/// the vertices' tolerance.
///
/// The two bounds are different questions. `TRIM_TOL_REL` asks whether the
/// vertex is on this curve *at all* — past it, the record is describing some
/// other edge and the file is wrong, not imprecise. [`MAX_ALLOWED_TOLERANCE`]
/// asks whether a vertex carrying the miss is an entity the kernel will
/// accept; past that, `check` would reject the imported body for tolerance
/// alone, so the exact path declines it here and the solid tessellates
/// instead. On a model small enough for the trim bound to be the tighter of
/// the two — anything under ten metres at millimetre scale — only the first
/// can fire.
fn verify_trim(
    trimmed: &TrimmedCurve,
    start: Point3,
    end: Point3,
    entity: u64,
) -> MapResult<(f64, f64)> {
    let tol = trim_tol(start, end);
    let at_start = trimmed.curve.point(trimmed.t_start);
    let at_end = trimmed.curve.point(trimmed.t_end);
    let (start_residual, end_residual) = ((at_start - start).norm(), (at_end - end).norm());
    // NaN-safe: a non-finite residual fails both comparisons below.
    if !(start_residual <= tol && end_residual <= tol) {
        return Err(invalid(
            entity,
            "edge geometry does not pass through the edge's vertex points",
        ));
    }
    if start_residual.max(end_residual) > MAX_ALLOWED_TOLERANCE {
        return Err(invalid(
            entity,
            "edge geometry misses its vertex points by more than the kernel's \
             maximum tolerance",
        ));
    }
    Ok((start_residual, end_residual))
}

// ---------------------------------------------------------------------
// Exact B-Rep mapping
// ---------------------------------------------------------------------

/// Everything created in the stores for one solid, for rollback when the
/// solid falls back or fails.
#[derive(Default)]
struct Created {
    body: Option<EntityId<Body>>,
    shells: Vec<EntityId<Shell>>,
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
    for &id in &created.shells {
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
    // Pcurves need no rollback: they are attached only once a body is known
    // good (see `finish_exact_body`), which is past every rollback path.
}

/// What one face bound's loop contributes to the face: a cycle of directed
/// edges (`EDGE_LOOP`), or the single vertex of a degenerate `VERTEX_LOOP`.
enum BoundLoop {
    Edges(Vec<(EntityId<Edge>, FinSense)>),
    Vertex(EntityId<Vertex>),
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
    /// Faces with more than one real bound, so which one is the outer bound
    /// was an open question at mapping time. [`choose_outer_bounds`] settles
    /// it from parameter-space area once pcurves exist.
    multi_bound_faces: Vec<EntityId<Face>>,
    /// `VERTEX_POINT` #id → mapped vertex (shared between edges).
    vertices: HashMap<u64, EntityId<Vertex>>,
    /// `EDGE_CURVE` #id → mapped edge (shared between faces, so mated
    /// fins arise naturally when both faces reference the same edge).
    edges: HashMap<u64, EntityId<Edge>>,
    /// One entry per (loop, edge) where the loop walks that edge twice — a
    /// seam. Settled by [`split_overshared_seams`](Self::split_overshared_seams)
    /// once the whole body is mapped and the edge's full fin count is known.
    seams: Vec<(EntityId<Loop>, EntityId<Edge>)>,
    /// Authored 2D trim geometry per freeform edge, keyed by the STEP id of
    /// the `PCURVE`'s basis surface, already reoriented to the edge's
    /// direction of travel. Candidates for
    /// [`transplant_authored_pcurves`] once the body's own pcurves exist.
    authored_pcurves: HashMap<EntityId<Edge>, Vec<(u64, NurbsCurve2)>>,
    /// Each face's surface as the file named it, so an authored pcurve's
    /// basis-surface reference can be matched to the fins it trims.
    face_surface_refs: HashMap<EntityId<Face>, u64>,
}

impl SolidBuilder<'_> {
    /// Build the body: the outer `CLOSED_SHELL`, then one inner shell per
    /// `BREP_WITH_VOIDS` void. A void arrives as an `ORIENTED_CLOSED_SHELL`
    /// whose flag says whether the underlying shell is used as authored;
    /// a `false` flag reverses every face, so the stored (effective) void
    /// boundary always has its normals pointing into the cavity — which is
    /// what [`ShellOrientation::Inward`] documents.
    fn build(
        &mut self,
        msb_id: u64,
        shell_ref: u64,
        voids: &[(u64, bool)],
    ) -> MapResult<EntityId<Body>> {
        let body = self.store.create_body(BodyType::Solid);
        self.created.body = Some(body);
        self.build_shell(msb_id, shell_ref, ShellOrientation::Outward, false, body)?;
        for &(void_ref, as_authored) in voids {
            self.build_shell(
                msb_id,
                void_ref,
                ShellOrientation::Inward,
                !as_authored,
                body,
            )?;
        }

        self.split_overshared_seams();

        // STEP carries no genus; recover it from the Euler-Poincaré formula
        // so `check` validates the imported topology's own consistency (an
        // odd or negative implied genus still fails the formula). The healer
        // re-runs this after any repair that changes the counts.
        super::heal::recover_genus(self.store, body);
        Ok(body)
    }

    /// Give a seam its own edge when the file hung other faces on it too.
    ///
    /// A seam is private to the face it opens: the cylinder's `EDGE_CURVE`
    /// spelled twice in one loop is a cut in that face's parameterization,
    /// not a boundary it shares with anything. Usually nothing else names
    /// that `EDGE_CURVE` and the two fins make an ordinary two-sided edge.
    /// But when the seam falls on a line that is *also* a real boundary —
    /// which is what a tangent boolean produces, the cylinder's seam landing
    /// exactly on the line where the block's wall was split by the tangency —
    /// OCC writes one `EDGE_CURVE` for both and the edge collects four fins.
    /// Read literally that is a non-manifold edge and `check` refuses the
    /// body, though nothing about the solid is actually ambiguous: two
    /// two-sided sheets happen to meet along one locus.
    ///
    /// So the seam moves onto its own copy of the edge, leaving the shared
    /// boundary with exactly the two fins it always had. The topology is the
    /// same solid; only the double-booking is undone. OCC keeps the single
    /// edge, so our edge count comes out one higher per split.
    fn split_overshared_seams(&mut self) {
        for (loop_id, edge_id) in std::mem::take(&mut self.seams) {
            let Some(edge) = self.store.edge(edge_id) else {
                continue;
            };
            // Two fins is a seam nobody else touched — the common case, and
            // already manifold. Only over-sharing needs undoing.
            if edge.fins.len() <= 2 {
                continue;
            }
            let (start, end, tolerance, t_start, t_end) = (
                edge.start_vertex,
                edge.end_vertex,
                edge.tolerance,
                edge.t_start,
                edge.t_end,
            );
            let Some(curve) = edge.curve.and_then(|c| self.geo.curve(c)).cloned() else {
                continue;
            };
            let Some(loop_ref) = self.store.loop_(loop_id) else {
                continue;
            };
            let moved: Vec<EntityId<Fin>> = loop_ref
                .fins
                .iter()
                .copied()
                .filter(|&fin| self.store.fin(fin).is_some_and(|fin| fin.edge == edge_id))
                .collect();
            // The seam is exactly the pair; anything else is a shape this
            // repair does not understand, so leave it for `check` to report.
            if moved.len() != 2 {
                continue;
            }

            let curve_id = self.geo.add_curve(curve);
            self.created.curves.push(curve_id);
            let copy = self
                .store
                .create_edge_with_curve(start, end, tolerance, curve_id, t_start, t_end);
            self.created.edges.push(copy);

            for &fin in &moved {
                self.store.fins.get_mut(fin).expect("live fin").edge = copy;
            }
            let old = self.store.edges.get_mut(edge_id).expect("live edge");
            old.fins.retain(|fin| !moved.contains(fin));
            let remaining = old.fins.clone();
            self.store.edges.get_mut(copy).expect("just created").fins = moved.clone();
            // Both edges lost or gained a side, so re-mate from scratch:
            // a pair mates, anything else is left open for `check`.
            for group in [moved, remaining] {
                let pair = (group.len() == 2).then(|| (group[0], group[1]));
                for &fin in &group {
                    self.store.fins.get_mut(fin).expect("live fin").mate = match pair {
                        Some((a, b)) if fin == a => Some(b),
                        Some((a, b)) if fin == b => Some(a),
                        _ => None,
                    };
                }
            }
        }
    }

    fn build_shell(
        &mut self,
        msb_id: u64,
        shell_ref: u64,
        orientation: ShellOrientation,
        flip: bool,
        body: EntityId<Body>,
    ) -> MapResult<()> {
        let shell_rec = typed_record(self.file, shell_ref, "CLOSED_SHELL", msb_id)?;
        let face_refs = ref_list(shell_rec, 1, shell_ref)?;
        if face_refs.is_empty() {
            return Err(invalid(shell_ref, "CLOSED_SHELL has no faces"));
        }
        let shell = self.store.create_shell(body, true, orientation);
        self.created.shells.push(shell);
        for face_ref in face_refs {
            self.map_face(shell, face_ref, shell_ref, flip)?;
        }
        Ok(())
    }

    /// Map one `ADVANCED_FACE` onto `shell`. `flip` reverses the face — the
    /// surface sense and every bound's traversal — for void shells whose
    /// `ORIENTED_CLOSED_SHELL` flag negates the authored orientation.
    fn map_face(
        &mut self,
        shell: EntityId<Shell>,
        face_ref: u64,
        referrer: u64,
        flip: bool,
    ) -> MapResult<()> {
        let rec = typed_record(self.file, face_ref, "ADVANCED_FACE", referrer)?;
        let bounds = ref_list(rec, 1, face_ref)?;
        let surface_ref = ref_attr(rec, 2, face_ref)?;
        let same_sense = bool_attr(rec, 3, face_ref)?;

        // Surface first: an unmappable surface is the more fundamental
        // finding than any bound-level problem.
        let surface = match resolve_surface(
            self.file,
            surface_ref,
            face_ref,
            self.scale,
            self.angle_scale,
        )? {
            RawSurface::Analytic(surface) => surface,
            RawSurface::Nurbs(nurbs) => Surface3::nurbs(*nurbs),
            // Sized here rather than in `resolve_surface`: only the face
            // knows how far along the sweep the patch has to reach.
            RawSurface::ExtrudedNurbs { curve, dir } => Surface3::nurbs(extruded_nurbs_surface(
                self.file,
                &bounds,
                face_ref,
                self.scale,
                self.angle_scale,
                &curve,
                &dir,
            )?),
            RawSurface::Extruded { .. } | RawSurface::Revolved { .. } => {
                return Err(unsupported(
                    surface_ref,
                    "exact swept-surface import (no analytic reduction applies); \
                     falling back to tessellation",
                ));
            }
        };
        if bounds.is_empty() {
            return Err(invalid(face_ref, "ADVANCED_FACE has no bounds"));
        }

        // At most one FACE_OUTER_BOUND; without one, the first bound plays
        // the outer role (AP203 permits plain FACE_BOUNDs only).
        //
        // Neither answer is trustworthy on a face with holes, and the NIST
        // corpus fails both ways: nist_ctc_01/nist_ctc_03/nist_ftc_06 tag no
        // outer bound at all on any of their 108 multi-bound faces, so "the
        // first" decides and lands on a hole 38 times; nist_stc_06 tags all
        // 64 of its multi-bound faces and points at a hole on 23 of them.
        // So a face with a real choice to make is noted here and the choice
        // re-decided by area in `choose_outer_bounds`, once the pcurves that
        // make its loops measurable exist. A tag that is right survives that
        // — the outer bound encloses the most area, so measuring agrees with
        // it — and one that is wrong does not. The mesh fallback has always
        // worked this way (`mesh_planar_face` swaps its widest ring to the
        // front rather than trusting the tag); this brings the exact path
        // into line with it.
        //
        // A degenerate VERTEX_LOOP has no extent and so cannot bound the
        // face's region: whenever a real loop is available it takes the outer
        // role, whatever the file tags. A face bounded *only* by vertex loops
        // still needs an outer loop, so there the first bound plays it (as the
        // sweep constructors do for a pole face).
        let mut flagged_outer = None;
        let mut degenerate = Vec::with_capacity(bounds.len());
        for (i, &bound_ref) in bounds.iter().enumerate() {
            let inst = instance(self.file, bound_ref, face_ref)?;
            if inst.entity.part("FACE_OUTER_BOUND").is_some() {
                if flagged_outer.is_some() {
                    return Err(invalid(face_ref, "face has multiple FACE_OUTER_BOUNDs"));
                }
                flagged_outer = Some(i);
            }
            let (loop_ref, _) = self.resolve_bound(bound_ref, face_ref)?;
            degenerate.push(self.is_vertex_loop(loop_ref, bound_ref)?);
        }
        let first_real = degenerate.iter().position(|&d| !d);
        let outer_index = match flagged_outer {
            Some(i) if !degenerate[i] => i,
            _ => first_real.unwrap_or(0),
        };

        let sense = if same_sense != flip {
            FaceSense::Positive
        } else {
            FaceSense::Negative
        };
        let face = self.store.create_face(shell, sense);
        self.created.faces.push(face);
        self.face_surface_refs.insert(face, surface_ref);
        // Only a face with two real bounds to choose between has anything to
        // re-decide; one bound (with or without vertex loops beside it) is
        // the outer one whatever the file says.
        if degenerate.iter().filter(|&&d| !d).count() > 1 {
            self.multi_bound_faces.push(face);
        }
        let surface_id = self.geo.add_surface(surface);
        self.created.surfaces.push(surface_id);
        self.store
            .faces
            .get_mut(face)
            .expect("just created")
            .surface = Some(surface_id);

        for (i, &bound_ref) in bounds.iter().enumerate() {
            let (loop_ref, orientation) = self.resolve_bound(bound_ref, face_ref)?;
            let is_outer = i == outer_index;
            let loop_id = match self.map_loop(loop_ref, bound_ref)? {
                BoundLoop::Vertex(vertex) => {
                    // A VERTEX_LOOP is orientation-free: a single point has
                    // no traversal to reverse, so `flip` does not apply.
                    self.store
                        .create_vertex_loop(face, LoopType::Vertex, vertex, is_outer)
                }
                BoundLoop::Edges(mut loop_edges) => {
                    if orientation == flip {
                        loop_edges.reverse();
                        for (_, sense) in &mut loop_edges {
                            *sense = sense.opposite();
                        }
                    }
                    let loop_type = if is_outer {
                        LoopType::Outer
                    } else {
                        LoopType::Inner
                    };
                    let loop_id = self.store.create_loop(face, loop_type, &loop_edges);
                    // An edge this loop walks twice is a seam: the cut that
                    // opens a closed surface into a rectangle. Note it now,
                    // decide later — whether it needs an edge of its own
                    // depends on what the rest of the shell does with it.
                    for (i, &(edge, _)) in loop_edges.iter().enumerate() {
                        if loop_edges[..i].iter().any(|&(e, _)| e == edge) {
                            self.seams.push((loop_id, edge));
                        }
                    }
                    loop_id
                }
            };
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

    /// Whether the loop named by `loop_ref` is a degenerate `VERTEX_LOOP`.
    fn is_vertex_loop(&self, loop_ref: u64, referrer: u64) -> MapResult<bool> {
        let inst = instance(self.file, loop_ref, referrer)?;
        Ok(inst.entity.part("VERTEX_LOOP").is_some())
    }

    fn map_loop(&mut self, loop_ref: u64, referrer: u64) -> MapResult<BoundLoop> {
        let inst = instance(self.file, loop_ref, referrer)?;
        if let Some(rec) = inst.entity.part("VERTEX_LOOP") {
            // `VERTEX_LOOP('', #v)`: the face closes at a single point — a
            // cone apex or a sphere pole, where the surface parameterization
            // is singular. It maps to a degenerate loop with no fins.
            let vertex_ref = ref_attr(rec, 1, loop_ref)?;
            let vertex = self.map_vertex(vertex_ref, loop_ref)?;
            return Ok(BoundLoop::Vertex(vertex));
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
        Ok(BoundLoop::Edges(edges))
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

        let raw = resolve_curve(
            self.file,
            geometry_ref,
            edge_ref,
            self.scale,
            self.angle_scale,
        )?;
        // TRIMMED_CURVE and SURFACE_CURVE wrapping are both transparent
        // here: the edge's vertices re-trim whatever basis the wrapper
        // carries, and a surface association says nothing about the locus.
        let curve = match raw.exact_curve() {
            Some(curve) => curve,
            None => {
                let what = match raw.basis() {
                    RawCurve::Conic(_) => {
                        "exact PARABOLA/HYPERBOLA import (geometry store has no conic variant)"
                    }
                    RawCurve::Composite(_) => {
                        "exact COMPOSITE_CURVE import (geometry store has no multi-segment curve)"
                    }
                    RawCurve::Analytic(_)
                    | RawCurve::Nurbs(_)
                    | RawCurve::Trimmed { .. }
                    | RawCurve::OnSurface { .. } => {
                        unreachable!("exact_curve covers the storable bases, basis() the wrappers")
                    }
                };
                return Err(unsupported(
                    geometry_ref,
                    format!("{what}; falling back to tessellation"),
                ));
            }
        };
        let trimmed = trim_curve(&curve, same_sense, start, end, closed, edge_ref)?;

        // The trim is allowed to miss the vertex by up to `TRIM_TOL_REL` —
        // STEP writes finite decimals, and a vertex point and the curve it
        // sits on are rounded independently. That miss is exactly what a
        // vertex tolerance is for (`spec/08-tolerances.md` §7.1 invariant 2):
        // leaving it at `SYSTEM_RESOLUTION` would claim a precision the file
        // never had, and `check_geometry` reports the difference as
        // `VertexOffEdge`. A vertex is shared, so each edge raises rather
        // than sets, and a closed edge visits its one vertex twice.
        //
        // The edge is created at `SYSTEM_RESOLUTION` below and gets its own
        // tolerance from `record_edge_tolerances`, on the finished body,
        // because an edge tolerance answers a different question — how far
        // the *curve* strays from the surfaces of every face it bounds, of
        // which only the first exists here. §7.1 invariant 4 would have the
        // edge's rise to cover this vertex residual as well; it is not raised
        // for that, since inflating it to satisfy an ordering would loosen
        // invariant 1 by an amount unrelated to what that measures. Tracked
        // as of-az8x.
        for (vertex, residual) in [
            (v_start, trimmed.start_residual),
            (v_end, trimmed.end_residual),
        ] {
            let vertex = self.store.vertices.get_mut(vertex).expect("just created");
            vertex.tolerance = vertex.tolerance.max(residual);
        }

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
    /// closed manifold. Void shells tessellate into the same mesh; a void
    /// whose `ORIENTED_CLOSED_SHELL` flag negates the authored orientation
    /// has its triangles rewound (and normals negated), so cavity triangles
    /// always face into the cavity and the mesh SDF signs correctly.
    fn mesh_solid(
        &mut self,
        msb_id: u64,
        shell_ref: u64,
        voids: &[(u64, bool)],
    ) -> Option<TriangleMesh> {
        let mut mesh = TriangleMesh::new();
        let mut ok = self.mesh_shell(&mut mesh, msb_id, shell_ref);
        for &(void_ref, as_authored) in voids {
            let tri_start = mesh.indices.len();
            let vertex_start = mesh.positions.len();
            ok &= self.mesh_shell(&mut mesh, msb_id, void_ref);
            if !as_authored {
                for tri in &mut mesh.indices[tri_start..] {
                    tri.swap(1, 2);
                }
                for normal in &mut mesh.normals[vertex_start..] {
                    *normal = -*normal;
                }
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

    /// Tessellate every face of one `CLOSED_SHELL` into `mesh`, reporting
    /// per-face diagnostics; false if any face failed.
    fn mesh_shell(&mut self, mesh: &mut TriangleMesh, msb_id: u64, shell_ref: u64) -> bool {
        let shell_rec = match typed_record(self.file, shell_ref, "CLOSED_SHELL", msb_id) {
            Ok(rec) => rec,
            Err(e) => {
                self.diagnostics.push(e.diagnostic());
                return false;
            }
        };
        let face_refs = match ref_list(shell_rec, 1, shell_ref) {
            Ok(refs) => refs,
            Err(e) => {
                self.diagnostics.push(e.diagnostic());
                return false;
            }
        };
        let mut ok = true;
        for face_ref in face_refs {
            if let Err(e) = self.mesh_face(mesh, face_ref, shell_ref) {
                self.diagnostics.push(e.diagnostic());
                ok = false;
            }
        }
        ok
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
            // The patch is sized to the face's own bounds, so meshing its
            // full parameter domain covers the face and no more — the one
            // NURBS case where ignoring the trim costs nothing.
            RawSurface::ExtrudedNurbs { curve, dir } => {
                let surface = extruded_nurbs_surface(
                    self.file,
                    &bounds,
                    face_ref,
                    self.scale,
                    self.angle_scale,
                    &curve,
                    &dir,
                )?;
                mesh_nurbs_face(mesh, &surface, same_sense);
                Ok(())
            }
            RawSurface::Extruded { basis, dir } => {
                self.mesh_extruded_face(mesh, face_ref, &bounds, &basis, &dir, same_sense)
            }
            RawSurface::Revolved {
                basis,
                origin,
                axis,
            } => self.mesh_revolved_face(mesh, face_ref, &basis, origin, &axis, same_sense),
        }
    }

    /// Grid a non-reducible extruded face: the bounded basis polyline swept
    /// along the extrusion direction, the sweep span recovered from the
    /// face's boundary (like the quadric mesher, the face is assumed to
    /// cover the full basis curve).
    fn mesh_extruded_face(
        &mut self,
        mesh: &mut TriangleMesh,
        face_ref: u64,
        bounds: &[u64],
        basis: &RawCurve,
        dir: &Vector3,
        same_sense: bool,
    ) -> MapResult<()> {
        let profile = self.sample_bounded(basis, face_ref)?;
        if profile.len() < 2 {
            return Err(invalid(face_ref, "extrusion basis samples to a point"));
        }
        let unit = dir / dir.norm();

        // Sweep span along the axis, from the boundary polylines (which the
        // adjacent faces also use, so the end rows weld).
        let mut v_lo = f64::INFINITY;
        let mut v_hi = f64::NEG_INFINITY;
        for &bound_ref in bounds {
            for p in self.bound_polygon(bound_ref, face_ref)? {
                let v = (p - profile[0]).dot(&unit);
                v_lo = v_lo.min(v);
                v_hi = v_hi.max(v);
            }
        }
        // NaN-safe: only a strictly positive span is acceptable.
        if (v_hi - v_lo).partial_cmp(&1e-12) != Some(std::cmp::Ordering::Greater) {
            return Err(invalid(
                face_ref,
                "face boundary does not span the extrusion direction",
            ));
        }

        // A closed profile wraps around (drop the duplicated closing point).
        let closed = (profile[profile.len() - 1] - profile[0]).norm()
            < 1e-9 * (1.0 + profile[0].coords.norm());
        let columns: &[Point3] = if closed {
            &profile[..profile.len() - 1]
        } else {
            &profile
        };
        let n_cols = columns.len();
        let flip = if same_sense { 1.0 } else { -1.0 };

        let base = mesh.positions.len();
        for (i, p) in columns.iter().enumerate() {
            // Central-difference tangent (wrapping when closed).
            let prev = if i > 0 {
                columns[i - 1]
            } else if closed {
                columns[n_cols - 1]
            } else {
                columns[0]
            };
            let next = if i + 1 < n_cols {
                columns[i + 1]
            } else if closed {
                columns[0]
            } else {
                columns[n_cols - 1]
            };
            let tangent = next - prev;
            let normal = tangent.cross(&unit);
            let normal = if normal.norm() > 1e-12 {
                normal.normalize() * flip
            } else {
                Vector3::zeros()
            };
            for v in [v_lo, v_hi] {
                mesh.positions.push(p + unit * v);
                mesh.normals.push(normal);
            }
        }
        // Vertex layout: column i contributes [bottom, top] at base + 2i.
        let quads = if closed { n_cols } else { n_cols - 1 };
        for i in 0..quads {
            let j = (i + 1) % n_cols;
            let (a, b) = (base + 2 * i, base + 2 * j); // bottom row, +u
            let (d, c) = (base + 2 * i + 1, base + 2 * j + 1); // top row
            // Winding follows du × dv = tangent × extrusion when outward.
            let tris = if same_sense {
                [[a, b, c], [a, c, d]]
            } else {
                [[a, c, b], [a, d, c]]
            };
            for tri in tris {
                if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                    mesh.indices.push(tri);
                }
            }
        }
        Ok(())
    }

    /// Grid a non-reducible revolved face: the bounded basis polyline
    /// revolved through the full turn (like the quadric mesher, trimmed
    /// sweeps are assumed to cover the full angular range). Profile points
    /// on the axis collapse to singular rows, exactly like sphere poles.
    fn mesh_revolved_face(
        &mut self,
        mesh: &mut TriangleMesh,
        face_ref: u64,
        basis: &RawCurve,
        origin: Point3,
        axis: &Vector3,
        same_sense: bool,
    ) -> MapResult<()> {
        let profile = self.sample_bounded(basis, face_ref)?;
        if profile.len() < 2 {
            return Err(invalid(face_ref, "revolution basis samples to a point"));
        }
        // A closed profile (revolved tube) wraps in v as well.
        let wrap_v = (profile[profile.len() - 1] - profile[0]).norm()
            < 1e-9 * (1.0 + profile[0].coords.norm());
        let rows_pts: &[Point3] = if wrap_v {
            &profile[..profile.len() - 1]
        } else {
            &profile
        };
        let n_rows = rows_pts.len();
        let n_u = angular_segments(TAU, self.options);
        let flip = if same_sense { 1.0 } else { -1.0 };
        let scale = 1.0 + origin.coords.norm();

        // Rodrigues rotation of `p` about the axis line by angle `theta`.
        let rotate = |p: &Point3, theta: f64| -> Point3 {
            let w = p - origin;
            let (sin, cos) = theta.sin_cos();
            origin + w * cos + axis.cross(&w) * sin + *axis * (axis.dot(&w) * (1.0 - cos))
        };

        let mut rows: Vec<Vec<usize>> = Vec::with_capacity(n_rows);
        for (j, p) in rows_pts.iter().enumerate() {
            let radial = (p - origin) - *axis * (p - origin).dot(axis);
            let singular = radial.norm() < 1e-9 * scale;
            // Profile tangent for normals (central difference, wrapping).
            let prev = rows_pts[if j > 0 {
                j - 1
            } else if wrap_v {
                n_rows - 1
            } else {
                0
            }];
            let next = rows_pts[if j + 1 < n_rows {
                j + 1
            } else if wrap_v {
                0
            } else {
                n_rows - 1
            }];
            let tangent = next - prev;
            let columns = if singular { 1 } else { n_u };
            let mut row = Vec::with_capacity(columns);
            for i in 0..columns {
                let theta = TAU * i as f64 / n_u as f64;
                let position = rotate(p, theta);
                // du (circumferential, +theta) × dv (profile tangent).
                let radial_here = (position - origin) - *axis * (position - origin).dot(axis);
                let du = axis.cross(&radial_here);
                let dv = rotate(&Point3::from(p.coords + tangent), theta) - position;
                let normal = du.cross(&dv);
                let normal = if normal.norm() > 1e-12 {
                    normal.normalize() * flip
                } else {
                    Vector3::zeros()
                };
                row.push(mesh.positions.len());
                mesh.positions.push(position);
                mesh.normals.push(normal);
            }
            rows.push(row);
        }

        let at = |j: usize, i: usize| -> usize {
            let row = &rows[j % n_rows];
            row[i % row.len()]
        };
        let bands = if wrap_v { n_rows } else { n_rows - 1 };
        for j in 0..bands {
            for i in 0..n_u {
                let (a, b) = (at(j, i), at(j, i + 1));
                let (d, c) = (at(j + 1, i), at(j + 1, i + 1));
                let tris = if same_sense {
                    [[a, b, c], [a, c, d]]
                } else {
                    [[a, c, b], [a, d, c]]
                };
                for tri in tris {
                    if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                        mesh.indices.push(tri);
                    }
                }
            }
        }
        Ok(())
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
        // The outer bound must be a real polygon. Hole bounds that sample to
        // fewer than 3 points cut nothing out and are tolerated (the
        // triangulator ignores them); an outer bound that does is a face with
        // no area, which is a malformed file, not a hole-free face.
        if !rings_3d.iter().any(|r| r.len() >= 3) {
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
        // Any point on the plane serves as the projection origin; the first
        // ring may be empty, so take it from the first that is not.
        let origin = rings_3d
            .iter()
            .flatten()
            .next()
            .copied()
            .expect("some ring has at least 3 points");
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
    ///
    /// A degenerate `VERTEX_LOOP` bound yields its single point: it cuts
    /// nothing out of a planar face (the triangulator ignores rings under
    /// three points), and on a quadric it is the apex/pole sample that bounds
    /// the `v` range the grid must span.
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

        let loop_inst = instance(self.file, loop_ref, bound_ref)?;
        if let Some(vertex_loop) = loop_inst.entity.part("VERTEX_LOOP") {
            let vertex_ref = ref_attr(vertex_loop, 1, loop_ref)?;
            let vrec = typed_record(self.file, vertex_ref, "VERTEX_POINT", loop_ref)?;
            let point = resolve_point(
                self.file,
                ref_attr(vrec, 1, vertex_ref)?,
                vertex_ref,
                self.scale,
            )?;
            return Ok(vec![point]);
        }
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

        let raw = resolve_curve(
            self.file,
            geometry_ref,
            edge_ref,
            self.scale,
            self.angle_scale,
        )?;
        let points = self.edge_points(raw, start, end, same_sense, closed, edge_ref)?;
        self.polylines.insert(edge_ref, points.clone());
        Ok(points)
    }

    /// Discretize resolved edge geometry between the edge's vertex points.
    /// Analytic curves and conics trim by the vertices exactly; sampled
    /// forms (NURBS, composites) snap their endpoints onto the vertices.
    fn edge_points(
        &mut self,
        raw: RawCurve,
        start: Point3,
        end: Point3,
        same_sense: bool,
        closed: bool,
        edge_ref: u64,
    ) -> MapResult<Vec<Point3>> {
        match raw {
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
                Ok(points)
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
                self.snap_endpoints(&mut points, start, end, edge_ref, "B-spline");
                Ok(points)
            }
            // Vertex trim via the closed-form parameter inverse; both conics
            // are open curves, so a closed edge is malformed.
            RawCurve::Conic(conic) => {
                if closed {
                    return Err(invalid(
                        edge_ref,
                        "a parabola/hyperbola cannot carry a closed edge",
                    ));
                }
                let oriented = if same_sense { conic } else { conic.reversed() };
                let t0 = oriented.param_of(&start);
                let t1 = oriented.param_of(&end);
                // NaN-safe: only a strictly positive advance is acceptable.
                if (t1 - t0).partial_cmp(&1e-12) != Some(std::cmp::Ordering::Greater) {
                    return Err(invalid(
                        edge_ref,
                        "edge endpoints do not advance along its conic (check same_sense)",
                    ));
                }
                let tol = TRIM_TOL_REL * (1.0 + start.coords.norm().max(end.coords.norm()));
                if (oriented.point(t0) - start).norm() > tol
                    || (oriented.point(t1) - end).norm() > tol
                {
                    return Err(invalid(
                        edge_ref,
                        "edge geometry does not pass through the edge's vertex points",
                    ));
                }
                // Conic parameters are not angles; the angular step is a
                // density proxy (t spans O(1) over visually curved arcs).
                let segments = angular_segments(t1 - t0, self.options);
                let mut points = Vec::with_capacity(segments + 1);
                points.push(start);
                for k in 1..segments {
                    points.push(oriented.point(t0 + (t1 - t0) * k as f64 / segments as f64));
                }
                points.push(end);
                Ok(points)
            }
            // The edge's vertices re-trim the basis; the wrapper's own
            // bounds only matter inside composite segments. A surface
            // association is likewise transparent — the fallback works in
            // 3D, where the pcurves it carries have nothing to add.
            RawCurve::Trimmed { basis, .. } | RawCurve::OnSurface { basis } => {
                self.edge_points(*basis, start, end, same_sense, closed, edge_ref)
            }
            RawCurve::Composite(segments) => {
                let mut points = self.sample_composite(&segments, edge_ref)?;
                if !same_sense {
                    points.reverse();
                }
                if points.len() < 2 {
                    return Err(invalid(
                        edge_ref,
                        "COMPOSITE_CURVE samples to fewer than 2 points",
                    ));
                }
                self.snap_endpoints(&mut points, start, end, edge_ref, "composite");
                Ok(points)
            }
        }
    }

    /// Force a sampled polyline's endpoints onto the edge's vertex points,
    /// warning when they were not already there.
    fn snap_endpoints(
        &mut self,
        points: &mut [Point3],
        start: Point3,
        end: Point3,
        edge_ref: u64,
        what: &str,
    ) {
        let scale = start.coords.norm().max(end.coords.norm());
        let tol = TRIM_TOL_REL * (1.0 + scale);
        let last = points.len() - 1;
        if (points[0] - start).norm() > tol || (points[last] - end).norm() > tol {
            self.diagnostics.push(Diagnostic {
                entity: Some(edge_ref),
                severity: Severity::Warning,
                message: format!(
                    "{what} edge curve endpoints do not match the edge's vertex points; snapping"
                ),
            });
        }
        points[0] = start;
        points[last] = end;
    }

    /// Sample a bounded curve (no vertices available — swept-surface bases
    /// and composite segments). Bounds come from the curve itself: the full
    /// period of a circle/ellipse, a NURBS knot domain, or a
    /// `TRIMMED_CURVE`'s explicit trim bounds.
    fn sample_bounded(&self, raw: &RawCurve, entity: u64) -> MapResult<Vec<Point3>> {
        match raw {
            RawCurve::Analytic(curve @ (Curve3::Circle { .. } | Curve3::Ellipse { .. })) => {
                let n = angular_segments(TAU, self.options);
                Ok((0..=n)
                    .map(|k| curve.point(TAU * k as f64 / n as f64))
                    .collect())
            }
            RawCurve::Analytic(_) => Err(unsupported(
                entity,
                "an unbounded curve here needs TRIMMED_CURVE bounds",
            )),
            RawCurve::Nurbs(curve) => {
                let (t0, t1) = curve.knot_vector().domain();
                let n = nurbs_segments(curve.knot_vector().control_count());
                Ok((0..=n)
                    .map(|k| curve.point(t0 + (t1 - t0) * k as f64 / n as f64))
                    .collect())
            }
            RawCurve::Conic(_) => Err(unsupported(
                entity,
                "a parabola/hyperbola here needs TRIMMED_CURVE bounds",
            )),
            RawCurve::Trimmed {
                basis,
                trims,
                sense,
            } => self.sample_trimmed(basis, trims, *sense, entity),
            RawCurve::Composite(segments) => self.sample_composite(segments, entity),
            // The surface association says nothing about the locus; the
            // fallback samples the 3D basis exactly as it would unwrapped.
            RawCurve::OnSurface { basis } => self.sample_bounded(basis, entity),
        }
    }

    /// Sample a `TRIMMED_CURVE` between its trim bounds, honoring
    /// `sense_agreement` (the trimmed curve runs trim_1 → trim_2 along the
    /// basis direction when true, against it when false).
    fn sample_trimmed(
        &self,
        basis: &RawCurve,
        trims: &[RawTrim; 2],
        sense: bool,
        entity: u64,
    ) -> MapResult<Vec<Point3>> {
        match basis {
            RawCurve::Analytic(curve @ Curve3::Line { .. }) => {
                // Line parameters scale by the underlying VECTOR magnitude,
                // which normalization discarded — only points are reliable.
                let (Some(p0), Some(p1)) = (trims[0].point, trims[1].point) else {
                    return Err(unsupported(
                        entity,
                        "TRIMMED_CURVE over a LINE without cartesian trim points",
                    ));
                };
                let tol = TRIM_TOL_REL * (1.0 + p0.coords.norm().max(p1.coords.norm()));
                let Curve3::Line { origin, dir } = curve else {
                    unreachable!("matched Line");
                };
                for p in [&p0, &p1] {
                    let off = (p - origin) - dir * (p - origin).dot(dir);
                    if off.norm() > tol {
                        return Err(invalid(entity, "trim point is not on its LINE basis"));
                    }
                }
                Ok(vec![p0, p1])
            }
            RawCurve::Analytic(curve @ Curve3::Circle { .. }) => {
                // Angle bounds from points (preferred) or parameter values;
                // a coincident pair bounds the full period.
                let angle = |trim: &RawTrim| -> MapResult<f64> {
                    if let Some(p) = trim.point {
                        return Ok(conic_angle(curve, &p).expect("circle is a conic"));
                    }
                    trim.param
                        .ok_or_else(|| invalid(entity, "empty TRIMMED_CURVE bound"))
                };
                let t0 = angle(&trims[0])?;
                let t1 = angle(&trims[1])?;
                let sweep = if sense {
                    let s = (t1 - t0).rem_euclid(TAU);
                    if s < 1e-9 { TAU } else { s }
                } else {
                    let s = (t0 - t1).rem_euclid(TAU);
                    if s < 1e-9 { -TAU } else { -s }
                };
                let n = angular_segments(sweep, self.options);
                Ok((0..=n)
                    .map(|k| curve.point(t0 + sweep * k as f64 / n as f64))
                    .collect())
            }
            RawCurve::Analytic(curve @ Curve3::Ellipse { .. }) => {
                // The reader may have swapped the ellipse's axes to enforce
                // major >= minor, shifting parameter meaning by a quarter
                // turn — so only cartesian trim points are trustworthy.
                let (Some(p0), Some(p1)) = (trims[0].point, trims[1].point) else {
                    return Err(unsupported(
                        entity,
                        "TRIMMED_CURVE over an ELLIPSE without cartesian trim points",
                    ));
                };
                let t0 = conic_angle(curve, &p0).expect("ellipse is a conic");
                let t1 = conic_angle(curve, &p1).expect("ellipse is a conic");
                let sweep = if sense {
                    let s = (t1 - t0).rem_euclid(TAU);
                    if s < 1e-9 { TAU } else { s }
                } else {
                    let s = (t0 - t1).rem_euclid(TAU);
                    if s < 1e-9 { -TAU } else { -s }
                };
                let n = angular_segments(sweep, self.options);
                Ok((0..=n)
                    .map(|k| curve.point(t0 + sweep * k as f64 / n as f64))
                    .collect())
            }
            RawCurve::Conic(conic) => {
                let param = |trim: &RawTrim| -> MapResult<f64> {
                    if let Some(p) = trim.point {
                        return Ok(conic.param_of(&p));
                    }
                    trim.param
                        .ok_or_else(|| invalid(entity, "empty TRIMMED_CURVE bound"))
                };
                // Non-periodic: the parameter order encodes the direction
                // (descending == against the basis, matching !sense).
                let t0 = param(&trims[0])?;
                let t1 = param(&trims[1])?;
                if (t1 - t0).abs() < 1e-12 {
                    return Err(invalid(entity, "conic TRIMMED_CURVE sweeps zero length"));
                }
                let n = angular_segments(t1 - t0, self.options);
                Ok((0..=n)
                    .map(|k| conic.point(t0 + (t1 - t0) * k as f64 / n as f64))
                    .collect())
            }
            RawCurve::Nurbs(curve) => {
                let (d0, d1) = curve.knot_vector().domain();
                let t0 = trims[0].param.map_or(d0, |t| t.clamp(d0, d1));
                let t1 = trims[1].param.map_or(d1, |t| t.clamp(d0, d1));
                if (t1 - t0).abs() < 1e-12 {
                    return Err(invalid(entity, "NURBS TRIMMED_CURVE sweeps zero length"));
                }
                let n = nurbs_segments(curve.knot_vector().control_count());
                let mut points: Vec<Point3> = (0..=n)
                    .map(|k| curve.point(t0 + (t1 - t0) * k as f64 / n as f64))
                    .collect();
                if !sense {
                    points.reverse();
                }
                Ok(points)
            }
            // Polyline is never produced by the reader; nested wrappers are
            // pathological files — degrade with a diagnostic, don't guess.
            RawCurve::Analytic(_)
            | RawCurve::Trimmed { .. }
            | RawCurve::Composite(_)
            | RawCurve::OnSurface { .. } => Err(unsupported(
                entity,
                "nested TRIMMED_CURVE/COMPOSITE_CURVE/SURFACE_CURVE bases",
            )),
        }
    }

    /// Chain a `COMPOSITE_CURVE`'s segment polylines, dropping duplicated
    /// junction points.
    fn sample_composite(
        &self,
        segments: &[(RawCurve, bool)],
        entity: u64,
    ) -> MapResult<Vec<Point3>> {
        let mut chain: Vec<Point3> = Vec::new();
        for (parent, seg_sense) in segments {
            let mut points = self.sample_bounded(parent, entity)?;
            if !seg_sense {
                points.reverse();
            }
            if let (Some(last), Some(first)) = (chain.last(), points.first()) {
                let tol = TRIM_TOL_REL * (1.0 + last.coords.norm());
                if (first - last).norm() <= tol {
                    points.remove(0);
                } else {
                    return Err(invalid(
                        entity,
                        "COMPOSITE_CURVE segments do not join end to start",
                    ));
                }
            }
            chain.extend(points);
        }
        Ok(chain)
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

/// The void shells of a `BREP_WITH_VOIDS`: each an `ORIENTED_CLOSED_SHELL`
/// resolved to its underlying `CLOSED_SHELL` plus the orientation flag
/// (true = use as authored). A bare `CLOSED_SHELL` reference — technically
/// malformed but seen in the wild — heals to an as-authored void.
fn resolve_voids(file: &StepFile, rec: &SimpleRecord, msb_id: u64) -> MapResult<Vec<(u64, bool)>> {
    let mut voids = Vec::new();
    for void_ref in ref_list(rec, 2, msb_id)? {
        let inst = instance(file, void_ref, msb_id)?;
        if let Some(oriented) = inst.entity.part("ORIENTED_CLOSED_SHELL") {
            // `(name, *, closed_shell_element, orientation)` — the inherited
            // face set is derived (`*`), like ORIENTED_EDGE's vertices.
            voids.push((
                ref_attr(oriented, 2, void_ref)?,
                bool_attr(oriented, 3, void_ref)?,
            ));
        } else if inst.entity.part("CLOSED_SHELL").is_some() {
            voids.push((void_ref, true));
        } else {
            return Err(invalid(
                void_ref,
                format!(
                    "BREP_WITH_VOIDS void is not an ORIENTED_CLOSED_SHELL ({})",
                    type_names(inst)
                ),
            ));
        }
    }
    Ok(voids)
}

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
    let mut heal_operations = 0;
    for inst in &file.data {
        // BREP_WITH_VOIDS is a MANIFOLD_SOLID_BREP subtype; in the common
        // simple encoding only its own type name appears.
        let Some(rec) = inst
            .entity
            .part("BREP_WITH_VOIDS")
            .or_else(|| inst.entity.part("MANIFOLD_SOLID_BREP"))
        else {
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
            &mut heal_operations,
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
            message: "no MANIFOLD_SOLID_BREP or BREP_WITH_VOIDS instances in the file".to_string(),
        });
    }
    // Product structure last: it places the solids the mapping just found,
    // and needs their instance names in `solids` order.
    let solid_ids: Vec<u64> = solids.iter().map(|s| s.step_id).collect();
    let instances = resolve_instances(file, &solid_ids, length_scale, &mut diagnostics);
    StepImport {
        solids,
        instances,
        diagnostics,
        length_scale,
        angle_scale,
        heal_operations,
    }
}

/// Last step of an exact import: derive the 2D trim geometry the file's
/// `PCURVE`s could not supply directly (see [`validate_associated_geometry`]),
/// settle which bound of each face is the outer one, then make each face's
/// surface sense agree with that bound's winding.
///
/// This runs on the finished body, after validation and any healing, rather
/// than face by face during mapping. Healing welds duplicate edges and
/// re-points the orphaned fins onto the survivor, and a pcurve is tied to
/// its edge's curve and parameter range — so deriving one before the healer
/// has settled which edge a fin uses would leave it describing an edge that
/// is no longer there.
///
/// The sense repair belongs here for the same reason and one more: it reads a
/// loop's winding, which needs both the pcurves *and* the settled outer bound,
/// so it can only be the last of the three
/// ([`reconcile_face_senses`] says why the flag yields to the winding).
///
/// Returns how many repairs it applied, for
/// [`StepImport::heal_operations`]. Both of the last two steps read a loop's
/// winding, which only the pcurves make measurable — with
/// [`StepReadOptions::pcurves`] off there is nothing to read, so neither runs
/// and each face keeps the outer bound and the sense flag the file authored.
fn finish_exact_body(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    body: EntityId<Body>,
    options: &StepReadOptions,
    multi_bound_faces: &[EntityId<Face>],
    msb_id: u64,
    diagnostics: &mut Vec<Diagnostic>,
) -> usize {
    if !options.pcurves {
        return 0;
    }
    attach_body_pcurves(store, geo, body);

    let redesignated = choose_outer_bounds(store, geo, multi_bound_faces);
    if redesignated > 0 {
        diagnostics.push(Diagnostic {
            entity: Some(msb_id),
            severity: Severity::Info,
            message: format!(
                "outer bound re-chosen by parameter-space area on {redesignated} \
                 of {} multi-bound face(s)",
                multi_bound_faces.len()
            ),
        });
    }

    // A sense flag contradicting its own loop is invisible to every pass in
    // `heal` (it changes no fin sense), and invisible again to the
    // topology-only check that decides whether `heal` is consulted at all —
    // so this is the only place it is caught. See of-hrgt.
    let corrections = reconcile_face_senses(body, store, geo, options.heal.strategy);
    for op in &corrections {
        diagnostics.push(Diagnostic {
            entity: Some(msb_id),
            severity: Severity::Info,
            message: format!("healed: {op}"),
        });
    }
    corrections.len()
}

/// Settle the outer bound of each face in `faces` by the area its loops
/// enclose in parameter space: the outer bound is the one containing the
/// others, and containment shows up as the largest `|area|`. Returns how many
/// faces changed.
///
/// This overrides `FACE_OUTER_BOUND` rather than deferring to it, because
/// files get the tag wrong (see `map_face`) and geometry does not: on a face
/// that is well formed at all, exactly one of its loops encloses the rest.
///
/// A face is left alone unless every one of its real loops can be measured.
/// A partial comparison could hand the role to the largest of the *readable*
/// loops while the true outer bound sits among the unreadable ones, which
/// would be a worse answer than the one it replaced.
fn choose_outer_bounds(
    store: &mut TopologyStore,
    geo: &GeometryStore,
    faces: &[EntityId<Face>],
) -> usize {
    let mut redesignated = 0;
    for &face_id in faces {
        let Some(face) = store.face(face_id) else {
            continue;
        };
        let Some(surface) = face.surface.and_then(|id| geo.surface(id)) else {
            continue;
        };
        let Some(outer) = face.outer_loop else {
            continue;
        };
        let loops: Vec<EntityId<Loop>> = std::iter::once(outer)
            .chain(face.inner_loops.iter().copied())
            .collect();

        let mut largest: Option<(f64, EntityId<Loop>)> = None;
        let mut all_readable = true;
        for &loop_id in &loops {
            // A degenerate loop encloses nothing and never competes; it is
            // not a gap in the comparison either.
            if store.loop_(loop_id).is_none_or(|lp| lp.fins.is_empty()) {
                continue;
            }
            let Some(twice_signed_area) = store.loop_winding(geo, surface, loop_id) else {
                all_readable = false;
                break;
            };
            let area = twice_signed_area.abs();
            if largest.is_none_or(|(best, _)| area > best) {
                largest = Some((area, loop_id));
            }
        }

        if !all_readable {
            continue;
        }
        if let Some((_, winner)) = largest {
            if winner != outer {
                store.set_outer_loop(face_id, winner);
                redesignated += 1;
            }
        }
    }
    redesignated
}

/// Carry how far each imported edge's curve actually sits from the surfaces
/// of the faces it bounds as that edge's tolerance — `spec/08-tolerances.md`
/// §7.1 invariant 1, the edge half of of-bb6 (`map_edge` records the
/// counterpart for a vertex, the trim residual, as it goes).
///
/// The reader has to *accept* such a gap. A STEP file writes an edge's curve
/// and its faces' surfaces as independently rounded decimals, and plenty of
/// producers write a curve that was never exactly on either surface to begin
/// with — the vendored NIST corpus carries authored gaps from 3e-9 mm up to
/// millimetre thousandths. What it must not do is accept the gap and then
/// stamp [`SYSTEM_RESOLUTION`] on the edge anyway, which claims a precision
/// no file supported and which [`TopologyStore::check_geometry`] then reports
/// as `EdgeOffSurface` — 828 of them on nist_ctc_02 alone.
///
/// Tolerances only rise: an edge already tolerant from healing keeps that
/// tolerance if it is the larger, since the healer's figure covers a gap this
/// measurement is not looking at.
///
/// This runs on the finished, healed body rather than per edge in `map_edge`
/// for two reasons. An edge's deviation is a maximum over *every* adjacent
/// face, and at `map_edge` time only the first of them exists — the second
/// face reaches the same edge through the cache and would never re-measure
/// it. And healing welds duplicate edges and re-points fins onto the
/// survivor, so which faces an edge bounds is not settled until it has run.
///
/// `Err` names the worst edge whose gap exceeds [`MAX_ALLOWED_TOLERANCE`].
/// There is no honest tolerance to give such an edge: the kernel's cap is a
/// cap, and writing the measured value would only move the complaint to
/// `check`'s per-entity range test. So the solid does not import exactly at
/// all and falls back to tessellation, which represents the same shape
/// without claiming anything about where its edges are. That refusal is what
/// makes the tolerance a bound rather than a decoration.
fn record_edge_tolerances(
    store: &mut TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
) -> Result<usize, (EntityId<Edge>, f64)> {
    let measured = store.measure_edges_off_surfaces(geo, body);
    if let Some(&worst) = measured
        .iter()
        .filter(|(_, deviation)| *deviation > MAX_ALLOWED_TOLERANCE)
        .max_by(|a, b| a.1.total_cmp(&b.1))
    {
        return Err(worst);
    }
    let mut raised = 0;
    for (edge_id, deviation) in measured {
        let Some(edge) = store.edges.get_mut(edge_id) else {
            continue;
        };
        if deviation > edge.tolerance {
            edge.tolerance = deviation;
            raised += 1;
        }
    }
    Ok(raised)
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
    heal_operations: &mut usize,
) -> SolidOutcome {
    let msb_id = inst.id;
    // `BREP_WITH_VOIDS(name, outer, voids)` extends the plain
    // `MANIFOLD_SOLID_BREP(name, outer)` with a list of
    // `ORIENTED_CLOSED_SHELL` cavities.
    let (rec, has_voids) = match inst.entity.part("BREP_WITH_VOIDS") {
        Some(rec) if rec.attributes.len() >= 3 => (rec, true),
        _ => (
            inst.entity
                .part("MANIFOLD_SOLID_BREP")
                .expect("caller selected solid instances"),
            false,
        ),
    };
    let shell_ref = match ref_attr(rec, 1, msb_id) {
        Ok(shell_ref) => shell_ref,
        Err(e) => {
            diagnostics.push(e.diagnostic());
            return SolidOutcome::Failed;
        }
    };
    let voids = if has_voids {
        match resolve_voids(file, rec, msb_id) {
            Ok(voids) => voids,
            Err(e) => {
                diagnostics.push(e.diagnostic());
                return SolidOutcome::Failed;
            }
        }
    } else {
        Vec::new()
    };

    // Exact path first.
    let mut builder = SolidBuilder {
        file,
        store,
        geo,
        scale,
        angle_scale,
        created: Created::default(),
        multi_bound_faces: Vec::new(),
        vertices: HashMap::new(),
        edges: HashMap::new(),
        seams: Vec::new(),
    };
    let built = builder.build(msb_id, shell_ref, &voids);
    let created = builder.created;
    let multi_bound_faces = builder.multi_bound_faces;
    match built {
        Ok(body) => {
            let mut failures = store.check(body);
            // Import heals, it does not reject (`spec/06-step-io.md` §4): a
            // body whose only defects are an unsewn shell, a vertex gap or a
            // flipped face is repaired and re-validated before the exact path
            // is abandoned for the tessellated one.
            if !failures.is_empty() && options.heal.strategy != HealStrategy::Off {
                let result = GeometryHealer::heal(body, store, geo, &options.heal);
                *heal_operations += result.operations.len();
                for op in &result.operations {
                    diagnostics.push(Diagnostic {
                        entity: Some(msb_id),
                        severity: Severity::Info,
                        message: format!("healed: {op}"),
                    });
                }
                for note in &result.notes {
                    diagnostics.push(Diagnostic {
                        entity: Some(msb_id),
                        severity: Severity::Info,
                        message: format!("heal: {note}"),
                    });
                }
                failures = result.remaining;
            }
            if failures.is_empty() {
                // Only once the topology is settled — healing decides which
                // faces an edge ends up bounding — is there anything to
                // measure the edges against.
                match record_edge_tolerances(store, geo, body) {
                    Ok(raised) => {
                        if raised > 0 {
                            diagnostics.push(Diagnostic {
                                entity: Some(msb_id),
                                severity: Severity::Info,
                                message: format!(
                                    "{raised} edge tolerance(s) raised to the measured \
                                     distance from their faces' surfaces"
                                ),
                            });
                        }
                        *heal_operations += finish_exact_body(
                            store,
                            geo,
                            body,
                            options,
                            &multi_bound_faces,
                            msb_id,
                            diagnostics,
                        );
                        return SolidOutcome::BRep(body);
                    }
                    Err((edge, deviation)) => diagnostics.push(Diagnostic {
                        entity: Some(msb_id),
                        severity: Severity::Warning,
                        message: format!(
                            "mapped body failed validation: edge {edge:?} strays \
                             {deviation} from the surface of a face it bounds, more \
                             than the kernel's maximum tolerance \
                             ({MAX_ALLOWED_TOLERANCE})"
                        ),
                    }),
                }
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
    match mesher.mesh_solid(msb_id, shell_ref, &voids) {
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
        let mut b = String::new();
        let shell = block_shell_at(&mut b, 0, x, y, z);
        writeln!(
            b,
            "#{} = MANIFOLD_SOLID_BREP('block', #{shell});",
            shell + 1
        )
        .unwrap();
        wrap(&b)
    }

    /// The same block with every planar face respelled as the degree-1
    /// B-spline patch spanning its own four corners. A bilinear patch over
    /// a rectangle *is* that plane, so the solid is unchanged — only the
    /// STEP spelling, and so the imported [`Surface3`] variant.
    fn bspline_block_step(x: f64, y: f64, z: f64) -> String {
        let mut b = String::new();
        let shell = block_shell_with_surfaces(&mut b, 0, x, y, z, true);
        writeln!(
            b,
            "#{} = MANIFOLD_SOLID_BREP('bspline block', #{shell});",
            shell + 1
        )
        .unwrap();
        wrap(&b)
    }

    /// Emit one outward-wound block `CLOSED_SHELL` with instance names
    /// offset by `base`; returns the shell's id. Shared by the plain block
    /// fixture and the `BREP_WITH_VOIDS` fixtures (outer + cavity shells).
    fn block_shell_at(b: &mut String, base: u64, x: f64, y: f64, z: f64) -> u64 {
        block_shell_with_surfaces(b, base, x, y, z, false)
    }

    /// `nurbs_faces` swaps each face's `PLANE` for the equivalent bilinear
    /// `B_SPLINE_SURFACE_WITH_KNOTS`; everything else — vertices, line edge
    /// curves, loop winding, face senses — is identical either way.
    fn block_shell_with_surfaces(
        b: &mut String,
        base: u64,
        x: f64,
        y: f64,
        z: f64,
        nurbs_faces: bool,
    ) -> u64 {
        let id = |k: u64| base + k;
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

        for (i, &(px, py, pz)) in corners.iter().enumerate() {
            writeln!(
                b,
                "#{} = CARTESIAN_POINT('', ({px:.6}, {py:.6}, {pz:.6}));",
                id(i as u64 + 1)
            )
            .unwrap();
        }
        for i in 0..8u64 {
            writeln!(b, "#{} = VERTEX_POINT('', #{});", id(9 + i), id(i + 1)).unwrap();
        }
        for (e, &(a, c)) in EDGE_PAIRS.iter().enumerate() {
            let eb = id(17 + 4 * e as u64);
            let (dx, dy, dz) = (
                corners[c].0 - corners[a].0,
                corners[c].1 - corners[a].1,
                corners[c].2 - corners[a].2,
            );
            writeln!(b, "#{eb} = DIRECTION('', ({dx:.6}, {dy:.6}, {dz:.6}));").unwrap();
            writeln!(b, "#{} = VECTOR('', #{eb}, 1.);", eb + 1).unwrap();
            writeln!(
                b,
                "#{} = LINE('', #{}, #{});",
                eb + 2,
                id(a as u64 + 1),
                eb + 1
            )
            .unwrap();
            writeln!(
                b,
                "#{} = EDGE_CURVE('', #{}, #{}, #{}, .T.);",
                eb + 3,
                id(9 + a as u64),
                id(9 + c as u64),
                eb + 2
            )
            .unwrap();
        }
        for (f, &(cycle, (nx, ny, nz))) in face_specs.iter().enumerate() {
            let fb = id(65 + 10 * f as u64);
            writeln!(b, "#{fb} = DIRECTION('', ({nx:.6}, {ny:.6}, {nz:.6}));").unwrap();
            writeln!(
                b,
                "#{} = AXIS2_PLACEMENT_3D('', #{}, #{fb}, $);",
                fb + 1,
                id(cycle[0] as u64 + 1)
            )
            .unwrap();
            if nurbs_faces {
                // Control rows `[[a, d], [b, c]]` put `u` along a→d and `v`
                // along a→b, so `du x dv` is the same outward normal the
                // `PLANE` placement above declares.
                let p = |k: usize| id(cycle[k] as u64 + 1);
                writeln!(
                    b,
                    "#{} = B_SPLINE_SURFACE_WITH_KNOTS('', 1, 1, ((#{}, #{}), (#{}, #{})), \
                     .UNSPECIFIED., .F., .F., .F., (2, 2), (2, 2), (0., 1.), (0., 1.), \
                     .UNSPECIFIED.);",
                    fb + 2,
                    p(0),
                    p(3),
                    p(1),
                    p(2)
                )
                .unwrap();
            } else {
                writeln!(b, "#{} = PLANE('', #{});", fb + 2, fb + 1).unwrap();
            }
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
                    fb + 3 + k as u64,
                    id(17 + 4 * idx as u64 + 3)
                )
                .unwrap();
            }
            writeln!(
                b,
                "#{} = EDGE_LOOP('', (#{}, #{}, #{}, #{}));",
                fb + 7,
                fb + 3,
                fb + 4,
                fb + 5,
                fb + 6
            )
            .unwrap();
            writeln!(b, "#{} = FACE_OUTER_BOUND('', #{}, .T.);", fb + 8, fb + 7).unwrap();
            writeln!(
                b,
                "#{} = ADVANCED_FACE('', (#{}), #{}, .T.);",
                fb + 9,
                fb + 8,
                fb + 2
            )
            .unwrap();
        }
        writeln!(
            b,
            "#{} = CLOSED_SHELL('', (#{}, #{}, #{}, #{}, #{}, #{}));",
            id(125),
            id(74),
            id(84),
            id(94),
            id(104),
            id(114),
            id(124)
        )
        .unwrap();
        id(125)
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

    /// AP203 full cone (of-26t): a base cap plus a conical wall bounded by
    /// the base circle and — at the apex, where the parameterization is
    /// singular — a degenerate `VERTEX_LOOP`.
    ///
    /// The cone placement's axis points *down* (`-z`) because a
    /// `CONICAL_SURFACE` widens along its axis: radius `r` in the base
    /// plane, narrowing upwards to the apex at `z = h`.
    fn cone_step(r: f64, h: f64) -> String {
        wrap(&format!(
            "\
#1 = CARTESIAN_POINT('', (0., 0., 0.));
#2 = CARTESIAN_POINT('', ({r:.6}, 0., 0.));
#3 = CARTESIAN_POINT('', (0., 0., {h:.6}));
#4 = DIRECTION('', (0., 0., 1.));
#5 = DIRECTION('', (0., 0., -1.));
#6 = DIRECTION('', (1., 0., 0.));
#7 = VERTEX_POINT('', #2);
#8 = VERTEX_POINT('', #3);
#9 = AXIS2_PLACEMENT_3D('', #1, #4, #6);
#10 = AXIS2_PLACEMENT_3D('', #1, #5, #6);
#11 = CIRCLE('', #9, {r:.6});
#12 = PLANE('', #10);
#13 = CONICAL_SURFACE('', #10, {r:.6}, {half_angle:.15});
#14 = EDGE_CURVE('', #7, #7, #11, .T.);
#15 = ORIENTED_EDGE('', *, *, #14, .F.);
#16 = EDGE_LOOP('', (#15));
#17 = FACE_OUTER_BOUND('', #16, .T.);
#18 = ADVANCED_FACE('', (#17), #12, .T.);
#19 = ORIENTED_EDGE('', *, *, #14, .T.);
#20 = EDGE_LOOP('', (#19));
#21 = FACE_BOUND('', #20, .T.);
#22 = VERTEX_LOOP('', #8);
#23 = FACE_BOUND('', #22, .T.);
#24 = ADVANCED_FACE('', (#21, #23), #13, .T.);
#25 = CLOSED_SHELL('', (#18, #24));
#26 = MANIFOLD_SOLID_BREP('cone', #25);",
            half_angle = (r / h).atan(),
        ))
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

    /// A block whose six faces are degree-1 B-spline patches carrying *no
    /// bounds at all*. The exact path refuses the empty bound list, so this
    /// is the fixture that still exercises the NURBS mesh fallback now that
    /// bounded B-spline faces import exactly (see [`bspline_block_step`]).
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

    /// The of-3qy.8 headline: bounded B-spline faces take the *exact* path.
    /// Same block, same topology, same volume as the planar spelling — but
    /// the store holds `Surface3::Nurbs`, not a plane the reader inferred.
    #[test]
    fn bspline_faced_block_imports_as_exact_nurbs_brep() {
        let (store, geo, report) = import(&bspline_block_step(2.0, 3.0, 4.0));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);

        assert!(store.check(body).is_empty());
        let counts = store.euler_counts(body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (8, 12, 6));
        for face in store.faces_of_body(body) {
            let surface = geo
                .surface(store.face(face).unwrap().surface.unwrap())
                .unwrap();
            assert!(
                matches!(surface, Surface3::Nurbs(_)),
                "expected an exact NURBS face, got {surface:?}"
            );
        }
        assert_edges_interpolate(&store, &geo, body);

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        assert!(
            (signed_volume(&mesh) - 24.0).abs() < 1e-9,
            "bilinear patches mesh exactly"
        );
    }

    // ---- degenerate VERTEX_LOOP bounds (of-26t) ----

    /// The cone's apex bound carries no edges at all. It must import as a
    /// degenerate loop holding the apex vertex — before of-26t the reader
    /// refused it and dropped the whole solid to the mesh fallback.
    #[test]
    fn cone_apex_vertex_loop_imports_as_exact_brep() {
        let (store, geo, report) = import(&cone_step(1.0, 2.0));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);

        assert!(store.check(body).is_empty(), "{:?}", store.check(body));
        let counts = store.euler_counts(body);
        // The apex vertex is reachable only through the degenerate loop, and
        // it counts in the Euler formula like any other: 2 - 1 + 2 - 1 = 2.
        assert_eq!(
            (counts.vertices, counts.edges, counts.faces, counts.rings),
            (2, 1, 2, 1)
        );
        assert_eq!(counts.genus, 0);

        let wall = store
            .faces_of_body(body)
            .into_iter()
            .find(|&f| {
                matches!(
                    geo.surface(store.face(f).unwrap().surface.unwrap())
                        .unwrap(),
                    Surface3::Cone { .. }
                )
            })
            .expect("one conical face");
        // The base circle bounds the region; the apex loop is the extra ring.
        let outer = store.face(wall).unwrap().outer_loop.expect("outer loop");
        assert_eq!(store.fins_of_loop(outer).len(), 1);
        let inner = store.face(wall).unwrap().inner_loops.clone();
        assert_eq!(inner.len(), 1);
        let apex_loop = store.loop_(inner[0]).unwrap();
        assert!(apex_loop.fins.is_empty(), "a vertex loop has no fins");
        assert_eq!(apex_loop.loop_type, LoopType::Vertex);
        let apex = store
            .vertex(apex_loop.vertex.expect("apex vertex"))
            .unwrap();
        assert!(
            (apex.point - Point3::new(0.0, 0.0, 2.0)).norm() < 1e-12,
            "apex at {:?}",
            apex.point
        );

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        let exact = PI * 2.0 / 3.0;
        assert!(
            (signed_volume(&mesh) - exact).abs() / exact < 0.02,
            "volume {} vs {exact}",
            signed_volume(&mesh)
        );
    }

    /// A degenerate loop has no extent, so it cannot bound the face's region
    /// however the file tags it: the real loop still takes the outer role.
    #[test]
    fn vertex_loop_tagged_as_outer_bound_yields_to_the_real_loop() {
        let src = cone_step(1.0, 2.0).replace(
            "#23 = FACE_BOUND('', #22, .T.);",
            "#23 = FACE_OUTER_BOUND('', #22, .T.);",
        );
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty(), "{:?}", store.check(body));

        let wall = store
            .faces_of_body(body)
            .into_iter()
            .find(|&f| {
                matches!(
                    geo.surface(store.face(f).unwrap().surface.unwrap())
                        .unwrap(),
                    Surface3::Cone { .. }
                )
            })
            .expect("one conical face");
        let outer = store.face(wall).unwrap().outer_loop.expect("outer loop");
        assert_eq!(
            store.fins_of_loop(outer).len(),
            1,
            "the edge loop must stay the outer loop"
        );
    }

    /// The mesh fallback reads the same bound. A composite-curve base edge
    /// forces tessellated import; the apex sample is what bounds the cone
    /// grid's `v` range, so the mesh closes at the apex instead of stopping
    /// at the base plane.
    #[test]
    fn cone_apex_vertex_loop_survives_mesh_fallback() {
        let src = cone_step(1.0, 2.0).replace(
            "#14 = EDGE_CURVE('', #7, #7, #11, .T.);",
            "#402 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS., .T., #11);\n\
             #403 = COMPOSITE_CURVE('', (#402), .T.);\n\
             #14 = EDGE_CURVE('', #7, #7, #403, .T.);",
        );
        let (_store, _geo, report) = import(&src);
        no_error_diagnostics(&report);
        match &report.solids[0].outcome {
            SolidOutcome::Mesh { mesh, .. } => {
                assert!(mesh.is_closed_manifold());
                let exact = PI * 2.0 / 3.0;
                assert!(
                    (signed_volume(mesh) - exact).abs() / exact < 0.02,
                    "fallback volume {} vs {exact}",
                    signed_volume(mesh)
                );
            }
            other => panic!("expected the mesh fallback, got {other:?}"),
        }
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

    // ---- outer-bound designation (of-he8) ----

    /// A planar face bounded by two concentric squares: a wide 10×10 ring
    /// and a narrow 2×2 one inside it. `wide_first` picks which of them the
    /// face starts out calling its outer bound — the choice a file's bound
    /// order or `FACE_OUTER_BOUND` tagging hands the reader, right or wrong.
    ///
    /// Returns the stores, the face, and the (wide, narrow) loops.
    #[allow(clippy::type_complexity)]
    fn face_with_two_rings(
        wide_first: bool,
    ) -> (
        TopologyStore,
        GeometryStore,
        EntityId<Face>,
        (EntityId<Loop>, EntityId<Loop>),
    ) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Sheet);
        let shell = store.create_shell(body, false, ShellOrientation::Outward);
        let face = store.create_face(shell, FaceSense::Positive);
        let plane =
            geo.add_surface(Surface3::plane(Point3::origin(), Vector3::z()).expect("valid plane"));
        store.faces.get_mut(face).expect("just created").surface = Some(plane);

        // Counterclockwise in the plane's (u, v), so both rings wind the way
        // a Positive face's outer bound should — the designation, not the
        // winding, is what these tests are about.
        let ring = |store: &mut TopologyStore, geo: &mut GeometryStore, half: f64| {
            let corners = [
                Point3::new(-half, -half, 0.0),
                Point3::new(half, -half, 0.0),
                Point3::new(half, half, 0.0),
                Point3::new(-half, half, 0.0),
            ];
            let vertices: Vec<EntityId<Vertex>> = corners
                .iter()
                .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
                .collect();
            let edges: Vec<(EntityId<Edge>, FinSense)> = (0..4)
                .map(|k| {
                    let (from, to) = (corners[k], corners[(k + 1) % 4]);
                    let step = to - from;
                    let curve = geo.add_curve(Curve3::line(from, step).expect("valid line"));
                    let edge = store.create_edge_with_curve(
                        vertices[k],
                        vertices[(k + 1) % 4],
                        SYSTEM_RESOLUTION,
                        curve,
                        0.0,
                        step.norm(),
                    );
                    (edge, FinSense::Forward)
                })
                .collect();
            edges
        };
        let wide_edges = ring(&mut store, &mut geo, 5.0);
        let narrow_edges = ring(&mut store, &mut geo, 1.0);

        let (first, second) = if wide_first {
            (&wide_edges, &narrow_edges)
        } else {
            (&narrow_edges, &wide_edges)
        };
        let first_loop = store.create_loop(face, LoopType::Outer, first);
        let second_loop = store.create_loop(face, LoopType::Inner, second);
        let (wide, narrow) = if wide_first {
            (first_loop, second_loop)
        } else {
            (second_loop, first_loop)
        };

        let attached = attach_body_pcurves(&mut store, &mut geo, body);
        assert_eq!(attached, 8, "both rings' fins must be fittable");
        (store, geo, face, (wide, narrow))
    }

    /// The heart of of-he8: a file that hands the outer role to a hole gets
    /// overruled by the geometry. The ring enclosing the other is the outer
    /// bound whatever the file said.
    #[test]
    fn outer_bound_is_the_ring_that_encloses_the_other() {
        let (mut store, geo, face, (wide, narrow)) = face_with_two_rings(false);
        assert_eq!(store.face(face).unwrap().outer_loop, Some(narrow));

        assert_eq!(choose_outer_bounds(&mut store, &geo, &[face]), 1);

        let f = store.face(face).unwrap();
        assert_eq!(f.outer_loop, Some(wide));
        assert_eq!(f.inner_loops, vec![narrow]);
        assert_eq!(store.loop_(wide).unwrap().loop_type, LoopType::Outer);
        assert_eq!(store.loop_(narrow).unwrap().loop_type, LoopType::Inner);
    }

    /// The overwhelmingly common case: the file was right, and measuring
    /// agrees with it rather than shuffling the loops for nothing.
    #[test]
    fn a_correct_outer_bound_survives_being_measured() {
        let (mut store, geo, face, (wide, narrow)) = face_with_two_rings(true);

        assert_eq!(choose_outer_bounds(&mut store, &geo, &[face]), 0);

        let f = store.face(face).unwrap();
        assert_eq!(f.outer_loop, Some(wide));
        assert_eq!(f.inner_loops, vec![narrow]);
    }

    /// A loop that cannot be measured makes the comparison incomplete, and
    /// half a comparison is worse than none: the largest *readable* ring may
    /// not be the largest ring. The face keeps what it had.
    #[test]
    fn an_unreadable_loop_leaves_the_designation_alone() {
        let (mut store, geo, face, (wide, narrow)) = face_with_two_rings(false);
        let fin = store.fins_of_loop(wide)[0];
        store.fins.get_mut(fin).expect("live fin").pcurve = None;

        assert_eq!(choose_outer_bounds(&mut store, &geo, &[face]), 0);
        assert_eq!(store.face(face).unwrap().outer_loop, Some(narrow));
    }

    /// A face whose surface never arrived cannot be measured either — there
    /// is no parameter space to measure in.
    #[test]
    fn a_face_without_a_surface_leaves_the_designation_alone() {
        let (mut store, geo, face, (_wide, narrow)) = face_with_two_rings(false);
        store.faces.get_mut(face).expect("live face").surface = None;

        assert_eq!(choose_outer_bounds(&mut store, &geo, &[face]), 0);
        assert_eq!(store.face(face).unwrap().outer_loop, Some(narrow));
    }

    /// End to end through the reader: a face with one bound is never in
    /// question, so an import of single-bound faces reports no re-choice.
    #[test]
    fn single_bound_faces_are_never_re_designated() {
        let (_store, _geo, report) = import(&block_step(2.0, 3.0, 4.0));
        assert!(
            !report
                .diagnostics
                .iter()
                .any(|d| d.message.contains("outer bound re-chosen")),
            "{:?}",
            report.diagnostics
        );
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
        match resolve_curve(&file, 5, 0, 1.0, 1.0).unwrap() {
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
        let RawCurve::Nurbs(curve) = resolve_curve(&file, 4, 0, 1.0, 1.0).unwrap() else {
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

    /// The rational spelling: weights live on a `RATIONAL_B_SPLINE_CURVE`
    /// part of a complex instance, and the partial records drop the
    /// inherited `name`, so every attribute index shifts (of-3qy.7).
    #[test]
    fn resolves_rational_bspline_curve_complex_instance() {
        // The standard rational quarter circle: weights (1, √2/2, 1) about
        // the unit circle's control triangle.
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (1., 0., 0.));
             #2 = CARTESIAN_POINT('', (1., 1., 0.));
             #3 = CARTESIAN_POINT('', (0., 1., 0.));
             #4 = ( BOUNDED_CURVE() B_SPLINE_CURVE(2, (#1, #2, #3), .UNSPECIFIED., .F., .U.) \
                  B_SPLINE_CURVE_WITH_KNOTS((3, 3), (0., 1.), .UNSPECIFIED.) CURVE() \
                  GEOMETRIC_REPRESENTATION_ITEM() \
                  RATIONAL_B_SPLINE_CURVE((1., 0.7071067811865476, 1.)) \
                  REPRESENTATION_ITEM('') );",
        );
        let RawCurve::Nurbs(curve) = resolve_curve(&file, 4, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(curve.degree(), 2);
        assert_eq!(curve.weights()[1], std::f64::consts::FRAC_1_SQRT_2);
        // The weights are what make this an exact arc: the midpoint lies on
        // the unit circle, which an unweighted curve would miss by ~0.03.
        let (t0, t1) = curve.knot_vector().domain();
        let mid = curve.point((t0 + t1) / 2.0);
        assert!(
            (mid.coords.norm() - 1.0).abs() < 1e-12,
            "midpoint {mid:?} is not on the unit circle — weights were dropped"
        );
    }

    #[test]
    fn resolves_rational_bspline_surface_complex_instance() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (1., 0., 0.));
             #2 = CARTESIAN_POINT('', (1., 1., 0.));
             #3 = CARTESIAN_POINT('', (0., 1., 0.));
             #4 = CARTESIAN_POINT('', (1., 0., 1.));
             #5 = CARTESIAN_POINT('', (1., 1., 1.));
             #6 = CARTESIAN_POINT('', (0., 1., 1.));
             #7 = ( BOUNDED_SURFACE() \
                  B_SPLINE_SURFACE(1, 2, ((#1, #2, #3), (#4, #5, #6)), .UNSPECIFIED., \
                  .F., .F., .U.) \
                  B_SPLINE_SURFACE_WITH_KNOTS((2, 2), (3, 3), (0., 1.), (0., 1.), \
                  .UNSPECIFIED.) GEOMETRIC_REPRESENTATION_ITEM() \
                  RATIONAL_B_SPLINE_SURFACE(((1., 0.7071067811865476, 1.), \
                  (1., 0.7071067811865476, 1.))) REPRESENTATION_ITEM('') SURFACE() );",
        );
        let RawSurface::Nurbs(surface) = resolve_surface(&file, 7, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS surface");
        };
        assert_eq!(surface.degree_u(), 1);
        assert_eq!(surface.degree_v(), 2);
        assert_eq!(surface.grid_size(), (2, 3));
        assert_eq!(surface.weight(1, 1), std::f64::consts::FRAC_1_SQRT_2);
        // A quarter-cylinder: every v-isoline is the exact unit arc.
        let mid = surface.point(0.5, 0.5);
        assert!(
            (mid.coords.xy().norm() - 1.0).abs() < 1e-12,
            "patch midpoint {mid:?} is off the unit cylinder — weights were dropped"
        );
    }

    /// The complex spelling without the `RATIONAL_` part — some exporters
    /// emit it for plain B-splines. Unit weights are the right reading.
    #[test]
    fn resolves_complex_bspline_without_rational_part_as_unweighted() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (1., 1., 0.));
             #3 = CARTESIAN_POINT('', (2., 0., 0.));
             #4 = ( BOUNDED_CURVE() B_SPLINE_CURVE(2, (#1, #2, #3), .UNSPECIFIED., .F., .U.) \
                  B_SPLINE_CURVE_WITH_KNOTS((3, 3), (0., 1.), .UNSPECIFIED.) CURVE() \
                  GEOMETRIC_REPRESENTATION_ITEM() REPRESENTATION_ITEM('') );",
        );
        let RawCurve::Nurbs(curve) = resolve_curve(&file, 4, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(curve.weights(), &[1.0, 1.0, 1.0]);
    }

    /// A complex surface with no `B_SPLINE_SURFACE_WITH_KNOTS` part states
    /// its knots through a subtype instead. That used to be refused; of-3qy.8
    /// derives them, so the patch now resolves exactly.
    #[test]
    fn resolves_complex_quasi_uniform_surface_with_derived_knots() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (0., 1., 0.));
             #3 = CARTESIAN_POINT('', (1., 0., 0.));
             #4 = CARTESIAN_POINT('', (1., 1., 1.));
             #5 = ( BOUNDED_SURFACE() \
                  B_SPLINE_SURFACE(1, 1, ((#1, #2), (#3, #4)), .UNSPECIFIED., .F., .F., .U.) \
                  GEOMETRIC_REPRESENTATION_ITEM() QUASI_UNIFORM_SURFACE() \
                  REPRESENTATION_ITEM('') SURFACE() );",
        );
        let RawSurface::Nurbs(surface) = resolve_surface(&file, 5, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS surface");
        };
        assert_eq!(surface.knot_vector_u().domain(), (0.0, 1.0));
        assert!((surface.point(0.0, 0.0) - Point3::new(0.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((surface.point(1.0, 1.0) - Point3::new(1.0, 1.0, 1.0)).norm() < 1e-12);
    }

    /// `QUASI_UNIFORM_*` states no knots at all: they are derived clamped
    /// and one unit per span, so the domain is `[0, count - degree]`.
    #[test]
    fn resolves_quasi_uniform_curve_with_derived_knots() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (1., 1., 0.));
             #3 = CARTESIAN_POINT('', (2., 1., 0.));
             #4 = CARTESIAN_POINT('', (3., 0., 0.));
             #5 = QUASI_UNIFORM_CURVE('', 2, (#1, #2, #3, #4), .UNSPECIFIED., .F., .U.);",
        );
        let RawCurve::Nurbs(curve) = resolve_curve(&file, 5, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(
            curve.knot_vector().knots(),
            &[0.0, 0.0, 0.0, 1.0, 2.0, 2.0, 2.0]
        );
        assert_eq!(curve.knot_vector().domain(), (0.0, 2.0));
        // Clamped, so it interpolates its end control points.
        assert!((curve.point(0.0) - Point3::new(0.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((curve.point(2.0) - Point3::new(3.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn resolves_quasi_uniform_surface_with_derived_knots() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (0., 1., 0.));
             #3 = CARTESIAN_POINT('', (1., 0., 0.));
             #4 = CARTESIAN_POINT('', (1., 1., 0.));
             #5 = QUASI_UNIFORM_SURFACE('', 1, 1, ((#1, #2), (#3, #4)), .PLANE_SURF., \
                  .F., .F., .U.);",
        );
        let RawSurface::Nurbs(surface) = resolve_surface(&file, 5, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS surface");
        };
        assert_eq!(surface.knot_vector_u().domain(), (0.0, 1.0));
        assert!((surface.point(0.5, 0.5) - Point3::new(0.5, 0.5, 0.0)).norm() < 1e-12);
    }

    /// A Bézier is a single span: `count == degree + 1`, knots `(0, 1)` at
    /// full multiplicity. Anything else is a malformed record.
    #[test]
    fn resolves_bezier_curve_and_rejects_a_wrong_control_count() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (1., 2., 0.));
             #3 = CARTESIAN_POINT('', (2., 0., 0.));
             #4 = BEZIER_CURVE('', 2, (#1, #2, #3), .UNSPECIFIED., .F., .U.);
             #5 = BEZIER_CURVE('', 1, (#1, #2, #3), .UNSPECIFIED., .F., .U.);",
        );
        let RawCurve::Nurbs(curve) = resolve_curve(&file, 4, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(curve.knot_vector().knots(), &[0.0, 0.0, 0.0, 1.0, 1.0, 1.0]);
        // Apex of the quadratic Bézier: halfway to the middle control point.
        assert!((curve.point(0.5) - Point3::new(1.0, 1.0, 0.0)).norm() < 1e-12);

        let Err(err) = resolve_curve(&file, 5, 0, 1.0, 1.0) else {
            panic!("expected a degree/control-count mismatch");
        };
        assert!(
            err.diagnostic()
                .message
                .contains("Bezier form has 3 control points"),
            "{err:?}"
        );
    }

    /// `UNIFORM_*` is the unclamped form: the domain starts `degree` knots
    /// in, so the curve never reaches its first control point.
    #[test]
    fn resolves_uniform_curve_with_unclamped_knots() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (1., 3., 0.));
             #3 = CARTESIAN_POINT('', (2., 0., 0.));
             #4 = UNIFORM_CURVE('', 2, (#1, #2, #3), .UNSPECIFIED., .F., .U.);",
        );
        let RawCurve::Nurbs(curve) = resolve_curve(&file, 4, 0, 1.0, 1.0).unwrap() else {
            panic!("expected a NURBS curve");
        };
        assert_eq!(
            curve.knot_vector().knots(),
            &[-2.0, -1.0, 0.0, 1.0, 2.0, 3.0]
        );
        assert_eq!(curve.knot_vector().domain(), (0.0, 1.0));
        // Start of the domain is the de Boor average, not control point #1.
        assert!(curve.point(0.0).y > 0.0);
    }

    /// A complex instance inheriting `B_SPLINE_CURVE` but stating its knots
    /// neither explicitly nor by subtype is not parameterizable — it must
    /// report unsupported rather than silently guessing a knot vector.
    #[test]
    fn rejects_a_knotless_bspline_complex_instance() {
        let file = parse_fixture(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (1., 1., 0.));
             #3 = (BOUNDED_CURVE() B_SPLINE_CURVE(1, (#1, #2), .POLYLINE_FORM., .F., .F.) \
                  CURVE() GEOMETRIC_REPRESENTATION_ITEM() REPRESENTATION_ITEM(''));",
        );
        let Err(err) = resolve_curve(&file, 3, 0, 1.0, 1.0) else {
            panic!("expected a knotless complex instance to be refused");
        };
        assert!(
            err.diagnostic().message.contains("complex curve instance"),
            "{err:?}"
        );
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

    /// The other spelling of a full circle: two `VERTEX_POINT` entities at
    /// the same place, so `closed` is false but the edge still goes all the
    /// way round (of-zdx). Reading the sweep literally gives zero.
    #[test]
    fn coincident_vertices_span_the_full_circle() {
        let circle = Curve3::circle(Point3::new(14.0, 0.0, 10.0), Vector3::z(), 6.0).unwrap();
        // The two coordinates OCC writes for the tangency point, which differ
        // only in the last bits of the y component.
        let start = Point3::new(20.0, -7.347880794884e-16, 10.0);
        let end = Point3::new(20.0, 0.0, 10.0);
        let trimmed = trim_curve(&circle, true, start, end, false, 1).unwrap();
        assert!((trimmed.t_end - trimmed.t_start - TAU).abs() < 1e-12);
        // Halfway round is the far side of the circle, not the seam.
        let mid = trimmed.curve.point((trimmed.t_start + trimmed.t_end) / 2.0);
        assert!((mid - Point3::new(8.0, 0.0, 10.0)).norm() < 1e-12, "{mid}");
    }

    /// The same coincidence on a line is not a closed edge — a line has no
    /// period to come back through — so it stays an error.
    #[test]
    fn coincident_vertices_on_a_line_are_still_refused() {
        let line = Curve3::line(Point3::origin(), Vector3::x()).unwrap();
        let vertex = Point3::new(3.0, 0.0, 0.0);
        let err = trim_curve(&line, true, vertex, vertex, false, 7).unwrap_err();
        assert!(
            matches!(err, MapError::Invalid { entity: 7, .. }),
            "{err:?}"
        );
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

    /// A trim that misses its vertices must say by how much: STEP writes
    /// finite decimals, so a vertex point and the curve it sits on are
    /// rounded apart, and the vertex carries the difference as its tolerance
    /// rather than claiming a precision the file never had (of-bbh8).
    #[test]
    fn trim_reports_how_far_it_misses_its_vertices() {
        let circle = Curve3::circle(Point3::origin(), Vector3::z(), 1.0).unwrap();
        // Off-radius at the start, off-plane at the end: two different ways
        // for a rounded coordinate to leave the curve.
        let trimmed = trim_curve(
            &circle,
            true,
            Point3::new(1.0 + 2e-7, 0.0, 0.0),
            Point3::new(0.0, 1.0, 3e-7),
            false,
            1,
        )
        .unwrap();
        assert!(
            (trimmed.start_residual - 2e-7).abs() < 1e-15,
            "got {}",
            trimmed.start_residual
        );
        assert!(
            (trimmed.end_residual - 3e-7).abs() < 1e-15,
            "got {}",
            trimmed.end_residual
        );

        // A curve that interpolates its vertices exactly owes nothing.
        let exact = trim_curve(
            &circle,
            true,
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            false,
            1,
        )
        .unwrap();
        assert!(exact.start_residual < 1e-15 && exact.end_residual < 1e-15);
    }

    /// A miss the kernel would not accept as a tolerance is refused outright,
    /// so the exact path never builds a body `check` rejects for tolerance
    /// alone. Only reachable above ten metres at millimetre scale, where
    /// `TRIM_TOL_REL` is the looser of the two bounds.
    #[test]
    fn trim_rejects_a_miss_beyond_the_kernel_tolerance_limit() {
        let circle = Curve3::circle(Point3::origin(), Vector3::z(), 5.0e4).unwrap();
        // 0.02 mm off a 50 m radius: inside the relative trim bound
        // (1e-6 x 5e4 = 0.05) and outside MAX_ALLOWED_TOLERANCE (0.01).
        let err = trim_curve(
            &circle,
            true,
            Point3::new(5.0e4 + 0.02, 0.0, 0.0),
            Point3::new(0.0, 5.0e4, 0.0),
            false,
            7,
        )
        .unwrap_err();
        assert!(
            matches!(err, MapError::Invalid { entity: 7, .. }),
            "{err:?}"
        );
        // The same geometry inside the limit still imports.
        let ok = trim_curve(
            &circle,
            true,
            Point3::new(5.0e4 + 0.002, 0.0, 0.0),
            Point3::new(0.0, 5.0e4, 0.0),
            false,
            7,
        )
        .unwrap();
        assert!(ok.start_residual <= MAX_ALLOWED_TOLERANCE);
    }

    // ---- edge tolerances (of-bb6) ----

    /// A 2 mm block whose bottom face's `PLANE` is tilted by `slope` about
    /// the corner it is anchored at, leaving everything else — the eight
    /// `VERTEX_POINT`s, the twelve `LINE` edges, the winding, the senses —
    /// exactly as the plain fixture writes them. So the four edges bounding
    /// that face are off *its* surface and still exactly on their other one,
    /// which is the shape of what real files do (of-bb6) with none of a real
    /// file's other defects.
    ///
    /// The plane passes through corner `(-1, -1, -1)`, so the deviation grows
    /// with `x` and its largest value over the boundary is at `x = 1`:
    /// [`tilt_deviation`].
    fn block_with_a_tilted_face(slope: f64) -> String {
        let src = block_step(2.0, 2.0, 2.0);
        let flat = "DIRECTION('', (0.000000, 0.000000, -1.000000));";
        assert_eq!(
            src.matches(flat).count(),
            1,
            "only the -Z face normal may match; edge directions carry the \
             corner-to-corner delta, not a unit vector"
        );
        src.replace(
            flat,
            &format!("DIRECTION('', ({slope:.6}, 0.000000, -1.000000));"),
        )
    }

    /// Distance from the tilted plane to the far side of the face it bounds:
    /// the displacement `(2, *, 0)` from the anchor corner, along the unit
    /// normal `(slope, 0, -1) / |(slope, 0, -1)|`.
    fn tilt_deviation(slope: f64) -> f64 {
        2.0 * slope / (1.0 + slope * slope).sqrt()
    }

    /// An imported edge carries how far its curve really sits from the
    /// surfaces of the faces it bounds, not `SYSTEM_RESOLUTION` (of-bb6).
    /// Only the edges that are off something are raised, and the body the
    /// reader hands back is geometrically clean because of it.
    #[test]
    fn an_edge_off_its_face_imports_with_the_measured_tolerance() {
        let slope = 1.0e-4;
        let (store, geo, report) = import(&block_with_a_tilted_face(slope));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert_eq!(
            store.check_with_geometry(&geo, body),
            Vec::new(),
            "the measurement is what makes the import clean"
        );

        let tilted = store
            .faces_of_body(body)
            .into_iter()
            .find(|&f| {
                matches!(
                    store.face(f).expect("live face").surface.and_then(|id| geo.surface(id)),
                    Some(Surface3::Plane { normal, .. }) if normal.x != 0.0
                )
            })
            .expect("one face's plane is tilted");
        let on_tilted = store.edges_of_face(tilted);
        let mut raised: Vec<EntityId<Edge>> = Vec::new();
        for face in store.faces_of_body(body) {
            for edge in store.edges_of_face(face) {
                let tolerance = store.edge(edge).expect("live edge").tolerance;
                if tolerance > SYSTEM_RESOLUTION && !raised.contains(&edge) {
                    raised.push(edge);
                }
            }
        }
        // Three of the tilted face's four edges. The fourth runs through the
        // corner the plane is anchored at and stays on it however far the
        // face tilts — a measurement that raised that one would be measuring
        // noise, and every edge of the other five faces likewise.
        assert_eq!(raised.len(), 3, "{raised:?}");
        assert!(
            raised.iter().all(|e| on_tilted.contains(e)),
            "only the tilted face's edges are off anything: {raised:?}"
        );

        let expected = tilt_deviation(slope);
        let worst = raised
            .iter()
            .map(|&e| store.edge(e).expect("live edge").tolerance)
            .fold(0.0f64, f64::max);
        assert!(
            (worst - expected).abs() < 1e-12,
            "expected the measured {expected}, got {worst}"
        );
    }

    /// Past [`MAX_ALLOWED_TOLERANCE`] there is no tolerance the kernel could
    /// give the edge honestly, so the solid does not take the exact path at
    /// all and degrades to tessellation — the same refusal
    /// [`trim_rejects_a_miss_beyond_the_kernel_tolerance_limit`] makes for a
    /// vertex, and for the same reason: the exact path must not build a body
    /// `check` would reject for tolerance alone.
    #[test]
    fn an_edge_too_far_off_its_face_degrades_to_tessellation() {
        // 0.04 mm on a 2 mm block, four times the kernel's cap.
        let slope = 0.02;
        assert!(tilt_deviation(slope) > MAX_ALLOWED_TOLERANCE);
        let (_, _, report) = import(&block_with_a_tilted_face(slope));
        assert!(
            matches!(report.solids[0].outcome, SolidOutcome::Mesh { .. }),
            "expected a tessellated fallback, got {:?}",
            report.solids[0].outcome
        );
        assert!(
            report.diagnostics.iter().any(|d| {
                d.severity == Severity::Warning
                    && d.message
                        .contains("more than the kernel's maximum tolerance")
            }),
            "the refusal must say what it refused: {:?}",
            report.diagnostics
        );
    }

    /// A tolerance already covering a wider gap — one healing set, say —
    /// survives the measurement: an edge's tolerance only ever rises, since
    /// the healer's figure answers a question this measurement is not asking.
    #[test]
    fn recording_only_raises_a_tolerance() {
        let (mut store, geo, report) = import(&block_step(2.0, 2.0, 2.0));
        let body = brep_body(&report.solids[0].outcome);
        let edge = store.edges_of_face(store.faces_of_body(body)[0])[0];
        let wide = MAX_ALLOWED_TOLERANCE / 2.0;
        store.edges.get_mut(edge).expect("live edge").tolerance = wide;

        let raised = record_edge_tolerances(&mut store, &geo, body).expect("a block is exact");
        assert_eq!(raised, 0, "a block's edges lie on their faces exactly");
        assert_eq!(store.edge(edge).expect("live edge").tolerance, wide);
    }

    /// Open cubic B-spline over `[0, 2]`, so a trim recovering `(0, 1)`
    /// would be visibly wrong rather than coincidentally right.
    fn bspline_arc() -> Curve3 {
        Curve3::nurbs(
            NurbsCurve::bspline(
                vec![
                    Point3::new(0.0, 0.0, 0.0),
                    Point3::new(1.0, 2.0, 0.0),
                    Point3::new(3.0, 2.0, 1.0),
                    Point3::new(4.0, 0.0, 0.0),
                ],
                KnotVector::new(3, vec![0.0, 0.0, 0.0, 0.0, 2.0, 2.0, 2.0, 2.0]).unwrap(),
            )
            .unwrap(),
        )
    }

    #[test]
    fn trims_a_bspline_by_projecting_its_vertices() {
        let curve = bspline_arc();
        let (t0, t1) = curve.domain();
        let (start, end) = (curve.point(t0), curve.point(t1));
        let trimmed = trim_curve(&curve, true, start, end, false, 1).unwrap();
        assert!((trimmed.t_start - t0).abs() < 1e-9);
        assert!((trimmed.t_end - t1).abs() < 1e-9);
        // An interior vertex trims to an interior parameter, not the end.
        let mid = curve.point(1.25);
        let partial = trim_curve(&curve, true, start, mid, false, 1).unwrap();
        assert!((partial.t_end - 1.25).abs() < 1e-6, "got {}", partial.t_end);
    }

    #[test]
    fn same_sense_false_reverses_a_bspline() {
        let curve = bspline_arc();
        let (t0, t1) = curve.domain();
        // The edge runs against the curve: end vertex first.
        let (start, end) = (curve.point(t1), curve.point(t0));
        let trimmed = trim_curve(&curve, false, start, end, false, 1).unwrap();
        assert!(trimmed.t_start < trimmed.t_end, "normalized trim direction");
        assert!(matches!(trimmed.curve, Curve3::Nurbs(_)), "stays exact");
        // The reversed curve interpolates the edge's vertices in order.
        assert!((trimmed.curve.point(trimmed.t_start) - start).norm() < 1e-9);
        assert!((trimmed.curve.point(trimmed.t_end) - end).norm() < 1e-9);
    }

    #[test]
    fn closed_bspline_edge_spans_the_whole_knot_interval() {
        // Control net returning to its start: the two domain ends meet, so
        // the edge takes the full interval rather than adding a period a
        // clamped B-spline does not have.
        let curve = Curve3::nurbs(
            NurbsCurve::bspline(
                vec![
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.5, 0.0),
                    Point3::new(-1.5, 0.0, 0.0),
                    Point3::new(0.0, -1.5, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                ],
                KnotVector::new(2, vec![0.0, 0.0, 0.0, 1.0, 2.0, 3.0, 3.0, 3.0]).unwrap(),
            )
            .unwrap(),
        );
        let vertex = curve.point(0.0);
        let trimmed = trim_curve(&curve, true, vertex, vertex, true, 1).unwrap();
        assert_eq!((trimmed.t_start, trimmed.t_end), (0.0, 3.0));
    }

    #[test]
    fn trim_rejects_bspline_vertices_off_the_curve() {
        let curve = bspline_arc();
        let start = curve.point(0.0);
        let err =
            trim_curve(&curve, true, start, Point3::new(0.0, 9.0, 9.0), false, 7).unwrap_err();
        assert!(
            matches!(err, MapError::Invalid { entity: 7, .. }),
            "{err:?}"
        );
    }

    #[test]
    fn reverse_curve_mirrors_a_bspline() {
        let curve = bspline_arc();
        let (t0, t1) = curve.domain();
        let back = reverse_curve(&curve);
        assert_eq!(back.domain(), (t0, t1));
        for i in 0..=10 {
            let t = t0 + (t1 - t0) * i as f64 / 10.0;
            assert!((back.point(t) - curve.point(t0 + t1 - t)).norm() < 1e-12);
        }
    }

    #[test]
    fn bspline_has_no_conic_angle() {
        let curve = bspline_arc();
        assert_eq!(conic_angle(&curve, &curve.point(1.0)), None);
    }

    #[test]
    fn trim_rejects_closed_edge_on_a_line() {
        let line = Curve3::line(Point3::origin(), Vector3::x()).unwrap();
        let err = trim_curve(&line, true, Point3::origin(), Point3::origin(), true, 7).unwrap_err();
        assert!(matches!(err, MapError::Invalid { .. }));
    }

    // ---- mesh fallback ----

    #[test]
    fn unbounded_nurbs_block_falls_back_to_watertight_mesh() {
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
                .any(|d| d.message.contains("no bounds")),
            "expected the unbounded-face diagnostic: {:?}",
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

    // ------------------------------------------------------------------
    // Schema breadth (of-3qy.9): voids, swept surfaces, trimmed/composite
    // curves, parabola/hyperbola
    // ------------------------------------------------------------------

    #[test]
    fn conic_parameterizations_invert_and_reverse() {
        let parabola = ConicCurve::Parabola {
            location: Point3::new(1.0, 2.0, 3.0),
            x_dir: Vector3::x(),
            y_dir: Vector3::y(),
            focal: 0.75,
        };
        let hyperbola = ConicCurve::Hyperbola {
            location: Point3::new(-1.0, 0.5, 0.0),
            x_dir: Vector3::y(),
            y_dir: Vector3::z(),
            a: 2.0,
            b: 0.5,
        };
        for conic in [&parabola, &hyperbola] {
            for t in [-1.5, -0.25, 0.0, 0.4, 2.0] {
                let p = conic.point(t);
                assert!((conic.param_of(&p) - t).abs() < 1e-12, "param roundtrip");
                let r = conic.reversed();
                assert!(
                    (r.point(-t) - p).norm() < 1e-12,
                    "reversal is the t → -t relabeling"
                );
                assert!((r.param_of(&p) - (-t)).abs() < 1e-12);
            }
        }
    }

    #[test]
    fn extrusion_reduces_to_plane_and_cylinder() {
        let line = Curve3::line(Point3::new(1.0, 0.0, 0.0), Vector3::x()).unwrap();
        let plane = reduce_extrusion(Some(&line), &Vector3::new(0.0, 0.0, 2.0), 1)
            .unwrap()
            .expect("line extrudes to a plane");
        assert!(matches!(plane, Surface3::Plane { .. }));

        let circle = Curve3::circle(Point3::new(0.0, 0.0, -1.0), Vector3::z(), 1.5).unwrap();
        let cylinder = reduce_extrusion(Some(&circle), &Vector3::new(0.0, 0.0, 3.0), 1)
            .unwrap()
            .expect("circle along its normal extrudes to a cylinder");
        assert!(
            matches!(cylinder, Surface3::Cylinder { radius, .. } if (radius - 1.5).abs() < 1e-12)
        );

        // A circle swept obliquely is no quadric the store holds.
        let skew = reduce_extrusion(Some(&circle), &Vector3::new(1.0, 0.0, 3.0), 1).unwrap();
        assert!(skew.is_none());
        // Sweeping a line along itself is degenerate, not a plane.
        assert!(reduce_extrusion(Some(&line), &Vector3::x(), 1).is_err());
    }

    /// One `ADVANCED_FACE` on a `SURFACE_OF_LINEAR_EXTRUSION` of a NURBS
    /// profile, spelled the way `bspline_patch_prism.stp` does it: the
    /// `VECTOR` is one unit of `+z` and the face reaches 20 units along
    /// `−z`. The face's own bounds are the only statement of extent, so
    /// the patch must be built from them (of-8ulj).
    fn nurbs_extrusion_face_fixture(depth: f64) -> StepFile {
        parse_fixture(&format!(
            "#1 = CARTESIAN_POINT('', (0., 0., 0.));
             #2 = CARTESIAN_POINT('', (0., 10., 0.));
             #3 = CARTESIAN_POINT('', (0., 10., {depth:.6}));
             #4 = CARTESIAN_POINT('', (0., 0., {depth:.6}));
             #5 = VERTEX_POINT('', #1);
             #6 = VERTEX_POINT('', #2);
             #7 = VERTEX_POINT('', #3);
             #8 = VERTEX_POINT('', #4);
             #9 = DIRECTION('', (0., 0., 1.));
             #10 = VECTOR('', #9, 1.);
             #11 = B_SPLINE_CURVE_WITH_KNOTS('', 1, (#1, #2), .UNSPECIFIED., .F., .F., \
                   (2, 2), (0., 1.), .PIECEWISE_BEZIER_KNOTS.);
             #12 = SURFACE_OF_LINEAR_EXTRUSION('', #11, #10);
             #13 = DIRECTION('', (0., 1., 0.));
             #14 = VECTOR('', #13, 1.);
             #15 = DIRECTION('', (0., 0., -1.));
             #16 = VECTOR('', #15, 1.);
             #17 = LINE('', #1, #14);
             #18 = LINE('', #2, #16);
             #19 = LINE('', #4, #14);
             #20 = LINE('', #1, #16);
             #21 = EDGE_CURVE('', #5, #6, #17, .T.);
             #22 = EDGE_CURVE('', #6, #7, #18, .T.);
             #23 = EDGE_CURVE('', #8, #7, #19, .T.);
             #24 = EDGE_CURVE('', #5, #8, #20, .T.);
             #25 = ORIENTED_EDGE('', *, *, #21, .T.);
             #26 = ORIENTED_EDGE('', *, *, #22, .T.);
             #27 = ORIENTED_EDGE('', *, *, #23, .F.);
             #28 = ORIENTED_EDGE('', *, *, #24, .F.);
             #29 = EDGE_LOOP('', (#25, #26, #27, #28));
             #30 = FACE_OUTER_BOUND('', #29, .T.);
             #31 = ADVANCED_FACE('', (#30), #12, .T.);"
        ))
    }

    /// The patch a face gets covers the face — in the direction the face
    /// actually reaches, not the one unit of `VECTOR` the entity spells.
    #[test]
    fn extruded_nurbs_patch_is_sized_to_its_face_not_its_vector() {
        let file = nurbs_extrusion_face_fixture(-20.0);
        let RawSurface::ExtrudedNurbs { curve, dir } =
            resolve_surface(&file, 12, 0, 1.0, 1.0).expect("the extrusion resolves")
        else {
            panic!("a NURBS basis extrudes to a ruled patch");
        };
        // The entity on its own says +z, one unit long.
        assert!((dir - Vector3::z()).norm() < 1e-12);

        let patch = extruded_nurbs_surface(&file, &[30], 31, 1.0, 1.0, &curve, &dir)
            .expect("the face's bounds size the patch");
        // v = 0 is the far row (−20), v = 1 the near one, both just past
        // the face by the margin.
        let margin = 20.0 * EXTRUSION_SPAN_MARGIN;
        for u in [0.0, 0.5, 1.0] {
            assert!(
                (patch.point(u, 0.0).z + 20.0 + margin).abs() < 1e-9,
                "far row at u = {u} is {:?}",
                patch.point(u, 0.0)
            );
            assert!(
                (patch.point(u, 1.0).z - margin).abs() < 1e-9,
                "near row at u = {u} is {:?}",
                patch.point(u, 1.0)
            );
        }
        // Every corner of the face lies on the patch (the bug: they used to
        // lie 20 mm off it, since the patch stopped one unit the other way).
        for (corner, v) in [
            (Point3::new(0.0, 10.0, -20.0), 0.0),
            (Point3::new(0.0, 0.0, -20.0), 0.0),
            (Point3::new(0.0, 10.0, 0.0), 1.0),
        ] {
            let u = corner.y / 10.0;
            let v = v * (1.0 - EXTRUSION_SPAN_MARGIN) + (1.0 - v) * EXTRUSION_SPAN_MARGIN;
            assert!(
                (patch.point(u, v) - corner).norm() < 1e-9,
                "{corner:?} is off the patch at ({u}, {v}): {:?}",
                patch.point(u, v)
            );
        }
    }

    /// A face whose bounds do not move along the sweep at all sizes no
    /// patch: better a structured refusal (and the mesh fallback) than a
    /// degenerate surface.
    #[test]
    fn an_extrusion_face_that_does_not_span_the_sweep_is_refused() {
        let file = nurbs_extrusion_face_fixture(0.0);
        let RawSurface::ExtrudedNurbs { curve, dir } =
            resolve_surface(&file, 12, 0, 1.0, 1.0).unwrap()
        else {
            panic!("a NURBS basis extrudes to a ruled patch");
        };
        let err = extruded_nurbs_surface(&file, &[30], 31, 1.0, 1.0, &curve, &dir)
            .expect_err("a flat face spans nothing");
        assert!(
            err.diagnostic().message.contains("does not span"),
            "unexpected error: {:?}",
            err.diagnostic()
        );
    }

    /// The extent a bounding curve is sized against must *contain* the
    /// curve, whatever kind it is — a bound that is too tight is exactly
    /// the patch-too-short bug again.
    #[test]
    fn curve_extent_contains_every_curve_kind() {
        let origin = Point3::origin();
        let unit = Vector3::z();
        let ends = [Point3::new(0.0, 0.0, -3.0), Point3::new(1.0, 0.0, 5.0)];

        // A segment is exactly its endpoints.
        let line = RawCurve::Analytic(Curve3::line(ends[0], Vector3::x()).unwrap());
        assert_eq!(curve_extent(&line, ends, &origin, &unit), (-3.0, 5.0));

        // A circle in a plane normal to the sweep has no extent along it;
        // one standing on edge reaches its full radius either way.
        let flat = RawCurve::Analytic(
            Curve3::circle(Point3::new(0.0, 0.0, 4.0), Vector3::z(), 2.0).unwrap(),
        );
        assert_eq!(curve_extent(&flat, ends, &origin, &unit), (4.0, 4.0));
        let upright = RawCurve::Analytic(
            Curve3::circle(Point3::new(0.0, 0.0, 4.0), Vector3::x(), 2.0).unwrap(),
        );
        assert_eq!(curve_extent(&upright, ends, &origin, &unit), (2.0, 6.0));

        // An ellipse reaches its major radius along the sweep when its
        // major axis points that way.
        let ellipse = RawCurve::Analytic(
            Curve3::ellipse(
                Point3::new(0.0, 0.0, 1.0),
                Vector3::x(),
                Vector3::z(),
                3.0,
                0.5,
            )
            .unwrap(),
        );
        assert_eq!(curve_extent(&ellipse, ends, &origin, &unit), (-2.0, 4.0));

        // A NURBS curve is bounded by its control hull, which contains it.
        let spline = NurbsCurve::bspline(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 9.0),
                Point3::new(2.0, 0.0, 0.0),
            ],
            KnotVector::new(2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap(),
        )
        .unwrap();
        let hull = curve_extent(
            &RawCurve::Nurbs(Box::new(spline.clone())),
            ends,
            &origin,
            &unit,
        );
        assert_eq!(hull, (0.0, 9.0));
        // The curve itself peaks at half the hull's height, well inside it.
        assert!((spline.point(0.5).z - 4.5).abs() < 1e-12);

        // Wrappers are transparent, as they are everywhere else here.
        let wrapped = RawCurve::OnSurface {
            basis: Box::new(RawCurve::Analytic(
                Curve3::circle(Point3::new(0.0, 0.0, 4.0), Vector3::x(), 2.0).unwrap(),
            )),
        };
        assert_eq!(curve_extent(&wrapped, ends, &origin, &unit), (2.0, 6.0));
    }

    #[test]
    fn revolution_reduces_to_quadrics() {
        let axis = Vector3::z();
        let origin = Point3::origin();

        // Parallel line → cylinder of the line-to-axis distance.
        let parallel = Curve3::line(Point3::new(2.0, 0.0, -1.0), Vector3::z()).unwrap();
        let cyl = reduce_revolution(Some(&parallel), origin, &axis, 1)
            .unwrap()
            .expect("cylinder");
        assert!(matches!(cyl, Surface3::Cylinder { radius, .. } if (radius - 2.0).abs() < 1e-12));

        // Perpendicular line → the plane holding it.
        let perp = Curve3::line(Point3::new(1.0, 0.0, 4.0), Vector3::x()).unwrap();
        let plane = reduce_revolution(Some(&perp), origin, &axis, 1)
            .unwrap()
            .expect("plane");
        assert!(matches!(plane, Surface3::Plane { .. }));

        // Oblique line meeting the axis → cone with the apex at the
        // intersection, opening toward the line's anchor point.
        let oblique =
            Curve3::line(Point3::new(1.0, 0.0, 0.0), Vector3::new(-1.0, 0.0, 1.0)).unwrap();
        let cone = reduce_revolution(Some(&oblique), origin, &axis, 1)
            .unwrap()
            .expect("cone");
        match cone {
            Surface3::Cone {
                origin: apex,
                axis: cone_axis,
                half_angle,
                radius,
            } => {
                assert!((apex - Point3::new(0.0, 0.0, 1.0)).norm() < 1e-9);
                assert!((half_angle - FRAC_PI_2 / 2.0).abs() < 1e-9, "45 degrees");
                assert_eq!(radius, 0.0);
                // The anchor (1, 0, 0) sits below the apex.
                assert!(cone_axis.z < 0.0);
            }
            other => panic!("expected a cone, got {other:?}"),
        }

        // A skew line sweeps a hyperboloid: no reduction.
        let skew = Curve3::line(Point3::new(1.0, 1.0, 0.0), Vector3::new(0.0, 1.0, 1.0)).unwrap();
        assert!(
            reduce_revolution(Some(&skew), origin, &axis, 1)
                .unwrap()
                .is_none()
        );

        // Meridian circle centered on the axis → sphere; off-axis → torus;
        // tube crossing the axis → no reduction.
        let meridian = Curve3::circle(Point3::origin(), Vector3::new(0.0, -1.0, 0.0), 2.0).unwrap();
        let sphere = reduce_revolution(Some(&meridian), origin, &axis, 1)
            .unwrap()
            .expect("sphere");
        assert!(matches!(sphere, Surface3::Sphere { radius, .. } if (radius - 2.0).abs() < 1e-12));

        let tube = Curve3::circle(
            Point3::new(3.0, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            1.0,
        )
        .unwrap();
        let torus = reduce_revolution(Some(&tube), origin, &axis, 1)
            .unwrap()
            .expect("torus");
        assert!(matches!(
            torus,
            Surface3::Torus {
                major_radius,
                minor_radius,
                ..
            } if (major_radius - 3.0).abs() < 1e-12 && (minor_radius - 1.0).abs() < 1e-12
        ));

        let crossing = Curve3::circle(
            Point3::new(0.5, 0.0, 0.0),
            Vector3::new(0.0, -1.0, 0.0),
            1.0,
        )
        .unwrap();
        assert!(
            reduce_revolution(Some(&crossing), origin, &axis, 1)
                .unwrap()
                .is_none()
        );
    }

    // ---- pcurves (of-3qy.11) ----

    /// The cylinder fixture with its seam edge's geometry spelled the way
    /// production CAD systems spell it: a `SEAM_CURVE` over the 3D line,
    /// carrying a `PCURVE` per branch of the wall's parameter space.
    fn cylinder_step_with_seam_curve(r: f64, h: f64) -> String {
        let src = cylinder_step(r, h);
        let seam_entities = "\
#40 = SEAM_CURVE('', #16, (#41, #45), .CURVE_3D.);
#41 = PCURVE('', #22, #42);
#42 = DEFINITIONAL_REPRESENTATION('', (#43), #47);
#43 = LINE('', #44, #46);
#44 = CARTESIAN_POINT('', (0., 0.));
#45 = PCURVE('', #22, #48);
#46 = VECTOR('', #49, 1.);
#47 = ( GEOMETRIC_REPRESENTATION_CONTEXT(2) PARAMETRIC_REPRESENTATION_CONTEXT() \
REPRESENTATION_CONTEXT('2D SPACE','') );
#48 = DEFINITIONAL_REPRESENTATION('', (#50), #47);
#49 = DIRECTION('', (0., 1.));
#50 = LINE('', #51, #46);
#51 = CARTESIAN_POINT('', (6.283185307179586, 0.));
#19 = EDGE_CURVE('', #8, #9, #40, .T.);";
        src.replace("#19 = EDGE_CURVE('', #8, #9, #16, .T.);", seam_entities)
    }

    /// Every fin of an exact import carries trim geometry, and it agrees
    /// with the edge it belongs to at every parameter.
    fn assert_pcurve_invariant(store: &TopologyStore, geo: &GeometryStore, body: EntityId<Body>) {
        use opensolid_brep::Curve2Eval;

        for face in store.faces_of_body(body) {
            let surface = geo
                .surface(store.face(face).unwrap().surface.unwrap())
                .unwrap();
            for loop_id in store.loops_of_face(face) {
                for &fin_id in store.fins_of_loop(loop_id) {
                    let fin = store.fin(fin_id).unwrap();
                    let pcurve = geo
                        .pcurve(
                            fin.pcurve
                                .unwrap_or_else(|| panic!("{fin_id:?} has no pcurve")),
                        )
                        .unwrap();
                    let edge = store.edge(fin.edge).unwrap();
                    let curve = geo.curve(edge.curve.unwrap()).unwrap();
                    for k in 0..=8 {
                        let t = edge.t_start + (edge.t_end - edge.t_start) * f64::from(k) / 8.0;
                        let uv = pcurve.point(t);
                        assert!(
                            (surface.point(uv.x, uv.y) - curve.point(t)).norm() < 1e-6,
                            "{fin_id:?} at t = {t}: pcurve leaves its edge"
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn every_fin_of_an_exact_import_gets_trim_geometry() {
        let (store, geo, report) = import(&cylinder_step(1.5, 5.0));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert_pcurve_invariant(&store, &geo, body);
    }

    /// The wall's seam edge is used twice by the one face, and its two fins
    /// must land a full period apart in `u` — otherwise the wall's boundary
    /// does not close in parameter space and encloses nothing.
    #[test]
    fn a_seam_edge_gives_its_two_fins_opposite_branches() {
        use opensolid_brep::Curve2Eval;

        let (store, geo, report) = import(&cylinder_step(1.5, 5.0));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);

        let wall = store
            .faces_of_body(body)
            .into_iter()
            .find(|&f| {
                matches!(
                    geo.surface(store.face(f).unwrap().surface.unwrap()),
                    Some(Surface3::Cylinder { .. })
                )
            })
            .expect("the fixture has a cylindrical wall");

        let fins = store.fins_of_loop(store.loops_of_face(wall)[0]);
        let mut by_edge: HashMap<EntityId<Edge>, Vec<f64>> = HashMap::new();
        for &fin_id in fins {
            let fin = store.fin(fin_id).unwrap();
            let edge = store.edge(fin.edge).unwrap();
            let mid = (edge.t_start + edge.t_end) / 2.0;
            let u = geo.pcurve(fin.pcurve.expect("fin has a pcurve")).unwrap();
            by_edge.entry(fin.edge).or_default().push(u.point(mid).x);
        }
        let seam = by_edge
            .values()
            .find(|us| us.len() == 2)
            .expect("the wall uses its seam edge twice");
        assert!(
            ((seam[1] - seam[0]).abs() - TAU).abs() < 1e-9,
            "seam fins must be one period apart in u, got {seam:?}"
        );
    }

    /// A `SEAM_CURVE` edge used to sink the whole solid to the mesh
    /// fallback: `resolve_curve` had no arm for it, and every cylinder,
    /// sphere and torus a production CAD system writes uses one.
    #[test]
    fn a_seam_curve_edge_imports_exactly() {
        let (store, geo, report) = import(&cylinder_step_with_seam_curve(1.5, 5.0));
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty());
        assert_edges_interpolate(&store, &geo, body);
        assert_pcurve_invariant(&store, &geo, body);

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        let exact = PI * 1.5 * 1.5 * 5.0;
        assert!((signed_volume(&mesh) - exact).abs() / exact < 0.02);
    }

    /// A `SURFACE_CURVE` whose associated geometry names bare surfaces
    /// rather than pcurves — the other half of Part 42's
    /// `pcurve_or_surface` select — is equally transparent.
    #[test]
    fn a_surface_curve_naming_bare_surfaces_imports_exactly() {
        let src = cylinder_step(1.5, 5.0).replace(
            "#19 = EDGE_CURVE('', #8, #9, #16, .T.);",
            "#40 = SURFACE_CURVE('', #16, (#22), .CURVE_3D.);\n\
             #19 = EDGE_CURVE('', #8, #9, #40, .T.);",
        );
        let (store, _, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty());
    }

    /// A `PCURVE` whose definitional representation is empty is malformed;
    /// the solid degrades rather than importing a lie.
    #[test]
    fn an_empty_definitional_representation_is_rejected() {
        let src = cylinder_step_with_seam_curve(1.5, 5.0).replace(
            "DEFINITIONAL_REPRESENTATION('', (#43), #47)",
            "DEFINITIONAL_REPRESENTATION('', (), #47)",
        );
        let (_, _, report) = import(&src);
        assert!(
            !matches!(report.solids[0].outcome, SolidOutcome::BRep(_)),
            "a malformed PCURVE must not import as an exact B-Rep"
        );
    }

    #[test]
    fn pcurves_can_be_turned_off() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let options = StepReadOptions {
            pcurves: false,
            ..Default::default()
        };
        let report = read_step(&cylinder_step(1.5, 5.0), &mut store, &mut geo, &options)
            .expect("fixture parses");
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty());
        for face in store.faces_of_body(body) {
            for loop_id in store.loops_of_face(face) {
                for &fin_id in store.fins_of_loop(loop_id) {
                    assert!(store.fin(fin_id).unwrap().pcurve.is_none());
                }
            }
        }
        assert_eq!(geo.pcurves.len(), 0);
    }

    /// The cylinder fixture with its wall spelled as the extrusion of its
    /// bottom rim circle — the reduction must recover the exact cylinder.
    #[test]
    fn cylinder_via_extruded_circle_imports_exact() {
        let src = cylinder_step(1.5, 5.0).replace(
            "#22 = CYLINDRICAL_SURFACE('', #10, 1.500000);",
            "#22 = SURFACE_OF_LINEAR_EXTRUSION('', #13, #15);",
        );
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty());
        assert!(store.faces_of_body(body).iter().any(|&f| matches!(
            geo.surface(store.face(f).unwrap().surface.unwrap()).unwrap(),
            Surface3::Cylinder { radius, .. } if (radius - 1.5).abs() < 1e-12
        )));
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        let exact = PI * 1.5 * 1.5 * 5.0;
        assert!((signed_volume(&mesh) - exact).abs() / exact < 0.02);
    }

    /// The sphere fixture with its surface spelled as the revolution of its
    /// seam meridian about the polar axis.
    #[test]
    fn sphere_via_surface_of_revolution_imports_exact() {
        let src = wrap(&sphere_step_at(1, 2.0).replace(
            "#12 = SPHERICAL_SURFACE('', #9, 2.000000);",
            "#90 = AXIS1_PLACEMENT('', #1, #4);\n#12 = SURFACE_OF_REVOLUTION('', #11, #90);",
        ));
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty());
        let face = store.faces_of_body(body)[0];
        assert!(matches!(
            geo.surface(store.face(face).unwrap().surface.unwrap())
                .unwrap(),
            Surface3::Sphere { radius, .. } if (radius - 2.0).abs() < 1e-12
        ));
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        let exact = 4.0 / 3.0 * PI * 8.0;
        assert!((signed_volume(&mesh) - exact).abs() / exact < 0.05);
    }

    /// The torus fixture with its surface spelled as the revolution of its
    /// tube circle about the main axis.
    #[test]
    fn torus_via_surface_of_revolution_imports_exact() {
        let src = torus_step(3.0, 1.0).replace(
            "#12 = TOROIDAL_SURFACE('', #8, 3.000000, 1.000000);",
            "#90 = AXIS1_PLACEMENT('', #1, #4);\n#12 = SURFACE_OF_REVOLUTION('', #11, #90);",
        );
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty());
        assert_eq!(store.euler_counts(body).genus, 1);
        let face = store.faces_of_body(body)[0];
        assert!(matches!(
            geo.surface(store.face(face).unwrap().surface.unwrap())
                .unwrap(),
            Surface3::Torus { major_radius, minor_radius, .. }
                if (major_radius - 3.0).abs() < 1e-12 && (minor_radius - 1.0).abs() < 1e-12
        ));
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        let exact = 2.0 * PI * PI * 3.0;
        assert!((signed_volume(&mesh) - exact).abs() / exact < 0.05);
    }

    /// A seam line wrapped in TRIMMED_CURVE must import exactly — edges
    /// re-trim by their vertices, so the wrapper is transparent.
    #[test]
    fn trimmed_curve_wrapping_is_transparent_to_exact_import() {
        let src = cylinder_step(1.5, 5.0).replace(
            "#19 = EDGE_CURVE('', #8, #9, #16, .T.);",
            "#91 = TRIMMED_CURVE('', #16, (#3), (#4), .T., .CARTESIAN.);\n\
             #19 = EDGE_CURVE('', #8, #9, #91, .T.);",
        );
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty());
        assert_edges_interpolate(&store, &geo, body);
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        let exact = PI * 1.5 * 1.5 * 5.0;
        assert!((signed_volume(&mesh) - exact).abs() / exact < 0.02);
    }

    /// A 4×4×4 block with a 2×2×2 cavity: outer shell as authored plus an
    /// `ORIENTED_CLOSED_SHELL(.F.)` void (authored outward, reversed by the
    /// flag into the cavity).
    fn voids_block_step(outer: f64, inner: f64) -> String {
        let mut b = String::new();
        let outer_shell = block_shell_at(&mut b, 0, outer, outer, outer);
        let inner_shell = block_shell_at(&mut b, 200, inner, inner, inner);
        writeln!(
            b,
            "#400 = ORIENTED_CLOSED_SHELL('', *, #{inner_shell}, .F.);"
        )
        .unwrap();
        writeln!(
            b,
            "#401 = BREP_WITH_VOIDS('holey', #{outer_shell}, (#400));"
        )
        .unwrap();
        wrap(&b)
    }

    #[test]
    fn brep_with_voids_imports_inner_shell_exactly() {
        let (store, geo, report) = import(&voids_block_step(4.0, 2.0));
        no_error_diagnostics(&report);
        assert_eq!(report.solids[0].name, "holey");
        let body = brep_body(&report.solids[0].outcome);
        assert!(store.check(body).is_empty(), "{:?}", store.check(body));

        let shells = store.shells_of_body(body);
        assert_eq!(shells.len(), 2, "outer plus one void shell");
        let orientations: Vec<ShellOrientation> = shells
            .iter()
            .map(|&s| store.shells.get(s).unwrap().orientation)
            .collect();
        assert_eq!(
            orientations,
            vec![ShellOrientation::Outward, ShellOrientation::Inward]
        );
        let counts = store.euler_counts(body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (16, 24, 12));
        assert_eq!(counts.genus, 0);

        // The reversed void faces tessellate wound into the cavity, so the
        // enclosed volume is the material between the two boxes.
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        assert!((signed_volume(&mesh) - (64.0 - 8.0)).abs() < 1e-9);
    }

    /// Voids through the mesh fallback: a composite-curve edge on the outer
    /// shell forces tessellated import, and the void shell's triangles are
    /// rewound into the cavity so the mesh volume subtracts it.
    #[test]
    fn brep_with_voids_survives_mesh_fallback() {
        let src = voids_block_step(4.0, 2.0).replace(
            "#20 = EDGE_CURVE('', #9, #10, #19, .T.);",
            "#402 = TRIMMED_CURVE('', #19, (#1), (#2), .T., .CARTESIAN.);\n\
             #403 = COMPOSITE_CURVE_SEGMENT(.CONTINUOUS., .T., #402);\n\
             #404 = COMPOSITE_CURVE('', (#403), .F.);\n\
             #20 = EDGE_CURVE('', #9, #10, #404, .T.);",
        );
        let (_store, _geo, report) = import(&src);
        no_error_diagnostics(&report);
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.message.contains("COMPOSITE_CURVE")),
            "composite edge reported: {:?}",
            report.diagnostics
        );
        match &report.solids[0].outcome {
            SolidOutcome::Mesh { mesh, .. } => {
                assert!(mesh.is_closed_manifold());
                assert!(
                    (signed_volume(mesh) - (64.0 - 8.0)).abs() < 1e-9,
                    "cavity subtracted: {}",
                    signed_volume(mesh)
                );
            }
            other => panic!("expected the mesh fallback, got {other:?}"),
        }
    }

    // ---- healing: unsewn and misoriented files (of-3qy.12) ----

    /// AP203 block whose faces never share a boundary: each of the six faces
    /// authors its own four `CARTESIAN_POINT`s, `VERTEX_POINT`s and
    /// `EDGE_CURVE`s, as an exporter that skipped the sewing step writes.
    ///
    /// `jitter` displaces each face's private copy of a shared corner along a
    /// per-face direction, modelling the last-decimal disagreement real files
    /// carry. Faces listed in `reversed` have their whole use authored
    /// backwards — `ADVANCED_FACE.same_sense` *and* the loop's traversal.
    fn unsewn_block_step(x: f64, y: f64, z: f64, jitter: f64, reversed: &[usize]) -> String {
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
        // Same cycles and outward normals as `block_shell_at`.
        let face_specs: [([usize; 4], (f64, f64, f64)); 6] = [
            ([0, 3, 2, 1], (0.0, 0.0, -1.0)),
            ([4, 5, 6, 7], (0.0, 0.0, 1.0)),
            ([0, 1, 5, 4], (0.0, -1.0, 0.0)),
            ([1, 2, 6, 5], (1.0, 0.0, 0.0)),
            ([2, 3, 7, 6], (0.0, 1.0, 0.0)),
            ([3, 0, 4, 7], (-1.0, 0.0, 0.0)),
        ];

        let mut b = String::new();
        let mut next = 1u64;
        let mut id = || {
            next += 1;
            next - 1
        };
        let mut face_ids = Vec::new();
        for (f, &(cycle, (nx, ny, nz))) in face_specs.iter().enumerate() {
            // A per-face nudge that is not parallel to any other face's, so
            // coincident corners scatter instead of stacking.
            let k = (f + 1) as f64;
            let raw = ((k * 0.7).sin(), (k * 1.3).sin(), (k * 2.1).sin());
            let len = (raw.0 * raw.0 + raw.1 * raw.1 + raw.2 * raw.2).sqrt();
            let nudge = (
                raw.0 / len * jitter,
                raw.1 / len * jitter,
                raw.2 / len * jitter,
            );

            let points: Vec<(f64, f64, f64)> = cycle
                .iter()
                .map(|&c| {
                    (
                        corners[c].0 + nudge.0,
                        corners[c].1 + nudge.1,
                        corners[c].2 + nudge.2,
                    )
                })
                .collect();
            let point_ids: Vec<u64> = points
                .iter()
                .map(|&(px, py, pz)| {
                    let pid = id();
                    writeln!(
                        b,
                        "#{pid} = CARTESIAN_POINT('', ({px:.9}, {py:.9}, {pz:.9}));"
                    )
                    .unwrap();
                    pid
                })
                .collect();
            let vertex_ids: Vec<u64> = point_ids
                .iter()
                .map(|&pid| {
                    let vid = id();
                    writeln!(b, "#{vid} = VERTEX_POINT('', #{pid});").unwrap();
                    vid
                })
                .collect();

            let mut edge_ids = Vec::new();
            for k in 0..4 {
                let (a, c) = (k, (k + 1) % 4);
                let (dx, dy, dz) = (
                    points[c].0 - points[a].0,
                    points[c].1 - points[a].1,
                    points[c].2 - points[a].2,
                );
                let dir = id();
                writeln!(b, "#{dir} = DIRECTION('', ({dx:.9}, {dy:.9}, {dz:.9}));").unwrap();
                let vec = id();
                writeln!(b, "#{vec} = VECTOR('', #{dir}, 1.);").unwrap();
                let line = id();
                writeln!(b, "#{line} = LINE('', #{}, #{vec});", point_ids[a]).unwrap();
                let edge = id();
                writeln!(
                    b,
                    "#{edge} = EDGE_CURVE('', #{}, #{}, #{line}, .T.);",
                    vertex_ids[a], vertex_ids[c]
                )
                .unwrap();
                edge_ids.push(edge);
            }

            let normal = id();
            writeln!(b, "#{normal} = DIRECTION('', ({nx:.6}, {ny:.6}, {nz:.6}));").unwrap();
            let placement = id();
            writeln!(
                b,
                "#{placement} = AXIS2_PLACEMENT_3D('', #{}, #{normal}, $);",
                point_ids[0]
            )
            .unwrap();
            let plane = id();
            writeln!(b, "#{plane} = PLANE('', #{placement});").unwrap();

            let backwards = reversed.contains(&f);
            let mut oriented: Vec<u64> = edge_ids
                .iter()
                .map(|&edge| {
                    let oe = id();
                    let flag = if backwards { ".F." } else { ".T." };
                    writeln!(b, "#{oe} = ORIENTED_EDGE('', *, *, #{edge}, {flag});").unwrap();
                    oe
                })
                .collect();
            if backwards {
                oriented.reverse();
            }
            let edge_loop = id();
            writeln!(
                b,
                "#{edge_loop} = EDGE_LOOP('', (#{}, #{}, #{}, #{}));",
                oriented[0], oriented[1], oriented[2], oriented[3]
            )
            .unwrap();
            let bound = id();
            writeln!(b, "#{bound} = FACE_OUTER_BOUND('', #{edge_loop}, .T.);").unwrap();
            let face = id();
            let same_sense = if backwards { ".F." } else { ".T." };
            writeln!(
                b,
                "#{face} = ADVANCED_FACE('', (#{bound}), #{plane}, {same_sense});"
            )
            .unwrap();
            face_ids.push(face);
        }

        let shell = id();
        let refs: Vec<String> = face_ids.iter().map(|f| format!("#{f}")).collect();
        writeln!(b, "#{shell} = CLOSED_SHELL('', ({}));", refs.join(", ")).unwrap();
        let solid = id();
        writeln!(b, "#{solid} = MANIFOLD_SOLID_BREP('block', #{shell});").unwrap();
        wrap(&b)
    }

    fn import_with(
        src: &str,
        options: &StepReadOptions,
    ) -> (TopologyStore, GeometryStore, StepImport) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let result = read_step(src, &mut store, &mut geo, options).expect("fixture parses");
        (store, geo, result)
    }

    /// The defect that motivates the healer: an unsewn shell maps cleanly but
    /// every edge ends up with one fin, so `check` rejects the body.
    #[test]
    fn unsewn_block_needs_healing_to_import_exactly() {
        let src = unsewn_block_step(2.0, 3.0, 4.0, 0.0, &[]);
        let (_store, _geo, report) = import_with(
            &src,
            &StepReadOptions {
                heal: HealOptions {
                    strategy: HealStrategy::Off,
                    ..HealOptions::default()
                },
                ..StepReadOptions::default()
            },
        );
        assert!(
            matches!(report.solids[0].outcome, SolidOutcome::Mesh { .. }),
            "unhealed, the exact path must fail: {:?}",
            report.solids[0].outcome
        );
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.message.contains("must be closed but has open")),
            "the open edges are reported: {:?}",
            report.diagnostics
        );
        assert_eq!(report.heal_operations, 0);
    }

    /// The same file with healing on (the default) imports as an exact
    /// B-Rep: 24 private corners sew to 8, 24 half-edges weld to 12.
    #[test]
    fn healing_promotes_an_unsewn_block_to_exact_brep() {
        let src = unsewn_block_step(2.0, 3.0, 4.0, 0.0, &[]);
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert_eq!(store.check(body), vec![]);

        let counts = store.euler_counts(body);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (8, 12, 6));
        assert_eq!(counts.genus, 0);
        assert!(report.heal_operations > 0);
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.severity == Severity::Info && d.message.starts_with("healed:")),
            "per-fix diagnostics: {:?}",
            report.diagnostics
        );

        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        assert!((signed_volume(&mesh) - 24.0).abs() < 1e-9);
    }

    /// Gaps at the last written decimal are closed and carried as tolerance,
    /// not snapped away silently — and the tolerant body that results still
    /// tessellates watertight, to the exact volume. The mesh gate here used to
    /// be loosened to 1e-4 with no manifold check at all, because the healed
    /// vertices' adjacent curves still ran to their pre-merge endpoints and
    /// the rim would not weld (of-61f, fixed in `tessellate::sample_loop`).
    #[test]
    fn healing_closes_written_precision_gaps() {
        let src = unsewn_block_step(2.0, 3.0, 4.0, 1e-6, &[]);
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert_eq!(store.check(body), vec![]);
        assert!(
            store
                .faces_of_body(body)
                .iter()
                .flat_map(|&f| store.edges_of_face(f))
                .any(|e| store.edge(e).unwrap().is_tolerant()),
            "the closed gap must live on as edge tolerance"
        );
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(
            mesh.is_closed_manifold(),
            "a tolerant healed body must weld watertight (of-61f)"
        );
        // What remains is the healing displacement itself, not a meshing
        // artifact: each corner now sits at its cluster centroid, up to the
        // 1e-6 closed gap from where it was authored, which moves the volume
        // of a 2x3x4 block (52 mm^2 of surface) by O(area * gap). Measured
        // 3.5e-6 — so this gate is ~3x the real error, where the old 1e-4 was
        // 30x it and hid a fully open mesh.
        let volume = signed_volume(&mesh);
        assert!(
            (volume - 24.0).abs() < 1e-5,
            "healed volume off by more than the closed gap can explain: {volume}"
        );
    }

    /// Unsewn *and* misoriented: gap closure makes the shell shareable, then
    /// orientation repair rights the two faces authored backwards.
    #[test]
    fn healing_repairs_orientation_after_sewing() {
        let src = unsewn_block_step(2.0, 3.0, 4.0, 0.0, &[2, 5]);
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert_eq!(store.check(body), vec![]);
        assert!(
            report
                .diagnostics
                .iter()
                .any(|d| d.message.contains("to match its neighbours")),
            "the reoriented faces are reported: {:?}",
            report.diagnostics
        );
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!(mesh.is_closed_manifold());
        assert!(
            (signed_volume(&mesh) - 24.0).abs() < 1e-9,
            "outward, not inside out: {}",
            signed_volume(&mesh)
        );
    }

    /// An inside-out file — every face authored backwards — is consistently
    /// oriented, so `check` alone cannot see it. The healer's volume-sign
    /// pass must.
    #[test]
    fn healing_uprights_a_wholly_inverted_block() {
        let src = unsewn_block_step(2.0, 3.0, 4.0, 0.0, &[0, 1, 2, 3, 4, 5]);
        let (store, geo, report) = import(&src);
        no_error_diagnostics(&report);
        let body = brep_body(&report.solids[0].outcome);
        assert_eq!(store.check(body), vec![]);
        let mesh = tessellate_body(&store, &geo, body, &TessellationOptions::default()).unwrap();
        assert!((signed_volume(&mesh) - 24.0).abs() < 1e-9);
    }

    /// `ReportOnly` says what it would fix and changes nothing: the import
    /// still degrades to the mesh fallback.
    #[test]
    fn report_only_healing_reports_but_does_not_promote() {
        let src = unsewn_block_step(2.0, 3.0, 4.0, 0.0, &[]);
        let (_store, _geo, report) = import_with(
            &src,
            &StepReadOptions {
                heal: HealOptions {
                    strategy: HealStrategy::ReportOnly,
                    ..HealOptions::default()
                },
                ..StepReadOptions::default()
            },
        );
        assert!(matches!(
            report.solids[0].outcome,
            SolidOutcome::Mesh { .. }
        ));
        assert_eq!(
            report.heal_operations, 20,
            "8 vertex merges + 12 edge welds"
        );
    }

    /// Files that were already valid must not acquire heal operations —
    /// healing runs only for bodies `check` rejected.
    #[test]
    fn a_clean_file_is_never_healed() {
        let (_store, _geo, report) = import(&block_step(2.0, 3.0, 4.0));
        assert_eq!(report.heal_operations, 0);
        let (_store, _geo, report) = import(&cylinder_step(1.5, 4.0));
        assert_eq!(report.heal_operations, 0);
    }
}
