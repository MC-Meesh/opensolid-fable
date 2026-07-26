//! STEP import for playground shapes (of-2y4.7).
//!
//! The kernel reader ([`opensolid_kernel::io::step::read`]) is the whole
//! engine here: it parses Part 21, maps every `MANIFOLD_SOLID_BREP` to an
//! exact B-Rep where it can, falls back to a closed tessellation where it
//! cannot, heals what it can repair, and reports a per-entity
//! [`Diagnostic`] for everything it decided along the way. None of that was
//! reachable from JS — this module is the adapter that turns a
//! [`StepImport`](opensolid_kernel::io::step::read::StepImport) into
//! playground shapes plus a machine-readable report.
//!
//! Each imported solid becomes a [`BoundedShape`] backed by a
//! [`MeshSdf`] — a true metric distance field, so an imported part composes
//! with scripted geometry through the ordinary F-Rep operations. Solids that
//! came back as exact B-Reps *also* keep the body itself
//! ([`ExactRep::Imported`]), which is what lets an import round-trip back out
//! through the STEP writer with its analytic surfaces intact rather than as
//! recovered facets.
//!
//! Plain Rust (no wasm-bindgen types) so both paths are exercised by native
//! `cargo test`; `lib.rs` wraps it for JS.
//!
//! # Assemblies
//!
//! The reader keeps imported geometry in part-local coordinates and reports
//! placement separately, one [`PlacedSolid`] per occurrence. [`assembled`]
//! is the placed view — every occurrence transformed and unioned — while the
//! per-solid shapes stay local, so a caller can hand back either. A flat
//! single-part file has exactly one identity occurrence, and then the two
//! coincide.

use std::rc::Rc;

use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Transform3};
use opensolid_frep::Shape;
use opensolid_kernel::brep::tessellate::tessellate_body;
use opensolid_kernel::brep::{GeometryStore, TessellationOptions, TopologyStore};
use opensolid_kernel::convert::MeshSdf;
use opensolid_kernel::io::step::product::PlacedSolid;
use opensolid_kernel::io::step::read::{
    Diagnostic, Severity, SolidOutcome, StepReadOptions, read_step_bytes,
};

use crate::bounded::BoundedShape;
use crate::exact::{ExactRep, ImportedBody, ImportedFile};

/// Default angular sampling of the imported-body tessellation: 32 segments
/// around a full circle, matching [`TessellationOptions::default`].
const DEFAULT_CIRCLE_SEGMENTS: f64 = 32.0;

/// Clamp on caller-supplied circle segments. The floor is the mesher's own
/// (three segments is the coarsest closed polygon); the ceiling keeps a
/// pathological request from meshing a cylinder into millions of triangles
/// inside a single tool call.
const MIN_CIRCLE_SEGMENTS: f64 = 3.0;
const MAX_CIRCLE_SEGMENTS: f64 = 512.0;

/// A playground shape recovered from a STEP solid: the SDF the rest of the
/// kernel composes with, plus the exact companion when the reader produced
/// a real B-Rep.
pub struct ImportedShape {
    pub inner: BoundedShape,
    pub exact: Option<ExactRep>,
}

impl ImportedShape {
    /// Whether this shape kept the file's *analytic surfaces* — and so
    /// re-exports as analytic STEP rather than as recovered facets.
    ///
    /// Deliberately narrower than "has an exact companion": a mesh-fallback
    /// solid and a placed assembly both carry an authoritative
    /// tessellation, which is a claim about triangles, not about surfaces.
    pub fn is_analytic(&self) -> bool {
        matches!(self.exact, Some(ExactRep::Imported(_)))
    }
}

/// Which of the reader's two paths a solid came back on.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ImportKind {
    /// Exact B-Rep: analytic surfaces, re-exportable as analytic STEP.
    Brep,
    /// Mesh fallback: a closed tessellation wrapped as an SDF. Valid
    /// geometry, but the analytic surfaces are gone.
    Mesh,
    /// Neither path succeeded; the diagnostics say why.
    Failed,
}

impl ImportKind {
    pub fn as_str(self) -> &'static str {
        match self {
            ImportKind::Brep => "brep",
            ImportKind::Mesh => "mesh",
            ImportKind::Failed => "failed",
        }
    }
}

/// One `MANIFOLD_SOLID_BREP` from the file, as a shape.
pub struct ImportedSolid {
    /// STEP instance name (`#id`) of the solid.
    pub step_id: u64,
    /// The entity's name attribute (often empty).
    pub name: String,
    pub kind: ImportKind,
    /// The shape, absent for a failed solid — or for an imported body that
    /// could not be turned into a distance field, in which case
    /// [`shape_error`](Self::shape_error) says why.
    pub shape: Option<ImportedShape>,
    /// Why a solid the reader imported still has no usable shape. Kept
    /// separate from the reader's diagnostics because the reader did not
    /// fail: the loss happened on this side of the boundary, and saying so
    /// is the difference between "your file is bad" and "we could not field
    /// it".
    pub shape_error: Option<String>,
    /// Triangles in the tessellation backing the shape (0 when there is none).
    pub triangles: usize,
}

/// Everything one STEP file yielded.
pub struct StepImport {
    pub solids: Vec<ImportedSolid>,
    /// One entry per placed occurrence, in the reader's order.
    pub instances: Vec<PlacedSolid>,
    pub diagnostics: Vec<Diagnostic>,
    /// Millimetres per file length unit; geometry is already scaled by it.
    pub length_scale: f64,
    /// Radians per file plane-angle unit; angles are already scaled by it.
    pub angle_scale: f64,
    /// Repairs the healer applied across every solid.
    pub heal_operations: usize,
    /// Whether the file carries real assembly structure (see
    /// [`StepImport::is_assembly`](opensolid_kernel::io::step::read::StepImport::is_assembly)).
    pub is_assembly: bool,
}

/// Tessellation fidelity for imported bodies, as segments around a full
/// circle (the [`TessellationOptions`] knob, in the units an agent thinks
/// in). Absent, non-finite, or out-of-range values fall back to the
/// kernel's 32.
pub fn tessellation_options(circle_segments: Option<f64>) -> TessellationOptions {
    let segments = match circle_segments {
        Some(n) if n.is_finite() => n.clamp(MIN_CIRCLE_SEGMENTS, MAX_CIRCLE_SEGMENTS),
        _ => DEFAULT_CIRCLE_SEGMENTS,
    };
    TessellationOptions {
        angular_step: std::f64::consts::TAU / segments,
    }
}

/// Read STEP Part 21 bytes and turn every solid into a shape.
///
/// STEP files are ASCII/Latin-1 and may be handed over as raw bytes, so this
/// takes bytes rather than a `str` — a file with a Latin-1 degree sign in a
/// product name is perfectly valid STEP and is not valid UTF-8.
///
/// # Errors
/// A human-readable message when the file is not syntactically valid Part 21.
/// Semantic problems never fail the call: they arrive as per-solid outcomes
/// and diagnostics, which is the entire point of the report.
pub fn import_step(bytes: &[u8], circle_segments: Option<f64>) -> Result<StepImport, String> {
    let tessellation = tessellation_options(circle_segments);
    let options = StepReadOptions {
        tessellation: tessellation.clone(),
        ..StepReadOptions::default()
    };
    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let report = read_step_bytes(bytes, &mut store, &mut geo, &options)
        .map_err(|e| format!("STEP import failed: {e}"))?;

    let is_assembly = report.is_assembly();
    let file = Rc::new(ImportedFile { store, geo });
    // Consume the reader's solids rather than borrowing them: the mesh
    // fallback hands over a `Box<MeshSdf>` that already owns a built BVH,
    // and rebuilding that from the mesh would be the most expensive line in
    // the import.
    let solids = report
        .solids
        .into_iter()
        .map(|solid| {
            let (kind, shape, shape_error, triangles) = match solid.outcome {
                SolidOutcome::BRep(body) => match brep_shape(&file, body, &tessellation) {
                    Ok((shape, triangles)) => (ImportKind::Brep, Some(shape), None, triangles),
                    Err(e) => (ImportKind::Brep, None, Some(e), 0),
                },
                SolidOutcome::Mesh { mesh, sdf } => {
                    let triangles = mesh.triangle_count();
                    let shape = ImportedShape {
                        inner: BoundedShape {
                            bounds: mesh_bounds(&mesh),
                            shape: Shape::new(*sdf),
                        },
                        // No analytic body — that is what "fallback" means —
                        // but the reader's tessellation *is* this solid, so
                        // measurement reads it rather than re-meshing the
                        // field built from it.
                        exact: Some(ExactRep::Tessellated(Rc::new(mesh))),
                    };
                    (ImportKind::Mesh, Some(shape), None, triangles)
                }
                SolidOutcome::Failed => (ImportKind::Failed, None, None, 0),
            };
            ImportedSolid {
                step_id: solid.step_id,
                name: solid.name,
                kind,
                shape,
                shape_error,
                triangles,
            }
        })
        .collect();

    Ok(StepImport {
        solids,
        instances: report.instances,
        diagnostics: report.diagnostics,
        length_scale: report.length_scale,
        angle_scale: report.angle_scale,
        heal_operations: report.heal_operations,
        is_assembly,
    })
}

/// Tessellate an imported body, wrap it as a field, and keep the body for
/// analytic re-export.
fn brep_shape(
    file: &Rc<ImportedFile>,
    body: opensolid_core::EntityId<opensolid_kernel::brep::topology::Body>,
    tessellation: &TessellationOptions,
) -> Result<(ImportedShape, usize), String> {
    let mesh = tessellate_body(&file.store, &file.geo, body, tessellation)
        .map_err(|e| format!("imported body could not be tessellated: {e}"))?;
    let sdf = MeshSdf::new(&mesh).map_err(|e| {
        format!("imported body tessellated but is not a usable distance field: {e}")
    })?;
    let triangles = mesh.triangle_count();
    let bounds = mesh_bounds(&mesh);
    Ok((
        ImportedShape {
            inner: BoundedShape {
                shape: Shape::new(sdf),
                bounds,
            },
            exact: Some(ExactRep::Imported(Rc::new(ImportedBody {
                file: Rc::clone(file),
                body,
                mesh,
            }))),
        },
        triangles,
    ))
}

/// A mesh moved into an assembly occurrence's place. Positions transform by
/// the isometry; normals by its rotation alone, since a rigid transform
/// neither scales nor shears them.
fn placed_mesh(mesh: &TriangleMesh, placement: &Transform3) -> TriangleMesh {
    TriangleMesh {
        positions: mesh.positions.iter().map(|p| placement * p).collect(),
        normals: mesh
            .normals
            .iter()
            .map(|n| placement.rotation * n)
            .collect(),
        indices: mesh.indices.clone(),
    }
}

/// Concatenate meshes into one, shifting each one's indices past the
/// vertices already emitted. Disjoint closed manifolds concatenate into a
/// closed manifold, which is what keeps mass properties integrable over the
/// result.
fn concat_meshes(meshes: &[TriangleMesh]) -> TriangleMesh {
    let mut out = TriangleMesh {
        positions: Vec::with_capacity(meshes.iter().map(|m| m.positions.len()).sum()),
        normals: Vec::with_capacity(meshes.iter().map(|m| m.normals.len()).sum()),
        indices: Vec::with_capacity(meshes.iter().map(|m| m.indices.len()).sum()),
    };
    for mesh in meshes {
        let offset = out.positions.len();
        out.positions.extend_from_slice(&mesh.positions);
        out.normals.extend_from_slice(&mesh.normals);
        out.indices
            .extend(mesh.indices.iter().map(|t| t.map(|i| i + offset)));
    }
    out
}

/// A mesh's extent, or a degenerate box at the origin for an empty mesh
/// (the meshing bounds have to be *some* finite box).
fn mesh_bounds(mesh: &TriangleMesh) -> BoundingBox3 {
    mesh.bounding_box()
        .unwrap_or_else(|| BoundingBox3::new(Point3::origin(), Point3::origin()))
}

impl StepImport {
    /// How many solids came back on each path.
    pub fn counts(&self) -> (usize, usize, usize) {
        let count = |kind| self.solids.iter().filter(|s| s.kind == kind).count();
        (
            count(ImportKind::Brep),
            count(ImportKind::Mesh),
            count(ImportKind::Failed),
        )
    }

    /// Whether any diagnostic is an error.
    pub fn has_errors(&self) -> bool {
        self.diagnostics
            .iter()
            .any(|d| d.severity == Severity::Error)
    }

    /// The file as a single shape: every usable solid placed by each of its
    /// occurrences, unioned. `None` when no solid produced a shape.
    ///
    /// A single solid at a single identity placement — the ordinary
    /// single-part file — is returned unchanged, analytic body and all, so
    /// it re-exports as analytic STEP. Anything genuinely composed loses the
    /// analytic representation (see
    /// [`exact_boolean`](crate::exact::exact_boolean)) but keeps a
    /// *tessellation*: the parts' own triangles, placed. That matters for
    /// more than tidiness — measuring the composed field instead would mean
    /// dual-contouring a union of N mesh SDFs, which on a five-part
    /// eighteen-occurrence assembly is a minute of CPU for an answer the
    /// parts already contain.
    ///
    /// The placed mesh is the union of the occurrences as *material*: a
    /// volume read off it sums the parts, so parts that interpenetrate are
    /// counted twice. For an assembly of solid parts — which is what a STEP
    /// assembly is — that is the total material, and the honest reading.
    pub fn assembled(&self) -> Option<ImportedShape> {
        let placed: Vec<(&ImportedShape, Option<&PlacedSolid>)> = if self.instances.is_empty() {
            // No product structure at all (or none that resolved): fall back
            // to the solids themselves, unplaced. The reader normally
            // guarantees one identity occurrence per solid, so this only
            // bites on files where even that failed — and losing the parts
            // entirely would be worse than showing them at the origin.
            self.solids
                .iter()
                .filter_map(|s| Some((s.shape.as_ref()?, None)))
                .collect()
        } else {
            self.instances
                .iter()
                .filter_map(|inst| {
                    let solid = self.solids.get(inst.solid)?;
                    Some((solid.shape.as_ref()?, Some(inst)))
                })
                .collect()
        };

        let (first, rest) = placed.split_first()?;
        if rest.is_empty() && first.1.is_none_or(|inst| inst.is_identity()) {
            return Some(ImportedShape {
                inner: first.0.inner.clone(),
                exact: first.0.exact.clone(),
            });
        }

        let place = |(shape, inst): &(&ImportedShape, Option<&PlacedSolid>)| match inst
            .filter(|i| !i.is_identity())
        {
            Some(i) => shape.inner.transform(&i.transform),
            None => shape.inner.clone(),
        };
        let inner = rest
            .iter()
            .fold(place(first), |acc, entry| acc.union(&place(entry)));

        // Every solid that produced a shape also carries its tessellation
        // (exact import or mesh fallback), so this is normally complete; if
        // some future path ever yields a shape without one, drop the mesh
        // rather than report a partial assembly as the whole.
        let meshes: Option<Vec<TriangleMesh>> = placed
            .iter()
            .map(|(shape, inst)| {
                let mesh = shape.exact.as_ref()?.exact_mesh()?;
                Some(match inst.filter(|i| !i.is_identity()) {
                    Some(i) => placed_mesh(mesh, &i.transform),
                    None => mesh.clone(),
                })
            })
            .collect();

        Some(ImportedShape {
            inner,
            exact: meshes.map(|meshes| ExactRep::Tessellated(Rc::new(concat_meshes(&meshes)))),
        })
    }

    /// The whole import as a JSON object string, for the JS layer to hand
    /// an agent: per-solid outcomes, every diagnostic, the healing count,
    /// the resolved units, and the assembly structure.
    ///
    /// `assembled_exact` is [`ImportedShape::is_analytic`] of whatever
    /// [`assembled`](Self::assembled) produced. It is a parameter rather
    /// than something recomputed here so the report cannot claim analytic
    /// surfaces for a shape the caller was not handed — and so building the
    /// assembly (which concatenates its parts' meshes) happens once.
    pub fn report_json(&self, assembled_exact: bool) -> String {
        let (brep, mesh, failed) = self.counts();
        let solids = self
            .solids
            .iter()
            .enumerate()
            .map(|(index, s)| {
                format!(
                    "{{\"index\":{},\"stepId\":{},\"name\":\"{}\",\"outcome\":\"{}\",\
                     \"triangles\":{},\"exact\":{},\"shapeError\":{}}}",
                    index,
                    s.step_id,
                    escape(&s.name),
                    s.kind.as_str(),
                    s.triangles,
                    // "exact" is the analytic-surfaces claim, not "we have a
                    // mesh": a fallback solid also carries its tessellation.
                    s.kind == ImportKind::Brep && s.shape.is_some(),
                    match &s.shape_error {
                        Some(e) => format!("\"{}\"", escape(e)),
                        None => "null".to_string(),
                    },
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        let diagnostics = self
            .diagnostics
            .iter()
            .map(|d| {
                format!(
                    "{{\"entity\":{},\"severity\":\"{}\",\"message\":\"{}\"}}",
                    match d.entity {
                        Some(id) => id.to_string(),
                        None => "null".to_string(),
                    },
                    severity_str(d.severity),
                    escape(&d.message),
                )
            })
            .collect::<Vec<_>>()
            .join(",");
        let count_severity = |want: Severity| {
            self.diagnostics
                .iter()
                .filter(|d| d.severity == want)
                .count()
        };

        let instances = self
            .instances
            .iter()
            .map(|i| {
                let t = i.transform.translation.vector;
                let r = i.transform.rotation.scaled_axis() * (180.0 / std::f64::consts::PI);
                let path = i
                    .path
                    .iter()
                    .map(|p| format!("\"{}\"", escape(p)))
                    .collect::<Vec<_>>()
                    .join(",");
                format!(
                    "{{\"solid\":{},\"stepId\":{},\"product\":\"{}\",\"path\":[{}],\
                     \"identity\":{},\"translation\":[{},{},{}],\"rotationAxisAngleDeg\":[{},{},{}]}}",
                    i.solid,
                    i.step_id,
                    escape(&i.product),
                    path,
                    i.is_identity(),
                    num(t.x),
                    num(t.y),
                    num(t.z),
                    num(r.x),
                    num(r.y),
                    num(r.z),
                )
            })
            .collect::<Vec<_>>()
            .join(",");

        format!(
            "{{\"solids\":[{}],\"counts\":{{\"brep\":{},\"mesh\":{},\"failed\":{}}},\
             \"assembledExact\":{},\"diagnostics\":[{}],\
             \"diagnosticCounts\":{{\"error\":{},\"warning\":{},\"info\":{}}},\
             \"healOperations\":{},\"lengthScale\":{},\"angleScale\":{},\
             \"isAssembly\":{},\"instances\":[{}]}}",
            solids,
            brep,
            mesh,
            failed,
            assembled_exact,
            diagnostics,
            count_severity(Severity::Error),
            count_severity(Severity::Warning),
            count_severity(Severity::Info),
            self.heal_operations,
            num(self.length_scale),
            num(self.angle_scale),
            self.is_assembly,
            instances,
        )
    }
}

fn severity_str(severity: Severity) -> &'static str {
    match severity {
        Severity::Info => "info",
        Severity::Warning => "warning",
        Severity::Error => "error",
    }
}

fn num(x: f64) -> String {
    if x.is_finite() {
        format!("{x}")
    } else {
        "null".to_string()
    }
}

/// JSON string escaping. Diagnostic messages and STEP name attributes are
/// arbitrary file content — a quote, a backslash, or a stray control byte in
/// a product name must not be able to break the report it is quoted into.
fn escape(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    for c in s.chars() {
        match c {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            c if (c as u32) < 0x20 => out.push_str(&format!("\\u{:04x}", c as u32)),
            c => out.push(c),
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::{ExactPrim, ExactRep, ExactSpec};
    use crate::step::export_step;
    use opensolid_kernel::massprops::mass_properties;

    /// The reader's own doc example: a radius-2 sphere as one spherical
    /// face closed by a seam meridian.
    const SPHERE: &str = "\
ISO-10303-21;
HEADER;
FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('', (0., 0., 0.));
#2 = CARTESIAN_POINT('', (0., 0., -2.));
#3 = CARTESIAN_POINT('', (0., 0., 2.));
#4 = DIRECTION('', (0., 0., 1.));
#5 = DIRECTION('', (0., -1., 0.));
#6 = DIRECTION('', (1., 0., 0.));
#7 = VERTEX_POINT('', #2);
#8 = VERTEX_POINT('', #3);
#9 = AXIS2_PLACEMENT_3D('', #1, #4, #6);
#10 = AXIS2_PLACEMENT_3D('', #1, #5, #6);
#11 = CIRCLE('', #10, 2.);
#12 = SPHERICAL_SURFACE('', #9, 2.);
#13 = EDGE_CURVE('', #7, #8, #11, .T.);
#14 = ORIENTED_EDGE('', *, *, #13, .T.);
#15 = ORIENTED_EDGE('', *, *, #13, .F.);
#16 = EDGE_LOOP('', (#14, #15));
#17 = FACE_OUTER_BOUND('', #16, .T.);
#18 = ADVANCED_FACE('', (#17), #12, .T.);
#19 = CLOSED_SHELL('', (#18));
#20 = MANIFOLD_SOLID_BREP('ball', #19);
ENDSEC;
END-ISO-10303-21;
";

    fn read(source: &str) -> StepImport {
        import_step(source.as_bytes(), None).expect("valid Part 21")
    }

    /// The report as `lib.rs` builds it: the analytic flag describes the
    /// shape `assembled()` actually hands out.
    fn report_of(import: &StepImport) -> String {
        import.report_json(import.assembled().is_some_and(|s| s.is_analytic()))
    }

    /// The measured volume of a shape's imported tessellation.
    fn imported_volume(shape: &ImportedShape) -> f64 {
        let mesh = shape
            .exact
            .as_ref()
            .and_then(|rep| rep.exact_mesh())
            .expect("an exact import serves its own mesh");
        mass_properties(mesh).expect("closed solid").volume
    }

    #[test]
    fn sphere_imports_as_an_exact_brep() {
        let import = read(SPHERE);
        assert_eq!(import.counts(), (1, 0, 0), "one exact solid");
        assert!(!import.has_errors(), "{:?}", import.diagnostics);
        let solid = &import.solids[0];
        assert_eq!(solid.step_id, 20);
        assert_eq!(solid.name, "ball");
        assert_eq!(solid.kind, ImportKind::Brep);
        assert!(solid.triangles > 0);
        let shape = solid.shape.as_ref().expect("shape");
        assert!(shape.exact.is_some(), "an exact import keeps its body");

        // 4/3 π r³ at r = 2 is 33.51; the tessellation inscribes the sphere,
        // so the polyhedral volume reads a few percent under.
        let volume = imported_volume(shape);
        let exact = 4.0 / 3.0 * std::f64::consts::PI * 8.0;
        assert!(
            volume < exact && volume > 0.9 * exact,
            "imported sphere volume {volume} (analytic {exact})",
        );
    }

    /// The field is what makes an import composable: the SDF must be
    /// negative inside the part and positive outside it.
    #[test]
    fn the_imported_shape_is_a_usable_distance_field() {
        let import = read(SPHERE);
        let shape = &import.solids[0].shape.as_ref().expect("shape").inner;
        use opensolid_frep::primitives::Sdf;
        assert!(
            shape.shape.eval(&Point3::new(0.0, 0.0, 0.0)) < -1.5,
            "inside"
        );
        assert!(
            shape.shape.eval(&Point3::new(5.0, 0.0, 0.0)) > 2.5,
            "outside"
        );
        // The tracked box is the tessellation's extent: a radius-2 sphere.
        assert!(shape.bounds.max.x > 1.7 && shape.bounds.max.x <= 2.0 + 1e-9);
    }

    /// Placement and concatenation are the whole of assembly assembly: a
    /// part placed twice must weigh twice as much and still bound a solid.
    /// (The alternative — measuring the union field — is a minute of CPU on
    /// a real assembly, so this arithmetic is load-bearing, not cosmetic.)
    #[test]
    fn placed_meshes_concatenate_into_one_closed_solid() {
        let import = read(SPHERE);
        let mesh = import.solids[0]
            .shape
            .as_ref()
            .expect("shape")
            .exact
            .as_ref()
            .expect("exact")
            .exact_mesh()
            .expect("mesh")
            .clone();
        let one = mass_properties(&mesh).expect("closed solid").volume;

        let moved = placed_mesh(&mesh, &Transform3::translation(10.0, 0.0, 0.0));
        assert!(
            moved.positions.iter().all(|p| p.x > 7.0),
            "the placed copy did not move",
        );

        let pair = concat_meshes(&[mesh, moved]);
        assert!(
            pair.is_closed_manifold(),
            "two disjoint solids are still closed"
        );
        let both = mass_properties(&pair).expect("closed solid");
        assert!(
            (both.volume - 2.0 * one).abs() < 1e-9,
            "{} vs {one}",
            both.volume
        );
        // Two equal spheres at x = 0 and x = 10: the centre of mass is between.
        assert!(
            (both.centroid.x - 5.0).abs() < 1e-9,
            "centroid {}",
            both.centroid.x
        );
    }

    /// The round trip that makes import worth having: a shape exported by
    /// the writer comes back as the same solid, and goes back out analytic.
    #[test]
    fn export_import_export_round_trips_analytically() {
        let block = crate::bounded::BoundedShape::box3(3.0, 2.0, 1.0);
        let rep = ExactRep::Spec(ExactSpec::new(ExactPrim::Block {
            hx: 3.0,
            hy: 2.0,
            hz: 1.0,
        }));
        let written = export_step(&block, Some(&rep), None, Some("mm")).expect("export");
        assert!(written.exact, "precondition: an analytic export");

        let import = read(&written.text);
        assert_eq!(import.counts(), (1, 0, 0));
        assert!(!import.has_errors(), "{:?}", import.diagnostics);
        let shape = import.solids[0].shape.as_ref().expect("shape");

        // 6 x 4 x 2 = 48, and a planar body tessellates exactly.
        let volume = imported_volume(shape);
        assert!((volume - 48.0).abs() < 1e-6, "re-imported volume {volume}");

        // And back out: the imported body carries the analytic surfaces, so
        // the writer must not have to fall back to the faceted path.
        let again =
            export_step(&shape.inner, shape.exact.as_ref(), None, Some("mm")).expect("re-export");
        assert!(again.exact, "an imported B-Rep must re-export analytically");
        let third = read(&again.text);
        assert_eq!(third.counts(), (1, 0, 0), "and survives a second lap");
    }

    #[test]
    fn assembled_is_the_solid_itself_for_a_single_part_file() {
        let import = read(SPHERE);
        let assembled = import.assembled().expect("one solid");
        assert!(
            assembled.exact.is_some(),
            "a single identity-placed solid keeps its analytic body",
        );
        assert_eq!(import.instances.len(), 1);
        assert!(
            !import.is_assembly,
            "a flat single-part file is not an assembly"
        );
    }

    #[test]
    fn a_file_with_no_solids_imports_empty() {
        let empty = "\
ISO-10303-21;
HEADER;
FILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));
ENDSEC;
DATA;
#1 = CARTESIAN_POINT('', (0., 0., 0.));
ENDSEC;
END-ISO-10303-21;
";
        let import = read(empty);
        assert_eq!(import.counts(), (0, 0, 0));
        assert!(import.assembled().is_none(), "nothing to assemble");
        let report = report_of(&import);
        assert!(report.contains("\"solids\":[]"), "{report}");
    }

    #[test]
    fn a_malformed_file_is_an_error_not_a_panic() {
        let Err(err) = import_step(b"not a STEP file at all", None) else {
            panic!("a file that is not Part 21 must not import");
        };
        assert!(err.starts_with("STEP import failed:"), "{err}");
    }

    /// STEP is Latin-1, so bytes that are not valid UTF-8 are a legal file
    /// and must not fail the read.
    #[test]
    fn latin1_bytes_are_read_as_bytes() {
        let source = SPHERE.replace("'ball'", "'ball \u{b0}'");
        let mut bytes: Vec<u8> = Vec::new();
        for c in source.chars() {
            if c == '\u{b0}' {
                bytes.push(0xb0); // Latin-1 degree sign: invalid UTF-8
            } else {
                bytes.push(c as u8);
            }
        }
        assert!(std::str::from_utf8(&bytes).is_err(), "precondition");
        let import = import_step(&bytes, None).expect("Latin-1 is valid STEP");
        assert_eq!(import.counts(), (1, 0, 0));
        assert!(import.solids[0].name.starts_with("ball"));
    }

    #[test]
    fn circle_segments_drive_the_tessellation_fidelity() {
        let coarse = import_step(SPHERE.as_bytes(), Some(8.0)).expect("import");
        let fine = import_step(SPHERE.as_bytes(), Some(96.0)).expect("import");
        assert!(
            fine.solids[0].triangles > coarse.solids[0].triangles * 4,
            "fine {} vs coarse {}",
            fine.solids[0].triangles,
            coarse.solids[0].triangles,
        );
        // A finer tessellation inscribes the sphere more closely, so its
        // volume must rise toward the analytic 33.51.
        let coarse_v = imported_volume(coarse.solids[0].shape.as_ref().unwrap());
        let fine_v = imported_volume(fine.solids[0].shape.as_ref().unwrap());
        assert!(fine_v > coarse_v, "fine {fine_v} vs coarse {coarse_v}");
    }

    #[test]
    fn out_of_range_circle_segments_clamp_rather_than_explode() {
        let tiny = tessellation_options(Some(1.0));
        assert!((tiny.angular_step - std::f64::consts::TAU / 3.0).abs() < 1e-12);
        let huge = tessellation_options(Some(1e9));
        assert!((huge.angular_step - std::f64::consts::TAU / 512.0).abs() < 1e-12);
        let nan = tessellation_options(Some(f64::NAN));
        assert!((nan.angular_step - std::f64::consts::TAU / 32.0).abs() < 1e-12);
        let default = tessellation_options(None);
        assert!((default.angular_step - std::f64::consts::TAU / 32.0).abs() < 1e-12);
    }

    #[test]
    fn the_report_names_every_solid_and_its_outcome() {
        let import = read(SPHERE);
        let report = report_of(&import);
        assert!(report.contains("\"stepId\":20"), "{report}");
        assert!(report.contains("\"outcome\":\"brep\""), "{report}");
        assert!(report.contains("\"name\":\"ball\""), "{report}");
        assert!(report.contains("\"exact\":true"), "{report}");
        assert!(
            report.contains("\"counts\":{\"brep\":1,\"mesh\":0,\"failed\":0}"),
            "{report}"
        );
        assert!(report.contains("\"lengthScale\":1"), "{report}");
        assert!(report.contains("\"isAssembly\":false"), "{report}");
        // One identity occurrence for a flat file.
        assert!(report.contains("\"identity\":true"), "{report}");
    }

    /// A name carrying a quote must not be able to break the report.
    #[test]
    fn report_escapes_hostile_names() {
        // Part 21 escapes an embedded apostrophe by doubling it, so this is
        // the name `he"llo'`.
        let source = SPHERE.replace("'ball'", "'he\"llo'''");
        let import = read(&source);
        let report = report_of(&import);
        assert!(report.contains("\\\"llo"), "{report}");
        // The whole report must still be one well-formed JSON object: a
        // trivial balance check is enough to catch an unescaped quote
        // splitting a string.
        assert_eq!(report.matches('{').count(), report.matches('}').count());
    }
}
