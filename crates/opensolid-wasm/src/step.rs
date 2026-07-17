//! STEP export for playground shapes (of-3qy.5).
//!
//! [`export_step`] serializes a shape to STEP AP203 text through one of two
//! paths, preferring exactness:
//!
//! - **Exact**: the shape carries an exact B-Rep companion ([`ExactRep`]) —
//!   a supported primitive chain or an exact boolean result — and it
//!   serializes through the kernel STEP writer with analytic surfaces.
//! - **Faceted**: everything else (smooth blends, rounded boxes, offsets,
//!   sweeps, anisotropic scales) converts via [`sdf_to_brep`] planar-region
//!   recovery into a faceted-but-valid B-Rep first. The file is real STEP,
//!   but organic geometry arrives as planar facets, at the requested
//!   meshing accuracy.
//!
//! Plain Rust (no wasm-bindgen types) so the logic and both paths are
//! exercised by native `cargo test`; `lib.rs` wraps it for JS.

use opensolid_kernel::brep::{GeometryStore, TopologyStore};
use opensolid_kernel::convert::sdf_to_brep::{SdfToBrepOptions, sdf_to_brep};
use opensolid_kernel::io::step::write::{LengthUnit, StepWriteOptions, write_step};

use crate::bounded::BoundedShape;
use crate::exact::ExactRep;

/// Depth bounds for the faceted path's adaptive meshing, matching the
/// interactive mesher's budget in [`crate::bounded`].
const ADAPTIVE_MIN_DEPTH: u32 = 4;
const ADAPTIVE_MAX_DEPTH: u32 = 9;

/// A serialized STEP file plus which path produced it.
#[derive(Debug)]
pub struct StepExport {
    /// Complete STEP AP203 Part 21 file text.
    pub text: String,
    /// `true` when the exact B-Rep path served analytic surfaces; `false`
    /// when the body was faceted via SDF → B-Rep planar-region recovery.
    pub exact: bool,
}

/// Map a document-unit key (as passed from the playground) to the writer's
/// [`LengthUnit`]. Anything unrecognised — including `None` — falls back to
/// millimetres, the conventional CAD exchange unit and the enum's default.
fn length_unit(key: Option<&str>) -> LengthUnit {
    match key {
        Some("cm") => LengthUnit::Centimetre,
        Some("m") => LengthUnit::Metre,
        Some("in") => LengthUnit::Inch,
        _ => LengthUnit::Millimetre,
    }
}

/// Serialize a shape to STEP: exact when an [`ExactRep`] is present and
/// writable, faceted otherwise. `accuracy` is the faceted path's target
/// chordal deviation in model units (same knob as adaptive meshing);
/// non-finite, non-positive, or absent values fall back to 0.5% of the
/// shape's extent. The exact path ignores it — analytic surfaces have no
/// tessellation error. `unit` is the document unit key (`"mm"`, `"cm"`,
/// `"m"`, `"in"`); it declares the STEP length unit only — coordinates are
/// written verbatim, never rescaled — and unknown keys fall back to
/// millimetres.
///
/// # Errors
/// A human-readable message when the faceted path cannot produce a valid
/// solid (empty shape, or a surface the mesher cannot close).
pub fn export_step(
    inner: &BoundedShape,
    exact: Option<&ExactRep>,
    accuracy: Option<f64>,
    unit: Option<&str>,
) -> Result<StepExport, String> {
    let options = StepWriteOptions {
        length_unit: length_unit(unit),
        ..StepWriteOptions::default()
    };
    if let Some(text) = exact.and_then(|rep| rep.to_step(&options)) {
        return Ok(StepExport { text, exact: true });
    }

    // Faceted fallback: mesh bounds and depth derivation mirror
    // `BoundedShape::mesh_adaptive` so the exported facets match what the
    // viewport shows at the same accuracy.
    let bounds = inner.mesh_bounds(64);
    let extent = {
        let size = bounds.max - bounds.min;
        size.x.max(size.y).max(size.z).max(1e-9)
    };
    let accuracy = match accuracy {
        Some(a) if a.is_finite() && a > 0.0 => a,
        _ => 5e-3 * extent,
    };
    let max_depth = (extent / accuracy)
        .log2()
        .ceil()
        .clamp(ADAPTIVE_MIN_DEPTH as f64, ADAPTIVE_MAX_DEPTH as f64) as u32;
    let mut opts = SdfToBrepOptions::new(bounds, max_depth);
    opts.mesh.accuracy = Some(accuracy);

    let mut store = TopologyStore::new();
    let mut geo = GeometryStore::new();
    let body = sdf_to_brep(&inner.shape, &mut store, &mut geo, &opts)
        .map_err(|e| format!("STEP export failed: {e}"))?;
    let text = write_step(&store, &geo, &[body], &options)
        .map_err(|e| format!("STEP export failed: {e}"))?;
    Ok(StepExport { text, exact: false })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::exact::{ExactPrim, ExactSpec, exact_boolean};
    use opensolid_core::EntityId;
    use opensolid_kernel::brep::BooleanOp;
    use opensolid_kernel::brep::topology::Body;

    use opensolid_kernel::io::step::read::{SolidOutcome, StepReadOptions, read_step};
    use opensolid_kernel::massprops::mass_properties;
    use std::rc::Rc;

    /// Re-import emitted STEP text, requiring every solid to come back as
    /// an exact B-Rep that passes `TopologyStore::check`.
    fn reimport(text: &str) -> (TopologyStore, GeometryStore, Vec<EntityId<Body>>) {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let import = read_step(text, &mut store, &mut geo, &StepReadOptions::default())
            .expect("emitted file must parse");
        assert!(
            !import.has_errors(),
            "reader reported errors: {:?}",
            import.diagnostics
        );
        let bodies: Vec<_> = import
            .solids
            .iter()
            .map(|solid| match &solid.outcome {
                SolidOutcome::BRep(body) => *body,
                other => panic!("expected exact B-Rep re-import, got {other:?}"),
            })
            .collect();
        for &body in &bodies {
            assert!(
                store.check(body).is_empty(),
                "re-imported body must pass check: {:?}",
                store.check(body)
            );
        }
        (store, geo, bodies)
    }

    /// The bead's acceptance test: an exact sphere − cylinder boolean
    /// exports analytic surfaces and re-imports through our own reader
    /// with volume identity.
    ///
    /// Trimmed sphere/cylinder faces cannot be re-tessellated standalone
    /// yet (the CDT pass lives on `BooleanOutput`), so volume identity is
    /// established the way the kernel's e2e suite does: measure the
    /// validated exact tessellation against the closed form, then require
    /// the re-imported body to re-export byte-identically — identical
    /// geometry down to every `f64` encloses the identical volume.
    #[test]
    fn exact_sphere_minus_cylinder_round_trips_with_volume_identity() {
        let sphere = ExactRep::Spec(ExactSpec::new(ExactPrim::Sphere { radius: 1.0 }));
        let hole = ExactRep::Spec(ExactSpec::new(ExactPrim::Cylinder {
            radius: 0.4,
            half_height: 2.0,
        }));
        let out = exact_boolean(BooleanOp::Subtract, &sphere, &hole)
            .expect("sphere - cylinder must stay on the exact path");

        // Napkin ring: a through-drilled sphere encloses 4π/3 (R² − r²)^³ᐟ².
        let analytic = 4.0 * std::f64::consts::PI / 3.0 * (1.0f64 - 0.4 * 0.4).powf(1.5);
        let v1 = mass_properties(&out.mesh)
            .expect("exact mesh is a closed manifold")
            .volume;
        let rel = (v1 - analytic).abs() / analytic;
        assert!(
            rel <= 5e-3,
            "exact tessellation volume {v1} deviates {rel:e} from analytic {analytic}"
        );

        let rep = ExactRep::Boolean(Rc::new(out));
        let text = rep
            .to_step(&StepWriteOptions::default())
            .expect("exact rep must serialize");
        assert!(
            text.contains("SPHERICAL_SURFACE") && text.contains("CYLINDRICAL_SURFACE"),
            "exact export must carry analytic surfaces"
        );

        let (store2, geo2, bodies) = reimport(&text);
        assert_eq!(bodies.len(), 1, "one MANIFOLD_SOLID_BREP expected");
        let text2 = write_step(&store2, &geo2, &[bodies[0]], &StepWriteOptions::default())
            .expect("re-imported body must serialize");
        assert_eq!(
            text, text2,
            "write ∘ read must be a byte-identical fixed point (volume identity)"
        );
    }

    /// Same boolean through the public entry point: the exact path wins and
    /// the faceted machinery is never consulted.
    #[test]
    fn export_step_prefers_the_exact_path() {
        let sphere = ExactRep::Spec(ExactSpec::new(ExactPrim::Sphere { radius: 1.0 }));
        let hole = ExactRep::Spec(ExactSpec::new(ExactPrim::Cylinder {
            radius: 0.4,
            half_height: 2.0,
        }));
        let out = exact_boolean(BooleanOp::Subtract, &sphere, &hole).expect("exact path");
        let rep = ExactRep::Boolean(Rc::new(out));

        let export = export_step(&BoundedShape::sphere(1.0), Some(&rep), Some(0.01), None)
            .expect("exact export succeeds");
        assert!(export.exact);
        assert!(export.text.contains("SPHERICAL_SURFACE"));
    }

    /// A pure primitive spec (no boolean run) exports exact analytic
    /// geometry even though it never built a store.
    #[test]
    fn primitive_spec_exports_analytic_surfaces() {
        let rep = ExactRep::Spec(ExactSpec::new(ExactPrim::Torus {
            major: 1.0,
            minor: 0.3,
        }));
        let export =
            export_step(&BoundedShape::torus(1.0, 0.3), Some(&rep), None, None).expect("exports");
        assert!(export.exact);
        assert!(export.text.contains("TOROIDAL_SURFACE"));
        let (_, _, bodies) = reimport(&export.text);
        assert_eq!(bodies.len(), 1);
    }

    /// The bead's faceted acceptance test: an organic (smooth-blended)
    /// shape exports as a faceted body that round-trips through the reader
    /// without check() failures.
    #[test]
    fn faceted_path_round_trips_without_check_failures() {
        let organic = BoundedShape::rounded_box(0.8, 0.5, 0.6, 0.15).smooth_union(
            &BoundedShape::sphere(0.45)
                .translate(opensolid_core::types::Vector3::new(0.0, 0.55, 0.0)),
            Some(0.2),
        );
        // Coarse accuracy keeps the faceted body small; validity is what is
        // under test, not fidelity.
        let export =
            export_step(&organic, None, Some(0.08), None).expect("faceted export succeeds");
        assert!(!export.exact, "organic shapes must take the faceted path");
        assert!(
            export.text.contains("PLANE"),
            "faceted export is planar faces"
        );
        let (_, _, bodies) = reimport(&export.text);
        assert_eq!(bodies.len(), 1, "one faceted solid expected");
    }

    /// The right-angle bracket from the agent gallery
    /// (`examples/agent-gallery/bracket-right-angle.md`), built here through
    /// the same `BoundedShape` API the JS `create_model` script drives, so
    /// this is the Rust-side half of that part's acceptance gate (of-2y4.1):
    /// the faceted STEP it exports must re-import through our own reader with
    /// volume identity.
    ///
    /// Volume identity follows the same reasoning as
    /// [`exact_sphere_minus_cylinder_round_trips_with_volume_identity`] —
    /// measure the exported body against the closed form, then require
    /// `write ∘ read` to be a fixed point — with one adaptation per side.
    /// The body cannot be measured after import (`tessellate_body` refuses
    /// planar faces with hole loops, and every drilled plate here has them),
    /// and the fixed point is numeric rather than bytewise (the reader
    /// re-normalizes `DIRECTION` vectors, moving scattered facet normals by
    /// ~1 ULP). Both are noted inline where they bite.
    ///
    /// The trailing 360° rotation mirrors the gallery script and is a
    /// workaround, not modelling: it is geometrically the identity, but it
    /// perturbs the tracked bounding box, and without that perturbation this
    /// part meshes open and `sdf_to_brep` refuses to close it (of-obv).
    #[test]
    fn bracket_faceted_step_round_trips_with_volume_identity() {
        use opensolid_core::types::Vector3;
        use opensolid_frep::Profile2D;
        use std::f64::consts::PI;

        // The L cross-section, drawn in (x, z); extrude sweeps it along +Y.
        // Bulge is the DXF convention, tan(sweep/4); negative is clockwise,
        // the concave direction for this interior corner.
        let bulge = (PI / 8.0).tan(); // tan(90°/4) = 0.41421356…
        let l_section = Profile2D::builder([-30.0, 0.0])
            .line_to([30.0, 0.0])
            .line_to([30.0, 5.0])
            .line_to([-22.0, 5.0])
            .arc_to([-25.0, 8.0], -bulge) // 3 mm interior fillet
            .line_to([-25.0, 40.0])
            .line_to([-30.0, 40.0])
            .build()
            .expect("L-section profile is a valid closed loop");
        let ell = BoundedShape::extrude(l_section, 40.0).expect("extrude the 40 mm width");

        let gusset_tri = Profile2D::builder([-25.0, 5.0])
            .line_to([-5.0, 5.0])
            .line_to([-25.0, 25.0])
            .build()
            .expect("gusset triangle is a valid closed loop");
        let gusset = BoundedShape::extrude(gusset_tri, 5.0)
            .expect("extrude the gusset")
            .translate(Vector3::new(0.0, 17.5, 0.0));

        // smoothUnion(gusset, 3): the 3 mm fillets on the gusset edges.
        let mut part = ell.smooth_union(&gusset, Some(3.0));

        // 4x M5 clearance holes. cylinder() is a +Y-axis cylinder, so each is
        // rotated onto its drilling axis: +Z for the base, +X for the wall.
        let z_hole = BoundedShape::cylinder(2.5, 10.0).rotate(Vector3::new(PI / 2.0, 0.0, 0.0));
        for y in [10.0, 30.0] {
            part = part.subtract(&z_hole.translate(Vector3::new(15.0, y, 0.0)));
        }
        let x_hole = BoundedShape::cylinder(2.5, 10.0).rotate(Vector3::new(0.0, 0.0, PI / 2.0));
        for y in [10.0, 30.0] {
            part = part.subtract(&x_hole.translate(Vector3::new(-27.5, y, 32.0)));
        }
        let part = part.rotate(Vector3::new(0.0, 2.0 * PI, 0.0)); // see doc comment

        // Hand-computed section (mm^3): the filleted L swept 40, plus the
        // gusset, less four Ø5 holes through 5 mm of plate. The smoothUnion
        // blend adds ~127 more at the gusset joints, so the tolerance is a
        // band: the mesher also reads ~0.3% under at this accuracy.
        let l_area = 300.0 + 175.0 + (9.0 - PI * 9.0 / 4.0);
        let analytic = l_area * 40.0 + 1000.0 - 4.0 * PI * 2.5f64.powi(2) * 5.0;
        // NAN accuracy falls back to 0.5% of the meshed extent — the same
        // default `export_step` applies below, so this measures the body the
        // export actually facets.
        let volume = mass_properties(&part.mesh_adaptive(f64::NAN, None))
            .expect("bracket mesh is a closed manifold")
            .volume;
        let rel = (volume - analytic).abs() / analytic;
        assert!(
            rel <= 2e-2,
            "bracket volume {volume} deviates {rel:e} from the analytic {analytic}; \
             a hole drilled on the wrong axis shows up here and nowhere else"
        );

        let export = export_step(&part, None, None, None).expect("faceted STEP export succeeds");
        assert!(
            !export.exact,
            "a smooth-blended part takes the faceted path"
        );
        assert!(
            export.text.contains("PLANE"),
            "faceted export is planar faces"
        );

        let (store2, geo2, bodies) = reimport(&export.text);
        assert_eq!(bodies.len(), 1, "one MANIFOLD_SOLID_BREP expected");

        // Measure the re-imported body itself — the thing the round trip is
        // really claiming, and only checkable now that `tessellate_body`
        // bridges the hole loops every drilled plate on this part carries
        // (of-fc8). It is held to the analytic volume, the same bar the
        // exported mesh meets above: a re-import that dropped a hole, paved
        // one over, or lost the gusset misses this band.
        //
        // It is not compared against `volume` at float tolerance, and must
        // not be: `export_step` runs its own `sdf_to_brep` recovery at its
        // own depth rather than serializing the `mesh_adaptive` triangles, so
        // the two are independent facetings of one SDF and sit ~2% apart by
        // construction. Identity of the *file* is what the fixed point below
        // establishes.
        use opensolid_kernel::brep::{TessellationOptions, tessellate_body};
        let reimported_volume = mass_properties(
            &tessellate_body(&store2, &geo2, bodies[0], &TessellationOptions::default())
                .expect("re-imported faceted body must tessellate (of-fc8)"),
        )
        .expect("re-imported bracket mesh is a closed manifold")
        .volume;
        let rel = (reimported_volume - analytic).abs() / analytic;
        assert!(
            rel <= 3e-2,
            "re-imported bracket volume {reimported_volume} deviates {rel:e} \
             from the analytic {analytic}; the STEP round trip lost material"
        );

        // Identity of the file itself: `write ∘ read` as a fixed point, as
        // the exact test does, but compared numerically rather than bytewise
        // — on the faceted path the reader re-normalizes DIRECTION vectors
        // and shifts scattered facet normals by ~1 ULP.
        let text2 = write_step(&store2, &geo2, &[bodies[0]], &StepWriteOptions::default())
            .expect("re-imported body must serialize");
        assert_step_numerically_identical(&export.text, &text2);
    }

    /// Compare two STEP texts, requiring every non-numeric token to match
    /// exactly and every numeric token to agree within 1e-9 relative. This is
    /// the float-tolerant sibling of a byte-identity assertion, for paths
    /// where a round-trip legitimately perturbs the last ULP.
    fn assert_step_numerically_identical(a: &str, b: &str) {
        let (la, lb): (Vec<_>, Vec<_>) = (a.lines().collect(), b.lines().collect());
        assert_eq!(la.len(), lb.len(), "STEP line counts differ");
        for (n, (x, y)) in la.iter().zip(lb.iter()).enumerate() {
            if x == y {
                continue;
            }
            let split = |s: &str| -> Vec<String> {
                s.split([',', '(', ')', ' ']).map(str::to_owned).collect()
            };
            let (tx, ty) = (split(x), split(y));
            assert_eq!(
                tx.len(),
                ty.len(),
                "line {} tokenizes differently:\n{x}\n{y}",
                n + 1
            );
            for (p, q) in tx.iter().zip(ty.iter()) {
                match (p.parse::<f64>(), q.parse::<f64>()) {
                    (Ok(fp), Ok(fq)) => {
                        let scale = fp.abs().max(fq.abs()).max(1.0);
                        assert!(
                            (fp - fq).abs() / scale <= 1e-9,
                            "line {} numeric drift: {fp} vs {fq}",
                            n + 1
                        );
                    }
                    _ => assert_eq!(p, q, "line {} differs on a non-numeric token", n + 1),
                }
            }
        }
    }

    /// A spec whose primitive has no exact B-Rep constructor (spindle
    /// torus) silently falls back to the faceted path.
    #[test]
    fn unsupported_spec_falls_back_to_faceted() {
        let rep = ExactRep::Spec(ExactSpec::new(ExactPrim::Torus {
            major: 0.2,
            minor: 0.5,
        }));
        let export = export_step(&BoundedShape::torus(0.2, 0.5), Some(&rep), Some(0.05), None)
            .expect("faceted fallback succeeds");
        assert!(!export.exact);
        let (_, _, bodies) = reimport(&export.text);
        assert!(!bodies.is_empty());
    }

    /// Degenerate accuracies fall back to the default instead of stalling
    /// or panicking, matching `mesh_adaptive`.
    #[test]
    fn degenerate_accuracy_falls_back_to_default() {
        for acc in [Some(0.0), Some(-1.0), Some(f64::NAN), None] {
            let export =
                export_step(&BoundedShape::box3(0.5, 0.5, 0.5), None, acc, None).expect("exports");
            assert!(!export.exact);
        }
    }

    /// An empty shape (disjoint intersection) reports a clean error instead
    /// of emitting an unreadable file.
    #[test]
    fn empty_shape_reports_an_error() {
        let a = BoundedShape::sphere(0.5);
        let b = BoundedShape::sphere(0.5)
            .translate(opensolid_core::types::Vector3::new(10.0, 0.0, 0.0));
        let err = export_step(&a.intersect(&b), None, Some(0.05), None).unwrap_err();
        assert!(err.contains("STEP export failed"), "{err}");
    }
}
