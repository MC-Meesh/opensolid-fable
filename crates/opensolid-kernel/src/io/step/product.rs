//! AP203/AP214 product structure → placed solid occurrences.
//!
//! [`read`](super::read) maps every `MANIFOLD_SOLID_BREP` in a file to one
//! [`ImportedSolid`](super::read::ImportedSolid), in the coordinate system
//! the solid was authored in. That is the whole story for a single-part
//! file, but an *assembly* file authors each part once at its own origin
//! and then places it — possibly several times — through the product
//! structure. Without resolving that structure a five-part assembly
//! imports as five bodies piled on top of each other at the origin.
//!
//! This module resolves it. [`resolve_instances`] walks the product graph
//! and returns one [`PlacedSolid`] per *occurrence*: the solid's index,
//! the rigid [`Transform3`] that places it in root-assembly space, and the
//! occurrence path that got it there. Geometry is never duplicated — an
//! instance is *(part, transform)*, exactly the model
//! `docs/design/ASSEMBLIES.md` §1 describes, so two bolts are two
//! transforms over one imported body.
//!
//! # The entity graph
//!
//! Two mechanisms place a shape inside another, and real files use both:
//!
//! - **`NEXT_ASSEMBLY_USAGE_OCCURRENCE`** (the AP203 assembly path). The
//!   NAUO names a *(parent product definition, child product definition)*
//!   pair; the placement lives beside it in a
//!   `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION` whose
//!   `represented_product_relation` is a `PRODUCT_DEFINITION_SHAPE` of that
//!   NAUO and whose `representation_relation` carries an
//!   `ITEM_DEFINED_TRANSFORMATION` between two `AXIS2_PLACEMENT_3D`s — one
//!   in the child's representation, one in the parent's. The transform is
//!   `M(parent item) · M(child item)⁻¹`.
//! - **`MAPPED_ITEM`** (the representation-level path, and how AP214/AP242
//!   files often instance a shape). A `MAPPED_ITEM` is an item *of* a
//!   representation; its `REPRESENTATION_MAP` names the mapped
//!   representation plus its origin placement, and its `mapping_target` is
//!   where that origin lands in the referring representation. Same
//!   arithmetic: `M(target) · M(origin)⁻¹`.
//!
//! A product definition reaches its representation through
//! `SHAPE_DEFINITION_REPRESENTATION` → `PRODUCT_DEFINITION_SHAPE`. Many
//! exporters split a part across two representations — a bare
//! `SHAPE_REPRESENTATION` holding only placements, tied to the
//! `ADVANCED_BREP_SHAPE_REPRESENTATION` holding the solid by a plain
//! `SHAPE_REPRESENTATION_RELATIONSHIP` (one with *no*
//! `REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION` part). Those are
//! treated as identity aliases, so a solid is found whichever half the
//! product points at.
//!
//! # Roots, and files with no structure at all
//!
//! The assembly roots are the product definitions that appear as the
//! *parent* of some NAUO but never as a *child*. A file with no NAUO at
//! all has every product as a root, which degrades exactly right: each
//! part's solids are placed at identity. Solids that no product reaches —
//! a representation tied to nothing, which malformed files do produce —
//! are placed at identity too, with an [`Severity::Info`] diagnostic, so
//! the returned list always accounts for every imported solid exactly
//! once at minimum.
//!
//! Cyclic assemblies (a product transitively containing itself) are
//! reported and cut rather than followed, and nesting is capped at
//! [`MAX_DEPTH`].

use std::collections::{HashMap, HashSet};

use nalgebra::{Matrix3, Rotation3, Translation3, UnitQuaternion};
use opensolid_core::{Transform3, Vector3};

use super::read::{
    Diagnostic, MapResult, Severity, conic_frame, instance, invalid, list_attr, name_attr,
    ref_attr, resolve_axis2, resolve_direction, resolve_point, type_names, typed_record,
    unsupported,
};
use super::{SimpleRecord, StepFile, Value};

/// Deepest assembly nesting followed before the walk gives up. Real
/// assemblies are a handful of levels; anything beyond this is a malformed
/// file the cycle guard did not already catch.
const MAX_DEPTH: usize = 64;

/// One placed occurrence of an imported solid.
///
/// The geometry itself stays in its authored (part-local) coordinates —
/// [`transform`](Self::transform) is the placement, applied by the caller
/// however it likes: [`transform_body`](opensolid_brep::transform_body)
/// for an exact B-Rep import, or an F-Rep
/// [`Transformed`](opensolid_frep::transform::Transformed) wrapper for a
/// mesh-fallback one. A part used twice yields two `PlacedSolid`s sharing
/// one [`solid`](Self::solid) index.
#[derive(Debug, Clone)]
pub struct PlacedSolid {
    /// Index into [`StepImport::solids`](super::read::StepImport::solids).
    pub solid: usize,
    /// STEP instance name (`#id`) of the placed `MANIFOLD_SOLID_BREP`.
    pub step_id: u64,
    /// Placement in root-assembly space. Translations are already scaled
    /// by the file's length unit, so this composes directly with imported
    /// geometry (both are millimetres).
    pub transform: Transform3,
    /// Occurrence path from the root product down to this solid: one
    /// entry per `NEXT_ASSEMBLY_USAGE_OCCURRENCE` / `MAPPED_ITEM`
    /// traversed, named by the occurrence (falling back to `#id` when
    /// unnamed). Empty when the solid belongs to the root product itself,
    /// or when the file has no product structure.
    pub path: Vec<String>,
    /// Name of the `PRODUCT` owning the solid's representation, or `""`
    /// when it cannot be resolved.
    pub product: String,
}

impl PlacedSolid {
    /// Whether this occurrence sits at the assembly origin unrotated.
    pub fn is_identity(&self) -> bool {
        self.transform == Transform3::identity()
    }
}

/// Resolve the file's product structure into one [`PlacedSolid`] per solid
/// occurrence.
///
/// `solids` lists the `MANIFOLD_SOLID_BREP` instance names in
/// [`StepImport::solids`](super::read::StepImport::solids) order; the
/// returned `solid` fields index into it. `scale` is the file's length
/// factor (millimetres per file unit), applied to every placement's
/// translation so the transforms compose with already-scaled geometry.
///
/// Never fails: anything unresolvable degrades to an identity placement
/// plus a diagnostic.
pub(super) fn resolve_instances(
    file: &StepFile,
    solids: &[u64],
    scale: f64,
    diagnostics: &mut Vec<Diagnostic>,
) -> Vec<PlacedSolid> {
    if solids.is_empty() {
        return Vec::new();
    }
    let structure = ProductStructure::build(file, solids, scale, diagnostics);
    let mut walk = Walk {
        structure: &structure,
        solids,
        out: Vec::new(),
        diagnostics,
    };
    walk.run();
    let Walk { mut out, .. } = walk;

    // Every solid must be accounted for: one reached by no product at all
    // (a representation tied to no PRODUCT_DEFINITION_SHAPE) still belongs
    // in the file's content, placed where it was authored. That is only
    // worth reporting when the file *has* product structure — a bare
    // geometry file with no products is normal, not degraded.
    let structured = !structure.rep_of_pd.is_empty();
    let placed: HashSet<usize> = out.iter().map(|p| p.solid).collect();
    for (index, &step_id) in solids.iter().enumerate() {
        if placed.contains(&index) {
            continue;
        }
        if structured {
            diagnostics.push(Diagnostic {
                entity: Some(step_id),
                severity: Severity::Info,
                message: "solid is not reached by the file's product structure; \
                          placed at the assembly origin"
                    .to_string(),
            });
        }
        out.push(PlacedSolid {
            solid: index,
            step_id,
            transform: Transform3::identity(),
            path: Vec::new(),
            product: String::new(),
        });
    }
    out.sort_by_key(|p| p.solid);
    out
}

// ---------------------------------------------------------------------
// Transforms
// ---------------------------------------------------------------------

/// Rigid frame of an `AXIS2_PLACEMENT_3D`: columns `(x, y, z)` of its
/// orthonormal basis, translated to its location.
fn axis2_transform(file: &StepFile, id: u64, referrer: u64, scale: f64) -> MapResult<Transform3> {
    let placement = resolve_axis2(file, id, referrer, scale)?;
    let (x, y) = conic_frame(&placement, id)?;
    Ok(frame(placement.location.coords, x, y, x.cross(&y)))
}

fn frame(origin: Vector3, x: Vector3, y: Vector3, z: Vector3) -> Transform3 {
    let rotation = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[x, y, z]));
    Transform3::from_parts(
        Translation3::from(origin),
        UnitQuaternion::from_rotation_matrix(&rotation),
    )
}

/// A `CARTESIAN_TRANSFORMATION_OPERATOR_3D(name, axis1, axis2,
/// local_origin, scale, axis3)` as a rigid frame. Any of the three axes
/// may be `$` (derived); the missing ones are completed to a right-handed
/// orthonormal basis. A declared scale other than 1 is rejected — the
/// kernel's [`Transform3`] is an isometry, and silently dropping the scale
/// would import geometry at the wrong size.
fn operator_transform(
    file: &StepFile,
    rec: &SimpleRecord,
    id: u64,
    scale: f64,
) -> MapResult<Transform3> {
    let origin = resolve_point(file, ref_attr(rec, 3, id)?, id, scale)?;
    let axis_at = |index: usize| -> MapResult<Option<Vector3>> {
        match rec.attributes.get(index) {
            None | Some(Value::Unset) | Some(Value::Derived) => Ok(None),
            Some(Value::Ref(dir)) => Ok(Some(resolve_direction(file, *dir, id)?)),
            Some(_) => Err(invalid(
                id,
                format!("CARTESIAN_TRANSFORMATION_OPERATOR_3D axis {index} is not a reference"),
            )),
        }
    };
    if let Some(Value::Real(factor)) = rec.attributes.get(4)
        && (factor - 1.0).abs() > 1e-12
    {
        return Err(unsupported(
            id,
            format!("CARTESIAN_TRANSFORMATION_OPERATOR_3D scale factor {factor} (rigid only)"),
        ));
    }
    let x_raw = axis_at(1)?;
    let y_raw = axis_at(2)?;
    let z_raw = axis_at(5)?;
    let (x, y, z) = complete_basis(x_raw, y_raw, z_raw).ok_or_else(|| {
        invalid(
            id,
            "CARTESIAN_TRANSFORMATION_OPERATOR_3D axes are degenerate",
        )
    })?;
    Ok(frame(origin.coords, x, y, z))
}

/// Complete a partially specified basis to a right-handed orthonormal one,
/// Gram-Schmidt in the order the operator declares (x wins over y wins over
/// z). Returns `None` if what was given is degenerate.
fn complete_basis(
    x: Option<Vector3>,
    y: Option<Vector3>,
    z: Option<Vector3>,
) -> Option<(Vector3, Vector3, Vector3)> {
    let normalize = |v: Vector3| {
        let n = v.norm();
        (n > 1e-12 && n.is_finite()).then(|| v / n)
    };
    let x = match x.and_then(normalize) {
        Some(x) => x,
        None => match (y.and_then(normalize), z.and_then(normalize)) {
            (Some(y), Some(z)) => normalize(y.cross(&z))?,
            (Some(y), None) => normalize(orthogonal_to(y))?,
            (None, Some(z)) => normalize(orthogonal_to(z))?,
            (None, None) => Vector3::x(),
        },
    };
    let y = match y.and_then(normalize) {
        Some(y) => normalize(y - x * y.dot(&x))?,
        None => match z.and_then(normalize) {
            Some(z) => normalize(z.cross(&x))?,
            None => normalize(orthogonal_to(x))?,
        },
    };
    Some((x, y, x.cross(&y)))
}

/// Any unit vector perpendicular to `v` (assumed unit-length).
fn orthogonal_to(v: Vector3) -> Vector3 {
    let seed = if v.x.abs() < 0.9 {
        Vector3::x()
    } else {
        Vector3::y()
    };
    seed - v * seed.dot(&v)
}

/// A transformation operator as used by `ITEM_DEFINED_TRANSFORMATION` and
/// `MAPPED_ITEM`: either an `AXIS2_PLACEMENT_3D` or a
/// `CARTESIAN_TRANSFORMATION_OPERATOR_3D`.
fn placement_or_operator(
    file: &StepFile,
    id: u64,
    referrer: u64,
    scale: f64,
) -> MapResult<Transform3> {
    let inst = instance(file, id, referrer)?;
    if inst.entity.part("AXIS2_PLACEMENT_3D").is_some() {
        axis2_transform(file, id, referrer, scale)
    } else if let Some(rec) = inst.entity.part("CARTESIAN_TRANSFORMATION_OPERATOR_3D") {
        operator_transform(file, rec, id, scale)
    } else {
        Err(unsupported(
            id,
            format!(
                "placement operator {} (expected AXIS2_PLACEMENT_3D or \
                 CARTESIAN_TRANSFORMATION_OPERATOR_3D)",
                type_names(inst)
            ),
        ))
    }
}

// ---------------------------------------------------------------------
// Structure extraction
// ---------------------------------------------------------------------

/// One child slot of a product: the NAUO (or mapped item) that places it.
#[derive(Debug, Clone)]
struct Occurrence {
    /// The child product definition (NAUO) — `None` for a `MAPPED_ITEM`,
    /// which names a representation directly.
    child_pd: Option<u64>,
    /// The mapped representation, for a `MAPPED_ITEM`.
    child_rep: Option<u64>,
    transform: Transform3,
    name: String,
}

/// Everything the walk needs, extracted from the entity graph in one pass.
struct ProductStructure {
    /// `PRODUCT_DEFINITION` → its shape representation.
    rep_of_pd: HashMap<u64, u64>,
    /// Representation → the solids it holds directly (only ones the reader
    /// actually mapped), in file order.
    rep_solids: HashMap<u64, Vec<u64>>,
    /// Representation → the `MAPPED_ITEM`s among its items.
    rep_mapped: HashMap<u64, Vec<Occurrence>>,
    /// Representations linked by a plain `SHAPE_REPRESENTATION_RELATIONSHIP`
    /// (no transformation): mutually identity-aliased.
    rep_aliases: HashMap<u64, Vec<u64>>,
    /// Parent product definition → its NAUO children.
    children: HashMap<u64, Vec<Occurrence>>,
    /// Product definition → the owning `PRODUCT`'s name.
    product_name: HashMap<u64, String>,
    /// Root product definitions, in file order.
    roots: Vec<u64>,
}

impl ProductStructure {
    fn build(
        file: &StepFile,
        solids: &[u64],
        scale: f64,
        diagnostics: &mut Vec<Diagnostic>,
    ) -> Self {
        let solid_set: HashSet<u64> = solids.iter().copied().collect();
        let mut rep_of_pd = HashMap::new();
        let mut rep_solids: HashMap<u64, Vec<u64>> = HashMap::new();
        let mut rep_mapped: HashMap<u64, Vec<Occurrence>> = HashMap::new();
        let mut rep_aliases: HashMap<u64, Vec<u64>> = HashMap::new();
        let mut product_name = HashMap::new();
        // NAUO id → (parent pd, child pd, occurrence name), in file order.
        let mut nauos: Vec<(u64, u64, u64, String)> = Vec::new();
        let mut pd_order: Vec<u64> = Vec::new();

        for inst in &file.data {
            let id = inst.id;
            if let Some(rec) = inst.entity.part("PRODUCT_DEFINITION") {
                pd_order.push(id);
                match product_of_definition(file, rec, id) {
                    Ok(name) => {
                        product_name.insert(id, name);
                    }
                    Err(e) => diagnostics.push(e.diagnostic()),
                }
            }
            if let Some(rec) = representation_part(&inst.entity) {
                let items = list_attr(rec, 1, id).ok().map(<[Value]>::to_vec);
                for item in items.into_iter().flatten() {
                    let Some(item_id) = item.as_ref_id() else {
                        continue;
                    };
                    if solid_set.contains(&item_id) {
                        rep_solids.entry(id).or_default().push(item_id);
                        continue;
                    }
                    let Some(item_inst) = file.get(item_id) else {
                        continue;
                    };
                    let Some(mapped) = item_inst.entity.part("MAPPED_ITEM") else {
                        continue;
                    };
                    match resolve_mapped_item(file, mapped, item_id, scale) {
                        Ok(occ) => rep_mapped.entry(id).or_default().push(occ),
                        Err(e) => diagnostics.push(e.diagnostic()),
                    }
                }
            }
            if let Some(rec) = inst.entity.part("SHAPE_DEFINITION_REPRESENTATION") {
                match shape_definition(file, rec, id) {
                    Ok(Some((pd, rep))) => {
                        rep_of_pd.entry(pd).or_insert(rep);
                    }
                    Ok(None) => {}
                    Err(e) => diagnostics.push(e.diagnostic()),
                }
            }
            // A relationship *with* a transformation is an assembly
            // placement (handled through its CDSR); one without is an
            // identity alias between two views of the same shape.
            if inst
                .entity
                .part("SHAPE_REPRESENTATION_RELATIONSHIP")
                .is_some()
                && inst
                    .entity
                    .part("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
                    .is_none()
                && let Ok((a, b)) = relationship_reps(inst, id)
            {
                rep_aliases.entry(a).or_default().push(b);
                rep_aliases.entry(b).or_default().push(a);
            }
            if let Some(rec) = inst.entity.part("NEXT_ASSEMBLY_USAGE_OCCURRENCE") {
                match assembly_usage(inst, rec, id) {
                    Ok((parent, child, name)) => nauos.push((id, parent, child, name)),
                    Err(e) => diagnostics.push(e.diagnostic()),
                }
            }
        }

        // Placements live in CONTEXT_DEPENDENT_SHAPE_REPRESENTATIONs beside
        // the NAUOs; resolving them needs rep_of_pd, hence the second pass.
        let mut nauo_transform: HashMap<u64, Transform3> = HashMap::new();
        for inst in &file.data {
            let Some(rec) = inst.entity.part("CONTEXT_DEPENDENT_SHAPE_REPRESENTATION") else {
                continue;
            };
            match resolve_context_dependent(file, rec, inst.id, &rep_of_pd, scale) {
                Ok((nauo, transform)) => {
                    nauo_transform.insert(nauo, transform);
                }
                Err(e) => diagnostics.push(e.diagnostic()),
            }
        }

        let mut children: HashMap<u64, Vec<Occurrence>> = HashMap::new();
        let mut non_root: HashSet<u64> = HashSet::new();
        for (nauo, parent, child, name) in nauos {
            let transform = match nauo_transform.get(&nauo) {
                Some(t) => *t,
                None => {
                    diagnostics.push(Diagnostic {
                        entity: Some(nauo),
                        severity: Severity::Warning,
                        message: "assembly occurrence has no \
                                  CONTEXT_DEPENDENT_SHAPE_REPRESENTATION placement; \
                                  component placed at its parent's origin"
                            .to_string(),
                    });
                    Transform3::identity()
                }
            };
            non_root.insert(child);
            children.entry(parent).or_default().push(Occurrence {
                child_pd: Some(child),
                child_rep: None,
                transform,
                name,
            });
        }

        let roots = pd_order
            .iter()
            .copied()
            .filter(|pd| !non_root.contains(pd))
            .collect();

        Self {
            rep_of_pd,
            rep_solids,
            rep_mapped,
            rep_aliases,
            children,
            product_name,
            roots,
        }
    }

    /// A representation plus every representation identity-aliased to it,
    /// transitively.
    fn alias_closure(&self, rep: u64) -> Vec<u64> {
        let mut seen = HashSet::from([rep]);
        let mut stack = vec![rep];
        let mut out = vec![rep];
        while let Some(current) = stack.pop() {
            for &next in self.rep_aliases.get(&current).into_iter().flatten() {
                if seen.insert(next) {
                    out.push(next);
                    stack.push(next);
                }
            }
        }
        out
    }
}

/// The item list attribute of a `REPRESENTATION` subtype, if `entity` is
/// one. `SHAPE_DEFINITION_REPRESENTATION` deliberately does not match: it
/// relates a definition to a representation rather than being one.
fn representation_part(entity: &super::EntityRecord) -> Option<&SimpleRecord> {
    let matches = |rec: &SimpleRecord| {
        (rec.type_name == "REPRESENTATION"
            || rec.type_name == "SHAPE_REPRESENTATION"
            || rec.type_name.ends_with("_SHAPE_REPRESENTATION"))
            && matches!(rec.attributes.get(1), Some(Value::List(_)))
    };
    match entity {
        super::EntityRecord::Simple(rec) => matches(rec).then_some(rec),
        super::EntityRecord::Complex(recs) => recs.iter().find(|rec| matches(rec)),
    }
}

/// `PRODUCT_DEFINITION(id, description, formation, frame)` → the name of
/// the `PRODUCT` it is a definition of.
fn product_of_definition(file: &StepFile, rec: &SimpleRecord, id: u64) -> MapResult<String> {
    let formation_ref = ref_attr(rec, 2, id)?;
    let formation = instance(file, formation_ref, id)?;
    let formation_rec = formation
        .entity
        .part("PRODUCT_DEFINITION_FORMATION_WITH_SPECIFIED_SOURCE")
        .or_else(|| formation.entity.part("PRODUCT_DEFINITION_FORMATION"))
        .ok_or_else(|| {
            invalid(
                formation_ref,
                format!(
                    "expected a PRODUCT_DEFINITION_FORMATION, found {}",
                    type_names(formation)
                ),
            )
        })?;
    let product_ref = ref_attr(formation_rec, 2, formation_ref)?;
    let product = typed_record(file, product_ref, "PRODUCT", formation_ref)?;
    // `PRODUCT(id, name, description, frame_of_reference)`: prefer the
    // human-facing name, fall back to the identifier.
    let name = product
        .attributes
        .get(1)
        .and_then(Value::as_str)
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .unwrap_or_else(|| name_attr(product));
    Ok(name)
}

/// `SHAPE_DEFINITION_REPRESENTATION(definition, used_representation)` →
/// *(product definition, representation)*.
///
/// Returns `None` — not an error — when the relation does not define a
/// product's shape. Real files anchor other things this way too: a bare
/// `PROPERTY_DEFINITION` ("shape for solid data with which properties are
/// associated") or a shape aspect, neither of which names a
/// `PRODUCT_DEFINITION`. Those are simply not part of the assembly tree.
fn shape_definition(file: &StepFile, rec: &SimpleRecord, id: u64) -> MapResult<Option<(u64, u64)>> {
    let shape_ref = ref_attr(rec, 0, id)?;
    let rep = ref_attr(rec, 1, id)?;
    let shape = instance(file, shape_ref, id)?;
    // `PRODUCT_DEFINITION_SHAPE(name, description, definition)`, whose
    // supertype `PROPERTY_DEFINITION` has the same three attributes.
    let Some(shape_rec) = shape
        .entity
        .part("PRODUCT_DEFINITION_SHAPE")
        .or_else(|| shape.entity.part("PROPERTY_DEFINITION"))
    else {
        return Ok(None);
    };
    let pd = ref_attr(shape_rec, 2, shape_ref)?;
    let defines_a_product = instance(file, pd, shape_ref)?
        .entity
        .part("PRODUCT_DEFINITION")
        .is_some();
    Ok(defines_a_product.then_some((pd, rep)))
}

/// The related representations of a `REPRESENTATION_RELATIONSHIP(name,
/// description, rep_1, rep_2)`, whether written simply or as one part of a
/// complex instance (where the subtype parts carry no attributes).
fn relationship_reps(inst: &super::Instance, id: u64) -> MapResult<(u64, u64)> {
    let rec = [
        "REPRESENTATION_RELATIONSHIP",
        "SHAPE_REPRESENTATION_RELATIONSHIP",
    ]
    .iter()
    .filter_map(|name| inst.entity.part(name))
    .find(|rec| rec.attributes.len() >= 4)
    .ok_or_else(|| {
        invalid(
            id,
            format!("{} carries no rep_1/rep_2 attributes", type_names(inst)),
        )
    })?;
    Ok((ref_attr(rec, 2, id)?, ref_attr(rec, 3, id)?))
}

/// `NEXT_ASSEMBLY_USAGE_OCCURRENCE(id, name, description,
/// relating_product_definition, related_product_definition,
/// reference_designator)` → *(parent, child, occurrence name)*. When the
/// NAUO is one part of a complex instance its attributes may sit on the
/// `ASSEMBLY_COMPONENT_USAGE` / `PRODUCT_DEFINITION_RELATIONSHIP` part
/// instead.
fn assembly_usage(
    inst: &super::Instance,
    nauo: &SimpleRecord,
    id: u64,
) -> MapResult<(u64, u64, String)> {
    let rec = if nauo.attributes.len() >= 5 {
        nauo
    } else {
        [
            "ASSEMBLY_COMPONENT_USAGE",
            "PRODUCT_DEFINITION_RELATIONSHIP",
        ]
        .iter()
        .filter_map(|name| inst.entity.part(name))
        .find(|rec| rec.attributes.len() >= 5)
        .ok_or_else(|| {
            invalid(
                id,
                "NEXT_ASSEMBLY_USAGE_OCCURRENCE carries no relating/related product definitions",
            )
        })?
    };
    let parent = ref_attr(rec, 3, id)?;
    let child = ref_attr(rec, 4, id)?;
    // Prefer the occurrence name, then its id; both are commonly set and
    // together they are what a CAD tree shows ("nut_1").
    let name = rec
        .attributes
        .get(1)
        .and_then(Value::as_str)
        .filter(|s| !s.is_empty())
        .map(str::to_string)
        .or_else(|| {
            rec.attributes
                .first()
                .and_then(Value::as_str)
                .filter(|s| !s.is_empty())
                .map(str::to_string)
        })
        .unwrap_or_else(|| format!("#{id}"));
    Ok((parent, child, name))
}

/// `MAPPED_ITEM(name, mapping_source, mapping_target)` → the mapped
/// representation and the transform placing it in the referring one.
fn resolve_mapped_item(
    file: &StepFile,
    rec: &SimpleRecord,
    id: u64,
    scale: f64,
) -> MapResult<Occurrence> {
    let source_ref = ref_attr(rec, 1, id)?;
    let target_ref = ref_attr(rec, 2, id)?;
    // `REPRESENTATION_MAP(mapping_origin, mapped_representation)`.
    let map = typed_record(file, source_ref, "REPRESENTATION_MAP", id)?;
    let origin_ref = ref_attr(map, 0, source_ref)?;
    let mapped_rep = ref_attr(map, 1, source_ref)?;
    let origin = placement_or_operator(file, origin_ref, source_ref, scale)?;
    let target = placement_or_operator(file, target_ref, id, scale)?;
    let name = name_attr(rec);
    Ok(Occurrence {
        child_pd: None,
        child_rep: Some(mapped_rep),
        transform: target * origin.inverse(),
        name: if name.is_empty() {
            format!("#{id}")
        } else {
            name
        },
    })
}

/// `CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(representation_relation,
/// represented_product_relation)` → *(the NAUO it places, its transform)*.
///
/// The `ITEM_DEFINED_TRANSFORMATION`'s two items live one in each related
/// representation. Which of `rep_1`/`rep_2` is the *child*'s decides the
/// direction, so it is read from the product structure rather than assumed:
/// exporters do write the pair either way round.
fn resolve_context_dependent(
    file: &StepFile,
    rec: &SimpleRecord,
    id: u64,
    rep_of_pd: &HashMap<u64, u64>,
    scale: f64,
) -> MapResult<(u64, Transform3)> {
    let relation_ref = ref_attr(rec, 0, id)?;
    let product_relation_ref = ref_attr(rec, 1, id)?;

    // The NAUO this placement is for.
    let shape = typed_record(file, product_relation_ref, "PRODUCT_DEFINITION_SHAPE", id)?;
    let nauo_ref = ref_attr(shape, 2, product_relation_ref)?;
    let nauo_inst = instance(file, nauo_ref, product_relation_ref)?;
    let nauo = nauo_inst
        .entity
        .part("NEXT_ASSEMBLY_USAGE_OCCURRENCE")
        .ok_or_else(|| {
            unsupported(
                nauo_ref,
                format!(
                    "CONTEXT_DEPENDENT_SHAPE_REPRESENTATION places {} \
                     (only NEXT_ASSEMBLY_USAGE_OCCURRENCE is mapped)",
                    type_names(nauo_inst)
                ),
            )
        })?;
    let (_, child_pd, _) = assembly_usage(nauo_inst, nauo, nauo_ref)?;

    // The transformation between the two representations.
    let relation = instance(file, relation_ref, id)?;
    let with_transform = relation
        .entity
        .part("REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION")
        .ok_or_else(|| {
            unsupported(
                relation_ref,
                format!(
                    "assembly placement {} carries no \
                     REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION",
                    type_names(relation)
                ),
            )
        })?;
    let operator_ref = ref_attr(with_transform, 0, relation_ref)?;
    let operator = instance(file, operator_ref, relation_ref)?;
    let idt = operator
        .entity
        .part("ITEM_DEFINED_TRANSFORMATION")
        .ok_or_else(|| {
            unsupported(
                operator_ref,
                format!(
                    "transformation operator {} (only ITEM_DEFINED_TRANSFORMATION is mapped)",
                    type_names(operator)
                ),
            )
        })?;
    let item_1 = placement_or_operator(file, ref_attr(idt, 2, operator_ref)?, operator_ref, scale)?;
    let item_2 = placement_or_operator(file, ref_attr(idt, 3, operator_ref)?, operator_ref, scale)?;

    // `item_1` belongs to `rep_1` and `item_2` to `rep_2`; the placement
    // maps the child's frame into the parent's.
    let (rep_1, rep_2) = relationship_reps(relation, relation_ref)?;
    let child_rep = rep_of_pd.get(&child_pd).copied();
    let child_is_second = child_rep == Some(rep_2) && child_rep != Some(rep_1);
    let transform = if child_is_second {
        item_1 * item_2.inverse()
    } else {
        item_2 * item_1.inverse()
    };
    Ok((nauo_ref, transform))
}

// ---------------------------------------------------------------------
// The walk
// ---------------------------------------------------------------------

struct Walk<'a> {
    structure: &'a ProductStructure,
    solids: &'a [u64],
    out: Vec<PlacedSolid>,
    diagnostics: &'a mut Vec<Diagnostic>,
}

impl Walk<'_> {
    fn run(&mut self) {
        for root in self.structure.roots.clone() {
            let mut visiting = Vec::new();
            self.product(root, Transform3::identity(), &[], &mut visiting);
        }
    }

    /// Place everything product definition `pd` contributes, itself placed
    /// by `transform`.
    fn product(
        &mut self,
        pd: u64,
        transform: Transform3,
        path: &[String],
        visiting: &mut Vec<u64>,
    ) {
        if visiting.contains(&pd) {
            self.diagnostics.push(Diagnostic {
                entity: Some(pd),
                severity: Severity::Error,
                message: "cyclic assembly structure: product definition contains itself; \
                          the recursive occurrence is dropped"
                    .to_string(),
            });
            return;
        }
        if visiting.len() >= MAX_DEPTH {
            self.diagnostics.push(Diagnostic {
                entity: Some(pd),
                severity: Severity::Error,
                message: format!("assembly nesting deeper than {MAX_DEPTH}; occurrence dropped"),
            });
            return;
        }
        visiting.push(pd);

        let product = self
            .structure
            .product_name
            .get(&pd)
            .cloned()
            .unwrap_or_default();
        if let Some(&rep) = self.structure.rep_of_pd.get(&pd) {
            let mut rep_stack = Vec::new();
            self.representation(rep, transform, path, &product, &mut rep_stack);
        }
        for occurrence in self.structure.children.get(&pd).into_iter().flatten() {
            let mut child_path = path.to_vec();
            child_path.push(occurrence.name.clone());
            if let Some(child) = occurrence.child_pd {
                self.product(
                    child,
                    transform * occurrence.transform,
                    &child_path,
                    visiting,
                );
            }
        }

        visiting.pop();
    }

    /// Place the solids of representation `rep` (and of everything it maps
    /// in), itself placed by `transform`.
    fn representation(
        &mut self,
        rep: u64,
        transform: Transform3,
        path: &[String],
        product: &str,
        rep_stack: &mut Vec<u64>,
    ) {
        if rep_stack.contains(&rep) || rep_stack.len() >= MAX_DEPTH {
            self.diagnostics.push(Diagnostic {
                entity: Some(rep),
                severity: Severity::Error,
                message: "cyclic or over-deep MAPPED_ITEM chain; occurrence dropped".to_string(),
            });
            return;
        }
        rep_stack.push(rep);
        for view in self.structure.alias_closure(rep) {
            for &step_id in self.structure.rep_solids.get(&view).into_iter().flatten() {
                let Some(solid) = self.solids.iter().position(|&s| s == step_id) else {
                    continue;
                };
                self.out.push(PlacedSolid {
                    solid,
                    step_id,
                    transform,
                    path: path.to_vec(),
                    product: product.to_string(),
                });
            }
            for occurrence in self.structure.rep_mapped.get(&view).into_iter().flatten() {
                let Some(child_rep) = occurrence.child_rep else {
                    continue;
                };
                let mut child_path = path.to_vec();
                child_path.push(occurrence.name.clone());
                self.representation(
                    child_rep,
                    transform * occurrence.transform,
                    &child_path,
                    product,
                    rep_stack,
                );
            }
        }
        rep_stack.pop();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_core::Point3;
    use std::f64::consts::FRAC_PI_2;

    fn parse(data: &str) -> StepFile {
        let src = format!(
            "ISO-10303-21;\nHEADER;\nFILE_SCHEMA(('CONFIG_CONTROL_DESIGN'));\nENDSEC;\n\
             DATA;\n{data}\nENDSEC;\nEND-ISO-10303-21;\n"
        );
        super::super::parse(&src).expect("valid Part 21")
    }

    fn resolve(data: &str, solids: &[u64]) -> (Vec<PlacedSolid>, Vec<Diagnostic>) {
        let file = parse(data);
        let mut diagnostics = Vec::new();
        let placed = resolve_instances(&file, solids, 1.0, &mut diagnostics);
        (placed, diagnostics)
    }

    /// Minimal product skeleton for one product: definition #base,
    /// formation #base+1, product #base+2, shape #base+3, and a
    /// representation #base+4 holding `items`.
    fn product_block(base: u64, name: &str, items: &str) -> String {
        format!(
            "#{pd} = PRODUCT_DEFINITION('design','',#{formation},#0);\n\
             #{formation} = PRODUCT_DEFINITION_FORMATION('','',#{product});\n\
             #{product} = PRODUCT('{name}','{name}','',());\n\
             #{shape} = PRODUCT_DEFINITION_SHAPE('','',#{pd});\n\
             #{rep} = ADVANCED_BREP_SHAPE_REPRESENTATION('',({items}),#0);\n\
             #{sdr} = SHAPE_DEFINITION_REPRESENTATION(#{shape},#{rep});\n",
            pd = base,
            formation = base + 1,
            product = base + 2,
            shape = base + 3,
            rep = base + 4,
            sdr = base + 5,
        )
    }

    /// An `AXIS2_PLACEMENT_3D` at `(x,y,z)` with the given z axis and x
    /// reference direction, occupying ids `base..base+4`.
    fn axis2(base: u64, at: [f64; 3], z: [f64; 3], x: [f64; 3]) -> String {
        format!(
            "#{p} = CARTESIAN_POINT('',({ax},{ay},{az}));\n\
             #{d} = DIRECTION('',({zx},{zy},{zz}));\n\
             #{r} = DIRECTION('',({xx},{xy},{xz}));\n\
             #{a} = AXIS2_PLACEMENT_3D('',#{p},#{d},#{r});\n",
            p = base,
            d = base + 1,
            r = base + 2,
            a = base + 3,
            ax = at[0],
            ay = at[1],
            az = at[2],
            zx = z[0],
            zy = z[1],
            zz = z[2],
            xx = x[0],
            xy = x[1],
            xz = x[2],
        )
    }

    const WORLD: [f64; 3] = [0.0, 0.0, 0.0];
    const ZP: [f64; 3] = [0.0, 0.0, 1.0];
    const XP: [f64; 3] = [1.0, 0.0, 0.0];

    #[test]
    fn flat_file_places_every_solid_at_the_origin() {
        // No product structure at all: the reader's existing behaviour must
        // survive as identity placements.
        let (placed, _) = resolve("#1 = MANIFOLD_SOLID_BREP('a',#2);\n", &[1]);
        assert_eq!(placed.len(), 1);
        assert_eq!(placed[0].solid, 0);
        assert!(placed[0].is_identity());
        assert!(placed[0].path.is_empty());
    }

    #[test]
    fn single_product_places_its_solid_at_the_origin() {
        let data = format!(
            "#1 = MANIFOLD_SOLID_BREP('part',#2);\n{}",
            product_block(10, "widget", "#1")
        );
        let (placed, diagnostics) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 1);
        assert!(placed[0].is_identity());
        assert_eq!(placed[0].product, "widget");
        assert!(
            diagnostics.is_empty(),
            "clean single product should be silent: {diagnostics:?}"
        );
    }

    /// Two-part assembly: the child is placed 10mm along +X of the parent.
    fn two_part_assembly() -> String {
        let mut data = String::new();
        data.push_str("#1 = MANIFOLD_SOLID_BREP('child',#2);\n");
        // Parent (assembly) product, its rep holds the target placement.
        data.push_str(&product_block(10, "assembly", "#30"));
        // Child product, its rep holds the solid and the origin placement.
        data.push_str(&product_block(20, "child", "#1,#40"));
        data.push_str(&axis2(30, [10.0, 0.0, 0.0], ZP, XP));
        data.push_str(&axis2(40, WORLD, ZP, XP));
        data.push_str(
            "#50 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','child_1','',#10,#20,$);\n\
             #51 = PRODUCT_DEFINITION_SHAPE('Placement','',#50);\n\
             #52 = ITEM_DEFINED_TRANSFORMATION('','',#43,#33);\n\
             #53 = ( REPRESENTATION_RELATIONSHIP('','',#24,#14) \
             REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#52) \
             SHAPE_REPRESENTATION_RELATIONSHIP() );\n\
             #54 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#53,#51);\n",
        );
        data
    }

    #[test]
    fn nauo_placement_moves_the_component() {
        let (placed, diagnostics) = resolve(&two_part_assembly(), &[1]);
        assert_eq!(placed.len(), 1, "diagnostics: {diagnostics:?}");
        assert_eq!(placed[0].path, vec!["child_1".to_string()]);
        assert_eq!(placed[0].product, "child");
        let origin = placed[0].transform * Point3::origin();
        assert!(
            (origin - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-12,
            "component landed at {origin:?}"
        );
    }

    #[test]
    fn reversed_relationship_pair_still_places_correctly() {
        // Same file with rep_1/rep_2 swapped — exporters write it both
        // ways, and the direction must come from the product structure,
        // not from the attribute order.
        let data = two_part_assembly().replace(
            "REPRESENTATION_RELATIONSHIP('','',#24,#14)",
            "REPRESENTATION_RELATIONSHIP('','',#14,#24)",
        );
        let data = data.replace(
            "ITEM_DEFINED_TRANSFORMATION('','',#43,#33)",
            "ITEM_DEFINED_TRANSFORMATION('','',#33,#43)",
        );
        let (placed, _) = resolve(&data, &[1]);
        let origin = placed[0].transform * Point3::origin();
        assert!(
            (origin - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-12,
            "component landed at {origin:?}"
        );
    }

    #[test]
    fn one_part_used_twice_yields_two_occurrences() {
        let mut data = two_part_assembly();
        data.push_str(&axis2(60, [0.0, 7.0, 0.0], ZP, XP));
        data.push_str(
            "#70 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','child_2','',#10,#20,$);\n\
             #71 = PRODUCT_DEFINITION_SHAPE('Placement','',#70);\n\
             #72 = ITEM_DEFINED_TRANSFORMATION('','',#43,#63);\n\
             #73 = ( REPRESENTATION_RELATIONSHIP('','',#24,#14) \
             REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#72) \
             SHAPE_REPRESENTATION_RELATIONSHIP() );\n\
             #74 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#73,#71);\n",
        );
        // The assembly rep must list the second target too.
        let data = data.replace(
            "ADVANCED_BREP_SHAPE_REPRESENTATION('',(#30),#0)",
            "ADVANCED_BREP_SHAPE_REPRESENTATION('',(#30,#60),#0)",
        );
        let (placed, _) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 2, "one part used twice is two occurrences");
        assert!(placed.iter().all(|p| p.solid == 0), "sharing one body");
        let mut origins: Vec<Point3> = placed
            .iter()
            .map(|p| p.transform * Point3::origin())
            .collect();
        origins.sort_by(|a, b| a.x.total_cmp(&b.x));
        assert!((origins[0] - Point3::new(0.0, 7.0, 0.0)).norm() < 1e-12);
        assert!((origins[1] - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn nested_subassembly_composes_transforms() {
        // root → mid (translate +X 10) → leaf (rotate 90° about Z, +Y 5).
        let mut data = String::from("#1 = MANIFOLD_SOLID_BREP('leaf',#2);\n");
        data.push_str(&product_block(10, "root", "#30"));
        data.push_str(&product_block(20, "mid", "#40,#50"));
        data.push_str(&product_block(60, "leaf", "#1,#70"));
        data.push_str(&axis2(30, [10.0, 0.0, 0.0], ZP, XP)); // mid in root
        data.push_str(&axis2(40, WORLD, ZP, XP)); // mid's own origin
        data.push_str(&axis2(50, [0.0, 5.0, 0.0], ZP, [0.0, 1.0, 0.0])); // leaf in mid
        data.push_str(&axis2(70, WORLD, ZP, XP)); // leaf's own origin
        data.push_str(
            "#80 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','mid_1','',#10,#20,$);\n\
             #81 = PRODUCT_DEFINITION_SHAPE('Placement','',#80);\n\
             #82 = ITEM_DEFINED_TRANSFORMATION('','',#43,#33);\n\
             #83 = ( REPRESENTATION_RELATIONSHIP('','',#24,#14) \
             REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#82) \
             SHAPE_REPRESENTATION_RELATIONSHIP() );\n\
             #84 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#83,#81);\n\
             #90 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','leaf_1','',#20,#60,$);\n\
             #91 = PRODUCT_DEFINITION_SHAPE('Placement','',#90);\n\
             #92 = ITEM_DEFINED_TRANSFORMATION('','',#73,#53);\n\
             #93 = ( REPRESENTATION_RELATIONSHIP('','',#64,#24) \
             REPRESENTATION_RELATIONSHIP_WITH_TRANSFORMATION(#92) \
             SHAPE_REPRESENTATION_RELATIONSHIP() );\n\
             #94 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#93,#91);\n",
        );
        let (placed, diagnostics) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 1, "diagnostics: {diagnostics:?}");
        assert_eq!(
            placed[0].path,
            vec!["mid_1".to_string(), "leaf_1".to_string()]
        );
        let origin = placed[0].transform * Point3::origin();
        assert!(
            (origin - Point3::new(10.0, 5.0, 0.0)).norm() < 1e-12,
            "leaf landed at {origin:?}"
        );
        // The leaf's placement rotates +X onto +Y (its ref_direction is
        // +Y), and the parent's translation does not rotate it further.
        let mapped = placed[0].transform * Vector3::x();
        assert!(
            (mapped - Vector3::y()).norm() < 1e-12,
            "leaf axes rotated to {mapped:?}"
        );
        assert!((placed[0].transform.rotation.angle() - FRAC_PI_2).abs() < 1e-12);
    }

    #[test]
    fn mapped_item_places_the_mapped_representation() {
        let mut data = String::from("#1 = MANIFOLD_SOLID_BREP('mapped',#2);\n");
        // The referring representation holds only the mapped item.
        data.push_str(&product_block(10, "assembly", "#30"));
        data.push_str(&axis2(40, WORLD, ZP, XP)); // map origin
        data.push_str(&axis2(50, [0.0, 0.0, 3.0], ZP, XP)); // map target
        data.push_str(
            "#20 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#1),#0);\n\
             #21 = REPRESENTATION_MAP(#43,#20);\n\
             #30 = MAPPED_ITEM('inst',#21,#53);\n",
        );
        let (placed, diagnostics) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 1, "diagnostics: {diagnostics:?}");
        assert_eq!(placed[0].path, vec!["inst".to_string()]);
        let origin = placed[0].transform * Point3::origin();
        assert!(
            (origin - Point3::new(0.0, 0.0, 3.0)).norm() < 1e-12,
            "mapped item landed at {origin:?}"
        );
    }

    #[test]
    fn plain_shape_representation_relationship_aliases_two_views() {
        // The product points at a placement-only SHAPE_REPRESENTATION; the
        // solid lives in an ADVANCED_BREP_SHAPE_REPRESENTATION linked by a
        // transformation-free relationship. The solid must still be found.
        let mut data = String::from("#1 = MANIFOLD_SOLID_BREP('part',#2);\n");
        data.push_str(&product_block(10, "widget", ""));
        data.push_str(
            "#20 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#1),#0);\n\
             #21 = ( REPRESENTATION_RELATIONSHIP('','',#20,#14) \
             SHAPE_REPRESENTATION_RELATIONSHIP() );\n",
        );
        let (placed, _) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 1);
        assert_eq!(placed[0].product, "widget");
        assert!(placed[0].is_identity());
    }

    #[test]
    fn missing_placement_degrades_to_identity_with_a_warning() {
        let data = two_part_assembly()
            .replace("#54 = CONTEXT_DEPENDENT_SHAPE_REPRESENTATION(#53,#51);", "");
        let (placed, diagnostics) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 1);
        assert!(placed[0].is_identity());
        assert!(
            diagnostics
                .iter()
                .any(|d| d.severity == Severity::Warning && d.message.contains("no CONTEXT")),
            "expected a missing-placement warning, got {diagnostics:?}"
        );
    }

    #[test]
    fn cyclic_assembly_is_cut_not_followed() {
        // Two products that contain each other. Neither is a root, so
        // nothing is walked and the solid falls through to the
        // unreachable-solid path — the point is that it terminates.
        let mut data = String::from("#1 = MANIFOLD_SOLID_BREP('a',#2);\n");
        data.push_str(&product_block(10, "a", "#1"));
        data.push_str(&product_block(20, "b", ""));
        data.push_str(
            "#50 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','b_in_a','',#10,#20,$);\n\
             #51 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','a_in_b','',#20,#10,$);\n",
        );
        let (placed, _) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 1, "the solid is still accounted for");
        assert!(placed[0].is_identity());
    }

    #[test]
    fn self_referential_product_reports_a_cycle() {
        let mut data = String::from("#1 = MANIFOLD_SOLID_BREP('a',#2);\n");
        data.push_str(&product_block(10, "a", "#1"));
        data.push_str(&product_block(20, "b", ""));
        // b contains a, and a contains a: `b` is the only root.
        data.push_str(
            "#50 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('1','a_in_b','',#20,#10,$);\n\
             #51 = NEXT_ASSEMBLY_USAGE_OCCURRENCE('2','a_in_a','',#10,#10,$);\n",
        );
        let (placed, diagnostics) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 1, "the recursive occurrence is dropped");
        assert!(
            diagnostics
                .iter()
                .any(|d| d.severity == Severity::Error && d.message.contains("cyclic")),
            "expected a cycle diagnostic, got {diagnostics:?}"
        );
    }

    #[test]
    fn length_scale_applies_to_placements() {
        let file = parse(&two_part_assembly());
        let mut diagnostics = Vec::new();
        // A file in centimetres: 10 file units are 100 millimetres.
        let placed = resolve_instances(&file, &[1], 10.0, &mut diagnostics);
        let origin = placed[0].transform * Point3::origin();
        assert!(
            (origin - Point3::new(100.0, 0.0, 0.0)).norm() < 1e-12,
            "component landed at {origin:?}"
        );
    }

    #[test]
    fn cartesian_transformation_operator_is_accepted_as_a_placement() {
        let mut data = String::from("#1 = MANIFOLD_SOLID_BREP('mapped',#2);\n");
        data.push_str(&product_block(10, "assembly", "#30"));
        data.push_str(&axis2(40, WORLD, ZP, XP));
        data.push_str(
            "#20 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#1),#0);\n\
             #21 = REPRESENTATION_MAP(#43,#20);\n\
             #22 = CARTESIAN_POINT('',(1.,2.,3.));\n\
             #23 = CARTESIAN_TRANSFORMATION_OPERATOR_3D('',$,$,#22,$,$);\n\
             #30 = MAPPED_ITEM('inst',#21,#23);\n",
        );
        let (placed, diagnostics) = resolve(&data, &[1]);
        assert_eq!(placed.len(), 1, "diagnostics: {diagnostics:?}");
        let origin = placed[0].transform * Point3::origin();
        assert!((origin - Point3::new(1.0, 2.0, 3.0)).norm() < 1e-12);
    }

    #[test]
    fn scaling_transformation_operator_is_rejected_not_silently_dropped() {
        let mut data = String::from("#1 = MANIFOLD_SOLID_BREP('mapped',#2);\n");
        data.push_str(&product_block(10, "assembly", "#30"));
        data.push_str(&axis2(40, WORLD, ZP, XP));
        data.push_str(
            "#20 = ADVANCED_BREP_SHAPE_REPRESENTATION('',(#1),#0);\n\
             #21 = REPRESENTATION_MAP(#43,#20);\n\
             #22 = CARTESIAN_POINT('',(0.,0.,0.));\n\
             #23 = CARTESIAN_TRANSFORMATION_OPERATOR_3D('',$,$,#22,2.,$);\n\
             #30 = MAPPED_ITEM('inst',#21,#23);\n",
        );
        let (placed, diagnostics) = resolve(&data, &[1]);
        assert!(
            diagnostics
                .iter()
                .any(|d| d.message.contains("scale factor")),
            "expected a scale-factor diagnostic, got {diagnostics:?}"
        );
        // The solid is still accounted for, unplaced rather than mis-sized.
        assert_eq!(placed.len(), 1);
        assert!(placed[0].is_identity());
    }

    #[test]
    fn complete_basis_fills_in_derived_axes() {
        let (x, y, z) = complete_basis(None, None, Some(Vector3::y())).expect("basis");
        assert!((x.norm() - 1.0).abs() < 1e-12);
        assert!(x.dot(&y).abs() < 1e-12);
        assert!((x.cross(&y) - z).norm() < 1e-12);
        assert!((z - Vector3::y()).norm() < 1e-12, "declared z survives");
    }

    #[test]
    fn complete_basis_orthogonalizes_a_skew_y() {
        let (x, y, z) = complete_basis(Some(Vector3::x()), Some(Vector3::new(0.5, 1.0, 0.0)), None)
            .expect("basis");
        assert!(x.dot(&y).abs() < 1e-12, "y is Gram-Schmidt'd against x");
        assert!((z - Vector3::z()).norm() < 1e-12);
    }
}
