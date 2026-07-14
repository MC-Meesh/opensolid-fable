//! Assemblies: composing independent parts into a multi-part document.
//!
//! This is Assembly MVP 1 (of-fsl.25.1) per `docs/design/ASSEMBLIES.md`
//! §5 and §9 — the pure-kernel layer, with no GUI, WASM, or mate solver.
//! It supplies exactly the three kernel capabilities the design calls out,
//! each of which reuses machinery that already exists:
//!
//! - **Instancing without geometry duplication.** An [`Instance`] is a
//!   *(part reference, [`Transform3`], fixed flag)* — never a copy of the
//!   geometry. Its field is the part's shape wrapped in the F-Rep
//!   [`Transformed`] combinator, which evaluates `inner(T⁻¹·p)`: the
//!   transform is applied lazily at query time, so ten instances of one
//!   bolt are ten thin wrappers over one shared [`Shape`]
//!   (`Arc<dyn Sdf>`). See [`Instance::field`].
//!
//! - **Interference (clash) detection.** Two instances overlap iff their
//!   solids share an interior point, i.e. `max(sdf_a, sdf_b) < 0` somewhere
//!   — the same `max` that implements CSG intersection. We answer it with
//!   the mesher's own interval machinery ([`Sdf::eval_interval`]): subdivide
//!   only the overlap of the two world bounding boxes, pruning cells where
//!   the intersection field is provably non-negative. The negative region's
//!   volume is the clash volume. See [`Assembly::interference`].
//!
//! - **Mass-property aggregation.** Per-part [`MassProperties`] are composed
//!   by textbook rigid-body arithmetic without re-meshing the union: volume
//!   and area sum, the centroid is the mass-weighted mean of the instances'
//!   world centroids, and the inertia tensor is each part's tensor rotated
//!   into the assembly frame and parallel-axis shifted to the assembly
//!   centroid. See [`Assembly::mass_properties`].

use nalgebra::Matrix3;
use std::sync::Arc;
use thiserror::Error;

use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{BoundingBox3, Point3, Transform3, Vector3};
use opensolid_frep::Shape;
use opensolid_frep::primitives::Sdf;
use opensolid_frep::transform::Transformed;

use crate::massprops::{MassProperties, MassPropertiesError, mass_properties};
use crate::mesh::{MeshOptions, mesh_sdf_indexed};

pub mod mates;
pub mod solver;

pub use mates::{Feature, FeatureRef, Mate, MateError, MateKind};
pub use solver::{
    SolveOptions, SolveResult, SolveStatus, seat_concentric_coincident, solve_mates,
    solve_mates_with,
};

/// Number of interval-subdivision cells along the longest axis of an
/// overlap box when estimating clash volume / testing interference. The
/// interval pruning is exact for the yes/no question at any resolution;
/// this only bounds the accuracy of the reported clash *volume* and the
/// resolution of near-tangent straddle cells.
const CLASH_RESOLUTION: usize = 24;

/// Fraction of a part's longest extent added as clearance on every side
/// when meshing an instance, so the surface stays strictly inside the
/// meshing bounds as [`mesh_sdf_indexed`] requires.
const MESH_MARGIN_FRAC: f64 = 0.1;

/// Errors from assembly aggregation.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum AssemblyError {
    /// A mass-property query on an assembly with no instances.
    #[error("assembly has no instances")]
    Empty,
    /// The instances enclose zero total mass (all zero-volume or
    /// zero-density), so a centroid is undefined.
    #[error("assembly encloses zero mass")]
    ZeroMass,
}

/// A part: a reusable solid plus the cached data an assembly needs to place
/// and reason about it. A part is authored once (its own script/model in
/// the full system) and referenced by any number of [`Instance`]s.
///
/// `bounds` is the part's local-space axis-aligned bounding box — tight
/// around the solid; `mass_properties` are computed once in local space at
/// unit density. Both are transformed per instance, never recomputed.
#[derive(Clone)]
pub struct Part {
    /// The part's signed distance field in its own local frame.
    pub shape: Shape,
    /// Local-space AABB enclosing the solid (tight; clearance is added
    /// where meshing needs it).
    pub bounds: BoundingBox3,
    /// Unit-density mass properties of the solid in local space.
    pub mass_properties: MassProperties,
}

impl Part {
    /// A part from an explicit shape, local bounds, and precomputed mass
    /// properties. Use when the caller already has all three (e.g. a part
    /// whose mass properties were computed elsewhere or are known
    /// analytically).
    pub fn new(shape: Shape, bounds: BoundingBox3, mass_properties: MassProperties) -> Self {
        Self {
            shape,
            bounds,
            mass_properties,
        }
    }

    /// A part from a shape and its local bounds, computing mass properties
    /// by meshing the field once at `resolution`.
    ///
    /// `bounds` must be a tight local AABB around the solid; the surface is
    /// meshed inside a slightly dilated copy so it stays clear of the mesh
    /// boundary.
    ///
    /// # Errors
    /// [`MassPropertiesError`] if the meshed solid is not a closed manifold
    /// enclosing positive volume (e.g. bounds that clip the surface, or a
    /// resolution too coarse to resolve it).
    pub fn from_sdf(
        shape: Shape,
        bounds: BoundingBox3,
        resolution: usize,
    ) -> Result<Self, MassPropertiesError> {
        let mesh = mesh_sdf_indexed(&shape, &mesh_options(&bounds, resolution));
        let mass_properties = mass_properties(&mesh)?;
        Ok(Self::new(shape, bounds, mass_properties))
    }
}

/// A placed occurrence of a [`Part`] in an assembly: a reference to the
/// part plus a pose. Multiple instances of one part share a single
/// `Arc<Part>` — no geometry is copied.
#[derive(Clone)]
pub struct Instance {
    /// The referenced part (shared; cloning an instance is a pointer bump).
    pub part: Arc<Part>,
    /// Placement of the part in assembly space (local → world).
    pub transform: Transform3,
    /// A fixed instance is the assembly's ground: the mate solver (MVP 2)
    /// holds its transform constant. Carried here so the data model is
    /// complete; unused by this MVP's kernel operations.
    pub fixed: bool,
    /// Material density. Mass is `density · volume`; defaults to 1.0 so an
    /// all-unit-density assembly's aggregate matches the unit-density mass
    /// properties of the combined solid.
    pub density: f64,
    /// Human-readable name for tree display and reports.
    pub name: String,
}

impl Instance {
    /// A floating, unit-density, unnamed instance of `part` placed by
    /// `transform`. Chain [`fixed`](Self::fixed), [`density`](Self::density),
    /// and [`named`](Self::named) to set the rest.
    pub fn new(part: Arc<Part>, transform: Transform3) -> Self {
        Self {
            part,
            transform,
            fixed: false,
            density: 1.0,
            name: String::new(),
        }
    }

    /// Set the fixed (ground) flag.
    #[must_use]
    pub fn fixed(mut self, fixed: bool) -> Self {
        self.fixed = fixed;
        self
    }

    /// Set the material density.
    #[must_use]
    pub fn density(mut self, density: f64) -> Self {
        self.density = density;
        self
    }

    /// Set the display name.
    #[must_use]
    pub fn named(mut self, name: impl Into<String>) -> Self {
        self.name = name.into();
        self
    }

    /// The instance's field in assembly space: the part's shape evaluated
    /// through this instance's transform. Zero geometry is copied — the
    /// returned [`Shape`] shares the part's underlying field and applies the
    /// transform at query time.
    pub fn field(&self) -> Shape {
        Shape::new(Transformed::new(self.part.shape.clone(), self.transform))
    }

    /// The instance's world-space AABB: the part's local bounds carried
    /// through the transform (bounding the 8 rotated corners).
    pub fn world_bounds(&self) -> BoundingBox3 {
        let b = &self.part.bounds;
        BoundingBox3::from_points((0..8).map(|i| {
            let corner = Point3::new(
                if i & 1 == 0 { b.min.x } else { b.max.x },
                if i & 2 == 0 { b.min.y } else { b.max.y },
                if i & 4 == 0 { b.min.z } else { b.max.z },
            );
            self.transform * corner
        }))
    }

    /// The instance's world-space centroid (part centroid through the
    /// transform) and mass (`density · volume`).
    fn world_centroid_and_mass(&self) -> (Point3, f64) {
        let mp = &self.part.mass_properties;
        (self.transform * mp.centroid, self.density * mp.volume)
    }
}

/// A report on whether two instances clash and by how much.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct InterferenceReport {
    /// True if the two solids share interior volume.
    pub interferes: bool,
    /// Estimated volume of the overlap region (0 when they do not clash).
    /// Estimated by interval subdivision at [`CLASH_RESOLUTION`]; exact
    /// cells that are provably inside both solids contribute exactly, and
    /// boundary-straddling cells are resolved by a center sample.
    pub volume: f64,
}

/// An assembly: a set of placed [`Instance`]s plus the [`Mate`]s that
/// constrain their poses. Instances are fixed at insertion time until
/// [`solve`](Self::solve) resolves the floating poses from the mates (MVP 2,
/// of-fsl.25.2).
#[derive(Clone, Default)]
pub struct Assembly {
    instances: Vec<Instance>,
    mates: Vec<Mate>,
}

impl Assembly {
    /// An empty assembly.
    pub fn new() -> Self {
        Self::default()
    }

    /// Insert an instance; returns its index (stable for the assembly's
    /// lifetime — instances are never removed by this MVP). Use the index in
    /// the [`FeatureRef`]s of mates that reference this instance.
    pub fn insert(&mut self, instance: Instance) -> usize {
        self.instances.push(instance);
        self.instances.len() - 1
    }

    /// Add a mate constraining two instances' features. Returns its index.
    pub fn add_mate(&mut self, mate: Mate) -> usize {
        self.mates.push(mate);
        self.mates.len() - 1
    }

    /// The mates, in insertion order.
    pub fn mates(&self) -> &[Mate] {
        &self.mates
    }

    /// The instances, in insertion order.
    pub fn instances(&self) -> &[Instance] {
        &self.instances
    }

    /// Number of instances.
    pub fn len(&self) -> usize {
        self.instances.len()
    }

    /// True if the assembly has no instances.
    pub fn is_empty(&self) -> bool {
        self.instances.is_empty()
    }

    /// Solve the mates for the floating instances' poses, returning the
    /// resolved poses and diagnostics *without* mutating the assembly. Fixed
    /// instances are held constant; floating ones move to satisfy the mates.
    /// See [`SolveStatus`] for how conflicting and under-constrained systems
    /// are reported (neither panics). Use [`solve_in_place`](Self::solve_in_place)
    /// to write the result back.
    pub fn solve(&self) -> SolveResult {
        let poses: Vec<Transform3> = self.instances.iter().map(|i| i.transform).collect();
        let fixed: Vec<bool> = self.instances.iter().map(|i| i.fixed).collect();
        solver::solve_mates(&poses, &fixed, &self.mates)
    }

    /// Solve the mates and write the resolved poses back into the instances,
    /// returning the same [`SolveResult`].
    pub fn solve_in_place(&mut self) -> SolveResult {
        let result = self.solve();
        for (inst, &t) in self.instances.iter_mut().zip(&result.transforms) {
            inst.transform = t;
        }
        result
    }

    /// Mesh the whole assembly as the concatenation of per-instance meshes
    /// (design §5, MVP option (a)): each instance's transformed field is
    /// meshed independently over its world bounds and the meshes are
    /// unioned into one vertex/index buffer. Correct and trivial for a
    /// bolted assembly; a welded-body single-field mesh is a follow-up.
    ///
    /// Returns an empty mesh for an empty assembly.
    pub fn mesh(&self, resolution: usize) -> TriangleMesh {
        let mut out = TriangleMesh::new();
        for inst in &self.instances {
            let field = inst.field();
            let opts = mesh_options(&inst.world_bounds(), resolution);
            let part_mesh = mesh_sdf_indexed(&field, &opts);
            let base = out.positions.len();
            out.positions.extend_from_slice(&part_mesh.positions);
            out.normals.extend_from_slice(&part_mesh.normals);
            out.indices.extend(
                part_mesh
                    .indices
                    .iter()
                    .map(|t| [t[0] + base, t[1] + base, t[2] + base]),
            );
        }
        out
    }

    /// Whether instances `i` and `j` clash (share interior volume).
    ///
    /// Faster than [`interference`](Self::interference): the interval search
    /// stops at the first cell proven to lie inside both solids, without
    /// integrating the full overlap volume.
    ///
    /// # Panics
    /// If `i` or `j` is out of range, or `i == j`.
    pub fn interferes(&self, i: usize, j: usize) -> bool {
        assert!(i != j, "an instance cannot interfere with itself");
        let overlap = self.instances[i]
            .world_bounds()
            .intersection(&self.instances[j].world_bounds());
        if overlap.is_empty() {
            return false;
        }
        let a = self.instances[i].field();
        let b = self.instances[j].field();
        clash_exists(&a, &b, &overlap, min_cell(&overlap))
    }

    /// A full [`InterferenceReport`] for instances `i` and `j`, including an
    /// estimate of the clash volume.
    ///
    /// # Panics
    /// If `i` or `j` is out of range, or `i == j`.
    pub fn interference(&self, i: usize, j: usize) -> InterferenceReport {
        assert!(i != j, "an instance cannot interfere with itself");
        let overlap = self.instances[i]
            .world_bounds()
            .intersection(&self.instances[j].world_bounds());
        if overlap.is_empty() {
            return InterferenceReport {
                interferes: false,
                volume: 0.0,
            };
        }
        let a = self.instances[i].field();
        let b = self.instances[j].field();
        let mut volume = 0.0;
        let interferes = clash_integrate(&a, &b, &overlap, min_cell(&overlap), &mut volume);
        InterferenceReport { interferes, volume }
    }

    /// Every clashing pair `(i, j)` with `i < j` and its report. Non-
    /// interfering pairs are omitted.
    pub fn all_interferences(&self) -> Vec<(usize, usize, InterferenceReport)> {
        let mut clashes = Vec::new();
        for i in 0..self.instances.len() {
            for j in (i + 1)..self.instances.len() {
                let report = self.interference(i, j);
                if report.interferes {
                    clashes.push((i, j, report));
                }
            }
        }
        clashes
    }

    /// Aggregate mass properties of the whole assembly, composed from the
    /// per-part cached [`MassProperties`] without re-meshing (design §5):
    ///
    /// - **volume / area**: summed over instances (geometry-invariant under
    ///   rigid motion);
    /// - **mass**: `Σ densityᵢ · volumeᵢ`;
    /// - **centroid**: mass-weighted mean of the instances' world centroids;
    /// - **inertia**: each part's tensor rotated into the assembly frame
    ///   (`R Iₚ Rᵀ`, mass-scaled by density) then parallel-axis shifted to
    ///   the assembly centroid and summed —
    ///   `I = Σ [ Rᵢ (ρᵢ Iₚᵢ) Rᵢᵀ + mᵢ (|dᵢ|²E − dᵢ dᵢᵀ) ]`.
    ///
    /// Overlapping instances double-count in their overlap; for a
    /// well-mated assembly the overlap is the (≈zero) interference volume.
    ///
    /// # Errors
    /// [`AssemblyError::Empty`] if there are no instances;
    /// [`AssemblyError::ZeroMass`] if the total mass is zero.
    pub fn mass_properties(&self) -> Result<AssemblyMassProperties, AssemblyError> {
        if self.instances.is_empty() {
            return Err(AssemblyError::Empty);
        }

        let mut volume = 0.0;
        let mut surface_area = 0.0;
        let mut mass = 0.0;
        let mut first_moment = Vector3::zeros();
        for inst in &self.instances {
            let mp = &inst.part.mass_properties;
            let (centroid, m) = inst.world_centroid_and_mass();
            volume += mp.volume;
            surface_area += mp.surface_area;
            mass += m;
            first_moment += centroid.coords * m;
        }

        if mass == 0.0 {
            return Err(AssemblyError::ZeroMass);
        }
        let centroid = Point3::from(first_moment / mass);

        // Inertia about the assembly centroid: rotate each part tensor into
        // the assembly frame, mass-scale by density, and parallel-axis shift.
        let mut inertia = Matrix3::zeros();
        for inst in &self.instances {
            let mp = &inst.part.mass_properties;
            let (world_centroid, m) = inst.world_centroid_and_mass();
            let rot: Matrix3<f64> = inst.transform.rotation.to_rotation_matrix().into_inner();
            // ρ scales the unit-density tensor to physical inertia; R rotates
            // it into the assembly frame.
            let rotated = rot * (mp.inertia * inst.density) * rot.transpose();
            let d = world_centroid - centroid;
            let shift = Matrix3::identity() * d.norm_squared() - d * d.transpose();
            inertia += rotated + shift * m;
        }

        Ok(AssemblyMassProperties {
            volume,
            surface_area,
            mass,
            centroid,
            inertia,
        })
    }
}

/// Aggregate mass properties of an assembly, in the assembly frame.
///
/// Unlike the per-solid [`MassProperties`] (always unit density), this
/// carries an explicit `mass` because instances may have differing
/// densities; `inertia` is the physical (mass-weighted) tensor about the
/// assembly [`centroid`](Self::centroid).
#[derive(Debug, Clone, PartialEq)]
pub struct AssemblyMassProperties {
    /// Total enclosed volume, summed over instances.
    pub volume: f64,
    /// Total surface area, summed over instances.
    pub surface_area: f64,
    /// Total mass, `Σ densityᵢ · volumeᵢ`.
    pub mass: f64,
    /// Center of mass of the assembly.
    pub centroid: Point3,
    /// Inertia tensor about the assembly centroid, in the assembly frame.
    /// Symmetric; physical (mass-weighted) units.
    pub inertia: Matrix3<f64>,
}

/// Meshing options for a solid whose surface lies within `bounds`: dilate
/// by a fraction of the longest extent so the surface clears the boundary,
/// as [`mesh_sdf_indexed`] requires.
fn mesh_options(bounds: &BoundingBox3, resolution: usize) -> MeshOptions {
    let e = bounds.extents();
    let margin = e.x.max(e.y).max(e.z) * MESH_MARGIN_FRAC;
    MeshOptions {
        bounds: bounds.dilate(margin),
        resolution,
    }
}

/// Cell size at which interval subdivision stops for `overlap`.
fn min_cell(overlap: &BoundingBox3) -> f64 {
    let e = overlap.extents();
    e.x.max(e.y).max(e.z) / CLASH_RESOLUTION as f64
}

/// Volume of a (non-empty) box.
fn box_volume(b: &BoundingBox3) -> f64 {
    let e = b.extents();
    e.x * e.y * e.z
}

/// Split a box into its 8 octants about its center.
fn octants(b: &BoundingBox3) -> [BoundingBox3; 8] {
    let c = b.center();
    std::array::from_fn(|i| {
        let (xlo, xhi) = if i & 1 == 0 {
            (b.min.x, c.x)
        } else {
            (c.x, b.max.x)
        };
        let (ylo, yhi) = if i & 2 == 0 {
            (b.min.y, c.y)
        } else {
            (c.y, b.max.y)
        };
        let (zlo, zhi) = if i & 4 == 0 {
            (b.min.z, c.z)
        } else {
            (c.z, b.max.z)
        };
        BoundingBox3::new(Point3::new(xlo, ylo, zlo), Point3::new(xhi, yhi, zhi))
    })
}

/// The intersection field's interval over `b`: `max(a, b)` propagates as the
/// pointwise-max interval. `lo ≥ 0` proves no point of `b` lies inside both
/// solids (at least one field is non-negative throughout); `hi < 0` proves
/// every point does.
fn clash_interval(a: &dyn Sdf, b: &dyn Sdf, box_: &BoundingBox3) -> (f64, f64) {
    let ia = a.eval_interval(box_);
    let ib = b.eval_interval(box_);
    (ia.lo.max(ib.lo), ia.hi.max(ib.hi))
}

/// Does `max(a, b) < 0` anywhere in `box_`? Early-out interval search.
fn clash_exists(a: &dyn Sdf, b: &dyn Sdf, box_: &BoundingBox3, min_cell: f64) -> bool {
    let (lo, hi) = clash_interval(a, b, box_);
    if lo >= 0.0 {
        return false;
    }
    if hi < 0.0 {
        return true;
    }
    let e = box_.extents();
    if e.x.max(e.y).max(e.z) <= min_cell {
        let c = box_.center();
        return a.eval(&c).max(b.eval(&c)) < 0.0;
    }
    octants(box_)
        .iter()
        .any(|oct| clash_exists(a, b, oct, min_cell))
}

/// Integrate the clash volume (`max(a, b) < 0`) over `box_`, accumulating
/// into `acc`; returns whether any clash was found. Cells proven inside
/// both solids contribute exactly; straddle cells at the minimum size are
/// resolved by a center sample.
fn clash_integrate(
    a: &dyn Sdf,
    b: &dyn Sdf,
    box_: &BoundingBox3,
    min_cell: f64,
    acc: &mut f64,
) -> bool {
    let (lo, hi) = clash_interval(a, b, box_);
    if lo >= 0.0 {
        return false;
    }
    if hi < 0.0 {
        *acc += box_volume(box_);
        return true;
    }
    let e = box_.extents();
    if e.x.max(e.y).max(e.z) <= min_cell {
        let c = box_.center();
        if a.eval(&c).max(b.eval(&c)) < 0.0 {
            *acc += box_volume(box_);
            return true;
        }
        return false;
    }
    let mut any = false;
    for oct in octants(box_) {
        // Evaluate every octant (don't short-circuit): the volume needs all.
        if clash_integrate(a, b, &oct, min_cell, acc) {
            any = true;
        }
    }
    any
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::Vector3 as V3;
    use opensolid_frep::primitives::{Box3, Sphere};
    use std::f64::consts::{FRAC_PI_2, PI};

    /// Analytic mass properties of an axis-aligned box centered at the origin
    /// with the given half-extents, unit density.
    fn box_mass_props(hx: f64, hy: f64, hz: f64) -> MassProperties {
        let (a, b, c) = (2.0 * hx, 2.0 * hy, 2.0 * hz);
        let volume = a * b * c;
        MassProperties {
            volume,
            surface_area: 2.0 * (a * b + b * c + c * a),
            centroid: Point3::origin(),
            inertia: Matrix3::from_diagonal(&V3::new(
                volume / 12.0 * (b * b + c * c),
                volume / 12.0 * (a * a + c * c),
                volume / 12.0 * (a * a + b * b),
            )),
        }
    }

    /// A box part with analytic mass properties (isolates aggregation math
    /// from meshing error).
    fn box_part(hx: f64, hy: f64, hz: f64) -> Arc<Part> {
        let shape = Shape::new(Box3 {
            center: Point3::origin(),
            half_extents: [hx, hy, hz],
        });
        let bounds = BoundingBox3::new(Point3::new(-hx, -hy, -hz), Point3::new(hx, hy, hz));
        Arc::new(Part::new(shape, bounds, box_mass_props(hx, hy, hz)))
    }

    fn unit_cube_part() -> Arc<Part> {
        box_part(0.5, 0.5, 0.5)
    }

    fn sphere_part(radius: f64) -> Arc<Part> {
        let shape = Shape::new(Sphere {
            center: Point3::origin(),
            radius,
        });
        let m = Vector3::new(radius, radius, radius);
        let bounds = BoundingBox3::new(Point3::origin() - m, Point3::origin() + m);
        Arc::new(Part::from_sdf(shape, bounds, 40).expect("sphere meshes closed"))
    }

    fn diag(m: &Matrix3<f64>) -> [f64; 3] {
        [m[(0, 0)], m[(1, 1)], m[(2, 2)]]
    }

    fn assert_products_zero(m: &Matrix3<f64>, tol: f64) {
        for i in 0..3 {
            for j in 0..3 {
                if i != j {
                    assert!(m[(i, j)].abs() < tol, "product I[{i}][{j}] = {}", m[(i, j)]);
                }
            }
        }
    }

    // --- instancing ---

    #[test]
    fn instances_share_one_part_no_geometry_copy() {
        let bolt = unit_cube_part();
        let mut asm = Assembly::new();
        let i0 = asm.insert(Instance::new(
            bolt.clone(),
            Transform3::translation(0.0, 0.0, 0.0),
        ));
        let i1 = asm.insert(Instance::new(
            bolt.clone(),
            Transform3::translation(5.0, 0.0, 0.0),
        ));
        // Both instances reference the very same Part allocation.
        assert!(Arc::ptr_eq(
            &asm.instances()[i0].part,
            &asm.instances()[i1].part
        ));
        // And the shared shape's underlying SDF is one instance.
        assert!(
            asm.instances()[i0]
                .part
                .shape
                .ptr_eq(&asm.instances()[i1].part.shape)
        );
    }

    #[test]
    fn instance_field_evaluates_part_through_transform() {
        let part = sphere_part(1.0);
        let inst = Instance::new(part, Transform3::translation(3.0, 0.0, 0.0));
        let field = inst.field();
        // At the placed center the field is -radius; on the placed surface, 0.
        assert!((field.eval(&Point3::new(3.0, 0.0, 0.0)) + 1.0).abs() < 1e-9);
        assert!(field.eval(&Point3::new(4.0, 0.0, 0.0)).abs() < 1e-9);
        // The untranslated origin is now outside.
        assert!((field.eval(&Point3::origin()) - 2.0).abs() < 1e-9);
    }

    #[test]
    fn world_bounds_track_the_transform() {
        let part = box_part(1.0, 1.0, 1.0); // [-1,1]^3
        // 90° about z then translate: extents unchanged (cube), center moves.
        let t = Transform3::translation(10.0, 0.0, 0.0) * Transform3::rotation(V3::z() * FRAC_PI_2);
        let inst = Instance::new(part, t);
        let wb = inst.world_bounds();
        assert!((wb.center() - Point3::new(10.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((wb.extents() - Vector3::new(2.0, 2.0, 2.0)).norm() < 1e-9);
    }

    // --- interference ---

    #[test]
    fn overlapping_spheres_clash_with_lens_volume() {
        // Two unit spheres, centers distance d = 1 apart. Analytic lens
        // volume = π(4r+d)(2r−d)²/12 = 5π/12 for r = 1, d = 1.
        let part = sphere_part(1.0);
        let mut asm = Assembly::new();
        asm.insert(Instance::new(
            part.clone(),
            Transform3::translation(-0.5, 0.0, 0.0),
        ));
        asm.insert(Instance::new(part, Transform3::translation(0.5, 0.0, 0.0)));

        assert!(asm.interferes(0, 1));
        let report = asm.interference(0, 1);
        assert!(report.interferes);
        let expected = 5.0 * PI / 12.0;
        let rel = (report.volume - expected).abs() / expected;
        assert!(
            rel < 0.05,
            "clash volume {} vs {expected} (rel {rel})",
            report.volume
        );
    }

    #[test]
    fn disjoint_spheres_do_not_clash() {
        let part = sphere_part(1.0);
        let mut asm = Assembly::new();
        asm.insert(Instance::new(
            part.clone(),
            Transform3::translation(-2.0, 0.0, 0.0),
        ));
        asm.insert(Instance::new(part, Transform3::translation(2.0, 0.0, 0.0)));

        assert!(!asm.interferes(0, 1));
        let report = asm.interference(0, 1);
        assert!(!report.interferes);
        assert_eq!(report.volume, 0.0);
    }

    #[test]
    fn all_interferences_lists_only_clashing_pairs() {
        let part = sphere_part(1.0);
        let mut asm = Assembly::new();
        asm.insert(Instance::new(
            part.clone(),
            Transform3::translation(0.0, 0.0, 0.0),
        ));
        asm.insert(Instance::new(
            part.clone(),
            Transform3::translation(0.5, 0.0, 0.0),
        )); // clashes with 0
        asm.insert(Instance::new(part, Transform3::translation(10.0, 0.0, 0.0))); // far away

        let clashes = asm.all_interferences();
        assert_eq!(clashes.len(), 1);
        assert_eq!((clashes[0].0, clashes[0].1), (0, 1));
        assert!(clashes[0].2.interferes && clashes[0].2.volume > 0.0);
    }

    #[test]
    #[should_panic(expected = "cannot interfere with itself")]
    fn self_interference_panics() {
        let mut asm = Assembly::new();
        asm.insert(Instance::new(unit_cube_part(), Transform3::identity()));
        asm.interferes(0, 0);
    }

    // --- mass-property aggregation ---

    #[test]
    fn single_instance_matches_transformed_part() {
        let part = box_part(0.5, 1.0, 1.5); // volume 6
        let mut asm = Assembly::new();
        asm.insert(Instance::new(part, Transform3::translation(4.0, -2.0, 1.0)));
        let mp = asm.mass_properties().unwrap();
        assert!((mp.volume - 6.0).abs() < 1e-12);
        assert!((mp.mass - 6.0).abs() < 1e-12);
        // Centroid rides along with the translation.
        assert!((mp.centroid - Point3::new(4.0, -2.0, 1.0)).norm() < 1e-12);
        // Inertia about the (moved) centroid is translation-invariant.
        let want = diag(&box_mass_props(0.5, 1.0, 1.5).inertia);
        for (got, want) in diag(&mp.inertia).iter().zip(want) {
            assert!((got - want).abs() < 1e-9, "{got} vs {want}");
        }
        assert_products_zero(&mp.inertia, 1e-9);
    }

    #[test]
    fn two_unit_cubes_aggregate_exactly() {
        let cube = unit_cube_part();
        let mut asm = Assembly::new();
        asm.insert(Instance::new(
            cube.clone(),
            Transform3::translation(0.0, 0.0, 0.0),
        ));
        asm.insert(Instance::new(cube, Transform3::translation(2.0, 0.0, 0.0)));
        let mp = asm.mass_properties().unwrap();

        assert!((mp.volume - 2.0).abs() < 1e-12);
        assert!((mp.surface_area - 12.0).abs() < 1e-12);
        assert!((mp.mass - 2.0).abs() < 1e-12);
        assert!((mp.centroid - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
        // Each cube: own inertia 1/6 + parallel-axis shift diag(0,1,1) about
        // the assembly centroid one unit away on x. Two cubes double it.
        let want = [1.0 / 3.0, 7.0 / 3.0, 7.0 / 3.0];
        for (got, want) in diag(&mp.inertia).iter().zip(want) {
            assert!((got - want).abs() < 1e-12, "{got} vs {want}");
        }
        assert_products_zero(&mp.inertia, 1e-12);
    }

    #[test]
    fn density_weights_the_centroid_and_mass() {
        let cube = unit_cube_part(); // volume 1
        let mut asm = Assembly::new();
        asm.insert(
            Instance::new(cube.clone(), Transform3::translation(0.0, 0.0, 0.0)).density(1.0),
        );
        asm.insert(Instance::new(cube, Transform3::translation(2.0, 0.0, 0.0)).density(3.0));
        let mp = asm.mass_properties().unwrap();

        assert!((mp.volume - 2.0).abs() < 1e-12); // volume ignores density
        assert!((mp.mass - 4.0).abs() < 1e-12); // 1·1 + 3·1
        // Centroid pulled toward the heavy instance: (1·0 + 3·2)/4 = 1.5.
        assert!((mp.centroid - Point3::new(1.5, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn rotating_an_instance_rotates_its_inertia() {
        // Box 1×2×3 (half-extents 0.5,1,1.5), rotated 90° about z: the x and
        // y principal moments swap.
        let part = box_part(0.5, 1.0, 1.5);
        let base = box_mass_props(0.5, 1.0, 1.5).inertia;
        let [ixx, iyy, izz] = diag(&base);

        let mut asm = Assembly::new();
        asm.insert(Instance::new(
            part,
            Transform3::rotation(V3::z() * FRAC_PI_2),
        ));
        let mp = asm.mass_properties().unwrap();

        // Centroid unmoved (rotation about the origin, box centered there).
        assert!(mp.centroid.coords.norm() < 1e-12);
        let [gxx, gyy, gzz] = diag(&mp.inertia);
        assert!(
            (gxx - iyy).abs() < 1e-9,
            "Ixx {gxx} should be old Iyy {iyy}"
        );
        assert!(
            (gyy - ixx).abs() < 1e-9,
            "Iyy {gyy} should be old Ixx {ixx}"
        );
        assert!((gzz - izz).abs() < 1e-9, "Izz {gzz} unchanged");
        assert_products_zero(&mp.inertia, 1e-9);
    }

    #[test]
    fn mass_properties_reject_empty_and_massless() {
        let empty = Assembly::new();
        assert_eq!(empty.mass_properties(), Err(AssemblyError::Empty));

        let mut asm = Assembly::new();
        asm.insert(Instance::new(unit_cube_part(), Transform3::identity()).density(0.0));
        assert_eq!(asm.mass_properties(), Err(AssemblyError::ZeroMass));
    }

    // --- meshing ---

    #[test]
    fn mesh_concatenates_per_instance_meshes() {
        let part = sphere_part(1.0);
        let single = mesh_sdf_indexed(
            &Instance::new(part.clone(), Transform3::translation(-2.0, 0.0, 0.0)).field(),
            &mesh_options(
                &Instance::new(part.clone(), Transform3::translation(-2.0, 0.0, 0.0))
                    .world_bounds(),
                16,
            ),
        );

        let mut asm = Assembly::new();
        asm.insert(Instance::new(
            part.clone(),
            Transform3::translation(-2.0, 0.0, 0.0),
        ));
        asm.insert(Instance::new(part, Transform3::translation(2.0, 0.0, 0.0)));
        let mesh = asm.mesh(16);

        // Two identical instances → twice the single-instance triangle count.
        assert_eq!(mesh.triangle_count(), 2 * single.triangle_count());
        // The combined bounds span both spheres.
        let bb = mesh.bounding_box().expect("non-empty");
        assert!(bb.min.x < -2.5 && bb.max.x > 2.5, "bounds {bb:?}");
    }

    #[test]
    fn empty_assembly_meshes_empty() {
        assert!(Assembly::new().mesh(16).is_empty());
    }
}
