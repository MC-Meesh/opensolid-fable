//! Geometry healing for imported B-Rep bodies (`spec/06-step-io.md` §6).
//!
//! Real-world STEP is messy. A file from CATIA, SolidWorks or Creo routinely
//! arrives with each face carrying its *own* `VERTEX_POINT`s and
//! `EDGE_CURVE`s for boundaries it shares with its neighbours (an "unsewn"
//! shell), with those duplicated points disagreeing in the last few decimals
//! (a "gap"), or with a handful of faces whose authored orientation runs
//! against the rest of the shell. None of that is a parsing problem — the
//! [reader](super::read) maps every entity happily — but the resulting body
//! fails [`TopologyStore::check`] and the import degrades to the mesh
//! fallback, losing exactness for a defect measured in nanometres.
//!
//! The healer repairs those bodies in place. It runs *after* the exact
//! mapping pass and *before* the mesh fallback: a body that heals clean is
//! promoted back to [`SolidOutcome::BRep`](super::read::SolidOutcome::BRep),
//! and one that does not still falls back exactly as before. Import heals; it
//! never rejects.
//!
//! # Phase 1 passes
//!
//! - **Gap closure** ([`GeometryHealer::fix_gaps`]) — cluster vertices that
//!   lie within `max_gap` of each other into one, then weld the duplicate
//!   edges that clustering exposes (edges that are still open on one side,
//!   now span the same vertex pair, *and* whose curves sample to the same
//!   points). Welding re-points the orphaned fins onto the surviving edge, so
//!   a shell whose faces never shared a boundary becomes watertight. Two
//!   coincident edges that are both already two-sided are left alone: they
//!   are two sheets touching, and joining them would make a non-manifold edge
//!   rather than repair one. The distance closed is absorbed
//!   into the surviving entity's tolerance — this is tolerant modelling
//!   (`spec/08-tolerances.md`), not a silent snap. It is absorbed *on top of*
//!   what the merged-away entities were already carrying, not merely up to
//!   it: a survivor stands in for its members against everything they were
//!   attached to, so the two distances add (of-bbh8).
//! - **Orientation repair** ([`GeometryHealer::fix_orientation`]) — two-colour
//!   the face adjacency graph so every pair of mated fins traverses its edge
//!   in opposite directions, flipping the minority side of each connected
//!   component. A shell that comes out consistently oriented but *inside out*
//!   (its enclosed signed volume has the wrong sign for its
//!   [`ShellOrientation`]) is then reversed wholesale.
//! - **Sense reconciliation** ([`reconcile_face_senses`]) — set each face's
//!   [`FaceSense`] to whatever the winding of its own outer loop says it must
//!   be. This one is *not* reachable from [`GeometryHealer::heal`], because it
//!   reads a winding and pcurves do not exist yet at that point; the reader
//!   calls it directly as the last step of an exact import. It is also the
//!   only pass that catches a face whose sense flag alone is wrong, which the
//!   two-colouring above is structurally blind to — see its own docs, and
//!   of-hrgt for what that blindness cost.
//!
//! # Phase 2 passes (`of-3qy.14`)
//!
//! - **Sliver collapse** ([`GeometryHealer::fix_degenerate_edges`]) — an
//!   edge whose whole curve fits inside the merge tolerance is not an edge,
//!   it is a corner the exporter wrote twice with a thread between. Phase 1
//!   deliberately refuses to merge its vertices (that would delete the edge
//!   behind the topology's back and break the Euler counts); this pass
//!   deletes it *through* the topology instead, with the KEV Euler operator
//!   ([`TopologyStore::kev`]), so `V` and `E` move together and the formula
//!   holds. The surviving vertex lands at the midpoint and absorbs, per
//!   member, the distance moved *plus* what that member already carried —
//!   the same accounting as a vertex merge (of-bbh8). Runs before gap
//!   closure, because collapsing a sliver is what makes its neighbourhood
//!   mergeable at all.
//! - **Edge/surface consistency**
//!   ([`GeometryHealer::fix_edge_surface_consistency`]) — an edge whose
//!   curve strays from an adjacent face's surface further than its
//!   tolerance claims is lying about where it is. The minimal repair is to
//!   stop lying: raise the tolerance to the measured distance (tolerant
//!   modelling, as everywhere else here). Past
//!   [`MAX_ALLOWED_TOLERANCE`] no honest tolerance exists, and the strong
//!   repair takes over: **edge-curve recomputation**, replacing the curve
//!   with the actual intersection of the two adjacent faces' surfaces
//!   ([`opensolid_brep::intersect`]), re-trimmed to the edge's vertices.
//!   The recomputed branch must hug the authored curve; a candidate that
//!   only shares its endpoints is refused, because picking it would be
//!   guessing. From [`GeometryHealer::heal`] only the strong repair runs
//!   (the reader's own `record_edge_tolerances` already absorbs the
//!   sub-cap band, and pre-empting it would repair bodies that were never
//!   going to fail); the standalone pass does both.
//! - **Pcurve recompute** ([`GeometryHealer::fix_pcurves`]) — a fin whose
//!   pcurve no longer tracks its edge's curve (`surface.point(pcurve(t))`
//!   departs from `curve.point(t)`, see [`opensolid_brep::fit_pcurve`]) is
//!   refit from the curve and surface as they now are. During import this
//!   has nothing to do — the reader derives every pcurve *after* healing
//!   settles which edge each fin uses — but a healed body re-healed later
//!   (or one whose edge curves the consistency pass just replaced) has
//!   pcurves that predate the repair, and this is what brings them back
//!   into lockstep. Fins with *no* pcurve are left alone: deriving trim
//!   geometry from scratch is [`opensolid_brep::attach_body_pcurves`]'s
//!   job, and a missing pcurve is honest where a stale one lies.
//!
//! A repair that rewires fins (orientation flips, sewing) leaves their
//! pcurves as mapped: a fin's pcurve depends on its edge's curve and its
//! face's surface, neither of which those repairs touch. The one repair
//! that *does* touch them — edge-curve recomputation — refits the affected
//! fins' pcurves itself, and drops any it cannot refit.
//!
//! # Where healing does *not* reach
//!
//! Healing operates on a *built* body, so it only helps solids the reader
//! mapped successfully and the checker then rejected. A solid that lands in
//! [`SolidOutcome::Failed`](super::read::SolidOutcome::Failed) because the
//! mapping itself failed — a dangling `#id`, an attribute of the wrong type,
//! a `CLOSED_SHELL` with no faces — never produces a body to repair, and no
//! amount of tolerance would make one. Those stay the reader's problem.
//!
//! # What healing will not do
//!
//! Healing never widens a tolerance past
//! [`MAX_ALLOWED_TOLERANCE`], never merges two vertices joined by an edge the
//! merge would not survive (a sliver is *collapsed* through the Euler
//! operator instead; anything longer is left alone), and never merges across
//! two shells of one body (that would make the shells non-manifold against
//! each other). It never replaces an edge curve with an intersection branch
//! it cannot match to the authored curve. Each refusal is recorded in
//! [`HealResult::notes`] rather than applied optimistically.
//!
//! # Example
//!
//! ```
//! use opensolid_brep::{GeometryStore, TopologyStore};
//! use opensolid_kernel::io::step::heal::{GeometryHealer, HealOptions};
//! # use opensolid_brep::{BodyType, FaceSense, FinSense, LoopType, ShellOrientation,
//! #     SYSTEM_RESOLUTION};
//! # use opensolid_core::Point3;
//! let mut store = TopologyStore::new();
//! let mut geo = GeometryStore::new();
//! # let body = store.create_body(BodyType::Solid);
//! # let shell = store.create_shell(body, true, ShellOrientation::Outward);
//! # let face = store.create_face(shell, FaceSense::Positive);
//! # let a = store.create_vertex(Point3::new(0.0, 0.0, 0.0), SYSTEM_RESOLUTION);
//! # let b = store.create_vertex(Point3::new(1.0, 0.0, 0.0), SYSTEM_RESOLUTION);
//! # let c = store.create_vertex(Point3::new(0.0, 1.0, 0.0), SYSTEM_RESOLUTION);
//! # let e = [
//! #     store.create_edge(a, b, SYSTEM_RESOLUTION),
//! #     store.create_edge(b, c, SYSTEM_RESOLUTION),
//! #     store.create_edge(c, a, SYSTEM_RESOLUTION),
//! # ];
//! # store.create_loop(face, LoopType::Outer, &e.map(|id| (id, FinSense::Forward)));
//! // `body` is some imported solid that failed `store.check(body)`.
//! let result = GeometryHealer::heal(body, &mut store, &mut geo, &HealOptions::default());
//! for op in &result.operations {
//!     println!("healed: {op}");
//! }
//! ```

use std::collections::HashMap;
use std::fmt;

use opensolid_brep::{
    Body, CheckFailure, Curve2, Curve2Eval, Curve3, CurveEval, CurveProject, Edge, EntityRef, Face,
    FaceSense, Fin, FinSense, GeometryStore, MAX_ALLOWED_TOLERANCE, SYSTEM_RESOLUTION, SeamSide,
    Shell, ShellOrientation, Surface3, SurfaceEval, SurfaceIntersection, SurfaceProject,
    TessellationOptions, TopologyStore, Vertex, fit_pcurve, intersect as intersect_surfaces,
    tessellate_face,
};
use opensolid_core::{EntityId, Point3, ToleranceContext};

/// Default vertex-merge tolerance as a fraction of the body's bounding-box
/// diagonal, used when [`HealOptions::max_gap`] is `None`.
///
/// STEP writes coordinates as finite decimals, so a shared corner written
/// twice disagrees at roughly the field's last digit — relative, not
/// absolute. `1e-5` of the diagonal is a micron on a 100 mm part: far below
/// any real feature, far above export round-off. (Compare the reader's
/// `TRIM_TOL_REL`, which is the same idea one decade tighter for a check
/// that must not merge anything.)
pub const HEAL_GAP_REL: f64 = 1e-5;

/// Parameter fractions at which two edges' curves are compared for welding.
/// Interior only: the endpoints are already known to agree (that is what put
/// the edges in the same candidate group).
const SAMPLE_FRACTIONS: [f64; 3] = [0.25, 0.5, 0.75];

/// How aggressively [`read_step`](super::read::read_step) repairs the bodies
/// it maps (`spec/06-step-io.md` §4).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum HealStrategy {
    /// Every pass: sliver collapse and gap closure, then edge-curve
    /// recomputation for edges past the kernel tolerance cap, then
    /// orientation repair. The default — import always heals, the question
    /// is only how far.
    #[default]
    Auto,
    /// Sliver collapse and gap closure only. Leaves authored face
    /// orientation and edge geometry untouched, for files whose sense flags
    /// and curves are trusted.
    Minimal,
    /// Plan every pass and report what it *would* do, without touching the
    /// body. The import still degrades to the mesh fallback; the reported
    /// operations say what an `Auto` run would have fixed.
    ReportOnly,
    /// Heal nothing. Not part of the spec's strategy set — it exists so a
    /// caller (or a regression test) can see the unhealed outcome.
    Off,
}

impl HealStrategy {
    /// Whether this strategy mutates the body at all.
    fn applies(self) -> bool {
        matches!(self, HealStrategy::Auto | HealStrategy::Minimal)
    }

    /// Whether this strategy includes the orientation-repair pass.
    fn orients(self) -> bool {
        matches!(self, HealStrategy::Auto | HealStrategy::ReportOnly)
    }

    /// Whether this strategy includes the gap-closure passes (sliver
    /// collapse and vertex/edge merging).
    fn closes_gaps(self) -> bool {
        !matches!(self, HealStrategy::Off)
    }

    /// Whether this strategy includes the geometry-consistency passes
    /// (edge-curve recomputation and pcurve refits). `Minimal` trusts the
    /// authored geometry, so only `Auto` runs them (and `ReportOnly` plans
    /// them).
    fn fixes_geometry(self) -> bool {
        matches!(self, HealStrategy::Auto | HealStrategy::ReportOnly)
    }
}

/// Tuning for [`GeometryHealer::heal`].
#[derive(Debug, Clone, PartialEq, Default)]
pub struct HealOptions {
    pub strategy: HealStrategy,
    /// Largest vertex-to-vertex distance that may be closed by merging, in
    /// millimetres (imported geometry is always millimetres — see
    /// [`StepImport::length_scale`](super::read::StepImport::length_scale)).
    ///
    /// `None` derives it per body as [`HEAL_GAP_REL`] × the bounding-box
    /// diagonal, which is what you want unless you know the producer's
    /// tolerance. Values are clamped to [`MAX_ALLOWED_TOLERANCE`]: healing may
    /// not create a body the kernel would reject for tolerance alone.
    pub max_gap: Option<f64>,
    /// Floor for the gap-closure tolerance (vertex merging and edge
    /// welding), in millimetres. The STEP reader sets it to the solid's
    /// declared closure (`GLOBAL_UNCERTAINTY_ASSIGNED_CONTEXT`) — the file's
    /// own statement of how far apart entities at asserted connectivities
    /// may sit — so a gap the file vouches for is closed even when the
    /// derived round-off gap would refuse it (of-00pu).
    ///
    /// It deliberately does **not** floor sliver collapse
    /// ([`GeometryHealer::fix_degenerate_edges`]): the declaration states
    /// connectivity slop between *distinct* entities, not minimum feature
    /// size, and collapsing every edge shorter than it would destroy real
    /// sub-closure features (the of-5rnp lesson). Clamped to
    /// [`MAX_ALLOWED_TOLERANCE`] like `max_gap`; `0.0` (the default) is a
    /// no-op.
    pub gap_floor: f64,
}

/// One repair the healer applied (or, under
/// [`HealStrategy::ReportOnly`], would have applied).
#[derive(Debug, Clone, PartialEq)]
pub enum HealOperation {
    /// Coincident vertices collapsed into one at their centroid.
    VerticesMerged {
        kept: EntityId<Vertex>,
        merged: usize,
        /// Largest distance any merged point moved.
        gap: f64,
    },
    /// Duplicate edges over one vertex pair collapsed into one, re-pointing
    /// the orphaned fins — the step that makes an unsewn shell watertight.
    EdgesWelded {
        kept: EntityId<Edge>,
        merged: usize,
        /// Largest distance between the welded curves' samples.
        gap: f64,
    },
    /// An entity's tolerance was raised to cover the distance a repair closed.
    ToleranceElevated {
        entity: EntityRef,
        new_tolerance: f64,
    },
    /// A face was reversed (surface sense and every loop's traversal) to agree
    /// with its neighbours.
    FaceReoriented { face: EntityId<Face> },
    /// A face's surface sense alone was corrected to agree with the winding of
    /// its own outer loop — the loops were left exactly as authored.
    FaceSenseCorrected {
        face: EntityId<Face>,
        /// The sense the face now carries.
        sense: FaceSense,
    },
    /// A whole shell was reversed: consistently oriented, but enclosing a
    /// signed volume of the wrong sign for its [`ShellOrientation`].
    ShellReversed {
        shell: EntityId<Shell>,
        faces: usize,
    },
    /// A degenerate (sliver) edge was contracted to a single vertex through
    /// the KEV Euler operator ([`TopologyStore::kev`]).
    EdgeCollapsed {
        edge: EntityId<Edge>,
        kept: EntityId<Vertex>,
        /// Distance between the collapsed edge's end vertices.
        length: f64,
    },
    /// An edge's curve strayed from its adjacent faces' surfaces beyond
    /// repair by tolerance, and was replaced with the surfaces' actual
    /// intersection curve.
    EdgeCurveRecomputed {
        edge: EntityId<Edge>,
        /// Largest curve-to-surface distance before the repair.
        deviation_before: f64,
        /// The same measurement over the replacement curve.
        deviation_after: f64,
    },
    /// A fin's pcurve was refit against its edge's curve and its face's
    /// surface, restoring the lockstep invariant (`opensolid_brep::pcurve`).
    PcurveRecomputed {
        fin: EntityId<Fin>,
        edge: EntityId<Edge>,
    },
}

impl fmt::Display for HealOperation {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            HealOperation::VerticesMerged { kept, merged, gap } => write!(
                f,
                "merged {merged} coincident vertices into {kept:?} (gap {gap:.3e} mm)"
            ),
            HealOperation::EdgesWelded { kept, merged, gap } => write!(
                f,
                "welded {merged} duplicate edges onto {kept:?} (deviation {gap:.3e} mm)"
            ),
            HealOperation::ToleranceElevated {
                entity,
                new_tolerance,
            } => write!(f, "raised {entity:?} tolerance to {new_tolerance:.3e} mm"),
            HealOperation::FaceReoriented { face } => {
                write!(f, "reversed {face:?} to match its neighbours")
            }
            HealOperation::FaceSenseCorrected { face, sense } => write!(
                f,
                "corrected {face:?} surface sense to {sense:?} to match its outer loop's winding"
            ),
            HealOperation::ShellReversed { shell, faces } => write!(
                f,
                "reversed {shell:?} ({faces} faces): enclosed volume had the wrong sign"
            ),
            HealOperation::EdgeCollapsed { edge, kept, length } => write!(
                f,
                "collapsed sliver edge {edge:?} ({length:.3e} mm) into vertex {kept:?}"
            ),
            HealOperation::EdgeCurveRecomputed {
                edge,
                deviation_before,
                deviation_after,
            } => write!(
                f,
                "recomputed {edge:?} curve from its faces' intersection \
                 (off-surface {deviation_before:.3e} mm -> {deviation_after:.3e} mm)"
            ),
            HealOperation::PcurveRecomputed { fin, edge } => {
                write!(f, "refit pcurve of {fin:?} against {edge:?}")
            }
        }
    }
}

/// What one healing run did, and what it could not fix.
#[derive(Debug, Default)]
pub struct HealResult {
    /// Repairs applied, in the order they were applied.
    pub operations: Vec<HealOperation>,
    /// [`TopologyStore::check`] failures the body had on entry.
    pub failures_before: Vec<CheckFailure>,
    /// Failures still present on exit. Under [`HealStrategy::ReportOnly`] this
    /// is `failures_before` unchanged — nothing was applied, so nothing was
    /// re-checked.
    pub remaining: Vec<CheckFailure>,
    /// Repairs deliberately *not* attempted, and why (a merge that would have
    /// collapsed an edge, a tolerance that would have exceeded the kernel
    /// limit, a pass skipped for want of tessellable geometry).
    pub notes: Vec<String>,
}

impl HealResult {
    /// Whether the body entered invalid and left valid.
    pub fn healed(&self) -> bool {
        !self.failures_before.is_empty() && self.remaining.is_empty()
    }

    /// Whether anything was applied (or, under
    /// [`HealStrategy::ReportOnly`], proposed).
    pub fn is_empty(&self) -> bool {
        self.operations.is_empty()
    }
}

/// Phase-1 repair passes over an imported body (`spec/06-step-io.md` §6).
pub struct GeometryHealer;

impl GeometryHealer {
    /// Run every pass the strategy enables, then re-validate.
    ///
    /// The body is left valid, or left exactly as the passes got it — there
    /// is no partial rollback here. Callers that need the original body back
    /// (the STEP reader does, to fall through to its mesh path) must snapshot
    /// or rebuild it themselves.
    pub fn heal(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        options: &HealOptions,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check(body),
            ..HealResult::default()
        };
        if store.body(body).is_none() {
            result.remaining = result.failures_before.clone();
            return result;
        }
        let strategy = options.strategy;

        if strategy.closes_gaps() {
            let max_gap = resolve_max_gap(store, body, options.max_gap, &mut result.notes);
            Self::collapse_degenerate_edges_into(body, store, geo, options, max_gap, &mut result);
            let merge_gap = floor_merge_gap(max_gap, options.gap_floor, &mut result.notes);
            Self::fix_gaps_into(body, store, geo, options, merge_gap, &mut result);
        }
        if strategy.fixes_geometry() {
            // Rescue only: from the import pipeline, an edge whose deviation
            // still fits under the kernel cap is the reader's to absorb as
            // tolerance (`record_edge_tolerances`); recomputing it here would
            // repair bodies that were never going to fail. Only the edges no
            // tolerance can save get their curves replaced.
            Self::fix_edge_surface_consistency_into(
                body,
                store,
                geo,
                true,
                strategy.applies(),
                &mut result,
            );
            Self::fix_pcurves_into(body, store, geo, strategy.applies(), &mut result);
        }
        if strategy.orients() {
            Self::fix_orientation_into(body, store, geo, strategy.applies(), &mut result);
        }

        if strategy.applies() {
            // Topology counts changed (welding removes edges and vertices),
            // so the genus the reader recovered at build time is stale.
            recover_genus(store, body);
            result.remaining = store.check(body);
        } else {
            result.remaining = result.failures_before.clone();
        }
        result
    }

    /// Close vertex gaps and weld the duplicate edges they expose.
    ///
    /// This is the sewing pass: it is what turns a shell whose faces each
    /// authored their own copy of every shared boundary into one whose edges
    /// have two fins apiece.
    pub fn fix_gaps(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        options: &HealOptions,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check(body),
            ..HealResult::default()
        };
        let max_gap = resolve_max_gap(store, body, options.max_gap, &mut result.notes);
        let merge_gap = floor_merge_gap(max_gap, options.gap_floor, &mut result.notes);
        Self::fix_gaps_into(body, store, geo, options, merge_gap, &mut result);
        result.remaining = if options.strategy.applies() {
            recover_genus(store, body);
            store.check(body)
        } else {
            result.failures_before.clone()
        };
        result
    }

    /// Collapse degenerate (sliver) edges — edges whose whole curve fits
    /// inside the merge tolerance — through the KEV Euler operator, so the
    /// Euler counts stay consistent (`spec/06-step-io.md` §6).
    pub fn fix_degenerate_edges(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        options: &HealOptions,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check(body),
            ..HealResult::default()
        };
        let max_gap = resolve_max_gap(store, body, options.max_gap, &mut result.notes);
        Self::collapse_degenerate_edges_into(body, store, geo, options, max_gap, &mut result);
        result.remaining = if options.strategy.applies() {
            recover_genus(store, body);
            store.check(body)
        } else {
            result.failures_before.clone()
        };
        result
    }

    /// Reconcile every edge with the surfaces of the faces it bounds
    /// (`spec/06-step-io.md` §6): raise the edge's tolerance to the measured
    /// deviation where that is an honest repair, and where no tolerance
    /// under [`MAX_ALLOWED_TOLERANCE`] is honest, replace the curve with the
    /// adjacent surfaces' intersection.
    ///
    /// Unlike the other passes this one judges *geometric* validity, so its
    /// `failures_before`/`remaining` come from
    /// [`TopologyStore::check_geometry`] rather than [`TopologyStore::check`].
    pub fn fix_edge_surface_consistency(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        options: &HealOptions,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check_geometry(geo, body),
            ..HealResult::default()
        };
        Self::fix_edge_surface_consistency_into(
            body,
            store,
            geo,
            false,
            options.strategy.applies(),
            &mut result,
        );
        result.remaining = if options.strategy.applies() {
            store.check_geometry(geo, body)
        } else {
            result.failures_before.clone()
        };
        result
    }

    /// Refit every pcurve that no longer tracks its edge's curve
    /// (`spec/06-step-io.md` §6). Fins with no pcurve at all are left for
    /// [`opensolid_brep::attach_body_pcurves`].
    ///
    /// Judges geometric validity, so `failures_before`/`remaining` come from
    /// [`TopologyStore::check_geometry`], like
    /// [`fix_edge_surface_consistency`](Self::fix_edge_surface_consistency).
    pub fn fix_pcurves(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        options: &HealOptions,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check_geometry(geo, body),
            ..HealResult::default()
        };
        Self::fix_pcurves_into(body, store, geo, options.strategy.applies(), &mut result);
        result.remaining = if options.strategy.applies() {
            store.check_geometry(geo, body)
        } else {
            result.failures_before.clone()
        };
        result
    }

    /// Make face orientation consistent across every shared edge, then
    /// reverse any shell left enclosing a signed volume of the wrong sign.
    pub fn fix_orientation(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
    ) -> HealResult {
        let mut result = HealResult {
            failures_before: store.check(body),
            ..HealResult::default()
        };
        Self::fix_orientation_into(body, store, geo, true, &mut result);
        result.remaining = store.check(body);
        result
    }

    fn fix_gaps_into(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        options: &HealOptions,
        max_gap: f64,
        result: &mut HealResult,
    ) {
        let plan = plan_gaps(store, geo, body, max_gap, &mut result.notes);
        if !options.strategy.applies() {
            report_gap_plan(&plan, &mut result.operations);
            return;
        }
        apply_vertex_merges(store, &plan.vertices, &mut result.operations);
        apply_edge_welds(store, &plan.edges, &mut result.operations);
    }

    /// `apply` comes from the strategy: a dry run plans the collapses and
    /// reports them without touching the body.
    fn collapse_degenerate_edges_into(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        options: &HealOptions,
        max_gap: f64,
        result: &mut HealResult,
    ) {
        let plan = plan_degenerate_collapses(store, geo, body, max_gap, &mut result.notes);
        if !options.strategy.applies() {
            for collapse in &plan {
                result.operations.push(HealOperation::EdgeCollapsed {
                    edge: collapse.edge,
                    kept: collapse.keep,
                    length: collapse.length,
                });
            }
            return;
        }
        apply_edge_collapses(store, &plan, &mut result.operations, &mut result.notes);
    }

    /// `rescue_only` restricts the pass to edges whose deviation exceeds
    /// [`MAX_ALLOWED_TOLERANCE`] — the ones no tolerance elevation can save.
    fn fix_edge_surface_consistency_into(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        rescue_only: bool,
        apply: bool,
        result: &mut HealResult,
    ) {
        let plan = plan_edge_surface_consistency(store, geo, body, rescue_only, &mut result.notes);
        if !apply {
            report_consistency_plan(store, &plan, &mut result.operations);
            return;
        }
        apply_consistency_repairs(store, geo, plan, &mut result.operations, &mut result.notes);
    }

    fn fix_pcurves_into(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &mut GeometryStore,
        apply: bool,
        result: &mut HealResult,
    ) {
        let plan = plan_pcurve_refits(store, geo, body, &mut result.notes);
        if !apply {
            for refit in &plan {
                result.operations.push(HealOperation::PcurveRecomputed {
                    fin: refit.fin,
                    edge: refit.edge,
                });
            }
            return;
        }
        apply_pcurve_refits(store, geo, plan, &mut result.operations);
    }

    /// `apply` false plans and reports without touching the body — what
    /// [`HealStrategy::ReportOnly`] wants. Note that a dry run sees the body
    /// as authored, so on a shell that gap closure has not sewn yet it can
    /// only report what is visible *before* sewing: the two passes are
    /// planned independently, where an `Auto` run orients the sewn result.
    fn fix_orientation_into(
        body: EntityId<Body>,
        store: &mut TopologyStore,
        geo: &GeometryStore,
        apply: bool,
        result: &mut HealResult,
    ) {
        let Some(flips) = plan_face_flips(store, body, &mut result.notes) else {
            return;
        };
        let reversals = plan_shell_reversals(store, geo, body, &flips, &mut result.notes);
        if flips.is_empty() && reversals.is_empty() {
            return;
        }

        for &face in &flips {
            result
                .operations
                .push(HealOperation::FaceReoriented { face });
        }
        for &(shell, faces) in &reversals {
            result
                .operations
                .push(HealOperation::ShellReversed { shell, faces });
        }

        if !apply {
            return;
        }
        let reversed: Vec<EntityId<Shell>> = reversals.iter().map(|&(s, _)| s).collect();
        let mut flip_set: Vec<EntityId<Face>> = flips;
        for shell in reversed {
            for &face in store.faces_of_shell(shell) {
                match flip_set.iter().position(|&f| f == face) {
                    // Flipping twice is a no-op: drop both.
                    Some(i) => {
                        flip_set.swap_remove(i);
                    }
                    None => flip_set.push(face),
                }
            }
        }
        for face in flip_set {
            flip_face(store, face);
        }
    }
}

// ---------------------------------------------------------------------
// Gap closure
// ---------------------------------------------------------------------

/// One cluster of coincident vertices, collapsed to `kept` at `point`.
#[derive(Debug)]
struct VertexMerge {
    kept: EntityId<Vertex>,
    merged: Vec<EntityId<Vertex>>,
    point: Point3,
    tolerance: f64,
    gap: f64,
}

/// One group of duplicate edges, collapsed to `kept`. Each merged entry
/// carries whether it runs against `kept` (its fins' senses flip).
#[derive(Debug)]
struct EdgeWeld {
    kept: EntityId<Edge>,
    merged: Vec<(EntityId<Edge>, bool)>,
    tolerance: f64,
    gap: f64,
}

/// Edge-weld candidate grouping: an unordered post-merge vertex pair plus the
/// shell using it (`None` when the edge already spans two shells).
type WeldKey = (EntityId<Vertex>, EntityId<Vertex>, Option<EntityId<Shell>>);

/// A weld candidate: the edge, whether it runs against the group's
/// representative, and the largest sample deviation from it.
type WeldMember = (EntityId<Edge>, bool, f64);

#[derive(Debug, Default)]
struct GapPlan {
    vertices: Vec<VertexMerge>,
    edges: Vec<EdgeWeld>,
}

/// Resolve the merge tolerance: the caller's, or [`HEAL_GAP_REL`] of the
/// body's bounding-box diagonal. Always clamped to
/// [`MAX_ALLOWED_TOLERANCE`], since a merge widens a tolerance by up to half
/// the gap and the kernel rejects bodies past that limit anyway.
fn resolve_max_gap(
    store: &TopologyStore,
    body: EntityId<Body>,
    requested: Option<f64>,
    notes: &mut Vec<String>,
) -> f64 {
    let raw = match requested {
        Some(gap) if gap.is_finite() && gap > 0.0 => gap,
        Some(gap) => {
            notes.push(format!("ignoring non-positive max_gap {gap}"));
            derived_max_gap(store, body)
        }
        None => derived_max_gap(store, body),
    };
    if raw > MAX_ALLOWED_TOLERANCE {
        notes.push(format!(
            "clamped max_gap {raw:.3e} mm to the kernel limit {MAX_ALLOWED_TOLERANCE:.3e} mm"
        ));
        return MAX_ALLOWED_TOLERANCE;
    }
    raw.max(SYSTEM_RESOLUTION)
}

/// Apply [`HealOptions::gap_floor`] to the resolved gap for the gap-closure
/// passes only. Clamped to [`MAX_ALLOWED_TOLERANCE`] for the same reason as
/// `resolve_max_gap`; a non-finite or non-positive floor is a no-op.
fn floor_merge_gap(max_gap: f64, gap_floor: f64, notes: &mut Vec<String>) -> f64 {
    if !(gap_floor.is_finite() && gap_floor > max_gap) {
        return max_gap;
    }
    let floored = gap_floor.min(MAX_ALLOWED_TOLERANCE);
    if floored > max_gap {
        notes.push(format!(
            "vertex-merge gap raised from {max_gap:.3e} mm to the {floored:.3e} mm gap floor \
             (declared closure)"
        ));
    }
    floored.max(max_gap)
}

fn derived_max_gap(store: &TopologyStore, body: EntityId<Body>) -> f64 {
    let mut lo = [f64::INFINITY; 3];
    let mut hi = [f64::NEG_INFINITY; 3];
    for vertex in body_vertices(store, body) {
        let p = store.vertex(vertex).expect("live vertex").point;
        for k in 0..3 {
            lo[k] = lo[k].min(p[k]);
            hi[k] = hi[k].max(p[k]);
        }
    }
    let diagonal: f64 = (0..3)
        .map(|k| {
            let d = hi[k] - lo[k];
            if d.is_finite() { d * d } else { 0.0 }
        })
        .sum::<f64>()
        .sqrt();
    (diagonal * HEAL_GAP_REL).max(SYSTEM_RESOLUTION)
}

fn plan_gaps(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    max_gap: f64,
    notes: &mut Vec<String>,
) -> GapPlan {
    let merges = plan_vertex_merges(store, geo, body, max_gap, notes);

    // Edge grouping keys use the *post-merge* vertex identity, so the plan is
    // computed in one pass with no mutation in between.
    let mut canonical: HashMap<EntityId<Vertex>, EntityId<Vertex>> = HashMap::new();
    for merge in &merges {
        for &v in &merge.merged {
            canonical.insert(v, merge.kept);
        }
    }
    let welds = plan_edge_welds(store, geo, body, max_gap, &canonical, notes);
    GapPlan {
        vertices: merges,
        edges: welds,
    }
}

fn plan_vertex_merges(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    max_gap: f64,
    notes: &mut Vec<String>,
) -> Vec<VertexMerge> {
    let vertices = body_vertices(store, body);
    if vertices.len() < 2 {
        return Vec::new();
    }
    let index: HashMap<EntityId<Vertex>, usize> =
        vertices.iter().enumerate().map(|(i, &v)| (v, i)).collect();
    let points: Vec<Point3> = vertices
        .iter()
        .map(|&v| store.vertex(v).expect("live vertex").point)
        .collect();

    // Hash the points onto a grid of `max_gap` cells and union each point
    // with the already-placed points of its 27-cell neighbourhood: linear in
    // the vertex count, unlike the all-pairs scan a 10^5-vertex import would
    // choke on.
    let cell = max_gap.max(SYSTEM_RESOLUTION);
    let key = |p: &Point3| -> [i64; 3] {
        [
            (p.x / cell).floor() as i64,
            (p.y / cell).floor() as i64,
            (p.z / cell).floor() as i64,
        ]
    };
    let mut grid: HashMap<[i64; 3], Vec<usize>> = HashMap::new();
    let mut dsu = DisjointSet::new(vertices.len());
    for (i, p) in points.iter().enumerate() {
        if !p.coords.iter().all(|c| c.is_finite()) {
            continue;
        }
        let k = key(p);
        for dx in -1..=1 {
            for dy in -1..=1 {
                for dz in -1..=1 {
                    let neighbour = [k[0] + dx, k[1] + dy, k[2] + dz];
                    for &j in grid.get(&neighbour).map(Vec::as_slice).unwrap_or(&[]) {
                        if (points[j] - p).norm() <= max_gap {
                            dsu.union(i, j);
                        }
                    }
                }
            }
        }
        grid.entry(k).or_default().push(i);
    }

    let mut clusters: HashMap<usize, Vec<usize>> = HashMap::new();
    for i in 0..vertices.len() {
        clusters.entry(dsu.find(i)).or_default().push(i);
    }

    // A cluster spanning two shells would sew them together at a point, which
    // is non-manifold between shells; a cluster that swallows an edge whole
    // would collapse that edge to nothing. Neither is a phase-1 repair.
    //
    // Holding both ends of an edge is not by itself swallowing it. A circle
    // whose seam vertex the writer spelled as two coincident `VERTEX_POINT`
    // entities — what OCC emits when the circle is tangent to a neighbouring
    // face, so the seam has to fall on the corner the tangency creates — puts
    // both ends in one cluster and still travels a full turn in between.
    // Merging that pair does not destroy the edge; it makes it the closed
    // edge the file meant, which the store already carries elsewhere (seam
    // meridians, circular caps).
    //
    // What survives the merge is precisely a loop, so both halves of that
    // word are checked: the curve must come back to where it started, or one
    // vertex cannot carry both ends, and it must leave the cluster in
    // between, or there is no edge left to carry. A sliver fails the second
    // test; a full-length curve whose endpoints were merely dragged together
    // fails the first.
    let shell_of = vertex_shells(store, body);
    let mut spans_edge: HashMap<usize, bool> = HashMap::new();
    for edge_id in body_edges(store, body) {
        let edge = store.edge(edge_id).expect("live edge");
        if edge.start_vertex == edge.end_vertex {
            continue;
        }
        let (Some(&a), Some(&b)) = (index.get(&edge.start_vertex), index.get(&edge.end_vertex))
        else {
            continue;
        };
        let root = dsu.find(a);
        if root != dsu.find(b) {
            continue;
        }
        // Without sampleable geometry there is nothing to prove the edge is a
        // loop, so assume the merge would collapse it.
        let survives = edge_curve_closes(store, geo, edge_id, max_gap)
            && edge_samples(store, geo, edge_id).is_some_and(|samples| {
                let members = clusters.get(&root).map(Vec::as_slice).unwrap_or(&[]);
                samples
                    .iter()
                    .any(|p| members.iter().all(|&i| (p - points[i]).norm() > max_gap))
            });
        if !survives {
            spans_edge.insert(root, true);
        }
    }

    let mut merges = Vec::new();
    let mut roots: Vec<usize> = clusters.keys().copied().collect();
    // Deterministic output order: arena ids are assigned in build order.
    roots.sort_unstable();
    for root in roots {
        let mut members = clusters.remove(&root).expect("root came from the map");
        if members.len() < 2 {
            continue;
        }
        members.sort_unstable();
        if spans_edge.contains_key(&root) {
            notes.push(format!(
                "skipped merging {} vertices near {:?}: an edge joins two of them \
                 (collapsing it is a phase-2 repair)",
                members.len(),
                points[members[0]]
            ));
            continue;
        }
        let shells: Vec<Option<EntityId<Shell>>> = members
            .iter()
            .map(|&i| shell_of.get(&vertices[i]).copied().flatten())
            .collect();
        if shells.windows(2).any(|w| w[0] != w[1]) {
            notes.push(format!(
                "skipped merging {} vertices near {:?}: they belong to different shells",
                members.len(),
                points[members[0]]
            ));
            continue;
        }

        let centroid = Point3::from(
            members
                .iter()
                .map(|&i| points[i].coords)
                .sum::<opensolid_core::Vector3>()
                / members.len() as f64,
        );
        let gap = members
            .iter()
            .map(|&i| (points[i] - centroid).norm())
            .fold(0.0, f64::max);
        // The surviving vertex must stay within its tolerance of the endpoint
        // of *every* edge the cluster brings with it — the invariant
        // `check_geometry` measures as `VertexOffEdge`. Member `i` was already
        // up to its own tolerance from those endpoints before the merge, and
        // the move to the centroid adds its displacement on top, so the two
        // sum per member. Taking `max(existing, gap)` instead was short by
        // whatever the members were already carrying, which is never zero for
        // a file whose vertices and curves were rounded independently
        // (of-bbh8).
        let tolerance = members
            .iter()
            .map(|&i| {
                (points[i] - centroid).norm()
                    + store.vertex(vertices[i]).expect("live vertex").tolerance
            })
            .fold(SYSTEM_RESOLUTION, f64::max);
        if tolerance > MAX_ALLOWED_TOLERANCE {
            notes.push(format!(
                "skipped merging {} vertices near {:?}: the merge needs tolerance \
                 {tolerance:.3e} mm, past the kernel limit {MAX_ALLOWED_TOLERANCE:.3e} mm",
                members.len(),
                points[members[0]]
            ));
            continue;
        }
        merges.push(VertexMerge {
            kept: vertices[members[0]],
            merged: members[1..].iter().map(|&i| vertices[i]).collect(),
            point: centroid,
            tolerance,
            gap,
        });
    }
    merges
}

fn plan_edge_welds(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    max_gap: f64,
    canonical: &HashMap<EntityId<Vertex>, EntityId<Vertex>>,
    notes: &mut Vec<String>,
) -> Vec<EdgeWeld> {
    let canon = |v: EntityId<Vertex>| canonical.get(&v).copied().unwrap_or(v);
    let shell_of = edge_shells(store, body);
    // `EntityId` is deliberately not `Ord` (an arena slot has no meaningful
    // order), so an *unordered* vertex pair is keyed by discovery rank.
    let rank: HashMap<EntityId<Vertex>, usize> = body_vertices(store, body)
        .into_iter()
        .enumerate()
        .map(|(i, v)| (v, i))
        .collect();
    let ordered = |a: EntityId<Vertex>, b: EntityId<Vertex>| {
        let (ra, rb) = (rank.get(&a).copied(), rank.get(&b).copied());
        if ra <= rb { (a, b) } else { (b, a) }
    };

    // Candidate group: same (unordered) post-merge vertex pair, same shell.
    let mut groups: HashMap<WeldKey, Vec<EntityId<Edge>>> = HashMap::new();
    let mut order: Vec<WeldKey> = Vec::new();
    for edge_id in body_edges(store, body) {
        let edge = store.edge(edge_id).expect("live edge");
        // Welding closes an *open* boundary. An edge that already has both
        // its sides is not open, and folding another edge into it could only
        // push it past two fins — the non-manifold state this pass exists to
        // avoid. Two coincident two-sided edges are two sheets touching, and
        // sewing them together is a decision about the model, not a repair.
        if edge.fins.len() >= 2 {
            continue;
        }
        let (a, b) = (canon(edge.start_vertex), canon(edge.end_vertex));
        let pair = ordered(a, b);
        let key = (pair.0, pair.1, shell_of.get(&edge_id).copied().flatten());
        let slot = groups.entry(key).or_insert_with(|| {
            order.push(key);
            Vec::new()
        });
        slot.push(edge_id);
    }

    let mut welds = Vec::new();
    for key in order {
        let group = groups.remove(&key).expect("key came from the same pass");
        if group.len() < 2 {
            continue;
        }
        // Two edges over one vertex pair need not be duplicates — the two
        // halves of a split circle share both ends. Sampling the curves tells
        // them apart, and tells a duplicate's direction at the same time.
        let mut samples: Vec<(EntityId<Edge>, [Point3; SAMPLE_FRACTIONS.len()])> = Vec::new();
        for &edge_id in &group {
            match edge_samples(store, geo, edge_id) {
                Some(points) => samples.push((edge_id, points)),
                None => notes.push(format!(
                    "cannot weld {edge_id:?}: it has no attached curve geometry to compare"
                )),
            }
        }

        // Greedy clustering against each cluster's representative: duplicate
        // counts per vertex pair are tiny (2 for a sewn-up shell, 3-4 for a
        // non-manifold one), so the quadratic scan is free.
        let mut clusters: Vec<(usize, Vec<WeldMember>)> = Vec::new();
        'candidate: for (idx, (edge_id, points)) in samples.iter().enumerate() {
            for (rep, members) in clusters.iter_mut() {
                let (_, rep_points) = &samples[*rep];
                if let Some((reversed, deviation)) = curve_match(rep_points, points, max_gap) {
                    members.push((*edge_id, reversed, deviation));
                    continue 'candidate;
                }
            }
            clusters.push((idx, Vec::new()));
        }

        for (rep, members) in clusters {
            if members.is_empty() {
                continue;
            }
            let kept = samples[rep].0;
            let gap = members.iter().map(|&(_, _, d)| d).fold(0.0, f64::max);
            // The surviving curve inherits every victim's fins, so it is now
            // measured against faces it was never on. Against victim `j`'s
            // surface it is off by whatever `j` was off by, plus the distance
            // between the two curves — the same per-member sum the vertex
            // merge above makes, for the same reason (of-bbh8).
            let tolerance = members
                .iter()
                .map(|&(e, _, deviation)| deviation + store.edge(e).expect("live edge").tolerance)
                .fold(
                    SYSTEM_RESOLUTION.max(store.edge(kept).expect("live edge").tolerance),
                    f64::max,
                );
            if tolerance > MAX_ALLOWED_TOLERANCE {
                notes.push(format!(
                    "skipped welding {} edges onto {kept:?}: the weld needs tolerance \
                     {tolerance:.3e} mm, past the kernel limit {MAX_ALLOWED_TOLERANCE:.3e} mm",
                    members.len()
                ));
                continue;
            }
            welds.push(EdgeWeld {
                kept,
                merged: members.iter().map(|&(e, r, _)| (e, r)).collect(),
                tolerance,
                gap,
            });
        }
    }
    welds
}

/// Whether two sampled curves are the same curve, and if so whether the
/// second runs against the first. Compares forward and reversed sample orders
/// and takes whichever fits inside `tolerance`.
fn curve_match(
    a: &[Point3; SAMPLE_FRACTIONS.len()],
    b: &[Point3; SAMPLE_FRACTIONS.len()],
    tolerance: f64,
) -> Option<(bool, f64)> {
    let n = SAMPLE_FRACTIONS.len();
    let forward = (0..n).map(|i| (a[i] - b[i]).norm()).fold(0.0f64, f64::max);
    let reverse = (0..n)
        .map(|i| (a[i] - b[n - 1 - i]).norm())
        .fold(0.0f64, f64::max);
    if forward <= tolerance && forward <= reverse {
        Some((false, forward))
    } else if reverse <= tolerance {
        Some((true, reverse))
    } else {
        None
    }
}

/// Whether the edge's own curve ends where it began, so a single vertex can
/// stand at both of its ends. True of the full circle a tangency forces a
/// writer to spell with two coincident vertices; false of a line, whatever
/// its vertices claim.
fn edge_curve_closes(
    store: &TopologyStore,
    geo: &GeometryStore,
    edge_id: EntityId<Edge>,
    tolerance: f64,
) -> bool {
    let Some(edge) = store.edge(edge_id) else {
        return false;
    };
    let Some(curve) = edge.curve.and_then(|c| geo.curve(c)) else {
        return false;
    };
    (curve.point(edge.t_end) - curve.point(edge.t_start)).norm() <= tolerance
}

fn edge_samples(
    store: &TopologyStore,
    geo: &GeometryStore,
    edge_id: EntityId<Edge>,
) -> Option<[Point3; SAMPLE_FRACTIONS.len()]> {
    let edge = store.edge(edge_id)?;
    let curve = geo.curve(edge.curve?)?;
    let span = edge.t_end - edge.t_start;
    let mut out = [Point3::origin(); SAMPLE_FRACTIONS.len()];
    for (slot, f) in out.iter_mut().zip(SAMPLE_FRACTIONS) {
        *slot = curve.point(edge.t_start + span * f);
    }
    Some(out)
}

fn report_gap_plan(plan: &GapPlan, operations: &mut Vec<HealOperation>) {
    for merge in &plan.vertices {
        operations.push(HealOperation::VerticesMerged {
            kept: merge.kept,
            merged: merge.merged.len(),
            gap: merge.gap,
        });
    }
    for weld in &plan.edges {
        operations.push(HealOperation::EdgesWelded {
            kept: weld.kept,
            merged: weld.merged.len(),
            gap: weld.gap,
        });
    }
}

fn apply_vertex_merges(
    store: &mut TopologyStore,
    merges: &[VertexMerge],
    operations: &mut Vec<HealOperation>,
) {
    for merge in merges {
        let mut adopted: Vec<EntityId<Edge>> = Vec::new();
        for &victim in &merge.merged {
            let Some(vertex) = store.vertices.remove(victim) else {
                continue;
            };
            for edge_id in vertex.edges {
                let Some(edge) = store.edges.get_mut(edge_id) else {
                    continue;
                };
                if edge.start_vertex == victim {
                    edge.start_vertex = merge.kept;
                }
                if edge.end_vertex == victim {
                    edge.end_vertex = merge.kept;
                }
                if !adopted.contains(&edge_id) {
                    adopted.push(edge_id);
                }
            }
        }
        let Some(kept) = store.vertices.get_mut(merge.kept) else {
            continue;
        };
        kept.point = merge.point;
        let elevated = merge.tolerance > kept.tolerance;
        kept.tolerance = kept.tolerance.max(merge.tolerance);
        for edge_id in adopted {
            if !kept.edges.contains(&edge_id) {
                kept.edges.push(edge_id);
            }
        }
        operations.push(HealOperation::VerticesMerged {
            kept: merge.kept,
            merged: merge.merged.len(),
            gap: merge.gap,
        });
        if elevated {
            operations.push(HealOperation::ToleranceElevated {
                entity: EntityRef::Vertex(merge.kept),
                new_tolerance: merge.tolerance,
            });
        }
    }
}

fn apply_edge_welds(
    store: &mut TopologyStore,
    welds: &[EdgeWeld],
    operations: &mut Vec<HealOperation>,
) {
    for weld in welds {
        if store.edges.get(weld.kept).is_none() {
            continue;
        }
        let mut moved = 0;
        for &(victim, reversed) in &weld.merged {
            let Some(edge) = store.edges.remove(victim) else {
                continue;
            };
            for &fin_id in &edge.fins {
                let Some(fin) = store.fins.get_mut(fin_id) else {
                    continue;
                };
                fin.edge = weld.kept;
                if reversed {
                    fin.sense = fin.sense.opposite();
                }
                fin.mate = None;
                let kept = store.edges.get_mut(weld.kept).expect("checked above");
                if !kept.fins.contains(&fin_id) {
                    kept.fins.push(fin_id);
                }
            }
            // The victim's curve stays in the geometry store: curves may be
            // shared, and an orphan curve is invisible to `check` where a
            // dangling id would not be.
            for vertex_id in [edge.start_vertex, edge.end_vertex] {
                if let Some(vertex) = store.vertices.get_mut(vertex_id) {
                    vertex.edges.retain(|&e| e != victim);
                }
            }
            moved += 1;
        }
        if moved == 0 {
            continue;
        }

        let kept = store.edges.get_mut(weld.kept).expect("checked above");
        let elevated = weld.tolerance > kept.tolerance;
        kept.tolerance = kept.tolerance.max(weld.tolerance);
        let fins = kept.fins.clone();
        // Exactly two fins is the manifold case, and the only one where a
        // mate link is meaningful; `check` reports anything else.
        if fins.len() == 2 {
            store.fins.get_mut(fins[0]).expect("live fin").mate = Some(fins[1]);
            store.fins.get_mut(fins[1]).expect("live fin").mate = Some(fins[0]);
        }
        // Both endpoints must list the surviving edge — a fin adopted from a
        // victim may be the only user of a vertex the kept edge never saw.
        let (start, end) = {
            let kept = store.edges.get(weld.kept).expect("checked above");
            (kept.start_vertex, kept.end_vertex)
        };
        for vertex_id in [start, end] {
            if let Some(vertex) = store.vertices.get_mut(vertex_id)
                && !vertex.edges.contains(&weld.kept)
            {
                vertex.edges.push(weld.kept);
            }
        }

        operations.push(HealOperation::EdgesWelded {
            kept: weld.kept,
            merged: moved,
            gap: weld.gap,
        });
        if elevated {
            operations.push(HealOperation::ToleranceElevated {
                entity: EntityRef::Edge(weld.kept),
                new_tolerance: weld.tolerance,
            });
        }
    }
}

// ---------------------------------------------------------------------
// Sliver collapse (phase 2)
// ---------------------------------------------------------------------

/// One degenerate edge to contract, with the survivor's new placement and
/// the tolerance that placement costs.
#[derive(Debug)]
struct EdgeCollapse {
    edge: EntityId<Edge>,
    keep: EntityId<Vertex>,
    point: Point3,
    tolerance: f64,
    length: f64,
}

fn plan_degenerate_collapses(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    max_gap: f64,
    notes: &mut Vec<String>,
) -> Vec<EdgeCollapse> {
    let mut plan = Vec::new();
    for edge_id in body_edges(store, body) {
        let edge = store.edge(edge_id).expect("live edge");
        // A closed edge already has one vertex at both ends; contracting it
        // would kill no vertex, so it is not a KEV.
        if edge.start_vertex == edge.end_vertex {
            continue;
        }
        let (Some(va), Some(vb)) = (
            store.vertex(edge.start_vertex),
            store.vertex(edge.end_vertex),
        ) else {
            continue;
        };
        let length = (vb.point - va.point).norm();
        // NaN-safe: only a provably short edge is a candidate.
        if !matches!(
            length.partial_cmp(&max_gap),
            Some(std::cmp::Ordering::Less | std::cmp::Ordering::Equal)
        ) {
            continue;
        }
        let mid = Point3::from((va.point.coords + vb.point.coords) / 2.0);
        // Degenerate means the *curve* fits in the gap, not just the ends: a
        // closed curve whose seam vertices drifted together travels a full
        // turn in between, and that is phase 1's vertex merge (the edge
        // survives as the closed edge the file meant), not a collapse.
        let Some(samples) = edge_samples(store, geo, edge_id) else {
            notes.push(format!(
                "skipped collapsing {edge_id:?}: no curve geometry to prove it is degenerate"
            ));
            continue;
        };
        if samples.iter().any(|p| (p - mid).norm() > max_gap) {
            continue;
        }
        // A second edge over the same pair would come out of the collapse as
        // a closed sliver ring — worse than the sliver going in.
        let (a, b) = (edge.start_vertex, edge.end_vertex);
        let twin = va.edges.iter().any(|&other| {
            other != edge_id
                && store.edge(other).is_some_and(|e| {
                    (e.start_vertex == a && e.end_vertex == b)
                        || (e.start_vertex == b && e.end_vertex == a)
                })
        });
        if twin {
            notes.push(format!(
                "skipped collapsing {edge_id:?}: another edge joins the same vertices \
                 (the collapse would leave a degenerate ring)"
            ));
            continue;
        }
        // The survivor stands in for both ends against everything they were
        // attached to, so each member's cost is its displacement *plus* what
        // it already carried (of-bbh8).
        let tolerance = [va, vb]
            .iter()
            .map(|v| (v.point - mid).norm() + v.tolerance)
            .fold(SYSTEM_RESOLUTION, f64::max);
        if tolerance > MAX_ALLOWED_TOLERANCE {
            notes.push(format!(
                "skipped collapsing {edge_id:?}: the collapse needs tolerance \
                 {tolerance:.3e} mm, past the kernel limit {MAX_ALLOWED_TOLERANCE:.3e} mm"
            ));
            continue;
        }
        plan.push(EdgeCollapse {
            edge: edge_id,
            keep: a,
            point: mid,
            tolerance,
            length,
        });
    }
    plan
}

fn apply_edge_collapses(
    store: &mut TopologyStore,
    plan: &[EdgeCollapse],
    operations: &mut Vec<HealOperation>,
    notes: &mut Vec<String>,
) {
    for collapse in plan {
        // A chain of slivers: an earlier collapse may have re-pointed this
        // edge's ends. The planned survivor must still be one of them.
        let Some(edge) = store.edge(collapse.edge) else {
            continue;
        };
        if edge.start_vertex != collapse.keep && edge.end_vertex != collapse.keep {
            notes.push(format!(
                "skipped collapsing {:?}: an earlier collapse moved its vertices",
                collapse.edge
            ));
            continue;
        }
        match store.kev(collapse.edge, collapse.keep) {
            Ok(()) => {
                let vertex = store
                    .vertices
                    .get_mut(collapse.keep)
                    .expect("kev keeps this vertex");
                vertex.point = collapse.point;
                let elevated = collapse.tolerance > vertex.tolerance;
                vertex.tolerance = vertex.tolerance.max(collapse.tolerance);
                operations.push(HealOperation::EdgeCollapsed {
                    edge: collapse.edge,
                    kept: collapse.keep,
                    length: collapse.length,
                });
                if elevated {
                    operations.push(HealOperation::ToleranceElevated {
                        entity: EntityRef::Vertex(collapse.keep),
                        new_tolerance: collapse.tolerance,
                    });
                }
            }
            Err(e) => notes.push(format!("could not collapse {:?}: {e}", collapse.edge)),
        }
    }
}

// ---------------------------------------------------------------------
// Edge/surface consistency (phase 2)
// ---------------------------------------------------------------------

/// Parameter fractions at which a candidate intersection branch is matched
/// against the authored curve. Interior only: the endpoints agree by
/// construction (the candidate was trimmed to the edge's vertices).
const BRANCH_MATCH_FRACTIONS: [f64; 3] = [0.25, 0.5, 0.75];

/// Uniform sample count for measuring a replacement curve against the two
/// surfaces it must lie on, and a pcurve against its lockstep invariant.
const RECOMPUTE_MEASURE_SAMPLES: usize = 9;

/// How far a candidate intersection branch may sit from the authored curve,
/// as a multiple of that curve's own off-surface deviation, before choosing
/// it would be guessing. A curve within `d` of both surfaces is within
/// `O(d)` of their intersection unless they meet at a grazing angle; one
/// decade of headroom covers the angles real shells produce, and past it
/// the honest answer is refusal (the far arc of a circle shares the
/// endpoints of the near one, and only this test tells them apart).
const BRANCH_MATCH_FACTOR: f64 = 10.0;

/// One planned repair for an edge that leaves its faces' surfaces.
enum ConsistencyRepair {
    /// Raise tolerances to the measured distances — the honest minimal
    /// repair while everything fits under the kernel cap. Covers the whole
    /// defect the bad curve causes: the edge's distance to its surfaces
    /// *and* each vertex's distance to the curve's ends (a curve written
    /// against the wrong support misses both).
    Elevate {
        edge: EntityId<Edge>,
        new_tolerance: f64,
        vertex_tolerances: Vec<(EntityId<Vertex>, f64)>,
    },
    /// Replace the curve outright: nothing under [`MAX_ALLOWED_TOLERANCE`]
    /// covers where it actually is.
    Recompute(Box<EdgeRecompute>),
}

struct EdgeRecompute {
    edge: EntityId<Edge>,
    curve: Curve3,
    t_start: f64,
    t_end: f64,
    deviation_before: f64,
    deviation_after: f64,
    /// `Some` when the replacement still needs more tolerance than the edge
    /// carries (numerical residue, not authored error).
    edge_tolerance: Option<f64>,
    /// Per-end raises covering each vertex's distance to the new curve.
    vertex_tolerances: Vec<(EntityId<Vertex>, f64)>,
}

fn plan_edge_surface_consistency(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    rescue_only: bool,
    notes: &mut Vec<String>,
) -> Vec<ConsistencyRepair> {
    let mut plan = Vec::new();
    for (edge_id, deviation) in store.measure_edges_off_surfaces(geo, body) {
        let Some(edge) = store.edge(edge_id) else {
            continue;
        };
        if deviation <= edge.tolerance.max(SYSTEM_RESOLUTION) {
            continue;
        }
        // Under the kernel cap, the minimal repair wins: record the measured
        // distances as tolerance and change no geometry.
        if deviation <= MAX_ALLOWED_TOLERANCE {
            if !rescue_only {
                plan.push(ConsistencyRepair::Elevate {
                    edge: edge_id,
                    new_tolerance: deviation,
                    vertex_tolerances: plan_endpoint_raises(store, geo, edge),
                });
            }
            continue;
        }
        match plan_curve_recompute(store, geo, edge_id, deviation, notes) {
            Some(recompute) => plan.push(ConsistencyRepair::Recompute(Box::new(recompute))),
            None => notes.push(format!(
                "edge {edge_id:?} strays {deviation:.3e} mm from its faces' surfaces — past \
                 the kernel limit {MAX_ALLOWED_TOLERANCE:.3e} mm, and no honest replacement \
                 curve was found"
            )),
        }
    }
    plan
}

/// Vertex tolerance raises covering each end's distance to the edge's own
/// curve endpoints (`spec/08-tolerances.md` §7.1 invariant 2) — the other
/// half of what a curve written against the wrong support breaks. Raises
/// past [`MAX_ALLOWED_TOLERANCE`] are not proposed; the residual failure is
/// `check_geometry`'s to report.
fn plan_endpoint_raises(
    store: &TopologyStore,
    geo: &GeometryStore,
    edge: &Edge,
) -> Vec<(EntityId<Vertex>, f64)> {
    let mut raises = Vec::new();
    let Some(curve) = edge.curve.and_then(|id| geo.curve(id)) else {
        return raises;
    };
    for (vertex_id, t) in [
        (edge.start_vertex, edge.t_start),
        (edge.end_vertex, edge.t_end),
    ] {
        let Some(vertex) = store.vertex(vertex_id) else {
            continue;
        };
        let residual = (vertex.point - curve.point(t)).norm();
        if residual.is_finite()
            && residual > vertex.tolerance
            && residual <= MAX_ALLOWED_TOLERANCE
            && !raises.iter().any(|&(v, _)| v == vertex_id)
        {
            raises.push((vertex_id, residual));
        }
    }
    raises
}

/// Derive a replacement curve for `edge_id` from the intersection of its two
/// adjacent faces' surfaces, trimmed to the edge's vertices. `None` (with a
/// note) when the edge does not have exactly two distinct measurable faces,
/// the surfaces do not intersect along a curve the kernel can represent, or
/// no intersection branch matches the authored curve.
fn plan_curve_recompute(
    store: &TopologyStore,
    geo: &GeometryStore,
    edge_id: EntityId<Edge>,
    deviation_before: f64,
    notes: &mut Vec<String>,
) -> Option<EdgeRecompute> {
    let edge = store.edge(edge_id)?;
    let old_curve = geo.curve(edge.curve?)?;
    let fins = store.fins_of_edge(edge_id);
    if fins.len() != 2 {
        notes.push(format!(
            "cannot recompute {edge_id:?}: it has {} fin(s), not two",
            fins.len()
        ));
        return None;
    }
    let (face_a, face_b) = (store.fin_face(fins[0]), store.fin_face(fins[1]));
    if face_a == face_b {
        notes.push(format!(
            "cannot recompute {edge_id:?}: both sides lie on one face (a seam has no \
             second surface to intersect)"
        ));
        return None;
    }
    let surface_of = |face: EntityId<Face>| {
        store
            .face(face)
            .and_then(|f| f.surface)
            .and_then(|id| geo.surface(id))
    };
    let (Some(surface_a), Some(surface_b)) = (surface_of(face_a), surface_of(face_b)) else {
        notes.push(format!(
            "cannot recompute {edge_id:?}: an adjacent face has no surface"
        ));
        return None;
    };
    let candidates = match intersect_surfaces(surface_a, surface_b, &ToleranceContext::default()) {
        Ok(SurfaceIntersection::Curves(curves)) => curves,
        Ok(_) => {
            notes.push(format!(
                "cannot recompute {edge_id:?}: its faces' surfaces do not intersect along \
                 a curve"
            ));
            return None;
        }
        Err(e) => {
            notes.push(format!("cannot recompute {edge_id:?}: {e}"));
            return None;
        }
    };

    let start = store.vertex(edge.start_vertex)?.point;
    let end = store.vertex(edge.end_vertex)?.point;
    let closed = edge.start_vertex == edge.end_vertex;
    let old_span = edge.t_end - edge.t_start;

    let mut best: Option<(f64, EdgeRecompute)> = None;
    for candidate in candidates {
        // SSI returns the locus unoriented; the edge runs start → end. Try
        // the branch both ways round where the curve type allows it.
        for curve in oriented_variants(candidate.curve) {
            let projected_start = curve.project_point(&start);
            if !projected_start.converged {
                continue;
            }
            let (start_residual, t0) = (projected_start.distance, projected_start.t);
            let (t1, end_residual) = if closed {
                // One vertex at both ends of a full period.
                let Some(period) = curve.period() else {
                    continue;
                };
                (t0 + period, start_residual)
            } else {
                let projected_end = curve.project_point(&end);
                if !projected_end.converged {
                    continue;
                }
                let mut t1 = projected_end.t;
                if t1 <= t0 {
                    match curve.period() {
                        Some(period) => t1 += period,
                        // A bounded curve running end-before-start is the
                        // reversed variant's business.
                        None => continue,
                    }
                }
                (t1, projected_end.distance)
            };
            if !t0.is_finite() || !t1.is_finite() || t1 <= t0 {
                continue;
            }
            // The vertices stay where they are; each must reach the new
            // curve within a tolerance the kernel accepts.
            if start_residual > MAX_ALLOWED_TOLERANCE || end_residual > MAX_ALLOWED_TOLERANCE {
                continue;
            }
            // The right branch hugs the authored curve; one that merely
            // shares its endpoints (the far arc of a circle does) is a
            // different edge.
            let span = t1 - t0;
            let score = BRANCH_MATCH_FRACTIONS
                .iter()
                .map(|f| {
                    (curve.point(t0 + span * f) - old_curve.point(edge.t_start + old_span * f))
                        .norm()
                })
                .fold(0.0f64, f64::max);
            if !score.is_finite()
                || score > BRANCH_MATCH_FACTOR * deviation_before.max(SYSTEM_RESOLUTION)
            {
                continue;
            }
            // Measure the replacement honestly rather than trusting SSI.
            let mut deviation_after = 0.0f64;
            let mut measurable = true;
            for i in 0..RECOMPUTE_MEASURE_SAMPLES {
                let t = t0 + span * (i as f64) / (RECOMPUTE_MEASURE_SAMPLES - 1) as f64;
                let p = curve.point(t);
                if !p.coords.iter().all(|c| c.is_finite()) {
                    measurable = false;
                    break;
                }
                for surface in [surface_a, surface_b] {
                    let projection = surface.project_point(&p);
                    if projection.converged && projection.distance > deviation_after {
                        deviation_after = projection.distance;
                    }
                }
            }
            if !measurable
                || deviation_after >= deviation_before
                || deviation_after > MAX_ALLOWED_TOLERANCE
            {
                continue;
            }
            let mut vertex_tolerances = Vec::new();
            for (vertex_id, residual) in [
                (edge.start_vertex, start_residual),
                (edge.end_vertex, end_residual),
            ] {
                let vertex = store.vertex(vertex_id).expect("live vertex");
                if residual > vertex.tolerance
                    && !vertex_tolerances.iter().any(|&(v, _)| v == vertex_id)
                {
                    vertex_tolerances.push((vertex_id, residual));
                }
            }
            if best.as_ref().is_none_or(|&(s, _)| score < s) {
                best = Some((
                    score,
                    EdgeRecompute {
                        edge: edge_id,
                        curve: curve.clone(),
                        t_start: t0,
                        t_end: t1,
                        deviation_before,
                        deviation_after,
                        edge_tolerance: (deviation_after > edge.tolerance)
                            .then_some(deviation_after),
                        vertex_tolerances,
                    },
                ));
            }
        }
    }
    if best.is_none() {
        notes.push(format!(
            "cannot recompute {edge_id:?}: no branch of its faces' intersection matches the \
             authored curve"
        ));
    }
    best.map(|(_, recompute)| recompute)
}

/// The candidate and, where the type supports it, the same locus running the
/// other way.
fn oriented_variants(curve: Curve3) -> Vec<Curve3> {
    let reversed = match &curve {
        Curve3::Line { origin, dir } => Curve3::line(*origin, -*dir).ok(),
        Curve3::Circle {
            center,
            axis,
            radius,
        } => Curve3::circle(*center, -*axis, *radius).ok(),
        Curve3::Ellipse {
            center,
            axis,
            major_dir,
            major_radius,
            minor_radius,
        } => Curve3::ellipse(*center, -*axis, *major_dir, *major_radius, *minor_radius).ok(),
        _ => None,
    };
    std::iter::once(curve).chain(reversed).collect()
}

fn report_consistency_plan(
    store: &TopologyStore,
    plan: &[ConsistencyRepair],
    operations: &mut Vec<HealOperation>,
) {
    for repair in plan {
        match repair {
            ConsistencyRepair::Elevate {
                edge,
                new_tolerance,
                vertex_tolerances,
            } => {
                operations.push(HealOperation::ToleranceElevated {
                    entity: EntityRef::Edge(*edge),
                    new_tolerance: *new_tolerance,
                });
                for &(vertex, new_tolerance) in vertex_tolerances {
                    operations.push(HealOperation::ToleranceElevated {
                        entity: EntityRef::Vertex(vertex),
                        new_tolerance,
                    });
                }
            }
            ConsistencyRepair::Recompute(recompute) => {
                operations.push(HealOperation::EdgeCurveRecomputed {
                    edge: recompute.edge,
                    deviation_before: recompute.deviation_before,
                    deviation_after: recompute.deviation_after,
                });
                // An Auto run refits these fins' pcurves with the curve.
                for &fin_id in store.fins_of_edge(recompute.edge) {
                    if store.fin(fin_id).is_some_and(|fin| fin.pcurve.is_some()) {
                        operations.push(HealOperation::PcurveRecomputed {
                            fin: fin_id,
                            edge: recompute.edge,
                        });
                    }
                }
            }
        }
    }
}

fn apply_consistency_repairs(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    plan: Vec<ConsistencyRepair>,
    operations: &mut Vec<HealOperation>,
    notes: &mut Vec<String>,
) {
    for repair in plan {
        match repair {
            ConsistencyRepair::Elevate {
                edge,
                new_tolerance,
                vertex_tolerances,
            } => {
                let Some(e) = store.edges.get_mut(edge) else {
                    continue;
                };
                if new_tolerance > e.tolerance {
                    e.tolerance = new_tolerance;
                    operations.push(HealOperation::ToleranceElevated {
                        entity: EntityRef::Edge(edge),
                        new_tolerance,
                    });
                }
                apply_vertex_raises(store, &vertex_tolerances, operations);
            }
            ConsistencyRepair::Recompute(recompute) => {
                if store.edges.get(recompute.edge).is_none() {
                    continue;
                }
                // The old curve stays in the geometry store: curves may be
                // shared, and an orphan curve is invisible to `check` where
                // a dangling id would not be.
                let curve_id = geo.add_curve(recompute.curve.clone());
                let e = store.edges.get_mut(recompute.edge).expect("checked above");
                e.curve = Some(curve_id);
                e.t_start = recompute.t_start;
                e.t_end = recompute.t_end;
                if let Some(tolerance) = recompute.edge_tolerance
                    && tolerance > e.tolerance
                {
                    e.tolerance = tolerance;
                    operations.push(HealOperation::ToleranceElevated {
                        entity: EntityRef::Edge(recompute.edge),
                        new_tolerance: tolerance,
                    });
                }
                operations.push(HealOperation::EdgeCurveRecomputed {
                    edge: recompute.edge,
                    deviation_before: recompute.deviation_before,
                    deviation_after: recompute.deviation_after,
                });
                apply_vertex_raises(store, &recompute.vertex_tolerances, operations);
                refit_pcurves_of_edge(store, geo, &recompute, operations, notes);
            }
        }
    }
}

fn apply_vertex_raises(
    store: &mut TopologyStore,
    raises: &[(EntityId<Vertex>, f64)],
    operations: &mut Vec<HealOperation>,
) {
    for &(vertex_id, tolerance) in raises {
        if let Some(vertex) = store.vertices.get_mut(vertex_id)
            && tolerance > vertex.tolerance
        {
            vertex.tolerance = tolerance;
            operations.push(HealOperation::ToleranceElevated {
                entity: EntityRef::Vertex(vertex_id),
                new_tolerance: tolerance,
            });
        }
    }
}

/// The fins riding a recomputed edge carried pcurves fit against the curve
/// that was just replaced. Refit each against the replacement; one that will
/// not fit is dropped, because a missing pcurve is honest where a stale one
/// lies.
fn refit_pcurves_of_edge(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    recompute: &EdgeRecompute,
    operations: &mut Vec<HealOperation>,
    notes: &mut Vec<String>,
) {
    let fins: Vec<EntityId<Fin>> = store.fins_of_edge(recompute.edge).to_vec();
    for fin_id in fins {
        let Some(fin) = store.fin(fin_id) else {
            continue;
        };
        if fin.pcurve.is_none() {
            continue;
        }
        let face = store.fin_face(fin_id);
        let surface = store
            .face(face)
            .and_then(|f| f.surface)
            .and_then(|id| geo.surface(id))
            .cloned();
        // The recompute plan required two distinct faces, so neither fin is
        // a seam use: both sit on the low branch.
        let fitted = surface.and_then(|s| {
            fit_pcurve(
                &s,
                &recompute.curve,
                recompute.t_start,
                recompute.t_end,
                SeamSide::Low,
            )
            .ok()
        });
        if fitted.is_none() {
            notes.push(format!(
                "dropped the pcurve of {fin_id:?}: it could not be refit against the \
                 recomputed curve of {:?}",
                recompute.edge
            ));
        }
        let refit = fitted.map(|pcurve| geo.add_pcurve(pcurve));
        let fin = store.fins.get_mut(fin_id).expect("live fin");
        if let Some(stale) = std::mem::replace(&mut fin.pcurve, refit) {
            geo.pcurves.remove(stale);
        }
        if refit.is_some() {
            operations.push(HealOperation::PcurveRecomputed {
                fin: fin_id,
                edge: recompute.edge,
            });
        }
    }
}

// ---------------------------------------------------------------------
// Pcurve recompute (phase 2)
// ---------------------------------------------------------------------

/// One stale pcurve and its replacement, ready to swap in.
struct PcurveRefit {
    fin: EntityId<Fin>,
    edge: EntityId<Edge>,
    pcurve: Curve2,
}

fn plan_pcurve_refits(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    notes: &mut Vec<String>,
) -> Vec<PcurveRefit> {
    let mut plan = Vec::new();
    for face in store.faces_of_body(body) {
        let Some(surface) = store
            .face(face)
            .and_then(|f| f.surface)
            .and_then(|id| geo.surface(id))
        else {
            continue;
        };
        // Seam branches are assigned by per-face use order — the same
        // convention `attach_body_pcurves` derives them under.
        let mut uses: HashMap<EntityId<Edge>, usize> = HashMap::new();
        for loop_id in store.loops_of_face(face) {
            for &fin_id in store.fins_of_loop(loop_id) {
                let edge_id = store.fin_edge(fin_id);
                let count = uses.entry(edge_id).or_insert(0);
                let seam = if *count == 0 {
                    SeamSide::Low
                } else {
                    SeamSide::High
                };
                *count += 1;

                // A fin with no pcurve is left alone: deriving trim geometry
                // from scratch is `attach_body_pcurves`'s job.
                let Some(pcurve) = store
                    .fin(fin_id)
                    .and_then(|f| f.pcurve)
                    .and_then(|id| geo.pcurve(id))
                else {
                    continue;
                };
                let Some(edge) = store.edge(edge_id) else {
                    continue;
                };
                let Some(curve) = edge.curve.and_then(|id| geo.curve(id)) else {
                    continue;
                };
                // NaN-safe: only a provably advancing range is refittable.
                if !edge.t_start.is_finite()
                    || edge.t_end.partial_cmp(&edge.t_start) != Some(std::cmp::Ordering::Greater)
                {
                    continue;
                }
                let allowed = edge.tolerance.max(SYSTEM_RESOLUTION);
                let before = pcurve_departure(surface, curve, pcurve, edge.t_start, edge.t_end);
                if before <= allowed {
                    continue;
                }
                match fit_pcurve(surface, curve, edge.t_start, edge.t_end, seam) {
                    Ok(refit) => {
                        let after =
                            pcurve_departure(surface, curve, &refit, edge.t_start, edge.t_end);
                        if after < before {
                            plan.push(PcurveRefit {
                                fin: fin_id,
                                edge: edge_id,
                                pcurve: refit,
                            });
                        } else {
                            notes.push(format!(
                                "left the pcurve of {fin_id:?} alone: a refit would not \
                                 improve it ({before:.3e} mm -> {after:.3e} mm)"
                            ));
                        }
                    }
                    Err(e) => notes.push(format!("cannot refit the pcurve of {fin_id:?}: {e}")),
                }
            }
        }
    }
    plan
}

/// Largest gap between `surface.point(pcurve(t))` and `curve.point(t)` over
/// the edge range — the lockstep invariant `opensolid_brep::pcurve` defines.
/// Infinite when the pcurve evaluates to a non-finite point, which is
/// maximally broken.
fn pcurve_departure(
    surface: &Surface3,
    curve: &Curve3,
    pcurve: &Curve2,
    t_start: f64,
    t_end: f64,
) -> f64 {
    let mut max = 0.0f64;
    for i in 0..RECOMPUTE_MEASURE_SAMPLES {
        let t = t_start + (t_end - t_start) * (i as f64) / (RECOMPUTE_MEASURE_SAMPLES - 1) as f64;
        let uv = pcurve.point(t);
        if !uv.coords.iter().all(|c| c.is_finite()) {
            return f64::INFINITY;
        }
        let gap = (surface.point(uv.x, uv.y) - curve.point(t)).norm();
        if gap > max {
            max = gap;
        }
    }
    max
}

fn apply_pcurve_refits(
    store: &mut TopologyStore,
    geo: &mut GeometryStore,
    plan: Vec<PcurveRefit>,
    operations: &mut Vec<HealOperation>,
) {
    for refit in plan {
        if store.fin(refit.fin).is_none() {
            continue;
        }
        let new_id = geo.add_pcurve(refit.pcurve);
        let fin = store.fins.get_mut(refit.fin).expect("checked above");
        if let Some(stale) = fin.pcurve.replace(new_id) {
            geo.pcurves.remove(stale);
        }
        operations.push(HealOperation::PcurveRecomputed {
            fin: refit.fin,
            edge: refit.edge,
        });
    }
}

// ---------------------------------------------------------------------
// Orientation repair
// ---------------------------------------------------------------------

/// Two-colour each shell's face-adjacency graph so that every pair of mated
/// fins traverses its edge in opposite directions, and return the faces that
/// must flip. `None` means the constraint is unsatisfiable (an odd cycle —
/// the shell is non-orientable, or its mate links are corrupt), in which case
/// no flip is safe.
fn plan_face_flips(
    store: &TopologyStore,
    body: EntityId<Body>,
    notes: &mut Vec<String>,
) -> Option<Vec<EntityId<Face>>> {
    let mut flips = Vec::new();
    for &shell in store.shells_of_body(body) {
        let faces = store.faces_of_shell(shell).to_vec();
        let mut colour: HashMap<EntityId<Face>, bool> = HashMap::new();
        for &seed in &faces {
            if colour.contains_key(&seed) {
                continue;
            }
            // One connected component. Its seed is held fixed, so the whole
            // component may come out globally reversed; the volume-sign pass
            // below is what settles that.
            let mut component = vec![seed];
            colour.insert(seed, false);
            let mut queue = vec![seed];
            while let Some(face) = queue.pop() {
                let flip_a = colour[&face];
                for fin_id in fins_of_face(store, face) {
                    let Some(mate_id) = store.fin_mate(fin_id) else {
                        continue;
                    };
                    let (Some(fin), Some(mate)) = (store.fin(fin_id), store.fin(mate_id)) else {
                        continue;
                    };
                    if fin.edge != mate.edge {
                        // Corrupt mate link; `check` reports it, and it makes
                        // the colouring meaningless.
                        notes.push(format!(
                            "orientation repair skipped: {fin_id:?} is mated across two edges"
                        ));
                        return None;
                    }
                    let neighbour = store.fin_face(mate_id);
                    // Consistent means the effective senses differ, where
                    // "effective" is the authored sense XOR the face's flip.
                    let same = forward(fin.sense) == forward(mate.sense);
                    let flip_b = same != flip_a;
                    match colour.get(&neighbour) {
                        Some(&existing) if existing != flip_b => {
                            notes.push(format!(
                                "orientation repair skipped: {shell:?} is not consistently \
                                 orientable (conflict at {neighbour:?})"
                            ));
                            return None;
                        }
                        Some(_) => {}
                        None => {
                            colour.insert(neighbour, flip_b);
                            component.push(neighbour);
                            queue.push(neighbour);
                        }
                    }
                }
            }
            // Either colour class is a valid answer; flipping the smaller one
            // keeps the operation log proportional to the actual defect.
            let flipped = component.iter().filter(|f| colour[f]).count();
            let invert = flipped * 2 > component.len();
            for face in component {
                if colour[&face] != invert {
                    flips.push(face);
                }
            }
        }
    }
    Some(flips)
}

/// Shells left enclosing a signed volume of the wrong sign for their
/// [`ShellOrientation`] once `flips` is applied, with their face counts.
///
/// A file can be perfectly self-consistent and still inside out — every face
/// authored backwards agrees with every other — which no combinatorial check
/// can see. Only a geometric measurement can, so this is the one pass that
/// looks at the surfaces rather than the graph.
///
/// The measurement is virtual: flipping a face's sense negates its
/// tessellated contribution exactly (planar faces are triangulated about the
/// sense-adjusted normal, quadric grids wound by the same flag), so the
/// post-flip total is a signed sum over the pre-flip contributions. That
/// keeps [`HealStrategy::ReportOnly`] honest — it reports the same reversal
/// an `Auto` run would apply, without touching the body.
///
/// The flag the tessellator winds each face by is the *less* authoritative of
/// a face's two orientation records: [`reconcile_face_senses`] later corrects
/// it against the outer loop's winding, never the other way round. So
/// wherever that winding is readable, the contribution is signed by the
/// winding-derived sense instead — the sum then measures the body as it will
/// stand once the flags are reconciled, and a file whose flags lie cannot
/// talk this pass into reversing loop windings that were never wrong
/// (of-8jqc). A face whose winding is unreadable (no pcurves — the import
/// path attaches them only after healing) falls back to its flag, which is
/// all there is to read.
///
/// Only structurally closed shells are measured: an open shell encloses no
/// volume, so the sum's sign would be an artefact of where the hole is.
fn plan_shell_reversals(
    store: &TopologyStore,
    geo: &GeometryStore,
    body: EntityId<Body>,
    flips: &[EntityId<Face>],
    notes: &mut Vec<String>,
) -> Vec<(EntityId<Shell>, usize)> {
    let mut reversals = Vec::new();
    for &shell in store.shells_of_body(body) {
        let faces = store.faces_of_shell(shell).to_vec();
        if !shell_is_closed(store, &faces) {
            continue;
        }
        let mut total = 0.0;
        let mut magnitude = 0.0;
        let mut measurable = true;
        for &face in &faces {
            let Ok(mesh) = tessellate_face(store, geo, face, &TessellationOptions::default())
            else {
                // NURBS patches and trimmed quadrics have no tessellator yet;
                // without every face the sum is not a closed volume.
                notes.push(format!(
                    "shell orientation left as authored: {face:?} cannot be tessellated"
                ));
                measurable = false;
                break;
            };
            let contribution: f64 = mesh
                .indices
                .iter()
                .map(|tri| {
                    let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                    a.dot(&b.cross(&c)) / 6.0
                })
                .sum();
            let mut signed = if flips.contains(&face) {
                -contribution
            } else {
                contribution
            };
            // Winding-vs-flag disagreement is invariant under a flip (both
            // records negate together), so this composes with the virtual
            // flip above to give exactly the post-flip, post-reconcile sign.
            if winding_contradicts_sense_flag(store, geo, face) {
                signed = -signed;
            }
            total += signed;
            magnitude += signed.abs();
        }
        if !measurable || magnitude <= 0.0 {
            continue;
        }
        // An outward shell bounds positive volume; a void shell's normals
        // point into the cavity, so it bounds negative volume.
        let want_positive =
            store.shell(shell).expect("live shell").orientation == ShellOrientation::Outward;
        let inverted = if want_positive {
            total < 0.0
        } else {
            total > 0.0
        };
        // Only a decisive sign counts: a sum that is numerical noise against
        // the faces' own magnitudes says the shell does not close, not that
        // it is inside out.
        if inverted && total.abs() > magnitude * 1e-6 {
            reversals.push((shell, faces.len()));
        }
    }
    reversals
}

/// Correct every face whose surface sense contradicts the winding of its own
/// outer loop, and report what was corrected.
///
/// # The defect this repairs
///
/// A face carries its orientation twice over: in [`FaceSense`], which says
/// whether the face normal is `S_u × S_v` or its reverse, and in the winding
/// of its outer loop, which is counterclockwise in `(u, v)` exactly when that
/// normal is `S_u × S_v`. `ADVANCED_FACE.same_sense` maps straight onto the
/// first; the loop the file authors is the second. A producer with
/// untrustworthy sense flags writes them disagreeing, and that face is then
/// inconsistent in a way *no* pass above can see: flipping the flag changes no
/// fin sense, so the two-colouring of [`plan_face_flips`] finds every mated
/// pair still traversing its edge in opposite directions and plans nothing.
/// Both measurement paths then break — the tessellator winds that face's
/// triangles against its neighbours', and
/// [`brep_mass_properties`](crate::brep_mass_properties) refuses a periodic
/// face outright, because the branch a seam fin takes is read off the flag
/// (`OpenParameterLoop`, of-hrgt).
///
/// # Why the flag yields to the loop, and not the other way round
///
/// The loops carry a constraint the flag does not: mated fins must traverse
/// their shared edge in opposite directions, which ties every face of a shell
/// to its neighbours. Reversing a loop to suit its flag would break that tie
/// and manufacture the very defect [`plan_face_flips`] exists to repair, on a
/// shell that just came through it clean. The flag constrains one face and
/// nothing else, so it is the one free to move. This is the argument
/// [`choose_outer_bounds`](super::read) already makes for `FACE_OUTER_BOUND`:
/// files get their flags wrong, and the geometry does not.
///
/// A face whose winding is unreadable — no surface, no outer loop, fins
/// without pcurves, or a loop enclosing no measurable area — is left exactly
/// as authored. Nothing is concluded from a winding that was not measured.
///
/// # Where this runs
///
/// Not from [`GeometryHealer::heal`]: reading a winding needs the fins'
/// pcurves, which the reader derives *after* healing has settled which edge
/// each fin uses (see
/// [`finish_exact_body`](super::read)). So this is the reader's last repair
/// rather than the healer's, and it obeys the same
/// [`HealStrategy`] — planned under [`HealStrategy::ReportOnly`], applied only
/// when the strategy mutates and orients at all.
pub fn reconcile_face_senses(
    body: EntityId<Body>,
    store: &mut TopologyStore,
    geo: &GeometryStore,
    strategy: HealStrategy,
) -> Vec<HealOperation> {
    if !strategy.orients() {
        return Vec::new();
    }
    let mut corrections: Vec<(EntityId<Face>, FaceSense)> = Vec::new();
    for &shell in store.shells_of_body(body) {
        for &face_id in store.faces_of_shell(shell) {
            let Some(face) = store.face(face_id) else {
                continue;
            };
            let (Some(surface), Some(outer)) =
                (face.surface.and_then(|id| geo.surface(id)), face.outer_loop)
            else {
                continue;
            };
            let Some(twice_signed_area) = store.loop_winding(geo, surface, outer) else {
                continue;
            };
            let wants_positive = twice_signed_area > 0.0;
            if wants_positive == (face.sense == FaceSense::Positive) {
                continue;
            }
            let sense = if wants_positive {
                FaceSense::Positive
            } else {
                FaceSense::Negative
            };
            corrections.push((face_id, sense));
        }
    }
    if strategy.applies() {
        for &(face_id, sense) in &corrections {
            if let Some(face) = store.faces.get_mut(face_id) {
                face.sense = sense;
            }
        }
    }
    corrections
        .into_iter()
        .map(|(face, sense)| HealOperation::FaceSenseCorrected { face, sense })
        .collect()
}

/// Whether `face`'s readable outer-loop winding contradicts its sense flag —
/// the disagreement [`reconcile_face_senses`] repairs, read without repairing
/// it. `false` wherever the winding is unreadable: nothing is concluded from
/// a winding that was not measured.
fn winding_contradicts_sense_flag(
    store: &TopologyStore,
    geo: &GeometryStore,
    face_id: EntityId<Face>,
) -> bool {
    let Some(face) = store.face(face_id) else {
        return false;
    };
    let (Some(surface), Some(outer)) = (face.surface.and_then(|id| geo.surface(id)), face.outer_loop)
    else {
        return false;
    };
    let Some(twice_signed_area) = store.loop_winding(geo, surface, outer) else {
        return false;
    };
    (twice_signed_area > 0.0) != (face.sense == FaceSense::Positive)
}

/// Reverse any shell left enclosing a signed volume of the wrong sign for its
/// [`ShellOrientation`], and report what was reversed.
///
/// This is [`plan_shell_reversals`] run where its measurement is finally
/// trustworthy: after the reader has attached pcurves and
/// [`reconcile_face_senses`] has settled every readable flag against its
/// loop's winding. The same pass inside [`GeometryHealer::heal`] runs before
/// either — on the import path no fin has a pcurve yet, so it can only read
/// the authored flags, and a file whose flags lie in the majority talks it
/// into reversing correctly-wound shells (of-8jqc). Measured here instead,
/// the winding-authoritative sum catches both that mid-heal wrong turn and
/// the file that really is inside out — including the sewn, self-consistent
/// one that never brings the healer in at all, which no combinatorial check
/// can see.
///
/// Obeys the same [`HealStrategy`] conventions as [`reconcile_face_senses`]:
/// planned (and reported) under [`HealStrategy::ReportOnly`], applied only
/// when the strategy mutates. Applying reverses each shell through
/// [`flip_face`], which flips flag and winding together, so the reversed
/// shell needs no second reconciliation.
pub fn fix_shell_volume_signs(
    body: EntityId<Body>,
    store: &mut TopologyStore,
    geo: &GeometryStore,
    strategy: HealStrategy,
    notes: &mut Vec<String>,
) -> Vec<HealOperation> {
    if !strategy.orients() {
        return Vec::new();
    }
    let reversals = plan_shell_reversals(store, geo, body, &[], notes);
    if strategy.applies() {
        for &(shell, _) in &reversals {
            for face in store.faces_of_shell(shell).to_vec() {
                flip_face(store, face);
            }
        }
    }
    reversals
        .into_iter()
        .map(|(shell, faces)| HealOperation::ShellReversed { shell, faces })
        .collect()
}

/// Whether every edge these faces use is shared by exactly two of them —
/// the structural closure that makes an enclosed volume meaningful.
fn shell_is_closed(store: &TopologyStore, faces: &[EntityId<Face>]) -> bool {
    faces.iter().all(|&face| {
        store
            .edges_of_face(face)
            .into_iter()
            .all(|edge| store.fins_of_edge(edge).len() == 2)
    })
}

/// Reverse one face: its surface sense, and the traversal of every loop on
/// it. Both together, so the face's outward normal and its loops' winding
/// stay consistent with each other.
fn flip_face(store: &mut TopologyStore, face: EntityId<Face>) {
    let Some(face_ref) = store.faces.get_mut(face) else {
        return;
    };
    face_ref.sense = match face_ref.sense {
        FaceSense::Positive => FaceSense::Negative,
        FaceSense::Negative => FaceSense::Positive,
    };
    for loop_id in store.loops_of_face(face) {
        let mut fins = store.loops.get(loop_id).expect("live loop").fins.clone();
        if fins.is_empty() {
            continue;
        }
        fins.reverse();
        let n = fins.len();
        for (i, &fin_id) in fins.iter().enumerate() {
            let fin = store.fins.get_mut(fin_id).expect("live fin");
            fin.sense = fin.sense.opposite();
            fin.next = Some(fins[(i + 1) % n]);
            fin.prev = Some(fins[(i + n - 1) % n]);
        }
        store.loops.get_mut(loop_id).expect("live loop").fins = fins;
    }
}

fn forward(sense: FinSense) -> bool {
    sense == FinSense::Forward
}

// ---------------------------------------------------------------------
// Shared helpers
// ---------------------------------------------------------------------

/// Recover a body's shell genus from the Euler-Poincaré formula.
///
/// STEP carries no genus, and healing changes the vertex and edge counts it
/// would be derived from, so both the reader (at build time) and the healer
/// (after any topology change) recompute it here. Any residual genus is
/// attributed to the first shell: a handled *void* would mis-assign, and then
/// fail `check` into the mesh fallback rather than import wrongly. An odd or
/// negative implied genus is not a genus at all — it means the topology is
/// inconsistent, and the formula is left to fail in `check`.
pub(crate) fn recover_genus(store: &mut TopologyStore, body: EntityId<Body>) {
    let Some(&shell) = store.body(body).and_then(|b| b.shells.first()) else {
        return;
    };
    let counts = store.euler_counts(body);
    // `euler_counts` folds the current genus into `counts.genus`; the formula
    // below solves for it afresh, so start from the un-attributed total.
    let others: i64 = store
        .shells_of_body(body)
        .iter()
        .skip(1)
        .map(|&s| store.shell(s).expect("live shell").genus as i64)
        .sum();
    let euler =
        counts.vertices as i64 - counts.edges as i64 + counts.faces as i64 - counts.rings as i64;
    let genus_x2 = 2 * counts.shells as i64 - euler - 2 * others;
    if genus_x2 >= 0 && genus_x2 % 2 == 0 {
        store.shells.get_mut(shell).expect("live shell").genus = (genus_x2 / 2) as u32;
    }
}

fn fins_of_face(store: &TopologyStore, face: EntityId<Face>) -> Vec<EntityId<Fin>> {
    store
        .loops_of_face(face)
        .into_iter()
        .flat_map(|loop_id| store.fins_of_loop(loop_id).to_vec())
        .collect()
}

/// Every vertex reachable from `body`, deduplicated, in a deterministic order.
fn body_vertices(store: &TopologyStore, body: EntityId<Body>) -> Vec<EntityId<Vertex>> {
    let mut out = Vec::new();
    for face in store.faces_of_body(body) {
        for loop_id in store.loops_of_face(face) {
            if let Some(v) = store.loop_(loop_id).and_then(|l| l.vertex)
                && !out.contains(&v)
            {
                out.push(v);
            }
            for &fin_id in store.fins_of_loop(loop_id) {
                let edge_id = store.fin_edge(fin_id);
                let Some(edge) = store.edge(edge_id) else {
                    continue;
                };
                for v in [edge.start_vertex, edge.end_vertex] {
                    if !out.contains(&v) {
                        out.push(v);
                    }
                }
            }
        }
    }
    out
}

/// Every edge reachable from `body`, deduplicated, in a deterministic order.
fn body_edges(store: &TopologyStore, body: EntityId<Body>) -> Vec<EntityId<Edge>> {
    let mut out = Vec::new();
    for face in store.faces_of_body(body) {
        for edge in store.edges_of_face(face) {
            if !out.contains(&edge) {
                out.push(edge);
            }
        }
    }
    out
}

/// Vertex → the single shell using it. `None` for a vertex already shared
/// between shells: it is exactly the case healing must not make worse.
fn vertex_shells(
    store: &TopologyStore,
    body: EntityId<Body>,
) -> HashMap<EntityId<Vertex>, Option<EntityId<Shell>>> {
    let mut out: HashMap<EntityId<Vertex>, Option<EntityId<Shell>>> = HashMap::new();
    for &shell in store.shells_of_body(body) {
        for &face in store.faces_of_shell(shell) {
            for edge_id in store.edges_of_face(face) {
                let Some(edge) = store.edge(edge_id) else {
                    continue;
                };
                for v in [edge.start_vertex, edge.end_vertex] {
                    out.entry(v)
                        .and_modify(|slot| {
                            if *slot != Some(shell) {
                                *slot = None;
                            }
                        })
                        .or_insert(Some(shell));
                }
            }
        }
    }
    out
}

/// Edge → the single shell using it, with the same `None` convention as
/// [`vertex_shells`].
fn edge_shells(
    store: &TopologyStore,
    body: EntityId<Body>,
) -> HashMap<EntityId<Edge>, Option<EntityId<Shell>>> {
    let mut out: HashMap<EntityId<Edge>, Option<EntityId<Shell>>> = HashMap::new();
    for &shell in store.shells_of_body(body) {
        for &face in store.faces_of_shell(shell) {
            for edge in store.edges_of_face(face) {
                out.entry(edge)
                    .and_modify(|slot| {
                        if *slot != Some(shell) {
                            *slot = None;
                        }
                    })
                    .or_insert(Some(shell));
            }
        }
    }
    out
}

/// Union-find over vertex indices, with path halving and union by size.
struct DisjointSet {
    parent: Vec<usize>,
    size: Vec<usize>,
}

impl DisjointSet {
    fn new(n: usize) -> Self {
        Self {
            parent: (0..n).collect(),
            size: vec![1; n],
        }
    }

    fn find(&mut self, mut i: usize) -> usize {
        while self.parent[i] != i {
            self.parent[i] = self.parent[self.parent[i]];
            i = self.parent[i];
        }
        i
    }

    fn union(&mut self, a: usize, b: usize) {
        let (mut ra, mut rb) = (self.find(a), self.find(b));
        if ra == rb {
            return;
        }
        if self.size[ra] < self.size[rb] {
            std::mem::swap(&mut ra, &mut rb);
        }
        self.parent[rb] = ra;
        self.size[ra] += self.size[rb];
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use opensolid_brep::{BodyType, Curve3, LoopType, Surface3, tessellate_body};
    use opensolid_core::Vector3;

    /// A unit block built face-by-face, each face optionally authoring its
    /// own copy of the vertices and edges it shares with its neighbours.
    ///
    /// `jitter` displaces each face's private copy of a shared corner by that
    /// distance along a per-face direction, modelling the last-decimal
    /// disagreement a real exporter leaves behind.
    struct BlockFixture {
        store: TopologyStore,
        geo: GeometryStore,
        body: EntityId<Body>,
    }

    const CORNERS: [[f64; 3]; 8] = [
        [0.0, 0.0, 0.0],
        [1.0, 0.0, 0.0],
        [1.0, 1.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 1.0],
        [1.0, 1.0, 1.0],
        [0.0, 1.0, 1.0],
    ];

    /// Vertex cycles counterclockwise viewed from outside, with the outward
    /// normal each implies.
    const FACES: [([usize; 4], [f64; 3]); 6] = [
        ([0, 3, 2, 1], [0.0, 0.0, -1.0]),
        ([4, 5, 6, 7], [0.0, 0.0, 1.0]),
        ([0, 1, 5, 4], [0.0, -1.0, 0.0]),
        ([1, 2, 6, 5], [1.0, 0.0, 0.0]),
        ([2, 3, 7, 6], [0.0, 1.0, 0.0]),
        ([3, 0, 4, 7], [-1.0, 0.0, 0.0]),
    ];

    /// Build a block where every face carries its own vertices and edges —
    /// an unsewn shell, as an exporter that never merged its boundaries
    /// writes. With `jitter = 0` the duplicates are exactly coincident;
    /// with `jitter > 0` they disagree by that distance.
    fn unsewn_block(jitter: f64, reversed_faces: &[usize]) -> BlockFixture {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        for (f, (cycle, normal)) in FACES.iter().enumerate() {
            let normal = Vector3::new(normal[0], normal[1], normal[2]);
            // A per-face nudge direction that is not parallel to any other
            // face's, so coincident corners scatter rather than stack.
            let nudge = Vector3::new(
                ((f + 1) as f64 * 0.7).sin(),
                ((f + 1) as f64 * 1.3).sin(),
                ((f + 1) as f64 * 2.1).sin(),
            )
            .normalize()
                * jitter;

            let points: Vec<Point3> = cycle
                .iter()
                .map(|&c| Point3::new(CORNERS[c][0], CORNERS[c][1], CORNERS[c][2]) + nudge)
                .collect();
            let vertices: Vec<_> = points
                .iter()
                .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
                .collect();

            let face = store.create_face(shell, FaceSense::Positive);
            let surface =
                geo.add_surface(Surface3::plane(points[0], normal).expect("axis-aligned plane"));
            store.faces.get_mut(face).expect("live face").surface = Some(surface);

            let mut fins = Vec::new();
            for k in 0..4 {
                let (a, b) = (k, (k + 1) % 4);
                let direction = points[b] - points[a];
                let length = direction.norm();
                let curve = geo.add_curve(
                    Curve3::line(points[a], direction / length).expect("nonzero direction"),
                );
                let edge = store.create_edge_with_curve(
                    vertices[a],
                    vertices[b],
                    SYSTEM_RESOLUTION,
                    curve,
                    0.0,
                    length,
                );
                fins.push((edge, FinSense::Forward));
            }
            if reversed_faces.contains(&f) {
                reverse_authored_face(&mut store, face, &mut fins);
            }
            store.create_loop(face, LoopType::Outer, &fins);
        }
        recover_genus(&mut store, body);
        BlockFixture { store, geo, body }
    }

    /// An unsewn block whose faces disagree about their *corners* but agree
    /// about their *edges*: each face's private copy of a corner is displaced
    /// by `jitter`, while the curve every copy of an edge carries is the one
    /// true line through the undisplaced corners.
    ///
    /// This is what a STEP file actually looks like — the randomized
    /// campaign's `unsew` duplicates only the topological records and leaves
    /// the `LINE`s shared — and unlike [`unsewn_block`] it separates the two
    /// gaps a merge has to account for. A displaced corner is already off the
    /// end of its own edge's curve, by the perpendicular part of its
    /// displacement, and the reader records that as the vertex's tolerance
    /// (`read.rs`). Merging then moves it again, to the cluster's centroid.
    /// The survivor owes both, which is the accounting of-bbh8 was about.
    fn unsewn_block_on_shared_curves(jitter: f64) -> BlockFixture {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        for (f, (cycle, normal)) in FACES.iter().enumerate() {
            let normal = Vector3::new(normal[0], normal[1], normal[2]);
            let nudge = Vector3::new(
                ((f + 1) as f64 * 0.7).sin(),
                ((f + 1) as f64 * 1.3).sin(),
                ((f + 1) as f64 * 2.1).sin(),
            )
            .normalize()
                * jitter;

            let base: Vec<Point3> = cycle
                .iter()
                .map(|&c| Point3::new(CORNERS[c][0], CORNERS[c][1], CORNERS[c][2]))
                .collect();
            let points: Vec<Point3> = base.iter().map(|&p| p + nudge).collect();
            let vertices: Vec<_> = points
                .iter()
                .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
                .collect();

            let face = store.create_face(shell, FaceSense::Positive);
            // The true plane, through the undisplaced corners: the curves lie
            // exactly on it, so nothing here is an `EdgeOffSurface`.
            let surface =
                geo.add_surface(Surface3::plane(base[0], normal).expect("axis-aligned plane"));
            store.faces.get_mut(face).expect("live face").surface = Some(surface);

            let mut fins = Vec::new();
            for k in 0..4 {
                let (a, b) = (k, (k + 1) % 4);
                let direction = base[b] - base[a];
                let dir = direction / direction.norm();
                let curve = Curve3::line(base[a], dir).expect("nonzero direction");
                // Trim where the displaced corners project onto the shared
                // line, which is what the reader recovers for them.
                let t_start = (points[a] - base[a]).dot(&dir);
                let t_end = (points[b] - base[a]).dot(&dir);
                for (&vertex, point, t) in [
                    (&vertices[a], points[a], t_start),
                    (&vertices[b], points[b], t_end),
                ] {
                    let residual = (point - curve.point(t)).norm();
                    let v = store.vertices.get_mut(vertex).expect("live vertex");
                    v.tolerance = v.tolerance.max(residual);
                }
                let curve = geo.add_curve(curve);
                let edge = store.create_edge_with_curve(
                    vertices[a],
                    vertices[b],
                    SYSTEM_RESOLUTION,
                    curve,
                    t_start,
                    t_end,
                );
                fins.push((edge, FinSense::Forward));
            }
            store.create_loop(face, LoopType::Outer, &fins);
        }
        recover_genus(&mut store, body);
        BlockFixture { store, geo, body }
    }

    /// Build a properly sewn block, then break only the orientation of the
    /// listed faces.
    fn sewn_block(reversed_faces: &[usize]) -> BlockFixture {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        let points: Vec<Point3> = CORNERS
            .iter()
            .map(|c| Point3::new(c[0], c[1], c[2]))
            .collect();
        let vertices: Vec<_> = points
            .iter()
            .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
            .collect();
        let mut edges: HashMap<(usize, usize), EntityId<Edge>> = HashMap::new();

        for (f, (cycle, normal)) in FACES.iter().enumerate() {
            let normal = Vector3::new(normal[0], normal[1], normal[2]);
            let face = store.create_face(shell, FaceSense::Positive);
            let surface = geo.add_surface(
                Surface3::plane(points[cycle[0]], normal).expect("axis-aligned plane"),
            );
            store.faces.get_mut(face).expect("live face").surface = Some(surface);

            let mut fins = Vec::new();
            for k in 0..4 {
                let (from, to) = (cycle[k], cycle[(k + 1) % 4]);
                let key = if from < to { (from, to) } else { (to, from) };
                let edge = *edges.entry(key).or_insert_with(|| {
                    let direction = points[key.1] - points[key.0];
                    let length = direction.norm();
                    let curve = geo.add_curve(
                        Curve3::line(points[key.0], direction / length).expect("nonzero direction"),
                    );
                    store.create_edge_with_curve(
                        vertices[key.0],
                        vertices[key.1],
                        SYSTEM_RESOLUTION,
                        curve,
                        0.0,
                        length,
                    )
                });
                let sense = if from == key.0 {
                    FinSense::Forward
                } else {
                    FinSense::Reversed
                };
                fins.push((edge, sense));
            }
            if reversed_faces.contains(&f) {
                reverse_authored_face(&mut store, face, &mut fins);
            }
            store.create_loop(face, LoopType::Outer, &fins);
        }
        recover_genus(&mut store, body);
        BlockFixture { store, geo, body }
    }

    /// Author one face's *whole* use backwards — the surface sense and the
    /// loop traversal together — which is how a misoriented face arrives in
    /// a real file: the exporter emitted the face use against the shell, not
    /// half of it. (A face whose sense disagrees with its own winding is a
    /// different, geometry-only defect that `check` cannot see; repairing it
    /// is of-3qy.15.)
    fn reverse_authored_face(
        store: &mut TopologyStore,
        face: EntityId<Face>,
        fins: &mut [(EntityId<Edge>, FinSense)],
    ) {
        fins.reverse();
        for (_, sense) in fins.iter_mut() {
            *sense = sense.opposite();
        }
        store.faces.get_mut(face).expect("live face").sense = FaceSense::Negative;
    }

    fn signed_volume(fixture: &BlockFixture) -> f64 {
        let mesh = tessellate_body(
            &fixture.store,
            &fixture.geo,
            fixture.body,
            &TessellationOptions::default(),
        )
        .expect("block tessellates");
        mesh.indices
            .iter()
            .map(|tri| {
                let [a, b, c] = tri.map(|i| mesh.positions[i].coords);
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    #[test]
    fn unsewn_block_is_invalid_before_healing() {
        let f = unsewn_block(0.0, &[]);
        let failures = f.store.check(f.body);
        assert!(
            failures
                .iter()
                .any(|x| matches!(x, CheckFailure::OpenEdgeInClosedShell { .. })),
            "an unsewn shell's edges each have one fin: {failures:?}"
        );
        assert_eq!(f.store.vertices.len(), 24, "6 faces x 4 private corners");
        assert_eq!(f.store.edges.len(), 24, "6 faces x 4 private edges");
    }

    #[test]
    fn healing_sews_an_exactly_coincident_block() {
        let mut f = unsewn_block(0.0, &[]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(f.store.check(f.body), vec![]);
        assert_eq!(f.store.vertices.len(), 8, "24 corners collapse to 8");
        assert_eq!(f.store.edges.len(), 12, "24 half-edges weld to 12");
        for edge in body_edges(&f.store, f.body) {
            assert_eq!(
                f.store.fins_of_edge(edge).len(),
                2,
                "{edge:?} must be shared by two faces after sewing"
            );
        }
        assert!((signed_volume(&f) - 1.0).abs() < 1e-9, "unit block volume");
    }

    #[test]
    fn healing_closes_a_real_gap_and_records_the_tolerance() {
        // A micron of disagreement on a 1 mm block — inside the derived
        // tolerance (1e-5 of the 1.73 mm diagonal), as export round-off is.
        let mut f = unsewn_block(1e-6, &[]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(f.store.vertices.len(), 8);
        assert_eq!(f.store.edges.len(), 12);

        // The closed distance must be absorbed as tolerance, not snapped away.
        for vertex in body_vertices(&f.store, f.body) {
            let v = f.store.vertex(vertex).expect("live vertex");
            assert!(v.is_tolerant(), "{vertex:?} should carry the closed gap");
            assert!(v.tolerance <= MAX_ALLOWED_TOLERANCE);
        }
        assert!(
            result
                .operations
                .iter()
                .any(|op| matches!(op, HealOperation::ToleranceElevated { .. })),
            "the elevation must be reported: {:?}",
            result.operations
        );
    }

    /// A merged vertex must cover the gap its members were *already* carrying
    /// on top of the distance it moves them — of-bbh8.
    ///
    /// The input is valid geometrically as well as topologically: every corner
    /// copy is within its own tolerance of the ends of its edges' curves. What
    /// healing changes is where the survivor sits, and elevating it to
    /// `max(existing, displacement)` left it short of the endpoints of the
    /// curves its merged-away members brought with them — a body the healer
    /// reported as fully repaired and `check_geometry` then rejected.
    #[test]
    fn a_merged_vertex_covers_the_gap_its_members_already_had() {
        let mut f = unsewn_block_on_shared_curves(1e-6);
        let before = f.store.check_geometry(&f.geo, f.body);
        assert!(
            before.is_empty(),
            "the fixture must be geometrically valid before healing: {before:#?}"
        );

        // The plan, before it is applied: the elevation the old rule would
        // have made is `max(displacement, what the members already carried)`,
        // and every merge here needs strictly more than that. Without this the
        // test would still pass on a fixture that never exercised the
        // difference.
        let mut notes = Vec::new();
        let max_gap = derived_max_gap(&f.store, f.body);
        for merge in plan_vertex_merges(&f.store, &f.geo, f.body, max_gap, &mut notes) {
            let members = std::iter::once(merge.kept).chain(merge.merged.iter().copied());
            let carried = members
                .map(|v| f.store.vertex(v).expect("live vertex").tolerance)
                .fold(SYSTEM_RESOLUTION, f64::max);
            assert!(
                merge.tolerance > carried.max(merge.gap),
                "{:?}: tolerance {:.6e} is what max(carried {:.6e}, gap {:.6e}) already \
                 gave — the fixture no longer distinguishes the two rules",
                merge.kept,
                merge.tolerance,
                carried,
                merge.gap
            );
        }

        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(f.store.vertices.len(), 8);
        assert_eq!(f.store.edges.len(), 12);

        let failures = f.store.check_geometry(&f.geo, f.body);
        assert!(
            failures.is_empty(),
            "a healed body must satisfy the geometric check too: {failures:#?}"
        );
    }

    #[test]
    fn gap_wider_than_the_tolerance_is_left_alone() {
        // A 0.2 mm gap on a 1 mm block is a modelling error, not export
        // round-off: healing must not weld it shut.
        let mut f = unsewn_block(0.2, &[]);
        let before = f.store.check(f.body).len();
        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(!result.healed());
        assert_eq!(f.store.vertices.len(), 24, "nothing merged");
        assert!(f.store.check(f.body).len() >= before.min(1));
    }

    #[test]
    fn orientation_repair_fixes_a_reversed_face() {
        let mut f = sewn_block(&[3]);
        let before = f.store.check(f.body);
        assert!(
            before
                .iter()
                .any(|x| matches!(x, CheckFailure::InconsistentOrientation { .. })),
            "one reversed face disagrees with its four neighbours: {before:?}"
        );

        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(op, HealOperation::FaceReoriented { .. }))
                .count(),
            1,
            "only the offending face flips: {:?}",
            result.operations
        );
        assert!(
            (signed_volume(&f) - 1.0).abs() < 1e-9,
            "the repaired block still bounds +1 mm^3, not -1"
        );
    }

    #[test]
    fn orientation_repair_uprights_an_inside_out_shell() {
        // Every face but one reversed: two-colouring alone would happily
        // "fix" this by reversing the single odd face out, leaving a block
        // that encloses -1 mm^3. The volume-sign pass must catch it.
        let mut f = sewn_block(&[0, 1, 2, 3, 4]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert!(
            result
                .operations
                .iter()
                .any(|op| matches!(op, HealOperation::ShellReversed { .. })),
            "the shell must be reported reversed: {:?}",
            result.operations
        );
        assert!(
            (signed_volume(&f) - 1.0).abs() < 1e-9,
            "volume must come out positive, got {}",
            signed_volume(&f)
        );
    }

    #[test]
    fn a_correctly_oriented_body_is_untouched() {
        let mut f = sewn_block(&[]);
        assert_eq!(f.store.check(f.body), vec![]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(result.operations.is_empty(), "{:?}", result.operations);
        assert_eq!(f.store.check(f.body), vec![]);
        assert!((signed_volume(&f) - 1.0).abs() < 1e-9);
    }

    #[test]
    fn gaps_and_orientation_heal_together() {
        let mut f = unsewn_block(1e-6, &[2, 5]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(f.store.vertices.len(), 8);
        assert_eq!(f.store.edges.len(), 12);
        assert!((signed_volume(&f) - 1.0).abs() < 1e-6);
    }

    #[test]
    fn minimal_strategy_leaves_orientation_alone() {
        let mut f = sewn_block(&[3]);
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions {
                strategy: HealStrategy::Minimal,
                ..HealOptions::default()
            },
        );
        assert!(!result.healed(), "Minimal must not touch orientation");
        assert!(
            result
                .operations
                .iter()
                .all(|op| !matches!(op, HealOperation::FaceReoriented { .. }))
        );
    }

    #[test]
    fn report_only_plans_without_mutating() {
        let mut f = unsewn_block(0.0, &[]);
        let vertices_before = f.store.vertices.len();
        let edges_before = f.store.edges.len();
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions {
                strategy: HealStrategy::ReportOnly,
                ..HealOptions::default()
            },
        );
        assert_eq!(f.store.vertices.len(), vertices_before);
        assert_eq!(f.store.edges.len(), edges_before);
        assert_eq!(result.remaining, result.failures_before);
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(op, HealOperation::VerticesMerged { .. }))
                .count(),
            8,
            "one merge per block corner: {:?}",
            result.operations
        );
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(op, HealOperation::EdgesWelded { .. }))
                .count(),
            12,
            "one weld per block edge: {:?}",
            result.operations
        );
    }

    /// The orientation pass must be a dry run under `ReportOnly` too — it is
    /// the pass that mutates faces the caller can still see, so reporting a
    /// flip and then performing it would be the worst of both.
    #[test]
    fn report_only_plans_orientation_without_flipping_anything() {
        let mut f = sewn_block(&[3]);
        let snapshot = |f: &BlockFixture| -> Vec<(FaceSense, Vec<FinSense>)> {
            f.store
                .faces_of_body(f.body)
                .iter()
                .map(|&face| {
                    let sense = f.store.face(face).expect("live face").sense;
                    let winding = fins_of_face(&f.store, face)
                        .into_iter()
                        .map(|fin| f.store.fin(fin).expect("live fin").sense)
                        .collect();
                    (sense, winding)
                })
                .collect()
        };
        let before = snapshot(&f);

        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions {
                strategy: HealStrategy::ReportOnly,
                ..HealOptions::default()
            },
        );
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(op, HealOperation::FaceReoriented { .. }))
                .count(),
            1,
            "the flip must still be reported: {:?}",
            result.operations
        );
        assert_eq!(
            before,
            snapshot(&f),
            "ReportOnly must leave every face's sense and winding as authored"
        );
        assert_eq!(result.remaining, result.failures_before);
    }

    #[test]
    fn off_strategy_does_nothing() {
        let mut f = unsewn_block(0.0, &[]);
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions {
                strategy: HealStrategy::Off,
                ..HealOptions::default()
            },
        );
        assert!(result.operations.is_empty());
        assert_eq!(result.remaining, result.failures_before);
        assert_eq!(f.store.vertices.len(), 24);
    }

    /// The predicate that decides whether a vertex cluster holding both ends
    /// of an edge may merge (of-zdx). A full circle survives — one vertex can
    /// stand at both ends of it, which is what a closed edge is. A line does
    /// not, however close together its vertices have drifted: merging them
    /// would leave an edge whose curve runs from a point back to itself
    /// through three metres of nowhere.
    #[test]
    fn only_a_closed_curve_survives_having_its_ends_merged() {
        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let seam = Point3::new(6.0, 0.0, 0.0);
        let a = store.create_vertex(seam, SYSTEM_RESOLUTION);
        let b = store.create_vertex(seam, SYSTEM_RESOLUTION);

        let circle = geo.add_curve(Curve3::circle(Point3::origin(), Vector3::z(), 6.0).unwrap());
        let round = store.create_edge_with_curve(
            a,
            b,
            SYSTEM_RESOLUTION,
            circle,
            0.0,
            std::f64::consts::TAU,
        );
        assert!(edge_curve_closes(&store, &geo, round, SYSTEM_RESOLUTION));

        let line = geo.add_curve(Curve3::line(Point3::origin(), Vector3::x()).unwrap());
        let straight = store.create_edge_with_curve(a, b, SYSTEM_RESOLUTION, line, 0.0, 3.0);
        assert!(!edge_curve_closes(
            &store,
            &geo,
            straight,
            SYSTEM_RESOLUTION
        ));
    }

    #[test]
    fn merging_never_collapses_an_edge() {
        // Two corners of one face brought within the merge tolerance of each
        // other: merging them would delete the edge between them and break
        // the Euler counts, so the cluster must be refused with a note.
        let mut f = unsewn_block(0.0, &[]);
        let vertices = body_vertices(&f.store, f.body);
        let victim = vertices[1];
        let anchor = f.store.vertex(vertices[0]).expect("live vertex").point;
        f.store.vertices.get_mut(victim).expect("live vertex").point = anchor;

        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(
            result
                .notes
                .iter()
                .any(|n| n.contains("an edge joins two of them")),
            "notes: {:?}",
            result.notes
        );
    }

    #[test]
    fn max_gap_is_clamped_to_the_kernel_tolerance_limit() {
        let mut f = unsewn_block(0.0, &[]);
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions {
                strategy: HealStrategy::Auto,
                max_gap: Some(1.0),
                ..HealOptions::default()
            },
        );
        assert!(
            result.notes.iter().any(|n| n.contains("clamped max_gap")),
            "notes: {:?}",
            result.notes
        );
    }

    /// [`HealOptions::gap_floor`] (the STEP reader's declared closure)
    /// raises the vertex-merge gap over the derived round-off gap: a jitter
    /// far past [`HEAL_GAP_REL`] × the diagonal refuses to sew by default
    /// but sews when the floor vouches for it.
    #[test]
    fn the_gap_floor_widens_vertex_merging() {
        // Unit block: derived gap ≈ 1.7e-5; this jitter is 60× outside it.
        let jitter = 1.0e-3;
        let mut refused = unsewn_block(jitter, &[]);
        let result = GeometryHealer::heal(
            refused.body,
            &mut refused.store,
            &mut refused.geo,
            &HealOptions::default(),
        );
        assert!(
            !result.healed(),
            "the jitter must exceed the derived gap for this pin to mean anything"
        );

        let mut floored = unsewn_block(jitter, &[]);
        let result = GeometryHealer::heal(
            floored.body,
            &mut floored.store,
            &mut floored.geo,
            &HealOptions {
                gap_floor: 4.0e-3,
                ..HealOptions::default()
            },
        );
        assert!(result.healed(), "remaining: {:?}", result.remaining);
        assert_eq!(floored.store.vertices.len(), 8);
        assert!((signed_volume(&floored) - 1.0).abs() < 1e-2);
        assert!(
            result.notes.iter().any(|n| n.contains("gap floor")),
            "notes: {:?}",
            result.notes
        );
    }

    #[test]
    fn heal_operations_render_for_diagnostics() {
        let mut f = unsewn_block(1e-6, &[]);
        let result = GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(!result.operations.is_empty());
        for op in &result.operations {
            let text = op.to_string();
            assert!(!text.is_empty());
            assert!(text.is_ascii());
        }
    }

    #[test]
    fn healing_a_valid_body_reports_nothing_to_do() {
        let mut f = sewn_block(&[]);
        let result =
            GeometryHealer::fix_gaps(f.body, &mut f.store, &f.geo, &HealOptions::default());
        assert!(result.is_empty());
        assert!(!result.healed(), "there was nothing wrong to heal");
    }

    // --- Phase 2 fixtures -------------------------------------------

    /// The block edge whose vertices sit at `a` and `b` (either order).
    fn find_block_edge(f: &BlockFixture, a: Point3, b: Point3) -> EntityId<Edge> {
        body_edges(&f.store, f.body)
            .into_iter()
            .find(|&e| {
                let edge = f.store.edge(e).expect("live edge");
                let s = f.store.vertex(edge.start_vertex).expect("live vertex").point;
                let t = f.store.vertex(edge.end_vertex).expect("live vertex").point;
                ((s - a).norm() < 1e-9 && (t - b).norm() < 1e-9)
                    || ((s - b).norm() < 1e-9 && (t - a).norm() < 1e-9)
            })
            .expect("block edge exists")
    }

    /// Split the block edge between `(0,0,0)` and `(1,0,0)` with a mid
    /// vertex `eps` from the first corner — the sliver an exporter leaves
    /// where a tiny feature collapsed. Both adjacent faces get the split, so
    /// the body stays sewn and valid. Returns the sliver edge.
    fn split_block_edge_with_sliver(f: &mut BlockFixture, eps: f64) -> EntityId<Edge> {
        let target = find_block_edge(
            f,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let (va, vb, curve_id, t_start, t_end) = {
            let e = f.store.edge(target).expect("live edge");
            (
                e.start_vertex,
                e.end_vertex,
                e.curve.expect("fixture edges carry curves"),
                e.t_start,
                e.t_end,
            )
        };
        assert!(
            (f.store.vertex(va).expect("live vertex").point - Point3::origin()).norm() < 1e-9,
            "sewn_block authors this edge from the origin corner"
        );

        let m = f
            .store
            .create_vertex(Point3::new(eps, 0.0, 0.0), SYSTEM_RESOLUTION);
        let sliver =
            f.store
                .create_edge_with_curve(va, m, SYSTEM_RESOLUTION, curve_id, t_start, eps);
        let long = f
            .store
            .create_edge_with_curve(m, vb, SYSTEM_RESOLUTION, curve_id, eps, t_end);

        // Replace each fin of the split edge with two fins over the halves.
        let old_fins: Vec<EntityId<Fin>> = f.store.fins_of_edge(target).to_vec();
        for old_fin in old_fins {
            let (loop_id, sense) = {
                let fin = f.store.fin(old_fin).expect("live fin");
                (fin.loop_ref, fin.sense)
            };
            let halves = if sense == FinSense::Forward {
                [(sliver, FinSense::Forward), (long, FinSense::Forward)]
            } else {
                [(long, FinSense::Reversed), (sliver, FinSense::Reversed)]
            };
            let new_fins: Vec<EntityId<Fin>> = halves
                .iter()
                .map(|&(edge, sense)| {
                    let fin = f.store.fins.insert(Fin {
                        edge,
                        loop_ref: loop_id,
                        sense,
                        next: None,
                        prev: None,
                        mate: None,
                        pcurve: None,
                    });
                    f.store
                        .edges
                        .get_mut(edge)
                        .expect("live edge")
                        .fins
                        .push(fin);
                    fin
                })
                .collect();
            let lp = f.store.loops.get_mut(loop_id).expect("live loop");
            let at = lp
                .fins
                .iter()
                .position(|&x| x == old_fin)
                .expect("fin sits in its loop");
            lp.fins.splice(at..=at, new_fins);
            f.store.fins.remove(old_fin);
            let fins = f.store.loops.get(loop_id).expect("live loop").fins.clone();
            let n = fins.len();
            for (i, &fin_id) in fins.iter().enumerate() {
                let fin = f.store.fins.get_mut(fin_id).expect("live fin");
                fin.next = Some(fins[(i + 1) % n]);
                fin.prev = Some(fins[(i + n - 1) % n]);
            }
        }
        for e in [sliver, long] {
            let fins = f.store.edges.get(e).expect("live edge").fins.clone();
            assert_eq!(fins.len(), 2, "both faces traverse each half");
            f.store.fins.get_mut(fins[0]).expect("live fin").mate = Some(fins[1]);
            f.store.fins.get_mut(fins[1]).expect("live fin").mate = Some(fins[0]);
        }
        f.store.edges.remove(target);
        for v in [va, vb] {
            f.store
                .vertices
                .get_mut(v)
                .expect("live vertex")
                .edges
                .retain(|&e| e != target);
        }
        sliver
    }

    /// Replace the `(0,0,0)`–`(1,0,0)` edge's curve with a parallel line
    /// displaced by `offset` — an exporter that wrote the edge's geometry
    /// against the wrong support. Returns the edge.
    fn displace_block_edge(f: &mut BlockFixture, offset: Vector3) -> EntityId<Edge> {
        let target = find_block_edge(
            f,
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
        );
        let bad = f.geo.add_curve(
            Curve3::line(Point3::origin() + offset, Vector3::x()).expect("unit direction"),
        );
        let e = f.store.edges.get_mut(target).expect("live edge");
        e.curve = Some(bad);
        e.t_start = 0.0;
        e.t_end = 1.0;
        target
    }

    // --- Sliver collapse ---------------------------------------------

    #[test]
    fn a_sliver_edge_is_collapsed_through_kev() {
        let mut f = sewn_block(&[]);
        let sliver = split_block_edge_with_sliver(&mut f, 1e-6);
        assert_eq!(
            f.store.check(f.body),
            vec![],
            "the split fixture must be valid before healing"
        );
        assert_eq!(f.store.vertices.len(), 9);
        assert_eq!(f.store.edges.len(), 13);

        let result =
            GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(
            result.operations.iter().any(
                |op| matches!(op, HealOperation::EdgeCollapsed { edge, .. } if *edge == sliver)
            ),
            "the sliver must be collapsed: {:?}",
            result.operations
        );
        assert_eq!(f.store.check(f.body), vec![]);
        assert_eq!(f.store.vertices.len(), 8, "the mid vertex is gone");
        assert_eq!(f.store.edges.len(), 12, "the sliver edge is gone");
        let failures = f.store.check_geometry(&f.geo, f.body);
        assert!(
            failures.is_empty(),
            "the collapse must be covered by tolerance: {failures:#?}"
        );
        assert!((signed_volume(&f) - 1.0).abs() < 1e-4);
    }

    #[test]
    fn report_only_plans_a_sliver_collapse_without_touching() {
        let mut f = sewn_block(&[]);
        let sliver = split_block_edge_with_sliver(&mut f, 1e-6);
        let result = GeometryHealer::heal(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions {
                strategy: HealStrategy::ReportOnly,
                ..HealOptions::default()
            },
        );
        assert!(
            result.operations.iter().any(
                |op| matches!(op, HealOperation::EdgeCollapsed { edge, .. } if *edge == sliver)
            ),
            "the collapse must still be reported: {:?}",
            result.operations
        );
        assert_eq!(f.store.vertices.len(), 9, "nothing may move");
        assert_eq!(f.store.edges.len(), 13);
    }

    /// The refusal half: a short edge that is longer than the tolerance is a
    /// real feature, not a sliver, and indiscriminate collapsing is exactly
    /// what healing must never do.
    #[test]
    fn a_short_but_honest_edge_is_not_collapsed() {
        let mut f = sewn_block(&[]);
        split_block_edge_with_sliver(&mut f, 0.2);
        let result =
            GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(
            result
                .operations
                .iter()
                .all(|op| !matches!(op, HealOperation::EdgeCollapsed { .. })),
            "0.2 mm on a 1 mm block is a feature: {:?}",
            result.operations
        );
        assert_eq!(f.store.vertices.len(), 9);
        assert_eq!(f.store.edges.len(), 13);
    }

    // --- Edge/surface consistency --------------------------------------

    #[test]
    fn an_edge_curve_past_the_tolerance_cap_is_recomputed_from_ssi() {
        let mut f = sewn_block(&[]);
        let target = displace_block_edge(&mut f, Vector3::new(0.0, 0.0, -0.05));
        assert!(
            f.store
                .check_geometry(&f.geo, f.body)
                .iter()
                .any(|x| matches!(x, CheckFailure::EdgeOffSurface { .. })),
            "the displaced curve must fail the geometric check first"
        );

        let result =
            GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(
            result.operations.iter().any(|op| matches!(
                op,
                HealOperation::EdgeCurveRecomputed { edge, .. } if *edge == target
            )),
            "the curve must be recomputed: {:?}",
            result.operations
        );
        assert_eq!(f.store.check_geometry(&f.geo, f.body), vec![]);
        // The replacement runs along the true corner line, trimmed to the
        // authored vertices.
        let e = f.store.edge(target).expect("live edge");
        let curve = f.geo.curve(e.curve.expect("edge keeps a curve")).unwrap();
        assert!((curve.point(e.t_start) - Point3::origin()).norm() < 1e-9);
        assert!((curve.point(e.t_end) - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-9);
        assert!((signed_volume(&f) - 1.0).abs() < 1e-9);
    }

    /// Under the kernel cap the minimal repair wins, and it is split between
    /// two owners: `heal` leaves the band entirely to the reader's tolerance
    /// recording, while the standalone pass records it itself.
    #[test]
    fn a_deviation_under_the_cap_is_elevated_not_recomputed() {
        let mut f = sewn_block(&[]);
        displace_block_edge(&mut f, Vector3::new(0.0, 0.0, -1e-4));
        let result =
            GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(
            result.operations.is_empty(),
            "heal leaves the sub-cap band to record_edge_tolerances: {:?}",
            result.operations
        );

        let mut f = sewn_block(&[]);
        let target = displace_block_edge(&mut f, Vector3::new(0.0, 0.0, -1e-4));
        let curve_before = f.store.edge(target).unwrap().curve;
        let result = GeometryHealer::fix_edge_surface_consistency(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions::default(),
        );
        assert!(
            result.operations.iter().any(|op| matches!(
                op,
                HealOperation::ToleranceElevated { entity: EntityRef::Edge(e), .. } if *e == target
            )),
            "the standalone pass records the measured distance: {:?}",
            result.operations
        );
        assert!(
            result
                .operations
                .iter()
                .all(|op| !matches!(op, HealOperation::EdgeCurveRecomputed { .. })),
            "an honest tolerance beats replacing authored geometry"
        );
        assert_eq!(
            f.store.edge(target).unwrap().curve,
            curve_before,
            "the authored curve must survive"
        );
        assert!(result.healed(), "remaining: {:?}", result.remaining);
    }

    #[test]
    fn past_the_cap_with_no_second_surface_is_refused_with_a_note() {
        let mut f = sewn_block(&[]);
        // Displaced off both adjacent planes, so the deviation survives
        // whichever surface remains measurable.
        let target = displace_block_edge(&mut f, Vector3::new(0.0, -0.05, -0.05));
        let face = f.store.fin_face(f.store.fins_of_edge(target)[0]);
        f.store.faces.get_mut(face).expect("live face").surface = None;

        let result =
            GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(
            result
                .operations
                .iter()
                .all(|op| !matches!(op, HealOperation::EdgeCurveRecomputed { .. })),
            "one surface cannot make an intersection: {:?}",
            result.operations
        );
        assert!(
            result
                .notes
                .iter()
                .any(|n| n.contains("no honest replacement curve")),
            "the refusal must be recorded: {:?}",
            result.notes
        );
    }

    /// The circle-arc machinery of the recompute planner: a quarter arc
    /// where a cylinder meets a plane, authored floating above the plane.
    /// The intersection is a full circle, and three of its four readings
    /// (far arc, either winding; near arc, wrong winding) share the edge's
    /// endpoints — only the branch-match test tells them apart.
    #[test]
    fn recompute_picks_the_matching_arc_of_a_circular_intersection() {
        use std::f64::consts::{FRAC_1_SQRT_2, FRAC_PI_2};

        let mut store = TopologyStore::new();
        let mut geo = GeometryStore::new();
        let body = store.create_body(BodyType::Solid);
        let shell = store.create_shell(body, true, ShellOrientation::Outward);

        let plane = geo.add_surface(Surface3::plane(Point3::origin(), Vector3::z()).unwrap());
        let cylinder =
            geo.add_surface(Surface3::cylinder(Point3::origin(), Vector3::z(), 1.0).unwrap());
        let face_a = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face_a).expect("live face").surface = Some(plane);
        let face_b = store.create_face(shell, FaceSense::Positive);
        store.faces.get_mut(face_b).expect("live face").surface = Some(cylinder);

        let delta = 0.05;
        let a = store.create_vertex(Point3::new(1.0, 0.0, 0.0), SYSTEM_RESOLUTION);
        let b = store.create_vertex(Point3::new(0.0, 1.0, 0.0), SYSTEM_RESOLUTION);
        // A quarter turn floating `delta` above the plane (still exactly on
        // the cylinder wall).
        let bad = geo.add_curve(
            Curve3::circle(Point3::new(0.0, 0.0, delta), Vector3::z(), 1.0).unwrap(),
        );
        let edge = store.create_edge_with_curve(a, b, SYSTEM_RESOLUTION, bad, 0.0, FRAC_PI_2);
        store.create_loop(face_a, LoopType::Outer, &[(edge, FinSense::Forward)]);
        store.create_loop(face_b, LoopType::Outer, &[(edge, FinSense::Reversed)]);

        let mut notes = Vec::new();
        let recompute = plan_curve_recompute(&store, &geo, edge, delta, &mut notes)
            .expect("a plane-cylinder edge must be recomputable");
        assert!(
            recompute.deviation_after < 1e-9,
            "the replacement lies on both surfaces: {:.3e}",
            recompute.deviation_after
        );
        assert!(
            (recompute.curve.point(recompute.t_start) - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-9
        );
        assert!(
            (recompute.curve.point(recompute.t_end) - Point3::new(0.0, 1.0, 0.0)).norm() < 1e-9
        );
        let mid = recompute
            .curve
            .point((recompute.t_start + recompute.t_end) / 2.0);
        let expected = Point3::new(FRAC_1_SQRT_2, FRAC_1_SQRT_2, 0.0);
        assert!(
            (mid - expected).norm() < 1e-9,
            "the near arc with the authored winding must win, got mid {mid}"
        );
    }

    // --- Pcurve recompute ----------------------------------------------

    #[test]
    fn a_stale_pcurve_is_refit() {
        use opensolid_brep::attach_body_pcurves;
        use opensolid_core::{Point2, Vector2};

        let mut f = sewn_block(&[]);
        attach_body_pcurves(&mut f.store, &mut f.geo, f.body);
        assert_eq!(f.store.check_geometry(&f.geo, f.body), vec![]);

        let fin = fins_of_face(&f.store, f.store.faces_of_body(f.body)[0])[0];
        let rogue = f
            .geo
            .add_pcurve(Curve2::line(Point2::new(7.0, 7.0), Vector2::x()).unwrap());
        f.store.fins.get_mut(fin).expect("live fin").pcurve = Some(rogue);
        assert!(
            f.store
                .check_geometry(&f.geo, f.body)
                .iter()
                .any(|x| matches!(x, CheckFailure::PcurveDeviation { .. })),
            "the rogue pcurve must fail the geometric check first"
        );

        let result = GeometryHealer::fix_pcurves(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions::default(),
        );
        assert!(
            result.operations.iter().any(|op| matches!(
                op,
                HealOperation::PcurveRecomputed { fin: fixed, .. } if *fixed == fin
            )),
            "the refit must be reported: {:?}",
            result.operations
        );
        assert!(
            result.healed(),
            "before: {:?}, after: {:?}",
            result.failures_before,
            result.remaining
        );
    }

    #[test]
    fn report_only_plans_a_pcurve_refit_without_touching() {
        use opensolid_brep::attach_body_pcurves;
        use opensolid_core::{Point2, Vector2};

        let mut f = sewn_block(&[]);
        attach_body_pcurves(&mut f.store, &mut f.geo, f.body);
        let fin = fins_of_face(&f.store, f.store.faces_of_body(f.body)[0])[0];
        let rogue = f
            .geo
            .add_pcurve(Curve2::line(Point2::new(7.0, 7.0), Vector2::x()).unwrap());
        f.store.fins.get_mut(fin).expect("live fin").pcurve = Some(rogue);

        let result = GeometryHealer::fix_pcurves(
            f.body,
            &mut f.store,
            &mut f.geo,
            &HealOptions {
                strategy: HealStrategy::ReportOnly,
                ..HealOptions::default()
            },
        );
        assert!(
            result
                .operations
                .iter()
                .any(|op| matches!(op, HealOperation::PcurveRecomputed { .. })),
            "the refit must still be planned: {:?}",
            result.operations
        );
        assert_eq!(
            f.store.fin(fin).expect("live fin").pcurve,
            Some(rogue),
            "ReportOnly must leave the rogue pcurve in place"
        );
        assert_eq!(result.remaining, result.failures_before);
    }

    #[test]
    fn recomputing_an_edge_refits_the_pcurves_riding_it() {
        use opensolid_brep::attach_body_pcurves;

        let mut f = sewn_block(&[]);
        attach_body_pcurves(&mut f.store, &mut f.geo, f.body);
        let target = displace_block_edge(&mut f, Vector3::new(0.0, 0.0, -0.05));

        let result =
            GeometryHealer::heal(f.body, &mut f.store, &mut f.geo, &HealOptions::default());
        assert!(result.operations.iter().any(|op| matches!(
            op,
            HealOperation::EdgeCurveRecomputed { edge, .. } if *edge == target
        )));
        assert_eq!(
            result
                .operations
                .iter()
                .filter(|op| matches!(
                    op,
                    HealOperation::PcurveRecomputed { edge, .. } if *edge == target
                ))
                .count(),
            2,
            "both fins riding the recomputed edge refit their pcurves: {:?}",
            result.operations
        );
        assert_eq!(f.store.check_geometry(&f.geo, f.body), vec![]);
    }
}
