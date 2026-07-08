//! Sweep MVP: extrude and revolve planar profiles into B-Rep bodies.
//!
//! A [`Profile`] is a closed, planar loop of [`Curve3`] segments (lines and
//! arcs). [`extrude`] sweeps it along a vector into a prism-like solid with
//! side walls and two caps; [`revolve`] sweeps it a full turn around an axis
//! lying in the profile plane into a solid with periodic faces (partial
//! angles are a later issue). Both return a [`SweptBody`] whose topology
//! passes the full body checker ([`TopologyStore::check`]), including the
//! Euler-Poincaré formula.
//!
//! The geometry slots of the topology store are still placeholders
//! (`Edge::curve`, `Face::surface`), so the swept body keeps its defining
//! sweep parameters and tessellates itself from those via
//! [`SweptBody::tessellate`]; binding real curve/surface geometry to the
//! topology is a later issue.
//!
//! Orientation conventions: the profile's winding (via Newell's method)
//! defines its plane normal; inputs are normalized internally so that
//! extrusion direction and winding agree and the revolved profile lies on
//! the positive-radius side of the axis. Face loops follow the outward-shell
//! convention used across the crate.
//!
//! ```
//! use opensolid_brep::sweep::{Profile, ProfileSegment, extrude};
//! use opensolid_core::types::{Point3, Vector3};
//!
//! let corners = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)];
//! let segments: Vec<ProfileSegment> = (0..4)
//!     .map(|i| {
//!         let (ax, ay) = corners[i];
//!         let (bx, by) = corners[(i + 1) % 4];
//!         ProfileSegment::line_between(Point3::new(ax, ay, 0.0), Point3::new(bx, by, 0.0))
//!     })
//!     .collect::<Result<_, _>>()?;
//! let rectangle = Profile::new(segments)?;
//!
//! let block = extrude(&rectangle, Vector3::new(0.0, 0.0, 3.0))?;
//! assert!(block.check().is_empty());
//! let counts = block.store.euler_counts(block.body);
//! assert_eq!((counts.vertices, counts.edges, counts.faces), (8, 12, 6));
//! assert!(block.tessellate(8)?.is_closed_manifold());
//! # Ok::<(), opensolid_core::error::CoreError>(())
//! ```

use crate::check::CheckFailure;
use crate::curve::{Curve3, CurveEval, TWO_PI, plane_basis};
use crate::topology::{
    Body, BodyType, Edge, Face, FaceSense, FinSense, Loop, LoopType, SYSTEM_RESOLUTION,
    ShellOrientation, TopologyStore, Vertex,
};
use crate::triangulate::ear_clip;
use opensolid_core::EntityId;
use opensolid_core::error::{CoreError, CoreResult};
use opensolid_core::mesh::TriangleMesh;
use opensolid_core::types::{Point3, Vector3};

/// Samples per segment used for profile validation (planarity, winding,
/// axis-side classification).
const VALIDATION_SAMPLES: usize = 8;

/// Relative tolerance factor: geometric checks use `TOL_SCALE * (1 + extent)`
/// where `extent` is the profile's bounding-box diagonal.
const TOL_SCALE: f64 = 1e-9;

/// One directed piece of a profile: a [`Curve3`] restricted to a parameter
/// range. `t_start > t_end` traverses the curve backwards.
#[derive(Debug, Clone)]
pub struct ProfileSegment {
    curve: Curve3,
    t_start: f64,
    t_end: f64,
}

impl ProfileSegment {
    /// A segment of `curve` from parameter `t_start` to `t_end` (reversed
    /// traversal if `t_start > t_end`).
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if a parameter is not finite, the range
    /// is empty, or the range spans a full period of a closed curve (closed
    /// profiles must consist of at least two segments).
    ///
    /// ```
    /// use opensolid_brep::curve::Curve3;
    /// use opensolid_brep::sweep::ProfileSegment;
    /// use opensolid_core::types::{Point3, Vector3};
    /// use std::f64::consts::FRAC_PI_2;
    ///
    /// let arc = Curve3::circle(Point3::origin(), Vector3::z(), 1.0)?;
    /// let quarter = ProfileSegment::new(arc.clone(), 0.0, FRAC_PI_2)?;
    /// assert!((quarter.start() - Point3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
    /// assert!((quarter.end() - Point3::new(0.0, 1.0, 0.0)).norm() < 1e-12);
    /// assert!(ProfileSegment::new(arc, 0.0, 0.0).is_err());
    /// # Ok::<(), opensolid_core::error::CoreError>(())
    /// ```
    pub fn new(curve: Curve3, t_start: f64, t_end: f64) -> CoreResult<Self> {
        for (name, t) in [("t_start", t_start), ("t_end", t_end)] {
            if !t.is_finite() {
                return Err(CoreError::InvalidArgument {
                    argument: name,
                    reason: format!("must be finite, got {t}"),
                });
            }
        }
        let span = (t_end - t_start).abs();
        if span <= 1e-12 {
            return Err(CoreError::InvalidArgument {
                argument: "t_end",
                reason: format!(
                    "parameter range is empty ({t_start} to {t_end}); \
                     a segment must cover a non-zero range"
                ),
            });
        }
        if curve.is_periodic() && span >= TWO_PI - 1e-9 {
            return Err(CoreError::InvalidArgument {
                argument: "t_end",
                reason: format!(
                    "parameter range spans a full period ({span} >= 2π); \
                     split a closed curve into at least two segments"
                ),
            });
        }
        Ok(Self {
            curve,
            t_start,
            t_end,
        })
    }

    /// A straight segment from `a` to `b`, parameterized by arc length.
    ///
    /// # Errors
    /// [`CoreError::Degenerate`] if the points coincide (or are non-finite).
    ///
    /// ```
    /// use opensolid_brep::sweep::ProfileSegment;
    /// use opensolid_core::types::Point3;
    ///
    /// let (a, b) = (Point3::origin(), Point3::new(3.0, 4.0, 0.0));
    /// let seg = ProfileSegment::line_between(a, b)?;
    /// assert!((seg.end() - b).norm() < 1e-12);
    /// assert!(ProfileSegment::line_between(a, a).is_err());
    /// # Ok::<(), opensolid_core::error::CoreError>(())
    /// ```
    pub fn line_between(a: Point3, b: Point3) -> CoreResult<Self> {
        let dir = b - a;
        let length = dir.norm();
        let line = Curve3::line(a, dir)?;
        Self::new(line, 0.0, length)
    }

    /// The underlying curve.
    pub fn curve(&self) -> &Curve3 {
        &self.curve
    }

    /// The traversed parameter range `(t_start, t_end)`.
    pub fn t_range(&self) -> (f64, f64) {
        (self.t_start, self.t_end)
    }

    /// The segment's start point, `curve.point(t_start)`.
    pub fn start(&self) -> Point3 {
        self.curve.point(self.t_start)
    }

    /// The segment's end point, `curve.point(t_end)`.
    pub fn end(&self) -> Point3 {
        self.curve.point(self.t_end)
    }

    /// Point at fraction `f ∈ [0, 1]` along the traversed range.
    fn point_at_fraction(&self, f: f64) -> Point3 {
        self.curve
            .point(self.t_start + (self.t_end - self.t_start) * f)
    }

    /// The same piece of curve traversed the other way.
    fn reversed(&self) -> Self {
        Self {
            curve: self.curve.clone(),
            t_start: self.t_end,
            t_end: self.t_start,
        }
    }
}

/// A closed, planar profile: an ordered loop of segments where each
/// segment's end meets the next segment's start and the last closes back to
/// the first.
///
/// The winding of the loop defines the profile's plane [`normal`]
/// (right-hand rule: the profile is counterclockwise seen from the normal's
/// tip). Self-intersection is not checked; a self-intersecting profile
/// produces garbage downstream and is the caller's responsibility for now.
///
/// [`normal`]: Profile::normal
#[derive(Debug, Clone)]
pub struct Profile {
    segments: Vec<ProfileSegment>,
    /// Unit winding normal of the profile plane (Newell's method).
    normal: Vector3,
    /// A point on the profile plane (sample centroid).
    plane_point: Point3,
    /// Scale-relative geometric tolerance used to validate this profile.
    tolerance: f64,
}

impl Profile {
    /// Validate and build a profile from an ordered loop of segments.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] with a teaching message if there are
    /// fewer than two segments or consecutive segments do not connect;
    /// [`CoreError::Degenerate`] if a segment's endpoints coincide, the
    /// profile is not planar, or it encloses no area.
    ///
    /// ```
    /// use opensolid_brep::sweep::{Profile, ProfileSegment};
    /// use opensolid_core::types::Point3;
    ///
    /// let a = Point3::new(0.0, 0.0, 0.0);
    /// let b = Point3::new(1.0, 0.0, 0.0);
    /// let c = Point3::new(0.0, 1.0, 0.0);
    /// let triangle = Profile::new(vec![
    ///     ProfileSegment::line_between(a, b)?,
    ///     ProfileSegment::line_between(b, c)?,
    ///     ProfileSegment::line_between(c, a)?,
    /// ])?;
    /// // Counterclockwise in the XY plane: the winding normal is +Z.
    /// assert!((triangle.normal() - opensolid_core::types::Vector3::z()).norm() < 1e-9);
    ///
    /// // An open chain is rejected.
    /// let open = Profile::new(vec![
    ///     ProfileSegment::line_between(a, b)?,
    ///     ProfileSegment::line_between(b, c)?,
    /// ]);
    /// assert!(open.is_err());
    /// # Ok::<(), opensolid_core::error::CoreError>(())
    /// ```
    pub fn new(segments: Vec<ProfileSegment>) -> CoreResult<Self> {
        if segments.len() < 2 {
            return Err(CoreError::InvalidArgument {
                argument: "segments",
                reason: format!(
                    "a closed profile needs at least 2 segments, got {}",
                    segments.len()
                ),
            });
        }

        let polyline = sample_closed(&segments, VALIDATION_SAMPLES);
        let extent = polyline_extent(&polyline);
        if !extent.is_finite() {
            return Err(CoreError::InvalidArgument {
                argument: "segments",
                reason: "profile contains non-finite points".into(),
            });
        }
        let tolerance = TOL_SCALE * (1.0 + extent);

        let n = segments.len();
        for i in 0..n {
            let seg = &segments[i];
            if (seg.end() - seg.start()).norm() <= tolerance {
                return Err(CoreError::Degenerate {
                    context: "Profile::new",
                    reason: format!("segment {i} is degenerate (its endpoints coincide)"),
                });
            }
            let j = (i + 1) % n;
            let gap = (segments[j].start() - seg.end()).norm();
            if gap > tolerance {
                return Err(CoreError::InvalidArgument {
                    argument: "segments",
                    reason: format!(
                        "segment {i} ends at {:?} but segment {j} starts at {:?} \
                         (gap {gap:.3e}); the loop must be connected and closed",
                        seg.end(),
                        segments[j].start()
                    ),
                });
            }
        }

        // Newell's method over the sampled polyline: the summed cross
        // products give twice the vector area, whose direction is the
        // winding normal.
        let centroid = polyline_centroid(&polyline);
        let mut area_vec = Vector3::zeros();
        for (i, p) in polyline.iter().enumerate() {
            let q = polyline[(i + 1) % polyline.len()];
            area_vec += (p - centroid).cross(&(q - centroid));
        }
        let doubled_area = area_vec.norm();
        if doubled_area <= tolerance * (1.0 + extent) {
            return Err(CoreError::Degenerate {
                context: "Profile::new",
                reason: "profile encloses no area (all points are collinear or \
                         the loop cancels itself)"
                    .into(),
            });
        }
        let normal = area_vec / doubled_area;

        for p in &polyline {
            let off = (p - centroid).dot(&normal).abs();
            if off > tolerance {
                return Err(CoreError::Degenerate {
                    context: "Profile::new",
                    reason: format!(
                        "profile is not planar: point {p:?} lies {off:.3e} from \
                         the plane of the loop"
                    ),
                });
            }
        }

        Ok(Self {
            segments,
            normal,
            plane_point: centroid,
            tolerance,
        })
    }

    /// The profile's segments in loop order.
    pub fn segments(&self) -> &[ProfileSegment] {
        &self.segments
    }

    /// Unit normal of the profile plane, oriented by the loop's winding
    /// (counterclockwise seen from the normal's tip).
    pub fn normal(&self) -> Vector3 {
        self.normal
    }

    /// The same loop traversed the other way (flips [`Profile::normal`]).
    fn reversed(&self) -> Self {
        Self {
            segments: self.segments.iter().rev().map(|s| s.reversed()).collect(),
            normal: -self.normal,
            plane_point: self.plane_point,
            tolerance: self.tolerance,
        }
    }

    /// Segment start points, in loop order: the profile's "vertices".
    fn vertex_points(&self) -> Vec<Point3> {
        self.segments.iter().map(|s| s.start()).collect()
    }

    /// Closed polyline approximation: `per_segment` samples from each
    /// segment (endpoints appear once, as the next segment's start).
    fn sample_closed_polyline(&self, per_segment: usize) -> Vec<Point3> {
        sample_closed(&self.segments, per_segment)
    }
}

fn sample_closed(segments: &[ProfileSegment], per_segment: usize) -> Vec<Point3> {
    let mut points = Vec::with_capacity(segments.len() * per_segment);
    for seg in segments {
        for i in 0..per_segment {
            points.push(seg.point_at_fraction(i as f64 / per_segment as f64));
        }
    }
    points
}

fn polyline_extent(points: &[Point3]) -> f64 {
    let mut min = points[0];
    let mut max = points[0];
    for p in points {
        min = Point3::new(min.x.min(p.x), min.y.min(p.y), min.z.min(p.z));
        max = Point3::new(max.x.max(p.x), max.y.max(p.y), max.z.max(p.z));
    }
    (max - min).norm()
}

fn polyline_centroid(points: &[Point3]) -> Point3 {
    let sum = points
        .iter()
        .fold(Vector3::zeros(), |acc, p| acc + p.coords);
    Point3::from(sum / points.len() as f64)
}

/// How a [`SweptBody`] was made; retained so the body can tessellate itself
/// while the topology's geometry slots are still placeholders. The stored
/// profile is the normalized one (winding aligned with the sweep).
#[derive(Debug, Clone)]
enum SweepKind {
    Extrude {
        profile: Profile,
        direction: Vector3,
    },
    Revolve {
        profile: Profile,
        axis_point: Point3,
        /// Unit axis direction.
        axis_dir: Vector3,
    },
}

/// A solid produced by [`extrude`] or [`revolve`]: a validated topology
/// graph plus the sweep parameters that define its geometry.
pub struct SweptBody {
    /// The topology graph holding the body.
    pub store: TopologyStore,
    /// The swept body inside [`SweptBody::store`].
    pub body: EntityId<Body>,
    kind: SweepKind,
}

/// The store has no useful `Debug` form; the body id and sweep parameters
/// are the observable summary.
impl std::fmt::Debug for SweptBody {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SweptBody")
            .field("body", &self.body)
            .field("kind", &self.kind)
            .finish_non_exhaustive()
    }
}

impl SweptBody {
    /// Validate the body with the full body checker
    /// ([`TopologyStore::check`]): referential integrity, loop
    /// connectivity, closure, manifoldness, orientation consistency,
    /// tolerance sanity, and the Euler-Poincaré formula. Empty means valid.
    ///
    /// ```
    /// use opensolid_brep::sweep::{Profile, ProfileSegment, extrude};
    /// use opensolid_core::types::{Point3, Vector3};
    ///
    /// let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
    /// let tri = Profile::new(vec![
    ///     ProfileSegment::line_between(p(0.0, 0.0), p(1.0, 0.0))?,
    ///     ProfileSegment::line_between(p(1.0, 0.0), p(0.0, 1.0))?,
    ///     ProfileSegment::line_between(p(0.0, 1.0), p(0.0, 0.0))?,
    /// ])?;
    /// let wedge = extrude(&tri, Vector3::z())?;
    /// assert!(wedge.check().is_empty());
    /// # Ok::<(), opensolid_core::error::CoreError>(())
    /// ```
    pub fn check(&self) -> Vec<CheckFailure> {
        self.store.check(self.body)
    }

    /// Tessellate the body into an indexed triangle mesh, sampling
    /// `resolution` points per profile segment and (for revolved bodies)
    /// `resolution` steps around the axis.
    ///
    /// The mesh is combinatorially closed and consistently oriented for any
    /// valid sweep. Extrusion caps are triangulated by a centroid fan, which
    /// is geometrically correct for convex and star-shaped profiles;
    /// non-star-shaped caps self-overlap geometrically but stay
    /// combinatorially closed.
    ///
    /// # Errors
    /// [`CoreError::InvalidArgument`] if `resolution` is below 4.
    ///
    /// ```
    /// use opensolid_brep::sweep::{Profile, ProfileSegment, revolve};
    /// use opensolid_core::types::{Point3, Vector3};
    ///
    /// // Rectangle with one side on the axis: revolves to a cylinder.
    /// let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
    /// let rect = Profile::new(vec![
    ///     ProfileSegment::line_between(p(0.0, 0.0), p(1.0, 0.0))?,
    ///     ProfileSegment::line_between(p(1.0, 0.0), p(1.0, 2.0))?,
    ///     ProfileSegment::line_between(p(1.0, 2.0), p(0.0, 2.0))?,
    ///     ProfileSegment::line_between(p(0.0, 2.0), p(0.0, 0.0))?,
    /// ])?;
    /// let cylinder = revolve(&rect, Point3::origin(), Vector3::y())?;
    /// let mesh = cylinder.tessellate(32)?;
    /// assert!(mesh.is_closed_manifold());
    /// assert!(cylinder.tessellate(2).is_err());
    /// # Ok::<(), opensolid_core::error::CoreError>(())
    /// ```
    pub fn tessellate(&self, resolution: usize) -> CoreResult<TriangleMesh> {
        if resolution < 4 {
            return Err(CoreError::InvalidArgument {
                argument: "resolution",
                reason: format!(
                    "must be at least 4 samples per segment and revolution, \
                     got {resolution}"
                ),
            });
        }
        let mut mesh = match &self.kind {
            SweepKind::Extrude { profile, direction } => {
                extrude_mesh(profile, *direction, resolution)
            }
            SweepKind::Revolve {
                profile,
                axis_point,
                axis_dir,
            } => revolve_mesh(profile, *axis_point, *axis_dir, resolution),
        };
        average_vertex_normals(&mut mesh);
        Ok(mesh)
    }
}

/// Extrude a planar closed profile along `direction` into a solid: one side
/// wall per profile segment plus two caps.
///
/// The direction may be oblique to the profile plane (a sheared prism) but
/// must not lie in it. Winding is normalized internally, so both extrusion
/// senses produce an outward-oriented shell.
///
/// Topology: for `n` segments, `2n` vertices, `3n` edges, `n + 2` faces,
/// genus 0 — a rectangle extrudes to exactly the block topology (8, 12, 6).
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `direction` is zero/non-finite or lies
/// in the profile plane.
///
/// ```
/// use opensolid_brep::sweep::{Profile, ProfileSegment, extrude};
/// use opensolid_core::types::{Point3, Vector3};
///
/// let p = |x: f64, y: f64| Point3::new(x, y, 0.0);
/// let square = Profile::new(vec![
///     ProfileSegment::line_between(p(0.0, 0.0), p(1.0, 0.0))?,
///     ProfileSegment::line_between(p(1.0, 0.0), p(1.0, 1.0))?,
///     ProfileSegment::line_between(p(1.0, 1.0), p(0.0, 1.0))?,
///     ProfileSegment::line_between(p(0.0, 1.0), p(0.0, 0.0))?,
/// ])?;
/// let cube = extrude(&square, Vector3::z())?;
/// assert!(cube.check().is_empty());
///
/// // A direction inside the profile plane cannot extrude.
/// assert!(extrude(&square, Vector3::x()).is_err());
/// # Ok::<(), opensolid_core::error::CoreError>(())
/// ```
pub fn extrude(profile: &Profile, direction: Vector3) -> CoreResult<SweptBody> {
    let norm = direction.norm();
    if norm == 0.0 || !norm.is_finite() {
        return Err(CoreError::InvalidArgument {
            argument: "direction",
            reason: format!("must be a nonzero finite vector, got {direction:?}"),
        });
    }
    let along_normal = direction.dot(&profile.normal);
    if along_normal.abs() <= TOL_SCALE * norm {
        return Err(CoreError::InvalidArgument {
            argument: "direction",
            reason: format!(
                "must not lie in the profile plane (direction {direction:?} is \
                 perpendicular to the plane normal {:?})",
                profile.normal
            ),
        });
    }
    // Normalize so the profile winds counterclockwise around the extrusion
    // direction; all loop orientations below assume it.
    let profile = if along_normal < 0.0 {
        profile.reversed()
    } else {
        profile.clone()
    };

    let verts = profile.vertex_points();
    let n = verts.len();

    let mut store = TopologyStore::new();
    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);

    let bottom: Vec<EntityId<Vertex>> = verts
        .iter()
        .map(|&p| store.create_vertex(p, SYSTEM_RESOLUTION))
        .collect();
    let top: Vec<EntityId<Vertex>> = verts
        .iter()
        .map(|&p| store.create_vertex(p + direction, SYSTEM_RESOLUTION))
        .collect();

    let bottom_edges: Vec<EntityId<Edge>> = (0..n)
        .map(|i| store.create_edge(bottom[i], bottom[(i + 1) % n], SYSTEM_RESOLUTION))
        .collect();
    let top_edges: Vec<EntityId<Edge>> = (0..n)
        .map(|i| store.create_edge(top[i], top[(i + 1) % n], SYSTEM_RESOLUTION))
        .collect();
    let lateral_edges: Vec<EntityId<Edge>> = (0..n)
        .map(|i| store.create_edge(bottom[i], top[i], SYSTEM_RESOLUTION))
        .collect();

    // Bottom cap faces away from the extrusion: traverse the profile
    // backwards. Top cap faces along it: traverse forwards.
    let bottom_face = store.create_face(shell, FaceSense::Positive);
    let bottom_loop: Vec<(EntityId<Edge>, FinSense)> = (0..n)
        .rev()
        .map(|i| (bottom_edges[i], FinSense::Reversed))
        .collect();
    store.create_loop(bottom_face, LoopType::Outer, &bottom_loop);

    let top_face = store.create_face(shell, FaceSense::Positive);
    let top_loop: Vec<(EntityId<Edge>, FinSense)> =
        (0..n).map(|i| (top_edges[i], FinSense::Forward)).collect();
    store.create_loop(top_face, LoopType::Outer, &top_loop);

    // Side wall i: bottom edge forward, up the far lateral, top edge
    // backward, down the near lateral.
    for i in 0..n {
        let j = (i + 1) % n;
        let face = store.create_face(shell, FaceSense::Positive);
        store.create_loop(
            face,
            LoopType::Outer,
            &[
                (bottom_edges[i], FinSense::Forward),
                (lateral_edges[j], FinSense::Forward),
                (top_edges[i], FinSense::Reversed),
                (lateral_edges[i], FinSense::Reversed),
            ],
        );
    }

    debug_check(&store, body);
    Ok(SweptBody {
        store,
        body,
        kind: SweepKind::Extrude { profile, direction },
    })
}

/// Revolve a planar closed profile a full turn around an axis lying in the
/// profile plane, producing a solid with periodic faces (partial angles are
/// a later issue).
///
/// The profile must not cross the axis; it may touch it along segments
/// (which vanish into the interior — a rectangle with one side on the axis
/// revolves to a cylinder) or at segment endpoints adjacent to such
/// segments (which become poles — a semicircle closed by its diameter
/// revolves to a sphere). A profile that never touches the axis revolves to
/// a genus-1 (torus-like) solid.
///
/// Topology: every off-axis profile vertex becomes a closed circular edge
/// with a single seam vertex; every profile segment off the axis becomes one
/// periodic face; on-axis segment endpoints of swept segments become pole
/// vertices carried by [`LoopType::Singular`] vertex loops.
///
/// # Errors
/// [`CoreError::InvalidArgument`] if `axis_dir` is zero/non-finite, the axis
/// does not lie in the profile plane, the profile crosses the axis, or the
/// profile touches the axis at an isolated point (a pinch would make the
/// solid non-manifold); [`CoreError::Degenerate`] if the profile lies
/// entirely on the axis.
///
/// ```
/// use opensolid_brep::curve::Curve3;
/// use opensolid_brep::sweep::{Profile, ProfileSegment, revolve};
/// use opensolid_core::types::{Point3, Vector3};
/// use std::f64::consts::FRAC_PI_2;
///
/// // Semicircle (arc + diameter on the axis) revolves to a sphere-like body.
/// let arc = Curve3::circle(Point3::origin(), Vector3::z(), 1.0)?;
/// let semicircle = Profile::new(vec![
///     ProfileSegment::new(arc, -FRAC_PI_2, FRAC_PI_2)?,
///     ProfileSegment::line_between(Point3::new(0.0, 1.0, 0.0), Point3::new(0.0, -1.0, 0.0))?,
/// ])?;
/// let sphere = revolve(&semicircle, Point3::origin(), Vector3::y())?;
/// assert!(sphere.check().is_empty());
/// let counts = sphere.store.euler_counts(sphere.body);
/// assert_eq!((counts.vertices, counts.edges, counts.faces), (2, 0, 1));
/// # Ok::<(), opensolid_core::error::CoreError>(())
/// ```
pub fn revolve(profile: &Profile, axis_point: Point3, axis_dir: Vector3) -> CoreResult<SweptBody> {
    let norm = axis_dir.norm();
    if norm == 0.0 || !norm.is_finite() {
        return Err(CoreError::InvalidArgument {
            argument: "axis_dir",
            reason: format!("must be a nonzero finite vector, got {axis_dir:?}"),
        });
    }
    if !axis_point.coords.iter().all(|c| c.is_finite()) {
        return Err(CoreError::InvalidArgument {
            argument: "axis_point",
            reason: format!("must have finite coordinates, got {axis_point:?}"),
        });
    }
    let axis = axis_dir / norm;

    let tilt = axis.dot(&profile.normal).abs();
    if tilt > TOL_SCALE {
        return Err(CoreError::InvalidArgument {
            argument: "axis_dir",
            reason: format!(
                "axis must lie in the profile plane; it makes angle \
                 {:.3e} rad with the plane",
                tilt.asin()
            ),
        });
    }
    let off_plane = (axis_point - profile.plane_point)
        .dot(&profile.normal)
        .abs();
    if off_plane > profile.tolerance {
        return Err(CoreError::InvalidArgument {
            argument: "axis_point",
            reason: format!(
                "axis must lie in the profile plane; the point is {off_plane:.3e} \
                 from it"
            ),
        });
    }

    // Signed radius: component of (p - axis_point) along the in-plane
    // direction perpendicular to the axis.
    let tol = profile.tolerance;
    let side = |profile: &Profile, p: &Point3| -> f64 {
        (p - axis_point).dot(&axis.cross(&profile.normal))
    };
    let samples = profile.sample_closed_polyline(VALIDATION_SAMPLES);
    let (mut s_min, mut s_max) = (f64::INFINITY, f64::NEG_INFINITY);
    for p in &samples {
        let s = side(profile, p);
        s_min = s_min.min(s);
        s_max = s_max.max(s);
    }
    if s_min >= -tol && s_max <= tol {
        return Err(CoreError::Degenerate {
            context: "revolve",
            reason: "profile lies entirely on the axis; there is nothing to revolve".into(),
        });
    }
    if s_min < -tol && s_max > tol {
        return Err(CoreError::InvalidArgument {
            argument: "profile",
            reason: format!(
                "profile must not cross the axis of revolution (signed radius \
                 ranges from {s_min:.3e} to {s_max:.3e})"
            ),
        });
    }
    // Normalize so the profile lies on the positive-radius side with its
    // winding counterclockwise in the (radial, axial) frame.
    let profile = if s_max <= tol {
        profile.reversed()
    } else {
        profile.clone()
    };

    let verts = profile.vertex_points();
    let n = verts.len();
    let vertex_on_axis: Vec<bool> = verts
        .iter()
        .map(|p| side(&profile, p).abs() <= tol)
        .collect();
    // A segment vanishes into the interior only if it lies *along* the
    // axis; an arc bridging two on-axis endpoints (sphere) still sweeps.
    let segment_on_axis: Vec<bool> = (0..n)
        .map(|i| {
            vertex_on_axis[i]
                && vertex_on_axis[(i + 1) % n]
                && side(&profile, &profile.segments[i].point_at_fraction(0.5)).abs() <= tol
        })
        .collect();

    for i in 0..n {
        let prev = (i + n - 1) % n;
        if vertex_on_axis[i] && !segment_on_axis[prev] && !segment_on_axis[i] {
            return Err(CoreError::InvalidArgument {
                argument: "profile",
                reason: format!(
                    "profile touches the axis at the isolated point {:?}; \
                     revolving would pinch the solid into a non-manifold body. \
                     Keep the profile off the axis or let it run along the axis",
                    verts[i]
                ),
            });
        }
    }

    let mut store = TopologyStore::new();
    let body = store.create_body(BodyType::Solid);
    let shell = store.create_shell(body, true, ShellOrientation::Outward);

    // Every off-axis profile vertex sweeps to a closed circular edge with a
    // single seam vertex at the profile's own position (angle zero).
    let circles: Vec<Option<EntityId<Edge>>> = (0..n)
        .map(|i| {
            (!vertex_on_axis[i]).then(|| {
                let seam = store.create_vertex(verts[i], SYSTEM_RESOLUTION);
                store.create_edge(seam, seam, SYSTEM_RESOLUTION)
            })
        })
        .collect();

    let project_to_axis = |p: Point3| axis_point + axis * (p - axis_point).dot(&axis);

    for i in 0..n {
        if segment_on_axis[i] {
            continue;
        }
        let j = (i + 1) % n;
        let face = store.create_face(shell, FaceSense::Positive);
        // Convention: the circle at the generating segment's end bounds the
        // face forward and is its outer loop; the circle at its start bounds
        // it reversed. Adjacent faces therefore use each circle in opposite
        // senses, giving properly opposed mates.
        match (circles[i], circles[j]) {
            (Some(start_circle), Some(end_circle)) => {
                store.create_loop(face, LoopType::Outer, &[(end_circle, FinSense::Forward)]);
                store.create_loop(face, LoopType::Inner, &[(start_circle, FinSense::Reversed)]);
            }
            (None, Some(end_circle)) => {
                store.create_loop(face, LoopType::Outer, &[(end_circle, FinSense::Forward)]);
                add_pole_loop(&mut store, face, project_to_axis(verts[i]), false);
            }
            (Some(start_circle), None) => {
                store.create_loop(face, LoopType::Outer, &[(start_circle, FinSense::Reversed)]);
                add_pole_loop(&mut store, face, project_to_axis(verts[j]), false);
            }
            (None, None) => {
                // Both endpoints on the axis (sphere-like face): two poles,
                // one of which stands in as the outer loop.
                add_pole_loop(&mut store, face, project_to_axis(verts[i]), true);
                add_pole_loop(&mut store, face, project_to_axis(verts[j]), false);
            }
        }
    }

    // A profile that never touches the axis sweeps to a torus-like shell:
    // one through-hole.
    if !vertex_on_axis.iter().any(|&b| b) {
        store.shells.get_mut(shell).expect("just created").genus = 1;
    }

    debug_check(&store, body);
    Ok(SweptBody {
        store,
        body,
        kind: SweepKind::Revolve {
            profile,
            axis_point,
            axis_dir: axis,
        },
    })
}

/// Add a degenerate vertex loop at an axis pole to `face`.
fn add_pole_loop(store: &mut TopologyStore, face: EntityId<Face>, point: Point3, as_outer: bool) {
    let pole = store.create_vertex(point, SYSTEM_RESOLUTION);
    let loop_id = store.loops.insert(Loop {
        face,
        fins: Vec::new(),
        loop_type: LoopType::Singular,
        vertex: Some(pole),
    });
    let f = store.faces.get_mut(face).expect("live face");
    if as_outer {
        f.outer_loop = Some(loop_id);
    } else {
        f.inner_loops.push(loop_id);
    }
}

/// Swept construction is deterministic; a check failure here is a kernel
/// bug, so (matching the Euler-operator policy) debug builds panic.
fn debug_check(store: &TopologyStore, body: EntityId<Body>) {
    if cfg!(debug_assertions) {
        let failures = store.check(body);
        assert!(
            failures.is_empty(),
            "sweep construction produced an invalid body: {failures:?}"
        );
    }
}

// ----------------------------------------------------------------------
// Tessellation
// ----------------------------------------------------------------------

fn extrude_mesh(profile: &Profile, direction: Vector3, resolution: usize) -> TriangleMesh {
    let ring = profile.sample_closed_polyline(resolution);
    let m = ring.len();

    let mut mesh = TriangleMesh::new();
    mesh.positions.extend(ring.iter().copied());
    mesh.positions.extend(ring.iter().map(|&p| p + direction));

    for k in 0..m {
        let k1 = (k + 1) % m;
        // Side wall quad, wound outward for a counterclockwise profile.
        mesh.indices.push([k, k1, m + k1]);
        mesh.indices.push([k, m + k1, m + k]);
    }

    // Caps: ear-clip the profile polygon so concave outlines (U/S/C) tile
    // without overlap. A centroid fan silently produced overlapping,
    // mixed-winding cap triangles for any non-star profile (of-6dw). Both
    // caps are parallel to the profile plane, so their outward normals are
    // ±profile.normal; the extrusion runs toward +direction, so the top
    // cap faces sign(profile.normal·direction)·profile.normal.
    let (e_u, e_v) = plane_basis(&profile.normal);
    let origin = ring[0];
    let ring_uv: Vec<(f64, f64)> = ring
        .iter()
        .map(|p| {
            let d = p - origin;
            (d.dot(&e_u), d.dot(&e_v))
        })
        .collect();
    // ear_clip winds triangles counterclockwise about profile.normal.
    let top_along_normal = profile.normal.dot(&direction) > 0.0;
    for [a, b, c] in ear_clip(&ring_uv) {
        if top_along_normal {
            mesh.indices.push([m + a, m + b, m + c]); // top faces +normal
            mesh.indices.push([a, c, b]); // bottom faces −normal
        } else {
            mesh.indices.push([m + a, m + c, m + b]); // top faces −normal
            mesh.indices.push([a, b, c]); // bottom faces +normal
        }
    }
    mesh
}

fn revolve_mesh(
    profile: &Profile,
    axis_point: Point3,
    axis: Vector3,
    resolution: usize,
) -> TriangleMesh {
    let radial = axis.cross(&profile.normal);
    // Right-handed rotation frame around the axis; angle zero is the
    // profile plane's positive-radius direction.
    let swing = axis.cross(&radial);
    let tol = profile.tolerance;

    let points = profile.sample_closed_polyline(resolution);
    let m = points.len();
    let coords: Vec<(f64, f64)> = points
        .iter()
        .map(|p| {
            let rel = p - axis_point;
            (rel.dot(&radial), rel.dot(&axis))
        })
        .collect();
    let on_axis: Vec<bool> = coords.iter().map(|&(s, _)| s.abs() <= tol).collect();

    let mut mesh = TriangleMesh::new();
    // Lazily allocated vertex ranges: poles get one vertex, off-axis points
    // a full ring of `resolution`, so points interior to on-axis segments
    // never enter the mesh.
    let mut first_index: Vec<Option<usize>> = vec![None; m];
    let mut index_of = |mesh: &mut TriangleMesh, j: usize, k: usize| -> usize {
        let base = *first_index[j].get_or_insert_with(|| {
            let base = mesh.positions.len();
            let (s, h) = coords[j];
            if on_axis[j] {
                mesh.positions.push(axis_point + axis * h);
            } else {
                for step in 0..resolution {
                    let theta = TWO_PI * step as f64 / resolution as f64;
                    let spoke = radial * theta.cos() + swing * theta.sin();
                    mesh.positions.push(axis_point + axis * h + spoke * s);
                }
            }
            base
        });
        if on_axis[j] { base } else { base + k }
    };

    for j in 0..m {
        let j1 = (j + 1) % m;
        if on_axis[j] && on_axis[j1] {
            continue;
        }
        for k in 0..resolution {
            let k1 = (k + 1) % resolution;
            let a_k = index_of(&mut mesh, j, k);
            let a_k1 = index_of(&mut mesh, j, k1);
            let b_k = index_of(&mut mesh, j1, k);
            let b_k1 = index_of(&mut mesh, j1, k1);
            for tri in [[a_k, b_k1, b_k], [a_k, a_k1, b_k1]] {
                if tri[0] != tri[1] && tri[1] != tri[2] && tri[0] != tri[2] {
                    mesh.indices.push(tri);
                }
            }
        }
    }
    mesh
}

/// Fill per-vertex normals with the normalized average of incident triangle
/// normals (positions and indices must already be set).
fn average_vertex_normals(mesh: &mut TriangleMesh) {
    let mut sums = vec![Vector3::zeros(); mesh.positions.len()];
    for tri in &mesh.indices {
        let e1 = mesh.positions[tri[1]] - mesh.positions[tri[0]];
        let e2 = mesh.positions[tri[2]] - mesh.positions[tri[0]];
        let face_normal = e1.cross(&e2);
        for &i in tri {
            sums[i] += face_normal;
        }
    }
    mesh.normals = sums
        .into_iter()
        .map(|sum| {
            let norm = sum.norm();
            if norm > 1e-12 {
                sum / norm
            } else {
                Vector3::zeros()
            }
        })
        .collect();
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::euler::EulerCounts;
    use std::f64::consts::{FRAC_PI_2, PI};

    fn p(x: f64, y: f64) -> Point3 {
        Point3::new(x, y, 0.0)
    }

    fn polygon(points: &[(f64, f64)]) -> Profile {
        let n = points.len();
        let segments: Vec<ProfileSegment> = (0..n)
            .map(|i| {
                let (ax, ay) = points[i];
                let (bx, by) = points[(i + 1) % n];
                ProfileSegment::line_between(p(ax, ay), p(bx, by)).expect("valid segment")
            })
            .collect();
        Profile::new(segments).expect("valid profile")
    }

    /// Unit-ish rectangle in the XY plane, counterclockwise.
    fn rectangle(width: f64, height: f64) -> Profile {
        polygon(&[(0.0, 0.0), (width, 0.0), (width, height), (0.0, height)])
    }

    fn checked_counts(body: &SweptBody) -> EulerCounts {
        let failures = body.check();
        assert!(failures.is_empty(), "check() must pass: {failures:?}");
        let counts = body.store.euler_counts(body.body);
        assert!(
            counts.euler_poincare_holds(),
            "Euler-Poincaré must hold: {counts:?}"
        );
        counts
    }

    /// Signed enclosed volume via the divergence theorem; positive for an
    /// outward-oriented closed mesh.
    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.indices
            .iter()
            .map(|tri| {
                let a = mesh.positions[tri[0]].coords;
                let b = mesh.positions[tri[1]].coords;
                let c = mesh.positions[tri[2]].coords;
                a.dot(&b.cross(&c)) / 6.0
            })
            .sum()
    }

    fn assert_volume(mesh: &TriangleMesh, expected: f64, rel_tol: f64) {
        assert!(mesh.is_closed_manifold(), "mesh must be a closed manifold");
        let vol = signed_volume(mesh);
        assert!(
            ((vol - expected) / expected).abs() < rel_tol,
            "volume {vol} differs from expected {expected}"
        );
    }

    // ------------------------------------------------------------------
    // Profile validation
    // ------------------------------------------------------------------

    #[test]
    fn profile_rejects_bad_inputs() {
        let a = p(0.0, 0.0);
        let b = p(1.0, 0.0);
        let c = p(0.0, 1.0);
        let seg = |a, b| ProfileSegment::line_between(a, b).unwrap();

        // Too few segments.
        assert!(Profile::new(vec![seg(a, b)]).is_err());
        // Open chain (does not close back to the start).
        assert!(Profile::new(vec![seg(a, b), seg(b, c)]).is_err());
        // Disconnected interior junction.
        assert!(Profile::new(vec![seg(a, b), seg(c, a), seg(b, c)]).is_err());
        // Non-planar loop.
        let lift = Point3::new(1.0, 1.0, 0.5);
        assert!(
            Profile::new(vec![seg(a, b), seg(b, lift), seg(lift, c), seg(c, a)]).is_err(),
            "non-planar profile must be rejected"
        );
        // Zero enclosed area (degenerate back-and-forth loop).
        assert!(Profile::new(vec![seg(a, b), seg(b, a)]).is_err());
    }

    #[test]
    fn profile_segment_rejects_bad_parameters() {
        let circle = Curve3::circle(Point3::origin(), Vector3::z(), 1.0).unwrap();
        assert!(ProfileSegment::new(circle.clone(), 0.0, f64::NAN).is_err());
        assert!(ProfileSegment::new(circle.clone(), 1.0, 1.0).is_err());
        // Full period must be split into multiple segments.
        assert!(ProfileSegment::new(circle, 0.0, TWO_PI).is_err());
        // Coincident line endpoints.
        assert!(ProfileSegment::line_between(p(1.0, 1.0), p(1.0, 1.0)).is_err());
    }

    #[test]
    fn profile_winding_sets_the_normal() {
        let ccw = rectangle(2.0, 1.0);
        assert!((ccw.normal() - Vector3::z()).norm() < 1e-9);
        let cw = polygon(&[(0.0, 0.0), (0.0, 1.0), (2.0, 1.0), (2.0, 0.0)]);
        assert!((cw.normal() + Vector3::z()).norm() < 1e-9);
    }

    // ------------------------------------------------------------------
    // Extrude
    // ------------------------------------------------------------------

    #[test]
    fn extruded_rectangle_matches_block_topology() {
        let block = extrude(&rectangle(2.0, 1.0), Vector3::new(0.0, 0.0, 3.0)).unwrap();
        let counts = checked_counts(&block);
        assert_eq!(counts.vertices, 8);
        assert_eq!(counts.edges, 12);
        assert_eq!(counts.faces, 6);
        assert_eq!(counts.rings, 0);
        assert_eq!(counts.shells, 1);
        assert_eq!(counts.genus, 0);
    }

    #[test]
    fn extruded_block_edges_are_manifold_with_opposed_mates() {
        let block = extrude(&rectangle(2.0, 1.0), Vector3::new(0.0, 0.0, 3.0)).unwrap();
        let store = &block.store;
        for &face in &store.faces_of_body(block.body) {
            for edge in store.edges_of_face(face) {
                let fins = store.fins_of_edge(edge);
                assert_eq!(fins.len(), 2, "every block edge bounds two faces");
                let (a, b) = (store.fin(fins[0]).unwrap(), store.fin(fins[1]).unwrap());
                assert_eq!(a.sense, b.sense.opposite(), "mates traverse oppositely");
            }
        }
    }

    #[test]
    fn extruded_rectangle_meshes_to_exact_block_volume() {
        let block = extrude(&rectangle(2.0, 1.0), Vector3::new(0.0, 0.0, 3.0)).unwrap();
        let mesh = block.tessellate(8).unwrap();
        assert_volume(&mesh, 6.0, 1e-9);
    }

    #[test]
    fn oblique_extrusion_keeps_cavalieri_volume() {
        // Shearing the direction leaves base area x normal height unchanged.
        let block = extrude(&rectangle(2.0, 1.0), Vector3::new(1.5, -0.5, 3.0)).unwrap();
        checked_counts(&block);
        assert_volume(&block.tessellate(8).unwrap(), 6.0, 1e-9);
    }

    #[test]
    fn extruding_against_the_winding_still_builds_an_outward_solid() {
        let block = extrude(&rectangle(2.0, 1.0), Vector3::new(0.0, 0.0, -3.0)).unwrap();
        checked_counts(&block);
        // Positive signed volume == outward orientation.
        assert_volume(&block.tessellate(8).unwrap(), 6.0, 1e-9);
    }

    #[test]
    fn concave_profile_caps_tile_without_overlap() {
        // U-profile: a centroid fan spills across the notch and covers it
        // with overlapping, mixed-winding cap triangles (of-6dw). Signed
        // volume still cancels correctly and every edge is still used
        // twice, so is_closed_manifold and the volume check both pass —
        // total_area is what exposes the overlap: it inflates when the
        // caps double back over the notch.
        let u = polygon(&[
            (0.0, 0.0),
            (3.0, 0.0),
            (3.0, 3.0),
            (2.0, 3.0),
            (2.0, 1.0),
            (1.0, 1.0),
            (1.0, 3.0),
            (0.0, 3.0),
        ]);
        let mesh = extrude(&u, Vector3::z()).unwrap().tessellate(16).unwrap();
        // Base area 7, height 1.
        assert_volume(&mesh, 7.0, 1e-9);
        // Two caps (2·7) plus side walls (perimeter 16 × height 1).
        let expected_area = 2.0 * 7.0 + 16.0;
        assert!(
            (mesh.total_area() - expected_area).abs() < 1e-9,
            "cap triangles overlap: total area {} != {expected_area}",
            mesh.total_area()
        );
    }

    #[test]
    fn extruded_stadium_profile_handles_arcs() {
        // Rectangle with semicircular ends: two lines + two half arcs.
        let right = Curve3::circle(p(1.0, 0.0), Vector3::z(), 0.5).unwrap();
        let left = Curve3::circle(p(-1.0, 0.0), Vector3::z(), 0.5).unwrap();
        let stadium = Profile::new(vec![
            ProfileSegment::line_between(p(-1.0, -0.5), p(1.0, -0.5)).unwrap(),
            ProfileSegment::new(right, -FRAC_PI_2, FRAC_PI_2).unwrap(),
            ProfileSegment::line_between(p(1.0, 0.5), p(-1.0, 0.5)).unwrap(),
            ProfileSegment::new(left, FRAC_PI_2, 3.0 * FRAC_PI_2).unwrap(),
        ])
        .unwrap();

        let solid = extrude(&stadium, Vector3::z()).unwrap();
        let counts = checked_counts(&solid);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (8, 12, 6));

        let area = 2.0 + PI * 0.25;
        assert_volume(&solid.tessellate(64).unwrap(), area, 1e-2);
    }

    #[test]
    fn extrude_rejects_in_plane_and_degenerate_directions() {
        let profile = rectangle(2.0, 1.0);
        assert!(extrude(&profile, Vector3::x()).is_err());
        assert!(extrude(&profile, Vector3::zeros()).is_err());
        assert!(extrude(&profile, Vector3::new(f64::NAN, 0.0, 1.0)).is_err());
        let err = extrude(&profile, Vector3::new(1.0, 1.0, 0.0)).unwrap_err();
        assert!(err.to_string().contains("profile plane"), "teaching: {err}");
    }

    // ------------------------------------------------------------------
    // Revolve
    // ------------------------------------------------------------------

    /// Rectangle with its left side on the Y axis: revolves to a cylinder.
    fn cylinder_profile(radius: f64, height: f64) -> Profile {
        polygon(&[(0.0, 0.0), (radius, 0.0), (radius, height), (0.0, height)])
    }

    #[test]
    fn revolved_rectangle_is_cylinder_like() {
        let cylinder =
            revolve(&cylinder_profile(1.0, 2.0), Point3::origin(), Vector3::y()).unwrap();
        let counts = checked_counts(&cylinder);
        // Two rim circles with one seam vertex each, two cap poles, three
        // periodic faces (bottom disk, wall, top disk).
        assert_eq!(counts.vertices, 4);
        assert_eq!(counts.edges, 2);
        assert_eq!(counts.faces, 3);
        assert_eq!(counts.rings, 3);
        assert_eq!(counts.genus, 0);
    }

    #[test]
    fn revolved_rectangle_meshes_to_cylinder_volume() {
        let cylinder =
            revolve(&cylinder_profile(1.0, 2.0), Point3::origin(), Vector3::y()).unwrap();
        let mesh = cylinder.tessellate(64).unwrap();
        assert_volume(&mesh, PI * 2.0, 1e-2);
    }

    /// Semicircle closed by its diameter along the Y axis.
    fn semicircle_profile(radius: f64) -> Profile {
        let arc = Curve3::circle(Point3::origin(), Vector3::z(), radius).unwrap();
        Profile::new(vec![
            ProfileSegment::new(arc, -FRAC_PI_2, FRAC_PI_2).unwrap(),
            ProfileSegment::line_between(p(0.0, radius), p(0.0, -radius)).unwrap(),
        ])
        .unwrap()
    }

    #[test]
    fn revolved_semicircle_is_sphere_like() {
        let sphere = revolve(&semicircle_profile(1.0), Point3::origin(), Vector3::y()).unwrap();
        let counts = checked_counts(&sphere);
        // One periodic face bounded by two poles; no edges at all.
        assert_eq!(counts.vertices, 2);
        assert_eq!(counts.edges, 0);
        assert_eq!(counts.faces, 1);
        assert_eq!(counts.rings, 1);
        assert_eq!(counts.genus, 0);
    }

    #[test]
    fn revolved_semicircle_meshes_to_sphere_volume() {
        let sphere = revolve(&semicircle_profile(1.0), Point3::origin(), Vector3::y()).unwrap();
        let mesh = sphere.tessellate(64).unwrap();
        assert_volume(&mesh, 4.0 * PI / 3.0, 1e-2);
    }

    #[test]
    fn revolved_offaxis_square_is_torus_like() {
        let square = polygon(&[(2.0, 0.0), (3.0, 0.0), (3.0, 1.0), (2.0, 1.0)]);
        let torus = revolve(&square, Point3::origin(), Vector3::y()).unwrap();
        let counts = checked_counts(&torus);
        assert_eq!(counts.vertices, 4);
        assert_eq!(counts.edges, 4);
        assert_eq!(counts.faces, 4);
        assert_eq!(counts.rings, 4);
        assert_eq!(counts.genus, 1, "an off-axis profile sweeps a torus");

        // Pappus: V = 2π * centroid radius * area.
        let mesh = torus.tessellate(64).unwrap();
        assert_volume(&mesh, TWO_PI * 2.5, 1e-2);
    }

    #[test]
    fn revolved_triangle_is_bicone_like() {
        // Triangle with one side on the axis: two cone faces sharing a rim.
        let triangle = polygon(&[(0.0, 0.0), (1.0, 1.0), (0.0, 2.0)]);
        let bicone = revolve(&triangle, Point3::origin(), Vector3::y()).unwrap();
        let counts = checked_counts(&bicone);
        assert_eq!(counts.vertices, 3); // 1 rim seam + 2 apex poles
        assert_eq!(counts.edges, 1); // the shared rim circle
        assert_eq!(counts.faces, 2);
        assert_eq!(counts.rings, 2);

        // Two cones of radius 1, height 1 each.
        let mesh = bicone.tessellate(64).unwrap();
        assert_volume(&mesh, 2.0 * PI / 3.0, 1e-2);
    }

    #[test]
    fn revolve_normalizes_winding_and_axis_side() {
        // Clockwise winding on the negative-radius side still produces an
        // outward-oriented cylinder.
        let cw_negative = polygon(&[(0.0, 0.0), (0.0, 2.0), (-1.0, 2.0), (-1.0, 0.0)]);
        let cylinder = revolve(&cw_negative, Point3::origin(), Vector3::y()).unwrap();
        let counts = checked_counts(&cylinder);
        assert_eq!((counts.vertices, counts.edges, counts.faces), (4, 2, 3));
        assert_volume(&cylinder.tessellate(64).unwrap(), PI * 2.0, 1e-2);
    }

    #[test]
    fn revolve_rejects_invalid_axes_and_profiles() {
        let profile = cylinder_profile(1.0, 2.0);
        // Degenerate axis.
        assert!(revolve(&profile, Point3::origin(), Vector3::zeros()).is_err());
        // Axis not in the profile plane (out-of-plane direction).
        assert!(revolve(&profile, Point3::origin(), Vector3::z()).is_err());
        // Axis parallel to the plane's Y but offset out of the plane.
        assert!(revolve(&profile, Point3::new(0.0, 0.0, 1.0), Vector3::y()).is_err());
        // Profile crossing the axis.
        let crossing = polygon(&[(-1.0, 0.0), (1.0, 0.0), (1.0, 1.0), (-1.0, 1.0)]);
        let err = revolve(&crossing, Point3::origin(), Vector3::y()).unwrap_err();
        assert!(err.to_string().contains("cross"), "teaching: {err}");
        // Pinch: touches the axis at an isolated vertex.
        let pinch = polygon(&[(0.0, 0.0), (1.0, -1.0), (2.0, 0.0), (1.0, 1.0)]);
        let err = revolve(&pinch, Point3::origin(), Vector3::y()).unwrap_err();
        assert!(err.to_string().contains("pinch"), "teaching: {err}");
    }

    #[test]
    fn tessellate_rejects_low_resolution() {
        let block = extrude(&rectangle(1.0, 1.0), Vector3::z()).unwrap();
        let err = block.tessellate(3).unwrap_err();
        assert!(
            matches!(
                &err,
                CoreError::InvalidArgument {
                    argument: "resolution",
                    ..
                }
            ),
            "unexpected error: {err}"
        );
    }

    #[test]
    fn tessellated_meshes_have_unit_normals() {
        let sphere = revolve(&semicircle_profile(1.0), Point3::origin(), Vector3::y()).unwrap();
        let mesh = sphere.tessellate(16).unwrap();
        assert_eq!(mesh.normals.len(), mesh.positions.len());
        for (i, n) in mesh.normals.iter().enumerate() {
            if mesh.indices.iter().any(|tri| tri.contains(&i)) {
                assert!((n.norm() - 1.0).abs() < 1e-9, "normal {i} not unit: {n:?}");
                // For a sphere around the origin, normals point radially out.
                let radial = mesh.positions[i].coords.normalize();
                assert!(n.dot(&radial) > 0.9, "normal {i} should point outward");
            }
        }
    }
}
