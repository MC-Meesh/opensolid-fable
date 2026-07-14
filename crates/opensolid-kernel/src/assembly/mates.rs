//! Mate vocabulary: the abstract features a mate references and the mate
//! constraints themselves (Assembly MVP 2, of-fsl.25.2).
//!
//! These types are deliberately decoupled from part geometry. A [`Feature`] is
//! a plane / axis / point expressed in a part's *local* frame; resolving a
//! named part feature (a fillet face, a reference axis) down to a [`Feature`]
//! is the caller's job. The [`solver`](super::solver) transforms these
//! features by candidate instance poses — it never touches a [`Part`](super::Part)
//! or a [`Shape`](opensolid_frep::Shape) — which keeps the rigid-body constraint
//! engine a pure, independently testable numerical layer.

use opensolid_core::types::{Point3, Transform3, Vector3};
use thiserror::Error;

/// Errors from constructing mate features and mates.
///
/// Distinct from [`AssemblyError`](super::AssemblyError), which covers the
/// aggregate mass-property queries; this covers the mate layer.
#[derive(Debug, Clone, PartialEq, Error)]
#[non_exhaustive]
pub enum MateError {
    /// A plane normal or axis direction was too short to normalize.
    #[error("degenerate {what}: direction has near-zero length")]
    DegenerateFeature {
        /// Which feature was degenerate (`"plane normal"`, `"axis direction"`).
        what: &'static str,
    },

    /// A mate was given a feature pair it does not support.
    #[error("{kind:?} mate does not accept this feature pair: {reason}")]
    FeatureMismatch {
        /// The mate kind that rejected the pair.
        kind: MateKind,
        /// Which pairings are valid.
        reason: &'static str,
    },

    /// A [`MateKind::Distance`] mate was created without an offset value.
    #[error("distance mate requires an offset value")]
    MissingValue,
}

/// A geometric feature in a part's local frame — the anchor a mate references.
///
/// Directions (`normal`, `direction`) are stored unit-length; the constructors
/// normalize and reject degenerate input. Points carry no direction.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Feature {
    /// A planar face / reference plane: a point on the plane and its outward
    /// unit normal.
    Plane {
        /// A point lying on the plane.
        point: Point3,
        /// Outward unit normal.
        normal: Vector3,
    },
    /// A cylindrical/conical axis or reference axis: a point on the line and
    /// its unit direction.
    Axis {
        /// A point on the axis line.
        point: Point3,
        /// Unit direction of the axis.
        direction: Vector3,
    },
    /// A reference point / vertex.
    Point {
        /// The point location.
        point: Point3,
    },
}

impl Feature {
    /// A plane feature; `normal` is normalized. Errors if `normal` is
    /// near-zero.
    pub fn plane(point: Point3, normal: Vector3) -> Result<Self, MateError> {
        Ok(Feature::Plane {
            point,
            normal: normalize(normal, "plane normal")?,
        })
    }

    /// An axis feature; `direction` is normalized. Errors if it is near-zero.
    pub fn axis(point: Point3, direction: Vector3) -> Result<Self, MateError> {
        Ok(Feature::Axis {
            point,
            direction: normalize(direction, "axis direction")?,
        })
    }

    /// A point feature.
    pub fn point(point: Point3) -> Self {
        Feature::Point { point }
    }

    /// This feature expressed in world space under the rigid pose `iso`.
    ///
    /// A rigid isometry rotates directions and maps points; normals and axis
    /// directions stay unit-length under rotation.
    pub(crate) fn to_world(self, iso: &Transform3) -> Feature {
        match self {
            Feature::Plane { point, normal } => Feature::Plane {
                point: iso * point,
                normal: iso.rotation * normal,
            },
            Feature::Axis { point, direction } => Feature::Axis {
                point: iso * point,
                direction: iso.rotation * direction,
            },
            Feature::Point { point } => Feature::Point { point: iso * point },
        }
    }
}

fn normalize(v: Vector3, what: &'static str) -> Result<Vector3, MateError> {
    let n = v.norm();
    // Matches the kernel's angular/linear resolution floor: a direction this
    // short carries no reliable orientation.
    if n < 1e-12 {
        return Err(MateError::DegenerateFeature { what });
    }
    Ok(v / n)
}

/// A reference to one instance's feature: which instance owns it, and the
/// feature geometry in that instance's local (part) frame.
///
/// The solver transforms `feature` by the instance's current pose each
/// iteration, so the same part referenced by two instances yields two
/// independent world-space features.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FeatureRef {
    /// Index of the owning instance in [`Assembly::instances`](super::Assembly::instances).
    pub instance: usize,
    /// The feature in the instance's local frame.
    pub feature: Feature,
}

impl FeatureRef {
    /// Construct a reference to `feature` on instance `instance`.
    pub fn new(instance: usize, feature: Feature) -> Self {
        Self { instance, feature }
    }
}

/// The kind of constraint a mate imposes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MateKind {
    /// Two planar faces flush and anti-parallel, or a point on a plane.
    Coincident,
    /// Two axes collinear (a shaft in a bore).
    Concentric,
    /// Two planes a fixed signed offset apart, or two points a fixed distance
    /// apart.
    Distance,
}

/// A constraint between a feature on one instance and a feature on another.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Mate {
    /// The constraint kind.
    pub kind: MateKind,
    /// First feature reference.
    pub a: FeatureRef,
    /// Second feature reference.
    pub b: FeatureRef,
    /// Offset (for [`Distance`](MateKind::Distance)); `None` for the others.
    pub value: Option<f64>,
}

impl Mate {
    /// A coincident mate. Accepts plane–plane (flush + anti-parallel) or
    /// point–plane (point lies on the plane).
    pub fn coincident(a: FeatureRef, b: FeatureRef) -> Result<Self, MateError> {
        match (a.feature, b.feature) {
            (Feature::Plane { .. }, Feature::Plane { .. })
            | (Feature::Point { .. }, Feature::Plane { .. })
            | (Feature::Plane { .. }, Feature::Point { .. }) => Ok(Mate {
                kind: MateKind::Coincident,
                a,
                b,
                value: None,
            }),
            _ => Err(MateError::FeatureMismatch {
                kind: MateKind::Coincident,
                reason: "coincident accepts plane–plane or point–plane",
            }),
        }
    }

    /// A concentric mate: two axes made collinear.
    pub fn concentric(a: FeatureRef, b: FeatureRef) -> Result<Self, MateError> {
        match (a.feature, b.feature) {
            (Feature::Axis { .. }, Feature::Axis { .. }) => Ok(Mate {
                kind: MateKind::Concentric,
                a,
                b,
                value: None,
            }),
            _ => Err(MateError::FeatureMismatch {
                kind: MateKind::Concentric,
                reason: "concentric accepts axis–axis",
            }),
        }
    }

    /// A distance mate at signed offset `value`. Accepts plane–plane (offset
    /// along the first plane's normal) or point–point (Euclidean separation).
    pub fn distance(a: FeatureRef, b: FeatureRef, value: f64) -> Result<Self, MateError> {
        match (a.feature, b.feature) {
            (Feature::Plane { .. }, Feature::Plane { .. })
            | (Feature::Point { .. }, Feature::Point { .. }) => Ok(Mate {
                kind: MateKind::Distance,
                a,
                b,
                value: Some(value),
            }),
            _ => Err(MateError::FeatureMismatch {
                kind: MateKind::Distance,
                reason: "distance accepts plane–plane or point–point",
            }),
        }
    }

    /// True when both feature references point at in-range instances and the
    /// feature pairing matches the kind. The solver tolerates a malformed mate
    /// (it contributes no residual), but callers can check up front.
    pub fn is_valid(&self, instance_count: usize) -> bool {
        if self.a.instance >= instance_count || self.b.instance >= instance_count {
            return false;
        }
        match self.kind {
            MateKind::Coincident => Mate::coincident(self.a, self.b).is_ok(),
            MateKind::Concentric => Mate::concentric(self.a, self.b).is_ok(),
            MateKind::Distance => self
                .value
                .is_some_and(|v| Mate::distance(self.a, self.b, v).is_ok()),
        }
    }
}
