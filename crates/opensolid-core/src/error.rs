//! Error types shared across the OpenSolid crates.
//!
//! See the crate-level documentation for the error-handling policy these
//! types implement.

use thiserror::Error;

use crate::mesh::ManifoldDefects;

/// Error returned by fallible public APIs in the OpenSolid crates.
///
/// Messages are written to be actionable: they name the offending argument
/// or operation and state the constraint that was violated, so a caller can
/// fix the input without reading the source.
#[derive(Debug, Clone, PartialEq, Error)]
#[non_exhaustive]
pub enum CoreError {
    /// A caller-supplied argument violated a documented constraint.
    #[error("invalid argument `{argument}`: {reason}")]
    InvalidArgument {
        /// Name of the offending parameter.
        argument: &'static str,
        /// The violated constraint, phrased for the caller.
        reason: String,
    },

    /// Input geometry is degenerate for the requested operation
    /// (zero-length direction, inverted bounding box, ...).
    #[error("degenerate geometry in {context}: {reason}")]
    Degenerate {
        /// Operation or constructor that rejected the geometry.
        context: &'static str,
        /// What makes the geometry degenerate.
        reason: String,
    },

    /// A computed quantity exceeded the allowed tolerance.
    #[error(
        "tolerance violation in {context}: deviation {deviation:e} exceeds tolerance {tolerance:e}"
    )]
    ToleranceViolation {
        /// Operation whose result was out of tolerance.
        context: &'static str,
        /// Measured deviation.
        deviation: f64,
        /// Maximum allowed deviation.
        tolerance: f64,
    },

    /// The requested capability is not implemented yet.
    #[error("not implemented: {feature}")]
    NotImplemented {
        /// The missing capability.
        feature: &'static str,
    },

    /// Meshing produced a surface that is not a closed manifold.
    ///
    /// Carries the defect counts rather than prose so callers can branch on
    /// *which* defect occurred: an open rim and a pinched edge have opposite
    /// remedies (see [`ManifoldDefects`]).
    #[error("non-manifold mesh in {context}: {}", .defects.describe().unwrap_or_else(|| "no defect recorded".to_string()))]
    NonManifold {
        /// Operation whose meshing failed to close.
        context: &'static str,
        /// What kept the mesh from being a closed manifold, by kind.
        defects: ManifoldDefects,
    },
}

impl CoreError {
    /// Stable machine-readable code, one per variant.
    ///
    /// These are part of the agent-facing contract (they cross the wasm
    /// boundary as `error.code`); never rename an existing code.
    pub fn code(&self) -> &'static str {
        match self {
            CoreError::InvalidArgument { .. } => "invalid_argument",
            CoreError::Degenerate { .. } => "degenerate_geometry",
            CoreError::ToleranceViolation { .. } => "tolerance_violation",
            CoreError::NotImplemented { .. } => "not_implemented",
            CoreError::NonManifold { .. } => "non_manifold_mesh",
        }
    }

    /// Coarse grouping for callers that branch on failure class rather than
    /// exact code: can the input be fixed, is the geometry itself bad, ...
    pub fn category(&self) -> &'static str {
        match self {
            CoreError::InvalidArgument { .. } => "argument",
            CoreError::Degenerate { .. } | CoreError::NonManifold { .. } => "geometry",
            CoreError::ToleranceViolation { .. } => "tolerance",
            CoreError::NotImplemented { .. } => "unsupported",
        }
    }

    /// A concrete next step for the caller, where one is known beyond what
    /// the message already states. `None` when the message itself names the
    /// violated constraint and the fix is to satisfy it.
    pub fn hint(&self) -> Option<String> {
        match self {
            CoreError::InvalidArgument { .. } | CoreError::Degenerate { .. } => None,
            CoreError::ToleranceViolation { .. } => Some(
                "The measured deviation exceeds the allowed tolerance; loosen the \
                 tolerance or adjust the inputs so the surfaces meet more closely."
                    .to_string(),
            ),
            CoreError::NotImplemented { .. } => Some(
                "This capability is not available in this build; restructure the \
                 model to avoid it."
                    .to_string(),
            ),
            CoreError::NonManifold { defects, .. } => {
                let mut parts = Vec::new();
                if defects.empty {
                    parts.push(
                        "The mesh is empty — the surface likely lies entirely \
                         outside the meshing bounds.",
                    );
                }
                if defects.boundary_edges > 0 {
                    parts.push(
                        "Open edges mean the surface reaches the meshing bounds; \
                         enlarge the bounds or shrink the shape so the surface \
                         closes strictly inside them.",
                    );
                }
                if defects.pinched_edges > 0 {
                    parts.push(
                        "Pinched edges are a mesher defect at near-tangent \
                         features; a finer accuracy does not reliably clear them — \
                         nudge the feature size or the overall proportions instead.",
                    );
                }
                if defects.misoriented_edges > 0 || defects.degenerate_triangles > 0 {
                    parts.push(
                        "Inconsistent orientation or degenerate triangles usually \
                         respond to a different accuracy; retry with a finer one.",
                    );
                }
                if parts.is_empty() {
                    None
                } else {
                    Some(parts.join(" "))
                }
            }
        }
    }
}

/// Convenience alias for results of fallible OpenSolid public APIs.
pub type CoreResult<T> = Result<T, CoreError>;

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn invalid_argument_message_names_argument_and_constraint() {
        let err = CoreError::InvalidArgument {
            argument: "radius",
            reason: "must be positive and finite, got -1".into(),
        };
        let msg = err.to_string();
        assert!(msg.contains("`radius`"), "missing argument name: {msg}");
        assert!(
            msg.contains("must be positive"),
            "missing constraint: {msg}"
        );
        assert!(msg.contains("-1"), "missing offending value: {msg}");
    }

    #[test]
    fn degenerate_message_names_context_and_reason() {
        let err = CoreError::Degenerate {
            context: "BoundingBox3::new",
            reason: "min exceeds max on axis x".into(),
        };
        let msg = err.to_string();
        assert!(msg.contains("BoundingBox3::new"), "missing context: {msg}");
        assert!(msg.contains("min exceeds max"), "missing reason: {msg}");
    }

    #[test]
    fn tolerance_violation_message_includes_both_values() {
        let err = CoreError::ToleranceViolation {
            context: "surface fit",
            deviation: 0.5,
            tolerance: 1e-6,
        };
        let msg = err.to_string();
        assert!(msg.contains("surface fit"), "missing context: {msg}");
        assert!(msg.contains("5e-1"), "missing deviation: {msg}");
        assert!(msg.contains("1e-6"), "missing tolerance: {msg}");
    }

    #[test]
    fn not_implemented_message_names_feature() {
        let err = CoreError::NotImplemented {
            feature: "NURBS surface intersection",
        };
        assert_eq!(
            err.to_string(),
            "not implemented: NURBS surface intersection"
        );
    }

    #[test]
    fn non_manifold_message_names_context_and_defects() {
        let err = CoreError::NonManifold {
            context: "sdf_to_brep",
            defects: ManifoldDefects {
                pinched_edges: 2,
                ..ManifoldDefects::default()
            },
        };
        let msg = err.to_string();
        assert!(msg.contains("sdf_to_brep"), "missing context: {msg}");
        assert!(msg.contains("2 pinched edge"), "missing defect: {msg}");
    }

    #[test]
    fn every_variant_has_stable_code_and_category() {
        let cases = [
            (
                CoreError::InvalidArgument {
                    argument: "radius",
                    reason: "must be positive".into(),
                },
                "invalid_argument",
                "argument",
            ),
            (
                CoreError::Degenerate {
                    context: "sweep",
                    reason: "zero-length path".into(),
                },
                "degenerate_geometry",
                "geometry",
            ),
            (
                CoreError::ToleranceViolation {
                    context: "surface fit",
                    deviation: 0.5,
                    tolerance: 1e-6,
                },
                "tolerance_violation",
                "tolerance",
            ),
            (
                CoreError::NotImplemented { feature: "fillets" },
                "not_implemented",
                "unsupported",
            ),
            (
                CoreError::NonManifold {
                    context: "sdf_to_brep",
                    defects: ManifoldDefects {
                        boundary_edges: 3,
                        ..ManifoldDefects::default()
                    },
                },
                "non_manifold_mesh",
                "geometry",
            ),
        ];
        for (err, code, category) in cases {
            assert_eq!(err.code(), code, "{err}");
            assert_eq!(err.category(), category, "{err}");
        }
    }

    #[test]
    fn non_manifold_hint_matches_defect_kind() {
        let rim = CoreError::NonManifold {
            context: "sdf_to_brep",
            defects: ManifoldDefects {
                boundary_edges: 4,
                ..ManifoldDefects::default()
            },
        };
        let hint = rim.hint().expect("open rim has a hint");
        assert!(hint.contains("meshing bounds"), "{hint}");

        let pinch = CoreError::NonManifold {
            context: "sdf_to_brep",
            defects: ManifoldDefects {
                pinched_edges: 1,
                ..ManifoldDefects::default()
            },
        };
        let hint = pinch.hint().expect("pinch has a hint");
        assert!(
            hint.contains("finer accuracy does not reliably clear"),
            "{hint}"
        );

        // Message-is-the-fix variants stay hintless rather than padding
        // the payload with restatements.
        let arg = CoreError::InvalidArgument {
            argument: "radius",
            reason: "must be positive".into(),
        };
        assert_eq!(arg.hint(), None);
    }

    #[test]
    fn implements_std_error() {
        let err: Box<dyn std::error::Error> = Box::new(CoreError::NotImplemented {
            feature: "sessions",
        });
        assert!(err.source().is_none());
    }
}
