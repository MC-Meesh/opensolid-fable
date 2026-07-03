//! Error types shared across the OpenSolid crates.
//!
//! See the crate-level documentation for the error-handling policy these
//! types implement.

use thiserror::Error;

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
    fn implements_std_error() {
        let err: Box<dyn std::error::Error> = Box::new(CoreError::NotImplemented {
            feature: "sessions",
        });
        assert!(err.source().is_none());
    }
}
