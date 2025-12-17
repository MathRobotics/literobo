use thiserror::Error;

#[derive(Debug, Error)]
pub enum KinematicsError {
    #[error("failed to parse URDF: {0}")]
    Parse(String),
    #[error("link `{0}` was not found in the URDF")]
    UnknownLink(String),
    #[error("no kinematic path found from `{base}` to `{end}`")]
    NoPath { base: String, end: String },
    #[error("joint `{0}` uses an unsupported type")]
    UnsupportedJoint(String),
    #[error("joint `{0}` must have a non-zero axis")]
    InvalidAxis(String),
    #[error("joint state length mismatch: expected {expected}, got {provided}")]
    StateLength { expected: usize, provided: usize },
}
