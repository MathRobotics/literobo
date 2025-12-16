use nalgebra::{Isometry3, Matrix6xX, Rotation3, Translation3, Unit, UnitQuaternion, Vector3};
use numpy::{PyArray2, ToPyArray};
use pyo3::exceptions::PyValueError;
use pyo3::prelude::*;
use pyo3::wrap_pyfunction;
use thiserror::Error;
use urdf_rs::{JointType, Robot};

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

#[derive(Clone, Copy, Debug)]
enum JointKind {
    Revolute,
    Prismatic,
    Fixed,
}

#[derive(Clone, Debug)]
struct ChainJoint {
     #[allow(dead_code)]
    name: String,
     #[allow(dead_code)]
    parent: String,
     #[allow(dead_code)]
    child: String,
    origin: Isometry3<f64>,
    axis: Vector3<f64>,
    kind: JointKind,
}

#[derive(Clone, Debug)]
pub struct KinematicChain {
    joints: Vec<ChainJoint>,
    base: String,
    end: String,
    dof: usize,
}

impl KinematicChain {
    pub fn from_urdf_str(
        urdf: &str,
        base_link: impl Into<String>,
        end_link: impl Into<String>,
    ) -> Result<Self, KinematicsError> {
        let robot = urdf_rs::read_from_string(urdf)
            .map_err(|err| KinematicsError::Parse(err.to_string()))?;
        Self::from_robot(robot, base_link, end_link)
    }

    pub fn from_urdf_file(
        path: impl AsRef<std::path::Path>,
        base_link: impl Into<String>,
        end_link: impl Into<String>,
    ) -> Result<Self, KinematicsError> {
        let robot =
            urdf_rs::read_file(path).map_err(|err| KinematicsError::Parse(err.to_string()))?;
        Self::from_robot(robot, base_link, end_link)
    }

    fn from_robot(
        robot: Robot,
        base_link: impl Into<String>,
        end_link: impl Into<String>,
    ) -> Result<Self, KinematicsError> {
        let base_link = base_link.into();
        let end_link = end_link.into();

        if !robot.links.iter().any(|l| l.name == base_link) {
            return Err(KinematicsError::UnknownLink(base_link));
        }
        if !robot.links.iter().any(|l| l.name == end_link) {
            return Err(KinematicsError::UnknownLink(end_link));
        }

        let mut adjacency: std::collections::HashMap<String, Vec<&urdf_rs::Joint>> =
            std::collections::HashMap::new();
        for joint in &robot.joints {
            adjacency
                .entry(joint.parent.link.clone())
                .or_default()
                .push(joint);
        }

        let mut queue = std::collections::VecDeque::new();
        queue.push_back(base_link.clone());
        let mut predecessors: std::collections::HashMap<String, (&urdf_rs::Joint, String)> =
            std::collections::HashMap::new();

        while let Some(link) = queue.pop_front() {
            if let Some(children) = adjacency.get(&link) {
                for joint in children {
                    let child = joint.child.link.clone();
                    if predecessors.contains_key(&child) || child == base_link {
                        continue;
                    }
                    predecessors.insert(child.clone(), (*joint, link.clone()));
                    if child == end_link {
                        break;
                    }
                    queue.push_back(child);
                }
            }
        }

        if !predecessors.contains_key(&end_link) {
            return Err(KinematicsError::NoPath {
                base: base_link.clone(),
                end: end_link.clone(),
            });
        }

        let mut chain: Vec<ChainJoint> = Vec::new();
        let mut current = end_link.clone();
        while current != base_link {
            let (joint, parent) = predecessors
                .get(&current)
                .expect("path reconstruction requires predecessors")
                .clone();
            chain.push(Self::chain_joint_from_urdf(joint)?);
            current = parent;
        }

        chain.reverse();
        let dof = chain
            .iter()
            .filter(|j| matches!(j.kind, JointKind::Prismatic | JointKind::Revolute))
            .count();

        Ok(KinematicChain {
            joints: chain,
            base: base_link,
            end: end_link,
            dof,
        })
    }

    fn chain_joint_from_urdf(joint: &urdf_rs::Joint) -> Result<ChainJoint, KinematicsError> {
        let origin = origin_to_isometry(&joint.origin);
        let raw_axis = {
            let xyz = joint.axis.xyz;
            Vector3::new(xyz[0], xyz[1], xyz[2])
        };

        let kind = match joint.joint_type {
            JointType::Revolute | JointType::Continuous => JointKind::Revolute,
            JointType::Prismatic => JointKind::Prismatic,
            JointType::Fixed => JointKind::Fixed,
            _ => return Err(KinematicsError::UnsupportedJoint(joint.name.clone())),
        };

        if matches!(kind, JointKind::Revolute | JointKind::Prismatic) && raw_axis.norm() == 0.0 {
            return Err(KinematicsError::InvalidAxis(joint.name.clone()));
        }

        let axis = if raw_axis.norm() > 0.0 {
            raw_axis.normalize()
        } else {
            raw_axis
        };

        Ok(ChainJoint {
            name: joint.name.clone(),
            parent: joint.parent.link.clone(),
            child: joint.child.link.clone(),
            origin,
            axis,
            kind,
        })
    }

    pub fn dof(&self) -> usize {
        self.dof
    }

    pub fn base(&self) -> &str {
        &self.base
    }

    pub fn end(&self) -> &str {
        &self.end
    }

    pub fn forward_kinematics(
        &self,
        joint_positions: &[f64],
    ) -> Result<Isometry3<f64>, KinematicsError> {
        let (pose, _) = self.compute_frames(joint_positions)?;
        Ok(pose)
    }

    pub fn jacobian(&self, joint_positions: &[f64]) -> Result<Matrix6xX<f64>, KinematicsError> {
        let (end_pose, frames) = self.compute_frames(joint_positions)?;
        let end_position = end_pose.translation.vector;

        let mut jac = Matrix6xX::zeros(self.dof);
        let mut idx = 0;

        for frame in frames {
            match frame.kind {
                JointKind::Revolute => {
                    jac.fixed_view_mut::<3, 1>(0, idx)
                        .copy_from(&(frame.axis_world.cross(&(end_position - frame.position))));
                    jac.fixed_view_mut::<3, 1>(3, idx)
                        .copy_from(&frame.axis_world);
                    idx += 1;
                }
                JointKind::Prismatic => {
                    jac.fixed_view_mut::<3, 1>(0, idx)
                        .copy_from(&frame.axis_world);
                    jac.fixed_view_mut::<3, 1>(3, idx)
                        .copy_from(&Vector3::zeros());
                    idx += 1;
                }
                JointKind::Fixed => {}
            }
        }

        Ok(jac)
    }

    fn compute_frames(
        &self,
        joint_positions: &[f64],
    ) -> Result<(Isometry3<f64>, Vec<JointFrame>), KinematicsError> {
        if joint_positions.len() != self.dof {
            return Err(KinematicsError::StateLength {
                expected: self.dof,
                provided: joint_positions.len(),
            });
        }

        let mut q_iter = joint_positions.iter();
        let mut frames = Vec::with_capacity(self.joints.len());
        let mut current = Isometry3::identity();

        for joint in &self.joints {
            current = current * joint.origin;
            let axis_world = current.rotation * joint.axis;
            let position = current.translation.vector;

            match joint.kind {
                JointKind::Revolute => {
                    let angle = *q_iter.next().expect("joint count already validated");
                    let rotation = UnitQuaternion::from_axis_angle(&axis_unit(joint.axis), angle);
                    current = current * Isometry3::from_parts(Translation3::identity(), rotation);
                }
                JointKind::Prismatic => {
                    let displacement = *q_iter.next().expect("joint count already validated");
                    current = current
                        * Isometry3::from_parts(
                            Translation3::from(joint.axis * displacement),
                            UnitQuaternion::identity(),
                        );
                }
                JointKind::Fixed => {}
            }

            frames.push(JointFrame {
                position,
                axis_world,
                kind: joint.kind,
            });
        }

        Ok((current, frames))
    }
}

fn axis_unit(axis: Vector3<f64>) -> Unit<Vector3<f64>> {
    Unit::new_normalize(axis)
}

fn origin_to_isometry(origin: &urdf_rs::Pose) -> Isometry3<f64> {
    let translation = Translation3::new(origin.xyz[0], origin.xyz[1], origin.xyz[2]);
    let rotation = Rotation3::from_euler_angles(origin.rpy[0], origin.rpy[1], origin.rpy[2]);
    Isometry3::from_parts(translation, UnitQuaternion::from_rotation_matrix(&rotation))
}

struct JointFrame {
    position: Vector3<f64>,
    axis_world: Vector3<f64>,
    kind: JointKind,
}

impl From<KinematicsError> for PyErr {
    fn from(err: KinematicsError) -> Self {
        PyValueError::new_err(err.to_string())
    }
}

#[allow(unsafe_op_in_unsafe_fn)]
#[pyclass(name = "Robot")]
struct PyRobot {
    inner: KinematicChain,
}

#[pymethods]
impl PyRobot {
    #[staticmethod]
    fn from_urdf_file(path: &str, base_link: &str, end_link: &str) -> PyResult<Self> {
        let chain =
            KinematicChain::from_urdf_file(path, base_link.to_string(), end_link.to_string())?;
        Ok(Self { inner: chain })
    }

    #[staticmethod]
    fn from_urdf_str(urdf: &str, base_link: &str, end_link: &str) -> PyResult<Self> {
        let chain =
            KinematicChain::from_urdf_str(urdf, base_link.to_string(), end_link.to_string())?;
        Ok(Self { inner: chain })
    }

    #[getter]
    fn dof(&self) -> usize {
        self.inner.dof()
    }

    fn forward_kinematics<'py>(
        &self,
        py: Python<'py>,
        joints: Vec<f64>,
    ) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let pose = self.inner.forward_kinematics(&joints)?;
        let matrix = pose.to_homogeneous();
        Ok(matrix.to_pyarray_bound(py))
    }

    fn jacobian<'py>(&self, py: Python<'py>, joints: Vec<f64>) -> PyResult<Bound<'py, PyArray2<f64>>> {
        let jac = self.inner.jacobian(&joints)?;
        Ok(jac.to_pyarray_bound(py))
    }
}

#[pyfunction(name = "from_urdf_file")]
fn py_from_urdf_file(path: &str, base_link: &str, end_link: &str) -> PyResult<PyRobot> {
    PyRobot::from_urdf_file(path, base_link, end_link)
}

#[pyfunction(name = "from_urdf_str")]
fn py_from_urdf_str(urdf: &str, base_link: &str, end_link: &str) -> PyResult<PyRobot> {
    PyRobot::from_urdf_str(urdf, base_link, end_link)
}

#[pymodule]
fn literobo(py: Python, m: &Bound<'_, PyModule>) -> PyResult<()> {
    m.add_class::<PyRobot>()?;
    m.add("VERSION", env!("CARGO_PKG_VERSION"))?;
    m.add("BASE_LINK_KEY", "base_link")?;
    m.add("END_LINK_KEY", "end_link")?;
    m.add_function(wrap_pyfunction!(py_from_urdf_file, m)?)?;
    m.add_function(wrap_pyfunction!(py_from_urdf_str, m)?)?;

    let _ = py; // silence unused warning in non-extension builds
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::f64::consts::FRAC_PI_2;

    fn sample_urdf() -> String {
        r#"
<robot name="planar">
  <link name="base"/>
  <link name="link1"/>
  <link name="link2"/>
  <link name="tool"/>
  <joint name="joint1" type="revolute">
    <parent link="base"/>
    <child link="link1"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="joint2" type="revolute">
    <parent link="link1"/>
    <child link="link2"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
  </joint>
  <joint name="tip" type="fixed">
    <parent link="link2"/>
    <child link="tool"/>
    <origin xyz="1 0 0" rpy="0 0 0"/>
  </joint>
</robot>
"#
        .to_string()
    }

    #[test]
    fn forward_kinematics_is_correct() {
        let urdf = sample_urdf();
        let chain = KinematicChain::from_urdf_str(&urdf, "base", "tool").unwrap();
        let pose = chain.forward_kinematics(&[0.0, 0.0]).unwrap();
        let translation = pose.translation.vector;
        assert!((translation.x - 2.0).abs() < 1e-8);
        assert!((translation.y).abs() < 1e-8);
        assert!((translation.z).abs() < 1e-8);

        let pose = chain.forward_kinematics(&[FRAC_PI_2, 0.0]).unwrap();
        let translation = pose.translation.vector;
        assert!((translation.x).abs() < 1e-8);
        assert!((translation.y - 2.0).abs() < 1e-8);
    }

    #[test]
    fn jacobian_matches_planar_expectation() {
        let urdf = sample_urdf();
        let chain = KinematicChain::from_urdf_str(&urdf, "base", "tool").unwrap();
        let jac = chain.jacobian(&[0.0, 0.0]).unwrap();

        assert!((jac[(0, 0)] - 0.0).abs() < 1e-8);
        assert!((jac[(1, 0)] - 2.0).abs() < 1e-8);
        assert!((jac[(5, 0)] - 1.0).abs() < 1e-8);

        assert!((jac[(0, 1)] - 0.0).abs() < 1e-8);
        assert!((jac[(1, 1)] - 1.0).abs() < 1e-8);
        assert!((jac[(5, 1)] - 1.0).abs() < 1e-8);

        assert_eq!(jac.ncols(), 2);
    }
}
