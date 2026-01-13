use crate::KinematicsError;
use crate::joint::{ChainJoint, JointKind, chain_joint_from_urdf};
use nalgebra::{Isometry3, Matrix6xX, UnitQuaternion, Vector3};
use urdf_rs::Robot;

#[derive(Clone, Debug)]
pub struct KinematicChain {
    joints: Vec<ChainJoint>,
    base: String,
    end: String,
    dof: usize,
}

struct JointFrame {
    position: Vector3<f64>,
    axis_world: Vector3<f64>,
    kind: JointKind,
}

impl KinematicChain {
    /// Build a kinematic chain from a URDF file.
    ///
    /// `base_link` と `end_link` を明示することで、ツリー構造の URDF から特定の
    /// サブチェーンを切り出し、ルートと終端を決め打ちで探索します。URDF に複数
    /// の分岐やフローティングベースがある場合でも、どのリンク間を結ぶかを
    /// 呼び出し側で制御でき、想定外の枝を拾わないようにしています。
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
            chain.push(chain_joint_from_urdf(joint)?);
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
        self.compute_end_pose(joint_positions)
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
                        .copy_from(&frame.axis_world);
                    jac.fixed_view_mut::<3, 1>(3, idx)
                        .copy_from(&(frame.axis_world.cross(&(end_position - frame.position))));
                    idx += 1;
                }
                JointKind::Prismatic => {
                    jac.fixed_view_mut::<3, 1>(3, idx)
                        .copy_from(&frame.axis_world);
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
        let mut frames = Vec::with_capacity(self.dof);
        let mut current = Isometry3::identity();

        for joint in &self.joints {
            current.translation.vector += current.rotation * joint.origin.translation.vector;
            current.rotation = current.rotation * joint.origin.rotation;

            match joint.kind {
                JointKind::Revolute => {
                    let angle = *q_iter.next().expect("joint count already validated");
                    let axis_world = current.rotation * joint.axis;
                    let position = current.translation.vector;
                    let rotation = UnitQuaternion::from_axis_angle(
                        joint
                            .axis_unit
                            .as_ref()
                            .expect("axis_unit present for revolute joint"),
                        angle,
                    );
                    current.rotation = current.rotation * rotation;
                    frames.push(JointFrame {
                        position,
                        axis_world,
                        kind: joint.kind,
                    });
                }
                JointKind::Prismatic => {
                    let displacement = *q_iter.next().expect("joint count already validated");
                    let axis_world = current.rotation * joint.axis;
                    let position = current.translation.vector;
                    current.translation.vector += current.rotation * (joint.axis * displacement);
                    frames.push(JointFrame {
                        position,
                        axis_world,
                        kind: joint.kind,
                    });
                }
                JointKind::Fixed => {}
            }
        }

        Ok((current, frames))
    }

    fn compute_end_pose(&self, joint_positions: &[f64]) -> Result<Isometry3<f64>, KinematicsError> {
        if joint_positions.len() != self.dof {
            return Err(KinematicsError::StateLength {
                expected: self.dof,
                provided: joint_positions.len(),
            });
        }

        let mut q_iter = joint_positions.iter();
        let mut current = Isometry3::identity();

        for joint in &self.joints {
            current.translation.vector += current.rotation * joint.origin.translation.vector;
            current.rotation = current.rotation * joint.origin.rotation;
            match joint.kind {
                JointKind::Revolute => {
                    let angle = *q_iter.next().expect("joint count already validated");
                    let rotation = UnitQuaternion::from_axis_angle(
                        joint
                            .axis_unit
                            .as_ref()
                            .expect("axis_unit present for revolute joint"),
                        angle,
                    );
                    current.rotation = current.rotation * rotation;
                }
                JointKind::Prismatic => {
                    let displacement = *q_iter.next().expect("joint count already validated");
                    current.translation.vector += current.rotation * (joint.axis * displacement);
                }
                JointKind::Fixed => {}
            }
        }

        Ok(current)
    }
}
