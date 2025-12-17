use crate::KinematicsError;
use nalgebra::{Isometry3, Rotation3, Translation3, Unit, UnitQuaternion, Vector3};
use urdf_rs::{Joint, JointType, Pose};

#[derive(Clone, Copy, Debug)]
pub enum JointKind {
    Revolute,
    Prismatic,
    Fixed,
}

#[derive(Clone, Debug)]
pub struct ChainJoint {
    #[allow(dead_code)]
    pub name: String,
    #[allow(dead_code)]
    pub parent: String,
    #[allow(dead_code)]
    pub child: String,
    pub origin: Isometry3<f64>,
    pub axis: Vector3<f64>,
    pub kind: JointKind,
}

pub fn chain_joint_from_urdf(joint: &Joint) -> Result<ChainJoint, KinematicsError> {
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

pub fn axis_unit(axis: Vector3<f64>) -> Unit<Vector3<f64>> {
    Unit::new_normalize(axis)
}

pub fn origin_to_isometry(origin: &Pose) -> Isometry3<f64> {
    let translation = Translation3::new(origin.xyz[0], origin.xyz[1], origin.xyz[2]);
    let rotation = Rotation3::from_euler_angles(origin.rpy[0], origin.rpy[1], origin.rpy[2]);
    Isometry3::from_parts(translation, UnitQuaternion::from_rotation_matrix(&rotation))
}
