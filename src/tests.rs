use crate::KinematicChain;
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

    assert!((jac[(2, 0)] - 1.0).abs() < 1e-8);
    assert!((jac[(3, 0)] - 0.0).abs() < 1e-8);
    assert!((jac[(4, 0)] - 2.0).abs() < 1e-8);

    assert!((jac[(2, 1)] - 1.0).abs() < 1e-8);
    assert!((jac[(3, 1)] - 0.0).abs() < 1e-8);
    assert!((jac[(4, 1)] - 1.0).abs() < 1e-8);

    assert_eq!(jac.ncols(), 2);
}
