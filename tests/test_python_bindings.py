from pathlib import Path

import numpy as np

import literobo


PLANAR_URDF = Path(__file__).resolve().parent.parent / "examples" / "planar.urdf"


def test_forward_kinematics_and_jacobian_shapes():
    robot = literobo.from_urdf_file(str(PLANAR_URDF), "base", "tool")

    assert robot.dof == 2

    joints = [0.0, 0.0]
    pose = robot.forward_kinematics(joints)
    jac = robot.jacobian(joints)

    np.testing.assert_allclose(pose.shape, (4, 4))
    np.testing.assert_allclose(jac.shape, (6, robot.dof))


def test_forward_kinematics_planar_translation():
    robot = literobo.from_urdf_file(str(PLANAR_URDF), "base", "tool")

    pose = robot.forward_kinematics([0.0, 0.0])

    np.testing.assert_allclose(pose[:3, 3], [2.0, 0.0, 0.0], rtol=1e-7, atol=1e-9)
    np.testing.assert_allclose(pose[3], [0.0, 0.0, 0.0, 1.0])
