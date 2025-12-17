"""Minimal Python example using the bundled planar URDF."""

from pathlib import Path

import numpy as np

import literobo


def main() -> None:
    urdf_path = Path(__file__).parent / "planar.urdf"
    robot = literobo.from_urdf_file(str(urdf_path), "base", "tool")

    q = np.array([0.0, 0.5])
    pose = robot.forward_kinematics(q)
    jac = robot.jacobian(q)

    np.set_printoptions(precision=4, suppress=True)
    print(f"Loaded URDF: {urdf_path}")
    print("Pose:\n", pose)
    print("Jacobian:\n", jac)


if __name__ == "__main__":
    main()
