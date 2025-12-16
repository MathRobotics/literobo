# literobo

Lightweight kinematics utilities for URDF robots, implemented in Rust with Python bindings.

## Features
- Parse URDF files and build a kinematic chain between any two links.
- Forward kinematics that returns a homogeneous transform.
- Geometric Jacobian for revolute and prismatic joints.
- Python bindings powered by [PyO3](https://pyo3.rs/) and packaged with [maturin](https://github.com/PyO3/maturin).

## Rust usage
```rust
use literobo::KinematicChain;

let chain = KinematicChain::from_urdf_file("robot.urdf", "base_link", "tool_link")?;
let pose = chain.forward_kinematics(&[0.0, 0.5, -1.0])?;
let jacobian = chain.jacobian(&[0.0, 0.5, -1.0])?;
println!("Pose:\n{}", pose.to_homogeneous());
println!("Jacobian:\n{}", jacobian);
```

## Python usage

The project is configured for `uv` so you can manage dependencies and builds without a virtualenv toolchain mismatch.

```bash
# Create an isolated environment
uv venv
source .venv/bin/activate

# Install build dependency and compile the wheel
uv pip install maturin
uv pip install .
```

```python
import numpy as np
import literobo

robot = literobo.from_urdf_file("robot.urdf", "base_link", "tool_link")
q = np.array([0.0, 0.5, -1.0])
pose = robot.forward_kinematics(q)
jacobian = robot.jacobian(q)

print("Pose:\n", pose)
print("Jacobian:\n", jacobian)
```

To publish, run `uv build` (which delegates to `maturin` under the hood) and upload the wheel to PyPI with `uv publish`.

## Quick sample run (Python)

Below is a minimal end-to-end example you can run locally:

```bash
# 1. Prepare environment (inside repo root)
uv venv
source .venv/bin/activate
uv pip install maturin  # build backend

# 2. Build & install the wheel from the checked-out source
uv pip install .

# 3. Create a tiny planar robot URDF
cat > planar.urdf <<'URDF'
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
URDF

# 4. Run a short script
python - <<'PY'
import numpy as np
import literobo

robot = literobo.from_urdf_file("planar.urdf", "base", "tool")
q = np.array([0.0, 0.5])

pose = robot.forward_kinematics(q)
jac = robot.jacobian(q)

print("Pose:\n", pose)
print("Jacobian:\n", jac)
PY
```
