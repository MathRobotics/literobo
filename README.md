# literobo

Lightweight kinematics utilities for URDF robots, implemented in Rust with Python bindings.

## Features
- Parse URDF files and build a kinematic chain between any two links.
- Forward kinematics that returns a homogeneous transform.
- Geometric Jacobian for revolute and prismatic joints.
- Python bindings powered by [PyO3](https://pyo3.rs/) and packaged with [maturin](https://github.com/PyO3/maturin).

## Layout

- `src/chain.rs` – core kinematics implementation
- `src/python.rs` – PyO3 bindings exposed to Python
- `examples/` – ready-to-run assets (`planar.urdf`, `quickstart.py`)

## Rust usage
```rust
use literobo::KinematicChain;

let chain = KinematicChain::from_urdf_file("robot.urdf", "base_link", "tool_link")?;
let pose = chain.forward_kinematics(&[0.0, 0.5, -1.0])?;
let jacobian = chain.jacobian(&[0.0, 0.5, -1.0])?;
println!("Pose:\n{}", pose.to_homogeneous());
println!("Jacobian:\n{}", jacobian);
```

### Why provide `base_link` / `end_link`

- A URDF describes a tree (or forest), so the loader must know which branch to treat as the active sub-chain.
- Explicit `base_link` and `end_link` let the library search only between those two links and ignore unrelated branches when extracting the shortest path.
- This also clarifies behavior for floating bases or multi–end-effector robots: callers can choose different start/end pairs from one URDF to construct multiple chains as needed.

## Development commands

Rust:

```bash
cargo fmt
cargo test
```

Python (using [uv](https://github.com/astral-sh/uv)):

```bash
uv venv
source .venv/bin/activate
uv run pip install maturin  # build backend
uv run pip install .        # build + install the wheel
uv run python examples/quickstart.py
```

## Python usage
The project is configured for `uv` so you can manage dependencies and builds without a virtualenv toolchain mismatch.

```bash
# Create an isolated environment
uv venv
source .venv/bin/activate

# Install build dependency and compile the wheel
uv run pip install maturin
uv run pip install .
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

# Re-select a subchain without reparsing the URDF
sub = robot.with_links("base_link", "link1")
pose_sub = sub.forward_kinematics(np.array([0.25]))
print("Subchain pose:\n", pose_sub)
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

# 3. Run the bundled sample
uv run python examples/quickstart.py
```
