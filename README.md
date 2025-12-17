# literobo

Lightweight kinematics utilities for URDF robots, implemented in Rust with Python bindings.

## Features
- Parse URDF files and build a kinematic chain between any two links.
- Forward kinematics that returns a homogeneous transform.
- Geometric Jacobian for revolute and prismatic joints.
- Python bindings powered by [PyO3](https://pyo3.rs/) and packaged with [maturin](https://github.com/PyO3/maturin).

## Layout

- `src/chain.rs` – core kinematics implementation
- `src/error.rs` – error types shared between Rust and Python
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
uv pip install maturin  # build backend
uv pip install .        # build + install the wheel
uv run python examples/quickstart.py
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

# 3. Run the bundled sample
uv run python examples/quickstart.py
```
