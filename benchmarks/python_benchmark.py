"""Benchmark Python bindings for kinematics computations."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import time
from typing import Iterable

import numpy as np

import literobo


_LINE_RE = re.compile(r"^(?P<label>\\w+): total=(?P<total>[0-9.]+)s avg=(?P<avg>[0-9.]+)us$")


def _bench(label: str, iterations: int, warmup: int, func) -> str:
    for _ in range(warmup):
        func()

    start = time.perf_counter()
    for _ in range(iterations):
        func()
    elapsed = time.perf_counter() - start
    per_call_us = (elapsed / iterations) * 1e6
    line = f"{label}: total={elapsed:.6f}s avg={per_call_us:.2f}us"
    print(line)
    return line


def _write_output(path: Path, lines: Iterable[str]) -> None:
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def _parse_output(path: Path) -> dict[str, float]:
    results: dict[str, float] = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        match = _LINE_RE.match(raw_line.strip())
        if match:
            results[match.group("label")] = float(match.group("avg"))
    return results


def _print_comparison(current: dict[str, float], baseline: dict[str, float]) -> None:
    print("\nComparison vs baseline:")
    for label, current_avg in current.items():
        base_avg = baseline.get(label)
        if base_avg is None:
            print(f"{label}: baseline not found")
            continue
        delta = current_avg - base_avg
        ratio = current_avg / base_avg if base_avg else float("inf")
        print(f"{label}: {current_avg:.2f}us (Δ {delta:+.2f}us, {ratio:.2f}x)")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--urdf",
        type=Path,
        default=Path(__file__).resolve().parent.parent / "examples" / "planar.urdf",
        help="Path to the URDF file.",
    )
    parser.add_argument("--base", default="base", help="Base link name.")
    parser.add_argument("--end", default="tool", help="End link name.")
    parser.add_argument("--iterations", type=int, default=100_000)
    parser.add_argument("--warmup", type=int, default=5_000)
    parser.add_argument(
        "--list-input",
        action="store_true",
        help="Benchmark with Python list input instead of numpy array.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        help="Optional path to save benchmark output (e.g. benchmarks/python_latest.txt).",
    )
    parser.add_argument(
        "--compare",
        type=Path,
        help="Optional baseline output to compare against (e.g. benchmarks/python_baseline.txt).",
    )
    args = parser.parse_args()

    robot = literobo.from_urdf_file(str(args.urdf), args.base, args.end)
    q_array = np.array([0.0, 0.5])
    q_list = [0.0, 0.5]
    joints = q_list if args.list_input else q_array
    pose_out = np.empty((4, 4), dtype=np.float64)
    jac_out = np.empty((6, robot.dof), dtype=np.float64)

    print(f"Robot DOF: {robot.dof} (joints input: {'list' if args.list_input else 'numpy'})")
    lines: list[str] = []
    lines.append(
        f"Robot DOF: {robot.dof} (joints input: {'list' if args.list_input else 'numpy'})"
    )
    lines.append(
        _bench(
            "forward_kinematics",
            args.iterations,
            args.warmup,
            lambda: robot.forward_kinematics(joints),
        )
    )
    lines.append(
        _bench(
            "jacobian",
            args.iterations,
            args.warmup,
            lambda: robot.jacobian(joints),
        )
    )
    lines.append(
        _bench(
            "forward_kinematics_into",
            args.iterations,
            args.warmup,
            lambda: robot.forward_kinematics_into(joints, pose_out),
        )
    )
    lines.append(
        _bench(
            "jacobian_into",
            args.iterations,
            args.warmup,
            lambda: robot.jacobian_into(joints, jac_out),
        )
    )

    if args.output:
        _write_output(args.output, lines)

    if args.compare:
        baseline = _parse_output(args.compare)
        current_results: dict[str, float] = {}
        for line in lines:
            match = _LINE_RE.match(line.strip())
            if match:
                current_results[match.group("label")] = float(match.group("avg"))
        _print_comparison(current_results, baseline)


if __name__ == "__main__":
    main()
