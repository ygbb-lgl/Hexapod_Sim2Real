"""Generate deterministic 50 Hz CSV fixtures for the tension predictor.

The generated trajectories exercise the complete data and training pipeline, but
they are synthetic and must not be mixed with real robot data when evaluating
deployment quality.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
from pathlib import Path
from typing import Dict, List, Sequence, Tuple


DT_S = 0.02
DEFAULT_ROWS = 600
VECTOR_DIMS: Sequence[Tuple[str, int]] = (
    ("velocity_command", 3),
    ("policy_action", 18),
    ("joint_position", 18),
    ("joint_velocity", 18),
    ("body_gravity_vector", 3),
    ("body_angular_velocity", 3),
    ("body_linear_acceleration", 3),
)
SCALAR_FIELDS = [
    "timestamp_ns",
    "trajectory_id",
    "sample_index",
    "control_dt_s",
    "force_raw_n",
    "torque_actual_nm",
    "torque_command_prev_nm",
    "torque_command_issued_nm",
    "motor_position_rad",
    "force_reference_n",
    "imu_valid",
    "tension_sensor_valid",
    "saturation_flag",
    "emergency_flag",
    "data_valid_flag",
]
FIELDNAMES = SCALAR_FIELDS + [
    f"{name}_{index}"
    for name, size in VECTOR_DIMS
    for index in range(size)
]


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(value, high))


def smooth_square(t_s: float, period_s: float, sharpness: float = 2.5) -> float:
    return math.tanh(sharpness * math.sin(2.0 * math.pi * t_s / period_s))


def command_for(scenario: int, t_s: float) -> Tuple[float, float, float]:
    ramp = 1.0 - math.exp(-t_s / 0.8)
    if scenario == 0:
        return (0.65 * ramp + 0.12 * math.sin(0.55 * t_s), 0.0, 0.08 * math.sin(0.8 * t_s))
    if scenario == 1:
        return (0.35 + 0.18 * math.sin(0.7 * t_s), 0.28 * math.sin(0.45 * t_s), 0.35 * math.sin(0.32 * t_s))
    if scenario == 2:
        return (0.62 * (0.5 + 0.5 * smooth_square(t_s, 4.0)), 0.0, 0.0)
    if scenario == 3:
        return (-0.42 * ramp + 0.08 * math.sin(0.8 * t_s), 0.08 * math.sin(0.4 * t_s), -0.12 * math.sin(0.55 * t_s))
    if scenario == 4:
        return (0.48 + 0.12 * math.sin(0.35 * t_s), 0.22 * math.sin(1.05 * t_s), 0.42 * math.sin(0.52 * t_s))
    return (0.25 + 0.32 * math.sin(0.24 * t_s), 0.12 * math.sin(0.67 * t_s), 0.2 * smooth_square(t_s, 5.0))


def force_reference_for(scenario: int, t_s: float, command: Sequence[float]) -> float:
    base = 105.0 + 18.0 * abs(command[0]) + 8.0 * abs(command[1])
    if scenario == 2:
        base += 18.0 * smooth_square(t_s + 0.7, 3.5)
    elif scenario == 5:
        base += 25.0 * math.sin(0.38 * t_s) + 8.0 * math.sin(1.15 * t_s)
    else:
        base += 10.0 * math.sin(0.22 * t_s + 0.4 * scenario)
    return clamp(base, 65.0, 165.0)


def put_vector(row: Dict[str, object], name: str, values: Sequence[float]) -> None:
    expected = dict(VECTOR_DIMS)[name]
    if len(values) != expected:
        raise ValueError(f"{name} has {len(values)} values; expected {expected}")
    for index, value in enumerate(values):
        row[f"{name}_{index}"] = f"{float(value):.8f}"


def generate_trajectory(path: Path, scenario: int, rows: int, seed: int) -> None:
    rng = random.Random(seed)
    trajectory_id = f"synthetic_50hz_{scenario + 1:02d}"
    timestamp_origin_ns = 10_000_000_000_000 + scenario * 100_000_000_000

    force_raw = 82.0 + 2.0 * scenario
    torque_actual = 8.0
    previous_command = 8.0
    spool_position = 0.08 * scenario
    spool_velocity = 0.0
    previous_velocity_command = [0.0, 0.0, 0.0]

    with path.open("w", newline="") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=FIELDNAMES)
        writer.writeheader()

        for sample_index in range(rows):
            t_s = sample_index * DT_S
            velocity_command = list(command_for(scenario, t_s))
            acceleration = [
                (velocity_command[index] - previous_velocity_command[index]) / DT_S
                for index in range(3)
            ]
            previous_velocity_command = velocity_command

            speed = math.hypot(velocity_command[0], velocity_command[1])
            gait_hz = 0.65 + 0.75 * min(speed, 1.0)
            gait_phase = 2.0 * math.pi * gait_hz * t_s
            policy_action: List[float] = []
            joint_position: List[float] = []
            joint_velocity: List[float] = []
            leg_activity = 0.0
            for joint_index in range(18):
                leg_index, joint_in_leg = divmod(joint_index, 3)
                side_phase = math.pi if leg_index in (1, 3, 5) else 0.0
                phase = gait_phase + side_phase + 0.17 * leg_index
                amplitude = (0.055, 0.13, 0.17)[joint_in_leg] * (0.45 + speed)
                offset = (0.0, 0.12, -0.24)[joint_in_leg]
                action = amplitude * math.sin(phase + 0.55 * joint_in_leg)
                position = offset + action + 0.008 * math.sin(0.31 * t_s + joint_index)
                velocity = (
                    amplitude * 2.0 * math.pi * gait_hz
                    * math.cos(phase + 0.55 * joint_in_leg)
                )
                policy_action.append(action)
                joint_position.append(position)
                joint_velocity.append(velocity)
                leg_activity += abs(velocity)
            leg_activity /= 18.0

            roll = 0.055 * velocity_command[1] + 0.025 * math.sin(0.9 * t_s)
            pitch = -0.07 * velocity_command[0] + 0.02 * math.sin(0.7 * t_s + 0.3)
            gravity = [math.sin(pitch), -math.sin(roll), -math.cos(roll) * math.cos(pitch)]
            angular_velocity = [
                0.0225 * math.cos(0.9 * t_s),
                -0.035 * math.cos(0.7 * t_s + 0.3),
                velocity_command[2],
            ]

            # At t_k the plant has already responded to u_{k-1}.  The new u_k is
            # computed only after F_k below, matching the real logger's ordering.
            torque_actual += 0.42 * (previous_command - torque_actual)
            spool_velocity += 0.18 * (0.035 * torque_actual - spool_velocity)
            spool_position += spool_velocity * DT_S
            force_equilibrium = (
                48.0
                + 2.45 * max(torque_actual, -5.0)
                + 15.0 * speed
                + 2.8 * leg_activity
                + 3.5 * abs(acceleration[0])
            )
            force_raw += 0.16 * (force_equilibrium - force_raw)
            force_raw += 0.35 * math.sin(1.7 * t_s + 0.6 * scenario)
            force_raw += rng.gauss(0.0, 0.22)
            force_raw = max(force_raw, 2.0)

            force_reference = force_reference_for(scenario, t_s, velocity_command)
            excitation = (
                3.2 * math.sin(0.83 * t_s + scenario)
                + 2.1 * math.sin(1.91 * t_s + 0.3 * scenario)
                + 1.2 * smooth_square(t_s + 0.4 * scenario, 2.6)
            )
            torque_command = (
                7.5
                + 0.19 * (force_reference - force_raw)
                + 4.0 * abs(velocity_command[0])
                + excitation
            )
            torque_command = clamp(torque_command, -12.0, 42.0)

            row: Dict[str, object] = {
                "timestamp_ns": timestamp_origin_ns + int(round(sample_index * DT_S * 1e9)),
                "trajectory_id": trajectory_id,
                "sample_index": sample_index,
                "control_dt_s": DT_S,
                "force_raw_n": f"{force_raw:.8f}",
                "torque_actual_nm": f"{torque_actual:.8f}",
                "torque_command_prev_nm": f"{previous_command:.8f}",
                "torque_command_issued_nm": f"{torque_command:.8f}",
                "motor_position_rad": f"{spool_position:.8f}",
                "force_reference_n": f"{force_reference:.8f}",
                "imu_valid": 1,
                "tension_sensor_valid": 1,
                "saturation_flag": 0,
                "emergency_flag": 0,
                "data_valid_flag": 1,
            }
            put_vector(row, "velocity_command", velocity_command)
            put_vector(row, "policy_action", policy_action)
            put_vector(row, "joint_position", joint_position)
            put_vector(row, "joint_velocity", joint_velocity)
            put_vector(row, "body_gravity_vector", gravity)
            put_vector(row, "body_angular_velocity", angular_velocity)
            put_vector(row, "body_linear_acceleration", acceleration)
            writer.writerow(row)
            previous_command = torque_command


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=Path(__file__).resolve().parent / "pre_data",
    )
    parser.add_argument("--trajectories", type=int, default=6)
    parser.add_argument("--rows", type=int, default=DEFAULT_ROWS)
    parser.add_argument("--seed", type=int, default=20260713)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.trajectories < 3:
        raise ValueError("at least 3 trajectories are required by the train/val/test split")
    if args.trajectories > 6:
        raise ValueError("this fixture generator defines 6 motion scenarios")
    if args.rows < 80:
        raise ValueError("rows must be at least 80 for the default history and horizon")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    for scenario in range(args.trajectories):
        path = args.output_dir / f"synthetic_traj_{scenario + 1:02d}_50hz.csv"
        generate_trajectory(path, scenario, args.rows, args.seed + scenario)
        print(f"generated {args.rows} rows -> {path}")


if __name__ == "__main__":
    main()
