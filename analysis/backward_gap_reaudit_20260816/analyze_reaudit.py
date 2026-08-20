#!/usr/bin/env python3
"""Reproduce the 2026-08-16 audit of the two 2026-07-29 motor logs.

This analysis deliberately does not load a policy checkpoint.  The user
confirmed that the checkpoint currently present in the repository is not the
one deployed on the robot.  The logged joint targets are sufficient for the
range audit and for the near-static MuJoCo actuator replay.

Run with the MuJoCo training environment:

  conda run --no-capture-output -n mujoco_rl \
    python analysis/backward_gap_reaudit_20260816/analyze_reaudit.py

The script writes ``derived_metrics.json`` next to itself.
"""

from __future__ import annotations

import csv
import hashlib
import json
from pathlib import Path

import numpy as np


SIM2REAL_ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
MUJOCO_ROOT = Path("/home/lgl/mujoco_hexapod")
ISAAC_ROOT = Path("/home/lgl/extended_legged_gym-feat-el_4090")
LOG_ROOT = SIM2REAL_ROOT / "deploy/deploy_real/0729-data"
IMU_LOG_ROOT = SIM2REAL_ROOT / "deploy/deploy_real/pre_data"
MUJOCO_SCENE = MUJOCO_ROOT / (
    "mujoco_playground/_src/locomotion/hexapod_tethered_04/xmls/"
    "scene_mjx_fullcollisions_flat_terrain.xml"
)

RUNS = ("motor_logs_20260729_105057", "motor_logs_20260729_105910")
IMU_SIDE_EVIDENCE_GROUPS = {
    "20260729_1203_1210": (
        "hexapod_20260729_120321.csv",
        "hexapod_20260729_120553.csv",
        "hexapod_20260729_120855.csv",
    ),
    "20260729_1219_1235": (
        "hexapod_20260729_121925.csv",
        "hexapod_20260729_122202.csv",
        "hexapod_20260729_122829.csv",
        "hexapod_20260729_123045.csv",
        "hexapod_20260729_123312.csv",
    ),
}
JOINT_LABELS = (
    "RF_HAA", "RF_HFE", "RF_KFE",
    "RM_HAA", "RM_HFE", "RM_KFE",
    "RB_HAA", "RB_HFE", "RB_KFE",
    "LF_HAA", "LF_HFE", "LF_KFE",
    "LM_HAA", "LM_HFE", "LM_KFE",
    "LB_HAA", "LB_HFE", "LB_KFE",
)
SIM_LIMITS = {"HAA": 0.8, "HFE": 0.6, "KFE": 0.6}


def as_builtin(value):
    """Convert NumPy scalars/arrays recursively for JSON serialization."""
    if isinstance(value, np.ndarray):
        return [as_builtin(item) for item in value.tolist()]
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): as_builtin(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [as_builtin(item) for item in value]
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_run(run_name: str) -> dict[int, dict[str, np.ndarray]]:
    run_dir = LOG_ROOT / run_name
    joints: dict[int, dict[str, np.ndarray]] = {}
    for path in sorted(run_dir.glob("motor_*_policy_idx_*.csv")):
        joint = int(path.stem.rsplit("_", 1)[-1])
        columns: dict[str, list[float]] = {
            "time_step": [], "counter": [], "target_pos": [],
            "actual_pos": [], "error": [], "action": [],
        }
        with path.open("r", encoding="utf-8", newline="") as handle:
            reader = csv.DictReader(handle)
            for row in reader:
                if any(row.get(key) in (None, "") for key in columns):
                    continue
                for key in columns:
                    columns[key].append(float(row[key]))
        joints[joint] = {
            key: np.asarray(values, dtype=np.float64)
            for key, values in columns.items()
        }
        joints[joint]["path"] = path  # type: ignore[assignment]
    if set(joints) != set(range(18)):
        raise RuntimeError(f"{run_name}: expected policy joints 0..17, got {sorted(joints)}")
    return joints


def best_target_to_actual_lag(target: np.ndarray, actual: np.ndarray) -> dict[str, float]:
    best = {"steps": 0, "correlation": float(np.corrcoef(target, actual)[0, 1])}
    for lag in range(1, 9):
        corr = float(np.corrcoef(target[:-lag], actual[lag:])[0, 1])
        if corr > best["correlation"]:
            best = {"steps": lag, "correlation": corr}
    return best


def audit_run(run_name: str, joints: dict[int, dict[str, np.ndarray]]) -> dict:
    lengths = [len(joints[index]["counter"]) for index in range(18)]
    all_target = np.concatenate([joints[index]["target_pos"] for index in range(18)])
    all_action = np.concatenate([joints[index]["action"] for index in range(18)])

    range_rows = []
    tracking_rows = []
    for joint_type in ("HAA", "HFE", "KFE"):
        indices = [index for index, label in enumerate(JOINT_LABELS) if label.endswith(joint_type)]
        target = np.concatenate([joints[index]["target_pos"] for index in indices])
        actual = np.concatenate([joints[index]["actual_pos"] for index in indices])
        residual = actual - target
        limit = SIM_LIMITS[joint_type]
        range_rows.append({
            "run": run_name.removeprefix("motor_logs_"),
            "joint_type": joint_type,
            "samples": len(target),
            "target_min_rad": float(target.min()),
            "target_max_rad": float(target.max()),
            "target_max_abs_rad": float(np.max(np.abs(target))),
            "sim_ctrl_limit_abs_rad": limit,
            "over_sim_limit_count": int(np.sum(np.abs(target) > limit)),
            "over_sim_limit_share": float(np.mean(np.abs(target) > limit)),
        })
        tracking_rows.append({
            "run": run_name.removeprefix("motor_logs_"),
            "joint_type": joint_type,
            "rowwise_bias_rad": float(np.mean(residual)),
            "rowwise_mae_rad": float(np.mean(np.abs(residual))),
            "rowwise_rmse_rad": float(np.sqrt(np.mean(residual ** 2))),
        })

    joint_rows = []
    for index in range(18):
        target = joints[index]["target_pos"]
        actual = joints[index]["actual_pos"]
        lag = best_target_to_actual_lag(target, actual)
        joint_rows.append({
            "run": run_name.removeprefix("motor_logs_"),
            "joint": index,
            "joint_label": JOINT_LABELS[index],
            "rows": len(target),
            "target_min_rad": float(target.min()),
            "target_max_rad": float(target.max()),
            "target_max_abs_rad": float(np.max(np.abs(target))),
            "target_over_0_6_count": int(np.sum(np.abs(target) > 0.6)),
            "target_over_0_8_count": int(np.sum(np.abs(target) > 0.8)),
            "best_lag_policy_steps": int(lag["steps"]),
            "best_lag_correlation": float(lag["correlation"]),
        })

    counter_checks = []
    for index in range(18):
        counter = joints[index]["counter"]
        counter_checks.append(bool(np.array_equal(counter, np.arange(1, len(counter) + 1))))

    return {
        "run": run_name.removeprefix("motor_logs_"),
        "csv_count": len(joints),
        "total_rows": int(sum(lengths)),
        "min_rows_per_joint": int(min(lengths)),
        "max_rows_per_joint": int(max(lengths)),
        "common_synchronized_rows": int(min(lengths)),
        "nominal_common_duration_s": float(min(lengths) * 0.02),
        "all_counters_contiguous_from_one": bool(all(counter_checks)),
        "target_equals_action_times_0_25_max_abs_error_rad": float(
            np.max(np.abs(all_target - 0.25 * all_action))
        ),
        "logged_error_identity_max_abs_error_rad": float(max(
            np.max(np.abs(
                joints[index]["error"]
                - (joints[index]["actual_pos"] - joints[index]["target_pos"])
            ))
            for index in range(18)
        )),
        "target_abs_quantiles_rad": {
            str(percentile): float(np.percentile(np.abs(all_target), percentile))
            for percentile in (50, 90, 95, 99, 99.5, 99.9, 100)
        },
        "target_min_rad": float(all_target.min()),
        "target_max_rad": float(all_target.max()),
        "action_min": float(all_action.min()),
        "action_max": float(all_action.max()),
        "range_by_joint_type": range_rows,
        "tracking_by_joint_type": tracking_rows,
        "joint_details": joint_rows,
    }


def infer_period_from_flush_mtimes(joints: dict[int, dict[str, np.ndarray]]) -> dict:
    """Infer the 105057 period from independent file-close endpoints.

    Each CSV has the same start but a different final counter.  Regressing the
    close/flush mtime against that final counter cancels a common startup time.
    This is strong historical evidence, but not a replacement for a monotonic
    timestamp logged on every control iteration.
    """
    counters = np.asarray([
        joints[index]["counter"][-1] for index in range(18)
    ], dtype=np.float64)
    mtimes = np.asarray([
        joints[index]["path"].stat().st_mtime_ns * 1e-9  # type: ignore[union-attr]
        for index in range(18)
    ], dtype=np.float64)
    x = counters - counters.mean()
    y = mtimes - mtimes.mean()
    slope_s = float(np.dot(x, y) / np.dot(x, x))
    prediction = slope_s * x
    r_squared = 1.0 - float(np.sum((y - prediction) ** 2) / np.sum(y ** 2))
    return {
        "method": "linear regression of per-CSV close mtime versus final counter",
        "files": 18,
        "period_ms": slope_s * 1000.0,
        "frequency_hz": 1.0 / slope_s,
        "r_squared": r_squared,
        "limitation": "inferred from file metadata; exact per-step monotonic time was not logged",
    }


def synchronized_targets(joints: dict[int, dict[str, np.ndarray]], frames: int) -> np.ndarray:
    return np.column_stack([
        joints[index]["target_pos"][:frames] for index in range(18)
    ])


def synchronized_actual(joints: dict[int, dict[str, np.ndarray]], frames: int) -> np.ndarray:
    return np.column_stack([
        joints[index]["actual_pos"][:frames] for index in range(18)
    ])


def replay_static_targets(joints: dict[int, dict[str, np.ndarray]], frames: int = 1000) -> dict:
    """Replay the near-static 105057 targets in the registered MuJoCo task."""
    import mujoco  # pylint: disable=import-outside-toplevel

    # This is the scene selected by
    # HexapodTethered04JoystickFullCollisionsFlatTerrain.  Loading it directly
    # avoids importing JAX and produces the same nominal MuJoCo model.  The
    # environment base class sets this CCD iteration count after loading.
    model = mujoco.MjModel.from_xml_path(str(MUJOCO_SCENE))
    model.opt.ccd_iterations = 20
    data = mujoco.MjData(model)
    data.qpos[:] = model.key("home").qpos
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)

    targets = synchronized_targets(joints, frames)
    real_actual = synchronized_actual(joints, frames)
    data.ctrl[:18] = targets[0]
    data.ctrl[18] = 0.0
    for _ in range(round(5.0 / model.opt.timestep)):
        mujoco.mj_step(model, data)

    qpos_addresses = []
    for actuator in range(18):
        joint_id = int(model.actuator_trnid[actuator, 0])
        qpos_addresses.append(int(model.jnt_qposadr[joint_id]))

    sim_actual = np.zeros_like(targets)
    base_z = np.zeros(frames, dtype=np.float64)
    substeps = round(0.02 / model.opt.timestep)
    for frame, target in enumerate(targets):
        data.ctrl[:18] = target
        data.ctrl[18] = 0.0
        for _ in range(substeps):
            mujoco.mj_step(model, data)
        sim_actual[frame] = data.qpos[qpos_addresses]
        base_z[frame] = data.qpos[2]

    static_hfe_rows = []
    for index in (1, 4, 7, 10, 13, 16):
        target = targets[:, index]
        real_residual = real_actual[:, index] - target
        sim_residual = sim_actual[:, index] - target
        static_hfe_rows.append({
            "joint": JOINT_LABELS[index],
            "target_std_rad": float(np.std(target)),
            "real_actual_minus_target_rad": float(np.mean(real_residual)),
            "sim_actual_minus_target_rad": float(np.mean(sim_residual)),
        })

    type_metrics = []
    for joint_type in ("HAA", "HFE", "KFE"):
        indices = [index for index, label in enumerate(JOINT_LABELS) if label.endswith(joint_type)]
        target = targets[:, indices]
        real_residual = real_actual[:, indices] - target
        sim_residual = sim_actual[:, indices] - target
        type_metrics.append({
            "joint_type": joint_type,
            "real_mae_rad": float(np.mean(np.abs(real_residual))),
            "sim_mae_rad": float(np.mean(np.abs(sim_residual))),
            "real_rmse_rad": float(np.sqrt(np.mean(real_residual ** 2))),
            "sim_rmse_rad": float(np.sqrt(np.mean(sim_residual ** 2))),
        })

    return {
        "run": "20260729_105057",
        "frames": frames,
        "nominal_duration_s": frames * 0.02,
        "settle_time_s": 5.0,
        "sim_timestep_s": float(model.opt.timestep),
        "policy_substeps": substeps,
        "base_z_min_m": float(base_z.min()),
        "base_z_max_m": float(base_z.max()),
        "static_hfe": static_hfe_rows,
        "type_metrics": type_metrics,
        "interpretation_limit": (
            "The first 1000 targets are near-static, so the replay tests the nominal "
            "PD/load equilibrium. It is not a closed-loop reproduction of backward walking."
        ),
    }


def audit_imu_side_evidence() -> list[dict]:
    """Audit same-day static IMU rows that are separate from the two motor logs.

    These 12:xx files cannot establish causality for the 10:50/10:59 falls.
    They are included only to test whether the deployed projected-gravity
    channel was internally consistent with the measured specific force.
    """
    output = []
    for group, filenames in IMU_SIDE_EVIDENCE_GROUPS.items():
        gravity_rows = []
        gyro_rows = []
        acceleration_rows = []
        retained_by_file = {}
        for filename in filenames:
            path = IMU_LOG_ROOT / filename
            retained = 0
            with path.open("r", encoding="utf-8", newline="") as handle:
                reader = csv.DictReader(handle)
                for row in reader:
                    try:
                        imu_valid = float(row["imu_valid"])
                        command = np.asarray([
                            float(row[f"velocity_command_{axis}"])
                            for axis in range(3)
                        ])
                        gyro = np.asarray([
                            float(row[f"body_angular_velocity_{axis}"])
                            for axis in range(3)
                        ])
                        joint_velocity = np.asarray([
                            float(row[f"joint_velocity_{joint}"])
                            for joint in range(18)
                        ])
                        gravity = np.asarray([
                            float(row[f"body_gravity_vector_{axis}"])
                            for axis in range(3)
                        ])
                        acceleration = np.asarray([
                            float(row[f"body_linear_acceleration_{axis}"])
                            for axis in range(3)
                        ])
                    except (KeyError, TypeError, ValueError):
                        continue
                    values = np.concatenate((
                        command, gyro, joint_velocity, gravity, acceleration,
                    ))
                    if not np.all(np.isfinite(values)):
                        continue
                    if not (
                        imu_valid == 1.0
                        and np.linalg.norm(command) < 0.02
                        and np.linalg.norm(gyro) < 0.05
                        and np.sqrt(np.mean(joint_velocity ** 2)) < 0.15
                    ):
                        continue
                    gravity_rows.append(gravity)
                    gyro_rows.append(gyro)
                    acceleration_rows.append(acceleration)
                    retained += 1
            retained_by_file[filename] = retained

        gravity = np.asarray(gravity_rows, dtype=np.float64)
        gyro = np.asarray(gyro_rows, dtype=np.float64)
        acceleration = np.asarray(acceleration_rows, dtype=np.float64)
        if len(gravity) == 0:
            raise RuntimeError(f"{group}: no static IMU rows passed the audit filter")
        gravity_norm = np.linalg.norm(gravity, axis=1)
        acceleration_norm = np.linalg.norm(acceleration, axis=1)
        nonzero = (gravity_norm > 0.0) & (acceleration_norm > 0.0)
        alignment = np.sum(
            acceleration[nonzero] * -gravity[nonzero], axis=1
        ) / (acceleration_norm[nonzero] * gravity_norm[nonzero])
        compensation_residual = np.linalg.norm(
            acceleration + 9.81 * gravity, axis=1
        )
        output.append({
            "group": group,
            "filenames": list(filenames),
            "static_filter": {
                "imu_valid": 1,
                "velocity_command_l2_lt": 0.02,
                "body_angular_velocity_l2_rad_s_lt": 0.05,
                "joint_velocity_rms_rad_s_lt": 0.15,
            },
            "retained_by_file": retained_by_file,
            "static_samples": len(gravity),
            "gravity_component_median": np.median(gravity, axis=0),
            "gyro_component_median_rad_s": np.median(gyro, axis=0),
            "acceleration_component_median_m_s2": np.median(acceleration, axis=0),
            "alignment_cosine_median": float(np.median(alignment)),
            "gravity_compensation_residual_m_s2_median": float(
                np.median(compensation_residual)
            ),
        })
    return output


def main() -> None:
    loaded = {run: load_run(run) for run in RUNS}
    audits = [audit_run(run, loaded[run]) for run in RUNS]
    replay = replay_static_targets(loaded[RUNS[0]])
    timing = infer_period_from_flush_mtimes(loaded[RUNS[0]])
    imu_side_evidence = audit_imu_side_evidence()

    source_paths = {
        "deployment": SIM2REAL_ROOT / "deploy/deploy_real/deploy_real_hexapod_tethered.py",
        "motor_thread": SIM2REAL_ROOT / "deploy/deploy_real/motor_igh_sdk/deploy_real_el4090_pysoem.py",
        "imu_deployment": SIM2REAL_ROOT / "deploy/deploy_real/imu_sdk_deta40/imu_sdk.py",
        "mujoco_task": MUJOCO_ROOT / "mujoco_playground/_src/locomotion/hexapod_tethered_04/joystick.py",
        "mujoco_randomizer": MUJOCO_ROOT / "mujoco_playground/_src/locomotion/hexapod_tethered_04/randomize.py",
        "mujoco_xml": MUJOCO_ROOT / "mujoco_playground/_src/locomotion/hexapod_tethered_04/xmls/hexapod_tethered_04_mjx_fullcollisions_flat.xml",
        "mujoco_scene": MUJOCO_SCENE,
        "isaac_safe_config": ISAAC_ROOT / "legged_gym/legged_gym/envs/el_4090/safe/el_4090_safe_config.py",
        "isaac_safe_env": ISAAC_ROOT / "legged_gym/legged_gym/envs/el_4090/safe/el_4090_safe.py",
    }

    output = {
        "scope": {
            "policy_checkpoint_loaded": False,
            "reason": "repository checkpoint is not the checkpoint deployed on the robot",
            "runs": [run.removeprefix("motor_logs_") for run in RUNS],
        },
        "run_audits": audits,
        "static_logged_target_replay": replay,
        "control_period_inference_105057": timing,
        "imu_side_evidence_12xx": imu_side_evidence,
        "source_sha256": {name: sha256(path) for name, path in source_paths.items()},
        "imu_side_evidence_sha256": {
            filename: sha256(IMU_LOG_ROOT / filename)
            for filenames in IMU_SIDE_EVIDENCE_GROUPS.values()
            for filename in filenames
        },
        "causal_limitations": [
            "Each CSV row logs the current measured position beside a newly computed target before that target is written to the motor command buffer.",
            "The two motor-log directories contain no velocity command, IMU attitude, projected gravity, motor current/torque, fault flags, or per-step monotonic timestamp.",
            "Run 105057 has unequal per-joint row counts, so only the common prefix is synchronized across all 18 joints.",
            "The exact robot-side checkpoint, deployment script, config, and source hashes were not recorded in these CSV files.",
            "The 12:xx IMU files are same-day side evidence, not synchronized measurements from either 10:xx motor-log run.",
        ],
    }
    output_path = OUT_DIR / "derived_metrics.json"
    output_path.write_text(
        json.dumps(as_builtin(output), ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    print(output_path)


if __name__ == "__main__":
    main()
