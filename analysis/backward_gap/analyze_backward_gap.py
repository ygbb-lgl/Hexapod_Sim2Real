#!/usr/bin/env python3
"""Reproduce the 2026-07-29 hexapod backward-locomotion diagnosis.

Run with the isaacgym environment because the deployed policy is TorchScript:

    /home/lgl/anaconda3/envs/isaacgym/bin/python \
      analysis/backward_gap/analyze_backward_gap.py

The script reads existing deployment logs only and writes:
  * derived_metrics.json
  * artifact.json
"""

from __future__ import annotations

import csv
import glob
import json
import math
import os
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path

import numpy as np
import torch


ROOT = Path(__file__).resolve().parents[2]
OUT_DIR = Path(__file__).resolve().parent
POLICY_PATH = ROOT / "deploy/pre_train/hexapod_tethered/policy_10000.pt"
TRAIN_CHECKPOINT = Path(
    "/home/lgl/mujoco_hexapod/logs_rslrl/"
    "HexapodTethered04JoystickFullCollisionsFlatRandomTerrain-20260514-222508/"
    "model_10000.pt"
)

JOINT_LABELS = [
    "RF_HAA", "RF_HFE", "RF_KFE",
    "RM_HAA", "RM_HFE", "RM_KFE",
    "RB_HAA", "RB_HFE", "RB_KFE",
    "LF_HAA", "LF_HFE", "LF_KFE",
    "LM_HAA", "LM_HFE", "LM_KFE",
    "LB_HAA", "LB_HFE", "LB_KFE",
]
REAR_SWING_JOINTS = [7, 8, 16, 17]


def read_csv_rows(path: Path) -> list[dict[str, object]]:
    """Read complete numeric rows and skip a possibly truncated final row."""
    rows: list[dict[str, object]] = []
    with path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        expected = set(reader.fieldnames or [])
        for raw in reader:
            if not expected or set(raw) != expected or any(raw[key] in (None, "") for key in expected):
                continue
            converted: dict[str, object] = {}
            for key in expected:
                try:
                    converted[key] = float(raw[key])
                except (TypeError, ValueError):
                    converted[key] = raw[key]
            rows.append(converted)
    return rows


def mean(values) -> float:
    if not isinstance(values, (list, tuple, np.ndarray)):
        values = list(values)
    return float(np.mean(np.asarray(values, dtype=np.float64)))


def aggregate_direction_response(
    detailed_runs: list[tuple[str, list[dict[str, float]]]],
) -> tuple[dict[str, dict[str, float]], list[dict[str, float]]]:
    cells: list[dict[str, float]] = []
    for run_name, rows in detailed_runs:
        for direction in ("back", "forward"):
            if direction == "back":
                selected = [
                    row for row in rows
                    if row["velocity_command_0"] < -0.2
                    and abs(row["velocity_command_1"]) < 0.15
                    and abs(row["velocity_command_2"]) < 0.15
                ]
            else:
                selected = [
                    row for row in rows
                    if row["velocity_command_0"] > 0.2
                    and abs(row["velocity_command_1"]) < 0.15
                    and abs(row["velocity_command_2"]) < 0.15
                ]
            if len(selected) < 20:
                continue
            for joint in REAR_SWING_JOINTS:
                action = np.asarray(
                    [row[f"policy_action_{joint}"] / 0.25 for row in selected],
                    dtype=np.float64,
                )
                q = np.asarray(
                    [row[f"joint_position_{joint}"] for row in selected],
                    dtype=np.float64,
                )
                dq = np.asarray(
                    [row[f"joint_velocity_{joint}"] for row in selected],
                    dtype=np.float64,
                )
                cells.append(
                    {
                        "run": run_name,
                        "direction": direction,
                        "joint": joint,
                        "frames": len(selected),
                        "cmd_vx": mean(row["velocity_command_0"] for row in selected),
                        "action_abs_mean": mean(np.abs(action)),
                        "action_std": float(np.std(action, ddof=1)),
                        "action_p2p": float(np.ptp(action)),
                        "action_gt2": mean(np.abs(action) > 2.0),
                        "q_p2p": float(np.ptp(q)),
                        "dq_rms": float(np.sqrt(np.mean(dq * dq))),
                    }
                )

    aggregate: dict[str, dict[str, float]] = {}
    for direction in ("back", "forward"):
        group = [cell for cell in cells if cell["direction"] == direction]
        aggregate[direction] = {
            "run_joint_cells": len(group),
            "frames_sum_across_joints": int(sum(cell["frames"] for cell in group)),
            "cmd_vx": mean(cell["cmd_vx"] for cell in group),
            "action_abs_mean": mean(cell["action_abs_mean"] for cell in group),
            "action_std": mean(cell["action_std"] for cell in group),
            "action_p2p": mean(cell["action_p2p"] for cell in group),
            "action_gt2": mean(cell["action_gt2"] for cell in group),
            "q_p2p": mean(cell["q_p2p"] for cell in group),
            "dq_rms": mean(cell["dq_rms"] for cell in group),
        }
    return aggregate, cells


def tracking_metrics() -> tuple[list[dict[str, float]], dict[str, dict[str, float]], dict[str, object]]:
    detailed: list[dict[str, float]] = []
    run_quality: dict[str, object] = {}
    base = ROOT / "deploy/deploy_real/0729-data"
    for folder in sorted(base.glob("motor_logs_*")):
        run_name = folder.name[len("motor_logs_"):]
        usable = 0
        total_rows = 0
        action_values: list[float] = []
        for path in sorted(folder.glob("motor_*_policy_idx_*.csv")):
            if path.stat().st_size == 0:
                continue
            rows = read_csv_rows(path)
            if not rows:
                continue
            usable += 1
            total_rows += len(rows)
            joint = int(path.stem.rsplit("_", 1)[-1])
            target = np.asarray([row["target_pos"] for row in rows], dtype=np.float64)
            actual = np.asarray([row["actual_pos"] for row in rows], dtype=np.float64)
            action = np.asarray([row["action"] for row in rows], dtype=np.float64)
            error = actual - target
            target_centered = target - target.mean()
            actual_centered = actual - actual.mean()
            corr = float(np.corrcoef(target_centered, actual_centered)[0, 1])
            gain = float(actual_centered.std() / target_centered.std())
            regression_slope, regression_intercept = np.polyfit(target, actual, 1)
            best_lag_steps = 0
            best_lag_correlation = corr
            best_lag_rmse = float(np.sqrt(np.mean(error * error)))
            for lag_steps in range(1, 9):
                lag_target = target[:-lag_steps]
                lag_actual = actual[lag_steps:]
                lag_correlation = float(np.corrcoef(lag_target, lag_actual)[0, 1])
                if lag_correlation > best_lag_correlation:
                    best_lag_steps = lag_steps
                    best_lag_correlation = lag_correlation
                    best_lag_rmse = float(
                        np.sqrt(np.mean((lag_actual - lag_target) ** 2))
                    )
            side_sign = 1.0 if joint < 9 else -1.0
            detailed.append(
                {
                    "run": run_name,
                    "joint": joint,
                    "joint_label": JOINT_LABELS[joint],
                    "joint_type": JOINT_LABELS[joint].split("_", 1)[1],
                    "bias_rad": float(error.mean()),
                    "abs_bias_rad": float(abs(error.mean())),
                    "side_adjusted_bias_rad": float(side_sign * error.mean()),
                    "rmse_rad": float(np.sqrt(np.mean(error * error))),
                    "correlation": corr,
                    "dynamic_gain": gain,
                    "regression_slope": float(regression_slope),
                    "regression_intercept_rad": float(regression_intercept),
                    "best_lag_steps": best_lag_steps,
                    "best_lag_ms_at_50hz": best_lag_steps * 20,
                    "best_lag_correlation": best_lag_correlation,
                    "best_lag_rmse_rad": best_lag_rmse,
                    "target_p2p_rad": float(np.ptp(target)),
                    "actual_p2p_rad": float(np.ptp(actual)),
                    "rows": len(rows),
                }
            )
            action_values.extend(action.tolist())
        if action_values:
            action_array = np.asarray(action_values, dtype=np.float64)
            run_quality[run_name] = {
                "usable_joint_csvs": usable,
                "rows": total_rows,
                "action_min": float(action_array.min()),
                "action_max": float(action_array.max()),
                "action_abs_gt_2_count": int(np.sum(np.abs(action_array) > 2.0)),
                "action_abs_gt_2_share": float(np.mean(np.abs(action_array) > 2.0)),
            }
        else:
            run_quality[run_name] = {
                "usable_joint_csvs": 0,
                "rows": 0,
                "note": "18 motor CSV files are zero-byte and unusable",
            }

    by_type: dict[str, dict[str, float]] = {}
    for joint_type in ("HAA", "HFE", "KFE"):
        group = [row for row in detailed if row["joint_type"] == joint_type]
        by_type[joint_type] = {
            "n_run_joint_pairs": len(group),
            "mean_abs_bias_rad": mean(row["abs_bias_rad"] for row in group),
            "mean_rmse_rad": mean(row["rmse_rad"] for row in group),
            "median_correlation": float(np.median([row["correlation"] for row in group])),
            "mean_dynamic_gain": mean(row["dynamic_gain"] for row in group),
            "mean_regression_slope": mean(row["regression_slope"] for row in group),
            "median_best_lag_steps": float(np.median([row["best_lag_steps"] for row in group])),
            "mean_best_lag_correlation": mean(
                row["best_lag_correlation"] for row in group
            ),
        }
    return detailed, by_type, run_quality


def directional_rear_hfe_tracking(
    detailed_runs: list[tuple[str, list[dict[str, float]]]],
) -> tuple[dict[str, dict[str, float]], list[dict[str, float]]]:
    """Compare physical HFE targets and actual positions in command-labeled runs."""
    cells: list[dict[str, float]] = []
    for run_name, rows in detailed_runs:
        for direction in ("back", "forward"):
            if direction == "back":
                selected_indices = [
                    index for index, row in enumerate(rows)
                    if row["velocity_command_0"] < -0.2
                    and abs(row["velocity_command_1"]) < 0.15
                    and abs(row["velocity_command_2"]) < 0.15
                ]
            else:
                selected_indices = [
                    index for index, row in enumerate(rows)
                    if row["velocity_command_0"] > 0.2
                    and abs(row["velocity_command_1"]) < 0.15
                    and abs(row["velocity_command_2"]) < 0.15
                ]
            if len(selected_indices) < 20:
                continue
            selected_set = set(selected_indices)
            for joint in (7, 16):
                target_all = np.asarray(
                    [row[f"policy_action_{joint}"] for row in rows],
                    dtype=np.float64,
                )
                actual_all = np.asarray(
                    [row[f"joint_position_{joint}"] for row in rows],
                    dtype=np.float64,
                )
                target = target_all[selected_indices]
                actual = actual_all[selected_indices]
                error = actual - target
                correlation = float(np.corrcoef(target, actual)[0, 1])
                best_lag_steps = 0
                best_lag_correlation = correlation
                best_lag_rmse = float(np.sqrt(np.mean(error * error)))
                for lag_steps in range(1, 7):
                    valid = [
                        index for index in selected_indices
                        if index + lag_steps < len(rows)
                        and index + lag_steps in selected_set
                    ]
                    if len(valid) < 20:
                        continue
                    lag_target = target_all[valid]
                    lag_actual = actual_all[
                        np.asarray(valid, dtype=np.int64) + lag_steps
                    ]
                    lag_correlation = float(
                        np.corrcoef(lag_target, lag_actual)[0, 1]
                    )
                    if lag_correlation > best_lag_correlation:
                        best_lag_steps = lag_steps
                        best_lag_correlation = lag_correlation
                        best_lag_rmse = float(
                            np.sqrt(np.mean((lag_actual - lag_target) ** 2))
                        )
                cells.append(
                    {
                        "run": run_name,
                        "direction": direction,
                        "joint": joint,
                        "joint_label": JOINT_LABELS[joint],
                        "frames": len(selected_indices),
                        "bias_rad": float(error.mean()),
                        "abs_bias_rad": float(abs(error.mean())),
                        "rmse_rad": float(np.sqrt(np.mean(error * error))),
                        "dynamic_gain": float(actual.std() / target.std()),
                        "correlation": correlation,
                        "target_std_rad": float(target.std()),
                        "actual_std_rad": float(actual.std()),
                        "best_lag_steps": best_lag_steps,
                        "best_lag_correlation": best_lag_correlation,
                        "best_lag_rmse_rad": best_lag_rmse,
                    }
                )

    aggregate: dict[str, dict[str, float]] = {}
    for direction in ("back", "forward"):
        group = [cell for cell in cells if cell["direction"] == direction]
        aggregate[direction] = {
            "run_joint_cells": len(group),
            "abs_bias_rad": mean(cell["abs_bias_rad"] for cell in group),
            "rmse_rad": mean(cell["rmse_rad"] for cell in group),
            "dynamic_gain": mean(cell["dynamic_gain"] for cell in group),
            "correlation": mean(cell["correlation"] for cell in group),
            "target_std_rad": mean(cell["target_std_rad"] for cell in group),
            "actual_std_rad": mean(cell["actual_std_rad"] for cell in group),
            "median_best_lag_steps": float(
                np.median([cell["best_lag_steps"] for cell in group])
            ),
            "best_lag_correlation": mean(
                cell["best_lag_correlation"] for cell in group
            ),
            "best_lag_rmse_rad": mean(
                cell["best_lag_rmse_rad"] for cell in group
            ),
        }
    return aggregate, cells


def build_real_observation_matrix(
    detailed_runs: list[tuple[str, list[dict[str, float]]]],
    policy,
) -> tuple[np.ndarray, list[str], list[dict[str, float]]]:
    labels = (
        ["linvel_x", "linvel_y", "linvel_z", "gyro_x", "gyro_y", "gyro_z",
         "gravity_x", "gravity_y", "gravity_z"]
        + [f"q_{idx}" for idx in range(18)]
        + [f"dq_{idx}" for idx in range(18)]
        + [f"last_action_{idx}" for idx in range(19)]
        + ["cmd_vx", "cmd_vy", "cmd_wz", "cable_force", "arm_trunk", "tether_y"]
    )
    chunks: list[np.ndarray] = []
    gravity_checks: list[dict[str, float]] = []
    for run_name, rows in detailed_runs:
        matrix = np.full((len(rows), 70), np.nan, dtype=np.float64)
        for row_index, row in enumerate(rows):
            matrix[row_index, 3:6] = [row[f"body_angular_velocity_{i}"] for i in range(3)]
            matrix[row_index, 6:9] = [row[f"body_gravity_vector_{i}"] for i in range(3)]
            matrix[row_index, 9:27] = [row[f"joint_position_{i}"] for i in range(18)]
            matrix[row_index, 27:45] = [row[f"joint_velocity_{i}"] for i in range(18)]
            matrix[row_index, 45:63] = [row[f"policy_action_{i}"] / 0.25 for i in range(18)]
            matrix[row_index, 64:67] = [row[f"velocity_command_{i}"] for i in range(3)]
            matrix[row_index, 67] = row["force_raw_n"]
            matrix[row_index, 68] = 0.0
        chunks.append(matrix)
        gravity = np.asarray(
            [[row[f"body_gravity_vector_{i}"] for i in range(3)] for row in rows],
            dtype=np.float64,
        )
        acceleration = np.asarray(
            [[row[f"body_linear_acceleration_{i}"] for i in range(3)] for row in rows],
            dtype=np.float64,
        )
        residual = acceleration.mean(axis=0) + 9.81 * gravity.mean(axis=0)
        gravity_checks.append(
            {
                "run": run_name,
                "frames": len(rows),
                "gravity_x_mean": float(gravity[:, 0].mean()),
                "gravity_y_mean": float(gravity[:, 1].mean()),
                "gravity_z_mean": float(gravity[:, 2].mean()),
                "acc_x_mean": float(acceleration[:, 0].mean()),
                "static_residual_x_mps2": float(residual[0]),
                "static_residual_y_mps2": float(residual[1]),
                "static_residual_z_mps2": float(residual[2]),
            }
        )
    return np.concatenate(chunks, axis=0), labels, gravity_checks


def observation_ood(matrix: np.ndarray, labels: list[str], policy) -> list[dict[str, float]]:
    normalizer_mean = policy.normalizer.mean.detach().cpu().numpy().astype(np.float64)
    normalizer_std = policy.normalizer.std.detach().cpu().numpy().astype(np.float64)
    z_abs = np.abs((matrix - normalizer_mean) / normalizer_std)
    results: list[dict[str, float]] = []
    for feature_index in range(matrix.shape[1]):
        values = z_abs[:, feature_index]
        values = values[np.isfinite(values)]
        if not len(values):
            continue
        results.append(
            {
                "feature_index": feature_index,
                "feature": labels[feature_index],
                "median_abs_z": float(np.median(values)),
                "p95_abs_z": float(np.quantile(values, 0.95)),
                "share_gt_3sigma": float(np.mean(values > 3.0)),
                "share_gt_5sigma": float(np.mean(values > 5.0)),
                "rows": len(values),
            }
        )
    return sorted(results, key=lambda row: row["median_abs_z"], reverse=True)


def policy_sensitivity(
    detailed_runs: list[tuple[str, list[dict[str, float]]]],
    policy,
) -> list[dict[str, float]]:
    normalizer_mean = policy.normalizer.mean.detach().cpu().numpy().astype(np.float32)
    samples: list[np.ndarray] = []
    commands: list[float] = []
    forces: list[float] = []
    for _, rows in detailed_runs:
        selected_indices = [
            idx for idx, row in enumerate(rows)
            if idx > 0
            and row["velocity_command_0"] < -0.2
            and abs(row["velocity_command_1"]) < 0.15
            and abs(row["velocity_command_2"]) < 0.15
        ][::20]
        for idx in selected_indices:
            row = rows[idx]
            previous = rows[idx - 1]
            obs = normalizer_mean.copy()
            obs[3:6] = [row[f"body_angular_velocity_{i}"] for i in range(3)]
            obs[6:9] = [row[f"body_gravity_vector_{i}"] for i in range(3)]
            obs[9:27] = [row[f"joint_position_{i}"] for i in range(18)]
            obs[27:45] = [row[f"joint_velocity_{i}"] for i in range(18)]
            obs[45:63] = [previous[f"policy_action_{i}"] / 0.25 for i in range(18)]
            obs[63] = normalizer_mean[63]
            obs[64:67] = [row[f"velocity_command_{i}"] for i in range(3)]
            obs[68] = 0.0
            obs[69] = normalizer_mean[69]
            samples.append(obs)
            commands.append(row["velocity_command_0"])
            forces.append(row["force_raw_n"])

    obs_base = np.asarray(samples, dtype=np.float32)
    commands_array = np.asarray(commands, dtype=np.float32)
    forces_array = np.asarray(forces, dtype=np.float32)
    scenarios = [
        ("匹配后退速度 + 训练典型张力", commands_array, np.full_like(commands_array, normalizer_mean[67])),
        ("零线速度 + 训练典型张力", np.zeros_like(commands_array), np.full_like(commands_array, normalizer_mean[67])),
        ("错误 +0.5 m/s + 训练典型张力", np.full_like(commands_array, 0.5), np.full_like(commands_array, normalizer_mean[67])),
        ("匹配后退速度 + 实物张力", commands_array, forces_array),
        ("错误 +0.5 m/s + 实物张力", np.full_like(commands_array, 0.5), forces_array),
    ]
    outputs: list[dict[str, float]] = []
    reference = None
    for name, velocity_x, cable_force in scenarios:
        obs = obs_base.copy()
        obs[:, 0] = velocity_x
        obs[:, 1:3] = normalizer_mean[1:3]
        obs[:, 67] = cable_force
        with torch.no_grad():
            action = policy(torch.from_numpy(obs)).detach().cpu().numpy()
        if reference is None:
            reference = action.copy()
        rear_action = action[:, REAR_SWING_JOINTS]
        outputs.append(
            {
                "scenario": name,
                "samples": len(obs),
                "rear_action_abs_mean": float(np.mean(np.abs(rear_action))),
                "rear_action_joint_mean_std": float(np.mean(np.std(rear_action, axis=0))),
                "rear_action_rms_delta_vs_reference": float(
                    np.sqrt(np.mean((rear_action - reference[:, REAR_SWING_JOINTS]) ** 2))
                ),
            }
        )
    return outputs


def checkpoint_matches_deployed_policy(policy) -> dict[str, object]:
    checkpoint = torch.load(
        TRAIN_CHECKPOINT,
        map_location="cpu",
        weights_only=True,
    )["model_state_dict"]
    deployed = policy.state_dict()
    pairs = [
        ("actor.actor.0.weight", "actor.0.weight"),
        ("actor.actor.0.bias", "actor.0.bias"),
        ("actor.actor.2.weight", "actor.2.weight"),
        ("actor.actor.2.bias", "actor.2.bias"),
        ("actor.actor.4.weight", "actor.4.weight"),
        ("actor.actor.4.bias", "actor.4.bias"),
        ("actor.actor.6.weight", "actor.6.weight"),
        ("actor.actor.6.bias", "actor.6.bias"),
        ("normalizer.mean", "actor_obs_normalizer._mean"),
        ("normalizer.std", "actor_obs_normalizer._std"),
    ]
    max_diffs = []
    for deployed_key, checkpoint_key in pairs:
        left = deployed[deployed_key].reshape(-1)
        right = checkpoint[checkpoint_key].reshape(-1)
        max_diffs.append(float(torch.max(torch.abs(left - right))))
    return {
        "checkpoint": TRAIN_CHECKPOINT.name,
        "training_run": TRAIN_CHECKPOINT.parent.name,
        "compared_tensors": len(pairs),
        "max_abs_parameter_or_normalizer_difference": max(max_diffs),
        "exact_match": max(max_diffs) == 0.0,
    }


def write_json(path: Path, payload: object) -> None:
    path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )


def build_artifact(derived: dict[str, object], generated_at: str) -> dict[str, object]:
    direction = derived["direction_response"]
    back = direction["back"]
    forward = direction["forward"]
    rear_action_ratio = back["action_abs_mean"] / forward["action_abs_mean"]
    rear_q_ratio = back["q_p2p"] / forward["q_p2p"]
    rear_dq_ratio = back["dq_rms"] / forward["dq_rms"]
    ood_by_name = {row["feature"]: row for row in derived["observation_ood"]}
    tracking = derived["tracking_by_joint_type"]
    directional_hfe = derived["directional_rear_hfe_tracking"]
    backward_hfe = directional_hfe["back"]
    forward_hfe = directional_hfe["forward"]
    gravity_checks = derived["gravity_acceleration_consistency"]
    inconsistent_gravity_runs = [
        row for row in gravity_checks if abs(row["static_residual_x_mps2"]) > 3.0
    ]

    headline = [{
        "rear_action_ratio": rear_action_ratio,
        "rear_action_back": back["action_abs_mean"],
        "rear_action_forward": forward["action_abs_mean"],
        "rear_dq_ratio": rear_dq_ratio,
        "rear_dq_back": back["dq_rms"],
        "rear_dq_forward": forward["dq_rms"],
        "cable_median_z": ood_by_name["cable_force"]["median_abs_z"],
        "cable_p95_z": ood_by_name["cable_force"]["p95_abs_z"],
        "gravity_x_median_z": ood_by_name["gravity_x"]["median_abs_z"],
        "gravity_x_gt3_share": ood_by_name["gravity_x"]["share_gt_3sigma"],
        "hfe_side_adjusted_bias_rad": tracking["HFE"]["mean_abs_bias_rad"],
        "hfe_dynamic_gain": tracking["HFE"]["mean_dynamic_gain"],
        "hfe_regression_slope": tracking["HFE"]["mean_regression_slope"],
    }]
    rear_ratio = [
        {"metric": "策略动作平均幅值", "backward_vs_forward": rear_action_ratio},
        {"metric": "关节位置峰峰值", "backward_vs_forward": rear_q_ratio},
        {"metric": "关节速度 RMS", "backward_vs_forward": rear_dq_ratio},
    ]
    ood_chart = [
        {
            "feature": feature,
            "feature_label": label,
            "median_abs_z": ood_by_name[feature]["median_abs_z"],
            "p95_abs_z": ood_by_name[feature]["p95_abs_z"],
            "share_gt_3sigma": ood_by_name[feature]["share_gt_3sigma"],
        }
        for feature, label in [
            ("cable_force", "绳索张力 obs[67]"),
            ("gravity_x", "重力 x obs[6]"),
            ("gravity_z", "重力 z obs[8]"),
            ("q_16", "LB_HFE 位置"),
        ]
    ]
    tracking_chart = []
    for joint_type in ("HAA", "HFE", "KFE"):
        tracking_chart.extend([
            {
                "joint_type": joint_type,
                "metric": "跟踪 RMSE",
                "value_rad": tracking[joint_type]["mean_rmse_rad"],
            },
            {
                "joint_type": joint_type,
                "metric": "平均绝对偏差",
                "value_rad": tracking[joint_type]["mean_abs_bias_rad"],
            },
        ])
    hfe_backward_forward_chart = []
    for direction_key, direction_label, values in (
        ("back", "后退", backward_hfe),
        ("forward", "前进", forward_hfe),
    ):
        hfe_backward_forward_chart.extend([
            {
                "direction": direction_key,
                "direction_label": direction_label,
                "metric": "目标波动标准差",
                "value_rad": values["target_std_rad"],
            },
            {
                "direction": direction_key,
                "direction_label": direction_label,
                "metric": "实际波动标准差",
                "value_rad": values["actual_std_rad"],
            },
            {
                "direction": direction_key,
                "direction_label": direction_label,
                "metric": "平均绝对偏差",
                "value_rad": values["abs_bias_rad"],
            },
        ])

    root_causes = [
        {
            "priority": 1,
            "finding": "后退 HFE 目标太小，被实物 HFE 的同向下垂和低动态增益吞掉",
            "evidence": (
                f"0729 六个 HFE 左右镜像后的平均偏差为 "
                f"{tracking['HFE']['mean_abs_bias_rad']:.3f} rad，动态增益 "
                f"{tracking['HFE']['mean_dynamic_gain']:.3f}；同日后退目标标准差仅 "
                f"{backward_hfe['target_std_rad']:.3f} rad"
            ),
            "confidence": "高（直接机制）",
            "recommended_test": "先做离地无负载，再做支撑承重的 HFE 阶跃/扫频；同时记录相电流和驱动限流状态",
        },
        {
            "priority": 2,
            "finding": "obs[67] 绳索张力符号/尺度与训练严重不一致",
            "evidence": (
                f"实物中位数为训练均值以上 {ood_by_name['cable_force']['median_abs_z']:.2f}σ；"
                f"{ood_by_name['cable_force']['share_gt_3sigma']:.1%} 样本超过 3σ"
            ),
            "confidence": "高",
            "recommended_test": "悬空/支撑条件下把 obs[67] 暂置训练均值 −3.82，只做低速动作 A/B",
        },
        {
            "priority": 3,
            "finding": "线速度回退估计不可观且重力补偿公式与策略重力公式不一致",
            "evidence": "IMU 文件对同一四元数使用两套不同的 gx/gy 公式；线速度未被现有日志记录",
            "confidence": "代码缺陷高、该次运行触发情况中等",
            "recommended_test": "静止 10 秒记录 velocity_source、linvel、两套重力向量及补偿后加速度",
        },
        {
            "priority": 4,
            "finding": "重力 x 观测越界或 IMU 安装/坐标变换不一致",
            "evidence": (
                f"重力 x 中位绝对偏差 {ood_by_name['gravity_x']['median_abs_z']:.2f}σ，"
                f"{ood_by_name['gravity_x']['share_gt_3sigma']:.1%} 样本超过 3σ；"
                f"{len(inconsistent_gravity_runs)}/{len(gravity_checks)} 次日志的"
                "均值满足 |a_x+9.81·g_x|>3 m/s²"
            ),
            "confidence": "高",
            "recommended_test": "机器人水平静止时校验 acc + 9.81·gravity≈0，并固定 IMU→URDF 外参",
        },
        {
            "priority": 5,
            "finding": "当前脚本与日志的动作限幅版本不一致",
            "evidence": "当前主脚本限幅 ±2，但两次可用 0729 日志均包含 |action|>2 的样本",
            "confidence": "高（复现性问题）",
            "recommended_test": "冻结一次部署提交和配置哈希；训练和部署统一动作限幅",
        },
        {
            "priority": 6,
            "finding": "单纯增加 URDF 质量或引用 90 N·m 额定扭矩即可解释",
            "evidence": "HFE 有明确承重偏差和低闭环增益；额定峰值不等于当前控制器实际输出电流/关节刚度",
            "confidence": "低概率",
            "recommended_test": "获取相电流/母线电压/限流标志后，再辨识实物 Kp 到输出力矩的真实映射",
        },
    ]

    sources = [
        {
            "id": "src_derived",
            "label": "本次离线诊断派生指标",
            "path": "analysis/backward_gap/derived_metrics.json",
            "query": {
                "engine": "DuckDB",
                "language": "sql",
                "sql": (
                    "SELECT * FROM "
                    "read_json_auto('analysis/backward_gap/derived_metrics.json');"
                ),
                "description": "读取由分析脚本生成并复核的派生指标快照",
                "tables_used": ["analysis/backward_gap/derived_metrics.json"],
                "filters": [
                    "后退：vx<-0.2 且 |vy|,|wz|<0.15",
                    "前进：vx>0.2 且 |vy|,|wz|<0.15",
                    "后腿摆动关节：policy index 7,8,16,17",
                ],
                "metric_definitions": [
                    "动作平均幅值：按 run×joint 先计算 |raw policy action| 均值，再等权平均",
                    "关节速度 RMS：按 run×joint 先计算 RMS，再等权平均",
                    "训练 z-score：(real observation - deployed normalizer mean) / deployed normalizer std",
                ],
            },
        },
        {
            "id": "src_0729_joint",
            "label": "2026-07-29 三次关节跟踪日志",
            "path": "deploy/deploy_real/0729-data",
        },
        {
            "id": "src_predata",
            "label": "2026-07-29 同日带命令/IMU/张力的八次实物日志",
            "path": "deploy/deploy_real/pre_data",
        },
        {
            "id": "src_deploy",
            "label": "当前 Sim2Real 主脚本",
            "path": "deploy/deploy_real/deploy_real_hexapod_tethered_plot_torque.py",
        },
        {
            "id": "src_imu",
            "label": "Deta40 IMU Python SDK",
            "path": "deploy/deploy_real/imu_sdk_deta40/imu_sdk.py",
        },
        {
            "id": "src_motor_py",
            "label": "电机 Python SDK",
            "path": "deploy/deploy_real/motor_igh_sdk/el4090_motor_sdk.py",
        },
        {
            "id": "src_motor_cpp",
            "label": "官方 C 电机 SDK",
            "path": "deploy/deploy_real/app_cpp/motor_control.c",
        },
        {
            "id": "src_policy",
            "label": "实际部署 TorchScript 策略",
            "path": "deploy/pre_train/hexapod_tethered/policy_10000.pt",
        },
    ]

    title = "六足机器人“实物无法后退”Sim2Real 技术诊断"
    manifest = {
        "version": 1,
        "surface": "report",
        "title": title,
        "description": "基于 2026-07-29 实物日志、部署代码、策略归一化器和电机 SDK 的证据化根因排序。",
        "generatedAt": generated_at,
        "sources": sources,
        "cards": [
            {
                "id": "card_action_ratio",
                "dataset": "headline_metrics",
                "sourceId": "src_derived",
                "description": "后退时四个后腿 HFE/KFE 的原始策略动作平均幅值相对前进明显收缩。",
                "metrics": [
                    {"label": "后退/前进动作幅值", "field": "rear_action_ratio", "format": "percent"},
                    {"label": "后退均值", "field": "rear_action_back", "format": "number"},
                    {"label": "前进均值", "field": "rear_action_forward", "format": "number"},
                ],
            },
            {
                "id": "card_hfe_tracking",
                "dataset": "headline_metrics",
                "sourceId": "src_derived",
                "description": "六个 HFE 对目标变化的平均幅值传递，以及左右镜像后的同向静差。",
                "metrics": [
                    {"label": "HFE 动态增益", "field": "hfe_dynamic_gain", "format": "percent"},
                    {"label": "平均偏差（rad）", "field": "hfe_side_adjusted_bias_rad", "format": "number"},
                    {"label": "回归斜率", "field": "hfe_regression_slope", "format": "number"},
                ],
            },
            {
                "id": "card_dq_ratio",
                "dataset": "headline_metrics",
                "sourceId": "src_derived",
                "description": "后退后腿关节速度 RMS 相对前进的比例。",
                "metrics": [
                    {"label": "后退/前进速度 RMS", "field": "rear_dq_ratio", "format": "percent"},
                    {"label": "后退 RMS", "field": "rear_dq_back", "format": "number"},
                    {"label": "前进 RMS", "field": "rear_dq_forward", "format": "number"},
                ],
            },
            {
                "id": "card_cable_z",
                "dataset": "headline_metrics",
                "sourceId": "src_derived",
                "description": "实物张力观测相对策略训练归一化分布的偏离。",
                "metrics": [
                    {"label": "张力中位偏离（σ）", "field": "cable_median_z", "format": "number"},
                    {"label": "P95 偏离（σ）", "field": "cable_p95_z", "format": "number"},
                ],
            },
            {
                "id": "card_gravity_z",
                "dataset": "headline_metrics",
                "sourceId": "src_derived",
                "description": "重力 x 观测相对策略训练归一化分布的偏离。",
                "metrics": [
                    {"label": "重力 x 中位偏离（σ）", "field": "gravity_x_median_z", "format": "number"},
                    {"label": "超过 3σ 的样本", "field": "gravity_x_gt3_share", "format": "percent"},
                ],
            },
        ],
        "charts": [
            {
                "id": "chart_rear_ratio",
                "title": "后退后腿响应相对前进的比例",
                "subtitle": "策略动作和关节速度均显著收缩，衰减在电机执行前已经出现。",
                "type": "bar",
                "intent": "comparison",
                "dataset": "rear_response_ratio",
                "sourceId": "src_derived",
                "encodings": {
                    "x": {"field": "metric", "type": "nominal", "label": "后腿响应指标"},
                    "y": {
                        "field": "backward_vs_forward",
                        "type": "quantitative",
                        "format": "percent",
                        "label": "后退 / 前进",
                    },
                },
                "valueFormat": "percent",
                "settings": {"showValues": True, "sort": "none"},
                "layout": "full",
            },
            {
                "id": "chart_hfe_backward_forward",
                "title": "前进与后退的后腿 HFE 目标—实际幅值",
                "subtitle": "八次同日带命令日志；数值为 RB_HFE/LB_HFE 按 run×joint 等权平均的标准差或绝对偏差。",
                "type": "bar",
                "intent": "comparison",
                "dataset": "hfe_backward_forward_chart",
                "sourceId": "src_derived",
                "encodings": {
                    "x": {"field": "direction_label", "type": "nominal", "label": "运动方向"},
                    "y": {
                        "field": "value_rad",
                        "type": "quantitative",
                        "label": "角度尺度",
                        "unit": "rad",
                    },
                    "color": {"field": "metric", "type": "nominal", "label": "指标"},
                },
                "unit": "rad",
                "settings": {"groupMode": "grouped", "showValues": True},
                "palette": {"kind": "categorical"},
                "legend": {"position": "bottom", "title": "指标"},
                "layout": "full",
            },
            {
                "id": "chart_ood",
                "title": "实物观测相对训练分布的中位偏离",
                "subtitle": "张力和重力 x 是唯一大范围越过 3σ 的已记录策略输入。",
                "type": "bar",
                "intent": "comparison",
                "dataset": "observation_ood_chart",
                "sourceId": "src_derived",
                "encodings": {
                    "x": {"field": "feature_label", "type": "nominal", "label": "观测特征"},
                    "y": {
                        "field": "median_abs_z",
                        "type": "quantitative",
                        "label": "中位绝对 z-score",
                        "unit": "σ",
                    },
                },
                "unit": "σ",
                "referenceLines": [
                    {"axis": "y", "value": 3, "label": "3σ 阈值", "lineStyle": "dashed", "color": "red"},
                ],
                "settings": {"showValues": True, "sort": "descending"},
                "layout": "full",
            },
            {
                "id": "chart_tracking",
                "title": "关节类型的实物跟踪误差",
                "subtitle": "HFE 的静差和 RMSE 明显高于 HAA/KFE，是后退抬腿失效的直接执行侧机制。",
                "type": "bar",
                "intent": "comparison",
                "dataset": "tracking_by_joint_type_chart",
                "sourceId": "src_derived",
                "encodings": {
                    "x": {"field": "joint_type", "type": "nominal", "label": "关节类型"},
                    "y": {
                        "field": "value_rad",
                        "type": "quantitative",
                        "label": "角度误差",
                        "unit": "rad",
                    },
                    "color": {"field": "metric", "type": "nominal", "label": "误差指标"},
                },
                "unit": "rad",
                "settings": {"groupMode": "grouped", "showValues": True},
                "palette": {"kind": "categorical"},
                "legend": {"position": "bottom", "title": "误差指标"},
                "layout": "full",
            },
        ],
        "tables": [
            {
                "id": "table_root_causes",
                "title": "根因排序与最小验证",
                "subtitle": "先区分 HFE 无负载零位、承重下垂和驱动限流，再修正导致后退目标过小的观测契约。",
                "dataset": "root_causes",
                "sourceId": "src_derived",
                "defaultSort": {"field": "priority", "direction": "asc"},
                "density": "spacious",
                "columns": [
                    {"field": "priority", "label": "优先级", "type": "number"},
                    {"field": "finding", "label": "候选根因", "type": "text"},
                    {"field": "evidence", "label": "证据", "type": "text"},
                    {"field": "confidence", "label": "置信度", "type": "text"},
                    {"field": "recommended_test", "label": "最小验证", "type": "text"},
                ],
                "layout": "full",
            },
            {
                "id": "table_sensitivity",
                "title": "离线策略单因素敏感性测试",
                "subtitle": "以 406 个真实后退状态采样；缺失的线速度/末端角使用受控替代，因此用于排序而非闭环证明。",
                "dataset": "policy_sensitivity",
                "sourceId": "src_derived",
                "defaultSort": {"field": "rear_action_rms_delta_vs_reference", "direction": "desc"},
                "density": "dense",
                "columns": [
                    {"field": "scenario", "label": "场景", "type": "text"},
                    {"field": "samples", "label": "样本", "type": "number"},
                    {"field": "rear_action_abs_mean", "label": "后腿 |action| 均值", "type": "number"},
                    {"field": "rear_action_joint_mean_std", "label": "跨样本平均标准差", "type": "number"},
                    {"field": "rear_action_rms_delta_vs_reference", "label": "相对基准 RMS 改变量", "type": "number"},
                ],
                "layout": "full",
            },
        ],
        "blocks": [
            {"id": "title", "type": "markdown", "body": f"# {title}", "layout": "full"},
            {
                "id": "technical_summary",
                "type": "markdown",
                "sourceId": "src_derived",
                "layout": "full",
                "body": (
                    "## 技术结论\n\n"
                    "直接失效机制是**后退 HFE 目标太小，又被实物 HFE 的承重偏差和低动态增益吞掉**。"
                    f"0729 两次可用日志中，六个 HFE 左右镜像后的平均偏差为 "
                    f"**{tracking['HFE']['mean_abs_bias_rad']:.3f} rad**，动态增益仅 "
                    f"**{tracking['HFE']['mean_dynamic_gain']:.1%}**，目标—实际回归斜率约 "
                    f"**{tracking['HFE']['mean_regression_slope']:.3f}**。\n\n"
                    f"同日带命令日志中，后退 RB/LB_HFE 目标标准差只有 "
                    f"**{backward_hfe['target_std_rad']:.3f} rad**，与其平均绝对偏差 "
                    f"**{backward_hfe['abs_bias_rad']:.3f} rad** 同量级；前进目标标准差则为 "
                    f"**{forward_hfe['target_std_rad']:.3f} rad**。因此前进能越过下垂区间，"
                    "后退指令则很容易在实际关节上消失。\n\n"
                    f"上游观测错配仍然重要，因为它解释了为什么后退目标异常偏小：obs[67] 张力中位偏离训练分布 "
                    f"**{ood_by_name['cable_force']['median_abs_z']:.2f}σ**，重力 x 偏离 "
                    f"**{ood_by_name['gravity_x']['median_abs_z']:.2f}σ**，线速度回退估计还使用了不一致的重力公式。"
                ),
            },
            {
                "id": "headline_metrics_block",
                "type": "metric-strip",
                "cardIds": [
                    "card_action_ratio",
                    "card_hfe_tracking",
                    "card_dq_ratio",
                    "card_cable_z",
                    "card_gravity_z",
                ],
                "layout": "full",
            },
            {
                "id": "scope",
                "type": "markdown",
                "layout": "full",
                "body": (
                    "## 范围与数据可用性\n\n"
                    "0729-data 三个目录中，10:50:57 和 10:59:10 各有 18 个可用关节 CSV；"
                    "10:55:29 的 18 个 CSV 均为 0 字节。关节 CSV 没有记录速度命令、IMU、张力、"
                    "电流或故障码，因此不能把其中某个时间窗直接标成前进/后退。\n\n"
                    "方向对比采用同日稍后的八次 81 列带命令日志作为支持证据。它们来自带数据记录的脚本版本，"
                    "不是对 10:50 三次实验的逐帧复原。"
                ),
            },
            {"id": "rear_ratio_block", "type": "chart", "chartId": "chart_rear_ratio", "layout": "full"},
            {
                "id": "hfe_direct_mechanism",
                "type": "markdown",
                "sourceId": "src_derived",
                "layout": "full",
                "body": (
                    "## HFE 的 target—actual 差异足以吞掉后退摆幅\n\n"
                    f"你的观察是对的：HFE 不是简单的 40–80 ms 滞后。去均值后的动态增益只有 "
                    f"{tracking['HFE']['mean_dynamic_gain']:.3f}，线性回归斜率只有 "
                    f"{tracking['HFE']['mean_regression_slope']:.3f}。右侧三个 HFE 的 actual−target "
                    f"平均为正，左侧三个平均为负；镜像到同一物理方向后，两侧分别约 "
                    f"{tracking['HFE']['mean_abs_bias_rad']:.3f} rad。这更像同一方向的承重下垂，"
                    "不是随机编码器噪声。\n\n"
                    f"在带命令日志中，后退 HFE 目标波动 {backward_hfe['target_std_rad']:.3f} rad、"
                    f"实际波动 {backward_hfe['actual_std_rad']:.3f} rad、绝对偏差 "
                    f"{backward_hfe['abs_bias_rad']:.3f} rad；前进对应值为 "
                    f"{forward_hfe['target_std_rad']:.3f}/{forward_hfe['actual_std_rad']:.3f}/"
                    f"{forward_hfe['abs_bias_rad']:.3f} rad。后退目标与偏差同量级，"
                    "所以策略虽然发出小幅动作，实体后腿仍可能完全不离地。"
                ),
            },
            {
                "id": "hfe_backward_forward_block",
                "type": "chart",
                "chartId": "chart_hfe_backward_forward",
                "layout": "full",
            },
            {
                "id": "observation_contract",
                "type": "markdown",
                "sourceId": "src_derived",
                "layout": "full",
                "body": (
                    "## 观测契约是首要故障面\n\n"
                    "部署策略的归一化器显示，训练张力均值/标准差约为 −3.82/7.37；"
                    "实物日志张力中位数约 +146 N。不能只把它看成传感器噪声，也不能只做简单取反："
                    "符号、单位和物理量定义都需要与 MuJoCo tendon actuator force 对齐。\n\n"
                    "线速度通道风险更隐蔽。Deta40 没有新鲜 INSGPS 速度时，会积分加速度作为速度；"
                    "但单 IMU 平移速度不可观，而且代码的重力补偿 gx/gy 与策略重力向量 gx/gy 公式不同。"
                    f"八次日志中有 {len(inconsistent_gravity_runs)} 次的均值出现 "
                    "|a_x+9.81·g_x|>3 m/s²，说明加速度轴和策略重力至少有一处符号/外参不一致。"
                    "现有日志没有记录最终送入 obs[0:3] 的值，必须先补日志才能闭环定责。"
                ),
            },
            {"id": "ood_block", "type": "chart", "chartId": "chart_ood", "layout": "full"},
            {
                "id": "actuation_findings",
                "type": "markdown",
                "sourceId": "src_derived",
                "layout": "full",
                "body": (
                    "## HFE 闭环刚度/限流需要独立于额定扭矩验证\n\n"
                    f"两次可用关节日志中，HFE 平均绝对静差为 {tracking['HFE']['mean_abs_bias_rad']:.3f} rad，"
                    f"平均跟踪 RMSE 为 {tracking['HFE']['mean_rmse_rad']:.3f} rad，动态增益约 "
                    f"{tracking['HFE']['mean_dynamic_gain']:.3f}。额定 90 N·m 只说明峰值能力，"
                    "不能证明当前 Kp 字段等于 80 N·m/rad，也不能证明驱动没有电流、母线电压、温度或速度限幅。"
                    "在没有相电流和 fault/limit flag 的情况下，不能用铭牌扭矩排除执行侧问题。\n\n"
                    "joint2motor 映射、方向变换和位置回读的相关性为正，说明不是整体符号接反。"
                    "但如果只是固定 encoder offset，去均值后的动态增益应该接近 1；现在只有约 0.56，"
                    "所以仅重标 motor_offsets 不能完整修复，必须区分无负载零位偏差与承重闭环下垂。"
                    "官方 C SDK 的 torque/current 编码范围与 Python 端不同，但腿部命令的 feedforward torque 当前为 0，"
                    "所以该差异不会造成这次位置目标方向性失效；它会污染将来的电流/扭矩解释。"
                ),
            },
            {"id": "tracking_block", "type": "chart", "chartId": "chart_tracking", "layout": "full"},
            {
                "id": "ranked_causes",
                "type": "markdown",
                "layout": "full",
                "body": (
                    "## 根因排序\n\n"
                    "优先级现在区分“直接机制”和“上游原因”：HFE target—actual 偏差是后腿不抬的直接机制；"
                    "张力、重力和线速度错配是后退目标过小的主要上游候选。"
                    "动作限幅版本不一致影响复现；单纯 URDF 加重或引用额定峰值扭矩仍排在最后。"
                ),
            },
            {"id": "root_causes_block", "type": "table", "tableId": "table_root_causes", "layout": "full"},
            {
                "id": "methodology",
                "type": "markdown",
                "sourceId": "src_derived",
                "layout": "full",
                "body": (
                    "## 方法与离线验证\n\n"
                    "方向段使用 |vy|、|wz|<0.15 排除明显横移/旋转，并对八次 run×四个后腿关节等权汇总。"
                    "训练分布来自实际部署 TorchScript 内置 normalizer；其 actor 权重和 normalizer 与 "
                    "20260514-222508 的 model_10000 检查点逐张量完全一致。\n\n"
                    "策略敏感性测试固定真实 q、dq、gravity、gyro、command 和上一帧动作，"
                    "只替换线速度 x 与张力。它用于证明策略会对这些错配产生大幅动作变化，"
                    "不是实物闭环因果证明。"
                ),
            },
            {"id": "sensitivity_block", "type": "table", "tableId": "table_sensitivity", "layout": "full"},
            {
                "id": "limitations",
                "type": "markdown",
                "layout": "full",
                "body": (
                    "## 限制与稳健性\n\n"
                    "精确的 10:50 三次关节日志缺少 command、obs[0:70]、原始未限幅 action、"
                    "电流/扭矩、驱动故障码和控制周期抖动；中间一次日志为空。"
                    "所以不能仅凭这些 CSV 区分“策略没发抬腿”与“发了但因接触/摩擦没抬”。\n\n"
                    "不过，同日带命令日志直接显示后退动作已在策略输出侧收缩；观测 z-score 又把异常集中在"
                    "张力和重力 x；同时，两个独立日志源都显示 HFE target—actual 的同向偏差和幅值衰减。"
                    "因此“弱后退目标 + HFE 承重跟踪不足”这一两级解释，对合理阈值和汇总方式较稳健。"
                ),
            },
            {
                "id": "next_steps",
                "type": "markdown",
                "layout": "full",
                "body": (
                    "## 建议的最小排查顺序\n\n"
                    "1. **先验证 HFE 执行链**：机器人离地，对六个 HFE 做 ±0.1/±0.2 rad 阶跃和低频扫频；"
                    "随后在安全支撑承重下重复。记录目标角、实际角、相电流、母线电压、温度、fault 和限流标志。\n"
                    "2. 比较无负载与承重结果：无负载仍有约 0.11 rad 偏差优先查 offset/编码；"
                    "无负载正常而承重下垂则查实际闭环刚度、电流限幅、减速器摩擦和重力补偿。\n"
                    "3. **仍不落地行走**：静止 10 秒记录 velocity_source、obs[0:70]、归一化 z-score、"
                    "raw action 与 clipped action。校正 IMU→URDF 外参，并让速度补偿与策略重力使用同一旋转定义；"
                    "若没有可靠平移速度源，先做 obs[0:3]=0 的支撑/悬空 A/B，随后用本体估计器或重新训练无速度策略。\n"
                    "4. 对齐 obs[67] 的物理量、符号和单位；在支撑条件下先用训练均值 −3.82 做单变量 A/B，"
                    "不要直接在地面高速测试，也不要只做简单取反。\n"
                    "5. 冻结代码/配置/策略哈希并统一动作限幅。当前训练 leg_action_clip=null，"
                    "主脚本却是 ±2；如需限幅，应把同一限幅放回训练。\n"
                    "6. 以上通过后，才继续做 URDF 惯量、足地摩擦和执行器模型辨识。"
                ),
            },
            {
                "id": "questions",
                "type": "markdown",
                "layout": "full",
                "body": (
                    "## 仍需确认的问题\n\n"
                    "最关键的四个现场答案是：该次运行是否真的收到新鲜 INSGPS 速度；"
                    "机器人水平静止时 obs gravity 是否仍有约 ±0.42 的 x 分量；"
                    "张力传感器的 +N 与 MuJoCo tendon actuator force 的符号/标定关系；"
                    "以及 10:50 实验运行的确切 Git 提交和脚本版本。"
                ),
            },
        ],
    }
    snapshot = {
        "version": 1,
        "generatedAt": generated_at,
        "status": "ready",
        "datasets": {
            "headline_metrics": headline,
            "rear_response_ratio": rear_ratio,
            "hfe_backward_forward_chart": hfe_backward_forward_chart,
            "observation_ood_chart": ood_chart,
            "tracking_by_joint_type_chart": tracking_chart,
            "root_causes": root_causes,
            "policy_sensitivity": derived["policy_sensitivity"],
        },
    }
    return {
        "surface": "report",
        "manifest": manifest,
        "snapshot": snapshot,
        "sources": sources,
    }


def main() -> None:
    policy = torch.jit.load(str(POLICY_PATH), map_location="cpu").eval()
    detailed_paths = sorted(
        Path(path) for path in glob.glob(
            str(ROOT / "deploy/deploy_real/pre_data/hexapod_20260729_*.csv")
        )
    )
    detailed_runs = [
        (path.stem[len("hexapod_20260729_"):], read_csv_rows(path))
        for path in detailed_paths
    ]

    direction_response, direction_cells = aggregate_direction_response(detailed_runs)
    directional_hfe, directional_hfe_cells = directional_rear_hfe_tracking(
        detailed_runs
    )
    tracking_detail, tracking_by_type, run_quality = tracking_metrics()
    observation_matrix, observation_labels, gravity_checks = build_real_observation_matrix(
        detailed_runs, policy
    )
    ood = observation_ood(observation_matrix, observation_labels, policy)
    sensitivity = policy_sensitivity(detailed_runs, policy)
    checkpoint_match = checkpoint_matches_deployed_policy(policy)

    normalizer_mean = policy.normalizer.mean.detach().cpu().numpy()
    normalizer_std = policy.normalizer.std.detach().cpu().numpy()
    force_values = observation_matrix[:, 67]
    force_values = force_values[np.isfinite(force_values)]
    generated_at = datetime.now(timezone.utc).isoformat(timespec="seconds")
    derived = {
        "generated_at": generated_at,
        "data_quality": {
            "detailed_run_count": len(detailed_runs),
            "detailed_rows": int(sum(len(rows) for _, rows in detailed_runs)),
            "joint_log_runs": run_quality,
            "exact_0729_joint_logs_have_command_or_observation_columns": False,
        },
        "direction_response": direction_response,
        "direction_cells": direction_cells,
        "directional_rear_hfe_tracking": directional_hfe,
        "directional_rear_hfe_tracking_cells": directional_hfe_cells,
        "tracking_detail": tracking_detail,
        "tracking_by_joint_type": tracking_by_type,
        "observation_ood": ood,
        "gravity_acceleration_consistency": gravity_checks,
        "policy_sensitivity": sensitivity,
        "policy_checkpoint_match": checkpoint_match,
        "policy_normalizer_key_features": {
            "linvel_x": {"mean": float(normalizer_mean[0]), "std": float(normalizer_std[0])},
            "gravity_x": {"mean": float(normalizer_mean[6]), "std": float(normalizer_std[6])},
            "cable_force": {"mean": float(normalizer_mean[67]), "std": float(normalizer_std[67])},
        },
        "real_cable_force_n": {
            "mean": float(force_values.mean()),
            "p05": float(np.quantile(force_values, 0.05)),
            "median": float(np.median(force_values)),
            "p95": float(np.quantile(force_values, 0.95)),
            "max": float(force_values.max()),
        },
        "code_contract_findings": {
            "current_main_joint_action_clip": [-2.0, 2.0],
            "training_leg_action_clip": None,
            "training_and_deploy_kp": 80.0,
            "training_and_deploy_kd": 1.3,
            "training_and_deploy_action_scale": 0.25,
            "official_cpp_torque_range_nm": [-30.0, 30.0],
            "python_sdk_torque_range_nm": [-85.0, 85.0],
            "leg_feedforward_torque_command_nm": 0.0,
        },
    }
    write_json(OUT_DIR / "derived_metrics.json", derived)
    write_json(OUT_DIR / "artifact.json", build_artifact(derived, generated_at))
    print(json.dumps(
        {
            "ok": True,
            "derived_metrics": str(OUT_DIR / "derived_metrics.json"),
            "artifact": str(OUT_DIR / "artifact.json"),
            "detailed_rows": derived["data_quality"]["detailed_rows"],
            "backward_rear_action_ratio": (
                direction_response["back"]["action_abs_mean"]
                / direction_response["forward"]["action_abs_mean"]
            ),
            "cable_force_median_abs_z": next(
                row["median_abs_z"] for row in ood if row["feature"] == "cable_force"
            ),
            "gravity_x_median_abs_z": next(
                row["median_abs_z"] for row in ood if row["feature"] == "gravity_x"
            ),
        },
        ensure_ascii=False,
        indent=2,
    ))


if __name__ == "__main__":
    main()
