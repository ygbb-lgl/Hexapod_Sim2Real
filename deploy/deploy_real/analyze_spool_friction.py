#!/usr/bin/env python3
"""Estimate equivalent transmission loss from motor torque and gauge tension.

The quantity reported here is

    F_loss = actual_motor_torque / spool_radius - digital_gauge_tension

It includes all losses between the motor torque estimate and the digital gauge:
motor/gearbox/spool losses, effective-radius error, pulley bearing friction and
rope bending losses. It is therefore an equivalent system loss, not an isolated
measurement of one pulley bearing's friction.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-friction-analysis")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from fit_force_sensor_calibration import (
    CJK_FONT_FAMILY,
    DEFAULT_DATA_ROOT,
    RegressionSpec,
    fit_regression,
    load_aligned_samples,
    raw_linear_equation,
)


DEFAULT_SPOOL_RADIUS_M = 0.06


def grouped_cv_prediction(
    spec: RegressionSpec,
    tension_n: np.ndarray,
    loss_n: np.ndarray,
    groups: np.ndarray,
) -> np.ndarray:
    x = tension_n.reshape(-1, 1)
    predicted = np.full(loss_n.shape, np.nan, dtype=float)
    for held_out_group in np.unique(groups):
        test_mask = groups == held_out_group
        train_mask = ~test_mask
        fitted = fit_regression(spec, x[train_mask], loss_n[train_mask])
        predicted[test_mask] = fitted.predict(x[test_mask])
    if not np.all(np.isfinite(predicted)):
        raise RuntimeError("Non-finite grouped cross-validation prediction")
    return predicted


def write_samples_csv(
    samples: list[dict[str, object]],
    gauge_tension_n: np.ndarray,
    actual_torque_nm: np.ndarray,
    motor_side_tension_n: np.ndarray,
    equivalent_loss_n: np.ndarray,
    cv_prediction_n: np.ndarray,
    fitted_prediction_n: np.ndarray,
    spool_radius_m: float,
    output_path: Path,
) -> None:
    fieldnames = [
        "torque_folder",
        "pair_index",
        "run_finished_at",
        "raw_csv",
        "actual_torque_nm",
        "spool_radius_m",
        "motor_side_tension_n",
        "gauge_tension_n",
        "equivalent_loss_n",
        "cv_predicted_loss_n",
        "cv_error_n",
        "fitted_loss_n",
        "fit_error_n",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames)
        writer.writeheader()
        for index, sample in enumerate(samples):
            writer.writerow(
                {
                    "torque_folder": sample["torque_folder"],
                    "pair_index": sample["pair_index"],
                    "run_finished_at": sample["run_finished_at"],
                    "raw_csv": sample["raw_csv"],
                    "actual_torque_nm": f"{actual_torque_nm[index]:.9f}",
                    "spool_radius_m": f"{spool_radius_m:.9f}",
                    "motor_side_tension_n": f"{motor_side_tension_n[index]:.9f}",
                    "gauge_tension_n": f"{gauge_tension_n[index]:.9f}",
                    "equivalent_loss_n": f"{equivalent_loss_n[index]:.9f}",
                    "cv_predicted_loss_n": f"{cv_prediction_n[index]:.9f}",
                    "cv_error_n": f"{cv_prediction_n[index] - equivalent_loss_n[index]:.9f}",
                    "fitted_loss_n": f"{fitted_prediction_n[index]:.9f}",
                    "fit_error_n": f"{fitted_prediction_n[index] - equivalent_loss_n[index]:.9f}",
                }
            )


def make_plot(
    gauge_tension_n: np.ndarray,
    equivalent_loss_n: np.ndarray,
    intercept_n: float,
    slope: float,
    cv_mae_n: float,
    cv_rmse_n: float,
    cv_r2: float,
    output_path: Path,
) -> None:
    x_min = max(0.0, float(np.min(gauge_tension_n)) - 10.0)
    x_max = float(np.max(gauge_tension_n)) + 15.0
    x_grid = np.linspace(x_min, x_max, 400)
    y_grid = intercept_n + slope * x_grid

    plt.rcParams.update(
        {
            "font.family": CJK_FONT_FAMILY,
            "font.size": 11,
            "axes.titlesize": 14,
            "axes.labelsize": 12,
            "axes.unicode_minus": False,
        }
    )
    figure, axis = plt.subplots(figsize=(10.5, 6.8), constrained_layout=True)
    axis.scatter(
        gauge_tension_n,
        equivalent_loss_n,
        s=34,
        color="#356AA0",
        edgecolor="white",
        linewidth=0.45,
        alpha=0.82,
        label=f"标定数据（n={len(gauge_tension_n)}）",
        zorder=3,
    )
    axis.plot(
        x_grid,
        y_grid,
        color="#D08C27",
        linewidth=2.5,
        label="稳健线性拟合趋势",
        zorder=4,
    )
    axis.axhline(0.0, color="#555B66", linewidth=1.2, linestyle=":")
    axis.set_title("等效传力损失随绳索拉力增大")
    axis.set_xlabel("数显拉力计读数 T（N）")
    axis.set_ylabel("等效损失  F_loss = τ_actual / 0.06 − T（N）")
    axis.grid(True, color="#D7DBE2", linewidth=0.8, alpha=0.75)
    axis.legend(loc="upper left", frameon=False)
    axis.text(
        0.98,
        0.04,
        f"F_loss = {intercept_n:.2f} + {slope:.4f} T\n"
        f"分组交叉验证：平均绝对误差 {cv_mae_n:.2f} N，"
        f"均方根误差 {cv_rmse_n:.2f} N，R² {cv_r2:.3f}",
        transform=axis.transAxes,
        ha="right",
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": "#B9BEC7", "alpha": 0.94},
    )
    figure.savefig(output_path, dpi=180, facecolor="white")
    plt.close(figure)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--output-dir", type=Path, default=None)
    parser.add_argument(
        "--spool-radius-m",
        type=float,
        default=DEFAULT_SPOOL_RADIUS_M,
        help="Effective spool radius in metres (default: 0.06)",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.spool_radius_m <= 0.0:
        raise ValueError("--spool-radius-m must be positive")

    data_root = args.data_root.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else data_root / "fit_results"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    samples, alignment_notes = load_aligned_samples(data_root)
    gauge_tension_n = np.asarray(
        [float(sample["reference_tension_n"]) for sample in samples], dtype=float
    )
    actual_torque_nm = np.asarray(
        [float(sample["mean_actual_torque_nm"]) for sample in samples], dtype=float
    )
    groups = np.asarray([str(sample["torque_folder"]) for sample in samples])
    motor_side_tension_n = actual_torque_nm / float(args.spool_radius_m)
    equivalent_loss_n = motor_side_tension_n - gauge_tension_n

    spec = RegressionSpec(
        kind="huber",
        alpha=1e-4,
        huber_epsilon=1.5,
    )
    cv_prediction_n = grouped_cv_prediction(
        spec, gauge_tension_n, equivalent_loss_n, groups
    )
    fitted = fit_regression(
        spec,
        gauge_tension_n.reshape(-1, 1),
        equivalent_loss_n,
    )
    fitted_prediction_n = fitted.predict(gauge_tension_n.reshape(-1, 1))
    intercept_n, raw_coefficients = raw_linear_equation(fitted)
    slope = float(raw_coefficients[0])

    cv_residual = cv_prediction_n - equivalent_loss_n
    cv_mae_n = float(np.mean(np.abs(cv_residual)))
    cv_rmse_n = float(np.sqrt(np.mean(cv_residual * cv_residual)))
    denominator = float(
        np.sum((equivalent_loss_n - np.mean(equivalent_loss_n)) ** 2)
    )
    cv_r2 = 1.0 - float(np.sum(cv_residual * cv_residual)) / denominator
    cv_max_abs_error_n = float(np.max(np.abs(cv_prediction_n - equivalent_loss_n)))

    samples_csv = output_dir / "friction_vs_tension_samples.csv"
    write_samples_csv(
        samples,
        gauge_tension_n,
        actual_torque_nm,
        motor_side_tension_n,
        equivalent_loss_n,
        cv_prediction_n,
        fitted_prediction_n,
        float(args.spool_radius_m),
        samples_csv,
    )

    figure_path = output_dir / "friction_vs_tension.png"
    make_plot(
        gauge_tension_n,
        equivalent_loss_n,
        intercept_n,
        slope,
        cv_mae_n,
        cv_rmse_n,
        cv_r2,
        figure_path,
    )

    payload = {
        "definition": "equivalent_loss_n = actual_torque_nm / spool_radius_m - gauge_tension_n",
        "interpretation": (
            "Equivalent total transmission loss, including motor/gearbox/spool, "
            "effective-radius error, pulleys and rope bending"
        ),
        "spool_radius_m": float(args.spool_radius_m),
        "sample_count": len(samples),
        "torque_folder_count": int(len(np.unique(groups))),
        "model": "Huber robust linear regression",
        "equation": f"F_loss_N = {intercept_n:.9g} + {slope:.9g} * T_gauge_N",
        "intercept_n": intercept_n,
        "slope_loss_per_tension": slope,
        "cross_validation": {
            "method": "leave-one-torque-folder-out",
            "mae_n": cv_mae_n,
            "rmse_n": cv_rmse_n,
            "r2": cv_r2,
            "max_abs_error_n": cv_max_abs_error_n,
        },
        "observed_ranges": {
            "gauge_tension_n": [
                float(np.min(gauge_tension_n)),
                float(np.max(gauge_tension_n)),
            ],
            "equivalent_loss_n": [
                float(np.min(equivalent_loss_n)),
                float(np.max(equivalent_loss_n)),
            ],
        },
        "alignment_notes": alignment_notes,
    }
    model_json = output_dir / "friction_vs_tension_model.json"
    model_json.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    print(f"Aligned samples: {len(samples)} from {len(np.unique(groups))} folders")
    for note in alignment_notes:
        print(f"[alignment] {note}")
    print(payload["definition"])
    print(payload["equation"])
    print(
        f"Grouped CV: MAE={cv_mae_n:.3f} N, RMSE={cv_rmse_n:.3f} N, "
        f"R2={cv_r2:.5f}, max_abs_error={cv_max_abs_error_n:.3f} N"
    )
    print(f"Samples: {samples_csv}")
    print(f"Model: {model_json}")
    print(f"Figure: {figure_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
