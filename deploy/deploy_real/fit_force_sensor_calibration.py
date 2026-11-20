#!/usr/bin/env python3
"""Align force-sensor calibration runs and fit T = f(Fx, Fy, Fz).

The last non-empty, non-CSV row in every ``*/summary.csv`` is interpreted as
the manually entered digital-gauge readings, ordered from left to right in the
same order as the timestamped experiment rows above it.

Outputs are written to ``force_sensor_calibration_data/fit_results``:

* aligned_samples.csv          one row per paired calibration run
* model_comparison.csv         leave-one-torque-folder-out CV metrics
* force_sensor_model.json      deployable linear equation and diagnostics
* force_sensor_calibration_fit.png
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import re
from dataclasses import dataclass
from pathlib import Path

os.environ.setdefault("MPLCONFIGDIR", "/tmp/matplotlib-force-calibration")

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_DATA_ROOT = SCRIPT_DIR / "force_sensor_calibration_data"
CJK_FONT_PATH = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
if not CJK_FONT_PATH.exists():
    CJK_FONT_PATH = Path("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf")
if CJK_FONT_PATH.exists():
    font_manager.fontManager.addfont(str(CJK_FONT_PATH))
    CJK_FONT_FAMILY = font_manager.FontProperties(fname=str(CJK_FONT_PATH)).get_name()
else:
    CJK_FONT_FAMILY = "DejaVu Sans"
FEATURE_NAMES = ("mean_force_x_n", "mean_force_y_n", "mean_force_z_n")
TORQUE_FIELD_NAMES = ("mean_cmd_torque_nm", "mean_actual_torque_nm")
FLOAT_RE = re.compile(
    r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"
)
TIMESTAMP_RE = re.compile(r"20\d{2}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}")


def _float(value: str, *, field: str, source: Path) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{source}: invalid {field}={value!r}") from exc
    if not math.isfinite(result):
        raise ValueError(f"{source}: non-finite {field}={value!r}")
    return result


def _read_summary(summary_path: Path) -> tuple[list[dict[str, str]], list[float], list[str]]:
    """Read timestamped rows and the final whitespace-separated gauge row.

    A timestamp search intentionally repairs a known manual-edit pattern such
    as ``35.86N2026-...`` without changing the source file.
    """

    text = summary_path.read_text(encoding="utf-8-sig")
    lines = [line.rstrip("\r\n") for line in text.splitlines()]
    nonempty = [line.strip() for line in lines if line.strip()]
    if not nonempty:
        raise ValueError(f"{summary_path}: empty file")

    header = next(csv.reader([nonempty[0]]))
    rows: list[dict[str, str]] = []
    notes: list[str] = []

    for line_number, raw_line in enumerate(lines[1:], start=2):
        if ".csv" not in raw_line:
            continue
        timestamp_match = TIMESTAMP_RE.search(raw_line)
        if timestamp_match is None:
            notes.append(
                f"{summary_path}: line {line_number} contains .csv but no ISO timestamp; skipped"
            )
            continue
        if timestamp_match.start() > 0:
            prefix = raw_line[: timestamp_match.start()].strip()
            if prefix:
                notes.append(
                    f"{summary_path}: line {line_number} ignored manual prefix {prefix!r}"
                )
        cleaned = raw_line[timestamp_match.start() :].strip()
        values = next(csv.reader([cleaned]))
        if len(values) != len(header):
            notes.append(
                f"{summary_path}: line {line_number} has {len(values)} fields; "
                f"expected {len(header)}; skipped"
            )
            continue
        rows.append(dict(zip(header, values)))

    label_candidates = [
        line.strip()
        for line in lines[1:]
        if line.strip() and ".csv" not in line and TIMESTAMP_RE.search(line) is None
    ]
    if not label_candidates:
        raise ValueError(f"{summary_path}: no manual gauge-reading row found")
    label_line = label_candidates[-1]
    labels = [float(token) for token in FLOAT_RE.findall(label_line)]
    if not labels:
        raise ValueError(f"{summary_path}: no numeric gauge readings in final row")

    if len(rows) != len(labels):
        notes.append(
            f"{summary_path}: {len(rows)} experiment rows but {len(labels)} gauge readings; "
            f"paired the first {min(len(rows), len(labels))} by time and skipped the remainder"
        )
    return rows, labels, notes


def load_aligned_samples(data_root: Path) -> tuple[list[dict[str, object]], list[str]]:
    samples: list[dict[str, object]] = []
    notes: list[str] = []

    summary_paths = sorted(
        data_root.glob("*/summary.csv"),
        key=lambda path: float(path.parent.name.removesuffix("Nm").replace("p", ".")),
    )
    if not summary_paths:
        raise FileNotFoundError(f"No */summary.csv files under {data_root}")

    for summary_path in summary_paths:
        rows, labels, file_notes = _read_summary(summary_path)
        notes.extend(file_notes)
        paired_count = min(len(rows), len(labels))
        for index in range(paired_count):
            row = rows[index]
            sample: dict[str, object] = {
                "torque_folder": summary_path.parent.name,
                "pair_index": index + 1,
                "run_finished_at": row.get("run_finished_at", ""),
                "raw_csv": row.get("raw_csv", ""),
                "reference_tension_n": float(labels[index]),
            }
            for feature in FEATURE_NAMES:
                sample[feature] = _float(
                    row.get(feature, ""), field=feature, source=summary_path
                )
            for torque_field in TORQUE_FIELD_NAMES:
                sample[torque_field] = _float(
                    row.get(torque_field, ""),
                    field=torque_field,
                    source=summary_path,
                )
            samples.append(sample)

    if not samples:
        raise ValueError("No aligned calibration samples were recovered")
    return samples, notes


def write_aligned_csv(samples: list[dict[str, object]], output_path: Path) -> None:
    fieldnames = [
        "torque_folder",
        "pair_index",
        "run_finished_at",
        "raw_csv",
        *TORQUE_FIELD_NAMES,
        *FEATURE_NAMES,
        "reference_tension_n",
        "cv_prediction_n",
        "cv_error_n",
        "fit_prediction_n",
        "fit_error_n",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(samples)


@dataclass(frozen=True)
class RegressionSpec:
    kind: str
    alpha: float = 0.0
    degree: int = 1
    huber_epsilon: float = 1.5


@dataclass
class FittedRegression:
    spec: RegressionSpec
    feature_mean: np.ndarray
    feature_scale: np.ndarray
    intercept_scaled: float
    coefficients_scaled: np.ndarray

    def predict(self, x: np.ndarray) -> np.ndarray:
        expanded = _expand_features(np.asarray(x, dtype=float), self.spec.degree)
        scaled = (expanded - self.feature_mean) / self.feature_scale
        return self.intercept_scaled + scaled @ self.coefficients_scaled


def _expand_features(x: np.ndarray, degree: int) -> np.ndarray:
    if x.ndim != 2:
        raise ValueError("x must be a two-dimensional array")
    if degree == 1:
        return x
    if degree != 2:
        raise ValueError(f"Unsupported polynomial degree: {degree}")
    columns = [x[:, index] for index in range(x.shape[1])]
    columns.extend(
        x[:, left] * x[:, right]
        for left in range(x.shape[1])
        for right in range(left, x.shape[1])
    )
    return np.column_stack(columns)


def _solve_weighted_linear(
    x_scaled: np.ndarray,
    y: np.ndarray,
    *,
    alpha: float,
    weights: np.ndarray | None = None,
) -> np.ndarray:
    design = np.column_stack((np.ones(len(x_scaled)), x_scaled))
    if weights is None:
        weighted_design = design
        weighted_y = y
    else:
        sqrt_weights = np.sqrt(np.clip(weights, 1e-12, None))
        weighted_design = design * sqrt_weights[:, None]
        weighted_y = y * sqrt_weights
    penalty = np.eye(design.shape[1], dtype=float) * float(alpha)
    penalty[0, 0] = 0.0  # Never penalize the intercept.
    lhs = weighted_design.T @ weighted_design + penalty
    rhs = weighted_design.T @ weighted_y
    return np.linalg.pinv(lhs) @ rhs


def fit_regression(spec: RegressionSpec, x: np.ndarray, y: np.ndarray) -> FittedRegression:
    expanded = _expand_features(np.asarray(x, dtype=float), spec.degree)
    y = np.asarray(y, dtype=float)
    feature_mean = np.mean(expanded, axis=0)
    feature_scale = np.std(expanded, axis=0)
    feature_scale = np.where(feature_scale > 1e-12, feature_scale, 1.0)
    scaled = (expanded - feature_mean) / feature_scale

    if spec.kind in {"ols", "ridge"}:
        beta = _solve_weighted_linear(
            scaled,
            y,
            alpha=spec.alpha if spec.kind == "ridge" else 0.0,
        )
    elif spec.kind == "huber":
        beta = _solve_weighted_linear(scaled, y, alpha=spec.alpha)
        design = np.column_stack((np.ones(len(scaled)), scaled))
        for _ in range(200):
            residual = y - design @ beta
            median = float(np.median(residual))
            mad = float(np.median(np.abs(residual - median)))
            robust_scale = 1.4826 * mad
            if robust_scale < 1e-9:
                robust_scale = max(float(np.sqrt(np.mean(residual * residual))), 1e-9)
            threshold = float(spec.huber_epsilon) * robust_scale
            abs_residual = np.abs(residual)
            weights = np.ones_like(abs_residual)
            outside = abs_residual > threshold
            weights[outside] = threshold / abs_residual[outside]
            updated = _solve_weighted_linear(
                scaled,
                y,
                alpha=spec.alpha,
                weights=weights,
            )
            if np.linalg.norm(updated - beta) <= 1e-10 * (1.0 + np.linalg.norm(beta)):
                beta = updated
                break
            beta = updated
    else:
        raise ValueError(f"Unsupported regression kind: {spec.kind}")

    return FittedRegression(
        spec=spec,
        feature_mean=feature_mean,
        feature_scale=feature_scale,
        intercept_scaled=float(beta[0]),
        coefficients_scaled=np.asarray(beta[1:], dtype=float),
    )


def _mae(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.mean(np.abs(y_pred - y_true)))


def _rmse(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    return float(np.sqrt(np.mean((y_pred - y_true) ** 2)))


def _r2(y_true: np.ndarray, y_pred: np.ndarray) -> float:
    denominator = float(np.sum((y_true - np.mean(y_true)) ** 2))
    if denominator <= 0.0:
        return float("nan")
    return 1.0 - float(np.sum((y_pred - y_true) ** 2)) / denominator


def build_candidates() -> dict[str, RegressionSpec]:
    candidates: dict[str, RegressionSpec] = {
        "xyz_ols": RegressionSpec(kind="ols"),
        "xyz_huber": RegressionSpec(kind="huber", alpha=1e-4, huber_epsilon=1.5),
    }
    for alpha in (0.01, 0.1, 1.0, 10.0, 100.0):
        candidates[f"xyz_ridge_alpha_{alpha:g}"] = RegressionSpec(
            kind="ridge", alpha=alpha
        )
    for alpha in (0.1, 1.0, 10.0, 100.0):
        candidates[f"xyz_poly2_ridge_alpha_{alpha:g}"] = RegressionSpec(
            kind="ridge", alpha=alpha, degree=2
        )
    return candidates


def cross_validate_candidates(
    candidates: dict[str, RegressionSpec],
    x: np.ndarray,
    y: np.ndarray,
    groups: np.ndarray,
) -> tuple[list[dict[str, object]], dict[str, np.ndarray]]:
    comparison: list[dict[str, object]] = []
    predictions: dict[str, np.ndarray] = {}

    unique_groups = np.unique(groups)
    for name, spec in candidates.items():
        predicted = np.full(y.shape, np.nan, dtype=float)
        for held_out_group in unique_groups:
            test_mask = groups == held_out_group
            train_mask = ~test_mask
            fitted = fit_regression(spec, x[train_mask], y[train_mask])
            predicted[test_mask] = fitted.predict(x[test_mask])
        if not np.all(np.isfinite(predicted)):
            raise RuntimeError(f"Non-finite cross-validation prediction from {name}")
        predictions[name] = predicted
        comparison.append(
            {
                "model": name,
                "cv_mae_n": _mae(y, predicted),
                "cv_rmse_n": _rmse(y, predicted),
                "cv_r2": _r2(y, predicted),
                "cv_max_abs_error_n": float(np.max(np.abs(predicted - y))),
                "deployment_candidate": not name.startswith("xyz_poly2"),
            }
        )
    comparison.sort(key=lambda row: (float(row["cv_mae_n"]), float(row["cv_rmse_n"])))
    return comparison, predictions


def choose_deployment_model(comparison: list[dict[str, object]]) -> str:
    """Choose a robust linear model while keeping polynomials diagnostic only.

    MAE differences below 0.1 N are immaterial for these manually read labels.
    Among models in that near-best set, prefer the lower RMSE so a handful of
    larger errors cannot be hidden by a tiny average-error advantage.
    """

    eligible = [row for row in comparison if bool(row["deployment_candidate"])]
    if not eligible:
        raise RuntimeError("No extrapolation-safe deployment candidates")
    best_mae = min(float(row["cv_mae_n"]) for row in eligible)
    near_best = [row for row in eligible if float(row["cv_mae_n"]) <= best_mae + 0.1]
    near_best.sort(
        key=lambda row: (float(row["cv_rmse_n"]), float(row["cv_mae_n"]))
    )
    return str(near_best[0]["model"])


def raw_linear_equation(fitted: FittedRegression) -> tuple[float, np.ndarray]:
    if fitted.spec.degree != 1:
        raise ValueError("A polynomial model cannot be converted to a 3-term equation")
    raw_coefficients = fitted.coefficients_scaled / fitted.feature_scale
    raw_intercept = fitted.intercept_scaled - float(
        np.dot(
            fitted.coefficients_scaled,
            fitted.feature_mean / fitted.feature_scale,
        )
    )
    return raw_intercept, raw_coefficients


def equation_text(intercept: float, coefficients: np.ndarray) -> str:
    terms = [f"{intercept:.9g}"]
    for coefficient, symbol in zip(coefficients, ("Fx", "Fy", "Fz")):
        sign = "+" if coefficient >= 0.0 else "-"
        terms.append(f" {sign} {abs(float(coefficient)):.9g}*{symbol}")
    return "T_N = " + "".join(terms)


def write_comparison_csv(
    comparison: list[dict[str, object]], selected_model: str, output_path: Path
) -> None:
    fieldnames = [
        "model",
        "selected",
        "deployment_candidate",
        "cv_mae_n",
        "cv_rmse_n",
        "cv_r2",
        "cv_max_abs_error_n",
    ]
    with output_path.open("w", newline="", encoding="utf-8") as output_file:
        writer = csv.DictWriter(output_file, fieldnames=fieldnames)
        writer.writeheader()
        for row in comparison:
            writer.writerow({**row, "selected": str(row["model"] == selected_model).lower()})


def make_plot(
    x: np.ndarray,
    y: np.ndarray,
    cv_prediction: np.ndarray,
    intercept: float,
    coefficients: np.ndarray,
    cv_metrics: dict[str, object],
    output_path: Path,
) -> dict[str, float]:
    force_norm = np.linalg.norm(x, axis=1)
    valid = force_norm > 1e-9
    unit_directions = x[valid] / force_norm[valid, None]
    representative_direction = np.median(unit_directions, axis=0)
    representative_direction /= np.linalg.norm(representative_direction)

    representative_slope = float(np.dot(coefficients, representative_direction))
    if representative_slope <= 0.0:
        raise RuntimeError(
            "Selected equation is not increasing along the median measured force direction"
        )

    observed_norm_min = float(np.min(force_norm))
    observed_norm_max = float(np.max(force_norm))
    observed_tension_min = float(np.min(y))
    observed_tension_max = float(np.max(y))
    norm_at_400_n = float((400.0 - intercept) / representative_slope)
    curve_max = max(observed_norm_max * 1.08, norm_at_400_n * 1.03)
    curve_min = max(0.0, min(observed_norm_min * 0.8, (0.0 - intercept) / representative_slope))
    norm_grid = np.linspace(curve_min, curve_max, 500)
    xyz_grid = norm_grid[:, None] * representative_direction[None, :]
    tension_grid = intercept + xyz_grid @ coefficients

    within_calibration = norm_grid <= observed_norm_max
    beyond_calibration = norm_grid >= observed_norm_max

    plt.rcParams.update(
        {
            "font.family": CJK_FONT_FAMILY,
            "font.size": 11,
            "axes.titlesize": 13,
            "axes.labelsize": 11,
            "axes.unicode_minus": False,
        }
    )
    figure, axes = plt.subplots(2, 1, figsize=(10.5, 10.0), constrained_layout=True)

    top = axes[0]
    top.scatter(
        force_norm,
        y,
        s=30,
        color="#356AA0",
        edgecolor="white",
        linewidth=0.45,
        alpha=0.82,
        label=f"标定数据（n={len(y)}）",
        zorder=3,
    )
    top.plot(
        norm_grid[within_calibration],
        tension_grid[within_calibration],
        color="#D08C27",
        linewidth=2.4,
        label="中位受力方向上的XYZ拟合模型",
        zorder=4,
    )
    top.plot(
        norm_grid[beyond_calibration],
        tension_grid[beyond_calibration],
        color="#D08C27",
        linewidth=2.4,
        linestyle="--",
        label="向400 N区域外推",
        zorder=4,
    )
    top.axvline(
        observed_norm_max,
        color="#555B66",
        linestyle=":",
        linewidth=1.5,
        label=f"实测上限（最大标签 {observed_tension_max:.1f} N）",
    )
    top.axhspan(200.0, 400.0, color="#D08C27", alpha=0.06, zorder=0)
    top.set_title("力传感器标定及200–400 N趋势")
    top.set_xlabel("力传感器XYZ合力  ||Fxyz||（N）")
    top.set_ylabel("数显拉力计读数 T（N）")
    top.set_ylim(bottom=min(0.0, observed_tension_min - 15.0), top=420.0)
    top.grid(True, color="#D7DBE2", linewidth=0.8, alpha=0.75)
    top.legend(loc="upper left", frameon=False, fontsize=9)

    bottom = axes[1]
    bottom.scatter(
        y,
        cv_prediction,
        s=34,
        color="#356AA0",
        edgecolor="white",
        linewidth=0.45,
        alpha=0.84,
        label="按力矩文件夹留一的预测结果",
        zorder=3,
    )
    limits = [
        min(float(np.min(y)), float(np.min(cv_prediction))) - 10.0,
        max(float(np.max(y)), float(np.max(cv_prediction))) + 10.0,
    ]
    bottom.plot(
        limits,
        limits,
        color="#444A55",
        linewidth=1.6,
        linestyle="--",
        label="理想1:1直线",
    )
    bottom.set_xlim(limits)
    bottom.set_ylim(limits)
    bottom.set_aspect("equal", adjustable="box")
    bottom.set_title("分组交叉验证")
    bottom.set_xlabel("数显拉力计实测值（N）")
    bottom.set_ylabel("交叉验证预测值（N）")
    bottom.grid(True, color="#D7DBE2", linewidth=0.8, alpha=0.75)
    bottom.legend(loc="upper left", frameon=False, fontsize=9)
    bottom.text(
        0.98,
        0.04,
        f"平均绝对误差 = {float(cv_metrics['cv_mae_n']):.2f} N\n"
        f"均方根误差 = {float(cv_metrics['cv_rmse_n']):.2f} N\n"
        f"R² = {float(cv_metrics['cv_r2']):.3f}",
        transform=bottom.transAxes,
        ha="right",
        va="bottom",
        bbox={"facecolor": "white", "edgecolor": "#B9BEC7", "alpha": 0.92},
    )

    figure.suptitle(
        "XYZ力传感器 → 绳索拉力",
        fontsize=16,
        fontweight="bold",
    )
    figure.savefig(output_path, dpi=180, facecolor="white")
    plt.close(figure)

    return {
        "observed_force_norm_min_n": observed_norm_min,
        "observed_force_norm_max_n": observed_norm_max,
        "observed_tension_min_n": observed_tension_min,
        "observed_tension_max_n": observed_tension_max,
        "representative_direction_fx": float(representative_direction[0]),
        "representative_direction_fy": float(representative_direction[1]),
        "representative_direction_fz": float(representative_direction[2]),
        "representative_tension_per_force_norm": representative_slope,
        "representative_force_norm_at_400_n": norm_at_400_n,
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-root", type=Path, default=DEFAULT_DATA_ROOT)
    parser.add_argument("--output-dir", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    data_root = args.data_root.resolve()
    output_dir = (
        args.output_dir.resolve()
        if args.output_dir is not None
        else data_root / "fit_results"
    )
    output_dir.mkdir(parents=True, exist_ok=True)

    samples, notes = load_aligned_samples(data_root)
    aligned_csv = output_dir / "aligned_samples.csv"

    x = np.asarray(
        [[float(sample[name]) for name in FEATURE_NAMES] for sample in samples],
        dtype=float,
    )
    y = np.asarray([float(sample["reference_tension_n"]) for sample in samples])
    groups = np.asarray([str(sample["torque_folder"]) for sample in samples])

    candidates = build_candidates()
    comparison, cv_predictions = cross_validate_candidates(candidates, x, y, groups)
    selected_model = choose_deployment_model(comparison)
    fitted = fit_regression(candidates[selected_model], x, y)
    intercept, coefficients = raw_linear_equation(fitted)
    equation = equation_text(intercept, coefficients)

    fitted_prediction = fitted.predict(x)
    selected_cv_prediction = cv_predictions[selected_model]
    for index, sample in enumerate(samples):
        sample["cv_prediction_n"] = float(selected_cv_prediction[index])
        sample["cv_error_n"] = float(selected_cv_prediction[index] - y[index])
        sample["fit_prediction_n"] = float(fitted_prediction[index])
        sample["fit_error_n"] = float(fitted_prediction[index] - y[index])
    write_aligned_csv(samples, aligned_csv)

    comparison_csv = output_dir / "model_comparison.csv"
    write_comparison_csv(comparison, selected_model, comparison_csv)
    selected_metrics = next(row for row in comparison if row["model"] == selected_model)

    figure_path = output_dir / "force_sensor_calibration_fit.png"
    curve_details = make_plot(
        x,
        y,
        selected_cv_prediction,
        intercept,
        coefficients,
        selected_metrics,
        figure_path,
    )

    model_payload = {
        "model_type": "linear_xyz",
        "selected_candidate": selected_model,
        "selection_rule": (
            "Among extrapolation-safe linear candidates within 0.1 N of the best "
            "grouped-CV MAE, choose the lowest grouped-CV RMSE"
        ),
        "equation": equation,
        "input_order": ["force_x_n", "force_y_n", "force_z_n"],
        "intercept_n": intercept,
        "coefficients": {
            "force_x_n": float(coefficients[0]),
            "force_y_n": float(coefficients[1]),
            "force_z_n": float(coefficients[2]),
        },
        "sample_count": len(samples),
        "torque_folder_count": int(len(np.unique(groups))),
        "cross_validation": {
            "method": "leave-one-torque-folder-out",
            "mae_n": float(selected_metrics["cv_mae_n"]),
            "rmse_n": float(selected_metrics["cv_rmse_n"]),
            "r2": float(selected_metrics["cv_r2"]),
            "max_abs_error_n": float(selected_metrics["cv_max_abs_error_n"]),
        },
        "curve_projection": curve_details,
        "alignment_notes": notes,
        "source_root": str(data_root),
    }
    model_json = output_dir / "force_sensor_model.json"
    model_json.write_text(
        json.dumps(model_payload, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )

    print(f"Aligned samples: {len(samples)} from {len(np.unique(groups))} folders")
    for note in notes:
        print(f"[alignment] {note}")
    print(f"Selected model: {selected_model}")
    print(equation)
    print(
        "Grouped CV: "
        f"MAE={float(selected_metrics['cv_mae_n']):.3f} N, "
        f"RMSE={float(selected_metrics['cv_rmse_n']):.3f} N, "
        f"R2={float(selected_metrics['cv_r2']):.5f}"
    )
    print(f"Aligned CSV: {aligned_csv}")
    print(f"Model comparison: {comparison_csv}")
    print(f"Model JSON: {model_json}")
    print(f"Figure: {figure_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
