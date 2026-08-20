#!/usr/bin/env python3
"""Plot force, cmd_x, and motor-19 torque signals from a hexapod CSV log."""

from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path

# Use writable, persistent caches and a non-interactive backend so the script also
# works on the robot or over SSH.
cache_root = Path(tempfile.gettempdir()) / "hexapod-plot-cache"
cache_root.mkdir(parents=True, exist_ok=True)
os.environ.setdefault("XDG_CACHE_HOME", str(cache_root))
os.environ.setdefault("MPLCONFIGDIR", str(cache_root / "matplotlib"))

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd


REQUIRED_COLUMNS = (
    "timestamp_ns",
    "force_reference_n",
    "force_raw_n",
    "velocity_command_0",
    "torque_actual_nm",
    "torque_command_issued_nm",
)


def _elapsed_time_s(data: pd.DataFrame) -> pd.Series:
    """Return seconds elapsed from the first recorded sample."""
    timestamp_ns = pd.to_numeric(data["timestamp_ns"], errors="coerce")
    if timestamp_ns.isna().any():
        raise ValueError("column 'timestamp_ns' contains non-numeric values")
    if not timestamp_ns.is_monotonic_increasing:
        raise ValueError("column 'timestamp_ns' is not monotonically increasing")
    return (timestamp_ns - timestamp_ns.iloc[0]) / 1e9


def plot_csv(csv_path: Path, output_dir: Path | None = None) -> Path:
    """Create one three-panel PNG for *csv_path* and return its path."""
    csv_path = csv_path.expanduser().resolve()
    if not csv_path.is_file():
        raise FileNotFoundError(f"CSV file does not exist: {csv_path}")
    if csv_path.suffix.lower() != ".csv":
        raise ValueError(f"input must be a CSV file: {csv_path}")

    data = pd.read_csv(csv_path)
    missing = [column for column in REQUIRED_COLUMNS if column not in data.columns]
    if missing:
        raise ValueError(
            f"{csv_path.name} is missing required columns: {', '.join(missing)}"
        )
    if data.empty:
        raise ValueError(f"{csv_path.name} contains no data rows")

    numeric_columns = REQUIRED_COLUMNS[1:]
    numeric_data = data.loc[:, numeric_columns].apply(pd.to_numeric, errors="coerce")
    invalid_columns = [
        column for column in numeric_columns if numeric_data[column].isna().any()
    ]
    if invalid_columns:
        raise ValueError(
            f"{csv_path.name} contains non-numeric or missing values in: "
            f"{', '.join(invalid_columns)}"
        )

    time_s = _elapsed_time_s(data)
    fig, axes = plt.subplots(
        3,
        1,
        figsize=(18, 14),
        sharex=True,
        constrained_layout=True,
    )
    fig.suptitle(csv_path.name, fontsize=18, fontweight="bold")

    axes[0].plot(
        time_s,
        numeric_data["force_reference_n"],
        label="Desired force",
        color="tab:orange",
        linewidth=1.8,
    )
    axes[0].plot(
        time_s,
        numeric_data["force_raw_n"],
        label="Actual force",
        color="tab:blue",
        linewidth=1.2,
        alpha=0.9,
    )
    axes[0].set_title("Desired and actual cable tension")
    axes[0].set_ylabel("Force (N)")
    axes[0].legend(loc="best")

    axes[1].plot(
        time_s,
        numeric_data["velocity_command_0"],
        label="cmdx",
        color="tab:green",
        linewidth=1.4,
    )
    axes[1].set_title("cmdx")
    axes[1].set_ylabel("Command (m/s)")
    axes[1].legend(loc="best")

    axes[2].plot(
        time_s,
        numeric_data["torque_actual_nm"],
        label="Actual torque",
        color="tab:blue",
        linewidth=1.2,
    )
    axes[2].plot(
        time_s,
        numeric_data["torque_command_issued_nm"],
        label="Input torque",
        color="tab:red",
        linewidth=1.5,
        alpha=0.9,
    )
    axes[2].set_title("Motor 19 actual and input torque")
    axes[2].set_xlabel("Elapsed time (s)")
    axes[2].set_ylabel("Torque (N·m)")
    axes[2].legend(loc="best")

    for axis in axes:
        axis.grid(True, linestyle="--", linewidth=0.6, alpha=0.45)
        axis.margins(x=0)

    destination = (output_dir or csv_path.parent).expanduser().resolve()
    destination.mkdir(parents=True, exist_ok=True)
    output_path = destination / f"{csv_path.stem}_plot.png"
    fig.savefig(output_path, dpi=180, bbox_inches="tight")
    plt.close(fig)
    return output_path


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Generate a three-panel plot for one CSV, or for every non-merge CSV "
            "in pre_data."
        )
    )
    parser.add_argument(
        "csv_file",
        nargs="?",
        type=Path,
        help="CSV to plot (for example: hexapod_20260729_120321.csv)",
    )
    parser.add_argument(
        "--all",
        action="store_true",
        help="plot every CSV beside this script except files containing 'merge'",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        help="directory for generated PNG files (default: beside each CSV)",
    )
    args = parser.parse_args()

    if args.all and args.csv_file is not None:
        parser.error("csv_file and --all cannot be used together")
    if not args.all and args.csv_file is None:
        parser.error("provide csv_file or use --all")
    return args


def main() -> int:
    args = _parse_args()
    if args.all:
        data_dir = Path(__file__).resolve().parent
        csv_files = [
            path
            for path in sorted(data_dir.glob("*.csv"))
            if "merge" not in path.name.lower()
        ]
        if not csv_files:
            raise FileNotFoundError(f"no non-merge CSV files found in {data_dir}")
    else:
        csv_files = [args.csv_file]

    for csv_path in csv_files:
        output_path = plot_csv(csv_path, args.output_dir)
        print(f"Generated: {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
