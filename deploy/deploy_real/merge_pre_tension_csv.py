"""Merge independent pre-tension trajectory CSV files into one training CSV.

The input files must use the schema produced by
``deploy_real_hexapod_tethered_plot_torque_data.py``.  This script writes the
header exactly once, preserves every row's ``trajectory_id`` and
``timestamp_ns``, and never changes the force/torque timing columns.

Run from the repository root::

    python deploy/deploy_real/merge_pre_tension_csv.py \
        --input-dir deploy/deploy_real/pre_data \
        --output deploy/deploy_real/pre_data/merged_training_data_0729_model9500.csv

Optional file pattern::

    python deploy/deploy_real/merge_pre_tension_csv.py \
        --input-dir deploy/deploy_real/pre_data \
        --pattern "hexapod_*.csv" \
        --output deploy/deploy_real/pre_data/merged_training_data.csv

After merging, use ``pre_tension.py inspect-csv`` before training.  By default,
the output file itself is excluded from the input glob, so rerunning this script
does not recursively merge the previous merged file.
"""

from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path
from typing import Dict, List, Optional, Set


def merge_csv_files(input_dir: Path, output_path: Path, pattern: str) -> Dict[str, int]:
    if not input_dir.is_dir():
        raise ValueError(f"input directory does not exist: {input_dir}")

    output_resolved = output_path.resolve()
    input_files = sorted(
        path for path in input_dir.glob(pattern)
        if path.is_file() and path.resolve() != output_resolved
    )
    if not input_files:
        raise ValueError(f"no input CSV files matched {input_dir / pattern}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    temporary_path = output_path.with_suffix(output_path.suffix + ".tmp")
    expected_fields: Optional[List[str]] = None
    trajectory_sources: Dict[str, Path] = {}
    trajectory_ids: Set[str] = set()
    total_rows = 0

    try:
        with temporary_path.open("w", newline="") as output_file:
            writer = None
            for input_path in input_files:
                with input_path.open("r", newline="") as input_file:
                    reader = csv.DictReader(input_file)
                    fields = list(reader.fieldnames or [])
                    if not fields:
                        raise ValueError(f"CSV has no header: {input_path}")
                    if "trajectory_id" not in fields or "timestamp_ns" not in fields:
                        raise ValueError(
                            f"CSV must contain trajectory_id and timestamp_ns: {input_path}"
                        )
                    if expected_fields is None:
                        expected_fields = fields
                        writer = csv.DictWriter(output_file, fieldnames=expected_fields)
                        writer.writeheader()
                    elif fields != expected_fields:
                        raise ValueError(
                            f"CSV columns/order differ from the first file: {input_path}"
                        )

                    file_ids: Set[str] = set()
                    file_rows = 0
                    for line_number, row in enumerate(reader, 2):
                        trajectory_id = row["trajectory_id"].strip()
                        if not trajectory_id:
                            raise ValueError(
                                f"empty trajectory_id at {input_path}:{line_number}"
                            )
                        try:
                            int(float(row["timestamp_ns"]))
                        except (TypeError, ValueError) as exc:
                            raise ValueError(
                                f"invalid timestamp_ns at {input_path}:{line_number}"
                            ) from exc
                        file_ids.add(trajectory_id)
                        writer.writerow(row)
                        file_rows += 1

                    if file_rows == 0:
                        raise ValueError(f"CSV contains no data rows: {input_path}")
                    for trajectory_id in file_ids:
                        previous = trajectory_sources.get(trajectory_id)
                        if previous is not None and previous != input_path:
                            raise ValueError(
                                f"trajectory_id {trajectory_id!r} appears in both "
                                f"{previous} and {input_path}; use a unique ID per run"
                            )
                        trajectory_sources[trajectory_id] = input_path
                    trajectory_ids.update(file_ids)
                    total_rows += file_rows

        os.replace(temporary_path, output_path)
    except Exception:
        if temporary_path.exists():
            temporary_path.unlink()
        raise

    return {
        "files": len(input_files),
        "trajectories": len(trajectory_ids),
        "rows": total_rows,
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--pattern", default="*.csv")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    result = merge_csv_files(args.input_dir, args.output, args.pattern)
    print(
        f"merged {result['files']} files, {result['trajectories']} trajectories, "
        f"{result['rows']} rows -> {args.output}"
    )


if __name__ == "__main__":
    main()
