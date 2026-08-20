#!/usr/bin/env python3
"""Materialize and verify the reviewed report rows in a small SQLite source.

The portable report contract requires durable SQL provenance for native cards,
charts, and tables.  This script stores the already reviewed, bounded snapshot
rows from artifact.json and executes the exact SQL recorded in the artifact.
Raw calculations remain reproducible through analyze_reaudit.py and
derived_metrics.json.
"""

from __future__ import annotations

import json
import sqlite3
from pathlib import Path


HERE = Path(__file__).resolve().parent
ARTIFACT = HERE / "artifact.json"
DATABASE = HERE / "report_source.sqlite"


def main() -> None:
    artifact = json.loads(ARTIFACT.read_text(encoding="utf-8"))
    datasets = artifact["snapshot"]["datasets"]
    source = next(
        item for item in artifact["manifest"]["sources"]
        if item["id"] == "src_derived"
    )
    sql = source["query"]["sql"]

    connection = sqlite3.connect(DATABASE)
    try:
        connection.execute("DROP TABLE IF EXISTS report_rows")
        connection.execute(
            "CREATE TABLE report_rows ("
            "dataset TEXT NOT NULL, row_order INTEGER NOT NULL, row_json TEXT NOT NULL, "
            "PRIMARY KEY (dataset, row_order))"
        )
        for dataset, rows in datasets.items():
            connection.executemany(
                "INSERT INTO report_rows(dataset, row_order, row_json) VALUES (?, ?, ?)",
                [
                    (
                        dataset,
                        row_order,
                        json.dumps(row, ensure_ascii=False, sort_keys=True),
                    )
                    for row_order, row in enumerate(rows)
                ],
            )
        connection.commit()

        reviewed = connection.execute(sql).fetchall()
        expected = sum(len(rows) for rows in datasets.values())
        if len(reviewed) != expected:
            raise RuntimeError(f"SQL returned {len(reviewed)} rows; expected {expected}")
    finally:
        connection.close()

    print(f"{DATABASE} ({expected} reviewed rows)")


if __name__ == "__main__":
    main()
