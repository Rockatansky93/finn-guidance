#!/usr/bin/env python3
"""Upload FINN Guidance field-run summaries to FINN Core.

The command records metadata and local file URIs in Core. It does not move the
full SQLite database or telemetry log; those stay on the field laptop unless a
later sync process copies them.
"""

from __future__ import annotations

import argparse
import json
import os
import sqlite3
import sys
import urllib.error
import urllib.request
from pathlib import Path
from typing import Any


def post_json(core_url: str, path: str, payload: dict[str, Any]) -> dict[str, Any]:
    body = json.dumps(payload).encode("utf-8")
    request = urllib.request.Request(
        core_url.rstrip("/") + path,
        data=body,
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return json.loads(response.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        detail = exc.read().decode("utf-8", errors="replace")
        raise RuntimeError(f"{exc.code} {exc.reason}: {detail}") from exc


def path_uri(path: Path) -> str:
    return path.resolve().as_uri()


def latest_telemetry_log() -> Path | None:
    candidates = []
    for log_dir in (Path("logs"), Path("pc/logs"), Path("target/debug/logs")):
        if log_dir.exists():
            candidates.extend(log_dir.glob("steer_*.jsonl"))
    if not candidates:
        return None
    return max(candidates, key=lambda p: p.stat().st_mtime)


def summarize_telemetry(path: Path) -> dict[str, Any]:
    counts: dict[str, int] = {}
    header: dict[str, Any] | None = None
    summary_windows = 0
    max_xte_abs_m = 0.0
    max_loop_us = 0
    total_iterations = 0
    bad_lines = 0

    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            try:
                record = json.loads(line)
            except json.JSONDecodeError:
                bad_lines += 1
                continue

            record_type = record.get("type", "unknown")
            counts[record_type] = counts.get(record_type, 0) + 1
            if record_type == "header" and header is None:
                header = record
            elif record_type == "summary":
                summary_windows += 1
                max_xte_abs_m = max(max_xte_abs_m, abs(float(record.get("max_xte_abs_m", 0) or 0)))
                max_loop_us = max(max_loop_us, int(record.get("max_loop_us", 0) or 0))
                total_iterations += int(record.get("iterations", 0) or 0)

    return {
        "path": str(path),
        "record_counts": counts,
        "bad_lines": bad_lines,
        "summary_windows": summary_windows,
        "max_xte_abs_m": max_xte_abs_m,
        "max_loop_us": max_loop_us,
        "total_iterations": total_iterations,
        "header": header or {},
    }


def summarize_coverage(db_path: Path, job_id: int | None) -> dict[str, Any]:
    conn = sqlite3.connect(db_path)
    conn.row_factory = sqlite3.Row
    try:
        if job_id is None:
            row = conn.execute(
                "SELECT * FROM jobs ORDER BY started_at DESC LIMIT 1"
            ).fetchone()
        else:
            row = conn.execute("SELECT * FROM jobs WHERE id = ?", (job_id,)).fetchone()
        if row is None:
            raise RuntimeError("no coverage job found")

        job = dict(row)
        point_stats = dict(conn.execute(
            """
            SELECT
                COUNT(*) AS point_count,
                COUNT(DISTINCT segment) AS segment_count,
                MIN(timestamp_ms) AS first_timestamp_ms,
                MAX(timestamp_ms) AS last_timestamp_ms,
                MIN(latitude) AS min_latitude,
                MAX(latitude) AS max_latitude,
                MIN(longitude) AS min_longitude,
                MAX(longitude) AS max_longitude,
                AVG(speed) AS mean_speed
            FROM coverage_points
            WHERE job_id = ?
            """,
            (job["id"],),
        ).fetchone())
        qualities = {
            quality: count
            for quality, count in conn.execute(
                """
                SELECT fix_quality, COUNT(*)
                FROM coverage_points
                WHERE job_id = ?
                GROUP BY fix_quality
                """,
                (job["id"],),
            )
        }
        return {
            "db_path": str(db_path),
            "job": job,
            "point_stats": point_stats,
            "fix_quality_counts": qualities,
        }
    finally:
        conn.close()


def create_field_run(args: argparse.Namespace) -> str:
    response = post_json(args.core_url, "/field-runs", {
        "source_node_id": args.node_id,
        "run_type": "guidance",
        "machine_id": args.machine_id,
        "implement_id": args.implement_id,
        "field_name": args.field_name,
        "metadata": {
            "project": "finn-guidance",
            "coverage_source": args.coverage_source,
        },
    })
    return response["field_run_id"]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--core-url", default=os.getenv("FINN_CORE_URL", "http://127.0.0.1:8000"))
    parser.add_argument("--field-run-id")
    parser.add_argument("--node-id", default="tractor-guidance")
    parser.add_argument("--machine-id", default="tractor-main")
    parser.add_argument("--implement-id")
    parser.add_argument("--field-name")
    parser.add_argument("--coverage-db", default="data/coverage.db")
    parser.add_argument("--coverage-job-id", type=int)
    parser.add_argument("--coverage-source", default="guidance_manual")
    parser.add_argument("--telemetry-log")
    parser.add_argument("--no-telemetry", action="store_true")
    parser.add_argument("--no-coverage", action="store_true")
    parser.add_argument("--no-analysis-task", action="store_true")
    args = parser.parse_args()

    field_run_id = args.field_run_id or create_field_run(args)
    result: dict[str, Any] = {"field_run_id": field_run_id, "uploads": []}
    create_analysis_task = not args.no_analysis_task

    telemetry_path = Path(args.telemetry_log) if args.telemetry_log else latest_telemetry_log()
    if not args.no_telemetry and telemetry_path and telemetry_path.exists():
        response = post_json(args.core_url, f"/field-runs/{field_run_id}/telemetry", {
            "source_node_id": args.node_id,
            "telemetry_type": "steering_log",
            "content_uri": path_uri(telemetry_path),
            "summary": summarize_telemetry(telemetry_path),
            "metadata": {"project": "finn-guidance"},
            "create_analysis_task": create_analysis_task,
        })
        result["uploads"].append({"kind": "telemetry", **response})

    coverage_db = Path(args.coverage_db)
    if not args.no_coverage and coverage_db.exists():
        response = post_json(args.core_url, f"/field-runs/{field_run_id}/coverage", {
            "source_node_id": args.node_id,
            "coverage_source": args.coverage_source,
            "content_uri": path_uri(coverage_db),
            "summary": summarize_coverage(coverage_db, args.coverage_job_id),
            "metadata": {"project": "finn-guidance"},
            "create_analysis_task": create_analysis_task,
        })
        result["uploads"].append({"kind": "coverage", **response})

    print(json.dumps(result, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"upload failed: {exc}", file=sys.stderr)
        raise SystemExit(1)
