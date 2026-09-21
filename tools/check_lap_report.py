#!/usr/bin/env python3
"""Validate the observable completion contract of the ten-lap self-test."""
import argparse
import json
import math
from pathlib import Path


def validate(report):
    if report.get("mode") != "ground_truth_reference_self_test" or report.get("outcome") != "complete":
        raise ValueError("run did not complete the reference self-test")
    if report.get("target_laps") != 10 or report.get("completed_laps") != 10:
        raise ValueError("expected exactly ten completed laps")
    for key in ("final_speed_mps", "max_abs_cte_m", "min_clearance_m", "simulation_time_s"):
        value = report.get(key)
        if not isinstance(value, (float, int)) or not math.isfinite(value):
            raise ValueError(f"missing/non-finite metric: {key}")
    if abs(report["final_speed_mps"]) >= .02:
        raise ValueError("vehicle has not stopped")
    if report["min_clearance_m"] < 0 or report["max_abs_cte_m"] > .5:
        raise ValueError("track-clearance or tracking-error acceptance failed")
    if report["simulation_time_s"] <= 0:
        raise ValueError("simulation time did not advance")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    args = parser.parse_args()
    try:
        report = json.loads(args.report.read_text(encoding="utf-8"))
        validate(report)
    except (OSError, ValueError, TypeError) as error:
        parser.exit(1, f"Ten-lap acceptance failed: {error}\n")
    print(f"PASS: 10 laps, stopped; max tracking error {report['max_abs_cte_m']:.3f} m; "
          f"minimum conservative clearance {report['min_clearance_m']:.3f} m")


if __name__ == "__main__":
    main()
