#!/usr/bin/env python3
"""Compare SI-unit vehicle logs without third-party dependencies."""
import argparse
import bisect
import csv
import hashlib
import json
import math
from pathlib import Path
import sys


SIGNALS = {
    "speed_mps": "m/s (signed body longitudinal velocity at CG)",
    "lateral_speed_mps": "m/s (body left velocity at CG)",
    "yaw_rate_radps": "rad/s (positive counterclockwise)",
    "steering_rad": "rad (equivalent road wheel angle, positive left)",
    "accel_x_mps2": "m/s^2 (CG physical acceleration, body forward)",
    "accel_y_mps2": "m/s^2 (CG physical acceleration, body left)",
}


def finite(value):
    number = float(value)
    if not math.isfinite(number):
        raise ValueError("values must be finite")
    return number


def load_csv(path, signals):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        header = next(reader, [])
        if not header or len(set(header)) != len(header):
            raise ValueError(f"{path}: missing or duplicate header")
        if not set(["time_s", *signals]).issubset(header):
            raise ValueError(f"{path}: required columns missing: time_s, {', '.join(signals)}")
        rows = []
        for line, fields in enumerate(reader, 2):
            if len(fields) != len(header):
                raise ValueError(f"{path}:{line}: wrong field count")
            try:
                # Reject missing/nonfinite values in every column, even unselected ones.
                row = dict(zip(header, map(finite, fields)))
            except ValueError as error:
                raise ValueError(f"{path}:{line}: invalid numeric value: {error}") from error
            if rows and row["time_s"] <= rows[-1]["time_s"]:
                raise ValueError(f"{path}:{line}: time_s must be strictly increasing")
            rows.append(row)
    if len(rows) < 2:
        raise ValueError(f"{path}: at least two samples required")
    return rows


def thresholds(items, signals):
    result = {}
    for item in items:
        name, separator, raw = item.partition("=")
        if not separator or name not in signals or name in result:
            raise ValueError(f"invalid or duplicate threshold: {item}")
        value = finite(raw)
        if value < 0:
            raise ValueError("thresholds must be nonnegative")
        result[name] = value
    return result


def evaluate(args):
    signals = args.signals
    if len(set(signals)) != len(signals):
        raise ValueError("signals must not be repeated")
    rmse_limits = thresholds(args.max_rmse, signals)
    error_limits = thresholds(args.max_error, signals)
    coverage_limit = finite(args.min_coverage)
    if not 0 < coverage_limit <= 1:
        raise ValueError("min-coverage must be in (0, 1]")
    offset = finite(args.sim_time_offset_s)
    measured = load_csv(args.measured, signals)
    simulated = load_csv(args.simulated, signals)
    times = [row["time_s"] + offset for row in simulated]
    if any(not math.isfinite(t) for t in times) or any(a >= b for a, b in zip(times, times[1:])):
        raise ValueError("shifted simulation timestamps must be finite and strictly increasing")
    start = max(measured[0]["time_s"], times[0])
    end = min(measured[-1]["time_s"], times[-1])
    selected = [row for row in measured if start <= row["time_s"] <= end]
    if end <= start or len(selected) < 2:
        raise ValueError("positive time overlap containing at least two measured samples required")
    errors = {name: [] for name in signals}
    for row in selected:
        t = row["time_s"]
        index = min(max(bisect.bisect_right(times, t) - 1, 0), len(times) - 2)
        weight = (t - times[index]) / (times[index + 1] - times[index])
        for name in signals:
            prediction = (1 - weight) * simulated[index][name] + weight * simulated[index + 1][name]
            errors[name].append(prediction - row[name])
    duration = measured[-1]["time_s"] - measured[0]["time_s"]
    coverage = (end - start) / duration
    violations = []
    if coverage < coverage_limit:
        violations.append("duration coverage below minimum")
    metrics = {}
    for name, values in errors.items():
        # hypot avoids overflowing intermediate squares for finite inputs.
        rmse = math.hypot(*values) / math.sqrt(len(values))
        maximum = max(map(abs, values))
        if not math.isfinite(rmse) or not math.isfinite(maximum):
            raise ValueError("nonfinite computed error; check numeric range")
        metrics[name] = {"unit": SIGNALS[name], "rmse": rmse, "max_abs_error": maximum}
        if name in rmse_limits and rmse > rmse_limits[name]:
            violations.append(f"{name}: RMSE exceeds threshold")
        if name in error_limits and maximum > error_limits[name]:
            violations.append(f"{name}: maximum absolute error exceeds threshold")
    def provenance(path):
        return {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
    return {
        "schema_version": 1,
        "calibration_status": "reference_unvalidated",
        "passed": not violations,
        "error_thresholds_configured": bool(rmse_limits or error_limits),
        "violations": violations,
        "inputs": {"measured": provenance(args.measured), "simulated": provenance(args.simulated)},
        "alignment": {"method": "linear simulation interpolation at measured timestamps; no extrapolation",
                      "sim_time_offset_s": offset, "error_weighting": "equal weight per measured sample"},
        "coverage": {"start_s": start, "end_s": end, "duration_s": end - start,
                     "duration_fraction": coverage, "measured_samples": len(measured),
                     "compared_samples": len(selected), "sample_fraction": len(selected) / len(measured),
                     "evaluated_start_s": selected[0]["time_s"], "evaluated_end_s": selected[-1]["time_s"],
                     "max_measured_gap_s": max(b["time_s"] - a["time_s"] for a, b in zip(measured, measured[1:])),
                     "max_simulated_gap_s": max(b - a for a, b in zip(times, times[1:]))},
        "thresholds": {"max_rmse": rmse_limits, "max_abs_error": error_limits, "min_duration_coverage": coverage_limit},
        "metrics": metrics,
    }


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    template = sub.add_parser("template", help="write header-only measurement template to stdout")
    template.add_argument("--signals", nargs="+", choices=list(SIGNALS), default=list(SIGNALS))
    compare = sub.add_parser("evaluate", help="compare measurement and simulation logs")
    compare.add_argument("--measured", type=Path, required=True)
    compare.add_argument("--simulated", type=Path, required=True)
    compare.add_argument("--signals", nargs="+", choices=list(SIGNALS), required=True)
    compare.add_argument("--sim-time-offset-s", type=float, default=0)
    compare.add_argument("--max-rmse", action="append", default=[], metavar="SIGNAL=VALUE")
    compare.add_argument("--max-error", action="append", default=[], metavar="SIGNAL=VALUE")
    compare.add_argument("--min-coverage", type=float, default=1.0)
    compare.add_argument("--report", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        if args.command == "template":
            if len(set(args.signals)) != len(args.signals):
                raise ValueError("signals must not be repeated")
            csv.writer(sys.stdout).writerow(["time_s", *args.signals])
            return 0
        if args.report.resolve() in (args.measured.resolve(), args.simulated.resolve()):
            raise ValueError("report must not overwrite an input CSV")
        report = evaluate(args)
        args.report.write_text(json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8")
        print("PASS" if report["passed"] else "FAIL")
        return 0 if report["passed"] else 1
    except (ValueError, OSError, csv.Error, OverflowError) as error:
        print(f"calibration: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
