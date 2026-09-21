"""Synthetic contract tests; these are not real-vehicle validation."""
import csv
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


CLI = Path(__file__).resolve().parents[1] / "calibration" / "evaluate.py"


class CalibrationTest(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)

    def write_csv(self, name, rows, columns=("time_s", "speed_mps")):
        path = self.root / name
        with path.open("w", newline="") as stream:
            writer = csv.writer(stream)
            writer.writerow(columns)
            writer.writerows(rows)
        return path

    def evaluate(self, measured, simulated, *extra):
        self.report = self.root / "report.json"
        return subprocess.run([sys.executable, str(CLI), "evaluate",
                               "--measured", str(measured), "--simulated", str(simulated),
                               "--signals", "speed_mps", "--report", str(self.report),
                               *extra], capture_output=True, text=True)

    def test_different_sampling_and_explicit_clock_offset(self):
        measured = self.write_csv("measured.csv", [(0, 0), (1, 2), (2, 4)])
        simulated = self.write_csv("simulated.csv", [(10, 0), (10.5, 1), (11.5, 3), (12, 4)])
        result = self.evaluate(measured, simulated, "--sim-time-offset-s", "-10",
                               "--max-rmse", "speed_mps=0", "--max-error", "speed_mps=0")
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(self.report.read_text())
        self.assertEqual(report["metrics"]["speed_mps"]["rmse"], 0)
        self.assertEqual(report["coverage"]["duration_fraction"], 1)
        self.assertEqual(report["calibration_status"], "reference_unvalidated")

    def test_known_error_and_failed_threshold_produces_report(self):
        measured = self.write_csv("measured.csv", [(0, 0), (1, 2), (2, 4)])
        simulated = self.write_csv("simulated.csv", [(0, 1), (1, 3), (2, 5)])
        result = self.evaluate(measured, simulated, "--max-rmse", "speed_mps=0.9")
        self.assertEqual(result.returncode, 1, result.stderr)
        report = json.loads(self.report.read_text())
        self.assertFalse(report["passed"])
        self.assertEqual(report["metrics"]["speed_mps"]["rmse"], 1)
        self.assertEqual(report["metrics"]["speed_mps"]["max_abs_error"], 1)

    def test_partial_coverage_fails_by_default_and_can_be_explicitly_allowed(self):
        measured = self.write_csv("measured.csv", [(0, 0), (1, 2), (2, 4), (3, 6)])
        simulated = self.write_csv("simulated.csv", [(1, 2), (2, 4)])
        self.assertEqual(self.evaluate(measured, simulated).returncode, 1)
        report = json.loads(self.report.read_text())
        self.assertAlmostEqual(report["coverage"]["duration_fraction"], 1 / 3)
        self.assertEqual(report["coverage"]["compared_samples"], 2)
        self.assertEqual(self.evaluate(measured, simulated, "--min-coverage", "0.3").returncode, 0)

    def test_rejects_invalid_csv(self):
        measured = self.write_csv("measured.csv", [(0, 0), (1, 2)])
        for rows in [[(0, 1), (1, "nan")], [(0, 1), (1, "inf")],
                     [(0, 1), (1, "")], [(1, 1), (0, 2)], [(0, 1), (0, 2)],
                     [(0, 1)], [(0, 1), (1, 2, 3)]]:
            with self.subTest(rows=rows):
                simulated = self.write_csv("simulated.csv", rows)
                self.assertEqual(self.evaluate(measured, simulated).returncode, 2)

    def test_rejects_missing_or_duplicate_columns(self):
        measured = self.write_csv("measured.csv", [(0, 0), (1, 2)])
        for header in [("time_s", "unknown"), ("time_s", "time_s")]:
            with self.subTest(header=header):
                simulated = self.write_csv("simulated.csv", [(0, 1), (1, 2)], header)
                self.assertEqual(self.evaluate(measured, simulated).returncode, 2)

    def test_rejects_nonoverlap_and_invalid_thresholds(self):
        measured = self.write_csv("measured.csv", [(0, 0), (1, 2)])
        simulated = self.write_csv("simulated.csv", [(2, 1), (3, 2)])
        self.assertEqual(self.evaluate(measured, simulated).returncode, 2)
        for extra in [("--max-rmse", "speed_mps=nan"), ("--max-error", "speed_mps=-1"),
                      ("--min-coverage", "0"), ("--sim-time-offset-s", "inf")]:
            with self.subTest(extra=extra):
                self.assertEqual(self.evaluate(measured, measured, *extra).returncode, 2)

    def test_template_contains_units_and_no_fabricated_rows(self):
        result = subprocess.run([sys.executable, str(CLI), "template", "--signals",
                                 "speed_mps", "yaw_rate_radps"], capture_output=True, text=True)
        self.assertEqual(result.returncode, 0)
        self.assertEqual(result.stdout.strip(), "time_s,speed_mps,yaw_rate_radps")

    def test_report_cannot_overwrite_measurement(self):
        measured = self.write_csv("measured.csv", [(0, 0), (1, 2)])
        before = measured.read_bytes()
        result = self.evaluate(measured, measured, "--report", str(measured))
        self.assertEqual(result.returncode, 2)
        self.assertEqual(measured.read_bytes(), before)


if __name__ == "__main__":
    unittest.main()
