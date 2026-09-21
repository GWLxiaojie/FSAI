import importlib.util
from pathlib import Path
import unittest

spec = importlib.util.spec_from_file_location("lap_report", Path(__file__).resolve().parents[1] / "check_lap_report.py")
lap_report = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lap_report)


class LapReportTest(unittest.TestCase):
    def result(self):
        return dict(mode="ground_truth_reference_self_test", outcome="complete", target_laps=10,
                    completed_laps=10, final_speed_mps=0.0, max_abs_cte_m=.11,
                    min_clearance_m=.69, simulation_time_s=2938.0)

    def test_accepts_ten_laps_and_full_stop(self):
        lap_report.validate(self.result())

    def test_incomplete_failed_moving_or_outside_runs_cannot_pass(self):
        for key, value in (("completed_laps", 9), ("target_laps", 1), ("outcome", "failed"),
                           ("final_speed_mps", .03), ("min_clearance_m", -.01),
                           ("max_abs_cte_m", .6), ("simulation_time_s", 0)):
            report = self.result()
            report[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                lap_report.validate(report)

    def test_missing_nonfinite_and_unmeasured_metrics_cannot_pass(self):
        for value in (None, float("nan"), float("inf")):
            report = self.result()
            report["min_clearance_m"] = value
            with self.assertRaises(ValueError):
                lap_report.validate(report)
        with self.assertRaises(ValueError):
            lap_report.validate({})
