import importlib.util
import contextlib
import io
from pathlib import Path
import unittest


PATH = Path(__file__).resolve().parents[1] / "drive_command.py"


class DriveCommandTest(unittest.TestCase):
    def test_defaults_match_ros_float_field_types(self):
        args = self.load().parse_args([])
        for name in ("torque", "brake", "steering"):
            self.assertIsInstance(getattr(args, name), float)

    def load(self):
        spec = importlib.util.spec_from_file_location("drive_command", PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module

    def test_simulated_duration_and_final_stop(self):
        module = self.load()
        runtime = FakeRuntime([0, 50_000_000, 100_000_000, 150_000_000, 200_000_000])
        args = module.parse_args(["--torque", "10", "--duration", "0.2", "--rate", "10"])
        module.run_session(runtime, args, sequence_start=40)
        self.assertEqual([cmd["sequence"] for cmd in runtime.commands], [40, 41, 42])
        self.assertEqual(runtime.commands[0]["stamp_ns"], 0)
        self.assertEqual(runtime.commands[1]["rear_axle_torque_nm"], 10)
        self.assertEqual(runtime.commands[-1]["rear_axle_torque_nm"], 0)
        self.assertEqual(runtime.commands[-1]["friction_brake_ratio"], 1)

    def test_missing_clock_times_out_without_publishing(self):
        module = self.load()
        runtime = FakeRuntime([])
        args = module.parse_args(["--clock-timeout", "0.1"])
        with self.assertRaisesRegex(RuntimeError, "waiting for /clock"):
            module.run_session(runtime, args)
        self.assertEqual(runtime.commands, [])

    def test_clock_regression_stops_and_never_resumes_drive(self):
        module = self.load()
        runtime = FakeRuntime([1_000_000_000, 1_100_000_000, 0, 200_000_000])
        args = module.parse_args(["--torque", "20"])
        with self.assertRaisesRegex(RuntimeError, "backwards"):
            module.run_session(runtime, args, sequence_start=10)
        self.assertEqual(runtime.commands[-1]["rear_axle_torque_nm"], 0)
        self.assertEqual(runtime.commands[-1]["friction_brake_ratio"], 1)
        sequences = [cmd["sequence"] for cmd in runtime.commands]
        self.assertEqual(sequences, sorted(set(sequences)))

    def test_paused_clock_times_out_and_stops(self):
        module = self.load()
        runtime = FakeRuntime([0])
        args = module.parse_args(["--clock-timeout", "0.1", "--torque", "20"])
        with self.assertRaisesRegex(RuntimeError, "stopped advancing"):
            module.run_session(runtime, args)
        self.assertEqual(runtime.commands[-1]["rear_axle_torque_nm"], 0)

    def test_rejects_invalid_arguments_before_loading_ros(self):
        module = self.load()
        for option, value in [("--torque", "nan"), ("--steering", "inf"),
                              ("--steering", "1.6"), ("--brake", "-1"),
                              ("--brake", "1.1"), ("--rate", "0"),
                              ("--duration", "-1"), ("--clock-timeout", "0"),
                              ("--topic", "relative")]:
            with self.subTest(option=option, value=value), contextlib.redirect_stderr(io.StringIO()):
                with self.assertRaises(SystemExit) as error:
                    module.parse_args([option, value])
                self.assertEqual(error.exception.code, 2)

    def test_sequence_exhaustion_reserves_final_stop(self):
        module = self.load()
        runtime = FakeRuntime([0, 10_000_000, 20_000_000])
        args = module.parse_args([])
        with self.assertRaisesRegex(RuntimeError, "sequence exhausted"):
            module.run_session(runtime, args, sequence_start=module.UINT64_MAX - 1)
        self.assertEqual(runtime.commands[-1]["sequence"], module.UINT64_MAX)
        self.assertEqual(runtime.commands[-1]["friction_brake_ratio"], 1)

    def test_shutdown_attempts_stop(self):
        module = self.load()
        runtime = FakeRuntime([0])
        runtime.ok = lambda: runtime.wall < 0.02
        module.run_session(runtime, module.parse_args([]))
        self.assertEqual(runtime.commands[-1]["rear_axle_torque_nm"], 0)
        self.assertEqual(runtime.commands[-1]["friction_brake_ratio"], 1)


class FakeRuntime:
    def __init__(self, clocks):
        self.clocks = iter(clocks)
        self.clock_ns = None
        self.clock_reversed = False
        self.wall = 0.0
        self.commands = []

    def monotonic(self):
        return self.wall

    def spin(self, timeout):
        self.wall += max(timeout, 0.001)
        clock = next(self.clocks, self.clock_ns)
        if clock is not None and self.clock_ns is not None and clock < self.clock_ns:
            self.clock_reversed = True
        self.clock_ns = clock

    def ok(self):
        return True

    def publish(self, command):
        self.commands.append(command)


if __name__ == "__main__":
    unittest.main()
