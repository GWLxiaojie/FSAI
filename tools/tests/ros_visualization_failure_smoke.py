#!/usr/bin/env python3
"""Opt-in real ROS launch regression; run after sourcing the built overlay.

No asset downloads are needed. --valid-display additionally verifies healthy
GUI shutdown and closing the RViz child before the simulation finishes.
"""
import argparse
import os
from pathlib import Path
import re
import resource
import signal
import subprocess
import sys
import tempfile
import time


ROOT = Path(__file__).resolve().parents[2]
LAUNCH = ROOT / "simulator/src/fsai_bringup/launch/simulator.launch.py"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--valid-display", help="Optional accessible X display, e.g. an owned Xvfb :89")
    args = parser.parse_args()
    started = time.monotonic()
    deadline = started + 9.0
    # A Qt abort is intentional in the negative case; do not invoke apport or
    # spend the deadline writing a large core dump.
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    environment = os.environ.copy()
    environment.update(ROS_DOMAIN_ID="181", ROS_LOCALHOST_ONLY="1", QT_QPA_PLATFORM="xcb")

    def run_case(label, *, display, visualize, steps, close_window=False, interrupt_launch=False):
        child = None
        output = ""
        log = tempfile.TemporaryFile(mode="w+")
        case_started = time.monotonic()
        window_closed = False
        interrupted = False

        def snapshot():
            # pread does not move the shared stdout file offset while children
            # are still writing; seek/read here could overwrite their logs.
            return os.pread(log.fileno(), os.fstat(log.fileno()).st_size, 0).decode("utf-8", errors="replace")

        try:
            env = environment.copy()
            env["DISPLAY"] = display
            command = ["ros2", "launch", str(LAUNCH), "vehicle:=reference_bicycle",
                       "scenario:=straight_acceleration", "run_mode:=realtime",
                       f"visualize:={str(visualize).lower()}", f"max_steps:={steps}"]
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True, env=env, cwd=ROOT)
            while child.poll() is None:
                if time.monotonic() >= deadline:
                    raise TimeoutError(f"{label}: launch exceeded the smoke-test deadline\n{snapshot()}")
                if interrupt_launch and not interrupted and time.monotonic() - case_started >= .8:
                    os.killpg(child.pid, signal.SIGINT)
                    interrupted = True
                if close_window and not window_closed and time.monotonic() - case_started >= .8:
                    output = snapshot()
                    match = re.search(r"\[rviz2-\d+\]: process started with pid \[(\d+)\]", output)
                    if match:
                        # This PID is a child just created by this launch. Signal
                        # only RViz, not the launch group (the Ctrl-C case).
                        os.kill(int(match.group(1)), signal.SIGINT)
                        window_closed = True
                time.sleep(.02)
            output = snapshot()
            return child.returncode, output, window_closed
        finally:
            if child is not None:
                try:
                    os.killpg(child.pid, signal.SIGINT)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=.4)
                except subprocess.TimeoutExpired:
                    pass
                # Catch descendants even when ros2 launch has already exited.
                try:
                    os.killpg(child.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                try:
                    child.wait(timeout=.2)
                except subprocess.TimeoutExpired:
                    pass
            log.close()

    try:
        code, output, _ = run_case("invalid display", display=":65534", visualize=True, steps=400)
        if code == 0 or "Visualization process failed" not in output:
            raise AssertionError(f"RViz startup failure was not propagated (exit={code})\n{output}")
        print("PASS: actual RViz/xcb display failure propagates a nonzero launch exit", flush=True)

        code, output, _ = run_case("normal headless stop", display=":65534", visualize=False, steps=20)
        if code != 0 or "Visualization process failed" in output:
            raise AssertionError(f"Normal shutdown incorrectly failed (exit={code})\n{output}")
        print("PASS: normal simulation shutdown remains successful", flush=True)

        if args.valid_display:
            code, output, _ = run_case("healthy GUI stop", display=args.valid_display, visualize=True, steps=200)
            if code != 0 or "Visualization process failed" in output:
                raise AssertionError(f"Healthy GUI global shutdown failed (exit={code})\n{output}")
            print("PASS: healthy visualization accepts normal global shutdown", flush=True)
            code, output, closed = run_case("window close", display=args.valid_display,
                                           visualize=True, steps=1000, close_window=True)
            if code != 0 or not closed or "Visualization window closed" not in output:
                raise AssertionError(f"Closing RViz did not stop the launch cleanly (exit={code})\n{output}")
            if "Simulation completed after 1000 steps" in output:
                raise AssertionError("The simulator kept running after the RViz window closed")
            print("PASS: closing the visualization child stops the whole launch", flush=True)
            code, output, _ = run_case("user Ctrl-C", display=args.valid_display,
                                       visualize=True, steps=1000, interrupt_launch=True)
            if "Visualization process failed" in output or code not in (0, 130, -signal.SIGINT):
                raise AssertionError(f"User Ctrl-C was mistaken for a visualization failure (exit={code})\n{output}")
            print("PASS: user Ctrl-C is not misreported as visualization failure", flush=True)
        else:
            print("INFO: healthy GUI/window-close cases require --valid-display; not exercised", flush=True)
    except (AssertionError, OSError, TimeoutError) as error:
        print(f"VISUALIZATION SMOKE FAIL: {error}", file=sys.stderr)
        return 1
    print(f"VISUALIZATION SMOKE PASS elapsed={time.monotonic() - started:.2f}s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
