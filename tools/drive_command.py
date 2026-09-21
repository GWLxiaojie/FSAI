#!/usr/bin/env python3
"""Publish simulated ADS-DV actuation using ROS /clock timestamps."""
import argparse
import math
import sys
import time


UINT64_MAX = (1 << 64) - 1


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--torque", type=float, default=0.0, help="rear axle N m; negative requests regeneration")
    parser.add_argument("--brake", type=float, default=0.0, help="friction brake request [0,1]")
    parser.add_argument("--steering", type=float, default=0.0, help="equivalent road wheel radians; positive left")
    parser.add_argument("--duration", type=float, default=2, help="duration in simulation seconds")
    parser.add_argument("--rate", type=float, default=50, help="publication rate in wall Hz")
    parser.add_argument("--topic", default="/fsai/actuation_command")
    parser.add_argument("--clock-timeout", type=float, default=5, help="wall seconds waiting for advancing /clock")
    args = parser.parse_args(argv)
    for name in ("torque", "brake", "steering", "duration", "rate", "clock_timeout"):
        if not math.isfinite(getattr(args, name)):
            parser.error(f"--{name.replace('_', '-')} must be finite")
    if not 0 <= args.brake <= 1:
        parser.error("--brake must be in [0,1]")
    if not abs(args.steering) < math.pi / 2:
        parser.error("--steering must be strictly between -pi/2 and pi/2")
    if args.duration <= 0 or args.clock_timeout <= 0 or not 0 < args.rate <= 1000:
        parser.error("duration/clock-timeout must be positive; rate must be in (0,1000]")
    if not args.topic or not args.topic.startswith("/"):
        parser.error("--topic must be an absolute ROS topic name")
    return args


def run_session(runtime, args, sequence_start=None):
    """Runtime supplies monotonic wall time, /clock, spin, ok, and publish.

    A publication is not an acknowledgement of actuator acceptance. The simulator
    safety state and watchdog remain authoritative.
    """
    sequence = time.time_ns() if sequence_start is None else sequence_start
    if not isinstance(sequence, int) or not 0 <= sequence < UINT64_MAX:
        raise ValueError("initial sequence is outside uint64 range")
    begin_wall = runtime.monotonic()
    last_progress_wall = begin_wall
    start_clock = None
    previous_clock = None
    next_publish = begin_wall
    period = 1 / args.rate

    def publish(stop=False):
        nonlocal sequence
        if sequence > UINT64_MAX:
            raise RuntimeError("command sequence exhausted")
        runtime.publish({
            "stamp_ns": runtime.clock_ns,
            "sequence": sequence,
            "steering_angle_rad": 0.0 if stop else args.steering,
            "front_axle_torque_nm": 0.0,
            "rear_axle_torque_nm": 0.0 if stop else args.torque,
            "friction_brake_ratio": 1.0 if stop else args.brake,
        })
        sequence += 1

    try:
        while runtime.ok():
            runtime.spin(min(period, 0.05))
            now = runtime.monotonic()
            if runtime.clock_reversed:
                raise RuntimeError("simulation clock moved backwards; stop and restart after simulator reset")
            clock = runtime.clock_ns
            if clock is None:
                if now - begin_wall >= args.clock_timeout:
                    raise RuntimeError("timed out waiting for /clock; no driving command sent")
                continue
            if start_clock is None:
                start_clock = clock
            if clock != previous_clock:
                previous_clock = clock
                last_progress_wall = now
            elif now - last_progress_wall >= args.clock_timeout:
                raise RuntimeError("simulation clock stopped advancing")
            if (clock - start_clock) / 1e9 >= args.duration:
                return
            if now >= next_publish:
                # Keep the last available sequence for the final braking command.
                if sequence >= UINT64_MAX:
                    raise RuntimeError("command sequence exhausted")
                publish()
                next_publish = now + period
    finally:
        if runtime.clock_ns is not None:
            try:
                publish(stop=True)
                runtime.spin(0.05)  # Best effort DDS delivery; no acknowledgement implied.
            except Exception as error:
                print(f"drive-command: final stop publication failed: {error}", file=sys.stderr)


def main(argv=None):
    args = parse_args(argv)
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
        from rosgraph_msgs.msg import Clock
        from fsai_interfaces.msg import ActuationCommand
    except ImportError as error:
        print(f"drive-command: source Humble and the workspace install/setup.bash: {error}", file=sys.stderr)
        return 2

    class RosRuntime:
        clock_ns = None
        clock_reversed = False

        def __init__(self, node):
            self.node = node
            self.publisher = node.create_publisher(ActuationCommand, args.topic, 10)
            clock_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                   durability=DurabilityPolicy.VOLATILE)
            self.subscription = node.create_subscription(Clock, "/clock", self.on_clock, clock_qos)

        def on_clock(self, message):
            stamp = message.clock
            current = stamp.sec * 1_000_000_000 + stamp.nanosec
            if self.clock_ns is not None and current < self.clock_ns:
                self.clock_reversed = True
            self.clock_ns = current

        def monotonic(self):
            return time.monotonic()

        def spin(self, timeout):
            rclpy.spin_once(self.node, timeout_sec=timeout)

        def ok(self):
            return rclpy.ok()

        def publish(self, command):
            message = ActuationCommand()
            stamp = command["stamp_ns"]
            message.stamp.sec, message.stamp.nanosec = divmod(stamp, 1_000_000_000)
            for key, value in command.items():
                if key != "stamp_ns":
                    setattr(message, key, value)
            self.publisher.publish(message)

    node = None
    try:
        rclpy.init(args=[])
        node = rclpy.create_node("fsai_drive_command")
        run_session(RosRuntime(node), args)
        return 0
    except KeyboardInterrupt:
        return 130
    except Exception as error:
        print(f"drive-command: {error}", file=sys.stderr)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
