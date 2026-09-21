#!/usr/bin/env python3
"""Opt-in, real DDS control smoke test. Requires the built Humble overlay."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time


def main():
    # Set before importing/initializing DDS; all children inherit this test domain.
    os.environ["ROS_DOMAIN_ID"] = "174"
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    started = time.monotonic()
    deadline = started + 17.0  # Reserve three seconds for child cleanup.
    children = []
    node = None
    log = tempfile.TemporaryFile(mode="w+")
    result = 1
    try:
        import rclpy
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from rosgraph_msgs.msg import Clock
        from std_msgs.msg import String
        from std_srvs.srv import Trigger
        from eufs_msgs.srv import SetMission
        from fsai_interfaces.msg import ActuatorState

        rclpy.init(args=[])
        node = rclpy.create_node("fsai_control_smoke")
        observations = {"clock": None, "state": None, "actuator": None,
                        "actuator_count": 0, "max_speed": 0.0, "max_torque": 0.0}

        def clock_callback(message):
            observations["clock"] = message.clock.sec + message.clock.nanosec * 1e-9

        def state_callback(message):
            observations["state"] = message.data

        def actuator_callback(message):
            observations["actuator"] = message
            observations["actuator_count"] += 1
            observations["max_speed"] = max(observations["max_speed"], message.u_mps)
            observations["max_torque"] = max(observations["max_torque"], message.rear_axle_torque_nm)

        node.create_subscription(Clock, "/clock", clock_callback,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        node.create_subscription(String, "/sim/state/as_state", state_callback, 10)
        node.create_subscription(ActuatorState, "/fsai/actuator_state", actuator_callback, 10)

        def start(command):
            child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                     start_new_session=True, cwd=Path(__file__).resolve().parents[2])
            children.append(child)
            return child

        launch = start(["ros2", "launch", "fsai_bringup", "simulator.launch.py",
                        "scenario:=manual", "max_steps:=4000", "run_mode:=realtime"])

        def wait_for(predicate, description):
            while not predicate():
                if time.monotonic() >= deadline:
                    raise RuntimeError(f"wall deadline exceeded: {description}")
                if launch.poll() is not None:
                    raise RuntimeError(f"simulator exited early ({launch.returncode}): {description}")
                rclpy.spin_once(node, timeout_sec=0.02)

        def service(name, kind, request):
            client = node.create_client(kind, name)
            try:
                wait_for(client.service_is_ready, f"discover {name}")
                future = client.call_async(request)
                wait_for(future.done, f"response from {name}")
                if future.exception():
                    raise RuntimeError(f"{name}: {future.exception()}")
                return future.result()
            finally:
                node.destroy_client(client)

        def require(condition, description):
            if not condition:
                raise AssertionError(description)

        wait_for(lambda: observations["clock"] is not None and observations["state"] == "OFF"
                 and observations["actuator"] is not None, "initial OFF state and clock")
        request = SetMission.Request()
        request.mission = 5
        response = service("/set_mission", SetMission, request)
        require(response.success, f"set_mission failed: {response.message}")
        wait_for(lambda: observations["state"] == "READY", "READY state")
        ready_time = observations["clock"]
        wait_for(lambda: observations["clock"] >= ready_time + 5.05, "five seconds READY")
        response = service("/go", Trigger, Trigger.Request())
        require(response.success, f"go failed: {response.message}")
        wait_for(lambda: observations["state"] == "DRIVING", "DRIVING state")
        drive = start([sys.executable, "tools/drive_command.py", "--torque", "30",
                       "--duration", "2", "--rate", "50", "--clock-timeout", "2"])
        wait_for(lambda: drive.poll() is not None, "drive command process completion")
        require(drive.returncode == 0, f"drive CLI failed ({drive.returncode})")
        require(observations["max_torque"] > 20, "rear axle torque never reached driving request")
        require(observations["max_speed"] > 0.05, "vehicle did not move forward")
        after_drive = observations["actuator_count"]
        wait_for(lambda: observations["actuator_count"] > after_drive
                 and abs(observations["actuator"].rear_axle_torque_nm) < 1e-9
                 and observations["actuator"].friction_brake_ratio > 0,
                 "zero drive torque and brake after CLI exit")
        print(f"DRIVE PASS max_u={observations['max_speed']:.6f} m/s "
              f"max_rear_torque={observations['max_torque']:.3f} N m", flush=True)
        response = service("/ebs", Trigger, Trigger.Request())
        require(response.success, f"ebs failed: {response.message}")
        wait_for(lambda: observations["state"] == "EMERGENCY_BRAKE", "EBS state")
        response = service("/go", Trigger, Trigger.Request())
        require(not response.success, "EBS was not latched: go succeeded")
        response = service("/set_mission", SetMission, request)
        require(not response.success, "EBS was not latched: set_mission succeeded")
        require(observations["state"] == "EMERGENCY_BRAKE", "EBS latch disappeared")
        print("EBS PASS go/set_mission rejected while latched", flush=True)
        before_reset = observations["clock"]
        count_before_reset = observations["actuator_count"]
        response = service("/reset", Trigger, Trigger.Request())
        require(response.success, f"reset failed: {response.message}")
        wait_for(lambda: observations["state"] == "OFF"
                 and observations["clock"] < before_reset
                 and observations["actuator_count"] > count_before_reset
                 and abs(observations["actuator"].u_mps) < 1e-9
                 and abs(observations["actuator"].rear_axle_torque_nm) < 1e-9,
                 "reset OFF with zero velocity/torque and reset clock")
        print(f"RESET PASS state=OFF u=0 torque=0 clock={observations['clock']:.3f}s", flush=True)
        result = 0
    except Exception as error:
        print(f"ROS CONTROL FAIL: {error}", file=sys.stderr, flush=True)
    finally:
        # These process groups were created by this test; never target other nodes.
        for child in reversed(children):
            try:
                os.killpg(child.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        cleanup_deadline = min(started + 19.5, time.monotonic() + 2.0)
        for child in reversed(children):
            try:
                child.wait(timeout=max(0.01, cleanup_deadline - time.monotonic()))
            except subprocess.TimeoutExpired:
                pass
            try:
                os.killpg(child.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            try:
                child.wait(timeout=0.1)
            except subprocess.TimeoutExpired:
                result = 1
        if node is not None:
            node.destroy_node()
        if "rclpy" in locals() and rclpy.ok():
            rclpy.shutdown()
        if result:
            log.seek(0)
            print(log.read()[-12000:], file=sys.stderr)
        log.close()
    print(f"ROS CONTROL {'PASS' if result == 0 else 'FAIL'} elapsed={time.monotonic() - started:.2f}s")
    return result


if __name__ == "__main__":
    sys.exit(main())
