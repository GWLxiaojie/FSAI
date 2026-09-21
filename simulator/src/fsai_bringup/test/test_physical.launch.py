import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import rclpy
from fsai_interfaces.msg import ActuatorState
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock


def generate_test_description():
    path = get_package_share_directory("fsai_bringup") + "/launch/simulator.launch.py"
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments={"run_mode": "realtime", "max_steps": "1200"}.items(),
    )
    return LaunchDescription([simulation, launch_testing.actions.ReadyToTest()])


class TestPhysicalLaunch(unittest.TestCase):
    def test_actual_launch_loads_pose_and_drives_after_readiness(self):
        rclpy.init()
        node = rclpy.create_node("physical_launch_test")
        clocks, states, odometry = [], [], []
        subscriptions = [
            node.create_subscription(Clock, "/clock", clocks.append, 10),
            node.create_subscription(ActuatorState, "/fsai/actuator_state", states.append, 10),
            node.create_subscription(Odometry, "/ground_truth/odom", odometry.append, 10),
        ]
        deadline = time.monotonic() + 15
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
                if states and states[-1].stamp.sec >= 5 and states[-1].u_mps > 0.2:
                    break
            self.assertGreaterEqual(len(clocks), 2)
            self.assertTrue(states, "physical actuator feedback must be published")
            self.assertTrue(odometry, "physical odometry must be published")
            self.assertTrue(any(state.u_mps > 0.2 for state in states))
            self.assertEqual(odometry[-1].header.frame_id, "map")
            self.assertAlmostEqual(odometry[-1].pose.pose.orientation.z, 2**-0.5, places=8)
            self.assertGreater(odometry[-1].pose.pose.position.y, 0)
        finally:
            for subscription in subscriptions:
                node.destroy_subscription(subscription)
            node.destroy_node()
            rclpy.shutdown()
