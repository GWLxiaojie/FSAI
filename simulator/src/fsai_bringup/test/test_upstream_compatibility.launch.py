import time
import unittest

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing
import launch_testing.actions
import rclpy
from rosgraph_msgs.msg import Clock


def generate_test_description():
    launch_path = (
        get_package_share_directory("fsai_bringup")
        + "/launch/upstream_compatibility.launch.py"
    )
    compatibility_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path)
    )

    return LaunchDescription(
        [compatibility_launch, launch_testing.actions.ReadyToTest()]
    )


class TestUpstreamCompatibility(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def test_clock_advances_within_twenty_seconds(self):
        node = rclpy.create_node("upstream_compatibility_clock_test")
        clock_messages = []
        subscription = node.create_subscription(
            Clock, "/clock", clock_messages.append, 10
        )
        deadline = time.monotonic() + 20.0

        try:
            while len(clock_messages) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.1)

            self.assertGreaterEqual(
                len(clock_messages),
                2,
                "Did not receive two /clock messages within 20 seconds",
            )
            first = clock_messages[0].clock
            second = clock_messages[1].clock
            first_nanoseconds = first.sec * 1_000_000_000 + first.nanosec
            second_nanoseconds = second.sec * 1_000_000_000 + second.nanosec
            self.assertGreater(second_nanoseconds, first_nanoseconds)
        finally:
            node.destroy_subscription(subscription)
            node.destroy_node()
