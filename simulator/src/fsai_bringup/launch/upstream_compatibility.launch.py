from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    core_params = PathJoinSubstitution(
        [
            FindPackageShare("vehicle_models"),
            "config",
            "DynamicBicycle",
            "ads-dv-calculated.yaml",
        ]
    )
    plugin_params = PathJoinSubstitution(
        [FindPackageShare("eufs_sim2"), "config", "plugin_params.yaml"]
    )

    simulation = Node(
        package="fsai_sim2_adapter",
        executable="fsai_simulation_node",
        name="eufs_sim2",
        output="screen",
        parameters=[{"core_params": core_params}, plugin_params],
        remappings=[
            ("/plugin/cone_fusion/gt_cones", "/cones/lenient"),
            ("/plugin/cone_fusion/cones", "/cones"),
            ("/plugin/cone_fusion/map", "/map"),
            ("/plugin/wheel_speed_plugin/wheel_speed", "/ros_can/wheel_speeds"),
            ("/plugin/vehicle_state_plugin/ground_truth/state", "/odom"),
            ("/plugin/state_publisher/ros_can/state_str", "/sim/ros_can/state_str"),
            ("/plugin/state_publisher/ros_can/state", "/sim/ros_can/state"),
            ("/complete_mission_flag", "/ros_can/mission_completed"),
            ("/fix", "/ros_can/fix"),
            ("/plugin/cone_fusion/camera/cones", "/camera/cones"),
            ("/plugin/cone_fusion/lidar_grid/cones", "/lidar_grid/cones"),
            ("/plugin/twist_publisher/twist", "/ros_can/twist"),
            ("/plugin/imu_plugin/imu/data", "/imu/data"),
        ],
    )

    return LaunchDescription([simulation])
