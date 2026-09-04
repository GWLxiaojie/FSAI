from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    vehicle = LaunchConfiguration("vehicle")
    track = LaunchConfiguration("track")
    scenario = LaunchConfiguration("scenario")
    run_mode = LaunchConfiguration("run_mode")
    core_type = LaunchConfiguration("core_type")
    max_steps = LaunchConfiguration("max_steps")

    vehicle_profile = PathJoinSubstitution(
        [FindPackageShare("fsai_bringup"), "vehicles", vehicle]
    )
    plugin_params = PathJoinSubstitution(
        [FindPackageShare("eufs_sim2"), "config", "plugin_params.yaml"]
    )

    simulation = Node(
        package="fsai_sim2_adapter",
        executable="fsai_simulation_node",
        name="eufs_sim2",
        output="screen",
        parameters=[
            {
                "core_type": core_type,
                "core_params": vehicle_profile,
                "run_mode": run_mode,
                "max_steps": max_steps,
                "track": track,
                "scenario": scenario,
            },
            plugin_params,
        ],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle", default_value="reference_bicycle"),
            DeclareLaunchArgument("track", default_value="skidpad_small"),
            DeclareLaunchArgument("scenario", default_value="straight_acceleration"),
            DeclareLaunchArgument("run_mode", default_value="realtime"),
            DeclareLaunchArgument("core_type", default_value="fsai"),
            DeclareLaunchArgument("max_steps", default_value="0"),
            simulation,
        ]
    )
