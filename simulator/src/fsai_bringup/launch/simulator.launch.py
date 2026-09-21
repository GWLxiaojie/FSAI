from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)

    root = Path(get_package_share_directory("fsai_bringup"))
    scenario_path = Path(arg("scenario"))
    if not scenario_path.is_absolute():
        scenario_path = root / "scenarios" / scenario_path
        if not scenario_path.suffix:
            scenario_path = scenario_path.with_suffix(".yaml")
    with scenario_path.open(encoding="utf-8") as stream:
        scenario = yaml.safe_load(stream)
    if not isinstance(scenario, dict):
        raise ValueError(f"{scenario_path}: scenario must be a mapping")
    vehicle = arg("vehicle") or scenario["vehicle_profile"]
    track = arg("track") or scenario["track_bundle"]
    track_path = Path(track)
    if not track_path.is_absolute():
        track_path = root / "tracks" / track
    profile_path = Path(vehicle)
    if not profile_path.is_absolute():
        profile_path = root / "vehicles" / vehicle

    core_type = arg("core_type")
    if core_type not in ("fsai", "eufs"):
        raise ValueError("core_type must be fsai or eufs")
    params = {
        "core_type": core_type,
        "core_params": str(profile_path),
        "run_mode": arg("run_mode"),
        "max_steps": int(arg("max_steps")),
        "track": str(track_path),
        "scenario": str(scenario_path),
        "seed": int(arg("seed")),
        "duration_limit_s": float(arg("duration_limit_s")),
        "outer_step_ms": int(arg("outer_step_ms")),
    }
    parameters = [params]
    if core_type == "eufs":
        params["core_params"] = str(
            Path(get_package_share_directory("vehicle_models"))
            / "config/DynamicBicycle/ads-dv-calculated.yaml"
        )
        parameters.append(
            str(Path(get_package_share_directory("eufs_sim2")) / "config/plugin_params.yaml")
        )

    simulation = Node(
        package="fsai_sim2_adapter",
        executable="fsai_simulation_node",
        name="eufs_sim2",
        output="screen",
        parameters=parameters,
    )
    actions = [simulation]
    if arg("visualize").lower() in ("true", "1", "yes"):
        description = (
            Path(get_package_share_directory("fsai_description"))
            / "urdf/reference_bicycle.urdf"
        ).read_text(encoding="utf-8")
        actions.append(
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": description, "use_sim_time": True}],
                output="screen",
            )
        )
        actions.append(
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(root / "config/simulator.rviz")],
                parameters=[{"use_sim_time": True}],
                output="screen",
            )
        )
    def on_simulation_exit(event, _context):
        if event.returncode != 0:
            raise RuntimeError(f"Simulation process failed with exit code {event.returncode}")
        return [EmitEvent(event=Shutdown(reason="Simulation completed"))]

    actions.append(
        RegisterEventHandler(
            OnProcessExit(
                target_action=simulation,
                on_exit=on_simulation_exit,
            )
        )
    )
    return actions


def generate_launch_description():
    defaults = {
        "vehicle": "",
        "track": "",
        "scenario": "straight_acceleration",
        "run_mode": "",
        "core_type": "fsai",
        "max_steps": "0",
        "visualize": "false",
        "seed": "-1",
        "duration_limit_s": "-1",
        "outer_step_ms": "-1",
    }
    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=value) for name, value in defaults.items()]
        + [OpaqueFunction(function=_launch)]
    )
