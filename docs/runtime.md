# Runtime and ROS interfaces

The supported runtime is Ubuntu 22.04 / ROS 2 Humble. The FSAI plant and the
original EUFS compatibility core are separate compositions. No upstream files
are modified by this project.

## Start and configure a run

```bash
tools/run_sim.sh --vehicle ads_dv --scenario straight_acceleration
tools/run_sim.sh --vehicle ads_dv --scenario manual --visualize
tools/run_official_laps.sh --fast
```

`straight_acceleration` and `brake_to_stop` are scripted 20-second regression
experiments. Their YAML files explicitly enable `auto_start`, wait five
simulation seconds in READY, and supply physical command schedules. External
commands are ignored during a scripted experiment. `manual` starts OFF, runs
for 120 simulation seconds, and accepts external control after arming; see
[drive-command.md](drive-command.md).

The launch entry resolves the scenario's `vehicle_profile` and `track_bundle`
unless the user supplies a vehicle or track override. A track bundle contains
validated `track.yaml` and `cones.csv`; its start pose is applied to the plant.
Absolute resource paths are accepted as well as installed resource names.
The bundled `skidpad_small` is a small cone corridor fixture, not a surveyed
Formula Student skidpad. Its initial heading points along the corridor (+Y).

The ROS launch arguments `seed`, `duration_limit_s`, `outer_step_ms`, `run_mode`
and `max_steps` provide explicit overrides. The defaults `-1` for seed/duration/
step and an empty mode mean “use the scenario value”. `max_steps=0` disables
the step-count limit; the scenario time limit still applies. A shorter duration
cannot truncate a configured command schedule. Supported outer steps are 1 ms
and 5 ms, preserving the 1 ms plant integration and sensor sample boundaries.
Invalid/missing resources, unsupported units/schema, and malformed values fail
startup. Node failures propagate a nonzero launch exit status.

`realtime` and `as_fast_as_possible` only change wall-clock pacing. Identical
scripted inputs and seeds produce identical state and camera-message sequences
within the same build. External controllers must keep up with simulation time;
use scripted inputs for reproducible faster-than-realtime experiments. A normal
completion stops at the exact configured step/time boundary and reports
FINISHED. This is an experiment completion, not a race/lap adjudication.

`official_ten_laps` is a separate ground-truth reference self-test. Its wrapper
downloads pinned official assets, supplies the absolute imported track path,
incrementally builds, and runs ten laps at 2.5 m/s. It starts five simulation
seconds after READY and ends only after the tenth lap and a sustained stop.
Footprint clearance, ordered route progress, a 3600 s time limit, and a JSON
completion report guard the run. The wrapper rejects an incomplete/missing
report or a failed acceptance threshold. Report files are created exclusively,
never overwritten. `--fast` disables GUI and changes wall-clock pacing only.
The default is localhost DDS domain 183 with a per-domain wrapper lock.
This controller uses true plant state and a known centerline, not perception or
localization. See [official-ten-laps.md](official-ten-laps.md) for provenance and
the distinction between reference lap counting and competition adjudication.

## Safety and commands

Physical commands use `fsai_interfaces/msg/ActuationCommand` on the profile's
`physical_command_topic` (default `/fsai/actuation_command`). Units are road-wheel
steering radians, total front/rear axle wheel-side torque in Nm, and a friction
brake ratio in [0, 1]. Commands carry a simulation-time `stamp` and a strictly
increasing `sequence`. The first sequence may be zero. Old, future-dated,
duplicate, out-of-order, or non-finite messages are rejected without refreshing
the watchdog. The latest accepted source stamp determines age.

| Service | Type | Behavior |
| --- | --- | --- |
| `/set_mission` | `eufs_msgs/srv/SetMission` | OFF → READY for a valid mission |
| `/go` | `std_srvs/srv/Trigger` | READY → DRIVING after five simulation seconds |
| `/ebs` | `std_srvs/srv/Trigger` | Latch emergency braking, including while OFF |
| `/reset` | `std_srvs/srv/Trigger` | Restore OFF, original pose, zero time, sensor seeds and command acceptance state |

Only DRIVING accepts drive commands. Commands received before arming cannot
later start the vehicle. Timeout removes drive torque and applies the profile's
safe brake ratio. EBS remains latched until reset. Reset disables an active
scripted controller and requires manual rearming; relaunch to repeat the entire
scripted scenario. It replaces the plant context before exposing the new state.

The profile's `interfaces.yaml` controls command topic, timeout, monotonic
sequence policy and EUFS acceleration compatibility. Compatibility is off by
default. Explicitly enabling it subscribes to `/cmd` with
`ackermann_msgs/msg/AckermannDriveStamped`; header stamps still use simulation
time. Positive acceleration maps to rear-axle torque using mass × tyre radius,
and negative acceleration maps to friction braking. This approximation logs its
conversion parameters and does not model a VCU. Use one command source at a time.

## Physical-mode outputs

| Topic | Message / units | Rate |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock`, simulation time | Outer step |
| `/fsai/actuator_state` | `fsai_interfaces/ActuatorState`, actual saturated actuator values and body velocities | Outer step |
| `/sim/state/as_state` | `std_msgs/String`, OFF/READY/DRIVING/EMERGENCY_BRAKE/FINISHED | Outer step |
| `/sim/state/mission` | `std_msgs/String`, selected mission or not_selected | Outer step |
| `/ground_truth/odom` | `nav_msgs/Odometry`, map pose and body-frame twist | Outer step |
| `/ground_truth/forces` | `eufs_msgs/CarForces`, tyre-frame Fx/Fy and normal Fz in N | Outer step |
| `/ground_truth/track_markers` | `visualization_msgs/MarkerArray`, stable cone IDs and optional road/terrain meshes in map | Startup/reset, latched |
| `/joint_states` | `sensor_msgs/JointState`, front steering and four wheel-spin angles | Outer step |
| `/sim/reference/status` | `std_msgs/String`, JSON progress/error/clearance/status; reference mode only | 50 Hz while driving |
| `/sim/reference/markers` | `visualization_msgs/MarkerArray`, reference lap/speed text | 50 Hz while driving |
| `/sensors/imu` | `sensor_msgs/Imu`, rad/s and m/s² | 200 Hz |
| `/sensors/wheel_speeds` | `eufs_msgs/WheelSpeedsStamped`, **revolutions/s**, steering radians | 100 Hz |
| `/sensors/oss` | `geometry_msgs/TwistWithCovarianceStamped`, body-frame m/s | 100 Hz |
| `/sensors/gnss` | `sensor_msgs/NavSatFix`, synthetic latitude/longitude | 10 Hz |
| `/sensors/camera/cones` | `eufs_msgs/ConeArrayWithCovariance`, body-frame metres and colors | 20 Hz |
| `/sensors/lidar/cones` | `eufs_msgs/ConeArrayWithCovariance`, body-frame metres, unknown colors | 20 Hz |

The body and sensor frame is `base_footprint`; TF publishes
`map → base_footprint`. IMU/OSS/wheel/GNSS outputs are **ideal planar sensors**,
without mounting offsets, sensor delays, bias drift, dropout or calibration.
The accelerometer reports physical body acceleration and +gravity on Z for a
level stationary car. GNSS uses a synthetic tangent-plane origin at latitude
0°, longitude 0°; it is not a surveyed location or a validated geodetic model.

Camera and LiDAR apply visibility filtering and independent seeded Gaussian
position noise. Camera range is 0.2–20 m, horizontal half-FOV 0.96 rad, σ=0.03 m.
LiDAR range is 0.2–100 m, half-FOV π/2, σ=0.02 m. These are explicitly
unvalidated example error settings, not identified ADS-DV sensor models.
Reset restores both random streams. Reproducibility across different standard
library implementations is not promised. There is no image or point-cloud
rendering. Ground-truth topics must remain isolated from autonomous algorithms.

Tyre force outputs divide the accepted bicycle-model axle force equally between
left and right wheels. They do not imply simulated per-wheel slip, differential
dynamics, suspension, or lateral load transfer. See [calibration.md](calibration.md)
for official dimensions, parameter provenance and remaining measurement needs.

## EUFS compatibility and visualization

`upstream_compatibility.launch.py` retains the upstream core, plugins, `/cmd`,
state-machine services and its existing topic remappings (`/odom`, `/cones`,
`/ros_can/*`, `/imu/data`, etc.). It does not use the FSAI safety/profile/scenario
path. Its upstream reset and noise behavior are not the deterministic FSAI reset
contract. The original EUFS wheel-speed units are preserved as revolutions/s.

Physical mode deliberately uses the namespaces in the table above instead of
silently publishing ground truth as upstream `/odom`. Existing consumers must
use ROS remapping explicitly. Do not remap ground-truth odometry into a sensor
input for autonomous acceptance tests.

`visualize:=true` starts `robot_state_publisher` and RViz with the installed
`config/simulator.rviz`: map, vehicle URDF, TF, cone colors and ground-truth
trajectory. The ADS-DV wrappers supply a generated URDF referencing the official
native GLBs, preserving meshes and embedded textures without decimation. Wheel
rotation and Ackermann steering are animated from plant outputs. Visual wheel
positions follow the official MovieNX assembly; they do not replace the current
plant parameters. The explicit `reference_bicycle` test profile retains its
lightweight URDF. Neither description adds a collision/suspension backend.
The ten-lap wrapper uses `config/official_laps.rviz` with a following camera,
official track/cone meshes and lap text. Visual process failure propagates as a
run failure; closing the window early aborts the experiment. A graphical Ubuntu
session is required. A local Xvfb/software-GL smoke check confirms official mesh
and texture loading, but headless CI does not prove graphics-driver compatibility
on every machine.

This runtime supplies validated resource loading, deterministic scripted
experiments and safe ROS control. Full RaceDirector/lap rules, cone collision
adjudication, arbitrary track generation, and a complete
sensor-based autonomous controller remain outside this runtime implementation.
MCAP recording is available separately through `tools/record_sim.sh`; it records
ROS messages, not a serialized plant/sensor snapshot.
