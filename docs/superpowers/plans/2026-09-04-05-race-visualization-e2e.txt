# Race, Visualization and Full-Lap E2E Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付比赛状态管理、可重放工件、车辆可视化、统一 launch 和不使用 Ground Truth 的整圈闭环 E2E。

**Architecture:** `RaceDirector` 只观察接受后的车辆状态和 TrackDefinition，并通过安全命令链请求停车。
`ScenarioRunner` 组合 plant、传感器、赛道、规则、记录和 Foxglove，交互与无头模式共享完全相同的数据路径。

**Tech Stack:** C++20, nlohmann_json, ROS 2 Humble, launch_testing, FlatBuffers, rosbag2_storage_mcap, URDF/Xacro, Foxglove Bridge.

**Spec:** `docs/superpowers/specs/2026-09-04-eufs-sim2-team-simulator-design.md`

## Global Constraints

RaceDirector 不得覆盖车辆位置、速度、轮速或执行器状态。
停车请求必须经过 SafetyChain。
reset 必须恢复全部可观察状态。
交互和无头模式必须产生相同物理和传感器序列。
运行工件必须记录代码 revision、依赖 revision、参数 hash、赛道 hash、seed 和平台。
Foxglove 不得参与物理、传感器或裁判计算。
正式控制器不得订阅 `/ground_truth/*`。
完整 E2E 必须按最终用户的 launch 入口运行。
可视化发布前必须在 Ubuntu 图形环境人工检查。

---

## File Map

```text
simulator/src/fsai_race/
  include/fsai_race/scenario.hpp
  include/fsai_race/race_director.hpp
  include/fsai_race/report.hpp
  src/
  test/
simulator/src/fsai_interfaces/msg/
  RaceState.msg
  SimulationEvent.msg
simulator/src/fsai_interfaces/schema/simulation_snapshot.fbs
simulator/src/fsai_sim2_adapter/
  include/fsai_sim2_adapter/artifact_writer.hpp
  include/fsai_sim2_adapter/scenario_runner.hpp
  src/
simulator/src/fsai_description/
  urdf/
  meshes/
simulator/src/fsai_bringup/
  launch/simulator.launch.py
  config/
  foxglove/
  scenarios/
  test/
simulator/src/fsai_reference_controller/
```

### Task 1: Define and strictly load Scenario schema version 1

**Files:**

- Create: `simulator/src/fsai_race/package.xml`
- Create: `simulator/src/fsai_race/CMakeLists.txt`
- Create: `simulator/src/fsai_race/include/fsai_race/scenario.hpp`
- Create: `simulator/src/fsai_race/src/scenario.cpp`
- Create: `simulator/src/fsai_race/test/scenario_test.cpp`
- Create: `simulator/src/fsai_bringup/scenarios/autocross_regression_01.yaml`

**Interfaces:**

- Consumes: Scenario YAML path.
- Produces: Immutable `Scenario LoadScenario(path)`.

- [ ] **Step 1: Write failing schema tests**

```cpp
TEST(Scenario, LoadsCompleteVersionOne) {
  auto scenario = LoadScenario(RegressionScenarioPath());
  EXPECT_EQ(scenario.schema_version, 1u);
  EXPECT_EQ(scenario.mode, RunMode::kAsFastAsPossible);
  EXPECT_EQ(scenario.plant_step, 1ms);
  EXPECT_EQ(scenario.outer_step, 5ms);
}

TEST(Scenario, RejectsUnknownAndInconsistentFields) {
  EXPECT_THAT(CaptureError(UnknownFieldScenario()),
              HasSubstr("unknown field"));
  EXPECT_THAT(CaptureError(InvalidStepScenario()),
              HasSubstr("outer_step must be a multiple of plant_step"));
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_race --ctest-args -R scenario_test --output-on-failure`

Expected: FAIL because the package and loader are absent.

- [ ] **Step 3: Implement exact Scenario fields**

```cpp
struct Scenario final {
  std::uint32_t schema_version;
  std::string name;
  Mission mission;
  std::string vehicle_profile;
  std::filesystem::path track_bundle;
  std::uint64_t seed;
  std::chrono::nanoseconds duration_limit;
  RunMode mode;
  std::chrono::nanoseconds plant_step;
  std::chrono::nanoseconds outer_step;
  std::uint32_t target_laps;
  ArtifactPolicy artifacts;
};
```

Require all fields.
Require 1 ms plant step and 5 ms outer step for formal scenarios.
Reject unknown keys, invalid mission, zero laps, negative duration and missing referenced files.
Resolve paths relative to the scenario file before making the object immutable.

- [ ] **Step 4: Run valid and invalid fixtures**

Expected: Every invalid fixture reports file, YAML path and rejected value.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_race simulator/src/fsai_bringup/scenarios
git commit -m "feat: load versioned simulation scenarios"
```

### Task 2: Implement deterministic RaceDirector rules

**Files:**

- Create: `simulator/src/fsai_race/include/fsai_race/race_director.hpp`
- Create: `simulator/src/fsai_race/src/race_director.cpp`
- Create: `simulator/src/fsai_race/test/race_director_test.cpp`
- Create: `simulator/src/fsai_track_tools/include/fsai_track_tools/track_query.hpp`
- Create: `simulator/src/fsai_track_tools/src/track_query.cpp`
- Create: `simulator/src/fsai_interfaces/msg/RaceState.msg`
- Create: `simulator/src/fsai_interfaces/msg/SimulationEvent.msg`
- Modify: `simulator/src/fsai_interfaces/package.xml`
- Modify: `simulator/src/fsai_interfaces/CMakeLists.txt`

**Interfaces:**

- Consumes: Accepted Ground Truth, TrackDefinition, collision candidates and simulation time.
- Produces: `RaceUpdate RaceDirector::Update(...)` with state, events and safety requests.

- [ ] **Step 1: Write failing race-rule tests**

```cpp
TEST(RaceDirector, CountsOnlyForwardGateCrossings) {
  director.Update(BeforeFinish(), 1s);
  auto forward = director.Update(AfterFinishForward(), 1100ms);
  auto reverse = director.Update(BeforeFinishReverse(), 1200ms);
  EXPECT_EQ(forward.state.completed_laps, 1u);
  EXPECT_EQ(reverse.state.completed_laps, 1u);
}

TEST(RaceDirector, ConeHitCreatesOneEventPerCone) {
  director.Update(OverlappingCone("blue-4"), 2s);
  auto update = director.Update(OverlappingCone("blue-4"), 2005ms);
  EXPECT_EQ(CountEvents(update, EventType::kConeHit), 0u);
}

TEST(RaceDirector, OutOfBoundsRequestsEbs) {
  auto update = director.Update(OutsideTrack(), 3s);
  EXPECT_TRUE(update.safety_request.trigger_ebs);
  EXPECT_EQ(update.state.result, Result::kDnf);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_race --ctest-args -R race_director_test --output-on-failure`

Expected: FAIL because RaceDirector is absent.

- [ ] **Step 3: Implement geometry and state transitions**

Detect gate crossing by signed distance change and require velocity along the gate normal to be positive.
Use the vehicle collision polygon from the active profile and a cone circle from the rules profile.
Latch a hit cone ID so sustained overlap emits one event.
Define in-bounds as the drivable polygon between ordered left and right boundaries.
Allow the configured startup envelope behind the start gate.
State progression must be `kWaiting -> kRunning -> kFinishing -> kFinished` or `kDnf`.
Duration limit, EBS, out-of-bounds and invalid state must produce deterministic ordered events.
RaceDirector may emit `SafetyRequest` but may not mutate PlantState.
`RaceState.msg` must contain state, result, mission, completed laps, current lap start and best lap time in integer nanoseconds.
`SimulationEvent.msg` must contain simulation timestamp, sequence number, event type, entity ID and detail string.

- [ ] **Step 4: Run all mission fixtures**

Cover acceleration, skidpad, autocross, trackdrive, manual test, timeout, cone hit and DNF.
Expected: Exact event order and lap time in integer nanoseconds.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_race simulator/src/fsai_track_tools simulator/src/fsai_interfaces
git commit -m "feat: add deterministic race director"
```

### Task 3: Serialize snapshots and canonical run reports

**Files:**

- Create: `simulator/src/fsai_interfaces/schema/simulation_snapshot.fbs`
- Modify: `simulator/src/fsai_interfaces/package.xml`
- Modify: `simulator/src/fsai_interfaces/CMakeLists.txt`
- Create: `simulator/src/fsai_race/include/fsai_race/report.hpp`
- Create: `simulator/src/fsai_race/src/report.cpp`
- Create: `simulator/src/fsai_race/test/report_test.cpp`
- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/artifact_writer.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/artifact_writer.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/artifact_writer_test.cpp`

**Interfaces:**

- Consumes: SimulationSnapshot, RaceUpdate, resolved configuration and ROS messages.
- Produces: `run.json`, `events.jsonl`, `simulation.mcap`, `final_snapshot.bin` and resolved YAML files.

- [ ] **Step 1: Write failing round-trip and report tests**

```cpp
TEST(Snapshot, RoundTripsEveryDeterministicState) {
  auto bytes = SerializeSnapshot(CompleteSnapshot());
  EXPECT_EQ(DeserializeSnapshot(bytes), CompleteSnapshot());
}

TEST(Report, UsesCanonicalFieldOrderAndIntegerTimes) {
  WriteReport(report, output);
  EXPECT_EQ(ReadText(output / "run.json"), GoldenRunJson());
}

TEST(ArtifactWriter, FailureDoesNotPublishPartialFinalFiles) {
  EXPECT_THROW(writer.Finalize(FailingSnapshot()), ArtifactError);
  EXPECT_FALSE(exists(output / "run.json"));
  EXPECT_TRUE(exists(output / ".failed"));
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_race fsai_sim2_adapter --ctest-args -R 'snapshot|report|artifact' --output-on-failure`

Expected: FAIL because schemas and writers are absent.

- [ ] **Step 3: Implement versioned transactional artifacts**

FlatBuffers schema version 1 must include plant, sensors, delay queues, RNG, race, safety, track revision and sim time.
Write files to a temporary run directory and atomically rename it only after all final checks pass.
Use integer nanoseconds for all report times.
Use canonical JSON key order, UTF-8, LF and final newline.
Record code revision, dependency lock hash, vehicle hash, track hash, seed, OS, architecture and result.
Record events in step order and event sequence order.
Use rosbag2 MCAP storage with simulation timestamps.

- [ ] **Step 4: Run round-trip, failed-write and replay tests**

Restore the final snapshot into a new process and reproduce the last 100 steps.
Expected: State, events and sensor output match exactly.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_interfaces simulator/src/fsai_race simulator/src/fsai_sim2_adapter
git commit -m "feat: write reproducible run artifacts"
```

### Task 4: Build the reference vehicle description

**Files:**

- Create: `simulator/src/fsai_description/package.xml`
- Create: `simulator/src/fsai_description/CMakeLists.txt`
- Create: `simulator/src/fsai_description/urdf/reference_bicycle.urdf.xacro`
- Create: `simulator/src/fsai_description/urdf/wheels.urdf.xacro`
- Create: `simulator/src/fsai_description/urdf/sensors.urdf.xacro`
- Create: `simulator/src/fsai_description/test/test_urdf.py`
- Modify: `simulator/src/fsai_bringup/vehicles/reference_bicycle/vehicle.yaml`

**Interfaces:**

- Consumes: Vehicle geometry and sensor mounting values from the profile.
- Produces: `robot_description` with matching frames and wheel joints.

- [ ] **Step 1: Write failing URDF consistency tests**

```python
def test_profile_and_urdf_geometry_match():
    profile = load_profile()
    robot = expand_xacro()
    assert wheelbase(robot) == profile["wheelbase_m"]["value"]
    assert track_width(robot) == profile["front_track_m"]["value"]

def test_required_frames_exist():
    assert required_frames() <= set(expand_xacro().links)
```

Required frames are `base_footprint`, `base_link`, `imu_link`, `gnss_link`, `oss_link`, `camera_link`, `lidar_link` and four wheel links.

- [ ] **Step 2: Run URDF tests**

Run: `pytest simulator/src/fsai_description/test/test_urdf.py -q`

Expected: FAIL because description files are absent.

- [ ] **Step 3: Implement Xacro from one geometry profile**

Pass dimensions and mounting transforms from launch-expanded profile arguments.
Use collision geometry separate from visual mesh.
Use simple primitive visuals as the guaranteed fallback.
Define continuous wheel rotation joints and steering joints for both front wheels.
Do not place dynamic mass or tyre parameters only in URDF.

- [ ] **Step 4: Run check_urdf and rendering smoke**

Run Xacro, `check_urdf` and a robot_state_publisher launch.
Expected: No missing links, duplicate frames or inertial warnings.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_description simulator/src/fsai_bringup/vehicles
git commit -m "feat: add reference vehicle description"
```

### Task 5: Publish visualization data and Foxglove layout

**Files:**

- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/visualization_publishers.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/visualization_publishers.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/visualization_publishers_test.cpp`
- Create: `simulator/src/fsai_bringup/foxglove/fsai_simulator.json`
- Create: `simulator/src/fsai_bringup/launch/visualization.launch.py`
- Create: `docs/visualization.md`

**Interfaces:**

- Consumes: Ground Truth debug stream, TrackDefinition, sensor observations, actuator diagnostics and RaceState.
- Produces: TF, joint states, marker arrays, plots and a read-only Foxglove layout.

- [ ] **Step 1: Write failing frame and marker tests**

The test must require every marker frame to be `map` or a declared vehicle sensor frame.
It must require blue, yellow, orange and big-orange RGBA values from one palette.
It must require left and right wheel steering signs to match the steering geometry.
It must require the layout to contain vehicle, track, trajectory, FOV, observation, actuator, tyre and race panels.

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R visualization_publishers_test --output-on-failure`

Expected: FAIL because publishers and layout are absent.

- [ ] **Step 3: Implement read-only visualization**

Publish map cones and trajectory in `map`.
Publish sensor FOVs from mounting frames.
Publish observed cones in each sensor frame.
Publish actual steering and wheel rotation as joint states.
Publish requested and actual actuator values, forces, slips, contact mode and race status.
Launch Foxglove Bridge only when `visualization:=true`.
Do not subscribe visualization publishers back into the simulation core.

- [ ] **Step 4: Perform automated and visual checks**

Run frame tests.
On Ubuntu, open the committed layout and check vehicle scale, wheel angles, TF alignment, cone colours, sensor FOVs and trajectory.
Capture one screenshot in the review artifact directory without committing it.
Expected: No visible intersection, inversion, scale or colour defect.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup/foxglove simulator/src/fsai_bringup/launch docs/visualization.md
git commit -m "feat: add Foxglove simulator view"
```

### Task 6: Assemble the formal ScenarioRunner and launch

**Files:**

- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/scenario_runner.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/scenario_runner.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/scenario_runner_test.cpp`
- Create: `simulator/src/fsai_bringup/launch/simulator.launch.py`
- Create: `tools/run_scenario.sh`
- Modify: `tools/run_sim.sh`

**Interfaces:**

- Consumes: Scenario, SimulationContext, SensorSuite, RaceDirector and ArtifactWriter.
- Produces: One complete deterministic scenario execution.

- [ ] **Step 1: Write failing step-order tests**

```cpp
TEST(ScenarioRunner, ExecutesDocumentedOuterStepOrder) {
  runner.StepOnce();
  EXPECT_EQ(trace.Stages(), Stages({
      "commands", "safety", "actuators", "plant",
      "ground_truth_history", "race", "sensor_sample",
      "delay_release", "publish"}));
}

TEST(ScenarioRunner, FailedStepPublishesNothingPartial) {
  plant.FailNextStep();
  EXPECT_THROW(runner.StepOnce(), SimulationError);
  EXPECT_EQ(publisher.MessageCount(), 0u);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R scenario_runner_test --output-on-failure`

Expected: FAIL because ScenarioRunner is absent.

- [ ] **Step 3: Implement one orchestration path**

Build all components from the resolved Scenario.
Execute the exact documented stage order.
Use the same `StepOnce` in realtime and as-fast-as-possible modes.
Publish `/clock` only after accepting the complete step.
On failure, save the last valid snapshot, failed command, hashes and seed, then stop cleanly.
`simulator.launch.py` must accept vehicle, track, scenario, run mode and visualization arguments.

- [ ] **Step 4: Run headless and interactive trace comparison**

Run 1,000 steps with visualization disabled and enabled.
Compare physical, sensor, race and event traces.
Expected: Exact match.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup/launch tools
git commit -m "feat: assemble complete scenario runner"
```

### Task 7: Implement a sensor-only reference controller

**Files:**

- Create: `simulator/src/fsai_reference_controller/package.xml`
- Create: `simulator/src/fsai_reference_controller/CMakeLists.txt`
- Create: `simulator/src/fsai_reference_controller/src/reference_controller.cpp`
- Create: `simulator/src/fsai_reference_controller/test/reference_controller_test.cpp`
- Create: `simulator/src/fsai_bringup/config/reference_controller.yaml`

**Interfaces:**

- Consumes: Camera and LiDAR ConeObservationArray, IMU, wheel speed and RaceState.
- Produces: ActuationCommand without Ground Truth or map subscriptions.

- [ ] **Step 1: Write failing graph and steering tests**

```cpp
TEST(ReferenceController, SteersTowardConeMidpoints) {
  auto command = controller.Update(SymmetricLeftCurveObservations());
  EXPECT_GT(command.steering_angle_rad, 0.0);
}

TEST(ReferenceController, StopsOnFinishedRace) {
  controller.SetRaceState(RaceState::kFinished);
  auto command = controller.Update(Observations());
  EXPECT_DOUBLE_EQ(command.rear_axle_torque_nm, 0.0);
  EXPECT_GT(command.friction_brake_ratio, 0.0);
}
```

The ROS graph test must reject subscriptions below `/ground_truth` and `/map`.

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_reference_controller --event-handlers console_direct+`

Expected: FAIL because the controller package is absent.

- [ ] **Step 3: Implement the bounded reference controller**

Pair blue and yellow observations by forward station.
Compute ordered midpoint targets in the vehicle frame.
Use pure pursuit steering with configured lookahead.
Use wheel-speed proportional control for target speed and limit axle torque.
Apply braking when observations are stale or RaceState is finishing, finished or DNF.
Subscribe only to `/sensors/*` and `/race/*`.

- [ ] **Step 4: Run controller unit and graph tests**

Expected: Commands are bounded, stale data brakes, and graph audit finds no forbidden subscriptions.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_reference_controller simulator/src/fsai_bringup/config
git commit -m "test: add sensor-only reference controller"
```

### Task 8: Add the complete full-lap E2E

**Files:**

- Create: `simulator/src/fsai_bringup/test/test_full_lap.launch.py`
- Create: `simulator/src/fsai_bringup/test/test_replay.py`
- Modify: `simulator/src/fsai_bringup/CMakeLists.txt`
- Modify: `.github/workflows/humble.yml`

**Interfaces:**

- Consumes: Formal launch, reference vehicle, custom track and reference controller.
- Produces: End-user full-lap acceptance and reproducibility gate.

- [ ] **Step 1: Write the failing launch acceptance**

The launch test must start `simulator.launch.py` exactly as documented.
It must require AS transition to `DRIVING`, at least one completed lap, final `FINISHED`, no Ground Truth subscription, finite telemetry, clean process exit and all six required artifact files.
It must enforce a 120-second simulation-time limit and a 60-second wall-time limit in as-fast-as-possible mode.

- [ ] **Step 2: Run the E2E before final tuning**

Run: `colcon test --base-paths simulator/src --packages-select fsai_bringup --ctest-args -R test_full_lap --output-on-failure`

Expected: FAIL at the first unmet launch, controller, race or artifact assertion.

- [ ] **Step 3: Fix only failures exposed through the user path**

Use the E2E trace and artifacts to correct the responsible component.
Do not add Ground Truth access, test-only state overrides or relaxed NaN checks.
Record any changed controller or vehicle parameter with source and calibration status.

- [ ] **Step 4: Run full Humble validation twice**

Run the complete test suite twice with the same revision and seed.
Compare `run.json`, `events.jsonl`, resolved configurations and final snapshot byte for byte.
Compare MCAP message topic, timestamp and serialized payload sequences.
Expected: Both full-lap runs pass and compare equal.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_bringup .github/workflows/humble.yml
git commit -m "test: validate complete autonomous lap"
```

### Task 9: Prepare the independent GitHub project for publication

**Files:**

- Create: `README.md`
- Create: `docs/installation.md`
- Create: `docs/architecture.md`
- Create: `tools/tests/test_release_contract.sh`
- Modify: `THIRD_PARTY_NOTICES.md`

**Interfaces:**

- Consumes: Completed build, test and E2E entry points.
- Produces: A cloneable standalone project ready for the user's own GitHub repository.

- [ ] **Step 1: Write the failing release contract**

```bash
#!/usr/bin/env bash
set -euo pipefail
grep -q "Ubuntu 22.04" README.md
grep -q "ROS 2 Humble" README.md
grep -q "tools/bootstrap_ubuntu.sh" README.md
grep -q "read-only EUFS" docs/architecture.md
grep -q "9f5df79a03725ea7d10542fc2ce8224d90836560" THIRD_PARTY_NOTICES.md
test ! -e LICENSE
```

- [ ] **Step 2: Run the release contract**

Run: `bash tools/tests/test_release_contract.sh`

Expected: FAIL because the top-level project documentation is absent.

- [ ] **Step 3: Write installation, architecture and ownership documentation**

README must provide clone, bootstrap, build, test, headless run and Foxglove run commands.
Installation documentation must begin from a clean Ubuntu 22.04 machine.
Architecture documentation must identify EUFS as a fixed read-only dependency and this repository as an independent project.
Third-party notices must list repository, commit, license and original copyright reference.
Do not add a project license in this phase.
Without a project license, the user's original source remains all rights reserved while dependency licenses remain effective.

- [ ] **Step 4: Verify a clean clone on Ubuntu**

Clone the local repository into a new temporary directory.
Run bootstrap, build, all tests and the full-lap E2E from the documented commands.
Expected: The clean clone passes without files from the original working directory.

- [ ] **Step 5: Commit the publication-ready project files**

```bash
git add README.md docs/installation.md docs/architecture.md THIRD_PARTY_NOTICES.md tools/tests/test_release_contract.sh
git commit -m "docs: prepare standalone simulator project"
```

- [ ] **Step 6: Stop for explicit GitHub authorization**

Ask the user for the exact GitHub repository URL, desired visibility and explicit permission to push.
After authorization, verify the URL belongs to the user, set it as `origin`, show the commits to be sent, and request confirmation for the final push command.
Never reuse the EUFS remote as `origin`.

## Plan Completion Gate

The formal launch must work with visualization enabled or disabled.
The controller must complete one lap using only sensor and race topics.
RaceDirector must finish through the normal safety chain.
Required artifacts must be complete and replayable.
Repeated runs must match at every deterministic comparison point.
Foxglove must pass automated frame checks and manual visual review.
