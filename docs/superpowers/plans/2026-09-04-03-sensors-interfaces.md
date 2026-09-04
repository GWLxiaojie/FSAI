# Sensors and ROS Interfaces Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 从已接受 Ground Truth 生成可重复的 IMU、GNSS、轮速、转角、OSS、Camera 锥桶和 LiDAR 锥桶观测，并发布正式 ROS 2 接口。

**Architecture:** `GroundTruthHistory` 保存带仿真时间的真值，所有传感器按自己的采样时刻插值。
误差、故障、延迟和 RNG 属于 `SensorSuiteState`，ROS 转换只发生在 `fsai_sim2_adapter` 边界。

**Tech Stack:** C++20, Eigen3, GoogleTest, ROS 2 Humble, sensor_msgs, geometry_msgs, eufs_msgs.

**Spec:** `docs/superpowers/specs/2026-09-04-eufs-sim2-team-simulator-design.md`

## Global Constraints

传感器不得读取未接受的中间积分状态。
消息 header 使用采样时间，不使用发布时间。
延迟队列使用仿真时间。
相同 seed 必须产生相同噪声、丢包和延迟序列。
传感器 bias、RNG、延迟队列和故障状态必须可序列化。
Camera 和 LiDAR 必须发布独立观测。
融合输出只能作为可选调试输出。
Ground Truth 只能发布到 `/ground_truth/*`。
无人驾驶节点不得订阅 `/ground_truth/*`。
第一版不生成图像或点云。

---

## File Map

```text
simulator/src/fsai_sensor_models/
  include/fsai_sensor_models/ground_truth_history.hpp
  include/fsai_sensor_models/scheduler.hpp
  include/fsai_sensor_models/random_stream.hpp
  include/fsai_sensor_models/delay_queue.hpp
  include/fsai_sensor_models/inertial_sensors.hpp
  include/fsai_sensor_models/vehicle_sensors.hpp
  include/fsai_sensor_models/cone_sensor.hpp
  include/fsai_sensor_models/sensor_suite.hpp
  include/fsai_sensor_models/sensor_profile_loader.hpp
  src/
  test/
simulator/src/fsai_interfaces/msg/
  ConeObservation.msg
  ConeObservationArray.msg
  SteeringReport.msg
  SensorStatus.msg
simulator/src/fsai_sim2_adapter/
  include/fsai_sim2_adapter/sensor_publishers.hpp
  src/sensor_publishers.cpp
  test/
```

### Task 1: Implement GroundTruthHistory interpolation

**Files:**

- Create: `simulator/src/fsai_sensor_models/package.xml`
- Create: `simulator/src/fsai_sensor_models/CMakeLists.txt`
- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/ground_truth_history.hpp`
- Create: `simulator/src/fsai_sensor_models/src/ground_truth_history.cpp`
- Create: `simulator/src/fsai_sensor_models/test/ground_truth_history_test.cpp`

**Interfaces:**

- Consumes: Timestamped `fsai::sim::GroundTruth` after accepted plant steps.
- Produces: `Push(SimTime, GroundTruth)` and `Sample(SimTime)`.

- [ ] **Step 1: Write failing boundary and interpolation tests**

```cpp
TEST(GroundTruthHistory, InterpolatesAtSensorSampleTime) {
  history.Push(0ms, TruthAt(0.0, Degrees(179.0)));
  history.Push(10ms, TruthAt(10.0, Degrees(-179.0)));
  auto sample = history.Sample(5ms);
  EXPECT_DOUBLE_EQ(sample.position_map_m.x(), 5.0);
  EXPECT_NEAR(std::abs(sample.yaw_rad), std::numbers::pi, 1e-12);
}

TEST(GroundTruthHistory, RejectsFutureAndExpiredQueries) {
  EXPECT_THROW(history.Sample(11ms), TimeRangeError);
  EXPECT_THROW(history.Sample(-1ms), TimeRangeError);
}
```

- [ ] **Step 2: Run the focused test**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sensor_models --ctest-args -R ground_truth_history_test --output-on-failure`

Expected: FAIL because the history type is missing.

- [ ] **Step 3: Implement deterministic bounded history**

Require strictly increasing timestamps.
Interpolate position, velocity, acceleration, wheel speed, force and steering linearly.
Interpolate yaw on the shortest wrapped path.
Use zero-order hold for contact modes and discrete events.
Retain enough history for the maximum configured sensor delay plus two outer steps.
Reject a query outside retained bounds.

- [ ] **Step 4: Run tests and a 10,000-sample sweep**

Expected: Every exact endpoint equals the stored sample and every interpolated value is finite.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sensor_models
git commit -m "feat: add ground truth history"
```

### Task 2: Implement scheduling, RNG and delay state

**Files:**

- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/scheduler.hpp`
- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/random_stream.hpp`
- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/delay_queue.hpp`
- Create: `simulator/src/fsai_sensor_models/src/scheduler.cpp`
- Create: `simulator/src/fsai_sensor_models/src/random_stream.cpp`
- Create: `simulator/src/fsai_sensor_models/test/scheduler_test.cpp`
- Create: `simulator/src/fsai_sensor_models/test/random_stream_test.cpp`
- Create: `simulator/src/fsai_sensor_models/test/delay_queue_test.cpp`

**Interfaces:**

- Consumes: Integer nanosecond periods, suite seed and sampled messages.
- Produces: Exact sample times, named random streams and release-time ordered messages.

- [ ] **Step 1: Write failing deterministic state tests**

```cpp
TEST(Scheduler, DoesNotDriftAtNonOuterRate) {
  Scheduler scheduler(10ms, 0ns);
  EXPECT_EQ(scheduler.DueTimes(0ms, 25ms),
            std::vector<SimTime>{0ms, 10ms, 20ms});
}

TEST(RandomStream, SnapshotRestoresSequence) {
  RandomStream random(42u, "imu");
  random.Normal();
  auto state = random.State();
  auto expected = random.Normal();
  random.Restore(state);
  EXPECT_DOUBLE_EQ(random.Normal(), expected);
}

TEST(DelayQueue, ReleasesBySimulationTimeThenSequence) {
  queue.Push(Message(2), 15ms);
  queue.Push(Message(1), 15ms);
  EXPECT_EQ(queue.ReleaseDue(15ms), Messages({1, 2}));
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sensor_models --ctest-args -R 'scheduler|random_stream|delay_queue' --output-on-failure`

Expected: FAIL because the utilities are missing.

- [ ] **Step 3: Implement exact state contracts**

Represent periods and timestamps as signed 64-bit nanoseconds.
Derive each stream seed by SHA-256 of suite seed plus sensor instance name.
Use a fixed PCG32 implementation and a fixed Marsaglia polar normal sampler.
Serialize generator words, cached normal sample and its valid flag.
Order delayed messages by release time, sample time and sequence number.
Reject negative delay and a period not representable in integer nanoseconds.

- [ ] **Step 4: Run utility and snapshot tests**

Generate 100,000 values, snapshot at 50,000 and require the restored suffix to compare exactly.
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sensor_models
git commit -m "feat: add deterministic sensor timing"
```

### Task 3: Implement IMU with mounting and bias state

**Files:**

- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/inertial_sensors.hpp`
- Create: `simulator/src/fsai_sensor_models/src/imu_sensor.cpp`
- Create: `simulator/src/fsai_sensor_models/test/imu_sensor_test.cpp`

**Interfaces:**

- Consumes: Interpolated Ground Truth, rigid mounting transform, IMU parameters and `ImuSensorState`.
- Produces: `SensorEmission<ImuSample> SampleImu(...)`.

- [ ] **Step 1: Write failing physical and time tests**

```cpp
TEST(ImuSensor, UsesCgPhysicalAcceleration) {
  auto out = sensor.Sample(TruthWithBodyAcceleration(2.0, -1.0), 5ms);
  EXPECT_NEAR(out.sample.linear_acceleration_mps2.x(), 2.0, 1e-12);
  EXPECT_NEAR(out.sample.linear_acceleration_mps2.y(), -1.0, 1e-12);
}

TEST(ImuSensor, AppliesLeverArmAcceleration) {
  auto out = sensor.Sample(
      TruthWithYawAcceleration(3.0), MountAt(1.0, 0.0, 0.0), 5ms);
  EXPECT_NEAR(out.sample.linear_acceleration_mps2.y(), 3.0, 1e-12);
}

TEST(ImuSensor, HeaderTimePrecedesDelayedRelease) {
  auto out = sensor.Sample(Truth(), 10ms);
  EXPECT_EQ(out.sample_time, 10ms);
  EXPECT_EQ(out.release_time, 15ms);
}
```

- [ ] **Step 2: Run the focused test**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sensor_models --ctest-args -R imu_sensor_test --output-on-failure`

Expected: FAIL because the IMU model is missing.

- [ ] **Step 3: Implement the documented processing order**

Apply sample-time interpolation, mounting translation and rotation, ideal observation, bias random walk, white noise, scale, quantization, saturation, dropout and delay in that order.
Use CG physical acceleration from Ground Truth.
Add centripetal and angular-acceleration lever-arm terms before rotating to the sensor frame.
Serialize gyro and accelerometer bias, RNG and scheduler state.
Use 200 Hz default frequency.

- [ ] **Step 4: Run deterministic IMU tests**

Run noise-disabled analytic tests and two equal-seed 60-second sequences.
Expected: Analytic tolerance passes and sequences compare exactly.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sensor_models
git commit -m "feat: add deterministic IMU model"
```

### Task 4: Implement wheel, steering, GNSS and OSS sensors

**Files:**

- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/vehicle_sensors.hpp`
- Create: `simulator/src/fsai_sensor_models/src/wheel_speed_sensor.cpp`
- Create: `simulator/src/fsai_sensor_models/src/steering_sensor.cpp`
- Create: `simulator/src/fsai_sensor_models/src/gnss_sensor.cpp`
- Create: `simulator/src/fsai_sensor_models/src/oss_sensor.cpp`
- Create: `simulator/src/fsai_sensor_models/test/vehicle_sensors_test.cpp`

**Interfaces:**

- Consumes: Ground Truth history and per-sensor parameters.
- Produces: `WheelSpeedSample`, `SteeringSample`, `GnssSample` and `OssSample`.

- [ ] **Step 1: Write failing measurement tests**

```cpp
TEST(WheelSpeedSensor, QuantizesIndependentWheels) {
  auto out = sensor.Sample(TruthWithWheelSpeeds({1.01, 2.02, 3.03, 4.04}), 0ms);
  EXPECT_EQ(out.value_ticks, std::array<int>{101, 202, 303, 404});
}

TEST(SteeringSensor, ReportsActualNotCommandedAngle) {
  auto out = sensor.Sample(TruthWithSteering(0.12), 0ms);
  EXPECT_DOUBLE_EQ(out.steering_angle_rad, 0.12);
}

TEST(GnssSensor, EmitsNoFixDuringConfiguredOutage) {
  sensor.SetFailure(FailureMode::kNoFix, 1s, 2s);
  EXPECT_FALSE(sensor.Sample(Truth(), 1500ms).has_fix);
}

TEST(OssSensor, AppliesMountingRotation) {
  auto out = sensor.Sample(TruthWithBodyVelocity(2.0, 1.0), RotatedMount(90deg), 0ms);
  EXPECT_NEAR(out.velocity_sensor_mps.x(), 1.0, 1e-12);
  EXPECT_NEAR(out.velocity_sensor_mps.y(), -2.0, 1e-12);
}
```

- [ ] **Step 2: Run the focused test**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sensor_models --ctest-args -R vehicle_sensors_test --output-on-failure`

Expected: FAIL because the four sensors are absent.

- [ ] **Step 3: Implement sensor-specific errors**

Wheel speed uses 100 Hz, encoder quantization, fixed delay and configurable dropped pulses.
Steering uses 100 Hz, bias, quantization, saturation and delay.
GNSS uses 10 Hz, map-frame position noise, slow bias and explicit fix status.
OSS uses 100 Hz, mounting transform, longitudinal and lateral velocity noise.
Each instance owns a named random stream and complete serializable state.

- [ ] **Step 4: Run analytic and equal-seed tests**

Expected: All ideal measurements match Ground Truth transforms and equal-seed outputs compare exactly.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sensor_models
git commit -m "feat: add vehicle feedback sensors"
```

### Task 5: Implement semantic Camera and LiDAR cone sensors

**Files:**

- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/cone_sensor.hpp`
- Create: `simulator/src/fsai_sensor_models/src/cone_sensor.cpp`
- Create: `simulator/src/fsai_sensor_models/test/cone_sensor_test.cpp`

**Interfaces:**

- Consumes: Immutable TrackDefinition, sampled vehicle pose, mounting transform and `ConeSensorParameters`.
- Produces: Timestamped arrays of sensor-frame `ConeObservation`.

- [ ] **Step 1: Write failing visibility and independence tests**

```cpp
TEST(ConeSensor, FiltersByMountingFovAndRange) {
  auto output = camera.Sample(track, VehicleAtOrigin(), 0ms);
  EXPECT_THAT(output.cones, ElementsAre(ConeId("front_near")));
}

TEST(ConeSensor, CameraAndLidarUseIndependentRandomStreams) {
  auto camera_only = RunCamera(seed);
  auto with_lidar = RunCameraAndLidar(seed);
  EXPECT_EQ(camera_only, with_lidar.camera);
}

TEST(ConeSensor, AppliesConfiguredColourConfusion) {
  auto output = always_confused_camera.Sample(track, VehicleAtOrigin(), 0ms);
  EXPECT_EQ(output.cones.front().colour, ConeColour::kUnknown);
}
```

- [ ] **Step 2: Run the focused test**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sensor_models --ctest-args -R cone_sensor_test --output-on-failure`

Expected: FAIL because `ConeSensor` is absent.

- [ ] **Step 3: Implement semantic observation models**

Transform map cones into the sensor frame before range and horizontal FOV checks.
Camera defaults to 20 Hz and applies detection probability, colour confusion matrix and Cartesian position noise.
LiDAR defaults to 20 Hz and applies detection probability plus range and bearing noise.
Sort output by stable cone ID before applying sequence numbers.
Never emit raw image or point-cloud fields.
Keep camera and LiDAR states independent.

- [ ] **Step 4: Run geometric, statistical and replay tests**

Use deterministic edge fixtures at exact range and FOV boundaries.
Run 100,000 samples and require observed rates within precomputed binomial confidence limits.
Restore a snapshot and require exact suffix replay.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sensor_models
git commit -m "feat: add semantic cone sensors"
```

### Task 6: Assemble SensorSuite and fault handling

**Files:**

- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/sensor_suite.hpp`
- Create: `simulator/src/fsai_sensor_models/include/fsai_sensor_models/sensor_profile_loader.hpp`
- Create: `simulator/src/fsai_sensor_models/src/sensor_suite.cpp`
- Create: `simulator/src/fsai_sensor_models/src/sensor_profile_loader.cpp`
- Create: `simulator/src/fsai_sensor_models/test/sensor_suite_test.cpp`
- Create: `simulator/src/fsai_sensor_models/test/sensor_profile_loader_test.cpp`

**Interfaces:**

- Consumes: Ground Truth after every plant step, TrackDefinition and `SensorSuiteState`.
- Produces: `SensorBatch Advance(SimTime, GroundTruth)` and snapshot restore.

- [ ] **Step 1: Write failing cross-rate and reset tests**

```cpp
TEST(SensorSuite, EmitsEveryDueSampleAcrossOuterStep) {
  auto first = suite.Advance(0ms, Truth());
  auto second = suite.Advance(25ms, Truth());
  EXPECT_EQ(first.ImuTimes(), Times({0ms}));
  EXPECT_EQ(first.GnssTimes(), Times({0ms}));
  EXPECT_EQ(second.ImuTimes(), Times({5ms, 10ms, 15ms, 20ms, 25ms}));
  EXPECT_TRUE(second.GnssTimes().empty());
}

TEST(SensorSuite, ResetClearsObservableState) {
  suite.Advance(1s, Truth());
  suite.Reset(seed);
  EXPECT_EQ(suite.State(), InitialSuiteState(seed));
}

TEST(SensorProfile, RejectsUnknownUnitAndMissingCalibrationStatus) {
  EXPECT_THROW(LoadSensorProfile(InvalidProfile()), SensorConfigError);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sensor_models --ctest-args -R sensor_suite_test --output-on-failure`

Expected: FAIL because suite orchestration is absent.

- [ ] **Step 3: Implement fixed ordering**

Within one outer step, order work by sample time, sensor configuration order and sequence number.
Push accepted Ground Truth before sampling.
Release delayed messages only after all samples due at that simulation time are generated.
Record failure start, failure end, dropout and recovery events.
Include every child state in `SensorSuiteState`.
Load `sensors.yaml` with `schema_version: 1` and reject unknown fields.
Require every numeric sensor parameter to provide value, unit, valid range, source and calibration status.
Build an immutable `SensorSuiteParameters` only after all sensor instances validate.

- [ ] **Step 4: Run full sensor tests**

Expected: All frequencies, delays, failures and resets pass with no hidden state.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sensor_models
git commit -m "feat: assemble sensor suite"
```

### Task 7: Publish ROS messages without leaking Ground Truth

**Files:**

- Create: `simulator/src/fsai_interfaces/msg/ConeObservation.msg`
- Create: `simulator/src/fsai_interfaces/msg/ConeObservationArray.msg`
- Create: `simulator/src/fsai_interfaces/msg/SteeringReport.msg`
- Create: `simulator/src/fsai_interfaces/msg/SensorStatus.msg`
- Modify: `simulator/src/fsai_interfaces/package.xml`
- Modify: `simulator/src/fsai_interfaces/CMakeLists.txt`
- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/sensor_publishers.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/sensor_publishers.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/sensor_publishers_test.cpp`
- Create: `simulator/src/fsai_bringup/vehicles/reference_bicycle/sensors.yaml`

**Interfaces:**

- Consumes: SensorBatch and simulation clock.
- Produces: `/sensors/*`, optional EUFS-compatible topics, and `/ground_truth/*` only when debug is enabled.

- [ ] **Step 1: Write failing timestamp and namespace tests**

The ROS test must publish a delayed IMU sampled at 10 ms and released at 15 ms.
It must assert that `header.stamp` equals 10 ms.
It must enumerate publishers and assert every non-debug sensor topic begins with `/sensors/`.
It must start with `publish_ground_truth=false` and assert no `/ground_truth/*` publisher exists.

- [ ] **Step 2: Run adapter tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R sensor_publishers_test --output-on-failure`

Expected: FAIL because publishers are missing.

- [ ] **Step 3: Implement message conversion and QoS**

Publish IMU as `sensor_msgs/Imu`, GNSS as `sensor_msgs/NavSatFix`, wheel speed and steering through `fsai_interfaces`, OSS as `geometry_msgs/TwistStamped`, and cones as `ConeObservationArray`.
Convert map-frame GNSS position with a configured WGS84 origin at the ROS boundary.
Use sensor-data QoS for observations and transient-local QoS only for static map debug output.
Expose EUFS message conversions behind `enable_eufs_compatibility`.

- [ ] **Step 4: Run launch test and subscription audit**

Start the formal test controller.
Inspect its graph with `rclcpp::Node::get_subscriber_names_and_types_by_node`.
Expected: No subscription resolves below `/ground_truth`.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_interfaces simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup/vehicles/reference_bicycle/sensors.yaml
git commit -m "feat: publish vehicle sensor interfaces"
```

## Plan Completion Gate

Every sensor must pass ideal analytic tests.
Equal seeds and snapshots must reproduce exact sequences.
Camera and LiDAR must remain independent.
Header time must always be sample time.
The formal controller graph must contain no Ground Truth subscription.
