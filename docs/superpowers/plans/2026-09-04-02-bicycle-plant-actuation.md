# Bicycle Plant and Actuation Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 交付无 ROS 的动态自行车 plant、执行器和安全链，并通过 `FsaiCoreAdapter` 接入自有仿真节点。

**Architecture:** `fsai_sim_core` 以不可变参数和显式 `PlantState` 实现确定性更新。
`SafetyChain` 先生成安全命令，`ActuatorModel` 生成实际输入，`HybridIntegrator` 唯一推进状态，`GroundTruthBuilder` 只读取接受后的解。

**Tech Stack:** C++20, Eigen3, OpenSSL libcrypto, GoogleTest, yaml-cpp only in adapter, ROS 2 Humble adapter.

**Spec:** `docs/superpowers/specs/2026-09-04-eufs-sim2-team-simulator-design.md`

## Global Constraints

`fsai_sim_core` 不得包含 ROS、EUFS、YAML、文件 I/O、GUI 或 wall clock 类型。
核心坐标系是地图 X/Y、车体 x 前 y 左 z 上、yaw 逆时针为正。
转角是前轴中心等效道路轮角，左转为正，单位 rad。
轴扭矩是车轮侧总扭矩，正值驱动车辆前进。
摩擦制动命令范围固定为 `[0, 1]`。
plant 内部步长是 1 ms，外层步长是 5 ms。
默认积分器是固定步 RK4。
正式侧向力使用参数化 Pacejka B、C、D、E。
静止转角不得产生轮胎力。
禁止用积分后截断纵向速度掩盖穿过零速。
每个可影响后续结果的状态必须可序列化。

---

## File Map

```text
simulator/src/fsai_sim_core/
  include/fsai_sim_core/types.hpp
  include/fsai_sim_core/parameters.hpp
  include/fsai_sim_core/safety_chain.hpp
  include/fsai_sim_core/actuator_model.hpp
  include/fsai_sim_core/dynamics_backend.hpp
  include/fsai_sim_core/bicycle_backend.hpp
  include/fsai_sim_core/hybrid_integrator.hpp
  include/fsai_sim_core/ground_truth_builder.hpp
  include/fsai_sim_core/plant.hpp
  src/
  test/
simulator/src/fsai_sim2_adapter/
  include/fsai_sim2_adapter/fsai_core_adapter.hpp
  include/fsai_sim2_adapter/vehicle_profile_loader.hpp
  src/
  test/
simulator/src/fsai_bringup/
  vehicles/reference_bicycle/
  scenarios/
```

### Task 1: Define stable core DTOs and parameter validation

**Files:**

- Create: `simulator/src/fsai_sim_core/package.xml`
- Create: `simulator/src/fsai_sim_core/CMakeLists.txt`
- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/types.hpp`
- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/parameters.hpp`
- Create: `simulator/src/fsai_sim_core/src/parameters.cpp`
- Create: `simulator/src/fsai_sim_core/test/types_test.cpp`
- Create: `simulator/src/fsai_sim_core/test/parameters_test.cpp`

**Interfaces:**

- Consumes: SI values constructed by the adapter.
- Produces: `Command`, `PlantState`, `VehicleParameters`, `GroundTruth`, `StepResult` and `ValidateParameters`.

- [ ] **Step 1: Write failing type and validation tests**

```cpp
TEST(Command, RejectsBrakeOutsideUnitInterval) {
  Command command{};
  command.friction_brake_ratio = 1.01;
  EXPECT_THROW(Validate(command), ValidationError);
}

TEST(VehicleParameters, DerivesAxleDistances) {
  VehicleParameters p = ValidParameters();
  p.wheelbase_m = 1.6;
  p.front_static_load_fraction = 0.45;
  EXPECT_DOUBLE_EQ(Derive(p).cg_to_rear_axle_m, 0.72);
  EXPECT_DOUBLE_EQ(Derive(p).cg_to_front_axle_m, 0.88);
}

TEST(VehicleParameters, HashIsStable) {
  EXPECT_EQ(ParameterHash(ValidParameters()),
            ParameterHash(ValidParameters()));
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R 'types|parameters' --output-on-failure`

Expected: FAIL because the package and DTOs do not exist.

- [ ] **Step 3: Implement exact public DTO fields**

```cpp
struct Command final {
  double steering_angle_rad{};
  double front_axle_torque_nm{};
  double rear_axle_torque_nm{};
  double friction_brake_ratio{};
};

struct ChassisState final {
  double x_m{}, y_m{}, yaw_rad{};
  double u_mps{}, v_mps{}, yaw_rate_radps{};
};

struct ActuatorState final {
  double steering_angle_rad{};
  double front_axle_torque_nm{};
  double rear_axle_torque_nm{};
  double friction_brake_ratio{};
};

enum class ContactMode : std::uint8_t {
  kKinematic, kStick, kTransition, kSlip, kBrakeHold
};

struct WheelState final {
  double omega_radps{};
  ContactMode contact_mode{ContactMode::kKinematic};
};

struct PlantState final {
  std::uint32_t schema_version{1};
  std::string backend_id{"fsai_bicycle"};
  std::string backend_revision{"1"};
  std::string parameter_hash;
  ChassisState chassis;
  ActuatorState actuator;
  std::array<WheelState, 4> wheels;
  std::vector<double> backend_state;
};
```

`VehicleParameters` must contain mass, yaw inertia, wheelbase, front static load fraction, effective tyre radius, gravity, air density, drag area, lift area, rolling resistance, maximum steering angle, maximum steering rate, maximum axle torque, maximum brake torque, Pacejka B/C/D/E for each axle, low-speed thresholds and timeout duration.
All validation errors must name the field and rejected value.
Hash canonical little-endian IEEE-754 bytes with SHA-256.

- [ ] **Step 4: Run tests and header isolation check**

Run:

```bash
colcon test --base-paths simulator/src --packages-select fsai_sim_core --event-handlers console_direct+
cmake -S simulator/src/fsai_sim_core -B /tmp/fsai-core-header-check -DFSIA_CORE_HEADER_CHECK=ON
cmake --build /tmp/fsai-core-header-check
```

Expected: PASS without a ROS environment sourced.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: define vehicle plant contracts"
```

### Task 2: Implement the safety chain and actuator dynamics

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/safety_chain.hpp`
- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/actuator_model.hpp`
- Create: `simulator/src/fsai_sim_core/src/safety_chain.cpp`
- Create: `simulator/src/fsai_sim_core/src/actuator_model.cpp`
- Create: `simulator/src/fsai_sim_core/test/safety_chain_test.cpp`
- Create: `simulator/src/fsai_sim_core/test/actuator_model_test.cpp`

**Interfaces:**

- Consumes: `Command`, `SafetyInput`, `SafetyState`, `ActuatorState`, parameters and duration.
- Produces: `SafetyResult Resolve(...)` and `ActuatorResult Update(...)`.

- [ ] **Step 1: Write failing safety and rate-limit tests**

```cpp
TEST(SafetyChain, BlocksDriveBeforeDrivingState) {
  auto result = Resolve(command, SafetyInput{.as_driving = false}, state, params);
  EXPECT_DOUBLE_EQ(result.command.front_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(result.command.rear_axle_torque_nm, 0.0);
}

TEST(SafetyChain, TimeoutAppliesConfiguredSafeBrake) {
  auto input = SafetyInput{.as_driving = true, .command_age = 101ms};
  auto result = Resolve(command, input, state, params);
  EXPECT_DOUBLE_EQ(result.command.rear_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(result.command.friction_brake_ratio,
                   params.timeout_brake_ratio);
}

TEST(ActuatorModel, LimitsSteeringRate) {
  auto result = UpdateActuator(
      state, Command{.steering_angle_rad = 0.5}, 10ms, params);
  EXPECT_NEAR(result.state.steering_angle_rad,
              params.max_steering_rate_radps * 0.01, 1e-12);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R 'safety_chain|actuator_model' --output-on-failure`

Expected: FAIL because both models are absent.

- [ ] **Step 3: Implement deterministic safety and actuators**

`SafetyState` must contain latched EBS, last accepted command time, delay buffers and filter states.
Priority must be EBS, timeout, non-driving state, then driver command.
EBS must command zero drive torque and `ebs_brake_ratio` until a complete simulation reset.
Clamp steering, torque and brake before rate limiting.
Pure delays must store timestamped inputs in `ActuatorState`.
First-order responses may only run when the profile provides a calibrated positive time constant.
Regenerative torque and friction brake must be combined once in the downstream force request.

- [ ] **Step 4: Run all core tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --event-handlers console_direct+`

Expected: PASS, including exact EBS latch and timeout events.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: add actuator safety chain"
```

### Task 3: Implement Pacejka bicycle force evaluation

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/dynamics_backend.hpp`
- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/bicycle_backend.hpp`
- Create: `simulator/src/fsai_sim_core/src/bicycle_backend.cpp`
- Create: `simulator/src/fsai_sim_core/test/bicycle_backend_test.cpp`

**Interfaces:**

- Consumes: `ChassisState`, `ActuatorState` and derived parameters.
- Produces: `DynamicsEvaluation EvaluateBicycle(...)` with derivatives and force diagnostics.

- [ ] **Step 1: Write failing force-sign and static tests**

```cpp
TEST(BicycleBackend, LeftTurnProducesPositiveLateralForce) {
  auto state = MovingState(10.0);
  auto input = ActuatorState{.steering_angle_rad = 0.1};
  auto out = EvaluateBicycle(state, input, params);
  EXPECT_GT(out.front_lateral_force_n, 0.0);
  EXPECT_GT(out.derivative.yaw_rate_radps2, 0.0);
}

TEST(BicycleBackend, StaticSteeringProducesNoTyreForce) {
  auto out = EvaluateBicycle(ChassisState{}, SteeredInput(), params);
  EXPECT_DOUBLE_EQ(out.front_lateral_force_n, 0.0);
  EXPECT_DOUBLE_EQ(out.rear_lateral_force_n, 0.0);
}
```

- [ ] **Step 2: Run the focused test**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R bicycle_backend_test --output-on-failure`

Expected: FAIL because `EvaluateBicycle` is absent.

- [ ] **Step 3: Implement the exact equations**

Use:

```text
alpha_f = atan2(v + l_f r, u_regularized) - delta
alpha_r = atan2(v - l_r r, u_regularized)
Fy(alpha) = -D sin(C atan(B alpha - E(B alpha - atan(B alpha))))
F_drive = (T_front + T_rear) / R_effective
F_drag = 0.5 rho CdA u abs(u)
F_roll = Crr mass g tanh(u / rolling_smoothing_speed)
X_dot = u cos(yaw) - v sin(yaw)
Y_dot = u sin(yaw) + v cos(yaw)
yaw_dot = r
u_dot = r v + sum(Fx_body) / mass
v_dot = -r u + sum(Fy_body) / mass
r_dot = sum(Mz) / Iz
```

Rotate front axle forces into the body frame.
Set slip angles and lateral forces to zero below the configured static-speed threshold.
Use static axle loads for phase one.
Report every force, slip angle and moment in `DynamicsEvaluation`.

- [ ] **Step 4: Run force tests and finite-value sweep**

Run the focused tests, then evaluate speeds from 0 to 40 m/s, steering from negative limit to positive limit, and legal torque and brake corners.
Expected: Every result is finite and force signs match the coordinate convention.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: add Pacejka bicycle dynamics"
```

### Task 4: Implement RK4 and zero-speed hybrid events

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/hybrid_integrator.hpp`
- Create: `simulator/src/fsai_sim_core/src/hybrid_integrator.cpp`
- Create: `simulator/src/fsai_sim_core/test/hybrid_integrator_test.cpp`
- Modify: `simulator/src/fsai_sim_core/include/fsai_sim_core/types.hpp`

**Interfaces:**

- Consumes: DynamicsBackend, chassis state, applied input, 5 ms outer step and 1 ms internal step.
- Produces: `IntegrationResult Integrate(...)` and low-speed mode events.

- [ ] **Step 1: Write failing convergence and stop-event tests**

```cpp
TEST(HybridIntegrator, UsesFiveOneMillisecondSubsteps) {
  auto result = integrator.Integrate(state, input, 5ms);
  EXPECT_EQ(result.diagnostics.internal_steps, 5u);
}

TEST(HybridIntegrator, LocatesStopWithoutReverseOvershoot) {
  auto state = MovingState(0.05);
  auto result = integrator.Integrate(state, FullBrake(), 100ms);
  EXPECT_DOUBLE_EQ(result.state.u_mps, 0.0);
  EXPECT_EQ(result.state.wheels[2].contact_mode, ContactMode::kBrakeHold);
  EXPECT_EQ(result.events.front().type, EventType::kVehicleStopped);
}

TEST(HybridIntegrator, RK4ConvergesAtFourthOrder) {
  EXPECT_LT(ErrorAtStep(0.5ms), ErrorAtStep(1ms) / 12.0);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R hybrid_integrator_test --output-on-failure`

Expected: FAIL because the integrator is absent.

- [ ] **Step 3: Implement continuous and discrete stepping**

Use fixed-step RK4 for continuous chassis state.
Require outer duration to be an integer multiple of 1 ms.
When a substep brackets `u=0` under net braking, locate the first zero crossing by bisection to 1 ns, integrate to the event, set `u=0` through the unilateral hold constraint, and integrate the remaining time in `kBrakeHold`.
Release hold only when forward drive force exceeds brake, rolling and configured release margin.
In kinematic low-speed mode derive lateral velocity and yaw rate consistently from steering without overwriting world pose.
Derive phase-one wheel speeds from accepted wheel-center velocities after every substep.

- [ ] **Step 4: Run convergence, stop and long-horizon tests**

Run all `fsai_sim_core` tests.
Run a 120-second constant command scenario and require finite state, bounded energy and no unexpected reverse motion.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: add hybrid RK4 integration"
```

### Task 5: Assemble Plant, GroundTruth and snapshot state

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/ground_truth_builder.hpp`
- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/plant.hpp`
- Create: `simulator/src/fsai_sim_core/src/ground_truth_builder.cpp`
- Create: `simulator/src/fsai_sim_core/src/plant.cpp`
- Create: `simulator/src/fsai_sim_core/test/plant_test.cpp`
- Create: `simulator/src/fsai_sim_core/test/snapshot_test.cpp`

**Interfaces:**

- Consumes: Safe Command, ActuatorModel, HybridIntegrator and immutable parameters.
- Produces: `Plant::Update` and serializable `PlantState`.

- [ ] **Step 1: Write failing orchestration and acceleration tests**

```cpp
TEST(Plant, UsesActuatorThenIntegratorThenGroundTruth) {
  auto result = plant.Update(state, command, 5ms);
  EXPECT_EQ(result.diagnostics.stage_order,
            std::vector<std::string>({
              "actuator", "integrator", "ground_truth"}));
}

TEST(GroundTruth, ReportsPhysicalCgAcceleration) {
  auto truth = BuildGroundTruth(accepted_state, evaluation);
  EXPECT_NEAR(truth.acceleration_body_mps2.x(),
              evaluation.derivative.u_mps2 -
              accepted_state.yaw_rate_radps * accepted_state.v_mps, 1e-12);
  EXPECT_NEAR(truth.acceleration_body_mps2.y(),
              evaluation.derivative.v_mps2 +
              accepted_state.yaw_rate_radps * accepted_state.u_mps, 1e-12);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R 'plant|snapshot' --output-on-failure`

Expected: FAIL because `Plant` and builder are absent.

- [ ] **Step 3: Implement the public update contract**

```cpp
class Plant final {
 public:
  explicit Plant(VehicleParameters parameters);
  StepResult Update(
      const PlantState& state,
      const Command& command,
      Duration dt) const;
 private:
  const VehicleParameters parameters_;
  ActuatorModel actuators_;
  HybridIntegrator integrator_;
  GroundTruthBuilder truth_builder_;
};
```

Reject schema, backend revision and parameter hash mismatches before stepping.
Return the next state, Ground Truth, ordered events and diagnostics together.
Serialize every actuator delay, filter and contact state.
Snapshot restore must reject a different parameter hash.

- [ ] **Step 4: Verify deterministic snapshot replay**

Advance 100 steps, snapshot, advance another 100 steps, restore, repeat the second 100 commands and compare `StepResult` sequences byte for byte.
Expected: Exact match.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: assemble deterministic vehicle plant"
```

### Task 6: Load vehicle profiles outside the core

**Files:**

- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/vehicle_profile_loader.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/vehicle_profile_loader.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/vehicle_profile_loader_test.cpp`
- Create: `simulator/src/fsai_bringup/vehicles/reference_bicycle/vehicle.yaml`
- Create: `simulator/src/fsai_bringup/vehicles/reference_bicycle/actuators.yaml`
- Create: `simulator/src/fsai_bringup/vehicles/reference_bicycle/tyres.yaml`
- Create: `simulator/src/fsai_bringup/vehicles/reference_bicycle/aero.yaml`

**Interfaces:**

- Consumes: Versioned YAML files with SI values, source, range and calibration status.
- Produces: Immutable `VehicleParameters LoadVehicleProfile(path)`.

- [ ] **Step 1: Write failing valid and invalid profile tests**

```cpp
TEST(VehicleProfileLoader, LoadsAndHashesReferenceProfile) {
  const auto first = LoadVehicleProfile(ReferenceProfilePath());
  const auto second = LoadVehicleProfile(ReferenceProfilePath());
  EXPECT_EQ(first.schema_version, 1u);
  EXPECT_EQ(ParameterHash(first), ParameterHash(second));

  auto changed = first;
  changed.mass_kg += 1.0;
  EXPECT_NE(ParameterHash(first), ParameterHash(changed));
}

TEST(VehicleProfileLoader, ReportsFileFieldAndValue) {
  EXPECT_THAT(
      CaptureError(InvalidMassProfile()),
      HasSubstr("vehicle.yaml: mass_kg: -1"));
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R vehicle_profile_loader_test --output-on-failure`

Expected: FAIL because the loader is absent.

- [ ] **Step 3: Implement strict YAML loading**

Require `schema_version: 1`.
Require every parameter to provide `value`, `unit`, `source`, `valid_min`, `valid_max` and `calibration_status`.
Reject unknown fields, duplicate semantic parameters, non-finite values and unit mismatches.
Construct `VehicleParameters` only after all files validate.
Log the resolved parameter hash.
Use the following explicit phase-one reference values, all marked `reference_unvalidated` and sourced from the pinned EUFS `DynamicBicycle/ads-dv-calculated.yaml` unless the row says `derived`.

| Parameter | Value | Unit | Source note |
|---|---:|---|---|
| Mass | 300.0 | kg | EUFS reference configuration |
| Gravity | 9.81 | m/s^2 | EUFS reference configuration |
| Yaw inertia | 172.44 | kg m^2 | EUFS reference configuration |
| Wheelbase | 1.53 | m | EUFS reference configuration |
| Front static load fraction | 0.5 | 1 | EUFS reference configuration |
| Front track | 1.2 | m | EUFS reference configuration |
| Rear track | 1.2 | m | EUFS reference configuration |
| Effective tyre radius | 0.2525 | m | EUFS reference configuration |
| Rolling resistance coefficient | 0.02 | 1 | EUFS reference configuration |
| Maximum steering angle | 0.384 | rad | EUFS reference configuration |
| Maximum steering rate | 0.39 | rad/s | EUFS reference configuration |
| Front maximum drive torque | 0.0 | N m | Reference rear-wheel-drive choice |
| Rear maximum drive torque | 393.9 | N m | Derived from 300 kg, 5.2 m/s^2 and 0.2525 m |
| Maximum friction brake torque | 393.9 | N m | Derived from the EUFS 5.2 m/s^2 deceleration limit |
| Pacejka B | 12.56 | 1/rad | EUFS reference configuration |
| Pacejka C | 1.4 | 1 | EUFS reference configuration |
| Pacejka D | 1.0 | axle-load multiplier | EUFS reference configuration |
| Pacejka E | 0.0 | 1 | Explicit phase-one reference choice |
| Lumped drag coefficient | 0.5 | N/(m/s)^2 | EUFS reference configuration |
| Lumped downforce coefficient | 0.3 | N/(m/s)^2 | EUFS reference configuration |
| Command timeout | 0.1 | s | Safety contract |
| Timeout brake command | 0.5 | 1 | Safety contract |
| EBS brake command | 1.0 | 1 | Safety contract |
| Static-mode speed threshold | 0.05 | m/s | Hybrid integrator contract |

The reference values are simulator smoke-test inputs, not validated parameters for the team car.
Replacing them with measured team values must not require recompiling `fsai_sim_core`.

- [ ] **Step 4: Run adapter and core tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core fsai_sim2_adapter --event-handlers console_direct+ --return-code-on-test-failure`

Expected: Reference profile loads, every invalid fixture reports file and field, and `fsai_sim_core` still builds without yaml-cpp.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup/vehicles
git commit -m "feat: load versioned vehicle profiles"
```

### Task 7: Implement FsaiCoreAdapter and physical ROS commands

**Files:**

- Create: `simulator/src/fsai_interfaces/msg/ActuationCommand.msg`
- Create: `simulator/src/fsai_interfaces/msg/ActuatorState.msg`
- Create: `simulator/src/fsai_bringup/vehicles/reference_bicycle/interfaces.yaml`
- Create: `simulator/src/fsai_interfaces/package.xml`
- Create: `simulator/src/fsai_interfaces/CMakeLists.txt`
- Create: `simulator/src/fsai_sim2_adapter/include/fsai_sim2_adapter/fsai_core_adapter.hpp`
- Create: `simulator/src/fsai_sim2_adapter/src/fsai_core_adapter.cpp`
- Create: `simulator/src/fsai_sim2_adapter/test/fsai_core_adapter_test.cpp`
- Modify: `simulator/src/fsai_sim2_adapter/src/simulation_node.cpp`
- Modify: `simulator/src/fsai_sim2_adapter/src/core_factory.cpp`

**Interfaces:**

- Consumes: `fsai_interfaces/ActuationCommand` and `Plant::Update`.
- Produces: EUFS `CoreSimulationBase` state access plus direct physical command input.

- [ ] **Step 1: Write failing mapping tests**

```cpp
TEST(FsaiCoreAdapter, MapsPhysicalCommandWithoutAccelerationConversion) {
  adapter.SetPhysicalCommand(Command{
      .steering_angle_rad = 0.1,
      .rear_axle_torque_nm = 80.0});
  adapter.Step(5ms);
  EXPECT_DOUBLE_EQ(adapter.LastAppliedCommand().rear_axle_torque_nm, 80.0);
}

TEST(FsaiCoreAdapter, RejectsEufsCommandUnlessCompatibilityEnabled) {
  EXPECT_THROW(adapter.SetCommand(EufsAccelerationCommand()), InterfaceError);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim2_adapter --ctest-args -R fsai_core_adapter_test --output-on-failure`

Expected: FAIL because `FsaiCoreAdapter` is absent.

- [ ] **Step 3: Implement adapter and message fields**

`ActuationCommand.msg` must contain builtin timestamp, steering angle rad, front and rear axle wheel-side torque Nm, friction brake ratio and sequence number.
`FsaiCoreAdapter` must retain `PlantState`, last `StepResult` and simulation time.
Map accepted chassis, acceleration, steering, wheel speed and force diagnostics to EUFS getters without recomputation.
Register core key `fsai`.
Keep `eufs` as compatibility key.
EUFS acceleration conversion must require `enable_eufs_acceleration_compatibility=true` and log mass, tyre radius, drive split and conversion revision.
`FsaiSimulationNode` must own `SafetyState` and call `SafetyChain::Resolve` before passing the safe command to `FsaiCoreAdapter`.
Command age must use simulation time and the last accepted command timestamp.
`interfaces.yaml` must declare the physical command topic, EUFS compatibility flag, command timeout and sequence policy.

- [ ] **Step 4: Run direct and launch tests**

Launch `FsaiSimulationNode` with `core_type:=fsai`, publish one physical command, and verify finite odometry, actuator state and increasing clock.
Expected: PASS without enabling the EUFS acceleration subscriber.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_interfaces simulator/src/fsai_sim2_adapter simulator/src/fsai_bringup/vehicles/reference_bicycle/interfaces.yaml
git commit -m "feat: connect physical vehicle commands"
```

### Task 8: Add phase-one physics scenarios

**Files:**

- Create: `simulator/src/fsai_sim_core/test/scenario_test.cpp`
- Create: `simulator/src/fsai_bringup/scenarios/stationary.yaml`
- Create: `simulator/src/fsai_bringup/scenarios/straight_acceleration.yaml`
- Create: `simulator/src/fsai_bringup/scenarios/constant_radius.yaml`
- Create: `simulator/src/fsai_bringup/scenarios/brake_to_stop.yaml`
- Create: `simulator/src/fsai_bringup/scenarios/command_timeout.yaml`
- Create: `simulator/src/fsai_bringup/scenarios/ebs.yaml`

**Interfaces:**

- Consumes: Plant, reference vehicle profile and fixed command schedules.
- Produces: Six deterministic phase-one acceptance traces.

- [ ] **Step 1: Write failing scenario assertions**

Stationary must remain within 1e-12 of zero state for 10 seconds despite a steering command.
Straight acceleration must keep absolute lateral position and yaw below 1e-9.
Constant-radius must have positive yaw rate and steady-state radius within the analytic tolerance recorded in the fixture.
Brake-to-stop must reach exact zero without negative longitudinal velocity.
Timeout must remove drive and apply configured brake after exactly 100 ms.
EBS must latch until full reset.

- [ ] **Step 2: Run scenario tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R scenario_test --output-on-failure`

Expected: At least one assertion fails before fixtures and runner integration exist.

- [ ] **Step 3: Add fixtures and deterministic trace serialization**

Use 1 ms internal and 5 ms outer steps in every fixture.
Write trace columns in a fixed field order and use `std::to_chars` with `max_digits10`.
Include parameter hash, backend revision and scenario seed in the header.

- [ ] **Step 4: Run all Plan 2 checks**

Run all core and adapter tests twice.
Compare the two trace directories byte for byte.
Expected: Exact match and all six physical acceptance conditions pass.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core simulator/src/fsai_bringup/scenarios
git commit -m "test: validate phase-one vehicle scenarios"
```

## Plan Completion Gate

`fsai_sim_core` must build without ROS sourced.
All six physical scenarios must pass.
The physical command path must not depend on EUFS acceleration semantics.
Parameter and snapshot mismatches must fail before stepping.
Two identical runs must generate byte-identical traces.
