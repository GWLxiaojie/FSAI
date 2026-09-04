# Four-Wheel Double-Track Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 在不改变外部命令、状态、传感器和 ROS 接口的前提下，将内部动力学升级为逐轮四轮双轨并接入 OCD 连续后端。

**Architecture:** `SteeringGeometry`、`WheelKinematics`、`LoadTransfer`、`TyreModel` 和 `ContactSolver` 各自拥有单一责任。
`HybridIntegrator` 继续独占时间推进，`FsaiFourWheelBackend` 和 `OcdDoubleTrackBackend` 只能提供连续导数和候选力。

**Tech Stack:** C++20, Eigen3, GoogleTest, Open-Car-Dynamics pinned at `94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`.

**Spec:** `docs/superpowers/specs/2026-09-04-eufs-sim2-team-simulator-design.md`

## Global Constraints

外部 `Command` 字段和物理语义不得改变。
`PlantState` 公共字段不得删除或重解释。
每个车轮必须拥有角速度、接触模式和接触记忆。
同一时刻只能有一个模块拥有轮胎力、轮荷和气动力。
ContactSolver 拥有 `kStick`、`kTransition`、`kSlip` 和 `kBrakeHold`。
DynamicsBackend 不得推进时间。
OCD `VehicleModel::step` 不得进入正式路径。
不同 backend 的隐藏状态不得盲目互相恢复。
没有实车留出数据前不得宣称高精度。
阶段一整圈 E2E 必须继续通过。

---

## File Map

```text
simulator/src/fsai_sim_core/
  include/fsai_sim_core/steering_geometry.hpp
  include/fsai_sim_core/wheel_kinematics.hpp
  include/fsai_sim_core/load_transfer.hpp
  include/fsai_sim_core/combined_tyre.hpp
  include/fsai_sim_core/contact_solver.hpp
  include/fsai_sim_core/four_wheel_backend.hpp
  include/fsai_sim_core/ocd_double_track_backend.hpp
  src/
  test/
simulator/src/fsai_bringup/
  vehicles/reference_four_wheel/
  scenarios/four_wheel/
simulator/fsai_sim.repos
simulator/dependencies.lock.yaml
THIRD_PARTY_NOTICES.md
```

### Task 1: Version the phase-two state without breaking phase one

**Files:**

- Modify: `simulator/src/fsai_sim_core/include/fsai_sim_core/types.hpp`
- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/state_migration.hpp`
- Create: `simulator/src/fsai_sim_core/src/state_migration.cpp`
- Create: `simulator/src/fsai_sim_core/test/state_migration_test.cpp`
- Modify: `simulator/src/fsai_interfaces/schema/simulation_snapshot.fbs`

**Interfaces:**

- Consumes: Phase-one PlantState schema 1.
- Produces: PlantState schema 2 and `MigrateStateV1ToV2`.

- [ ] **Step 1: Write failing migration tests**

```cpp
TEST(StateMigration, PreservesPublicPhaseOneValues) {
  auto old_state = PhaseOneState();
  auto migrated = MigrateStateV1ToV2(old_state);
  EXPECT_EQ(migrated.chassis, old_state.chassis);
  EXPECT_EQ(migrated.actuator, old_state.actuator);
  EXPECT_EQ(migrated.parameter_hash, old_state.parameter_hash);
}

TEST(StateRestore, RejectsBackendRevisionMismatch) {
  EXPECT_THROW(Restore(snapshot, DifferentBackendRevision()),
               SnapshotCompatibilityError);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R state_migration_test --output-on-failure`

Expected: FAIL because schema 2 and migration are absent.

- [ ] **Step 3: Add explicit phase-two fields**

Each `WheelState` must contain omega, steer angle, longitudinal slip, slip angle, normal load, longitudinal force, lateral force, contact mode, mode hysteresis and continuation force.
`BackendState` must be a tagged versioned value with separate bicycle, four-wheel and OCD payloads.
Migration must derive phase-one wheel speeds from accepted wheel-center velocities and set `kKinematic` contact.
Cross-backend restore must require an explicit migration function.
Increment FlatBuffers schema while preserving version-one readers.

- [ ] **Step 4: Run old snapshot and new round-trip tests**

Expected: Version-one fixtures migrate, version-two fixtures round-trip and mismatched revisions fail.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core simulator/src/fsai_interfaces
git commit -m "feat: version four-wheel plant state"
```

### Task 2: Implement Ackermann steering geometry and wheel velocities

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/steering_geometry.hpp`
- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/wheel_kinematics.hpp`
- Create: `simulator/src/fsai_sim_core/src/steering_geometry.cpp`
- Create: `simulator/src/fsai_sim_core/src/wheel_kinematics.cpp`
- Create: `simulator/src/fsai_sim_core/test/steering_geometry_test.cpp`
- Create: `simulator/src/fsai_sim_core/test/wheel_kinematics_test.cpp`

**Interfaces:**

- Consumes: Equivalent centre steering, wheelbase, track widths and chassis velocities.
- Produces: Four road-wheel angles and four wheel-frame contact velocities.

- [ ] **Step 1: Write failing symmetry and turn tests**

```cpp
TEST(SteeringGeometry, ZeroInputReturnsFourZeros) {
  EXPECT_EQ(SteeringGeometry(params).Angles(0.0),
            std::array<double, 4>{0.0, 0.0, 0.0, 0.0});
}

TEST(SteeringGeometry, PositiveTurnHasLargerLeftAngle) {
  auto angles = SteeringGeometry(params).Angles(0.2);
  EXPECT_GT(angles[kFrontLeft], angles[kFrontRight]);
  EXPECT_GT(angles[kFrontRight], 0.0);
}

TEST(WheelKinematics, PureYawHasOppositeSideLongitudinalOffsets) {
  auto velocities = WheelVelocities(YawingState(), angles, geometry);
  EXPECT_LT(velocities[kFrontLeft].x(), velocities[kFrontRight].x());
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R 'steering_geometry|wheel_kinematics' --output-on-failure`

Expected: FAIL because both modules are absent.

- [ ] **Step 3: Implement exact geometry**

For nonzero equivalent angle use `R = L / tan(delta)`.
For a positive turn use `delta_FL = atan(L / (R - track_front / 2))` and `delta_FR = atan(L / (R + track_front / 2))`.
Mirror signs and inner wheel for negative turns.
Use a series expansion below `abs(delta) < 1e-8`.
At wheel location `(x_i, y_i)` compute body velocity `(u - r y_i, v + r x_i)` and rotate by negative road-wheel angle into the wheel frame.

- [ ] **Step 4: Run property sweeps**

Sweep legal steering and yaw rates.
Require left-right mirror symmetry, finite values and convergence to parallel steering near zero.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: add four-wheel steering kinematics"
```

### Task 3: Implement quasi-static normal loads

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/load_transfer.hpp`
- Create: `simulator/src/fsai_sim_core/src/load_transfer.cpp`
- Create: `simulator/src/fsai_sim_core/test/load_transfer_test.cpp`

**Interfaces:**

- Consumes: Mass, CG geometry, aero loads and accepted CG acceleration.
- Produces: Four nonnegative normal-load candidates and diagnostics.

- [ ] **Step 1: Write failing conservation tests**

```cpp
TEST(LoadTransfer, StaticLoadsSumToWeightAndDownforce) {
  auto loads = ComputeLoads(StaticInput(), params);
  EXPECT_NEAR(Sum(loads), params.mass_kg * params.gravity_mps2 +
              StaticInput().downforce_n, 1e-9);
}

TEST(LoadTransfer, BrakingMovesLoadForward) {
  auto static_loads = ComputeLoads(StaticInput(), params);
  auto braking_loads = ComputeLoads(InputWithAx(-5.0), params);
  EXPECT_GT(FrontSum(braking_loads), FrontSum(static_loads));
}

TEST(LoadTransfer, LeftTurnLoadsRightWheels) {
  auto loads = ComputeLoads(InputWithAy(5.0), params);
  EXPECT_GT(RightSum(loads), LeftSum(loads));
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R load_transfer_test --output-on-failure`

Expected: FAIL because load transfer is absent.

- [ ] **Step 3: Implement conserved axle and lateral transfer**

Use `Fz_front = m g l_r / L - m ax h_cg / L + aero_front`.
Use `Fz_rear = m g l_f / L + m ax h_cg / L + aero_rear`.
Split lateral transfer by configured front roll-transfer fraction.
For each axle use right-minus-left load difference `m ay h_cg roll_fraction / track`.
Preserve total vertical load.
If a candidate becomes negative, emit wheel-lift mode with zero normal load and redistribute no force silently.

- [ ] **Step 4: Run grid and conservation tests**

Sweep longitudinal and lateral acceleration inside configured validity bounds.
Expected: Total load conservation and explicit wheel-lift events.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: add conserved wheel loads"
```

### Task 4: Implement per-wheel combined-slip tyre forces

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/combined_tyre.hpp`
- Create: `simulator/src/fsai_sim_core/src/combined_tyre.cpp`
- Create: `simulator/src/fsai_sim_core/test/combined_tyre_test.cpp`

**Interfaces:**

- Consumes: Wheel-frame velocity, omega, normal load and tyre parameters.
- Produces: Slip ratio, slip angle and saturated wheel-frame force.

- [ ] **Step 1: Write failing slip and friction-limit tests**

```cpp
TEST(CombinedTyre, DriveSlipProducesPositiveLongitudinalForce) {
  auto out = tyre.Evaluate(InputWithKappa(0.1));
  EXPECT_GT(out.force_wheel_n.x(), 0.0);
}

TEST(CombinedTyre, LeftSlipConventionProducesExpectedForce) {
  auto out = tyre.Evaluate(InputWithAlpha(-0.1));
  EXPECT_GT(out.force_wheel_n.y(), 0.0);
}

TEST(CombinedTyre, ForceStaysInsideFrictionEllipse) {
  auto out = tyre.Evaluate(CombinedLimitInput());
  EXPECT_LE(FrictionEllipseValue(out), 1.0 + 1e-12);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R combined_tyre_test --output-on-failure`

Expected: FAIL because the combined tyre model is absent.

- [ ] **Step 3: Implement regularized slips and saturation**

Use `kappa = (R omega - Vx) / sqrt(Vx^2 + v_kappa_eps^2)`.
Compute slip angle from wheel-frame velocity with the same sign convention as the phase-one Pacejka model.
Evaluate pure longitudinal and lateral Pacejka curves from versioned parameters.
Apply a smooth friction ellipse scaling once.
Return zero force for zero normal load.
Do not clamp wheel speed or vehicle speed.

- [ ] **Step 4: Run sign, continuity and energy tests**

Sweep through zero speed and zero slip.
Require continuous force, correct dissipation under braking and no force above the configured friction envelope.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: add combined-slip tyre forces"
```

### Task 5: Implement ContactSolver state transitions

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/contact_solver.hpp`
- Create: `simulator/src/fsai_sim_core/src/contact_solver.cpp`
- Create: `simulator/src/fsai_sim_core/test/contact_solver_test.cpp`

**Interfaces:**

- Consumes: Wheel state, demanded contact force, available friction, drive torque and brake torque.
- Produces: Applied force, wheel derivative, next contact mode and transition events.

- [ ] **Step 1: Write failing mode-transition tests**

```cpp
TEST(ContactSolver, StickMaintainsPureRolling) {
  auto out = solver.Solve(StickInput());
  EXPECT_EQ(out.mode, ContactMode::kStick);
  EXPECT_NEAR(out.constraint_residual, 0.0, 1e-10);
}

TEST(ContactSolver, ExceedingFrictionEntersTransitionThenSlip) {
  auto transition = solver.Solve(OverLimitFromStick());
  EXPECT_EQ(transition.mode, ContactMode::kTransition);
  auto slip = solver.Solve(NextInput(transition));
  EXPECT_EQ(slip.mode, ContactMode::kSlip);
}

TEST(ContactSolver, BrakeHoldCannotCreateReverseMotion) {
  auto out = solver.Solve(BrakeHoldInputWithNoDrive());
  EXPECT_DOUBLE_EQ(out.wheel_omega_dot_radps2, 0.0);
  EXPECT_EQ(out.mode, ContactMode::kBrakeHold);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R contact_solver_test --output-on-failure`

Expected: FAIL because ContactSolver is absent.

- [ ] **Step 3: Implement explicit mode ownership**

In `kStick` solve the force required for pure rolling and accept it only within the friction bound.
In `kTransition` blend the stored continuation force to the slip force over the configured force-continuity interval.
In `kSlip` use combined-slip tyre force and wheel torque balance.
In `kBrakeHold` enforce zero wheel speed and zero reverse chassis motion until drive exceeds brake and release margin.
Serialize hysteresis and continuation state.
Emit one event per mode change.

- [ ] **Step 4: Run chattering and replay tests**

Cycle demand around each threshold for 10,000 steps.
Expected: No one-step chatter and exact replay after snapshot restore.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: solve hybrid tyre contact"
```

### Task 6: Assemble FsaiFourWheelBackend

**Files:**

- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/four_wheel_backend.hpp`
- Create: `simulator/src/fsai_sim_core/src/four_wheel_backend.cpp`
- Create: `simulator/src/fsai_sim_core/test/four_wheel_backend_test.cpp`
- Create: `simulator/src/fsai_bringup/vehicles/reference_four_wheel/vehicle.yaml`
- Create: `simulator/src/fsai_bringup/vehicles/reference_four_wheel/tyres.yaml`
- Create: `simulator/src/fsai_bringup/vehicles/reference_four_wheel/aero.yaml`

**Interfaces:**

- Consumes: Phase-two state and actual actuator outputs.
- Produces: Four-wheel continuous derivatives, candidate forces and diagnostics.

- [ ] **Step 1: Write failing dynamics tests**

```cpp
TEST(FourWheelBackend, StraightSymmetryProducesNoYawMoment) {
  auto out = backend.Evaluate(SymmetricStraightInput());
  EXPECT_NEAR(out.chassis_derivative.yaw_rate_radps2, 0.0, 1e-12);
}

TEST(FourWheelBackend, WheelTorqueBalanceMatchesEquation) {
  auto out = backend.Evaluate(SingleDrivenWheelInput());
  EXPECT_NEAR(out.wheel_omega_dot[kRearLeft],
              (drive_torque - brake_torque -
               out.tyre_force_x[kRearLeft] * radius) / wheel_inertia,
              1e-12);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R four_wheel_backend_test --output-on-failure`

Expected: FAIL because the backend is absent.

- [ ] **Step 3: Assemble modules without duplicate ownership**

Compute steering, wheel velocities and normal loads once.
Ask ContactSolver for each applied contact force.
Rotate wheel forces into body coordinates.
Sum chassis force and yaw moment once.
Compute aerodynamic drag and downforce once.
Return derivatives only.
Register backend ID `fsai_four_wheel` revision `1`.

- [ ] **Step 4: Run physical scenario suite**

Run straight acceleration, split torque, constant radius, trail braking, brake hold, wheel lift and reverse-command rejection.
Require finite values, force balance and step-size convergence.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core simulator/src/fsai_bringup/vehicles/reference_four_wheel
git commit -m "feat: add four-wheel dynamics backend"
```

### Task 7: Extend HybridIntegrator for wheel events

**Files:**

- Modify: `simulator/src/fsai_sim_core/include/fsai_sim_core/hybrid_integrator.hpp`
- Modify: `simulator/src/fsai_sim_core/src/hybrid_integrator.cpp`
- Create: `simulator/src/fsai_sim_core/test/four_wheel_integrator_test.cpp`

**Interfaces:**

- Consumes: Four-wheel derivatives and candidate contact transitions.
- Produces: Accepted chassis and wheel state at a common timestamp.

- [ ] **Step 1: Write failing simultaneous-event tests**

```cpp
TEST(FourWheelIntegrator, LocatesFirstWheelLockEvent) {
  auto out = integrator.Integrate(NearLockState(), FullBrake(), 5ms);
  EXPECT_EQ(out.events.front().wheel, kFrontLeft);
  EXPECT_EQ(out.events.front().type, EventType::kWheelLocked);
}

TEST(FourWheelIntegrator, AdvancesChassisAndWheelsToSameTime) {
  auto out = integrator.Integrate(state, input, 5ms);
  EXPECT_EQ(out.chassis_time, out.wheel_time);
  EXPECT_EQ(out.chassis_time, 5ms);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R four_wheel_integrator_test --output-on-failure`

Expected: FAIL because the integrator does not locate wheel events.

- [ ] **Step 3: Implement earliest-event splitting**

Integrate continuous chassis and wheel states with one RK4 tableau.
Detect candidate wheel lock, unlock, stick-slip and wheel-lift crossings in every internal substep.
Locate the earliest event to 1 ns by bisection.
Accept simultaneous events within 1 ns in wheel index order.
Apply mode changes, then integrate the remaining duration.
Reject an event loop exceeding one event per wheel per internal substep.

- [ ] **Step 4: Run convergence and long-horizon tests**

Expected: No time skew, no infinite event loop and convergent state under step halving.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_sim_core
git commit -m "feat: integrate four-wheel contact events"
```

### Task 8: Add the OCD continuous backend adapter

**Files:**

- Modify: `simulator/fsai_sim.repos`
- Modify: `simulator/dependencies.lock.yaml`
- Modify: `THIRD_PARTY_NOTICES.md`
- Create: `simulator/src/fsai_sim_core/include/fsai_sim_core/ocd_double_track_backend.hpp`
- Create: `simulator/src/fsai_sim_core/src/ocd_double_track_backend.cpp`
- Create: `simulator/src/fsai_sim_core/test/ocd_double_track_backend_test.cpp`

**Interfaces:**

- Consumes: OCD double-track equation model at fixed revision and phase-two state.
- Produces: DynamicsBackend ID `ocd_double_track` revision tied to the pinned commit.

- [ ] **Step 1: Write failing ownership and mapping tests**

```cpp
TEST(OcdAdapter, DoesNotAdvanceTimeInternally) {
  auto before = state;
  backend.Evaluate(state, input);
  EXPECT_EQ(state, before);
}

TEST(OcdAdapter, MapsEveryHiddenContinuousState) {
  auto encoded = backend.EncodeState(CompleteOcdState());
  EXPECT_EQ(backend.DecodeState(encoded), CompleteOcdState());
}

TEST(OcdAdapter, UsesImposedContactForcesOutsideSlip) {
  auto out = backend.Evaluate(StickModeInput());
  EXPECT_EQ(out.applied_contact_forces, imposed_forces);
}
```

- [ ] **Step 2: Run focused tests**

Run: `colcon test --base-paths simulator/src --packages-select fsai_sim_core --ctest-args -R ocd_double_track_backend_test --output-on-failure`

Expected: FAIL because OCD is not imported or adapted.

- [ ] **Step 3: Pin and adapt OCD equations**

Add `https://github.com/TUMFTM/Open-Car-Dynamics.git` at `94f8fb187fb0ed22bba1d809bd74f66d1ff75af4`.
Use `ocd_vehicle_dynamics_double_track_cpp` equation evaluation.
Do not call `ocd_vehicle_model_cpp::VehicleModel::step`.
Map heave, roll, pitch, their rates, wheel heave, wheel omega and tyre relaxation state explicitly.
Let OCD own its dynamic load and slip-region candidate forces.
Pass imposed ContactSolver forces for non-slip modes.
Disable duplicate FSAI aero and load transfer when this backend is active.

- [ ] **Step 4: Run adapter and cross-backend tests**

Compare zero input, straight acceleration and steady circle against OCD direct equation fixtures at the fixed revision.
Expected: Adapter outputs match fixture tolerances and state remains externally owned.

- [ ] **Step 5: Commit**

```bash
git add simulator/fsai_sim.repos simulator/dependencies.lock.yaml THIRD_PARTY_NOTICES.md simulator/src/fsai_sim_core
git commit -m "feat: adapt OCD double-track equations"
```

### Task 9: Prove external interface stability and full-lap regression

**Files:**

- Create: `simulator/src/fsai_bringup/scenarios/four_wheel/autocross_regression.yaml`
- Create: `simulator/src/fsai_bringup/test/test_backend_compatibility.launch.py`
- Modify: `.github/workflows/humble.yml`
- Modify: `.github/workflows/jazzy.yml`

**Interfaces:**

- Consumes: Bicycle, FSAI four-wheel and OCD backend IDs.
- Produces: Interface-equivalence and phase-two scenario gates.

- [ ] **Step 1: Write failing compatibility assertions**

For all three backends, enumerate topic names and message types and require exact equality.
Require the same ActuationCommand input schema.
Require snapshots to reject cross-backend restore.
Require the reference controller to complete the reference lap with each enabled backend.
Require Ground Truth subscription audit to remain empty.

- [ ] **Step 2: Run the compatibility E2E**

Run: `colcon test --base-paths simulator/src --packages-select fsai_bringup --ctest-args -R test_backend_compatibility --output-on-failure`

Expected: FAIL until phase-two launch selection and profiles are complete.

- [ ] **Step 3: Add backend selection without interface branches**

Select DynamicsBackend from immutable vehicle profile.
Do not branch ROS topic names, units, sensor semantics or race rules by backend.
Record backend ID and revision in run report, snapshots and MCAP metadata.
Keep bicycle as a supported reference backend.

- [ ] **Step 4: Run all Humble and Jazzy gates**

Run complete Humble lint, unit, scenario and E2E twice.
Run Jazzy compile and core tests.
Compare repeated outputs for each backend.
Expected: All pass without changing acceptance tolerances between repeated runs.

- [ ] **Step 5: Commit**

```bash
git add simulator/src/fsai_bringup .github/workflows
git commit -m "test: validate double-track backends"
```

## Plan Completion Gate

All external ROS names, messages and units must remain unchanged.
Four-wheel forces and loads must satisfy conservation checks.
Contact transitions must be replayable and free of threshold chatter.
OCD must remain a continuous equation adapter, not a second integrator.
Bicycle full-lap regression must still pass.
No high-precision claim may be made without separate real-vehicle validation.
