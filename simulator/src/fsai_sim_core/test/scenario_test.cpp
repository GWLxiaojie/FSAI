#include <chrono>
#include <cmath>

#include <gtest/gtest.h>

#include "fsai_sim_core/plant.hpp"
#include "fsai_sim_core/safety_chain.hpp"

using namespace std::chrono_literals;
using fsai::sim::Command;
using fsai::sim::Plant;
using fsai::sim::PlantState;
using fsai::sim::ReferenceBicycleParameters;
using fsai::sim::Resolve;
using fsai::sim::SafetyInput;

namespace {

PlantState StepMany(Plant &plant, PlantState state, const Command &command, int steps) {
  for (int i = 0; i < steps; ++i) {
    state = plant.Update(state, command, 5ms).next_state;
  }
  return state;
}

}  // namespace

TEST(Scenario, StationaryRemainsNearZero) {
  Plant plant(ReferenceBicycleParameters());
  auto state = plant.InitialState();
  Command command{};
  command.steering_angle_rad = 0.2;
  state = StepMany(plant, state, command, 2000);
  EXPECT_NEAR(state.chassis.x_m, 0.0, 1e-12);
  EXPECT_NEAR(state.chassis.y_m, 0.0, 1e-12);
  EXPECT_NEAR(state.chassis.u_mps, 0.0, 1e-12);
}

TEST(Scenario, StraightAccelerationKeepsYawSmall) {
  Plant plant(ReferenceBicycleParameters());
  auto state = plant.InitialState();
  Command command{};
  command.rear_axle_torque_nm = 80.0;
  state = StepMany(plant, state, command, 400);
  EXPECT_LT(std::abs(state.chassis.y_m), 1e-9);
  EXPECT_LT(std::abs(state.chassis.yaw_rad), 1e-9);
  EXPECT_GT(state.chassis.u_mps, 0.5);
}

TEST(Scenario, BrakeToStopDoesNotReverse) {
  Plant plant(ReferenceBicycleParameters());
  auto state = plant.InitialState();
  Command drive{};
  drive.rear_axle_torque_nm = 120.0;
  state = StepMany(plant, state, drive, 200);
  Command brake{};
  brake.friction_brake_ratio = 1.0;
  state = StepMany(plant, state, brake, 800);
  EXPECT_GE(state.chassis.u_mps, 0.0);
  EXPECT_NEAR(state.chassis.u_mps, 0.0, 1e-9);
}

TEST(Scenario, TimeoutRemovesDrive) {
  const auto params = ReferenceBicycleParameters();
  Command command{};
  command.rear_axle_torque_nm = 80.0;
  auto result = Resolve(
    command,
    SafetyInput{.as_driving = true, .command_age = 100ms + 1ns},
    PlantState{},
    params);
  EXPECT_DOUBLE_EQ(result.command.rear_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(result.command.friction_brake_ratio, params.timeout_brake_ratio);
}

TEST(Scenario, EbsLatchesUntilReset) {
  const auto params = ReferenceBicycleParameters();
  Command command{};
  command.rear_axle_torque_nm = 80.0;
  PlantState state;
  auto latched = Resolve(
    command,
    SafetyInput{.as_driving = true, .ebs_request = true},
    state,
    params);
  state.ebs_latched = latched.ebs_latched;
  auto still = Resolve(
    command,
    SafetyInput{.as_driving = true},
    state,
    params);
  EXPECT_TRUE(still.ebs_latched);
  EXPECT_DOUBLE_EQ(still.command.rear_axle_torque_nm, 0.0);
}

TEST(PhysicsRegression, RegenerationCannotLaunchFromRest) {
  Plant plant(ReferenceBicycleParameters());
  Command c{}; c.rear_axle_torque_nm = -100.0;
  auto s = StepMany(plant, plant.InitialState(), c, 200);
  EXPECT_DOUBLE_EQ(s.chassis.u_mps, 0.0);
  EXPECT_DOUBLE_EQ(s.chassis.x_m, 0.0);
}

TEST(PhysicsRegression, BrakeHoldBalancesDriveAndReportsZeroAcceleration) {
  Plant plant(ReferenceBicycleParameters());
  auto s = plant.InitialState();
  for (auto &w : s.wheels) w.contact_mode = fsai::sim::ContactMode::kBrakeHold;
  Command c{}; c.rear_axle_torque_nm = 100; c.friction_brake_ratio = 1;
  auto result = plant.Update(s, c, 5ms);
  EXPECT_DOUBLE_EQ(result.next_state.chassis.x_m, 0.0);
  EXPECT_DOUBLE_EQ(result.next_state.chassis.u_mps, 0.0);
  EXPECT_DOUBLE_EQ(result.ground_truth.ax_body_mps2, 0.0);
  EXPECT_TRUE(result.events.empty());
}

TEST(PhysicsRegression, LowSpeedGeometryAndWheelSpeedsAreConsistent) {
  auto p = ReferenceBicycleParameters(); Plant plant(p);
  auto s = plant.InitialState(); s.chassis.u_mps = .04;
  s.actuator.steering_angle_rad = .384;
  Command c{}; c.steering_angle_rad = .384;
  auto a = plant.Update(s, c, 5ms);
  auto q = a.next_state.chassis;
  EXPECT_NEAR(q.yaw_rate_radps, q.u_mps * std::tan(.384) / p.wheelbase_m, 1e-12);
  EXPECT_NEAR(q.v_mps, fsai::sim::Derive(p).cg_to_rear_axle_m*q.yaw_rate_radps, 1e-12);
  EXPECT_LT(a.ground_truth.wheel_omega_radps[0], a.ground_truth.wheel_omega_radps[1]);
  EXPECT_LT(a.ground_truth.wheel_omega_radps[2], a.ground_truth.wheel_omega_radps[3]);
  EXPECT_NEAR(a.ground_truth.ay_body_mps2,
    std::tan(std::atan(.5*std::tan(.384))) *
      (a.ground_truth.ax_body_mps2 + q.yaw_rate_radps*q.v_mps) +
      q.yaw_rate_radps*q.u_mps, 1e-9);
}

TEST(PhysicsRegression, StopEventsUseSimulationTimeAndPlantPreservesReceptionTime) {
  Plant plant(ReferenceBicycleParameters()); auto s=plant.InitialState();
  s.sim_time=10s; s.last_command_time=9s; s.chassis.u_mps=.001;
  Command c{}; c.friction_brake_ratio=1;
  auto a=plant.Update(s,c,5ms);
  ASSERT_FALSE(a.events.empty());
  EXPECT_GT(a.events.front().time, 10s);
  EXPECT_LE(a.events.front().time, 10s+5ms);
  EXPECT_EQ(a.next_state.last_command_time, 9s);
}

TEST(PhysicsRegression, RegenerationStopsAndOneAndFiveMsAgree) {
  Plant plant(ReferenceBicycleParameters()); auto s=plant.InitialState();
  s.chassis.u_mps=1.; Command c{}; c.rear_axle_torque_nm=-100.;
  auto a=s, b=s;
  for(int i=0;i<2000;++i) a=plant.Update(a,c,1ms).next_state;
  for(int i=0;i<400;++i) b=plant.Update(b,c,5ms).next_state;
  EXPECT_DOUBLE_EQ(a.chassis.u_mps,0.);
  EXPECT_DOUBLE_EQ(b.chassis.u_mps,0.);
  EXPECT_NEAR(a.chassis.x_m,b.chassis.x_m,1e-10);
}

TEST(PhysicsRegression, SteeringRampIndependentOfOuterStep) {
  Plant plant(ReferenceBicycleParameters()); auto a=plant.InitialState();
  a.chassis.u_mps=8; auto b=a;
  Command c{};c.steering_angle_rad=.2;c.rear_axle_torque_nm=50;
  for(int i=0;i<1000;++i) a=plant.Update(a,c,1ms).next_state;
  for(int i=0;i<200;++i) b=plant.Update(b,c,5ms).next_state;
  EXPECT_NEAR(a.chassis.x_m,b.chassis.x_m,1e-10);
  EXPECT_NEAR(a.chassis.y_m,b.chassis.y_m,1e-10);
  EXPECT_NEAR(a.chassis.yaw_rad,b.chassis.yaw_rad,1e-10);
}

TEST(PhysicsRegression, HoldReleasesOnlyWhenCapacityIsExceeded) {
  auto p=ReferenceBicycleParameters(); Plant plant(p); auto s=plant.InitialState();
  for(auto &w:s.wheels) w.contact_mode=fsai::sim::ContactMode::kBrakeHold;
  Command c{};c.rear_axle_torque_nm=100.;c.friction_brake_ratio=100./p.max_brake_torque_nm;
  auto held=plant.Update(s,c,5ms);
  EXPECT_DOUBLE_EQ(held.next_state.chassis.u_mps,0.);
  EXPECT_TRUE(held.events.empty());
  c.friction_brake_ratio=0.;auto released=plant.Update(held.next_state,c,5ms);
  ASSERT_EQ(released.events.size(),1u);
  EXPECT_EQ(released.events.front().type,fsai::sim::EventType::kHoldReleased);
  EXPECT_EQ(released.events.front().time,5ms);
  EXPECT_GT(released.next_state.chassis.u_mps,0.);
}

TEST(PhysicsRegression, InitialFrictionBrakePreventsLaunch) {
  Plant plant(ReferenceBicycleParameters());auto s=plant.InitialState();
  Command c{};c.rear_axle_torque_nm=100.;c.friction_brake_ratio=1.;
  s=StepMany(plant,s,c,200);
  EXPECT_DOUBLE_EQ(s.chassis.x_m,0.);
  EXPECT_DOUBLE_EQ(s.chassis.u_mps,0.);
}
