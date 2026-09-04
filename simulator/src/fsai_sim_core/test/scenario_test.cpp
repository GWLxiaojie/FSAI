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
