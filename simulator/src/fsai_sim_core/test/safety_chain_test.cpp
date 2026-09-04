#include <chrono>

#include <gtest/gtest.h>

#include "fsai_sim_core/safety_chain.hpp"

using namespace std::chrono_literals;
using fsai::sim::Command;
using fsai::sim::EventType;
using fsai::sim::PlantState;
using fsai::sim::ReferenceBicycleParameters;
using fsai::sim::Resolve;
using fsai::sim::SafetyInput;

TEST(SafetyChain, BlocksDriveBeforeDrivingState) {
  Command command{};
  command.rear_axle_torque_nm = 80.0;
  command.front_axle_torque_nm = 10.0;
  PlantState state;
  auto result = Resolve(
    command,
    SafetyInput{.as_driving = false},
    state,
    ReferenceBicycleParameters());
  EXPECT_DOUBLE_EQ(result.command.front_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(result.command.rear_axle_torque_nm, 0.0);
}

TEST(SafetyChain, TimeoutAppliesConfiguredSafeBrake) {
  Command command{};
  command.rear_axle_torque_nm = 80.0;
  const auto params = ReferenceBicycleParameters();
  auto result = Resolve(
    command,
    SafetyInput{.as_driving = true, .command_age = 101ms},
    PlantState{},
    params);
  EXPECT_DOUBLE_EQ(result.command.rear_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(result.command.friction_brake_ratio, params.timeout_brake_ratio);
  ASSERT_FALSE(result.events.empty());
  EXPECT_EQ(result.events.front().type, EventType::kCommandTimeout);
}

TEST(SafetyChain, EbsLatchesUntilReset) {
  Command command{};
  command.rear_axle_torque_nm = 80.0;
  PlantState state;
  const auto params = ReferenceBicycleParameters();
  auto first = Resolve(
    command,
    SafetyInput{.as_driving = true, .ebs_request = true},
    state,
    params);
  EXPECT_TRUE(first.ebs_latched);
  EXPECT_DOUBLE_EQ(first.command.friction_brake_ratio, params.ebs_brake_ratio);
  state.ebs_latched = true;
  auto second = Resolve(
    command,
    SafetyInput{.as_driving = true, .ebs_request = false},
    state,
    params);
  EXPECT_TRUE(second.ebs_latched);
  EXPECT_DOUBLE_EQ(second.command.rear_axle_torque_nm, 0.0);
}
