#include <chrono>
#include <numbers>

#include <gtest/gtest.h>

#include "fsai_sim2_adapter/fsai_core_adapter.hpp"

using namespace std::chrono_literals;
using fsai::sim2_adapter::FsaiCoreAdapter;

TEST(FsaiCoreAdapter, DefaultsToNonDrivingAndRequiresFreshCommandAfterEnabling) {
  FsaiCoreAdapter adapter(fsai::sim::ReferenceBicycleParameters());
  adapter.SetPhysicalCommand({.rear_axle_torque_nm = 80.0});
  adapter.Step(5ms);
  EXPECT_DOUBLE_EQ(adapter.plant_state().chassis.u_mps, 0.0);
  adapter.SetDriving(true);
  adapter.SetPhysicalCommand({.rear_axle_torque_nm = 80.0});
  adapter.Step(5ms);
  EXPECT_GT(adapter.plant_state().chassis.u_mps, 0.0);
}

TEST(FsaiCoreAdapter, ReceivedCommandExpiresWithoutAnotherCallback) {
  FsaiCoreAdapter adapter(fsai::sim::ReferenceBicycleParameters());
  adapter.SetDriving(true);
  adapter.SetPhysicalCommand({.rear_axle_torque_nm = 80.0});
  for (int i = 0; i < 100; ++i) {
    adapter.Step(5ms);
  }
  EXPECT_EQ(adapter.plant_state().last_command_time, 0ms);
  EXPECT_DOUBLE_EQ(adapter.LastAppliedCommand().rear_axle_torque_nm, 0.0);
  EXPECT_DOUBLE_EQ(adapter.LastAppliedCommand().friction_brake_ratio, 0.5);
}

TEST(FsaiCoreAdapter, EbsLatchesUntilResetAndResetReturnsToNonDriving) {
  FsaiCoreAdapter adapter(fsai::sim::ReferenceBicycleParameters());
  adapter.SetDriving(true);
  adapter.RequestEbs();
  adapter.SetPhysicalCommand({.rear_axle_torque_nm = 80.0});
  adapter.Step(5ms);
  EXPECT_TRUE(adapter.plant_state().ebs_latched);
  EXPECT_DOUBLE_EQ(adapter.LastAppliedCommand().friction_brake_ratio, 1.0);
  EXPECT_DOUBLE_EQ(adapter.LastAppliedCommand().rear_axle_torque_nm, 0.0);
  adapter.Reset();
  adapter.SetPhysicalCommand({.rear_axle_torque_nm = 80.0});
  adapter.Step(5ms);
  EXPECT_FALSE(adapter.plant_state().ebs_latched);
  EXPECT_DOUBLE_EQ(adapter.LastAppliedCommand().rear_axle_torque_nm, 0.0);
}

TEST(FsaiCoreAdapter, EufsWheelSpeedsAreRevolutionsPerSecond) {
  FsaiCoreAdapter adapter(fsai::sim::ReferenceBicycleParameters());
  adapter.SetDriving(true);
  adapter.SetPhysicalCommand({.rear_axle_torque_nm = 80.0});
  adapter.Step(5ms);
  FsaiCoreAdapter::WheelSpeeds::Vector wheels;
  (void)adapter.GetState(wheels);
  EXPECT_NEAR(wheels(eufs::sim2::sensors::WheelSpeedsMember::_rl),
    adapter.plant_state().wheels[2].omega_radps / (2.0 * std::numbers::pi), 1e-12);
}
