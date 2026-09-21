#include <filesystem>
#include <limits>
#include <gtest/gtest.h>

#include "fsai_sim2_adapter/runtime_config.hpp"

namespace {
const auto kBringup = std::filesystem::path(__FILE__).parent_path().parent_path().parent_path() /
  "fsai_bringup";
}

TEST(RuntimeConfig, LoadsActualTrackScenarioAndInterfaceContracts) {
  const auto track = fsai::sim2_adapter::LoadTrack(kBringup / "tracks/skidpad_small");
  EXPECT_EQ(track.cones.size(), 10u);
  EXPECT_EQ(track.frame_id, "map");
  const auto scenario = fsai::sim2_adapter::LoadScenario(
    kBringup / "scenarios/straight_acceleration.yaml");
  EXPECT_EQ(scenario.duration, std::chrono::seconds(20));
  EXPECT_EQ(scenario.outer_step, std::chrono::milliseconds(5));
  const auto interfaces = fsai::sim2_adapter::LoadInterfaceConfig(
    kBringup / "vehicles/reference_bicycle/interfaces.yaml");
  EXPECT_EQ(interfaces.command_topic, "/fsai/actuation_command");
  EXPECT_EQ(interfaces.command_timeout, std::chrono::milliseconds(100));
}

TEST(RuntimeConfig, RejectsMissingResourcesAndNonFiniteDurations) {
  EXPECT_THROW(fsai::sim2_adapter::LoadTrack(kBringup / "tracks/missing"), std::exception);
  EXPECT_THROW(fsai::sim2_adapter::LoadScenario(kBringup / "scenarios/missing.yaml"), std::exception);
  EXPECT_THROW(fsai::sim2_adapter::SecondsToDuration(-1.0, "duration"), std::exception);
  EXPECT_THROW(fsai::sim2_adapter::SecondsToDuration(
    std::numeric_limits<double>::infinity(), "duration"), std::exception);
}
