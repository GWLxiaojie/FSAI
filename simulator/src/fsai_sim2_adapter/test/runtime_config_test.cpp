#include <filesystem>
#include <limits>
#include <fstream>
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

TEST(RuntimeConfig, ReferenceDriverRequiresExclusiveAutomaticControlAndValidTargets) {
  const auto directory = std::filesystem::temp_directory_path() /
    ("fsai-reference-config-" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
  ASSERT_TRUE(std::filesystem::create_directory(directory));
  const auto file = directory / "scenario.yaml";
  const std::string base =
    "schema_version: 1\nname: test\nmission: track_drive\nvehicle_profile: ads_dv\n"
    "track_bundle: official_sprint\nseed: 1\nduration_limit_s: 3600\n"
    "mode: as_fast_as_possible\nplant_step_ms: 1\nouter_step_ms: 5\n";
  auto write = [&](const std::string &extra) { std::ofstream(file) << base << extra; };
  write("reference_driver: true\nauto_start: true\ntarget_laps: 10\ncruise_speed_mps: 2.5\n");
  auto loaded = fsai::sim2_adapter::LoadScenario(file);
  EXPECT_TRUE(loaded.reference_driver);
  EXPECT_EQ(loaded.target_laps, 10u);
  EXPECT_DOUBLE_EQ(loaded.cruise_speed_mps, 2.5);
  write("reference_driver: true\nauto_start: false\n");
  EXPECT_THROW(fsai::sim2_adapter::LoadScenario(file), std::exception);
  write("reference_driver: true\nauto_start: true\ntarget_laps: 0\n");
  EXPECT_THROW(fsai::sim2_adapter::LoadScenario(file), std::exception);
  write("reference_driver: true\nauto_start: true\ncruise_speed_mps: 8.0\n");
  EXPECT_THROW(fsai::sim2_adapter::LoadScenario(file), std::exception);
  write("reference_driver: true\nauto_start: true\ncommands: []\n");
  EXPECT_THROW(fsai::sim2_adapter::LoadScenario(file), std::exception);
  std::filesystem::remove_all(directory);
}
