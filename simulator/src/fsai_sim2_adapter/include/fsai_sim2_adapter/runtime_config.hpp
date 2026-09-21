#ifndef FSAI_SIM2_ADAPTER__RUNTIME_CONFIG_HPP_
#define FSAI_SIM2_ADAPTER__RUNTIME_CONFIG_HPP_

#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

#include "fsai_sim_core/types.hpp"

namespace fsai::sim2_adapter {

struct InterfaceConfig {
  std::string command_topic;
  bool eufs_compatibility{false};
  fsai::sim::Duration command_timeout{};
};

struct TrackCone {
  std::string color;
  double x{};
  double y{};
};

struct TrackConfig {
  std::string name;
  std::string frame_id;
  fsai::sim::ChassisState start;
  std::vector<TrackCone> cones;
};

struct ScheduledCommand {
  fsai::sim::SimTime start{};
  fsai::sim::SimTime end{};
  fsai::sim::Command command;
};

struct ScenarioConfig {
  std::string name;
  std::string mission;
  std::string vehicle_profile;
  std::string track_bundle;
  std::string mode;
  std::uint32_t seed{};
  fsai::sim::Duration duration{};
  fsai::sim::Duration outer_step{};
  bool auto_start{false};
  std::vector<ScheduledCommand> commands;
};

InterfaceConfig LoadInterfaceConfig(const std::filesystem::path &file);
TrackConfig LoadTrack(const std::filesystem::path &directory);
ScenarioConfig LoadScenario(const std::filesystem::path &file);
fsai::sim::Duration SecondsToDuration(double seconds, const std::string &field);

}  // namespace fsai::sim2_adapter
#endif
