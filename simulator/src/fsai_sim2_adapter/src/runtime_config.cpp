#include "fsai_sim2_adapter/runtime_config.hpp"

#include <cmath>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>

#include <yaml-cpp/yaml.h>

namespace fsai::sim2_adapter {
namespace {
YAML::Node Read(const std::filesystem::path &file) {
  try {
    auto root = YAML::LoadFile(file.string());
    if (!root.IsMap()) {
      throw std::invalid_argument("expected a YAML mapping");
    }
    return root;
  } catch (const std::exception &e) {
    throw std::invalid_argument(file.string() + ": " + e.what());
  }
}

double Number(const YAML::Node &node, const std::string &field) {
  const double value = node[field].as<double>();
  if (!std::isfinite(value)) {
    throw std::invalid_argument(field + ": expected a finite value");
  }
  return value;
}

std::string Text(const YAML::Node &node, const std::string &field) {
  const auto result = node[field].as<std::string>();
  if (result.empty()) {
    throw std::invalid_argument(field + ": must not be empty");
  }
  return result;
}

void Schema(const YAML::Node &node) {
  if (node["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("schema_version: only version 1 is supported");
  }
}

double CsvNumber(const std::string &text, const std::string &location) {
  std::size_t consumed = 0;
  const double value = std::stod(text, &consumed);
  if (consumed != text.size() || !std::isfinite(value)) {
    throw std::invalid_argument(location + ": invalid finite number '" + text + "'");
  }
  return value;
}
}  // namespace

fsai::sim::Duration SecondsToDuration(double seconds, const std::string &field) {
  if (!std::isfinite(seconds) || seconds < 0 || seconds > 86400.0) {
    throw std::invalid_argument(field + ": expected seconds in [0, 86400]");
  }
  return fsai::sim::Duration{static_cast<std::int64_t>(std::llround(seconds * 1e9))};
}

InterfaceConfig LoadInterfaceConfig(const std::filesystem::path &file) {
  try {
    const auto root = Read(file);
    InterfaceConfig result;
    result.command_topic = Text(root, "physical_command_topic");
    result.eufs_compatibility = root["enable_eufs_acceleration_compatibility"].as<bool>();
    result.command_timeout = SecondsToDuration(Number(root, "command_timeout_s"), "command_timeout_s");
    if (result.command_timeout.count() == 0 || Text(root, "sequence_policy") != "monotonic") {
      throw std::invalid_argument("positive timeout and sequence_policy=monotonic are required");
    }
    return result;
  } catch (const std::exception &e) {
    throw std::invalid_argument(file.string() + ": " + e.what());
  }
}

TrackConfig LoadTrack(const std::filesystem::path &directory) {
  try {
    const auto root = Read(directory / "track.yaml");
    Schema(root);
    TrackConfig result;
    result.name = Text(root, "name");
    result.frame_id = Text(root, "frame_id");
    if (result.frame_id != "map") {
      throw std::invalid_argument("frame_id: this planar runtime supports map");
    }
    const auto start = root["vehicle_start"];
    result.start.x_m = Number(start, "x_m");
    result.start.y_m = Number(start, "y_m");
    result.start.yaw_rad = Number(start, "yaw_rad");
    for (const auto *key : {"start_gate", "finish_gate"}) {
      const auto gate = root[key];
      (void)Number(gate, "x_m");
      (void)Number(gate, "y_m");
      (void)Number(gate, "yaw_rad");
      if (Number(gate, "width_m") <= 0) {
        throw std::invalid_argument(std::string(key) + ".width_m must be positive");
      }
    }
    std::ifstream csv(directory / "cones.csv");
    std::string line;
    if (!csv || !std::getline(csv, line)) {
      throw std::invalid_argument("cones.csv: missing or empty");
    }
    if (!line.empty() && line.back() == '\r') { line.pop_back(); }
    if (line != "tag,x,y,direction,x_variance,y_variance,xy_covariance") {
      throw std::invalid_argument("cones.csv:1: invalid header");
    }
    std::size_t row = 1;
    const std::set<std::string> colors{"blue", "yellow", "orange", "big_orange", "unknown"};
    while (std::getline(csv, line)) {
      ++row;
      if (!line.empty() && line.back() == '\r') { line.pop_back(); }
      if (line.empty()) { continue; }
      std::vector<std::string> fields;
      std::istringstream stream(line);
      std::string value;
      while (std::getline(stream, value, ',')) { fields.push_back(value); }
      const auto location = "cones.csv:" + std::to_string(row);
      if (fields.size() != 7 || !colors.contains(fields[0])) {
        throw std::invalid_argument(location + ": invalid field count or cone tag");
      }
      std::vector<double> values;
      for (std::size_t i = 1; i < fields.size(); ++i) {
        values.push_back(CsvNumber(fields[i], location + ":field " + std::to_string(i + 1)));
      }
      if (values[3] < 0 || values[4] < 0 || values[5] * values[5] > values[3] * values[4]) {
        throw std::invalid_argument(location + ": covariance must be positive semidefinite");
      }
      result.cones.push_back({fields[0], values[0], values[1]});
    }
    if (result.cones.empty()) { throw std::invalid_argument("cones.csv: no cones"); }
    return result;
  } catch (const std::exception &e) {
    throw std::invalid_argument(directory.string() + ": " + e.what());
  }
}

ScenarioConfig LoadScenario(const std::filesystem::path &file) {
  try {
    const auto root = Read(file);
    Schema(root);
    ScenarioConfig result;
    result.name = Text(root, "name");
    result.mission = Text(root, "mission");
    const std::set<std::string> missions{"manual", "acceleration", "skidpad", "autocross", "track_drive"};
    if (!missions.contains(result.mission)) { throw std::invalid_argument("unknown mission"); }
    result.vehicle_profile = Text(root, "vehicle_profile");
    result.track_bundle = Text(root, "track_bundle");
    result.mode = Text(root, "mode");
    if (result.mode != "realtime" && result.mode != "as_fast_as_possible") {
      throw std::invalid_argument("mode must be realtime or as_fast_as_possible");
    }
    const auto seed = root["seed"].as<std::int64_t>();
    if (seed < 0 || seed > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("seed out of uint32 range");
    }
    result.seed = static_cast<std::uint32_t>(seed);
    result.duration = SecondsToDuration(Number(root, "duration_limit_s"), "duration_limit_s");
    result.outer_step = SecondsToDuration(Number(root, "outer_step_ms") / 1000.0, "outer_step_ms");
    if (Number(root, "plant_step_ms") != 1.0) {
      throw std::invalid_argument("plant_step_ms: current integrator requires 1 ms");
    }
    if (result.duration.count() <= 0 || result.outer_step.count() <= 0 ||
        result.outer_step > std::chrono::milliseconds(100) ||
        result.outer_step.count() % 1000000 != 0 ||
        result.duration.count() % result.outer_step.count() != 0) {
      throw std::invalid_argument("duration must be positive and divisible by an outer step of 1..100 ms");
    }
    result.auto_start = root["auto_start"] ? root["auto_start"].as<bool>() : false;
    if (root["commands"]) {
      if (!root["commands"].IsSequence()) { throw std::invalid_argument("commands must be a list"); }
      for (const auto &entry : root["commands"]) {
        ScheduledCommand command;
        command.start = SecondsToDuration(Number(entry, "start_s"), "start_s");
        command.end = SecondsToDuration(Number(entry, "end_s"), "end_s");
        if (command.start < std::chrono::seconds(5) || command.end <= command.start ||
            command.end > result.duration ||
            (!result.commands.empty() && command.start < result.commands.back().end)) {
          throw std::invalid_argument("commands must be ordered, non-overlapping, after 5s readiness, and within duration");
        }
        auto &cmd = command.command;
        cmd.steering_angle_rad = entry["steering_angle_rad"] ? Number(entry, "steering_angle_rad") : 0;
        cmd.front_axle_torque_nm = entry["front_axle_torque_nm"] ? Number(entry, "front_axle_torque_nm") : 0;
        cmd.rear_axle_torque_nm = entry["rear_axle_torque_nm"] ? Number(entry, "rear_axle_torque_nm") : 0;
        cmd.friction_brake_ratio = entry["friction_brake_ratio"] ? Number(entry, "friction_brake_ratio") : 0;
        if (cmd.friction_brake_ratio < 0 || cmd.friction_brake_ratio > 1) {
          throw std::invalid_argument("friction_brake_ratio outside [0,1]");
        }
        result.commands.push_back(command);
      }
      if (!result.commands.empty() && !result.auto_start) {
        throw std::invalid_argument("scripted commands require auto_start=true");
      }
    }
    return result;
  } catch (const std::exception &e) {
    throw std::invalid_argument(file.string() + ": " + e.what());
  }
}
}  // namespace fsai::sim2_adapter
