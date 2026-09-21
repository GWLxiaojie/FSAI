#include "fsai_sim2_adapter/simulation_node.hpp"

#include <functional>
#include <memory>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "fsai_sim_core/types.hpp"

#include "eufs_sim2/core/eufs_core.hpp"
#include "fsai_sim2_adapter/fsai_core_adapter.hpp"
#include "fsai_sim2_adapter/vehicle_profile_loader.hpp"
#include "eufs_sim2/plugin/cone_collision_tracker.hpp"
#include "eufs_sim2/plugin/cone_fusion.hpp"
#include "eufs_sim2/plugin/control_input.hpp"
#include "eufs_sim2/plugin/force_publisher.hpp"
#include "eufs_sim2/plugin/gnss.hpp"
#include "eufs_sim2/plugin/gt_transform.hpp"
#include "eufs_sim2/plugin/imu_plugin.hpp"
#include "eufs_sim2/plugin/oss_plugin.hpp"
#include "eufs_sim2/plugin/state_machine_plugin.hpp"
#include "eufs_sim2/plugin/state_publisher.hpp"
#include "eufs_sim2/plugin/track_changer.hpp"
#include "eufs_sim2/plugin/twist_publisher.hpp"
#include "eufs_sim2/plugin/vehicle_state.hpp"
#include "eufs_sim2/plugin/wheel_speed.hpp"
#include "eufs_sim2/time/time.hpp"
#include "vehicle_models/types/param.hpp"

namespace fsai::sim2_adapter {

void RegisterDefaultComposition(CoreFactory &cores, PluginRegistry &plugins) {
  cores.Register("eufs", [](const CoreConfig &config) {
    eufs::vehicle_models::Param params;
    params.SetFromYaml(config.parameter_file.string());
    return std::make_unique<eufs::sim2::core::EufsCore>(params);
  });
  cores.Register("fsai", [](const CoreConfig &config) {
    auto parameters = LoadVehicleProfile(config.parameter_file);
    return std::make_unique<FsaiCoreAdapter>(std::move(parameters));
  });

  plugins.Register("track_changer_plugin", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::TrackChangerPlugin>(std::move(name));
  });
  plugins.Register("control_input", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::ControlInputPlugin>(std::move(name));
  });
  plugins.Register("state_machine", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::StateMachinePlugin>(std::move(name));
  });
  plugins.Register("vehicle_state_plugin", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::VehicleStatePlugin>(std::move(name));
  });
  plugins.Register("wheel_speed_plugin", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::WheelSpeedPlugin>(std::move(name), "/wheel_speed");
  });
  plugins.Register("gnss_plugin", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::GNSSPlugin>(std::move(name));
  });
  plugins.Register("oss_plugin", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::OSSPlugin>(std::move(name), "/oss/data");
  });
  plugins.Register("imu_plugin", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::IMUPlugin>(std::move(name), "/imu/data");
  });
  plugins.Register("state_publisher", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::StatePublisherPlugin>(std::move(name));
  });
  plugins.Register("gt_transform", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::GTTransform>(std::move(name));
  });
  plugins.Register("force_publisher", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::ForcePublisher>(std::move(name));
  });
  plugins.Register("twist_publisher", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::TwistPublisher>(std::move(name));
  });
  plugins.Register("cone_fusion", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::ConeFusion>(std::move(name));
  });
  plugins.Register("cone_collision_tracker", [](std::string name) {
    return std::make_unique<eufs::sim2::plugin::ConeCollisionTracker>(std::move(name));
  });
}

FsaiSimulationNode::FsaiSimulationNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("eufs_sim2", options) {
  RegisterDefaultComposition(core_factory_, plugin_registry_);
  core_type_ = declare_parameter<std::string>("core_type", "fsai");
  const auto core_params = declare_parameter<std::string>("core_params", "");
  if (core_params.empty()) { throw ConfigurationError("core_params must not be empty"); }
  const auto track_path = declare_parameter<std::string>("track", "");
  const auto scenario_path = declare_parameter<std::string>("scenario", "");
  const auto run_mode = declare_parameter<std::string>("run_mode", "");
  const auto max_steps = declare_parameter<int>("max_steps", 0);
  if (max_steps < 0) { throw ConfigurationError("max_steps must not be negative"); }
  max_steps_ = static_cast<std::size_t>(max_steps);
  shutdown_on_finish_ = declare_parameter<bool>("shutdown_on_finish", true);
  const bool enable_timer = declare_parameter<bool>("enable_timer", true);
  plugin_names_ = declare_parameter<std::vector<std::string>>("plugin_names", std::vector<std::string>{});

  if (core_type_ == "fsai") {
    if (!plugin_names_.empty()) {
      throw ConfigurationError("EUFS plugins may only be loaded by core_type=eufs");
    }
    scenario_ = LoadScenario(scenario_path);
    track_ = LoadTrack(track_path);
    interfaces_ = LoadInterfaceConfig(std::filesystem::path(core_params) / "interfaces.yaml");
    vehicle_parameters_ = LoadVehicleProfile(core_params);
    report_path_ = declare_parameter<std::string>("report_path", "");
    if (!report_path_.empty() && std::filesystem::exists(report_path_)) {
      throw ConfigurationError("report_path already exists; reports are never overwritten: " + report_path_);
    }
    const auto duration = declare_parameter<double>("duration_limit_s", -1.0);
    const auto seed = declare_parameter<std::int64_t>("seed", -1);
    const auto step_ms = declare_parameter<int>("outer_step_ms", -1);
    if (duration != -1.0) {
      scenario_.duration = SecondsToDuration(duration, "duration_limit_s");
    }
    if (seed < -1 || seed > std::numeric_limits<std::uint32_t>::max()) {
      throw ConfigurationError("seed must be -1 or a uint32 value");
    }
    if (seed >= 0) { scenario_.seed = static_cast<std::uint32_t>(seed); }
    if (step_ms != -1) {
      scenario_.outer_step = SecondsToDuration(step_ms / 1000.0, "outer_step_ms");
    }
    // Sensor sample times lie on 5 ms boundaries; unsupported step sizes must
    // never silently change their sample times.
    if (scenario_.outer_step.count() <= 0 ||
        std::chrono::milliseconds(5) % scenario_.outer_step != fsai::sim::Duration{0} ||
        scenario_.duration.count() <= 0 ||
        scenario_.duration % scenario_.outer_step != fsai::sim::Duration{0}) {
      throw ConfigurationError("outer_step_ms must divide 5 ms and duration must be a positive multiple");
    }
    for (const auto &command : scenario_.commands) {
      if (command.end > scenario_.duration) {
        throw ConfigurationError("duration override truncates a scripted command");
      }
    }
    outer_step_ = scenario_.outer_step;
    scripted_run_ = scenario_.auto_start;
    run_mode_ = ParseRunMode(run_mode.empty() ? scenario_.mode : run_mode);
    // Explicit ROS override can enable a compatibility subscriber; the physical
    // safety chain remains authoritative.
    interfaces_.eufs_compatibility = declare_parameter<bool>(
      "enable_eufs_acceleration_compatibility", interfaces_.eufs_compatibility);
  } else {
    for (const auto &name : plugin_names_) { plugin_registry_.Validate(name); }
    run_mode_ = ParseRunMode(run_mode.empty() ? "realtime" : run_mode);
  }

  SimulationContextFactory factory;
  context_ = std::make_unique<ContextOwner>(factory.Create(
    core_factory_, plugin_registry_, core_type_, CoreConfig{.parameter_file = core_params},
    eufs::sim2::time::Duration{static_cast<std::size_t>(outer_step_.count())}));
  clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
  if (core_type_ == "fsai") {
    ResetPhysical();
    SetupPhysicalInterfaces();
  }
  if (enable_timer) {
    const auto wall_period = run_mode_ == RunMode::kAsFastAsPossible ?
      std::chrono::nanoseconds(1) : outer_step_;
    timer_ = create_wall_timer(wall_period, [this]() {
      try {
        AdvanceOneStep();
      } catch (const std::exception &error) {
        RCLCPP_FATAL(get_logger(), "Simulation failed at step %zu: %s", steps_taken_, error.what());
        if (timer_) { timer_->cancel(); }
        stopped_ = true;
        throw;
      }
    });
  }
}

FsaiCoreAdapter &FsaiSimulationNode::Adapter() {
  return dynamic_cast<FsaiCoreAdapter &>(context_->Current().simulation->GetCore());
}

void FsaiSimulationNode::ResetPhysical() {
  auto core = std::make_unique<FsaiCoreAdapter>(vehicle_parameters_);
  core->SetInitialPose(track_.start);
  core->enable_eufs_acceleration_compatibility(interfaces_.eufs_compatibility);
  SimulationContext replacement;
  replacement.simulation = std::make_shared<eufs::sim2::SimulationBase>(std::move(core));
  replacement.runner = std::make_unique<SimulationRunner>(
    *replacement.simulation, eufs::sim2::time::Duration{static_cast<std::size_t>(outer_step_.count())});
  context_->Replace(std::move(replacement));
  safety_state_ = SafetyState::kOff;
  mission_ = 0;
  ready_since_ = {};
  last_sequence_.reset();
  last_stamp_.reset();
  camera_rng_.seed(scenario_.seed);
  lidar_rng_.seed(scenario_.seed ^ 0x9e3779b9U);
  next_cones_ = next_wheels_ = next_gnss_ = {};
  steps_taken_ = 0;
  stopped_ = false;
  wheel_angles_ = {};
  last_joint_time_ = {};
  track_markers_published_ = false;
  ResetReference();
}

void FsaiSimulationNode::OnActuation(
  const fsai_interfaces::msg::ActuationCommand &message) {
  if (stopped_ || safety_state_ != SafetyState::kDriving || scripted_run_) { return; }
  const auto now = Adapter().plant_state().sim_time;
  if (message.stamp.sec < 0 || message.stamp.nanosec >= 1000000000U) {
    RCLCPP_WARN(get_logger(), "Rejected malformed command stamp");
    return;
  }
  const auto stamp = std::chrono::seconds(message.stamp.sec) +
    std::chrono::nanoseconds(message.stamp.nanosec);
  if (stamp > now || now - stamp >= interfaces_.command_timeout ||
      (last_stamp_ && stamp < *last_stamp_) ||
      (last_sequence_ && message.sequence <= *last_sequence_)) {
    RCLCPP_WARN(get_logger(), "Rejected stale, future, or non-monotonic command");
    return;
  }
  fsai::sim::Command command;
  command.steering_angle_rad = message.steering_angle_rad;
  command.front_axle_torque_nm = message.front_axle_torque_nm;
  command.rear_axle_torque_nm = message.rear_axle_torque_nm;
  command.friction_brake_ratio = message.friction_brake_ratio;
  try {
    Adapter().SetPhysicalCommand(command, stamp);
    last_sequence_ = message.sequence;
    last_stamp_ = stamp;
  } catch (const std::exception &error) {
    RCLCPP_WARN(get_logger(), "Rejected command: %s", error.what());
  }
}

void FsaiSimulationNode::InitialisePlugins() {
  if (core_type_ == "fsai") { return; }
  for (const auto &name : plugin_names_) {
    auto plugin = plugin_registry_.Create(name);
    plugin->SetupROS(shared_from_this());
    plugin->CreateSensorFailureService(name);
    context_->Current().simulation->RegisterPlugin(std::move(plugin));
  }
}

void FsaiSimulationNode::AdvanceOneStep() {
  if (stopped_) { return; }
  if (core_type_ == "fsai" && scripted_run_) {
    const auto now = Adapter().plant_state().sim_time;
    std::string reason;
    if (safety_state_ == SafetyState::kOff) {
      const std::int16_t mission = scenario_.mission == "acceleration" ? 1 :
        scenario_.mission == "skidpad" ? 2 : scenario_.mission == "autocross" ? 3 :
        scenario_.mission == "track_drive" ? 4 : 5;
      (void)SelectMission(mission, reason);
    }
    if (safety_state_ == SafetyState::kReady && now - ready_since_ >= std::chrono::seconds(5)) {
      (void)StartDriving(reason);
    }
    if (safety_state_ == SafetyState::kDriving && !scenario_.reference_driver) {
      for (const auto &scheduled : scenario_.commands) {
        if (now >= scheduled.start && now < scheduled.end) {
          Adapter().SetPhysicalCommand(scheduled.command, now);
          break;
        }
      }
    }
  }
  if (core_type_ == "fsai" && scenario_.reference_driver && scripted_run_) {
    const auto now = Adapter().plant_state().sim_time;
    if (reference_failure_reason_.empty() &&
        ((max_steps_ > 0 && steps_taken_ >= max_steps_) || now >= scenario_.duration)) {
      reference_failure_reason_ = "reference run reached its time/step limit before successful completion";
      safety_state_ = SafetyState::kEmergencyBrake;
      Adapter().RequestEbs();
    }
    StepReference();
    if (stopped_) { return; }
  }
  try {
    context_->Current().runner->StepOnce();
  } catch (const std::exception &error) {
    if (reference_driver_ && scripted_run_ && !report_written_) {
      FinishReference("failed", std::string("plant failed: ") + error.what());
    }
    throw;
  }
  ++steps_taken_;
  const auto time = context_->Current().simulation->GetCore().GetTime();
  clock_publisher_->publish(eufs::sim2::time::TimeToClockMsg(time));
  if (core_type_ == "fsai") { PublishPhysicalState(); }
  if (!(core_type_ == "fsai" && scenario_.reference_driver && scripted_run_) &&
      ((max_steps_ > 0 && steps_taken_ >= max_steps_) ||
      (core_type_ == "fsai" && time.count() >= static_cast<std::size_t>(scenario_.duration.count())))) {
    stopped_ = true;
    if (core_type_ == "fsai") {
      safety_state_ = SafetyState::kFinished;
      Adapter().SetDriving(false);
      std_msgs::msg::String state;
      state.data = StateName();
      safety_publisher_->publish(state);
    }
    if (timer_) { timer_->cancel(); }
    RCLCPP_INFO(get_logger(), "Simulation completed after %zu steps (%.3f simulation seconds)",
      steps_taken_, time.count() * 1e-9);
    if (shutdown_on_finish_) { rclcpp::shutdown(get_node_base_interface()->get_context()); }
  }
}

}  // namespace fsai::sim2_adapter
