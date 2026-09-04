#include "fsai_sim2_adapter/simulation_node.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "eufs_sim2/core/eufs_core.hpp"
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

FsaiSimulationNode::FsaiSimulationNode()
    : rclcpp::Node("eufs_sim2") {
  RegisterDefaultComposition(core_factory_, plugin_registry_);

  const auto core_params = declare_parameter<std::string>("core_params");
  declare_parameter<std::vector<std::string>>("plugin_names");
  plugin_names_ = get_parameter("plugin_names").get_value<std::vector<std::string>>();
  for (const auto &name : plugin_names_) {
    plugin_registry_.Validate(name);
  }

  auto core = core_factory_.Create(
    "eufs",
    CoreConfig{.parameter_file = core_params});
  simulation_ = std::make_shared<eufs::sim2::SimulationBase>(std::move(core));

  clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  timer_ = create_wall_timer(kSimulationStep, std::bind(&FsaiSimulationNode::Step, this));
}

void FsaiSimulationNode::InitialisePlugins() {
  for (const auto &name : plugin_names_) {
    auto plugin = plugin_registry_.Create(name);
    plugin->SetupROS(shared_from_this());
    plugin->CreateSensorFailureService(name);
    simulation_->RegisterPlugin(std::move(plugin));
  }
}

void FsaiSimulationNode::Step() {
  simulation_->Step(kSimulationStep);
  clock_publisher_->publish(
    eufs::sim2::time::TimeToClockMsg(simulation_->GetCore().GetTime()));
}

}  // namespace fsai::sim2_adapter
