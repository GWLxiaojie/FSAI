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

FsaiSimulationNode::FsaiSimulationNode()
    : rclcpp::Node("eufs_sim2") {
  const auto core_params = declare_parameter<std::string>("core_params");

  eufs::vehicle_models::Param params;
  params.SetFromYaml(core_params);
  auto core = std::make_unique<eufs::sim2::core::EufsCore>(params);
  simulation_ = std::make_shared<eufs::sim2::SimulationBase>(std::move(core));

  clock_publisher_ = create_publisher<rosgraph_msgs::msg::Clock>("/clock", 1);
  timer_ = create_wall_timer(kSimulationStep, std::bind(&FsaiSimulationNode::Step, this));
}

void FsaiSimulationNode::InitialisePlugins() {
  declare_parameter<std::vector<std::string>>("plugin_names");
  const auto plugin_names =
    get_parameter("plugin_names").get_value<std::vector<std::string>>();

  for (const auto &name : plugin_names) {
    auto plugin = CreatePlugin(name);
    if (!plugin) {
      RCLCPP_ERROR(get_logger(), "Unknown EUFS plugin: %s", name.c_str());
      continue;
    }

    plugin->SetupROS(shared_from_this());
    plugin->CreateSensorFailureService(name);
    simulation_->RegisterPlugin(std::move(plugin));
  }
}

std::unique_ptr<eufs::sim2::plugin::Plugin> FsaiSimulationNode::CreatePlugin(
    const std::string &name) {
  if (name.starts_with("track_changer_plugin")) {
    return std::make_unique<eufs::sim2::plugin::TrackChangerPlugin>(name);
  }
  if (name.starts_with("control_input")) {
    return std::make_unique<eufs::sim2::plugin::ControlInputPlugin>(name);
  }
  if (name.starts_with("state_machine")) {
    return std::make_unique<eufs::sim2::plugin::StateMachinePlugin>(name);
  }
  if (name.starts_with("vehicle_state_plugin")) {
    return std::make_unique<eufs::sim2::plugin::VehicleStatePlugin>(name);
  }
  if (name.starts_with("wheel_speed_plugin")) {
    return std::make_unique<eufs::sim2::plugin::WheelSpeedPlugin>(name, "/wheel_speed");
  }
  if (name.starts_with("gnss_plugin")) {
    return std::make_unique<eufs::sim2::plugin::GNSSPlugin>(name);
  }
  if (name.starts_with("oss_plugin")) {
    return std::make_unique<eufs::sim2::plugin::OSSPlugin>(name, "/oss/data");
  }
  if (name.starts_with("imu_plugin")) {
    return std::make_unique<eufs::sim2::plugin::IMUPlugin>(name, "/imu/data");
  }
  if (name.starts_with("state_publisher")) {
    return std::make_unique<eufs::sim2::plugin::StatePublisherPlugin>(name);
  }
  if (name.starts_with("gt_transform")) {
    return std::make_unique<eufs::sim2::plugin::GTTransform>(name);
  }
  if (name.starts_with("force_publisher")) {
    return std::make_unique<eufs::sim2::plugin::ForcePublisher>(name);
  }
  if (name.starts_with("twist_publisher")) {
    return std::make_unique<eufs::sim2::plugin::TwistPublisher>(name);
  }
  if (name.starts_with("cone_fusion")) {
    return std::make_unique<eufs::sim2::plugin::ConeFusion>(name);
  }
  if (name.starts_with("cone_collision_tracker")) {
    return std::make_unique<eufs::sim2::plugin::ConeCollisionTracker>(name);
  }
  return nullptr;
}

void FsaiSimulationNode::Step() {
  simulation_->Step(kSimulationStep);
  clock_publisher_->publish(
    eufs::sim2::time::TimeToClockMsg(simulation_->GetCore().GetTime()));
}

}  // namespace fsai::sim2_adapter
