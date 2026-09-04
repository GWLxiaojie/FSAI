#ifndef FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_
#define FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_

#include <chrono>  // NOLINT
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "eufs_sim2/simulation.hpp"
#include "fsai_sim2_adapter/core_factory.hpp"
#include "fsai_sim2_adapter/plugin_registry.hpp"

namespace fsai::sim2_adapter {

void RegisterDefaultComposition(CoreFactory &cores, PluginRegistry &plugins);

class FsaiSimulationNode : public rclcpp::Node {
 public:
  FsaiSimulationNode();

  void InitialisePlugins();

 private:
  void Step();

  static constexpr std::chrono::milliseconds kSimulationStep{5};

  CoreFactory core_factory_;
  PluginRegistry plugin_registry_;
  std::vector<std::string> plugin_names_;
  std::shared_ptr<eufs::sim2::SimulationBase> simulation_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_
