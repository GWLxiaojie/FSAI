#ifndef FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_
#define FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_

#include <chrono>  // NOLINT
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "eufs_sim2/simulation.hpp"
#include "fsai_sim2_adapter/core_factory.hpp"
#include "fsai_sim2_adapter/plugin_registry.hpp"
#include "fsai_sim2_adapter/simulation_context.hpp"
#include "fsai_sim2_adapter/simulation_runner.hpp"

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
  std::unique_ptr<ContextOwner> context_;
  RunMode run_mode_{RunMode::kRealtime};
  std::size_t max_steps_{0};
  std::size_t steps_taken_{0};
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_
