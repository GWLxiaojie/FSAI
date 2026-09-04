#ifndef FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_
#define FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_

#include <chrono>  // NOLINT
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

#include "eufs_sim2/plugin/base.hpp"
#include "eufs_sim2/simulation.hpp"

namespace fsai::sim2_adapter {

class FsaiSimulationNode : public rclcpp::Node {
 public:
  FsaiSimulationNode();

  void InitialisePlugins();

 private:
  std::unique_ptr<eufs::sim2::plugin::Plugin> CreatePlugin(const std::string &name);
  void Step();

  static constexpr std::chrono::milliseconds kSimulationStep{5};

  std::shared_ptr<eufs::sim2::SimulationBase> simulation_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_
