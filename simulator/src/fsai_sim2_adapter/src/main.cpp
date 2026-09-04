#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "fsai_sim2_adapter/simulation_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<fsai::sim2_adapter::FsaiSimulationNode>();
  node->InitialisePlugins();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
