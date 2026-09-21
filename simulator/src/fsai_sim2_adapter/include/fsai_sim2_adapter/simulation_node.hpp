#ifndef FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_
#define FSAI_SIM2_ADAPTER__SIMULATION_NODE_HPP_

#include <cstddef>
#include <array>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <eufs_msgs/msg/cone_array_with_covariance.hpp>
#include <eufs_msgs/msg/car_forces.hpp>
#include <eufs_msgs/msg/wheel_speeds_stamped.hpp>
#include <eufs_msgs/srv/set_mission.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "fsai_interfaces/msg/actuation_command.hpp"
#include "fsai_interfaces/msg/actuator_state.hpp"
#include "fsai_sim2_adapter/fsai_core_adapter.hpp"
#include "fsai_sim2_adapter/runtime_config.hpp"
#include "fsai_sim2_adapter/simulation_context.hpp"

namespace fsai::sim2_adapter {

void RegisterDefaultComposition(CoreFactory &cores, PluginRegistry &plugins);

class FsaiSimulationNode : public rclcpp::Node {
 public:
  explicit FsaiSimulationNode(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());
  void InitialisePlugins();
  // Uses the wall timer's exact path. Call only from the executor thread.
  void AdvanceOneStep();

 private:
  enum class SafetyState { kOff, kReady, kDriving, kEmergencyBrake, kFinished };
  void OnActuation(const fsai_interfaces::msg::ActuationCommand &message);
  void SetupPhysicalInterfaces();
  void PublishPhysicalState();
  void ResetPhysical();
  void ResetReference();
  void StepReference();
  void PublishReferenceStatus();
  void FinishReference(const std::string &outcome, const std::string &reason);
  void PublishTrackMarkers();
  void PublishJointStates();
  bool SelectMission(std::int16_t mission, std::string &reason);
  bool StartDriving(std::string &reason);
  FsaiCoreAdapter &Adapter();
  std::string StateName() const;

  CoreFactory core_factory_;
  PluginRegistry plugin_registry_;
  std::vector<std::string> plugin_names_;
  std::unique_ptr<ContextOwner> context_;
  std::string core_type_;
  InterfaceConfig interfaces_;
  ScenarioConfig scenario_;
  TrackConfig track_;
  fsai::sim::VehicleParameters vehicle_parameters_;
  RunMode run_mode_{RunMode::kRealtime};
  fsai::sim::Duration outer_step_{std::chrono::milliseconds(5)};
  SafetyState safety_state_{SafetyState::kOff};
  std::int16_t mission_{0};
  fsai::sim::SimTime ready_since_{};
  std::optional<std::uint64_t> last_sequence_;
  std::optional<fsai::sim::SimTime> last_stamp_;
  std::size_t max_steps_{0};
  std::size_t steps_taken_{0};
  bool stopped_{false};
  bool shutdown_on_finish_{true};
  bool scripted_run_{false};
  std::unique_ptr<ReferenceDriver> reference_driver_;
  ReferenceDriverOutput reference_output_;
  fsai::sim::SimTime next_reference_update_{};
  fsai::sim::SimTime reference_started_at_{};
  fsai::sim::Duration reference_fault_stop_duration_{};
  double max_abs_cte_m_{};
  double min_reference_clearance_m_{};
  std::string reference_outcome_{"ready"};
  std::string reference_failure_reason_;
  std::string report_path_;
  bool report_written_{false};
  std::array<double, 4> wheel_angles_{};
  fsai::sim::SimTime last_joint_time_{};
  bool track_markers_published_{false};
  std::mt19937 camera_rng_;
  std::mt19937 lidar_rng_;
  fsai::sim::SimTime next_cones_{};
  fsai::sim::SimTime next_wheels_{};
  fsai::sim::SimTime next_gnss_{};

  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_publisher_;
  rclcpp::Publisher<fsai_interfaces::msg::ActuatorState>::SharedPtr actuator_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr oss_publisher_;
  rclcpp::Publisher<eufs_msgs::msg::CarForces>::SharedPtr forces_publisher_;
  rclcpp::Publisher<eufs_msgs::msg::WheelSpeedsStamped>::SharedPtr wheel_publisher_;
  rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr camera_publisher_;
  rclcpp::Publisher<eufs_msgs::msg::ConeArrayWithCovariance>::SharedPtr lidar_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr track_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr safety_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mission_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr reference_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr reference_marker_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_publisher_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;
  rclcpp::Subscription<fsai_interfaces::msg::ActuationCommand>::SharedPtr actuation_subscription_;
  rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr eufs_command_subscription_;
  std::vector<rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr> trigger_services_;
  rclcpp::Service<eufs_msgs::srv::SetMission>::SharedPtr mission_service_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace fsai::sim2_adapter
#endif
