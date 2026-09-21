#include "fsai_sim2_adapter/simulation_node.hpp"

#include <cmath>
#include <functional>
#include <numbers>
#include <utility>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include "eufs_sim2/time/time.hpp"

namespace fsai::sim2_adapter {

std::string FsaiSimulationNode::StateName() const {
  switch (safety_state_) {
    case SafetyState::kOff: return "OFF";
    case SafetyState::kReady: return "READY";
    case SafetyState::kDriving: return "DRIVING";
    case SafetyState::kEmergencyBrake: return "EMERGENCY_BRAKE";
    case SafetyState::kFinished: return "FINISHED";
  }
  return "OFF";
}

bool FsaiSimulationNode::SelectMission(std::int16_t mission, std::string &reason) {
  if (safety_state_ != SafetyState::kOff || mission < 1 || mission > 10) {
    reason = "Select a valid mission while OFF; reset is required after EBS or finish";
    return false;
  }
  safety_state_ = SafetyState::kReady;
  mission_ = mission;
  ready_since_ = Adapter().plant_state().sim_time;
  reason = "READY; /go is available after five simulation seconds";
  return true;
}

bool FsaiSimulationNode::StartDriving(std::string &reason) {
  if (safety_state_ != SafetyState::kReady ||
      Adapter().plant_state().sim_time - ready_since_ < std::chrono::seconds(5)) {
    reason = "Vehicle must be READY for five simulation seconds before /go";
    return false;
  }
  safety_state_ = SafetyState::kDriving;
  Adapter().SetDriving(true);
  reason = "DRIVING";
  return true;
}

void FsaiSimulationNode::SetupPhysicalInterfaces() {
  actuator_publisher_ = create_publisher<fsai_interfaces::msg::ActuatorState>("/fsai/actuator_state", 10);
  odometry_publisher_ = create_publisher<nav_msgs::msg::Odometry>("/ground_truth/odom", 10);
  imu_publisher_ = create_publisher<sensor_msgs::msg::Imu>("/sensors/imu", 10);
  gnss_publisher_ = create_publisher<sensor_msgs::msg::NavSatFix>("/sensors/gnss", 10);
  oss_publisher_ = create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/sensors/oss", 10);
  forces_publisher_ = create_publisher<eufs_msgs::msg::CarForces>("/ground_truth/forces", 10);
  wheel_publisher_ = create_publisher<eufs_msgs::msg::WheelSpeedsStamped>("/sensors/wheel_speeds", 10);
  camera_publisher_ = create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>("/sensors/camera/cones", 10);
  lidar_publisher_ = create_publisher<eufs_msgs::msg::ConeArrayWithCovariance>("/sensors/lidar/cones", 10);
  rclcpp::PublisherOptions latched_options;
  latched_options.use_intra_process_comm = rclcpp::IntraProcessSetting::Disable;
  track_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
    "/ground_truth/track_markers", rclcpp::QoS(1).transient_local(), latched_options);
  safety_publisher_ = create_publisher<std_msgs::msg::String>("/sim/state/as_state", 10);
  mission_publisher_ = create_publisher<std_msgs::msg::String>("/sim/state/mission", 10);
  transform_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
  actuation_subscription_ = create_subscription<fsai_interfaces::msg::ActuationCommand>(
    interfaces_.command_topic, 10,
    std::bind(&FsaiSimulationNode::OnActuation, this, std::placeholders::_1));
  if (interfaces_.eufs_compatibility) {
    RCLCPP_WARN(get_logger(),
      "EUFS acceleration compatibility enabled: mass=%.6f kg, tyre radius=%.6f m; "
      "rear axle only, conversion revision 1, no resistance compensation",
      vehicle_parameters_.mass_kg, vehicle_parameters_.effective_tyre_radius_m);
    eufs_command_subscription_ = create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
      "/cmd", 10, [this](const ackermann_msgs::msg::AckermannDriveStamped &message) {
        fsai_interfaces::msg::ActuationCommand command;
        command.stamp = message.header.stamp;
        command.sequence = last_sequence_.value_or(0) + 1;
        command.steering_angle_rad = message.drive.steering_angle;
        const double torque = message.drive.acceleration * vehicle_parameters_.mass_kg *
          vehicle_parameters_.effective_tyre_radius_m;
        if (torque >= 0) {
          command.rear_axle_torque_nm = torque;
        } else {
          command.friction_brake_ratio = std::min(1.0, -torque / vehicle_parameters_.max_brake_torque_nm);
        }
        OnActuation(command);
      });
  }

  mission_service_ = create_service<eufs_msgs::srv::SetMission>("/set_mission",
    [this](const std::shared_ptr<eufs_msgs::srv::SetMission::Request> request,
    std::shared_ptr<eufs_msgs::srv::SetMission::Response> response) {
      response->success = SelectMission(request->mission, response->message);
    });
  auto add_service = [this](const std::string &name,
    std::function<bool(std::string &)> action) {
      trigger_services_.push_back(create_service<std_srvs::srv::Trigger>(name,
        [action](const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
          response->success = action(response->message);
        }));
    };
  add_service("/go", [this](std::string &reason) { return StartDriving(reason); });
  add_service("/ebs", [this](std::string &reason) {
    safety_state_ = SafetyState::kEmergencyBrake;
    Adapter().RequestEbs();
    reason = "EBS latched; /reset is required";
    return true;
  });
  add_service("/reset", [this](std::string &reason) {
    try {
      ResetPhysical();
      // Reset returns to OFF even for a scripted run. A new launch starts a new
      // scripted experiment; /reset gives control back to the service client.
      scripted_run_ = false;
      if (timer_) { timer_->reset(); }
      clock_publisher_->publish(eufs::sim2::time::TimeToClockMsg(Adapter().GetTime()));
      PublishPhysicalState();
      reason = "Reset complete: OFF, simulation time zero, original track pose and seed restored";
      return true;
    } catch (const std::exception &error) {
      reason = error.what();
      return false;
    }
  });
  PublishPhysicalState();
}

void FsaiSimulationNode::PublishPhysicalState() {
  const auto &adapter = Adapter();
  const auto &plant = adapter.plant_state();
  const auto &chassis = plant.chassis;
  const auto stamp = eufs::sim2::time::TimeToTimeMsg(adapter.GetTime());
  const double c = std::cos(chassis.yaw_rad);
  const double s = std::sin(chassis.yaw_rad);
  geometry_msgs::msg::Quaternion orientation;
  orientation.z = std::sin(chassis.yaw_rad / 2.0);
  orientation.w = std::cos(chassis.yaw_rad / 2.0);

  fsai_interfaces::msg::ActuatorState actuator;
  actuator.stamp = stamp;
  actuator.steering_angle_rad = plant.actuator.steering_angle_rad;
  actuator.front_axle_torque_nm = plant.actuator.front_axle_torque_nm;
  actuator.rear_axle_torque_nm = plant.actuator.rear_axle_torque_nm;
  actuator.friction_brake_ratio = plant.actuator.friction_brake_ratio;
  actuator.u_mps = chassis.u_mps;
  actuator.v_mps = chassis.v_mps;
  actuator.yaw_rate_radps = chassis.yaw_rate_radps;
  actuator_publisher_->publish(actuator);

  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = track_.frame_id;
  odometry.child_frame_id = "base_footprint";
  odometry.pose.pose.position.x = chassis.x_m;
  odometry.pose.pose.position.y = chassis.y_m;
  odometry.pose.pose.orientation = orientation;
  odometry.twist.twist.linear.x = chassis.u_mps;
  odometry.twist.twist.linear.y = chassis.v_mps;
  odometry.twist.twist.angular.z = chassis.yaw_rate_radps;
  odometry_publisher_->publish(odometry);

  const auto &truth = adapter.last_result().ground_truth;
  eufs_msgs::msg::CarForces forces;
  forces.front_left_lateral = forces.front_right_lateral = truth.front_lateral_force_n / 2.0;
  forces.rear_left_lateral = forces.rear_right_lateral = truth.rear_lateral_force_n / 2.0;
  forces.front_left_longitudinal = forces.front_right_longitudinal = truth.front_longitudinal_force_n / 2.0;
  forces.rear_left_longitudinal = forces.rear_right_longitudinal = truth.rear_longitudinal_force_n / 2.0;
  forces.front_left_vertical = forces.front_right_vertical = truth.front_normal_force_n / 2.0;
  forces.rear_left_vertical = forces.rear_right_vertical = truth.rear_normal_force_n / 2.0;
  forces_publisher_->publish(forces);

  geometry_msgs::msg::TransformStamped transform;
  transform.header = odometry.header;
  transform.child_frame_id = odometry.child_frame_id;
  transform.transform.translation.x = chassis.x_m;
  transform.transform.translation.y = chassis.y_m;
  transform.transform.rotation = orientation;
  transform_broadcaster_->sendTransform(transform);

  // Ideal planar sensors: their missing hardware errors are explicit rather
  // than borrowing uncontrolled upstream RNG or uncalibrated error models.
  if (plant.sim_time.count() % 5000000 == 0) {
    sensor_msgs::msg::Imu imu;
    imu.header.stamp = stamp;
    imu.header.frame_id = "base_footprint";
    imu.orientation = orientation;
    imu.angular_velocity.z = chassis.yaw_rate_radps;
    imu.linear_acceleration.x = adapter.last_result().ground_truth.ax_body_mps2;
    imu.linear_acceleration.y = adapter.last_result().ground_truth.ay_body_mps2;
    imu.linear_acceleration.z = vehicle_parameters_.gravity_mps2;
    imu_publisher_->publish(imu);
  }
  if (plant.sim_time >= next_wheels_) {
    geometry_msgs::msg::TwistWithCovarianceStamped oss;
    oss.header.stamp = stamp;
    oss.header.frame_id = "base_footprint";
    oss.twist.twist.linear.x = chassis.u_mps;
    oss.twist.twist.linear.y = chassis.v_mps;
    oss_publisher_->publish(oss);
    eufs_msgs::msg::WheelSpeedsStamped wheels;
    wheels.header.stamp = stamp;
    wheels.header.frame_id = "base_footprint";
    constexpr double revolutions = 2.0 * std::numbers::pi;
    wheels.speeds.lf_speed = plant.wheels[0].omega_radps / revolutions;
    wheels.speeds.rf_speed = plant.wheels[1].omega_radps / revolutions;
    wheels.speeds.lb_speed = plant.wheels[2].omega_radps / revolutions;
    wheels.speeds.rb_speed = plant.wheels[3].omega_radps / revolutions;
    wheels.speeds.steering = plant.actuator.steering_angle_rad;
    wheel_publisher_->publish(wheels);
    next_wheels_ = plant.sim_time + std::chrono::milliseconds(10);
  }
  if (plant.sim_time >= next_gnss_) {
    sensor_msgs::msg::NavSatFix fix;
    fix.header.stamp = stamp;
    fix.header.frame_id = "base_footprint";
    // Local tangent-plane synthetic origin (0 deg, 0 deg), not a surveyed track.
    constexpr double earth_radius = 6378137.0;
    fix.latitude = chassis.y_m / earth_radius * 180.0 / std::numbers::pi;
    fix.longitude = chassis.x_m / earth_radius * 180.0 / std::numbers::pi;
    fix.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    gnss_publisher_->publish(fix);
    next_gnss_ = plant.sim_time + std::chrono::milliseconds(100);
  }

  if (plant.sim_time >= next_cones_) {
    auto observation = [&](double range, double half_fov, double sigma,
      bool colors, std::mt19937 &rng) {
      eufs_msgs::msg::ConeArrayWithCovariance message;
      message.header.stamp = stamp;
      message.header.frame_id = "base_footprint";
      std::normal_distribution<double> noise(0.0, sigma);
      for (const auto &cone : track_.cones) {
        const double dx = cone.x - chassis.x_m;
        const double dy = cone.y - chassis.y_m;
        const double x = c * dx + s * dy;
        const double y = -s * dx + c * dy;
        if (std::hypot(x, y) > range || std::hypot(x, y) < 0.2 ||
            std::abs(std::atan2(y, x)) > half_fov) { continue; }
        eufs_msgs::msg::ConeWithCovariance observed;
        observed.point.x = x + noise(rng);
        observed.point.y = y + noise(rng);
        observed.covariance = {sigma * sigma, 0.0, 0.0, sigma * sigma};
        if (!colors || cone.color == "unknown") { message.unknown_color_cones.push_back(observed); }
        else if (cone.color == "blue") { message.blue_cones.push_back(observed); }
        else if (cone.color == "yellow") { message.yellow_cones.push_back(observed); }
        else if (cone.color == "orange") { message.orange_cones.push_back(observed); }
        else { message.big_orange_cones.push_back(observed); }
      }
      return message;
    };
    camera_publisher_->publish(observation(20.0, 0.96, 0.03, true, camera_rng_));
    lidar_publisher_->publish(observation(100.0, std::numbers::pi / 2.0, 0.02, false, lidar_rng_));
    next_cones_ = plant.sim_time + std::chrono::milliseconds(50);

    visualization_msgs::msg::MarkerArray markers;
    for (std::size_t i = 0; i < track_.cones.size(); ++i) {
      const auto &cone = track_.cones[i];
      visualization_msgs::msg::Marker marker;
      marker.header = odometry.header;
      marker.ns = "track";
      marker.id = static_cast<int>(i);
      marker.type = visualization_msgs::msg::Marker::CYLINDER;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.position.x = cone.x;
      marker.pose.position.y = cone.y;
      marker.pose.position.z = 0.15;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = marker.scale.y = 0.22;
      marker.scale.z = 0.3;
      marker.color.a = 1.0;
      if (cone.color == "blue") { marker.color.b = 1.0; }
      else if (cone.color == "yellow") { marker.color.r = marker.color.g = 1.0; }
      else if (cone.color == "orange" || cone.color == "big_orange") {
        marker.color.r = 1.0; marker.color.g = 0.4;
      } else { marker.color.r = marker.color.g = marker.color.b = 0.5; }
      markers.markers.push_back(marker);
    }
    track_publisher_->publish(markers);
  }
  std_msgs::msg::String safety;
  safety.data = StateName();
  safety_publisher_->publish(safety);
  static const char *missions[] = {"not_selected", "acceleration", "skidpad", "autocross",
    "track_drive", "manual", "ads_ebs_test", "ads_inspection", "ddt_inspection_a",
    "ddt_inspection_b", "ddt_autonomous_demo"};
  std_msgs::msg::String mission;
  mission.data = missions[mission_];
  mission_publisher_->publish(mission);
}
}  // namespace fsai::sim2_adapter
