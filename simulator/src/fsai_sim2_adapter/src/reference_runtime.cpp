#include "fsai_sim2_adapter/simulation_node.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

#include <fcntl.h>
#include <unistd.h>
#include <json/json.h>

#include "eufs_sim2/time/time.hpp"

namespace fsai::sim2_adapter {
namespace {
void WriteExclusive(const std::filesystem::path &path, const std::string &bytes) {
  if (!path.parent_path().empty()) { std::filesystem::create_directories(path.parent_path()); }
  const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL, 0644);
  if (fd < 0) {
    throw std::runtime_error("cannot create new report " + path.string() + ": " + std::strerror(errno));
  }
  std::size_t offset = 0;
  while (offset < bytes.size()) {
    const auto count = ::write(fd, bytes.data() + offset, bytes.size() - offset);
    if (count < 0 && errno == EINTR) { continue; }
    if (count <= 0) {
      const std::string reason = std::strerror(errno);
      ::close(fd);
      throw std::runtime_error("report write failed: " + reason);
    }
    offset += static_cast<std::size_t>(count);
  }
  const int sync_result = ::fsync(fd);
  const int close_result = ::close(fd);
  if (sync_result != 0 || close_result != 0) { throw std::runtime_error("report flush failed"); }
}
}

void FsaiSimulationNode::ResetReference() {
  reference_driver_.reset();
  if (scenario_.reference_driver) {
    if (track_.control_centerline.empty()) {
      throw ConfigurationError("reference_driver requires track/control_centerline.csv");
    }
    ReferenceDriverConfig config;
    config.target_laps = scenario_.target_laps;
    config.cruise_speed_mps = scenario_.cruise_speed_mps;
    reference_driver_ = std::make_unique<ReferenceDriver>(
      track_.control_centerline, vehicle_parameters_, config);
  }
  reference_output_ = {};
  next_reference_update_ = reference_started_at_ = {};
  reference_fault_stop_duration_ = {};
  max_abs_cte_m_ = 0;
  min_reference_clearance_m_ = std::numeric_limits<double>::infinity();
  reference_outcome_ = "ready";
  reference_failure_reason_.clear();
  report_written_ = false;
}

void FsaiSimulationNode::StepReference() {
  if (!reference_driver_) { return; }
  const auto &plant = Adapter().plant_state();
  const auto now = plant.sim_time;
  if (safety_state_ != SafetyState::kDriving && safety_state_ != SafetyState::kEmergencyBrake) {
    return;
  }
  if (now < next_reference_update_) { return; }
  if (next_reference_update_.count() == 0) { reference_started_at_ = now; }
  next_reference_update_ = now + std::chrono::milliseconds(20);
  reference_output_ = reference_driver_->Update(plant);
  max_abs_cte_m_ = std::max(max_abs_cte_m_, std::abs(reference_output_.cross_track_error_m));
  min_reference_clearance_m_ = std::min(min_reference_clearance_m_, reference_output_.minimum_clearance_m);
  if (reference_output_.fault || !reference_failure_reason_.empty() ||
      safety_state_ == SafetyState::kEmergencyBrake) {
    if (reference_failure_reason_.empty()) {
      reference_failure_reason_ = reference_output_.reason.empty() ? "EBS requested" : reference_output_.reason;
    }
    reference_outcome_ = "braking_after_failure";
    safety_state_ = SafetyState::kEmergencyBrake;
    Adapter().RequestEbs();
    Adapter().SetPhysicalCommand({.friction_brake_ratio = 1.0}, now);
    if (std::hypot(plant.chassis.u_mps, plant.chassis.v_mps) < 0.02 &&
        std::abs(plant.chassis.yaw_rate_radps) < 0.02) {
      reference_fault_stop_duration_ += std::chrono::milliseconds(20);
    } else {
      reference_fault_stop_duration_ = {};
    }
    PublishReferenceStatus();
    if (reference_fault_stop_duration_ >= std::chrono::milliseconds(500)) {
      FinishReference("failed", reference_failure_reason_);
    }
    return;
  }
  Adapter().SetPhysicalCommand(reference_output_.command, now);
  reference_outcome_ = reference_output_.completed_laps >= scenario_.target_laps ? "stopping" : "running";
  PublishReferenceStatus();
  if (reference_output_.complete) { FinishReference("complete", reference_output_.reason); }
}

void FsaiSimulationNode::PublishReferenceStatus() {
  if (!reference_driver_ || !reference_publisher_) { return; }
  Json::Value status;
  status["mode"] = "ground_truth_reference_self_test";
  status["completed_laps"] = reference_output_.completed_laps;
  status["target_laps"] = scenario_.target_laps;
  status["progress_m"] = reference_output_.progress_m;
  status["track_length_m"] = reference_driver_->track_length_m();
  status["cross_track_error_m"] = reference_output_.cross_track_error_m;
  status["max_abs_cte_m"] = max_abs_cte_m_;
  if (std::isfinite(min_reference_clearance_m_)) { status["min_clearance_m"] = min_reference_clearance_m_; }
  else { status["min_clearance_m"] = Json::nullValue; }
  status["target_speed_mps"] = reference_output_.target_speed_mps;
  status["speed_mps"] = Adapter().plant_state().chassis.u_mps;
  status["simulation_time_s"] = Adapter().plant_state().sim_time.count() * 1e-9;
  status["outcome"] = reference_outcome_;
  status["reason"] = reference_failure_reason_.empty() ? reference_output_.reason : reference_failure_reason_;
  Json::StreamWriterBuilder writer;
  writer["indentation"] = "";
  std_msgs::msg::String message;
  message.data = Json::writeString(writer, status);
  reference_publisher_->publish(message);

  if (reference_marker_publisher_) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = eufs::sim2::time::TimeToTimeMsg(Adapter().GetTime());
    marker.header.frame_id = "map";
    marker.ns = "reference_status";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = Adapter().plant_state().chassis.x_m;
    marker.pose.position.y = Adapter().plant_state().chassis.y_m;
    marker.pose.position.z = 2.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = 0.28;
    marker.color.a = 1.0;
    marker.color.r = reference_outcome_ == "failed" ? 1.0 : 0.1;
    marker.color.g = reference_outcome_ == "failed" ? 0.1 : 1.0;
    std::ostringstream label;
    label << "Reference self-test\nLap " << reference_output_.completed_laps << " / " << scenario_.target_laps
      << "  " << reference_outcome_ << "\n" << std::fixed << std::setprecision(2)
      << Adapter().plant_state().chassis.u_mps << " m/s";
    marker.text = label.str();
    markers.markers.push_back(marker);
    reference_marker_publisher_->publish(markers);
  }
}

void FsaiSimulationNode::FinishReference(const std::string &outcome, const std::string &reason) {
  stopped_ = true;
  if (timer_) { timer_->cancel(); }
  reference_outcome_ = outcome;
  if (outcome != "complete") { reference_failure_reason_ = reason; }
  safety_state_ = outcome == "complete" ? SafetyState::kFinished : SafetyState::kEmergencyBrake;
  Adapter().SetDriving(false);
  PublishReferenceStatus();
  PublishPhysicalState();
  if (!report_path_.empty() && !report_written_) {
    Json::Value report;
    report["schema_version"] = 1;
    report["mode"] = "ground_truth_reference_self_test";
    report["track_name"] = track_.name;
    report["track_directory"] = track_.source_directory;
    report["control_centerline_sha256"] = track_.control_centerline_sha256;
    report["parameter_hash"] = Adapter().plant_state().parameter_hash;
    report["scenario_name"] = scenario_.name;
    report["seed"] = scenario_.seed;
    report["target_laps"] = scenario_.target_laps;
    report["completed_laps"] = reference_output_.completed_laps;
    report["progress_m"] = reference_output_.progress_m;
    report["track_length_m"] = reference_driver_->track_length_m();
    report["max_abs_cte_m"] = max_abs_cte_m_;
    if (std::isfinite(min_reference_clearance_m_)) { report["min_clearance_m"] = min_reference_clearance_m_; }
    else { report["min_clearance_m"] = Json::nullValue; }
    report["simulation_time_s"] = Adapter().plant_state().sim_time.count() * 1e-9;
    report["driving_time_s"] = (Adapter().plant_state().sim_time - reference_started_at_).count() * 1e-9;
    report["final_speed_mps"] = Adapter().plant_state().chassis.u_mps;
    report["outcome"] = outcome;
    report["reason"] = reason;
    report["cruise_speed_mps"] = scenario_.cruise_speed_mps;
    report["body_front_extent_m"] = 1.60;
    report["body_rear_extent_m"] = 1.34;
    report["body_width_m"] = 1.43;
    Json::StreamWriterBuilder writer;
    writer["indentation"] = "  ";
    WriteExclusive(report_path_, Json::writeString(writer, report) + "\n");
    report_written_ = true;
  }
  RCLCPP_INFO(get_logger(), "Reference run %s: %u/%u laps, t=%.2fs, %s", outcome.c_str(),
    reference_output_.completed_laps, scenario_.target_laps,
    Adapter().plant_state().sim_time.count() * 1e-9, reason.c_str());
  if (outcome != "complete") { throw std::runtime_error("reference run failed: " + reason); }
  if (shutdown_on_finish_) { rclcpp::shutdown(get_node_base_interface()->get_context()); }
}
}  // namespace fsai::sim2_adapter
