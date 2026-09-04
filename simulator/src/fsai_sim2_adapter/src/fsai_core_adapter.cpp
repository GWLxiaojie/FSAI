#include "fsai_sim2_adapter/fsai_core_adapter.hpp"

#include <cstdint>
#include <utility>

#include "eufs_sim2/type/state.hpp"
#include "fsai_sim_core/safety_chain.hpp"

namespace fsai::sim2_adapter {

FsaiCoreAdapter::FsaiCoreAdapter(fsai::sim::VehicleParameters parameters)
    : plant_(std::move(parameters)),
      plant_state_(plant_.InitialState()) {}

void FsaiCoreAdapter::Step(Duration dt) {
  const auto sim_dt = std::chrono::nanoseconds(static_cast<std::int64_t>(dt.count()));
  fsai::sim::SafetyInput safety_input;
  safety_input.as_driving = as_driving_;
  safety_input.ebs_request = plant_state_.ebs_latched;
  safety_input.command_age = plant_state_.sim_time - plant_state_.last_command_time;
  if (plant_state_.sim_time.count() == 0) {
    safety_input.command_age = fsai::sim::Duration{0};
  }
  auto resolved = fsai::sim::Resolve(
    pending_command_,
    safety_input,
    plant_state_,
    plant_.parameters());
  last_applied_ = resolved.command;
  plant_state_.ebs_latched = resolved.ebs_latched;
  last_result_ = plant_.Update(plant_state_, last_applied_, sim_dt);
  plant_state_ = last_result_.next_state;
}

void FsaiCoreAdapter::SetCommand(ControlInput cmd) {
  if (!eufs_compatibility_) {
    throw InterfaceError(
      "EUFS acceleration commands require enable_eufs_acceleration_compatibility");
  }
  fsai::sim::Command command;
  command.steering_angle_rad = cmd.steering_angle;
  command.rear_axle_torque_nm =
    cmd.acceleration * plant_.parameters().mass_kg *
    plant_.parameters().effective_tyre_radius_m;
  SetPhysicalCommand(command);
}

void FsaiCoreAdapter::SetPhysicalCommand(const fsai::sim::Command &command) {
  pending_command_ = command;
  plant_state_.last_command_time = plant_state_.sim_time;
}

void FsaiCoreAdapter::SetDriving(bool as_driving) {
  as_driving_ = as_driving;
}

void FsaiCoreAdapter::RequestEbs() {
  plant_state_.ebs_latched = true;
}

FsaiCoreAdapter::Time FsaiCoreAdapter::GetTime() const {
  return Time{static_cast<std::size_t>(plant_state_.sim_time.count())};
}

FsaiCoreAdapter::VehicleState::Vector FsaiCoreAdapter::GetState(
  VehicleState::Vector &vec) const {
  const auto &chassis = plant_state_.chassis;
  vec[eufs::sim2::type::VehicleStateMember::_x] = chassis.x_m;
  vec[eufs::sim2::type::VehicleStateMember::_y] = chassis.y_m;
  vec[eufs::sim2::type::VehicleStateMember::_z] = 0.0;
  vec[eufs::sim2::type::VehicleStateMember::_yaw] = chassis.yaw_rad;
  vec[eufs::sim2::type::VehicleStateMember::_v_x] = chassis.u_mps;
  vec[eufs::sim2::type::VehicleStateMember::_v_y] = chassis.v_mps;
  vec[eufs::sim2::type::VehicleStateMember::_v_yaw] = chassis.yaw_rate_radps;
  vec[eufs::sim2::type::VehicleStateMember::_a_x] = last_result_.ground_truth.ax_body_mps2;
  vec[eufs::sim2::type::VehicleStateMember::_a_y] = last_result_.ground_truth.ay_body_mps2;
  vec[eufs::sim2::type::VehicleStateMember::_steering] =
    plant_state_.actuator.steering_angle_rad;
  return vec;
}

FsaiCoreAdapter::WheelSpeeds::Vector FsaiCoreAdapter::GetState(
  WheelSpeeds::Vector &vec) const {
  const auto &wheels = plant_state_.wheels;
  vec[eufs::sim2::sensors::WheelSpeedsMember::_fl] = wheels[0].omega_radps;
  vec[eufs::sim2::sensors::WheelSpeedsMember::_fr] = wheels[1].omega_radps;
  vec[eufs::sim2::sensors::WheelSpeedsMember::_rl] = wheels[2].omega_radps;
  vec[eufs::sim2::sensors::WheelSpeedsMember::_rr] = wheels[3].omega_radps;
  vec[eufs::sim2::sensors::WheelSpeedsMember::_steering] =
    plant_state_.actuator.steering_angle_rad;
  return vec;
}

FsaiCoreAdapter::VehicleForces FsaiCoreAdapter::GetVehicleForces() const {
  VehicleForces forces;
  const double fy_f = last_result_.ground_truth.front_lateral_force_n / 2.0;
  const double fy_r = last_result_.ground_truth.rear_lateral_force_n / 2.0;
  forces[eufs::sim2::sensors::VehicleForcesMember::_fl_la] = fy_f;
  forces[eufs::sim2::sensors::VehicleForcesMember::_fr_la] = fy_f;
  forces[eufs::sim2::sensors::VehicleForcesMember::_rl_la] = fy_r;
  forces[eufs::sim2::sensors::VehicleForcesMember::_rr_la] = fy_r;
  return forces;
}

void FsaiCoreAdapter::Reset() {
  plant_state_ = plant_.InitialState();
  pending_command_ = {};
  last_applied_ = {};
  last_result_ = {};
}

}  // namespace fsai::sim2_adapter
