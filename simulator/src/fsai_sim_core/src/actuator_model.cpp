#include "fsai_sim_core/actuator_model.hpp"

#include <algorithm>
#include <cmath>

namespace fsai::sim {

namespace {

double ClampAbsRate(double current, double target, double max_rate, double dt_s) {
  const double max_delta = max_rate * dt_s;
  const double delta = std::clamp(target - current, -max_delta, max_delta);
  return current + delta;
}

}  // namespace

ActuatorResult ActuatorModel::Update(
  const ActuatorState &state,
  const Command &command,
  Duration dt,
  const VehicleParameters &parameters) const {
  Validate(command);
  if (dt <= Duration{0}) {
    throw ValidationError("actuator dt: " + std::to_string(dt.count()));
  }
  const double dt_s = std::chrono::duration<double>(dt).count();
  ActuatorResult result;
  result.state = state;

  const double steering_target = std::clamp(
    command.steering_angle_rad,
    -parameters.max_steering_angle_rad,
    parameters.max_steering_angle_rad);
  result.state.steering_angle_rad = ClampAbsRate(
    state.steering_angle_rad,
    steering_target,
    parameters.max_steering_rate_radps,
    dt_s);

  result.state.front_axle_torque_nm = std::clamp(
    command.front_axle_torque_nm,
    -parameters.max_front_axle_torque_nm,
    parameters.max_front_axle_torque_nm);
  result.state.rear_axle_torque_nm = std::clamp(
    command.rear_axle_torque_nm,
    -parameters.max_rear_axle_torque_nm,
    parameters.max_rear_axle_torque_nm);
  result.state.friction_brake_ratio = std::clamp(
    command.friction_brake_ratio, 0.0, 1.0);
  return result;
}

}  // namespace fsai::sim
