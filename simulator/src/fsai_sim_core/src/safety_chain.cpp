#include "fsai_sim_core/safety_chain.hpp"

#include <algorithm>

namespace fsai::sim {

SafetyResult Resolve(
  const Command &command,
  const SafetyInput &input,
  const PlantState &state,
  const VehicleParameters &parameters) {
  Validate(command);
  SafetyResult result;
  result.command = command;
  result.ebs_latched = state.ebs_latched;

  auto zero_drive = [&]() {
    result.command.front_axle_torque_nm = 0.0;
    result.command.rear_axle_torque_nm = 0.0;
  };

  if (input.ebs_request || state.ebs_latched) {
    result.ebs_latched = true;
    zero_drive();
    result.command.friction_brake_ratio = parameters.ebs_brake_ratio;
    result.events.push_back(SimulationEvent{
      .time = state.sim_time,
      .type = EventType::kEbsLatched,
      .detail = "ebs",
    });
    return result;
  }

  if (input.command_age > parameters.command_timeout) {
    zero_drive();
    result.command.friction_brake_ratio = parameters.timeout_brake_ratio;
    result.events.push_back(SimulationEvent{
      .time = state.sim_time,
      .type = EventType::kCommandTimeout,
      .detail = "command timeout",
    });
    return result;
  }

  if (!input.as_driving) {
    zero_drive();
    result.events.push_back(SimulationEvent{
      .time = state.sim_time,
      .type = EventType::kNonDriving,
      .detail = "not in driving state",
    });
    return result;
  }

  result.command.steering_angle_rad = std::clamp(
    result.command.steering_angle_rad,
    -parameters.max_steering_angle_rad,
    parameters.max_steering_angle_rad);
  result.command.front_axle_torque_nm = std::clamp(
    result.command.front_axle_torque_nm,
    -parameters.max_front_axle_torque_nm,
    parameters.max_front_axle_torque_nm);
  result.command.rear_axle_torque_nm = std::clamp(
    result.command.rear_axle_torque_nm,
    -parameters.max_rear_axle_torque_nm,
    parameters.max_rear_axle_torque_nm);
  result.command.friction_brake_ratio = std::clamp(
    result.command.friction_brake_ratio, 0.0, 1.0);
  return result;
}

}  // namespace fsai::sim
