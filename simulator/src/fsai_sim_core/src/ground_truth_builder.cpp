#include "fsai_sim_core/ground_truth_builder.hpp"

namespace fsai::sim {

GroundTruth BuildGroundTruth(
  const ChassisState &accepted_state,
  const ActuatorState &actuator,
  const std::array<WheelState, 4> &wheels,
  const DynamicsEvaluation &evaluation) {
  GroundTruth truth;
  truth.chassis = accepted_state;
  truth.ax_body_mps2 =
    evaluation.derivative.u_mps2 - accepted_state.yaw_rate_radps * accepted_state.v_mps;
  truth.ay_body_mps2 =
    evaluation.derivative.v_mps2 + accepted_state.yaw_rate_radps * accepted_state.u_mps;
  truth.steering_angle_rad = actuator.steering_angle_rad;
  for (std::size_t i = 0; i < wheels.size(); ++i) {
    truth.wheel_omega_radps[i] = wheels[i].omega_radps;
  }
  truth.front_lateral_force_n = evaluation.front_lateral_force_n;
  truth.rear_lateral_force_n = evaluation.rear_lateral_force_n;
  return truth;
}

GroundTruth GroundTruthBuilder::Build(
  const ChassisState &accepted_state,
  const ActuatorState &actuator,
  const std::array<WheelState, 4> &wheels,
  const DynamicsEvaluation &evaluation) const {
  return BuildGroundTruth(accepted_state, actuator, wheels, evaluation);
}

}  // namespace fsai::sim
