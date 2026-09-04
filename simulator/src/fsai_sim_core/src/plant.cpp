#include "fsai_sim_core/plant.hpp"

#include <utility>

namespace fsai::sim {

Plant::Plant(VehicleParameters parameters)
    : parameters_(parameters),
      parameter_hash_(ParameterHash(parameters)) {
  ValidateParameters(parameters_);
}

PlantState Plant::InitialState() const {
  PlantState state;
  state.parameter_hash = parameter_hash_;
  state.backend_id = "fsai_bicycle";
  state.backend_revision = "1";
  return state;
}

StepResult Plant::Update(
  const PlantState &state,
  const Command &command,
  Duration dt) const {
  Validate(command);
  if (state.schema_version != 1) {
    throw ValidationError("schema_version: " + std::to_string(state.schema_version));
  }
  if (state.backend_id != "fsai_bicycle" || state.backend_revision != "1") {
    throw ValidationError("backend_id: " + state.backend_id);
  }
  if (state.parameter_hash != parameter_hash_) {
    throw ValidationError("parameter_hash: " + state.parameter_hash);
  }

  StepResult result;
  result.diagnostics.stage_order = {"actuator", "integrator", "ground_truth"};

  const auto actuator = actuators_.Update(state.actuator, command, dt, parameters_);
  auto integrated = integrator_.Integrate(
    state.chassis, actuator.state, dt, parameters_, state.wheels);

  result.next_state = state;
  result.next_state.chassis = integrated.state;
  result.next_state.actuator = actuator.state;
  result.next_state.wheels = integrated.wheels;
  result.next_state.sim_time = state.sim_time + dt;
  result.next_state.last_command_time = result.next_state.sim_time;
  result.next_state.parameter_hash = parameter_hash_;

  result.ground_truth = truth_builder_.Build(
    integrated.state,
    actuator.state,
    integrated.wheels,
    integrated.last_evaluation);
  result.events = std::move(integrated.events);
  result.diagnostics.internal_steps = integrated.diagnostics.internal_steps;
  result.diagnostics.front_slip_angle_rad =
    integrated.last_evaluation.front_slip_angle_rad;
  result.diagnostics.rear_slip_angle_rad =
    integrated.last_evaluation.rear_slip_angle_rad;
  result.diagnostics.drive_force_n = integrated.last_evaluation.drive_force_n;
  result.diagnostics.drag_force_n = integrated.last_evaluation.drag_force_n;
  result.diagnostics.roll_force_n = integrated.last_evaluation.roll_force_n;
  return result;
}

}  // namespace fsai::sim
