#include "fsai_sim_core/hybrid_integrator.hpp"

#include <algorithm>
#include <cmath>

namespace fsai::sim {
namespace {

ChassisState ApplyDerivative(
  const ChassisState &state,
  const ChassisDerivative &derivative,
  double dt_s) {
  ChassisState next = state;
  next.x_m += derivative.x_mps * dt_s;
  next.y_m += derivative.y_mps * dt_s;
  next.yaw_rad += derivative.yaw_radps * dt_s;
  next.u_mps += derivative.u_mps2 * dt_s;
  next.v_mps += derivative.v_mps2 * dt_s;
  next.yaw_rate_radps += derivative.yaw_rate_radps2 * dt_s;
  return next;
}

ChassisState WeightedSum(
  const ChassisState &base,
  const ChassisDerivative &k,
  double scale) {
  ChassisState next = base;
  next.x_m += k.x_mps * scale;
  next.y_m += k.y_mps * scale;
  next.yaw_rad += k.yaw_radps * scale;
  next.u_mps += k.u_mps2 * scale;
  next.v_mps += k.v_mps2 * scale;
  next.yaw_rate_radps += k.yaw_rate_radps2 * scale;
  return next;
}

ChassisDerivative ScaleAdd(
  const ChassisDerivative &a,
  const ChassisDerivative &b,
  const ChassisDerivative &c,
  const ChassisDerivative &d) {
  ChassisDerivative out;
  out.x_mps = (a.x_mps + 2.0 * b.x_mps + 2.0 * c.x_mps + d.x_mps) / 6.0;
  out.y_mps = (a.y_mps + 2.0 * b.y_mps + 2.0 * c.y_mps + d.y_mps) / 6.0;
  out.yaw_radps = (a.yaw_radps + 2.0 * b.yaw_radps + 2.0 * c.yaw_radps + d.yaw_radps) / 6.0;
  out.u_mps2 = (a.u_mps2 + 2.0 * b.u_mps2 + 2.0 * c.u_mps2 + d.u_mps2) / 6.0;
  out.v_mps2 = (a.v_mps2 + 2.0 * b.v_mps2 + 2.0 * c.v_mps2 + d.v_mps2) / 6.0;
  out.yaw_rate_radps2 =
    (a.yaw_rate_radps2 + 2.0 * b.yaw_rate_radps2 + 2.0 * c.yaw_rate_radps2 +
     d.yaw_rate_radps2) /
    6.0;
  return out;
}

DynamicsEvaluation Eval(
  const ChassisState &state,
  const ActuatorState &input,
  const VehicleParameters &parameters) {
  return EvaluateBicycle(state, input, parameters);
}

ChassisState Rk4Step(
  const ChassisState &state,
  const ActuatorState &input,
  double dt_s,
  const VehicleParameters &parameters,
  DynamicsEvaluation *last) {
  const auto k1 = Eval(state, input, parameters);
  const auto s2 = WeightedSum(state, k1.derivative, dt_s / 2.0);
  const auto k2 = Eval(s2, input, parameters);
  const auto s3 = WeightedSum(state, k2.derivative, dt_s / 2.0);
  const auto k3 = Eval(s3, input, parameters);
  const auto s4 = WeightedSum(state, k3.derivative, dt_s);
  const auto k4 = Eval(s4, input, parameters);
  const auto combined = ScaleAdd(k1.derivative, k2.derivative, k3.derivative, k4.derivative);
  if (last != nullptr) {
    *last = k1;
  }
  return ApplyDerivative(state, combined, dt_s);
}

void ApplyKinematicConstraint(
  ChassisState &state,
  const ActuatorState &input,
  const VehicleParameters &parameters) {
  const DerivedParameters derived = Derive(parameters);
  const double delta = input.steering_angle_rad;
  const double beta = std::atan(
    derived.cg_to_rear_axle_m / parameters.wheelbase_m * std::tan(delta));
  state.v_mps = state.u_mps * std::tan(beta);
  state.yaw_rate_radps =
    state.u_mps * std::cos(beta) * std::tan(delta) / parameters.wheelbase_m;
}

void DeriveWheelSpeeds(
  std::array<WheelState, 4> &wheels,
  const ChassisState &state,
  const ActuatorState &input,
  const VehicleParameters &parameters) {
  const double radius = parameters.effective_tyre_radius_m;
  const double rear = state.u_mps / radius;
  const double cos_delta = std::cos(input.steering_angle_rad);
  const double front = (std::abs(cos_delta) < 1e-6)
                         ? rear
                         : state.u_mps / (radius * cos_delta);
  wheels[0].omega_radps = front;
  wheels[1].omega_radps = front;
  wheels[2].omega_radps = rear;
  wheels[3].omega_radps = rear;
}

bool IsBraking(const DynamicsEvaluation &evaluation, double u) {
  return evaluation.net_longitudinal_force_n * u < 0.0;
}

}  // namespace

IntegrationResult HybridIntegrator::Integrate(
  const ChassisState &state,
  const ActuatorState &input,
  Duration outer_step,
  const VehicleParameters &parameters,
  const std::array<WheelState, 4> &wheels) const {
  if (outer_step <= Duration{0} || outer_step % kInternalStep != Duration{0}) {
    throw ValidationError(
      "outer_step: " + std::to_string(outer_step.count()));
  }

  IntegrationResult result;
  result.state = state;
  result.wheels = wheels;
  result.diagnostics.internal_steps = 0;

  Duration remaining = outer_step;
  while (remaining > Duration{0}) {
    Duration step = std::min(kInternalStep, remaining);
    const double dt_s = std::chrono::duration<double>(step).count();
    const bool holding = result.wheels[2].contact_mode == ContactMode::kBrakeHold;
    auto preview = Eval(result.state, input, parameters);

    if (holding) {
      const bool release = preview.drive_force_n >
                           -preview.brake_force_n + parameters.hold_release_force_n;
      if (!release) {
        result.state.u_mps = 0.0;
        result.state.v_mps = 0.0;
        result.state.yaw_rate_radps = 0.0;
        result.last_evaluation = preview;
        remaining -= step;
        result.diagnostics.internal_steps += 1;
        DeriveWheelSpeeds(result.wheels, result.state, input, parameters);
        continue;
      }
      for (auto &wheel : result.wheels) {
        wheel.contact_mode = ContactMode::kKinematic;
      }
      result.events.push_back(SimulationEvent{
        .time = {},
        .type = EventType::kHoldReleased,
        .detail = "hold released",
      });
    }

    ChassisState trial = Rk4Step(result.state, input, dt_s, parameters, &result.last_evaluation);
    const double euler_u =
      result.state.u_mps + result.last_evaluation.derivative.u_mps2 * dt_s;
    const bool approaching_stop =
      result.state.u_mps > 0.0 &&
      IsBraking(result.last_evaluation, result.state.u_mps) &&
      (trial.u_mps <= 0.0 || euler_u <= 0.0);
    if (approaching_stop) {
      Duration lo{0};
      Duration hi = step;
      ChassisState event_state = result.state;
      while (hi - lo > Duration{1}) {
        const Duration mid{(lo.count() + hi.count()) / 2};
        const double mid_s = std::chrono::duration<double>(mid).count();
        event_state = Rk4Step(result.state, input, mid_s, parameters, nullptr);
        if (event_state.u_mps <= 0.0) {
          hi = mid;
        } else {
          lo = mid;
        }
      }
      const double event_s = std::chrono::duration<double>(hi).count();
      result.state = Rk4Step(result.state, input, event_s, parameters, &result.last_evaluation);
      result.state.u_mps = 0.0;
      result.state.v_mps = 0.0;
      result.state.yaw_rate_radps = 0.0;
      for (auto &wheel : result.wheels) {
        wheel.contact_mode = ContactMode::kBrakeHold;
      }
      result.events.push_back(SimulationEvent{
        .time = {},
        .type = EventType::kVehicleStopped,
        .detail = "stopped",
      });
      remaining -= hi;
      result.diagnostics.internal_steps += 1;
      DeriveWheelSpeeds(result.wheels, result.state, input, parameters);
      continue;
    }

    result.state = trial;
    if (std::abs(result.state.u_mps) < parameters.static_speed_threshold_mps &&
        result.wheels[2].contact_mode != ContactMode::kBrakeHold) {
      ApplyKinematicConstraint(result.state, input, parameters);
      for (auto &wheel : result.wheels) {
        if (wheel.contact_mode != ContactMode::kBrakeHold) {
          wheel.contact_mode = ContactMode::kKinematic;
        }
      }
    }
    remaining -= step;
    result.diagnostics.internal_steps += 1;
    DeriveWheelSpeeds(result.wheels, result.state, input, parameters);
  }

  result.last_evaluation = Eval(result.state, input, parameters);
  return result;
}

}  // namespace fsai::sim
