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
  auto out = EvaluateBicycle(state, input, parameters);
  if (std::abs(state.u_mps) < parameters.static_speed_threshold_mps &&
      std::abs(state.v_mps) < parameters.static_speed_threshold_mps &&
      std::abs(state.yaw_rate_radps)*parameters.wheelbase_m < parameters.static_speed_threshold_mps) {
    const auto dynamic = out;
    // u is body-x velocity. Differentiate the no-slip constraints at fixed
    // steering over this integration substep, so IMU truth agrees with motion.
    const double h = std::tan(input.steering_angle_rad) / parameters.wheelbase_m;
    const double lr = Derive(parameters).cg_to_rear_axle_m;
    const double k = lr * h;
    const double mass = parameters.mass_kg;
    const double inertia = parameters.yaw_inertia_kgm2;
    const double original_fy = mass * (out.derivative.v_mps2 + state.yaw_rate_radps * state.u_mps);
    const double original_front_fy_body = (out.yaw_moment_nm + lr*original_fy)/parameters.wheelbase_m;
    const double front_drive_y = original_front_fy_body -
      out.front_lateral_force_n*std::cos(input.steering_angle_rad);
    // Project applied forces onto the no-slip velocity direction. This includes
    // the translational and yaw kinetic energy rather than discarding v/r work.
    const double acceleration = (out.net_longitudinal_force_n+k*original_fy+h*out.yaw_moment_nm) /
      (mass*(1.0+k*k)+inertia*h*h);
    out.derivative.u_mps2 = acceleration;
    out.derivative.v_mps2 = k * acceleration;
    out.derivative.yaw_rate_radps2 = h * acceleration;
    const double fy = mass * (k*acceleration + state.yaw_rate_radps*state.u_mps);
    out.yaw_moment_nm = inertia*h*acceleration;
    const double front_fy_body = (out.yaw_moment_nm+lr*fy)/parameters.wheelbase_m;
    out.front_lateral_force_n = (front_fy_body-front_drive_y)/std::cos(input.steering_angle_rad);
    out.rear_lateral_force_n = fy-front_fy_body;
    out.net_longitudinal_force_n = mass*(acceleration-state.yaw_rate_radps*state.v_mps);
    const double load = mass*parameters.gravity_mps2 +
      parameters.lumped_downforce_n_s2_per_m2*state.u_mps*state.u_mps;
    const double limit_front = parameters.pacejka_D_front*parameters.front_static_load_fraction*load;
    const double limit_rear = parameters.pacejka_D_rear*(1.0-parameters.front_static_load_fraction)*load;
    if (std::hypot(out.front_longitudinal_force_n,out.front_lateral_force_n)>limit_front+1e-9 ||
        std::hypot(out.rear_longitudinal_force_n,out.rear_lateral_force_n)>limit_rear+1e-9) {
      return dynamic;  // Do not invent no-slip constraint forces beyond adhesion.
    }
    out.kinematic_constraint_active = true;
  }
  return out;
}

ChassisState Rk4Step(
  const ChassisState &state,
  const ActuatorState &input,
  double dt_s,
  const VehicleParameters &parameters,
  DynamicsEvaluation *last) {
  // Continue the pre-event force direction across trial zero crossings. The
  // root finder then locates the stop before switching to the static branch.
  const auto evaluate = [&](ChassisState stage) {
    if (state.u_mps > 0.0 && stage.u_mps <= 0.0) stage.u_mps = 1e-12;
    if (state.u_mps < 0.0 && stage.u_mps >= 0.0) stage.u_mps = -1e-12;
    return Eval(stage,input,parameters);
  };
  const auto k1 = evaluate(state);
  const auto s2 = WeightedSum(state, k1.derivative, dt_s / 2.0);
  const auto k2 = evaluate(s2);
  const auto s3 = WeightedSum(state, k2.derivative, dt_s / 2.0);
  const auto k3 = evaluate(s3);
  const auto s4 = WeightedSum(state, k3.derivative, dt_s);
  const auto k4 = evaluate(s4);
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
    state.u_mps * std::tan(delta) / parameters.wheelbase_m;
}

void DeriveWheelSpeeds(
  std::array<WheelState, 4> &wheels,
  const ChassisState &state,
  const ActuatorState &input,
  const VehicleParameters &parameters) {
  const double radius = parameters.effective_tyre_radius_m;
  const double curvature = std::tan(input.steering_angle_rad) / parameters.wheelbase_m;
  const double front_v = state.v_mps + Derive(parameters).cg_to_front_axle_m * state.yaw_rate_radps;
  for (std::size_t side=0; side<2; ++side) {
    const double sign = side == 0 ? 1.0 : -1.0;  // FL, FR, RL, RR.
    const double y_front = sign * parameters.front_track_m / 2.0;
    const double angle = std::atan2(parameters.wheelbase_m * curvature,1.0-y_front*curvature);
    const double front_u = state.u_mps - state.yaw_rate_radps*y_front;
    wheels[side].omega_radps = (front_u*std::cos(angle)+front_v*std::sin(angle))/radius;
    wheels[side+2].omega_radps =
      (state.u_mps-state.yaw_rate_radps*sign*parameters.rear_track_m/2.0)/radius;
  }
}

bool IsBraking(const DynamicsEvaluation &evaluation, double u) {
  return evaluation.net_longitudinal_force_n * u < 0.0;
}

}  // namespace

HybridIntegrator::HybridIntegrator(Duration internal_step) : internal_step_(internal_step) {
  if (internal_step <= Duration{0} || internal_step > kInternalStep) {
    throw ValidationError("internal_step must be in (0, 1 ms]");
  }
}

IntegrationResult HybridIntegrator::Integrate(
  const ChassisState &state,
  const ActuatorState &input,
  Duration outer_step,
  const VehicleParameters &parameters,
  const std::array<WheelState, 4> &wheels) const {
  if (outer_step <= Duration{0} || outer_step % internal_step_ != Duration{0}) {
    throw ValidationError(
      "outer_step: " + std::to_string(outer_step.count()));
  }

  IntegrationResult result;
  result.state = state;
  result.wheels = wheels;
  result.diagnostics.internal_steps = 0;

  Duration remaining = outer_step;
  while (remaining > Duration{0}) {
    Duration step = std::min(internal_step_, remaining);
    const double dt_s = std::chrono::duration<double>(step).count();
    const bool holding = result.wheels[2].contact_mode == ContactMode::kBrakeHold;
    if (Eval(result.state,input,parameters).kinematic_constraint_active) {
      auto constrained = result.state;
      ApplyKinematicConstraint(constrained,input,parameters);
      if (Eval(constrained,input,parameters).kinematic_constraint_active) result.state=constrained;
    }
    auto preview = Eval(result.state, input, parameters);

    if (holding) {
      const bool release = preview.net_longitudinal_force_n > parameters.hold_release_force_n;
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
        .time = outer_step - remaining,
        .type = EventType::kHoldReleased,
        .detail = "hold released",
      });
    }

    ChassisState trial = Rk4Step(result.state, input, dt_s, parameters, &result.last_evaluation);
    const bool approaching_stop =
      result.state.u_mps != 0.0 &&
      IsBraking(result.last_evaluation, result.state.u_mps) &&
      trial.u_mps * result.state.u_mps <= 0.0;
    if (approaching_stop) {
      Duration lo{0};
      Duration hi = step;
      ChassisState event_state = result.state;
      while (hi - lo > Duration{1}) {
        const Duration mid{(lo.count() + hi.count()) / 2};
        const double mid_s = std::chrono::duration<double>(mid).count();
        event_state = Rk4Step(result.state, input, mid_s, parameters, nullptr);
        if (event_state.u_mps * result.state.u_mps <= 0.0) {
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
        .time = outer_step - remaining + hi,
        .type = EventType::kVehicleStopped,
        .detail = "stopped",
      });
      remaining -= hi;
      result.diagnostics.internal_steps += 1;
      DeriveWheelSpeeds(result.wheels, result.state, input, parameters);
      continue;
    }

    result.state = trial;
    if (Eval(result.state,input,parameters).kinematic_constraint_active &&
        result.wheels[2].contact_mode != ContactMode::kBrakeHold) {
      auto constrained = result.state;
      ApplyKinematicConstraint(constrained,input,parameters);
      if (Eval(constrained,input,parameters).kinematic_constraint_active) result.state=constrained;
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
  if (result.wheels[2].contact_mode == ContactMode::kBrakeHold) {
    // Include the holding constraint reaction in accepted-state ground truth.
    result.last_evaluation.derivative = {};
    result.last_evaluation.net_longitudinal_force_n = 0.0;
    result.last_evaluation.yaw_moment_nm = 0.0;
    result.last_evaluation.front_lateral_force_n = 0.0;
    result.last_evaluation.rear_lateral_force_n = 0.0;
    result.last_evaluation.front_longitudinal_force_n = 0.0;
    result.last_evaluation.rear_longitudinal_force_n = 0.0;
  }
  return result;
}

}  // namespace fsai::sim
