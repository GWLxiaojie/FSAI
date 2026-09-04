#include "fsai_sim_core/bicycle_backend.hpp"

#include <algorithm>
#include <cmath>

namespace fsai::sim {
namespace {

double Pacejka(double alpha, double B, double C, double D, double E) {
  const double Balpha = B * alpha;
  const double phi = Balpha - E * (Balpha - std::atan(Balpha));
  return -D * std::sin(C * std::atan(phi));
}

double RegularizeU(double u, double threshold) {
  if (std::abs(u) >= threshold) {
    return u;
  }
  if (u == 0.0) {
    return threshold;
  }
  return std::copysign(threshold, u);
}

}  // namespace

DynamicsEvaluation EvaluateBicycle(
  const ChassisState &chassis,
  const ActuatorState &actuator,
  const VehicleParameters &parameters) {
  const DerivedParameters derived = Derive(parameters);
  DynamicsEvaluation out;
  const double u = chassis.u_mps;
  const double v = chassis.v_mps;
  const double r = chassis.yaw_rate_radps;
  const double delta = actuator.steering_angle_rad;
  const double yaw = chassis.yaw_rad;
  const double lf = derived.cg_to_front_axle_m;
  const double lr = derived.cg_to_rear_axle_m;
  const double radius = parameters.effective_tyre_radius_m;

  const bool stationary = std::abs(u) < parameters.static_speed_threshold_mps &&
                          std::abs(v) < parameters.static_speed_threshold_mps &&
                          std::abs(r) < parameters.static_speed_threshold_mps;

  if (stationary) {
    out.front_slip_angle_rad = 0.0;
    out.rear_slip_angle_rad = 0.0;
    out.front_lateral_force_n = 0.0;
    out.rear_lateral_force_n = 0.0;
  } else {
    const double u_reg = RegularizeU(u, parameters.static_speed_threshold_mps);
    out.front_slip_angle_rad = std::atan2(v + lf * r, u_reg) - delta;
    out.rear_slip_angle_rad = std::atan2(v - lr * r, u_reg);
    const double Df = parameters.pacejka_D_front * derived.front_static_load_n;
    const double Dr = parameters.pacejka_D_rear * derived.rear_static_load_n;
    out.front_lateral_force_n = Pacejka(
      out.front_slip_angle_rad,
      parameters.pacejka_B_front,
      parameters.pacejka_C_front,
      Df,
      parameters.pacejka_E_front);
    out.rear_lateral_force_n = Pacejka(
      out.rear_slip_angle_rad,
      parameters.pacejka_B_rear,
      parameters.pacejka_C_rear,
      Dr,
      parameters.pacejka_E_rear);
  }

  out.drive_force_n =
    (actuator.front_axle_torque_nm + actuator.rear_axle_torque_nm) / radius;
  const double brake_capacity = actuator.friction_brake_ratio *
                                parameters.max_brake_torque_nm / radius;
  // Brake opposes motion and cannot reverse the vehicle.
  if (u > 0.0) {
    out.brake_force_n = -brake_capacity;
  } else if (u < 0.0) {
    out.brake_force_n = brake_capacity;
  } else {
    out.brake_force_n = 0.0;
  }

  out.drag_force_n = -parameters.lumped_drag_n_s2_per_m2 * u * std::abs(u);
  out.roll_force_n = -parameters.rolling_resistance * parameters.mass_kg *
                     parameters.gravity_mps2 *
                     std::tanh(u / parameters.rolling_smoothing_speed_mps);

  const double fx_front_tyre = 0.0;
  const double fy_f = out.front_lateral_force_n;
  const double fy_r = out.rear_lateral_force_n;
  const double fx_f_body = fx_front_tyre * std::cos(delta) - fy_f * std::sin(delta);
  const double fy_f_body = fy_f * std::cos(delta) + fx_front_tyre * std::sin(delta);
  const double fx_body = out.drive_force_n + out.brake_force_n + out.drag_force_n +
                         out.roll_force_n + fx_f_body;
  const double fy_body = fy_f_body + fy_r;
  out.net_longitudinal_force_n = fx_body;
  out.yaw_moment_nm = lf * fy_f_body - lr * fy_r;

  out.derivative.x_mps = u * std::cos(yaw) - v * std::sin(yaw);
  out.derivative.y_mps = u * std::sin(yaw) + v * std::cos(yaw);
  out.derivative.yaw_radps = r;
  out.derivative.u_mps2 = r * v + fx_body / parameters.mass_kg;
  out.derivative.v_mps2 = -r * u + fy_body / parameters.mass_kg;
  out.derivative.yaw_rate_radps2 = out.yaw_moment_nm / parameters.yaw_inertia_kgm2;
  return out;
}

DynamicsEvaluation BicycleBackend::Evaluate(
  const ChassisState &chassis,
  const ActuatorState &actuator,
  const VehicleParameters &parameters) const {
  return EvaluateBicycle(chassis, actuator, parameters);
}

}  // namespace fsai::sim
