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
  const double aerodynamic_load = parameters.lumped_downforce_n_s2_per_m2 * u * u;
  out.front_normal_force_n = derived.front_static_load_n +
    parameters.front_static_load_fraction * aerodynamic_load;
  out.rear_normal_force_n = derived.rear_static_load_n +
    (1.0-parameters.front_static_load_fraction) * aerodynamic_load;

  const bool stationary = std::abs(u) < parameters.static_speed_threshold_mps &&
                          std::abs(v) < parameters.static_speed_threshold_mps &&
                          std::abs(r) < parameters.static_speed_threshold_mps;

  if (stationary) {
    out.front_slip_angle_rad = 0.0;
    out.rear_slip_angle_rad = 0.0;
    out.front_lateral_force_n = 0.0;
    out.rear_lateral_force_n = 0.0;
  } else {
    const double u_reg = RegularizeU(std::abs(u), parameters.static_speed_threshold_mps);
    out.front_slip_angle_rad = std::atan2(v + lf * r, u_reg) - delta;
    out.rear_slip_angle_rad = std::atan2(v - lr * r, u_reg);
    // Reference model: aerodynamic centre at the static CG, no load transfer.
    const double downforce = parameters.lumped_downforce_n_s2_per_m2 * u * u;
    const double Df = parameters.pacejka_D_front *
      (derived.front_static_load_n + parameters.front_static_load_fraction * downforce);
    const double Dr = parameters.pacejka_D_rear *
      (derived.rear_static_load_n + (1.0-parameters.front_static_load_fraction) * downforce);
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

  const double front_drive = std::max(0.0, actuator.front_axle_torque_nm) / radius;
  const double rear_drive = std::max(0.0, actuator.rear_axle_torque_nm) / radius;
  out.drive_force_n = front_drive * std::cos(delta) + rear_drive;
  const double brake_capacity = actuator.friction_brake_ratio *
                                parameters.max_brake_torque_nm / radius;
  const double front_share = parameters.front_static_load_fraction;
  const double brake_body_capacity = brake_capacity *
    (front_share * std::cos(delta) + 1.0 - front_share);
  // Regen is dissipative and provides no holding torque at rest. Friction brakes
  // supply a static reaction up to their capacity, including on the initial step.
  if (u == 0.0 && v == 0.0 && r == 0.0 && out.drive_force_n <= brake_body_capacity) {
    out.brake_force_n = -out.drive_force_n;
    return out;
  }
  const double direction = u < 0.0 ? -1.0 : 1.0;
  const double regen_front = u == 0.0 ? 0.0 : std::max(0.0,-actuator.front_axle_torque_nm)/radius;
  const double regen_rear = u == 0.0 ? 0.0 : std::max(0.0,-actuator.rear_axle_torque_nm)/radius;
  const double front_brake = direction * (front_share * brake_capacity + regen_front);
  const double rear_brake = direction * ((1.0-front_share) * brake_capacity + regen_rear);
  out.brake_force_n = -front_brake * std::cos(delta) - rear_brake;

  out.drag_force_n = -parameters.lumped_drag_n_s2_per_m2 * u * std::abs(u);
  out.roll_force_n = -parameters.rolling_resistance * parameters.mass_kg *
                     parameters.gravity_mps2 *
                     std::tanh(u / parameters.rolling_smoothing_speed_mps);

  // Unified axle force envelope: regen and friction share the same tyre budget.
  // Longitudinal priority with an isotropic friction circle is a reference
  // approximation, not a calibrated VCU blending law or a wheel-slip/ABS model.
  const double load = parameters.mass_kg * parameters.gravity_mps2 +
    parameters.lumped_downforce_n_s2_per_m2 * u * u;
  const double limit_f = parameters.pacejka_D_front * front_share * load;
  const double limit_r = parameters.pacejka_D_rear * (1.0-front_share) * load;
  const double fx_front_tyre = std::clamp(front_drive-front_brake,-limit_f,limit_f);
  const double fx_rear_tyre = std::clamp(rear_drive-rear_brake,-limit_r,limit_r);
  out.front_longitudinal_force_n = fx_front_tyre;
  out.rear_longitudinal_force_n = fx_rear_tyre;
  const double lateral_f = std::sqrt(std::max(0.0,limit_f*limit_f-fx_front_tyre*fx_front_tyre));
  const double lateral_r = std::sqrt(std::max(0.0,limit_r*limit_r-fx_rear_tyre*fx_rear_tyre));
  out.front_lateral_force_n = std::clamp(out.front_lateral_force_n,-lateral_f,lateral_f);
  out.rear_lateral_force_n = std::clamp(out.rear_lateral_force_n,-lateral_r,lateral_r);
  const double fy_f = out.front_lateral_force_n;
  const double fy_r = out.rear_lateral_force_n;
  const double fx_f_body = fx_front_tyre * std::cos(delta) - fy_f * std::sin(delta);
  const double fy_f_body = fy_f * std::cos(delta) + fx_front_tyre * std::sin(delta);
  const double fx_body = fx_rear_tyre + out.drag_force_n + out.roll_force_n + fx_f_body;
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
