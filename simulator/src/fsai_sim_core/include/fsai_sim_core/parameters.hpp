#ifndef FSAI_SIM_CORE__PARAMETERS_HPP_
#define FSAI_SIM_CORE__PARAMETERS_HPP_

#include <string>

#include "fsai_sim_core/types.hpp"

namespace fsai::sim {

struct VehicleParameters final {
  std::uint32_t schema_version{1};
  double mass_kg{};
  double gravity_mps2{9.81};
  double yaw_inertia_kgm2{};
  double wheelbase_m{};
  double front_static_load_fraction{};
  double front_track_m{};
  double rear_track_m{};
  double effective_tyre_radius_m{};
  double rolling_resistance{};
  double max_steering_angle_rad{};
  double max_steering_rate_radps{};
  double max_front_axle_torque_nm{};
  double max_rear_axle_torque_nm{};
  double max_brake_torque_nm{};
  double pacejka_B_front{};
  double pacejka_C_front{};
  double pacejka_D_front{};
  double pacejka_E_front{};
  double pacejka_B_rear{};
  double pacejka_C_rear{};
  double pacejka_D_rear{};
  double pacejka_E_rear{};
  double air_density_kgpm3{1.225};
  double lumped_drag_n_s2_per_m2{};
  double lumped_downforce_n_s2_per_m2{};
  Duration command_timeout{std::chrono::milliseconds(100)};
  double timeout_brake_ratio{0.5};
  double ebs_brake_ratio{1.0};
  double static_speed_threshold_mps{0.05};
  double rolling_smoothing_speed_mps{0.1};
  double hold_release_force_n{1.0};
};

struct DerivedParameters final {
  double cg_to_front_axle_m{};
  double cg_to_rear_axle_m{};
  double front_static_load_n{};
  double rear_static_load_n{};
};

DerivedParameters Derive(const VehicleParameters &parameters);
void ValidateParameters(const VehicleParameters &parameters);
std::string ParameterHash(const VehicleParameters &parameters);
VehicleParameters ReferenceBicycleParameters();

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__PARAMETERS_HPP_
