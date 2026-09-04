#include "fsai_sim_core/parameters.hpp"

#include <bit>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <sstream>
#include <vector>

#include <openssl/evp.h>

namespace fsai::sim {
namespace {

void AppendLittleEndianDouble(std::vector<unsigned char> &out, double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value));
  std::memcpy(&bits, &value, sizeof(value));
  if constexpr (std::endian::native == std::endian::big) {
    bits = __builtin_bswap64(bits);
  }
  for (int i = 0; i < 8; ++i) {
    out.push_back(static_cast<unsigned char>((bits >> (8 * i)) & 0xffu));
  }
}

void Reject(const char *field, double value) {
  throw ValidationError(
    std::string(field) + ": " + std::to_string(value));
}

}  // namespace

DerivedParameters Derive(const VehicleParameters &parameters) {
  DerivedParameters derived;
  derived.cg_to_rear_axle_m =
    parameters.front_static_load_fraction * parameters.wheelbase_m;
  derived.cg_to_front_axle_m =
    (1.0 - parameters.front_static_load_fraction) * parameters.wheelbase_m;
  const double weight = parameters.mass_kg * parameters.gravity_mps2;
  derived.front_static_load_n = parameters.front_static_load_fraction * weight;
  derived.rear_static_load_n = weight - derived.front_static_load_n;
  return derived;
}

void ValidateParameters(const VehicleParameters &parameters) {
  if (!(parameters.mass_kg > 0.0) || !std::isfinite(parameters.mass_kg)) {
    Reject("mass_kg", parameters.mass_kg);
  }
  if (!(parameters.gravity_mps2 > 0.0) || !std::isfinite(parameters.gravity_mps2)) {
    Reject("gravity_mps2", parameters.gravity_mps2);
  }
  if (!(parameters.yaw_inertia_kgm2 > 0.0) ||
      !std::isfinite(parameters.yaw_inertia_kgm2)) {
    Reject("yaw_inertia_kgm2", parameters.yaw_inertia_kgm2);
  }
  if (!(parameters.wheelbase_m > 0.0) || !std::isfinite(parameters.wheelbase_m)) {
    Reject("wheelbase_m", parameters.wheelbase_m);
  }
  if (!(parameters.front_static_load_fraction > 0.0) ||
      !(parameters.front_static_load_fraction < 1.0) ||
      !std::isfinite(parameters.front_static_load_fraction)) {
    Reject("front_static_load_fraction", parameters.front_static_load_fraction);
  }
  if (!(parameters.effective_tyre_radius_m > 0.0) ||
      !std::isfinite(parameters.effective_tyre_radius_m)) {
    Reject("effective_tyre_radius_m", parameters.effective_tyre_radius_m);
  }
  if (!(parameters.max_steering_angle_rad > 0.0)) {
    Reject("max_steering_angle_rad", parameters.max_steering_angle_rad);
  }
  if (!(parameters.max_steering_rate_radps > 0.0)) {
    Reject("max_steering_rate_radps", parameters.max_steering_rate_radps);
  }
  if (parameters.timeout_brake_ratio < 0.0 || parameters.timeout_brake_ratio > 1.0) {
    Reject("timeout_brake_ratio", parameters.timeout_brake_ratio);
  }
  if (parameters.ebs_brake_ratio < 0.0 || parameters.ebs_brake_ratio > 1.0) {
    Reject("ebs_brake_ratio", parameters.ebs_brake_ratio);
  }
}

void Validate(const Command &command) {
  if (!std::isfinite(command.steering_angle_rad)) {
    Reject("steering_angle_rad", command.steering_angle_rad);
  }
  if (!std::isfinite(command.front_axle_torque_nm)) {
    Reject("front_axle_torque_nm", command.front_axle_torque_nm);
  }
  if (!std::isfinite(command.rear_axle_torque_nm)) {
    Reject("rear_axle_torque_nm", command.rear_axle_torque_nm);
  }
  if (!std::isfinite(command.friction_brake_ratio) ||
      command.friction_brake_ratio < 0.0 ||
      command.friction_brake_ratio > 1.0) {
    Reject("friction_brake_ratio", command.friction_brake_ratio);
  }
}

std::string ParameterHash(const VehicleParameters &parameters) {
  std::vector<unsigned char> canonical;
  const double fields[] = {
    parameters.mass_kg,
    parameters.gravity_mps2,
    parameters.yaw_inertia_kgm2,
    parameters.wheelbase_m,
    parameters.front_static_load_fraction,
    parameters.front_track_m,
    parameters.rear_track_m,
    parameters.effective_tyre_radius_m,
    parameters.rolling_resistance,
    parameters.max_steering_angle_rad,
    parameters.max_steering_rate_radps,
    parameters.max_front_axle_torque_nm,
    parameters.max_rear_axle_torque_nm,
    parameters.max_brake_torque_nm,
    parameters.pacejka_B_front,
    parameters.pacejka_C_front,
    parameters.pacejka_D_front,
    parameters.pacejka_E_front,
    parameters.pacejka_B_rear,
    parameters.pacejka_C_rear,
    parameters.pacejka_D_rear,
    parameters.pacejka_E_rear,
    parameters.air_density_kgpm3,
    parameters.lumped_drag_n_s2_per_m2,
    parameters.lumped_downforce_n_s2_per_m2,
    parameters.timeout_brake_ratio,
    parameters.ebs_brake_ratio,
    parameters.static_speed_threshold_mps,
    parameters.rolling_smoothing_speed_mps,
    parameters.hold_release_force_n,
    static_cast<double>(parameters.command_timeout.count()),
  };
  for (double field : fields) {
    AppendLittleEndianDouble(canonical, field);
  }

  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int digest_length = 0;
  if (EVP_Digest(
        canonical.data(),
        canonical.size(),
        digest,
        &digest_length,
        EVP_sha256(),
        nullptr) != 1) {
    throw ValidationError("parameter hash failed");
  }

  std::ostringstream hex;
  hex << std::hex << std::setfill('0');
  for (unsigned int i = 0; i < digest_length; ++i) {
    hex << std::setw(2) << static_cast<int>(digest[i]);
  }
  return hex.str();
}

VehicleParameters ReferenceBicycleParameters() {
  VehicleParameters parameters;
  parameters.mass_kg = 300.0;
  parameters.gravity_mps2 = 9.81;
  parameters.yaw_inertia_kgm2 = 172.44;
  parameters.wheelbase_m = 1.53;
  parameters.front_static_load_fraction = 0.5;
  parameters.front_track_m = 1.2;
  parameters.rear_track_m = 1.2;
  parameters.effective_tyre_radius_m = 0.2525;
  parameters.rolling_resistance = 0.02;
  parameters.max_steering_angle_rad = 0.384;
  parameters.max_steering_rate_radps = 0.39;
  parameters.max_front_axle_torque_nm = 0.0;
  parameters.max_rear_axle_torque_nm = 393.9;
  parameters.max_brake_torque_nm = 393.9;
  parameters.pacejka_B_front = 12.56;
  parameters.pacejka_C_front = 1.4;
  parameters.pacejka_D_front = 1.0;
  parameters.pacejka_E_front = 0.0;
  parameters.pacejka_B_rear = 12.56;
  parameters.pacejka_C_rear = 1.4;
  parameters.pacejka_D_rear = 1.0;
  parameters.pacejka_E_rear = 0.0;
  parameters.lumped_drag_n_s2_per_m2 = 0.5;
  parameters.lumped_downforce_n_s2_per_m2 = 0.3;
  parameters.command_timeout = std::chrono::milliseconds(100);
  parameters.timeout_brake_ratio = 0.5;
  parameters.ebs_brake_ratio = 1.0;
  parameters.static_speed_threshold_mps = 0.05;
  ValidateParameters(parameters);
  return parameters;
}

}  // namespace fsai::sim
