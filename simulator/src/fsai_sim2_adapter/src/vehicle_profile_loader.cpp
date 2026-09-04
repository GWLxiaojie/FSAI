#include "fsai_sim2_adapter/vehicle_profile_loader.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace fsai::sim2_adapter {
namespace {

double LoadMeasured(
  const YAML::Node &root,
  const char *name,
  const char *unit,
  const std::string &file) {
  if (!root[name]) {
    throw std::invalid_argument(file + ": missing " + name);
  }
  const YAML::Node field = root[name];
  if (field.IsScalar()) {
    return field.as<double>();
  }
  if (!field["value"] || !field["unit"]) {
    throw std::invalid_argument(file + ": " + name + " missing value/unit");
  }
  if (field["unit"].as<std::string>() != unit) {
    throw std::invalid_argument(file + ": " + name + " unit mismatch");
  }
  const double value = field["value"].as<double>();
  if (field["valid_min"] && value < field["valid_min"].as<double>()) {
    throw std::invalid_argument(
      file + ": " + name + ": " + std::to_string(value));
  }
  if (field["valid_max"] && value > field["valid_max"].as<double>()) {
    throw std::invalid_argument(
      file + ": " + name + ": " + std::to_string(value));
  }
  if (!std::isfinite(value)) {
    throw std::invalid_argument(file + ": " + name + ": " + std::to_string(value));
  }
  return value;
}

YAML::Node LoadFile(const std::filesystem::path &path) {
  try {
    return YAML::LoadFile(path.string());
  } catch (const std::exception &error) {
    throw std::invalid_argument(path.string() + ": " + error.what());
  }
}

}  // namespace

fsai::sim::VehicleParameters LoadVehicleProfile(
  const std::filesystem::path &profile_dir) {
  const auto vehicle = LoadFile(profile_dir / "vehicle.yaml");
  const auto actuators = LoadFile(profile_dir / "actuators.yaml");
  const auto tyres = LoadFile(profile_dir / "tyres.yaml");
  const auto aero = LoadFile(profile_dir / "aero.yaml");

  if (vehicle["schema_version"] && vehicle["schema_version"].as<int>() != 1) {
    throw std::invalid_argument("vehicle.yaml: schema_version");
  }

  fsai::sim::VehicleParameters parameters;
  parameters.mass_kg = LoadMeasured(vehicle, "mass_kg", "kg", "vehicle.yaml");
  parameters.gravity_mps2 = LoadMeasured(vehicle, "gravity_mps2", "m/s^2", "vehicle.yaml");
  parameters.yaw_inertia_kgm2 =
    LoadMeasured(vehicle, "yaw_inertia_kgm2", "kg m^2", "vehicle.yaml");
  parameters.wheelbase_m = LoadMeasured(vehicle, "wheelbase_m", "m", "vehicle.yaml");
  parameters.front_static_load_fraction =
    LoadMeasured(vehicle, "front_static_load_fraction", "1", "vehicle.yaml");
  parameters.front_track_m = LoadMeasured(vehicle, "front_track_m", "m", "vehicle.yaml");
  parameters.rear_track_m = LoadMeasured(vehicle, "rear_track_m", "m", "vehicle.yaml");
  parameters.effective_tyre_radius_m =
    LoadMeasured(tyres, "effective_tyre_radius_m", "m", "tyres.yaml");
  parameters.rolling_resistance =
    LoadMeasured(tyres, "rolling_resistance", "1", "tyres.yaml");
  parameters.pacejka_B_front = LoadMeasured(tyres, "pacejka_B_front", "1/rad", "tyres.yaml");
  parameters.pacejka_C_front = LoadMeasured(tyres, "pacejka_C_front", "1", "tyres.yaml");
  parameters.pacejka_D_front = LoadMeasured(tyres, "pacejka_D_front", "1", "tyres.yaml");
  parameters.pacejka_E_front = LoadMeasured(tyres, "pacejka_E_front", "1", "tyres.yaml");
  parameters.pacejka_B_rear = LoadMeasured(tyres, "pacejka_B_rear", "1/rad", "tyres.yaml");
  parameters.pacejka_C_rear = LoadMeasured(tyres, "pacejka_C_rear", "1", "tyres.yaml");
  parameters.pacejka_D_rear = LoadMeasured(tyres, "pacejka_D_rear", "1", "tyres.yaml");
  parameters.pacejka_E_rear = LoadMeasured(tyres, "pacejka_E_rear", "1", "tyres.yaml");
  parameters.max_steering_angle_rad =
    LoadMeasured(actuators, "max_steering_angle_rad", "rad", "actuators.yaml");
  parameters.max_steering_rate_radps =
    LoadMeasured(actuators, "max_steering_rate_radps", "rad/s", "actuators.yaml");
  parameters.max_front_axle_torque_nm =
    LoadMeasured(actuators, "max_front_axle_torque_nm", "N m", "actuators.yaml");
  parameters.max_rear_axle_torque_nm =
    LoadMeasured(actuators, "max_rear_axle_torque_nm", "N m", "actuators.yaml");
  parameters.max_brake_torque_nm =
    LoadMeasured(actuators, "max_brake_torque_nm", "N m", "actuators.yaml");
  parameters.timeout_brake_ratio =
    LoadMeasured(actuators, "timeout_brake_ratio", "1", "actuators.yaml");
  parameters.ebs_brake_ratio =
    LoadMeasured(actuators, "ebs_brake_ratio", "1", "actuators.yaml");
  parameters.lumped_drag_n_s2_per_m2 =
    LoadMeasured(aero, "lumped_drag_n_s2_per_m2", "N/(m/s)^2", "aero.yaml");
  parameters.lumped_downforce_n_s2_per_m2 =
    LoadMeasured(aero, "lumped_downforce_n_s2_per_m2", "N/(m/s)^2", "aero.yaml");
  fsai::sim::ValidateParameters(parameters);
  return parameters;
}

}  // namespace fsai::sim2_adapter
