#ifndef FSAI_SIM2_ADAPTER__VEHICLE_PROFILE_LOADER_HPP_
#define FSAI_SIM2_ADAPTER__VEHICLE_PROFILE_LOADER_HPP_

#include <filesystem>
#include <string>

#include "fsai_sim_core/parameters.hpp"

namespace fsai::sim2_adapter {

fsai::sim::VehicleParameters LoadVehicleProfile(const std::filesystem::path &profile_dir);

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__VEHICLE_PROFILE_LOADER_HPP_
