#ifndef FSAI_SIM_CORE__SAFETY_CHAIN_HPP_
#define FSAI_SIM_CORE__SAFETY_CHAIN_HPP_

#include "fsai_sim_core/parameters.hpp"
#include "fsai_sim_core/types.hpp"

namespace fsai::sim {

struct SafetyInput final {
  bool as_driving{false};
  bool ebs_request{false};
  Duration command_age{0};
};

struct SafetyResult final {
  Command command;
  bool ebs_latched{false};
  std::vector<SimulationEvent> events;
};

SafetyResult Resolve(
  const Command &command,
  const SafetyInput &input,
  const PlantState &state,
  const VehicleParameters &parameters);

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__SAFETY_CHAIN_HPP_
