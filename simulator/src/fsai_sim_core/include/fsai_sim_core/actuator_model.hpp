#ifndef FSAI_SIM_CORE__ACTUATOR_MODEL_HPP_
#define FSAI_SIM_CORE__ACTUATOR_MODEL_HPP_

#include "fsai_sim_core/parameters.hpp"
#include "fsai_sim_core/types.hpp"

namespace fsai::sim {

struct ActuatorResult final {
  ActuatorState state;
};

class ActuatorModel final {
 public:
  ActuatorResult Update(
    const ActuatorState &state,
    const Command &command,
    Duration dt,
    const VehicleParameters &parameters) const;
};

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__ACTUATOR_MODEL_HPP_
