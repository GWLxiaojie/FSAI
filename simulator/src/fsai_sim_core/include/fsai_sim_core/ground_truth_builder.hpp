#ifndef FSAI_SIM_CORE__GROUND_TRUTH_BUILDER_HPP_
#define FSAI_SIM_CORE__GROUND_TRUTH_BUILDER_HPP_

#include "fsai_sim_core/dynamics_backend.hpp"
#include "fsai_sim_core/types.hpp"

namespace fsai::sim {

GroundTruth BuildGroundTruth(
  const ChassisState &accepted_state,
  const ActuatorState &actuator,
  const std::array<WheelState, 4> &wheels,
  const DynamicsEvaluation &evaluation);

class GroundTruthBuilder final {
 public:
  GroundTruth Build(
    const ChassisState &accepted_state,
    const ActuatorState &actuator,
    const std::array<WheelState, 4> &wheels,
    const DynamicsEvaluation &evaluation) const;
};

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__GROUND_TRUTH_BUILDER_HPP_
