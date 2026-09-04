#ifndef FSAI_SIM_CORE__BICYCLE_BACKEND_HPP_
#define FSAI_SIM_CORE__BICYCLE_BACKEND_HPP_

#include "fsai_sim_core/dynamics_backend.hpp"

namespace fsai::sim {

DynamicsEvaluation EvaluateBicycle(
  const ChassisState &chassis,
  const ActuatorState &actuator,
  const VehicleParameters &parameters);

class BicycleBackend final : public DynamicsBackend {
 public:
  DynamicsEvaluation Evaluate(
    const ChassisState &chassis,
    const ActuatorState &actuator,
    const VehicleParameters &parameters) const override;
};

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__BICYCLE_BACKEND_HPP_
