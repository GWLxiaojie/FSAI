#ifndef FSAI_SIM_CORE__BICYCLE_BACKEND_HPP_
#define FSAI_SIM_CORE__BICYCLE_BACKEND_HPP_

#include "fsai_sim_core/dynamics_backend.hpp"

namespace fsai::sim {

// Reference planar single-track model (not a calibrated vehicle/VCU model).
// Downforce and friction-brake capacity are split using static axle fractions.
// Each axle shares an isotropic mu*Fz envelope between regen, friction braking,
// drive and lateral force; longitudinal force has priority. There is no wheel
// inertia/slip, load transfer, ABS, power/battery limit, or calibrated blending.
// Wheel speeds are geometric rolling projections, unsuitable for slip-control
// validation. Parameters retain their profile's reference_unvalidated status.
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
