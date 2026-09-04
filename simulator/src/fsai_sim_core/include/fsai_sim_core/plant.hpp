#ifndef FSAI_SIM_CORE__PLANT_HPP_
#define FSAI_SIM_CORE__PLANT_HPP_

#include "fsai_sim_core/actuator_model.hpp"
#include "fsai_sim_core/ground_truth_builder.hpp"
#include "fsai_sim_core/hybrid_integrator.hpp"
#include "fsai_sim_core/parameters.hpp"
#include "fsai_sim_core/types.hpp"

namespace fsai::sim {

class Plant final {
 public:
  explicit Plant(VehicleParameters parameters);

  const VehicleParameters &parameters() const { return parameters_; }
  const std::string &parameter_hash() const { return parameter_hash_; }

  PlantState InitialState() const;

  StepResult Update(
    const PlantState &state,
    const Command &command,
    Duration dt) const;

 private:
  VehicleParameters parameters_;
  std::string parameter_hash_;
  ActuatorModel actuators_;
  HybridIntegrator integrator_;
  GroundTruthBuilder truth_builder_;
};

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__PLANT_HPP_
