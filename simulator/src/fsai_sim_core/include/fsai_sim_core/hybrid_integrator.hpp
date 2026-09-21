#ifndef FSAI_SIM_CORE__HYBRID_INTEGRATOR_HPP_
#define FSAI_SIM_CORE__HYBRID_INTEGRATOR_HPP_

#include "fsai_sim_core/bicycle_backend.hpp"
#include "fsai_sim_core/parameters.hpp"
#include "fsai_sim_core/types.hpp"

namespace fsai::sim {

struct IntegrationResult final {
  ChassisState state;
  std::array<WheelState, 4> wheels{};
  std::vector<SimulationEvent> events;  // Time offsets from the start of this call.
  Diagnostics diagnostics;
  DynamicsEvaluation last_evaluation;
};

class HybridIntegrator final {
 public:
  static constexpr Duration kInternalStep{std::chrono::milliseconds(1)};

  // Production uses 1 ms. Finer grids support numerical convergence checks.
  explicit HybridIntegrator(Duration internal_step = kInternalStep);

  IntegrationResult Integrate(
    const ChassisState &state,
    const ActuatorState &input,
    Duration outer_step,
    const VehicleParameters &parameters,
    const std::array<WheelState, 4> &wheels) const;

 private:
  Duration internal_step_;
};

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__HYBRID_INTEGRATOR_HPP_
