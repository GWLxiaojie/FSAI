#include "fsai_sim2_adapter/simulation_runner.hpp"

#include <stdexcept>

namespace fsai::sim2_adapter {

SimulationRunner::SimulationRunner(
  eufs::sim2::SimulationBase &simulation,
  Duration outer_step)
    : simulation_(simulation),
      outer_step_(outer_step) {
  if (outer_step_.count() == 0) {
    throw std::invalid_argument("outer_step must be positive");
  }
}

void SimulationRunner::StepOnce() {
  simulation_.Step(outer_step_);
}

void SimulationRunner::RunSteps(std::size_t count) {
  for (std::size_t i = 0; i < count; ++i) {
    StepOnce();
  }
}

}  // namespace fsai::sim2_adapter
