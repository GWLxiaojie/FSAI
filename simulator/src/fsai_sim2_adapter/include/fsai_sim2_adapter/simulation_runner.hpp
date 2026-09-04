#ifndef FSAI_SIM2_ADAPTER__SIMULATION_RUNNER_HPP_
#define FSAI_SIM2_ADAPTER__SIMULATION_RUNNER_HPP_

#include <cstddef>
#include <stdexcept>
#include <string>

#include "eufs_sim2/simulation.hpp"
#include "eufs_sim2/time/time.hpp"

namespace fsai::sim2_adapter {

enum class RunMode {
  kRealtime,
  kAsFastAsPossible
};

class SimulationRunner final {
 public:
  using Duration = eufs::sim2::time::Duration;

  SimulationRunner(eufs::sim2::SimulationBase &simulation, Duration outer_step);

  void StepOnce();
  void RunSteps(std::size_t count);
  Duration outer_step() const { return outer_step_; }

 private:
  eufs::sim2::SimulationBase &simulation_;
  Duration outer_step_;
};

inline RunMode ParseRunMode(const std::string &name) {
  if (name == "realtime") {
    return RunMode::kRealtime;
  }
  if (name == "as_fast_as_possible") {
    return RunMode::kAsFastAsPossible;
  }
  throw std::invalid_argument("unknown run_mode: " + name);
}

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__SIMULATION_RUNNER_HPP_
