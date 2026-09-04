#ifndef FSAI_SIM2_ADAPTER__SIMULATION_CONTEXT_HPP_
#define FSAI_SIM2_ADAPTER__SIMULATION_CONTEXT_HPP_

#include <memory>
#include <stdexcept>
#include <utility>

#include "eufs_sim2/simulation.hpp"
#include "fsai_sim2_adapter/core_factory.hpp"
#include "fsai_sim2_adapter/plugin_registry.hpp"
#include "fsai_sim2_adapter/simulation_runner.hpp"

namespace fsai::sim2_adapter {

class ConfigurationError : public std::runtime_error {
 public:
  explicit ConfigurationError(const std::string &message)
      : std::runtime_error(message) {}
};

struct SimulationContext final {
  std::unique_ptr<eufs::sim2::core::CoreSimulationBase> core;
  std::shared_ptr<eufs::sim2::SimulationBase> simulation;
  std::unique_ptr<SimulationRunner> runner;
};

class SimulationContextFactory final {
 public:
  SimulationContext Create(
    CoreFactory &cores,
    PluginRegistry &plugins,
    const std::string &core_key,
    const CoreConfig &config,
    SimulationRunner::Duration outer_step) const;
};

class ContextOwner final {
 public:
  explicit ContextOwner(SimulationContext context)
      : current_(std::move(context)) {}

  SimulationContext &Current() { return current_; }
  const SimulationContext &Current() const { return current_; }

  void Replace(SimulationContext candidate) {
    if (!candidate.simulation || !candidate.runner) {
      throw ConfigurationError("incomplete simulation context");
    }
    current_ = std::move(candidate);
  }

 private:
  SimulationContext current_;
};

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__SIMULATION_CONTEXT_HPP_
