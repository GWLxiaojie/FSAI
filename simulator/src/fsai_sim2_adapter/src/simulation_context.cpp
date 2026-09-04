#include "fsai_sim2_adapter/simulation_context.hpp"

namespace fsai::sim2_adapter {

SimulationContext SimulationContextFactory::Create(
  CoreFactory &cores,
  PluginRegistry &plugins,
  const std::string &core_key,
  const CoreConfig &config,
  SimulationRunner::Duration outer_step) const {
  (void)plugins;
  SimulationContext context;
  try {
    auto core = cores.Create(core_key, config);
    context.simulation = std::make_shared<eufs::sim2::SimulationBase>(std::move(core));
    context.runner = std::make_unique<SimulationRunner>(*context.simulation, outer_step);
  } catch (const std::exception &error) {
    throw ConfigurationError(error.what());
  }
  return context;
}

}  // namespace fsai::sim2_adapter
