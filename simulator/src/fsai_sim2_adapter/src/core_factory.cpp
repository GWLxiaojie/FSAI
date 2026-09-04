#include "fsai_sim2_adapter/core_factory.hpp"

#include <stdexcept>
#include <utility>

namespace fsai::sim2_adapter {

void CoreFactory::Register(std::string key, Creator creator) {
  if (key.empty()) {
    throw std::invalid_argument("core factory key must not be empty");
  }
  if (!creator) {
    throw std::invalid_argument("core factory creator must not be empty");
  }
  if (creators_.contains(key)) {
    throw std::invalid_argument("duplicate core factory key: " + key);
  }
  creators_.emplace(std::move(key), std::move(creator));
}

std::unique_ptr<eufs::sim2::core::CoreSimulationBase> CoreFactory::Create(
  std::string_view key,
  const CoreConfig &config) const {
  const auto found = creators_.find(std::string(key));
  if (found == creators_.end()) {
    throw std::invalid_argument("unknown core factory key: " + std::string(key));
  }
  return found->second(config);
}

}  // namespace fsai::sim2_adapter
