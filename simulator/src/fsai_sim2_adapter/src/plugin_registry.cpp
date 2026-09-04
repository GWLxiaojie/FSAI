#include "fsai_sim2_adapter/plugin_registry.hpp"

#include <stdexcept>
#include <utility>

namespace fsai::sim2_adapter {
namespace {

bool MatchesBoundaryPrefix(std::string_view instance_name, std::string_view prefix) {
  if (instance_name == prefix) {
    return true;
  }
  return instance_name.size() > prefix.size() &&
         instance_name.substr(0, prefix.size()) == prefix &&
         instance_name[prefix.size()] == '.';
}

}  // namespace

void PluginRegistry::Register(std::string prefix, Creator creator) {
  if (prefix.empty()) {
    throw std::invalid_argument("plugin registry prefix must not be empty");
  }
  if (!creator) {
    throw std::invalid_argument("plugin registry creator must not be empty");
  }
  if (creators_.contains(prefix)) {
    throw std::invalid_argument("duplicate plugin registry prefix: " + prefix);
  }
  creators_.emplace(std::move(prefix), std::move(creator));
}

void PluginRegistry::Validate(std::string_view instance_name) const {
  (void)Lookup(instance_name);
}

std::unique_ptr<eufs::sim2::plugin::Plugin> PluginRegistry::Create(
  std::string_view instance_name) const {
  return Lookup(instance_name)(std::string(instance_name));
}

const PluginRegistry::Creator &PluginRegistry::Lookup(std::string_view instance_name) const {
  const std::string *best_prefix = nullptr;
  const Creator *best_creator = nullptr;
  for (const auto &[prefix, creator] : creators_) {
    if (!MatchesBoundaryPrefix(instance_name, prefix)) {
      continue;
    }
    if (best_prefix == nullptr || prefix.size() > best_prefix->size()) {
      best_prefix = &prefix;
      best_creator = &creator;
    }
  }
  if (best_creator == nullptr) {
    throw std::invalid_argument("unknown plugin instance: " + std::string(instance_name));
  }
  return *best_creator;
}

}  // namespace fsai::sim2_adapter
