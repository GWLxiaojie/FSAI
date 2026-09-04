#ifndef FSAI_SIM2_ADAPTER__PLUGIN_REGISTRY_HPP_
#define FSAI_SIM2_ADAPTER__PLUGIN_REGISTRY_HPP_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>

#include "eufs_sim2/plugin/base.hpp"

namespace fsai::sim2_adapter {

class PluginRegistry final {
 public:
  using Creator = std::function<std::unique_ptr<eufs::sim2::plugin::Plugin>(std::string)>;

  void Register(std::string prefix, Creator creator);
  void Validate(std::string_view instance_name) const;
  std::unique_ptr<eufs::sim2::plugin::Plugin> Create(std::string_view instance_name) const;

 private:
  const Creator &Lookup(std::string_view instance_name) const;

  std::map<std::string, Creator> creators_;
};

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__PLUGIN_REGISTRY_HPP_
