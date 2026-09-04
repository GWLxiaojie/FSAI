#ifndef FSAI_SIM2_ADAPTER__CORE_FACTORY_HPP_
#define FSAI_SIM2_ADAPTER__CORE_FACTORY_HPP_

#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <string_view>

#include "eufs_sim2/core/base.hpp"

namespace fsai::sim2_adapter {

struct CoreConfig final {
  std::filesystem::path parameter_file;
};

class CoreFactory final {
 public:
  using Creator = std::function<std::unique_ptr<eufs::sim2::core::CoreSimulationBase>(
    const CoreConfig &)>;

  void Register(std::string key, Creator creator);
  std::unique_ptr<eufs::sim2::core::CoreSimulationBase> Create(
    std::string_view key,
    const CoreConfig &config) const;

 private:
  std::map<std::string, Creator> creators_;
};

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__CORE_FACTORY_HPP_
