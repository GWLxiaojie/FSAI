#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include <gtest/gtest.h>

#include "eufs_sim2/plugin/base.hpp"
#include "fsai_sim2_adapter/plugin_registry.hpp"

namespace {

class FakePlugin : public eufs::sim2::plugin::Plugin {
 public:
  explicit FakePlugin(std::string name) : Plugin(std::move(name)) {}
};

fsai::sim2_adapter::PluginRegistry::Creator MakeNamedPlugin(
  std::string label,
  std::string *sink) {
  return [label = std::move(label), sink](std::string name) {
    if (sink != nullptr) {
      *sink = label;
    }
    return std::make_unique<FakePlugin>(std::move(name));
  };
}

TEST(PluginRegistry, UsesLongestBoundaryPrefix) {
  fsai::sim2_adapter::PluginRegistry registry;
  std::string created;
  registry.Register("sensor", MakeNamedPlugin("generic", &created));
  registry.Register("sensor.camera", MakeNamedPlugin("camera", &created));
  auto plugin = registry.Create("sensor.camera.front");
  ASSERT_NE(plugin, nullptr);
  EXPECT_EQ(created, "camera");
}

TEST(PluginRegistry, RejectsNonBoundaryPrefix) {
  fsai::sim2_adapter::PluginRegistry registry;
  registry.Register("sensor", MakeNamedPlugin("generic", nullptr));
  EXPECT_THROW(registry.Create("sensorfoo"), std::invalid_argument);
}

TEST(PluginRegistry, RejectsDuplicateAndUnknownKeys) {
  fsai::sim2_adapter::PluginRegistry registry;
  registry.Register("sensor", MakeNamedPlugin("generic", nullptr));
  EXPECT_THROW(
    registry.Register("sensor", MakeNamedPlugin("other", nullptr)),
    std::invalid_argument);
  EXPECT_THROW(registry.Create("missing"), std::invalid_argument);
}

}  // namespace
