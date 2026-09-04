#include <stdexcept>
#include <string>

#include <gtest/gtest.h>

#include "fsai_sim2_adapter/core_factory.hpp"

namespace {

fsai::sim2_adapter::CoreFactory::Creator MakeEufsCreator() {
  return [](const fsai::sim2_adapter::CoreConfig &)
           -> std::unique_ptr<eufs::sim2::core::CoreSimulationBase> {
    throw std::logic_error("creator should not run in this test");
  };
}

TEST(CoreFactory, RejectsDuplicateAndUnknownKeys) {
  fsai::sim2_adapter::CoreFactory factory;
  factory.Register("eufs", MakeEufsCreator());
  EXPECT_THROW(factory.Register("eufs", MakeEufsCreator()), std::invalid_argument);
  EXPECT_THROW(
    factory.Create("missing", fsai::sim2_adapter::CoreConfig{}),
    std::invalid_argument);
}

TEST(CoreFactory, RejectsEmptyKey) {
  fsai::sim2_adapter::CoreFactory factory;
  EXPECT_THROW(factory.Register("", MakeEufsCreator()), std::invalid_argument);
}

}  // namespace
