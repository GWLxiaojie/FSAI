#include <chrono>  // NOLINT
#include <memory>
#include <utility>

#include <gtest/gtest.h>

#include "eufs_sim2/core/eufs_core.hpp"
#include "eufs_sim2/simulation.hpp"
#include "vehicle_models/types/param.hpp"

namespace {

using namespace std::chrono_literals;

eufs::vehicle_models::Param TestParams() {
  eufs::vehicle_models::Param params;
  params.inertia = {.m = 300.0, .g = 9.81, .I_z = 172.44};
  params.kinematic = eufs::vehicle_models::Param::Kinematic(1.53, 0.5, 1.2);
  params.tyre = {
    .A = 1.0,
    .B = 1.4,
    .C = 12.56,
    .radius = 0.2525,
    .rolling_resistance = 0.02,
  };
  params.powertrain = {.overdrive = 0.01};
  params.steering = {.max_rate = 0.39, .max_angle = 0.384};
  params.aero = {.c_down = 0.3, .c_drag = 0.5};
  params.input_ranges = {
    .acc = {.min = -5.2, .max = 5.2},
    .vel = {.min = 0.0, .max = 30.0},
    .delta = {.min = -0.37, .max = 0.37},
  };
  return params;
}

TEST(EufsComposition, AdvancesOriginalCoreThroughSimulationBase) {
  auto core = std::make_unique<eufs::sim2::core::EufsCore>(TestParams());
  eufs::sim2::SimulationBase simulation(std::move(core));
  simulation.Step(5ms);
  EXPECT_EQ(simulation.GetCore().GetTime(), 5ms);
}

}  // namespace
