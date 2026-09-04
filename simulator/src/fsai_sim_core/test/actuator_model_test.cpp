#include <chrono>

#include <gtest/gtest.h>

#include "fsai_sim_core/actuator_model.hpp"

using namespace std::chrono_literals;
using fsai::sim::ActuatorModel;
using fsai::sim::ActuatorState;
using fsai::sim::Command;
using fsai::sim::ReferenceBicycleParameters;

TEST(ActuatorModel, LimitsSteeringRate) {
  const auto params = ReferenceBicycleParameters();
  ActuatorState state{};
  Command command{};
  command.steering_angle_rad = 0.5;
  ActuatorModel model;
  auto result = model.Update(state, command, 10ms, params);
  EXPECT_NEAR(
    result.state.steering_angle_rad,
    params.max_steering_rate_radps * 0.01,
    1e-12);
}
