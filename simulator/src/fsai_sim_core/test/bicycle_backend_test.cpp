#include <cmath>

#include <gtest/gtest.h>

#include "fsai_sim_core/bicycle_backend.hpp"

using fsai::sim::ActuatorState;
using fsai::sim::ChassisState;
using fsai::sim::EvaluateBicycle;
using fsai::sim::ReferenceBicycleParameters;

TEST(BicycleBackend, LeftTurnProducesPositiveLateralForce) {
  const auto params = ReferenceBicycleParameters();
  ChassisState state{};
  state.u_mps = 10.0;
  ActuatorState input{};
  input.steering_angle_rad = 0.1;
  auto out = EvaluateBicycle(state, input, params);
  EXPECT_GT(out.front_lateral_force_n, 0.0);
  EXPECT_GT(out.derivative.yaw_rate_radps2, 0.0);
}

TEST(BicycleBackend, StaticSteeringProducesNoTyreForce) {
  const auto params = ReferenceBicycleParameters();
  ChassisState state{};
  ActuatorState input{};
  input.steering_angle_rad = 0.2;
  auto out = EvaluateBicycle(state, input, params);
  EXPECT_DOUBLE_EQ(out.front_lateral_force_n, 0.0);
  EXPECT_DOUBLE_EQ(out.rear_lateral_force_n, 0.0);
}

TEST(BicycleBackend, ResultsStayFinite) {
  const auto params = ReferenceBicycleParameters();
  for (double u = 0.0; u <= 40.0; u += 5.0) {
    ChassisState state{};
    state.u_mps = u;
    ActuatorState input{};
    input.steering_angle_rad = 0.2;
    input.rear_axle_torque_nm = 50.0;
    auto out = EvaluateBicycle(state, input, params);
    EXPECT_TRUE(std::isfinite(out.derivative.u_mps2));
    EXPECT_TRUE(std::isfinite(out.front_lateral_force_n));
    EXPECT_TRUE(std::isfinite(out.rear_lateral_force_n));
  }
}
