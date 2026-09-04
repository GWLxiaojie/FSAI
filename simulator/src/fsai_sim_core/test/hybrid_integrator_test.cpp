#include <array>
#include <chrono>

#include <gtest/gtest.h>

#include "fsai_sim_core/hybrid_integrator.hpp"

using namespace std::chrono_literals;
using fsai::sim::ActuatorState;
using fsai::sim::ChassisState;
using fsai::sim::ContactMode;
using fsai::sim::EventType;
using fsai::sim::HybridIntegrator;
using fsai::sim::ReferenceBicycleParameters;
using fsai::sim::WheelState;

TEST(HybridIntegrator, UsesFiveOneMillisecondSubsteps) {
  const auto params = ReferenceBicycleParameters();
  HybridIntegrator integrator;
  ChassisState state{};
  state.u_mps = 5.0;
  ActuatorState input{};
  std::array<WheelState, 4> wheels{};
  auto result = integrator.Integrate(state, input, 5ms, params, wheels);
  EXPECT_EQ(result.diagnostics.internal_steps, 5u);
}

TEST(HybridIntegrator, LocatesStopWithoutReverseOvershoot) {
  const auto params = ReferenceBicycleParameters();
  HybridIntegrator integrator;
  ChassisState state{};
  state.u_mps = 0.05;
  ActuatorState input{};
  input.friction_brake_ratio = 1.0;
  std::array<WheelState, 4> wheels{};
  auto result = integrator.Integrate(state, input, 100ms, params, wheels);
  EXPECT_DOUBLE_EQ(result.state.u_mps, 0.0);
  EXPECT_GE(result.state.u_mps, 0.0);
  EXPECT_EQ(result.wheels[2].contact_mode, ContactMode::kBrakeHold);
  ASSERT_FALSE(result.events.empty());
  EXPECT_EQ(result.events.front().type, EventType::kVehicleStopped);
}
