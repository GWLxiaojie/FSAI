#include <array>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <sstream>

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

TEST(HybridIntegrator, LowSpeedReactionForcesAgreeWithAcceleration) {
  const auto p=ReferenceBicycleParameters(); ChassisState s{}; s.u_mps=.04;
  ActuatorState a{}; a.steering_angle_rad=.3;
  auto result=HybridIntegrator{}.Integrate(s,a,5ms,p,{});
  const auto &q=result.state;const auto &e=result.last_evaluation;
  ASSERT_TRUE(e.kinematic_constraint_active);
  const double front_y=e.front_lateral_force_n*std::cos(.3);
  const double rear_y=e.rear_lateral_force_n;
  EXPECT_NEAR(front_y+rear_y,p.mass_kg*(e.derivative.v_mps2+q.yaw_rate_radps*q.u_mps),1e-10);
  const auto d=fsai::sim::Derive(p);
  EXPECT_NEAR(d.cg_to_front_axle_m*front_y-d.cg_to_rear_axle_m*rear_y,
    p.yaw_inertia_kgm2*e.derivative.yaw_rate_radps2,1e-10);
}

TEST(HybridIntegrator, LowSpeedConstraintDoesNotExceedTyreEnvelope) {
  const auto p=ReferenceBicycleParameters();ChassisState s{};s.u_mps=.01;
  ActuatorState a{};a.steering_angle_rad=.384;a.rear_axle_torque_nm=393.9;
  auto result=HybridIntegrator{}.Integrate(s,a,1ms,p,{});
  const auto &e=result.last_evaluation;
  const double load=p.mass_kg*p.gravity_mps2+
    p.lumped_downforce_n_s2_per_m2*result.state.u_mps*result.state.u_mps;
  EXPECT_LE(std::hypot(e.rear_longitudinal_force_n,e.rear_lateral_force_n),
    p.pacejka_D_rear*(1-p.front_static_load_fraction)*load+1e-8);
}

TEST(HybridIntegrator, Rk4ErrorConvergesWhenInternalStepIsHalved) {
  auto p = ReferenceBicycleParameters();
  p.rolling_resistance = 0.0;
  // Analytic numerical stress case, not a calibrated aero profile: du/dt=-u^2.
  p.lumped_drag_n_s2_per_m2 = p.mass_kg;
  ChassisState s{}; s.u_mps = 30.0;
  const double exact = 30.0 / (1.0 + 30.0 * .05);
  const auto coarse = HybridIntegrator(1ms).Integrate(s, {}, 50ms, p, {});
  const auto medium = HybridIntegrator(500us).Integrate(s, {}, 50ms, p, {});
  const auto fine = HybridIntegrator(250us).Integrate(s, {}, 50ms, p, {});
  const double e1 = std::abs(coarse.state.u_mps - exact);
  const double e2 = std::abs(medium.state.u_mps - exact);
  const double e3 = std::abs(fine.state.u_mps - exact);
  EXPECT_GT(e1, 8.0 * e2);
  EXPECT_GT(e2, 8.0 * e3);
  EXPECT_LT(e1, 24.0 * e2);
  EXPECT_LT(e2, 24.0 * e3);
  EXPECT_LT(e3, 1e-8);
  const auto scientific = [](double value) {
    std::ostringstream text;
    text << std::scientific << std::setprecision(12) << value;
    return text.str();
  };
  RecordProperty("error_1ms", scientific(e1));
  RecordProperty("error_500us", scientific(e2));
  RecordProperty("error_250us", scientific(e3));
}
