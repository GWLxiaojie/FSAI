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

TEST(BicycleBackend, DownforceChangesTyreCapacity) {
  auto p=ReferenceBicycleParameters(); ChassisState s{};s.u_mps=20.;
  ActuatorState a{};a.steering_angle_rad=.1;
  p.lumped_downforce_n_s2_per_m2=0;
  auto low=EvaluateBicycle(s,a,p);p.lumped_downforce_n_s2_per_m2=5;
  EXPECT_GT(EvaluateBicycle(s,a,p).front_lateral_force_n,low.front_lateral_force_n);
}

TEST(BicycleBackend, FrontDriveIsProjectedAndCombinedForceIsBounded) {
  auto p=ReferenceBicycleParameters();p.max_front_axle_torque_nm=1000;
  ChassisState s{};s.u_mps=10.;ActuatorState a{};a.steering_angle_rad=.2;
  auto coast=EvaluateBicycle(s,a,p);a.front_axle_torque_nm=10;
  auto drive=EvaluateBicycle(s,a,p);
  EXPECT_GT(drive.yaw_moment_nm,coast.yaw_moment_nm);
  s.v_mps=1.;a={};a.rear_axle_torque_nm=-393.9;a.friction_brake_ratio=1;
  auto braking=EvaluateBicycle(s,a,p);
  const double weight=p.mass_kg*p.gravity_mps2 + p.lumped_downforce_n_s2_per_m2*100;
  const double fx=braking.net_longitudinal_force_n-braking.drag_force_n-braking.roll_force_n;
  const double fy=braking.derivative.v_mps2*p.mass_kg;
  EXPECT_LE(std::hypot(fx,fy),weight+1e-8);
}
