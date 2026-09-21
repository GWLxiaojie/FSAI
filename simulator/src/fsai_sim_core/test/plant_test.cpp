#include <chrono>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "fsai_sim_core/plant.hpp"

using namespace std::chrono_literals;
using fsai::sim::Command;
using fsai::sim::Plant;
using fsai::sim::ReferenceBicycleParameters;
using fsai::sim::ValidationError;

TEST(Plant, UsesActuatorThenIntegratorThenGroundTruth) {
  Plant plant(ReferenceBicycleParameters());
  auto state = plant.InitialState();
  Command command{};
  auto result = plant.Update(state, command, 5ms);
  EXPECT_EQ(
    result.diagnostics.stage_order,
    (std::vector<std::string>({"actuator", "integrator", "ground_truth"})));
}

TEST(GroundTruth, ReportsPhysicalCgAcceleration) {
  Plant plant(ReferenceBicycleParameters());
  auto state = plant.InitialState();
  state.chassis.u_mps = 8.0;
  Command command{};
  command.rear_axle_torque_nm = 80.0;
  auto result = plant.Update(state, command, 5ms);
  EXPECT_TRUE(std::isfinite(result.ground_truth.ax_body_mps2));
  EXPECT_TRUE(std::isfinite(result.ground_truth.ay_body_mps2));
}

TEST(GroundTruth, ReportsAcceptedAxleForcesAndAerodynamicNormalLoad) {
  const auto p = ReferenceBicycleParameters();
  Plant plant(p); auto s = plant.InitialState(); s.chassis.u_mps = 10.;
  Command command{}; command.rear_axle_torque_nm = 50.;
  const auto result = plant.Update(s, command, 5ms);
  const auto &truth = result.ground_truth;
  EXPECT_NEAR(truth.rear_longitudinal_force_n, 50./p.effective_tyre_radius_m, 1e-10);
  EXPECT_DOUBLE_EQ(truth.front_longitudinal_force_n, 0.);
  EXPECT_NEAR(truth.front_normal_force_n + truth.rear_normal_force_n,
    p.mass_kg*p.gravity_mps2 + p.lumped_downforce_n_s2_per_m2 *
      truth.chassis.u_mps*truth.chassis.u_mps, 1e-9);
}

TEST(Plant, RejectsParameterHashMismatch) {
  Plant plant(ReferenceBicycleParameters());
  auto state = plant.InitialState();
  state.parameter_hash = "deadbeef";
  EXPECT_THROW(plant.Update(state, Command{}, 5ms), ValidationError);
}

TEST(Plant, RejectsSnapshotsFromPreviousDynamicsRevision) {
  Plant plant(ReferenceBicycleParameters());auto s=plant.InitialState();
  s.backend_revision="1";
  EXPECT_THROW(plant.Update(s,Command{},5ms),ValidationError);
}

TEST(Plant, SnapshotReplayMatches) {
  Plant plant(ReferenceBicycleParameters());
  auto state = plant.InitialState();
  Command command{};
  command.rear_axle_torque_nm = 40.0;
  for (int i = 0; i < 100; ++i) {
    auto result = plant.Update(state, command, 5ms);
    state = result.next_state;
  }
  const auto snapshot = state;
  std::vector<double> first_pass;
  auto replay_state = snapshot;
  for (int i = 0; i < 100; ++i) {
    auto result = plant.Update(replay_state, command, 5ms);
    first_pass.push_back(result.next_state.chassis.u_mps);
    replay_state = result.next_state;
  }
  replay_state = snapshot;
  for (int i = 0; i < 100; ++i) {
    auto result = plant.Update(replay_state, command, 5ms);
    EXPECT_DOUBLE_EQ(result.next_state.chassis.u_mps, first_pass[static_cast<std::size_t>(i)]);
    replay_state = result.next_state;
  }
}
