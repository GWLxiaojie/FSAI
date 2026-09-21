#include <algorithm>
#include <cmath>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "fsai_sim2_adapter/reference_driver.hpp"
#include "fsai_sim_core/plant.hpp"

using namespace std::chrono_literals;
using fsai::sim2_adapter::ReferenceDriver;
using fsai::sim2_adapter::ReferenceDriverConfig;
using fsai::sim2_adapter::ReferenceWaypoint;

namespace {
std::vector<ReferenceWaypoint> Circle(double radius = 12.0, double width = 3.5) {
  std::vector<ReferenceWaypoint> points;
  for (int i = 0; i < 240; ++i) {
    const double angle = 2.0 * std::numbers::pi * i / 240;
    points.push_back({radius * std::cos(angle), radius * std::sin(angle), width});
  }
  return points;
}

fsai::sim::PlantState CircleStart(const fsai::sim::Plant &plant) {
  auto state = plant.InitialState();
  state.chassis.x_m = 12.0;
  state.chassis.yaw_rad = std::numbers::pi / 2.0;
  return state;
}
}  // namespace

TEST(ReferenceDriver, CompletesTenRealPlantLapsThenStops) {
  const auto parameters = fsai::sim::ReferenceBicycleParameters();
  fsai::sim::Plant plant(parameters);
  ReferenceDriver driver(Circle(), parameters);
  auto state = CircleStart(plant);
  fsai::sim2_adapter::ReferenceDriverOutput result;
  double maximum_error = 0.0;
  for (int i = 0; i < 20000; ++i) {
    result = driver.Update(state);
    ASSERT_FALSE(result.fault) << result.reason << " at " << state.sim_time.count() * 1e-9;
    maximum_error = std::max(maximum_error, std::abs(result.cross_track_error_m));
    if (result.complete) { break; }
    state = plant.Update(state, result.command, 20ms).next_state;
  }
  EXPECT_TRUE(result.complete);
  EXPECT_EQ(result.completed_laps, 10u);
  EXPECT_LT(maximum_error, 0.45);
  EXPECT_NEAR(state.chassis.u_mps, 0.0, 0.02);
  EXPECT_DOUBLE_EQ(result.command.rear_axle_torque_nm, 0.0);
  EXPECT_GT(result.command.friction_brake_ratio, 0.0);
}

TEST(ReferenceDriver, ChecksFullOfficialBodyEnvelopeNotOnlyTheCentre) {
  const auto parameters = fsai::sim::ReferenceBicycleParameters();
  fsai::sim::Plant plant(parameters);
  auto state = CircleStart(plant);
  state.chassis.yaw_rad = 0.0;
  ReferenceDriver driver(Circle(12.0, 3.0), parameters);
  const auto result = driver.Update(state);
  EXPECT_NEAR(result.cross_track_error_m, 0.0, 1e-9);
  EXPECT_TRUE(result.fault);
  EXPECT_LT(result.minimum_clearance_m, 0.0);
  EXPECT_GT(result.command.friction_brake_ratio, 0.0);
  EXPECT_DOUBLE_EQ(result.command.rear_axle_torque_nm, 0.0);
}

TEST(ReferenceDriver, StartLineDitheringDoesNotCountLapsAndTeleportsLatchFault) {
  const auto parameters = fsai::sim::ReferenceBicycleParameters();
  fsai::sim::Plant plant(parameters);
  auto state = CircleStart(plant);
  ReferenceDriver driver(Circle(), parameters);
  for (int i = 0; i < 1000; ++i) {
    state.chassis.y_m = i % 2 == 0 ? 0.01 : -0.01;
    const auto result = driver.Update(state);
    ASSERT_FALSE(result.fault) << result.reason;
    EXPECT_EQ(result.completed_laps, 0u);
    state.sim_time += 20ms;
  }
  state.chassis.x_m = -12.0;
  auto result = driver.Update(state);
  EXPECT_TRUE(result.fault);
  EXPECT_EQ(result.completed_laps, 0u);
  state = CircleStart(plant);
  result = driver.Update(state);
  EXPECT_TRUE(result.fault) << "Fault must stay latched until explicit reset";
  driver.Reset();
  EXPECT_FALSE(driver.Update(state).fault);
}

TEST(ReferenceDriver, RejectsInvalidRoutesAndBrokenSimulationCadence) {
  const auto parameters = fsai::sim::ReferenceBicycleParameters();
  EXPECT_THROW(ReferenceDriver({}, parameters), std::invalid_argument);
  EXPECT_THROW(ReferenceDriver(Circle(12.0, 1.0), parameters), std::invalid_argument);
  auto route = Circle();
  route[1] = route[0];
  EXPECT_THROW(ReferenceDriver(route, parameters), std::invalid_argument);
  fsai::sim::Plant plant(parameters);
  auto state = CircleStart(plant);
  ReferenceDriver driver(Circle(), parameters);
  ASSERT_FALSE(driver.Update(state).fault);
  state.sim_time += 40ms;
  EXPECT_TRUE(driver.Update(state).fault);
}

TEST(ReferenceDriver, RepeatsExactlyAcrossOuterIntegrationSteps) {
  const auto parameters = fsai::sim::ReferenceBicycleParameters();
  fsai::sim::Plant plant(parameters);
  auto run = [&](fsai::sim::Duration outer_step) {
    ReferenceDriverConfig config;
    config.target_laps = 2;
    ReferenceDriver driver(Circle(), parameters, config);
    auto state = CircleStart(plant);
    std::vector<double> trace;
    for (int i = 0; i < 5000; ++i) {
      const auto result = driver.Update(state);
      EXPECT_FALSE(result.fault) << result.reason;
      trace.insert(trace.end(), {state.chassis.x_m, state.chassis.y_m, state.chassis.yaw_rad,
        state.chassis.u_mps, result.progress_m, result.command.steering_angle_rad,
        result.command.rear_axle_torque_nm, result.command.friction_brake_ratio});
      if (result.complete) { return trace; }
      for (auto elapsed = 0ms; elapsed < 20ms; elapsed +=
        std::chrono::duration_cast<std::chrono::milliseconds>(outer_step)) {
        state = plant.Update(state, result.command, outer_step).next_state;
      }
    }
    ADD_FAILURE() << "Controller failed to stop after two laps";
    return trace;
  };
  const auto first = run(5ms);
  const auto repeated = run(5ms);
  const auto smaller_step = run(1ms);
  EXPECT_EQ(first, repeated);
  EXPECT_EQ(first, smaller_step);
}

TEST(ReferenceDriver, TracksAlternatingCurvatureWithRealPlant) {
  // Algorithm fixture only, never an official track: a smooth radial contour
  // with inward/outward bends exercises curvature sign changes and preview.
  std::vector<ReferenceWaypoint> route;
  for (int i = 0; i < 720; ++i) {
    const double angle = 2.0 * std::numbers::pi * i / 720;
    const double radius = 30.0 + 5.0 * std::cos(3.0 * angle);
    route.push_back({radius * std::cos(angle), radius * std::sin(angle), 3.5});
  }
  const auto parameters = fsai::sim::ReferenceBicycleParameters();
  ReferenceDriverConfig config;
  config.target_laps = 3;
  ReferenceDriver driver(route, parameters, config);
  fsai::sim::Plant plant(parameters);
  auto state = CircleStart(plant);
  state.chassis.x_m = 35.0;
  fsai::sim2_adapter::ReferenceDriverOutput result;
  for (int i = 0; i < 18000; ++i) {
    result = driver.Update(state);
    ASSERT_FALSE(result.fault) << result.reason << ", progress=" << result.progress_m;
    ASSERT_LE(std::abs(result.command.steering_angle_rad), parameters.max_steering_angle_rad);
    if (result.complete) { break; }
    state = plant.Update(state, result.command, 20ms).next_state;
  }
  EXPECT_TRUE(result.complete);
  EXPECT_EQ(result.completed_laps, 3u);
}

TEST(ReferenceDriver, NearbyCrossingBranchCannotShortcutOrderedProgress) {
  std::vector<ReferenceWaypoint> route;
  for (int i = 0; i < 360; ++i) {
    const double angle = 2.0 * std::numbers::pi * i / 360;
    route.push_back({15.0 * std::sin(angle), 10.0 * std::sin(2.0 * angle), 4.0});
  }
  const auto parameters = fsai::sim::ReferenceBicycleParameters();
  fsai::sim::Plant plant(parameters);
  auto state = plant.InitialState();
  state.chassis.yaw_rad = std::atan2(20.0, 15.0);
  ReferenceDriver driver(route, parameters);
  ASSERT_FALSE(driver.Update(state).fault);
  // Both branches meet at (0,0), but entering the other branch is not half a
  // lap of progress. Move in small admissible increments to avoid relying on
  // the separate teleport detector.
  fsai::sim2_adapter::ReferenceDriverOutput result;
  for (int i = 1; i < 150; ++i) {
    state.sim_time += 20ms;
    state.chassis.x_m = -0.03 * i;
    state.chassis.y_m = 0.04 * i;
    state.chassis.u_mps = 2.5;
    result = driver.Update(state);
    EXPECT_EQ(result.completed_laps, 0u);
    EXPECT_LT(result.progress_m, 5.0);
    if (result.fault) { break; }
  }
  EXPECT_TRUE(result.fault);
}
