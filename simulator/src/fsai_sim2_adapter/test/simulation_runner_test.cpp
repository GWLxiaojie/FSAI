#include <chrono>
#include <memory>
#include <stdexcept>
#include <vector>

#include <gtest/gtest.h>

#include "eufs_sim2/core/base.hpp"
#include "eufs_sim2/simulation.hpp"
#include "fsai_sim2_adapter/simulation_runner.hpp"

using namespace std::chrono_literals;

namespace {

class FakeCore final : public eufs::sim2::core::CoreSimulationBase {
 public:
  void Step(Duration dt) override {
    steps_.push_back(dt);
    time_ += dt;
  }
  void SetCommand(ControlInput) override {}
  Time GetTime() const override { return time_; }
  VehicleState::Vector GetState(VehicleState::Vector &vec) const override { return vec; }
  WheelSpeeds::Vector GetState(WheelSpeeds::Vector &vec) const override { return vec; }
  VehicleForces GetVehicleForces() const override { return {}; }
  void Reset() override {
    time_ = {};
    steps_.clear();
  }
  const std::vector<Duration> &StepDurations() const { return steps_; }

 private:
  Time time_{};
  std::vector<Duration> steps_;
};

}  // namespace

TEST(SimulationRunner, AdvancesExactSteps) {
  auto core = std::make_unique<FakeCore>();
  auto *fake = core.get();
  eufs::sim2::SimulationBase simulation(std::move(core));
  fsai::sim2_adapter::SimulationRunner runner(simulation, 5ms);
  runner.RunSteps(3);
  ASSERT_EQ(fake->StepDurations().size(), 3u);
  EXPECT_EQ(fake->GetTime(), 15ms);
}

TEST(SimulationRunner, RejectsZeroStep) {
  auto core = std::make_unique<FakeCore>();
  eufs::sim2::SimulationBase simulation(std::move(core));
  EXPECT_THROW(
    fsai::sim2_adapter::SimulationRunner(simulation, 0ms),
    std::invalid_argument);
}
