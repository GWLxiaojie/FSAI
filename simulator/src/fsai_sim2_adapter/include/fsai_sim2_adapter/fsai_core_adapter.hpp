#ifndef FSAI_SIM2_ADAPTER__FSAI_CORE_ADAPTER_HPP_
#define FSAI_SIM2_ADAPTER__FSAI_CORE_ADAPTER_HPP_

#include <stdexcept>
#include <string>

#include "eufs_sim2/core/base.hpp"
#include "fsai_sim_core/plant.hpp"
#include "fsai_sim_core/safety_chain.hpp"

namespace fsai::sim2_adapter {

class InterfaceError : public std::runtime_error {
 public:
  explicit InterfaceError(const std::string &message)
      : std::runtime_error(message) {}
};

class FsaiCoreAdapter final : public eufs::sim2::core::CoreSimulationBase {
 public:
  using Duration = eufs::sim2::core::Duration;
  using Time = eufs::sim2::core::Time;
  using ControlInput = eufs::sim2::core::ControlInput;
  using VehicleState = eufs::sim2::core::VehicleState;
  using WheelSpeeds = eufs::sim2::core::WheelSpeeds;
  using VehicleForces = eufs::sim2::core::VehicleForces;
  explicit FsaiCoreAdapter(fsai::sim::VehicleParameters parameters);

  void Step(Duration dt) override;
  void SetCommand(ControlInput cmd) override;
  void SetPhysicalCommand(const fsai::sim::Command &command);
  void SetPhysicalCommand(const fsai::sim::Command &command, fsai::sim::SimTime stamp);
  void SetInitialPose(const fsai::sim::ChassisState &pose);
  void SetDriving(bool as_driving);
  void RequestEbs();
  [[nodiscard]] Time GetTime() const override;
  [[nodiscard]] VehicleState::Vector GetState(VehicleState::Vector &vec) const override;
  [[nodiscard]] WheelSpeeds::Vector GetState(WheelSpeeds::Vector &vec) const override;
  [[nodiscard]] VehicleForces GetVehicleForces() const override;
  void Reset() override;

  [[nodiscard]] const fsai::sim::Command &LastAppliedCommand() const {
    return last_applied_;
  }
  [[nodiscard]] const fsai::sim::PlantState &plant_state() const { return plant_state_; }
  [[nodiscard]] const fsai::sim::StepResult &last_result() const { return last_result_; }
  [[nodiscard]] const fsai::sim::VehicleParameters &parameters() const {
    return plant_.parameters();
  }
  void enable_eufs_acceleration_compatibility(bool enabled) {
    eufs_compatibility_ = enabled;
  }

 private:
  fsai::sim::Plant plant_;
  fsai::sim::PlantState plant_state_;
  fsai::sim::Command pending_command_{};
  fsai::sim::Command last_applied_{};
  fsai::sim::StepResult last_result_{};
  fsai::sim::ChassisState initial_pose_{};
  bool as_driving_{false};
  bool eufs_compatibility_{false};
};

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__FSAI_CORE_ADAPTER_HPP_
