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
  explicit FsaiCoreAdapter(fsai::sim::VehicleParameters parameters);

  void Step(Duration dt) override;
  void SetCommand(ControlInput cmd) override;
  void SetPhysicalCommand(const fsai::sim::Command &command);
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
  void enable_eufs_acceleration_compatibility(bool enabled) {
    eufs_compatibility_ = enabled;
  }

 private:
  fsai::sim::Plant plant_;
  fsai::sim::PlantState plant_state_;
  fsai::sim::Command pending_command_{};
  fsai::sim::Command last_applied_{};
  fsai::sim::StepResult last_result_{};
  bool as_driving_{true};
  bool eufs_compatibility_{false};
};

}  // namespace fsai::sim2_adapter

#endif  // FSAI_SIM2_ADAPTER__FSAI_CORE_ADAPTER_HPP_
