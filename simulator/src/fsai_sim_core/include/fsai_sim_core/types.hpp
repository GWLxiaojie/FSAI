#ifndef FSAI_SIM_CORE__TYPES_HPP_
#define FSAI_SIM_CORE__TYPES_HPP_

#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace fsai::sim {

using Duration = std::chrono::nanoseconds;
using SimTime = std::chrono::nanoseconds;

class ValidationError : public std::runtime_error {
 public:
  explicit ValidationError(const std::string &message)
      : std::runtime_error(message) {}
};

struct Command final {
  double steering_angle_rad{};
  double front_axle_torque_nm{};
  double rear_axle_torque_nm{};
  double friction_brake_ratio{};
};

void Validate(const Command &command);

struct ChassisState final {
  double x_m{};
  double y_m{};
  double yaw_rad{};
  double u_mps{};
  double v_mps{};
  double yaw_rate_radps{};
};

struct ActuatorState final {
  double steering_angle_rad{};
  double front_axle_torque_nm{};
  double rear_axle_torque_nm{};
  double friction_brake_ratio{};
};

enum class ContactMode : std::uint8_t {
  kKinematic = 0,
  kStick,
  kTransition,
  kSlip,
  kBrakeHold
};

struct WheelState final {
  double omega_radps{};
  ContactMode contact_mode{ContactMode::kKinematic};
};

struct PlantState final {
  std::uint32_t schema_version{1};
  std::string backend_id{"fsai_bicycle"};
  std::string backend_revision{"1"};
  std::string parameter_hash;
  ChassisState chassis;
  ActuatorState actuator;
  std::array<WheelState, 4> wheels{};
  std::vector<double> backend_state;
  bool ebs_latched{false};
  SimTime last_command_time{0};
  SimTime sim_time{0};
};

enum class EventType : std::uint8_t {
  kNone = 0,
  kVehicleStopped,
  kHoldReleased,
  kEbsLatched,
  kCommandTimeout,
  kNonDriving
};

struct SimulationEvent final {
  SimTime time{};
  EventType type{EventType::kNone};
  std::string detail;
};

struct ChassisDerivative final {
  double x_mps{};
  double y_mps{};
  double yaw_radps{};
  double u_mps2{};
  double v_mps2{};
  double yaw_rate_radps2{};
};

struct GroundTruth final {
  ChassisState chassis;
  double ax_body_mps2{};
  double ay_body_mps2{};
  double steering_angle_rad{};
  std::array<double, 4> wheel_omega_radps{};
  double front_lateral_force_n{};
  double rear_lateral_force_n{};
};

struct Diagnostics final {
  std::uint32_t internal_steps{};
  std::vector<std::string> stage_order;
  double front_slip_angle_rad{};
  double rear_slip_angle_rad{};
  double drive_force_n{};
  double drag_force_n{};
  double roll_force_n{};
};

struct StepResult final {
  PlantState next_state;
  GroundTruth ground_truth;
  std::vector<SimulationEvent> events;
  Diagnostics diagnostics;
};

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__TYPES_HPP_
