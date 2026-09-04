#ifndef FSAI_SIM_CORE__DYNAMICS_BACKEND_HPP_
#define FSAI_SIM_CORE__DYNAMICS_BACKEND_HPP_

#include "fsai_sim_core/parameters.hpp"
#include "fsai_sim_core/types.hpp"

namespace fsai::sim {

struct DynamicsEvaluation final {
  ChassisDerivative derivative;
  double front_lateral_force_n{};
  double rear_lateral_force_n{};
  double front_slip_angle_rad{};
  double rear_slip_angle_rad{};
  double drive_force_n{};
  double brake_force_n{};
  double drag_force_n{};
  double roll_force_n{};
  double yaw_moment_nm{};
  double net_longitudinal_force_n{};
};

class DynamicsBackend {
 public:
  virtual ~DynamicsBackend() = default;
  virtual DynamicsEvaluation Evaluate(
    const ChassisState &chassis,
    const ActuatorState &actuator,
    const VehicleParameters &parameters) const = 0;
};

}  // namespace fsai::sim

#endif  // FSAI_SIM_CORE__DYNAMICS_BACKEND_HPP_
