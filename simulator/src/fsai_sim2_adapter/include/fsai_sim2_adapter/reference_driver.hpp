#ifndef FSAI_SIM2_ADAPTER__REFERENCE_DRIVER_HPP_
#define FSAI_SIM2_ADAPTER__REFERENCE_DRIVER_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "fsai_sim_core/parameters.hpp"
#include "fsai_sim_core/types.hpp"

namespace fsai::sim2_adapter {

// Ordered, closed centreline in map coordinates. Width is the FULL drivable
// corridor width in metres, not its half-width. Do not append a duplicate end.
struct ReferenceWaypoint {
  double x_m{};
  double y_m{};
  double width_m{};
};

struct ReferenceDriverConfig {
  std::uint32_t target_laps{10};
  double cruise_speed_mps{2.5};
  double body_width_m{1.43};
  double front_extent_m{1.60};
  double rear_extent_m{1.34};
  double clearance_m{0.10};
};

struct ReferenceDriverOutput {
  fsai::sim::Command command;
  std::uint32_t completed_laps{};
  double progress_m{};
  double cross_track_error_m{};
  double minimum_clearance_m{};
  double target_speed_mps{};
  bool complete{false};
  bool fault{false};
  std::string reason;
};

// Simulator self-test driver. Uses perfect PlantState, never a claim of
// sensor-based autonomous capability. Update exactly every 20 ms of sim time.
// The body envelope extends from -rear_extent_m to +front_extent_m about
// base_footprint/CG. Defaults cover the official visual mesh after CG alignment.
class ReferenceDriver {
 public:
  ReferenceDriver(std::vector<ReferenceWaypoint> centreline,
    fsai::sim::VehicleParameters parameters, ReferenceDriverConfig config = {});
  ReferenceDriverOutput Update(const fsai::sim::PlantState &state);
  void Reset();
  double track_length_m() const { return length_m_; }

 private:
  struct Projection {
    double arc{};
    double distance{};
    double signed_error{};
    double width{};
  };
  ReferenceWaypoint PointAt(double arc) const;
  Projection Project(double x, double y, double near_arc, double window) const;
  double CurvatureAt(double arc) const;
  ReferenceDriverOutput Stop(const fsai::sim::PlantState &state, bool fault,
    const std::string &reason);

  std::vector<ReferenceWaypoint> points_;
  std::vector<double> arcs_;
  fsai::sim::VehicleParameters parameters_;
  ReferenceDriverConfig config_;
  double length_m_{};
  double progress_m_{};
  double integral_speed_error_{};
  double last_steering_{};
  bool initialized_{false};
  fsai::sim::SimTime last_time_{};
  fsai::sim::Duration stopped_duration_{};
  fsai::sim::ChassisState previous_chassis_;
  ReferenceDriverOutput output_;
};
}  // namespace fsai::sim2_adapter
#endif
