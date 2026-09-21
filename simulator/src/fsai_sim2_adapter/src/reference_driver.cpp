#include "fsai_sim2_adapter/reference_driver.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace fsai::sim2_adapter {
namespace {
constexpr double kControlPeriod = 0.020;
constexpr double kBrakeDeceleration = 0.8;

bool Finite(const fsai::sim::ChassisState &state) {
  return std::isfinite(state.x_m) && std::isfinite(state.y_m) &&
    std::isfinite(state.yaw_rad) && std::isfinite(state.u_mps) &&
    std::isfinite(state.v_mps) && std::isfinite(state.yaw_rate_radps);
}
}

ReferenceDriver::ReferenceDriver(std::vector<ReferenceWaypoint> centreline,
  fsai::sim::VehicleParameters parameters, ReferenceDriverConfig config)
    : points_(std::move(centreline)), parameters_(std::move(parameters)), config_(config) {
  fsai::sim::ValidateParameters(parameters_);
  if (points_.size() < 8 || config_.target_laps == 0 ||
      !std::isfinite(config_.cruise_speed_mps) || config_.cruise_speed_mps <= 0 ||
      config_.cruise_speed_mps > 3.0 ||
      !std::isfinite(config_.body_width_m) || config_.body_width_m <= 0 ||
      !std::isfinite(config_.front_extent_m) || config_.front_extent_m <= 0 ||
      !std::isfinite(config_.rear_extent_m) || config_.rear_extent_m <= 0 ||
      !std::isfinite(config_.clearance_m) || config_.clearance_m < 0) {
    throw std::invalid_argument("reference driver requires a closed route, positive dimensions/laps and speed in (0,3] m/s");
  }
  arcs_.push_back(0.0);
  for (std::size_t i = 0; i < points_.size(); ++i) {
    const auto &point = points_[i];
    const auto &next = points_[(i + 1) % points_.size()];
    if (!std::isfinite(point.x_m) || !std::isfinite(point.y_m) ||
        !std::isfinite(point.width_m) ||
        point.width_m <= config_.body_width_m + 2.0 * config_.clearance_m) {
      throw std::invalid_argument("reference centreline has non-finite coordinates or insufficient width");
    }
    const double segment = std::hypot(next.x_m - point.x_m, next.y_m - point.y_m);
    if (!std::isfinite(segment) || segment < 1e-6 || segment > 10.0) {
      throw std::invalid_argument("reference centreline requires distinct points at most 10m apart (including closure)");
    }
    arcs_.push_back(arcs_.back() + segment);
  }
  length_m_ = arcs_.back();
  if (length_m_ < 20.0) {
    throw std::invalid_argument("reference closed route must be at least 20m long");
  }
  Reset();
}

void ReferenceDriver::Reset() {
  progress_m_ = integral_speed_error_ = last_steering_ = 0.0;
  initialized_ = false;
  last_time_ = {};
  stopped_duration_ = {};
  previous_chassis_ = {};
  output_ = {};
}

ReferenceWaypoint ReferenceDriver::PointAt(double arc) const {
  double wrapped = std::fmod(arc, length_m_);
  if (wrapped < 0) { wrapped += length_m_; }
  const auto upper = std::upper_bound(arcs_.begin(), arcs_.end(), wrapped);
  const auto i = std::min<std::size_t>(points_.size() - 1, upper - arcs_.begin() - 1);
  const double t = (wrapped - arcs_[i]) / (arcs_[i + 1] - arcs_[i]);
  const auto &a = points_[i];
  const auto &b = points_[(i + 1) % points_.size()];
  return {a.x_m + t * (b.x_m - a.x_m), a.y_m + t * (b.y_m - a.y_m),
    a.width_m + t * (b.width_m - a.width_m)};
}

ReferenceDriver::Projection ReferenceDriver::Project(
  double x, double y, double near_arc, double window) const {
  Projection best{near_arc, std::numeric_limits<double>::infinity(), 0.0, 0.0};
  const double low = near_arc - window;
  const double high = near_arc + window;
  const auto first_cycle = static_cast<long long>(std::floor(low / length_m_));
  const auto last_cycle = static_cast<long long>(std::floor(high / length_m_));
  for (auto cycle = first_cycle; cycle <= last_cycle; ++cycle) {
    const double offset = cycle * length_m_;
    const double local_low = std::max(0.0, low - offset);
    auto start = std::upper_bound(arcs_.begin(), arcs_.end(), local_low);
    std::size_t i = start == arcs_.begin() ? 0 : start - arcs_.begin() - 1;
    for (; i < points_.size() && offset + arcs_[i] <= high; ++i) {
      const auto &a = points_[i];
      const auto &b = points_[(i + 1) % points_.size()];
      const double dx = b.x_m - a.x_m;
      const double dy = b.y_m - a.y_m;
      const double segment = arcs_[i + 1] - arcs_[i];
      const double min_t = std::clamp((low - offset - arcs_[i]) / segment, 0.0, 1.0);
      const double max_t = std::clamp((high - offset - arcs_[i]) / segment, 0.0, 1.0);
      const double t = std::clamp(((x - a.x_m) * dx + (y - a.y_m) * dy) /
        (segment * segment), min_t, max_t);
      const double distance = std::hypot(x - (a.x_m + t * dx), y - (a.y_m + t * dy));
      if (distance < best.distance) {
        best = {offset + arcs_[i] + t * segment, distance,
          (dx * (y - a.y_m) - dy * (x - a.x_m)) / segment,
          a.width_m + t * (b.width_m - a.width_m)};
      }
    }
  }
  return best;
}

double ReferenceDriver::CurvatureAt(double arc) const {
  const auto a = PointAt(arc - 1.0);
  const auto b = PointAt(arc);
  const auto c = PointAt(arc + 1.0);
  const double abx = b.x_m - a.x_m;
  const double aby = b.y_m - a.y_m;
  const double bcx = c.x_m - b.x_m;
  const double bcy = c.y_m - b.y_m;
  const double denominator = std::hypot(abx, aby) * std::hypot(bcx, bcy) *
    std::hypot(c.x_m - a.x_m, c.y_m - a.y_m);
  return denominator > 1e-9 ? 2.0 * (abx * bcy - aby * bcx) / denominator : 0.0;
}

ReferenceDriverOutput ReferenceDriver::Stop(
  const fsai::sim::PlantState &state, bool fault, const std::string &reason) {
  output_.fault = output_.fault || fault;
  if (!reason.empty()) { output_.reason = reason; }
  output_.target_speed_mps = 0.0;
  output_.command.front_axle_torque_nm = 0.0;
  output_.command.rear_axle_torque_nm = 0.0;
  output_.command.friction_brake_ratio = fault ? 1.0 : 0.4;
  // Retain steering during braking: straightening immediately can carry a car
  // out of a narrow corner before longitudinal velocity reaches zero.
  output_.command.steering_angle_rad = last_steering_;
  if (Finite(state.chassis) && std::hypot(state.chassis.u_mps, state.chassis.v_mps) < 0.02 &&
      std::abs(state.chassis.yaw_rate_radps) < 0.02) {
    stopped_duration_ += std::chrono::milliseconds(20);
  } else {
    stopped_duration_ = {};
  }
  output_.complete = !output_.fault && output_.completed_laps == config_.target_laps &&
    stopped_duration_ >= std::chrono::milliseconds(500);
  return output_;
}

ReferenceDriverOutput ReferenceDriver::Update(const fsai::sim::PlantState &state) {
  const auto &car = state.chassis;
  if (output_.fault || output_.complete) {
    return Stop(state, output_.fault, output_.reason);
  }
  if (!Finite(car) || !std::isfinite(state.actuator.steering_angle_rad) ||
      car.u_mps < -0.1 || std::hypot(car.u_mps, car.v_mps) > 8.0) {
    return Stop(state, true, "non-finite, reverse, or excessive-speed state");
  }
  if (state.ebs_latched) { return Stop(state, true, "EBS latched"); }
  if (initialized_) {
    if (state.sim_time - last_time_ != std::chrono::milliseconds(20)) {
      return Stop(state, true, "reference driver must advance by exactly 20ms simulation time");
    }
    const double travelled = std::hypot(car.x_m - previous_chassis_.x_m, car.y_m - previous_chassis_.y_m);
    const double possible = std::max(0.15,
      (std::hypot(car.u_mps, car.v_mps) +
       std::hypot(previous_chassis_.u_mps, previous_chassis_.v_mps)) * kControlPeriod + 0.05);
    if (travelled > possible) { return Stop(state, true, "vehicle pose jumped; ordered progress invalid"); }
  } else if (std::hypot(car.x_m - points_[0].x_m, car.y_m - points_[0].y_m) > 1.0) {
    return Stop(state, true, "reference run must start within one metre of the ordered start");
  }

  const double possible_progress = initialized_ ?
    std::max(0.15, std::hypot(car.u_mps, car.v_mps) * kControlPeriod * 2.0 + 0.05) : 1.0;
  const auto centre = Project(car.x_m, car.y_m, progress_m_, possible_progress);
  if (centre.distance > centre.width / 2.0) {
    return Stop(state, true, "vehicle centre outside its ordered route corridor");
  }
  progress_m_ = std::max(progress_m_, centre.arc);
  output_.progress_m = progress_m_;
  output_.cross_track_error_m = centre.signed_error;
  output_.completed_laps = std::min(config_.target_laps,
    static_cast<std::uint32_t>(std::floor(progress_m_ / length_m_)));
  last_time_ = state.sim_time;
  previous_chassis_ = car;
  initialized_ = true;

  const double cosine = std::cos(car.yaw_rad);
  const double sine = std::sin(car.yaw_rad);
  double clearance = std::numeric_limits<double>::infinity();
  // Sample the complete rectangle perimeter at <=0.2m, including every corner.
  // A 0.1m Lipschitz allowance conservatively covers space between samples.
  const double body_length = config_.front_extent_m + config_.rear_extent_m;
  const double half_width = config_.body_width_m / 2.0;
  const auto inspect = [&](double local_x, double local_y) {
    const double x = car.x_m + cosine * local_x - sine * local_y;
    const double y = car.y_m + sine * local_x + cosine * local_y;
    const auto projected = Project(x, y, progress_m_, body_length + 1.0);
    clearance = std::min(clearance, projected.width / 2.0 - projected.distance -
      config_.clearance_m - 0.10);
  };
  const int length_samples = static_cast<int>(std::ceil(body_length / 0.2));
  const int width_samples = static_cast<int>(std::ceil(config_.body_width_m / 0.2));
  for (int i = 0; i <= length_samples; ++i) {
    const double x = -config_.rear_extent_m + body_length * i / length_samples;
    inspect(x, -half_width);
    inspect(x, half_width);
  }
  for (int i = 0; i <= width_samples; ++i) {
    const double y = -half_width + config_.body_width_m * i / width_samples;
    inspect(-config_.rear_extent_m, y);
    inspect(config_.front_extent_m, y);
  }
  output_.minimum_clearance_m = clearance;
  if (clearance < 0) { return Stop(state, true, "vehicle footprint crosses the route safety margin"); }
  if (output_.completed_laps >= config_.target_laps) { return Stop(state, false, "target laps reached; stopping"); }

  // Pure pursuit from the rear axle, using the actual wheelbase/CG geometry.
  const double rear_distance = fsai::sim::Derive(parameters_).cg_to_rear_axle_m;
  const double rear_x = car.x_m - rear_distance * cosine;
  const double rear_y = car.y_m - rear_distance * sine;
  const double lookahead = std::clamp(1.5 + 0.45 * std::max(0.0, car.u_mps), 1.5, 3.0);
  const auto rear = Project(rear_x, rear_y, progress_m_ - rear_distance, 2.0);
  const auto target = PointAt(rear.arc + lookahead);
  const double dx = target.x_m - rear_x;
  const double dy = target.y_m - rear_y;
  const double lateral = -sine * dx + cosine * dy;
  const double steering = std::atan2(2.0 * parameters_.wheelbase_m * lateral, dx * dx + dy * dy);
  const double saturated = std::clamp(steering,
    -parameters_.max_steering_angle_rad, parameters_.max_steering_angle_rad);
  last_steering_ = std::clamp(saturated,
    state.actuator.steering_angle_rad - parameters_.max_steering_rate_radps * kControlPeriod,
    state.actuator.steering_angle_rad + parameters_.max_steering_rate_radps * kControlPeriod);

  double target_speed = config_.cruise_speed_mps;
  const double preview = std::max(8.0, car.u_mps * car.u_mps / (2.0 * kBrakeDeceleration) + 3.0);
  for (double ahead = 0; ahead <= preview; ahead += 0.5) {
    const double curvature = std::abs(CurvatureAt(progress_m_ + ahead));
    const double curve_speed = std::sqrt(0.65 / std::max(curvature, 1e-6));
    // Steering-rate preview bounds turn entry/exit speed as well as lateral g.
    const double before = std::atan(parameters_.wheelbase_m * CurvatureAt(progress_m_ + ahead - 0.5));
    const double after = std::atan(parameters_.wheelbase_m * CurvatureAt(progress_m_ + ahead + 0.5));
    const double rate_speed = parameters_.max_steering_rate_radps * 0.65 /
      std::max(std::abs(after - before), 1e-6);
    const double local_limit = std::min(curve_speed, rate_speed);
    target_speed = std::min(target_speed,
      std::sqrt(local_limit * local_limit + 2.0 * kBrakeDeceleration * ahead));
  }
  const double remaining = config_.target_laps * length_m_ - progress_m_;
  // Cross the timing line at <=0.6m/s, then brake to rest. Zero target before
  // crossing would asymptotically stop short and never complete the last lap.
  target_speed = std::min(target_speed,
    std::sqrt(0.6 * 0.6 + 2.0 * kBrakeDeceleration * std::max(0.0, remaining)));
  output_.target_speed_mps = target_speed;
  const double speed_error = target_speed - car.u_mps;
  integral_speed_error_ = std::clamp(integral_speed_error_ + speed_error * kControlPeriod, -0.5, 0.5);
  const double acceleration = std::clamp(1.8 * speed_error + 0.5 * integral_speed_error_, -1.5, 1.0);
  const double resistance = parameters_.rolling_resistance * parameters_.mass_kg *
    parameters_.gravity_mps2 * std::tanh(car.u_mps / parameters_.rolling_smoothing_speed_mps) +
    parameters_.lumped_drag_n_s2_per_m2 * car.u_mps * std::abs(car.u_mps);
  const double wheel_torque = (parameters_.mass_kg * acceleration + resistance) *
    parameters_.effective_tyre_radius_m;
  output_.command = {};
  output_.command.steering_angle_rad = last_steering_;
  if (wheel_torque >= 0) {
    output_.command.rear_axle_torque_nm = std::min(wheel_torque, parameters_.max_rear_axle_torque_nm);
  } else {
    output_.command.friction_brake_ratio = std::clamp(
      -wheel_torque / parameters_.max_brake_torque_nm, 0.0, 1.0);
  }
  return output_;
}
}  // namespace fsai::sim2_adapter
