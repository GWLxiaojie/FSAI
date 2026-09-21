#include <gtest/gtest.h>
#include <limits>

#include "fsai_sim_core/parameters.hpp"

using fsai::sim::Derive;
using fsai::sim::ParameterHash;
using fsai::sim::ReferenceBicycleParameters;
using fsai::sim::ValidateParameters;
using fsai::sim::ValidationError;
using fsai::sim::VehicleParameters;

TEST(VehicleParameters, DerivesAxleDistances) {
  VehicleParameters p = ReferenceBicycleParameters();
  p.wheelbase_m = 1.6;
  p.front_static_load_fraction = 0.45;
  EXPECT_DOUBLE_EQ(Derive(p).cg_to_rear_axle_m, 0.72);
  EXPECT_DOUBLE_EQ(Derive(p).cg_to_front_axle_m, 0.88);
}

TEST(VehicleParameters, HashIsStable) {
  EXPECT_EQ(
    ParameterHash(ReferenceBicycleParameters()),
    ParameterHash(ReferenceBicycleParameters()));
}

TEST(VehicleParameters, HashChangesWithMass) {
  auto first = ReferenceBicycleParameters();
  auto second = first;
  second.mass_kg += 1.0;
  EXPECT_NE(ParameterHash(first), ParameterHash(second));
}

TEST(VehicleParameters, RejectsNonPositiveMass) {
  auto parameters = ReferenceBicycleParameters();
  parameters.mass_kg = -1.0;
  EXPECT_THROW(ValidateParameters(parameters), ValidationError);
}

TEST(VehicleParameters, RejectsEveryNonFiniteScalarAndInvalidDomains) {
  using P=VehicleParameters;
  double P::* fields[]={&P::mass_kg,&P::gravity_mps2,&P::yaw_inertia_kgm2,
    &P::wheelbase_m,&P::front_static_load_fraction,&P::front_track_m,&P::rear_track_m,
    &P::effective_tyre_radius_m,&P::rolling_resistance,&P::max_steering_angle_rad,
    &P::max_steering_rate_radps,&P::max_front_axle_torque_nm,&P::max_rear_axle_torque_nm,
    &P::max_brake_torque_nm,&P::pacejka_B_front,&P::pacejka_C_front,&P::pacejka_D_front,
    &P::pacejka_E_front,&P::pacejka_B_rear,&P::pacejka_C_rear,&P::pacejka_D_rear,
    &P::pacejka_E_rear,&P::air_density_kgpm3,&P::lumped_drag_n_s2_per_m2,
    &P::lumped_downforce_n_s2_per_m2,&P::timeout_brake_ratio,&P::ebs_brake_ratio,
    &P::static_speed_threshold_mps,&P::rolling_smoothing_speed_mps,&P::hold_release_force_n};
  for(auto field:fields) for(double invalid:{std::numeric_limits<double>::quiet_NaN(),
      std::numeric_limits<double>::infinity()}) {
    auto p=ReferenceBicycleParameters(); p.*field=invalid;
    EXPECT_THROW(ValidateParameters(p),ValidationError);
  }
  auto p=ReferenceBicycleParameters(); p.rolling_smoothing_speed_mps=0;
  EXPECT_THROW(ValidateParameters(p),ValidationError);
  p=ReferenceBicycleParameters();p.max_rear_axle_torque_nm=-1;
  EXPECT_THROW(ValidateParameters(p),ValidationError);
}
