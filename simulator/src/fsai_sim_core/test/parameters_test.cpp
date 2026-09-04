#include <gtest/gtest.h>

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
