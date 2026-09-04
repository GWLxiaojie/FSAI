#include <gtest/gtest.h>

#include "fsai_sim_core/types.hpp"

using fsai::sim::Command;
using fsai::sim::Validate;
using fsai::sim::ValidationError;

TEST(Command, RejectsBrakeOutsideUnitInterval) {
  Command command{};
  command.friction_brake_ratio = 1.01;
  EXPECT_THROW(Validate(command), ValidationError);
}

TEST(Command, AcceptsUnitIntervalBrake) {
  Command command{};
  command.friction_brake_ratio = 1.0;
  EXPECT_NO_THROW(Validate(command));
}
