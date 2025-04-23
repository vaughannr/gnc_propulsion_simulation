#include <gtest/gtest.h>
#include "propulsion.hpp"

TEST(ThrusterTest, ComputeThrust) {
    Thruster thruster{100.0, 0.5}
    EXPECT_DOUBLE_EQ(thruster.computeThrust(0.5), 50.0);