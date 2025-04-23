#include <gtest/gtest.h>
#include "propulsion.hpp"

TEST(PIDControllerTest, ComputeCorrection) {
    PIDController pid(1.0, 0.1, 0.05);
    EXPECT_NEAR(pid.computeCorrection(95.0), 6.0, 0.1);
}