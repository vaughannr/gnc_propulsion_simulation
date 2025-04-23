#include <gtest/gtest.h>
#include <physics.hpp>

TEST(PhysicsTest, ComputeAcceleration) {
    Vector3 thrust{10.0, 1.0, 2.0};
    double mass = 2.0;
    Vector3 acceleration = computeAcceleration(thrust, mass);
    EXPECT_EQ(acceleration.x, 5.0);
    EXPECT_EQ(acceleration.y, 0.5);
    EXPECT_EQ(acceleration.z, 1.0);
}