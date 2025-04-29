#include <gtest/gtest.h>
#include <vector>
#include <stdexcept>
#include "physics.hpp"

TEST(Vector3Test, ComputeAcceleration) {
    Vector3 thrust{10.0, 1.0, 2.0};
    double mass = 2.0;
    Vector3 acceleration = computeAcceleration(thrust, mass);
    EXPECT_EQ(acceleration.x, 5.0);
    EXPECT_EQ(acceleration.y, 0.5);
    EXPECT_EQ(acceleration.z, 1.0);
}


// Test that the constructor throws for non-positive mass.
TEST(PhysicsBodyTest, ConstructorRejectsNonPositiveMass) {
    EXPECT_THROW(PhysicsBody body(0), std::invalid_argument);
    EXPECT_THROW(PhysicsBody body(-5), std::invalid_argument);
}

// Test that initial position, velocity, and mass are properly set.
TEST(PhysicsBodyTest, InitialConditions) {
    double mass = 10.0;
    double pos = 5.0;
    double vel = 3.0;
    PhysicsBody body(mass, pos, vel);
    EXPECT_DOUBLE_EQ(body.getMass(), mass);
    EXPECT_DOUBLE_EQ(body.getPosition(), pos);
    EXPECT_DOUBLE_EQ(body.getVelocity(), vel);
    EXPECT_DOUBLE_EQ(body.getAccumulatedForce(), 0.0);
}

// Test updating the body when no force is applied.
// Expect constant velocity motion.
TEST(PhysicsBodyTest, UpdateNoForce) {
    PhysicsBody body(5.0, 0.0, 4.0); // mass 5, initial pos = 0, velocity = 4
    double dt = 2.0;
    // With no force, acceleration = 0, so:
    // New position = 0 + 4 * 2 = 8 and velocity remains 4.
    body.update(dt);
    EXPECT_NEAR(body.getPosition(), 8.0, 1e-6);
    EXPECT_NEAR(body.getVelocity(), 4.0, 1e-6);
}

// Test updating with a constant force applied.
TEST(PhysicsBodyTest, UpdateWithForce) {
    // For a body with mass = 2 and a force of 6:
    // Acceleration = 6 / 2 = 3 m/s^2.
    // For dt = 3 seconds:
    // New velocity = 0 + 3*3 = 9.
    // New position = 0 + 0*3 + 0.5*3*9 = 13.5.
    PhysicsBody body(2.0, 0.0, 0.0);
    body.applyForce(6.0);
    double dt = 3.0;
    body.update(dt);
    EXPECT_NEAR(body.getVelocity(), 9.0, 1e-6);
    EXPECT_NEAR(body.getPosition(), 13.5, 1e-6);
}

// Test that update() throws an exception when a negative time step is provided.
TEST (PhysicsBodyTest, UpdateNegativeDt) {
    PhysicsBody body(5.0, 0.0, 0.0);
    EXPECT_THROW(body.update(-1.0), std::invalid_argument);
}

// Test that after an update, the accumulated force is reset.
TEST(PhysicsBodyTest, ForceResetAfterUpdate) {
    PhysicsBody body(4.0, 10.0, 1.0);
    body.applyForce(8.0);
    body.update(1.0);
    EXPECT_NEAR(body.getAccumulatedForce(), 0.0, 1e-6);
}

// Test updating with a zero time step. The state should remain unchanged,
// although any applied force is cleared.
TEST(PhysicsBodyTest, UpdateZeroDt) {
    PhysicsBody body(5.0, 7.0, 2.0);
    body.applyForce(10.0);
    body.update(0.0);
    EXPECT_NEAR(body.getPosition(), 7.0, 1e-6);
    EXPECT_NEAR(body.getVelocity(), 2.0, 1e-6);
    EXPECT_NEAR(body.getAccumulatedForce(), 0.0, 1e-6);
}

// Test the PhysicsEngine updating multiple bodies in a container.
TEST(PhysicsEngineTest, UpdateBodiesVector) {
    std::vector<PhysicsBody> bodies;
    // Create two bodies:
    // Body 1: mass = 2, pos = 0, vel = 0; apply force 4 -> acceleration = 2.
    // Body 2: mass = 3, pos = 5, vel = 1; apply force 3 -> acceleration = 1.
    bodies.emplace_back(2.0, 0.0, 0.0);
    bodies.emplace_back(3.0, 5.0, 1.0);
    bodies[0].applyForce(4.0);
    bodies[1].applyForce(3.0);

    double dt = 2.0;
    PhysicsEngine engine;
    engine.updateBodies(bodies, dt);

    // For Body 1:
    // acceleration = 4/2 = 2; new velocity = 0 + 2*2 = 4;
    // new position = 0 + 0*2 + 0.5*2*4 = 4.
    EXPECT_NEAR(bodies[0].getVelocity(), 4.0, 1e-6);
    EXPECT_NEAR(bodies[0].getPosition(), 4.0, 1e-6);

    // For Body 2:
    // acceleration = 3/3 = 1; new velocity = 1 + 1*2 = 3;
    // new position = 5 + 1*2 + 0.5*1*4 = 5 + 2 + 2 = 9.
    EXPECT_NEAR(bodies[1].getVelocity(), 3.0, 1e-6);
    EXPECT_NEAR(bodies[1].getPosition(), 9.0, 1e-6);
}

// // Main entry point for running the tests.
// int main(int argc, char **argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
