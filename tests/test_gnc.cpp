#include <gtest/gtest.h>
#include "gnc.hpp"

// ------------------------------
// Unit tests for individual modules
// ------------------------------

//
TEST(GuidanceModuleTest, SetTarget) {
    GuidanceModule guidance;
    guidance.setTarget(50.0);
    EXPECT_DOUBLE_EQ(guidance.getTarget(), 50.0);
    guidance.setTarget(100.0);
    EXPECT_DOUBLE_EQ(guidance.getTarget(), 100.0);
}

//
TEST(GuidanceModuleTest, SetGain) {
    GuidanceModule guidance;
    guidance.setGain(0.5);
    EXPECT_DOUBLE_EQ(guidance.getGain(), 0.5);
    guidance.setGain(1.0);
    EXPECT_DOUBLE_EQ(guidance.getGain(), 1.0);
}

// Test the GuidanceModule: with kp = 0.2 and target = 100.0 and a current position of 50,
// the computed desired velocity should be 0.2*(100-50) = 10.0.
TEST(GuidanceModuleTest, ComputeDesiredVelocity) {
    GuidanceModule guidance(0.2, 100.0);
    State currentState {50.0, 0.0};
    double desiredVelocity = guidance.computeDesiredVelocity(currentState);
    EXPECT_DOUBLE_EQ(desiredVelocity, 10.0);
}

//
TEST(ControlModuleTest, SetGain) {
    ControlModule control;
    control.setGain(1.5);
    EXPECT_DOUBLE_EQ(control.getGain(), 1.5);
    control.setGain(2.0);
    EXPECT_DOUBLE_EQ(control.getGain(), 2.0);
}

// Test the ControlModule: with kp = 2.0, a desired velocity of 15.0, and a current velocity of 5.0,
// the control acceleration should be 2.0*(15-5) = 20.0.
TEST(ControlModuleTest, ComputeControlAcceleration) {
    ControlModule control(2.0);
    State currentState {0.0, 5.0};
    double desiredVelocity = 15.0;
    double acceleration = control.computeControlAcceleration(desiredVelocity, currentState);
    EXPECT_DOUBLE_EQ(acceleration, 20.0);
}

// Test the NavigationModule: ensure that for a positive dt, the state is updated using kinematic equations.
// For an initial state of {position = 0, velocity = 0}, an acceleration of 10, and dt = 2,
// we expect: position = 0 + 0*2 + 0.5*10*(4)=20 and velocity = 0 + 10*2 = 20.
TEST(NavigationModuleTest, UpdateStatePositiveDt) {
    NavigationModule nav;
    State state {0.0, 0.0};
    double acceleration = 10.0;
    double dt = 2.0;
    nav.updateState(state, acceleration, dt);
    EXPECT_NEAR(state.position, 20.0, 1e-6);
    EXPECT_NEAR(state.velocity, 20.0, 1e-6);
}

// Test that the NavigationModule throws an exception when dt is negative.
TEST(NavigationModuleTest, NegativeDtThrows) {
    NavigationModule nav;
    State state {0.0, 0.0};
    EXPECT_THROW(nav.updateState(state, 10.0, -1.0), std::invalid_argument);
}

// ------------------------------
// Unit tests for the overall GNCSystem
// ------------------------------

TEST(GNCSystemTest, SetGains) {
    GNCSystem gnc(0.0, 0.0, 100.0);
    EXPECT_DOUBLE_EQ(gnc.getGuidanceGain(), 0.1);
    EXPECT_DOUBLE_EQ(gnc.getControlGain(), 1.0);
    gnc.setGuidanceGain(0.5);
    EXPECT_DOUBLE_EQ(gnc.getGuidanceGain(), 0.5);
    gnc.setControlGain(2.0);
    EXPECT_DOUBLE_EQ(gnc.getControlGain(), 2.0);
}

TEST(GNCSystemTest, SetTarget) {
    GNCSystem gnc(0.0, 0.0, 100.0);
    EXPECT_DOUBLE_EQ(gnc.getTarget(), 100.0);
    gnc.setTarget(200.0);
    EXPECT_DOUBLE_EQ(gnc.getTarget(), 200.0);
}

// Test a single update step for the GNCSystem.
// With initial state position = 0, velocity = 0 and target = 100:
//   Guidance: desired velocity = 0.1*(100 - 0) = 10
//   Control: acceleration = 1.0*(10 - 0) = 10
//   Navigation: new position = 0 + 0*1 + 0.5*10*1^2 = 5, new velocity = 10.
TEST(GNCSystemTest, SingleUpdate) {
    GNCSystem gnc(0.0, 0.0, 100.0);
    double dt = 1.0;
    gnc.update(dt);
    State state = gnc.getCurrentState();
    EXPECT_NEAR(state.position, 5.0, 1e-6);
    EXPECT_NEAR(state.velocity, 10.0, 1e-6);
    EXPECT_NEAR(gnc.getLatestControlAcceleration(), 10.0, 1e-6);
}

// If dt = 0, then the state should not change even though the control acceleration is computed.
TEST(GNCSystemTest, ZeroDtUpdate) {
    GNCSystem gnc(10.0, 2.0, 50.0);
    gnc.update(0.0);
    State state = gnc.getCurrentState();
    EXPECT_NEAR(state.position, 10.0, 1e-6);
    EXPECT_NEAR(state.velocity, 2.0, 1e-6);
}

// Test the overall convergence of the GNCSystem toward the target position.
// We simulate a sequence of updates with a small time step, and after many iterations,
// the position should be near the target and the velocity near zero.
TEST(GNCSystemTest, ConvergenceToTarget) {
    GNCSystem gnc(0.0, 0.0, 100.0);
    double dt = 0.1;
    for (int i = 0; i < 3000; ++i) {
        gnc.update(dt);
    }
    State state = gnc.getCurrentState();
    // Because of the proportional laws used, the system should be near 100 with negligible velocity.
    EXPECT_NEAR(state.position, 100.0, 1e-1);
    EXPECT_NEAR(state.velocity, 0.0, 1e-1);
}

// // Main entry point for running all tests.
// int main(int argc, char **argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }
