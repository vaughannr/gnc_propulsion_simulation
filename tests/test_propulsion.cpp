#include "propulsion.hpp"
#include <gtest/gtest.h>

// Test ignition when sufficient fuel is available.
TEST(PropulsionSystemTest, IgniteWithFuel) {
  PropulsionSystem engine(10.0, 5000.0);
  EXPECT_NO_THROW(engine.ignite());
  EXPECT_TRUE(engine.isRunning());

  // After shutting down, the thrust should be zero and the engine not running.
  engine.shutdown();
  EXPECT_DOUBLE_EQ(engine.getCurrentThrust(), 0.0);
  EXPECT_FALSE(engine.isRunning());
}

// Test that ignition throws an error when fuel is zero.
TEST(PropulsionSystemTest, IgniteWithoutFuel) {
  PropulsionSystem engine(0.0, 5000.0);
  EXPECT_THROW(engine.ignite(), std::runtime_error);
}

// Test setting the throttle within the valid range.
TEST(PropulsionSystemTest, SetThrottleValid) {
  PropulsionSystem engine(10.0, 5000.0);
  engine.ignite();
  EXPECT_NO_THROW(engine.setThrottle(0.5));
  EXPECT_DOUBLE_EQ(engine.getCurrentThrust(), 2500.0); // 0.5 * 5000
}

// Test that setting throttle outside of [0, 1] throws an exception.
TEST(PropulsionSystemTest, SetThrottleInvalid) {
  PropulsionSystem engine(10.0, 5000.0);
  engine.ignite();
  EXPECT_THROW(engine.setThrottle(1.5), std::invalid_argument);
  EXPECT_THROW(engine.setThrottle(-0.2), std::invalid_argument);
  engine.shutdown();
  EXPECT_THROW(engine.setThrottle(0.5),
               std::runtime_error); // Engine not running.
}

// Test fuel consumption over a time interval.
TEST(PropulsionSystemTest, UpdateFuelConsumption) {
  PropulsionSystem engine(10.0, 5000.0);
  engine.ignite();
  engine.setThrottle(0.5); // Expected thrust = 2500 N.

  double initialFuel = engine.getFuelMass();
  double dt = 1.0; // 1 second timestep.
  engine.update(dt);

  // Expected fuel consumption = currentThrust * dt * consumptionRate.
  double expectedConsumption = 2500.0 * dt * 0.0001;
  EXPECT_NEAR(engine.getFuelMass(), initialFuel - expectedConsumption, 1e-6);
  EXPECT_TRUE(engine.isRunning());
}

// Test behavior when fuel is exhausted in an update.
TEST(PropulsionSystemTest, FuelExhaustion) {
  // Choose a fuel mass that is too low to sustain the selected throttle for
  // dt=1 s.
  PropulsionSystem engine(0.1, 5000.0);
  engine.ignite();
  engine.setThrottle(0.8); // Expected thrust = 4000 N.

  // For dt = 1 second, fuel needed = 4000 * 1 * 0.0001 = 0.4 kg,
  // but we only have 0.1 kg. Thus, update() should adjust the thrust based on
  // available fuel.
  engine.update(1.0);

  // Fuel should now be 0, and the engine should be shut down.
  EXPECT_NEAR(engine.getFuelMass(), 0.0, 1e-6);
  EXPECT_FALSE(engine.isRunning());

  // The achievable thrust is computed as: remainingFuel / (dt *
  // consumptionRate)
  double achievableThrust = 0.1 / (1.0 * 0.0001); // 1000 N.
  EXPECT_NEAR(engine.getCurrentThrust(), achievableThrust, 1e-6);
}

// // Main entry point for gtest.
// int main(int argc, char **argv) {
//     ::testing::InitGoogleTest(&argc, argv);
//     return RUN_ALL_TESTS();
// }

// class ThrusterEfficiencyTest : public
// ::testing::TestWithParam<std::tuple<double, double, double>> {};

// TEST_P(ThrusterEfficiencyTest, ComputeThrust) {
//     auto [maxThrust, efficiency, throttle] = GetParam();

//     Thruster thruster{maxThrust, efficiency};
//     double expectedThrust = maxThrust * efficiency * throttle;
//     double actualThrust = thruster.computeThrust(throttle);
//     EXPECT_DOUBLE_EQ(actualThrust, expectedThrust);
// }

// INSTANTIATE_TEST_CASE_P(ThrusterTests, ThrusterEfficiencyTest,
// testing::Values(
//     std::make_tuple(1000.0, 0.5, 0.5),
//     std::make_tuple(2000.0, 0.7, 0.8),
//     std::make_tuple(1500.0, 0.9, 1.0),
//     std::make_tuple(0.0, 1.0, 1.0)
// ));