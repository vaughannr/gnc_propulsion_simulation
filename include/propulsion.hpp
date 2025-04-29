#pragma once

#include <stdexcept>

class PropulsionSystem {
public:
  // Constructor: fuelMass in kilograms, maxThrust in Newtons.
  PropulsionSystem(double fuelMass, double maxThrust);

  // Ignites the propulsion system.
  // Throws a runtime_error if there is no fuel available.
  void ignite();

  // Shuts down the system (resets thrust to zero).
  void shutdown();

  // Sets the throttle level (from 0.0 to 1.0)
  // Throws an invalid_argument if throttle is not within [0, 1].
  // Throws a runtime_error if the engine is not running.
  void setThrottle(double throttle);

  // Simulates the passage of time by dt seconds,
  // consuming fuel based on current thrust and a fixed consumption rate.
  // If the fuel needed exceeds what’s available, the engine’s thrust is reduced
  // to reflect the achievable thrust, fuel is set to zero, and the system shuts
  // down.
  void update(double dt);

  // Getter for the remaining fuel mass.
  double getFuelMass() const;

  // Getter for the current thrust.
  double getCurrentThrust() const;

  // Returns true if the propulsion system is running.
  bool isRunning() const;

private:
  double fuelMass_;
  double maxThrust_;
  double currentThrust_;
  bool running_;
  const double consumptionRate_; // fuel consumption constant (kg/(N·s))
};