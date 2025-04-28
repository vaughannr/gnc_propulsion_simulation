#include "propulsion.hpp"

PropulsionSystem::PropulsionSystem(double fuelMass, double maxThrust)
    : fuelMass(fuelMass),
      maxThrust(maxThrust),
      currentThrust(0.0),
      running(false),
      consumptionRate(0.0001) // example constant: 0.0001 kg/(N·s)
{
}

void PropulsionSystem::ignite() {
    if (fuelMass <= 0) {
        throw std::runtime_error("Cannot ignite: No fuel available!");
    }
    running = true;
}

void PropulsionSystem::shutdown() {
    running = false;
    currentThrust = 0.0;
}

void PropulsionSystem::setThrottle(double throttle) {
    if (throttle < 0.0 || throttle > 1.0) {
        throw std::invalid_argument("Throttle must be in the range [0, 1].");
    }
    if (!running) {
        throw std::runtime_error("Engine not running. Ignite the engine first.");
    }
    currentThrust = maxThrust * throttle;
}

void PropulsionSystem::update(double dt) {
    if (!running) return;

    double fuelNeeded = currentThrust * dt * consumptionRate;
    if (fuelNeeded >= fuelMass) {
        // Not enough fuel for the entire dt: adjust achievable thrust.
        currentThrust = fuelMass / (dt * consumptionRate);
        fuelMass = 0.0;
        running = false;  // Fuel is exhausted; engine shuts down.
    } else {
        fuelMass -= fuelNeeded;
    }
}

double PropulsionSystem::getFuelMass() const {
    return fuelMass;
}

double PropulsionSystem::getCurrentThrust() const {
    return currentThrust;
}

bool PropulsionSystem::isRunning() const {
    return running;
}
