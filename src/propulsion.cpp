#include "propulsion.hpp"

PropulsionSystem::PropulsionSystem(double fuelMass, double maxThrust)
    : fuelMass_(fuelMass),
      maxThrust_(maxThrust),
      currentThrust_(0.0),
      running_(false),
      consumptionRate_(0.0001) // example constant: 0.0001 kg/(N·s)
{
}

void PropulsionSystem::ignite() {
    if (fuelMass_ <= 0) {
        throw std::runtime_error("Cannot ignite: No fuel available!");
    }
    running_ = true;
}

void PropulsionSystem::shutdown() {
    running_ = false;
    currentThrust_ = 0.0;
}

void PropulsionSystem::setThrottle(double throttle) {
    if (throttle < 0.0 || throttle > 1.0) {
        throw std::invalid_argument("Throttle must be in the range [0, 1].");
    }
    if (!running_) {
        throw std::runtime_error("Engine not running. Ignite the engine first.");
    }
    currentThrust_ = maxThrust_ * throttle;
}

void PropulsionSystem::update(double dt) {
    if (!running_) return;

    double fuelNeeded = currentThrust_ * dt * consumptionRate_;
    if (fuelNeeded >= fuelMass_) {
        // Not enough fuel for the entire dt: adjust achievable thrust.
        currentThrust_ = fuelMass_ / (dt * consumptionRate_);
        fuelMass_ = 0.0;
        running_ = false;  // Fuel is exhausted; engine shuts down.
    } else {
        fuelMass_ -= fuelNeeded;
    }
}

double PropulsionSystem::getFuelMass() const {
    return fuelMass_;
}

double PropulsionSystem::getCurrentThrust() const {
    return currentThrust_;
}

bool PropulsionSystem::isRunning() const {
    return running_;
}
