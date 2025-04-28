#include "gnc.hpp"
#include <stdexcept>

// --- GuidanceModule Definitions ---
void GuidanceModule::setTarget(double target) {
    targetPosition_ = target;
}

double GuidanceModule::computeDesiredVelocity(const State &currentState) const {
    return kp_ * (targetPosition_ - currentState.position);
}

void GuidanceModule::setGain(double kp) {
    kp_ = kp;
}

// --- ControlModule Definitions ---
double ControlModule::computeControlAcceleration(double desiredVelocity, const State &currentState) const {
    return kp_ * (desiredVelocity - currentState.velocity);
}

void ControlModule::setGain(double kp) {
    kp_ = kp;
}

// --- NavigationModule Definitions ---
void NavigationModule::updateState(State &state, double acceleration, double dt) const {
    if (dt < 0) {
        throw std::invalid_argument("dt cannot be negative");
    }
    state.position += state.velocity * dt + 0.5 * acceleration * dt * dt;
    state.velocity += acceleration * dt;
}

// --- GNCSystem Definitions ---
void GNCSystem::setGuidanceGain(double kp) {
    guidance_.setGain(kp);
}

void GNCSystem::setControlGain(double kp) {
    control_.setGain(kp);
}

void GNCSystem::update(double dt) {
    double desiredVelocity = guidance_.computeDesiredVelocity(state_);
    double controlAcceleration = control_.computeControlAcceleration(desiredVelocity, state_);
    latestControlAcceleration_ = controlAcceleration;
    navigation_.updateState(state_, controlAcceleration, dt);
}

const State &GNCSystem::getCurrentState() const {
    return state_;
}

double GNCSystem::getLatestControlAcceleration() const {
    return latestControlAcceleration_;
}

void GNCSystem::setTarget(double target) {
    guidance_.setTarget(target);
}
