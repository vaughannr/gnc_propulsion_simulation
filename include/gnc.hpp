#pragma once
#include <stdexcept>

// The state of the system comprising position and velocity.
struct State {
    double position;
    double velocity;
};

// ------------------------------
// Guidance Module
// ------------------------------
// This module computes a desired velocity based on the difference between a
// target position and the current position. A proportional gain scales the error.
class GuidanceModule {
public:
    GuidanceModule(double kp = 0.1, double target = 0.0)
        : kp_(kp), targetPosition_(target) {}

    // Sets the target position.
    void setTarget(double target);

    // Returns the target position.
    double getTarget() const;

    // Returns the desired velocity, calculated as:
    // desired_velocity = kp * (targetPosition - currentPosition)
    double computeDesiredVelocity(const State &currentState) const;

    // Adjust the proportional gain.
    void setGain(double kp);

    // Returns the current gain.
    double getGain() const;

private:
    double kp_;
    double targetPosition_;
};

// ------------------------------
// Control Module
// ------------------------------
// This module computes the control acceleration required to track the desired
// velocity. It uses a simple proportional law based on the difference between
// the desired velocity and the current velocity.
class ControlModule {
public:
    ControlModule(double kp = 1.0)
        : kp_(kp) {}

    // Returns the control acceleration:
    // acceleration = kp * (desiredVelocity - currentVelocity)
    double computeControlAcceleration(double desiredVelocity, const State &currentState) const;

    // Adjust the proportional gain.
    void setGain(double kp);

    // Returns the current gain.
    double getGain() const;

private:
    double kp_;
};

// ------------------------------
// Navigation Module
// ------------------------------
// This module is responsible for integrating the control acceleration to update
// the system state. Using basic kinematics, we update as follows:
//   new velocity = current velocity + acceleration * dt
//   new position = current position + current velocity * dt + 0.5 * acceleration * dt^2
class NavigationModule {
public:
    // Update the state given a control acceleration and a time step dt.
    // Throws an error if dt is negative.
    void updateState(State &state, double acceleration, double dt) const;
};

// ------------------------------
// GNC System
// ------------------------------
// The GNCSystem class combines the three modules into one unit. Given an initial
// state (position and velocity) and a target position, its update() method uses:
//   1. Guidance to compute a desired velocity.
//   2. Control to compute the requisite acceleration.
//   3. Navigation to update the state.
class GNCSystem {
public:
    // Constructor accepts an initial position, velocity, and a target position.
    GNCSystem(double initialPosition, double initialVelocity, double target)
        : state_{initialPosition, initialVelocity},
          guidance_(0.1, target),
          control_(1.0),
          latestControlAcceleration_(0.0)
    {}

    // Optionally adjust the guidance module gain.
    void setGuidanceGain(double kp);

    // Return the current guidance module gain.
    double getGuidanceGain() const;

    // Optionally adjust the control module gain.
    void setControlGain(double kp);

    // Return the current control module gain.
    double getControlGain() const;

    // Update the entire GNC system by a time step dt.
    // This method computes the desired velocity, then acceleration, stores the
    // latest control acceleration, and finally updates the state.
    void update(double dt);

    // Return the current state of the system.
    const State &getCurrentState() const;

    // Return the control acceleration from the most recent update.
    double getLatestControlAcceleration() const;

    // Set a new target position.
    void setTarget(double target);

    double getTarget() const;

private:
    State state_;                     // The current state.
    GuidanceModule guidance_;         // Guidance algorithm instance.
    ControlModule control_;           // Control algorithm instance.
    NavigationModule navigation_;     // Navigation/integration instance.
    double latestControlAcceleration_;// Most recent control output.
};