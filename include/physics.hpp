#pragma once
#include <cmath>
#include <stdexcept>

struct Vector3 {double x, y, z;};
Vector3 computeAcceleration(const Vector3& force, double mass) {
    return {force.x / mass, force.y / mass, force.z / mass};
}

// -----------------------------------------------------------------------------
// PhysicsBody Class
// -----------------------------------------------------------------------------
// This class simulates a one-dimensional physics object.
// - The object has a mass (required to be positive), a position, and a velocity.
// - External forces can be applied (accumulated) and then integrated over a
//   timestep. During integration, we assume that the force is constant over dt.
// - The state is updated using the equations:
//       acceleration = F / mass
//       new position = position + velocity * dt + 0.5 * acceleration * dt^2
//       new velocity = velocity + acceleration * dt
// - After an update, the accumulated force is automatically reset.
class PhysicsBody {
public:
    // Constructor requires a positive mass; initial position and velocity are optional.
    PhysicsBody(double mass, double initialPosition = 0.0, double initialVelocity = 0.0)
        : mass_(mass),
          position_(initialPosition),
          velocity_(initialVelocity),
          accumulatedForce_(0.0)
    {
        if (mass_ <= 0.0) {
            throw std::invalid_argument("Mass must be positive.");
        }
    }

    // Apply an external force to the body.
    // Forces are summed until update() is called.
    void applyForce(double force) {
        accumulatedForce_ += force;
    }

    // Resets the accumulated force to zero.
    void resetForce() {
        accumulatedForce_ = 0.0;
    }

    // Integrate the body state over time dt.
    // Throws an exception if dt is negative.
    void update(double dt) {
        if (dt < 0.0) {
            throw std::invalid_argument("Time step dt must be non-negative.");
        }
        double acceleration = accumulatedForce_ / mass_;
        position_ += velocity_ * dt + 0.5 * acceleration * dt * dt;
        velocity_ += acceleration * dt;
        // Reset the force accumulator after applying the integration.
        resetForce();
    }

    // Getters for the physics state.
    double getMass() const { return mass_; }
    double getPosition() const { return position_; }
    double getVelocity() const { return velocity_; }
    double getAccumulatedForce() const { return accumulatedForce_; }

private:
    const double mass_;
    double position_;
    double velocity_;
    double accumulatedForce_;
};

// -----------------------------------------------------------------------------
// PhysicsEngine Class
// -----------------------------------------------------------------------------
// This simple physics engine provides an interface for updating multiple
// physics bodies with a single time step. The engine is template-based so that
// it can work with any standard container (e.g., std::vector) of PhysicsBody objects.
class PhysicsEngine {
public:
    PhysicsEngine() = default;

    template <typename Container>
    void updateBodies(Container &bodies, double dt) const {
        for (auto &body : bodies) {
            body.update(dt);
        }
    }
};

