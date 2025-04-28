#pragma once
#include <cmath>

struct Vector3 {double x, y, z;};
Vector3 computeAcceleration(const Vector3& force, double mass) {
    return {force.x / mass, force.y / mass, force.z / mass};
}
