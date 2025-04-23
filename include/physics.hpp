#pragma once
#include <cmath>

struct Vector3 {double x, y, z;};
Vector3 computeAcceleration(const Vector3& force, double mass);
