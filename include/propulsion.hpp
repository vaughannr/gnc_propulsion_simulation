#pragma once

struct Thruster {
    double maxThrust;   // N
    double fuelRate;    // kg/s

    double computeThrust(double throttle);
};