#pragma once

struct PIDController {
    double kp, ki, kd;
    double target;
    double computeCorrection(double current);
};