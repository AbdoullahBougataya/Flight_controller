#include "../include/PID.h"

void PIDController_Init(PIDController *pid) {
    // Reset all variables
    integrator = 0.0f;
    prevError = 0.0f;

    differentiator = 0.0f;
    prevMeasurement = 0.0f;

    out = 0.0f;
}

void PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    // Error signal
}
