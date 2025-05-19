#include "../include/PID.h"

void PIDController_Init(PIDController *pid, float dt) {
    // Set the sampling period
    pid->T = dt;

    // Reset all variables
    pid->integrator = 0.0f;
    pid->prevError = 0.0f;

    pid->differentiator = 0.0f;
    pid->prevMeasurement = 0.0f;

    pid->out = 0.0f;
}

float PIDController_Update(PIDController *pid, float setpoint, float measurement) {
    // Error
    float error = setpoint - measurement;

    // Proportional
    float proportional = pid->Kp * error;

    // Integral
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (error + pid->prevError);

    // Anti-wind-up via dynamic integrator clamping
    float limMinInt, limMaxInt;

    if (pid->limMax > proportional) {
        limMaxInt = pid->limMax - proportional;
    } else {
        limMaxInt = 0.0f;
    }

    if (pid->limMin < proportional) {
        limMinInt = pid->limMin - proportional;
    } else {
        limMinInt = 0.0f;
    }

    // Clamp integrator
    if (pid->integrator > limMaxInt) {
        pid->integrator = limMaxInt;
    }
    else if (pid->integrator < limMinInt) {
        pid->integrator = limMinInt;
    }

    // Derivative
    pid->differentiator = - (2.0f * pid->Kd * (measurement - pid->prevMeasurement)
                          + (2.0f * pid->tau - pid->T) * pid->differentiator)
                          / (2.0f * pid->tau + pid->T);

    // Computing the output and applying the limits
    pid->out = proportional + pid->integrator + pid->differentiator;

    if (pid->out > pid->limMax) {
        pid->out = pid->limMax;
    }
    else if (pid->out < pid->limMin) {
        pid->out = pid->limMin;
    }

    // Store the error and the measurement
    pid->prevError = error;
    pid->prevMeasurement = measurement;

    // Return the ouput
    return (pid->out);
}
