#ifndef EXTENDED_KALMAN_FLTR_H
#define EXTENDED_KALMAN_FLTR_H

typedef struct {
    // Controller gains
    float Kp;
    float Kd;
    float Ki;

    // Derivative low-pass filter time constant
    float tau;

    // Output limits
    float limMin;
    float limMax;

    // Sample time (in seconds)
    float T;

    // Controller "Memory"
    float integrator;
    float prevError;           // Integrator
    float differentiator;
    float prevMeasurement;     // Differentiator

    // out
    float out;
} extendedKalmanFilter;

#endif
