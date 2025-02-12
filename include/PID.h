#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

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
} PIDController;

void PIDController_Init(PIDController *pid, float dt);
float PIDController_Update(PIDController *pid, float setpoint, float measurement);

#endif
