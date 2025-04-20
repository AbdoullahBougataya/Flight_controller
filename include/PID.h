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

/**
  * @fn PIDController_Init
  * @brief Initalize the PID controller
  * @param pid The PID object
  */
void PIDController_Init(PIDController *pid);

/**
  * @fn RCFilter_Init
  * @brief Update the controller on every iteration
  * @param pid The PID object
  * @param setpoint The setpoint of the PID controller
  * @param measurement The measurement from the sensor
  * @param dt The sampling period in [s]
  */
float PIDController_Update(PIDController *pid, float setpoint, float measurement, float dt);

#endif
