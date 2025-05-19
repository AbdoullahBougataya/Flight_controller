#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "ESP32Servo.h"

typedef struct {
    Servo esc;
} Motor;

/**
 * @brief Initializes a motor
 * 
 * @param s Pointer to the Motor structure
 * @param pin GPIO pin number connected to the ESC
 * @param min Minimum throttle value [us]
 * @param max Maximum throttle value [us]
 * @param frequency PWM frequency of the ESC
 */
 void Motor_Init(Motor* s, int pin, int min, int max, int frequency);

/**
 * @brief Sets the throttle value for the motor
 * 
 * @param s Pointer to the Motor structure
 * @param value Throttle value to set [0, 1000]
 */
 void setMotorThrottle(Motor* s, int value);

#endif
