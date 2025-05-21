#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include "ESP32Servo.h"

typedef struct {
    Servo esc;
    int throttle;
    int frequency;
} Motor;

/**
 * @fn Motor_Init
 * @brief Initializes a motor
 *
 * @param motor The motor object
 * @param pin GPIO pin number connected to the ESC
 * @param min Minimum throttle value [us]
 * @param max Maximum throttle value [us]
 * @param frequency PWM frequency of the ESC
 */
 void Motor_Init(Motor* motor, int pin, int min, int max, int frequency);

/**
 * @fn setMotorThrottle
 * @brief Throttles the motor
 *
 * @param motor The motor object
 */
 void setMotorThrottle(Motor* motor);

#endif
