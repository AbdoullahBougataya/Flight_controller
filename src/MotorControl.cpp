#include "../include/MotorControl.h"

void Motor_Init(Motor* motor, int pin, int min, int max, int frequency) {
    // Set the frequency of the ESC
    motor->frequency = frequency;
    motor->esc.setPeriodHertz(frequency);

    // Attach the motor to the ESC object
    motor->esc.attach(pin, min, max);

    // Reset ESC's throttle
    motor->esc.write(0);
}

void setMotorThrottle(Motor* motor) {
    // Ensure that the motor throttle is between 0 and 1000
    motor->throttle = fmin(fmax(motor->throttle, 0), 1000);

    // Throttle the motor speed [0, 1000]
    motor->esc.write(motor->throttle + 1000);
}
