#include "../include/MotorControl.h"

void Motor_Init(Motor* s, int pin, int min, int max, int frequency) {
    // Set the frequency of the ESC
    s->esc.setPeriodHertz(frequency);

    // Attach the motor to the ESC object
    s->esc.attach(pin, min, max);

    // Reset ESC's throttle
    s->esc.write(0);
}

void setMotorThrottle(Motor* s, int value) {
    value = fmin(fmax(value, 0), 1000);
    
    // Set the throttle
    s->throttle = value + 1000;

    // Throttle the motor speed [0, 1000]
    s->esc.write(s->throttle);
}
