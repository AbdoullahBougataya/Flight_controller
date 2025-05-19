#include "../include/MotorControl.h"

void Motor_Init(Motor* s, int pin, int min, int max, int frequency) {
    // Set the frequency of the ESC
    s->esc.setPeriodHertz(frequency);

    // Attach the motor to the ESC object
    s->esc.attach(pin, min, max);

    // Reset ESC's throttle
    s->esc.write(0);

    // Delay the initialization for ESC start up sequence
    delay(5000);
}

void setMotorThrottle(Motor* s, int value) {
    value = fmin(fmax(value, 0), 1000);

    // Throttle the motor speed [0, 1000]
    s->esc.write(value + 1000);
}
