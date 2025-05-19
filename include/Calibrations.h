#ifndef CALIBRATIONS_H
#define CALIBRATIONS_H

#include <Arduino.h>
#include <stdint.h>
#include "BMI160.h"

#define LED_PIN             38      // The pin that commands the LED
#define LED_BRIGHTNESS     200      // The brightness of the onboard LED

void CalibrateGyroscope(int SC, float *offset);

#endif
