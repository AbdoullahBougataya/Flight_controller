#ifndef CALIBRATIONS_H
#define CALIBRATIONS_H

#include <Arduino.h>
#include <stdint.h>
#include "BMI160.h"

void CalibrateGyroscope(int SC, float *offset);

#endif
