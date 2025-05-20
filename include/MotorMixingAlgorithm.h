#ifndef MOTOR_MIXING_ALGORITHM_H
#define MOTOR_MIXING_ALGORITHM_H
#include "MotorControl.h"

void MMA(Motor* motor, int* remoteController, float* controlSignals, int motor_count);

#endif
