#ifndef MOTOR_MIXING_ALGORITHM_H
#define MOTOR_MIXING_ALGORITHM_H
#include "MotorControl.h"

/**
  * @fn MMA
  * @brief Motor Mixing Algorithm: Algorithm that determines the throttle for each motor
  * @param motor The motors objects
  * @param remoteController The remote controller command from 0 to a 1000
  * @param controlSignals The Control signal array output from the controllers
  * @param motor_count The number of motors used
  */
void MMA(Motor* motor, int* remoteController, float* controlSignals, int motor_count);

#endif
