#ifndef MOTOR_MIXING_ALGORITHM_H
#define MOTOR_MIXING_ALGORITHM_H
#include "MotorControl.h"

/**
  * @fn MMA
  * @brief Motor Mixing Algorithm: Algorithm that determines the throttle for each motor
  * @param motor The motors objects
  * @param controlSignals The Control signal array output from the controllers
  * @param motor_count The number of motors used
  * @param throttle the throttle
  */
void MMA(Motor* motor, float* controlSignals, int motor_count, int throttle);

#endif
