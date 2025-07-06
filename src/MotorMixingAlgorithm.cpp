#include "../include/MotorMixingAlgorithm.h"

void MMA(Motor* motor, float* controlSignals, int motor_count, int hovering_throttle) {
    /* # Please refer to the equations below for the formula # */
    for (int i = 0; i < motor_count; i++)
    {
        motor[i].throttle = hovering_throttle + controlSignals[2] + pow(-1, i + 1) * controlSignals[3] + pow(-1, floor(i / 2)) * controlSignals[1] + pow(-1, floor((i+3)/2)) * controlSignals[0];
    }
    /*          |                               |                           |                  |                     |                     |                       |                      |
                V                               V                           V                  V                     V                     V                       V                      V
      Throttle[left-forward] =              Throttle                        -                 Yaw                    +                   Pitch                     -                    Roll
      Throttle[right-forward] =             Throttle                        +                 Yaw                    +                   Pitch                     +                    Roll
      Throttle[right-backward] =            Throttle                        -                 Yaw                    -                   Pitch                     +                    Roll
      Throttle[left-backward] =             Throttle                        +                 Yaw                    -                   Pitch                     -                    Roll
    */
}
