#include "../include/MotorMixingAlgorithm.h"

void MMA(Motor* motor, int* remoteController, float* controlSignals, int motor_count) {
    for (int i = 0; i < motor_count; i++)
    {
        motor[i].throttle = remoteController[2] + controlSignals[2] + pow(-1, i + 1) * controlSignals[3] + pow(-1, floor(i / 2)) * controlSignals[1] + pow(-1, floor((i+3)/2)) * controlSignals[0];
    }
    /**/
}
