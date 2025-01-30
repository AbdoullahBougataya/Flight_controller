#ifndef EXTENDED_KALMAN_FLTR_H
#define EXTENDED_KALMAN_FLTR_H

typedef struct {

    // out
    float out;
} extendedKalmanFilter;

void PIDController_Init(extendedKalmanFilter *fltr, float dt);

#endif
