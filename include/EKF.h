#ifndef EXTENDED_KALMAN_FLTR_H
#define EXTENDED_KALMAN_FLTR_H

typedef struct {

    // out
    float out;
} extendedKalmanFilter;

void PIDController_Init(extendedKalmanFilter *ekf, float dt);
void PIDController_Init(extendedKalmanFilter *ekf);

#endif
