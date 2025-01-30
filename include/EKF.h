#ifndef KALMAN_ROLL_PITCH_H
#define KALMAN_ROLL_PITCH_H

#include <math.h>

#define G_MPS2                            9.81000000000000000000f  // Gravitational acceleration (g)

typedef struct {
    float phi_rad;
    float theta_rad;

    float P[4];
    float Q[2];
    float R[3];
    
} kalmanRollPitch;

void KalmanRollPitch_Init(kalmanRollPitch *kal, float PInit, float *Q, float *R);
void KalmanRollPitch_Predict(kalmanRollPitch *kal, float *sensorData, float T);
void KalmanRollPitch_Update(kalmanRollPitch *kal, float *sensorData);

#endif
