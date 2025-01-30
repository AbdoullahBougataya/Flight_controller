#include "../include/EKF.h"

void KalmanRollPitch_Init(kalmanRollPitch *kal, float PInit, float *Q, float *R) {

    // Euler angles reset
    kal->phi_rad = 0.0f; // Roll
    kal->theta_rad = 0.0f; // pitch

    kal->P[0] = Pinit; kal->P[1] = 0.0f;
    kal->P[2] = 0.0f;  kal->P[3] = Pinit;

    kal->Q[0] = Q[0]; kal->Q[1] = Q[1];

    kal->R[0] = R[0]; kal->R[1] = R[1]; kal->R[2] = R[2];
}

void KalmanRollPitch_Predict(kalmanRollPitch *kal, float *sensorData, float T) {
    float p = sensorData[]
}
