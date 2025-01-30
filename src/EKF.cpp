#include "../include/EKF.h"

void KalmanRollPitch_Init(kalmanRollPitch *kal, float PInit, float *Q, float *R) {

    kal->phi_rad = 0.0f;
    kal->theta_rad = 0.0f;

    kal->P[0] = Pinit; kal->P[1] = 0.0f;
    kal->P[2] = 0.0f;  kal->P[3] = Pinit;

}
