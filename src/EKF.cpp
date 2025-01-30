#include "../include/EKF.h"

void KalmanRollPitch_Init(kalmanRollPitch *kal, float PInit, float *Q, float *R) {

    // Euler angles reset
    kal->phi_rad = 0.0f; // Roll
    kal->theta_rad = 0.0f; // pitch

    // Initialize the covariance matrix
    kal->P[0] = Pinit; kal->P[1] = 0.0f;
    kal->P[2] = 0.0f;  kal->P[3] = Pinit;

    kal->Q[0] = Q[0]; kal->Q[1] = Q[1];

    kal->R[0] = R[0]; kal->R[1] = R[1]; kal->R[2] = R[2];
}

void KalmanRollPitch_Predict(kalmanRollPitch *kal, float *sensorData, float T) {

    /* Extract measurements and map them to their proper axis */
    float p = -sensorData[0];
    float q = -sensorData[1];
    float r = -sensorData[2];

    /* Predict */
    // Common trigonometry
    float sp = sin(kal->phi_rad);
    float cp = cos(kal->phi_rad);
    float tt = tan(kal->theta_rad);

    /* x(+) = x + T * f(x, u) || x being the angle to be predicted and T being the period and f(x, u) being the state transistion function */
    kal->phi_rad = kal->phi_rad + T * (p + tt * (q * sp + r * cp));
    kal->theta_rad = kal->theta_rad + T * (q * cp - r * sp);

    // Update trigonometry
          sp = sin(kal->phi_rad);         cp = cos(kal->phi_rad);
    float st = sin(kal->theta_rad); float ct = cos(kal->theta_rad); tt = st / ct;

    /* Jacobian of the new state transistion function */
    float A[4] = { tt * (q * cp - r * sp), (r * cp + q * sp) * (tt * tt + 1.0f), -(r * cp + q * sp), 0.0f};

    /* Update the covariance matrix P(+) = P + T * (A*P + P*A' + Q) */
    float Ptmp[4] = {T * (kal->Q[0] + 2.0f * A[0] * kal->P[0] + A[1] * kal->P[1] + A[1] * kal->P[2]), T * (A[0] * kal->)}
}
