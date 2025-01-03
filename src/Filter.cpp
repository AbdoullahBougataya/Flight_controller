#include"../include/Filter.h"
#include <math.h>

// Low-pass EMA Filter
void EMAFilter(float* rawAccelGyro, float* filteredAccelGyro) {
  for (uint8_t i = 0; i < 6; i++) {
    // Default to zero in low amplitude noise
    if (rawAccelGyro[i] <= 0.3 && rawAccelGyro[i] >= -0.2) {
      rawAccelGyro[i] = 0;
    }
    filteredAccelGyro[i] = EMA_ALPHA * rawAccelGyro[i] + (1 - EMA_ALPHA) * filteredAccelGyro[i];
  }
}

// Complimentary filter
void complimentaryFilter(float* filteredAccelGyro, float phiHat_rad, float thetaHat_rad, float dt) {
  // Using gravity to estimate the Euler angles
  float phiHat_acc_rad = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]);                 // Roll estimate
  float thetaHat_acc_rad = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f));  // Pitch estimate

  // Using gyroscope to estimate the euler rates
  float phiDot_rps   = (sinf(phiHat_rad) * filteredAccelGyro[1] + cosf(phiHat_rad) * filteredAccelGyro[2]) * tanf(thetaHat_rad) + filteredAccelGyro[0];  // Roll rate (rad/s)
  float thetaDot_rps =  cosf(phiHat_rad) * filteredAccelGyro[1] - sinf(phiHat_rad) * filteredAccelGyro[2];                                               // Pitch rate (rad/s)

  // Complementary filter implementation
  phiHat_rad = COMP_FLTR_ALPHA * phiHat_acc_rad + (1.0f - COMP_FLTR_ALPHA) * (phiHat_rad + dt * phiDot_rps);          // Roll estimate
  thetaHat_rad = COMP_FLTR_ALPHA * thetaHat_acc_rad + (1.0f - COMP_FLTR_ALPHA) * (thetaHat_rad + dt * thetaDot_rps);  // Pitch estimate
}
