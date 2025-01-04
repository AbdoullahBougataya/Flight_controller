#include <Arduino.h>

#define COMP_FLTR_ALPHA 0.05000000000000000000f // Complimentary filter coefficient
#define G_MPS2          9.81000000000000000000f // Gravitational acceleration (g)

/**
  * @fn complimentaryFilter
  * @brief Apply a complimentary filter
  * @param filteredAccelGyro Filtered input signal array in (rad/s)
  * @param phiHat_rad  Output pitch angle of the complimentary filter in (rad)
  * @param thetaHat_rad  Output roll angle of the complimentary filter in (rad)
  * @param dt  Sampling period
  */
void complimentaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, float dt);
