#include <Arduino.h>

#define COMP_FLTR_ALPHA 0.50000000000000000000f // Complimentary filter coefficient
#define EMA_ALPHA       0.01000000000000000000f // EMA filter coefficient
#define G_MPS2          9.81000000000000000000f // Gravitational acceleration (g)

/**
* @fn complimentaryFilter
* @brief Apply a complimentary filter
* @param filteredAccelGyro Filtered input signal array
* @param phiHat_rad  Output pitch of the complimentary filter
* @param thetaHat_rad  Output roll of the complimentary filter
* @param dt  Sampling period
*/
void complimentaryFilter(float* filteredAccelGyro, float phiHat_rad, float thetaHat_rad, float dt);
