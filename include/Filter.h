#include <Arduino.h>

#define COMP_FLTR_ALPHA 0.25000000000000000000f // Complimentary filter coefficient
#define EMA_ALPHA       0.65000000000000000000f // EMA filter coefficient
#define G_MPS2          9.81000000000000000000f // Gravitational acceleration (g)

/**
  * @fn complimentaryFilter
  * @brief Apply a complimentary filter
  * @param filteredAccelGyro Filtered input signal array in (m/s²) and (rad/s)
  * @param phiHat_rad  Output pitch angle of the complimentary filter in (rad)
  * @param thetaHat_rad  Output roll angle of the complimentary filter in (rad)
  * @param dt  Sampling period
  */
void complimentaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, float dt);

/**
  * @fn EMAFilter
  * @brief Apply an EMA Filter to the data
  * @param rawAccelGyro Raw data in (m/s²) and (rad/s)
  * @param filteredAccelGyro  EMA Filtered signal in (m/s²) and (rad/s)
  */
void EMAFilter(float* rawAccelGyro, float* filteredAccelGyro);
