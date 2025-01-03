#include <Arduino.h>

#define COMP_FLTR_ALPHA 0.50000000000000000000f // Complimentary filter coefficient
#define EMA_ALPHA       0.01000000000000000000f // EMA filter coefficient
#define G_MPS2          9.81000000000000000000f // Gravitational acceleration (g)

/**
* @fn complimentaryFilter
* @brief Apply a complimentary filter
* @param type  three type
* @n     onlyAccel    :   only get the accel data
* @n     onlyGyro     :   only get the gyro data
* @n     bothAccelGyro:   get boath accel and gyro data
* @param data  save returned data to parameter data
* @return BMI160_OK(0) means succse
*/
void complimentaryFilter(float* filteredAccelGyro, float phiHat_rad, float thetaHat_rad, float dt);
