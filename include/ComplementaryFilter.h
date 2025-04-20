#ifndef COMPLEMENTARY_FILTER_H
#define COMPLEMENTARY_FILTER_H

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#define G_MPS2          9.81000000000000000000f  // Gravitational acceleration (g)
#define PI              3.14159265358979323846f  // Pi

typedef struct {
    // Accelerometer/Gyroscope ratio
    float alpha;

    // Sample time (in seconds)
    float T;

    // Euler Angles
    float eulerAngles[3];
} ComplementaryFilter;

/**
  * @fn ComplementaryFilter_Init
  * @brief Initalize the Complementary Filter
  * @param cf The complementary filter object
  * @param alpha Accelerometer/Gyroscope ratio
  */
 void ComplementaryFilter_Init(ComplementaryFilter* cf, float alpha);

/**
  * @fn ComplementaryFilter_Update
  * @brief Update the Complementary Filter
  * @param cf The complementary filter object
  * @param inp The sensor data in [rad]
  * @param dt The sampling period in [s]
  * @return Pointer to the euler angles array
  */
 float *ComplementaryFilter_Update(ComplementaryFilter* cf, float* inp, float dt);

#endif
