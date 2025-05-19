#ifndef COMPLEMENTARY_FILTER_2D_H
#define COMPLEMENTARY_FILTER_2D_H

#include <Arduino.h>
#include <stdint.h>
#include <math.h>

#define G_MPS2          9.81000000000000000000f  // Gravitational acceleration (g)

typedef struct {
    // Accelerometer/Barometer ratio
    float alpha;

    // Sample time (in seconds)
    float T;

    // Altitude from the barometer
    int alt;

    // vertical acceleration from the accelerometer
    float accel[2];

    // Velocity from the accelerometer and from the barometer (in meters per seconds)
    float velocities[2];

    // Output vertical velocity (in meters per seconds)
    float velocity;
} ComplementaryFilter2D;

/**
  * @fn ComplementaryFilter2D_Init
  * @brief Initalize the 2D Complementary Filter
  * @param cf2 The 2D complementary filter object
  * @param alpha Accelerometer/Gyroscope ratio
  */
 void ComplementaryFilter2D_Init(ComplementaryFilter2D* cf2, float alpha);
/**
  * @fn ComplementaryFilter2D_Update
  * @brief Update the 2D Complementary Filter
  * @param cf2 The 2D complementary filter object
  * @param accel The IMU sensor data in [rad]
  * @param euler The euler angles array φ, θ and ψ
  * @param alt The Barometer sensor data in [m]
  * @param dt The sampling period in [s]
  * @return The normal velocity
  */
 float ComplementaryFilter2D_Update(ComplementaryFilter2D* cf2, float* accel, float* euler, float alt, float dt);

#endif
