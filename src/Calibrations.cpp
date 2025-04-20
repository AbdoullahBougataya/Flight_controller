#include "../include/Calibrations.h"

float *CalibrateGyroscope(int SC)
{
    BMI160 imu; // Declaring the imu object
    uint8_t rslt = 0; // Define the result of the data extraction from the imu
    int16_t accelGyro[6] = { 0 }; // Raw data from the sensor
    float accelGyroData[6] = { 0.0 }; // Data that is going to be processed
    float gyroRateOffset[3] = { 0.0 }; // Gyro rate offset
    float gyroRateCumulativeOffset[3] = { 0.0 }; // Define a temporary variable to sum the offsets
    for (int i = 0; i < SC; i++) {

        // Initialize sensor data arrays
        memset(accelGyro, 0, sizeof(accelGyro));
        memset(accelGyroData, 0, sizeof(accelGyroData));

        // Get both accel and gyro data from the IMU
        // Parameter accelGyro is the pointer to store the data
        rslt = imu.getAccelGyroData(accelGyro);
        if (rslt == 0)
        {
          // Formatting the data
          offset(accelGyro, accelGyroData);
          for (byte j = 0; j < 3; j++) {
            gyroRateCumulativeOffset[j] += accelGyroData[j]; // Accumulating the gyroscope error
          }
        }
        else
        {
          Serial.print("!!! Data reading error !!!");
          Serial.println();
        }
      }
      for (byte i = 0; i < 3; i++) {
        gyroRateOffset[i] = gyroRateCumulativeOffset[i] / SC;
      }
      return gyroRateOffset;
}
