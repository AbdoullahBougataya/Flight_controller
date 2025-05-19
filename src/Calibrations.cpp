#include "../include/Calibrations.h"

void CalibrateGyroscope(int SC, float *GyroOffset)
{
    BMI160 imu; // Declaring the imu object
    uint8_t rslt = 0; // Define the result of the data extraction from the imu
    const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground
    int16_t accelGyro[6] = { 0 }; // Raw data from the sensor
    float accelGyroData[6] = { 0.0 }; // Data that is going to be processed
    float gyroRateCumulativeOffset[3] = { 0.0 }; // Define a temporary variable to sum the offsets
    // Reset the BMI160 to erased any preprogrammed instructions
    if (imu.softReset() != BMI160_OK) {
      // Turn on the RED LED light
      neopixelWrite(LED_PIN, LED_BRIGHTNESS, 0, 0);
      Serial.println("BMI160: Reset error");
      while (1);
    }

    // Initialize the BMI160 on I²C
    if (imu.Init(addr) != BMI160_OK) {
      // Turn on the RED LED light
      neopixelWrite(LED_PIN, LED_BRIGHTNESS, 0, 0);
      Serial.println("BMI160: Init error");
      while (1);
    }
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
          // Turn on the RED LED light
          neopixelWrite(LED_PIN, LED_BRIGHTNESS, 0, 0);
          Serial.print("BMI160: Data reading error");
          Serial.println();
        }
      }
      for (byte i = 0; i < 3; i++) {
        GyroOffset[i] = gyroRateCumulativeOffset[i] / SC;
      }
}
