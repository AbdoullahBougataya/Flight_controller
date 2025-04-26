#include "./BMI160.h"

#define CALIBRATION_SAMPLES            1000;
#define G_MPS2                            9.81000000000000000000f  // Gravitational acceleration (g)

BMI160 imu; // Declaring the imu object

uint8_t rslt = 0; // Define the result of the data extraction from the imu

const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground

int16_t accelGyro[6] = { 0 }; // Raw data from the sensor

float accelGyroData[6] = { 0.0 }; // Data that is going to be processed

float accelOffset[3] = { 0.0 };

void setup()
{
    Serial.begin(115200);

    // Reset the BMI160 to erased any preprogrammed instructions
    if (imu.softReset() != BMI160_OK) {
        Serial.println("BMI160: Reset error");
        while (1);
    }

    // Initialize the BMI160 on I²C
    if (imu.Init(addr) != BMI160_OK) {
        Serial.println("BMI160: Init error");
        while (1);
    }

    float accelCumulativeOffset[3] = { 0.0 }; // Define a temporary variable to sum the offsets

    for (int i = 0; i < 1000; i++) {

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
          accelGyroData[5] -= G_MPS2;
          for (byte j = 0; j < 3; j++) {
            accelCumulativeOffset[j] += accelGyroData[j + 3]; // Accumulating the gyroscope error
          }
        }
        else
        {
          Serial.print("!!! Data reading error !!!");
          Serial.println();
        }
      }
      for (byte i = 0; i < 3; i++) {
        accelOffset[i] = accelCumulativeOffset[i] / CALIBRATION_SAMPLES;
      }
}

void loop()
{
    for (byte i = 0; i < 3; i++) {
        Serial.print(accelOffset[i]);Serial.print(", \t");
    }
    Serial.println();
}
