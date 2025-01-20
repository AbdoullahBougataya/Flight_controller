#include "./include/BMI160.h"
#include <math.h>

BMI160 imu;

const int8_t addr = 0x68;

#define RAD2DEG 57.2957795130823208767f   // Radians to degrees (per second)

// Define sensor data arrays
int16_t accelerometer[3] = { 0 };
int16_t gyroscope[3] = { 0 };
float rawAccelerometer[3] = { 0 };
float rawGyroscope[3] = { 0 };

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize the hardware bmin160
  if (imu.softReset() != BMI160_OK) {
    Serial.println("reset false");
    while (1);
  }

  // Set and init the imu i2c address
  if (imu.I2cInit(addr) != BMI160_OK) {
    Serial.println("init false");
    while (1);
  }
}

void loop() {
  // Initialize sensor data arrays
  accelerometer[3] = { 0 };
  gyroscope[3] = { 0 };
  rawAccelerometer[3] = { 0 };
  rawGyroscope[3] = { 0 };

  // Get both accel and gyro data from imu
  uint8_t rslt = imu.getAccelGyroData(accelGyro);

  if (rslt == 0) {
    Serial.print(accelGyro[0] * 3.14/180.0);Serial.print("\t");
    Serial.print(accelGyro[1] * 3.14/180.0);Serial.print("\t");
    Serial.print(accelGyro[2] * 3.14/180.0);Serial.print("\t");
    Serial.println();
  }
  else {
    Serial.println("err");
  }
  delay(10);
}
