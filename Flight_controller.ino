#include "./include/BMI160.h"
#include "./include/FIRFilter.h"
#include <math.h>

BMI160 imu;

FIRFilter lpfAcc;

const int8_t addr = 0x68;

#define RAD2DEG 57.2957795130823208767f   // Radians to degrees (per second)

// Define sensor data arrays
int16_t accelGyro[6] = { 0 };
float rawAccelGyro[6] = { 0 };

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize the hardware bmin160
  if (imu.softReset() != BMI160_OK) {
    Serial.println("reset false");
    while (1);
  }

  // Set and init the imu i2c address
  if (imu.Init(addr) != BMI160_OK) {
    Serial.println("init false");
    while (1);
  }
  // Initialise FIR Filter
  FIRFilter_Init(&lpfAcc);
}

void loop() {
  // Initialize sensor data arrays
  accelGyro[6] = { 0 };
  rawAccelGyro[6] = { 0 };

  // Get both accel and gyro data from imu
  // Parameter accelGyro is the pointer to store the data
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
