#include "./include/BMI160.h"
#include "./include/Filter.h"
#include <math.h>

BMI160 bmi160;
const int8_t addr = 0x68;

#define RAD2DEG         57.2957795130823208767f   // Radians to degrees (per second)

// Define the time step
float dt = 0.0;
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Define sensor data arrays
int16_t accelGyro[6] = { 0 };
float filteredAccelGyro[6] = { 0 };
float rawAccelGyro[6] = { 0 };

// Declare sensor fusion variables
float phiHat_rad = 0.0f;
float thetaHat_rad = 0.0f;

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize the hardware bmin160
  if (bmi160.softReset() != BMI160_OK) {
    Serial.println("reset false");
    while (1);
  }

  // Set and init the bmi160 i2c address
  if (bmi160.Init(addr) != BMI160_OK) {
    Serial.println("init false");
    while (1);
  }
}

void loop() {
  // Calculate time stamp (in seconds)
  currentTime = millis() / 1000.0;
  dt = (currentTime - lastTime);
  lastTime = currentTime;

  // Get both accel and gyro data from bmi160
  // Parameter accelGyro is the pointer to store the data
  uint8_t rslt = bmi160.getAccelGyroData(accelGyro);

  if (rslt == 0) {
    offset(accelGyro, rawAccelGyro);

    EMAFilter(rawAccelGyro, filteredAccelGyro);

    complimentaryFilter(filteredAccelGyro, phiHat_rad, thetaHat_rad, dt);

    Serial.print("Roll-estimate:");
    Serial.print(phiHat_rad * RAD2DEG);
    Serial.print("\t");
    Serial.print("Pitch-estimate:");
    Serial.print(thetaHat_rad * RAD2DEG);
    Serial.print("\t");
    Serial.println();
  }
  else {
    Serial.println("err");
  }
  delay(50);
}
