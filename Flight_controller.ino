#include "./include/BMI160.h"
#include "./include/Filter.h"
#include <math.h>

BMI160 bmi160;
const int8_t addr = 0x68;

#define RAD2DEG 57.2957795130823208767f   // Radians to degrees (per second)

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
  // Initialize sensor data arrays
  accelGyro[6] = { 0 };
  rawAccelGyro[6] = { 0 };

  // Get both accel and gyro data from bmi160
  // Parameter accelGyro is the pointer to store the data
  uint8_t rslt = bmi160.getAccelGyroData(accelGyro);

  if (rslt == 0) {
    offset(accelGyro, rawAccelGyro);

    EMAFilter(rawAccelGyro, filteredAccelGyro);

    complimentaryFilter(filteredAccelGyro, phiHat_rad, thetaHat_rad, dt);

    Serial.print(phiHat_rad * RAD2DEG);
    Serial.print("\t");
    Serial.print(thetaHat_rad * RAD2DEG);
    Serial.print("\t");
    Serial.println();
  }
  else {
    Serial.println("err");
  }
  delay(10);
}

// Complimentary filter
void complimentaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, float dt) {
  // Using gravity to estimate the Euler angles
  float phiHat_acc_rad = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]);                 // Roll estimate
  float thetaHat_acc_rad = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f));  // Pitch estimate

  // Using gyroscope to estimate the euler rates
  float phiDot_rps   = (sinf(phiHat_rad) * filteredAccelGyro[1] + cosf(phiHat_rad) * filteredAccelGyro[2]) * tanf(thetaHat_rad) + filteredAccelGyro[0];  // Roll rate (rad/s)
  float thetaDot_rps =  cosf(phiHat_rad) * filteredAccelGyro[1] - sinf(phiHat_rad) * filteredAccelGyro[2];                                               // Pitch rate (rad/s)

  // Complementary filter implementation
  phiHat_rad = COMP_FLTR_ALPHA * phiHat_acc_rad + (1.0f - COMP_FLTR_ALPHA) * (phiHat_rad + dt * phiDot_rps);          // Roll estimate
  thetaHat_rad = COMP_FLTR_ALPHA * thetaHat_acc_rad + (1.0f - COMP_FLTR_ALPHA) * (thetaHat_rad + dt * thetaDot_rps);  // Pitch estimate
}
