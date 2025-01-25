/*
  This code is partitioned into four sections:
    * Constants & Global variables declarations. 📝
    * Initialization & setup section. 📌
    * Looping and realtime processing. 🔁
    * Function declarations. ▶️
           -------------------------------------------
  Tasks:
    To do 🎯:
      1. Implement additional signal processing for PID control.
      2. Implement the PID control algorithms.
      3. Implement PWM signal generation algorithms.
    In progress ⏳:
      1. Filters the sensor data.
      2. Perform sensor fusion
    Done ✅:
      1. Initialize the sensors.
      2. Calibrate the gyroscope.
           -------------------------------------------
  The flight controller code was highly inspired from various source in the internet, most notably:
    * Carbon aeronautics series on making a Quadcopter using Teensy (Arduino compatible).
    * Phil's Lab series on DSP using STM32 (Included more advanced topics like the Filtering, EKF, Compilmentary...).
 */

#include "./include/BMI160.h"
#include "./include/RCFilter.h"
#include <math.h>

// Section 1: Constants & Global variables declarations.

BMI160 imu; // Declaring the imu object

RCFilter lpFRC[6]; // Declaring the RC filter object

void complementaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, float dt);

const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground

#define COMP_FLTR_ALPHA 0.01000000000000000000f  // Complimentary filter coefficient
#define RAD2DEG        57.2957795130823208767f   // Radians to degrees (per second)
#define G_MPS2          9.81000000000000000000f  // Gravitational acceleration (g)

uint8_t rslt = 0; // Define the result of the data extraction from the imu

float gyroRateOffset[3] = { 0.0 }; // Gyro rates offsets

// Define the time step
unsigned long dt = 0;
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Define sensor data arrays
int16_t accelGyro[6] = { 0 }; // Raw data from the sensor
float accelGyroData[6] = { 0 }; // Data that is going to be processed

// Declare sensor fusion variables
float phiHat_rad = 0.0f; // Euler Roll
float thetaHat_rad = 0.0f; // Euler Pitch

// Section 2: Initialization & setup section.

void setup() {
  Serial.begin(115200); // Setup the baud rate of the serial monitor
  delay(100); // Controller startup delay

  // Reset the BMI160 to erased any preprogrammed instructions
  if (imu.softReset() != BMI160_OK) {
    Serial.println("reset false");
    while (1);
  }

  // Initialize the BMI160 on I²C
  if (imu.Init(addr) != BMI160_OK) {
    Serial.println("init false");
    while (1);
  }

  for (int i = 0; i < 6; i++) {
    RCFilter_Init(&lpFRC[i], 5.0f, 36.5f); // Initialize the RCFilter fc = 5 Hz ; Ts = 36.5 ms
  }

  float gyroRateCumulativeOffset[3] = { 0.0 }; // Define a temporary variable to sum the offsets

  // For five seconds the gyroscope will be calibrating (make sure you put it on a flat surface)
  Serial.println("Calibrating...");
  for (int i = 0; i < 200; i++) {
    // Initialize sensor data arrays
    memset(accelGyro, 0, sizeof(accelGyro));
    memset(accelGyroData, 0, sizeof(accelGyroData));

    // Get both accel and gyro data from the BMI160
    // Parameter accelGyro is the pointer to store the data
    imu.getAccelGyroData(accelGyro);
    offset(accelGyro, accelGyroData);
    for (byte j = 0; j < 3; j++) {
      gyroRateCumulativeOffset[j] += accelGyroData[j];
    }
    delay(1);
  }
  // Calculate the average offset
  for (byte i = 0; i < 3; i++) {
    gyroRateOffset[i] = gyroRateCumulativeOffset[i] / 200;
  }
}

// Section 3: Looping and realtime processing.

void loop() {

  // Calculate time stamp (in milliseconds)
  currentTime = millis();
  dt = (currentTime - lastTime) & 0xFFFFFFFF;
  lastTime = currentTime;

  // Initialize sensor data arrays
  memset(accelGyro, 0, sizeof(accelGyro));
  memset(accelGyroData, 0, sizeof(accelGyroData));

  // Get both accel and gyro data from the BMI160
  // Parameter accelGyro is the pointer to store the data
  rslt = imu.getAccelGyroData(accelGyro);


  // if the data is succesfully extracted
  if (rslt == 0) {
    // Format and offset the accelerometer data
    offset(accelGyro, accelGyroData);

    // Substract the offsets from the Gyro readings
    for (byte i = 0; i < 3; i++) {
      accelGyroData[i] -= gyroRateOffset[i];
    }

    for (int i = 0; i < 6; i++){
      accelGyroData[i] = RCFilter_Update(&lpFRC[i], accelGyroData[i]); // Update the RCFilter
    }

    /*
       A complimentary filter is a premitive technique of sensor fusion
       to use both the accelerometer and the gyroscope to predict the
       euler angles (phi: roll, theta: pitch)
    */
    complementaryFilter(accelGyroData, phiHat_rad, thetaHat_rad, dt); // This function transform the gyro rates and the Accelerometer angles into equivalent euler angles

    // Print the euler angles to the serial monitor
    Serial.print(phiHat_rad * RAD2DEG);Serial.print("\t");
    Serial.print(thetaHat_rad * RAD2DEG);Serial.print("\t");
    Serial.println();
  }
  else {
    // Print an error otherwise
    Serial.println("!!! Data extraction failed !!!");
  }
}

// Section 4: Function declarations.

// Complimentary filter (Check Phil's Lab video for more details)
void complementaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, unsigned long dt) {
  // Using gravity to estimate the Euler angles
  float phiHat_acc_rad = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]);                 // Roll estimate
  float thetaHat_acc_rad = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f));  // Pitch estimate

  // Using gyroscope to estimate the euler rates (Transforming body rates to euler rates)
  float phiDot_rps   = (sinf(phiHat_rad) * filteredAccelGyro[1] + cosf(phiHat_rad) * filteredAccelGyro[2]) * tanf(thetaHat_rad) + filteredAccelGyro[0];  // Roll rate (rad/s)
  float thetaDot_rps =  cosf(phiHat_rad) * filteredAccelGyro[1] - sinf(phiHat_rad) * filteredAccelGyro[2];                                               // Pitch rate (rad/s)

  // Complementary filter implementation (Just like mixing the data from the gyroscope and the accelerometer with COMP_FLTR_ALPHA proportions)
  phiHat_rad = COMP_FLTR_ALPHA * phiHat_acc_rad + (1.0f - COMP_FLTR_ALPHA) * (phiHat_rad + (dt/1000.0) * phiDot_rps);          // Roll estimate
  thetaHat_rad = COMP_FLTR_ALPHA * thetaHat_acc_rad + (1.0f - COMP_FLTR_ALPHA) * (thetaHat_rad + (dt / 1000.0) * thetaDot_rps);  // Pitch estimate
}

