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
      4. Take instructions in real time from the RC controller.
    In progress ⏳:
      1. Perform sensor fusion
    Done ✅:
      1. Initialize the sensors.
      2. Calibrate the gyroscope.
      3. Filters the sensor data.
           -------------------------------------------
  The flight controller code was highly inspired from various source in the internet, most notably:
    * Carbon aeronautics series on making a Quadcopter using Teensy (Arduino compatible).
    * Phil's Lab series on DSP using STM32 (Included more advanced topics like the Filtering, EKF, Compilmentary...).
 */

#include "../include/BMI160.h"
#include "../include/RCFilter.h"
#include "../include/PID.h"
#include <math.h>

// Section 1: Constants & Global variables declarations.

BMI160 imu; // Declaring the imu object

RCFilter lpFRC[6]; // Declaring the RC filter object

PIDController pid; // Declaring the PID object

#define RAD2DEG                          57.29577951308232087680f  // Radians to degrees (per second)
#define G_MPS2                            9.81000000000000000000f  // Gravitational acceleration (g)
#define PI                                3.14159265358979323846f  // Pi
#define SAMPLING_PERIOD                   0.01000000000000000000f  // Sampling period of the sensor in seconds
#define SERIAL_BANDWIDTH_115200      115200                        // The serial monitor's bandwidth
#define STARTUP_DELAY                   100                        // 100 ms for the microcontroller to start
#define INTERRUPT_1_MCU_PIN               2                        // The pin that receives the interrupt 1 signal from the IMU
#define RC_LOW_PASS_FLTR_CUTOFF_5HZ       5.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define GYRO_CALIBRATION_SAMPLES_200    200                        // It takes 200 samples to calibrate the gyroscope
#define COMP_FLTR_ALPHA                   0.03000000000000000000f  // Complimentary filter coefficient


float PID_output = 0.0f;

const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground

volatile bool dataReady = false; // Sensor Data Ready ? yes:true | no:false

uint8_t rslt = 0; // Define the result of the data extraction from the imu

float gyroRateOffset[3] = { 0.0 }; // Gyro rates offsets

// Define sensor data arrays
int16_t accelGyro[6] = { 0 }; // Raw data from the sensor
float accelGyroData[6] = { 0 }; // Data that is going to be processed

// Declare sensor fusion variables
float phiHat_rad = 0.0f; // Euler Roll
float thetaHat_rad = 0.0f; // Euler Pitch

// Functions
void complementaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, float dt, float alpha);
void AccelGyroISR(); // This is the Interrupt Service Routine for retrieving data from the sensor

// Section 2: Initialization & setup section.

void setup() {
  /*========================================*/
  /*     The PID controller settings        */
  /*========================================*/
  /*        Controller coefficients         */
  /**/pid.Kp = 1.0f;                        //
  /**/pid.Ki = 0.6f;                        //
  /**/pid.Kd = 0.0f;                        //
  /*----------------------------------------*/
  /*Derivative low-pass filter time constant*/
  /**/pid.tau = 1.0f;                       //
  /*----------------------------------------*/
  /*               Clampings                */
  /**/pid.limMin = -1.0f;                   //
  /**/pid.limMax =  1.0f;                   //
  /*========================================*/
  // initialize serial communication at 115200 bits per second:
  Serial.begin(SERIAL_BANDWIDTH_115200);
  delay(STARTUP_DELAY);

  // Reset the BMI160 to erased any preprogrammed instructions
  if (imu.softReset() != BMI160_OK) {
    Serial.println("Reset error");
    while (1);
  }

  // Initialize the BMI160 on I²C
  if (imu.Init(addr) != BMI160_OK) {
    Serial.println("Init error");
    while (1);
  }

  // Initialize the BMI160 interrupt 1
  if (imu.setInt() != BMI160_OK) {
    Serial.println("Interrupt error");
    while (1);
  }

  // Everytime a pulse is received from the sensor, the AccelGyroISR() will set the dataReady to true, which will enable the code to be ran in the loop
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_1_MCU_PIN), AccelGyroISR, RISING);

  for (int i = 0; i < 6; i++) {
    RCFilter_Init(&lpFRC[i], RC_LOW_PASS_FLTR_CUTOFF_5HZ, SAMPLING_PERIOD); // Initialize the RCFilter fc = 5 Hz ; Ts = 0.01 s
  }

  PIDController_Init(&pid, SAMPLING_PERIOD);

  float gyroRateCumulativeOffset[3] = { 0.0 }; // Define a temporary variable to sum the offsets

  // For five seconds the gyroscope will be calibrating (make sure you put it on a flat surface)
  Serial.print("Calibrating...");
  for (int i = 0; i < GYRO_CALIBRATION_SAMPLES_200; i++) {

    // Initialize sensor data arrays
    memset(accelGyro, 0, sizeof(accelGyro));
    memset(accelGyroData, 0, sizeof(accelGyroData));

    // Get both accel and gyro data from the BMI160
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
    delay(1);
  }
  // Calculate the average offset
  for (byte i = 0; i < 3; i++) {
    gyroRateOffset[i] = gyroRateCumulativeOffset[i] / GYRO_CALIBRATION_SAMPLES_200;
  }
}

// Section 3: Looping and realtime processing.

void loop() {

// Checking if there is data ready in the sensor
  if (dataReady)
  {
    dataReady = false; // Reseting the dataReady flag
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

      for (int i = 0; i < 6; i++) {
        accelGyroData[i] = RCFilter_Update(&lpFRC[i], accelGyroData[i]); // Update the RCFilter
      }

      /*
        A complimentary filter is a premitive technique of sensor fusion
        to use both the accelerometer and the gyroscope to predict the
        euler angles (phi: roll, theta: pitch)
      */
      complementaryFilter(accelGyroData, phiHat_rad, thetaHat_rad, SAMPLING_PERIOD, COMP_FLTR_ALPHA); // This function transform the gyro rates and the Accelerometer angles into equivalent euler angles

      // Update the PID controller
      PID_output = PIDController_Update(&pid, 0.0f, phiHat_rad);

      // Print the euler angles to the serial monitor
      Serial.print(phiHat_rad * RAD2DEG);Serial.print("\t");
      Serial.print(thetaHat_rad * RAD2DEG);Serial.print("\t");
      Serial.print(PID_output * RAD2DEG);Serial.print("\t");
      Serial.println();
    }
    else
    {
      Serial.print("!!! Data reading error !!!");
      Serial.println();
    }
  }
}

// Section 4: Function declarations.

// Accelerometer and Gyroscope interrupt service routine
void AccelGyroISR() {
  dataReady = true;
}

// Complementary filter (Check Phil's Lab video for more details)
void complementaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, float dt, float alpha) {
  // Using gravity to estimate the Euler angles
  float phiHat_acc_rad = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]);                 // Roll estimate
  float thetaHat_acc_rad = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f));  // Pitch estimate

  // Using gyroscope to estimate the euler rates (Transforming body rates to euler rates)
  float phiDot_rps   = (sinf(phiHat_rad) * filteredAccelGyro[1] + cosf(phiHat_rad) * filteredAccelGyro[2]) * tanf(thetaHat_rad) + filteredAccelGyro[0];  // Roll rate (rad/s)
  float thetaDot_rps =  cosf(phiHat_rad) * filteredAccelGyro[1] - sinf(phiHat_rad) * filteredAccelGyro[2];                                               // Pitch rate (rad/s)

  // Complementary filter implementation [Just like mixing the data from the gyroscope and the accelerometer with alpha proportions](alpha should be between 0 and 1)
  phiHat_rad = fminf(fmaxf(alpha, 0.0f), 1.0f) * phiHat_acc_rad + (1.0f - fminf(fmaxf(alpha, 0.0f), 1.0f)) * (phiHat_rad + dt * phiDot_rps);          // Roll estimate
  thetaHat_rad = fminf(fmaxf(alpha, 0.0f), 1.0f) * thetaHat_acc_rad + (1.0f - fminf(fmaxf(alpha, 0.0f), 1.0f)) * (thetaHat_rad + dt * thetaDot_rps);  // Pitch estimate

  // Bound the values of the pitch and roll
  phiHat_rad = fminf(fmaxf(phiHat_rad, -PI), PI);
  thetaHat_rad = fminf(fmaxf(thetaHat_rad, -PI), PI);
  if (abs(phiHat_rad) < 1) {
    phiHat_rad = 0;
  }
  if (abs(thetaHat_rad) < 1) {
    thetaHat_rad = 0;
  }
}
