/*!
 * @file  flight-controller.ino
 * @brief  Runs the flight control code for our ESP32 based quadcopter
 * @author  [Abdellah Bougataya](sience.story@gmail.com)
 * @version  V1.0
 * @date  2024-12-20
 * @url  https://github.com/AbdoullahBougataya/Flight_controller
 */
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


#include "./include/RCFilter.h"
#include "./include/BMP390.h"
#include "./include/ComplementaryFilter.h"
#include "./include/2D_ComplementaryFilter.h"
#include "./include/Calibrations.h"
#include <math.h>

// Section 1: Constants & Global variables declarations.

BMI160 imu; // Declaring the imu object

ComplementaryFilter CF; // Declaring the Complementary filter object

RCFilter lpFRC[6]; // Declaring the RC filter object

#define RAD2DEG                          57.29577951308232087680f  // Radians to degrees (per second)
#define G_MPS2                            9.81000000000000000000f  // Gravitational acceleration (g)
#define PI                                3.14159265358979323846f  // Pi
#define SAMPLING_PERIOD                   0.01000000000000000000f  // Sampling period of the sensor in seconds
#define SERIAL_BANDWIDTH_115200      115200                        // The serial monitor's bandwidth
#define STARTUP_DELAY                   100                        // 100 ms for the microcontroller to start
#define INTERRUPT_1_MCU_PIN              17                        // The pin that receives the interrupt 1 signal from the IMU
#define RC_LOW_PASS_FLTR_CUTOFF_5HZ       5.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define RC_LOW_PASS_FLTR_CUTOFF_10HZ     10.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define GYRO_CALIBRATION_SAMPLES_200    200                        // It takes 200 samples to calibrate the gyroscope
#define GYRO_CALIBRATION_SAMPLES_400    400                        // It takes 400 samples to calibrate the gyroscope
#define COMP_FLTR_ALPHA                   0.50000000000000000000f  // Complimentary filter coefficient

const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground

volatile bool dataReady = false; // Sensor Data Ready ? yes:true | no:false

uint8_t rslt; // Define the result of the data extraction from the imu

unsigned long ST = 0;
float T;

float *gyroRateOffset; // Gyro rates offsets

// Define sensor data arrays
int16_t accelGyro[6] = { 0 }; // Raw data from the sensor
float accelGyroData[6] = { 0.0 }; // Data that is going to be processed

// Declare sensor fusion variables
float *eulerAngles; // Euler angles φ, θ and ψ

// Functions
void AccelGyroISR(); // This is the Interrupt Service Routine for retrieving data from the sensor

// Section 2: Initialization & setup section.

void setup() {
  // Initialize serial communication at 115200 bytes per second:
  Serial.begin(SERIAL_BANDWIDTH_115200);

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
    RCFilter_Init(&lpFRC[i], RC_LOW_PASS_FLTR_CUTOFF_10HZ); // Initialize the RCFilter fc = 5 Hz
  }

  ComplementaryFilter_Init(&CF, COMP_FLTR_ALPHA);

  gyroRateOffset = CalibrateGyroscope(GYRO_CALIBRATION_SAMPLES_400);

}

// Section 3: Looping and realtime processing.

void loop() {
  T = (millis() - ST) / 1000;
  ST = millis();
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
  }
  // if the data is succesfully extracted
  if (rslt == 0) {
    // Format and offset the accelerometer data
    offset(accelGyro, accelGyroData);

    // Substract the offsets from the Gyro readings
    for (byte i = 0; i < 3; i++) {
      accelGyroData[i] -= gyroRateOffset[i];
    }

    for (int i = 0; i < 6; i++) {
      accelGyroData[i] = RCFilter_Update(&lpFRC[i], accelGyroData[i], T); // Update the RCFilter ; Ts = 0.01 s
    }

    /*
      A complimentary filter is a premitive technique of sensor fusion
      to use both the accelerometer and the gyroscope to predict the
      euler angles (phi: roll, theta: pitch, psi: yaw)
    */
    eulerAngles = ComplementaryFilter_Update(&CF, accelGyroData, T); // This function transform the gyro rates and the Accelerometer angles into equivalent euler angles

    Serial.print(accelGyroData[0]);Serial.print(", \t");
    Serial.print(accelGyroData[1]);Serial.print(", \t");
    Serial.print(accelGyroData[2]);Serial.print(", \t");
    Serial.print(eulerAngles[0] * RAD2DEG);Serial.print(", \t");
    Serial.print(eulerAngles[1] * RAD2DEG);Serial.print(", \t");
    Serial.println();
  }
  else
  {
    Serial.print("!!! Data reading error !!!");
    Serial.println();
  }
}

// Section 4: Function declarations.

// Accelerometer and Gyroscope interrupt service routine
void AccelGyroISR() {
  if(dataReady == false)
  {
    dataReady = true;
  }
}
