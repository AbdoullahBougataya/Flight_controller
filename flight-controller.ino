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

BMP390_BAROMETER barometer(&Wire, barometer.eSDOVDD);

ComplementaryFilter CF; // Declaring the Complementary filter object

ComplementaryFilter2D CF2; // Declaring the 2D Complementary filter object

RCFilter lpFRC[8]; // Declaring the RC filter object (IMU + Barometer + Vertical velocity)

#define RAD2DEG                          57.29577951308232087680f  // Radians to degrees (per second)
#define G_MPS2                            9.81000000000000000000f  // Gravitational acceleration (g)
#define PI                                3.14159265358979323846f  // Pi
#define SAMPLING_PERIOD                   0.01000000000000000000f  // Sampling period of the sensor in seconds
#define SERIAL_BANDWIDTH_115200      115200                        // The serial monitor's bandwidth
#define STARTUP_DELAY                   100                        // 100 ms for the microcontroller to start
#define INTERRUPT_1_MCU_PIN              17                        // The pin that receives the interrupt 1 signal from the Barometer
#define RC_LOW_PASS_FLTR_CUTOFF_4HZ       4.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define RC_LOW_PASS_FLTR_CUTOFF_5HZ       5.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define RC_LOW_PASS_FLTR_CUTOFF_10HZ     10.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define GYRO_CALIBRATION_SAMPLES_200    200                        // It takes 200 samples to calibrate the gyroscope
#define GYRO_CALIBRATION_SAMPLES_400    400                        // It takes 400 samples to calibrate the gyroscope
#define COMP_FLTR_ALPHA                   0.10000000000000000000f  // Complimentary filter coefficient
#define COMP_FLTR_2D_ALPHA                0.50000000000000000000f  // 2D Complimentary filter coefficient
#define ALTITUDE                         70.00000000000000000000f  // Current altitude of the Quadcopter

volatile uint8_t barometerFlag = 0; // Barometer interrupt flag

const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground (IMU)

uint8_t IMUrslt; // Define the result of the data extraction from the imu

uint8_t Barslt; // Define the result of the data extraction from the barometer

unsigned long ST = 0; // Start Time [us]
float T;              // Measured Period [s]

float gyroRateOffset[3] = { 0.0 }; // Offset of the Gyroscope

// Define sensor data arrays
int16_t accelGyroData_int[6] = { 0 }; // Raw data from the sensor
float accelGyroData[6] = { 0.0 }; // Data that is going to be processed
float altitude = 0.0; // Altitude from the barometer

// Declare sensor fusion variables
float eulerAngles[3] = { 0.0 }; // Euler angles φ, θ and ψ
float verticalVelocity = 0.0; // The vertical velocity of the Quadcopter

/* External interrupt flag */
void barometerInterrupt()
{
  if(!barometerFlag) barometerFlag = 1;
}

// Section 2: Initialization & setup section.

void setup() {
  // Initialize serial communication at 115200 bytes per second:
  Serial.begin(SERIAL_BANDWIDTH_115200);

  // Begin the communication with the barometer
  while ( ERR_OK != (Barslt = barometer.begin()) ){

    if (ERR_DATA_BUS == Barslt) {
      Serial.println("BMP390: Data bus error");
    } else if (ERR_IC_VERSION == Barslt) {
      Serial.println("BMP390: Chip versions do not match");
    }

    delay(3000);
  }
  Serial.println("BMP390: Begin ok");

    /**
   * 6 commonly used sampling modes that allows users to configure easily, mode:
   *      eUltraLowPrecision, Ultra-low precision, suitable for monitoring weather (lowest power consumption), the power is mandatory mode.
   *      eLowPrecision, Low precision, suitable for random detection, power is normal mode
   *      eNormalPrecision1, Normal precision 1, suitable for dynamic detection on handheld devices (e.g on mobile phones), power is normal mode.
   *      eNormalPrecision2, Normal precision 2, suitable for drones, power is normal mode.
   *      eHighPrecision, High precision, suitable for low-power handled devices (e.g mobile phones), power is in normal mode.
   *      eUltraPrecision, Ultra-high precision, suitable for indoor navigation, its acquisition rate will be extremely low, and the acquisition cycle is 1000 ms.
   */
  while (!barometer.setSamplingMode(barometer.eHighPrecision)) {
    Serial.println("BMP390: Set sampling mode fail, retrying....");
    delay(3000);
  }

  /* Set the internal IIR low pass filter:
  * IIR filter coefficient configuration (IIR filtering) mode IIR filter coefficient setting, configurable modes:
  *           BMP390_IIR_CONFIG_COEF_0, BMP390_IIR_CONFIG_COEF_1, BMP390_IIR_CONFIG_COEF_3,
  *           BMP390_IIR_CONFIG_COEF_7, BMP390_IIR_CONFIG_COEF_15, BMP390_IIR_CONFIG_COEF_31,
  *           BMP390_IIR_CONFIG_COEF_63, BMP390_IIR_CONFIG_COEF_127
  */
  barometer.setIIRMode(BMP390_IIR_CONFIG_COEF_31);

  /**
   * Interrupt configuration
   * mode The interrupt mode needs to set. The following modes add up to mode:
   *      Interrupt pin output mode: eINTPinPP: Push pull, eINTPinOD: Open drain
   *      Interrupt pin active level: eINTPinActiveLevelLow: Active low, eINTPinActiveLevelHigh: Active high
   *      Register interrupt latch: eINTLatchDIS: Disable, eINTLatchEN: Enable
   *      FIFO water level reached interrupt: eIntFWtmDis: Disable, eIntFWtmEn: Enable
   *      FIFO full interrupt: eINTFFullDIS: Disable, eINTFFullEN: Enable
   *      Interrupt pin initial (invalid, non-interrupt) level: eINTInitialLevelLOW: Low, eINTInitialLevelHIGH: High
   *      Temperature/pressure data ready interrupt: eINTDataDrdyDIS: Disable, eINTDataDrdyEN: Enable
   * Notice: In non-latching mode (eINTLatchDIS), interrupt signal is 2.5 ms pulse signal
   * Note: When using eINTPinActiveLevelLow (Active low interrupt pin), you need to use eINTInitialLevelHIGH (Initial
   *       level of interrupt pin is high). Please use “FALLING” to trigger the following interrupt.
   *       When using eINTPinActiveLevelHigh (Active low interrupt pin), you need to use eINTInitialLevelLOW (Initial
   *       level of interrupt pin is high). Please use “RISING” to trigger the following interrupt.
   */
  barometer.setINTMode(barometer.eINTPinPP |
    barometer.eINTPinActiveLevelHigh |
    barometer.eINTLatchDIS |
    barometer.eINTInitialLevelLOW |
    barometer.eINTDataDrdyEN);

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

  for (int i = 0; i < 8; i++) {
    RCFilter_Init(&lpFRC[i], RC_LOW_PASS_FLTR_CUTOFF_5HZ, SAMPLING_PERIOD); // Initialize the RCFilter fc = 5 Hz ; Ts = 0.01 s
  }

  ComplementaryFilter_Init(&CF, COMP_FLTR_ALPHA);

  ComplementaryFilter2D_Init(&CF2, COMP_FLTR_2D_ALPHA);

  attachInterrupt(digitalPinToInterrupt(INTERRUPT_1_MCU_PIN), barometerInterrupt, CHANGE); // Execute the ISR on interrupt level change
  Serial.println("BMI160: Calibrating");
  CalibrateGyroscope(GYRO_CALIBRATION_SAMPLES_400, gyroRateOffset);
}

// Section 3: Looping and realtime processing.

void loop() {
  // Dynamic Period
  T = (micros() - ST) / 1000000.0;
  ST = micros();

  // Read altitude from the Barometer
  if (barometerFlag) {
    barometerFlag = 0; // Clear the flag
    /* When data is ready and the interrupt is triggered, read altitude, unit: m */
    altitude = barometer.readAltitudeM();
    altitude = RCFilter_Update(&lpFRC[6], altitude); // Update the RCFilter
  }

  // Initialize sensor data arrays
  memset(accelGyroData_int, 0, sizeof(accelGyroData_int));
  memset(accelGyroData, 0, sizeof(accelGyroData));

  // Get both accel and gyro data from the BMI160
  // Parameter accelGyro is the pointer to store the data
  IMUrslt = imu.getAccelGyroData(accelGyroData_int);

  // if the data is succesfully extracted
  if (IMUrslt == 0) {
    IMUrslt = 1;
    // Format and offset the accelerometer data
    offset(accelGyroData_int, accelGyroData);

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
      euler angles (phi: roll, theta: pitch, psi: yaw)
    */
    ComplementaryFilter_Update(&CF, accelGyroData, eulerAngles, T); // This function transform the gyro rates and the Accelerometer angles into equivalent euler angles
  }
  else
  {
    Serial.print("BMI160: Data reading error");
    Serial.println();
  }
  verticalVelocity = ComplementaryFilter2D_Update(&CF2, accelGyroData, eulerAngles, altitude, T);
  verticalVelocity = RCFilter_Update(&lpFRC[7], verticalVelocity); // Update the RCFilter
  Serial.print(accelGyroData[3]);Serial.print(", \t");
  Serial.print(accelGyroData[4]);Serial.print(", \t");
  Serial.print(accelGyroData[5]);Serial.print(", \t");
  Serial.print(T);Serial.print(", \t");
  Serial.print(altitude);Serial.print(", \t");
  Serial.print(verticalVelocity);Serial.println();

  while ((micros() - ST) / 1000000.0 <= 0.01)
  {
    delay(1);
  }
}
