/*!
 * @file  flight-controller.ino
 * @brief  Runs the flight control code for our ESP32 based quadcopter
 * @author  [Abdellah Bougataya](sience.story@gmail.com)
 * @version  V1.0
 * @date  2024-12-20
 * @url  https://github.com/AbdoullahBougataya/Flight_controller
 */

 #include "./include/RCFilter.h"
 #include "./include/BMP390.h"
 #include "./include/ComplementaryFilter.h"
 #include "./include/2D_ComplementaryFilter.h"
 #include "./include/Calibrations.h"
 #include "./include/MotorControl.h"
 #include "./include/MotorMixingAlgorithm.h"
 #include "./include/PID.h"
 #include "./include/PPMDecoder.h"
 #include "settings.h"
 #include <math.h>

// Section 1: Constants & Global variables declarations.

Motor motor[MTR_NUMBER]; // Declaring the Motor object (clockwise starting from Left-Front)

PPMDecoder ppm(PPM_PIN, CHANNEL_NUMBER); // Declaring the PPM object that receives data from the RC

PIDController pid[DEGREES_OF_CONTROL]; // Declaring the PID objects ({Roll, Pitch, Yaw} rate controllers + {Roll, Pitch} angle controllers + Vertical velocity controller)

BMI160 imu; // Declaring the imu object

BMP390_BAROMETER barometer(&Wire, barometer.eSDOVDD);

ComplementaryFilter CF; // Declaring the Complementary filter object

ComplementaryFilter2D CF2; // Declaring the 2D Complementary filter object

RCFilter lpFRC[8]; // Declaring the RC filter objects (IMU + Barometer + Vertical velocity)

volatile uint8_t barometerFlag = 0; // Barometer interrupt flag

const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground (IMU)

const int motorPins[MTR_NUMBER] = {LEFT_FRONT, RIGHT_FRONT, RIGHT_BACK, LEFT_BACK}; // Pins connected to the ESCs

uint8_t IMUrslt; // Define the result of the data extraction from the imu

uint8_t Barslt; // Define the result of the data extraction from the barometer

unsigned long ST = 0; // Start Time [us]
float T;              // Measured Period [s]

float gyroRateOffset[3] = { 0.0 }; // Offset of the Gyroscope

// Define RC Command array
int remoteController[CHANNEL_NUMBER] = { 0 }; // Data from the RC {Roll, Pitch, Thrust, Yaw}

// Define angular rate reference array
float rateReference[DEGREES_OF_CONTROL] = { 0.0 };

// Define the feedback rate measurements array
float rateMeasurement[DEGREES_OF_CONTROL] = { 0.0 };

// Define Control Signals array
float controlSignals[CHANNEL_NUMBER] = { 0 }; // Control Signals array {Roll, Pitch, Thrust, Yaw}

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

  // Initialize PPM communication with the receiver
  ppm.begin();

  // Turn on the GREEN LED light
  neopixelWrite(LED_PIN, 0, LED_BRIGHTNESS, 0);

  /***********************************************************
  ============================================================
                  PID Controllers Section
  ============================================================
  ************************************************************
  */

  /* =================== Rate Controllers =================== */
  /*
    =========================================================
    ------------ Roll & Pitch rates Controllers -------------
    =========================================================
  */
    for(int i = 0; i < 2; i++)
    {
      pid[i].Kp     = ROLL_RATE_AND_PITCH_RATE_PROPORTIONAL_GAIN;
      pid[i].Ki     = ROLL_RATE_AND_PITCH_RATE_INTEGRAL_GAIN;
      pid[i].Kd     = ROLL_RATE_AND_PITCH_RATE_DERIVATIVE_GAIN;
      pid[i].tau    = 1.5f;
      pid[i].limMin = ROLL_AND_PITCH_MIN_LIMIT;
      pid[i].limMax = ROLL_AND_PITCH_MAX_LIMIT;
      PIDController_Init(&pid[i], SAMPLING_PERIOD);
    }
  /*
    =========================================================
    ------------------ Yaw rate Controllers -----------------
    =========================================================
  */
    pid[3].Kp     = YAW_RATE_PROPORTIONAL_GAIN;
    pid[3].Ki     = YAW_RATE_INTEGRAL_GAIN;
    pid[3].Kd     = YAW_RATE_DERIVATIVE_GAIN;
    pid[3].tau    = 1.5f;
    pid[3].limMin = YAW_MIN_LIMIT;
    pid[3].limMax = YAW_MAX_LIMIT;
    PIDController_Init(&pid[3], SAMPLING_PERIOD);
    /* =================== Vertical Controllers =================== */
    /*
      =========================================================
      ------------- Vertical velocity Controller --------------
      =========================================================
    */
    pid[2].Kp     = VERTICAL_VELOCITY_PROPORTIONAL_GAIN;
    pid[2].Ki     = VERTICAL_VELOCITY_INTEGRAL_GAIN;
    pid[2].Kd     = VERTICAL_VELOCITY_DERIVATIVE_GAIN;
    pid[2].tau    = 1.5f;
    pid[2].limMin = VERTICAL_V_MIN_LIMIT;
    pid[2].limMax = VERTICAL_V_MAX_LIMIT;
    PIDController_Init(&pid[2], SAMPLING_PERIOD);
/*====================================================================================*/
/**************************************************************************************/
/*====================================================================================*/
  // Begin the communication with the barometer
  while ( ERR_OK != (Barslt = barometer.begin()) ) {

    // Turn on the RED LED light
    neopixelWrite(LED_PIN, LED_BRIGHTNESS, 0, 0);

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
    // Turn on the RED LED light
    neopixelWrite(LED_PIN, LED_BRIGHTNESS, 0, 0);
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

  for (int i = 0; i < 8; i++) {
    RCFilter_Init(&lpFRC[i], RC_LOW_PASS_FLTR_CUTOFF_7HZ, SAMPLING_PERIOD); // Initialize the RCFilter fc = 5 Hz ; Ts = 0.01 s
  }

  // Initialize the complementary filter
  ComplementaryFilter_Init(&CF, COMP_FLTR_ALPHA);

  // Initialize the 2D complementary filter
  ComplementaryFilter2D_Init(&CF2, COMP_FLTR_2D_ALPHA);
  // Setup the interrupt for the barometer
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_1_MCU_PIN), barometerInterrupt, CHANGE); // Execute the ISR on interrupt level change

  // Calibrate the gyroscope
  Serial.println("BMI160: Calibrating");
  CalibrateGyroscope(GYRO_CALIBRATION_SAMPLES_400, gyroRateOffset);

  // Initialize the ESCs
  for (int i = 0; i < MTR_NUMBER; i++) {
    Motor_Init(&motor[i], motorPins[i], 1000, 2000, 50); // Initialize the ESC
  }
  delay(5000);
}

// Section 3: Looping and realtime processing.

void loop() {
  // Dynamic Period
  T = (micros() - ST) / 1000000.0;
  ST = micros();

  // Receive the informations from the receiver
  if (ppm.available()) {
    for (int i = 0; i < CHANNEL_NUMBER; i++) {
      remoteController[i] = fminf(fmaxf(ppm.getChannelValue(i) - 1000, 0), 1000); // Read channel values from the reciever
    }
  }

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

    // Turn on the GREEN LED light
    neopixelWrite(LED_PIN, 0, LED_BRIGHTNESS, 0);

    IMUrslt = 1;

    // Format and offset the accelerometer data
    offset(accelGyroData_int, accelGyroData);

    // Substract the offsets from the Gyro readings
    for (byte i = 0; i < 3; i++) {
      accelGyroData[i] -= gyroRateOffset[i];
    }

    // Initialize the RC low pass filters
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
    // Turn on the RED LED light
    neopixelWrite(LED_PIN, LED_BRIGHTNESS, 0, 0);
    Serial.print("BMI160: Data reading error");
    Serial.println();
  }
  /*
      Takes the data from the IMU and the baromter and mix them using a ratio alpha and outputs the vertical velocity of the quadcopter.
  */
  verticalVelocity = RCFilter_Update(&lpFRC[7], ComplementaryFilter2D_Update(&CF2, accelGyroData, eulerAngles, altitude, T));

  // Mapping the measurements to the rateMeasurement array
  /*
    * Rearranging the measurements in the order [roll, pitch, vertical velocity, yaw]
    * Mapping the measurements from open intervals to set intervals between -500 and 500 (except vertical velocity and yaw are from 0 to 1000)
    */
  for(int i = 0; i < DEGREES_OF_CONTROL - 2; i++)
  {                             /* Go to definition for better explanation */
    rateMeasurement[i] = accelGyroData[i] * THOUSAND_OVER_THIRTY;
  }
  rateMeasurement[2] = verticalVelocity * THOUSAND_OVER_TWENTY + HALF_INTERVAL;
  rateMeasurement[3] = accelGyroData[2] * THOUSAND_OVER_ELEVEN + HALF_INTERVAL;

  /*
  ++++++++++++++++++++++++++++++++++++++ Update the control system ++++++++++++++++++++++++++++++++++++++
  */
 for (int i = 0; i < DEGREES_OF_CONTROL; i++) {
    // Angular control of the drone
    rateReference[i] = (i < 2)?(ANGULAR_GAIN * (remoteController[i] - eulerAngles[i] * THOUSAND_OVER_PI - HALF_INTERVAL)):remoteController[i]; // Multiplying the angular error with the angular gain to control the angle
    rateReference[i] = (i < 2)?fmin(fmax(rateReference[i], -HALF_INTERVAL), HALF_INTERVAL):rateReference[i];                                   // Clamping the output

    // Updating the PID Controllers
    controlSignals[i]  = PIDController_Update(&pid[i], rateReference[i], rateMeasurement[i]);
  }
 /*-------------------------------------------------------------------------------------------*/

  // Command the individual motors using the Motor Mixing Algorithm
  MMA(motor, controlSignals, MTR_NUMBER, HOVERING_THROTTLE);

  // Set the motor throttle
  for (int i = 0; i < MTR_NUMBER; i++) {
    setMotorThrottle(&motor[i]);
  }

  // Data logging
  Serial.print(accelGyroData[3]);Serial.print(", \t");
  Serial.print(accelGyroData[4]);Serial.print(", \t");
  Serial.print(accelGyroData[5]);Serial.print(", \t");
  Serial.print(T);Serial.print(", \t");
  Serial.print(altitude);Serial.print(", \t");
  Serial.print(verticalVelocity);Serial.println();

  // Delay the loop until the period finishes
  while ((micros() - ST) / 1000000.0 <= 0.01);
}
