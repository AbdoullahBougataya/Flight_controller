// The PID Autotuning algorithm was actually developed totally by Abdullah Suhail by past experience in his last project of the Automated and Controlled Water Transfer System.

#include "./include/BMI160.h"
#include "./include/RCFilter.h"
#include <Arduino_FreeRTOS.h>
#include <math.h>
BMI160 imu; // Declaring the imu object

RCFilter lpFRC[6]; // Declaring the RC filter object

#define COMP_FLTR_ALPHA 0.03000000000000000000f  // Complimentary filter coefficient
#define RAD2DEG        57.29577951308232087670f  // Radians to degrees (per second)
#define G_MPS2          9.81000000000000000000f  // Gravitational acceleration (g)
#define SAMPLING_PERIOD 0.01000000000000000000f  // Sampling period of the sensor in seconds

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

// Define the tasks
void TaskBlink( void *pvParameters ); // This task makes the LED blink
void TaskPrintSensorData( void *pvParameters ); // This task prints the euler angles to the serial monitor

// Functions
void complementaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, float dt);
void AccelGyroISR(); // This is the Interrupt Service Routine for retrieving data from the sensor

//PID AUTOTUNING SECTION
 //1- PID Parameters 
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.01;
 //2- PID Variables
float setpoint = 0.0;     
float input = 0.0;        
float error = 0.0;        
float previousError = 0.0;
float integral = 0.0;     
float derivative = 0.0;   
float output = 0.0;

 //3- PWM MAX AND MINIMUM LIMITS
float outputMin = 0.0;
float outputMax = 255.0;
float maxPWM = 0.30 * outputMax;
 
 //4- Anti-windup limits 
float output = abs(Kp * error + Ki * integral + Kd * derivative);
if (output < maxPWM && output > outputMin) {
  integral += error; // Only accumulate integral if output is within range
}
 //5- Failsafe Mechanism
float FailsafePWM = .15 * outputMax;
bool FailsafeAct = false;

 //6- Time Tracking Logic
unsigned long PastTime = 0;
unsigned long PresentTime = 0;
float TimeTaken = 0;

 //7- Motor control PWM output pins (I just chose random numbers)
int motor1Pin = 9;
int motor2Pin = 10;
int motor3Pin = 11;
int motor4Pin = 12;



// Section 2: Initialization & setup section.

void setup() {

  // Initiating the Tasks in FreeRTOS (Change 128 to change the stack size if stack overflow is encountered)
  xTaskCreate(TaskBlink, "Blink", 128, NULL, 1, NULL);
  xTaskCreate(TaskPrintSensorData, "PrintSensorData", 128, NULL, 2, NULL);

  // initialize serial communication at 115200 bits per second:
  Serial.begin(115200);
  vTaskDelay(100 / portTICK_PERIOD_MS); // Use vTaskDelay instead of delay

  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);
  analogWrite(motor1Pin, 0);
  analogWrite(motor2Pin, 0);
  analogWrite(motor3Pin, 0);
  analogWrite(motor4Pin, 0);

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
  attachInterrupt(digitalPinToInterrupt(2), AccelGyroISR, RISING);

  for (int i = 0; i < 6; i++) {
    RCFilter_Init(&lpFRC[i], 5.0f, 10.0f); // Initialize the RCFilter fc = 5 Hz ; Ts = 10 ms
  }

  float gyroRateCumulativeOffset[3] = { 0.0 }; // Define a temporary variable to sum the offsets

  // For five seconds the gyroscope will be calibrating (make sure you put it on a flat surface)
  for (int i = 0; i < 200; i++) {

    // Initialize sensor data arrays
    memset(accelGyro, 0, sizeof(accelGyro));
    memset(accelGyroData, 0, sizeof(accelGyroData));

    // Get both accel and gyro data from the BMI160
    // Parameter accelGyro is the pointer to store the data
    imu.getAccelGyroData(accelGyro);

    // Formatting the data
    offset(accelGyro, accelGyroData);
    for (byte j = 0; j < 3; j++) {
      gyroRateCumulativeOffset[j] += accelGyroData[j]; // Accumulating the gyroscope error
    }
    vTaskDelay(1 / portTICK_PERIOD_MS); // Use vTaskDelay instead of delay
  }
  // Calculate the average offset
  for (byte i = 0; i < 3; i++) {
    gyroRateOffset[i] = gyroRateCumulativeOffset[i] / 200;
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

      for (int i = 0; i < 6; i++){
        accelGyroData[i] = RCFilter_Update(&lpFRC[i], accelGyroData[i]); // Update the RCFilter
      }

      /*
        A complimentary filter is a premitive technique of sensor fusion
        to use both the accelerometer and the gyroscope to predict the
        euler angles (phi: roll, theta: pitch)
      */
      complementaryFilter(accelGyroData, phiHat_rad, thetaHat_rad, 10.0f); // This function transform the gyro rates and the Accelerometer angles into equivalent euler angles
    }
  }
}

// Section 4: Tasks definitions.

// This task makes the Arduino LED blink
void TaskBlink(void *pvParameters)
{
  (void) pvParameters;

  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(13, HIGH); // turn the LED on (HIGH is the voltage level)
    vTaskDelay(100 / portTICK_PERIOD_MS); // wait for the tenth of a second
    digitalWrite(13, LOW); // turn the LED off by making the voltage LOW
    vTaskDelay(100 / portTICK_PERIOD_MS); // wait for the tenth of a second
  }
}

// This task prints the euler angles to the serial monitor
void TaskPrintSensorData( void *pvParameters )
{
  (void) pvParameters;
  for (;;)
  {
    // Print the euler angles to the serial monitor
    Serial.print(phiHat_rad * RAD2DEG);Serial.print("\t");
    Serial.print(thetaHat_rad * RAD2DEG);Serial.print("\t");
    Serial.println();
  }
}

// Section 5: Function declarations.

// Accelerometer and Gyroscope interrupt service routine
void AccelGyroISR() {
  dataReady = true;
}

// Complimentary filter (Check Phil's Lab video for more details)
void complementaryFilter(float* filteredAccelGyro, float &phiHat_rad, float &thetaHat_rad, float dt) {
  // Using gravity to estimate the Euler angles
  float phiHat_acc_rad = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]);                 // Roll estimate
  float thetaHat_acc_rad = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f));  // Pitch estimate

  // Using gyroscope to estimate the euler rates (Transforming body rates to euler rates)
  float phiDot_rps   = (sinf(phiHat_rad) * filteredAccelGyro[1] + cosf(phiHat_rad) * filteredAccelGyro[2]) * tanf(thetaHat_rad) + filteredAccelGyro[0];  // Roll rate (rad/s)
  float thetaDot_rps =  cosf(phiHat_rad) * filteredAccelGyro[1] - sinf(phiHat_rad) * filteredAccelGyro[2];                                               // Pitch rate (rad/s)

  // Complementary filter implementation (Just like mixing the data from the gyroscope and the accelerometer with COMP_FLTR_ALPHA proportions)
  phiHat_rad = COMP_FLTR_ALPHA * phiHat_acc_rad + (1.0f - COMP_FLTR_ALPHA) * (phiHat_rad + (dt / 1000.0) * phiDot_rps);          // Roll estimate
  thetaHat_rad = COMP_FLTR_ALPHA * thetaHat_acc_rad + (1.0f - COMP_FLTR_ALPHA) * (thetaHat_rad + (dt / 1000.0) * thetaDot_rps);  // Pitch estimate

This is my own code developed by Abdullah Suhail I made it intentionally so that an error can occur while compiling the code because some functions weren't clear to me.

  // PID parameters
float Kp = 1.0;
float Ki = 0.1;
float Kd = 0.01;

// PID variables
float setpoint = 0.0;     
float input = 0.0;        
float error = 0.0;        
float previousError = 0.0;
float integral = 0.0;     
float derivative = 0.0;   
float output = 0.0;

// Anti-windup limits for integral
float integralMin = -10.0;
float integralMax = 10.0;

// Output PWM limits
float outputMin = 0.0;
float outputMax = 255.0;

// PWM limits for normal operation and failsafe
float maxPWM = 0.30 * outputMax;  // 30% of max PWM
float failsafePWM = 0.15 * outputMax;  // 15% of max PWM

// IMU (MPU6050)
MPU6050 imu;

// Time tracking
unsigned long previousTime = 0;
unsigned long currentTime = 0;
float elapsedTime = 0.0;

// Motor control PWM output pins
int motor1Pin = 9;
int motor2Pin = 10;
int motor3Pin = 11;
int motor4Pin = 12;

// Variable to simulate or detect failsafe condition
bool failsafeActive = false; // Set this based on system health check or failure condition

void setup() {
  Serial.begin(115200);
  
  // Initialize IMU
  Wire.begin();
  imu.initialize();

  // Motor pins setup
  pinMode(motor1Pin, OUTPUT);
  pinMode(motor2Pin, OUTPUT);
  pinMode(motor3Pin, OUTPUT);
  pinMode(motor4Pin, OUTPUT);

  // Start with motors off
  analogWrite(motor1Pin, 0);
  analogWrite(motor2Pin, 0);
  analogWrite(motor3Pin, 0);
  analogWrite(motor4Pin, 0);
}

void loop() {
  // Read IMU data
  imu.getAcceleration(&ax, &ay, &az);
  imu.getRotation(&gx, &gy, &gz);

  // Calculate pitch, roll, and yaw from accelerometer and gyroscope
  float pitch = atan2(ay, az) * 180 / PI;
  float roll = atan2(ax, az) * 180 / PI;

  // Use the PID controller to calculate the output for stabilization
  // For simplicity, use pitch or roll as the input (you can expand to both)
  input = pitch;  // or roll depending on the axis you're controlling
  float pidOutput = computePID(input);

  // Apply PID output to motor control signals (adjust as per quadcopter motor configuration)
  if (!failsafeActive) {
    analogWrite(motor1Pin, constrain(pidOutput, 0, maxPWM));
    analogWrite(motor2Pin, constrain(pidOutput, 0, maxPWM));
    analogWrite(motor3Pin, constrain(pidOutput, 0, maxPWM));
    analogWrite(motor4Pin, constrain(pidOutput, 0, maxPWM));
  } else {
    // If failsafe is active, set motors to 15% of max PWM
    analogWrite(motor1Pin, failsafePWM);
    analogWrite(motor2Pin, failsafePWM);
    analogWrite(motor3Pin, failsafePWM);
    analogWrite(motor4Pin, failsafePWM);
  }

  // Health check to trigger failsafe (replace with actual logic)
  // Example condition for failsafe trigger
  if (sensorErrorDetected() || communicationLost()) {
    failsafeActive = true;
  } else {
    failsafeActive = false;
  }

  // Update time variables for next PID cycle
  currentTime = millis();
  elapsedTime = (currentTime - previousTime) / 1000.0;  // Convert ms to seconds
  previousTime = currentTime;
}

// PID controller function
float computePID(float input) {
  // Calculate error
  error = setpoint - input;

  // Proportional term
  float P = Kp * error;

  // Integral term with anti-windup
  integral += error * elapsedTime;  // Accumulate error over time
  if (integral > integralMax) {
    integral = integralMax;
  } else if (integral < integralMin) {
    integral = integralMin;
  }
  float I = Ki * integral;

  // Derivative term
  derivative = (error - previousError) / elapsedTime;
  float D = Kd * derivative;

  // Compute raw PID output
  output = P + I + D;

  // Apply output limits
  if (output > outputMax) {
    output = outputMax;
  } else if (output < outputMin) {
    output = outputMin;
  }

  // Update variables for next iteration
  previousError = error;

  return output;
}

// Placeholder for failsafe detection logic
bool sensorErrorDetected() {
  // Replace with actual error detection code (e.g., IMU failure or communication issues)
  return false;
}

bool communicationLost() {
  // Replace with actual communication loss detection code
  return false;
}

}