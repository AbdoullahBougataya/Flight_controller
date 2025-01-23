#include "./include/BMI160.h"
#include <math.h>

#define RAD2DEG 57.2957795130823208767f   // Radians to degrees (per second)

BMI160 imu;
const int8_t addr = 0x68;

bool accelerometerDataReady = false;
bool gyroscopeDataReady = false;

// Define sensor data arrays
volatile int16_t accelerometer[3] = { 0 };
volatile int16_t gyroscope[3] = { 0 };
float rawAccelerometer[3] = { 0 };
float rawGyroscope[3] = { 0 };

void updateAccelerometer() {
  accelerometerDataReady = true;
}

void updateGyroscope() {
  gyroscopeDataReady = true;
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialize the hardware BMI160
  if (imu.softReset() != BMI160_OK) {
    Serial.println("reset false");
    while (1);
  }

  // Set and init the imu I²C address
  if (imu.I2cInit(addr) != BMI160_OK) {
    Serial.println("init false");
    while (1);
  }

  //set interrput number to int1
  if (imu.setInt(1) != BMI160_OK){
    Serial.println("set interrput fail");
    while(1);
  }

  //set interrput number to int2
  if (imu.setInt(2) != BMI160_OK){
    Serial.println("set interrput fail");
    while(1);
  }
  attachInterrupt(0, updateGyroscope, FALLING);
  attachInterrupt(1, updateAccelerometer, FALLING);
}

void loop() {
  if(accelerometerDataReady) {
    accelerometer[3] = { 0 };
    imu.getAccelData(accelerometer);
    accelerometerDataReady = false;
  }
  if(gyroscopeDataReady) {
    gyroscope[3] = { 0 };
    imu.getGyroData(gyroscope);
    gyroscopeDataReady = false;
  }
  // Reset sensor data arrays
  rawAccelerometer[3] = { 0 };
  rawGyroscope[3] = { 0 };

  Serial.print(accelerometer[0] / 16384.0);Serial.print("\t");
  Serial.print(accelerometer[1] / 16384.0);Serial.print("\t");
  Serial.print(accelerometer[2] / 16384.0);Serial.print("\t");
  Serial.println();
  delay(10);
}


