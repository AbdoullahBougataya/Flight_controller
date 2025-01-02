#include "./include/BMI160.h"
#include <math.h>

BMI160 bmi160;
const int8_t addr = 0x68;

#define G_MPS2 9.81000000000000000000f // g
#define ALPHA 0.02000000000000000000f // Filter coefficient
#define DPS2RPS 0.01745329251994329576f // degrees to radians per second
#define RAD2DEG 57.2957795130823208767f // Radians to degrees

// Define the time step
float dt = 0.0;
static unsigned long lastTime = 0;

float phiHat_deg = 0.0f;
float thetaHat_deg = 0.0f;
float phiHat_rad = 0.0f;
float thetaHat_rad = 0.0f;

void setup(){
  Serial.begin(115200);
  delay(100);

  //init the hardware bmin160
  if (bmi160.softReset() != BMI160_OK){
    Serial.println("reset false");
    while(1);
  }

  //set and init the bmi160 i2c address
  if (bmi160.Init(addr) != BMI160_OK){
    Serial.println("init false");
    while(1);
  }
}
void loop(){
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // Time in seconds
  lastTime = currentTime;
  int rslt;
  int16_t accelGyro[6]={0};
  float filteredAccelGyro[6]={0};
  float rawAccelGyro[6]={0};
  //get both accel and gyro data from bmi160
  //parameter accelGyro is the pointer to store the data
  rslt = bmi160.getAccelGyroData(accelGyro);
  if(rslt == 0){
    rawAccelGyro[0] = (accelGyro[0] + 9) * DPS2RPS;
    rawAccelGyro[1] = (accelGyro[1] - 4) * DPS2RPS;
    rawAccelGyro[2] = (accelGyro[2] - 7) * DPS2RPS;
    rawAccelGyro[3] = ((accelGyro[3] / 16384.0) - 0.03) * G_MPS2;
    rawAccelGyro[4] = ((accelGyro[4] / 16384.0) + 0.03) * G_MPS2 + 0.4;
    rawAccelGyro[5] = ((accelGyro[5] / 16384.0) - 0.03) * G_MPS2;
    for(int i = 0; i < 6; i++)
    {
      if (rawAccelGyro[i] <= 0.3 && rawAccelGyro[i] >= -0.2)
      {
        rawAccelGyro[i] = 0;
      }
      filteredAccelGyro[i] = 50 * (ALPHA * rawAccelGyro[i] + (1 - ALPHA) * filteredAccelGyro[i]); // Low-pass EMA Filter
      Serial.print(filteredAccelGyro[i]);Serial.print("\t");
    }
    // Using gravity to estimate the euler angles
    phiHat_deg = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]) * RAD2DEG; // Roll estimate
    thetaHat_deg = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f)) * RAD2DEG; // Pitch estimate
    Serial.print("Roll-estimate:");
    Serial.print(phiHat_deg);
    Serial.print("\t");
    Serial.print("Pitch-estimate:");
    Serial.print(thetaHat_deg);
    Serial.print("\t");
    // Using gyroscope to estimate the euler rates
    float phiDot_rad = filteredAccelGyro[0] + tanf(thetaHat_rad) * (sinf(phiHat_rad) * filteredAccelGyro[1] + cosf(phiHat_rad) * filteredAccelGyro[2]); // Roll rate (rad/s)
    float thetaDot_rad =                                            cosf(phiHat_rad) * filteredAccelGyro[1] - sinf(phiHat_rad) * filteredAccelGyro[2]; // Pitch rate (rad/s)
    // Euler integration of Roll and Pitch rates
    phiHat_rad += dt * phiDot_rad; // Roll estimate
    thetaHat_rad += dt * thetaDot_rad; // Pitch estimate
    Serial.print("Roll-estimate-rad:");
    Serial.print(phiHat_rad);
    Serial.print("\t");
    Serial.print("Pitch-estimate-rad:");
    Serial.print(thetaHat_rad);
    Serial.print("\t");
    Serial.println();
  }else{
    Serial.println("err");
  }
  delay(50);
}
