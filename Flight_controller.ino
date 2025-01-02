#include "./include/BMI160.h"
#include <math.h>

BMI160 bmi160;
const int8_t addr = 0x68;

#define G_MPS2 9.81000000000000000000f // g
#define ALPHA 0.01000000000000000000f
#define DPS2RPS 0.01745329251994329576f // degrees to radians per second
#define RAD2DEG 57.2957795130823208767f // Radians to degrees

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
  int rslt;
  int16_t accelGyro[6]={0};
  float filteredAccelGyro[6]={0};
  float rawAccelGyro[6]={0};
  float phiHat_deg = 0.0f;
  float thetaHat_deg = 0.0f;
  float phiHat_rad = 0.0f;
  float thetaHat_rad = 0.0f;
  //get both accel and gyro data from bmi160
  //parameter accelGyro is the pointer to store the data
  rslt = bmi160.getAccelGyroData(accelGyro);
  if(rslt == 0){
    rawAccelGyro[0] = (accelGyro[0] + 9) * DPS2RPS;
    rawAccelGyro[1] = (accelGyro[1] - 4) * DPS2RPS;
    rawAccelGyro[2] = (accelGyro[2] - 7) * DPS2RPS;
    rawAccelGyro[3] = ((accelGyro[3] / 16384.0) - 0.03) * G_MPS2;
    rawAccelGyro[4] = ((accelGyro[4] / 16384.0) + 0.03) * G_MPS2;
    rawAccelGyro[5] = ((accelGyro[5] / 16384.0) - 0.03) * G_MPS2;
    for(int i = 0; i < 6; i++)
    {
      if (rawAccelGyro[i] <= 0.3 && rawAccelGyro[i] >= -0.2)
      {
        rawAccelGyro[i] = 0;
      }
      filteredAccelGyro[i] = 100 * (ALPHA * rawAccelGyro[i] + (1 - ALPHA) * filteredAccelGyro[i]); // Low-pass EMA Filter
      Serial.print(filteredAccelGyro[i]);Serial.print("\t");
    }
    // Using gravity to estimate the euler angles
    phiHat_deg = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]) * RAD2DEG;
    thetaHat_deg = asinf(filteredAccelGyro[3] / G_MPS2) * RAD2DEG;
    // Transforming Pitch, Roll and Yaw rates to euler rates
    float phiDot_rps = filteredAccelGyro[0] + sinf(phiHat_rad) * tanf(thetaHat_rad) * filteredAccelGyro[1] + cosf(phiHat_rad) * tanf(thetaHat_rad) * filteredAccelGyro[2];
    float thetaDot_rps = cosf(phiHat_rad) * filteredAccelGyro[1] - sinf(phiHat_rad) * filteredAccelGyro[2];
    // Integrating the Euler rates to get the euler angles
    phiHat_rad += ;
    Serial.print(phiHat_deg);
    Serial.print("\t");
    Serial.print(thetaHat_deg);
    Serial.print("\t");
    Serial.println();
  }else{
    Serial.println("err");
  }
  delay(50);
}
