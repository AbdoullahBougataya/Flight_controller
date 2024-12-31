#include "BMI160.h"

BMI160 bmi160;
const int8_t addr = 0x68;

#define G_MPS2 9.81000000000000000000f // g

float raw_acc_x, raw_acc_y, raw_acc_z;
float raw_roll, raw_pitch, raw_yaw;

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
  int i = 0;
  int rslt;
  int16_t accelGyro[6]={0};

  //get both accel and gyro data from bmi160
  //parameter accelGyro is the pointer to store the data
  rslt = bmi160.getAccelGyroData(accelGyro);
  if(rslt == 0){
    raw_roll = (accelGyro[0] + 9) / 16.4;
    raw_pitch= (accelGyro[1] - 4) / 16.4;
    raw_yaw  = (accelGyro[2] - 7) / 16.4;
    raw_acc_x= ((accelGyro[3] / 16384.0) - 0.03);
    raw_acc_y= ((accelGyro[4] / 16384.0) + 0.03);
    raw_acc_z= ((accelGyro[5] / 16384.0) - 0.03);
    Serial.print("Roll:");
    Serial.print(raw_roll);
    Serial.print("\t");
    Serial.print("Pitch:");
    Serial.print(raw_pitch);
    Serial.print("\t");
    Serial.print("Yaw:");
    Serial.print(raw_yaw);
    Serial.print("\t");
    Serial.print("X-Acceleration:");
    Serial.print(raw_acc_x);
    Serial.print("\t");
    Serial.print("Y-Acceleration:");
    Serial.print(raw_acc_y);
    Serial.print("\t");
    Serial.print("Z-Acceleration:");
    Serial.print(raw_acc_z);
    Serial.println("\t");
  }else{
    Serial.println("err");
  }
  delay(50);
}
