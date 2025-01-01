#include "./include/BMI160.h"
//
BMI160 bmi160;
const int8_t addr = 0x68;

#define G_MPS2 9.81000000000000000000f // g
#define CUTOFF_FREQUENCY 1 //Cutoff frequency

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
    for(i=0;i<6;i++){
      if (i<3){
        //the first three are gyro data
        Serial.print(accelGyro[i]*3.14/180.0);Serial.print("\t");
      }else{
        //the following three data are accel data
        Serial.print(accelGyro[i]/16384.0);Serial.print("\t");
      }
    }
    print(raw_roll, raw_pitch, raw_yaw, raw_acc_x, raw_acc_y, raw_acc_z);

  }else{
    Serial.println("err");
  }
  
  delay(50);
}

void print(raw_roll, raw_pitch, raw_yaw, raw_acc_x, raw_acc_y, raw_acc_z) {
    Serial.print("Roll:\t");
    Serial.print(raw_roll);
    Serial.print("\t");
    Serial.print("Pitch:\t");
    Serial.print(raw_pitch);
    Serial.print("\t");
    Serial.print("Yaw:\t");
    Serial.print(raw_yaw);
    Serial.print("\t");
    Serial.print("X-Acceleration:\t");
    Serial.print(raw_acc_x);
    Serial.print("\t");
    Serial.print("Y-Acceleration:\t");
    Serial.print(raw_acc_y);
    Serial.print("\t");
    Serial.print("Z-Acceleration:\t");
    Serial.println(raw_acc_z);
}