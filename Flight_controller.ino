#include "./include/BMI160.h"

BMI160 bmi160;
const int8_t addr = 0x68;

#define G_MPS2 9.81000000000000000000f // g
int alpha = 0.75;

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
  int16_t filteredAccelGyro[6]={0};
  //get both accel and gyro data from bmi160
  //parameter accelGyro is the pointer to store the data
  rslt = bmi160.getAccelGyroData(accelGyro);
  if(rslt == 0){
    accelGyro[0] = (accelGyro[0] + 9) / 16.4;
    accelGyro[1] = (accelGyro[1] - 4) / 16.4;
    accelGyro[2] = (accelGyro[2] - 7) / 16.4;
    accelGyro[3] = ((accelGyro[3] / 16384.0) - 0.03);
    accelGyro[4] = ((accelGyro[4] / 16384.0) + 0.03);
    accelGyro[5] = ((accelGyro[5] / 16384.0) - 0.03);
    for(int i = 0; i < 6; i++)
    {
      filteredAccelGyro[i] = alpha * accelGyro[i] + (1 - alpha) * filteredAccelGyro[i]; // EMA Filter
      Serial.print(filteredAccelGyro[i]);Serial.print("\t");
    }
    Serial.println();
  }else{
    Serial.println("err");
  }
  delay(50);
}

/*

*/
