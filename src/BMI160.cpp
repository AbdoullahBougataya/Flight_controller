#include"../include/BMI160.h"

BMI160::BMI160()
{
  Wire.begin();
  Obmi160=(struct bmi160Dev *)malloc(sizeof(struct bmi160Dev));
  Oaccel= (struct bmi160SensorData*)malloc(sizeof(struct bmi160SensorData));
  Ogyro = (struct bmi160SensorData*)malloc(sizeof(struct bmi160SensorData));
}

int8_t BMI160::Init(int8_t i2c_addr)
{
  Obmi160->id = i2c_addr;
  return BMI160::Init(Obmi160);
}

int8_t BMI160::Init(struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t chip_id=0;
  uint8_t data=0;
  if (dev==NULL){
    return BMI160_E_NULL_PTR;
  }
  if (rslt == BMI160_OK){
    rslt = getRegs(BMI160_CHIP_ID_ADDR, &chip_id, 1, dev);
    if ((rslt == BMI160_OK)&&(chip_id==BMI160_CHIP_ID)){
      dev->any_sig_sel = eBmi160BothAnySigMotionDisabled;
      dev->chipId = chip_id;
      rslt = softReset(dev);
      if (rslt==BMI160_OK){
        rslt = setSensConf(dev);
      }
    }else{
      rslt = BMI160_E_DEV_NOT_FOUND;
    }
  }
  return rslt;
}
int8_t BMI160::softReset()
{
  int8_t rslt=BMI160_OK;
  if (Obmi160 == NULL){
    rslt = BMI160_E_NULL_PTR;
  }
  rslt = softReset(Obmi160);
  return rslt;
}

int8_t BMI160::softReset(struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t data = BMI160_SOFT_RESET_CMD;
  if (dev==NULL){
    rslt = BMI160_E_NULL_PTR;
  }
  rslt = BMI160::setRegs(BMI160_COMMAND_REG_ADDR, &data, 1, dev);
  delay(BMI160_SOFT_RESET_DELAY_MS);
  if (rslt == BMI160_OK){
    BMI160::defaultParamSettg(dev);
  }
  return rslt;
}

void BMI160::defaultParamSettg(struct bmi160Dev *dev)
{
  /* Initializing accel and gyro params with
  * default values */
  dev->accelCfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.odr = BMI160_ACCEL_ODR_100HZ;
  dev->accelCfg.power = BMI160_ACCEL_SUSPEND_MODE;
  dev->accelCfg.range = BMI160_ACCEL_RANGE_2G;
  dev->gyroCfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
  dev->gyroCfg.odr = BMI160_GYRO_ODR_100HZ;
  dev->gyroCfg.power = BMI160_GYRO_SUSPEND_MODE;
  dev->gyroCfg.range = BMI160_GYRO_RANGE_2000_DPS;

  /* To maintain the previous state of accel configuration */
  dev->prevAccelCfg = dev->accelCfg;
  /* To maintain the previous state of gyro configuration */
  dev->prevGyroCfg = dev->gyroCfg;
}

int8_t BMI160::setSensConf()
{
  return BMI160::setSensConf(Obmi160);
}

int8_t BMI160::setSensConf(struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  dev->accelCfg.odr = BMI160_ACCEL_ODR_100HZ;
  dev->accelCfg.range = BMI160_ACCEL_RANGE_2G;
  dev->accelCfg.bw = BMI160_ACCEL_BW_OSR2_AVG2;

  dev->accelCfg.power = BMI160_ACCEL_NORMAL_MODE;

  dev->gyroCfg.odr = BMI160_GYRO_ODR_100HZ;
  dev->gyroCfg.range = BMI160_GYRO_RANGE_2000_DPS;
  dev->gyroCfg.bw = BMI160_GYRO_BW_OSR2_MODE;

  dev->gyroCfg.power = BMI160_GYRO_NORMAL_MODE;


  rslt = BMI160::setAccelConf(dev);
  if (rslt == BMI160_OK) {
    rslt = BMI160::setGyroConf(dev);
    if (rslt == BMI160_OK) {
      /* write power mode for accel and gyro */
      rslt = BMI160::setPowerMode(dev);
      if (rslt == BMI160_OK)
        rslt = BMI160::checkInvalidSettg(dev);
    }
  }

  return rslt;
}

int8_t BMI160::setAccelConf(struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t data[2] = {0};
  rslt = BMI160::checkAccelConfig(data, dev);
  if (rslt == BMI160_OK) {
    rslt = BMI160::setRegs(BMI160_ACCEL_CONFIG_ADDR, &data[0], 1, dev);
    if (rslt == BMI160_OK) {
      dev->prevAccelCfg.odr = dev->accelCfg.odr;
      dev->prevAccelCfg.bw = dev->accelCfg.bw;
      delay(1);
      rslt = BMI160::setRegs(BMI160_ACCEL_RANGE_ADDR, &data[1], 1, dev);
      if (rslt == BMI160_OK){
        dev->prevAccelCfg.range = dev->accelCfg.range;
      }
    }
  }
  return rslt;
}

int8_t BMI160::checkAccelConfig(uint8_t *data, struct bmi160Dev *dev)
{
  int8_t rslt;

  /* read accel Output data rate and bandwidth */
  rslt = BMI160::getRegs(BMI160_ACCEL_CONFIG_ADDR, data, 2, dev);
  if (rslt == BMI160_OK) {
    rslt = BMI160::processAccelOdr(&data[0], dev);
    if (rslt == BMI160_OK) {
      rslt = BMI160::processAccelBw(&data[0], dev);
      if (rslt == BMI160_OK)
        rslt = BMI160::processAccelRange(&data[1], dev);
    }
  }

  return rslt;
}

int8_t BMI160::processAccelOdr(uint8_t *data,  struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t odr = 0;

  if (dev->accelCfg.odr <= BMI160_ACCEL_ODR_MAX) {
    if (dev->accelCfg.odr != dev->prevAccelCfg.odr) {
      odr = (uint8_t)dev->accelCfg.odr;
      temp = *data & ~BMI160_ACCEL_ODR_MASK;
      /* Adding output data rate */
      *data = temp | (odr & BMI160_ACCEL_ODR_MASK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160::processAccelBw(uint8_t *data, struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t bw = 0;

  if (dev->accelCfg.bw <= BMI160_ACCEL_BW_MAX) {
    if (dev->accelCfg.bw != dev->prevAccelCfg.bw) {
      bw = (uint8_t)dev->accelCfg.bw;
      temp = *data & ~BMI160_ACCEL_BW_MASK;
      /* Adding bandwidth */
      *data = temp | ((bw << 4) & BMI160_ACCEL_ODR_MASK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160::processAccelRange(uint8_t *data, struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t range = 0;

  if (dev->accelCfg.range <= BMI160_ACCEL_RANGE_MAX) {
    if (dev->accelCfg.range != dev->prevAccelCfg.range) {
      range = (uint8_t)dev->accelCfg.range;
      temp = *data & ~BMI160_ACCEL_RANGE_MASK;
      /* Adding range */
      *data = temp | (range & BMI160_ACCEL_RANGE_MASK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160::setGyroConf(struct bmi160Dev *dev)
{
  int8_t rslt;
  uint8_t data[2]={0};

  rslt = BMI160::checkGyroConfig(data, dev);

  if (rslt == BMI160_OK) {
    // Write output data rate and bandwidth
    rslt = BMI160::setRegs(BMI160_GYRO_CONFIG_ADDR, &data[0], 1, dev);
    if (rslt == BMI160_OK) {
      dev->prevGyroCfg.odr = dev->gyroCfg.odr;
      dev->prevGyroCfg.bw = dev->gyroCfg.bw;
      delay(1);
      // Write gyro range
      rslt = BMI160::setRegs(BMI160_GYRO_RANGE_ADDR, &data[1], 1, dev);
      if (rslt == BMI160_OK)
        dev->prevGyroCfg.range = dev->gyroCfg.range;
    }
  }

  return rslt;
}

int8_t BMI160::checkGyroConfig(uint8_t *data, struct bmi160Dev *dev)
{
  int8_t rslt;

  /* read gyro Output data rate and bandwidth */
  rslt = BMI160::getRegs(BMI160_GYRO_CONFIG_ADDR, data, 2, dev);
  if (rslt == BMI160_OK) {
    rslt = BMI160::processGyroOdr(&data[0], dev);
    if (rslt == BMI160_OK) {
      rslt = BMI160::processGyroBw(&data[0], dev);
      if (rslt == BMI160_OK)
        rslt = BMI160::processGyroRange(&data[1], dev);
    }
  }

  return rslt;
}

int8_t BMI160::processGyroOdr(uint8_t *data, struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t odr = 0;

  if (dev->gyroCfg.odr <= BMI160_GYRO_ODR_MAX) {
    if (dev->gyroCfg.odr != dev->prevGyroCfg.odr) {
      odr = (uint8_t)dev->gyroCfg.odr;
      temp = (*data & ~BMI160_GYRO_ODR_MASK);
      /* Adding output data rate */
      *data = temp | (odr & BMI160_GYRO_ODR_MASK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160::processGyroBw(uint8_t *data, struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t bw = 0;

  if (dev->gyroCfg.bw <= BMI160_GYRO_BW_MAX) {
    bw = (uint8_t)dev->gyroCfg.bw;
    temp = *data & ~BMI160_GYRO_BW_MASK;
    /* Adding bandwidth */
    *data = temp | ((bw << 4) & BMI160_GYRO_BW_MASK);
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160::processGyroRange(uint8_t *data, struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  uint8_t temp = 0;
  uint8_t range = 0;

  if (dev->gyroCfg.range <= BMI160_GYRO_RANGE_MAX) {
    if (dev->gyroCfg.range != dev->prevGyroCfg.range) {
      range = (uint8_t)dev->gyroCfg.range;
      temp = *data & ~BMI160_GYRO_RANGE_MSK;
      /* Adding range */
      *data = temp | (range & BMI160_GYRO_RANGE_MSK);
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160::setPowerMode(struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  rslt = BMI160::setAccelPwr(dev);
  if (rslt == BMI160_OK){
    rslt = BMI160::setGyroPwr(dev);
  }
  return rslt;
}

int8_t BMI160::setAccelPwr(struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  uint8_t data = 0;

  if ((dev->accelCfg.power >= BMI160_ACCEL_SUSPEND_MODE) &&
    (dev->accelCfg.power <= BMI160_ACCEL_LOWPOWER_MODE)) {
    if (dev->accelCfg.power != dev->prevAccelCfg.power) {
      rslt = BMI160::processUnderSampling(&data, dev);
      if (rslt == BMI160_OK) {
        /* Write accel power */
        rslt = BMI160::setRegs(BMI160_COMMAND_REG_ADDR, &dev->accelCfg.power, 1, dev);
        /* Add delay of 5 ms */
        if (dev->prevAccelCfg.power == BMI160_ACCEL_SUSPEND_MODE){
          delay(5);
        }
        dev->prevAccelCfg.power = dev->accelCfg.power;
      }
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160::processUnderSampling(uint8_t *data, struct bmi160Dev *dev)
{
  int8_t rslt;
  uint8_t temp = 0;
  uint8_t pre_filter = 0;

  rslt = BMI160::getRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
  if (rslt == BMI160_OK) {
    if (dev->accelCfg.power == BMI160_ACCEL_LOWPOWER_MODE) {
      temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
      /* Set under-sampling parameter */
      *data = temp | ((1 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK);
      /* Write data */
      rslt = setRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
      /* disable the pre-filter data in
       * low power mode */
      if (rslt == BMI160_OK)
        /* Disable the Pre-filter data*/
        rslt = BMI160::setRegs(BMI160_INT_DATA_0_ADDR, &pre_filter, 2, dev);
    } else {
      if (*data & BMI160_ACCEL_UNDERSAMPLING_MASK) {
        temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
        /* disable under-sampling parameter
        if already enabled */
        *data = temp;
        /* Write data */
        rslt = BMI160::setRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
      }
    }
  }

  return rslt;
}

int8_t BMI160::setGyroPwr(struct bmi160Dev *dev)
{
  int8_t rslt = 0;
  if ((dev->gyroCfg.power == BMI160_GYRO_SUSPEND_MODE) || (dev->gyroCfg.power == BMI160_GYRO_NORMAL_MODE)
    || (dev->gyroCfg.power == BMI160_GYRO_FASTSTARTUP_MODE)) {
    if (dev->gyroCfg.power != dev->prevGyroCfg.power) {
      /* Write gyro power */
      rslt = BMI160::setRegs(BMI160_COMMAND_REG_ADDR, &dev->gyroCfg.power, 1, dev);
      if (dev->prevGyroCfg.power == BMI160_GYRO_SUSPEND_MODE) {
        /* Delay of 81 ms */
        delay(81);
      } else if ((dev->prevGyroCfg.power == BMI160_GYRO_FASTSTARTUP_MODE)
        && (dev->gyroCfg.power == BMI160_GYRO_NORMAL_MODE)) {
        /* This delay is required for transition from
        fast-startup mode to normal mode */
        delay(10);
      } else {
        /* do nothing */
      }
      dev->prevGyroCfg.power = dev->gyroCfg.power;
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160::checkInvalidSettg( struct bmi160Dev *dev)
{
  int8_t rslt;
  uint8_t data = 0;

  // read the error reg
  rslt = BMI160::getRegs(BMI160_ERROR_REG_ADDR, &data, 1, dev);

  data = data >> 1;
  data = data & BMI160_ERR_REG_MASK;
  if (data == 1)
    rslt = BMI160_E_ACCEL_ODR_BW_INVALID;
  else if (data == 2)
    rslt = BMI160_E_GYRO_ODR_BW_INVALID;
  else if (data == 3)
    rslt = BMI160_E_LWP_PRE_FLTR_INT_INVALID;
  else if (data == 7)
    rslt = BMI160_E_LWP_PRE_FLTR_INVALID;

  return rslt;
}

int8_t BMI160::getSensorData(uint8_t type, int16_t* data)
{
  int8_t rslt=BMI160_OK;
  rslt = BMI160::getSensorData((BMI160_ACCEL_SEL | BMI160_GYRO_SEL),Oaccel, Ogyro, Obmi160);
  if(rslt == BMI160_OK){
    data[0]=Ogyro->x;
    data[1]=Ogyro->y;
    data[2]=Ogyro->z;
    data[3]=Oaccel->x;
    data[4]=Oaccel->y;
    data[5]=Oaccel->z;
  }
  return rslt;
}

int8_t BMI160::getAccelGyroData( int16_t* data)
{
  int8_t rslt = BMI160_OK;
  rslt = getSensorData((BMI160_ACCEL_SEL | BMI160_GYRO_SEL),Oaccel, Ogyro, Obmi160);
  if(rslt == BMI160_OK){
    data[0]=Ogyro->x;
    data[1]=Ogyro->y;
    data[2]=Ogyro->z;
    data[3]=Oaccel->x;
    data[4]=Oaccel->y;
    data[5]=Oaccel->z;
  }
  return rslt;
}

int8_t BMI160::getAccelGyroData( int16_t* data, uint32_t* timestamp)
{
  int8_t rslt = BMI160_OK;
  rslt = getSensorData((BMI160_ACCEL_SEL | BMI160_GYRO_SEL | BMI160_TIME_SEL),Oaccel, Ogyro, Obmi160);
  if(rslt == BMI160_OK){
    data[0]=Ogyro->x;
    data[1]=Ogyro->y;
    data[2]=Ogyro->z;
    data[3]=Oaccel->x;
    data[4]=Oaccel->y;
    data[5]=Oaccel->z;
    timestamp[0]=Oaccel->sensortime;
    timestamp[1]=Ogyro->sensortime;
  }
  return rslt;
}

int8_t BMI160::getSensorData(uint8_t select_sensor, struct bmi160SensorData *accel, struct bmi160SensorData *gyro,struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t time_sel;
  uint8_t sen_sel;
  uint8_t len = 0;

  /*Extract the sensor  and time select information*/
  sen_sel = select_sensor & BMI160_SEN_SEL_MASK;
  time_sel = ((sen_sel & BMI160_TIME_SEL) >> 2);
  sen_sel = sen_sel & (BMI160_ACCEL_SEL | BMI160_GYRO_SEL);
  if (time_sel == 1)
    len = 3;

  /* Null-pointer check */
  if (dev != NULL) {
    /* Null-pointer check */
      if ((gyro == NULL) || (accel == NULL))
        rslt = BMI160_E_NULL_PTR;
      else
        rslt = BMI160::getAccelGyroData(len, accel, gyro, dev);
  } else {
    rslt = BMI160_E_NULL_PTR;
  }

  return rslt;
}

int8_t BMI160::getAccelGyroData(uint8_t len, struct bmi160SensorData *accel, struct bmi160SensorData *gyro, struct bmi160Dev *dev)
{
  int8_t rslt;
  uint8_t idx = 0;
  uint8_t data_array[15] = {0};
  uint8_t time_0 = 0;
  uint16_t time_1 = 0;
  uint32_t time_2 = 0;
  uint8_t lsb;
  uint8_t msb;
  int16_t msblsb;

  /* read both accel and gyro sensor data
   * along with time if requested */
  rslt = BMI160::getRegs(BMI160_GYRO_DATA_ADDR, data_array, 12 + len, dev);
  if (rslt == BMI160_OK) {
    /* Gyro Data */
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    gyro->x = msblsb; /* gyro X axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    gyro->y = msblsb; /* gyro Y axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    gyro->z = msblsb; /* gyro Z axis data */

    /* Accel Data */
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->x = (int16_t)msblsb; /* accel X axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->y = (int16_t)msblsb; /* accel Y axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    accel->z = (int16_t)msblsb; /* accel Z axis data */

    if (len == 3) {
      time_0 = data_array[idx++];
      time_1 = (uint16_t)(data_array[idx++] << 8);
      time_2 = (uint32_t)(data_array[idx++] << 16);
      accel->sensortime = (uint32_t)(time_2 | time_1 | time_0);
      gyro->sensortime = (uint32_t)(time_2 | time_1 | time_0);
    } else {
      accel->sensortime = 0;
      gyro->sensortime = 0;
      ;
    }
  } else {
    rslt = BMI160_E_COM_FAIL;
  }

  return rslt;
}

int8_t BMI160::setInt()
{
  return setInt(Obmi160);
}

int8_t BMI160::setInt(struct bmi160Dev *dev)
{
  int8_t rslt=BMI160_OK;
  uint8_t data[4]={0};
  if (dev == NULL)
  {
    rslt = BMI160_E_NULL_PTR;
  } else {
    data[0] = BMI160_DATA_RDY_INT_EN_MASK;
    rslt = setRegs(BMI160_INT_ENABLE_1_ADDR, &data[0], 1, dev);
    if (rslt == BMI160_OK)
    {
      data[1] = BMI160_INT1_DATA_READY_MASK;
      rslt = setRegs(BMI160_INT_MAP_1_ADDR, &data[1], 1, dev);
      if (rslt == BMI160_OK)
      {
        data[2] = BMI160_INT1_PULL_PUSH_ACTIVE_HIGH_EDGE_TRIG;
        rslt = setRegs(BMI160_INT_OUT_CTRL_ADDR, &data[2], 1, dev);
        if (rslt == BMI160_OK)
        {
          data[3] = BMI160_INT1_NON_LATCHED_MASK;
          rslt = setRegs(BMI160_INT_LATCH_ADDR, &data[3], 1, dev);
        }
      }
    }
  }
  return rslt;
}

int8_t BMI160::getRegs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160Dev *dev)
{

  int8_t rslt = BMI160_OK;
  //Null-pointer check
  if (dev == NULL) {
    rslt = BMI160_E_NULL_PTR;
  } else {
    //Configuring reg_addr for I²C Interface
    rslt = BMI160::I2cGetRegs(dev, reg_addr, data, len);
    delay(1);
    if (rslt != BMI160_OK){
      rslt = BMI160_E_COM_FAIL;
    }
  }

  return rslt;
}

int8_t BMI160::I2cGetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  Wire.beginTransmission(dev->id);
  Wire.write(reg_addr);
  Wire.endTransmission(true);
  delay(10);
  Wire.requestFrom(dev->id,len);

  for(int i = 0; i < len; i++){
    data[i]=Wire.read();
    delay(1);
  }
  return BMI160_OK;
}

int8_t BMI160::setRegs(uint8_t reg_addr, uint8_t *data, uint16_t len, struct bmi160Dev *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t count = 0;
  //Null-pointer check
  if (dev == NULL) {
    rslt = BMI160_E_NULL_PTR;
  } else {
    //Configuring reg_addr for I²C Interface
    rslt = BMI160::I2cSetRegs(dev,reg_addr,data,len);
    delay(1);

    if (rslt != BMI160_OK)
      rslt = BMI160_E_COM_FAIL;
  }

  return rslt;
}

int8_t BMI160::I2cSetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if ((dev->prevAccelCfg.power == BMI160_ACCEL_NORMAL_MODE)||(dev->prevGyroCfg.power == BMI160_GYRO_NORMAL_MODE)){
    Wire.beginTransmission(dev->id);
    Wire.write(reg_addr);
    for(int i = 0; i < len; i++){
      Wire.write(data[i]);
      delay(1);
    }
    Wire.endTransmission(true);
  }else{
    for(int i = 0; i < len; i++){
      Wire.beginTransmission(dev->id);
      Wire.write(reg_addr);
      Wire.write(data[i]);

      Wire.endTransmission(true);
      delay(1);
    }
  }
  return BMI160_OK;
}
// Offsets of the sensor data
void offset(int16_t* accelGyro, float* rawAccelGyro) {
  rawAccelGyro[0] = (accelGyro[0] / 16.4) * DPS2RPS;
  rawAccelGyro[1] = (accelGyro[1] / 16.4) * DPS2RPS;
  rawAccelGyro[2] = (accelGyro[2] / 16.4) * DPS2RPS;
  rawAccelGyro[3] = (accelGyro[3] / 16384.0) * G_MPS2;
  rawAccelGyro[4] = ((accelGyro[4] / 16384.0) + BMI160_ACC_Y_OFFSET) * G_MPS2; // Offset added
  rawAccelGyro[5] = (accelGyro[5] / 16384.0) * G_MPS2;
}
