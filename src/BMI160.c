#include"../include/BMI160.h"

int8_t BMI160_Init(BMI160 *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t chip_id=0;
  if (dev==NULL){
    return BMI160_E_NULL_PTR;
  }
  if (rslt == BMI160_OK){
    rslt = BMI160_getRegs(BMI160_CHIP_ID_ADDR, &chip_id, 1, dev);
    if ((rslt == BMI160_OK)&&(chip_id==BMI160_CHIP_ID)){
      dev->chipId = chip_id;
      rslt = BMI160_softReset(dev);
      if (rslt==BMI160_OK){
        rslt = BMI160_setSensConf(dev);
      }
    }else{
      rslt = BMI160_E_DEV_NOT_FOUND;
    }
  }
  return rslt;
}

int8_t BMI160_softReset(BMI160 *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t data = BMI160_SOFT_RESET_CMD;
  if (dev==NULL){
    rslt = BMI160_E_NULL_PTR;
  }
  rslt = BMI160_setRegs(BMI160_COMMAND_REG_ADDR, &data, 1, dev);
  vTaskDelay(BMI160_SOFT_RESET_DELAY_MS / portTICK_PERIOD_MS);
  if (rslt == BMI160_OK){
    BMI160_defaultParamSettg(dev);
  }
  return rslt;
}

void BMI160_defaultParamSettg(BMI160 *dev)
{
  /* Initializing accel and gyro params with
  * default values */
  dev->accelCfg.bw = BMI160_ACCEL_BW_NORMAL_AVG4;
  dev->accelCfg.odr = BMI160_ACCEL_ODR_100HZ;
  dev->accelCfg.power = BMI160_ACCEL_SUSPEND_MODE;
  dev->accelCfg.range = BMI160_ACCEL_RANGE_4G;
  dev->gyroCfg.bw = BMI160_GYRO_BW_NORMAL_MODE;
  dev->gyroCfg.odr = BMI160_GYRO_ODR_100HZ;
  dev->gyroCfg.power = BMI160_GYRO_SUSPEND_MODE;
  dev->gyroCfg.range = BMI160_GYRO_RANGE_2000_DPS;

  /* To maintain the previous state of accel configuration */
  dev->prevAccelCfg = dev->accelCfg;
  /* To maintain the previous state of gyro configuration */
  dev->prevGyroCfg = dev->gyroCfg;
}

int8_t BMI160_setSensConf(BMI160 *dev)
{
  int8_t rslt = BMI160_OK;
  dev->accelCfg.odr = BMI160_ACCEL_ODR_400HZ;
  dev->accelCfg.range = BMI160_ACCEL_RANGE_2G;
  dev->accelCfg.bw = BMI160_ACCEL_BW_OSR2_AVG2;

  dev->accelCfg.power = BMI160_ACCEL_NORMAL_MODE;

  dev->gyroCfg.odr = BMI160_GYRO_ODR_400HZ;
  dev->gyroCfg.range = BMI160_GYRO_RANGE_2000_DPS;
  dev->gyroCfg.bw = BMI160_GYRO_BW_OSR2_MODE;

  dev->gyroCfg.power = BMI160_GYRO_NORMAL_MODE;


  rslt = BMI160_setAccelConf(dev);
  if (rslt == BMI160_OK) {
    rslt = BMI160_setGyroConf(dev);
    if (rslt == BMI160_OK) {
      /* write power mode for accel and gyro */
      rslt = BMI160_setPowerMode(dev);
      if (rslt == BMI160_OK)
        rslt = BMI160_checkInvalidSettg(dev);
    }
  }

  return rslt;
}

int8_t BMI160_setAccelConf(BMI160 *dev)
{
  int8_t rslt = BMI160_OK;
  uint8_t data[2] = {0};
  rslt = BMI160_checkAccelConfig(data, dev);
  if (rslt == BMI160_OK) {
    rslt = BMI160_setRegs(BMI160_ACCEL_CONFIG_ADDR, &data[0], 1, dev);
    if (rslt == BMI160_OK) {
      dev->prevAccelCfg.odr = dev->accelCfg.odr;
      dev->prevAccelCfg.bw = dev->accelCfg.bw;
      vTaskDelay(1 / portTICK_PERIOD_MS);
      rslt = BMI160_setRegs(BMI160_ACCEL_RANGE_ADDR, &data[1], 1, dev);
      if (rslt == BMI160_OK){
        dev->prevAccelCfg.range = dev->accelCfg.range;
      }
    }
  }
  return rslt;
}

int8_t BMI160_checkAccelConfig(uint8_t *data, BMI160 *dev)
{
  int8_t rslt;

  /* read accel Output data rate and bandwidth */
  rslt = BMI160_getRegs(BMI160_ACCEL_CONFIG_ADDR, data, 2, dev);
  if (rslt == BMI160_OK) {
    rslt = BMI160_processAccelOdr(&data[0], dev);
    if (rslt == BMI160_OK) {
      rslt = BMI160_processAccelBw(&data[0], dev);
      if (rslt == BMI160_OK)
        rslt = BMI160_processAccelRange(&data[1], dev);
    }
  }

  return rslt;
}

int8_t BMI160_processAccelOdr(uint8_t *data,  BMI160 *dev)
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

int8_t BMI160_processAccelBw(uint8_t *data, BMI160 *dev)
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

int8_t BMI160_processAccelRange(uint8_t *data, BMI160 *dev)
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

int8_t BMI160_setGyroConf(BMI160 *dev)
{
  int8_t rslt;
  uint8_t data[2]={0};

  rslt = BMI160_checkGyroConfig(data, dev);

  if (rslt == BMI160_OK) {
    // Write output data rate and bandwidth
    rslt = BMI160_setRegs(BMI160_GYRO_CONFIG_ADDR, &data[0], 1, dev);
    if (rslt == BMI160_OK) {
      dev->prevGyroCfg.odr = dev->gyroCfg.odr;
      dev->prevGyroCfg.bw = dev->gyroCfg.bw;
      vTaskDelay(1 / portTICK_PERIOD_MS);
      // Write gyro range
      rslt = BMI160_setRegs(BMI160_GYRO_RANGE_ADDR, &data[1], 1, dev);
      if (rslt == BMI160_OK)
        dev->prevGyroCfg.range = dev->gyroCfg.range;
    }
  }

  return rslt;
}

int8_t BMI160_checkGyroConfig(uint8_t *data, BMI160 *dev)
{
  int8_t rslt;

  /* read gyro Output data rate and bandwidth */
  rslt = BMI160_getRegs(BMI160_GYRO_CONFIG_ADDR, data, 2, dev);
  if (rslt == BMI160_OK) {
    rslt = BMI160_processGyroOdr(&data[0], dev);
    if (rslt == BMI160_OK) {
      rslt = BMI160_processGyroBw(&data[0], dev);
      if (rslt == BMI160_OK)
        rslt = BMI160_processGyroRange(&data[1], dev);
    }
  }

  return rslt;
}

int8_t BMI160_processGyroOdr(uint8_t *data, BMI160 *dev)
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

int8_t BMI160_processGyroBw(uint8_t *data, BMI160 *dev)
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

int8_t BMI160_processGyroRange(uint8_t *data, BMI160 *dev)
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

int8_t BMI160_setPowerMode(BMI160 *dev)
{
  int8_t rslt = 0;
  rslt = BMI160_setAccelPwr(dev);
  if (rslt == BMI160_OK){
    rslt = BMI160_setGyroPwr(dev);
  }
  return rslt;
}

int8_t BMI160_setAccelPwr(BMI160 *dev)
{
  int8_t rslt = 0;
  uint8_t data = 0;

  if ((dev->accelCfg.power >= BMI160_ACCEL_SUSPEND_MODE) &&
    (dev->accelCfg.power <= BMI160_ACCEL_LOWPOWER_MODE)) {
    if (dev->accelCfg.power != dev->prevAccelCfg.power) {
      rslt = BMI160_processUnderSampling(&data, dev);
      if (rslt == BMI160_OK) {
        /* Write accel power */
        rslt = BMI160_setRegs(BMI160_COMMAND_REG_ADDR, &dev->accelCfg.power, 1, dev);
        /* Add delay of 5 ms */
        if (dev->prevAccelCfg.power == BMI160_ACCEL_SUSPEND_MODE){
          vTaskDelay(5 / portTICK_PERIOD_MS);
        }
        dev->prevAccelCfg.power = dev->accelCfg.power;
      }
    }
  } else {
    rslt = BMI160_E_OUT_OF_RANGE;
  }

  return rslt;
}

int8_t BMI160_processUnderSampling(uint8_t *data, BMI160 *dev)
{
  int8_t rslt;
  uint8_t temp = 0;
  uint8_t pre_filter = 0;

  rslt = BMI160_getRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
  if (rslt == BMI160_OK) {
    if (dev->accelCfg.power == BMI160_ACCEL_LOWPOWER_MODE) {
      temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
      /* Set under-sampling parameter */
      *data = temp | ((1 << 7) & BMI160_ACCEL_UNDERSAMPLING_MASK);
      /* Write data */
      rslt = BMI160_setRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
      /* disable the pre-filter data in
       * low power mode */
      if (rslt == BMI160_OK)
        /* Disable the Pre-filter data*/
        rslt = BMI160_setRegs(BMI160_INT_DATA_0_ADDR, &pre_filter, 2, dev);
    } else {
      if (*data & BMI160_ACCEL_UNDERSAMPLING_MASK) {
        temp = *data & ~BMI160_ACCEL_UNDERSAMPLING_MASK;
        /* disable under-sampling parameter
        if already enabled */
        *data = temp;
        /* Write data */
        rslt = BMI160_setRegs(BMI160_ACCEL_CONFIG_ADDR, data, 1, dev);
      }
    }
  }

  return rslt;
}

int8_t BMI160_setGyroPwr(BMI160 *dev)
{
  int8_t rslt = 0;
  if ((dev->gyroCfg.power == BMI160_GYRO_SUSPEND_MODE) || (dev->gyroCfg.power == BMI160_GYRO_NORMAL_MODE)
    || (dev->gyroCfg.power == BMI160_GYRO_FASTSTARTUP_MODE)) {
    if (dev->gyroCfg.power != dev->prevGyroCfg.power) {
      /* Write gyro power */
      rslt = BMI160_setRegs(BMI160_COMMAND_REG_ADDR, &dev->gyroCfg.power, 1, dev);
      if (dev->prevGyroCfg.power == BMI160_GYRO_SUSPEND_MODE) {
        /* Delay of 81 ms */
        vTaskDelay(81 / portTICK_PERIOD_MS);
      } else if ((dev->prevGyroCfg.power == BMI160_GYRO_FASTSTARTUP_MODE)
        && (dev->gyroCfg.power == BMI160_GYRO_NORMAL_MODE)) {
        /* This delay is required for transition from
        fast-startup mode to normal mode */
        vTaskDelay(10 / portTICK_PERIOD_MS);
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

int8_t BMI160_checkInvalidSettg( BMI160 *dev)
{
  int8_t rslt;
  uint8_t data = 0;

  // read the error reg
  rslt = BMI160_getRegs(BMI160_ERROR_REG_ADDR, &data, 1, dev);

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

int8_t BMI160_getAccelGyroData(BMI160 *dev, int16_t *data)
{
  int8_t rslt;
  uint8_t idx = 0;
  uint8_t data_array[15] = {0};
  uint8_t lsb;
  uint8_t msb;
  int16_t msblsb;

  /* read both accel and gyro sensor data
   * along with time if requested */
  rslt = BMI160_getRegs(BMI160_GYRO_DATA_ADDR, data_array, 12, dev);
  if (rslt == BMI160_OK) {
    /* Gyro Data */
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    data[0] = msblsb; /* gyro X axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    data[1] = msblsb; /* gyro Y axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    data[2] = msblsb; /* gyro Z axis data */

    /* Accel Data */
    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    data[3] = (int16_t)msblsb; /* accel X axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    data[4] = (int16_t)msblsb; /* accel Y axis data */

    lsb = data_array[idx++];
    msb = data_array[idx++];
    msblsb = (int16_t)((msb << 8) | lsb);
    data[5] = (int16_t)msblsb; /* accel Z axis data */

  } else {
    rslt = BMI160_E_COM_FAIL;
  }

  return rslt;
}

int8_t BMI160_setInt(BMI160 *dev)
{
  int8_t rslt=BMI160_OK;
  uint8_t data[4]={0};
  if (dev == NULL)
  {
    rslt = BMI160_E_NULL_PTR;
  } else {
    data[0] = BMI160_DATA_RDY_INT_EN_MASK;
    rslt = BMI160_setRegs(BMI160_INT_ENABLE_1_ADDR, &data[0], 1, dev);
    if (rslt == BMI160_OK)
    {
      data[1] = BMI160_INT1_DATA_READY_MASK;
      rslt = BMI160_setRegs(BMI160_INT_MAP_1_ADDR, &data[1], 1, dev);
      if (rslt == BMI160_OK)
      {
        data[2] = BMI160_INT1_PULL_PUSH_ACTIVE_HIGH_EDGE_TRIG;
        rslt = BMI160_setRegs(BMI160_INT_OUT_CTRL_ADDR, &data[2], 1, dev);
        if (rslt == BMI160_OK)
        {
          data[3] = BMI160_INT1_NON_LATCHED_MASK;
          rslt = BMI160_setRegs(BMI160_INT_LATCH_ADDR, &data[3], 1, dev);
        }
      }
    }
  }
  return rslt;
}

// Offsets of the sensor data
int8_t BMI160_offset(int16_t* accelGyro, float* rawAccelGyro) {
  int8_t rslt = BMI160_OK;
  rawAccelGyro[0] = (accelGyro[0] / 16.4) * DPS2RPS;
  rawAccelGyro[1] = (accelGyro[1] / 16.4) * DPS2RPS;
  rawAccelGyro[2] = (accelGyro[2] / 16.4) * DPS2RPS;
  rawAccelGyro[3] = (accelGyro[3] / 8192.0) * G_MPS2;
  rawAccelGyro[4] = ((accelGyro[4] / 8192.0) + BMI160_ACC_Y_OFFSET) * G_MPS2; // Offset added
  rawAccelGyro[5] = (accelGyro[5] / 8192.0) * G_MPS2;
  return rslt;
}

int8_t BMI160_getRegs(uint8_t reg_addr, uint8_t *data, uint16_t len, BMI160 *dev)
{

  int8_t rslt = BMI160_OK;
  //Null-pointer check
  if (dev == NULL) {
    rslt = BMI160_E_NULL_PTR;
  } else {
    //Configuring reg_addr for I²C Interface
    rslt = BMI160_I2cGetRegs(dev, reg_addr, data, len);
    vTaskDelay(1 / portTICK_PERIOD_MS);
    if (rslt != BMI160_OK){
      rslt = BMI160_E_COM_FAIL;
    }
  }

  return rslt;
}

int8_t BMI160_I2cGetRegs(BMI160 *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  i2c_master_bus_config_t i2c_mst_config = {
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .i2c_port = TEST_I2C_PORT,
      .scl_io_num = I2C_MASTER_SCL_IO,
      .sda_io_num = I2C_MASTER_SDA_IO,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = true,
  };

  i2c_master_bus_handle_t bus_handle;
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
  i2c_device_config_t dev_cfg = {
      .dev_addr_length = I2C_ADDR_BIT_LEN_7,
      .device_address = dev->id,
      .scl_speed_hz = 100000,
  };
  i2c_master_dev_handle_t dev_handle;
  ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

  ESP_ERROR_CHECK(i2c_master_transmit_receive(dev_handle, &reg_addr, sizeof(reg_addr), data, len, -1));

  return BMI160_OK;
}

int8_t BMI160_setRegs(uint8_t reg_addr, uint8_t *data, uint16_t len, BMI160 *dev)
{
  int8_t rslt = BMI160_OK;
  //Null-pointer check
  if (dev == NULL) {
    rslt = BMI160_E_NULL_PTR;
  } else {
    //Configuring reg_addr for I²C Interface
    rslt = BMI160_I2cSetRegs(dev,reg_addr,data,len);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    if (rslt != BMI160_OK)
      rslt = BMI160_E_COM_FAIL;
  }

  return rslt;
}

int8_t BMI160_I2cSetRegs(BMI160 *dev, uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  if ((dev->prevAccelCfg.power == BMI160_ACCEL_NORMAL_MODE)||(dev->prevGyroCfg.power == BMI160_GYRO_NORMAL_MODE)){
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = TEST_I2C_PORT,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };

    i2c_master_bus_handle_t bus_handle;
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = dev->id,
        .scl_speed_hz = 100000,
    };
    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &reg_addr, 1, -1));
    for(int i = 0; i < len; i++){
      ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &data[i], 1, -1));
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
  }else{
    for(int i = 0; i < len; i++){
        i2c_master_bus_config_t i2c_mst_config = {
          .clk_source = I2C_CLK_SRC_DEFAULT,
          .i2c_port = TEST_I2C_PORT,
          .scl_io_num = I2C_MASTER_SCL_IO,
          .sda_io_num = I2C_MASTER_SDA_IO,
          .glitch_ignore_cnt = 7,
          .flags.enable_internal_pullup = true,
      };

      i2c_master_bus_handle_t bus_handle;
      ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &bus_handle));
      i2c_device_config_t dev_cfg = {
          .dev_addr_length = I2C_ADDR_BIT_LEN_7,
          .device_address = dev->id,
          .scl_speed_hz = 100000,
      };
      i2c_master_dev_handle_t dev_handle;
      ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
      ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &reg_addr, 1, -1));
      ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, &data[i], 1, -1));

      ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
      vTaskDelay(1 / portTICK_PERIOD_MS);
    }
  }
  return BMI160_OK;
}
