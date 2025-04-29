#include "../include/BMP390.h"

BMP390::BMP390(uint8_t chipID)
{
  BMP3Info.chipID = chipID;
}

int BMP390::begin(void)
{
  uint8_t id;
  if(0 == readReg(BMP390_CHIP_ID, &id, sizeof(id)))   // Judge whether the data bus is successful
  {
    DBG("ERR_DATA_BUS");
    return ERR_DATA_BUS;
  }

  DBG("real sensor id=");DBG(id);
  if(BMP3Info.chipID != id)   // Judge whether the chip version matches
  {
    DBG("ERR_IC_VERSION");
    return ERR_IC_VERSION;
  }

  BMP3Info.seaLevelPressPa = STANDARD_SEA_LEVEL_PRESSURE_PA;
  setPWRMode(ePressEN | eTempEN | eNormalMode);   // Set normal aquisition mode, enable temperature and pressure aquisition.
  delay(50);
  getBMP390CalibData();
  delay(50);
  // cacheErrorStatus();
  // cacheSensorStatus();
  // cacheSensorEvent();
  // cacheINTStatus();
  DBG("begin ok!");
  return ERR_OK;
}

/*****************Integrated configuration of each register ******************************/
void BMP390::setINTMode(uint8_t mode)
{
  writeReg(BMP390_INT_CTRL, &mode, sizeof(mode));
}

void BMP390::setPWRMode(uint8_t mode)
{
  readReg(BMP390_PWR_CTRL, &BMP3Info.PWRMode, sizeof(BMP3Info.PWRMode));
  if(*((uint8_t *)&BMP3Info.PWRMode) == mode){
    DBG("Same configuration as before！");
  }else{
    if(eSleepMode != BMP3Info.PWRMode.powerMode){   // You need to turn the device into sleep mode before changing its mode
      BMP3Info.PWRMode.powerMode = eSleepMode;
      writeReg(BMP390_PWR_CTRL, &BMP3Info.PWRMode, sizeof(mode));
      delay(20);   // Give it some time to enter sleep mode
    }
    memcpy(&BMP3Info.PWRMode, &mode, 1);
    writeReg(BMP390_PWR_CTRL, &BMP3Info.PWRMode, sizeof(mode));
    delay(20);   // Give it some time to switch mode
  }
}

void BMP390::setOSRMode(uint8_t mode)
{
  writeReg(BMP390_OSR, &mode, sizeof(mode));
}

bool BMP390::setODRMode(uint8_t mode)
{
  bool ret = true;

  writeReg(BMP390_ODR, &mode, sizeof(mode));
  uint32_t samplingPeriodus = getSamplingPeriodUS();   // Judge whether the ODR setting is resonable
  if(0 == samplingPeriodus){
    DBG("ODRSetting error!!!");
    ret = false;
  }

  return ret;
}

void BMP390::setIIRMode(uint8_t mode)
{
  writeReg(BMP390_IIR_CONFIG, &mode, sizeof(mode));
}

#define CASE_SAMPLING_MODE(MODE, PWR, OSR, ODR, IIR)   case MODE:\
                                                       setPWRMode(PWR);\
                                                       setOSRMode(OSR);\
                                                       setODRMode(ODR);\
                                                       setIIRMode(IIR);\
                                                       break;   // Simplify common mode configuration written macros
bool BMP390::setSamplingMode(ePrecisionMode_t mode)
{
  bool ret = true;
  switch (mode)
  {
    CASE_SAMPLING_MODE(eUltraLowPrecision, ePressEN | eTempEN | eForcedMode, ePressOSRMode1 | eTempOSRMode1, BMP390_ODR_0P01_HZ, BMP390_IIR_CONFIG_COEF_0)
    CASE_SAMPLING_MODE(eLowPrecision, ePressEN | eTempEN | eNormalMode, ePressOSRMode2 | eTempOSRMode1, BMP390_ODR_100_HZ, BMP390_IIR_CONFIG_COEF_0)
    CASE_SAMPLING_MODE(eNormalPrecision1, ePressEN | eTempEN | eNormalMode, ePressOSRMode4 | eTempOSRMode1, BMP390_ODR_50_HZ, BMP390_IIR_CONFIG_COEF_3)
    CASE_SAMPLING_MODE(eNormalPrecision2, ePressEN | eTempEN | eNormalMode, ePressOSRMode8 | eTempOSRMode1, BMP390_ODR_50_HZ, BMP390_IIR_CONFIG_COEF_1)
    CASE_SAMPLING_MODE(eHighPrecision, ePressEN | eTempEN | eNormalMode, ePressOSRMode8 | eTempOSRMode1, BMP390_ODR_12P5_HZ, BMP390_IIR_CONFIG_COEF_1)
    CASE_SAMPLING_MODE(eUltraPrecision, ePressEN | eTempEN | eNormalMode, ePressOSRMode16 | eTempOSRMode2, BMP390_ODR_25_HZ, BMP390_IIR_CONFIG_COEF_3)
    default:
      DBG("samping mode error!");
      ret = false;
  }
  delay(10);
  return ret;
}

/***************** Get and process data registers ******************************/

uint32_t BMP390::getSamplingPeriodUS(void)
{
  uint8_t ODRSetting;
  uint32_t samplingPeriodUS;

  readReg(BMP390_ODR, &ODRSetting, sizeof(ODRSetting));
  readReg(BMP390_PWR_CTRL, &BMP3Info.PWRMode, sizeof(BMP3Info.PWRMode));
  readReg(BMP390_OSR, &BMP3Info.overSamplingMode, sizeof(BMP3Info.overSamplingMode));

  samplingPeriodUS = 234 + BMP3Info.PWRMode.pressEN * (392 + pow(2, BMP3Info.overSamplingMode.OSRPress) * 2020) +
                      BMP3Info.PWRMode.tempEN * (163 + pow(2, BMP3Info.overSamplingMode.OSRTemp) * 2020);

  return (pgm_read_dword(&correspondingSamplingPeriod[ODRSetting]) < samplingPeriodUS) ? 0:samplingPeriodUS;
}

float BMP390::readPressPa(void)
{
  uint8_t buf[6] = {0};
  uint32_t uncompPress, uncompTemp;

  readReg(BMP390_P_DATA_PA, &buf, sizeof(buf));
  uncompPress = (uint32_t)buf[0] | ((uint32_t)buf[1] << 8) | ((uint32_t)buf[2] << 16);
  uncompTemp = (uint32_t)buf[3] | ((uint32_t)buf[4] << 8) | ((uint32_t)buf[5] << 16);

  /* Update the compensation temperature in the correction structure, which needs to be used to calculate the pressure */
  calibTemperatureC(uncompTemp);

  return calibPressurePa(uncompPress);
}

bool BMP390::calibratedAbsoluteDifference(float altitude)
{
  bool ret = false;
  if(STANDARD_SEA_LEVEL_PRESSURE_PA == BMP3Info.seaLevelPressPa){
    float pressure = readPressPa();
    BMP3Info.seaLevelPressPa = (pressure / pow(1.0 - (altitude / 44307.7), 5.255302));
    ret = true;
  }
  return ret;
}

float BMP390::readAltitudeM(void)
{
  float pressure = readPressPa();
  return (1.0 - pow(pressure / STANDARD_SEA_LEVEL_PRESSURE_PA, 0.190284)) * 44307.7;
}

void BMP390::getBMP390CalibData(void)
{
  uint8_t regData[BMP390_CALIB_DATA_LEN] = {0};

  readReg(BMP390_CALIB_DATA, &regData, BMP390_CALIB_DATA_LEN);

  /* 1 / 2^8 */
  // 0.00390625f;
  BMP3Info.regCalibData.parT1 = BMP390_CONCAT_BYTES(regData[1], regData[0]);
  BMP3Info.quantizedCalibData.parT1 = ((float)BMP3Info.regCalibData.parT1 / pow(2, -8));
  // 1073741824.0f;
  BMP3Info.regCalibData.parT2 = BMP390_CONCAT_BYTES(regData[3], regData[2]);
  BMP3Info.quantizedCalibData.parT2 = ((float)BMP3Info.regCalibData.parT2 / pow(2, 30));
  // 281474976710656.0f;
  BMP3Info.regCalibData.parT3 = (int8_t)regData[4];
  BMP3Info.quantizedCalibData.parT3 = ((float)BMP3Info.regCalibData.parT3 / pow(2, 48));
  // 1048576.0f;
  BMP3Info.regCalibData.parP1 = (int16_t)BMP390_CONCAT_BYTES(regData[6], regData[5]);
  BMP3Info.quantizedCalibData.parP1 = ((float)(BMP3Info.regCalibData.parP1 - (16384)) / pow(2, 20));
  // 536870912.0f;
  BMP3Info.regCalibData.parP2 = (int16_t)BMP390_CONCAT_BYTES(regData[8], regData[7]);
  BMP3Info.quantizedCalibData.parP2 = ((float)(BMP3Info.regCalibData.parP2 - (16384)) / pow(2, 29));
  // 4294967296.0f;
  BMP3Info.regCalibData.parP3 = (int8_t)regData[9];
  BMP3Info.quantizedCalibData.parP3 = ((float)BMP3Info.regCalibData.parP3 / pow(2, 32));
  // 137438953472.0f;
  BMP3Info.regCalibData.parP4 = (int8_t)regData[10];
  BMP3Info.quantizedCalibData.parP4 = ((float)BMP3Info.regCalibData.parP4 / pow(2, 37));

  /* 1 / 2^3 */
  // 0.125f;
  BMP3Info.regCalibData.parP5 = BMP390_CONCAT_BYTES(regData[12], regData[11]);
  BMP3Info.quantizedCalibData.parP5 = ((float)BMP3Info.regCalibData.parP5 / pow(2, -3));
  // 64.0f;
  BMP3Info.regCalibData.parP6 = BMP390_CONCAT_BYTES(regData[14], regData[13]);
  BMP3Info.quantizedCalibData.parP6 = ((float)BMP3Info.regCalibData.parP6 / pow(2, 6));
  // 256.0f;
  BMP3Info.regCalibData.parP7 = (int8_t)regData[15];
  BMP3Info.quantizedCalibData.parP7 = ((float)BMP3Info.regCalibData.parP7 / pow(2, 8));
  // 32768.0f;
  BMP3Info.regCalibData.parP8 = (int8_t)regData[16];
  BMP3Info.quantizedCalibData.parP8 = ((float)BMP3Info.regCalibData.parP8 / pow(2, 15));
  // 281474976710656.0f;
  BMP3Info.regCalibData.parP9 = (int16_t)BMP390_CONCAT_BYTES(regData[18], regData[17]);
  BMP3Info.quantizedCalibData.parP9 = ((float)BMP3Info.regCalibData.parP9 / pow(2, 48));
  // 281474976710656.0f;
  BMP3Info.regCalibData.parP10 = (int8_t)regData[19];
  BMP3Info.quantizedCalibData.parP10 = ((float)BMP3Info.regCalibData.parP10 / pow(2, 48));
  // 36893488147419103232.0f;
  BMP3Info.regCalibData.parP11 = (int8_t)regData[20];
  BMP3Info.quantizedCalibData.parP11 = ((float)BMP3Info.regCalibData.parP11 / pow(2, 65));

}

float BMP390::calibTemperatureC(uint32_t uncompTemp)
{
  /* Temporary variable for compensation */
  float partialData1;
  float partialData2;

  partialData1 = (float)(uncompTemp - BMP3Info.quantizedCalibData.parT1);
  partialData2 = (float)(partialData1 * BMP3Info.quantizedCalibData.parT2);

  /* Update the compensation temperature in the correction structure, which needs to be used to calculate the pressure */
  BMP3Info.quantizedCalibData.tempLin = partialData2 + pow(partialData1, 2) * BMP3Info.quantizedCalibData.parT3;

  /* Return compensation temperature */
  return BMP3Info.quantizedCalibData.tempLin;
}

float BMP390::calibPressurePa(uint32_t uncompPress)
{
  /* Temporary variable for compensation */
  float partialData1;
  float partialData2;
  float partialData3;
  float partialData4;
  float partialOut1;
  float partialOut2;

  /* Variable to store compensation pressure */
  float compPress;

  /* calibration data */
  partialData1 = BMP3Info.quantizedCalibData.parP6 * BMP3Info.quantizedCalibData.tempLin;
  partialData2 = BMP3Info.quantizedCalibData.parP7 * pow(BMP3Info.quantizedCalibData.tempLin, 2);
  partialData3 = BMP3Info.quantizedCalibData.parP8 * pow(BMP3Info.quantizedCalibData.tempLin, 3);
  partialOut1 = BMP3Info.quantizedCalibData.parP5 + partialData1 + partialData2 + partialData3;

  partialData1 = BMP3Info.quantizedCalibData.parP2 * BMP3Info.quantizedCalibData.tempLin;
  partialData2 = BMP3Info.quantizedCalibData.parP3 * pow(BMP3Info.quantizedCalibData.tempLin, 2);
  partialData3 = BMP3Info.quantizedCalibData.parP4 * pow(BMP3Info.quantizedCalibData.tempLin, 3);
  partialOut2 = uncompPress * (BMP3Info.quantizedCalibData.parP1 + partialData1 + partialData2 + partialData3);

  partialData1 = (float)uncompPress * (float)uncompPress;
  partialData2 = BMP3Info.quantizedCalibData.parP9 + BMP3Info.quantizedCalibData.parP10 * BMP3Info.quantizedCalibData.tempLin;
  partialData3 = partialData1 * partialData2;
  partialData4 = partialData3 + pow((float)uncompPress, 3) * BMP3Info.quantizedCalibData.parP11;
  compPress = partialOut1 + partialOut2 + partialData4;

  return compPress - BMP3Info.seaLevelPressPa + STANDARD_SEA_LEVEL_PRESSURE_PA;
}

/***************** Initialization and reading and writing of I2C interface ******************************/

BMP390_I2C::BMP390_I2C(TwoWire *pWire, eSDOPinMode_t mode, uint8_t chipID)
  :BMP390(chipID)
{
  if(mode){
    _deviceAddr = BMP390_I2C_ADDR_SDO_VDD;
  }else{
    _deviceAddr = BMP390_I2C_ADDR_SDO_GND;
  }
  _pWire = pWire;
}

int BMP390_I2C::begin(void)
{
  _pWire->begin();   // Wire.h(I2C)library function initialize wire library
  return BMP390::begin();   // Use the initialization function of the parent class
}

void BMP390_I2C::writeReg(uint8_t reg, const void* pBuf, size_t size)
{
  if(pBuf == NULL){
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t *)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire->write(reg);

  for(size_t i = 0; i < size; i++){
    _pWire->write(_pBuf[i]);
  }
  _pWire->endTransmission();
}

size_t BMP390_I2C::readReg(uint8_t reg, void* pBuf, size_t size)
{
  size_t count = 0;
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t * _pBuf = (uint8_t*)pBuf;

  _pWire->beginTransmission(_deviceAddr);
  _pWire -> write(reg);
  if(0 != _pWire->endTransmission())   // Used Wire.endTransmission() to end a slave transmission started by beginTransmission() and arranged by write().
  {
    DBG("endTransmission ERROR!!");
  }else{
    _pWire->requestFrom(_deviceAddr, (uint8_t)size);   // Master device requests size bytes from slave device, which can be accepted by master device with read() or available()

    while (_pWire->available())
    {
      _pBuf[count++] = _pWire->read();   // Use read() to receive and put into buf
    }
    // _pWire->endTransmission();
  }
  return count;
}

BMP390_BAROMETER::BMP390_BAROMETER(TwoWire *pWire, eSDOPinMode_t mode)
  :BMP390_I2C(pWire, mode, BMP390_ID)
{

}
