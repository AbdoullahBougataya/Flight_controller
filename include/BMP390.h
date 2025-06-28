#ifndef __BMP390_H__
#define __BMP390_H__

#include <Arduino.h>
#include <Wire.h>


// #define ENABLE_DBG   //!< open this macro and you can see the details of the program
#ifdef ENABLE_DBG
  #define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
  #define DBG(...)
#endif

#define STANDARD_SEA_LEVEL_PRESSURE_PA  101325   ///< Standard sea level pressure, unit: pa

#define BMP390_I2C_ADDR_SDO_GND   uint8_t(0x76)   ///< I2C communication address when SDO is grounded
#define BMP390_I2C_ADDR_SDO_VDD   uint8_t(0x77)   ///< I2C communication address when SDO is connected to power

#define BMP390_ID 0x60            ///< BMP390 chip version

/* BMP390 register address */
#define BMP390_CHIP_ID     uint8_t(0x00)   ///< The “CHIP_ID” register contains the chip identification code.
#define BMP390_REV_ID      uint8_t(0x01)   ///< The “Rev_ID” register contains the mask revision of the ASIC.
#define BMP390_ERR_REG     uint8_t(0x02)   ///< Sensor Error conditions are reported in the “ERR_REG” register.
#define BMP390_STATUS      uint8_t(0x03)   ///< The Sensor Status Flags are stored in the “STATUS” register.

#define BMP390_P_DATA_PA    uint8_t(0x04)   ///< The 24Bit pressure data is split and stored in three consecutive registers.
#define BMP390_T_DATA_C     uint8_t(0x07)   ///< The 24Bit temperature data is split and stored in three consecutive registersd.
#define BMP390_TIME        uint8_t(0x0C)    ///< The 24Bit sensor time is split and stored in three consecutive registers.

#define BMP390_EVENT       uint8_t(0x10)   ///< The “EVENT” register contains the sensor status flags.
#define BMP390_INT_STATUS  uint8_t(0x11)   ///< The “INT_STATUS” register shows interrupt status and is cleared after reading.

#define BMP390_INT_CTRL    uint8_t(0x19)   ///< Interrupt configuration can be set in the “INT_CTRL” register.
#define BMP390_IF_CONF     uint8_t(0x1A)   ///< The “IF_CONF” register controls the serial interface settings.
#define BMP390_PWR_CTRL    uint8_t(0x1B)   ///< The “PWR_CTRL” register enables or disables pressure and temperature measurement.
#define BMP390_OSR         uint8_t(0x1C)   ///< The “OSR” register controls the oversampling settings for pressure and temperature measurements.
#define BMP390_ODR         uint8_t(0x1D)   ///< The “ODR” register set the configuration of the output data rates by means of setting the subdivision/subsampling.
#define BMP390_IIR_CONFIG  uint8_t(0x1F)   ///< The “CONFIG” register controls the IIR filter coefficients
#define BMP390_CALIB_DATA  uint8_t(0x31)   ///< 0x31-0x45 is calibration data
#define BMP390_CMD         uint8_t(0x7E)   ///< Command register, can soft reset and clear all FIFO data

/* Set the constant of output data rate in subdivision/sub-sampling mode */
#define BMP390_ODR_200_HZ         uint8_t(0x00)   ///< Prescaler:1; ODR 200Hz; Sampling period:5 ms
#define BMP390_ODR_100_HZ         uint8_t(0x01)   ///< Prescaler:2; Sampling period:10 ms
#define BMP390_ODR_50_HZ          uint8_t(0x02)   ///< Prescaler:4; Sampling period:20 ms
#define BMP390_ODR_25_HZ          uint8_t(0x03)   ///< Prescaler:8; Sampling period:40 ms
#define BMP390_ODR_12P5_HZ        uint8_t(0x04)   ///< Prescaler:16; Sampling period:80 ms
#define BMP390_ODR_6P25_HZ        uint8_t(0x05)   ///< Prescaler:32; Sampling period:160 ms
#define BMP390_ODR_3P1_HZ         uint8_t(0x06)   ///< Prescaler:64; Sampling period:320 ms
#define BMP390_ODR_1P5_HZ         uint8_t(0x07)   ///< Prescaler:127; Sampling period:640 ms
#define BMP390_ODR_0P78_HZ        uint8_t(0x08)   ///< Prescaler:256; Sampling period:1.280 s
#define BMP390_ODR_0P39_HZ        uint8_t(0x09)   ///< Prescaler:512; Sampling period:2.560 s
#define BMP390_ODR_0P2_HZ         uint8_t(0x0A)   ///< Prescaler:1024 Sampling period:5.120 s
#define BMP390_ODR_0P1_HZ         uint8_t(0x0B)   ///< Prescaler:2048; Sampling period:10.24 s
#define BMP390_ODR_0P05_HZ        uint8_t(0x0C)   ///< Prescaler:4096; Sampling period:20.48 s
#define BMP390_ODR_0P02_HZ        uint8_t(0x0D)   ///< Prescaler:8192; Sampling period:40.96 s
#define BMP390_ODR_0P01_HZ        uint8_t(0x0E)   ///< Prescaler:16384; Sampling period:81.92 s
#define BMP390_ODR_0P006_HZ       uint8_t(0x0F)   ///< Prescaler:32768; Sampling period:163.84 s
#define BMP390_ODR_0P003_HZ       uint8_t(0x10)   ///< Prescaler:65536; Sampling period:327.68 s
#define BMP390_ODR_0P0015_HZ      uint8_t(0x11)   ///< Prescaler:131072; ODR 25/16384Hz; Sampling period:655.36 s

/* IIR filter coefficient setting constant */
#define BMP390_IIR_CONFIG_COEF_0           uint8_t(0x00)   ///< Filter coefficient is 0 -> bypass mode
#define BMP390_IIR_CONFIG_COEF_1           uint8_t(0x02)   ///< Filter coefficient is 1
#define BMP390_IIR_CONFIG_COEF_3           uint8_t(0x04)   ///< Filter coefficient is 3
#define BMP390_IIR_CONFIG_COEF_7           uint8_t(0x06)   ///< Filter coefficient is 7
#define BMP390_IIR_CONFIG_COEF_15          uint8_t(0x08)   ///< Filter coefficient is 15
#define BMP390_IIR_CONFIG_COEF_31          uint8_t(0x0A)   ///< Filter coefficient is 31
#define BMP390_IIR_CONFIG_COEF_63          uint8_t(0x0C)   ///< Filter coefficient is 63
#define BMP390_IIR_CONFIG_COEF_127         uint8_t(0x0E)   ///< Filter coefficient is 127

/* CMD(0x7E) register command */
#define BMP390_CMD_NOP         0x00    ///< reserved. No command.
#define BMP390_CMD_SOFTRESET   0xB6    ///< Triggers a reset, all user configuration settings are overwritten with their default state.



/* Convenience Macro */
#define BMP390_CALIB_DATA_LEN   (21)   ///< Number of calibration data bytes in the BMP390 register
#define BMP390_CONCAT_BYTES(msb, lsb)   (((uint16_t)msb << 8) | (uint16_t)lsb)   ///< Macro combines two 8-bit data into one 16-bit data

/* Sampling period corresponding to ODR in microseconds */
static const uint32_t PROGMEM correspondingSamplingPeriod[] = {
  5000, 10000, 20000, 40000, 80000, 160000, 320000, 640000, 1280000, 2560000, 5120000, 10240000, 20480000,
  40960000, 81920000, 163840000, 327680000, 655360000
};

class BMP390
{
public:
#define BMP390_ERR_OK              0    // No error
#define ERR_DATA_BUS      (-1)   // data bus error
#define ERR_IC_VERSION    (-2)   // the chip version not match

/***************** structs like register configuration ******************************/
  /**
   * @struct sIntMode_t
   * @brief interrupt configuration can be set in "INT_CTRL"(0x19) register
   * @details it affects INT_STATUS register and INT pin
   * @note register struct:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7    |   b6    |    b5    |    b4    |    b3    |    b2     |    b1     |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n | reserved | drdy_en |  int_ds  | reserved | reserved | int_latch | int_level |  int_od  |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   INTOD: 1; /**< pin output mode, power up is 0,  0: push-pull,  1: open-drain */
    uint8_t   INTActiveLevel: 1; /**< pin level, power up is 1,  0: active low,  1: active high */
    uint8_t   INTLatch: 1; /**< enable INT pin and INT_STATUS register lock-in interrupt, power up is 0,  0: disable,  1: enable */
    uint8_t   INTInitialLevel: 1; /**< power up is 0,  0: low,  1: high */
    uint8_t   INTDrdyEN: 1; /**< enable INT pin and INT_STATUS temperature/pressure data ready interrupt, power up is 0,  0: disable,  1: enable */
    uint8_t   reserved: 1; /**< reserved bit */
  } __attribute__ ((packed)) sIntMode_t;

  /**
   * @struct sSerialMode_t
   * @brief “IF_CONF”(0x1A)register serial port control setting
   * @note register struct:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7   |    b6   |    b5   |    b4   |    b3   |      b2     |     b1      |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n |                      reserved                   | i2c_wdt_sel | i2c_wdt_en  |   spi3   |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   I2CWDTEN: 1; /**< power up is 0,  0: disable I2C WDT,  1: enable I2C WDT */
    uint8_t   I2CWDTSel: 1; /**< power up is 0,  0: set I2C WDT timeout value to 1.25ms,  1: set I2C WDT timeout value to 40ms */
    uint8_t   reserved: 5; /**< reserved bit */
  } __attribute__ ((packed)) sSerialMode_t;

  /**
   * @struct sPWRCTRL_t
   * @brief “PWR_CTRL”(0x1B)the register enable or disable pressure and temperature measurement
   * @details measurement mode can be set here:
   * @n Sleep mode: It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. All registers are accessible for reading the chip ID and compensation coefficient.
   * @n Forced mode: In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
   * @n Normal mode: Continuously loop between the measurement period and the standby period. Measurement rate is set in odr_sel register, and prescalers with different sampling frequency Fsampling=200Hz can be selected.
   * @note register struct:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n |      reserved2      |     power_modes     |     reserved1       |  temp_en  | press_en |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   pressEN: 1; /**< power up is 0,  0: disable pressure sensing,  1: enable pressure sensing */
    uint8_t   tempEN: 1; /**< power up is 0,  0: disable temperature sensing,  1: enable temperature sensing */
    uint8_t   reserved1: 2; /**< reserved bit */
    uint8_t   powerMode: 2; /**< power up is 0,  0: sleep mode,  1or2: enforcing mode,  3: normal mode */
    uint8_t   reserved2: 2; /**< reserved bit */
  } __attribute__ ((packed)) sPWRCTRL_t;

  /**
   * @struct sOverSamplingMode_t
   * @brief “OSR”(0x1C)oversampling setting of register control pressure and temperature measurement
   * @details 6 configurations of temperature and pressure oversampling mode:
   * @n ------------------------------------------------------------------------------------------
   * @n |   oversampling setting   |   osr_p    |  pressure oversampling  |     typical pressure resolution     |  recommended temperature oversampling  |
   * @n ------------------------------------------------------------------------------------------
   * @n |    ultra-low power consumption    |    000     |      ×1      |    16 bit / 2.64 Pa    |        ×1        |
   * @n ------------------------------------------------------------------------------------------
   * @n |     low power consumption     |    001     |      ×2      |    16 bit / 2.64 Pa    |        ×1        |
   * @n ------------------------------------------------------------------------------------------
   * @n |   standard resolution  |    010     |      ×4      |    18 bit / 0.66 Pa    |        ×1        |
   * @n ------------------------------------------------------------------------------------------
   * @n |    high resolution    |    011     |      ×8      |    19 bit / 0.33 Pa    |        ×1        |
   * @n ------------------------------------------------------------------------------------------
   * @n |   ultrahigh resolution   |    100     |      ×16     |    20 bit / 0.17 Pa    |        ×2        |
   * @n ------------------------------------------------------------------------------------------
   * @n |   highest resolution   |    101     |      ×32     |    21 bit / 0.085 Pa   |        ×2        |
   * @n ------------------------------------------------------------------------------------------
   * @note register struct:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7    |    b6    |    b5    |    b4    |    b3    |    b2    |    b1     |    b0    |
   * @n ------------------------------------------------------------------------------------------
   * @n |      reserved       |             osr_t              |              osr_p              |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   OSRPress: 3; /**< power up is 0,  6 pressure oversampling modes can be set */
    uint8_t   OSRTemp: 3; /**< power up is 0,  temperature mode is also available in 6 settings, similar to pressure mode setting, but it is recommended to use the temperature oversampling mode recommended in the table */
    uint8_t   reserved: 2; /**< reserved bit */
  } __attribute__ ((packed)) sOverSamplingMode_t;

/***************** the struct to calibrate compensation data ******************************/

  /**
   * @struct sCalibData_t
   * @brief buffer the struct of calibrating compensation data in register
   */
  typedef struct
  {
    uint16_t parT1;
    uint16_t parT2;
    int8_t parT3;
    int16_t parP1;
    int16_t parP2;
    int8_t parP3;
    int8_t parP4;
    uint16_t parP5;
    uint16_t parP6;
    int8_t parP7;
    int8_t parP8;
    int16_t parP9;
    int8_t parP10;
    int8_t parP11;
    int64_t tempLin;
  } sCalibData_t;

  /**
   * @struct sQuantizedCalibData_t
   * @brief quantized compensation data
   */
  typedef struct
  {
    float parT1;
    float parT2;
    float parT3;
    float parP1;
    float parP2;
    float parP3;
    float parP4;
    float parP5;
    float parP6;
    float parP7;
    float parP8;
    float parP9;
    float parP10;
    float parP11;
    float tempLin;
  } sQuantizedCalibData_t;

/***************** device status information struct ******************************/

  /**
   * @struct sSensorErrorStatus_t
   * @brief sensor error cases are reported in "ERR_REG" register
   * @note register struct:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7   |    b6   |    b5   |    b4   |    b3   |      b2    |     b1    |     b0      |
   * @n ------------------------------------------------------------------------------------------
   * @n |                      reserved                   |  conf_err  |  cmd_err  |  fatal_err  |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   fatalError: 1; /**< fatal error, unrecoverable error */
    uint8_t   CMDError: 1; /**< the command fails to be executed and is cleared after being read */
    uint8_t   configError: 1; /**< detect the sensor configuration error (only work in normal mode), and it's cleared after being read */
    uint8_t   reserved: 5; /**< reserved bit */
  } __attribute__ ((packed)) sSensorErrorStatus_t;

  /**
   * @struct sSensorStatus_t
   * @brief the sensor status is buffered in "STATUS" register
   * @note register struct:
   * @n ------------------------------------------------------------------------------------------
   * @n |      b7     |     b6    |     b5     |    b4   |    b3   |    b2   |    b1   |    b0   |
   * @n ------------------------------------------------------------------------------------------
   * @n |  reserved2  | drdy_temp | drdy_press | cmd_rdy |                reserved1              |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   reserved1: 4; /**< reserved bit */
    uint8_t   CMDReady: 1; /**< CMD decoder status */
    uint8_t   pressDrdy: 1; /**< pressure data ready */
    uint8_t   tempDrdy: 1; /**< temperature data ready */
    uint8_t   reserved2: 1; /**< reserved bit */
  } __attribute__ ((packed)) sSensorStatus_t;

  /**
   * @struct sSensorEvent_t
   * @brief "EVENT" register includes sensor status
   * @note register struct:
   * @n ------------------------------------------------------------------------------------------
   * @n |    b7   |    b6   |   b5   |   b4   |   b3   |   b2   |      b1      |       b0       |
   * @n ------------------------------------------------------------------------------------------
   * @n |                      reserved                         |  itf_act_pt  |  por_detected  |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   porDetected: 1; /**< the device is set to "1" after power on or soft reset, and cleared after reading */
    uint8_t   itfActPt: 1; /**< the device is set to "1" when a serial port transaction occurs during pressure or temperature conversion, and is cleared after reading */
    uint8_t   reserved: 6; /**< reserved bit */
  } __attribute__ ((packed)) sSensorEvent_t;

  /**
   * @struct sSensorINTStatus_t
   * @brief "INT_STATUS" register displays interrupt status and it's cleared after reading
   * @note register struct:
   * @n ------------------------------------------------------------------------------------------
   * @n |   b7  |   b6   |    b5   |    b4   |    b3   |      b2    |      b1     |      b0      |
   * @n ------------------------------------------------------------------------------------------
   * @n |             reserved2              |   drdy  | reserved1  |  ffull_int  |   fwtm_int   |
   * @n ------------------------------------------------------------------------------------------
   */
  typedef struct
  {
    uint8_t   reserved1: 1; /**< reserved bit */
    uint8_t   dataReady: 1; /**< data ready interrupt */
    uint8_t   reserved2: 4; /**< reserved bit */
  } __attribute__ ((packed)) sSensorINTStatus_t;

  /**
   * @struct sBMP390DeviceInfo_t
   * @brief BMP390 device information struct
   */
  typedef struct
  {
    /* chip ID */
    uint8_t chipID;

    /* calibration compensation coefficient of measured data */
    sCalibData_t regCalibData;   /**< store the calibration data read by register */
    sQuantizedCalibData_t quantizedCalibData;   /**< buffer the quantized calibration data */
    float seaLevelPressPa;   /**< sea level atmospheric pressure used to calculate altitude */

    /* sensor configuration data */
    sIntMode_t intMode;
    sSerialMode_t serialMode;
    sPWRCTRL_t PWRMode;
    sOverSamplingMode_t overSamplingMode;

    /* sensor status data */
    sSensorErrorStatus_t errStatus;
    sSensorStatus_t sensorStatus;
    sSensorEvent_t sensorEvent;
    sSensorINTStatus_t INTStatus;

  } sBMP390DeviceInfo_t;

/***************** enumerate data types of every register detailed configuration ******************************/

  /**
   * @enum  eINTPinMode_t
   * @brief  interrupt pin output mode
   */
  typedef enum
  {
    eINTPinPP = 0,   /**< push-pull */
    eINTPinOD,   /**< open-drain */
  }eINTPinMode_t;

  /**
   * @enum  eINTPinActiveLevel_t
   * @brief  interrupt pin signal level
   */
  typedef enum
  {
    eINTPinActiveLevelLow = 0<<1,   /**< active low */
    eINTPinActiveLevelHigh = 1<<1,   /**< active high */
  }eINTPinActiveLevel_t;

  /**
   * @enum  eINTLatch_t
   * @brief  whether to enable INT pin and INT_STATUS register lock-in interrupt
   */
  typedef enum
  {
    eINTLatchDIS = 0<<2,   /**< disable */
    eINTLatchEN = 1<<2,   /**< enable */
  }eINTLatch_t;


  /**
   * @enum  eINTInitialLevel_t
   * @brief  interrupt data pin level
   */
  typedef enum
  {
    eINTInitialLevelLOW = 0<<5,   /**< low level */
    eINTInitialLevelHIGH = 1<<5,   /**< high level */
  }eINTInitialLevel_t;

  /**
   * @enum  eINTDataDrdy_t
   * @brief  whether to enable INT pin and INT_STATUS temperature/pressure data ready interrupt
   */
  typedef enum
  {
    eINTDataDrdyDIS = 0<<6,   /**< disable */
    eINTDataDrdyEN = 1<<6,   /**< enable */
  }eINTDataDrdy_t;

  /**
   * @enum  eI2CWDT_t
   * @brief  whether to enable I2C WDT
   */
  typedef enum
  {
    eI2CWDTDIS = 0<<1,   /**< disable */
    eI2CWDTEN = 1<<1,   /**< enable */
  }eI2CWDT_t;

  /**
   * @enum  eI2CWDTSel_t
   * @brief  configure I2C WDT
   */
  typedef enum
  {
    eI2CWDTSel1p25 = 0<<2,   /**< set I2C WDT timeout value to 1.25ms */
    eI2CWDTSel40 = 1<<2,   /**< set I2C WDT timeout value to 40ms */
  }eI2CWDTSel_t;

  /**
   * @enum  ePressMeasure_t
   * @brief  whether to enable pressure sensing
   */
  typedef enum
  {
    ePressDIS = 0, /**< disable pressure sensing */
    ePressEN,   /**< enable pressure sensing */
  }ePressMeasure_t;

  /**
   * @enum  eTempMeasure_t
   * @brief  whether to enable temperature sensing
   */
  typedef enum
  {
    eTempDIS = 0<<1, /**< disable temperature sensing */
    eTempEN = 1<<1,   /**< enable temperature sensing */
  }eTempMeasure_t;

  /**
   * @enum  ePowerMode_t
   * @brief  measurement (power supply) mode setting
   */
  typedef enum
  {
    eSleepMode = 0<<4,   /**< Sleep mode: It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. All registers are accessible for reading the chip ID and compensation coefficient. */
    eForcedMode = 1<<4, /**< Forced mode: In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register. */
    eForcedMode2 = 2<<4, /**< Forced mode: In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register. */
    eNormalMode = 3<<4,  /**< Normal mode: Continuously loop between the measurement period and the standby period, output data rates are affected by ODR mode setting. */
  }ePowerMode_t;

  /**
   * @enum  ePressOSRMode_t
   * @brief  6 pressure oversampling modes
   */
  typedef enum
  {
    ePressOSRMode1 = 0,   /**< sampling×1, 16 bit / 2.64 Pa(recommended temperature oversampling×1) */
    ePressOSRMode2,   /**< sampling×2, 16 bit / 2.64 Pa(recommended temperature oversampling×1) */
    ePressOSRMode4,   /**< sampling×4, 18 bit / 0.66 Pa(recommended temperature oversampling×1) */
    ePressOSRMode8,   /**< sampling×8, 19 bit / 0.33 Pa(recommended temperature oversampling×2) */
    ePressOSRMode16,   /**< sampling×16, 20 bit / 0.17 Pa(recommended temperature oversampling×2) */
    ePressOSRMode32,   /**< sampling×32, 21 bit / 0.085 Pa(recommended temperature oversampling×2) */
  }ePressOSRMode_t;

  /**
   * @enum  eTempOSRMode_t
   * @brief  6 temperature oversampling modes
   */
  typedef enum
  {
    eTempOSRMode1 = 0<<3,   /**< sampling×1, 16 bit / 0.0050 °C */
    eTempOSRMode2 = 1<<3,   /**< sampling×2, 16 bit / 0.0025 °C */
    eTempOSRMode4 = 2<<3,   /**< sampling×4, 18 bit / 0.0012 °C */
    eTempOSRMode8 = 3<<3,   /**< sampling×8, 19 bit / 0.0006 °C */
    eTempOSRMode16 = 4<<3,   /**< sampling×16, 20 bit / 0.0003 °C */
    eTempOSRMode32 = 5<<3,   /**< sampling×32, 21 bit / 0.00015 °C */
  }eTempOSRMode_t;

/***************** enumerated data types for users easy to select ******************************/

  /**
   * @enum  eSDOPinMode_t
   * @brief  SDO wiring status
   */
  typedef enum
  {
    eSDOGND = 0,   /**< SDO connects GND */
    eSDOVDD,   /**< SDO connects VDD */
  }eSDOPinMode_t;

  /**
   * @enum  ePrecisionMode_t
   * @brief  the recommended modes for the best settings
   */
  typedef enum
  {
    eUltraLowPrecision = 0, /**< ultra-low precision, suitable for weather monitoring (minimum power consumption), power mode is enforcing mode, IDD[µA]=4, RMSNoise[cm]=55 */
    eLowPrecision, /**< low precision, suitable for random detection, power mode is normal mode, IDD[µA]=358, RMSNoise[cm]=36 */
    eNormalPrecision1, /**< normal accuracy 1, suitable for dynamic detection on handheld devices (such as mobile phones), power mode is normal mode, IDD[µA]=310, RMSNoise[cm]=10 */
    eNormalPrecision2, /**< normal accuracy 2, suitable for drones, power mode is normal mode, IDD[µA]=570, RMSNoise[cm]=11 */
    eHighPrecision, /**< high precision, suitable for low-power handheld devices (such as mobile phones), power mode is normal mode, IDD[µA]=145, RMSNoise[cm]=11 */
    eUltraPrecision, /**< ultra high precision, suitable for indoor guide, power mode is normal mode, IDD[µA]=560, RMSNoise[cm]=5 */
  }ePrecisionMode_t;

public:
  /**
   * @fn BMP390
   * @brief constructor
   * @param chipID chip ID
   * @return None
   */
  BMP390(uint8_t chipID);

  /**
   * @fn begin
   * @brief initialization function
   * @return int type, means returning initialization status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

/***************** integrated configuration of each register ******************************/


  /**
   * @fn setINTMode
   * @brief interrupt configuration (INT)
   * @param mode interrupt mode to be set, the following patterns constitute mode:
   * @n       interrupt pin output mode: eINTPinPP:  push-pull , eINTPinOD:  open-drain
   * @n       interrupt pin active level: eINTPinActiveLevelLow:  active low , eINTPinActiveLevelHigh:  active high
   * @n       interrupt register lock-in: eINTLatchDIS:  disable , eINTLatchEN:  enable
   * @n       interrupt pin initial (invalid, no interrupt) level: eINTInitialLevelLOW:  low level , eINTInitialLevelHIGH:  high level
   * @n       temperature/pressure data ready interrupt: eINTDataDrdyDIS:  disable , eINTDataDrdyEN:  enable
   * @return None
   */
  void setINTMode(uint8_t mode);

  /**
   * @fn setPWRMode
   * @brief configuration of measurement mode and power mode
   * @param mode measurement mode and power mode to be set, the following patterns constitute mode:
   * @n       ePressDIS:  disable pressure measurement , ePressEN:  enable pressure measurement
   * @n       eTempDIS:  disable temperature measurement , eTempEN:  enable temperature measurement
   * @n       eSleepMode, eForcedMode, eNormalMode three modes:
   * @n         Sleep mode: It will be in sleep mode by default after power-on reset. In this mode, no measurement is performed and power consumption is minimal. All registers are accessible for reading the chip ID and compensation coefficient.
   * @n         Forced mode: In this mode, the sensor will take a single measurement according to the selected measurement and filtering options. After the measurement is completed, the sensor will return to sleep mode, and the measurement result can be obtained in the register.
   * @n         normal mode: continuous loop between the measurement period and standby period, output data rates are affected by ODR mode setting.
   * @return None
   */
  void setPWRMode(uint8_t mode);

  /**
   * @fn setOSRMode
   * @brief oversampling configuration of pressure and temperature measurement (OSR:over-sampling register)
   * @param mode oversampling mode of pressure & temperature measurement to be set, the following patterns constitute mode:
   * @n       6 pressure oversampling modes:
   * @n         ePressOSRMode1,  pressure sampling×1, 16 bit / 2.64 Pa(recommended temperature oversampling×1)
   * @n         ePressOSRMode2,  pressure sampling×2, 16 bit / 2.64 Pa(recommended temperature oversampling×1)
   * @n         ePressOSRMode4,  pressure sampling×4, 18 bit / 0.66 Pa(recommended temperature oversampling×1)
   * @n         ePressOSRMode8,  pressure sampling×8, 19 bit / 0.33 Pa(recommended temperature oversampling×2)
   * @n         ePressOSRMode16,  pressure sampling×16, 20 bit / 0.17 Pa(recommended temperature oversampling×2)
   * @n         ePressOSRMode32,  pressure sampling×32, 21 bit / 0.085 Pa(recommended temperature oversampling×2)
   * @n       6 temperature oversampling modes
   * @n         eTempOSRMode1,  temperature sampling×1, 16 bit / 0.0050 °C
   * @n         eTempOSRMode2,  temperature sampling×2, 16 bit / 0.0025 °C
   * @n         eTempOSRMode4,  temperature sampling×4, 18 bit / 0.0012 °C
   * @n         eTempOSRMode8,  temperature sampling×8, 19 bit / 0.0006 °C
   * @n         eTempOSRMode16,  temperature sampling×16, 20 bit / 0.0003 °C
   * @n         eTempOSRMode32,  temperature sampling×32, 21 bit / 0.00015 °C
   * @return None
   */
  void setOSRMode(uint8_t mode);

  /**
   * @fn setODRMode
   * @brief set output data rate configuration in subdivision/sub-sampling mode (ODR:output data rates)
   * @param mode output data rate to be set, configurable modes:
   * @n       BMP390_ODR_200_HZ, BMP390_ODR_100_HZ, BMP390_ODR_50_HZ, BMP390_ODR_25_HZ, BMP390_ODR_12P5_HZ,
   * @n       BMP390_ODR_6P25_HZ, BMP390_ODR_3P1_HZ, BMP390_ODR_1P5_HZ, BMP390_ODR_0P78_HZ, BMP390_ODR_0P39_HZ,
   * @n       BMP390_ODR_0P2_HZ, BMP390_ODR_0P1_HZ, BMP390_ODR_0P05_HZ, BMP390_ODR_0P02_HZ, BMP390_ODR_0P01_HZ,
   * @n       BMP390_ODR_0P006_HZ, BMP390_ODR_0P003_HZ, BMP390_ODR_0P0015_HZ
   * @return boolean, indicates configuration results
   * @retval True indicates configuration succeeds, successfully update the configuration
   * @retval False indicates configuration fails and remains its original state
   */
  bool setODRMode(uint8_t mode);

  /**
   * @fn setIIRMode
   * @brief IIR filter coefficient configuration (IIR filtering)
   * @param mode IIR filter coefficient setting, configurable modes:
   * @n       BMP390_IIR_CONFIG_COEF_0, BMP390_IIR_CONFIG_COEF_1, BMP390_IIR_CONFIG_COEF_3,
   * @n       BMP390_IIR_CONFIG_COEF_7, BMP390_IIR_CONFIG_COEF_15, BMP390_IIR_CONFIG_COEF_31,
   * @n       BMP390_IIR_CONFIG_COEF_63, BMP390_IIR_CONFIG_COEF_127
   * @return None
   */
  void setIIRMode(uint8_t mode);

    /**
   * @fn setSamplingMode
   * @brief common sampling modes for users easy to configure
   * @param mode:
   * @n       eUltraLowPrecision, ultra-low precision, suitable for weather monitoring (minimum power consumption), power mode is enforcing mode
   * @n       eLowPrecision, low precision, suitable for random detection, power mode is normal mode
   * @n       eNormalPrecision1, normal accuracy 1, suitable for dynamic detection on handheld devices (such as mobile phones), power mode is normal mode
   * @n       eNormalPrecision2, normal accuracy 2, suitable for drones, power mode is normal mode
   * @n       eHighPrecision, high precision, suitable for low-power handheld devices (such as mobile phones), power mode is normal mode
   * @n       eUltraPrecision, ultra high precision, suitable for indoor guide, the collection rate is very low and the collection period is 1000ms, power mode is normal mode
   * @return boolean, indicates configuration results
   * @retval True indicates configuration succeeds, successfully update the configuration
   * @retval False indicates configuration fails and remains its original state
   */
  bool setSamplingMode(ePrecisionMode_t mode);

/***************** data register acquisition and processing ******************************/

  /**
   * @fn getSamplingPeriodUS
   * @brief get the sensor sampling period in the current sampling mode
   * @return return sampling period, unit is us
   */
  uint32_t getSamplingPeriodUS(void);

  /**
   * @fn readPressPa
   * @brief get pressured measured value from the register, operating range(300‒1250 hPa)
   * @return return pressure measured value, unit is Pa
   * @attention if a reference value is previously provided, calculate the absolute value of the pressure at the current position according to the calibrated sea level atmospheric pressure
   */
  float readPressPa(void);

  /**
   * @fn calibratedAbsoluteDifference
   * @brief use the given current altitude as a reference value, eliminate the absolute difference of subsequent pressure and altitude data
   * @param altitude current altitude
   * @return boolean, indicates whether the reference value is set successfully
   * @retval True indicates the reference value is set successfully
   * @retval False indicates fail to set the reference value
   */
  bool calibratedAbsoluteDifference(float altitude);

  /**
   * @fn readAltitudeM
   * @brief calculate altitude according to the atmospheric pressure measured by the sensor
   * @return return altitude, unit is m
   * @attention if a reference value is previously provided, calculate the absolute value of the altitude at the current position according to the calibrated sea level atmospheric pressure
   */
  float readAltitudeM(void);

protected:

  /**
   * @fn getBMP390CalibData
   * @brief get sCalibData_t compensation calibration data
   * @return None
   */
  void getBMP390CalibData(void);

  /**
   * @fn calibTemperatureC
   * @brief use the calibration coefficient to compensate the original data
   * @return return the compensated temperature measured value, the unit is Celsius
   */
  float calibTemperatureC(uint32_t uncompTemp);

  /**
   * @fn calibPressurePa
   * @brief use the calibration coefficient to compensate the original data
   * @return return the compensated pressure measured value, unit is Pa
   */
  float calibPressurePa(uint32_t uncompPress);

/***************** register reading and writing ports ******************************/

  /**
   * @fn writeReg
   * @brief write register function, design it as a pure virtual function, implement the function body through a derived class
   * @param reg  register address 8bits
   * @param pBuf to write data storage and buffer
   * @param size to write data length
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size)=0;

  /**
   * @fn readReg
   * @brief read register function, design it as a pure virtual function, implement the function body through a derived class
   * @param reg  register address 8bits
   * @param pBuf to read data storage and buffer
   * @param size to read data length
   * @return return read length, returning 0 means read failure
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size)=0;

private:
  // private variables
  sBMP390DeviceInfo_t BMP3Info;
};

/***************** initialization and write/read of I2C interface ******************************/

class BMP390_I2C:public BMP390
{
public:
  /**
   * @fn BMP390_I2C
   * @brief constructor, set sensor I2C communication address according to SDO pin wiring
   * @param pWire Wire object is defined in Wire.h, so just use &Wire and the methods in Wire can be pointed to and used
   * @param mode SDO pin connects to GND, the current I2C address is 0x76;SDO pin connects to VDDIO, the current I2C address is 0x77
   * @param chipID chip ID
   * @return None
   */
  BMP390_I2C(TwoWire *pWire, eSDOPinMode_t mode, uint8_t chipID);

  /**
   * @fn begin
   * @brief subclass initialization function
   * @return int type, means returning initialization status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  virtual int begin(void);

protected:
  /**
   * @fn writeReg
   * @brief write register values through I2C bus
   * @param reg  register address 8bits
   * @param pBuf to write data storage and buffer
   * @param size to write data length
   * @return None
   */
  virtual void writeReg(uint8_t reg, const void* pBuf, size_t size);

  /**
   * @fn readReg
   * @brief read register values through I2C bus
   * @param reg  register address 8bits
   * @param pBuf to read data storage and buffer
   * @param size to read data length
   * @return return read length, returning 0 means read failure
   */
  virtual size_t readReg(uint8_t reg, void* pBuf, size_t size);

private:
  TwoWire *_pWire;   // pointer to I2C communication method
  uint8_t _deviceAddr;   // I2C communication device address
};

/***************** BMP390 chip ******************************/
/***************** initialization and write/read of I2C interface ******************************/

class BMP390_BAROMETER:public BMP390_I2C
{
public:
  /**
   * @fn BMP390_I2C
   * @brief constructor, set sensor I2C communication address according to SDO pin wiring
   * @param pWire Wire object is defined in Wire.h, so just use &Wire and the methods in Wire can be pointed to and used
   * @param mode SDO pin connects to GND, the current I2C address is 0x76; SDO pin connects to VDDIO, the current I2C address is 0x77
   * @return None
   */
  BMP390_BAROMETER(TwoWire *pWire=&Wire, eSDOPinMode_t mode=eSDOVDD);

};

#endif
