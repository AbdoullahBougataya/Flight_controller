#include <Arduino.h>
#include <Wire.h>

/** Mask definitions */
#define BMI160_ACCEL_BW_MASK                    UINT8_C(0x70)
#define BMI160_ACCEL_ODR_MASK                   UINT8_C(0x0F)
#define BMI160_ACCEL_UNDERSAMPLING_MASK         UINT8_C(0x80)
#define BMI160_ACCEL_RANGE_MASK                 UINT8_C(0x0F)
#define BMI160_GYRO_BW_MASK                     UINT8_C(0x30)
#define BMI160_GYRO_ODR_MASK                    UINT8_C(0x0F)
#define BMI160_GYRO_RANGE_MSK                   UINT8_C(0x07)

/** BMI160 Register map */
#define BMI160_CHIP_ID_ADDR              UINT8_C(0x00)
#define BMI160_ERROR_REG_ADDR            UINT8_C(0x02)
#define BMI160_GYRO_DATA_ADDR            UINT8_C(0x0C)
#define BMI160_ACCEL_DATA_ADDR           UINT8_C(0x12)
#define BMI160_ACCEL_CONFIG_ADDR         UINT8_C(0x40)
#define BMI160_ACCEL_RANGE_ADDR          UINT8_C(0x41)
#define BMI160_GYRO_CONFIG_ADDR          UINT8_C(0x42)
#define BMI160_GYRO_RANGE_ADDR           UINT8_C(0x43)
#define BMI160_INT_DATA_0_ADDR           UINT8_C(0x58)
#define BMI160_COMMAND_REG_ADDR          UINT8_C(0x7E)
#define BMI160_INT_ENABLE_1_ADDR         UINT8_C(0x51)
#define BMI160_INT_MAP_1_ADDR            UINT8_C(0x56)
#define BMI160_INT_OUT_CTRL_ADDR         UINT8_C(0x53)
#define BMI160_INT_LATCH_ADDR            UINT8_C(0x54)

/** Error code definitions */
#define BMI160_OK                         INT8_C(0)
#define BMI160_E_NULL_PTR                 INT8_C(-1)
#define BMI160_E_COM_FAIL                 INT8_C(-2)
#define BMI160_E_DEV_NOT_FOUND            INT8_C(-3)
#define BMI160_E_OUT_OF_RANGE             INT8_C(-4)
#define BMI160_E_ACCEL_ODR_BW_INVALID   INT8_C(-6)
#define BMI160_E_GYRO_ODR_BW_INVALID    INT8_C(-7)
#define BMI160_E_LWP_PRE_FLTR_INT_INVALID INT8_C(-8)
#define BMI160_E_LWP_PRE_FLTR_INVALID   INT8_C(-9)

/** BMI160 unique chip identifier */
#define BMI160_CHIP_ID                   UINT8_C(0xD1)

/** Soft reset command */
#define BMI160_SOFT_RESET_CMD            UINT8_C(0xb6)
#define BMI160_SOFT_RESET_DELAY_MS       UINT8_C(15)

/** Power mode settings */
/* Accel power mode */
#define BMI160_ACCEL_NORMAL_MODE         UINT8_C(0x11)
#define BMI160_ACCEL_LOWPOWER_MODE       UINT8_C(0x12)
#define BMI160_ACCEL_SUSPEND_MODE        UINT8_C(0x10)

/* Gyro power mode */
#define BMI160_GYRO_SUSPEND_MODE         UINT8_C(0x14)
#define BMI160_GYRO_NORMAL_MODE          UINT8_C(0x15)
#define BMI160_GYRO_FASTSTARTUP_MODE     UINT8_C(0x17)

/** Range settings */
/* Accel Range */
#define BMI160_ACCEL_RANGE_2G            UINT8_C(0x03)
#define BMI160_ACCEL_RANGE_4G            UINT8_C(0x05)
#define BMI160_ACCEL_RANGE_8G            UINT8_C(0x08)
#define BMI160_ACCEL_RANGE_16G           UINT8_C(0x0C)

/* Gyro Range */
#define BMI160_GYRO_RANGE_2000_DPS       UINT8_C(0x00)
#define BMI160_GYRO_RANGE_1000_DPS       UINT8_C(0x01)
#define BMI160_GYRO_RANGE_500_DPS        UINT8_C(0x02)
#define BMI160_GYRO_RANGE_250_DPS        UINT8_C(0x03)
#define BMI160_GYRO_RANGE_125_DPS        UINT8_C(0x04)

/** Bandwidth settings */
/* Accel Bandwidth */
#define BMI160_ACCEL_BW_OSR4_AVG1        UINT8_C(0x00)
#define BMI160_ACCEL_BW_OSR2_AVG2        UINT8_C(0x01)
#define BMI160_ACCEL_BW_NORMAL_AVG4      UINT8_C(0x02)
#define BMI160_ACCEL_BW_RES_AVG8         UINT8_C(0x03)
#define BMI160_ACCEL_BW_RES_AVG16        UINT8_C(0x04)
#define BMI160_ACCEL_BW_RES_AVG32        UINT8_C(0x05)
#define BMI160_ACCEL_BW_RES_AVG64        UINT8_C(0x06)
#define BMI160_ACCEL_BW_RES_AVG128       UINT8_C(0x07)

#define BMI160_GYRO_BW_OSR4_MODE         UINT8_C(0x00)
#define BMI160_GYRO_BW_OSR2_MODE         UINT8_C(0x01)
#define BMI160_GYRO_BW_NORMAL_MODE       UINT8_C(0x02)

/* Output Data Rate settings */
/* Accel Output data rate */
#define BMI160_ACCEL_ODR_RESERVED        UINT8_C(0x00)
#define BMI160_ACCEL_ODR_0_78HZ          UINT8_C(0x01)
#define BMI160_ACCEL_ODR_1_56HZ          UINT8_C(0x02)
#define BMI160_ACCEL_ODR_3_12HZ          UINT8_C(0x03)
#define BMI160_ACCEL_ODR_6_25HZ          UINT8_C(0x04)
#define BMI160_ACCEL_ODR_12_5HZ          UINT8_C(0x05)
#define BMI160_ACCEL_ODR_25HZ            UINT8_C(0x06)
#define BMI160_ACCEL_ODR_50HZ            UINT8_C(0x07)
#define BMI160_ACCEL_ODR_100HZ           UINT8_C(0x08)
#define BMI160_ACCEL_ODR_200HZ           UINT8_C(0x09)
#define BMI160_ACCEL_ODR_400HZ           UINT8_C(0x0A)
#define BMI160_ACCEL_ODR_800HZ           UINT8_C(0x0B)
#define BMI160_ACCEL_ODR_1600HZ          UINT8_C(0x0C)
#define BMI160_ACCEL_ODR_RESERVED0       UINT8_C(0x0D)
#define BMI160_ACCEL_ODR_RESERVED1       UINT8_C(0x0E)
#define BMI160_ACCEL_ODR_RESERVED2       UINT8_C(0x0F)

/* Gyro Output data rate */
#define BMI160_GYRO_ODR_RESERVED         UINT8_C(0x00)
#define BMI160_GYRO_ODR_25HZ             UINT8_C(0x06)
#define BMI160_GYRO_ODR_50HZ             UINT8_C(0x07)
#define BMI160_GYRO_ODR_100HZ            UINT8_C(0x08)
#define BMI160_GYRO_ODR_200HZ            UINT8_C(0x09)
#define BMI160_GYRO_ODR_400HZ            UINT8_C(0x0A)
#define BMI160_GYRO_ODR_800HZ            UINT8_C(0x0B)
#define BMI160_GYRO_ODR_1600HZ           UINT8_C(0x0C)
#define BMI160_GYRO_ODR_3200HZ           UINT8_C(0x0D)

/* Maximum limits definition */
#define BMI160_ACCEL_ODR_MAX             UINT8_C(15)
#define BMI160_ACCEL_BW_MAX              UINT8_C(2)
#define BMI160_ACCEL_RANGE_MAX           UINT8_C(12)
#define BMI160_GYRO_ODR_MAX              UINT8_C(13)
#define BMI160_GYRO_BW_MAX               UINT8_C(2)
#define BMI160_GYRO_RANGE_MAX            UINT8_C(4)

/* Sensor & time select definition*/
#define BMI160_ACCEL_SEL    UINT8_C(0x01)
#define BMI160_GYRO_SEL     UINT8_C(0x02)
#define BMI160_TIME_SEL     UINT8_C(0x04)

/* Sensor Offset */
// Gyro offset
#define BMI160_GYRO_X_OFFSET INT8_C(9)
#define BMI160_GYRO_Y_OFFSET INT8_C(-4)
#define BMI160_GYRO_Z_OFFSET INT8_C(-7)
// Accelerometer offset
#define BMI160_ACC_X_OFFSET    -0.03
#define BMI160_ACC_Y_OFFSET     0.07
#define BMI160_ACC_Z_OFFSET    -0.03

// (°/s) to (rad/s)
#define DPS2RPS                 0.01745329251994329576f

// Earths gravitational acceleration
#define G_MPS2                  9.81000000000000000000f

/* Sensor select mask*/
#define BMI160_SEN_SEL_MASK   UINT8_C(0x07)

/** Mask definitions for INT_EN registers */
#define BMI160_DATA_RDY_INT_EN_MASK             UINT8_C(0x10)

/** Mask definitions for INT_MAP register */
#define BMI160_INT1_DATA_READY_MASK             UINT8_C(0x80)

/** Mask definitions for INT_OUT_CTRL register */
#define BMI160_INT1_PULL_PUSH_ACTIVE_HIGH_EDGE_TRIG UINT8_C(0x0B)

/** Mask definitions for INT_LATCH register */
#define BMI160_INT1_NON_LATCHED_MASK               UINT8_C(0x00)

/* Error code mask */
#define BMI160_ERR_REG_MASK   UINT8_C(0x0F)

/* BMI160 I2C address */
#define BMI160_I2C_ADDR                 UINT8_C(0x68)

/**
 * @enum eBmi160AnySigMotionActiveInterruptState
 * @brief bmi160 active state of any & sig motion interrupt.
 */
enum eBmi160AnySigMotionActiveInterruptState {
  eBmi160BothAnySigMotionDisabled = -1, /**< Both any & sig motion are disabled */
  eBmi160AnyMotionEnabled,              /**< Any-motion selected */
  eBmi160SigMotionEnabled               /**< Sig-motion selected */
};

/**
 * @struct bmi160Cfg
 * @brief bmi160 sensor configuration structure
 */
struct bmi160Cfg {
  uint8_t power;  /**< power mode */
  uint8_t odr;    /**< output data rate */
  uint8_t range;  /**< range */
  uint8_t bw;     /**< bandwidth */
};

/**
 * @brief Aux sensor configuration structure
 */
struct bmi160AuxCfg {
  uint8_t auxSensorEnable : 1;  /**< Aux sensor, 1 - enable 0 - disable */
  uint8_t manualEnable : 1;     /**< Aux manual/auto mode status */
  uint8_t auxRdBurstLen : 2;    /**< Aux read burst length */
  uint8_t auxOdr :4;            /**< output data rate */
  uint8_t auxI2cAddr;           /**< i2c addr of auxiliary sensor */
};

/* type definitions */
typedef int8_t (*bmi160ComFptrT)(uint8_t dev_addr, uint8_t reg_addr, uint8_t *data, uint16_t len);
typedef void (*bmi160DelayFptrT)(uint32_t period);

struct bmi160Dev {
  uint8_t chipId;   /**< Chip Id */
  uint8_t id;       /**< Device Id */
  enum eBmi160AnySigMotionActiveInterruptState any_sig_sel;/**< Hold active interrupts status for any and sig motion 0 - Any-motion enable, 1 - Sig-motion enable,  -1 neither any-motion nor sig-motion selected */
  struct bmi160Cfg accelCfg;                               /**< Structure to configure Accel sensor */
  struct bmi160Cfg prevAccelCfg;/**< Structure to hold previous/old accel config parameters.This is used at driver level to prevent overwriting of samedata, hence user does not change it in the code */
  struct bmi160Cfg gyroCfg;     /**< Structure to configure Gyro sensor */
  struct bmi160Cfg prevGyroCfg; /**< Structure to hold previous/old gyro config parameters.This is used at driver level to prevent overwriting of same data, hence user does not change it in the code */
  struct bmi160AuxCfg auxCfg;   /**< Structure to configure the auxiliary sensor */
  struct bmi160AuxCfg prevAuxCfg;/**< Structure to hold previous/old aux config parameters.This is used at driver level to prevent overwriting of samedata, hence user does not change it in the code */
  struct bmi160FifoFrame *fifo; /**< FIFO related configurations */
  bmi160ComFptrT read;          /**< Read function pointer */
  bmi160ComFptrT write;         /**< Write function pointer */
  bmi160DelayFptrT delayMs;     /**<  Delay function pointer */
};

/**
 * @brief bmi160 sensor data structure which comprises of accel data
 */
struct bmi160SensorData {
  int16_t x;           /**< X-axis sensor data */
  int16_t y;           /**< Y-axis sensor data */
  int16_t z;           /**< Z-axis sensor data */
  uint32_t sensortime; /**< sensor time */
};

class BMI160{
public:
  BMI160();

  /**
   * @fn Init
   * @brief set the i2c addr and init the i2c.
   * @param i2c_addr  bmi160 i2c addr
   * @n     0x68: connect SDIO pin of the BMI160 to GND which means the default I2C address
   * @n     0x69: set I2C address by parameter
   * @return BMI160_OK(0) means success
   */
  int8_t Init(int8_t i2c_addr = BMI160_I2C_ADDR);

  /**
   * @fn getSensorData
   * @brief select mode and save returned data to parameter data.
   * @param type  three type
   * @n     onlyAccel    :   only get the accel data
   * @n     onlyGyro     :   only get the gyro data
   * @n     bothAccelGyro:   get boath accel and gyro data
   * @param data  save returned data to parameter data
   * @return BMI160_OK(0) means succse
   */
  int8_t getSensorData(uint8_t type,int16_t* data);

  /**
   * @fn getAccelGyroData
   * @brief get the accel and gyro data
   * @param data pointer to store the accel and gyro data
   * @return BMI160_OK(0) means succse
   */
  int8_t getAccelGyroData(int16_t* data);

  /**
   * @fn getAccelGyroData
   * @brief get the accel and gyro data
   * @param data pointer to store the accel and gyro data
   * @param timestamp pointer to store the timestamp for accel and gyro
   * @return BMI160_OK(0) means succse
   */
  int8_t getAccelGyroData(int16_t* data, uint32_t* timestamp);

  /**
   * @fn softReset
   * @brief reset bmi160 hardware
   * @return BMI160_OK(0) means success
   */
  int8_t softReset();

  /**
   * @fn setInt
   * @brief setup the bmi160 interrupt 1
   * @return BMI160_OK(0) means success
   */
  int8_t setInt();

  private:
    int8_t Init(struct bmi160Dev *dev);

    int8_t softReset(struct bmi160Dev *dev);
    void   defaultParamSettg(struct bmi160Dev *dev);

    int8_t setSensConf();
    int8_t setSensConf(struct bmi160Dev *dev);

    int8_t setAccelConf(struct bmi160Dev *dev);
    int8_t checkAccelConfig(uint8_t *data, struct bmi160Dev *dev);
    int8_t processAccelOdr(uint8_t *data, struct bmi160Dev *dev);
    int8_t processAccelBw(uint8_t *data, struct bmi160Dev *dev);
    int8_t processAccelRange(uint8_t *data, struct bmi160Dev *dev);

    int8_t setGyroConf(struct bmi160Dev *dev);
    int8_t checkGyroConfig(uint8_t *data, struct bmi160Dev *dev);
    int8_t processGyroOdr(uint8_t *data, struct bmi160Dev *dev);
    int8_t processGyroBw(uint8_t *data, struct bmi160Dev *dev);
    int8_t processGyroRange(uint8_t *data, struct bmi160Dev *dev);

    int8_t setPowerMode(struct bmi160Dev *dev);
    int8_t setAccelPwr(struct bmi160Dev *dev);
    int8_t processUnderSampling(uint8_t *data, struct bmi160Dev *dev);
    int8_t setGyroPwr(struct bmi160Dev *dev);

    int8_t checkInvalidSettg( struct bmi160Dev *dev);

    int8_t getSensorData(uint8_t select_sensor, struct bmi160SensorData *accel, struct bmi160SensorData *gyro,struct bmi160Dev *dev);
    int8_t getAccelGyroData(uint8_t len, struct bmi160SensorData *accel, struct bmi160SensorData *gyro, struct bmi160Dev *dev);

    int8_t getRegs(uint8_t reg_addr, uint8_t * data, uint16_t len, struct bmi160Dev *dev);
    int8_t setRegs(uint8_t reg_addr, uint8_t * data, uint16_t len, struct bmi160Dev *dev);

    int8_t I2cGetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len);
    int8_t I2cSetRegs(struct bmi160Dev *dev, uint8_t reg_addr, uint8_t *data, uint16_t len);

    int8_t setInt(struct bmi160Dev *dev);

    struct bmi160Dev* Obmi160;
    struct bmi160SensorData* Oaccel;
    struct bmi160SensorData* Ogyro;
};

/**
  * @fn offset
  * @brief Apply offset to the sensor output
  * @param accelGyro Data from the sensor
  * @param rawAccelGyro Offseted data
  */
void offset(int16_t* accelGyro, float* rawAccelGyro);
