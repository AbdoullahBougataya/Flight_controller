#include <stdio.h>
#include <string.h>
#include <math.h>
#include "inttypes.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "../include/BMI160.h"
#include "../include/PID.h"
#include "../include/RCFilter.h"

// Section 1: Constants & Global variables declarations.

BMI160 imu; // Declaring the imu object

RCFilter lpFRC[6]; // Declaring the RC filter object

#define RAD2DEG                         57.29577951308232087680f   // Radians to degrees (per second)
#define G_MPS2                           9.81000000000000000000f   // Gravitational acceleration (g)
#define PI                               3.14159265358979323846f   // Pi
#define SAMPLING_PERIOD                  0.01000000000000000000f   // Sampling period of the sensor in seconds
#define STARTUP_DELAY                  100                         // 100 ms for the microcontroller to start
#define INTERRUPT_1_MCU_PIN              2                         // The pin that receives the interrupt 1 signal from the IMU
#define RC_LOW_PASS_FLTR_CUTOFF_5HZ      5.00000000000000000000f   // The cutoff frequency for the RC low pass filter
#define RC_LOW_PASS_FLTR_CUTOFF_10HZ    10.00000000000000000000f   // The cutoff frequency for the RC low pass filter
#define GYRO_CALIBRATION_SAMPLES_200   200                         // It takes 200 samples to calibrate the gyroscope
#define COMP_FLTR_ALPHA                  0.03000000000000000000f   // Complimentary filter coefficient
#define SENSORS_OUTPUTS                  8                         // The number of output variables the sensors have
#define MOTORS_NMBR                      4                         // The quadcopter has 4 motors to throttle

volatile bool IMUDataReady = false; // IMU Data Ready ? yes:true | no:false

float gyroRateOffset[3] = {0.0}; // Gyro rates offsets

// Tasks
void getSensorsData(void *arg); // Gets the Sensors data
void controller(void *arg); // Operates the controller
void motorsCommand(void *arg); // Commands the motors

// Tasks handles
TaskHandle_t getSensorsDataTaskHandle;
TaskHandle_t controllerTaskHandle;
TaskHandle_t motorsCommandTaskHandle;

// Queue handles
QueueHandle_t sensorsToController;
QueueHandle_t controllerToMotorsCommand;

// Functions
void complementaryFilter(float *filteredAccelGyro, float *phiHat_rad, float *thetaHat_rad, float dt, float alpha);
static void IRAM_ATTR AccelGyroISR(void *arg); // This is the Interrupt Service Routine for retrieving data from the sensor
void standby(void); // A function that waits and does nothing
void IMUStartUpSequence(BMI160 *imu); // A function that starts up the IMU
void attachInterrupt(uint8_t intr_pin, gpio_isr_t ISR, gpio_int_type_t edge); // Attaches the interrupt to an MCU pin
void calibrateGyroscope(int num); // Calibrates the gyroscope
void dataRCLowPassFilterInit(RCFilter *filt, float cutoffFreqHz, float sampleTime); // Initialize a 6D RC Low pass filter
void offset(int16_t* accelGyro, float* accelGyroDataRPS); // Adds the offset to the gyroscope
void dataRCLowPassFilterUpdate(RCFilter *filt, float *accelGyroDataRPS); // Update the 6D RC Low pass filter
void sensorsToControllerTransfer(float *accelGyroDataRPS, float *phiHat_rad, float *thetaHat_rad);

// Section 2: Tasks initialization & setup section.

void app_main(void)
{
    (void)vTaskDelay(STARTUP_DELAY / portTICK_PERIOD_MS);

    /* Queues Initialization */
    // Sensors to Controller Queue
    sensorsToController = xQueueCreate(SENSORS_OUTPUTS, sizeof(float)); // 8 Float variables to be sent from the sensor to the Controller

    // Controller to motor Command Queue
    controllerToMotorsCommand = xQueueCreate(MOTORS_NMBR, sizeof(int)); // Throttle of the 4 motors in int

    /* Tasks Initialization */
    // Sensors Task
    if (xTaskCreatePinnedToCore(getSensorsData, "getSensorsData", 2048, NULL, 0, &getSensorsDataTaskHandle, 1) != pdPASS)
    {
        ESP_LOGE("Flight_controller", "Sensors task creation error\n");
        (void)standby();
    }

    // Controller Task
    if (xTaskCreatePinnedToCore(controller, "controller", 2048, NULL, 0, &controllerTaskHandle, 1) != pdPASS)
    {
        ESP_LOGE("Flight_controller", "Controller task creation error\n");
        (void)standby();
    }

    // Motors Command Task
    if (xTaskCreatePinnedToCore(motorsCommand, "motorsCommand", 2048, NULL, 0, &motorsCommandTaskHandle, 1) != pdPASS)
    {
        ESP_LOGE("Flight_controller", "Motors command task creation error\n");
        (void)standby();
    }

    while (1) {(void)vTaskDelete(NULL);}
}

// Section 3: Tasks.

// Sensors Task
void getSensorsData(void *arg)
{
    const char *TAG = "Sensor";

    // Start up and setup the IMU
    const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground

    // Define sensor data arrays
    int16_t accelGyro[6] = {0};   // Raw data from the sensor
    float accelGyroDataRPS[6] = {0}; // Data that is going to be processed

    // Declare sensor fusion variables
    float phiHat_rad = 0.0f;   // Euler Roll
    float thetaHat_rad = 0.0f; // Euler Pitch

    imu.id = addr; // Initializing the address of the imu object
    (void)IMUStartUpSequence(&imu);

    // Everytime a pulse is received from the sensor, the AccelGyroISR() will set the dataReady to true, which will enable the code to be ran in the loop
    (void)attachInterrupt(INTERRUPT_1_MCU_PIN, AccelGyroISR, GPIO_INTR_POSEDGE);

    // Initialize the RC Low pass filter for the sensor data
    (void)dataRCLowPassFilterInit(lpFRC, RC_LOW_PASS_FLTR_CUTOFF_10HZ, SAMPLING_PERIOD);

    // Calibrate the gyroscope
    (void)calibrateGyroscope(GYRO_CALIBRATION_SAMPLES_200);

    while (1)
    {
        // Checking if there is data ready in the sensor
        if (IMUDataReady)
        {
            IMUDataReady = false; // Reseting the dataReady flag

            // Initialize sensor data arrays
            (void)memset(accelGyro, 0, sizeof(accelGyro));
            (void)memset(accelGyroDataRPS, 0, sizeof(accelGyroDataRPS));

            // Get both accel and gyro data from the BMI160
            // Parameter accelGyro is the pointer to store the data
            // if the data is succesfully extracted
            if (BMI160_getAccelGyroData(&imu, accelGyro) == BMI160_OK)
            {
                (void)offset(accelGyro, accelGyroDataRPS); // Adds the offset to the gyroscope

                (void)dataRCLowPassFilterUpdate(lpFRC, accelGyroDataRPS); // Update the 6D RC Low pass filter

                /*
                    A complimentary filter is a premitive technique of sensor fusion
                    to use both the accelerometer and the gyroscope to predict the
                    euler angles (phi: roll, theta: pitch)
                */
                (void)complementaryFilter(accelGyroDataRPS, &phiHat_rad, &thetaHat_rad, SAMPLING_PERIOD, COMP_FLTR_ALPHA); // This function transform the gyro rates and the Accelerometer angles into equivalent euler angles

                // Print the euler angles to the serial monitor
                (void)printf("%f", phiHat_rad * RAD2DEG);
                (void)printf("%f\n", thetaHat_rad * RAD2DEG);

                // Send the output variables to the Controller
                (void)sensorsToControllerTransfer(accelGyroDataRPS, &phiHat_rad, &thetaHat_rad);
            }
            else
            {
                ESP_LOGE(TAG, "!!! Data reading error !!!\n");
            }
        }
    }
}

// Controller Task
void controller(void *arg) {
    /* Controller Code here... */
    while(1) {
        (void)standby();
    }
}

// Motors Command Task
void motorsCommand(void *arg) {
    /* Motor Command Code here... */
    while(1) {
        (void)standby();
    }
}

// Section 5: Function declarations.

// Accelerometer and Gyroscope interrupt service routine
static void AccelGyroISR(void *arg)
{
    IMUDataReady = true;
    if (gpio_isr_handler_add(INTERRUPT_1_MCU_PIN, AccelGyroISR, NULL) != ESP_OK)
    {
        ESP_LOGE("AccelGyroISR", "ISR handling error\n");
        (void)standby();
    }
    if (gpio_intr_enable(INTERRUPT_1_MCU_PIN) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE("AccelGyroISR", "Interrupt enabling parameter error\n");
        (void)standby();
    }
}

// Complementary filter (Check Phil's Lab video for more details)
void complementaryFilter(float *filteredAccelGyro, float *phiHat_rad, float *thetaHat_rad, float dt, float alpha)
{
    // Using gravity to estimate the Euler angles
    float phiHat_acc_rad = atanf(filteredAccelGyro[4] / filteredAccelGyro[5]);                // Roll estimate
    float thetaHat_acc_rad = asinf(fminf(fmaxf(filteredAccelGyro[3] / G_MPS2, -1.0f), 1.0f)); // Pitch estimate

    // Using gyroscope to estimate the euler rates (Transforming body rates to euler rates)
    float phiDot_rps = (sinf(*phiHat_rad) * filteredAccelGyro[1] + cosf(*phiHat_rad) * filteredAccelGyro[2]) * tanf(*thetaHat_rad) + filteredAccelGyro[0]; // Roll rate (rad/s)
    float thetaDot_rps = cosf(*phiHat_rad) * filteredAccelGyro[1] - sinf(*phiHat_rad) * filteredAccelGyro[2];                                             // Pitch rate (rad/s)

    // Complementary filter implementation [Just like mixing the data from the gyroscope and the accelerometer with alpha proportions](alpha should be between 0 and 1)
    *phiHat_rad = fminf(fmaxf(alpha, 0.0f), 1.0f) * phiHat_acc_rad + (1.0f - fminf(fmaxf(alpha, 0.0f), 1.0f)) * (*phiHat_rad + dt * phiDot_rps);         // Roll estimate
    *thetaHat_rad = fminf(fmaxf(alpha, 0.0f), 1.0f) * thetaHat_acc_rad + (1.0f - fminf(fmaxf(alpha, 0.0f), 1.0f)) * (*thetaHat_rad + dt * thetaDot_rps); // Pitch estimate

    // Bound the values of the pitch and roll
    *phiHat_rad = fminf(fmaxf(*phiHat_rad, -PI), PI);
    *thetaHat_rad = fminf(fmaxf(*thetaHat_rad, -PI), PI);

    // If the angle is absolutly less than 1, then it zeroes out
    if (fabs(*phiHat_rad * RAD2DEG) < 1)
    {
        *phiHat_rad = 0;
    }
    if (fabs(*thetaHat_rad * RAD2DEG) < 1)
    {
        *thetaHat_rad = 0;
    }
}

// A function that waits and does nothing
void standby(void)
{
    while (1) {
        (void)vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

// A function that starts up the IMU
void IMUStartUpSequence(BMI160 *imu)
{
    const char *TAG = "IMUStartUpSequence";

    // Reset the BMI160 to erased any preprogrammed instructions
    if (BMI160_softReset(imu) != BMI160_OK)
    {
        ESP_LOGE(TAG, "Reset error\n");
        (void)standby();
    }
    // Initialize the BMI160 on I²C
    if (BMI160_Init(imu) != BMI160_OK)
    {
        ESP_LOGE(TAG, "Init error\n");
        (void)standby();
    }

    // Initialize the BMI160 interrupt 1
    if (BMI160_setInt(imu) != BMI160_OK)
    {
        ESP_LOGE(TAG, "Interrupt error\n");
        (void)standby();
    }
}

// Attaches the interrupt to an MCU pin
void attachInterrupt(uint8_t intr_pin, gpio_isr_t ISR, gpio_int_type_t edge)
{
    const char *TAG = "attachInterrupt";

    if (gpio_reset_pin(intr_pin) != ESP_OK)
    {
        ESP_LOGE(TAG, "Master pin error\n");
        (void)standby();
    }
    if (gpio_set_direction(intr_pin, GPIO_MODE_INPUT) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE(TAG, "GPIO error\n");
        (void)standby();
    }
    if (gpio_set_pull_mode(intr_pin, GPIO_PULLUP_ONLY) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE(TAG, "Pull mode parameter error\n");
        (void)standby();
    }
    if (gpio_set_intr_type(intr_pin, edge) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE(TAG, "Interrupt type parameter error\n");
        (void)standby();
    }
    if (gpio_install_isr_service(0) != ESP_OK)
    {
        ESP_LOGE(TAG, "ISR installation error\n");
        (void)standby();
    }
    if (gpio_isr_handler_add(intr_pin, ISR, NULL) != ESP_OK)
    {
        ESP_LOGE(TAG, "ISR handling error\n");
        (void)standby();
    }
    if (gpio_intr_enable(intr_pin) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE(TAG, "Interrupt enabling parameter error\n");
        (void)standby();
    }
}

// Calibrates the gyroscope
void calibrateGyroscope(int num)
{
    const char *TAG = "calibrateGyroscope";
    // Define sensor data arrays
    int16_t accelGyro[6] = {0};   // Raw data from the sensor
    float accelGyroDataRPS[6] = {0}; // Data that is going to be processed
    float gyroRateCumulativeOffset[3] = {0.0}; // Define a temporary variable to sum the offsets

    // For x seconds the gyroscope will be calibrating (make sure you put it on a flat surface)
    ESP_LOGI(TAG, "Calibrating...\n");
    for (int i = 0; i < num; i++)
    {

        // Initialize sensor data arrays
        (void)memset(accelGyro, 0, sizeof(accelGyro));
        (void)memset(accelGyroDataRPS, 0, sizeof(accelGyroDataRPS));

        // Get both accel and gyro data from the BMI160
        // Parameter accelGyro is the pointer to store the data
        if (BMI160_getAccelGyroData(&imu, accelGyro) == BMI160_OK)
        {
            // Formatting the data
            if (BMI160_offset(accelGyro, accelGyroDataRPS) != BMI160_OK)
            {
                ESP_LOGE(TAG, "Offset error\n");
            }
            for (uint8_t j = 0; j < 3; j++)
            {
                gyroRateCumulativeOffset[j] += accelGyroDataRPS[j]; // Accumulating the gyroscope error
            }
        }
        else
        {
            ESP_LOGE(TAG, "!!! Data reading error !!!\n");
        }
        (void)vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    // Calculate the average offset
    for (uint8_t i = 0; i < 3; i++)
    {
        gyroRateOffset[i] = gyroRateCumulativeOffset[i] / num;
    }
}

// Initialize a 6D RC Low pass filter
void dataRCLowPassFilterInit(RCFilter *filt, float cutoffFreqHz, float sampleTime)
{
    const char *TAG = "IMURCLowPassFilter";
    for (uint8_t i = 0; i < 6; i++)
    {
        // Initialize the RCFilter
        if (RCFilter_Init(&filt[i], cutoffFreqHz, sampleTime) != RCFilter_OK)
        {
            ESP_LOGE(TAG, "RC filter init error\n");
            (void)standby();
        }
    }
}

// Adds the offset to the gyroscope
void offset(int16_t *accelGyro, float *accelGyroDataRPS)
{
    const char *TAG = "offset";
    // Format and offset the accelerometer data
    if (BMI160_offset(accelGyro, accelGyroDataRPS) != BMI160_OK)
    {
        ESP_LOGE(TAG, "Offset error\n");
    }

    // Substract the offsets from the Gyro readings
    for (uint8_t i = 0; i < 3; i++)
    {
        accelGyroDataRPS[i] -= gyroRateOffset[i];
    }
}

// Update the 6D RC Low pass filter
void dataRCLowPassFilterUpdate(RCFilter *filt, float *accelGyroDataRPS)
{
    for (uint8_t i = 0; i < 6; i++)
    {
        accelGyroDataRPS[i] = RCFilter_Update(&filt[i], accelGyroDataRPS[i]); // Update the RCFilter
    }
}

// Transfers data from the sensor to the controller
void sensorsToControllerTransfer(float *accelGyroDataRPS, float *phiHat_rad, float *thetaHat_rad)
{
    // Send the output variables to the Controller
    for (int i = 0; i < 6; i++)
    {
        if (xQueueSend(sensorsToController, &accelGyroDataRPS[i], portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE("Sensors", "Queue send Error\n");
        }
    }
    if (xQueueSend(sensorsToController, &phiHat_rad, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE("Sensors", "Queue send Error\n");
    }
    if (xQueueSend(sensorsToController, &thetaHat_rad, portMAX_DELAY) != pdTRUE)
    {
        ESP_LOGE("Sensors", "Queue send Error\n");
    }
}
