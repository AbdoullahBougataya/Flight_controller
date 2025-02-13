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

struct bmi160Dev *imu; // Declaring the imu object

RCFilter lpFRC[6]; // Declaring the RC filter object

#define RAD2DEG 57.29577951308232087680f                      // Radians to degrees (per second)
#define G_MPS2 9.81000000000000000000f                        // Gravitational acceleration (g)
#define PI 3.14159265358979323846f                            // Pi
#define SAMPLING_PERIOD 0.01000000000000000000f               // Sampling period of the sensor in seconds
#define STARTUP_DELAY 100                                     // 100 ms for the microcontroller to start
#define INTERRUPT_1_MCU_PIN 2                                 // The pin that receives the interrupt 1 signal from the IMU
#define RC_LOW_PASS_FLTR_CUTOFF_5HZ 5.00000000000000000000f   // The cutoff frequency for the RC low pass filter
#define RC_LOW_PASS_FLTR_CUTOFF_10HZ 10.00000000000000000000f // The cutoff frequency for the RC low pass filter
#define GYRO_CALIBRATION_SAMPLES_200 200                      // It takes 200 samples to calibrate the gyroscope
#define COMP_FLTR_ALPHA 0.03000000000000000000f               // Complimentary filter coefficient

const int8_t addr = 0x68; // 0x68 for SA0 connected to the ground

volatile bool dataReady = false; // Sensor Data Ready ? yes:true | no:false

char *TAG = "Flight_controller";

uint8_t rslt = 0; // Define the result of the data extraction from the imu

float gyroRateOffset[3] = {0.0}; // Gyro rates offsets

// Define sensor data arrays
int16_t accelGyro[6] = {0};   // Raw data from the sensor
float accelGyroData[6] = {0}; // Data that is going to be processed

// Declare sensor fusion variables
float phiHat_rad = 0.0f;   // Euler Roll
float thetaHat_rad = 0.0f; // Euler Pitch

// Functions
void complementaryFilter(float *filteredAccelGyro, float *phiHat_rad, float *thetaHat_rad, float dt, float alpha);
static void IRAM_ATTR AccelGyroISR(void *arg); // This is the Interrupt Service Routine for retrieving data from the sensor
void cleanup(void); // Cleanup function to free allocated memory
void standby(void); // A function that waits and does nothing

// Section 2: Initialization & setup section.

void app_main(void)
{
    vTaskDelay(STARTUP_DELAY / portTICK_PERIOD_MS);
    TAG = "IMU_Sensor";
    imu = (struct bmi160Dev *)malloc(sizeof(struct bmi160Dev));
    if (imu == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for imu object\n");
        standby();
    }
    imu->id = addr;
    // Reset the BMI160 to erased any preprogrammed instructions
    if (BMI160_softReset(imu) != BMI160_OK)
    {
        ESP_LOGE(TAG, "Reset error\n");
        cleanup();
        standby();
    }
    // Initialize the BMI160 on I²C
    if (BMI160_Init(imu) != BMI160_OK)
    {
        ESP_LOGE(TAG, "Init error\n");
        cleanup();
        standby();
    }

    // Initialize the BMI160 interrupt 1
    if (BMI160_setInt(imu) != BMI160_OK)
    {
        ESP_LOGE(TAG, "Interrupt error\n");
        cleanup();
        standby();
    }

    // Everytime a pulse is received from the sensor, the AccelGyroISR() will set the dataReady to true, which will enable the code to be ran in the loop
    if (gpio_reset_pin(INTERRUPT_1_MCU_PIN) != ESP_OK)
    {
        ESP_LOGE(TAG, "Master pin error\n");
        cleanup();
        standby();
    }
    if (gpio_set_direction(INTERRUPT_1_MCU_PIN, GPIO_MODE_INPUT) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE(TAG, "GPIO error\n");
        cleanup();
        standby();
    }
    if (gpio_set_pull_mode(INTERRUPT_1_MCU_PIN, GPIO_PULLUP_ONLY) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE(TAG, "Parameter error\n");
        cleanup();
        standby();
    }
    if (gpio_set_intr_type(INTERRUPT_1_MCU_PIN, GPIO_INTR_POSEDGE) == ESP_ERR_INVALID_ARG)
    {
        ESP_LOGE(TAG, "Parameter error\n");
        cleanup();
        standby();
    }
    gpio_install_isr_service(0)
    gpio_isr_handler_add(INTERRUPT_1_MCU_PIN, AccelGyroISR, NULL);
    gpio_intr_enable(INTERRUPT_1_MCU_PIN);

    for (uint8_t i = 0; i < 6; i++)
    {
        RCFilter_Init(&lpFRC[i], RC_LOW_PASS_FLTR_CUTOFF_10HZ, SAMPLING_PERIOD); // Initialize the RCFilter fc = 5 Hz ; Ts = 0.01 s
    }

    float gyroRateCumulativeOffset[3] = {0.0}; // Define a temporary variable to sum the offsets

    // For five seconds the gyroscope will be calibrating (make sure you put it on a flat surface)
    ESP_LOGI(TAG, "Calibrating...\n");
    for (int i = 0; i < GYRO_CALIBRATION_SAMPLES_200; i++)
    {

        // Initialize sensor data arrays
        memset(accelGyro, 0, sizeof(accelGyro));
        memset(accelGyroData, 0, sizeof(accelGyroData));

        // Get both accel and gyro data from the BMI160
        // Parameter accelGyro is the pointer to store the data
        rslt = BMI160_getAccelGyroData(imu, accelGyro);
        if (rslt == 0)
        {
            // Formatting the data
            BMI160_offset(accelGyro, accelGyroData);
            for (uint8_t j = 0; j < 3; j++)
            {
                gyroRateCumulativeOffset[j] += accelGyroData[j]; // Accumulating the gyroscope error
            }
        }
        else
        {
            ESP_LOGE(TAG, "!!! Data reading error !!!\n");
        }
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    // Calculate the average offset
    for (uint8_t i = 0; i < 3; i++)
    {
        gyroRateOffset[i] = gyroRateCumulativeOffset[i] / GYRO_CALIBRATION_SAMPLES_200;
    }

    // Section 3: Looping and realtime processing.

    while (1)
    {
        // Checking if there is data ready in the sensor
        if (dataReady)
        {
            dataReady = false; // Reseting the dataReady flag
            // Initialize sensor data arrays
            memset(accelGyro, 0, sizeof(accelGyro));
            memset(accelGyroData, 0, sizeof(accelGyroData));

            // Get both accel and gyro data from the BMI160
            // Parameter accelGyro is the pointer to store the data
            rslt = BMI160_getAccelGyroData(imu, accelGyro);

            // if the data is succesfully extracted
            if (rslt == 0)
            {
                // Format and offset the accelerometer data
                BMI160_offset(accelGyro, accelGyroData);

                // Substract the offsets from the Gyro readings
                for (uint8_t i = 0; i < 3; i++)
                {
                    accelGyroData[i] -= gyroRateOffset[i];
                }

                for (uint8_t i = 0; i < 6; i++)
                {
                    accelGyroData[i] = RCFilter_Update(&lpFRC[i], accelGyroData[i]); // Update the RCFilter
                }

                /*
                    A complimentary filter is a premitive technique of sensor fusion
                    to use both the accelerometer and the gyroscope to predict the
                    euler angles (phi: roll, theta: pitch)
                */
                complementaryFilter(accelGyroData, &phiHat_rad, &thetaHat_rad, SAMPLING_PERIOD, COMP_FLTR_ALPHA); // This function transform the gyro rates and the Accelerometer angles into equivalent euler angles

                // Print the euler angles to the serial monitor
                printf("%f", phiHat_rad * RAD2DEG);
                printf("%f\n", thetaHat_rad * RAD2DEG);
            }
            else
            {
                ESP_LOGE(TAG, "!!! Data reading error !!!\n");
            }
        }
    }
}

// Section 4: Function declarations.

// Accelerometer and Gyroscope interrupt service routine
static void AccelGyroISR(void *arg)
{
    dataReady = true;
    gpio_isr_handler_add(INTERRUPT_1_MCU_PIN, AccelGyroISR, NULL);
    gpio_intr_enable(INTERRUPT_1_MCU_PIN);
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

// Cleanup function to free allocated memory
void cleanup(void)
{
    if (imu != NULL) {
        free(imu);
        imu = NULL;
    }
}

// A function that waits and does nothing
void standby(void)
{
    while (1) {
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
