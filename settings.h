/*************************************************************************************************
 * ============================ Scientifical & mathematical constants ============================
 * ***********************************************************************************************
 */
#define RAD2DEG                          57.29577951308232087680f  // Radians to degrees (per second)
#define G_MPS2                            9.81000000000000000000f  // Gravitational acceleration (g)
#define PI                                3.14159265358979323846f  // π
#define THOUSAND_OVER_PI                318.30988618379067153776f  // 1000/π
#define ALTITUDE                         70.00000000000000000000f  // Current altitude of the Quadcopter
/*=================================================================================================*/

/*************************************************************************************************
 * =========================================== MCU Pins ===========================================
 * ***********************************************************************************************
 */
#define INTERRUPT_1_MCU_PIN              17                        // The pin that receives the interrupt 1 signal from the Barometer
#define LED_PIN                          38                        // The pin that commands the LED
#define PPM_PIN                          20                        // The reciever pin of the flight controller
#define LEFT_FRONT                       14
#define RIGHT_FRONT                       4
#define RIGHT_BACK                        2
#define LEFT_BACK                        19
/*=================================================================================================*/

/*************************************************************************************************
 * =========================================== General settings ===========================================
 * ***********************************************************************************************
 */
#define SAMPLING_PERIOD                   0.01000000000000000000f  // Sampling period of the sensor in seconds
#define SERIAL_BANDWIDTH_115200      115200                        // The serial monitor's bandwidth
#define STARTUP_DELAY                   100                        // 100 ms for the microcontroller to start
#define CHANNEL_NUMBER                    6                        // The number of RC channels
#define LED_BRIGHTNESS                  100                        // The brightness of the onboard LED
/*=================================================================================================*/

/*************************************************************************************************
 * =========================================== Filter ratios ===========================================
 * ***********************************************************************************************
 */
#define RC_LOW_PASS_FLTR_CUTOFF_4HZ       4.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define RC_LOW_PASS_FLTR_CUTOFF_5HZ       5.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define RC_LOW_PASS_FLTR_CUTOFF_7HZ       7.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define RC_LOW_PASS_FLTR_CUTOFF_10HZ     10.00000000000000000000f  // The cutoff frequency for the RC low pass filter
#define COMP_FLTR_ALPHA                   0.10000000000000000000f  // Complimentary filter coefficient
#define COMP_FLTR_2D_ALPHA                0.75000000000000000000f  // 2D Complimentary filter coefficient
/*=================================================================================================*/

/*************************************************************************************************
 * =========================================== Calibrations samples ===========================================
 * ***********************************************************************************************
 */
#define GYRO_CALIBRATION_SAMPLES_200    200                        // It takes 200 samples to calibrate the gyroscope
#define GYRO_CALIBRATION_SAMPLES_400    400                        // It takes 400 samples to calibrate the gyroscope
/*=================================================================================================*/

/*************************************************************************************************
 * =========================================== Controller gains ===========================================
 * ***********************************************************************************************
 */
//////////////////ROLL & PITCH RATES PID///////////////////
#define ROLL_RATE_AND_PITCH_RATE_PROPORTIONAL_GAIN     1.0f
#define ROLL_RATE_AND_PITCH_RATE_INTEGRAL_GAIN         0.0f
#define ROLL_RATE_AND_PITCH_RATE_DERIVATIVE_GAIN       0.0f
///////////////////////////////////////////////////////////
/*-------------------------------------------------------*/
//////////////////////YAW RATE PID/////////////////////////
#define YAW_RATE_PROPORTIONAL_GAIN                     1.0f
#define YAW_RATE_INTEGRAL_GAIN                         0.0f
#define YAW_RATE_DERIVATIVE_GAIN                       0.0f
///////////////////////////////////////////////////////////
/*-------------------------------------------------------*/
//////////////////VERTICAL VELOCITY PID////////////////////
#define VERTICAL_VELOCITY_PROPORTIONAL_GAIN            1.0f
#define VERTICAL_VELOCITY_INTEGRAL_GAIN                0.0f
#define VERTICAL_VELOCITY_DERIVATIVE_GAIN              0.0f
///////////////////////////////////////////////////////////
/*=======================================================*/
#define ANGULAR_GAIN                                   1.0f
/*=======================================================*/
//////////////////ROLL & PITCH CLAMPING////////////////////
#define ROLL_AND_PITCH_MIN_LIMIT                    -500.0f
#define ROLL_AND_PITCH_MAX_LIMIT                     500.0f
///////////////////////////////////////////////////////////
/*-------------------------------------------------------*/
//////////////////////YAW CLAMPING/////////////////////////
#define YAW_MIN_LIMIT                               -500.0f
#define YAW_MAX_LIMIT                                500.0f
///////////////////////////////////////////////////////////
/*-------------------------------------------------------*/
///////////////VERTICAL VELOCITY CLAMPING//////////////////
#define VERTICAL_V_MIN_LIMIT                        -500.0f
#define VERTICAL_V_MAX_LIMIT                         500.0f
///////////////////////////////////////////////////////////
/*=======================================================*/
 /*=================================================================================================*/
