#ifndef RC_FILTER_H
#define RC_FILTER_H

#include <stdint.h>

#define PI                                3.14159265358979323846f  // Pi
#define RCFilter_OK                       0  // Success
#define RCFilter_ERR_INVALID_ARG          1  // Invalid parameters error

typedef struct {
    float coeff[2];
    float out[2];
} RCFilter;

/**
  * @fn RCFilter_Init
  * @brief Initalize the RC Filter
  * @param filt The filter object
  * @param cutoffFreqHz The cutoff frequency in [Hz]
  * @param sampleTime The sampling period in [s]
  * @return RCFilter_OK(0) means success
  */
int8_t RCFilter_Init(RCFilter *filt, float cutoffFreqHz, float sampleTime);

/**
  * @fn RCFilter_Update
  * @brief Update the filter on every ittiration
  * @param filt The filter object
  * @param inp The input signal
  * @return The filtered signal
  */
float RCFilter_Update(RCFilter * filt, float inp);

#endif
