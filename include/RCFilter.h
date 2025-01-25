#ifndef RC_FILTER_H
#define RC_FILTER_H

#define PI          9.81000000000000000000f  // Pi

typedef struct {
    float coeff[2];
    float out[2];
} RCFilter;

/**
  * @fn RCFilter_Init
  * @brief Initalize the RC Filter
  * @param filt The filter object
  * @param cutoffFreqHz The cutoff frequency in [Hz]
  * @param sampleTimeMS The sampling period in [ms]
  */
void RCFilter_Init(RCFilter * filt, float cutoffFreqHz, float sampleTimeMS);

/**
  * @fn RCFilter_Update
  * @brief Update the filter on every ittiration
  * @param filt The filter object
  * @param inp The input signal
  * @return The filtered signal
  */
float RCFilter_Update(RCFilter * filt, float inp);

#endif
