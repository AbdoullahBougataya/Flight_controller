#ifndef COMP_FILTER_H
#define COMP_FILTER_H

#include <Arduino.h>
#include <stdint.h>

#define G_MPS2                            9.81000000000000000000f  // Gravitational acceleration (g)

typedef struct {
    float alpha;
    float out[2];
} CompFilter;

/**
  * @fn RCFilter_Init
  * @brief Initalize the RC Filter
  * @param filt The filter object
  * @param cutoffFreqHz The cutoff frequency in [Hz]
  * @param sampleTime The sampling period in [s]
  */
void RCFilter_Init(CompFilter * filt, float cutoffFreqHz, float sampleTime);

/**
  * @fn RCFilter_Update
  * @brief Update the filter on every ittiration
  * @param filt The filter object
  * @param inp The input signal
  * @return The filtered signal
  */
float RCFilter_Update(CompFilter * filt, float inp);

#endif
