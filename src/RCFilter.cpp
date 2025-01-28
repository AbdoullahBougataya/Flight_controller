#include "../include/RCFilter.h"

void RCFilter_Init(RCFilter * filt, float cutoffFreqHz, float sampleTime) {

    /* Compute equivalent 'RC' constant from cut-off frequency */
    float RC = 1.0f / (2 * PI * cutoffFreqHz);
    /* Pre-compute filter coefficients for first-order low-pass filter */
    filt->coeff[0] = sampleTime / (sampleTime + RC);
    filt->coeff[1] = RC / (sampleTime + RC);

    /* Clear the outputs */
    filt->out[0] = 0.0f;
    filt->out[1] = 0.0f;

}

float RCFilter_Update(RCFilter * filt, float inp) {

    /* Shift output samples */
    filt->out[1] = filt->out[0];

    /* Compute new output sample */
    filt->out[0] = filt->coeff[0] * inp + filt->coeff[1] * filt->out[1];

    /* Return filtered sample */
    return (filt->out[0]);

}
