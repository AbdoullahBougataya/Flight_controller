#include "../include/RCFilter.h"

void RCFilter_Init(RCFilter * filt, float cutoffFreqHz) {

    /* Compute equivalent 'RC' constant from cut-off frequency */
    filt->RC = 1.0f / (2 * PI * cutoffFreqHz);

    /* Clear the outputs and coefficients */
    filt->out[0] = 0.0f;
    filt->out[1] = 0.0f;
    filt->coeff[0] = 0.0f;
    filt->coeff[1] = 0.0f;

}

float RCFilter_Update(RCFilter * filt, float inp, float sampleTime) {

    /* Compute filter coefficients for first-order low-pass filter */
    filt->coeff[0] = sampleTime / (sampleTime + filt->RC);
    filt->coeff[1] = filt->RC / (sampleTime + filt->RC);

    /* Shift output samples */
    filt->out[1] = filt->out[0];

    /* Compute new output sample */
    filt->out[0] = filt->coeff[0] * inp + filt->coeff[1] * filt->out[1];

    /* Return filtered sample */
    return (filt->out[0]);

}
