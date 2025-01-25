#include "../include/RCFilter.h"

void RCFilter_Init(RCFilter * filt, float cutoffFreqHz, float sampleTimeMS) {

    /* Compute equivalent 'RC' constant from cut-off frequency */

    /* Pre-compute filter coefficients for first-order low-pass filter */

}

float RCFilter_Update(RCFilter * filt, float inp) {

    /* Shift output samples */

    /* Compute new output sample */

    /* Return filtered sample */

}
