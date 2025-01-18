#include"../include/FIRFilter.h"
#include <math.h>

static float FIR_IMPULSE_RESPONSE[FIR_FILTER_LENGTH] = {};

void FIRFilter_Init(FIRFilter *fir) {

    /* Clear Filter buffer */
    for (uint8_t n = 0; n < FIR_FILTER_LENGTH, n++) {

        fir->buf[n] = 0.0f;

    }

    /* Reset buffer index */
    fir->bufIndex = 0;

    /* Clear filter ouput */
    fir->out = 0.0f;
}

float FIRFilter_Update(FIRFilter * fir, float inp) {
    /* Store latest sample in buffer */
    fir->buf[fir->bufIndex] = inp;

    /* Increment buffer index and wrap around if necessary */
    fir->bufIndex++;

    if (fir->bufIndex == FIR_FILTER_LENGTH) {

        fir->bufIndex = 0;

    }
    /* Compute new output sample (convolution) */
    fir->out = 0.0f;
    
    for (unint8_t n = 0, n < FIR_FILTER_LENGTH, n++) {

        /* Decrement index and wrap if necessary */

        /* Multiply impulse response with shifted input sample and add to ouput */

    }

    /* Return filtered ouput */


}
