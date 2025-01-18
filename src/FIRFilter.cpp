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

float FIR
