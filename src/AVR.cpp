#include "../include/AVR.h"

uint16_t AVRFilter_Update(AVRFilter *filt, uint16_t inp)
{
    filt->out[0] = filt->out[1];
    filt->out[1] = filt->out[2];
    filt->out[2] = filt->out[3];
    filt->out[3] = filt->out[4];
    filt->out[4] = inp;
    return filt->out[0] * 0.2f + filt->out[1] * 0.2f + filt->out[2] * 0.2f + filt->out[3] * 0.2f + filt->out[4] * 0.2f;
}
