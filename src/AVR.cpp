#include "../include/AVR.h"

unsigned int AVRFilter_Update(AVRFilter *filt, uint16_t inp)
{
    AVRFilter->out[0] = AVRFilter->out[1];
    AVRFilter->out[1] = AVRFilter->out[2];
    AVRFilter->out[2] = AVRFilter->out[3];
    AVRFilter->out[3] = AVRFilter->out[4];
    AVRFilter->out[4] = inp;
    return AVRFilter->out[0] * 0.2f + AVRFilter->out[1] * 0.2f + AVRFilter->out[2] * 0.2f + AVRFilter->out[3] * 0.2f + AVRFilter->out[4] * 0.2f;
}
