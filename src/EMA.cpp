#include "../include/EMA.h"

unsigned int EMAFilter_Update(EMAFilter *filt, uint16_t inp)
{
    EMAFilter->out[0] = EMAFilter->out[1];
    EMAFilter->out[1] = EMAFilter->out[2];
    EMAFilter->out[2] = EMAFilter->out[3];
    EMAFilter->out[3] = EMAFilter->out[4];
    EMAFilter->out[4] = inp;
    return EMAFilter->out[0] * 0.2f + EMAFilter->out[1] * 0.2f + EMAFilter->out[2] * 0.2f + EMAFilter->out[3] * 0.2f + EMAFilter->out[4] * 0.2f;
}
