#include "../include/EMA.h"

unsigned int EMAFilter_Update(EMAFilter *filt, uint16_t remoteController)
{
    out[0] = out[1];
    out[1] = out[2];
    out[2] = out[3];
    out[3] = out[4];
    out[4] = remoteController;
    return out[0] * 0.2f + out[1] * 0.2f + out[2] * 0.2f + out[3] * 0.2f + out[4] * 0.2f;
}
