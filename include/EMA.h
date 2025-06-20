#ifndef EMA_FILTER_H
#define EMA_FILTER_H

#include <stdint.h>

void EMAFilter_Update(EMAFilter *filt, uint16_t remoteController);

#endif
