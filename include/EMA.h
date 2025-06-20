#ifndef EMA_FILTER_H
#define EMA_FILTER_H

#include <stdint.h>

unsigned int EMAFilter_Update(EMAFilter *filt, uint16_t remoteController);

#endif
