#ifndef EMA_FILTER_H
#define EMA_FILTER_H

#include <stdint.h>

typedef struct {
    uint16_t out[5];
} EMAFilter;

uint16_t EMAFilter_Update(EMAFilter *filt, uint16_t remoteController);

#endif
