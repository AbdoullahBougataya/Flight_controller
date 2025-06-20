#ifndef AVR_FILTER_H
#define AVR_FILTER_H

#include <stdint.h>

typedef struct {
    uint16_t out[5]={0, 0, 0, 0, 0};
} AVRFilter;

uint16_t AVRFilter_Update(AVRFilter *filt, uint16_t inp);

#endif
