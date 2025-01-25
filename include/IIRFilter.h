#ifndef IIR_FILTER_H
#define IIR_FILTER_H

#include <Arduino.h>
#include <stdint.h>

typedef struct {
    float alpha;
    float beta;
    float output;
} IIRFilter;

void IIRFilter_Init(IIRFilter *filt, float alpha, float beta);

void IIRFilter_Update(IIRFilter *filt, float input);
