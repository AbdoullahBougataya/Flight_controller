#include "../include/IIRFilter.h"

void IIRFilter_Init(IIRFilter *filt, float alpha, float beta) {
    filt->alpha = alpha;
    filt->beta = beta;
    filt->output = 0.0f;
}

float IIRFilter_Update(IIRFilter *filt, float input) {
    /* y[n] = alpha * x[n] - beta * y[n - 1] */
    filt->output = filt->alpha * input - filt->beta * filt->output;

    /* Return filter output */
    return (filt->output);
}
