
#pragma once
#ifdef __cplusplus
extern "C" {
#endif
#include "freertos/queue.h"
#include <complex.h>
#include <math.h>

typedef struct {
    float real;
    float imag;
} Complex;

void fft(Complex *X, int N, int inverse);

#ifdef __cplusplus
}
#endif