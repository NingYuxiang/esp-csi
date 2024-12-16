/* SPI Slave example, receiver (uses SPI Slave driver to communicate with sender)

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "driver/spi_slave.h"
#include "driver/gpio.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <app_ifft.h>
#include "IQmathLib.h"
#define PI 3.14159265358979323846f



// Bit reversal of given index 'x' with 'log2n' bits
unsigned int inline bitReverse(unsigned int x, int log2n) {
    int n = 0;
    for (int i = 0; i < log2n; i++) {
        n <<= 1;
        n |= (x & 1);
        x >>= 1;
    }
    return n;
}

void IRAM_ATTR fft(Complex *X, int N, int inverse) 
{
    int log2N = log2(N);
    Complex *temp = (Complex *)malloc(N * sizeof(Complex));

    // Bit-reversed addressing permutation
    for (int i = 0; i < N; i++) {
        temp[i] = X[bitReverse(i, log2N)];
    }
    for (int i = 0; i < N; i++) {
        X[i] = temp[i];
    }

    // Cooley-Tukey iterative FFT
    for (int s = 1; s <= log2N; ++s) {
        int m = 1 << s; // 2 power s
        int m2 = m >> 1; // m/2
        Complex w;
        w.real = 1.0;
        w.imag = 0.0;
        Complex wm;
        wm.real = cosf(-2.0f * PI / m);
        wm.imag = sinf(-2.0f * PI / m);
        if (inverse) wm.imag = -wm.imag;

        for (int j = 0; j < m2; ++j) {
            for (int k = j; k < N; k += m) {
                Complex t, u;
                u = X[k];
                t.real = w.real * X[k + m2].real - w.imag * X[k + m2].imag;
                t.imag = w.real * X[k + m2].imag + w.imag * X[k + m2].real;
                X[k].real = u.real + t.real;
                X[k].imag = u.imag + t.imag;
                X[k + m2].real = u.real - t.real;
                X[k + m2].imag = u.imag - t.imag;
            }
            float tmpReal = w.real * wm.real - w.imag * wm.imag;
            w.imag = w.real * wm.imag + w.imag * wm.real;
            w.real = tmpReal;
        }
    }

    // Scale for inverse FFT
    if (inverse) {
        for (int i = 0; i < N; i++) {
            X[i].real /= N;
            X[i].imag /= N;
        }
    }

    free(temp);
}

// Simple test function for FFT and IFFT
void test_fft_ifft() 
{
    int N = 64;
    Complex x[N];
    
    // Sample input: 0, 1, 2, ... , N-1
    for (int i = 0; i < N; i++) {
        x[i].real = i;
        x[i].imag = i/2;
    }

       // Perform IFFT
    fft(x, N, 1); // inverse = 1 for inverse FFT
    // printf("\nIFFT:\n");
    // for (int i = 0; i < N; i++) {
    //     printf("%f + %fi\n", x[i].real, x[i].imag);
    // }

    // Perform FFT
    fft(x, N, 0); // inverse = 0 for forward FFT
    // printf("FFT:\n");
    // for (int i = 0; i < N; i++) {
    //     printf("%f + %fi\n", x[i].real, x[i].imag);
    // }

 
}
