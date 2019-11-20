#ifndef FFT_H_
#define FFT_H_

#include "stm32l4xx_hal.h"
#include "arm_math.h"

#define SAMPLE_RATE 1000.0 //(2.0 * MAX_FREQ)
#define NUM_OF_FFT_SAMPLES 512
#define FFT_SIZE (NUM_OF_FFT_SAMPLES / 2)

void initFFT();
void createFFTBuffer(uint32_t *ADCBuffer);
void getMaxHertz(float *hertz);
void transmitOutputBuffer();
void transmitInputBuffer();

#endif
