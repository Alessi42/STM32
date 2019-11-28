#include "fft.h"

arm_cfft_radix4_instance_f32 S; /* ARM CFFT module */

float32_t Input[NUM_OF_FFT_SAMPLES];
float32_t Output[FFT_SIZE];
uint32_t *ADCBuffer;

float32_t maxValue; /* Max FFT value is stored here */
uint32_t maxIndex; /* Index in Output array where max value is */

float nyquistFrequency= 0.5*SAMPLE_RATE;
float hertzPerBin = 500.0/((float)FFT_SIZE/2);

void initFFT() {
	/* Initialize the CFFT/CIFFT module, intFlag = 0, doBitReverse = 1 */
	arm_cfft_radix4_init_f32(&S, FFT_SIZE, 0, 1);
}


void createFFTBuffer(uint32_t *_ADCBuffer) {
	ADCBuffer = _ADCBuffer;
	static uint16_t i;

	for(i=0;i<NUM_OF_FFT_SAMPLES;i+=2) {
		// set real to be the buffer value centred about 0
		Input[i] = ((float32_t)ADCBuffer[i])/4096.0; //  - 2048.0

		// set the imaginary part to 0
		Input[i+1] = 0;
	}
	return;
}

void getMaxHertz(float *hertz) {
	// calculate the fft of the buffer
	/* Process the data through the CFFT/CIFFT module */
	arm_cfft_radix4_f32(&S, &Input);
	/* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
	arm_cmplx_mag_f32(&Input, &Output, FFT_SIZE);
	// Remove the DC offset
	Output[0] = 0;
	/* Calculates maxValue and returns corresponding value */
	arm_max_f32(&Output, FFT_SIZE / 2, &maxValue, &maxIndex);
	// find the max frequency
	*hertz = hertzPerBin * (float32_t)maxIndex;
	return;
}

void transmitOutputBuffer() {
	TransmitBuffer(&Output, FFT_SIZE, 'o');
}

void transmitInputBuffer() {
	transmitBufferADC(ADCBuffer,NUM_OF_FFT_SAMPLES,'i');
}
