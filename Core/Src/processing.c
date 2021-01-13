#include <stdlib.h>
#include "processing.h"


void doFFT(uint16_t *inputBuffer, float32_t *frequency, float32_t *average, float32_t output[], uint32_t *maxIndex) {
    float32_t maxValue;
    uint16_t fftLength = arm_cfft_sR_f32_len4096.fftLen;

    // Buffer to store adc sample, every other value is an unused imaginary value
    float32_t complexBuffer[FFT_MAX_SIZE * 2];

    makeComplexBuffer(inputBuffer, complexBuffer, fftLength);

    // Fill complexBuffer with testing data
    // ONLY UNCOMMENT WHEN TESTING
    stub_generateTestData(complexBuffer);

    // Process the data through the CFFT/CIFFT module
    arm_cfft_f32(&arm_cfft_sR_f32_len4096, complexBuffer, 0, 1);

    // Process the data through the Complex Magnitude Module for
    //  calculating the magnitude at each bin
    arm_cmplx_mag_f32(complexBuffer, output, fftLength);

    // Ignore the DC Value
    output[0] = 0.0f;

    // Ignore frequencies below 100 Hz
    uint16_t n, top;
    top = 10; //(50*SAMPLE_RATE)/fftLength;
    for (n = 0; n < top; n++) {
        output[n] = 0.0f;
    }

    // Calculates maxValue and returns corresponding BIN value
    arm_max_f32(output, fftLength / 2, &maxValue, maxIndex);

    // Calculate frequency value of peak bin
    float32_t nyquistFrequency = SAMPLE_RATE / 2;
    float32_t hertzPerBin = nyquistFrequency / ((float) fftLength / 2);

    *frequency = hertzPerBin * (float32_t) *maxIndex;

    arm_mean_f32(output, fftLength, average);
}


void stub_generateTestData(float32_t *buffer) {
    static float32_t angle = 0.0f;
    static float32_t frequency = 388.0f; // Hz
    uint16_t i;

    float32_t radiansPerSample = frequency * 2.0f * (float32_t) M_PI / (float32_t) SAMPLE_RATE;

    // 2048 samples
    for (i = 0; i < 2048; i += 2) {
        // Base signal at the current frequency
        buffer[i] = sin(angle);

        // Random noise
        buffer[i] += (float32_t) rand() / (float32_t) RAND_MAX;

        // Set imaginary part to 0
        buffer[i + 1] = 0.0f;

        // Move in the time domain to the next point in the sample
        angle += radiansPerSample;
    }

    // Increase the frequency for the next sample test
    frequency += 10.0f;

    // Set the frequency down after it passes the set limit
    if (frequency > 1200.0f)
        frequency = 100.0f;
}

// convert the integer data buffer from the ADC into complex floating point buffer for FFT processing
void makeComplexBuffer(uint16_t *buffer, float32_t *output, uint16_t fftLength) {

    uint16_t i;
// Convert the uint16 inputs to float32 and store the values in complexBuffer
    for (i = 0; i < fftLength * 2; i += 2) {
        // '& 0x0fff' removes any bits larger than 12 bits, which is the ADC resolution
        // Divide to 4095, which is the max value, to convert to a float range between 0 and 1
        output[i] = (float32_t) (buffer[i / 2] & 0x0fff) / 4095.0f;

        // Set imaginary part to 0
        output[i + 1] = 0.0f;
    }
}