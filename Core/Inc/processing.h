#ifndef GROUP_21_STM32_RADAR_PROCESSING_H
#define GROUP_21_STM32_RADAR_PROCESSING_H

#include "arm_math.h"
#include "arm_const_structs.h"

/// @brief
#define FFT_MAX_SIZE       4096

/// @brief stub ADC sample rate
#define SAMPLE_RATE        8000

/**
 * @brief performs FFT on given input buffer
 *
 * @param inputBuffer - input
 * @param frequency - output main frequency component reference
 * @param average - output reference
 * @param output - output Full FFT reference
 * @param maxIndex - output index of frequency in output  reference
 */
void doFFT(uint16_t *inputBuffer, float32_t *frequency, float32_t *average, float32_t output[], uint32_t *maxIndex);


/**
 * @brief convert the integer data buffer from the ADC into complex floating point buffer for FFT processing
 *
 * @param buffer - input data buffer pointer
 * @param output - output buffer reference
 * @param fftLength
 *
 * @note private function used internally by doFFT - static
 */
static void makeComplexBuffer(uint16_t *buffer, float32_t *output, uint16_t fftLength);

/**
 * @brief generate dummy test data
 *
 * @param buffer - output buffer reference
 *
 * @note private function used internally by doFFT - static
 */
static void stub_generateTestData(float32_t *buffer);


#endif //GROUP_21_STM32_RADAR_PROCESSING_H
