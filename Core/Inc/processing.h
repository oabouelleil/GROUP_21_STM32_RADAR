//
// Created by oabou on 06/01/2021.
//

#ifndef GROUP_21_STM32_RADAR_PROCESSING_H
#define GROUP_21_STM32_RADAR_PROCESSING_H

#include "arm_math.h"
#include "arm_const_structs.h"

#define FFT_MAX_SIZE       4096
#define SAMPLE_RATE        8000

extern void doFFT(uint16_t *inputBuffer, float32_t *frequency, float32_t *average, float32_t output[],
                  uint32_t *maxIndex, uint16_t fftLengthIndex);

#endif //GROUP_21_STM32_RADAR_PROCESSING_H
