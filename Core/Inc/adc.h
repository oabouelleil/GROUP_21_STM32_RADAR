//
// Created by georg on 1/6/2021.
//

#ifndef GROUP_21_STM32_RADAR_ADC_H
#define GROUP_21_STM32_RADAR_ADC_H

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.h"

#define ADC_BUF_LEN 4096

extern UART_HandleTypeDef huart2;

enum {
    OUT_LCD,
    OUT_UART
};

extern uint16_t adc_buf[ADC_BUF_LEN];

uint8_t stub_printf(uint32_t out_device, char *format, ...);

void INIT_ADC_DMA(void);

void MX_DMA_Init(void);

void MX_LCD_Init(void);

void MX_USART2_UART_Init(void);

void MX_ADC1_Init(void);

float voltageConversion(uint16_t digitalVoltage);

#endif //GROUP_21_STM32_RADAR_ADC_H
