#ifndef GROUP_21_STM32_RADAR_ADC_H
#define GROUP_21_STM32_RADAR_ADC_H

#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

/// @TODO fix this in include paths
#include "../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.h"

/// @brief output data buffer size
#define ADC_BUF_LEN 4096

/// @internal should probably live in main.h (global_defines.h?)
typedef float float32_t;                           // TODO find out current FPU situation

/// @internal external panic function
extern void Error_Handler(void);

/// @brief output data placeholder
extern uint16_t adc_buf[ADC_BUF_LEN];

/// @internal display printing stubs - implemented on task5
extern UART_HandleTypeDef huart2;
enum {
    OUT_LCD,
    OUT_UART
};

/// @brief ADC status enum
typedef enum {
    ADC_OK = 0x00,
    ADC_ERROR = 0x01,
} ADC_StatusTypeDef;


/// @internal Peripheral initialization functions (should be called directly but through ADC_Init - static)
static void MX_DMA_Init(void);

static void MX_LCD_Init(void);

static void MX_USART2_UART_Init(void);

static void MX_ADC1_Init(void);

/**
 * @brief initialize ADC related periphereals
 * @return ADC status enum
 */
ADC_StatusTypeDef ADC_Init(void);

/**
 *
 * @param - digitalVoltage: 0 to 4095
 * @return - Analog voltage from digital signal
 */
float voltageConversion(uint16_t digitalVoltage);

/**
 * @brief stub function to be replaced by display_printf from task5
 *
 * @param - out_device: The medium to be printed through (OUT_LCD, OUT_UART)
 * @param - format: Formatted string
 * @param - ...: Arguments
 * @return 0 or 1 depending on the peripheral availability
 */
uint8_t stub_printf(uint32_t out_device, char *format, ...);

#endif //GROUP_21_STM32_RADAR_ADC_H
