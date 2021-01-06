#ifndef GROUP_21_STM32_RADAR_DISPLAY_H
#define GROUP_21_STM32_RADAR_DISPLAY_H

#include "main.h"

/// @TODO fix this in include paths
#include "STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.h"

/// @internal should probably live in main.h (global_defines.h?)
typedef float float32_t;                           // TODO find out current fpu situation

#define CARRIER_FREQUENCY     10587000000.0f       // 10.587GHz
#define SPEED_OF_LIGHT        299792458.0f         // francesco virgolini
#define KMH_FACTOR            (float32_t)3.6f      // 1 m/s in kmh
#define MPH_FACTOR            (float32_t)2.237f    // 1 m/s in mph


/// @brief DISPLAY_Init peripherals flags
#define INIT_JOYSTICK         (1u << 0u)  // 2^0, bit 0, (unsigned) 0x00
#define INIT_LCD              (1u << 1u)  // 2^1, bit 1
#define INIT_UART             (1u << 2u)  // 2^2, bit 2
#define INIT_UART_DMA         (1u << 3u)  // 2^3, bit 3 // TODO
#define INIT_UART_IT          (1u << 4u)  // 2^4, bit 4 // TODO
#define INIT_USB              (1u << 5u)  // 2^5, bit 5 // TODO
#define INIT_AUDIO            (1u << 6u)  // 2^6, bit 6 // TODO

/// @brief DISPLAT_INIT all peripherals mask
#define INIT_ALL  (INIT_LCD | INIT_UART | INIT_UART_DMA | INIT_UART_IT | INIT_USB | INIT_AUDIO | INIT_JOYSTICK)

/// @brief which joystick button was pressed
extern volatile uint8_t JoyState;

/// @brief set when a joystick button is pressed, must be unset by user
extern volatile uint8_t FLAG_JOY_UPDATE;

///@internal menu mode placeholder variables
extern int8_t displayMode;
extern int8_t displayData;

/// @brief display status enum
typedef enum {
    DISPLAY_OK = 0x00,
    DISPLAY_ERROR = 0x01,
} DISPLAY_StatusTypeDef;

/// @brief output device options enum
typedef enum {
    OUT_LCD,
    OUT_AUDIO,
    OUT_UART,
    OUT_USB
} OutDevice_TypeDef;

/// @internal display menu options enum
enum {
    DISPLAY_MODE_MPH, DISPLAY_MODE_KMH, DISPLAY_MODE_MPS, DISPLAY_MODE_HZ
};

/// @internal Peripheral initialization (should be called through DISPLAY_Init)
static void MX_USART2_UART_Init(void);

static void MX_LCD_Init(void);

/**
 * @brief initialize display peripheraals
 * @param peripherals_mask (bitmask) - see display.h/#INIT_ALL
 * @return display status enum
 *
 * @note primer on bitmask: https://stackoverflow.com/questions/18591924/how-to-use-bitmask
 *
 * @internal currently 6 flags are considered, so uint8 suffices. Best type for masked flags? - no type safety with int
 */
DISPLAY_StatusTypeDef DISPLAY_Init(uint8_t peripherals_mask);

/**
 * @brief single printf like function to print to different devices
 *
 * @param out_device device to print to
 * @param format, ...: format string
 *
 * @return display status enum
 *
 * @internal 32bit enum fields might seem overkill but is processed faster in stm32
 */
DISPLAY_StatusTypeDef stub_printf(OutDevice_TypeDef out_device, char *format, ...);

/// @brief STUB
uint8_t get_display_mode(int mode);

/**
 * @brief process radar output and print to menu (display data)
 *
 * @param radar_output_freq: output from the radar module in hertz
 * @param mode: menu mode
 */
void menuRight(float32_t radar_output_freq, int mode);

/// @brief go up in the menu (joystick up button)
void menuUp(void);

/// @brief go down in the menu (joystick up button)
void menuDown(void);

/// @brief go left in the menu (joystick up button)
void menuLeft(void);


#endif //GROUP_21_STM32_RADAR_DISPLAY_H