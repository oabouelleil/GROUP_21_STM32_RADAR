#include "display.h"

// peripheral handler globals - note: peripherals in HAL are oop-like internally with dispatch tables
LCD_HandleTypeDef hlcd;
UART_HandleTypeDef huart2;

// interrupt set (volatile) joystick globals
volatile uint8_t JoyState = 0;
volatile uint8_t FLAG_JOY_UPDATE = 0;

// menu position placeholder (y, x). TODO use matrix lingo for menu
int8_t displayMode = 0;
int8_t displayData = 0;


// called by interrupt handler - see stm32l4xx_it.c
inline void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    // DBG toggle led
    BSP_LED_Toggle(LED_RED);
    //BSP_LCD_GLASS_Clear();

    // set update flag
    FLAG_JOY_UPDATE = 1;

    // check which button was pressed
    JoyState = (uint8_t) GPIO_Pin;

    // Clear IO Expander IT - not used in L476 (we have one irq per joy button)
    //BSP_IO_ITClear(BSP_IO_ITGetStatus(JOY_ALL_PINS));
}


DISPLAY_StatusTypeDef DISPLAY_Init(uint8_t peripherals_mask) {

    // Initialize UART in normal mode
    if ((peripherals_mask & INIT_UART) == INIT_UART) {
        if (MX_USART2_UART_Init() != HAL_OK)
            return DISPLAY_UART_ERROR;
    }

    // Initialize LCD
    if ((peripherals_mask & INIT_LCD) == INIT_LCD) {
        // init LCD with .ioc set configuration
        if (MX_LCD_Init() != HAL_OK)
            return DISPLAY_LCD_ERROR;

        // https://www.st.com/resource/en/user_manual/dm00440740-stm32cube-bsp-drivers-development-guidelines-stmicroelectronics.pdf
        // redundant? - might conflict with other custom peripheral configuration - use mx_ instead?
        BSP_LCD_GLASS_Init();
    }

    //Initialize LEDS
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_GREEN);

    // Initialize joystick in Interrupt mode
    if ((peripherals_mask & INIT_JOYSTICK) == INIT_JOYSTICK) {
        if (BSP_JOY_Init(JOY_MODE_EXTI) != HAL_OK)
            return DISPLAY_JOY_ERROR;
    }

    //Clear LCD Display
    BSP_LCD_GLASS_Clear();

    //Toggle Green LED - Everything OK.
    BSP_LED_On(LED_GREEN);

    return DISPLAY_OK;
}


DISPLAY_StatusTypeDef display_printf(OutDevice_TypeDef out_device, char *format, ...) {

    va_list args;
    va_start(args, format);

    // dynamically allocate buffer size
    char buffer[vsnprintf(NULL, 0, format, args) + 1]; //uint8_t
    vsnprintf(buffer, sizeof buffer, format, args);

    va_end(args);

    // select output device
    switch (out_device) {
        case OUT_LCD:
            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t *) buffer);

            break;

        case OUT_AUDIO:
            //TODO

            break;

        case OUT_UART:
            HAL_UART_Transmit(&huart2, (uint8_t *) buffer, strlen(buffer), HAL_MAX_DELAY);
            break;

        case OUT_USB:
            //TODO

            //USBD_CDC_SetTxBuffer(&USBD_Device, buffer, strlen(buffer));
            //USBD_CDC_TransmitPacket(&USBD_Device);
            break;

        default:
            return DISPLAY_ERROR;
    }

    return DISPLAY_OK;
}

// TODO
void stub_display_signal_level(int confidence) {
    BSP_LCD_GLASS_DisplayBar(3);
    //BSP_LCD_GLASS_ClearBar(1);
    //BSP_LCD_GLASS_BarLevelConfig();

}


// stub function to replace menu.get_mode()
uint8_t get_display_mode(int mode) {
    return mode; //MPH (STUB)
}


void menuRight(float32_t radar_output_freq, int mode) {
    displayData = 1;
    // ================ maths ====================
    // https://www.sciencedirect.com/topics/engineering/doppler-frequency-shift

    // calculate the number of Doppler Hertz per m/s for this radar frequency
    static const float32_t doppler_freq_shift = 2.0f * CARRIER_FREQUENCY / SPEED_OF_LIGHT;

    float32_t speed_mps, speed_mph, speed_kmh;

    speed_mps = radar_output_freq / doppler_freq_shift;

    // ===========================================

    //display Data in format relevant to mode
    switch (get_display_mode(mode)) {
        case 0:
            speed_mph = speed_mps * MPH_FACTOR;

            display_printf(OUT_LCD, "%3dmph", (uint16_t) speed_mph);
            display_printf(OUT_UART, "% 9.1f\r", speed_mph);
            break;

        case 1:
            speed_kmh = speed_mps * KMH_FACTOR;

            display_printf(OUT_LCD, "%3dkmh", (uint16_t) speed_kmh);
            display_printf(OUT_UART, "% 9.1f\r", speed_kmh);
            break;

        case 2:
            display_printf(OUT_LCD, "%2d mps", (uint16_t) speed_mps);
            display_printf(OUT_UART, "% 9.1f\r", speed_mps);
            break;

        case 3:
            display_printf(OUT_LCD, "%4dHz", (uint16_t) radar_output_freq);
            display_printf(OUT_UART, "% 9.1f\r", radar_output_freq);
            break;

        default:
            break;
    }
}

void menuUp(void) {
    displayMode--;
    if (displayMode < 0) displayMode = 3;
    if (displayData == 0) {
        switch (displayMode) {
            case (0):
                display_printf(OUT_LCD, "MPH");
                break;
            case (1):
                display_printf(OUT_LCD, "KMH");
                break;
            case (2):
                display_printf(OUT_LCD, "MPS");
                break;
            case (3):
                display_printf(OUT_LCD, "Hz");
                break;
        }
    } else menuRight(100, displayMode);
}

void menuDown(void) {
    displayMode++;
    if (displayMode > 3) displayMode = 0;
    if (displayData == 0) {
        switch (displayMode) {
            case (0):
                display_printf(OUT_LCD, "MPH");
                break;
            case (1):
                display_printf(OUT_LCD, "KMH");
                break;
            case (2):
                display_printf(OUT_LCD, "MPS");
                break;
            case (3):
                display_printf(OUT_LCD, "Hz");
                break;
        }
    } else menuRight(100, displayMode);
}

void menuLeft(void) {
    displayData = 0;
    switch (displayMode) {
        case (0):
            display_printf(OUT_LCD, "MPH");
            break;
        case (1):
            display_printf(OUT_LCD, "KMH");
            break;
        case (2):
            display_printf(OUT_LCD, "MPS");
            break;
        case (3):
            display_printf(OUT_LCD, "Hz");
            break;
        default:
            //panic?
            break;
    }
}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval HAL_OK if all initialization correct
  *
  * @license CubeMX auto generated - (C) COPYRIGHT STMicroelectronics (partially modified)
  */
HAL_StatusTypeDef MX_LCD_Init(void) {

    hlcd.Instance = LCD;
    hlcd.Init.Prescaler = LCD_PRESCALER_1;
    hlcd.Init.Divider = LCD_DIVIDER_16;
    hlcd.Init.Duty = LCD_DUTY_1_4;
    hlcd.Init.Bias = LCD_BIAS_1_4;
    hlcd.Init.VoltageSource = LCD_VOLTAGESOURCE_INTERNAL;
    hlcd.Init.Contrast = LCD_CONTRASTLEVEL_0;
    hlcd.Init.DeadTime = LCD_DEADTIME_0;
    hlcd.Init.PulseOnDuration = LCD_PULSEONDURATION_0;
    hlcd.Init.MuxSegment = LCD_MUXSEGMENT_DISABLE;
    hlcd.Init.BlinkMode = LCD_BLINKMODE_OFF;
    hlcd.Init.BlinkFrequency = LCD_BLINKFREQUENCY_DIV8;
    hlcd.Init.HighDrive = LCD_HIGHDRIVE_DISABLE;

    return HAL_LCD_Init(&hlcd);
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval HAL_OK if all initialization correct
  *
  * @license CubeMX auto generated - (C) COPYRIGHT STMicroelectronics (partially modified)
  */
HAL_StatusTypeDef MX_USART2_UART_Init(void) {

    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

    return HAL_UART_Init(&huart2);
}