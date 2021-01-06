#include "display.h"

// https://www.st.com/resource/en/user_manual/dm00440740-stm32cube-bsp-drivers-development-guidelines-stmicroelectronics.pdf

LCD_HandleTypeDef hlcd;
UART_HandleTypeDef huart2;

volatile uint8_t JoyState = 0;
volatile uint8_t FLAG_JOY_UPDATE = 0;

int8_t displayMode = 0;
int8_t displayData = 0;

DISPLAY_StatusTypeDef DISPLAY_INIT(uint8_t peripherals) {

    // Initialize UART in normal mode
    if ((peripherals & INIT_UART) == INIT_UART) {
        MX_USART2_UART_Init();
    }

    if ((peripherals & INIT_LCD) == INIT_LCD) {
        //redundant?
        MX_LCD_Init();

        // Initialize LCD Display
        BSP_LCD_GLASS_Init();
    }

    //Initialize LEDS
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_GREEN);

    // Initialize joystick in Interrupt mode
    if ((peripherals & INIT_JOYSTICK) == INIT_JOYSTICK) {
        if (BSP_JOY_Init(JOY_MODE_EXTI) != HAL_OK) {
            // Toggle Red LED - Not good.
            BSP_LED_On(LED_RED);

            return DISPLAY_ERROR;
        }
    }

    //Clear LCD Display
    BSP_LCD_GLASS_Clear();

    //Toggle Green LED - Everything OK.
    BSP_LED_On(LED_GREEN);

    return DISPLAY_OK;
}


DISPLAY_StatusTypeDef stub_printf(uint32_t out_device, char *format, ...)
{
    char buffer[64]; //uint8_t * ?

    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);

    switch (out_device) {
        case OUT_LCD:
            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t *) buffer);
            //setLCDBars(signal_quality); //TODO
            break;

            //TODO
        case OUT_AUDIO:
            break;

        case OUT_UART:
            HAL_UART_Transmit(&huart2, (uint8_t *) buffer, strlen(buffer), HAL_MAX_DELAY);
            break;

        case OUT_USB:
            //USBD_CDC_SetTxBuffer(&USBD_Device, buffer, strlen(buffer));
            //USBD_CDC_TransmitPacket(&USBD_Device);
            break;

        default:
            return DISPLAY_ERROR;
    }

    return DISPLAY_OK;
}


// stub function to replace menu.get_mode()
uint8_t get_display_mode(int mode) {
    return mode; //MPH (STUB)
}


void display_data(float32_t radar_output_freq, int mode) {
    displayData = 1;
    // ================ maths ====================
    // https://www.sciencedirect.com/topics/engineering/doppler-frequency-shift

    // calculate the number of Doppler Hertz per m/s for this radar frequency
    static float32_t doppler_freq_shift = 2.0f * CARRIER_FREQUENCY / SPEED_OF_LIGHT;

    float32_t speed_mps, speed_mph, speed_kmh;

    speed_mps = radar_output_freq / doppler_freq_shift;

    // ===========================================

    //display Data in format relevant to mode
    switch (get_display_mode(mode)) {
        case 0:
            speed_mph = speed_mps * MPH_FACTOR;

            stub_printf(OUT_LCD, "%3dmph", (uint16_t) speed_mph);
            stub_printf(OUT_UART, "% 9.1f\r", speed_mph);
            break;

        case 1:
            speed_kmh = speed_mps * KMH_FACTOR;

            stub_printf(OUT_LCD, "%3dkmh", (uint16_t) speed_kmh);
            stub_printf(OUT_UART, "% 9.1f\r", speed_kmh);
            break;

        case 2:
            stub_printf(OUT_LCD, "%2d mps", (uint16_t) speed_mps);
            stub_printf(OUT_UART, "% 9.1f\r", speed_mps);
            break;

        case 3:
            stub_printf(OUT_LCD, "%4dHz", (uint16_t) radar_output_freq);
            stub_printf(OUT_UART, "% 9.1f\r", radar_output_freq);
            break;

        default:
            break;
    }
}


inline void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    BSP_LED_Toggle(LED_RED);
    BSP_LCD_GLASS_Clear();

    FLAG_JOY_UPDATE = 1;

    // check which button was pressed
    // JoyState = BSP_JOY_GetState();
    JoyState = (uint8_t) GPIO_Pin;

    // Clear IO Expander IT
    //BSP_IO_ITClear(BSP_IO_ITGetStatus(JOY_ALL_PINS));
}


void menuUp() {
    displayMode--;
    if (displayMode < 0) displayMode = 3;
    if (displayData == 0) {
        switch (displayMode) {
            case (0):
                stub_printf(OUT_LCD, "MPH");
                break;
            case (1):
                stub_printf(OUT_LCD, "KMH");
                break;
            case (2):
                stub_printf(OUT_LCD, "MPS");
                break;
            case (3):
                stub_printf(OUT_LCD, "Hz");
                break;
        }
    } else display_data(100, displayMode);
}

void menuDown() {
    displayMode++;
    if (displayMode > 3) displayMode = 0;
    if (displayData == 0) {
        switch (displayMode) {
            case (0):
                stub_printf(OUT_LCD, "MPH");
                break;
            case (1):
                stub_printf(OUT_LCD, "KMH");
                break;
            case (2):
                stub_printf(OUT_LCD, "MPS");
                break;
            case (3):
                stub_printf(OUT_LCD, "Hz");
                break;
        }
    } else display_data(100, displayMode);
}

void menuLeft() {
    displayData = 0;
    switch (displayMode) {
        case (0):
            stub_printf(OUT_LCD, "MPH");
            break;
        case (1):
            stub_printf(OUT_LCD, "KMH");
            break;
        case (2):
            stub_printf(OUT_LCD, "MPS");
            break;
        case (3):
            stub_printf(OUT_LCD, "Hz");
            break;
    }
}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */ //static?
void MX_LCD_Init(void) {

    /* USER CODE BEGIN LCD_Init 0 */

    /* USER CODE END LCD_Init 0 */

    /* USER CODE BEGIN LCD_Init 1 */

    /* USER CODE END LCD_Init 1 */
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
    if (HAL_LCD_Init(&hlcd) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN LCD_Init 2 */

    /* USER CODE END LCD_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */ //static?
void MX_USART2_UART_Init(void) {

    /* USER CODE BEGIN USART2_Init 0 */

    /* USER CODE END USART2_Init 0 */

    /* USER CODE BEGIN USART2_Init 1 */

    /* USER CODE END USART2_Init 1 */
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
    if (HAL_UART_Init(&huart2) != HAL_OK) {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}