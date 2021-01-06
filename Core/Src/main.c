/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_host.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

//TODO fix this in include paths, shouldnt have to include drivers
#include "../../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_glass_lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef float float32_t; //TODO float32_t


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define	CARRIER_FREQUENCY	10587000000.0f	// 10.587GHz
#define	SPEED_OF_LIGHT		299792458.0f	//
#define KMH_FACTOR			(float32_t)3.6f		// 1 m/s in kmh
#define MPH_FACTOR			(float32_t)2.237f	// 1 m/s in mph
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
LCD_HandleTypeDef hlcd;

UART_HandleTypeDef huart2;

//static JOYState_TypeDef JoyState = JOY_NONE;

/* USER CODE BEGIN PV */
volatile uint8_t JoyState = 0;
volatile uint8_t FLAG_JOY_UPDATE = 0;

int8_t displayMode = 0;
int8_t displayData = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_LCD_Init(void);
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// +++++++++++++++++++++++++++++ JOYSTICK PLAYGROUND ++++++++++++++++++++++++++++++++++++

// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


// output device options
enum {
    OUT_LCD,
    OUT_AUDIO,
    OUT_UART,
    OUT_USB
};

// display menu options
enum
{
    DISPLAY_MODE_MPH,
    DISPLAY_MODE_KMH,
    DISPLAY_MODE_MPS,
    DISPLAY_MODE_HZ
};

//single fprintf like function to print to different devices
uint8_t stub_printf(uint32_t out_device, // 32bit enum fields might seem overkill but is processed faster
                    char *format, ...)   // accept format string
{
    char buffer[64]; //uint8_t * ?

    va_list args;
    va_start(args, format);
    vsprintf(buffer, format, args);
    va_end(args);

    switch(out_device) {
        case OUT_LCD:
            BSP_LCD_GLASS_Clear();
            BSP_LCD_GLASS_DisplayString((uint8_t *) buffer);
            //setLCDBars(signal_quality); //TODO
            break;

            //TODO
        case OUT_AUDIO:
            break;

        case OUT_UART:
            HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
            break;

        case OUT_USB:
            //USBD_CDC_SetTxBuffer(&USBD_Device, buffer, strlen(buffer));
            //USBD_CDC_TransmitPacket(&USBD_Device);
            break;

        default:
            return 1;
    }

    return 0;
}

// stub function for debugging
inline void stub_display_proxy(char *format, ...)
{
#ifdef UART_DBG
    #pragma message("UART Debug enabled");
	//display to uart
#endif

    //display to LCD
}

// stub function to replace menu.get_mode()
uint8_t get_display_mode(int mode)
{
    return mode; //MPH (STUB)
}


void display_data(float32_t radar_output_freq,int mode)
{
    displayData = 1;
    // ================ maths ====================
    // https://www.sciencedirect.com/topics/engineering/doppler-frequency-shift

    // calculate the number of Doppler Hertz per m/s for this radar frequency
    static float32_t doppler_freq_shift = 2.0f * CARRIER_FREQUENCY / SPEED_OF_LIGHT;

    float32_t speed_mps, speed_mph, speed_kmh;

    speed_mps = radar_output_freq / doppler_freq_shift;

    // ===========================================

    //display Data in format relevant to mode
    switch(get_display_mode(mode))
    {
        case 0:
            speed_mph = speed_mps * MPH_FACTOR;

            stub_printf(OUT_LCD,"%3dmph",(uint16_t)speed_mph);
            stub_printf(OUT_UART,"% 9.1f\r",speed_mph);
            break;

        case 1:
            speed_kmh = speed_mps * KMH_FACTOR;

            stub_printf(OUT_LCD,"%3dkmh",(uint16_t)speed_kmh);
            stub_printf(OUT_UART,"% 9.1f\r",speed_kmh);
            break;

        case 2:
            stub_printf(OUT_LCD,"%2d mps",(uint16_t)speed_mps);
            stub_printf(OUT_UART,"% 9.1f\r",speed_mps);
            break;

        case 3:
            stub_printf(OUT_LCD,"%4dHz",(uint16_t)radar_output_freq);
            stub_printf(OUT_UART,"% 9.1f\r",radar_output_freq);
            break;

        default:
            break;
    }
}


inline void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BSP_LED_Toggle(LED_RED);
    BSP_LCD_GLASS_Clear();

    FLAG_JOY_UPDATE = 1;

    // check which button was pressed
    // JoyState = BSP_JOY_GetState();
    JoyState = (uint8_t) GPIO_Pin;

//		switch(JoyState){
//		case( SEL_JOY_PIN ): //JOY_SEL
//			stub_printf(OUT_LCD,"S");
//			break;
//		case( UP_JOY_PIN ): //JOY_UP
//			stub_printf(OUT_LCD,"U");
//			break;
//		case( DOWN_JOY_PIN ): // JOY_DOWN
//			stub_printf(OUT_LCD,"D");
//			break;
//		case( RIGHT_JOY_PIN ): // RIGHT
//			stub_printf(OUT_LCD,"R");
//			break;
//		case( LEFT_JOY_PIN ): // LEFT
//			stub_printf(OUT_LCD,"L");
//		default:
//			break;
//		}

    // Clear IO Expander IT
    //BSP_IO_ITClear(BSP_IO_ITGetStatus(JOY_ALL_PINS));
}

void menuUp()
{
    displayMode--;
    if (displayMode < 0) displayMode = 3;
    if (displayData == 0) {
        switch(displayMode){
            case (0):
                stub_printf(OUT_LCD,"MPH");
                break;
            case (1):
                stub_printf(OUT_LCD,"KMH");
                break;
            case (2):
                stub_printf(OUT_LCD,"MPS");
                break;
            case (3):
                stub_printf(OUT_LCD,"Hz");
                break;
        }
    } else display_data(100, displayMode);
}

void menuDown()
{
    displayMode++;
    if (displayMode > 3) displayMode = 0;
    if (displayData == 0) {
        switch(displayMode){
            case (0):
                stub_printf(OUT_LCD,"MPH");
                break;
            case (1):
                stub_printf(OUT_LCD,"KMH");
                break;
            case (2):
                stub_printf(OUT_LCD,"MPS");
                break;
            case (3):
                stub_printf(OUT_LCD,"Hz");
                break;
        }
    } else display_data(100, displayMode);
}

void menuLeft ()
{
    displayData = 0;
    switch(displayMode){
        case (0):
            stub_printf(OUT_LCD,"MPH");
            break;
        case (1):
            stub_printf(OUT_LCD,"KMH");
            break;
        case (2):
            stub_printf(OUT_LCD,"MPS");
            break;
        case (3):
            stub_printf(OUT_LCD,"Hz");
            break;
    }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    /* USER CODE BEGIN 1 */

    /* USER CODE END 1 */

    /* MCU Configuration--------------------------------------------------------*/

    /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
    HAL_Init();

    /* USER CODE BEGIN Init */

    /* USER CODE END Init */

    /* Configure the system clock */
    SystemClock_Config();

    /* USER CODE BEGIN SysInit */

    /* USER CODE END SysInit */

    /* Initialize all configured peripherals */
    MX_GPIO_Init();
    MX_USART2_UART_Init();
    MX_USB_HOST_Init();
    MX_LCD_Init();
    /* USER CODE BEGIN 2 */

    // Initialize LCD Display
    BSP_LCD_GLASS_Init();

    // Initialize joystick in Interrupt mode
    BSP_JOY_Init(JOY_MODE_EXTI);

    //Initialize LEDS
    BSP_LED_Init(LED_RED);
    BSP_LED_Init(LED_GREEN);

    //Toggle Green LED - Everything OK.
    BSP_LED_On(LED_GREEN);

    //Clear LCD Display
    BSP_LCD_GLASS_Clear();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */

    //JOYState_TypeDef joyState= JOY_NONE;
    stub_printf(OUT_LCD, "init");

    while (1)
    {
        /* USER CODE END WHILE */
        MX_USB_HOST_Process();

        /* USER CODE BEGIN 3 */

        while(FLAG_JOY_UPDATE) {
            switch(JoyState){
                case( SEL_JOY_PIN ): //JOY_SEL
                    break;
                case( UP_JOY_PIN ): //JOY_UP
                    menuUp();
                    break;
                case( DOWN_JOY_PIN ): // JOY_DOWN
                    menuDown();
                    break;
                case( RIGHT_JOY_PIN ): // RIGHT
                    display_data(100, displayMode);
                    break;
                case( LEFT_JOY_PIN ): // LEFT
                    menuLeft();
                default:
                    break;
            }

            FLAG_JOY_UPDATE = 0;
        }

        // as u see there is nothing related to joystick here - its implemented using interrupts

        // TODO: horizontal bars on rhs to display signal quality. https://www.st.com/resource/en/user_manual/dm00440740-stm32cube-bsp-drivers-development-guidelines-stmicroelectronics.pdf
        //setLCDBars(2);

        //HAL_Delay(5);

    }
    /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

    /** Configure LSE Drive Capability
    */
    HAL_PWR_EnableBkUpAccess();
    __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
    /** Initializes the RCC Oscillators according to the specified parameters
    * in the RCC_OscInitTypeDef structure.
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_LSE
                                       |RCC_OSCILLATORTYPE_MSI;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.MSIState = RCC_MSI_ON;
    RCC_OscInitStruct.MSICalibrationValue = 0;
    RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 20;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    /** Initializes the CPU, AHB and APB buses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                  |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
    {
        Error_Handler();
    }
    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USART2
                                         |RCC_PERIPHCLK_USB;
    PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
    PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
    PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
    PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
    PeriphClkInit.PLLSAI1.PLLSAI1N = 24;
    PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
    PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
    PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        Error_Handler();
    }
    /** Configure the main internal regulator output voltage
    */
    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
    {
        Error_Handler();
    }
    /** Enable MSI Auto calibration
    */
    HAL_RCCEx_EnableMSIPLLMode();
}

/**
  * @brief LCD Initialization Function
  * @param None
  * @retval None
  */
static void MX_LCD_Init(void)
{

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
    if (HAL_LCD_Init(&hlcd) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN LCD_Init 2 */

    /* USER CODE END LCD_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

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
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
        Error_Handler();
    }
    /* USER CODE BEGIN USART2_Init 2 */

    /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_0, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2|GPIO_PIN_3, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9|GPIO_PIN_11, GPIO_PIN_RESET);

    /*Configure GPIO pin Output Level */
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_7, GPIO_PIN_RESET);

    /*Configure GPIO pins : PE2 PE4 PE5 PE6
                             PE7 PE9 */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6
                          |GPIO_PIN_7|GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF13_SAI1;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PE3 PE8 PE0 */
    GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PC13 PC10 */
    GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PC0 PC1 PC2 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PA0 PA1 PA2 PA3
                             PA5 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pin : PA4 */
    GPIO_InitStruct.Pin = GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /*Configure GPIO pins : PB2 PB3 */
    GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PE10 PE11 PE12 PE13
                             PE14 PE15 */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF10_QUADSPI;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

    /*Configure GPIO pins : PB10 PB11 */
    GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pins : PC9 PC11 */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /*Configure GPIO pins : PD0 PD2 */
    GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PD1 PD3 PD4 */
    GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_4;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pin : PD7 */
    GPIO_InitStruct.Pin = GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /*Configure GPIO pins : PB6 PB7 */
    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PB8 */
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /*Configure GPIO pin : PE1 */
    GPIO_InitStruct.Pin = GPIO_PIN_1;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    BSP_LED_Off(LED_GREEN);
    BSP_LED_On(LED_RED);

    stub_printf(OUT_LCD, "PANIC");
    for(;;); //halt

    /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
