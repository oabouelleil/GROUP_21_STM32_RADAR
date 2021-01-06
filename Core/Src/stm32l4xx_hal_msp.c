/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : stm32l4xx_hal_msp.c
  * Description        : This file provides code for the MSP Initialization
  *                      and de-Initialization codes.
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
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN Define */

/* USER CODE END Define */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN Macro */

/* USER CODE END Macro */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* External functions --------------------------------------------------------*/
/* USER CODE BEGIN ExternalFunctions */

/* USER CODE END ExternalFunctions */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void) {
    /* USER CODE BEGIN MspInit 0 */

    /* USER CODE END MspInit 0 */

    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();

    /* System interrupt init*/

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

/**
* @brief LCD MSP Initialization
* This function configures the hardware resources used in this example
* @param hlcd: LCD handle pointer
* @retval None
*/
void HAL_LCD_MspInit(LCD_HandleTypeDef *hlcd) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (hlcd->Instance == LCD) {
        /* USER CODE BEGIN LCD_MspInit 0 */

        /* USER CODE END LCD_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_LCD_CLK_ENABLE();

        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**LCD GPIO Configuration
        PC3     ------> LCD_VLCD
        PA6     ------> LCD_SEG3
        PA7     ------> LCD_SEG4
        PC4     ------> LCD_SEG22
        PC5     ------> LCD_SEG23
        PB0     ------> LCD_SEG5
        PB1     ------> LCD_SEG6
        PB12     ------> LCD_SEG12
        PB13     ------> LCD_SEG13
        PB14     ------> LCD_SEG14
        PB15     ------> LCD_SEG15
        PD8     ------> LCD_SEG28
        PD9     ------> LCD_SEG29
        PD10     ------> LCD_SEG30
        PD11     ------> LCD_SEG31
        PD12     ------> LCD_SEG32
        PD13     ------> LCD_SEG33
        PD14     ------> LCD_SEG34
        PD15     ------> LCD_SEG35
        PC6     ------> LCD_SEG24
        PC7     ------> LCD_SEG25
        PC8     ------> LCD_SEG26
        PA8     ------> LCD_COM0
        PA9     ------> LCD_COM1
        PA10     ------> LCD_COM2
        PA15 (JTDI)     ------> LCD_SEG17
        PC12     ------> LCD_SEG42
        PB4 (NJTRST)     ------> LCD_SEG8
        PB5     ------> LCD_SEG9
        PB9     ------> LCD_COM3
        */
        GPIO_InitStruct.Pin = GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
                              | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9
                              | GPIO_PIN_10 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13
                              | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5
                              | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                              | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF11_LCD;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN LCD_MspInit 1 */

        /* USER CODE END LCD_MspInit 1 */
    }

}

/**
* @brief LCD MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param hlcd: LCD handle pointer
* @retval None
*/
void HAL_LCD_MspDeInit(LCD_HandleTypeDef *hlcd) {
    if (hlcd->Instance == LCD) {
        /* USER CODE BEGIN LCD_MspDeInit 0 */

        /* USER CODE END LCD_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_LCD_CLK_DISABLE();

        /**LCD GPIO Configuration
        PC3     ------> LCD_VLCD
        PA6     ------> LCD_SEG3
        PA7     ------> LCD_SEG4
        PC4     ------> LCD_SEG22
        PC5     ------> LCD_SEG23
        PB0     ------> LCD_SEG5
        PB1     ------> LCD_SEG6
        PB12     ------> LCD_SEG12
        PB13     ------> LCD_SEG13
        PB14     ------> LCD_SEG14
        PB15     ------> LCD_SEG15
        PD8     ------> LCD_SEG28
        PD9     ------> LCD_SEG29
        PD10     ------> LCD_SEG30
        PD11     ------> LCD_SEG31
        PD12     ------> LCD_SEG32
        PD13     ------> LCD_SEG33
        PD14     ------> LCD_SEG34
        PD15     ------> LCD_SEG35
        PC6     ------> LCD_SEG24
        PC7     ------> LCD_SEG25
        PC8     ------> LCD_SEG26
        PA8     ------> LCD_COM0
        PA9     ------> LCD_COM1
        PA10     ------> LCD_COM2
        PA15 (JTDI)     ------> LCD_SEG17
        PC12     ------> LCD_SEG42
        PB4 (NJTRST)     ------> LCD_SEG8
        PB5     ------> LCD_SEG9
        PB9     ------> LCD_COM3
        */
        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5 | GPIO_PIN_6
                               | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_12);

        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8 | GPIO_PIN_9
                               | GPIO_PIN_10 | GPIO_PIN_15);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_12 | GPIO_PIN_13
                               | GPIO_PIN_14 | GPIO_PIN_15 | GPIO_PIN_4 | GPIO_PIN_5
                               | GPIO_PIN_9);

        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9 | GPIO_PIN_10 | GPIO_PIN_11
                               | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15);

        /* USER CODE BEGIN LCD_MspDeInit 1 */

        /* USER CODE END LCD_MspDeInit 1 */
    }

}

/**
* @brief UART MSP Initialization
* This function configures the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspInit(UART_HandleTypeDef *huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if (huart->Instance == USART2) {
        /* USER CODE BEGIN USART2_MspInit 0 */

        /* USER CODE END USART2_MspInit 0 */
        /* Peripheral clock enable */
        __HAL_RCC_USART2_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**USART2 GPIO Configuration
        PD5     ------> USART2_TX
        PD6     ------> USART2_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_5 | GPIO_PIN_6;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN USART2_MspInit 1 */

        /* USER CODE END USART2_MspInit 1 */
    }

}

/**
* @brief UART MSP De-Initialization
* This function freeze the hardware resources used in this example
* @param huart: UART handle pointer
* @retval None
*/
void HAL_UART_MspDeInit(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        /* USER CODE BEGIN USART2_MspDeInit 0 */

        /* USER CODE END USART2_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART2_CLK_DISABLE();

        /**USART2 GPIO Configuration
        PD5     ------> USART2_TX
        PD6     ------> USART2_RX
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_5 | GPIO_PIN_6);

        /* USER CODE BEGIN USART2_MspDeInit 1 */

        /* USER CODE END USART2_MspDeInit 1 */
    }

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
