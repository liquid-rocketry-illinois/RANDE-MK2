/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file         stm32h7xx_hal_msp.c
 * @brief        This file provides code for the MSP Initialization
 *               and de-Initialization codes.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
extern DMA_HandleTypeDef hdma_adc1;

extern DMA_HandleTypeDef hdma_adc2;

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

    /* System interrupt init*/

    /* USER CODE BEGIN MspInit 1 */

    /* USER CODE END MspInit 1 */
}

static uint32_t HAL_RCC_ADC12_CLK_ENABLED = 0;

/**
 * @brief ADC MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    if(hadc->Instance == ADC1) {
        /* USER CODE BEGIN ADC1_MspInit 0 */

        /* USER CODE END ADC1_MspInit 0 */
        /* Peripheral clock enable */
        HAL_RCC_ADC12_CLK_ENABLED++;
        if(HAL_RCC_ADC12_CLK_ENABLED == 1) {
            __HAL_RCC_ADC12_CLK_ENABLE();
        }

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        /**ADC1 GPIO Configuration
        PA6     ------> ADC1_INP3
        PA7     ------> ADC1_INN3
        PC4     ------> ADC1_INP4
        PC5     ------> ADC1_INN4
        PB0     ------> ADC1_INN5
        PB1     ------> ADC1_INP5
        PF11     ------> ADC1_INP2
        PF12     ------> ADC1_INN2
        */
        GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_1;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* ADC1 DMA Init */
        /* ADC1 Init */
        hdma_adc1.Instance = DMA1_Stream0;
        hdma_adc1.Init.Request = DMA_REQUEST_ADC1;
        hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_adc1.Init.Mode = DMA_CIRCULAR;
        hdma_adc1.Init.Priority = DMA_PRIORITY_LOW;
        hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(&hdma_adc1) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc1);

        /* USER CODE BEGIN ADC1_MspInit 1 */

        /* USER CODE END ADC1_MspInit 1 */
    }
    else if(hadc->Instance == ADC2) {
        /* USER CODE BEGIN ADC2_MspInit 0 */

        /* USER CODE END ADC2_MspInit 0 */
        /* Peripheral clock enable */
        HAL_RCC_ADC12_CLK_ENABLED++;
        if(HAL_RCC_ADC12_CLK_ENABLED == 1) {
            __HAL_RCC_ADC12_CLK_ENABLE();
        }

        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_GPIOF_CLK_ENABLE();
        /**ADC2 GPIO Configuration
        PA2     ------> ADC2_INP14
        PA3     ------> ADC2_INP15
        PA4     ------> ADC2_INP18
        PA5     ------> ADC2_INP19
        PF13     ------> ADC2_INP2
        PF14     ------> ADC2_INP6
        */
        GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = GPIO_PIN_13 | GPIO_PIN_14;
        GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

        /* ADC2 DMA Init */
        /* ADC2 Init */
        hdma_adc2.Instance = DMA1_Stream1;
        hdma_adc2.Init.Request = DMA_REQUEST_ADC2;
        hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
        hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
        hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
        hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
        hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
        hdma_adc2.Init.Mode = DMA_CIRCULAR;
        hdma_adc2.Init.Priority = DMA_PRIORITY_LOW;
        hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(&hdma_adc2) != HAL_OK) {
            Error_Handler();
        }

        __HAL_LINKDMA(hadc, DMA_Handle, hdma_adc2);

        /* USER CODE BEGIN ADC2_MspInit 1 */

        /* USER CODE END ADC2_MspInit 1 */
    }
}

/**
 * @brief ADC MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hadc: ADC handle pointer
 * @retval None
 */
void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {
    if(hadc->Instance == ADC1) {
        /* USER CODE BEGIN ADC1_MspDeInit 0 */

        /* USER CODE END ADC1_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_ADC12_CLK_ENABLED--;
        if(HAL_RCC_ADC12_CLK_ENABLED == 0) {
            __HAL_RCC_ADC12_CLK_DISABLE();
        }

        /**ADC1 GPIO Configuration
        PA6     ------> ADC1_INP3
        PA7     ------> ADC1_INN3
        PC4     ------> ADC1_INP4
        PC5     ------> ADC1_INN4
        PB0     ------> ADC1_INN5
        PB1     ------> ADC1_INP5
        PF11     ------> ADC1_INP2
        PF12     ------> ADC1_INN2
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_6 | GPIO_PIN_7);

        HAL_GPIO_DeInit(GPIOC, GPIO_PIN_4 | GPIO_PIN_5);

        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_0 | GPIO_PIN_1);

        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_11 | GPIO_PIN_12);

        /* ADC1 DMA DeInit */
        HAL_DMA_DeInit(hadc->DMA_Handle);
        /* USER CODE BEGIN ADC1_MspDeInit 1 */

        /* USER CODE END ADC1_MspDeInit 1 */
    }
    else if(hadc->Instance == ADC2) {
        /* USER CODE BEGIN ADC2_MspDeInit 0 */

        /* USER CODE END ADC2_MspDeInit 0 */
        /* Peripheral clock disable */
        HAL_RCC_ADC12_CLK_ENABLED--;
        if(HAL_RCC_ADC12_CLK_ENABLED == 0) {
            __HAL_RCC_ADC12_CLK_DISABLE();
        }

        /**ADC2 GPIO Configuration
        PA2     ------> ADC2_INP14
        PA3     ------> ADC2_INP15
        PA4     ------> ADC2_INP18
        PA5     ------> ADC2_INP19
        PF13     ------> ADC2_INP2
        PF14     ------> ADC2_INP6
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | GPIO_PIN_5);

        HAL_GPIO_DeInit(GPIOF, GPIO_PIN_13 | GPIO_PIN_14);

        /* ADC2 DMA DeInit */
        HAL_DMA_DeInit(hadc->DMA_Handle);
        /* USER CODE BEGIN ADC2_MspDeInit 1 */

        /* USER CODE END ADC2_MspDeInit 1 */
    }
}

/**
 * @brief UART MSP Initialization
 * This function configures the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if(huart->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspInit 0 */

        /* USER CODE END USART3_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInitStruct.Usart234578ClockSelection = RCC_USART234578CLKSOURCE_D2PCLK1;
        if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /* Peripheral clock enable */
        __HAL_RCC_USART3_CLK_ENABLE();

        __HAL_RCC_GPIOD_CLK_ENABLE();
        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
        HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

        /* USER CODE BEGIN USART3_MspInit 1 */

        /* USER CODE END USART3_MspInit 1 */
    }
}

/**
 * @brief UART MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param huart: UART handle pointer
 * @retval None
 */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
    if(huart->Instance == USART3) {
        /* USER CODE BEGIN USART3_MspDeInit 0 */

        /* USER CODE END USART3_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USART3_CLK_DISABLE();

        /**USART3 GPIO Configuration
        PD8     ------> USART3_TX
        PD9     ------> USART3_RX
        */
        HAL_GPIO_DeInit(GPIOD, GPIO_PIN_8 | GPIO_PIN_9);

        /* USER CODE BEGIN USART3_MspDeInit 1 */

        /* USER CODE END USART3_MspDeInit 1 */
    }
}

/**
 * @brief HCD MSP Initialization
 * This function configures the hardware resources used in this example
 * @param hhcd: HCD handle pointer
 * @retval None
 */
void HAL_HCD_MspInit(HCD_HandleTypeDef* hhcd) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
    if(hhcd->Instance == USB_OTG_FS) {
        /* USER CODE BEGIN USB_OTG_FS_MspInit 0 */

        /* USER CODE END USB_OTG_FS_MspInit 0 */

        /** Initializes the peripherals clock
         */
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USB;
        PeriphClkInitStruct.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
        if(HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
            Error_Handler();
        }

        /** Enable USB Voltage detector
         */
        HAL_PWREx_EnableUSBVoltageDetector();

        __HAL_RCC_GPIOA_CLK_ENABLE();
        /**USB_OTG_FS GPIO Configuration
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
        */
        GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* Peripheral clock enable */
        __HAL_RCC_USB_OTG_FS_CLK_ENABLE();
        /* USB_OTG_FS interrupt Init */
        HAL_NVIC_SetPriority(OTG_FS_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);
        /* USER CODE BEGIN USB_OTG_FS_MspInit 1 */

        /* USER CODE END USB_OTG_FS_MspInit 1 */
    }
}

/**
 * @brief HCD MSP De-Initialization
 * This function freeze the hardware resources used in this example
 * @param hhcd: HCD handle pointer
 * @retval None
 */
void HAL_HCD_MspDeInit(HCD_HandleTypeDef* hhcd) {
    if(hhcd->Instance == USB_OTG_FS) {
        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 0 */

        /* USER CODE END USB_OTG_FS_MspDeInit 0 */
        /* Peripheral clock disable */
        __HAL_RCC_USB_OTG_FS_CLK_DISABLE();

        /**USB_OTG_FS GPIO Configuration
        PA11     ------> USB_OTG_FS_DM
        PA12     ------> USB_OTG_FS_DP
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11 | GPIO_PIN_12);

        /* USB_OTG_FS interrupt DeInit */
        HAL_NVIC_DisableIRQ(OTG_FS_IRQn);
        /* USER CODE BEGIN USB_OTG_FS_MspDeInit 1 */

        /* USER CODE END USB_OTG_FS_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
