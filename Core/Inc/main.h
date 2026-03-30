/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.h
 * @brief          : Header for main.c file.
 *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32h7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */
void setup(void);
void loop(void);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SCH7_Pin GPIO_PIN_2
#define SCH7_GPIO_Port GPIOE
#define SCH3_Pin GPIO_PIN_3
#define SCH3_GPIO_Port GPIOE
#define SCH6_Pin GPIO_PIN_4
#define SCH6_GPIO_Port GPIOE
#define SCH5_Pin GPIO_PIN_5
#define SCH5_GPIO_Port GPIOE
#define SCH4_Pin GPIO_PIN_6
#define SCH4_GPIO_Port GPIOE
#define USRBTN_Pin GPIO_PIN_13
#define USRBTN_GPIO_Port GPIOC
#define SCH1_Pin GPIO_PIN_7
#define SCH1_GPIO_Port GPIOF
#define SCH2_Pin GPIO_PIN_8
#define SCH2_GPIO_Port GPIOF
#define SCH0_Pin GPIO_PIN_9
#define SCH0_GPIO_Port GPIOF
#define MUXA0_Pin GPIO_PIN_0
#define MUXA0_GPIO_Port GPIOG
#define ACT_EN0_Pin GPIO_PIN_1
#define ACT_EN0_GPIO_Port GPIOG
#define SCOPE_Pin GPIO_PIN_12
#define SCOPE_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_14
#define LED_GPIO_Port GPIOB
#define CELL1_DIN_Pin GPIO_PIN_15
#define CELL1_DIN_GPIO_Port GPIOB
#define CELL1_DIN_EXTI_IRQn EXTI15_10_IRQn
#define SCH9_Pin GPIO_PIN_2
#define SCH9_GPIO_Port GPIOG
#define SCH8_Pin GPIO_PIN_3
#define SCH8_GPIO_Port GPIOG
#define SCH15_Pin GPIO_PIN_8
#define SCH15_GPIO_Port GPIOC
#define SCH14_Pin GPIO_PIN_9
#define SCH14_GPIO_Port GPIOC
#define SCOPE2_Pin GPIO_PIN_15
#define SCOPE2_GPIO_Port GPIOA
#define SCH13_Pin GPIO_PIN_10
#define SCH13_GPIO_Port GPIOC
#define SCH12_Pin GPIO_PIN_11
#define SCH12_GPIO_Port GPIOC
#define SCH11_Pin GPIO_PIN_12
#define SCH11_GPIO_Port GPIOC
#define MUXA2_Pin GPIO_PIN_0
#define MUXA2_GPIO_Port GPIOD
#define MUXA1_Pin GPIO_PIN_1
#define MUXA1_GPIO_Port GPIOD
#define SCH10_Pin GPIO_PIN_2
#define SCH10_GPIO_Port GPIOD
#define ACT_EN1_Pin GPIO_PIN_7
#define ACT_EN1_GPIO_Port GPIOD
#define CELL2_DIN_Pin GPIO_PIN_12
#define CELL2_DIN_GPIO_Port GPIOG
#define CELL2_DIN_EXTI_IRQn EXTI15_10_IRQn
#define LED2_Pin GPIO_PIN_1
#define LED2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern UART_HandleTypeDef huart3;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
