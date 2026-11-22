/*
 * HF_PA_Control_STM32F103C8T6
 * Контроллер HF-усилителя мощности на STM32F103C8T6.
 *
 * Проект: R9OFG.RU — https://r9ofg.ru
 * Дата: 22.11.2026
 */

/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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

/* прототипы функций */
void ADC_1_start_conversion(void);

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_PTT_Pin GPIO_PIN_13
#define LED_PTT_GPIO_Port GPIOC
#define CONTROL_U_Pin GPIO_PIN_0
#define CONTROL_U_GPIO_Port GPIOA
#define ADC1_KEY_Pin GPIO_PIN_1
#define ADC1_KEY_GPIO_Port GPIOA
#define ADC1_REF_Pin GPIO_PIN_3
#define ADC1_REF_GPIO_Port GPIOA
#define ADC1_FOR_Pin GPIO_PIN_2
#define ADC1_FOR_GPIO_Port GPIOA
#define FAN_Pin GPIO_PIN_4
#define FAN_GPIO_Port GPIOA
#define PTT_GND_Pin GPIO_PIN_6
#define PTT_GND_GPIO_Port GPIOA
#define PTT_GND_EXTI_IRQn EXTI9_5_IRQn
#define A_CODE_Pin GPIO_PIN_0
#define A_CODE_GPIO_Port GPIOB
#define B_CODE_Pin GPIO_PIN_1
#define B_CODE_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_10
#define LCD_CS_GPIO_Port GPIOB
#define LCD_RST_Pin GPIO_PIN_11
#define LCD_RST_GPIO_Port GPIOB
#define LCD_DC_Pin GPIO_PIN_12
#define LCD_DC_GPIO_Port GPIOB
#define LED_DISP_Pin GPIO_PIN_13
#define LED_DISP_GPIO_Port GPIOB
#define C_EXT_Pin GPIO_PIN_14
#define C_EXT_GPIO_Port GPIOB
#define D_EXT_Pin GPIO_PIN_15
#define D_EXT_GPIO_Port GPIOB
#define PE_V1_Pin GPIO_PIN_8
#define PE_V1_GPIO_Port GPIOA
#define PE_V2_Pin GPIO_PIN_9
#define PE_V2_GPIO_Port GPIOA
#define PE_V3_Pin GPIO_PIN_10
#define PE_V3_GPIO_Port GPIOA
#define A_EXT_Pin GPIO_PIN_2
#define A_EXT_GPIO_Port GPIOB
#define B_EXT_Pin GPIO_PIN_14
#define B_EXT_GPIO_Port GPIOC
#define PTT_ON_Pin GPIO_PIN_15
#define PTT_ON_GPIO_Port GPIOA
#define PTT_ON_EXTI_IRQn EXTI15_10_IRQn
#define C_CODE_Pin GPIO_PIN_3
#define C_CODE_GPIO_Port GPIOB
#define D_CODE_Pin GPIO_PIN_4
#define D_CODE_GPIO_Port GPIOB
#define PE_V4_Pin GPIO_PIN_5
#define PE_V4_GPIO_Port GPIOB
#define PE_V5_Pin GPIO_PIN_8
#define PE_V5_GPIO_Port GPIOB
#define PE_V6_Pin GPIO_PIN_9
#define PE_V6_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

#define ADC_CHANNELS_NUM	3

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
