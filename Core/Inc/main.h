/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f4xx_hal.h"

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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define K1_Pin GPIO_PIN_3
#define K1_GPIO_Port GPIOE
#define K0_Pin GPIO_PIN_4
#define K0_GPIO_Port GPIOE
#define SPI2_MISO_ENC_Pin GPIO_PIN_2
#define SPI2_MISO_ENC_GPIO_Port GPIOC
#define SPI2_MOSI_ENC_Pin GPIO_PIN_3
#define SPI2_MOSI_ENC_GPIO_Port GPIOC
#define K_UP_Pin GPIO_PIN_0
#define K_UP_GPIO_Port GPIOA
#define ADC3_IN3_W_Pin GPIO_PIN_3
#define ADC3_IN3_W_GPIO_Port GPIOA
#define ADC2_IN4_V_Pin GPIO_PIN_4
#define ADC2_IN4_V_GPIO_Port GPIOA
#define ADC1_IN5_U_Pin GPIO_PIN_5
#define ADC1_IN5_U_GPIO_Port GPIOA
#define led0_Pin GPIO_PIN_6
#define led0_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define SPI2_SS_ENC_Pin GPIO_PIN_15
#define SPI2_SS_ENC_GPIO_Port GPIOE
#define SPI2_SCK_ENC_Pin GPIO_PIN_10
#define SPI2_SCK_ENC_GPIO_Port GPIOB
#define TIM4_CH1_U_Pin GPIO_PIN_12
#define TIM4_CH1_U_GPIO_Port GPIOD
#define TIM4_CH2_V_Pin GPIO_PIN_13
#define TIM4_CH2_V_GPIO_Port GPIOD
#define TIM4_CH3_W_Pin GPIO_PIN_14
#define TIM4_CH3_W_GPIO_Port GPIOD
#define TIM3_CH1_ENC_A_Pin GPIO_PIN_6
#define TIM3_CH1_ENC_A_GPIO_Port GPIOC
#define TIM3_CH2_ENC_B_Pin GPIO_PIN_7
#define TIM3_CH2_ENC_B_GPIO_Port GPIOC
#define ESC_SPI_IRQ_Pin GPIO_PIN_0
#define ESC_SPI_IRQ_GPIO_Port GPIOD
#define ESC_SPI_IRQ_EXTI_IRQn EXTI0_IRQn
#define ESC_SYNC0_Pin GPIO_PIN_3
#define ESC_SYNC0_GPIO_Port GPIOD
#define ESC_SYNC0_EXTI_IRQn EXTI3_IRQn
#define ESC_SYNC1_Pin GPIO_PIN_4
#define ESC_SYNC1_GPIO_Port GPIOD
#define ESC_SYNC1_EXTI_IRQn EXTI4_IRQn
#define ESC_EEP_LOAD_Pin GPIO_PIN_5
#define ESC_EEP_LOAD_GPIO_Port GPIOD
#define SPI1_SCK_ESC_Pin GPIO_PIN_3
#define SPI1_SCK_ESC_GPIO_Port GPIOB
#define SPI1_MISO_ESC_Pin GPIO_PIN_4
#define SPI1_MISO_ESC_GPIO_Port GPIOB
#define SPI1_MOSI_ESC_Pin GPIO_PIN_5
#define SPI1_MOSI_ESC_GPIO_Port GPIOB
#define SPI1_SS_ESC_Pin GPIO_PIN_6
#define SPI1_SS_ESC_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
