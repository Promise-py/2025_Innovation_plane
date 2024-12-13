/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#define CS1_ACCEL_Pin CS1_Pin
#define CS1_ACCEL_GPIO_Port GPIOA
// #define INT1_ACCEL_Pin GPIO_PIN_4
// #define INT1_ACCEL_GPIO_Port GPIOC
// #define INT1_ACCEL_EXTI_IRQn EXTI4_IRQn
// #define INT1_GRYO_Pin GPIO_PIN_5
// #define INT1_GRYO_GPIO_Port GPIOC
// #define INT1_GRYO_EXTI_IRQn EXTI9_5_IRQn
#define CS1_GYRO_Pin CS2_Pin
#define CS1_GYRO_GPIO_Port GPIOA

#define DW_RESET_EXTI_IRQn EXTI1_IRQn
// #define LED_Pin GPIO_PIN_13
// #define LED_GPIO_Port GPIOC
#define DW_RESET_Pin GPIO_PIN_0
#define DW_RESET_GPIO_Port GPIOA
#define DW_WUP_Pin GPIO_PIN_1
#define DW_WUP_GPIO_Port GPIOA
#define DW_NSS_Pin GPIO_PIN_2
#define DW_NSS_GPIO_Port GPIOA
#define DW_IRQn_Pin GPIO_PIN_4
#define DW_IRQn_GPIO_Port GPIOA
#define DW_SCK_Pin GPIO_PIN_5
#define DW_SCK_GPIO_Port GPIOA
#define DW_MISO_Pin GPIO_PIN_6
#define DW_MISO_GPIO_Port GPIOA
#define DW_MOSI_Pin GPIO_PIN_7
#define DW_MOSI_GPIO_Port GPIOA
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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DW_RESET_Pin GPIO_PIN_0
#define DW_RESET_GPIO_Port GPIOA
#define DW_WUP_Pin GPIO_PIN_1
#define DW_WUP_GPIO_Port GPIOA
#define DW_NSS_Pin GPIO_PIN_2
#define DW_NSS_GPIO_Port GPIOA
#define DW_IRQn_Pin GPIO_PIN_4
#define DW_IRQn_GPIO_Port GPIOA
#define CS1_Pin GPIO_PIN_11
#define CS1_GPIO_Port GPIOA
#define CS2_Pin GPIO_PIN_12
#define CS2_GPIO_Port GPIOA
#define INT1_Pin GPIO_PIN_15
#define INT1_GPIO_Port GPIOA
#define INT1_EXTI_IRQn EXTI15_10_IRQn
#define INT2_Pin GPIO_PIN_3
#define INT2_GPIO_Port GPIOB
#define INT2_EXTI_IRQn EXTI3_IRQn
#define Trig_Pin GPIO_PIN_6
#define Trig_GPIO_Port GPIOB
#define Echo_Pin GPIO_PIN_7
#define Echo_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
