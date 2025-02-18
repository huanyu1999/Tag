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
#include "instance.h"

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
extern uint8_t bee_flag;
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DW1000_RSTn_Pin GPIO_PIN_0
#define DW1000_RSTn_GPIO_Port GPIOA
#define RUN_LED1_Pin GPIO_PIN_1
#define RUN_LED1_GPIO_Port GPIOA
#define RUN_LED2_Pin GPIO_PIN_2
#define RUN_LED2_GPIO_Port GPIOA
#define DW1000_IRQ_Pin GPIO_PIN_0
#define DW1000_IRQ_GPIO_Port GPIOB
#define DW1000_IRQ_EXTI_IRQn EXTI0_IRQn
#define DW1000_WakeUp_Pin GPIO_PIN_1
#define DW1000_WakeUp_GPIO_Port GPIOB
#define BEE_Pin GPIO_PIN_5
#define BEE_GPIO_Port GPIOB
#define BeeCtrl_KEY_Pin GPIO_PIN_6
#define BeeCtrl_KEY_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
