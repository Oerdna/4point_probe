/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32l4xx_hal.h"

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
#define SSB3_Pin GPIO_PIN_0
#define SSB3_GPIO_Port GPIOA
#define PWM_Frozen_Pin GPIO_PIN_1
#define PWM_Frozen_GPIO_Port GPIOA
#define SSB1_Pin GPIO_PIN_3
#define SSB1_GPIO_Port GPIOA
#define DAC_current_Pin GPIO_PIN_4
#define DAC_current_GPIO_Port GPIOA
#define SSB4_Pin GPIO_PIN_0
#define SSB4_GPIO_Port GPIOB
#define TECswitch_Pin GPIO_PIN_1
#define TECswitch_GPIO_Port GPIOB
#define PWM_Heat_Pin GPIO_PIN_8
#define PWM_Heat_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define S1_R_select_Pin GPIO_PIN_3
#define S1_R_select_GPIO_Port GPIOB
#define S0_R_select_Pin GPIO_PIN_4
#define S0_R_select_GPIO_Port GPIOB
#define E_R_select_Pin GPIO_PIN_5
#define E_R_select_GPIO_Port GPIOB
#define m20V_Pin GPIO_PIN_6
#define m20V_GPIO_Port GPIOB
#define m50V_Pin GPIO_PIN_7
#define m50V_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
