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
#include "stm32f3xx_hal.h"

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
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define Analog2_Pin GPIO_PIN_2
#define Analog2_GPIO_Port GPIOC
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define Analog1_Pin GPIO_PIN_4
#define Analog1_GPIO_Port GPIOA
#define HomingA_Pin GPIO_PIN_6
#define HomingA_GPIO_Port GPIOA
#define IN1_B_limit_Pin GPIO_PIN_10
#define IN1_B_limit_GPIO_Port GPIOB
#define IN1_A_limit_Pin GPIO_PIN_11
#define IN1_A_limit_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_13
#define LD2_GPIO_Port GPIOB
#define PWM1_Pin GPIO_PIN_14
#define PWM1_GPIO_Port GPIOB
#define PWM2_Pin GPIO_PIN_15
#define PWM2_GPIO_Port GPIOB
#define IN2_A_limit_Pin GPIO_PIN_10
#define IN2_A_limit_GPIO_Port GPIOA
#define IN2_B_limit_Pin GPIO_PIN_11
#define IN2_B_limit_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define IN1_A_Pin GPIO_PIN_15
#define IN1_A_GPIO_Port GPIOA
#define IN1_B_Pin GPIO_PIN_10
#define IN1_B_GPIO_Port GPIOC
#define IN2_A_Pin GPIO_PIN_11
#define IN2_A_GPIO_Port GPIOC
#define IN2_B_Pin GPIO_PIN_12
#define IN2_B_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define AordD_Pin GPIO_PIN_7
#define AordD_GPIO_Port GPIOB
#define HomingB_Pin GPIO_PIN_9
#define HomingB_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
