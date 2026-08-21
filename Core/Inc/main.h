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
#include "stm32f0xx_hal.h"

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

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define RGB_UART2_TX_Pin GPIO_PIN_2
#define RGB_UART2_TX_GPIO_Port GPIOA
#define RGB_UART2_RX_Pin GPIO_PIN_3
#define RGB_UART2_RX_GPIO_Port GPIOA
#define IND_1_Pin GPIO_PIN_5
#define IND_1_GPIO_Port GPIOA
#define RGB_UART3_TX_Pin GPIO_PIN_10
#define RGB_UART3_TX_GPIO_Port GPIOB
#define IR_IIC2_INT_Pin GPIO_PIN_12
#define IR_IIC2_INT_GPIO_Port GPIOB
#define IR_IIC2_INT_EXTI_IRQn EXTI4_15_IRQn
#define IR_IIC2_SCL_Pin GPIO_PIN_13
#define IR_IIC2_SCL_GPIO_Port GPIOB
#define IR_IIC2_SDA_Pin GPIO_PIN_14
#define IR_IIC2_SDA_GPIO_Port GPIOB
#define IR_IIC1_INT_Pin GPIO_PIN_5
#define IR_IIC1_INT_GPIO_Port GPIOB
#define IR_IIC1_INT_EXTI_IRQn EXTI4_15_IRQn
#define IR_IIC1_SCL_Pin GPIO_PIN_6
#define IR_IIC1_SCL_GPIO_Port GPIOB
#define IR_IIC1_SDA_Pin GPIO_PIN_7
#define IR_IIC1_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
