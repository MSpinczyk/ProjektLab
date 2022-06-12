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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define S7G_Pin GPIO_PIN_0
#define S7G_GPIO_Port GPIOC
#define S7D_Pin GPIO_PIN_1
#define S7D_GPIO_Port GPIOC
#define S7E_Pin GPIO_PIN_2
#define S7E_GPIO_Port GPIOC
#define S7C_Pin GPIO_PIN_3
#define S7C_GPIO_Port GPIOC
#define PV_Pin GPIO_PIN_1
#define PV_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define SPI_NSS_Pin GPIO_PIN_4
#define SPI_NSS_GPIO_Port GPIOA
#define S7B_Pin GPIO_PIN_4
#define S7B_GPIO_Port GPIOC
#define S7F_Pin GPIO_PIN_5
#define S7F_GPIO_Port GPIOC
#define Button1_Pin GPIO_PIN_0
#define Button1_GPIO_Port GPIOB
#define Button1_EXTI_IRQn EXTI0_IRQn
#define S7A_Pin GPIO_PIN_6
#define S7A_GPIO_Port GPIOC
#define S7DP_Pin GPIO_PIN_7
#define S7DP_GPIO_Port GPIOC
#define S7Com4_Pin GPIO_PIN_8
#define S7Com4_GPIO_Port GPIOC
#define S7Com3_Pin GPIO_PIN_9
#define S7Com3_GPIO_Port GPIOC
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define ClickInterrupt_Pin GPIO_PIN_15
#define ClickInterrupt_GPIO_Port GPIOA
#define ClickInterrupt_EXTI_IRQn EXTI15_10_IRQn
#define S7Com2_Pin GPIO_PIN_10
#define S7Com2_GPIO_Port GPIOC
#define S7Com1_Pin GPIO_PIN_11
#define S7Com1_GPIO_Port GPIOC
#define Button2_Pin GPIO_PIN_5
#define Button2_GPIO_Port GPIOB
#define Button2_EXTI_IRQn EXTI9_5_IRQn
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
