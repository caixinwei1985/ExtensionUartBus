/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_crs.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_exti.h"
#include "stm32f0xx_ll_cortex.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_pwr.h"
#include "stm32f0xx_ll_dma.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_hal_flash.h"

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
#define SI_Pin LL_GPIO_PIN_14
#define SI_GPIO_Port GPIOC
#define SO_Pin LL_GPIO_PIN_15
#define SO_GPIO_Port GPIOC
#define RXD6_DET_Pin LL_GPIO_PIN_2
#define RXD6_DET_GPIO_Port GPIOC
#define RXD2_DET_Pin LL_GPIO_PIN_4
#define RXD2_DET_GPIO_Port GPIOA
#define RXD4_DET_Pin LL_GPIO_PIN_5
#define RXD4_DET_GPIO_Port GPIOA
#define RXD3_DET_Pin LL_GPIO_PIN_0
#define RXD3_DET_GPIO_Port GPIOB
#define RXD7_DET_Pin LL_GPIO_PIN_15
#define RXD7_DET_GPIO_Port GPIOB
#define RXD8_DET_Pin LL_GPIO_PIN_8
#define RXD8_DET_GPIO_Port GPIOA
#define RXD1_DET_Pin LL_GPIO_PIN_11
#define RXD1_DET_GPIO_Port GPIOA
#define MI_Pin LL_GPIO_PIN_10
#define MI_GPIO_Port GPIOC
#define MO_Pin LL_GPIO_PIN_11
#define MO_GPIO_Port GPIOC
#define RXD5_DET_Pin LL_GPIO_PIN_3
#define RXD5_DET_GPIO_Port GPIOB
#define A0_Pin LL_GPIO_PIN_4
#define A0_GPIO_Port GPIOB
#define A1_Pin LL_GPIO_PIN_5
#define A1_GPIO_Port GPIOB
#define A2_Pin LL_GPIO_PIN_6
#define A2_GPIO_Port GPIOB
#define A3_Pin LL_GPIO_PIN_7
#define A3_GPIO_Port GPIOB
#define DL_SEL_Pin	LL_GPIO_PIN_14
#define DL_SEL_GPIO_Port GPIOB
#define LED1_GPIO_Port GPIOF
#define LED1_Pin LL_GPIO_PIN_0
#define LED2_GPIO_Port GPIOF
#define LED2_Pin LL_GPIO_PIN_1
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
