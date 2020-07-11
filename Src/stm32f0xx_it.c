/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f0xx_it.h"
#include "timer.h"
#include "cmdlink.h"
#include "serialbufproc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
extern USART_TypeDef*  s_usarts[UART_CH_MAX]; 
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim1;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M0 Processor Interruption and Exception Handlers          */ 
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */

  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */

  /* USER CODE END SVC_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	NVIC_ClearPendingIRQ(SysTick_IRQn);
	timer_counting();
  readtimer_update();
  TT_timing_1ms();
	HAL_SYSTICK_IRQHandler();
	HAL_IncTick();
  /* USER CODE END SysTick_IRQn 0 */
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM1 break, update, trigger and commutation interrupts.
  */
void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 0 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_BRK_UP_TRG_COM_IRQn 1 */

  /* USER CODE END TIM1_BRK_UP_TRG_COM_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt / USART1 wake-up interrupt through EXTI line 25.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */

  /* USER CODE END USART1_IRQn 0 */
  /* USER CODE BEGIN USART1_IRQn 1 */
  uint8_t ch;
  e_channelTypedef* dev_chs;
  ch = Get_TTMode();
  
  if(LL_USART_IsActiveFlag_TC(USART1))
  {
    LL_USART_ClearFlag_TC(USART1);
    if(ch == 0)
    {
      hal_transfer_byte_IT(0);
    }
  }
  
  if(LL_USART_IsActiveFlag_RXNE(USART1))
  {
    if(ch != 0)
    {
      dev_chs = Get_DeviceCHs();
      s_usarts[dev_chs[ch-1]]->TDR = USART1->RDR;
      Fresh_TT_timer();
    }
    else
    {
        hal_receive_byte_IT(0);
    }
  }

    NVIC_ClearPendingIRQ(USART1_IRQn);
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  /* USER CODE BEGIN USART2_IRQn 1 */
  uint8_t ch;
  e_channelTypedef* dev_chs;
  ch = Get_TTMode();
  if(LL_USART_IsActiveFlag_TC(USART2))
  {
    LL_USART_ClearFlag_TC(USART2);
    if(ch == 0)
    {
      hal_transfer_byte_IT(1);
    }
  }

  if(LL_USART_IsActiveFlag_RXNE(USART2))
  {
    if(ch != 0)
    {
      dev_chs = Get_DeviceCHs();
      // if USART order number to TT channel eques to 1(USART2)  
      if(1 == dev_chs[ch-1])
      {
        USART1->TDR = USART2->RDR;
      }
      Fresh_TT_timer();
    }
    else
    {
      hal_receive_byte_IT(1);
    }
  }
  NVIC_ClearPendingIRQ(USART2_IRQn);
  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 to USART8 global interrupts / USART3 wake-up interrupt through EXTI line 28.
  */
void USART3_8_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_8_IRQn 0 */

	uint8_t ch;
	e_channelTypedef* dev_chs;
	ch = Get_TTMode();
  /* USER CODE END USART3_8_IRQn 0 */
  
  /* USER CODE BEGIN USART3_8_IRQn 1 */
    
	if(LL_USART_IsActiveFlag_RXNE(USART3))
	{
		if(ch != 0)
		{
			dev_chs = Get_DeviceCHs();
			// if USART order number to TT channel eques to 2(USART3)  
			if(2 == dev_chs[ch-1])
			{
				USART1->TDR = USART3->RDR;
			}
			Fresh_TT_timer();
		}
		else
			hal_receive_byte_IT(2);
	}
	if(LL_USART_IsActiveFlag_TC(USART3))
	{
		LL_USART_ClearFlag_TC(USART3);
		if(ch == 0)
			hal_transfer_byte_IT(2);
	}
	
	if(LL_USART_IsActiveFlag_RXNE(USART4))
	{
		if(ch != 0)
		{
			dev_chs = Get_DeviceCHs();
			// if USART order number to TT channel eques to 3(USART4)  
			if(3 == dev_chs[ch-1])
			{
				USART1->TDR = USART4->RDR;
			}
			Fresh_TT_timer();
		}
		else
			hal_receive_byte_IT(3);
	}
	
	if(LL_USART_IsActiveFlag_TC(USART4))
	{
			LL_USART_ClearFlag_TC(USART4);
		if(ch == 0)
			hal_transfer_byte_IT(3);
	}
	
	if(LL_USART_IsActiveFlag_RXNE(USART5))
	{
			hal_receive_byte_IT(4);
	}
	
	if(LL_USART_IsActiveFlag_TC(USART5))
	{
			LL_USART_ClearFlag_TC(USART5);
			hal_transfer_byte_IT(4);
	}
	
	if(LL_USART_IsActiveFlag_RXNE(USART6))
	{
			hal_receive_byte_IT(5);
	}
	
	if(LL_USART_IsActiveFlag_TC(USART6))
	{
			LL_USART_ClearFlag_TC(USART6);
			hal_transfer_byte_IT(5);
	}
	
	if(LL_USART_IsActiveFlag_RXNE(USART7))
	{
		if(ch != 0)
		{
			dev_chs = Get_DeviceCHs();
			// if USART order number to TT channel eques to 6(USART7)  
			if(6 == dev_chs[ch-1])
			{
				USART1->TDR = USART7->RDR;
			}
			Fresh_TT_timer();
		}
		else
			hal_receive_byte_IT(6);
	}
	
	if(LL_USART_IsActiveFlag_TC(USART7))
	{
			LL_USART_ClearFlag_TC(USART7);
		if(ch == 0)
			hal_transfer_byte_IT(6);
	}
	
	if(LL_USART_IsActiveFlag_RXNE(USART8))
	{
		if(ch != 0)
		{
			dev_chs = Get_DeviceCHs();
			// if USART order number to TT channel eques to 7(USART8)  
			if(7 == dev_chs[ch-1])
			{
				USART1->TDR = USART8->RDR;
			}
			Fresh_TT_timer();
		}
		else
			hal_receive_byte_IT(7);
	}
	
	if(LL_USART_IsActiveFlag_TC(USART8))
	{
			LL_USART_ClearFlag_TC(USART8);
		if(ch == 0)
			hal_transfer_byte_IT(7);
	}
    
    
		
		NVIC_ClearPendingIRQ(USART3_8_IRQn);
  /* USER CODE END USART3_8_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
