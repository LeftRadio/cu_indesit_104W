/*************************************************************************************
*
Description :  Main Interrupt Service Routines
Version     :  1.0.0
Date        :  7.12.2011
Author      :  Left Radio
Comments:   :  This file provides template for all exceptions handler and
*              peripherals interrupt service routine.
**************************************************************************************/


/* Includes ------------------------------------------------------------------*/
#include "GlobalInit.h"
#include "motor.h"
#include "heater.h"
#include "debug.h"
#include "systick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define __DBUG_INTR_MOTOR

#define __COEFF											((float)0.9)
#define __220_HALF_PERIOD__						((float)(10.0 * __COEFF))

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint16_t tachometr_counter = 0;

/* Exported variables --------------------------------------------------------*/
extern __IO uint8_t RTC_Sec_Event;
extern __IO FlagStatus DMA_USART_Recive_Complete;

/* Private function prototypes -----------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/


/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
	/* Go to infinite loop when Hard Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
	/* Go to infinite loop when Memory Manage exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
	/* Go to infinite loop when Bus Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
	/* Go to infinite loop when Usage Fault exception occurs */
	while (1)
	{
	}
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}


//void RCC_IRQHandler(void)
//{
//  /* Go to infinite loop when Memory Manage exception occurs */
//  while (1)
//  {
//	  TIM_SetCompare2(TIM3, 0);
//	  TIM_SetCompare3(TIM3, 0);
//	  delay_ms(500);
//
//	  TIM_SetCompare2(TIM3, 999);
//	  TIM_SetCompare3(TIM3, 300);
//	  delay_ms(500);
//  }
//}

/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

void RTC_IRQHandler(void)
{
	uint32_t Curent_RTC_Counter;
	ITStatus bitstatus = (ITStatus)(RTC->CRL & RTC_IT_SEC);
//	if (RTC_GetITStatus(RTC_IT_SEC) != RESET)

	if (((RTC->CRH & RTC_IT_SEC) != (uint16_t)RESET) && (bitstatus != (uint16_t)RESET))
	{
		NVIC_ClearPendingIRQ(RTC_IRQn);
		RTC->CRL &= (uint16_t)~RTC_IT_SEC;
		while ((RTC->CRL & RTC_FLAG_RTOFF) == (uint16_t)RESET)
		{
		}
//		RTC_ClearITPendingBit(RTC_IT_SEC);
//		RTC_WaitForLastTask();

		/* If counter is equal to 86399 */
		Curent_RTC_Counter = RTC_GetCounter();
		if(Curent_RTC_Counter == 86399)
		{
			/* Wait until last write operation on RTC registers has finished */
			RTC_WaitForLastTask();
			/* Reset counter value */
			RTC_SetCounter(0x0);
			/* Wait until last write operation on RTC registers has finished */
			RTC_WaitForLastTask();
		}

		RTC_Sec_Event = 1;
	}
}


/**
  * @brief  This function handles TIM2 interrupt request.
  * @param  None
  * @retval None
  */
void TIM1_UP_TIM16_IRQHandler(void)
{
	float RPS = 0;
	TIM1->SR &= ~TIM_SR_UIF;

	RPS = (Timer1_GetFrequency() * ((float)tachometr_counter));
	CU_Motor_Tachometr_Processing(RPS);
	tachometr_counter = 0;
}



/**
  * @brief  This function handles TIM4 interrupt request.
  * @param  None
  * @retval None
  */
void TIM4_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)
	{
		TIM4->SR &= ~TIM_SR_UIF;

#ifdef __MOTOR_REGULATE_RELAY_METHOD

#else
//		if(CU_Motor_GetState()->state != 0)
//		{
		GPIO_MOTOR_OUT_PORT->BSRR = GPIO_MOTOR_OUT_PWM;
		delay_us(100);
		GPIO_MOTOR_OUT_PORT->BRR = GPIO_MOTOR_OUT_PWM;
//		}
#endif

		/* TIM4 disable counter */
		TIM4->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
	}
}



/**
  * @brief  This function handles External lines 9 to 5 interrupt request.
  * 		Zero cross 220 line motor input
  * @param  None
  * @retval None
  */
void EXTI9_5_IRQHandler(void)
{
	static FlagStatus start = SET;

#ifdef __MOTOR_REGULATE_RELAY_METHOD
	static uint8_t period_count = 0;
	static uint8_t skip_periods_num = 0;
	static uint8_t period_div = 0;
#else
	static int speed = 0;
	static float period_delay_ms = 0;
#endif

	uint32_t interrupt_status = EXTI->IMR & GPIO_MOTOR_ZC_A1;

#ifdef __DBUG_INTR_MOTOR
	char dbg_str[10] = "\r\n ";
	static uint8_t dbg_full_period_cnt = 0;
#endif

	if (((EXTI->PR & GPIO_MOTOR_ZC_A1) != (uint32_t)RESET) && (interrupt_status != (uint32_t)RESET))
	{
		EXTI->PR = GPIO_MOTOR_ZC_A1;		/* Clear the EXTI line pending bit */

		if(CU_Motor_GetState()->state == 1)
		{

#ifdef __MOTOR_REGULATE_RELAY_METHOD

			if((start == SET) || (skip_periods_num != CU_Motor_GetState()->speed))
			{
#ifdef __DBUG_INTR_MOTOR
				Debug_Send_String("\r\n intr: start");
				dbg_full_period_cnt = 0;
#endif
				start = RESET;
				skip_periods_num = CU_Motor_GetState()->speed;
				period_count = 0;
				period_div = 0;

				if(skip_periods_num >= 100)
				{
#ifdef __DBUG_INTR_MOTOR
					Debug_Send_String("\r\n intr: skip_periods_num >= 100");
#endif
					CU_Motor_GetState()->state = 0;
					GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_PWM);
					return;
				}
				else
				{
					if(skip_periods_num == 0)
					{
						//						period_div = 0;
						CU_Motor_GetState()->state = 0;
						GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_PWM);
						return;
					}
					else period_div = (uint8_t)(100.0 / (float)skip_periods_num);

#ifdef __DBUG_INTR_MOTOR
					ConvertToString(skip_periods_num, &dbg_str[2], 3);
					Debug_Send_String("\r\n intr: skip_per_num ");
					Debug_Send_String(dbg_str);

					ConvertToString(period_div, &dbg_str[2], 3);
					Debug_Send_String("\r\n intr: period_div ");
					Debug_Send_String(dbg_str);
#endif
				}
			}

			period_count++;
			if(period_count > 100)
			{
				period_count = 0;

#ifdef __DBUG_INTR_MOTOR
				dbg_full_period_cnt++;
				ConvertToString(dbg_full_period_cnt, &dbg_str[2], 3);
				Debug_Send_String("\r\n intr: *** dbg period_cnt ***");
				Debug_Send_String(dbg_str);
#endif
			}

			// full ON
			if(period_div == 0)
			{
				GPIO_SetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_PWM);
			}
			else
			{
#ifdef __DBUG_INTR_MOTOR
				ConvertToString(period_count, &dbg_str[2], 3);
				Debug_Send_String("\r\n intr: period_cnt ");
				Debug_Send_String(dbg_str);
#endif
				if((period_count % period_div) == 0)
				{
#ifdef __DBUG_INTR_MOTOR
					Debug_Send_String("\r\n intr: triac ON");
#endif
					GPIO_SetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_PWM);
				}
				else
				{
#ifdef __DBUG_INTR_MOTOR
					Debug_Send_String("\r\n intr: triac OFF");
#endif
					GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_PWM);
				}
			}

#else	/*__MOTOR_REGULATE_PHASE_METHOD */

			GPIO_MOTOR_OUT_PORT->BRR = GPIO_MOTOR_OUT_PWM;		// RESET

			if((start == SET) || (speed != CU_Motor_GetTachoSpeed()))
			{
#ifdef __DBUG_INTR_MOTOR
				Debug_Send_String("\r\n intr: start");
				dbg_full_period_cnt = 0;
#endif
				start = RESET;
				speed = CU_Motor_GetTachoSpeed();

				if((speed == 0) || (speed >= 1000))
				{
#ifdef __DBUG_INTR_MOTOR
					Debug_Send_String("\r\n intr: speed == 0 || speed >= 1000");
#endif
					CU_Motor_GetState()->state = 0;
					return;
				}
				else
				{
					period_delay_ms = __220_HALF_PERIOD__ - (__220_HALF_PERIOD__ / (1000.0 / (float)speed));
					if(period_delay_ms > __220_HALF_PERIOD__ - 1) period_delay_ms = __220_HALF_PERIOD__ - 1;

#ifdef __DBUG_INTR_MOTOR
					ConvertToString(speed, &dbg_str[2], 4);
					Debug_Send_String("\r\n intr: speed ");
					Debug_Send_String(dbg_str);

					ConvertToString(period_delay_ms, &dbg_str[2], 3);
					Debug_Send_String("\r\n intr: period_delay ");
					Debug_Send_String(dbg_str);
#endif
				}
			}

			if(period_delay_ms != 0)
			{
				/* set autoreload counter, (500 - 1) = 0.5ms, (1000 - 1) = 1ms */
				TIM4->ARR = (uint32_t)(period_delay_ms * 999.0);
				TIM4->CR1 |= TIM_CR1_CEN;
			}
			else
			{
				TIM4->CR1 &= (uint16_t)(~((uint16_t)TIM_CR1_CEN));
				GPIO_MOTOR_OUT_PORT->BSRR = GPIO_MOTOR_OUT_PWM;		// SET
			}
#endif /*__MOTOR_REGULATE_METHOD */

		}
		else
		{
#ifdef __DBUG_INTR_MOTOR
			Debug_Send_String("\r\n intr: motor status OFF, disable intr");
#endif

			NVIC_DisableIRQ(EXTI9_5_IRQn);
			GPIO_MOTOR_OUT_PORT->BRR = GPIO_MOTOR_OUT_PWM;		// RESET

			/* set start flag status for the next motor starts */
			start = SET;
		}
	}


	/* tacho interrupt */
	interrupt_status = EXTI->IMR & GPIO_MOTOR_TACHO_A63;

	if (((EXTI->PR & GPIO_MOTOR_TACHO_A63) != (uint32_t)RESET) && (interrupt_status != (uint32_t)RESET))
	{
		EXTI->PR = GPIO_MOTOR_TACHO_A63;		/* Clear the EXTI line pending bit */
//		if ((GPIOA->IDR & GPIO_Pin_8) != (uint32_t)Bit_RESET) tachometr_counter++;

		 tachometr_counter++;
	}
}




/**
  * @brief  DMA Channel 1, water temp resistor ADC.
  * @param  None
  * @retval None
  */
void DMA1_Channel1_IRQHandler(void)
{
	uint8_t i = 0;
	uint32_t ADC_SumValue = 0;
	static uint32_t EndValue = 0;
	static uint16_t cnt = 0;

	if ((DMA1->ISR & DMA1_IT_TC1) != (uint32_t)RESET)
	{
		for(i = 0; i < 100; i++) ADC_SumValue += Heater_ADCConvertedValue[i];
		ADC_SumValue /= 100;

		if(++cnt >= 10)
		{
			CU_Heater_SetTerhmisorValue((float)EndValue / 6.666F);
			EndValue = cnt = 0;
		}
		else
		{
			EndValue += ADC_SumValue;
		}
	}

	/* Clear DMA1 Channel 1 Half Transfer, Transfer Complete and Global interrupt pending bits */
	DMA1->IFCR = DMA1_IT_GL1;
}


///**
//  * @brief  DMA Channel 4 - USART TX.
//  * @param  None
//  * @retval None
//  */
//void DMA1_Channel4_IRQHandler(void)
//{
//
//	if ((DMA1->ISR & DMA1_IT_TC4) != (uint32_t)RESET)
//	{
//
//	}
//
//	/* Clear DMA1 Channel 4 Half Transfer, Transfer Complete and Global interrupt pending bits */
//	DMA1->IFCR = DMA1_IT_GL4;
//}


/**
  * @brief  DMA Channel 5 - USART RX.
  * @param  None
  * @retval None
  */
void DMA1_Channel5_IRQHandler(void)
{
	if ((DMA1->ISR & DMA1_IT_TC5) != (uint32_t)RESET)
	{
		DMA1->IFCR = DMA1_IT_GL5;
		DMA_USART_Recive_Complete = SET;
	}
}



//
//
///**
//  * @brief  This function handles I2C ERROR interrupt request.
//  * @param  None
//  * @retval None
//  */
//void ADC1_IRQHandler(void)
//{
//   //ADC1->SR &= ~ADC_SR_EOC; // очищаем флаг прерывания
//
////   if(ADC1->DR > 1647)Draw_Batt((uint8_t)((ADC1->DR - 1647) / 3.33), 1);
//
//}







/***********************END OF FILE*******************************/
