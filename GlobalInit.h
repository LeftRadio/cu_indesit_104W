/**
  ******************************************************************************
  * @file	 	GlobalInit.h
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		header
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _GLOBAL_INIT__H
#define _GLOBAL_INIT__H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported define -----------------------------------------------------------*/

#define GPIO_EN_HV_LINE_PIN					    GPIO_Pin_5					// PC5		A25
#define GPIO_ON_OFF_LAMP					    GPIO_Pin_10					// PC10		A43
#define GPIO_TEMP_CONTROL					    GPIO_Pin_1					// PA1		A8

#define GPIO_FAST_MODE_BUTTON_PIN			    GPIO_Pin_4
#define GPIO_ECONOM_MODE_BUTTON_PIN			    GPIO_Pin_5
#define GPIO_LOW_MOTOR_SPEED_BUTTON_PIN			GPIO_Pin_6
#define GPIO_BUTTONS_PORT                       GPIOA

#define TIM4_INTR_PRIORITY					    1
#define EXTI_INTR_PRIORITY					    2
#define TIM1_INTR_PRIORITY					    10
#define DMA_USART_INTR_PRIORITY				    13
#define RTC_INTR_PRIORITY					    14
#define DMA_ADC_INTR_PRIORITY				    15

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern uint8_t USART_RxBuffer[8];

/* Exported function ---------------------------------------------------------*/
void GlobalInit(void);
void USART_DMA_Chennel4_Configuration(uint32_t len, uint8_t init);
void Timer1_ReConfig(float cnt_div);
float Timer1_GetFrequency(void);
void Jump_To_Bootloader(void);




#endif /* _GLOBAL_INIT__H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
