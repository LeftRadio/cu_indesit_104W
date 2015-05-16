/**
  ******************************************************************************
  * @file	 	pumps.h
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		header
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HEATER_H
#define __HEATER_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
#include "Program.h"
#include "cu_sm_types.h"

/* Exported define -----------------------------------------------------------*/
#define GPIO_HEATER							GPIO_Pin_8					// PA8		A42
#define GPIO_HEATER_PORT					GPIOA

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern __IO uint16_t Heater_ADCConvertedValue[100];

/* Exported function ---------------------------------------------------------*/
void CU_Heater_Configuration(void);

void CU_Heater_SetState(SM_Heater_State NewState, FlagStatus start);
SM_Heater_State* CU_Heater_GetState(void);

void CU_Heater_SetTerhmisorValue(float TempSensorValue);




#endif /* __HEATER_H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
