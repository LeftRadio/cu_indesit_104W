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
#ifndef __PUMPS_H
#define __PUMPS_H

/* Includes ------------------------------------------------------------------*/
#include "cu_sm_types.h"
#include "Program.h"

/* Exported define -----------------------------------------------------------*/
#define GPIO_PUMP_IN_A						GPIO_Pin_2					// PB2		A28
#define GPIO_PUMP_IN_B						GPIO_Pin_10					// PB10		A30
#define GPIO_PUMP_OUT						GPIO_Pin_14					// PA14		A48

#define GPIO_PUMP_WATER_0					GPIO_Pin_7					// PC7		A38
#define GPIO_PUMP_WATER_1					GPIO_Pin_8					// PC8		A39

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
void CU_Pumps_Configuration(void);
void CU_Pumps_SetState(SM_Pumps_State NewState, FlagStatus start);
SM_Pumps_State* CU_Pumps_GetState(void);
void CU_Pumps_EndState(void);
volatile uint16_t Pumps_Read_WaterLevel(void);





#endif /* __PUMPS_H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
