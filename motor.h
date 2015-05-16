/**
  ******************************************************************************
  * @file	 	temp.h
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		header
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _MOTOR__H
#define _MOTOR__H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
//#include "Program.h"
#include "cu_sm_types.h"

/* Exported define -----------------------------------------------------------*/
#define __MOTOR_REGULATE_PHASE_METHOD
//#define __MOTOR_REGULATE_RELAY_METHOD


#define GPIO_MOTOR_TACHO_A63						EXTI_Line8
#define GPIO_MOTOR_ZC_A1							EXTI_Line9

#define GPIO_MOTOR_OUT_PORT							GPIOA
#define GPIO_MOTOR_OUT_REL_A						GPIO_Pin_15					// PA15		A44
#define GPIO_MOTOR_OUT_REL_B						GPIO_Pin_12					// PA12		A46
#define GPIO_MOTOR_OUT_PWM							GPIO_Pin_11					// PA11		A45


/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
typedef struct
{
	uint16_t a;
} CU_Motor_TypeDef;

/* Exported variables --------------------------------------------------------*/
extern __IO float MP_Coeff;
extern __IO float MD_Coeff;

/* Exported function ---------------------------------------------------------*/
void CU_Motor_Configuration(void);

void CU_Motor_SetState(SM_Motor_State NewState, FlagStatus start);
SM_Motor_State* CU_Motor_GetState(void);

void CU_Motor_Tachometr_Processing(float E_RPS);
int CU_Motor_GetTachoSpeed(void);
void CU_Motor_Reset(void);




#endif /* _MOTOR__H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
