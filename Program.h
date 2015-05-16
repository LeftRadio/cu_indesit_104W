/**
  ******************************************************************************
  * @file	 	program.h
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		header
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __PROGRAM_H
#define __PROGRAM_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include "cu_sm_types.h"
#include "motor.h"
#include "pumps.h"
#include "heater.h"
#include "selector.h"
#include "programs_data.h"
#include "systick.h"
#include "debug.h"


/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
void CU_Program_Configuration(void);
void  CU_Program_ReadUserTempControl(void);

void CU_Program_NextSM(void);
void CU_Program_EndSM(void);
uint32_t CU_Program_CalcEndTime(uint32_t start_time);

CU_Program* CU_GetProgram(void);
uint8_t* CU_Program_GetActiveStep(void);




#endif /* __PROGRAM_H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
