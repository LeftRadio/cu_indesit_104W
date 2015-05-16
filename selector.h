/**
  ******************************************************************************
  * @file	 	selector.h
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		header
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SELECTOR_H
#define __SELECTOR_H

/* Includes ------------------------------------------------------------------*/
//#include "stm32f10x.h"
#include "cu_sm_types.h"

/* Exported define -----------------------------------------------------------*/
/* program selector */
#define GPIO_PROGRAM_SELECTOR_OUT			GPIO_Pin_11					// PB15		A31
#define SELECTOR_PRG_COUNT					29

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
extern const uint8_t selectorProgramCodes[SELECTOR_PRG_COUNT];

/* Exported function ---------------------------------------------------------*/
void CU_Selector_Configuration(void);
uint8_t CU_Selector_Read(void);
void CU_Selector_Manual_ON(void);
void CU_Selector_Manual_OFF(void);




#endif /* __SELECTOR_H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
