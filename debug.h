/**
  ******************************************************************************
  * @file	 	debug.h
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		header
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _DEBUG__H
#define _DEBUG__H

/* Includes ------------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
//#define __MOTOR_REGULATE_RELAY_METHOD

#ifndef __MOTOR_REGULATE_RELAY_METHOD
	#define __MOTOR_REGULATE_PHASE_METHOD
#endif

/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/
//void USARTSend(unsigned char *pucBuffer, unsigned long ulCount);
void Debug_Send_SystemStatus(void);
void Debug_Send_Time(void);
void Debug_Send_String(char *string);

void ConvertToString(uint32_t Num, char* Str, uint8_t NumSymbol);

#endif /* _DEBUG__H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
