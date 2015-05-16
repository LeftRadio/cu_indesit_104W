/**
  ******************************************************************************
  * @file	 	debug.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "GlobalInit.h"
#include "Program.h"
#include "motor.h"
#include "debug.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  : USARTSend
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void USARTSend(unsigned char *pucBuffer, unsigned long ulCount)
{
//    memcpy(USART_TxBuffer, pucBuffer, ulCount);
//    USART_DMA_Chennel4_Configuration(ulCount, 0);


	while(ulCount > 0)
    {
        USART_SendData(USART1, *pucBuffer);
        /* Loop until the end of transmission */
        while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET)
        {
        }

        pucBuffer++;
        ulCount--;
    }
}


void Debug_Send_Time(void)
{
	unsigned int rtc_counter = 0;
	unsigned char hours = 0, minutes = 0, seconds = 0;
	char time_string[25] = "    00:00:00    ";

	rtc_counter = RTC_GetCounter();

	hours = rtc_counter / 3600;
	minutes = (rtc_counter % 3600) / 60;
	seconds = (rtc_counter % 3600) % 60;

	ConvertToString(hours, &time_string[4], 2);
	ConvertToString(minutes, &time_string[7], 2);
	ConvertToString(seconds, &time_string[10], 2);
	time_string[12] = 0;

	USARTSend((uint8_t*)time_string, strlen(time_string));
}

/*******************************************************************************
* Function Name  : Debug_SystemStatus
* Description    :
* Input          :
* Return         :
*******************************************************************************/
void Debug_Send_String(char *string)
{
//	unsigned int rtc_counter = 0;
//	unsigned char hours = 0, minutes = 0, seconds = 0;
//	char time_string[13] = "    00:00:00";

//	rtc_counter = RTC_GetCounter();

//	hours = rtc_counter / 3600;
//	minutes = (rtc_counter % 3600) / 60;
//	seconds = (rtc_counter % 3600) % 60;
//
//	ConvertToString(hours, &time_string[4], 2);
//	ConvertToString(minutes, &time_string[7], 2);
//	ConvertToString(seconds, &time_string[10], 2);
//	time_string[12] = 0;

	USARTSend((uint8_t*)string, strlen(string));
//	USARTSend((uint8_t*)time_string, strlen(time_string));
}


/*******************************************************************************
* Function Name  : Debug_SystemStatus
* Description    :
* Input          :
* Return         :
*******************************************************************************/
void Debug_Send_SystemStatus(void)
{
	uint8_t i;

	static unsigned int rtc_counter = 0;	// old_rtc_counter = 0;
	static unsigned char hours = 0, minutes = 0, seconds = 0;
	SM_Motor_State *motor_state;

	char time_string[9] = "00:00:00";

	char prg_string[6][6] = {
			{"FM   "}, {"EM   "}, {"T   "},
			{"WL   "}, {"WE   "}, {"WS   "}
	};


	char sm_[2][6] = {
			{"WT   "}, {"RP   "}
	};
	char sm_heater_string[3][6] = {
			{"ST   "}, {"IT   "}, {"RT   "}
	};
	char sm_pump_string[2][6] = {
			{"PI   "}, {"PO   "}
	};
	char sm_motor_string[3][7] = {
			{"ST   "}, {"DR   "}, {"SP   "}
	};

	char ports_string[4][12] = {
			{"A   "}, {"B   "}, {"C   "}, {"D   "}
	};

	CU_Program *Program = CU_GetProgram();

	USARTSend((uint8_t*)"\r\n\r\n", 4);
	USARTSend((uint8_t*)"--------------------------------------------------\r\n", 52);
	USARTSend((uint8_t*)"Loaded program\r\n", 16);

	ConvertToString(Program->fast_mode, &prg_string[0][3], 1);
	ConvertToString(Program->economy_mode, &prg_string[1][3], 1);
	ConvertToString(Program->temp, &prg_string[2][3], 2);
	ConvertToString(Program->water_level, &prg_string[3][3], 2);
	ConvertToString(Program->water_extraction, &prg_string[4][3], 1);
	ConvertToString(Program->water_extraction_speed, &prg_string[5][3], 2);

	for(i = 0; i < 6; i++) USARTSend((uint8_t*)&prg_string[i][0], 6);


	/* Compute  hours, minutes, seconds */
	rtc_counter = RTC_GetCounter();
	RTC_WaitForLastTask();
	hours = rtc_counter / 3600;
	minutes = (rtc_counter % 3600) / 60;
	seconds = (rtc_counter % 3600) % 60;

	ConvertToString(hours, &time_string[0], 2);
	ConvertToString(minutes, &time_string[3], 2);
	ConvertToString(seconds, &time_string[6], 2);
	time_string[8] = 0;

	USARTSend((uint8_t*)"\r\n\r\n", 4);
	USARTSend((uint8_t*)"TIME", 4);
	USARTSend((uint8_t*)"\r\n", 2);
	USARTSend((uint8_t*)time_string, 8);


	// SM
	USARTSend((uint8_t*)"\r\n\r\n", 4);
	USARTSend((uint8_t*)"SM:", 3);
	USARTSend((uint8_t*)"\r\n", 2);
	ConvertToString(Program->sm_states->work_time, &sm_[0][3], 3);
	ConvertToString(Program->sm_states->repeat, &sm_[1][3], 3);
	for(i = 0; i < 2; i++)
	{
		USARTSend((uint8_t*)&sm_[i][0], 6);
		USARTSend((uint8_t*)"\r\n", 2);
	}

	// heater
	USARTSend((uint8_t*)"\r\n\r\n", 4);
	USARTSend((uint8_t*)"HEATER:", 7);
	USARTSend((uint8_t*)"\r\n", 2);
	ConvertToString(Program->sm_states->sm_heater_state.state, &sm_heater_string[0][3], 2);
	ConvertToString(Program->sm_states->sm_heater_state.intent_water_temp, &sm_heater_string[1][3], 2);
	ConvertToString(Program->sm_states->sm_heater_state.termistor_water_temp, &sm_heater_string[2][3], 2);
	for(i = 0; i < 3; i++)
	{
		USARTSend((uint8_t*)&sm_heater_string[i][0], 6);
		USARTSend((uint8_t*)"\r\n", 2);
	}

	// pumps
	USARTSend((uint8_t*)"\r\n\r\n", 4);
	USARTSend((uint8_t*)"PUMPS:", 6);
	USARTSend((uint8_t*)"\r\n", 2);
	ConvertToString(Program->sm_states->sm_pumps_state.pump_in_state, &sm_pump_string[0][3], 2);
	ConvertToString(Program->sm_states->sm_pumps_state.pump_out_state, &sm_pump_string[1][3], 2);
	for(i = 0; i < 2; i++)
	{
		USARTSend((uint8_t*)&sm_pump_string[i][0], 6);
		USARTSend((uint8_t*)"\r\n", 2);
	}

	// motor
	USARTSend((uint8_t*)"\r\n\r\n", 4);
	USARTSend((uint8_t*)"MOTOR:", 6);
	USARTSend((uint8_t*)"\r\n", 2);
	motor_state = CU_Motor_GetState();
	ConvertToString(motor_state->state, &sm_motor_string[0][3], 2);
	ConvertToString(motor_state->direction, &sm_motor_string[1][3], 2);
	ConvertToString(motor_state->speed, &sm_motor_string[2][3], 3);
	for(i = 0; i < 3; i++)
	{
		USARTSend((uint8_t*)&sm_motor_string[i][0], 6);
		USARTSend((uint8_t*)"\r\n", 2);
	}


	// ports
	USARTSend((uint8_t*)"\r\n\r\n", 4);
	USARTSend((uint8_t*)"PORTS IN:", 9);
	USARTSend((uint8_t*)"\r\n", 2);
	ConvertToString(GPIO_ReadInputData(GPIOA), &ports_string[0][2], 8);
	ConvertToString(GPIO_ReadInputData(GPIOB), &ports_string[1][2], 8);
	ConvertToString(GPIO_ReadInputData(GPIOC), &ports_string[2][2], 8);
	ConvertToString(GPIO_ReadInputData(GPIOD), &ports_string[3][2], 8);
	for(i = 0; i < 4; i++)
	{
		USARTSend((uint8_t*)&ports_string[i][0], 10);
		USARTSend((uint8_t*)"\r\n", 2);
	}

	USARTSend((uint8_t*)"\r\n\r\n", 4);
	USARTSend((uint8_t*)"PORTS OUT:", 10);
	USARTSend((uint8_t*)"\r\n", 2);
	ConvertToString(GPIO_ReadOutputData(GPIOA), &ports_string[0][2], 8);
	ConvertToString(GPIO_ReadOutputData(GPIOB), &ports_string[1][2], 8);
	ConvertToString(GPIO_ReadOutputData(GPIOC), &ports_string[2][2], 8);
	ConvertToString(GPIO_ReadOutputData(GPIOD), &ports_string[3][2], 8);
	for(i = 0; i < 4; i++)
	{
		USARTSend((uint8_t*)&ports_string[i][0], 10);
		USARTSend((uint8_t*)"\r\n", 2);
	}
}



/*******************************************************************************
* Function Name  : int_to_char
* Description    :
* Input          :
* Return         :
*******************************************************************************/
static __inline char int_to_char(uint8_t val)
{
	return (48 + val);
}

/*******************************************************************************
* Function Name  : ConvertToString
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void ConvertToString(uint32_t Num, char* Str, uint8_t NumSymbol)
{
	int32_t i, j, Rate;
	uint8_t A[NumSymbol];
	uint32_t NumPow = 1;

	for(i = 0; i < NumSymbol+1; i++) NumPow *= 10;
	if(Num > NumPow - 1) Num = NumPow - 1;

	for(i = NumSymbol - 1; i >= 0; i--)
	{
		A[i] = 0; Rate = 1;

		for(j = 0; j < i; j++) Rate *= 10;
		while(Num >= Rate){ Num = Num - Rate; A[i]++; }

		(*Str) = int_to_char(A[i]);
		Str++;
	}
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
