/**
  ******************************************************************************
  * @file	 	temp.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>

#include "stm32f10x.h"
#include "GlobalInit.h"
#include "Program.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define __DEBUG_SM

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO uint8_t RTC_Sec_Event = 0;
__IO FlagStatus DMA_USART_Recive_Complete = RESET;
__IO uint32_t rtc_sm_end_counter = 0;

#ifdef __DEBUG_SM
char dbg_str[15] = "\r\n ";
uint8_t tData = 0;
#endif

/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void task_General (void);
static void Host_Work_Command (void);
static __inline uint16_t CharToByte(char ch);


/* Private functions ---------------------------------------------------------*/

/* ---------------------------------------------------- MAIN ---------------------------------------------------- */

/*******************************************************************************
* Function Name  : main
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
int main(void)
{
    //Initialize the system stuff
    SystemInit();
    GlobalInit();

    /* Turn on work indication lamp, all OK */
    GPIO_SetBits(GPIOC, GPIO_ON_OFF_LAMP);

    /* Main cycle */
    while (1)
    {
        task_General();

        if(DMA_USART_Recive_Complete == SET)
        {
        	DMA_USART_Recive_Complete = RESET;
            Host_Work_Command();

            DMA1_Channel5->CCR &= ~DMA_CCR5_EN;

            memset(USART_RxBuffer, 0, 8);

            DMA1_Channel5->CNDTR = (uint32_t)0x04;
            DMA1_Channel5->CMAR = (uint32_t)&USART_RxBuffer[0];
            DMA1_Channel5->CCR |= DMA_CCR5_EN;
        }

    }
}


/* ---------------------------------------------------- HOST WORK CMD ---------------------------------------------------- */

/*******************************************************************************
* Function Name  : Host_Work_Command
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
static void Host_Work_Command (void)
{
    float cmd_data = 0;
#ifdef __DEBUG_MOTOR
    char tstr[8] = "";
#endif

    if(memcmp(USART_RxBuffer, "stp+", 4) == 0)
    {
        RTC_WaitForLastTask();
        rtc_sm_end_counter = RTC_GetCounter() - 1;
        RTC_WaitForLastTask();

        Debug_Send_String("\r\n\r\n HOST CMD SWITCH TO NEXT STEP... \r\n\r\n");
    }
    else if(memcmp(USART_RxBuffer, "stp-", 4) == 0)
    {
        //(CU_Program_GetActiveStep() - 1)
    }
    else if(memcmp(USART_RxBuffer, "rsts", 3) == 0)
    {
        Debug_Send_String("\r\n\r\n HOST CMD RESET...\r\n\r\n");
        /* Reset */
        NVIC_SystemReset();
    }
    else if(USART_RxBuffer[0] == 'm')
    {
    	Debug_Send_String("\r\n\r\n HOST CMD MOTOR...\r\n\r\n");

    	if(USART_RxBuffer[1] == 't')
    	{
    		cmd_data = (float)(CharToByte(USART_RxBuffer[2])) + ((float)(CharToByte(USART_RxBuffer[3])) / 10.F);
    		Timer1_ReConfig(cmd_data);
    	}
    	else if(USART_RxBuffer[1] == 'p')
    	{
    		cmd_data = (float)(CharToByte(USART_RxBuffer[2])) + ((float)(CharToByte(USART_RxBuffer[3])) / 10.F);
    		MP_Coeff = cmd_data;
    	}
    	else if(USART_RxBuffer[1] == 'd')
    	{
    		cmd_data = (float)(CharToByte(USART_RxBuffer[2])) + ((float)(CharToByte(USART_RxBuffer[3])) / 10.F);
    		MD_Coeff = cmd_data;
    	}
    	else
    	{
    		cmd_data = (CharToByte(USART_RxBuffer[1]) * 100) + (CharToByte(USART_RxBuffer[2]) * 10) + CharToByte(USART_RxBuffer[3]);

    		if(cmd_data >= 999) cmd_data = 999;
    		else if(cmd_data < 0) cmd_data = 0;

    		CU_GetProgram()->sm_states->sm_motor_state.state = ENABLE;
    		CU_GetProgram()->sm_states->sm_motor_state.speed = cmd_data;
#ifdef __DEBUG_MOTOR
    		Debug_Send_String("\r\n motor speed set to   ");
    		ConvertToString(CU_GetProgram()->sm_states->sm_motor_state.speed, &tstr[2], 3);
    		Debug_Send_String(tstr);
#endif

    		CU_Motor_SetState(CU_GetProgram()->sm_states->sm_motor_state, SET);
    	}
    }
    else if(memcmp(USART_RxBuffer, "pin", 3) == 0)
    {
    	if(USART_RxBuffer[3] == '+') CU_GetProgram()->sm_states->sm_pumps_state.pump_in_state = 3;
    	else if(USART_RxBuffer[3] == '-') CU_GetProgram()->sm_states->sm_pumps_state.pump_in_state = 0;
    	else return;

    	CU_Pumps_SetState(CU_GetProgram()->sm_states->sm_pumps_state, SET);
    }
    else if(memcmp(USART_RxBuffer, "pou", 3) == 0)
    {
    	if(USART_RxBuffer[3] == '+') CU_GetProgram()->sm_states->sm_pumps_state.pump_out_state = 1;
    	else if(USART_RxBuffer[3] == '-') CU_GetProgram()->sm_states->sm_pumps_state.pump_out_state = 0;
    	else return;

    	CU_Pumps_SetState(CU_GetProgram()->sm_states->sm_pumps_state, SET);
    }
    else if(memcmp(USART_RxBuffer, "hea", 3) == 0)
    {
    	if(USART_RxBuffer[3] == '+') CU_GetProgram()->sm_states->sm_heater_state.state = 1;
    	else if(USART_RxBuffer[3] == '-') CU_GetProgram()->sm_states->sm_heater_state.state = 0;
    	else return;

    	CU_Heater_SetState(CU_GetProgram()->sm_states->sm_heater_state, SET);
    }
    else if(memcmp(USART_RxBuffer, "boot", 4) == 0)
    {
        Jump_To_Bootloader();
    }
    else
    {

	}
}


/* ---------------------------------------------------- GENERAL TASK ---------------------------------------------------- */

/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
static void task_General (void)
{
    static __IO uint8_t start_flag = SET;
    uint32_t rtc_counter;

    RTC_WaitForLastTask();
    rtc_counter = RTC_GetCounter();
    RTC_WaitForLastTask();

    /* If new program is start calc and save end time for new state */
    if(start_flag != RESET)
    {
        rtc_sm_end_counter = CU_Program_CalcEndTime(rtc_counter);
    }
    else if(rtc_sm_end_counter < rtc_counter)
    {
        CU_Program_EndSM();
        CU_Program_NextSM();

        /* reset start flag fo the next active state */
        start_flag = SET;
        return;
    }

    /* ----------- working program ----------- */
    CU_Pumps_SetState(CU_GetProgram()->sm_states->sm_pumps_state, start_flag);
    CU_Heater_SetState(CU_GetProgram()->sm_states->sm_heater_state, start_flag);
    CU_Motor_SetState(CU_GetProgram()->sm_states->sm_motor_state, start_flag);

    /* Reset start flag */
    if(start_flag != RESET) start_flag = RESET;
}







static __inline uint16_t CharToByte(char ch)
{
	return (uint8_t)((uint8_t)ch - (uint8_t)48);
}



/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
