/**
  ******************************************************************************
  * @file	 	pumps.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "pumps.h"
#include "debug.h"
#include "systick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define __DEBUG_PUMPS

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO SM_Pumps_State Pumps_ActiveState = {0, RESET, 0};

/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Pumps_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	/* --------- OUT pumps --------- */
	GPIO_InitStructure.GPIO_Pin = GPIO_PUMP_OUT;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIO_ResetBits(GPIOA, GPIO_PUMP_OUT);

	/* --------- IN pumps --------- */
	GPIO_InitStructure.GPIO_Pin = GPIO_PUMP_IN_A | GPIO_PUMP_IN_B;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	GPIO_SetBits(GPIOB, GPIO_PUMP_IN_A | GPIO_PUMP_IN_B);

	/* --------- INPUTS --------- */
	GPIO_InitStructure.GPIO_Pin = GPIO_PUMP_WATER_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* --------- Lock GPIO --------- */
	GPIO_PinLockConfig(GPIOA, GPIO_PUMP_OUT);
	GPIO_PinLockConfig(GPIOB, GPIO_PUMP_IN_A | GPIO_PUMP_IN_B);
	GPIO_PinLockConfig(GPIOB, GPIO_PUMP_WATER_1);
}


/*******************************************************************************
* Function Name  : CU_pumps_SetState
* Description    :
* Input          : SM_pumps_State
* Return         : None
*******************************************************************************/
void CU_Pumps_UpdateState(SM_Pumps_State NewState)
{
	Pumps_ActiveState.pump_in_state = NewState.pump_in_state;
	Pumps_ActiveState.pump_in_water_level_control = NewState.pump_in_water_level_control;
	Pumps_ActiveState.pump_out_state = NewState.pump_out_state;
}


/*******************************************************************************
* Function Name  : CU_pumps_SetState
* Description    :
* Input          : SM_pumps_State
* Return         : None
*******************************************************************************/
void CU_Pumps_SetState(SM_Pumps_State NewState, FlagStatus start_flag)
{
#ifdef __DEBUG_PUMPS
	char dbg_str[15] = "\r\n ";
#endif

	/* Exit if new state is equal current active state */
	if((NewState.pump_in_state == Pumps_ActiveState.pump_in_state) && \
	        (NewState.pump_out_state == Pumps_ActiveState.pump_out_state) && \
	        (start_flag == RESET))
	{
		return;
	}
	else	/* else if new state is not equal current active state */
	{
		switch(NewState.pump_in_state)
		{
		case 0:	/* All IN pumps OFF */
			GPIO_SetBits(GPIOB, GPIO_PUMP_IN_A | GPIO_PUMP_IN_B);
#ifdef __DEBUG_PUMPS
		Debug_Send_String("\r\n\r\n pumps in: state all OFF");
#endif
			break;

		case 1:	/* IN pump A ON */
			GPIO_ResetBits(GPIOB, GPIO_PUMP_IN_A);
			GPIO_SetBits(GPIOB, GPIO_PUMP_IN_B);
#ifdef __DEBUG_PUMPS
		Debug_Send_String("\r\n\r\n pumps in: state A ON");
#endif
			break;

		case 2:	/* IN pump B ON */
			GPIO_SetBits(GPIOB, GPIO_PUMP_IN_A);
			GPIO_ResetBits(GPIOB, GPIO_PUMP_IN_B);
#ifdef __DEBUG_PUMPS
		Debug_Send_String("\r\n\r\n pumps in: state B ON");
#endif
			break;

		case 3: /* All IN pumps ON */
			GPIO_ResetBits(GPIOB, GPIO_PUMP_IN_A | GPIO_PUMP_IN_B);
#ifdef __DEBUG_PUMPS
		Debug_Send_String("\r\n\r\n pumps in: state all ON");
#endif
			break;

		default:
			break;
		}


		switch(NewState.pump_out_state)
		{
		case 0:	/* OUT pump OFF */
			GPIO_ResetBits(GPIOA, GPIO_PUMP_OUT);
			break;

		case 1:	/* OUT pump ON */
			GPIO_SetBits(GPIOA, GPIO_PUMP_OUT);
			break;

		default:
			break;
		}

		/* Update pumps actived state */
		CU_Pumps_UpdateState(NewState);

#ifdef __DEBUG_PUMPS
		ConvertToString(Pumps_ActiveState.pump_in_state, &dbg_str[2], 4);
		Debug_Send_String("\r\n\r\n pumps in: state");
		Debug_Send_String(dbg_str);
		Debug_Send_String("\r\n pumps in: water heater level control");
		ConvertToString((uint8_t)Pumps_ActiveState.pump_in_water_level_control, &dbg_str[2], 4);
		Debug_Send_String(dbg_str);
#endif
	}
}


/*******************************************************************************
* Function Name  : CU_pumps_SetState
* Description    :
* Input          : SM_pumps_State
* Return         : None
*******************************************************************************/
SM_Pumps_State* CU_Pumps_GetState(void)
{
#ifdef __DEBUG_PUMPS
	Debug_Send_String("pumps GET state\r\n");
#endif
	return (SM_Pumps_State*)&Pumps_ActiveState;
}


/**
  * @brief  pumps end state special function
  * @param  None
  * @retval None
  */
void CU_Pumps_EndState(void)
{
    SM_Pumps_State off = {0, 0, 0};

	if((Pumps_ActiveState.pump_in_state != 0) && (Pumps_ActiveState.pump_in_water_level_control == SET))
	{
#ifdef __DEBUG_PUMPS
		Debug_Send_String("\r\n  pumps ON, water control ON");
#endif

        /**< Read water level pin from praesure sensor */
		if(Pumps_Read_WaterLevel() == 0)
		{
#ifdef __DEBUG_PUMPS
			Debug_Send_String("\r\n pumps: water level NO signal, repeat cycle");
#endif
			CU_GetProgram()->sm_states->repeat = ((uint16_t)((*CU_Program_GetActiveStep()) - 1) << 8) | 0x01;
			return;
		}
		else
		{
#ifdef __DEBUG_PUMPS
			Debug_Send_String("\r\n pumps: now water control is OK, return");
#endif
		}
	}
	else if((Pumps_ActiveState.pump_in_state == 0) && (Pumps_ActiveState.pump_in_water_level_control == SET))
    {
#ifdef __DEBUG_PUMPS
		Debug_Send_String("\r\n pumps: pumps OFF, water control ON");
#endif
        /**< Read water level pin from praesure sensor */
		if(Pumps_Read_WaterLevel() == 0)
        {
#ifdef __DEBUG_PUMPS
            Debug_Send_String("\r\n pumps: NO signal, reconfig state for pumps ON");
#endif
        }
    }
	else
	{
#ifdef __DEBUG_PUMPS
		Debug_Send_String("\r\n pumps: water control OFF");
#endif
	}

	CU_Pumps_SetState(off, 1);
}


/**
  * @brief  pumps end state special function
  * @param  None
  * @retval None
  */
volatile uint16_t Pumps_Read_WaterLevel(void)
{
    volatile uint16_t dat = 0;
    dat = (GPIOC->IDR & GPIO_PUMP_WATER_1);
    return dat;
}


/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
