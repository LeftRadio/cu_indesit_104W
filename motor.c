/**
  ******************************************************************************
  * @file	 	motor.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "motor.h"
#include "GlobalInit.h"
#include "debug.h"
#include "systick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
//#define __DEBUG_MOTOR

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO SM_Motor_State ActiveState = {0, 0, 0};
__IO int tacho_current_speed = 0;
extern __IO uint16_t tachometr_counter;

__IO float MP_Coeff = 8.0F;
__IO float MD_Coeff = 1.0F;

/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void CU_Motor_UpdateState(SM_Motor_State NewState);
static void CU_Motor_ON(SM_Motor_State NewState);
static void CU_Motor_OFF(void);

//static uint16_t CU_Motor_DoFullPID(uint16_t In, uint16_t Ref, uint16_t *Coeff);
static float CU_Motor_Do_P(float In, float Ref, float Coeff);
static float CU_Motor_Do_D(float In, float Coeff, FlagStatus Init);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Motor_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	EXTI_InitTypeDef EXTI_InitStructure;

	// previosly OFF state
	GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_PWM);
	GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_REL_A | GPIO_MOTOR_OUT_REL_B);

	// then init outs for relay(direction) and triac
	GPIO_InitStructure.GPIO_Pin = GPIO_MOTOR_OUT_REL_A | GPIO_MOTOR_OUT_REL_B | GPIO_MOTOR_OUT_PWM;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIO_MOTOR_OUT_PORT, &GPIO_InitStructure);

	GPIO_PinLockConfig(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_REL_A | GPIO_MOTOR_OUT_REL_B | GPIO_MOTOR_OUT_PWM);


	/* ---------------------------------------- INPUTS ---------------------------------------- */

	/* Configure pins as input PD */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource8);
	GPIO_EXTILineConfig(GPIO_PortSourceGPIOB, GPIO_PinSource9);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line9;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
#ifdef __MOTOR_REGULATE_RELAY_METHOD
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
#else	/* __MOTOR_REGULATE_PHASE_METHOD */
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
#endif
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Configure EXTI line */
	EXTI_InitStructure.EXTI_Line = EXTI_Line8;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);


	GPIO_PinLockConfig(GPIOB, GPIO_Pin_8 | GPIO_Pin_9);


	// another OFF state
	GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_PWM);
	GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_REL_A | GPIO_MOTOR_OUT_REL_B);

	NVIC_SetPriority(EXTI9_5_IRQn, EXTI_INTR_PRIORITY);
}


/*******************************************************************************
* Function Name  : CU_Motor_SetState
* Description    :
* Input          : SM_Motor_State
* Return         : None
*******************************************************************************/
void CU_Motor_SetState(SM_Motor_State NewState, FlagStatus start_flag)
{
	/* Exit if new state is equal current active state */
	if(((NewState.state == ActiveState.state) && (NewState.direction == ActiveState.direction)) && \
			(start_flag == RESET))
	{
#ifdef __DEBUG_MOTOR
//		Debug_Send_String("\r\n\r\n motor state not change, return");
#endif
		return;
	}
	else	/* else if new state is not equal current active state */
	{
		// if new state is ON
		if(NewState.state != 0)
		{
#ifdef __DEBUG_MOTOR
			Debug_Send_String("\r\n\r\n motor command ON");
#endif
			CU_Motor_ON(NewState);
		}

		// if new state is OFF
		else
		{
#ifdef __DEBUG_MOTOR
			Debug_Send_String("\r\n\r\n motor command OFF");
#endif
			CU_Motor_OFF();
		}
	}
}


/*******************************************************************************
* Function Name  : CU_Motor_SetState
* Description    :
* Input          : SM_Motor_State
* Return         : None
*******************************************************************************/
SM_Motor_State* CU_Motor_GetState(void)
{
#ifdef __DEBUG_MOTOR
//	Debug_Send_String("motor GET state\r\n");
#endif
	return (SM_Motor_State*)&ActiveState;
}


/*******************************************************************************
* Function Name  : CU_Motor_UpdateState
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
static void CU_Motor_UpdateState(SM_Motor_State NewState)
{
#ifdef __DEBUG_MOTOR
	Debug_Send_String("\r\n\r\n motor UPDATE state");
#endif
	ActiveState.state = NewState.state;
	ActiveState.direction = NewState.direction;
	ActiveState.speed = NewState.speed;
}


/*******************************************************************************
* Function Name  : CU_Motor_ON_Right
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
static void CU_Motor_ON(SM_Motor_State NewState)
{
	// on motor
	if(NewState.state == 0)
	{
#ifdef __DEBUG_MOTOR
//		Debug_Send_String("\r\n\r\n ERROR! command ON but State is OFF value");
#endif
		return;
	}
	else
	{
		// OFF if motor now ON
		if((ActiveState.state != 0) && (NewState.direction != ActiveState.direction))
		{
			CU_Motor_OFF();
		}

		// left
		if(NewState.direction == 0)
		{
#ifdef __DEBUG_MOTOR
//			Debug_Send_String("\r\n\r\n motor relays SET left");
#endif
			GPIO_SetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_REL_A);
			GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_REL_B);
		}
		// right
		else
		{
#ifdef __DEBUG_MOTOR
//			Debug_Send_String("\r\n\r\n motor relays SET right");
#endif
			GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_REL_A);
			GPIO_SetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_REL_B);
		}

		delay_ms(200);

#ifdef __DEBUG_MOTOR
//		Debug_Send_String("\r\n\r\n motor relay actually ON, ENABLE zero cross interupts");
#endif
		// Update motor state
		CU_Motor_UpdateState(NewState);

		/* Clear the EXTI line pending bit, enable zero cross detector input interrupts */
		EXTI->PR = GPIO_MOTOR_ZC_A1;
		NVIC_EnableIRQ(EXTI9_5_IRQn);
	}
}


/*******************************************************************************
* Function Name  : CU_Motor_OFF
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
static void CU_Motor_OFF(void)
{
	SM_Motor_State OFF_State = {0, 0, 0};
	CU_Motor_Reset();

	GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_PWM);

	/* Set state now, no delay for stop motor physicaly, needed for the zero cross interupts */
	ActiveState.state = 0;

	delay_ms(100);
	GPIO_ResetBits(GPIO_MOTOR_OUT_PORT, GPIO_MOTOR_OUT_REL_A | GPIO_MOTOR_OUT_REL_B);

	while(1)
	{
		/* wait for the stop motor physicaly */
		while(tachometr_counter != 0)
		{
		}
		delay_ms(100);

		/* exit if success stop */
		if(tachometr_counter == 0) break;
	}

#ifdef __DEBUG_MOTOR
	Debug_Send_String("\r\n\r\n motor actually OFF");
#endif

	/* Update motor state */
	CU_Motor_UpdateState(OFF_State);
}



/*******************************************************************************
* Function Name  : CU_Motor_Tachometr_Processing
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Motor_Tachometr_Processing(float E_RPS)
{
#ifdef __DEBUG_MOTOR
	char dbg_str[15] = "\r\n ";
	static uint8_t dbg_cnt = 0;
#endif

	__IO float p = 0, d = 0;
	float Intent_RPS = CU_Motor_GetState()->speed;	// motor speed 100% = 10000	wh/min

	p = CU_Motor_Do_P(E_RPS, Intent_RPS, MP_Coeff);
	d = CU_Motor_Do_D(E_RPS, MD_Coeff, RESET);

	tacho_current_speed += (int)(p + d);
	if(tacho_current_speed >= 800) tacho_current_speed = 800;


#ifdef __DEBUG_MOTOR
	if(++dbg_cnt >= 100)
	{
		dbg_cnt = 0;

		ConvertToString((uint32_t)E_RPS, &dbg_str[2], 4);
		Debug_Send_String("\r\n\r\n tacho nRPS: ");
		Debug_Send_String(dbg_str);

		if(p >= 0) ConvertToString((uint32_t)p, &dbg_str[2], 4);
		else
		{
			dbg_str[2] = '-';
			p = p * (-1);
			ConvertToString((uint32_t )p, &dbg_str[3], 3);
		}
		dbg_str[6] = ' ';
		if(d >= 0) ConvertToString((uint32_t )d, &dbg_str[7], 4);
		else
		{
			dbg_str[7] = '-';
			d = d * (-1);
			ConvertToString((uint32_t)p, &dbg_str[8], 3);
		}

		Debug_Send_String("\r\n motor p, d");
		Debug_Send_String(dbg_str);

		ConvertToString(tacho_current_speed, &dbg_str[2], 4);
		Debug_Send_String("\r\n tacho new p d sum speed: ");
		Debug_Send_String(dbg_str);
	}
#endif


}


/* @brief  PID in C, Error computed inside the routine
 * @param : Input (measured value)
 *   Ref: reference (target value)
 *   Coeff: pointer to the coefficient table
 * @retval : PID output (command)
 */
static float CU_Motor_Do_P(float In, float Ref, float Coeff)
{
	return ((Ref - In) / Coeff);
}


/* @brief  PID in C, Error computed inside the routine
 * @param : Input (measured value)
 *   Ref: reference (target value)
 *   Coeff: pointer to the coefficient table
 * @retval : PID output (command)
 */
static float CU_Motor_Do_D(float In, float Coeff, FlagStatus Init)
{
	static float old_tacho_val = 0;
	float temp = (old_tacho_val - In) / Coeff;

	if(Init != SET) old_tacho_val = In;
	else old_tacho_val = 0;

	return temp;
}


/*******************************************************************************
* Function Name  : CU_Motor_SetState
* Description    :
* Input          : SM_Motor_State
* Return         : None
*******************************************************************************/
int CU_Motor_GetTachoSpeed(void)
{
#ifdef __DEBUG_MOTOR
//	Debug_Send_String("motor GET state\r\n");
#endif
	if(tacho_current_speed == 0) return 1;
	return tacho_current_speed;
}


void CU_Motor_Reset(void)
{
	tacho_current_speed = CU_Motor_GetState()->speed;
	CU_Motor_Do_D(0, 0.0, SET);
}

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
