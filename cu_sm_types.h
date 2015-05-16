/**
  ******************************************************************************
  * @file			.h
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		header
  ******************************************************************************
**/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __CU_SM_TYPES_H
#define __CU_SM_TYPES_H

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"

/* Exported define -----------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported typedef ----------------------------------------------------------*/
typedef struct
{
	uint8_t state;
	uint8_t intent_water_temp;
	uint8_t termistor_water_temp;
} SM_Heater_State;

typedef struct
{
	uint8_t pump_in_state;
	FlagStatus pump_in_water_level_control;
	uint8_t pump_out_state;
} SM_Pumps_State;

typedef struct
{
	uint8_t state;
	uint8_t direction;
	uint16_t speed;
} SM_Motor_State;

typedef struct
{
	SM_Heater_State sm_heater_state;
	SM_Pumps_State sm_pumps_state;
	SM_Motor_State sm_motor_state;

	uint32_t work_time;
	uint16_t repeat;
} StateMashine;

typedef struct
{
	uint8_t fast_mode;
	uint8_t economy_mode;
	uint8_t temp;
	uint8_t water_level;
	uint8_t water_extraction;
	uint8_t water_extraction_speed;
	StateMashine *sm_states;
	StateMashine *sm_state_0;
	uint16_t *sm_sequence;
	uint16_t sm_steps;
} CU_Program;


/* Exported variables --------------------------------------------------------*/
/* Exported function ---------------------------------------------------------*/




#endif /* __CU_SM_TYPES_H */
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/


