/**
  ******************************************************************************
  * @file	 	programs_data.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "programs_data.h"
#include "Program.h"
#include "selector.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define PRG1_STATE_MASHINE_STEPS						(uint32_t)12
#define PRG2_STATE_MASHINE_STEPS						(uint32_t)6

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* ------------------------------------------ PROGRAM 1 ------------------------------------------ */
__IO StateMashine sm_program_1[] = {

		// state ( indx 0 ) ( ALL OFF )
		{
				{0, 0, 0},			// heater
				{0, 0, 0},			// pumps
				{0, 0, 0},			// motor
				10,					// time
				0					// repeat
		},

		// state ( indx 1 ) (1 and 2 POMP IN)           // water in
		{
				{0, 0, 0},			// heater
				{3, SET, 0},		// pumps
				{0, 0, 0},			// motor
				15,					// time
				0					// repeat
		},

		// state ( indx 2 ) ( MOTOR LEFT )              // preapere
		{
				{0, 0, 0},			// heater
				{0, SET, 0},		// pumps
				{1, 0, 40},		// motor
				60 * 10,				// time
				0					    // repeat
		},

		// state ( indx 3 ) ( MOTOR LEFT )
		{
				{1, 40, 0},			// heater
				{0, SET, 0},		// pumps
				{1, 0, 60},		// motor
				60,					// time
				0					// repeat
		},

		// state ( indx 4 ) ( MOTOR RIGHT )
		{
				{1, 40, 0},			// heater
				{0, 0, 0},		// pumps
				{1, 1, 60},		// motor
				60,					// time
				(5 << 8) | 20		// repeat, from step 3( sm_program_1_sequence[5] ), 40 cnt
		},

		// state ( indx 5 ) (POMP OUT)
		{
				{0, 0, 0},			// heater
				{0, 0, 1},			// pumps
				{0, 0, 0},			// motor
				30,					// time
				0					    // repeat
		},


		// state ( indx 6 ) (1 and 2 POMP IN R)
		{
				{0, 0, 0},			// heater
				{3, SET, 0},		// pumps
				{0, 0, 0},		    // motor
				40,					// time
				0					// repeat
		},

		// state Heater( indx 7 )
		{
				{1, 40, 0},			// heater
				{0, 0, 0},		    // pumps
				{1, 0, 100},		// motor
				2 * 60,				// time
				0					    // repeat
		},

		// state ( indx 8 ) (POMP OUT R)
		{
				{0, 0, 0},			// heater
				{0, 0, 1},			// pumps
				{1, 1, 200},		// motor
				30,					// time
				(8 << 8) | 3		// repeat, from step 8( sm_program_1_sequence[8] ), 3 cycle
		},


};

/* State mashine steps sequence, indexes for sm_states[] program */
const uint16_t sm_program_1_sequence[PRG1_STATE_MASHINE_STEPS] = {

		0,

		/* preapere cycle */
		1,			// water in
		2,		    // preapere cycle

		0,			//

		/* repeated cycle */
		1,			// water in
		3, 4,		// main cycle
		5,			// water out

		/* repeated cycle */
		6,			// water in
		7, 		// R cycle
		8,			// water out

		0,
};

/* main type for program */
__IO CU_Program Program_1 = {
		0,
		0,
		0,
		0,
		0,
		0,
		(StateMashine*)sm_program_1,
		(StateMashine*)sm_program_1,
		(uint16_t*)sm_program_1_sequence,
		PRG1_STATE_MASHINE_STEPS
};



/* ------------------------------------------ PROGRAM 2 ------------------------------------------ */
__IO StateMashine sm_program_2[] = {

		// state ( indx 0 ) ( ALL OFF )
		{
				{0, 0, 0},			// heater
				{0, 0, 0},			// pumps
				{0, 0, 0},			// motor
				10,					// time
				0					// repeat
		},

		// state ( indx 1 ) (1 and 2 POMP IN COLD)
		{
				{0, 0, 0},			// heater
				{3, RESET, 0},		// pumps
				{0, 0, 0},			// motor
				100,				// time
				0					// repeat
		},

		// state ( indx 2 ) ( MOTOR RIGHT )
		{
				{1, 50, 0},			// heater
				{0, 0, 0},			// pumps
				{1, 1, 10},		    // motor
				40 * 60,			// time
				0					//
		},

		// state ( indx 3 ) (POMP OUT HOT)
		{
				{0, 0, 0},			// heater
				{0, 0, 1},			// pumps
				{0, 0, 0},			// motor
				60,					// time
				(1 << 8) | 30		// repeat, from step 2(index), 25 cnt
		},

		// state ( indx 4 ) (POMP OUT HOT)
		{
				{0, 0, 0},			// heater
				{0, 0, 1},			// pumps
				{0, 0, 0},			// motor
				40,					// time
				0           		// repeat
		},
};

/* State mashine steps sequence, indexes for sm_states[] program */
const uint16_t sm_program_2_sequence[PRG2_STATE_MASHINE_STEPS] = {

		0,

		/* repeated cycle */
		1,			// water in
		2, 		    // 1st cycle
		3,			// water out
        4,
		0,
};

/* main type for program */
__IO CU_Program Program_2 = {
		0,
		0,
		0,
		0,
		0,
		0,
		(StateMashine*)sm_program_2,
		(StateMashine*)sm_program_2,
		(uint16_t*)sm_program_2_sequence,
		PRG2_STATE_MASHINE_STEPS
};




/* ------------------------------------------ ALL PROGRAMS ------------------------------------------ */
__IO CU_Program *All_Programs[] = {
	&Program_1, &Program_2
};


/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/







/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
