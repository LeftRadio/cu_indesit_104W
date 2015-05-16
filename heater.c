/**
  ******************************************************************************
  * @file	 	Heater.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include "GlobalInit.h"
#include "heater.h"
#include "debug.h"
#include "systick.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define __DEBUG_Heater

#define ADC1_DR_Address    			((uint32_t)0x4001244C)


/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO SM_Heater_State Heater_ActiveState = {0, 0};
__IO uint16_t Heater_ADCConvertedValue[100] = {0};

/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Heater_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	ADC_InitTypeDef ADC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	/* --------- OUTPUTS --------- */
	GPIO_InitStructure.GPIO_Pin = GPIO_HEATER;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIO_HEATER_PORT, &GPIO_InitStructure);
	GPIO_ResetBits(GPIO_HEATER_PORT, GPIO_HEATER);

	/* --------- INPUTS (ADC) --------- */
	RCC_ADCCLKConfig(RCC_PCLK2_Div8);

	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 ;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* DMA1 channel1 configuration ----------------------------------------------*/
	DMA_DeInit(DMA1_Channel1);
	DMA_InitStructure.DMA_PeripheralBaseAddr = ADC1_DR_Address;
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)&Heater_ADCConvertedValue;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = 100;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(DMA1_Channel1, &DMA_InitStructure);

	/* Enable DMA1 channel1 */
	DMA_Cmd(DMA1_Channel1, ENABLE);

	DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);
	NVIC_SetPriority(DMA1_Channel1_IRQn, DMA_ADC_INTR_PRIORITY);

	/* ADC1 configuration ------------------------------------------------------*/
	ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_InitStructure.ADC_ScanConvMode = DISABLE;
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
	ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStructure.ADC_NbrOfChannel = 1;
	ADC_Init(ADC1, &ADC_InitStructure);

	/* ADC1 regular channel14 configuration */
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);

	/* Enable ADC1 DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADC1 */
	ADC_Cmd(ADC1, ENABLE);

	/* Enable ADC1 reset calibration register */
	ADC_ResetCalibration(ADC1);
	/* Check the end of ADC1 reset calibration register */
	while(ADC_GetResetCalibrationStatus(ADC1));

	/* Start ADC1 calibration */
	ADC_StartCalibration(ADC1);
	/* Check the end of ADC1 calibration */
	while(ADC_GetCalibrationStatus(ADC1));

	/* Start ADC1 Software Conversion */
	ADC_SoftwareStartConvCmd(ADC1, ENABLE);


	/* --------- Lock GPIO --------- */
	GPIO_PinLockConfig(GPIO_HEATER_PORT, GPIO_HEATER);
}



/*******************************************************************************
* Function Name  : CU_Heater_SetState
* Description    :
* Input          : SM_Heater_State
* Return         : None
*******************************************************************************/
void CU_Heater_SetState(SM_Heater_State NewState, FlagStatus start_flag)
{
#ifdef __DEBUG_Heater
	char dbg_str[15] = {"\r\n"};
	static uint16_t cnt = 0;
#endif

	/* Exit if new state is equal current active state */
	if((NewState.state == Heater_ActiveState.state) && (start_flag == RESET))
	{
		if(Heater_ActiveState.state != 0)
		{
			if(Heater_ActiveState.termistor_water_temp < (Heater_ActiveState.intent_water_temp - 1))
			{
				GPIO_HEATER_PORT->BSRR = GPIO_HEATER;
			}
			else if(Heater_ActiveState.termistor_water_temp > (Heater_ActiveState.intent_water_temp + 1))
			{
				GPIO_HEATER_PORT->BRR = GPIO_HEATER;
			}
		}
		else
		{
#ifdef __DEBUG_Heater
//		Debug_Send_String("\r\n\r\n heater: State is OFF and not a change, return");
#endif
		}
	}
	else	/* else if new state is not equal current active state */
	{
		if(NewState.state != 0)
		{
			if(CU_GetProgram()->temp != 0)
			{
				NewState.intent_water_temp = CU_GetProgram()->temp;
				NVIC_EnableIRQ(DMA1_Channel1_IRQn);
#ifdef __DEBUG_Heater
				Debug_Send_String("\r\n\r\n heater: State is ON");
#endif
			}
		}
		else
		{
#ifdef __DEBUG_Heater
			Debug_Send_String("\r\n\r\n heater: State is OFF");
#endif
			GPIO_HEATER_PORT->BRR = GPIO_HEATER;
			NVIC_DisableIRQ(DMA1_Channel1_IRQn);
		}

		Heater_ActiveState.state = NewState.state;
		Heater_ActiveState.intent_water_temp = NewState.intent_water_temp;
	}

#ifdef __DEBUG_Heater
	if((cnt == 500) || (start_flag == SET))
	{
		cnt = 0;
		ConvertToString((uint32_t)(GPIO_HEATER_PORT->ODR & GPIO_HEATER), &dbg_str[2], 4);
		Debug_Send_String("\r\n heater: Realy OUT PIN, 0 - OFF");
		Debug_Send_String(dbg_str);
	}
#endif
}


/*******************************************************************************
* Function Name  : CU_Heater_SetState
* Description    :
* Input          : SM_Heater_State
* Return         : None
*******************************************************************************/
SM_Heater_State* CU_Heater_GetState(void)
{
#ifdef __DEBUG_Heater
	Debug_Send_String("\r\n heater: GET state");
#endif
	return (SM_Heater_State*)&Heater_ActiveState;
}



/*******************************************************************************
* Function Name  : CU_Heater_SetState
* Description    :
* Input          : SM_Heater_State
* Return         : None
*******************************************************************************/
void CU_Heater_SetTerhmisorValue(float val)
{
#ifdef __DEBUG_Heater
	char string[10] = "\r\n";
	static uint16_t cnt = 0;

	if(++cnt >= 500)
	{
		cnt = 0;
		ConvertToString((uint8_t)(Heater_ActiveState.intent_water_temp), &string[2], 3);
		Debug_Send_String("\r\n heater: Intent Val");
		Debug_Send_String(string);
		ConvertToString((uint8_t)(val / 80.0), &string[2], 4);
		Debug_Send_String("\r\n heater: TempSensor Val");
		Debug_Send_String(string);
	}
#endif

	Heater_ActiveState.termistor_water_temp = (uint8_t)(val / 80.0);
}






/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
