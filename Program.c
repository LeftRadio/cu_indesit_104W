/**
  ******************************************************************************
  * @file	 	program.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "GlobalInit.h"
#include "Program.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define __DEBUG_PRG

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
__IO CU_Program SelectedProgram =
{
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    (void*)0,		//(StateMashine*)&sm_test[0],
    (void*)0,		//(uint16_t*)&sm_sequence[0],
    0				//STATE_MASHINE_STEPS
};

__IO uint8_t prg_step = 0;

#ifdef __DEBUG_PRG
char prg_dbg_str[15] = "\r\n ";
#endif

/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
static void CU_Program_EndStop(void);

/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Nprg_stepame  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Program_Configuration(void)
{
    uint8_t i;

    for(i = 0; i < SELECTOR_PRG_COUNT; i++)
    {
        if(CU_Selector_Read() == selectorProgramCodes[i])
        {
            SelectedProgram.fast_mode = All_Programs[i]->fast_mode;
            SelectedProgram.economy_mode = All_Programs[i]->economy_mode;
            //SelectedProgram.temp = All_Programs[i]->temp;
            SelectedProgram.water_extraction = All_Programs[i]->water_extraction;
            SelectedProgram.water_extraction_speed = All_Programs[i]->water_extraction_speed;
            SelectedProgram.water_level = All_Programs[i]->water_level;

            SelectedProgram.sm_states = All_Programs[i]->sm_states;
            SelectedProgram.sm_state_0 = All_Programs[i]->sm_states;
            SelectedProgram.sm_sequence = All_Programs[i]->sm_sequence;
            SelectedProgram.sm_steps = All_Programs[i]->sm_steps;

            break;
        }
    }


    /* ERROR */
    if(i > 1)
    {
#ifdef __DEBUG_PRG
        ConvertToString(i, &prg_dbg_str[2], 3);
        Debug_Send_String("\r\n prg: program ERROR load from index, index to big! ");
        Debug_Send_String(prg_dbg_str);
#endif
        CU_Program_EndStop();
    }
    else
    {
#ifdef __DEBUG_PRG
        ConvertToString(i, &prg_dbg_str[2], 3);
        Debug_Send_String("\r\n prg: program success load from index ");
        Debug_Send_String(prg_dbg_str);
        ConvertToString(SelectedProgram.temp, &prg_dbg_str[2], 4);
        Debug_Send_String("\r\n prg: user temp value C");
        Debug_Send_String(prg_dbg_str);
#endif

        CU_Program_NextSM();
    }
}

/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void  CU_Program_ReadUserTempControl(void)
{
    uint8_t i;
    uint32_t ADC_val = 0;
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    /* configure ADC channel 1 for read user temp control value */
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_TEMP_CONTROL;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* ADC1 configuration ------------------------------------------------------*/
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    /* ADC1 regular channel14 configuration */
    ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);

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

    for(i = 0; i < 100; i++)
    {
        while(ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) == (uint8_t)RESET)
        {
        }
        ADC_val += ADC_GetConversionValue(ADC1);
    }

    ADC_val /= 100;

    /* disable and reset ADC for the next used */
    ADC_SoftwareStartConvCmd(ADC1, DISABLE);
    ADC_Cmd(ADC1, DISABLE);
    ADC_DeInit(ADC1);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, DISABLE);

    if(ADC_val < 500) SelectedProgram.temp = 0;
    else if((ADC_val > 500) && (ADC_val < 1500)) SelectedProgram.temp = 30;
    else if((ADC_val > 1500) && (ADC_val < 2800)) SelectedProgram.temp = 40;
    else if((ADC_val > 2800) && (ADC_val < 3500)) SelectedProgram.temp = 50;
    else if((ADC_val > 3500) && (ADC_val < 3800)) SelectedProgram.temp = 60;
    else SelectedProgram.temp = 70;
}


/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Program_NextSM(void)
{
    uint16_t step_index_tmp = 0;
    uint16_t repeat = SelectedProgram.sm_states->repeat;
    uint8_t repeat_cnt = (uint8_t)(repeat & 0x00FF);

#ifdef __DEBUG_PRG
        Debug_Send_String("\r\n\r\n ------- switch to sm_next program ------- ");
        Debug_Send_Time();
#endif

    /* Repeat logic */
    if((repeat != 0) && (repeat_cnt > 0))
    {
        /* Decode start prg_step for repeat */
        prg_step = (uint8_t)((repeat & 0xFF00) >> 8);
        repeat_cnt--;

        /* Decrement counter and update sm_states->repeat */
        SelectedProgram.sm_states->repeat = ((uint16_t)prg_step << 8) | (uint16_t)(repeat_cnt);

#ifdef __DEBUG_PRG
        ConvertToString(prg_step, &prg_dbg_str[2], 3);
        Debug_Send_String("\r\n prg: repeat start prg_step");
        Debug_Send_String(prg_dbg_str);
        ConvertToString(repeat_cnt, &prg_dbg_str[2], 3);
        Debug_Send_String("\r\n prg: repeat counter");
        Debug_Send_String(prg_dbg_str);
#endif
    }



    /* Switch programs */
    if(prg_step < SelectedProgram.sm_steps)
    {
        step_index_tmp = SelectedProgram.sm_sequence[prg_step];
        SelectedProgram.sm_states = &SelectedProgram.sm_state_0[step_index_tmp];						//&sm_test[sm_sequence[prg_step]];
        prg_step++;

#ifdef __DEBUG_PRG
        ConvertToString(step_index_tmp, &prg_dbg_str[2], 3);
        prg_dbg_str[5] = 0;
        Debug_Send_String("\r\n sm: program index ");
        Debug_Send_String(prg_dbg_str);
#endif
    }
    else
    {
        SelectedProgram.sm_states = &SelectedProgram.sm_state_0[0];
        prg_step = 0;

        CU_Program_EndStop();
    }
}


void CU_Program_EndSM(void)
{
#ifdef __DEBUG_PRG
    Debug_Send_String("\r\n\r\n ------- sm_end program ------- ");
    Debug_Send_Time();
#endif

    CU_Motor_Reset();
    CU_Pumps_EndState();
}


uint32_t CU_Program_CalcEndTime(uint32_t start_time)
{
#ifdef __DEBUG_PRG
    Debug_Send_String("\r\n\r\n ------- sm_start program ------- ");
#endif
    return (start_time + SelectedProgram.sm_states->work_time);
}

/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
CU_Program* CU_GetProgram(void)
{
    return (CU_Program*)&SelectedProgram;
}


uint8_t* CU_Program_GetActiveStep(void)
{
    return (uint8_t*)&prg_step;
}


/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
static void CU_Program_EndStop(void)
{
    /* Global DISABLE ~HV line*/
    GPIO_SetBits(GPIOC, GPIO_EN_HV_LINE_PIN);

    /* Stop work */
    __disable_irq();

#ifdef __DEBUG_PRG
    Debug_Send_String("\r\n\r\n --------------------- END ALL PROGRAMS --------------------- ");
#endif

    while(1)
    {
        GPIO_SetBits(GPIOC, GPIO_ON_OFF_LAMP);
        delay_ms(500);
        GPIO_ResetBits(GPIOC, GPIO_ON_OFF_LAMP);
        delay_ms(500);
    }
}




/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
