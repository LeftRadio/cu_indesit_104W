/**
  ******************************************************************************
  * @file	 	selector.c
  * @author  	Neil Lab :: Left Radio
  * @version 	v1.0.0
  * @date
  * @brief		sourse
  ******************************************************************************
**/

/* Includes ------------------------------------------------------------------*/
#include "selector.h"
#include "GlobalInit.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
const uint8_t selectorProgramCodes[SELECTOR_PRG_COUNT] =
{
    4, 7, 5, 21, 23, 22, 20, 28, 30,
    31, 29, 13, 15, 14, 12, 14, 15, 13, 29,
    31, 30, 28, 20, 22, 23, 21, 5, 7, 6
};

/* Extern function -----------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Selector_Configuration(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    /* program selector output */
    GPIO_InitStructure.GPIO_Pin = GPIO_PROGRAM_SELECTOR_OUT;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinLockConfig(GPIOB, GPIO_PROGRAM_SELECTOR_OUT);
    CU_Selector_Manual_OFF();

    /* program selector inputs */
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_PinLockConfig(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_Init(GPIOC, &GPIO_InitStructure);
    GPIO_PinLockConfig(GPIOC, GPIO_Pin_1);
}


/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
uint8_t CU_Selector_Read(void)
{
    __IO uint8_t data = (uint8_t)(GPIO_ReadInputData(GPIOB) >> 12) | (GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_1) << 4);

    if((GPIO_BUTTONS_PORT->IDR & GPIO_FAST_MODE_BUTTON_PIN) != 0)
    {
        return 7;
    }
    else return (((data >> 3) & 0x01) << 0) | ((data & 0x01) << 1) | \
                    (((data >> 1) & 0x01) << 2) | (((data >> 4) & 0x01) << 3) | \
                    (((data >> 2) & 0x01) << 4);

//	return out_data;
}


/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Selector_Manual_ON(void)
{
    GPIOB->BRR = GPIO_PROGRAM_SELECTOR_OUT;
}


/*******************************************************************************
* Function Name  :
* Description    :
* Input          : None
* Return         : None
*******************************************************************************/
void CU_Selector_Manual_OFF(void)
{
    GPIOB->BSRR = GPIO_PROGRAM_SELECTOR_OUT;
}








/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
